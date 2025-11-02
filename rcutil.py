#!/usr/bin/env python3
#
# Copyright (c) 2025 University of York and others
#
# Command line utility supporting the AURO2025 assessment.
# 
# Contributors:
#   * Pedro Ribeiro - initial implementation

import argparse
import argcomplete
import os
import shlex
import shutil
import sys
import zipfile
import tempfile
import subprocess
from pathlib import Path
import io
import json
import re
from launch import LaunchService
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from enum import Enum

version = "1.0"

required_params = ['num_robots', 'random_seed', 'experiment_duration', 'use_nav2', 'sensor_noise', 'use_rviz', 'visualise_sensors', 'odometry_source', 'barrels', 'headless', 'limit_real_time_factor', 'wait_for_barrels', 'vision_sensor_skip_frames', 'vision_sensor_frame_divider', 'vision_sensor_debug', 'initial_pose_file', 'initial_pose_package']

# ANSI colors
RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
BLUE = "\033[94m"
RESET = "\033[0m"

class FolderStatus(Enum):
    UNCHANGED = 0
    LOCAL = 1
    REMOTE = 2
    BOTH = 3

def get_launch_args_from_file(launch_path: Path):
    """
    Safely execute a ROS 2 launch file in a subprocess to extract declared arguments.
    Returns a list of argument names or an error message.
    """
    code = f"""
import importlib.util, json, sys
launch_path = r'''{str(launch_path)}'''
spec = importlib.util.spec_from_file_location('launch_module', launch_path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
ld = module.generate_launch_description()
args = [e.name for e in getattr(ld, 'entities', []) if e.__class__.__name__ == 'DeclareLaunchArgument']
print(json.dumps(args))
"""

    try:
        result = subprocess.run(
            [sys.executable, "-c", code],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,  # silence all output
            text=True,
            timeout=5.0,  # optional timeout
        )
        output = result.stdout.strip()
        if not output:
            return []
        return json.loads(output)
    except subprocess.TimeoutExpired:
        return ["⚠️ Timeout while loading launch file"]
    except json.JSONDecodeError:
        return ["⚠️ Could not parse arguments"]
    except Exception as e:
        return [f"⚠️ Error: {e}"]

# -----------------------------
# Workspace detection
# -----------------------------
def is_ros2_workspace(path: Path) -> bool:
    src = path
    if not src.is_dir():
        return False
    for root, _, files in os.walk(src):
        if "package.xml" in files:
            return True
    return False

def is_ros2_workspace_zip(zip_path: Path) -> bool:
    """
    Check if a zip file contains a ROS2 workspace.
    Returns True if any 'package.xml' exists in the zip, anywhere in the hierarchy.
    """
    if not zip_path.is_file():
        return False

    with zipfile.ZipFile(zip_path, "r") as z:
        for name in z.namelist():
            # Skip directories
            if name.endswith("/"):
                continue
            # Check for package.xml anywhere
            if Path(name).name == "package.xml":
                return True
    return False


# -----------------------------
# Colcon build
# -----------------------------
def colcon_build(workspace_path: Path, colcon_args=None):
    colcon_args = colcon_args or ["--symlink-install"]
    cmd_list = ["colcon", "build"] + colcon_args
    ros2_setup = "/opt/ros/humble/setup.bash"

    print(f"Running: {' '.join(cmd_list)} in {workspace_path}")

    # Clear environment variables for ament/cmake
    env = os.environ.copy()
    for var in ["AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH"]:
        env.pop(var, None)

    # After cleaning env vars, fresh source humble's setup.basj
    cmd_str = f"source {ros2_setup} && " + " ".join(shlex.quote(arg) for arg in cmd_list)

    subprocess.run(cmd_str, cwd=workspace_path, shell=True, env=env, executable="/bin/bash", check=True)

def clean_ros2_workspace(workspace_path: Path):
    """
    Clean a ROS2 workspace by deleting build/, install/, and log/ directories.
    Also unsets ROS2 environment variables that may point to old workspace paths.
    
    :param ws_path: Path to the root of the ROS2 workspace
    """
    dirs_to_clean = ["build", "install", "log"]

    print(f"Cleaning ROS2 workspace at: {workspace_path}\n")

    # Delete directories if they exist
    for d in dirs_to_clean:
        path = workspace_path / d
        if path.exists():
            print(f"Deleting {path} ...")
            shutil.rmtree(path)
        else:
            print(f"{path} does not exist, skipping.")

    # Unset environment variables that may point to the old workspace
    env_vars = ["AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH", "LD_LIBRARY_PATH", "PYTHONPATH"]
    for var in env_vars:
        os.environ.pop(var, None)

# -----------------------------
# Launch files inspection
# -----------------------------
def get_launch_arguments_for_file(launch_path: Path):
    """
    Return the declared launch arguments for a single launch file.

    - For Python files: returns list of arguments (safe subprocess execution).
    - For non-Python launch files (.xml, .yaml): returns empty list.
    """
    if launch_path.suffix == ".py":
        return get_launch_args_from_file(launch_path)
    elif launch_path.suffix in [".yaml", ".xml"]:
        return []  # Non-Python launch files, cannot parse
    else:
        return []  # Unknown extension, treat as empty

def list_launch_arguments(workspace_path: Path, exclude_dirs={'build', 'install', 'log'}):
    """
    List all launch files and their declared arguments in a ROS 2 workspace.
    - Python files are parsed in a subprocess.
    - XML/YAML launch files are listed with empty argument lists.
    """
    valid_exts = (".py")
    exclude_dirs = set(exclude_dirs or [])
    launch_info = {}

    for root, _, files in os.walk(workspace_path):
        rel_root = Path(root).relative_to(workspace_path)

        # Skip excluded directories
        if any(part in exclude_dirs for part in rel_root.parts):
            continue

        if "launch" not in root:
            continue  # only scan launch directories

        for f in files:
            if not f.endswith(valid_exts):
                continue

            launch_path = Path(root) / f
            rel_path = str(launch_path.relative_to(workspace_path))

            # Delegate to helper function
            launch_info[rel_path] = get_launch_arguments_for_file(launch_path)

    return launch_info

# -----------------------------
# GIT diff checks
# -----------------------------
def get_folder_status(repo_path: Path, remote_url: str, branch: str, folders_to_check: list):
    """
    Determine the status of folders relative to a remote branch.
    Returns a dict: {folder: FolderStatus}
    """
    repo_path = repo_path.resolve()
    temp_ref = f"refs/remotes/temp_remote/{branch}"

    # Fetch remote branch
    subprocess.run(
        ["git", "fetch", "--depth=1", remote_url, f"{branch}:{temp_ref}"],
        cwd=repo_path,
        check=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )

    folder_status = {}
    for folder in folders_to_check:
        folder_path = Path(folder).as_posix()

        # Local changes: local branch not in remote
        result_local = subprocess.run(
            ["git", "diff", "--name-only", f"{temp_ref}..HEAD", "--", folder_path],
            cwd=repo_path, capture_output=True, text=True
        )
        local_changed = bool(result_local.stdout.strip())

        # Remote changes: remote branch not in local
        result_remote = subprocess.run(
            ["git", "diff", "--name-only", f"HEAD..{temp_ref}", "--", folder_path],
            cwd=repo_path, capture_output=True, text=True
        )
        remote_changed = bool(result_remote.stdout.strip())

        if local_changed and remote_changed:
            folder_status[folder] = FolderStatus.BOTH
        elif local_changed:
            folder_status[folder] = FolderStatus.LOCAL
        elif remote_changed:
            folder_status[folder] = FolderStatus.REMOTE
        else:
            folder_status[folder] = FolderStatus.UNCHANGED

    # Clean up temporary ref
    subprocess.run(["git", "update-ref", "-d", temp_ref], cwd=repo_path,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    return folder_status

def emoji_label_align(emoji: str, text: str, width=20) -> str:
    return f"{emoji} {text.ljust(width)}"    

# -----------------------------
# ZIP workspace
# -----------------------------
def zip_workspace(workspace_path: Path, output_file: Path, exclude_dirs=None):
    """
    Create a ZIP archive of a ROS 2 workspace.

    :param workspace_path: Path to the ROS 2 workspace.
    :param output_file: Output ZIP file path.
    :param exclude_dirs: List of top-level folders to omit (default: ['build', 'install', 'log', '.git']).
    """
    exclude_dirs = set(exclude_dirs or ["build", "install", "log", ".git"])
    workspace_path = workspace_path.resolve()
    output_file = output_file.resolve()

    print(f"Zipping workspace {workspace_path} -> {output_file}")
    with zipfile.ZipFile(output_file, "w", zipfile.ZIP_DEFLATED) as zf:
        for root, _, files in os.walk(workspace_path):
            rel_root = Path(root).relative_to(workspace_path)
            
            # Skip if any top-level folder is in the exclusion list
            if rel_root.parts and rel_root.parts[0] in exclude_dirs:
                continue

            for f in files:
                file_path = Path(root) / f
                arcname = file_path.relative_to(workspace_path)
                zf.write(file_path, arcname)

    print("✅ Workspace zipped successfully.")

def get_folder_status_with_diff(repo_path: Path, remote_url: str, branch: str, folders_to_check: list):
    """
    Determine folder status and capture local diffs relative to remote branch.
    
    Returns:
        dict: {folder: (FolderStatus, diff_text)}
    """
    repo_path = repo_path.resolve()
    temp_ref = f"refs/remotes/temp_remote/{branch}"

    # Fetch remote branch into temporary ref
    subprocess.run(
        ["git", "fetch", "--depth=1", remote_url, f"{branch}:{temp_ref}"],
        cwd=repo_path,
        check=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )

    folder_info = {}
    for folder in folders_to_check:
        folder_path = Path(folder).as_posix()

        # Local changes
        result_local = subprocess.run(
            ["git", "diff", "--name-only", f"{temp_ref}..HEAD", "--", folder_path],
            cwd=repo_path, capture_output=True, text=True
        )
        local_changed = bool(result_local.stdout.strip())

        # Remote changes
        result_remote = subprocess.run(
            ["git", "diff", "--name-only", f"HEAD..{temp_ref}", "--", folder_path],
            cwd=repo_path, capture_output=True, text=True
        )
        remote_changed = bool(result_remote.stdout.strip())

        # Determine status
        if local_changed and remote_changed:
            status = FolderStatus.BOTH
        elif local_changed:
            status = FolderStatus.LOCAL
        elif remote_changed:
            status = FolderStatus.REMOTE
        else:
            status = FolderStatus.UNCHANGED

        # Capture diff for local changes
        diff_text = ""
        if status in [FolderStatus.LOCAL, FolderStatus.BOTH]:
            diff_result = subprocess.run(
                ["git", "diff", f"{temp_ref}", "--", folder_path],
                cwd=repo_path, capture_output=True, text=True
            )
            diff_text = diff_result.stdout

        folder_info[folder] = (status, diff_text)

    # Clean up temporary ref
    subprocess.run(
        ["git", "update-ref", "-d", temp_ref],
        cwd=repo_path, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )

    return folder_info

def color_diff(diff_text: str) -> str:
    colored_lines = []
    for line in diff_text.splitlines():
        if line.startswith("+") and not line.startswith("+++"):
            colored_lines.append(f"\033[92m{line}\033[0m")  # green for additions
        elif line.startswith("-") and not line.startswith("---"):
            colored_lines.append(f"\033[91m{line}\033[0m")  # red for deletions
        else:
            colored_lines.append(line)
    return "\n".join(colored_lines)

# -----------------------------
# devcontainer.json parsing
# -----------------------------
def remove_json_comments(text: str) -> str:
    """Remove // and /* */ comments so JSON can be parsed."""
    # Remove // comments
    text = re.sub(r'//.*', '', text)
    # Remove /* */ comments (if present)
    text = re.sub(r'/\*.*?\*/', '', text, flags=re.DOTALL)
    return text

def parse_devcontainer_json(file_path: str):
    """Parse a devcontainer.json file and extract apt, pip, and scenarios."""
    file_path = Path(file_path)
    if not file_path.exists():
        print(f"❌ File not found: {file_path}")
        return None

    try:
        raw = file_path.read_text(encoding='utf-8')
    except Exception as e:
        print(f"❌ Could not read file: {e}")
        return None

    # Clean comments
    clean_json = remove_json_comments(raw)

    # Parse JSON safely
    try:
        data = json.loads(clean_json)
    except json.JSONDecodeError as e:
        print(f"❌ Malformed JSON: {e.msg} (line {e.lineno}, column {e.colno})")
        return None

    # Extract fields safely
    auro = data.get("customizations", {}).get("auro", {})
    return {
        "apt": auro.get("apt", []),
        "pip": auro.get("pip", []),
        "scenarios": auro.get("scenarios", [])
    }

#
# Scenario validation
#
def validate_parameters(scenarios, allowed_params):
    """Check each scenario's parameters against the allowed set."""
    all_ok = True
    for s in scenarios:
        name = s.get("name", "Unnamed Scenario")
        params = s.get("parameters", {}) or {}
        invalid = [p for p in params.keys() if p not in allowed_params]

        if invalid:
            all_ok = False
            print(f"⚠️ Scenario '{name}' has invalid parameters: {invalid}")
        else:
            print(f"✅ Scenario '{name}' parameters are valid.")

    if all_ok:
        print("\n✅ All scenarios passed parameter validation.")
    else:
        print("\n❌ Some scenarios contain invalid parameters.")

def get_scenario_by_name(scenarios, name):
    """Return a scenario dict matching the given name (case-insensitive)."""
    for s in scenarios:
        if s.get("name", "").lower() == name.lower():
            return s
    return None

def get_scenario_by_index(scenarios, index):
    """
    Return a scenario by its 1-based index (the same numbering you print).
    Converts string indices to int automatically.
    """
    if not scenarios:
        print("⚠️ No scenarios available.")
        return None

    try:
        idx = int(index)  # convert to integer
    except ValueError:
        print(f"⚠️ Invalid index '{index}': must be an integer")
        return None

    if idx < 1 or idx > len(scenarios):
        print(f"⚠️ Invalid index {idx}. Must be between 1 and {len(scenarios)}.")
        return None

    return scenarios[idx - 1]  # convert to 0-based

def run_launch_with_params(package_name, launch_file_name, params_dict):
    """
    Run a ROS2 launch file programmatically with LaunchService and parameters.
    """
    # Locate the launch file
    package_share = get_package_share_directory(package_name)
    launch_file = Path(package_share) / "launch" / launch_file_name

    if not launch_file.exists():
        raise FileNotFoundError(f"Launch file not found: {launch_file}")

    # Include launch file and pass parameters as launch arguments
    launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_file)),
        launch_arguments=[(k, str(v)) for k, v in params_dict.items()]
    )

    # Create a LaunchService and run
    ls = LaunchService()
    ls.include_launch_description(launch_description)

    print(f"🚀 Launching {launch_file_name} with parameters:")
    for k, v in params_dict.items():
        print(f"   {k}: {v}")

    ls.run()

# -----------------------------
# Apt/pip package dependency install
# -----------------------------
def install_apt_packages(packages):
    """
    Install APT packages using sudo apt-get install -y.
    """
    if not packages:
        print("No APT packages to install.")
        return

    print(f"Installing APT packages: {packages}")
    try:
        # Update first
        subprocess.run(["sudo", "apt-get", "update"], check=True)
        # Install packages
        subprocess.run(["sudo", "apt-get", "install", "-y"] + packages, check=True)
        print("✅ APT packages installed successfully.")
    except subprocess.CalledProcessError as e:
        print(f"❌ Error installing APT packages: {e}")

def install_pip_packages(packages):
    """
    Install PIP packages using pip3.
    """
    if not packages:
        print("No PIP packages to install.")
        return

    print(f"Installing PIP packages: {packages}")
    try:
        subprocess.run([sys.executable, "-m", "pip", "install"] + packages, check=True)
        print("✅ PIP packages installed successfully.")
    except subprocess.CalledProcessError as e:
        print(f"❌ Error installing PIP packages: {e}")

# -----------------------------
# Main CLI
# -----------------------------
def main():
    parser = argparse.ArgumentParser(description=f"AURO2025 assessment workspace CLI tool [version: {version}]")
    subparsers = parser.add_subparsers(dest="command", required=True, help="Subcommands")

    # Build command
    build_parser = subparsers.add_parser("build", help="Build a ROS 2 workspace")
    build_parser.add_argument("workspace", nargs="?", default=".", help="Path to ROS 2 workspace directory (default: current folder)")
    build_parser.add_argument("--colcon-args", nargs=argparse.REMAINDER, help="Extra arguments for colcon build (default: symlink-install)")

    # Check submission
    check_parser = subparsers.add_parser("check-submission", help="Check if workspace is consistent prior to submission")
    check_parser.add_argument("workspace", nargs="?", default=".", help="Path to ROS 2 workspace")

    # Check ZIP command
    check_zip_parser = subparsers.add_parser("check-zip", help="Check if a ZIP file is a ROS 2 workspace")
    check_zip_parser.add_argument("zipfile", help="Path to ROS 2 workspace ZIP")

    # Clean ROS2 workspace
    clean_build_parser = subparsers.add_parser("clean", help="Clean ROS2 workspace")
    clean_build_parser.add_argument("workspace", nargs="?", default=".", help="Path to ROS 2 workspace directory (default: current folder)")

    # Install dependencies command
    install_dependencies_parser = subparsers.add_parser("install-dependencies", help="Install apt/pip dependencies as specified in devcontainer.json file")
    install_dependencies_parser.add_argument("workspace", nargs="?", default=".", help="Path to ROS 2 workspace directory (default: current folder)")

    # List launch command
    list_launch_parser = subparsers.add_parser("list-launch", help="List launch files and their arguments")
    list_launch_parser.add_argument("workspace", nargs="?", default=".", help="Path to ROS 2 workspace directory (default: current folder)")

    # List scenarios command
    scenarios_parser = subparsers.add_parser("list-scenarios", help="List all scenarios defined in devcontainer.json file")
    scenarios_parser.add_argument("workspace", nargs="?", default=".", help="Path to ROS 2 workspace directory (default: current folder)")

    # Run scenario command
    run_scenario_parser = subparsers.add_parser("run-scenario", help="Run a scenario specified in the devcontainer.json file")
    run_scenario_parser.add_argument("workspace", nargs="?", default=".", help="Path to ROS 2 workspace directory (default: current folder)")
    run_scenario_parser.add_argument("scenario", help="Natural number specifying which scenario to execute, ordered according to the list specified in devcontainer.json file")

    # Zip workspace command
    zip_parser = subparsers.add_parser("zip-workspace", help="Create a ZIP archive of the workspace, omitting .git and build folders.")
    zip_parser.add_argument("workspace", nargs="?", default=".", help="Path to ROS 2 workspace directory (default: current folder)")
    zip_parser.add_argument("output_file", help="Output ZIP file path")

    argcomplete.autocomplete(parser)
    args = parser.parse_args()

    if args.command == "build":
        workspace_path = Path(args.workspace).resolve()
        if not workspace_path.is_dir() or not is_ros2_workspace(workspace_path):
            print(f"❌ Not a valid ROS 2 workspace: {workspace_path}")
            sys.exit(1)
        print(f"✅ ROS 2 workspace detected: {workspace_path}")
        colcon_build(workspace_path, args.colcon_args)
    
    elif args.command == "clean":
        workspace_path = Path(args.workspace).resolve()
        if not workspace_path.is_dir() or not is_ros2_workspace(workspace_path):
            print(f"❌ Not a valid ROS 2 workspace: {workspace_path}")
            sys.exit(1)
        clean_ros2_workspace(workspace_path)

    elif args.command == "list-launch":
        workspace_path = Path(args.workspace).resolve()
        if not workspace_path.is_dir() or not is_ros2_workspace(workspace_path):
            print(f"❌ Not a valid ROS 2 workspace: {workspace_path}")
            sys.exit(1)
        launch_info = list_launch_arguments(workspace_path)
        if not launch_info:
            print("No launch files found.")
        else:
            for file, arguments in launch_info.items():
                print(f"{file}: {arguments}")

    elif args.command == "check-zip":
        zip_path = Path(args.zipfile).resolve()
        if not zip_path.is_file() or zip_path.suffix != ".zip":
            print(f"❌ ZIP file not found: {zip_path}")
            sys.exit(1)
        if is_ros2_workspace_zip(zip_path):
            print(f"✅ ZIP seems to be a valid ROS 2 workspace: {zip_path}")
        else:
            print(f"❌ Not a valid ROS 2 workspace ZIP: {zip_path}")
            sys.exit(1)

    elif args.command == "zip-workspace":
        workspace_path = Path(args.workspace).resolve()
        if not workspace_path.is_dir() or not is_ros2_workspace(workspace_path):
            print(f"❌ Not a valid ROS 2 workspace: {workspace_path}")
            sys.exit(1)
        output_file = Path(args.output_file).resolve()
        zip_workspace(workspace_path, output_file)

    elif args.command == "check-json":
        workspace_path = Path(args.workspace).resolve()
        devcontainer_file = workspace_path / ".devcontainer/devcontainer.json"
        result = parse_devcontainer_json(devcontainer_file)

    elif args.command == "list-scenarios":

        # List all scenarios and check that solution_launch.py supports all listed parameters.
        workspace_path = Path(args.workspace).resolve()
        devcontainer_file = workspace_path / ".devcontainer/devcontainer.json"
        result = parse_devcontainer_json(devcontainer_file)

        solution_launch_file = workspace_path / "solution/launch/solution_launch.py"
        launch_info = get_launch_arguments_for_file(solution_launch_file)

        if not launch_info:
            print(f"{RED}❌ No file '{solution_launch_file}' found.{RESET}")
            sys.exit(1)

        if result:
            print(f"Scenarios defined in '{devcontainer_file}':")

            for i, s in enumerate(result["scenarios"], 1):
                name = s.get("name", f"Scenario {i}")
                desc = s.get("description", "(no description)")
                params = s.get("parameters", {}) or {}

                print(f"  {i}. {name}")
                print(f"     Description: {desc}")
                print(f"     Parameters: {params}")

                invalid = [p for p in params.keys() if p not in launch_info]
                if invalid:
                    all_ok = False
                    print(f"     ⚠️ Invalid parameters not found in launch file: {invalid}")
                else:
                    print(f"     ✅ Parameters valid")

    elif args.command == "install-dependencies":
        
        workspace_path = Path(args.workspace).resolve()
        devcontainer_file = workspace_path / ".devcontainer/devcontainer.json"
        result = parse_devcontainer_json(devcontainer_file)

        if result:
            install_apt_packages(result["apt"])
            install_pip_packages(result["pip"])

    elif args.command == "run-scenario":

        # Get all scenarios.
        workspace_path = Path(args.workspace).resolve()
        devcontainer_file = workspace_path / ".devcontainer/devcontainer.json"
        result = parse_devcontainer_json(devcontainer_file)

        solution_launch_file = workspace_path / "solution/launch/solution_launch.py"
        launch_info = get_launch_arguments_for_file(solution_launch_file)

        if not launch_info:
            print(f"{RED}❌ No file '{solution_launch_file}' found.{RESET}")
            sys.exit(1)

        if result:
            scenario = get_scenario_by_index(result["scenarios"], args.scenario)
            
            if scenario:
                params = scenario.get("parameters", {}) or {}
                run_launch_with_params(
                    package_name="solution",
                    launch_file_name="solution_launch.py",
                    params_dict=params
                )
            else:
                print(f"{RED}❌ No scenario with index {args.scenario} found.{RESET}")
        else:
            print(f"{RED}❌ No scenarios found.{RESET}")

    elif args.command == "check-submission":
        workspace_path = Path(args.workspace).resolve()

        if not workspace_path.is_dir() or not is_ros2_workspace(workspace_path):
            print(f"❌ Not a valid ROS 2 workspace: {workspace_path}, skipping checking of submission.")
            sys.exit(1)

        solution_launch_file = workspace_path / "solution/launch/solution_launch.py"
        launch_info = get_launch_arguments_for_file(solution_launch_file)

        if not launch_info:
            print(f"{RED}❌ No file '{solution_launch_file}' found. ROS2 workspace should have a valid launch file named 'solution_launch.py' under package 'solution'.{RESET}")
            sys.exit(1)
        else:
            # Validate that all expected launch arguments are in the set!
            missing = []
            for param in required_params: 
                if param not in launch_info:
                    missing.append(param)

            if len(missing) > 0:
                print(f"❌ Required parameters {missing} are missing from '{solution_launch_file}', please fix before submission.")

        devcontainer_file = workspace_path / ".devcontainer/devcontainer.json"
        result = parse_devcontainer_json(devcontainer_file)

        if result:
            all_ok = True
            invalid_scenarios = []

            if len(result["scenarios"]) == 0:
                print(f"     ⚠️ No scenarios found.")

            for i, s in enumerate(result["scenarios"], 1):
                name = s.get("name", f"Scenario {i}")
                desc = s.get("description", "(no description)")
                params = s.get("parameters", {}) or {}

                invalid = [p for p in params.keys() if p not in launch_info]
                if invalid:
                    all_ok = False
                    invalid_scenarios.append(name)

            if not all_ok:
                print(f"     ⚠️ Invalid scenarios found: {invalid_scenarios}. You may want to check with 'list-scenarios'")

        if not (workspace_path / ".git").exists():
            print(f"❌ Not a Git repository: {workspace_path}, so skipping checking of modifications to ROS2 packages.")
            sys.exit(1)

        # Replace repo and branch with your variables
        folders_to_check = {'assessment', 'assessment_interfaces', 'utils'}
        folders_to_report = {'assessment', 'assessment_interfaces'}
        tool_folder = {'rcutil.py'}
        folder_info = get_folder_status_with_diff(workspace_path, "https://github.com/UoY-RoboStar/AURO2025", "main", folders_to_check)

        # Status labels and colors
        STATUS_LABELS = {
            FolderStatus.LOCAL: ("LOCAL", RED),
            FolderStatus.REMOTE: ("REMOTE", BLUE),
            FolderStatus.BOTH: ("BOTH", YELLOW),
            FolderStatus.UNCHANGED: ("OK", GREEN),
        }

        has_local_changes = False
        tool_changes = False
        changed = []
        max_len = max(len(f) for f in folders_to_check)

        for folder, (status, diff) in folder_info.items():

            if folder in tool_folder and status in [FolderStatus.REMOTE]:
                tool_changes = True

            if folder not in folders_to_report:
                continue

            label, color = STATUS_LABELS[status]
            print(f"{color}Changes detected in folder:{RESET} {folder.ljust(max_len)}")

            if status in [FolderStatus.LOCAL, FolderStatus.BOTH] and diff:
                # print("\n".join(color_diff(diff)))
                has_local_changes = True
                changed.append(folder)

        if tool_changes:
            print(f"\n{YELLOW}⚠️  There are updates for this tool. You may want to consider updating it from the AURO2025 repository.{RESET}")

        # Extra messages for context
        for folder, (status, _) in folder_info.items():
            if status in [FolderStatus.REMOTE, FolderStatus.BOTH]:
                if folder in folders_to_report:
                    print(f"\n{YELLOW}⚠️  You should check whether changes to the 'assessment' or 'assessment_interfaces' packages in the AURO2025 repository should be considered prior to submitting your work.{RESET}")
                    break  # only show once

        if has_local_changes:
            print(f"\n{RED}❌ You have local changes that are unexpected or disallowed.{RESET}")
            sys.exit(1)
        else:
            print(f"\n{GREEN}✅ Assessment packages have not been localy changed with respect to the AURO2025 repository's main branch.{RESET}")

if __name__ == "__main__":
    main()
