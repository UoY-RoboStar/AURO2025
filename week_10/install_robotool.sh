#!/bin/bash

# Download and install the robotool package
wget -O ~/robotool.zip https://github.com/UoY-RoboStar/robotool/releases/download/v1.1.2025063001/robotool.product-linux.gtk.x86_64.zip && unzip -o ~/robotool.zip -d ~/robotool && rm ~/robotool.zip

# Update package list, install fdr4, and required dependencies for FDR4/RoboTool
sudo add-apt-repository -y ppa:linuxuprising/libpng12
sudo apt update && sudo apt install -y libpng12-0 libtinfo5 libwebkit2gtk-4.0-37

# Add FDR4's GPG key and repository
sudo sh -c 'echo "deb http://dl.cocotec.io/fdr/debian/ fdr release\n" > /etc/apt/sources.list.d/fdr.list'
wget -qO - http://dl.cocotec.io/fdr/linux_deploy.key | sudo apt-key add -
sudo apt update && sudo apt install -y fdr

# Fix TLS certificate store for FDR4
sudo mkdir -p /etc/pki/tls/certs/
sudo ln -f -s /etc/ssl/certs/ca-certificates.crt /etc/pki/tls/certs/ca-bundle.crt


