from typing import TypeVar, Generic, List, Iterator, Set

T = TypeVar("T")

# Explicit implementation in Python of ran and extract RoboChart functions over Sequences, that use 1-based indexing.
class Seq(Generic[T]):
    def __init__(self, data: List[T]):
        self.data = data

    def __getitem__(self, index: int) -> T:
        return self.data[index - 1]

    def __setitem__(self, index: int, value: T) -> None:
        self.data[index - 1] = value

    def __len__(self) -> int:
        return len(self.data)

    def __iter__(self) -> Iterator[T]:
        return iter(self.data)

    def __repr__(self) -> str:
        return f"{self.data}"

# Returns the subset of the list, in order, containing the elements at indices in s. 
def extract(s:Set[int], l:Seq[T]):
    return [l[i] for i in sorted(s)]

# Returns the elements of l as a set
def ran(l:Seq[T]):
    return set(l)