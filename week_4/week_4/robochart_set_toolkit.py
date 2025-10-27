
from typing import TypeVar, Set

T = TypeVar("T")

def union(a:Set[T], b:Set[T]):
    return a | b