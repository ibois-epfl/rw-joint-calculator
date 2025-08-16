"""
a module for basic geometry dataclasses
"""

from dataclasses import dataclass

dataclass
class Point:
    x: float
    y: float
    z: float

@dataclass
class Vector:
    x: float
    y: float
    z: float
