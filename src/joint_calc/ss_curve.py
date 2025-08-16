"""
This module contains functions for generating and manipulating stress-strain curves.
"""
import enum

class ElasticStressStrainCurveType(enum.Enum):
    """
    Enum for different types of stress-strain curves.
    """
    C18_ELASTIC_TRANS = 300 #MPa
    C24_ELASTIC_TRANS = 370 #MPa
    D30_ELASTIC_TRANS = 640 #MPa

class StressStrainCurveType:
    def __init__(self, elastic: bool,
                 curve_type: ElasticStressStrainCurveType):
        self.elastic = elastic
        self.curve_type = curve_type

class StressStrainCurve:
    def __init__(self, curve_type: StressStrainCurveType):
        self.curve_type = curve_type

    def get_stress(self, strain: float) -> float:
        """
        Get the stress value for a given strain based on the curve type.
        """
        if self.curve_type.elastic:
            return self.curve_type.curve_type.value * strain
        else:
            raise ValueError("Non-elastic curve types are not supported")