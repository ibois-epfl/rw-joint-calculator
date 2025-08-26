import Rhino


def volume_lambda(brep):
    volume = abs(Rhino.Geometry.VolumeMassProperties.Compute(brep).Volume)
    return volume


def surface_lambda(brep):
    return Rhino.Geometry.AreaMassProperties.Compute(brep).Area
