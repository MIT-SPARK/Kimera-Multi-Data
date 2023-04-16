import importlib

def get(robot):
    """
    Gets the extrinsic tfs of the specified robot.
    
    Parameters
    ----------
    robot : str -- name of robot

    Returns
    -------
    module
    """
    return importlib.import_module(f".tfs.{robot}", __package__)
