"""
Configuration Package

Contains system configuration and parameters:
- parameters.json: System parameters and specifications
"""

import json
import os


def load_parameters():
    """
    Load system parameters from JSON configuration file.
    
    Returns:
        params: Dictionary containing all system parameters
    """
    config_path = os.path.join(os.path.dirname(__file__), 'parameters.json')
    with open(config_path, 'r') as f:
        return json.load(f)


__all__ = ['load_parameters']
