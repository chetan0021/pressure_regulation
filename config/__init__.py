"""
Configuration Module

Handles loading and validation of system parameters from JSON configuration files.

Author: Industrial Pressure Control System
Date: 2026-02-18
"""

import json
import os
import logging

logger = logging.getLogger(__name__)


def load_parameters(config_file='config/parameters.json'):
    """
    Load system parameters from JSON configuration file.
    
    Args:
        config_file: Path to configuration file
        
    Returns:
        dict: System parameters
    """
    try:
        with open(config_file, 'r') as f:
            params = json.load(f)
        logger.info(f"Parameters loaded from {config_file}")
        return params
    except FileNotFoundError:
        logger.error(f"Configuration file not found: {config_file}")
        raise
    except json.JSONDecodeError as e:
        logger.error(f"Invalid JSON in configuration file: {e}")
        raise


def save_parameters(params, config_file='config/parameters.json'):
    """
    Save system parameters to JSON configuration file.
    
    Args:
        params: System parameters dictionary
        config_file: Path to configuration file
    """
    try:
        with open(config_file, 'w') as f:
            json.dump(params, f, indent=4)
        logger.info(f"Parameters saved to {config_file}")
    except Exception as e:
        logger.error(f"Failed to save parameters: {e}")
        raise


__all__ = ['load_parameters', 'save_parameters']
