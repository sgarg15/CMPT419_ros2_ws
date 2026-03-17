"""
Load game and sim parameters from YAML config files.

Config files are installed to share/ros2d2/config/ and loaded via ament_index.
"""

import os
from typing import Dict, Any, Optional

from ros2d2.dynamics import GameParams


def _get_config_path(filename: str) -> Optional[str]:
    """Return path to config file, or None if not found."""
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg_share = get_package_share_directory("ros2d2")
        path = os.path.join(pkg_share, "config", filename)
        if os.path.isfile(path):
            return path
    except Exception:
        pass
    # Fallback: look relative to this package (for running without install)
    this_dir = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(this_dir, "..", "..", "config", filename)
    if os.path.isfile(path):
        return path
    return None


def load_yaml(filename: str) -> Dict[str, Any]:
    """Load YAML file from ros2d2 config. Returns empty dict if file not found."""
    path = _get_config_path(filename)
    if not path:
        return {}
    try:
        import yaml
        with open(path, "r") as f:
            return yaml.safe_load(f) or {}
    except Exception:
        return {}


def load_game_params() -> GameParams:
    """Load GameParams from config/game_params.yaml. Falls back to defaults."""
    data = load_yaml("game_params.yaml")
    return GameParams.from_dict(data)


def load_sim_params() -> Dict[str, Any]:
    """Load sim_params from config/sim_params.yaml."""
    return load_yaml("sim_params.yaml")
