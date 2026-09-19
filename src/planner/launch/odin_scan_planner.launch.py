"""Compatibility name for this workspace's BTC/ICP navigation launch."""
import importlib.util
from pathlib import Path


def generate_launch_description():
    spec = importlib.util.spec_from_file_location(
        "scan_navigation_launch", Path(__file__).with_name("navigation.launch.py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()
