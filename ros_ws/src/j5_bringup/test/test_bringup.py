import importlib.util
from pathlib import Path

from launch import LaunchService


def test_default_bringup_exits_without_starting_placeholder():
    path = Path(__file__).parents[1] / "launch" / "bringup.launch.py"
    spec = importlib.util.spec_from_file_location("bringup", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    service = LaunchService()
    service.include_launch_description(module.generate_launch_description())
    assert service.run() == 0
