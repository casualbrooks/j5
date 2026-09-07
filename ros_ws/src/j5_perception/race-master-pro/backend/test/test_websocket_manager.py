import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from app.websocket_manager import ConnectionManager


def test_counts_only_connected_workers_with_requested_capability():
    manager = ConnectionManager()
    standalone_worker = object()
    ros_worker = object()
    disconnected_worker = object()
    manager.active_connections["cv_system"] = [standalone_worker, ros_worker]
    manager.register_worker_capabilities(
        standalone_worker, ["configure:live", "configure:video"]
    )
    manager.register_worker_capabilities(ros_worker, ["detections:ros-topic"])
    manager.register_worker_capabilities(disconnected_worker, ["configure:video"])

    assert manager.capable_worker_count("configure:live") == 1
    assert manager.capable_worker_count("configure:video") == 1
    assert manager.capable_worker_count("detections:ros-topic") == 1
