"""
WebSocket connection manager.
Handles client groups (spectator, organizer, cv_system), broadcasting, and heartbeat.
"""

import asyncio
import json
from typing import TYPE_CHECKING, Any, Optional
from datetime import datetime

if TYPE_CHECKING:
    from fastapi import WebSocket
else:
    WebSocket = Any


class ConnectionManager:
    def __init__(self):
        self.active_connections: dict[str, list[WebSocket]] = {
            "spectator": [],
            "organizer": [],
            "cv_system": [],
        }
        self.worker_capabilities: dict[int, set[str]] = {}

    async def connect(self, websocket: WebSocket, client_type: str = "spectator"):
        await websocket.accept()
        if client_type not in self.active_connections:
            client_type = "spectator"
        self.active_connections[client_type].append(websocket)
        await self.broadcast_connection_count()

    def disconnect(self, websocket: WebSocket, client_type: str = "spectator"):
        self.worker_capabilities.pop(id(websocket), None)
        if client_type in self.active_connections:
            try:
                self.active_connections[client_type].remove(websocket)
            except ValueError:
                pass

    async def broadcast_connection_count(self):
        """Broadcast current connection counts to all clients."""
        counts = {k: len(v) for k, v in self.active_connections.items()}
        total = sum(counts.values())
        await self.broadcast_all(
            {
                "type": "connectionCount",
                "data": {"total": total, **counts},
                "timestamp": datetime.now().isoformat(),
            }
        )

    async def broadcast_all(self, message: dict):
        """Send message to ALL connected clients."""
        data = json.dumps(message)
        for group in self.active_connections.values():
            dead: list[WebSocket] = []
            for ws in group:
                try:
                    await ws.send_text(data)
                except Exception:
                    dead.append(ws)
            for ws in dead:
                try:
                    group.remove(ws)
                except ValueError:
                    pass

    async def broadcast_to_group(self, message: dict, group: str):
        """Send message to a specific client group."""
        if group not in self.active_connections:
            return
        data = json.dumps(message)
        dead: list[WebSocket] = []
        for ws in self.active_connections[group]:
            try:
                await ws.send_text(data)
            except Exception:
                dead.append(ws)
        for ws in dead:
            try:
                self.active_connections[group].remove(ws)
            except ValueError:
                pass

    async def send_personal(self, websocket: WebSocket, message: dict):
        """Send message to a specific client."""
        try:
            await websocket.send_text(json.dumps(message))
        except Exception:
            pass

    @property
    def total_connections(self) -> int:
        return sum(len(v) for v in self.active_connections.values())

    def register_worker_capabilities(
        self, websocket: WebSocket, capabilities: list[str]
    ) -> None:
        self.worker_capabilities[id(websocket)] = {
            str(capability) for capability in capabilities
        }

    def capable_worker_count(self, capability: str) -> int:
        connected_ids = {
            id(websocket) for websocket in self.active_connections.get("cv_system", [])
        }
        return sum(
            1
            for websocket_id, capabilities in self.worker_capabilities.items()
            if websocket_id in connected_ids and capability in capabilities
        )


# Global singleton
manager = ConnectionManager()
