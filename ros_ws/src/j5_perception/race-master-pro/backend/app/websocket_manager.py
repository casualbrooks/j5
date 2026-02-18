"""
WebSocket connection manager.
Handles client groups (spectator, organizer, cv_system), broadcasting, and heartbeat.
"""

import asyncio
import json
from typing import Optional
from fastapi import WebSocket
from datetime import datetime


class ConnectionManager:
    def __init__(self):
        self.active_connections: dict[str, list[WebSocket]] = {
            "spectator": [],
            "organizer": [],
            "cv_system": [],
        }

    async def connect(self, websocket: WebSocket, client_type: str = "spectator"):
        await websocket.accept()
        if client_type not in self.active_connections:
            client_type = "spectator"
        self.active_connections[client_type].append(websocket)
        await self.broadcast_connection_count()

    def disconnect(self, websocket: WebSocket, client_type: str = "spectator"):
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


# Global singleton
manager = ConnectionManager()
