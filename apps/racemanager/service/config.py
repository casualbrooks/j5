"""Configuration for the race manager service."""

from __future__ import annotations

import os
from functools import lru_cache
from pathlib import Path
from typing import Optional

from dotenv import load_dotenv


ENV_PATH = Path(__file__).resolve().parent / ".env"
load_dotenv(ENV_PATH)


class Settings:
    """Runtime configuration sourced from environment variables."""

    def __init__(
        self,
        *,
        db_backend: Optional[str] = None,
        atlas_uri: Optional[str] = None,
        race_db_name: Optional[str] = None,
        laps_collection: Optional[str] = None,
        sqlite_path: Optional[str] = None,
        websocket_public_url: Optional[str] = None,
        http_host: Optional[str] = None,
        http_port: Optional[int] = None,
    ) -> None:
        self.db_backend = (db_backend or os.getenv("DB_BACKEND", "sqlite")).lower()
        self.atlas_uri = atlas_uri or os.getenv(
            "ATLAS_URI", "mongodb://localhost:27017"
        )
        self.race_db_name = race_db_name or os.getenv("RACE_DB_NAME", "j5_racing")
        self.laps_collection = laps_collection or os.getenv(
            "LAPS_COLLECTION", "laps_live"
        )
        self.sqlite_path = sqlite_path or os.getenv(
            "SQLITE_PATH", "apps/racemanager/service/data/race_manager.db"
        )
        self.websocket_public_url = websocket_public_url or os.getenv("WS_PUBLIC_URL")
        self.http_host = http_host or os.getenv("HTTP_HOST", "0.0.0.0")
        self.http_port = http_port or int(os.getenv("HTTP_PORT", "4000"))

    def dict(self, *, include_sensitive: bool = True) -> dict[str, Optional[str | int]]:
        config = {
            "db_backend": self.db_backend,
            "atlas_uri": self.atlas_uri,
            "race_db_name": self.race_db_name,
            "laps_collection": self.laps_collection,
            "sqlite_path": self.sqlite_path,
            "websocket_public_url": self.websocket_public_url,
            "http_host": self.http_host,
            "http_port": self.http_port,
        }
        if not include_sensitive:
            config["atlas_uri"] = "<redacted>"
        return config


@lru_cache(maxsize=1)
def get_settings() -> Settings:
    return Settings()
