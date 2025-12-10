"""Configuration for the race manager service.

The settings are intentionally minimal to keep them compatible with
Atlas or a local Mongo instance. They align with the environment
variables referenced in the existing documentation.
"""

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
        atlas_uri: Optional[str] = None,
        race_db_name: Optional[str] = None,
        laps_collection: Optional[str] = None,
        websocket_public_url: Optional[str] = None,
        http_host: Optional[str] = None,
        http_port: Optional[int] = None,
    ) -> None:
        self.atlas_uri = atlas_uri or os.getenv(
            "ATLAS_URI", "mongodb://localhost:27017"
        )
        self.race_db_name = race_db_name or os.getenv("RACE_DB_NAME", "j5_racing")
        self.laps_collection = laps_collection or os.getenv(
            "LAPS_COLLECTION", "laps_live"
        )
        self.websocket_public_url = websocket_public_url or os.getenv("WS_PUBLIC_URL")
        self.http_host = http_host or os.getenv("HTTP_HOST", "0.0.0.0")
        self.http_port = http_port or int(os.getenv("HTTP_PORT", "4000"))

    def dict(self, *, include_sensitive: bool = True) -> dict[str, Optional[str | int]]:
        config = {
            "atlas_uri": self.atlas_uri,
            "race_db_name": self.race_db_name,
            "laps_collection": self.laps_collection,
            "websocket_public_url": self.websocket_public_url,
            "http_host": self.http_host,
            "http_port": self.http_port,
        }
        if not include_sensitive:
            config["atlas_uri"] = "<redacted>"
        return config


@lru_cache(maxsize=1)
def get_settings() -> Settings:
    """Return a cached Settings instance."""

    return Settings()
