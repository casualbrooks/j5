"""Configuration for the race manager service.

The settings are intentionally minimal to keep them compatible with
Atlas or a local Mongo instance. They align with the environment
variables referenced in the existing documentation.
"""

from __future__ import annotations

import os
from functools import lru_cache
from typing import Optional


class Settings:
    """Runtime configuration sourced from environment variables."""

    def __init__(
        self,
        *,
        atlas_uri: Optional[str] = None,
        race_db_name: Optional[str] = None,
        laps_collection: Optional[str] = None,
        websocket_public_url: Optional[str] = None,
    ) -> None:
        self.atlas_uri = atlas_uri or os.getenv(
            "ATLAS_URI", "mongodb://localhost:27017"
        )
        self.race_db_name = race_db_name or os.getenv("RACE_DB_NAME", "j5_racing")
        self.laps_collection = laps_collection or os.getenv(
            "LAPS_COLLECTION", "laps_live"
        )
        self.websocket_public_url = websocket_public_url or os.getenv("WS_PUBLIC_URL")

    def dict(self) -> dict[str, Optional[str]]:
        return {
            "atlas_uri": self.atlas_uri,
            "race_db_name": self.race_db_name,
            "laps_collection": self.laps_collection,
            "websocket_public_url": self.websocket_public_url,
        }


@lru_cache(maxsize=1)
def get_settings() -> Settings:
    """Return a cached Settings instance."""

    return Settings()
