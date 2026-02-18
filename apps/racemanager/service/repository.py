"""Repository helpers for laps and standings.

Supports either MongoDB (`backend="mongo"`) or SQLite (`backend="sqlite"`).
SQLite is the default for local standalone deployments.
"""

from __future__ import annotations

import sqlite3
from abc import ABC, abstractmethod
from datetime import datetime
from pathlib import Path
from typing import Iterable, List

from .schemas import LapIngest, LeaderboardEntry, LeaderboardResponse


class RaceRepository(ABC):
    @abstractmethod
    def insert_lap(self, lap: LapIngest, speed_kph: float) -> str:
        raise NotImplementedError

    @abstractmethod
    def leaderboard(self, race_id: str) -> LeaderboardResponse:
        raise NotImplementedError


class SQLiteRaceRepository(RaceRepository):
    def __init__(self, db_path: str) -> None:
        self.db_path = Path(db_path)
        self.db_path.parent.mkdir(parents=True, exist_ok=True)
        self.conn = sqlite3.connect(str(self.db_path), check_same_thread=False)
        self.conn.row_factory = sqlite3.Row
        self._ensure_schema()

    def _ensure_schema(self) -> None:
        self.conn.execute(
            """
            CREATE TABLE IF NOT EXISTS laps_live (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                race_id TEXT NOT NULL,
                car_id TEXT NOT NULL,
                session_lap INTEGER NOT NULL,
                lap_time_ms INTEGER NOT NULL,
                track_distance_m REAL NOT NULL,
                timestamp TEXT NOT NULL,
                speed_kph REAL NOT NULL,
                is_valid INTEGER NOT NULL,
                source TEXT NOT NULL,
                replay_id TEXT,
                on_track INTEGER NOT NULL,
                direction_ok INTEGER NOT NULL,
                min_lap_time_ok INTEGER NOT NULL
            )
            """
        )
        self.conn.execute(
            """
            CREATE INDEX IF NOT EXISTS idx_laps_race_car
            ON laps_live (race_id, car_id)
            """
        )
        self.conn.execute(
            """
            CREATE INDEX IF NOT EXISTS idx_laps_race_timestamp
            ON laps_live (race_id, timestamp)
            """
        )
        self.conn.commit()

    def insert_lap(self, lap: LapIngest, speed_kph: float) -> str:
        cursor = self.conn.execute(
            """
            INSERT INTO laps_live (
                race_id, car_id, session_lap, lap_time_ms, track_distance_m,
                timestamp, speed_kph, is_valid, source, replay_id,
                on_track, direction_ok, min_lap_time_ok
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
            (
                lap.raceId,
                lap.carId,
                lap.sessionLap,
                lap.lapTimeMs,
                lap.trackDistanceM,
                lap.timestamp.isoformat(),
                speed_kph,
                int(lap.validity.is_valid),
                lap.source,
                lap.replayId,
                int(lap.validity.onTrack),
                int(lap.validity.directionOk),
                int(lap.validity.minLapTimeOk),
            ),
        )
        self.conn.commit()
        return str(cursor.lastrowid)

    def leaderboard(self, race_id: str) -> LeaderboardResponse:
        now = datetime.utcnow()
        rows = list(
            self.conn.execute(
                """
                SELECT
                    car_id as carId,
                    COUNT(*) as lapCount,
                    MIN(lap_time_ms) as bestLapMs,
                    AVG(speed_kph) as avgSpeedKph,
                    SUM(lap_time_ms) as totalTimeMs
                FROM laps_live
                WHERE race_id = ?
                GROUP BY car_id
                ORDER BY totalTimeMs ASC
                """,
                (race_id,),
            )
        )
        leaderboard = self._attach_gaps(rows)
        entries = [LeaderboardEntry(**entry) for entry in leaderboard]
        return LeaderboardResponse(raceId=race_id, leaderboard=entries, asOf=now)

    @staticmethod
    def _attach_gaps(rows: Iterable[sqlite3.Row]) -> list[dict]:
        entries = [dict(row) for row in rows]
        if not entries:
            return []
        leader_time = int(entries[0]["totalTimeMs"])
        for entry in entries:
            entry["gapToLeaderMs"] = max(0, int(entry["totalTimeMs"]) - leader_time)
            entry["avgSpeedKph"] = round(float(entry["avgSpeedKph"]), 3)
        return entries


class MongoRaceRepository(RaceRepository):
    def __init__(
        self, uri: str, db_name: str, laps_collection: str = "laps_live"
    ) -> None:
        from pymongo import ASCENDING, MongoClient

        self._client = MongoClient(uri)
        self._laps = self._client[db_name][laps_collection]
        self._laps.create_index(
            [("raceId", ASCENDING), ("carId", ASCENDING)], background=True
        )
        self._laps.create_index(
            [("raceId", ASCENDING), ("timestamp", ASCENDING)], background=True
        )

    def insert_lap(self, lap: LapIngest, speed_kph: float) -> str:
        payload = lap.dict()
        payload.update({"speedKph": speed_kph, "isValid": lap.validity.is_valid})
        result = self._laps.insert_one(payload)
        return str(result.inserted_id)

    def leaderboard(self, race_id: str) -> LeaderboardResponse:
        now = datetime.utcnow()
        pipeline: List[dict] = [
            {"$match": {"raceId": race_id}},
            {
                "$group": {
                    "_id": "$carId",
                    "lapCount": {"$sum": 1},
                    "bestLapMs": {"$min": "$lapTimeMs"},
                    "avgSpeedKph": {"$avg": "$speedKph"},
                    "totalTimeMs": {"$sum": "$lapTimeMs"},
                }
            },
            {"$sort": {"totalTimeMs": 1}},
        ]
        rows = list(self._laps.aggregate(pipeline))
        entries = self._attach_gaps(rows)
        models = [LeaderboardEntry(**entry) for entry in entries]
        return LeaderboardResponse(raceId=race_id, leaderboard=models, asOf=now)

    @staticmethod
    def _attach_gaps(rows: Iterable[dict]) -> list[dict]:
        rows = list(rows)
        if not rows:
            return []
        leader_time = rows[0]["totalTimeMs"]
        for row in rows:
            row["carId"] = row.pop("_id")
            row["gapToLeaderMs"] = max(0, row["totalTimeMs"] - leader_time)
            row["avgSpeedKph"] = round(float(row["avgSpeedKph"]), 3)
        return rows


def get_repository(
    backend: str,
    *,
    mongo_uri: str,
    db_name: str,
    laps_collection: str,
    sqlite_path: str,
) -> RaceRepository:
    backend_normalized = backend.lower()
    if backend_normalized == "sqlite":
        return SQLiteRaceRepository(sqlite_path)
    if backend_normalized == "mongo":
        return MongoRaceRepository(mongo_uri, db_name, laps_collection)
    raise ValueError(f"Unsupported backend: {backend}")
