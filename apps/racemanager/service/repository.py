"""Repository helpers for laps, CRUD resources, events, and dashboard backing data.

Supports either MongoDB (`backend="mongo"`) or SQLite (`backend="sqlite"`).
SQLite is the default for local standalone deployments.
"""

from __future__ import annotations

import json
import sqlite3
from abc import ABC, abstractmethod
from datetime import datetime
from pathlib import Path
from typing import Iterable, List, Optional

from .schemas import LeaderboardEntry, LeaderboardResponse, LapIngest, RaceEventIngest


class RaceRepository(ABC):
    @abstractmethod
    def insert_lap(self, lap: LapIngest, speed_kph: float) -> str:
        raise NotImplementedError

    @abstractmethod
    def leaderboard(self, race_id: str) -> LeaderboardResponse:
        raise NotImplementedError

    @abstractmethod
    def create_entity(self, kind: str, record: dict) -> dict:
        raise NotImplementedError

    @abstractmethod
    def list_entities(
        self, kind: str, filters: Optional[dict] = None, *, limit: Optional[int] = None
    ) -> list[dict]:
        raise NotImplementedError

    @abstractmethod
    def get_entity(self, kind: str, entity_id: str) -> dict | None:
        raise NotImplementedError

    @abstractmethod
    def update_entity(self, kind: str, entity_id: str, record: dict) -> dict | None:
        raise NotImplementedError

    @abstractmethod
    def delete_entity(self, kind: str, entity_id: str) -> bool:
        raise NotImplementedError

    @abstractmethod
    def insert_event(self, race_id: str, event: RaceEventIngest) -> str:
        raise NotImplementedError

    @abstractmethod
    def list_events(self, race_id: str, *, limit: int = 20) -> list[dict]:
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
            CREATE TABLE IF NOT EXISTS entities (
                kind TEXT NOT NULL,
                id TEXT NOT NULL,
                data_json TEXT NOT NULL,
                created_at TEXT NOT NULL,
                updated_at TEXT NOT NULL,
                PRIMARY KEY(kind, id)
            )
            """
        )
        self.conn.execute(
            """
            CREATE TABLE IF NOT EXISTS race_events (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                race_id TEXT NOT NULL,
                event_type TEXT NOT NULL,
                car_id TEXT,
                racer_id TEXT,
                checkpoint_id TEXT,
                lap_number INTEGER,
                payload_json TEXT NOT NULL,
                timestamp TEXT NOT NULL
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
        self.conn.execute(
            """
            CREATE INDEX IF NOT EXISTS idx_entities_kind
            ON entities (kind, updated_at DESC)
            """
        )
        self.conn.execute(
            """
            CREATE INDEX IF NOT EXISTS idx_events_race_timestamp
            ON race_events (race_id, timestamp DESC)
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
                WHERE race_id = ? AND is_valid = 1
                GROUP BY car_id
                ORDER BY totalTimeMs ASC
                """,
                (race_id,),
            )
        )
        leaderboard = self._attach_gaps(rows)
        entries = [LeaderboardEntry(**entry) for entry in leaderboard]
        return LeaderboardResponse(raceId=race_id, leaderboard=entries, asOf=now)

    def create_entity(self, kind: str, record: dict) -> dict:
        now = datetime.utcnow().isoformat()
        payload = {
            **record,
            "createdAt": record.get("createdAt", now),
            "updatedAt": now,
        }
        self.conn.execute(
            """
            INSERT INTO entities (kind, id, data_json, created_at, updated_at)
            VALUES (?, ?, ?, ?, ?)
            """,
            (
                kind,
                payload["id"],
                json.dumps(payload),
                payload["createdAt"],
                payload["updatedAt"],
            ),
        )
        self.conn.commit()
        return payload

    def list_entities(
        self, kind: str, filters: Optional[dict] = None, *, limit: Optional[int] = None
    ) -> list[dict]:
        sql = "SELECT data_json FROM entities WHERE kind = ? ORDER BY updated_at DESC"
        params: list[object] = [kind]
        if limit is not None:
            sql += " LIMIT ?"
            params.append(limit)
        rows = [json.loads(row[0]) for row in self.conn.execute(sql, params)]
        if not filters:
            return rows
        return [
            row
            for row in rows
            if all(row.get(key) == value for key, value in filters.items())
        ]

    def get_entity(self, kind: str, entity_id: str) -> dict | None:
        row = self.conn.execute(
            "SELECT data_json FROM entities WHERE kind = ? AND id = ?",
            (kind, entity_id),
        ).fetchone()
        return json.loads(row[0]) if row else None

    def update_entity(self, kind: str, entity_id: str, record: dict) -> dict | None:
        existing = self.get_entity(kind, entity_id)
        if not existing:
            return None
        updated = {
            **existing,
            **record,
            "id": entity_id,
            "createdAt": existing.get("createdAt", datetime.utcnow().isoformat()),
            "updatedAt": datetime.utcnow().isoformat(),
        }
        self.conn.execute(
            """
            UPDATE entities
            SET data_json = ?, updated_at = ?
            WHERE kind = ? AND id = ?
            """,
            (json.dumps(updated), updated["updatedAt"], kind, entity_id),
        )
        self.conn.commit()
        return updated

    def delete_entity(self, kind: str, entity_id: str) -> bool:
        cursor = self.conn.execute(
            "DELETE FROM entities WHERE kind = ? AND id = ?",
            (kind, entity_id),
        )
        self.conn.commit()
        return cursor.rowcount > 0

    def insert_event(self, race_id: str, event: RaceEventIngest) -> str:
        cursor = self.conn.execute(
            """
            INSERT INTO race_events (
                race_id, event_type, car_id, racer_id, checkpoint_id, lap_number, payload_json, timestamp
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?)
            """,
            (
                race_id,
                event.type,
                event.carId,
                event.racerId,
                event.checkpointId,
                event.lapNumber,
                json.dumps(event.payload),
                event.timestamp.isoformat(),
            ),
        )
        self.conn.commit()
        return str(cursor.lastrowid)

    def list_events(self, race_id: str, *, limit: int = 20) -> list[dict]:
        rows = self.conn.execute(
            """
            SELECT id, race_id, event_type, car_id, racer_id, checkpoint_id, lap_number, payload_json, timestamp
            FROM race_events
            WHERE race_id = ?
            ORDER BY timestamp DESC
            LIMIT ?
            """,
            (race_id, limit),
        )
        return [
            {
                "id": str(row["id"]),
                "raceId": row["race_id"],
                "type": row["event_type"],
                "carId": row["car_id"],
                "racerId": row["racer_id"],
                "checkpointId": row["checkpoint_id"],
                "lapNumber": row["lap_number"],
                "payload": json.loads(row["payload_json"]),
                "timestamp": row["timestamp"],
            }
            for row in rows
        ]

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
        self._db = self._client[db_name]
        self._laps = self._db[laps_collection]
        self._events = self._db["race_events"]
        self._laps.create_index(
            [("raceId", ASCENDING), ("carId", ASCENDING)], background=True
        )
        self._laps.create_index(
            [("raceId", ASCENDING), ("timestamp", ASCENDING)], background=True
        )
        self._events.create_index(
            [("raceId", ASCENDING), ("timestamp", ASCENDING)], background=True
        )

    def _collection(self, kind: str):
        return self._db[kind]

    def insert_lap(self, lap: LapIngest, speed_kph: float) -> str:
        payload = lap.dict()
        payload.update({"speedKph": speed_kph, "isValid": lap.validity.is_valid})
        result = self._laps.insert_one(payload)
        return str(result.inserted_id)

    def leaderboard(self, race_id: str) -> LeaderboardResponse:
        now = datetime.utcnow()
        pipeline: List[dict] = [
            {"$match": {"raceId": race_id, "isValid": True}},
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

    def create_entity(self, kind: str, record: dict) -> dict:
        now = datetime.utcnow().isoformat()
        payload = {
            **record,
            "createdAt": record.get("createdAt", now),
            "updatedAt": now,
        }
        self._collection(kind).insert_one(payload)
        return payload

    def list_entities(
        self, kind: str, filters: Optional[dict] = None, *, limit: Optional[int] = None
    ) -> list[dict]:
        cursor = (
            self._collection(kind).find(filters or {}, {"_id": 0}).sort("updatedAt", -1)
        )
        if limit is not None:
            cursor = cursor.limit(limit)
        return list(cursor)

    def get_entity(self, kind: str, entity_id: str) -> dict | None:
        return self._collection(kind).find_one({"id": entity_id}, {"_id": 0})

    def update_entity(self, kind: str, entity_id: str, record: dict) -> dict | None:
        existing = self.get_entity(kind, entity_id)
        if not existing:
            return None
        updated = {
            **existing,
            **record,
            "id": entity_id,
            "createdAt": existing.get("createdAt", datetime.utcnow().isoformat()),
            "updatedAt": datetime.utcnow().isoformat(),
        }
        self._collection(kind).replace_one({"id": entity_id}, updated, upsert=False)
        return updated

    def delete_entity(self, kind: str, entity_id: str) -> bool:
        result = self._collection(kind).delete_one({"id": entity_id})
        return result.deleted_count > 0

    def insert_event(self, race_id: str, event: RaceEventIngest) -> str:
        payload = event.dict()
        payload.update({"raceId": race_id})
        result = self._events.insert_one(payload)
        return str(result.inserted_id)

    def list_events(self, race_id: str, *, limit: int = 20) -> list[dict]:
        return list(
            self._events.find({"raceId": race_id}, {"_id": 0})
            .sort("timestamp", -1)
            .limit(limit)
        )

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
