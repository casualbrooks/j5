"""Mongo repository helpers for laps and standings."""

from __future__ import annotations

from datetime import datetime
from typing import Iterable, List

from pymongo import ASCENDING, MongoClient
from pymongo.collection import Collection

from .schemas import LapIngest, LeaderboardEntry, LeaderboardResponse


class RaceRepository:
    def __init__(
        self, client: MongoClient, db_name: str, laps_collection: str = "laps_live"
    ) -> None:
        self.client = client
        self.db = client[db_name]
        self.laps: Collection = self.db[laps_collection]
        self._ensure_indexes()

    def _ensure_indexes(self) -> None:
        # Keep indexes lightweight so they work on capped collections too.
        self.laps.create_index(
            [("raceId", ASCENDING), ("carId", ASCENDING)], background=True
        )
        self.laps.create_index(
            [("raceId", ASCENDING), ("timestamp", ASCENDING)], background=True
        )

    def insert_lap(self, lap: LapIngest, speed_kph: float) -> str:
        payload = lap.dict()
        payload.update(
            {
                "speedKph": speed_kph,
                "isValid": lap.validity.is_valid,
            }
        )
        result = self.laps.insert_one(payload)
        return str(result.inserted_id)

    def leaderboard(
        self,
        race_id: str,
        *,
        include_replay: bool = False,
        include_invalid: bool = False,
    ) -> LeaderboardResponse:
        now = datetime.utcnow()
        match: dict[str, object] = {"raceId": race_id}
        if not include_replay:
            match["source"] = "live"
        if not include_invalid:
            match["isValid"] = True

        pipeline: List[dict] = [
            {"$match": match},
            {"$sort": {"carId": 1, "timestamp": 1}},
            {
                "$group": {
                    "_id": "$carId",
                    "lapCount": {"$sum": 1},
                    "bestLapMs": {"$min": "$lapTimeMs"},
                    "avgLapMs": {"$avg": "$lapTimeMs"},
                    "avgSpeedKph": {"$avg": "$speedKph"},
                    "lastLapMs": {"$last": "$lapTimeMs"},
                    "lastSpeedKph": {"$last": "$speedKph"},
                    "lastTimestamp": {"$last": "$timestamp"},
                    "totalTimeMs": {"$sum": "$lapTimeMs"},
                }
            },
            {"$sort": {"totalTimeMs": 1}},
        ]
        raw = list(self.laps.aggregate(pipeline))
        leaderboard = self._attach_gaps(raw)
        entries = [LeaderboardEntry(**entry) for entry in leaderboard]
        return LeaderboardResponse(raceId=race_id, leaderboard=entries, asOf=now)

    @staticmethod
    def _attach_gaps(rows: Iterable[dict]) -> list[dict]:
        rows = list(rows)
        if not rows:
            return []
        leader_time = rows[0]["totalTimeMs"]
        for row in rows:
            row["carId"] = row.pop("_id")
            row["gapToLeaderMs"] = max(0, row["totalTimeMs"] - leader_time)
        return rows


def get_repository(uri: str, db_name: str, laps_collection: str) -> RaceRepository:
    client = MongoClient(uri)
    return RaceRepository(client, db_name, laps_collection)
