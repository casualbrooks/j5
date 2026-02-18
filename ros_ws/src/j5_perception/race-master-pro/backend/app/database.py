"""
Async SQLite database layer using aiosqlite.
Handles connection lifecycle, schema init, and generic CRUD helpers.
"""

import aiosqlite
import os
from pathlib import Path
from typing import Optional

from app.schemas import SCHEMA_SQL

# Default DB path: <project_root>/data/racetracker.db
_DEFAULT_DB_PATH = str(Path(__file__).parent.parent / "data" / "racetracker.db")
DB_PATH = os.getenv("DATABASE_PATH", _DEFAULT_DB_PATH)


async def get_db() -> aiosqlite.Connection:
    """Get a database connection. Caller must close or use as context manager."""
    db = await aiosqlite.connect(DB_PATH)
    db.row_factory = aiosqlite.Row
    await db.execute("PRAGMA journal_mode=WAL")
    await db.execute("PRAGMA foreign_keys=ON")
    return db


async def init_db():
    """Initialize database schema. Safe to call multiple times (IF NOT EXISTS)."""
    os.makedirs(os.path.dirname(DB_PATH), exist_ok=True)
    async with aiosqlite.connect(DB_PATH) as db:
        await db.executescript(SCHEMA_SQL)
        await db.commit()
    print(f"Database initialized at {DB_PATH}")


# ── Generic CRUD Helpers ────────────────────────────────────


async def insert_row(table: str, data: dict) -> dict:
    """Insert a row into a table. Returns the inserted data."""
    columns = ", ".join(data.keys())
    placeholders = ", ".join(["?"] * len(data))
    sql = f"INSERT INTO {table} ({columns}) VALUES ({placeholders})"
    async with aiosqlite.connect(DB_PATH) as db:
        db.row_factory = aiosqlite.Row
        await db.execute("PRAGMA foreign_keys=ON")
        await db.execute(sql, list(data.values()))
        await db.commit()
    return data


async def get_row(table: str, id_value: str, id_column: str = "id") -> Optional[dict]:
    """Get a single row by ID."""
    sql = f"SELECT * FROM {table} WHERE {id_column} = ?"
    async with aiosqlite.connect(DB_PATH) as db:
        db.row_factory = aiosqlite.Row
        cursor = await db.execute(sql, [id_value])
        row = await cursor.fetchone()
        return dict(row) if row else None


async def get_all_rows(
    table: str, where: Optional[dict] = None, order_by: Optional[str] = None
) -> list[dict]:
    """Get all rows from a table, optionally filtered and ordered."""
    sql = f"SELECT * FROM {table}"
    params: list = []
    if where:
        conditions = " AND ".join([f"{k} = ?" for k in where.keys()])
        sql += f" WHERE {conditions}"
        params = list(where.values())
    if order_by:
        sql += f" ORDER BY {order_by}"
    async with aiosqlite.connect(DB_PATH) as db:
        db.row_factory = aiosqlite.Row
        cursor = await db.execute(sql, params)
        rows = await cursor.fetchall()
        return [dict(row) for row in rows]


async def update_row(
    table: str, id_value: str, data: dict, id_column: str = "id"
) -> Optional[dict]:
    """Update a row by ID. Returns updated data or None."""
    set_clause = ", ".join([f"{k} = ?" for k in data.keys()])
    sql = f"UPDATE {table} SET {set_clause} WHERE {id_column} = ?"
    params = list(data.values()) + [id_value]
    async with aiosqlite.connect(DB_PATH) as db:
        db.row_factory = aiosqlite.Row
        await db.execute("PRAGMA foreign_keys=ON")
        await db.execute(sql, params)
        await db.commit()
        cursor = await db.execute(
            f"SELECT * FROM {table} WHERE {id_column} = ?", [id_value]
        )
        row = await cursor.fetchone()
        return dict(row) if row else None


async def delete_row(table: str, id_value: str, id_column: str = "id") -> bool:
    """Delete a row by ID. Returns True if deleted."""
    sql = f"DELETE FROM {table} WHERE {id_column} = ?"
    async with aiosqlite.connect(DB_PATH) as db:
        await db.execute("PRAGMA foreign_keys=ON")
        cursor = await db.execute(sql, [id_value])
        await db.commit()
        return cursor.rowcount > 0


async def query(sql: str, params: Optional[list] = None) -> list[dict]:
    """Execute a raw SQL query and return results."""
    async with aiosqlite.connect(DB_PATH) as db:
        db.row_factory = aiosqlite.Row
        cursor = await db.execute(sql, params or [])
        rows = await cursor.fetchall()
        return [dict(row) for row in rows]
