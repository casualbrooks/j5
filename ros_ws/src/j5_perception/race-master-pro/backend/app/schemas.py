"""
SQLite schema definitions for Race Master Pro.
All tables, indexes, and seed data in one place.
"""

SCHEMA_SQL = """
-- ═══════════════════════════════════════════════════════════
-- Race Master Pro — SQLite Schema
-- ═══════════════════════════════════════════════════════════

-- Seasons group championships across a year
CREATE TABLE IF NOT EXISTS seasons (
    id TEXT PRIMARY KEY,
    name TEXT NOT NULL,
    year INTEGER NOT NULL,
    status TEXT NOT NULL DEFAULT 'active'
        CHECK(status IN ('active', 'completed', 'upcoming')),
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Championships belong to a season, contain race events
CREATE TABLE IF NOT EXISTS championships (
    id TEXT PRIMARY KEY,
    season_id TEXT NOT NULL REFERENCES seasons(id) ON DELETE CASCADE,
    name TEXT NOT NULL,
    points_system TEXT NOT NULL DEFAULT '{"1":25,"2":18,"3":15,"4":12,"5":10,"6":8,"7":6,"8":4,"9":2,"10":1}',
    status TEXT NOT NULL DEFAULT 'active'
        CHECK(status IN ('active', 'completed', 'upcoming')),
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Tracks store layout and configuration
CREATE TABLE IF NOT EXISTS tracks (
    id TEXT PRIMARY KEY,
    name TEXT NOT NULL,
    scale TEXT NOT NULL DEFAULT '1:24',
    track_distance REAL,
    layout_points TEXT DEFAULT '[]',
    boundary_polygon TEXT DEFAULT '[]',
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Checkpoints on a track (start, finish, sector markers)
CREATE TABLE IF NOT EXISTS checkpoints (
    id TEXT PRIMARY KEY,
    track_id TEXT NOT NULL REFERENCES tracks(id) ON DELETE CASCADE,
    name TEXT NOT NULL,
    type TEXT NOT NULL CHECK(type IN ('start', 'finish', 'checkpoint', 'sector')),
    position TEXT NOT NULL DEFAULT '{"x":0,"y":0}',
    sort_order INTEGER DEFAULT 0
);

-- Camera configurations per track
CREATE TABLE IF NOT EXISTS camera_configs (
    id TEXT PRIMARY KEY,
    track_id TEXT REFERENCES tracks(id) ON DELETE SET NULL,
    name TEXT NOT NULL,
    source TEXT NOT NULL DEFAULT '0',
    ros_topic TEXT,
    calibration TEXT,
    status TEXT NOT NULL DEFAULT 'disconnected'
        CHECK(status IN ('connected', 'disconnected', 'calibrating', 'error'))
);

-- Racer profiles (persistent across races)
CREATE TABLE IF NOT EXISTS racer_profiles (
    id TEXT PRIMARY KEY,
    name TEXT NOT NULL,
    number TEXT DEFAULT '',
    profile_pic TEXT,
    vehicle_description TEXT,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Race events belong to a championship, held at a track
CREATE TABLE IF NOT EXISTS race_events (
    id TEXT PRIMARY KEY,
    championship_id TEXT REFERENCES championships(id) ON DELETE SET NULL,
    track_id TEXT REFERENCES tracks(id) ON DELETE SET NULL,
    name TEXT NOT NULL,
    event_date TEXT,
    round_number INTEGER DEFAULT 1
);

-- Individual races within an event
CREATE TABLE IF NOT EXISTS races (
    id TEXT PRIMARY KEY,
    event_id TEXT REFERENCES race_events(id) ON DELETE SET NULL,
    type TEXT NOT NULL DEFAULT 'main'
        CHECK(type IN ('practice', 'qualifying', 'main')),
    status TEXT NOT NULL DEFAULT 'setup'
        CHECK(status IN ('setup', 'active', 'paused', 'finished', 'cancelled')),
    total_laps INTEGER DEFAULT 10,
    start_time TIMESTAMP,
    end_time TIMESTAMP
);

-- Results per racer per race
CREATE TABLE IF NOT EXISTS race_results (
    id TEXT PRIMARY KEY,
    race_id TEXT NOT NULL REFERENCES races(id) ON DELETE CASCADE,
    racer_profile_id TEXT NOT NULL REFERENCES racer_profiles(id) ON DELETE CASCADE,
    finish_position INTEGER DEFAULT 0,
    total_time REAL DEFAULT 0,
    best_lap_time REAL,
    avg_lap_time REAL,
    laps_completed INTEGER DEFAULT 0,
    status TEXT NOT NULL DEFAULT 'racing'
        CHECK(status IN ('racing', 'finished', 'dnf')),
    points_earned INTEGER DEFAULT 0
);

-- Individual lap records
CREATE TABLE IF NOT EXISTS lap_records (
    id TEXT PRIMARY KEY,
    race_id TEXT NOT NULL REFERENCES races(id) ON DELETE CASCADE,
    racer_profile_id TEXT NOT NULL REFERENCES racer_profiles(id) ON DELETE CASCADE,
    lap_number INTEGER NOT NULL,
    lap_time REAL NOT NULL,
    sector1_time REAL,
    sector2_time REAL,
    sector3_time REAL,
    recorded_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Computer vision detections
CREATE TABLE IF NOT EXISTS detections (
    id TEXT PRIMARY KEY,
    race_id TEXT REFERENCES races(id) ON DELETE CASCADE,
    camera_id TEXT REFERENCES camera_configs(id) ON DELETE SET NULL,
    racer_profile_id TEXT REFERENCES racer_profiles(id) ON DELETE SET NULL,
    position_x REAL,
    position_y REAL,
    confidence REAL,
    recorded_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Behavioral metrics (aggregated per racer per season)
CREATE TABLE IF NOT EXISTS behavioral_metrics (
    racer_profile_id TEXT NOT NULL REFERENCES racer_profiles(id) ON DELETE CASCADE,
    season_id TEXT NOT NULL REFERENCES seasons(id) ON DELETE CASCADE,
    consistency_score REAL DEFAULT 0,
    avg_lap_variance REAL DEFAULT 0,
    improvement_rate REAL DEFAULT 0,
    total_races INTEGER DEFAULT 0,
    dnf_count INTEGER DEFAULT 0,
    avg_finish_position REAL DEFAULT 0,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    PRIMARY KEY (racer_profile_id, season_id)
);

-- ═══════════════════════════════════════════════════════════
-- Performance Indexes
-- ═══════════════════════════════════════════════════════════

CREATE INDEX IF NOT EXISTS idx_championships_season ON championships(season_id);
CREATE INDEX IF NOT EXISTS idx_checkpoints_track ON checkpoints(track_id);
CREATE INDEX IF NOT EXISTS idx_camera_configs_track ON camera_configs(track_id);
CREATE INDEX IF NOT EXISTS idx_race_events_championship ON race_events(championship_id);
CREATE INDEX IF NOT EXISTS idx_race_events_track ON race_events(track_id);
CREATE INDEX IF NOT EXISTS idx_races_event ON races(event_id);
CREATE INDEX IF NOT EXISTS idx_races_status ON races(status);
CREATE INDEX IF NOT EXISTS idx_race_results_race ON race_results(race_id);
CREATE INDEX IF NOT EXISTS idx_race_results_racer ON race_results(racer_profile_id);
CREATE INDEX IF NOT EXISTS idx_lap_records_race ON lap_records(race_id);
CREATE INDEX IF NOT EXISTS idx_lap_records_racer ON lap_records(racer_profile_id);
CREATE INDEX IF NOT EXISTS idx_lap_records_race_racer ON lap_records(race_id, racer_profile_id);
CREATE INDEX IF NOT EXISTS idx_detections_race ON detections(race_id);
CREATE INDEX IF NOT EXISTS idx_detections_timestamp ON detections(recorded_at);

-- Generic key/value system state store for setup wizard + runtime snapshots
CREATE TABLE IF NOT EXISTS system_state (
    key TEXT PRIMARY KEY,
    value TEXT NOT NULL,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
"""
