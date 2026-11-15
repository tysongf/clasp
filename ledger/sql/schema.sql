-- CLASP-STD-001 Fleet Ledger SQLite Schema
-- Keyed by [CLASP-141] immutable DataMatrix unique_serial.

PRAGMA foreign_keys = ON;

CREATE TABLE IF NOT EXISTS ports (
    unique_serial      TEXT PRIMARY KEY NOT NULL
        CHECK (length(unique_serial) BETWEEN 2 AND 64),
    port_tier          TEXT NOT NULL
        CHECK (port_tier IN ('CLASP-M', 'CLASP-F', 'CLASP-FP')),
    payload_contents   TEXT NOT NULL
        CHECK (
            length(trim(payload_contents)) > 0
            AND upper(trim(payload_contents)) NOT IN ('LH2', 'HYDRAZINE', 'MMH', 'NTO')
        ),
    fill_state         TEXT NOT NULL
        CHECK (fill_state IN ('EMPTY', 'CHILLING', 'PARTIAL', 'FULL', 'VENTING', 'UNKNOWN', 'NA')),
    fill_fraction      REAL
        CHECK (fill_fraction IS NULL OR (fill_fraction >= 0.0 AND fill_fraction <= 1.0)),
    fluid_mass_kg      REAL
        CHECK (fluid_mass_kg IS NULL OR fluid_mass_kg >= 0.0),
    cycle_count        INTEGER NOT NULL DEFAULT 0
        CHECK (cycle_count >= 0),
    certificate_number TEXT,
    last_effector_id   TEXT,
    last_updated_utc   TEXT NOT NULL
);

CREATE TABLE IF NOT EXISTS mating_events (
    event_id           INTEGER PRIMARY KEY AUTOINCREMENT,
    unique_serial      TEXT NOT NULL
        REFERENCES ports(unique_serial) ON DELETE RESTRICT ON UPDATE RESTRICT,
    active_effector_id TEXT NOT NULL
        CHECK (length(trim(active_effector_id)) > 0),
    event_type         TEXT NOT NULL
        CHECK (event_type IN ('REGISTER', 'MATE', 'DEMATE', 'TRANSFER', 'INSPECT', 'RECERTIFY')),
    payload_contents   TEXT NOT NULL
        CHECK (
            length(trim(payload_contents)) > 0
            AND upper(trim(payload_contents)) NOT IN ('LH2', 'HYDRAZINE', 'MMH', 'NTO')
        ),
    fill_state         TEXT NOT NULL
        CHECK (fill_state IN ('EMPTY', 'CHILLING', 'PARTIAL', 'FULL', 'VENTING', 'UNKNOWN', 'NA')),
    fill_fraction      REAL
        CHECK (fill_fraction IS NULL OR (fill_fraction >= 0.0 AND fill_fraction <= 1.0)),
    fluid_mass_kg      REAL
        CHECK (fluid_mass_kg IS NULL OR fluid_mass_kg >= 0.0),
    delta_cycles       INTEGER NOT NULL DEFAULT 0
        CHECK (delta_cycles >= 0),
    chilldown_temp_k   REAL
        CHECK (chilldown_temp_k IS NULL OR chilldown_temp_k >= 0.0),
    trapped_loss_g     REAL
        CHECK (trapped_loss_g IS NULL OR trapped_loss_g >= 0.0),
    occurred_at_utc    TEXT NOT NULL,
    notes              TEXT NOT NULL DEFAULT ''
);

CREATE INDEX IF NOT EXISTS idx_mating_events_serial_time
    ON mating_events(unique_serial, occurred_at_utc DESC, event_id DESC);

CREATE VIEW IF NOT EXISTS port_cycle_status AS
SELECT
    unique_serial,
    port_tier,
    payload_contents,
    fill_state,
    fill_fraction,
    fluid_mass_kg,
    cycle_count,
    CASE port_tier
        WHEN 'CLASP-M' THEN 500
        ELSE 1000
    END AS rated_cycle_limit,
    (CASE port_tier
        WHEN 'CLASP-M' THEN 500
        ELSE 1000
    END) - cycle_count AS cycles_remaining,
    certificate_number,
    last_effector_id,
    last_updated_utc
FROM ports;

