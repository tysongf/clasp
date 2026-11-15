#include "clasp/ledger.hpp"

#include <sqlite3.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <sstream>
#include <utility>

namespace clasp::ledger {
namespace {

constexpr const char* kSchemaSql = R"SQL(
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
)SQL";

class Statement {
public:
    Statement(sqlite3* db, const char* sql) : db_(db) {
        if (sqlite3_prepare_v2(db_, sql, -1, &stmt_, nullptr) != SQLITE_OK) {
            throw LedgerError(std::string("SQLite prepare failed: ") + sqlite3_errmsg(db_));
        }
    }

    ~Statement() {
        if (stmt_ != nullptr) {
            sqlite3_finalize(stmt_);
        }
    }

    Statement(const Statement&) = delete;
    Statement& operator=(const Statement&) = delete;

    void bind_text(int idx, std::string_view value) {
        if (sqlite3_bind_text(stmt_, idx, value.data(), static_cast<int>(value.size()), SQLITE_TRANSIENT) != SQLITE_OK) {
            throw LedgerError(std::string("SQLite bind_text failed: ") + sqlite3_errmsg(db_));
        }
    }

    void bind_opt_text(int idx, const std::optional<std::string>& value) {
        if (value.has_value()) {
            bind_text(idx, *value);
        } else if (sqlite3_bind_null(stmt_, idx) != SQLITE_OK) {
            throw LedgerError(std::string("SQLite bind_null failed: ") + sqlite3_errmsg(db_));
        }
    }

    void bind_int64(int idx, std::int64_t value) {
        if (sqlite3_bind_int64(stmt_, idx, value) != SQLITE_OK) {
            throw LedgerError(std::string("SQLite bind_int64 failed: ") + sqlite3_errmsg(db_));
        }
    }

    void bind_opt_double(int idx, const std::optional<double>& value) {
        if (value.has_value()) {
            if (sqlite3_bind_double(stmt_, idx, *value) != SQLITE_OK) {
                throw LedgerError(std::string("SQLite bind_double failed: ") + sqlite3_errmsg(db_));
            }
        } else if (sqlite3_bind_null(stmt_, idx) != SQLITE_OK) {
            throw LedgerError(std::string("SQLite bind_null failed: ") + sqlite3_errmsg(db_));
        }
    }

    int step() {
        int rc = sqlite3_step(stmt_);
        if (rc != SQLITE_ROW && rc != SQLITE_DONE) {
            throw LedgerError(std::string("SQLite step failed: ") + sqlite3_errmsg(db_));
        }
        return rc;
    }

    std::string col_text(int col) const {
        const auto* raw = reinterpret_cast<const char*>(sqlite3_column_text(stmt_, col));
        return raw ? std::string(raw) : std::string{};
    }

    std::optional<std::string> col_opt_text(int col) const {
        if (sqlite3_column_type(stmt_, col) == SQLITE_NULL) {
            return std::nullopt;
        }
        return col_text(col);
    }

    std::int64_t col_int64(int col) const {
        return sqlite3_column_int64(stmt_, col);
    }

    std::optional<double> col_opt_double(int col) const {
        if (sqlite3_column_type(stmt_, col) == SQLITE_NULL) {
            return std::nullopt;
        }
        return sqlite3_column_double(stmt_, col);
    }

private:
    sqlite3* db_{nullptr};
    sqlite3_stmt* stmt_{nullptr};
};

class Transaction {
public:
    explicit Transaction(sqlite3* db) : db_(db) {
        exec("BEGIN IMMEDIATE;");
    }

    ~Transaction() {
        if (!committed_ && db_ != nullptr) {
            sqlite3_exec(db_, "ROLLBACK;", nullptr, nullptr, nullptr);
        }
    }

    Transaction(const Transaction&) = delete;
    Transaction& operator=(const Transaction&) = delete;

    void commit() {
        exec("COMMIT;");
        committed_ = true;
    }

private:
    sqlite3* db_{nullptr};
    bool committed_{false};

    void exec(const char* sql) {
        char* err = nullptr;
        if (sqlite3_exec(db_, sql, nullptr, nullptr, &err) != SQLITE_OK) {
            std::string msg = err ? err : sqlite3_errmsg(db_);
            sqlite3_free(err);
            throw LedgerError("SQLite transaction error: " + msg);
        }
    }
};

std::string upper_trimmed(std::string_view sv) {
    auto is_space = [](unsigned char c) { return std::isspace(c) != 0; };
    while (!sv.empty() && is_space(static_cast<unsigned char>(sv.front()))) {
        sv.remove_prefix(1);
    }
    while (!sv.empty() && is_space(static_cast<unsigned char>(sv.back()))) {
        sv.remove_suffix(1);
    }
    std::string out(sv);
    std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) {
        return static_cast<char>(std::toupper(c));
    });
    return out;
}

void validate_payload_contents(std::string_view contents) {
    const std::string norm = upper_trimmed(contents);
    if (norm.empty() || norm.size() > 128) {
        throw LedgerError("payload_contents must be 1..128 non-whitespace characters");
    }
    constexpr std::array<std::string_view, 4> kExcluded = {"LH2", "HYDRAZINE", "MMH", "NTO"};
    for (std::string_view excluded : kExcluded) {
        if (norm == excluded) {
            throw LedgerError("Excluded service fluid in CLASP-STD-001 Rev A (§1.2 / [CLASP-052]): " + norm);
        }
    }
}

void validate_optional_measures(
    const std::optional<double>& fill_fraction,
    const std::optional<double>& fluid_mass_kg,
    const std::optional<double>& chilldown_temp_k = std::nullopt,
    const std::optional<double>& trapped_loss_g = std::nullopt
) {
    if (fill_fraction.has_value()) {
        if (!std::isfinite(*fill_fraction) || *fill_fraction < 0.0 || *fill_fraction > 1.0) {
            throw LedgerError("fill_fraction must be within [0.0, 1.0]");
        }
    }
    if (fluid_mass_kg.has_value()) {
        if (!std::isfinite(*fluid_mass_kg) || *fluid_mass_kg < 0.0) {
            throw LedgerError("fluid_mass_kg must be >= 0.0");
        }
    }
    if (chilldown_temp_k.has_value()) {
        if (!std::isfinite(*chilldown_temp_k) || *chilldown_temp_k < 0.0) {
            throw LedgerError("chilldown_temp_k must be >= 0.0");
        }
    }
    if (trapped_loss_g.has_value()) {
        if (!std::isfinite(*trapped_loss_g) || *trapped_loss_g < 0.0) {
            throw LedgerError("trapped_loss_g must be >= 0.0");
        }
    }
}

PortRecord read_port_row(const Statement& st) {
    PortRecord p;
    p.unique_serial = st.col_text(0);
    p.port_tier = parse_port_tier(st.col_text(1));
    p.payload_contents = st.col_text(2);
    p.fill_state = parse_fill_state(st.col_text(3));
    p.fill_fraction = st.col_opt_double(4);
    p.fluid_mass_kg = st.col_opt_double(5);
    p.cycle_count = st.col_int64(6);
    p.certificate_number = st.col_opt_text(7);
    p.last_effector_id = st.col_opt_text(8);
    p.last_updated_utc = st.col_text(9);
    return p;
}

MatingEvent read_event_row(const Statement& st) {
    MatingEvent ev;
    ev.event_id = st.col_int64(0);
    ev.unique_serial = st.col_text(1);
    ev.active_effector_id = st.col_text(2);
    ev.event_type = parse_event_type(st.col_text(3));
    ev.payload_contents = st.col_text(4);
    ev.fill_state = parse_fill_state(st.col_text(5));
    ev.fill_fraction = st.col_opt_double(6);
    ev.fluid_mass_kg = st.col_opt_double(7);
    ev.delta_cycles = st.col_int64(8);
    ev.chilldown_temp_k = st.col_opt_double(9);
    ev.trapped_loss_g = st.col_opt_double(10);
    ev.occurred_at_utc = st.col_text(11);
    ev.notes = st.col_text(12);
    return ev;
}

std::string format_opt_double(const std::optional<double>& value) {
    if (!value.has_value()) {
        return "null";
    }
    std::ostringstream oss;
    oss << *value;
    return oss.str();
}

}  // namespace

std::string_view to_string(PortTier tier) noexcept {
    switch (tier) {
        case PortTier::ClaspM:
            return "CLASP-M";
        case PortTier::ClaspF:
            return "CLASP-F";
        case PortTier::ClaspFP:
            return "CLASP-FP";
    }
    return "CLASP-M";
}

std::string_view to_string(FillState state) noexcept {
    switch (state) {
        case FillState::Empty:
            return "EMPTY";
        case FillState::Chilling:
            return "CHILLING";
        case FillState::Partial:
            return "PARTIAL";
        case FillState::Full:
            return "FULL";
        case FillState::Venting:
            return "VENTING";
        case FillState::Unknown:
            return "UNKNOWN";
        case FillState::NA:
            return "NA";
    }
    return "UNKNOWN";
}

std::string_view to_string(EventType type) noexcept {
    switch (type) {
        case EventType::Register:
            return "REGISTER";
        case EventType::Mate:
            return "MATE";
        case EventType::Demate:
            return "DEMATE";
        case EventType::Transfer:
            return "TRANSFER";
        case EventType::Inspect:
            return "INSPECT";
        case EventType::Recertify:
            return "RECERTIFY";
    }
    return "MATE";
}

PortTier parse_port_tier(std::string_view value) {
    const std::string norm = upper_trimmed(value);
    if (norm == "CLASP-M") return PortTier::ClaspM;
    if (norm == "CLASP-F") return PortTier::ClaspF;
    if (norm == "CLASP-FP") return PortTier::ClaspFP;
    throw LedgerError("Invalid PortTier: " + std::string(value));
}

FillState parse_fill_state(std::string_view value) {
    const std::string norm = upper_trimmed(value);
    if (norm == "EMPTY") return FillState::Empty;
    if (norm == "CHILLING") return FillState::Chilling;
    if (norm == "PARTIAL") return FillState::Partial;
    if (norm == "FULL") return FillState::Full;
    if (norm == "VENTING") return FillState::Venting;
    if (norm == "UNKNOWN") return FillState::Unknown;
    if (norm == "NA") return FillState::NA;
    throw LedgerError("Invalid FillState: " + std::string(value));
}

EventType parse_event_type(std::string_view value) {
    const std::string norm = upper_trimmed(value);
    if (norm == "REGISTER") return EventType::Register;
    if (norm == "MATE") return EventType::Mate;
    if (norm == "DEMATE") return EventType::Demate;
    if (norm == "TRANSFER") return EventType::Transfer;
    if (norm == "INSPECT") return EventType::Inspect;
    if (norm == "RECERTIFY") return EventType::Recertify;
    throw LedgerError("Invalid EventType: " + std::string(value));
}

std::int64_t rated_cycle_limit(PortTier tier) noexcept {
    return tier == PortTier::ClaspM ? 500 : 1000;
}

bool is_valid_serial(std::string_view serial) noexcept {
    if (serial.size() < 2 || serial.size() > 64) {
        return false;
    }
    auto is_alnum_upper = [](unsigned char c) {
        return (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9');
    };
    if (!is_alnum_upper(static_cast<unsigned char>(serial.front()))) {
        return false;
    }
    for (unsigned char c : serial) {
        if (!(is_alnum_upper(c) || c == '.' || c == '_' || c == '-')) {
            return false;
        }
    }
    return true;
}

DataMatrixTag parse_datamatrix_tag(std::string_view payload) {
    const std::size_t sep = payload.find('|');
    if (sep == std::string_view::npos || payload.find('|', sep + 1) != std::string_view::npos) {
        throw LedgerError("Invalid [CLASP-141] DataMatrix payload (expected '<TIER>|<SERIAL>')");
    }
    const std::string_view tier_part = payload.substr(0, sep);
    const std::string_view serial_part = payload.substr(sep + 1);
    if (!is_valid_serial(serial_part)) {
        throw LedgerError("Invalid [CLASP-141] unique_serial in DataMatrix payload");
    }
    return DataMatrixTag{parse_port_tier(tier_part), std::string(serial_part)};
}

std::string format_datamatrix_tag(const DataMatrixTag& tag) {
    if (!is_valid_serial(tag.unique_serial)) {
        throw LedgerError("Invalid unique_serial for DataMatrix tag");
    }
    return std::string(to_string(tag.port_tier)) + "|" + tag.unique_serial;
}

std::string current_utc_iso8601() {
    std::time_t now = std::time(nullptr);
    std::tm utc{};
#if defined(_WIN32)
    gmtime_s(&utc, &now);
#else
    gmtime_r(&now, &utc);
#endif
    char buf[32]{};
    std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &utc);
    return std::string(buf);
}

std::string json_quote(std::string_view value) {
    std::string out;
    out.reserve(value.size() + 2);
    out.push_back('"');
    for (unsigned char c : value) {
        switch (c) {
            case '"':
                out += "\\\"";
                break;
            case '\\':
                out += "\\\\";
                break;
            case '\b':
                out += "\\b";
                break;
            case '\f':
                out += "\\f";
                break;
            case '\n':
                out += "\\n";
                break;
            case '\r':
                out += "\\r";
                break;
            case '\t':
                out += "\\t";
                break;
            default:
                if (c < 0x20) {
                    char hex[7]{};
                    std::snprintf(hex, sizeof(hex), "\\u%04x", static_cast<unsigned int>(c));
                    out += hex;
                } else {
                    out.push_back(static_cast<char>(c));
                }
                break;
        }
    }
    out.push_back('"');
    return out;
}

FleetLedger::FleetLedger(const std::filesystem::path& db_path) {
    if (db_path != ":memory:" && db_path.has_parent_path()) {
        std::filesystem::create_directories(db_path.parent_path());
    }
    if (sqlite3_open(db_path.c_str(), &db_) != SQLITE_OK) {
        std::string msg = db_ ? sqlite3_errmsg(db_) : "unknown error";
        if (db_ != nullptr) {
            sqlite3_close(db_);
            db_ = nullptr;
        }
        throw LedgerError("Failed to open SQLite ledger: " + msg);
    }
    sqlite3_busy_timeout(db_, 5000);
    exec_sql("PRAGMA foreign_keys = ON;");
}

FleetLedger::~FleetLedger() {
    if (db_ != nullptr) {
        sqlite3_close(db_);
        db_ = nullptr;
    }
}

FleetLedger::FleetLedger(FleetLedger&& other) noexcept : db_(other.db_) {
    other.db_ = nullptr;
}

FleetLedger& FleetLedger::operator=(FleetLedger&& other) noexcept {
    if (this != &other) {
        if (db_ != nullptr) {
            sqlite3_close(db_);
        }
        db_ = other.db_;
        other.db_ = nullptr;
    }
    return *this;
}

void FleetLedger::exec_sql(const char* sql) const {
    char* err = nullptr;
    if (sqlite3_exec(db_, sql, nullptr, nullptr, &err) != SQLITE_OK) {
        std::string msg = err ? err : sqlite3_errmsg(db_);
        sqlite3_free(err);
        throw LedgerError("SQLite execution failed: " + msg);
    }
}

void FleetLedger::init_schema() {
    exec_sql(kSchemaSql);
}

PortRecord FleetLedger::register_port(const PortRecord& port, std::string_view active_effector_id) {
    if (!is_valid_serial(port.unique_serial)) {
        throw LedgerError("Invalid unique_serial: " + port.unique_serial);
    }
    if (port.cycle_count < 0) {
        throw LedgerError("cycle_count must be >= 0");
    }
    if (active_effector_id.empty()) {
        throw LedgerError("active_effector_id must not be empty");
    }
    validate_payload_contents(port.payload_contents);
    validate_optional_measures(port.fill_fraction, port.fluid_mass_kg);

    const std::string timestamp = port.last_updated_utc.empty() ? current_utc_iso8601() : port.last_updated_utc;
    const std::optional<std::string> effector =
        port.last_effector_id.has_value() ? port.last_effector_id : std::optional<std::string>(std::string(active_effector_id));

    Transaction tx(db_);
    {
        Statement ins(db_, R"SQL(
            INSERT INTO ports (
                unique_serial, port_tier, payload_contents, fill_state,
                fill_fraction, fluid_mass_kg, cycle_count, certificate_number,
                last_effector_id, last_updated_utc
            ) VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8, ?9, ?10);
        )SQL");
        ins.bind_text(1, port.unique_serial);
        ins.bind_text(2, to_string(port.port_tier));
        ins.bind_text(3, port.payload_contents);
        ins.bind_text(4, to_string(port.fill_state));
        ins.bind_opt_double(5, port.fill_fraction);
        ins.bind_opt_double(6, port.fluid_mass_kg);
        ins.bind_int64(7, port.cycle_count);
        ins.bind_opt_text(8, port.certificate_number);
        ins.bind_opt_text(9, effector);
        ins.bind_text(10, timestamp);
        ins.step();
    }
    {
        Statement ev(db_, R"SQL(
            INSERT INTO mating_events (
                unique_serial, active_effector_id, event_type, payload_contents,
                fill_state, fill_fraction, fluid_mass_kg, delta_cycles,
                chilldown_temp_k, trapped_loss_g, occurred_at_utc, notes
            ) VALUES (?1, ?2, 'REGISTER', ?3, ?4, ?5, ?6, ?7, NULL, NULL, ?8, 'Initial port registration');
        )SQL");
        ev.bind_text(1, port.unique_serial);
        ev.bind_text(2, active_effector_id);
        ev.bind_text(3, port.payload_contents);
        ev.bind_text(4, to_string(port.fill_state));
        ev.bind_opt_double(5, port.fill_fraction);
        ev.bind_opt_double(6, port.fluid_mass_kg);
        ev.bind_int64(7, port.cycle_count);
        ev.bind_text(8, timestamp);
        ev.step();
    }
    tx.commit();

    return *get_port(port.unique_serial);
}

PortRecord FleetLedger::record_event(const MatingEventInput& event) {
    if (!is_valid_serial(event.unique_serial)) {
        throw LedgerError("Invalid unique_serial: " + event.unique_serial);
    }
    if (event.active_effector_id.empty()) {
        throw LedgerError("active_effector_id must not be empty");
    }
    if (event.delta_cycles < 0) {
        throw LedgerError("delta_cycles must be >= 0");
    }
    validate_payload_contents(event.payload_contents);
    validate_optional_measures(
        event.fill_fraction,
        event.fluid_mass_kg,
        event.chilldown_temp_k,
        event.trapped_loss_g
    );

    const std::string timestamp = event.occurred_at_utc.empty() ? current_utc_iso8601() : event.occurred_at_utc;

    Transaction tx(db_);
    {
        Statement chk(db_, "SELECT 1 FROM ports WHERE unique_serial = ?1;");
        chk.bind_text(1, event.unique_serial);
        if (chk.step() != SQLITE_ROW) {
            throw LedgerError("Unknown [CLASP-141] unique_serial: " + event.unique_serial);
        }
    }
    {
        Statement ins(db_, R"SQL(
            INSERT INTO mating_events (
                unique_serial, active_effector_id, event_type, payload_contents,
                fill_state, fill_fraction, fluid_mass_kg, delta_cycles,
                chilldown_temp_k, trapped_loss_g, occurred_at_utc, notes
            ) VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8, ?9, ?10, ?11, ?12);
        )SQL");
        ins.bind_text(1, event.unique_serial);
        ins.bind_text(2, event.active_effector_id);
        ins.bind_text(3, to_string(event.event_type));
        ins.bind_text(4, event.payload_contents);
        ins.bind_text(5, to_string(event.fill_state));
        ins.bind_opt_double(6, event.fill_fraction);
        ins.bind_opt_double(7, event.fluid_mass_kg);
        ins.bind_int64(8, event.delta_cycles);
        ins.bind_opt_double(9, event.chilldown_temp_k);
        ins.bind_opt_double(10, event.trapped_loss_g);
        ins.bind_text(11, timestamp);
        ins.bind_text(12, event.notes);
        ins.step();
    }
    {
        Statement upd(db_, R"SQL(
            UPDATE ports
            SET payload_contents = ?1,
                fill_state       = ?2,
                fill_fraction    = ?3,
                fluid_mass_kg    = ?4,
                cycle_count      = cycle_count + ?5,
                last_effector_id = ?6,
                last_updated_utc = ?7
            WHERE unique_serial  = ?8;
        )SQL");
        upd.bind_text(1, event.payload_contents);
        upd.bind_text(2, to_string(event.fill_state));
        upd.bind_opt_double(3, event.fill_fraction);
        upd.bind_opt_double(4, event.fluid_mass_kg);
        upd.bind_int64(5, event.delta_cycles);
        upd.bind_text(6, event.active_effector_id);
        upd.bind_text(7, timestamp);
        upd.bind_text(8, event.unique_serial);
        upd.step();
    }
    tx.commit();

    return *get_port(event.unique_serial);
}

std::optional<PortRecord> FleetLedger::get_port(std::string_view unique_serial) const {
    Statement st(db_, R"SQL(
        SELECT unique_serial, port_tier, payload_contents, fill_state,
               fill_fraction, fluid_mass_kg, cycle_count, certificate_number,
               last_effector_id, last_updated_utc
        FROM ports
        WHERE unique_serial = ?1;
    )SQL");
    st.bind_text(1, unique_serial);
    if (st.step() != SQLITE_ROW) {
        return std::nullopt;
    }
    return read_port_row(st);
}

std::vector<PortRecord> FleetLedger::list_ports() const {
    Statement st(db_, R"SQL(
        SELECT unique_serial, port_tier, payload_contents, fill_state,
               fill_fraction, fluid_mass_kg, cycle_count, certificate_number,
               last_effector_id, last_updated_utc
        FROM ports
        ORDER BY unique_serial ASC;
    )SQL");
    std::vector<PortRecord> result;
    while (st.step() == SQLITE_ROW) {
        result.push_back(read_port_row(st));
    }
    return result;
}

std::vector<MatingEvent> FleetLedger::list_events(std::string_view unique_serial, std::size_t limit) const {
    Statement st(db_, R"SQL(
        SELECT event_id, unique_serial, active_effector_id, event_type,
               payload_contents, fill_state, fill_fraction, fluid_mass_kg,
               delta_cycles, chilldown_temp_k, trapped_loss_g, occurred_at_utc, notes
        FROM mating_events
        WHERE unique_serial = ?1
        ORDER BY occurred_at_utc DESC, event_id DESC
        LIMIT ?2;
    )SQL");
    st.bind_text(1, unique_serial);
    st.bind_int64(2, static_cast<std::int64_t>(limit));
    std::vector<MatingEvent> result;
    while (st.step() == SQLITE_ROW) {
        result.push_back(read_event_row(st));
    }
    return result;
}

std::string FleetLedger::export_json(bool include_events, std::size_t events_per_port) const {
    const std::vector<PortRecord> ports = list_ports();
    std::ostringstream oss;
    oss << "{\n";
    oss << "  \"schema_version\": \"1.0.0\",\n";
    oss << "  \"generated_utc\": " << json_quote(current_utc_iso8601()) << ",\n";
    oss << "  \"ports\": [";
    if (!ports.empty()) {
        oss << "\n";
    }
    for (std::size_t i = 0; i < ports.size(); ++i) {
        const auto& p = ports[i];
        oss << "    {\n";
        oss << "      \"unique_serial\": " << json_quote(p.unique_serial) << ",\n";
        oss << "      \"port_tier\": " << json_quote(to_string(p.port_tier)) << ",\n";
        oss << "      \"payload_contents\": " << json_quote(p.payload_contents) << ",\n";
        oss << "      \"fill_state\": " << json_quote(to_string(p.fill_state)) << ",\n";
        oss << "      \"fill_fraction\": " << format_opt_double(p.fill_fraction) << ",\n";
        oss << "      \"fluid_mass_kg\": " << format_opt_double(p.fluid_mass_kg) << ",\n";
        oss << "      \"cycle_count\": " << p.cycle_count << ",\n";
        oss << "      \"rated_cycle_limit\": " << rated_cycle_limit(p.port_tier) << ",\n";
        oss << "      \"certificate_number\": "
            << (p.certificate_number.has_value() ? json_quote(*p.certificate_number) : "null") << ",\n";
        oss << "      \"last_effector_id\": "
            << (p.last_effector_id.has_value() ? json_quote(*p.last_effector_id) : "null") << ",\n";
        oss << "      \"last_updated_utc\": " << json_quote(p.last_updated_utc);

        if (include_events) {
            const auto events = list_events(p.unique_serial, events_per_port);
            oss << ",\n      \"recent_events\": [";
            if (!events.empty()) {
                oss << "\n";
            }
            for (std::size_t j = 0; j < events.size(); ++j) {
                const auto& ev = events[j];
                oss << "        {\n";
                oss << "          \"event_id\": " << ev.event_id << ",\n";
                oss << "          \"unique_serial\": " << json_quote(ev.unique_serial) << ",\n";
                oss << "          \"active_effector_id\": " << json_quote(ev.active_effector_id) << ",\n";
                oss << "          \"event_type\": " << json_quote(to_string(ev.event_type)) << ",\n";
                oss << "          \"payload_contents\": " << json_quote(ev.payload_contents) << ",\n";
                oss << "          \"fill_state\": " << json_quote(to_string(ev.fill_state)) << ",\n";
                oss << "          \"fill_fraction\": " << format_opt_double(ev.fill_fraction) << ",\n";
                oss << "          \"fluid_mass_kg\": " << format_opt_double(ev.fluid_mass_kg) << ",\n";
                oss << "          \"delta_cycles\": " << ev.delta_cycles << ",\n";
                oss << "          \"chilldown_temp_k\": " << format_opt_double(ev.chilldown_temp_k) << ",\n";
                oss << "          \"trapped_loss_g\": " << format_opt_double(ev.trapped_loss_g) << ",\n";
                oss << "          \"occurred_at_utc\": " << json_quote(ev.occurred_at_utc) << ",\n";
                oss << "          \"notes\": " << json_quote(ev.notes) << "\n";
                oss << "        }" << (j + 1 < events.size() ? "," : "") << "\n";
            }
            if (!events.empty()) {
                oss << "      ";
            }
            oss << "]\n";
        } else {
            oss << "\n";
        }
        oss << "    }" << (i + 1 < ports.size() ? "," : "") << "\n";
    }
    if (!ports.empty()) {
        oss << "  ";
    }
    oss << "]\n";
    oss << "}\n";
    return oss.str();
}

}  // namespace clasp::ledger

