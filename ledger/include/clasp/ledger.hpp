#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

struct sqlite3;

namespace clasp::ledger {

enum class PortTier {
    ClaspM,
    ClaspF,
    ClaspFP,
};

enum class FillState {
    Empty,
    Chilling,
    Partial,
    Full,
    Venting,
    Unknown,
    NA,
};

enum class EventType {
    Register,
    Mate,
    Demate,
    Transfer,
    Inspect,
    Recertify,
};

struct DataMatrixTag {
    PortTier port_tier;
    std::string unique_serial;
};

struct PortRecord {
    std::string unique_serial;
    PortTier port_tier{PortTier::ClaspM};
    std::string payload_contents{"NONE"};
    FillState fill_state{FillState::NA};
    std::optional<double> fill_fraction;
    std::optional<double> fluid_mass_kg;
    std::int64_t cycle_count{0};
    std::optional<std::string> certificate_number;
    std::optional<std::string> last_effector_id;
    std::string last_updated_utc;
};

struct MatingEventInput {
    std::string unique_serial;
    std::string active_effector_id;
    EventType event_type{EventType::Mate};
    std::string payload_contents;
    FillState fill_state{FillState::Unknown};
    std::optional<double> fill_fraction;
    std::optional<double> fluid_mass_kg;
    std::int64_t delta_cycles{0};
    std::optional<double> chilldown_temp_k;
    std::optional<double> trapped_loss_g;
    std::string occurred_at_utc;
    std::string notes;
};

struct MatingEvent {
    std::int64_t event_id{0};
    std::string unique_serial;
    std::string active_effector_id;
    EventType event_type{EventType::Mate};
    std::string payload_contents;
    FillState fill_state{FillState::Unknown};
    std::optional<double> fill_fraction;
    std::optional<double> fluid_mass_kg;
    std::int64_t delta_cycles{0};
    std::optional<double> chilldown_temp_k;
    std::optional<double> trapped_loss_g;
    std::string occurred_at_utc;
    std::string notes;
};

class LedgerError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

std::string_view to_string(PortTier tier) noexcept;
std::string_view to_string(FillState state) noexcept;
std::string_view to_string(EventType type) noexcept;

PortTier parse_port_tier(std::string_view value);
FillState parse_fill_state(std::string_view value);
EventType parse_event_type(std::string_view value);

/// Rated mating cycle limit per [CLASP-150]: 500 for CLASP-M, 1000 for CLASP-F/-FP.
std::int64_t rated_cycle_limit(PortTier tier) noexcept;

/// Parses a [CLASP-141] static optical DataMatrix payload of form "<TIER>|<SERIAL>".
DataMatrixTag parse_datamatrix_tag(std::string_view payload);

/// Formats a [CLASP-141] static optical DataMatrix payload as "<TIER>|<SERIAL>".
std::string format_datamatrix_tag(const DataMatrixTag& tag);

/// Validates [CLASP-141] unique serial format (2..64 chars, leading alphanumeric, [A-Z0-9._-]).
bool is_valid_serial(std::string_view serial) noexcept;

/// Returns current UTC timestamp in ISO-8601 format (YYYY-MM-DDTHH:MM:SSZ).
std::string current_utc_iso8601();

/// Escapes and quotes a string per RFC 8259.
std::string json_quote(std::string_view value);

/// SQLite-backed fleet ledger keyed by [CLASP-141] unique_serial.
class FleetLedger {
public:
    explicit FleetLedger(const std::filesystem::path& db_path);
    ~FleetLedger();

    FleetLedger(const FleetLedger&) = delete;
    FleetLedger& operator=(const FleetLedger&) = delete;
    FleetLedger(FleetLedger&& other) noexcept;
    FleetLedger& operator=(FleetLedger&& other) noexcept;

    /// Creates tables, constraints, indexes, and views if not present.
    void init_schema();

    /// Registers a new passive port and logs an initial REGISTER event atomically.
    PortRecord register_port(const PortRecord& port, std::string_view active_effector_id = "REGISTRY");

    /// Appends a mating/transfer/inspection event and updates the port state atomically.
    PortRecord record_event(const MatingEventInput& event);

    /// Looks up a port record by its immutable [CLASP-141] unique_serial.
    std::optional<PortRecord> get_port(std::string_view unique_serial) const;

    /// Returns all registered ports ordered by unique_serial.
    std::vector<PortRecord> list_ports() const;

    /// Returns recent events for a port ordered newest-first.
    std::vector<MatingEvent> list_events(std::string_view unique_serial, std::size_t limit = 50) const;

    /// Serializes the entire ledger to JSON conforming to clasp_port_ledger.schema.json.
    std::string export_json(bool include_events = false, std::size_t events_per_port = 10) const;

private:
    sqlite3* db_{nullptr};

    void exec_sql(const char* sql) const;
};

}  // namespace clasp::ledger

