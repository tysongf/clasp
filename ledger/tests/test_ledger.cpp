#include "clasp/ledger.hpp"

#include <cassert>
#include <iostream>
#include <string>

namespace {

using namespace clasp::ledger;

template <typename Fn>
void expect_throws(Fn&& fn, const char* msg) {
    bool threw = false;
    try {
        fn();
    } catch (const LedgerError&) {
        threw = true;
    }
    if (!threw) {
        std::cerr << "Expected LedgerError: " << msg << "\n";
        assert(false);
    }
}

void test_datamatrix_tag_parsing() {
    const auto tag = parse_datamatrix_tag("CLASP-F|SN-2026-0042");
    assert(tag.port_tier == PortTier::ClaspF);
    assert(tag.unique_serial == "SN-2026-0042");
    assert(format_datamatrix_tag(tag) == "CLASP-F|SN-2026-0042");

    expect_throws([] { parse_datamatrix_tag("CLASP-F"); }, "missing delimiter");
    expect_throws([] { parse_datamatrix_tag("CLASP-X|SN-001"); }, "invalid tier");
    expect_throws([] { parse_datamatrix_tag("CLASP-M|-BAD"); }, "invalid serial prefix");
}

void test_register_and_event_lifecycle() {
    FleetLedger ledger(":memory:");
    ledger.init_schema();
    // Idempotent init
    ledger.init_schema();

    PortRecord tank;
    tank.unique_serial = "TANK-LOX-001";
    tank.port_tier = PortTier::ClaspF;
    tank.payload_contents = "LOX";
    tank.fill_state = FillState::Empty;
    tank.fill_fraction = 0.0;
    tank.fluid_mass_kg = 0.0;
    tank.certificate_number = "CLASP-CERT-0007";
    tank.last_updated_utc = "2026-11-15T14:00:00Z";

    const auto created = ledger.register_port(tank, "DEPOT-INIT");
    assert(created.unique_serial == "TANK-LOX-001");
    assert(created.cycle_count == 0);
    assert(rated_cycle_limit(created.port_tier) == 1000);

    PortRecord bin;
    bin.unique_serial = "BIN-REG-009";
    bin.port_tier = PortTier::ClaspM;
    bin.payload_contents = "REGOLITH";
    bin.fill_state = FillState::Partial;
    bin.fill_fraction = 0.45;
    ledger.register_port(bin);
    assert(rated_cycle_limit(bin.port_tier) == 500);

    // Duplicate serial must fail
    expect_throws([&] { ledger.register_port(tank); }, "duplicate serial");

    // Chilldown & transfer event
    MatingEventInput transfer;
    transfer.unique_serial = "TANK-LOX-001";
    transfer.active_effector_id = "ARM-ALPHA-01";
    transfer.event_type = EventType::Transfer;
    transfer.payload_contents = "LOX";
    transfer.fill_state = FillState::Full;
    transfer.fill_fraction = 0.98;
    transfer.fluid_mass_kg = 1450.0;
    transfer.delta_cycles = 1;
    transfer.chilldown_temp_k = 114.5;
    transfer.trapped_loss_g = 4.9;
    transfer.occurred_at_utc = "2026-11-15T15:00:00Z";
    transfer.notes = "Nominal chilldown \"verified\" on flange";

    const auto updated = ledger.record_event(transfer);
    assert(updated.cycle_count == 1);
    assert(updated.fill_state == FillState::Full);
    assert(updated.fill_fraction.has_value() && *updated.fill_fraction == 0.98);
    assert(updated.fluid_mass_kg.has_value() && *updated.fluid_mass_kg == 1450.0);
    assert(updated.last_effector_id == "ARM-ALPHA-01");

    const auto events = ledger.list_events("TANK-LOX-001");
    assert(events.size() == 2);
    assert(events[0].event_type == EventType::Transfer);
    assert(events[0].chilldown_temp_k.has_value() && *events[0].chilldown_temp_k == 114.5);
    assert(events[1].event_type == EventType::Register);

    // JSON export includes escaped notes & schema fields
    const std::string json_out = ledger.export_json(true);
    assert(json_out.find("\"schema_version\": \"1.0.0\"") != std::string::npos);
    assert(json_out.find("\"rated_cycle_limit\": 1000") != std::string::npos);
    assert(json_out.find("\"rated_cycle_limit\": 500") != std::string::npos);
    assert(json_out.find("Nominal chilldown \\\"verified\\\" on flange") != std::string::npos);
}

void test_spec_guardrails() {
    FleetLedger ledger(":memory:");
    ledger.init_schema();

    PortRecord bad_lh2;
    bad_lh2.unique_serial = "TANK-LH2-001";
    bad_lh2.port_tier = PortTier::ClaspF;
    bad_lh2.payload_contents = "LH2";
    bad_lh2.fill_state = FillState::Empty;
    expect_throws([&] { ledger.register_port(bad_lh2); }, "LH2 excluded in Rev A");

    PortRecord bad_hydrazine = bad_lh2;
    bad_hydrazine.unique_serial = "TANK-HYD-001";
    bad_hydrazine.payload_contents = "HYDRAZINE";
    expect_throws([&] { ledger.register_port(bad_hydrazine); }, "Hypergolics excluded in Rev A");

    PortRecord bad_frac = bad_lh2;
    bad_frac.unique_serial = "TANK-OK-002";
    bad_frac.payload_contents = "LOX";
    bad_frac.fill_fraction = 1.25;
    expect_throws([&] { ledger.register_port(bad_frac); }, "fill_fraction > 1.0");

    MatingEventInput unknown_port;
    unknown_port.unique_serial = "DOES-NOT-EXIST";
    unknown_port.active_effector_id = "ARM-01";
    unknown_port.payload_contents = "LOX";
    unknown_port.fill_state = FillState::Empty;
    expect_throws([&] { ledger.record_event(unknown_port); }, "unknown serial");
}

}  // namespace

int main() {
    test_datamatrix_tag_parsing();
    test_register_and_event_lifecycle();
    test_spec_guardrails();
    std::cout << "All CLASP C++ fleet-ledger tests passed.\n";
    return 0;
}
