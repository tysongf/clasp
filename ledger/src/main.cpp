#include "clasp/ledger.hpp"

#include <cstdlib>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace {

void print_usage() {
    std::cerr << "Usage:\n"
              << "  clasp_ledger_cli init <db>\n"
              << "  clasp_ledger_cli register <db> <serial> <tier> <contents> <fill_state> "
                 "[--cert CERT] [--effector ID] [--fill-fraction F] [--fluid-kg KG]\n"
              << "  clasp_ledger_cli record-event <db> <serial> <effector_id> <event_type> "
                 "<contents> <fill_state> [--delta-cycles N] [--fill-fraction F] "
                 "[--fluid-kg KG] [--chilldown-k K] [--trapped-loss-g G] [--notes TEXT]\n"
              << "  clasp_ledger_cli get <db> <serial>\n"
              << "  clasp_ledger_cli list <db>\n"
              << "  clasp_ledger_cli export-json <db> [--with-events]\n"
              << "  clasp_ledger_cli parse-tag <datamatrix_payload>\n";
}

double parse_double_arg(const std::string& s, std::string_view flag) {
    char* end = nullptr;
    double val = std::strtod(s.c_str(), &end);
    if (end == s.c_str() || *end != '\0') {
        throw clasp::ledger::LedgerError("Invalid numeric value for " + std::string(flag) + ": " + s);
    }
    return val;
}

std::int64_t parse_int64_arg(const std::string& s, std::string_view flag) {
    char* end = nullptr;
    long long val = std::strtoll(s.c_str(), &end, 10);
    if (end == s.c_str() || *end != '\0') {
        throw clasp::ledger::LedgerError("Invalid integer value for " + std::string(flag) + ": " + s);
    }
    return static_cast<std::int64_t>(val);
}

std::string require_flag_value(const std::vector<std::string>& args, std::size_t& i) {
    if (i + 1 >= args.size()) {
        throw clasp::ledger::LedgerError("Missing value after flag " + args[i]);
    }
    return args[++i];
}

void print_port_summary(const clasp::ledger::PortRecord& p) {
    const auto limit = clasp::ledger::rated_cycle_limit(p.port_tier);
    std::cout << p.unique_serial
              << " | tier=" << clasp::ledger::to_string(p.port_tier)
              << " | contents=" << p.payload_contents
              << " | fill=" << clasp::ledger::to_string(p.fill_state)
              << " | cycles=" << p.cycle_count << "/" << limit
              << " | updated=" << p.last_updated_utc << "\n";
}

}  // namespace

int main(int argc, char** argv) {
    try {
        if (argc < 2) {
            print_usage();
            return 1;
        }

        const std::vector<std::string> args(argv + 1, argv + argc);
        const std::string& cmd = args[0];

        if (cmd == "parse-tag") {
            if (args.size() != 2) {
                print_usage();
                return 1;
            }
            const auto tag = clasp::ledger::parse_datamatrix_tag(args[1]);
            std::cout << "port_tier=" << clasp::ledger::to_string(tag.port_tier)
                      << " unique_serial=" << tag.unique_serial << "\n";
            return 0;
        }

        if (cmd == "init") {
            if (args.size() != 2) {
                print_usage();
                return 1;
            }
            clasp::ledger::FleetLedger ledger(args[1]);
            ledger.init_schema();
            std::cout << "Initialized CLASP ledger: " << args[1] << "\n";
            return 0;
        }

        if (cmd == "register") {
            if (args.size() < 6) {
                print_usage();
                return 1;
            }
            clasp::ledger::FleetLedger ledger(args[1]);
            ledger.init_schema();

            clasp::ledger::PortRecord port;
            port.unique_serial = args[2];
            port.port_tier = clasp::ledger::parse_port_tier(args[3]);
            port.payload_contents = args[4];
            port.fill_state = clasp::ledger::parse_fill_state(args[5]);
            std::string effector = "REGISTRY";

            for (std::size_t i = 6; i < args.size(); ++i) {
                if (args[i] == "--cert") {
                    port.certificate_number = require_flag_value(args, i);
                } else if (args[i] == "--effector") {
                    effector = require_flag_value(args, i);
                } else if (args[i] == "--fill-fraction") {
                    port.fill_fraction = parse_double_arg(require_flag_value(args, i), "--fill-fraction");
                } else if (args[i] == "--fluid-kg") {
                    port.fluid_mass_kg = parse_double_arg(require_flag_value(args, i), "--fluid-kg");
                } else {
                    throw clasp::ledger::LedgerError("Unknown flag: " + args[i]);
                }
            }

            const auto saved = ledger.register_port(port, effector);
            print_port_summary(saved);
            return 0;
        }

        if (cmd == "record-event") {
            if (args.size() < 7) {
                print_usage();
                return 1;
            }
            clasp::ledger::FleetLedger ledger(args[1]);
            clasp::ledger::MatingEventInput ev;
            ev.unique_serial = args[2];
            ev.active_effector_id = args[3];
            ev.event_type = clasp::ledger::parse_event_type(args[4]);
            ev.payload_contents = args[5];
            ev.fill_state = clasp::ledger::parse_fill_state(args[6]);

            for (std::size_t i = 7; i < args.size(); ++i) {
                if (args[i] == "--delta-cycles") {
                    ev.delta_cycles = parse_int64_arg(require_flag_value(args, i), "--delta-cycles");
                } else if (args[i] == "--fill-fraction") {
                    ev.fill_fraction = parse_double_arg(require_flag_value(args, i), "--fill-fraction");
                } else if (args[i] == "--fluid-kg") {
                    ev.fluid_mass_kg = parse_double_arg(require_flag_value(args, i), "--fluid-kg");
                } else if (args[i] == "--chilldown-k") {
                    ev.chilldown_temp_k = parse_double_arg(require_flag_value(args, i), "--chilldown-k");
                } else if (args[i] == "--trapped-loss-g") {
                    ev.trapped_loss_g = parse_double_arg(require_flag_value(args, i), "--trapped-loss-g");
                } else if (args[i] == "--notes") {
                    ev.notes = require_flag_value(args, i);
                } else {
                    throw clasp::ledger::LedgerError("Unknown flag: " + args[i]);
                }
            }

            const auto updated = ledger.record_event(ev);
            print_port_summary(updated);
            return 0;
        }

        if (cmd == "get") {
            if (args.size() != 3) {
                print_usage();
                return 1;
            }
            clasp::ledger::FleetLedger ledger(args[1]);
            const auto port = ledger.get_port(args[2]);
            if (!port.has_value()) {
                std::cerr << "Port not found: " << args[2] << "\n";
                return 2;
            }
            print_port_summary(*port);
            return 0;
        }

        if (cmd == "list") {
            if (args.size() != 2) {
                print_usage();
                return 1;
            }
            clasp::ledger::FleetLedger ledger(args[1]);
            for (const auto& p : ledger.list_ports()) {
                print_port_summary(p);
            }
            return 0;
        }

        if (cmd == "export-json") {
            if (args.size() < 2 || args.size() > 3) {
                print_usage();
                return 1;
            }
            bool with_events = false;
            if (args.size() == 3) {
                if (args[2] != "--with-events") {
                    throw clasp::ledger::LedgerError("Unknown flag: " + args[2]);
                }
                with_events = true;
            }
            clasp::ledger::FleetLedger ledger(args[1]);
            std::cout << ledger.export_json(with_events);
            return 0;
        }

        print_usage();
        return 1;
    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << "\n";
        return 1;
    }
}

