// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include "verilator_sim_ctrl.h"

#include <getopt.h>
#include <iostream>
#include <signal.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <verilated.h>

// This is defined by Verilator and passed through the command line
#ifndef VM_TRACE
#define VM_TRACE 0
#endif

/**
 * Get the current simulation time
 *
 * Called by $time in Verilog, converts to double, to match what SystemC does
 */
double sc_time_stamp() { return VerilatorSimCtrl::GetInstance().GetSimTime(); }

#ifdef VL_USER_STOP
/**
 * A simulation stop was requested, e.g. through $stop() or $error()
 *
 * This function overrides Verilator's default implementation to more gracefully
 * shut down the simulation.
 */
void vl_stop(const char *filename, int linenum, const char *hier) VL_MT_UNSAFE {
  VerilatorSimCtrl::GetInstance().RequestStop(false);
}
#endif

VerilatorSimCtrl &VerilatorSimCtrl::GetInstance() {
  static VerilatorSimCtrl instance;
  return instance;
}

void VerilatorSimCtrl::SetTop(VerilatedToplevel *top, CData *sig_rst,
                              VerilatorSimCtrlFlags flags) {
  top_ = top;
  sig_rst_ = sig_rst;
  flags_ = flags;
}

std::pair<int, bool> VerilatorSimCtrl::Exec(int argc, char **argv) {
  bool exit_app = false;
  bool good_cmdline = ParseCommandArgs(argc, argv, exit_app);
  if (exit_app) {
    return std::make_pair(good_cmdline ? 0 : 1, false);
  }

  RunSimulation();

  int retcode = WasSimulationSuccessful() ? 0 : 1;
  return std::make_pair(retcode, true);
}

static bool read_ul_arg(unsigned long *arg_val, const char *arg_name,
                        const char *arg_text) {
  assert(arg_val && arg_name && arg_text);

  bool bad_fmt = false;
  bool out_of_range = false;

  // We have a stricter input format that strtoul: no leading space and no
  // leading plus or minus signs (strtoul has magic behaviour if the input
  // starts with a minus sign, but we don't want that). We're using auto base
  // detection, but a valid number will always start with 0-9 (since hex
  // constants start with "0x")
  if (!(('0' <= arg_text[0]) && (arg_text[0] <= '9'))) {
    bad_fmt = true;
  } else {
    char *txt_end;
    errno = 0;
    *arg_val = strtoul(arg_text, &txt_end, 0);

    // If txt_end doesn't point at a \0 then we didn't read the entire
    // argument.
    if (*txt_end) {
      bad_fmt = true;
    } else {
      // If the value was too big to fit in an unsigned long, strtoul sets
      // errno to ERANGE.
      if (errno != 0) {
        assert(errno == ERANGE);
        out_of_range = true;
      }
    }
  }

  if (bad_fmt) {
    std::cerr << "ERROR: Bad format for " << arg_name << " argument: `"
              << arg_text << "' is not an unsigned integer.\n";
    return false;
  }
  if (out_of_range) {
    std::cerr << "ERROR: Bad format for " << arg_name << " argument: `"
              << arg_text << "' is too big.\n";
    return false;
  }

  return true;
}

bool VerilatorSimCtrl::ParseCommandArgs(int argc, char **argv, bool &exit_app) {
  const struct option long_options[] = {
      {"term-after-cycles", required_argument, nullptr, 'c'},
      {"trace", optional_argument, nullptr, 't'},
      {"help", no_argument, nullptr, 'h'},
      {nullptr, no_argument, nullptr, 0}};

  while (1) {
    int c = getopt_long(argc, argv, "-:c:th", long_options, nullptr);
    if (c == -1) {
      break;
    }

    // Disable error reporting by getopt
    opterr = 0;

    switch (c) {
      case 0:
      case 1:
        break;
      case 't':
        if (!tracing_possible_) {
          std::cerr << "ERROR: Tracing has not been enabled at compile time."
                    << std::endl;
          exit_app = true;
          return false;
        }
        if (optarg != nullptr) {
          trace_file_path_.assign(optarg);
        }
        TraceOn();
        break;
      case 'c':
        if (!read_ul_arg(&term_after_cycles_, "term-after-cycles", optarg)) {
          exit_app = true;
          return false;
        }
        break;
      case 'h':
        PrintHelp();
        exit_app = true;
        break;
      case ':':  // missing argument
        std::cerr << "ERROR: Missing argument." << std::endl << std::endl;
        exit_app = true;
        return false;
      case '?':
      default:;
        // Ignore unrecognized options since they might be consumed by
        // Verilator's built-in parsing below.
    }
  }

  // Pass args to verilator
  Verilated::commandArgs(argc, argv);

  // Parse arguments for all registered extensions
  for (auto it = extension_array_.begin(); it != extension_array_.end(); ++it) {
    if (!(*it)->ParseCLIArguments(argc, argv, exit_app)) {
      exit_app = true;
      return false;
      if (exit_app) {
        return true;
      }
    }
  }
  return true;
}

void VerilatorSimCtrl::RunSimulation() {
  RegisterSignalHandler();

  // Print helper message for tracing
  if (TracingPossible()) {
    std::cout << "Tracing can be toggled by sending SIGUSR1 to this process:"
              << std::endl
              << "$ kill -USR1 " << getpid() << std::endl;
  }
  // Call all extension pre-exec methods
  for (auto it = extension_array_.begin(); it != extension_array_.end(); ++it) {
    (*it)->PreExec();
  }
  // Run the simulation
  Run();
  // Call all extension post-exec methods
  for (auto it = extension_array_.begin(); it != extension_array_.end(); ++it) {
    (*it)->PostExec();
  }
  // Print simulation speed info
  PrintStatistics();
  // Print helper message for tracing
  if (TracingEverEnabled()) {
    std::cout << std::endl
              << "You can view the simulation traces by calling" << std::endl
              << "$ gtkwave " << GetTraceFileName() << std::endl;
  }
}

void VerilatorSimCtrl::SetInitialResetDelay(unsigned int cycles) {
  initial_reset_delay_cycles_ = cycles;
}

void VerilatorSimCtrl::SetResetDuration(unsigned int cycles) {
  reset_duration_cycles_ = cycles;
}

void VerilatorSimCtrl::SetTimeout(unsigned int cycles) {
  term_after_cycles_ = cycles;
}

void VerilatorSimCtrl::RequestStop(bool simulation_success) {
  request_stop_ = true;
  simulation_success_ &= simulation_success;
}

void VerilatorSimCtrl::RegisterExtension(SimCtrlExtension *ext) {
  extension_array_.push_back(ext);
}

VerilatorSimCtrl::VerilatorSimCtrl()
    : top_(nullptr),
      cycle_(0),
      sim_time_(0),
#ifdef VM_TRACE_FMT_FST
      trace_file_path_("sim.fst"),
#else
      trace_file_path_("sim.vcd"),
#endif
      tracing_enabled_(false),
      tracing_enabled_changed_(false),
      tracing_ever_enabled_(false),
      tracing_possible_(VM_TRACE),
      initial_reset_delay_cycles_(2),
      reset_duration_cycles_(2),
      request_stop_(false),
      simulation_success_(true),
      tracer_(VerilatedTracer()),
      term_after_cycles_(0) {
}

void VerilatorSimCtrl::RegisterSignalHandler() {
  const int sigTypes[] = {
    SIGINT, SIGABRT, SIGFPE, SIGUSR1, SIGILL
  };
  struct sigaction sigIntHandler;

  sigIntHandler.sa_handler = SignalHandler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;

  for (int i = 0; i < sizeof(sigTypes)/sizeof(sigTypes[0]); ++i) {
    sigaction(sigTypes[i], &sigIntHandler, NULL);
  }
}

void VerilatorSimCtrl::SignalHandler(int sig) {
  VerilatorSimCtrl &simctrl = VerilatorSimCtrl::GetInstance();

  switch (sig) {
    // Abort (eg. assertion failure in DPI model)
    case SIGABRT:
    // Exceptions
    case SIGFPE:
    case SIGILL:
    case SIGSEGV:
      // Try to ensure that traces are up to date
      if (simctrl.TracingEverEnabled()) {
        simctrl.tracer_.close();
      }

      simctrl.RequestStop(true);
      exit(1);
      break;

    case SIGINT:
      simctrl.RequestStop(true);
      break;
    case SIGUSR1:
      if (simctrl.TracingEnabled()) {
        simctrl.TraceOff();
      } else {
        simctrl.TraceOn();
      }
      break;
  }
}

void VerilatorSimCtrl::PrintHelp() const {
  std::cout << "Execute a simulation model for " << GetName() << "\n\n";
  if (tracing_possible_) {
    std::cout << "-t|--trace\n"
                 "   --trace=FILE\n"
                 "  Write a trace file from the start\n\n";
  }
  std::cout << "-c|--term-after-cycles=N\n"
               "  Terminate simulation after N cycles. 0 means no timeout.\n\n"
               "-h|--help\n"
               "  Show help\n\n"
               "All arguments are passed to the design and can be used "
               "in the design, e.g. by DPI modules.\n\n";
}

bool VerilatorSimCtrl::TraceOn() {
  bool old_tracing_enabled = tracing_enabled_;

  tracing_enabled_ = tracing_possible_;
  tracing_ever_enabled_ = tracing_enabled_;

  if (old_tracing_enabled != tracing_enabled_) {
    tracing_enabled_changed_ = true;
  }
  return tracing_enabled_;
}

bool VerilatorSimCtrl::TraceOff() {
  if (tracing_enabled_) {
    tracing_enabled_changed_ = true;
  }
  tracing_enabled_ = false;
  return tracing_enabled_;
}

void VerilatorSimCtrl::PrintStatistics() const {
  // Count of cycles of the main system clock.
  uint64_t cycles = GetCycles();
  double speed_hz = cycles / (GetExecutionTimeMs() / 1000.0);
  double speed_khz = speed_hz / 1000.0;

  std::cout << std::endl
            << "Simulation statistics" << std::endl
            << "=====================" << std::endl
            << "Executed cycles:  " << std::dec << cycles << std::endl
            << "Wallclock time:   " << GetExecutionTimeMs() / 1000.0 << " s"
            << std::endl
            << "Simulation speed: " << speed_hz << " cycles/s "
            << "(" << speed_khz << " kHz)" << std::endl;

  int trace_size_byte;
  if (tracing_enabled_ && FileSize(GetTraceFileName(), trace_size_byte)) {
    std::cout << "Trace file size:  " << trace_size_byte << " B" << std::endl;
  }
}

std::string VerilatorSimCtrl::GetTraceFileName() const {
  return trace_file_path_;
}

void VerilatorSimCtrl::Run() {
  assert(top_ && "Use SetTop() first.");

  // We always need to enable this as tracing can be enabled at runtime
  if (tracing_possible_) {
    Verilated::traceEverOn(true);
    top_->trace(tracer_, 99, 0);
  }

  // Evaluate all initial blocks, including the DPI setup routines
  top_->eval();

  std::cout << std::endl
            << "Simulation running, end by pressing CTRL-c." << std::endl;

  time_begin_ = std::chrono::steady_clock::now();
  UnsetReset();
  Trace();

  unsigned long start_reset_cycle_ = initial_reset_delay_cycles_;
  unsigned long end_reset_cycle_ = start_reset_cycle_ + reset_duration_cycles_;

  // Initialize the count of system clock cycles.
  cycle_ = 0u;

  while (1) {
    if (cycle_ == start_reset_cycle_) {
      SetReset();
    } else if (cycle_ == end_reset_cycle_) {
      UnsetReset();
    }

    // Determine the maximal amount of simulation time that may elapse
    // before another clock edge (positive or negative) occurs.
    uint32_t time_del = std::numeric_limits<uint32_t>::max();
    for (auto it = clks_.begin(); it != clks_.end(); ++it) {
      uint32_t next = (*it).EdgeTimeDelta();
      if (time_del > next) {
        // This clock has an earlier transition; we must not advance the
        // simulation beyond any clock edge, because we cannot know to which
        // edge(s) the design is sensitive.
        time_del = next;
      }
    }

    // Note: the system clock (used to report the overall simulation performance)
    // must be the first clock in the list.
    bool sys_clk = true;
    bool sys_clk_rise = false;
    // Handle all clock transitions at this time.
    for (auto it = clks_.begin(); it != clks_.end(); ++it) {
      bool pos_edge = (*it).PosEdgeAfterTime(time_del);
      if (sys_clk) {
        sys_clk_rise = pos_edge;
      }
      sys_clk = false;
    }

    sim_time_ += time_del;

    // Call all extension on-clock methods
    if (sys_clk_rise) {
      cycle_++;

      for (auto it = extension_array_.begin(); it != extension_array_.end();
           ++it) {
        // The value passed is the number of half-clock periods.
        (*it)->OnClock(cycle_ * 2u);
      }
    }

    top_->eval();

    Trace();

    if (request_stop_) {
      std::cout << "Received stop request, shutting down simulation."
                << std::endl;
      break;
    }
    if (Verilated::gotFinish()) {
      std::cout << "Received $finish() from Verilog, shutting down simulation."
                << std::endl;
      break;
    }
    if (term_after_cycles_ && cycle_ >= term_after_cycles_) {
      std::cout << "Simulation timeout of " << term_after_cycles_
                << " cycles reached, shutting down simulation." << std::endl;
      break;
    }
  }

  top_->final();
  time_end_ = std::chrono::steady_clock::now();

  if (TracingEverEnabled()) {
    tracer_.close();
  }
}

std::string VerilatorSimCtrl::GetName() const {
  if (top_) {
    return top_->name();
  }
  return "unknown";
}

unsigned int VerilatorSimCtrl::GetExecutionTimeMs() const {
  return std::chrono::duration_cast<std::chrono::milliseconds>(time_end_ -
                                                               time_begin_)
      .count();
}

void VerilatorSimCtrl::SetReset() {
  if (flags_ & ResetPolarityNegative) {
    *sig_rst_ = 0;
  } else {
    *sig_rst_ = 1;
  }
}

void VerilatorSimCtrl::UnsetReset() {
  if (flags_ & ResetPolarityNegative) {
    *sig_rst_ = 1;
  } else {
    *sig_rst_ = 0;
  }
}

bool VerilatorSimCtrl::FileSize(std::string filepath, int &size_byte) const {
  struct stat statbuf;
  if (stat(filepath.data(), &statbuf) != 0) {
    size_byte = 0;
    return false;
  }

  size_byte = statbuf.st_size;
  return true;
}

void VerilatorSimCtrl::Trace() {
  // We cannot output a message when calling TraceOn()/TraceOff() as these
  // functions can be called from a signal handler. Instead we print the message
  // here from the main loop.
  if (tracing_enabled_changed_) {
    if (TracingEnabled()) {
      std::cout << "Tracing enabled." << std::endl;
    } else {
      std::cout << "Tracing disabled." << std::endl;
    }
    tracing_enabled_changed_ = false;
  }

  if (!TracingEnabled()) {
    return;
  }

  if (!tracer_.isOpen()) {
    tracer_.open(GetTraceFileName().c_str());
    std::cout << "Writing simulation traces to " << GetTraceFileName()
              << std::endl;
  }

  tracer_.dump(GetSimTime());
}
