// Copyright 2026 vlai.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

namespace xarm_hardware {

enum class CommandMode { kPosition, kEffort };

// Keep command buffers consistent with the measured state while ownership of
// the hardware interfaces changes. This prevents a stale position command
// from being replayed after operating in effort mode.
inline void synchronize_commands_for_mode_switch(
    CommandMode mode, double current_position, double& position_command,
    double& velocity_command, double& effort_command) {
  velocity_command = 0.0;
  effort_command = 0.0;
  if (mode == CommandMode::kPosition) {
    position_command = current_position;
  }
}

}  // namespace xarm_hardware
