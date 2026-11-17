#include <cmath>

#include "xarm_hardware/command_mode_switch.hpp"

int main() {
  double position_command = -1.25;
  double velocity_command = 3.0;
  double effort_command = 4.0;

  xarm_hardware::synchronize_commands_for_mode_switch(
      xarm_hardware::CommandMode::kPosition, 0.42, position_command,
      velocity_command, effort_command);

  if (std::abs(position_command - 0.42) >= 1e-12 ||
      velocity_command != 0.0 || effort_command != 0.0) {
    return 1;
  }

  effort_command = 7.0;
  xarm_hardware::synchronize_commands_for_mode_switch(
      xarm_hardware::CommandMode::kEffort, 0.42, position_command,
      velocity_command, effort_command);

  return effort_command == 0.0 ? 0 : 1;
}
