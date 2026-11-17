#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cerrno>
#include <csignal>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "xarm_can_sdk.h"

namespace {

constexpr int kMotorCount = 8;
constexpr int kArmJointCount = 7;
constexpr double kPositionMin = -12.5;
constexpr double kPositionMax = 12.5;
constexpr double kKpMax = 500.0;
constexpr double kKdMax = 5.0;
constexpr double kMinimumJerkPeakVelocity = 1.875;

volatile sig_atomic_t g_stop_requested = 0;

void signal_handler(int) { g_stop_requested = 1; }

// This locks only the eight arm motor IDs, not the whole CAN interface.
// Head/waist controllers may use their disjoint IDs on the same bus.
class ArmMotorLock {
public:
    explicit ArmMotorLock(const std::string &can_if) {
        constexpr const char *kInterfaceChars =
            "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-";
        if (can_if.empty() || can_if.find_first_not_of(kInterfaceChars) != std::string::npos) {
            throw std::runtime_error("invalid CAN interface name for motor lock: " + can_if);
        }
        const std::string path = "/tmp/k1-" + can_if + "-arm-01-08.lock";
        fd_ = open(path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0600);
        if (fd_ < 0) {
            throw std::runtime_error("cannot open motor lock " + path + ": " + std::strerror(errno));
        }
        if (flock(fd_, LOCK_EX | LOCK_NB) < 0) {
            const int error = errno;
            close(fd_);
            fd_ = -1;
            if (error == EWOULDBLOCK || error == EAGAIN) {
                throw std::runtime_error("arm motor IDs 0x01-0x08 are already in use on " + can_if);
            }
            throw std::runtime_error("cannot lock arm motor IDs on " + can_if + ": " +
                                     std::strerror(error));
        }
    }

    ArmMotorLock(const ArmMotorLock &) = delete;
    ArmMotorLock &operator=(const ArmMotorLock &) = delete;
    ~ArmMotorLock() { if (fd_ >= 0) close(fd_); }

private:
    int fd_ = -1;
};

struct Options {
    std::string file;
    std::string can_if;
    std::string file2;
    std::string can_if2;
    double minimum_ramp_seconds = 3.0;
    double max_velocity_rad_s = 0.35;
    double max_delta_rad = 4.0;
    int action_hold_ms = 0;
    int hold_ms = 1000;
    int frame_interval_us = 2500;
    long start_group = 0;
    long end_group = -1;
    bool enable_fd = true;
    bool dry_run = false;
    bool probe_only = false;
};

struct LogFrame {
    uint32_t id = 0;
    std::array<uint8_t, 8> data{};
};

using FrameGroup = std::array<LogFrame, kMotorCount>;

struct ParsedAction {
    std::string path;
    char side = '\0';
    std::vector<FrameGroup> groups;
    size_t source_group_count = 0;
    size_t selected_start_group = 0;
    size_t selected_end_group = 0;
};

struct MotionParam {
    float position = 0.0F;
    float kp = 0.0F;
    float kd = 0.0F;
};

std::string sdk_error() {
    char buffer[512] = {0};
    xarm_sdk_get_last_error(buffer, static_cast<int>(sizeof(buffer)));
    return buffer;
}

void require_sdk_ok(int result, const std::string &step) {
    if (result != XARM_SDK_OK) {
        throw std::runtime_error(step + " failed: " + sdk_error());
    }
}

double uint_to_float(uint32_t value, double minimum, double maximum, int bits) {
    const uint32_t span = (1U << bits) - 1U;
    return static_cast<double>(value) * (maximum - minimum) /
               static_cast<double>(span) +
           minimum;
}

uint16_t position_raw(const LogFrame &frame) {
    return static_cast<uint16_t>((static_cast<uint16_t>(frame.data[0]) << 8U) |
                                 frame.data[1]);
}

MotionParam decode_motion_param(const LogFrame &frame) {
    const uint16_t position = position_raw(frame);
    const uint16_t kp = static_cast<uint16_t>(((frame.data[3] & 0x0FU) << 8U) |
                                               frame.data[4]);
    const uint16_t kd = static_cast<uint16_t>((frame.data[5] << 4U) |
                                               (frame.data[6] >> 4U));
    return MotionParam{
        static_cast<float>(uint_to_float(position, kPositionMin, kPositionMax, 16)),
        static_cast<float>(uint_to_float(kp, 0.0, kKpMax, 12)),
        static_cast<float>(uint_to_float(kd, 0.0, kKdMax, 12)),
    };
}

bool is_control_frame(const LogFrame &frame, uint8_t tail) {
    if (frame.data[7] != tail) return false;
    for (int i = 0; i < 7; ++i) {
        if (frame.data[static_cast<size_t>(i)] != 0xFFU) return false;
    }
    return true;
}

uint8_t parse_hex_byte(const std::string &text, size_t offset) {
    return static_cast<uint8_t>(std::stoul(text.substr(offset, 2), nullptr, 16));
}

LogFrame parse_log_line(const std::string &input, size_t line_number,
                        const std::string &path) {
    std::string line = input;
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line.size() != 20 || line[3] != '#') {
        throw std::runtime_error(path + ":" + std::to_string(line_number) +
                                 " invalid frame format: " + line);
    }

    LogFrame frame;
    try {
        frame.id = static_cast<uint32_t>(std::stoul(line.substr(0, 3), nullptr, 16));
        for (size_t i = 0; i < frame.data.size(); ++i) {
            frame.data[i] = parse_hex_byte(line, 4 + i * 2);
        }
    } catch (const std::exception &) {
        throw std::runtime_error(path + ":" + std::to_string(line_number) +
                                 " invalid hexadecimal frame: " + line);
    }
    return frame;
}

char infer_side(const std::string &path) {
    const std::string name = std::filesystem::path(path).filename().string();
    if (!name.empty() && (name.front() == 'L' || name.front() == 'R')) {
        return name.front();
    }
    throw std::runtime_error("action filename must begin with L or R: " + path);
}

ParsedAction parse_action(const std::string &path) {
    std::ifstream stream(path);
    if (!stream) throw std::runtime_error("cannot open action log: " + path);

    std::vector<LogFrame> frames;
    std::string line;
    size_t line_number = 0;
    while (std::getline(stream, line)) {
        ++line_number;
        if (line.empty() || line == "\r") continue;
        frames.push_back(parse_log_line(line, line_number, path));
    }

    if (frames.size() < 24 || (frames.size() - 16) % kMotorCount != 0) {
        throw std::runtime_error(path +
                                 " must contain 8 enable frames, complete 8-frame "
                                 "motion groups, and 8 disable frames");
    }

    for (int i = 0; i < kMotorCount; ++i) {
        const uint32_t expected_id = static_cast<uint32_t>(i + 1);
        if (frames[static_cast<size_t>(i)].id != expected_id ||
            !is_control_frame(frames[static_cast<size_t>(i)], 0xFCU)) {
            throw std::runtime_error(path + " has an invalid enable frame block");
        }
        const size_t tail_index = frames.size() - kMotorCount + static_cast<size_t>(i);
        if (frames[tail_index].id != expected_id ||
            !is_control_frame(frames[tail_index], 0xFDU)) {
            throw std::runtime_error(path + " has an invalid disable frame block");
        }
    }

    ParsedAction action;
    action.path = path;
    action.side = infer_side(path);
    for (size_t offset = kMotorCount; offset < frames.size() - kMotorCount;
         offset += kMotorCount) {
        FrameGroup group;
        for (int i = 0; i < kMotorCount; ++i) {
            const LogFrame &frame = frames[offset + static_cast<size_t>(i)];
            if (frame.id != static_cast<uint32_t>(i + 1)) {
                throw std::runtime_error(path + " motion group at line " +
                                         std::to_string(offset + 1) +
                                         " is not ordered 001 through 008");
            }
            group[static_cast<size_t>(i)] = frame;
        }
        action.groups.push_back(group);
    }
    if (action.groups.empty()) throw std::runtime_error(path + " has no motion groups");
    action.source_group_count = action.groups.size();
    action.selected_end_group = action.groups.size();
    return action;
}

void select_group_range(ParsedAction &action, long start_group, long end_group) {
    const size_t source_count = action.groups.size();
    const size_t start = static_cast<size_t>(start_group);
    const size_t end = end_group < 0 ? source_count : static_cast<size_t>(end_group);

    if (start >= source_count) {
        throw std::runtime_error(action.path + " --start-group " +
                                 std::to_string(start_group) +
                                 " is outside source group count " +
                                 std::to_string(source_count));
    }
    if (end > source_count) {
        throw std::runtime_error(action.path + " --end-group " +
                                 std::to_string(end_group) +
                                 " exceeds source group count " +
                                 std::to_string(source_count));
    }
    if (start >= end) {
        throw std::runtime_error("selected action range must satisfy start < end");
    }

    const auto first = action.groups.begin() +
                       static_cast<std::vector<FrameGroup>::difference_type>(start);
    const auto last = action.groups.begin() +
                      static_cast<std::vector<FrameGroup>::difference_type>(end);
    std::vector<FrameGroup> selected(first, last);
    action.groups = std::move(selected);
    action.selected_start_group = start;
    action.selected_end_group = end;
}

std::array<float, kMotorCount> group_positions(const FrameGroup &group) {
    std::array<float, kMotorCount> result{};
    for (int i = 0; i < kMotorCount; ++i) {
        result[static_cast<size_t>(i)] =
            decode_motion_param(group[static_cast<size_t>(i)]).position;
    }
    return result;
}

std::array<MotionParam, kMotorCount> group_params(const FrameGroup &group) {
    std::array<MotionParam, kMotorCount> result{};
    for (int i = 0; i < kMotorCount; ++i) {
        result[static_cast<size_t>(i)] =
            decode_motion_param(group[static_cast<size_t>(i)]);
    }
    return result;
}

std::array<float, kMotorCount> zero_positions(char side) {
    static constexpr std::array<uint16_t, kMotorCount> kRight = {
        0x7FFE, 0x7FF5, 0x8002, 0x8002, 0x8007, 0x7FEA, 0x8003, 0x7FFF,
    };
    static constexpr std::array<uint16_t, kMotorCount> kLeft = {
        0x7FFF, 0x7FED, 0x7FFB, 0x800F, 0x7FFF, 0x7FF3, 0x8013, 0x7FF7,
    };
    const auto &raw = side == 'L' ? kLeft : kRight;
    std::array<float, kMotorCount> result{};
    for (int i = 0; i < kMotorCount; ++i) {
        result[static_cast<size_t>(i)] = static_cast<float>(
            uint_to_float(raw[static_cast<size_t>(i)], kPositionMin, kPositionMax, 16));
    }
    return result;
}

int open_bound_can_socket(const std::string &can_if, bool nonblocking,
                          bool enable_fd) {
    const int socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd < 0) throw std::runtime_error("cannot create CAN socket for " + can_if);

    if (enable_fd) {
        int enabled = 1;
        if (setsockopt(socket_fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enabled,
                       sizeof(enabled)) < 0) {
            close(socket_fd);
            throw std::runtime_error("cannot enable CAN-FD socket mode for " + can_if);
        }
    }

    struct ifreq request {};
    std::strncpy(request.ifr_name, can_if.c_str(), IFNAMSIZ - 1);
    if (ioctl(socket_fd, SIOCGIFINDEX, &request) < 0) {
        close(socket_fd);
        throw std::runtime_error("CAN interface does not exist: " + can_if);
    }

    struct sockaddr_can address {};
    address.can_family = AF_CAN;
    address.can_ifindex = request.ifr_ifindex;
    if (bind(socket_fd, reinterpret_cast<struct sockaddr *>(&address), sizeof(address)) < 0) {
        close(socket_fd);
        throw std::runtime_error("cannot bind CAN socket to " + can_if);
    }

    if (nonblocking) {
        const int flags = fcntl(socket_fd, F_GETFL, 0);
        if (flags < 0 || fcntl(socket_fd, F_SETFL, flags | O_NONBLOCK) < 0) {
            close(socket_fd);
            throw std::runtime_error("cannot make CAN socket nonblocking: " + can_if);
        }
    }
    return socket_fd;
}

class ArmRuntime {
public:
    ArmRuntime(const ParsedAction &action, const std::string &can_if, bool enable_fd)
        : action_(action), can_if_(can_if), enable_fd_(enable_fd) {}

    ArmRuntime(const ArmRuntime &) = delete;
    ArmRuntime &operator=(const ArmRuntime &) = delete;

    ~ArmRuntime() {
        if (handle_ != nullptr) {
            if (enabled_) xarm_sdk_disable_all(handle_);
            xarm_sdk_destroy(handle_);
        }
        if (monitor_fd_ >= 0) close(monitor_fd_);
        if (send_fd_ >= 0) close(send_fd_);
    }

    void initialize() {
        monitor_fd_ = open_bound_can_socket(can_if_, true, enable_fd_);
        send_fd_ = open_bound_can_socket(can_if_, false, enable_fd_);

        std::array<struct can_filter, kMotorCount> filters{};
        for (int i = 0; i < kMotorCount; ++i) {
            filters[static_cast<size_t>(i)].can_id = static_cast<canid_t>(0x11 + i);
            filters[static_cast<size_t>(i)].can_mask = CAN_SFF_MASK;
        }
        if (setsockopt(monitor_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(),
                       static_cast<socklen_t>(sizeof(filters))) < 0) {
            throw std::runtime_error("cannot install feedback filters on " + can_if_);
        }

        require_sdk_ok(xarm_sdk_create(can_if_.c_str(), enable_fd_ ? 1 : 0, &handle_),
                       "xarm_sdk_create(" + can_if_ + ")");

        const int motor_types[kArmJointCount] = {
            XARM_SDK_MOTOR_DM8009, XARM_SDK_MOTOR_DM8009,
            XARM_SDK_MOTOR_DM4340, XARM_SDK_MOTOR_DM4340,
            XARM_SDK_MOTOR_DM4310, XARM_SDK_MOTOR_DM4310,
            XARM_SDK_MOTOR_DM4310,
        };
        const uint32_t send_ids[kArmJointCount] = {0x01, 0x02, 0x03, 0x04,
                                                    0x05, 0x06, 0x07};
        const uint32_t recv_ids[kArmJointCount] = {0x11, 0x12, 0x13, 0x14,
                                                    0x15, 0x16, 0x17};
        require_sdk_ok(xarm_sdk_init_arm_motors(handle_, motor_types, send_ids, recv_ids,
                                                 kArmJointCount),
                       "xarm_sdk_init_arm_motors(" + can_if_ + ")");
        require_sdk_ok(xarm_sdk_init_gripper_motor(handle_, XARM_SDK_MOTOR_DM4310,
                                                    0x08, 0x18),
                       "xarm_sdk_init_gripper_motor(" + can_if_ + ")");
        require_sdk_ok(xarm_sdk_set_callback_mode_state_all(handle_),
                       "xarm_sdk_set_callback_mode_state_all(" + can_if_ + ")");
        require_sdk_ok(xarm_sdk_enable_all(handle_),
                       "xarm_sdk_enable_all(" + can_if_ + ")");
        enabled_ = true;
    }

    std::array<float, kMotorCount> read_current_positions() {
        std::array<bool, kMotorCount> seen{};
        drain_monitor(nullptr);

        // Keep the bounded eight-reply query window so an installed gripper
        // replying later than the arm joints still supplies its measured start.
        for (int attempt = 0; attempt < 40 && !g_stop_requested; ++attempt) {
            require_sdk_ok(xarm_sdk_refresh_all(handle_),
                           "xarm_sdk_refresh_all(" + can_if_ + ")");
            xarm_sdk_recv_all(handle_, 5000);
            drain_monitor(&seen);
            if (std::all_of(seen.begin(), seen.end(), [](bool value) { return value; })) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }

        if (g_stop_requested) throw std::runtime_error("interrupted while reading arm state");
        if (!std::all_of(seen.begin(), seen.begin() + kArmJointCount,
                         [](bool value) { return value; })) {
            std::string missing;
            for (int i = 0; i < kArmJointCount; ++i) {
                if (!seen[static_cast<size_t>(i)]) {
                    if (!missing.empty()) missing += ",";
                    missing += "0x" + hex_id(0x11 + i);
                }
            }
            throw std::runtime_error("missing motor feedback on " + can_if_ + ": " + missing);
        }

        std::array<xarm_sdk_joint_state_t, kArmJointCount> arm_states{};
        require_sdk_ok(xarm_sdk_get_arm_joint_states(handle_, arm_states.data(),
                                                       kArmJointCount),
                       "xarm_sdk_get_arm_joint_states(" + can_if_ + ")");

        std::array<float, kMotorCount> result{};
        for (int i = 0; i < kArmJointCount; ++i) {
            result[static_cast<size_t>(i)] = arm_states[static_cast<size_t>(i)].pos;
        }
        gripper_feedback_available_ = seen[kArmJointCount];
        if (gripper_feedback_available_) {
            xarm_sdk_joint_state_t gripper_state{};
            require_sdk_ok(xarm_sdk_get_gripper_state(handle_, &gripper_state),
                           "xarm_sdk_get_gripper_state(" + can_if_ + ")");
            result[kArmJointCount] = gripper_state.pos;
        } else {
            // Keep the eight-frame log and all gripper commands. Without a reply,
            // the cached SDK value is not a measurement: use the first selected
            // log target as the gripper's command-space start instead.
            result[kArmJointCount] =
                group_positions(action_.groups.front())[kArmJointCount];
            std::cerr << "[WARN] no gripper feedback 0x018 on " << can_if_
                      << "; using first log target (unmeasured); "
                         "gripper commands will still be sent to 0x008\n";
        }
        for (float position : result) {
            if (!std::isfinite(position) || position < kPositionMin ||
                position > kPositionMax) {
                throw std::runtime_error("invalid current position returned on " + can_if_);
            }
        }
        return result;
    }

    void command_positions(const std::array<float, kMotorCount> &positions,
                           const std::array<MotionParam, kMotorCount> &params) {
        std::array<xarm_sdk_mit_param_t, kArmJointCount> arm_commands{};
        for (int i = 0; i < kArmJointCount; ++i) {
            arm_commands[static_cast<size_t>(i)] = {
                positions[static_cast<size_t>(i)], 0.0F,
                params[static_cast<size_t>(i)].kp,
                params[static_cast<size_t>(i)].kd, 0.0F,
            };
        }
        xarm_sdk_mit_param_t gripper_command{
            positions[7], 0.0F, params[7].kp, params[7].kd, 0.0F,
        };
        require_sdk_ok(xarm_sdk_arm_mit_control(handle_, arm_commands.data(),
                                                 kArmJointCount),
                       "xarm_sdk_arm_mit_control(" + can_if_ + ")");
        require_sdk_ok(xarm_sdk_gripper_mit_control(handle_, &gripper_command),
                       "xarm_sdk_gripper_mit_control(" + can_if_ + ")");
        xarm_sdk_recv_all(handle_, 200);
        drain_monitor(nullptr);
    }

    void send_body_frame(const LogFrame &log_frame) {
        struct can_frame frame {};
        frame.can_id = log_frame.id & CAN_SFF_MASK;
        frame.can_dlc = 8;
        std::copy(log_frame.data.begin(), log_frame.data.end(), frame.data);
        const ssize_t written = write(send_fd_, &frame, CAN_MTU);
        if (written != CAN_MTU) {
            throw std::runtime_error("failed to send action frame on " + can_if_);
        }
    }

    void pump_feedback() {
        xarm_sdk_recv_all(handle_, 100);
        drain_monitor(nullptr);
    }

    void disable() {
        if (enabled_) {
            require_sdk_ok(xarm_sdk_disable_all(handle_),
                           "xarm_sdk_disable_all(" + can_if_ + ")");
            enabled_ = false;
        }
    }

    const ParsedAction &action() const { return action_; }
    const std::string &can_if() const { return can_if_; }
    bool has_gripper_feedback() const { return gripper_feedback_available_; }

private:
    static std::string hex_id(int value) {
        std::ostringstream stream;
        stream << std::uppercase << std::hex << std::setw(3) << std::setfill('0') << value;
        return stream.str();
    }

    void drain_monitor(std::array<bool, kMotorCount> *seen) {
        while (true) {
            struct canfd_frame frame {};
            const ssize_t size = read(monitor_fd_, &frame, sizeof(frame));
            if (size < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) break;
                throw std::runtime_error("failed to read motor feedback on " + can_if_);
            }
            if (size != CAN_MTU && size != CANFD_MTU) continue;
            const uint32_t id = frame.can_id & CAN_SFF_MASK;
            if (seen != nullptr && id >= 0x11 && id <= 0x18) {
                (*seen)[static_cast<size_t>(id - 0x11)] = true;
            }
        }
    }

    const ParsedAction &action_;
    std::string can_if_;
    bool enable_fd_ = true;
    xarm_sdk_handle_t handle_ = nullptr;
    int monitor_fd_ = -1;
    int send_fd_ = -1;
    bool enabled_ = false;
    bool gripper_feedback_available_ = false;
};

double maximum_delta(const std::vector<std::array<float, kMotorCount>> &starts,
                     const std::vector<std::array<float, kMotorCount>> &targets) {
    double maximum = 0.0;
    for (size_t arm = 0; arm < starts.size(); ++arm) {
        for (int joint = 0; joint < kMotorCount; ++joint) {
            maximum = std::max(maximum,
                               std::abs(static_cast<double>(
                                   targets[arm][static_cast<size_t>(joint)] -
                                   starts[arm][static_cast<size_t>(joint)])));
        }
    }
    return maximum;
}

double ramp_duration(const Options &options, double delta) {
    if (delta > options.max_delta_rad) {
        throw std::runtime_error("required joint transition " + std::to_string(delta) +
                                 " rad exceeds --max-delta " +
                                 std::to_string(options.max_delta_rad));
    }
    return std::max(options.minimum_ramp_seconds,
                    kMinimumJerkPeakVelocity * delta / options.max_velocity_rad_s);
}

double minimum_jerk(double u) {
    u = std::clamp(u, 0.0, 1.0);
    return 10.0 * u * u * u - 15.0 * u * u * u * u +
           6.0 * u * u * u * u * u;
}

void run_ramp(const std::vector<std::unique_ptr<ArmRuntime>> &runtimes,
              const std::vector<std::array<float, kMotorCount>> &starts,
              const std::vector<std::array<float, kMotorCount>> &targets,
              const std::vector<std::array<MotionParam, kMotorCount>> &params,
              double duration_seconds) {
    constexpr auto period = std::chrono::milliseconds(20);
    const int steps = std::max(1, static_cast<int>(std::ceil(duration_seconds / 0.02)));
    auto next_tick = std::chrono::steady_clock::now();

    for (int step = 0; step <= steps && !g_stop_requested; ++step) {
        const double blend = minimum_jerk(static_cast<double>(step) / steps);
        for (size_t arm = 0; arm < runtimes.size(); ++arm) {
            std::array<float, kMotorCount> command{};
            for (int joint = 0; joint < kMotorCount; ++joint) {
                const size_t index = static_cast<size_t>(joint);
                command[index] = starts[arm][index] +
                                 static_cast<float>((targets[arm][index] - starts[arm][index]) *
                                                    blend);
            }
            runtimes[arm]->command_positions(command, params[arm]);
        }
        next_tick += period;
        std::this_thread::sleep_until(next_tick);
    }
    if (g_stop_requested) throw std::runtime_error("action interrupted");
}

void play_body(const std::vector<std::unique_ptr<ArmRuntime>> &runtimes,
               int frame_interval_us) {
    const size_t group_count = runtimes.front()->action().groups.size();
    for (const auto &runtime : runtimes) {
        if (runtime->action().groups.size() != group_count) {
            throw std::runtime_error("dual-arm action logs must have equal motion group counts");
        }
    }

    auto next_tick = std::chrono::steady_clock::now();
    const auto interval = std::chrono::microseconds(frame_interval_us);
    for (size_t group = 0; group < group_count && !g_stop_requested; ++group) {
        for (int motor = 0; motor < kMotorCount && !g_stop_requested; ++motor) {
            for (const auto &runtime : runtimes) {
                runtime->send_body_frame(
                    runtime->action().groups[group][static_cast<size_t>(motor)]);
            }
            next_tick += interval;
            std::this_thread::sleep_until(next_tick);
        }
        for (const auto &runtime : runtimes) runtime->pump_feedback();
    }
    if (g_stop_requested) throw std::runtime_error("action interrupted");
}

void hold_final_action_group(const std::vector<std::unique_ptr<ArmRuntime>> &runtimes,
                             int hold_ms, int frame_interval_us) {
    if (hold_ms <= 0) return;

    const auto group_period = std::chrono::microseconds(
        static_cast<long long>(frame_interval_us) * kMotorCount);
    const auto hold_duration = std::chrono::milliseconds(hold_ms);
    const int repetitions = std::max(
        1, static_cast<int>(std::ceil(
               static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                                        hold_duration)
                                        .count()) /
               static_cast<double>(group_period.count()))));
    auto next_tick = std::chrono::steady_clock::now();
    const auto frame_interval = std::chrono::microseconds(frame_interval_us);

    for (int repetition = 0; repetition < repetitions && !g_stop_requested;
         ++repetition) {
        for (int motor = 0; motor < kMotorCount && !g_stop_requested; ++motor) {
            for (const auto &runtime : runtimes) {
                runtime->send_body_frame(
                    runtime->action().groups.back()[static_cast<size_t>(motor)]);
            }
            next_tick += frame_interval;
            std::this_thread::sleep_until(next_tick);
        }
        for (const auto &runtime : runtimes) runtime->pump_feedback();
    }
    if (g_stop_requested) throw std::runtime_error("action interrupted");
}

void print_usage(const char *program) {
    std::cout
        << "Usage: " << program << " -f FILE -c CAN [options]\n"
        << "  -f, --file FILE           raw action log (enable/body/disable)\n"
        << "  -c, --can CAN             CAN interface for FILE\n"
        << "  -f2, --file2 FILE         optional second-arm raw action log\n"
        << "  -c2, --can2 CAN           CAN interface for FILE2\n"
        << "      --ramp-seconds SEC    minimum S-curve duration (default 3.0)\n"
        << "      --max-velocity RAD_S  S-curve peak joint speed limit (default 0.35)\n"
        << "      --max-delta RAD       reject larger transitions (default 4.0)\n"
        << "      --action-hold-ms MS   repeat final action group before return (default 0)\n"
        << "      --hold-ms MS          zero-position hold before disable (default 1000)\n"
        << "      --frame-us US         interval per raw CAN frame (default 2500)\n"
        << "      --start-group N       first source motion group, inclusive (default 0)\n"
        << "      --end-group N         last source motion group, exclusive (default all)\n"
        << "      --fd 0|1              xarm SDK CAN-FD mode (default 1)\n"
        << "      --probe               read and print current state, then disable\n"
        << "      --dry-run             validate logs without opening CAN\n";
}

Options parse_options(int argc, char **argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        auto value = [&](const std::string &name) -> std::string {
            if (i + 1 >= argc) throw std::runtime_error("missing value for " + name);
            return argv[++i];
        };

        if (argument == "-f" || argument == "--file") {
            options.file = value(argument);
        } else if (argument == "-c" || argument == "--can") {
            options.can_if = value(argument);
        } else if (argument == "-f2" || argument == "--file2") {
            options.file2 = value(argument);
        } else if (argument == "-c2" || argument == "--can2") {
            options.can_if2 = value(argument);
        } else if (argument == "--ramp-seconds") {
            options.minimum_ramp_seconds = std::stod(value(argument));
        } else if (argument == "--max-velocity") {
            options.max_velocity_rad_s = std::stod(value(argument));
        } else if (argument == "--max-delta") {
            options.max_delta_rad = std::stod(value(argument));
        } else if (argument == "--action-hold-ms") {
            options.action_hold_ms = std::stoi(value(argument));
        } else if (argument == "--hold-ms") {
            options.hold_ms = std::stoi(value(argument));
        } else if (argument == "--frame-us") {
            options.frame_interval_us = std::stoi(value(argument));
        } else if (argument == "--start-group") {
            options.start_group = std::stol(value(argument));
        } else if (argument == "--end-group") {
            options.end_group = std::stol(value(argument));
        } else if (argument == "--fd") {
            const std::string mode = value(argument);
            if (mode != "0" && mode != "1") throw std::runtime_error("--fd must be 0 or 1");
            options.enable_fd = mode == "1";
        } else if (argument == "--dry-run") {
            options.dry_run = true;
        } else if (argument == "--probe") {
            options.probe_only = true;
        } else if (argument == "-h" || argument == "--help") {
            print_usage(argv[0]);
            std::exit(0);
        } else {
            throw std::runtime_error("unknown option: " + argument);
        }
    }

    if (options.file.empty()) throw std::runtime_error("-f/--file is required");
    if (!options.dry_run && options.can_if.empty()) {
        throw std::runtime_error("-c/--can is required unless --dry-run is used");
    }
    if (options.file2.empty() != options.can_if2.empty() && !options.dry_run) {
        throw std::runtime_error("-f2 and -c2 must be provided together");
    }
    if (options.minimum_ramp_seconds <= 0.0 || options.max_velocity_rad_s <= 0.0 ||
        options.max_delta_rad <= 0.0 || options.action_hold_ms < 0 ||
        options.hold_ms < 0 ||
        options.frame_interval_us <= 0) {
        throw std::runtime_error("motion timing and safety limits must be positive");
    }
    if (options.start_group < 0 || options.end_group < -1 ||
        (options.end_group >= 0 && options.start_group >= options.end_group)) {
        throw std::runtime_error(
            "group range must satisfy 0 <= --start-group < --end-group");
    }
    return options;
}

void print_action_summary(const ParsedAction &action, int frame_interval_us) {
    const double seconds = static_cast<double>(action.groups.size() * kMotorCount *
                                               frame_interval_us) /
                           1'000'000.0;
    std::cout << "[INFO] " << action.path << ": side=" << action.side
              << " source_groups=" << action.source_group_count << " range=["
              << action.selected_start_group << "," << action.selected_end_group
              << ") groups=" << action.groups.size() << " body=" << std::fixed
              << std::setprecision(2) << seconds << "s\n";
}

}  // namespace

int main(int argc, char **argv) {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    try {
        const Options options = parse_options(argc, argv);
        std::vector<ParsedAction> actions;
        actions.push_back(parse_action(options.file));
        if (!options.file2.empty()) actions.push_back(parse_action(options.file2));
        for (auto &action : actions) {
            select_group_range(action, options.start_group, options.end_group);
        }
        for (const auto &action : actions) print_action_summary(action, options.frame_interval_us);

        if (actions.size() == 2 && actions[0].side == actions[1].side) {
            throw std::runtime_error("dual-arm logs must contain one L action and one R action");
        }
        if (options.dry_run) {
            if (actions.size() == 2 && actions[0].groups.size() != actions[1].groups.size()) {
                throw std::runtime_error("dual-arm action logs have different motion group counts");
            }
            std::cout << "[PASS] log validation completed; CAN was not opened\n";
            return 0;
        }

        if (actions.size() == 2 && options.can_if == options.can_if2) {
            throw std::runtime_error("left and right arm actions cannot use the same CAN interface");
        }
        // Acquire every affected group before creating a socket or enabling a motor.
        std::vector<std::unique_ptr<ArmMotorLock>> motor_locks;
        motor_locks.push_back(std::make_unique<ArmMotorLock>(options.can_if));
        if (actions.size() == 2) {
            motor_locks.push_back(std::make_unique<ArmMotorLock>(options.can_if2));
        }

        std::vector<std::unique_ptr<ArmRuntime>> runtimes;
        runtimes.push_back(std::make_unique<ArmRuntime>(actions[0], options.can_if,
                                                        options.enable_fd));
        if (actions.size() == 2) {
            runtimes.push_back(std::make_unique<ArmRuntime>(actions[1], options.can_if2,
                                                            options.enable_fd));
        }

        std::vector<std::array<float, kMotorCount>> current_positions;
        std::vector<std::array<float, kMotorCount>> first_targets;
        std::vector<std::array<MotionParam, kMotorCount>> first_params;
        for (auto &runtime : runtimes) {
            std::cout << "[INFO] initializing " << runtime->can_if()
                      << "; required feedback 0x011-0x017, optional gripper 0x018\n";
            runtime->initialize();
            current_positions.push_back(runtime->read_current_positions());
            first_targets.push_back(group_positions(runtime->action().groups.front()));
            first_params.push_back(group_params(runtime->action().groups.front()));
            runtime->command_positions(current_positions.back(), first_params.back());
        }

        if (options.probe_only) {
            for (size_t arm = 0; arm < runtimes.size(); ++arm) {
                std::cout << "[INFO] current " << runtimes[arm]->can_if() << ":";
                for (int joint = 0; joint < kMotorCount; ++joint) {
                    std::cout << " J" << (joint + 1) << "=" << std::fixed
                              << std::setprecision(4)
                              << current_positions[arm][static_cast<size_t>(joint)];
                    if (joint == kArmJointCount &&
                        !runtimes[arm]->has_gripper_feedback()) {
                        std::cout << "(log target, unmeasured)";
                    }
                }
                std::cout << " rad\n";
            }
            for (auto &runtime : runtimes) runtime->disable();
            std::cout << "[PASS] state probe completed; no action body was sent\n";
            return 0;
        }

        const double start_delta = maximum_delta(current_positions, first_targets);
        const double start_seconds = ramp_duration(options, start_delta);
        std::cout << "[INFO] current-to-action S-curve: delta=" << std::fixed
                  << std::setprecision(3) << start_delta << "rad duration="
                  << std::setprecision(2) << start_seconds << "s\n";
        run_ramp(runtimes, current_positions, first_targets, first_params, start_seconds);

        std::cout << "[INFO] playing original action body\n";
        play_body(runtimes, options.frame_interval_us);
        if (options.action_hold_ms > 0) {
            std::cout << "[INFO] action body completed; holding final action pose for "
                      << options.action_hold_ms << "ms\n";
            hold_final_action_group(runtimes, options.action_hold_ms,
                                    options.frame_interval_us);
            std::cout << "[INFO] final action pose hold completed; starting return\n";
        }

        std::vector<std::array<float, kMotorCount>> end_starts;
        std::vector<std::array<float, kMotorCount>> zero_targets;
        std::vector<std::array<MotionParam, kMotorCount>> end_params;
        for (auto &runtime : runtimes) {
            // Preserve command continuity at the action/return boundary.  The arm may
            // still be tracking the final log target here; replacing that target with
            // the instantaneous measured position would abruptly brake the motion.
            end_starts.push_back(group_positions(runtime->action().groups.back()));
            zero_targets.push_back(zero_positions(runtime->action().side));
            end_params.push_back(group_params(runtime->action().groups.back()));
        }

        const double return_delta = maximum_delta(end_starts, zero_targets);
        const double return_seconds = ramp_duration(options, return_delta);
        std::cout << "[INFO] action-to-zero S-curve: delta=" << std::fixed
                  << std::setprecision(3) << return_delta << "rad duration="
                  << std::setprecision(2) << return_seconds << "s\n";
        run_ramp(runtimes, end_starts, zero_targets, end_params, return_seconds);
        std::cout << "[INFO] return ramp completed; holding zero for "
                  << options.hold_ms << "ms\n";

        if (options.hold_ms > 0) {
            const int hold_steps = std::max(1, options.hold_ms / 20);
            for (int step = 0; step < hold_steps && !g_stop_requested; ++step) {
                for (size_t arm = 0; arm < runtimes.size(); ++arm) {
                    runtimes[arm]->command_positions(zero_targets[arm], end_params[arm]);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
        }
        if (g_stop_requested) throw std::runtime_error("action interrupted");

        std::cout << "[INFO] zero hold completed; disabling arm\n";
        for (auto &runtime : runtimes) runtime->disable();
        std::cout << "[PASS] action completed, returned to zero, and disabled\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "[ERROR] " << error.what() << "\n";
        return g_stop_requested ? 130 : 1;
    }
}
