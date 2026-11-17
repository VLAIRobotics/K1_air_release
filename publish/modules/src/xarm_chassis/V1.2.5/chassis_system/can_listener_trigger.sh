#!/bin/bash
#
# CAN 监听触发器脚本 V1.2.5（新遥控器位协议，需重置版本）
# 功能：监听底盘 CAN 上的 0x051 帧，根据第 8 数据字节触发预录动作。
# 规则：脚本启动后必须先松开 SD（重置），才能松开 SA（执行）触发动作；
#       每个动作完成后必须再次松开 SD，才能执行下一个动作。
#
# 第 8 数据字节（data[7]）位定义：
#   bit1: SA，1->0（松开）执行当前选择的动作
#   bit6: SD，1->0（松开）重置/解锁动作执行
#   bit2/bit3: SB 三档，00=一档（套数0），10=二档（套数1），01=三档（套数2）
#   bit4/bit5: SC 三档，00=一档（动作0），10=二档（动作1），01=三档（动作2）
#   bit0/bit7: 忽略
#

set -u

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# 原始动作由 ramp_action_player 在运行时根据机械臂当前位置生成 S 曲线：
# 当前姿态 -> 原始动作 -> 零位 -> 失能。原始日志保持不变。
ACTION_PLAYER="$SCRIPT_DIR/../../../xarm_action/action/ramp_action_player"
LOG_FILE_DIR="$SCRIPT_DIR/../../../xarm_action/action/examples"

# 配置参数
CAN_INTERFACE="${CAN_INTERFACE:-can2}"
LEFT_ARM_CAN="${LEFT_ARM_CAN:-can0}"
RIGHT_ARM_CAN="${RIGHT_ARM_CAN:-can1}"
RAMP_SECONDS="${RAMP_SECONDS:-3.0}"
MAX_JOINT_VELOCITY="${MAX_JOINT_VELOCITY:-0.35}"
ACTION_HOLD_MS="${ACTION_HOLD_MS:-0}"
HOLD_MS="${HOLD_MS:-1000}"
# Rws.log 的原始 546 组中，主要动作暂定为 [150,400)。保留原日志，
# 仅在播放时裁剪前置静止段和录制尾段；可通过 systemd Environment 覆盖。
RWS_START_GROUP="${RWS_START_GROUP:-150}"
RWS_END_GROUP="${RWS_END_GROUP:-400}"
# Rws 已完成单臂实车调参；这些参数只作用于 Rws，避免影响其他未验证动作。
RWS_RAMP_SECONDS="${RWS_RAMP_SECONDS:-0.75}"
RWS_MAX_JOINT_VELOCITY="${RWS_MAX_JOINT_VELOCITY:-1.40}"
RWS_HOLD_MS="${RWS_HOLD_MS:-300}"
# Rhs1 已完成单臂实车调参。
RHS1_START_GROUP="${RHS1_START_GROUP:-110}"
RHS1_END_GROUP="${RHS1_END_GROUP:-475}"
RHS1_RAMP_SECONDS="${RHS1_RAMP_SECONDS:-0.75}"
RHS1_MAX_JOINT_VELOCITY="${RHS1_MAX_JOINT_VELOCITY:-1.40}"
RHS1_HOLD_MS="${RHS1_HOLD_MS:-300}"
# Rhs2 已完成单臂实车调参。
RHS2_START_GROUP="${RHS2_START_GROUP:-190}"
RHS2_END_GROUP="${RHS2_END_GROUP:-625}"
RHS2_RAMP_SECONDS="${RHS2_RAMP_SECONDS:-0.75}"
RHS2_MAX_JOINT_VELOCITY="${RHS2_MAX_JOINT_VELOCITY:-1.40}"
RHS2_HOLD_MS="${RHS2_HOLD_MS:-300}"
# 双臂爱心动作：播放到第 600 组后固定保持该组 2 秒，再回零。
AX_START_GROUP="${AX_START_GROUP:-110}"
AX_END_GROUP="${AX_END_GROUP:-601}"
AX_ACTION_HOLD_MS="${AX_ACTION_HOLD_MS:-2000}"
AX_RAMP_SECONDS="${AX_RAMP_SECONDS:-0.75}"
AX_MAX_JOINT_VELOCITY="${AX_MAX_JOINT_VELOCITY:-1.40}"
AX_HOLD_MS="${AX_HOLD_MS:-300}"
COOLDOWN_SECONDS=1

# 状态变量
is_running=0
last_end_time=0
child_pid=""
candump_pid=""
last_byte8=""
last_action_type=""
need_reset=1  # 默认需要重置，脚本启动后必须先松开 SD
current_action_name=""  # 当前准备触发的动作名称
last_sa_bit=""          # 用于识别 SA 是否从 1 松开到 0
last_sd_bit=""          # 用于识别 SD 是否从 1 松开到 0

# 仅停止本脚本启动的子进程，不影响用户在其他终端运行的 candump。
cleanup() {
    echo ""
    echo "[INFO] 脚本退出，清理资源..."
    if [ "$is_running" -eq 1 ] && [ -n "$child_pid" ] && kill -0 "$child_pid" 2>/dev/null; then
        kill -TERM "$child_pid" 2>/dev/null || true
        wait "$child_pid" 2>/dev/null || true
    fi
    if [ -n "$candump_pid" ] && kill -0 "$candump_pid" 2>/dev/null; then
        kill -TERM "$candump_pid" 2>/dev/null || true
        wait "$candump_pid" 2>/dev/null || true
    fi
    exit 0
}

trap cleanup INT TERM

die() {
    echo "[ERROR] $*" >&2
    exit 1
}

# 在启动前检查所有运行时依赖。这样 systemd 日志会直接显示缺失项，
# 而不是进入无提示的监听循环。
check_prerequisites() {
    command -v candump >/dev/null 2>&1 || die "未找到 candump；请安装 can-utils"
    command -v ip >/dev/null 2>&1 || die "未找到 ip 命令；请安装 iproute2"

    [ -x "$ACTION_PLAYER" ] || die "平滑动作播放器不存在或不可执行: $ACTION_PLAYER"

    local required_log
    for required_log in Rws.log Rhs1.log Rhs2.log Lax.log Rax.log; do
        [ -f "$LOG_FILE_DIR/$required_log" ] || die "动作日志不存在: $LOG_FILE_DIR/$required_log"
    done

    ip link show dev "$CAN_INTERFACE" >/dev/null 2>&1 \
        || die "CAN 接口不存在: $CAN_INTERFACE"

    local link_info
    link_info=$(ip -o link show dev "$CAN_INTERFACE")
    [[ "$link_info" == *"<"*"UP"* ]] \
        || die "CAN 接口未启用: $CAN_INTERFACE；请先运行 setup_can_interfaces.sh"

    local arm_can
    for arm_can in "$LEFT_ARM_CAN" "$RIGHT_ARM_CAN"; do
        ip link show dev "$arm_can" >/dev/null 2>&1 \
            || die "机械臂 CAN 接口不存在: $arm_can"
        link_info=$(ip -o link show dev "$arm_can")
        [[ "$link_info" == *"<"*"UP"* ]] \
            || die "机械臂 CAN 接口未启用: $arm_can"
    done
}

# 检查冷却时间
check_cooldown() {
    local current_time=$(date +%s)
    local time_since_last_end=$((current_time - last_end_time))

    if [ "$is_running" -eq 1 ]; then
        return 1
    fi

    if [ "$time_since_last_end" -lt "$COOLDOWN_SECONDS" ] && [ "$last_end_time" -ne 0 ]; then
        return 1
    fi

    return 0
}

# 执行触发命令
execute_command() {
    local action=$1
    local can_id=$2
    local log_file=$3
    local can_if=$4
    local log_file2=${5:-}
    local can_if2=${6:-}
    local start_group=${7:-}
    local end_group=${8:-}
    local action_ramp_seconds=${9:-$RAMP_SECONDS}
    local action_max_velocity=${10:-$MAX_JOINT_VELOCITY}
    local action_hold_ms=${11:-$HOLD_MS}
    local final_pose_hold_ms=${12:-$ACTION_HOLD_MS}
    local range_args=()

    if [ -n "$start_group" ]; then
        range_args+=(--start-group "$start_group")
    fi
    if [ -n "$end_group" ]; then
        range_args+=(--end-group "$end_group")
    fi

    echo "[INFO] 执行: ${action}"
    echo "[INFO] 动作参数: 斜坡=${action_ramp_seconds}s | 最大关节速度=${action_max_velocity}rad/s | 动作末姿态保持=${final_pose_hold_ms}ms | 回零保持=${action_hold_ms}ms"
    is_running=1

    if [ -n "$log_file2" ] && [ -n "$can_if2" ]; then
        "$ACTION_PLAYER" \
            -f "$log_file" -c "$can_if" -f2 "$log_file2" -c2 "$can_if2" \
            --ramp-seconds "$action_ramp_seconds" \
            --max-velocity "$action_max_velocity" \
            --action-hold-ms "$final_pose_hold_ms" \
            --hold-ms "$action_hold_ms" \
            "${range_args[@]}" &
    else
        "$ACTION_PLAYER" \
            -f "$log_file" -c "$can_if" \
            --ramp-seconds "$action_ramp_seconds" \
            --max-velocity "$action_max_velocity" \
            --action-hold-ms "$final_pose_hold_ms" \
            --hold-ms "$action_hold_ms" \
            "${range_args[@]}" &
    fi
    child_pid=$!

    if wait "$child_pid"; then
        exit_code=0
        echo "[INFO] 完成 (退出码: $exit_code)"
    else
        exit_code=$?
        echo "[ERROR] 动作发送失败 (退出码: $exit_code)" >&2
    fi

    is_running=0
    child_pid=""
    last_end_time=$(date +%s)

    # 动作执行完成后，需要等待重置信号
    need_reset=1
    current_action_name=""  # 清空当前动作名称
    echo "[INFO] 等待重置信号（请按下并松开 SD）..."
}

# SA 触发了预留/非法组合时，同样消费本次解锁资格。
# 这样每一次已接受的 SA 松开都必须由下一次 SD 松开重新解锁。
consume_trigger_without_action() {
    need_reset=1
    current_action_name=""
    echo "[INFO] 本次触发未执行动作，已重新上锁；请按下并松开 SD 后再试"
}

# 获取动作名称
get_action_name() {
    local suite=$1
    local action=$2

    if [ "$suite" -eq 0 ]; then
        if [ "$action" -eq 0 ]; then
            echo "套数0-动作0: Rws.log"
        elif [ "$action" -eq 1 ]; then
            echo "套数0-动作1: Rhs1.log"
        elif [ "$action" -eq 2 ]; then
            echo "套数0-动作2: 预留（暂无动作）"
        else
            echo "非法动作档位"
        fi
    elif [ "$suite" -eq 1 ]; then
        if [ "$action" -eq 0 ]; then
            echo "套数1-动作0: Rhs2.log"
        elif [ "$action" -eq 1 ]; then
            echo "套数1-动作1: Lax.log+Rax.log"
        elif [ "$action" -eq 2 ]; then
            echo "套数1-动作2: 预留（暂无动作）"
        else
            echo "非法动作档位"
        fi
    elif [ "$suite" -eq 2 ]; then
        echo "套数2: 预留（暂无动作）"
    else
        echo "非法套数档位"
    fi
}

# 将两个 bit 表示的三档开关转换成 0、1、2。
# 返回 -1 表示非法状态 11。
decode_three_position() {
    local first_bit=$1
    local second_bit=$2

    case "${first_bit}${second_bit}" in
        00) echo 0 ;;
        10) echo 1 ;;
        01) echo 2 ;;
        *)  echo -1 ;;
    esac
}

echo "========================================"
echo "  CAN 监听触发器 V1.2.5（新遥控器位协议，需重置）"
echo "========================================"
echo "监听: $CAN_INTERFACE | 冷却: ${COOLDOWN_SECONDS}s"
echo ""
echo "遥控器控制说明:"
echo "  SA (bit1): 从按下(1)松开到(0)时执行当前动作"
echo "  SD (bit6): 从按下(1)松开到(0)时重置/解锁"
echo "  SB (bit2/bit3): 00=一档, 10=二档, 01=三档"
echo "  SC (bit4/bit5): 00=一档, 10=二档, 01=三档"
echo ""
echo "动作映射:"
echo "  套数0 (上): 动作0→Rws.log | 动作1→Rhs1.log"
echo "  套数1 (中): 动作0→Rhs2.log | 动作1→Lax.log+Rax.log"
echo "  套数2 (下): 预留"
echo ""
echo "规则: 必须先松开 SD 重置，才能松开 SA 执行动作"
echo "      SA 触发预留或非法档位时，也必须重新松开 SD 才能再次触发"
echo "========================================"
echo "按 Ctrl+C 退出"
echo "接口映射: 左臂=$LEFT_ARM_CAN | 右臂=$RIGHT_ARM_CAN | 底盘=$CAN_INTERFACE"
echo "默认平滑参数: 最短斜坡=${RAMP_SECONDS}s | 最大关节速度=${MAX_JOINT_VELOCITY}rad/s | 动作末姿态保持=${ACTION_HOLD_MS}ms | 回零保持=${HOLD_MS}ms"
echo "Rws 参数: 范围=[${RWS_START_GROUP},${RWS_END_GROUP}) | 斜坡=${RWS_RAMP_SECONDS}s | 最大关节速度=${RWS_MAX_JOINT_VELOCITY}rad/s | 回零保持=${RWS_HOLD_MS}ms"
echo "Rhs1 参数: 范围=[${RHS1_START_GROUP},${RHS1_END_GROUP}) | 斜坡=${RHS1_RAMP_SECONDS}s | 最大关节速度=${RHS1_MAX_JOINT_VELOCITY}rad/s | 回零保持=${RHS1_HOLD_MS}ms"
echo "Rhs2 参数: 范围=[${RHS2_START_GROUP},${RHS2_END_GROUP}) | 斜坡=${RHS2_RAMP_SECONDS}s | 最大关节速度=${RHS2_MAX_JOINT_VELOCITY}rad/s | 回零保持=${RHS2_HOLD_MS}ms"
echo "Lax+Rax 参数: 范围=[${AX_START_GROUP},${AX_END_GROUP}) | 斜坡=${AX_RAMP_SECONDS}s | 最大关节速度=${AX_MAX_JOINT_VELOCITY}rad/s | 动作末姿态保持=${AX_ACTION_HOLD_MS}ms | 回零保持=${AX_HOLD_MS}ms"
echo ""

# 启动一个长期运行的监听器，并在 candump 层过滤 CAN ID 0x051。
# exec 使 candump_pid 就是 candump 本身，cleanup 可以精确停止该进程。
check_prerequisites
coproc CANDUMP { exec candump "$CAN_INTERFACE,051:7FF"; }
candump_pid=$CANDUMP_PID
candump_fd=${CANDUMP[0]}

# 主循环：动作逻辑只依赖第 8 字节，因此仅在该字节变化时解析。
while IFS= read -r line <&"$candump_fd"; do
    read -r -a frame_fields <<< "$line"

    # 默认 candump 输出：can2 051 [8] byte1 ... byte8
    if [ "${#frame_fields[@]}" -lt 11 ] || [ "${frame_fields[2]}" != "[8]" ]; then
        echo "[WARN] 收到格式异常或非 8 字节的 0x051 帧: $line"
        continue
    fi

    can_id=${frame_fields[1]}
    byte8=${frame_fields[10]}

    # 第 8 字节为十六进制文本（例如 42），转换后逐 bit 解析。
    if ! [[ "$byte8" =~ ^[[:xdigit:]]{1,2}$ ]]; then
        echo "[WARN] 第8字节不是有效十六进制数据: $byte8"
        continue
    fi

    # 前 7 字节（例如 z/y/z 数据）变化不参与动作判断，避免无效重复解析。
    if [ -n "$last_byte8" ] && [ "${byte8^^}" = "$last_byte8" ]; then
        continue
    fi
    last_byte8=${byte8^^}

    byte8_num=$((16#$byte8))
    sa_bit=$(( (byte8_num >> 1) & 1 ))
    sd_bit=$(( (byte8_num >> 6) & 1 ))
    sb_bit2=$(( (byte8_num >> 2) & 1 ))
    sb_bit3=$(( (byte8_num >> 3) & 1 ))
    sc_bit4=$(( (byte8_num >> 4) & 1 ))
    sc_bit5=$(( (byte8_num >> 5) & 1 ))

    suite=$(decode_three_position "$sb_bit2" "$sb_bit3")
    action=$(decode_three_position "$sc_bit4" "$sc_bit5")

    # 获取当前准备触发的动作名称
    action_name=$(get_action_name "$suite" "$action")

    # 如果动作名称变了，打印准备触发的动作
    if [ "$action_name" != "$current_action_name" ]; then
        current_action_name="$action_name"
        echo "[INFO] 当前选择: $action_name (SB=${sb_bit2}${sb_bit3}, SC=${sc_bit4}${sc_bit5})"
    fi

    # 第一帧只记录按键状态，避免把初始 0 误判为“松开”。
    if [ -z "$last_sa_bit" ] || [ -z "$last_sd_bit" ]; then
        last_sa_bit=$sa_bit
        last_sd_bit=$sd_bit
        echo "[INFO] 已初始化 SA=$sa_bit, SD=$sd_bit；等待按键松开事件"
        continue
    fi

    sa_released=0
    sd_released=0
    if [ "$last_sa_bit" -eq 1 ] && [ "$sa_bit" -eq 0 ]; then sa_released=1; fi
    if [ "$last_sd_bit" -eq 1 ] && [ "$sd_bit" -eq 0 ]; then sd_released=1; fi
    last_sa_bit=$sa_bit
    last_sd_bit=$sd_bit

    # SD 松开优先：同一帧 SA/SD 都松开时只重置，不执行动作。
    if [ "$sd_released" -eq 1 ]; then
        need_reset=0
        last_action_type=""
        echo "[INFO] 收到 SD 松开：已重置，可以执行动作"
    elif [ "$sa_released" -eq 1 ]; then
        # SA 松开 = 执行信号；未重置时禁止执行。
        if [ "$need_reset" -eq 1 ]; then
            echo "[WARN] 需要先松开 SD 重置，SA 本次执行已忽略"
            continue
        fi

        # 检查冷却时间
        if ! check_cooldown; then
            echo "[WARN] 冷却中，忽略触发"
            continue
        fi

        # SB/SC 的三档状态决定动作；第三档和 11 不执行动作，但会消费本次触发。
        if [ "$suite" -eq -1 ] || [ "$action" -eq -1 ]; then
            echo "[WARN] SB/SC 存在非法档位编码 11，忽略执行"
            consume_trigger_without_action
        elif [ "$suite" -eq 0 ]; then  # SB 一档
            if [ "$action" -eq 0 ]; then  # SC 一档（bit4/bit5 = 00）
                current_action="action1"
                if [ "$current_action" != "$last_action_type" ]; then
                    last_action_type="$current_action"
                    execute_command "套数0-动作0: Rws.log" "$can_id" "$LOG_FILE_DIR/Rws.log" "$RIGHT_ARM_CAN" "" "" "$RWS_START_GROUP" "$RWS_END_GROUP" "$RWS_RAMP_SECONDS" "$RWS_MAX_JOINT_VELOCITY" "$RWS_HOLD_MS"
                fi
            elif [ "$action" -eq 1 ]; then  # SC 二档（bit4/bit5 = 10）
                current_action="action2"
                if [ "$current_action" != "$last_action_type" ]; then
                    last_action_type="$current_action"
                    execute_command "套数0-动作1: Rhs1.log" "$can_id" "$LOG_FILE_DIR/Rhs1.log" "$RIGHT_ARM_CAN" "" "" "$RHS1_START_GROUP" "$RHS1_END_GROUP" "$RHS1_RAMP_SECONDS" "$RHS1_MAX_JOINT_VELOCITY" "$RHS1_HOLD_MS"
                fi
            else
                echo "[INFO] SC 三档为预留，暂无动作"
                consume_trigger_without_action
            fi
        elif [ "$suite" -eq 1 ]; then  # SB 二档
            if [ "$action" -eq 0 ]; then  # SC 一档（bit4/bit5 = 00）
                current_action="action3"
                if [ "$current_action" != "$last_action_type" ]; then
                    last_action_type="$current_action"
                    execute_command "套数1-动作0: Rhs2.log" "$can_id" "$LOG_FILE_DIR/Rhs2.log" "$RIGHT_ARM_CAN" "" "" "$RHS2_START_GROUP" "$RHS2_END_GROUP" "$RHS2_RAMP_SECONDS" "$RHS2_MAX_JOINT_VELOCITY" "$RHS2_HOLD_MS"
                fi
            elif [ "$action" -eq 1 ]; then  # SC 二档（bit4/bit5 = 10）
                current_action="action4"
                if [ "$current_action" != "$last_action_type" ]; then
                    last_action_type="$current_action"
                    execute_command "套数1-动作1: Lax.log+Rax.log" "$can_id" "$LOG_FILE_DIR/Lax.log" "$LEFT_ARM_CAN" "$LOG_FILE_DIR/Rax.log" "$RIGHT_ARM_CAN" "$AX_START_GROUP" "$AX_END_GROUP" "$AX_RAMP_SECONDS" "$AX_MAX_JOINT_VELOCITY" "$AX_HOLD_MS" "$AX_ACTION_HOLD_MS"
                fi
            else
                echo "[INFO] SC 三档为预留，暂无动作"
                consume_trigger_without_action
            fi
        elif [ "$suite" -eq 2 ]; then  # SB 三档（预留）
            echo "[INFO] SB 三档为预留，暂无动作"
            consume_trigger_without_action
        else
            echo "[WARN] 未知 SB 档位: $suite"
            consume_trigger_without_action
        fi
    fi
done

# candump 非预期退出时以失败状态退出，使 systemd 按 Restart=on-failure 重启服务。
wait "$candump_pid"
candump_status=$?
candump_pid=""
echo "[ERROR] candump 已退出 (退出码: $candump_status)，监听器停止" >&2
exit "$candump_status"
