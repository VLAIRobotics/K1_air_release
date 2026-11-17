#!/bin/bash
#
# CAN 监听触发器脚本 V1.2.2（新遥控器位协议，需重置版本）
# 功能：监听 can2 上的 0x051 帧，根据第 8 数据字节触发预录动作。
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
# 正式 x_air_sdk 的动作包位于 xarm_action/action/ 下。
SEND_DATA_TOOL="$SCRIPT_DIR/../../../xarm_action/action/send_data"
LOG_FILE_DIR="$SCRIPT_DIR/../../../xarm_action/action/1"

# 配置参数
CAN_INTERFACE="can2"
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

    [ -x "$SEND_DATA_TOOL" ] || die "动作发送工具不存在或不可执行: $SEND_DATA_TOOL"

    local required_log
    for required_log in Rws.log Rhs1.log Rhs2.log Lax1.log Rax1.log; do
        [ -f "$LOG_FILE_DIR/$required_log" ] || die "动作日志不存在: $LOG_FILE_DIR/$required_log"
    done

    ip link show dev "$CAN_INTERFACE" >/dev/null 2>&1 \
        || die "CAN 接口不存在: $CAN_INTERFACE"

    local link_info
    link_info=$(ip -o link show dev "$CAN_INTERFACE")
    [[ "$link_info" == *"<"*"UP"* ]] \
        || die "CAN 接口未启用: $CAN_INTERFACE；请先运行 setup_can_interfaces.sh"
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

    echo "[INFO] 执行: ${action}"
    is_running=1

    if [ -n "$log_file2" ] && [ -n "$can_if2" ]; then
        "$SEND_DATA_TOOL" -f "$log_file" -c "$can_if" -f2 "$log_file2" -c2 "$can_if2" &
    else
        "$SEND_DATA_TOOL" -f "$log_file" -c "$can_if" &
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
            echo "套数1-动作1: Lax1.log+Rax1.log"
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
echo "  CAN 监听触发器（新遥控器位协议，需重置）"
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
echo "  套数1 (中): 动作0→Rhs2.log | 动作1→Lax1.log+Rax1.log"
echo "  套数2 (下): 预留"
echo ""
echo "规则: 必须先松开 SD 重置，才能松开 SA 执行动作"
echo "      SA 触发预留或非法档位时，也必须重新松开 SD 才能再次触发"
echo "========================================"
echo "按 Ctrl+C 退出"
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
                    execute_command "套数0-动作0: Rws.log" "$can_id" "$LOG_FILE_DIR/Rws.log" "can1"
                fi
            elif [ "$action" -eq 1 ]; then  # SC 二档（bit4/bit5 = 10）
                current_action="action2"
                if [ "$current_action" != "$last_action_type" ]; then
                    last_action_type="$current_action"
                    execute_command "套数0-动作1: Rhs1.log" "$can_id" "$LOG_FILE_DIR/Rhs1.log" "can1"
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
                    execute_command "套数1-动作0: Rhs2.log" "$can_id" "$LOG_FILE_DIR/Rhs2.log" "can1"
                fi
            elif [ "$action" -eq 1 ]; then  # SC 二档（bit4/bit5 = 10）
                current_action="action4"
                if [ "$current_action" != "$last_action_type" ]; then
                    last_action_type="$current_action"
                    execute_command "套数1-动作1: Lax1.log+Rax1.log" "$can_id" "$LOG_FILE_DIR/Lax1.log" "can0" "$LOG_FILE_DIR/Rax1.log" "can1"
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
