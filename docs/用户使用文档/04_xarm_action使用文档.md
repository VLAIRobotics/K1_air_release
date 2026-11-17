# 机械臂动作说明

## 动作列表

| 动作名称 | 动作描述 | CAN接口 |
|---------|---------|--------|
| Lax | 左臂比爱心 | can0 (左臂) |
| Rax | 右臂比爱心 | can1 (右臂) |
| Rhs1 | 右臂小挥手 | can1 (右臂) |
| Rhs2 | 右臂大挥手 | can1 (右臂) |
| Rws | 右臂握手 | can1 (右臂) |

> **注意**：使用时请确保动作与CAN接口正确对应（L=左臂使用can0，R=右臂使用can1）

---

## 脚本功能说明

### 1. action_interactive.sh

**功能**：动作录制与数据转换工具

**使用流程**：
1. 启动脚本后，先配置CAN接口
2. 选择"启动CAN数据帧监听"开始录制（选择监听从臂的数据）
3. 执行摇操作机械臂进行动作录制
4. 停止监听后，选择"转换日志为可发送格式"
5. 生成的转换文件可用于 `send_data` 发送

**主要功能**：
- CAN接口配置
- 数据帧监听（仅监听00x ID，写入一轮跳过九轮）
- 日志格式转换（全部转换，保留开头和结尾特定帧）

---

### 2. send_data

**功能**：动作数据发送工具，支持手动交互回放和供 CAN 监听器调用的命令行自动回放。

**编译指令**（以下命令均在 `x_air_sdk` 仓库根目录执行）：

```bash
# x86_64 平台编译
cd publish/modules/src/xarm_action/action
gcc -o send_data send_data.c -lpthread

# aarch64 平台编译（需安装 aarch64 交叉编译工具链）
aarch64-linux-gnu-gcc -o send_data_aarch64 send_data.c -lpthread
```

**运行前准备**：
1. 确保机械臂处于**垂直零位**（初始位置）。
2. 确认 CAN 已完成配置；左臂使用 `can0`，右臂使用 `can1`。
3. 命令行自动模式可先在仓库根目录运行：`sudo bash publish/xarm_can/package/libexec/setup_can_interfaces.sh`。

**手动交互模式**：
```bash
cd publish/modules/src/xarm_action/action
./send_data
```
程序显示菜单后，选择单文件或双文件发送，再输入日志文件路径和目标 CAN 接口。
程序会从自身所在的仓库模块目录定位 CAN 配置脚本；若仅复制 `send_data` 可执行文件而未保留仓库目录结构，请先按“运行前准备”手动配置 CAN。

**命令行自动模式**：
```bash
# 单臂：右臂握手
./publish/modules/src/xarm_action/action/send_data -f publish/modules/src/xarm_action/action/1/Rws.log -c can1

# 双臂：双臂爱心
./publish/modules/src/xarm_action/action/send_data -f publish/modules/src/xarm_action/action/1/Lax1.log -c can0 -f2 publish/modules/src/xarm_action/action/1/Rax1.log -c2 can1
```

`can_listener_trigger.sh` 使用上述参数接口自动回放动作；监听器调用时不应向手动菜单输入内容。
单臂自动模式中 `-c` 可省略并默认使用 `can0`；双臂自动模式必须同时提供 `-f/-f2/-c2`，避免两个动作误发到同一 CAN 接口。参数错误或动作发送失败时，`send_data` 会返回非零退出码，监听器据此重新上锁。

**发送策略**：
- 固定频率：400Hz（间隔2.5ms）。

---

### 3. smooth_action.sh

**功能**：动作零位帧插入与平滑插值工具

**使用流程**：

2. 修改脚本中的零位定义（如需调整）
3. 运行脚本：
   ```bash
   cd /home/vlai/x_air/publish/modules/src/xarm_action
   python3 smooth_action.sh
   ```

**处理逻辑**：
- 在动作数据帧前后插入零位帧和使能/失能帧
- 在零位帧与动作帧之间添加线性平滑插值（每帧变化1）
- 所有8个关节同时插值，保证动作同步性

**采样策略**：
- **开头插值**（零位→动作第一帧）：1/10采样（保留1组，删除9组）
- **结尾插值**（动作最后一帧→零位）：1/3采样（保留1组，删除2组）

**零位定义**：
- **左臂零位**（L开头文件）：001#7FFF7FE7AD9997FF, 002#7FED7FD7AD9997FF, ...
- **右臂零位**（R开头文件）：001#7FFE7FE7AD9997FF, 002#7FF57FE7AD9997FF, ...

**处理后文件结构**：
| 位置 | 内容 | 行数 |
|------|------|------|
| 开头 | 使能帧 | 8行 |
| 开头 | 零位帧 | 8行 |
| 开头 | 平滑插值帧 | 可变 |
| 中间 | 原始动作帧 | 原始数量 |
| 结尾 | 平滑插值帧 | 可变 |
| 结尾 | 零位帧 | 8行 |
| 结尾 | 失能帧 | 8行 |

**注意**：脚本会直接覆盖 `1` 目录中的文件。如需重新处理，请先从 `1 copy` 目录恢复原始文件：
```bash
cp /home/vlai/x_air/publish/modules/src/xarm_action/1\ copy/*.log /home/vlai/x_air/publish/modules/src/xarm_action/1/
```

**自定义零位录制**：

若要自定义启动和结束的零位，严格按照以下步骤：

1. 运行 `/home/vlai/x_air/publish/modules/src/xarm_action/action_interactive.sh` 脚本，启动监听
2. 启动摇操作，**左臂（主臂）控制右臂（从臂）**，保留右臂（从臂）的数据帧，转换成发送格式
3. 启动摇操作，**右臂（主臂）控制左臂（从臂）**，保留左臂（从臂）的数据帧，转换成发送格式

> **关键注意事项**：
> - 启动摇操作时双臂应该是**垂直零位**（自然下垂），而不是抬起状态
> - 启动摇操作后**不要动**，等待5秒后停止监听
> - 转换发送格式后，取最新的一组数据帧（001-008）作为零位数据
> - 将左右臂对应的数据帧替换至 `smooth_action.sh` 脚本中的零位定义
> - **安全警告**：由于单边摇操作主从臂的重力补偿有差别，必须严格区分CAN口和对应的主从臂，**只取从臂的数据帧**。若不小心录制了主臂的数据帧并加入动作，会导致机械臂大幅度突变，操作时请保持安全距离

---

### 4. mirror_arm.py

**功能**：单臂动作镜像工具，通过录制单条从臂的动作数据帧，自动镜像生成另一条臂的对称动作数据帧

**使用流程**：

1. 运行 `/home/vlai/x_air/publish/modules/src/xarm_action/action_interactive.sh` 脚本，启动监听
2. 启动摇操作，**左臂（主臂）控制右臂（从臂）**，录制右臂（从臂）的动作数据帧，转换成发送格式
3. 使用 `mirror_arm.py` 将右臂数据帧镜像生成左臂对称动作

**命令格式**：
```bash
cd /home/vlai/x_air/publish/modules/src/xarm_action/1
python3 mirror_arm.py <输入文件> <输出文件> [--compare <对比文件>]
```

**示例**：
```bash
# 将右臂动作 Rax1.log 镜像为左臂动作 Lax1_mirrored.log
python3 mirror_arm.py Rax1.log Lax1_mirrored.log

# 镜像后与原始左臂数据对比（验证用）
python3 mirror_arm.py Rax1.log Lax1_mirrored.log --compare Lax1.log
```

**关节镜像规则**：
| 关节 | 对称类型 | 镜像规则 |
|------|---------|---------|
| 001 | MIRROR | 位置值取反（0x10000 - value） |
| 002 | MIRROR | 位置值取反（0x10000 - value） |
| 003 | STRONG_MIRROR | 位置值取反（0x10000 - value） |
| 004 | DIRECT | 位置值不变 |
| 005 | MIRROR | 位置值取反（0x10000 - value） |
| 006 | MIRROR | 位置值取反（0x10000 - value） |
| 007 | MIRROR | 位置值取反（0x10000 - value） |
| 008 | STRONG_MIRROR | 位置值取反（0x10000 - value） |

> **关键注意事项**：
> - 必须录制**从臂**的动作数据帧，不能使用主臂数据
> - 镜像前确保原始数据已通过 `smooth_action.sh` 处理（插入零位帧和平滑插值）
> - 镜像后输出文件可直接用于 `smooth_action.sh` 二次处理或 `send_data` 发送
> - 使能帧（`FFFFFFFFFFFFFFFC`）和失能帧（`FFFFFFFFFFFFFFFD`）不会被镜像，保持原始值
> - 若要录制左臂动作镜像右臂，操作步骤相同，只是主从臂互换

---

## 注意事项

1. **初始位置**：发送动作前务必确保机械臂处于垂直零位
2. **CAN接口**：左臂使用 can0，右臂使用 can1，请勿混淆
3. **安全提示**：运行动作前请确保工作区域安全，避免碰撞
4. **录制提示**：录制时请选择监听从臂（跟随臂）的数据
5. **文件格式**：发送前需先使用 `action_interactive.sh` 转换日志格式
