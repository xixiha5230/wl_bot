# Micro Wheeled-leg Robot — ESP-IDF Firmware

用 ESP-IDF 原生复刻 MuShibo/Micro-Wheeled_leg-Robot 的固件，目标芯片为上游项目使用的
经典 ESP32（ESP32-D0WDQ6，4 MB Flash）。参考实现为 Arduino 版 `wl_pro_robot.ino`。

## 硬件与引脚

| 功能 | 引脚 |
| --- | --- |
| 左编码器 AS5600 (I2C0) | SDA GPIO19 / SCL GPIO18 |
| 右编码器 AS5600 + MPU6050 (I2C1) | SDA GPIO23 / SCL GPIO5 |
| 左电机 L6234 3PWM + EN | U/V/W = 32/33/25，EN = 22 |
| 右电机 L6234 3PWM + EN | U/V/W = 26/27/14，EN = 12 |
| 腿部 STS3032 舵机 UART2 | TX GPIO17 / RX GPIO16 @ 1 Mbps |
| 电池分压采样 | GPIO35（ADC1_CH7，分压比 3.97） |
| 电量 LED | GPIO13 |

Wi-Fi 采用 **APSTA**：

- **STA**：连接家庭路由器（凭据由构建时的 `.env` / 环境变量提供，见下），通过
  DHCP 获取内网 IP；主机名由 mDNS 广播，可用 `http://wlrobot.local/` 访问。
- **AP 兜底**：始终开启热点 `WLROBOT` / `12345678`，地址 `192.168.4.1`
  （特意避开常见的 `192.168.1.x` 家庭网段）。

Wi-Fi 凭据（不进仓库）。推荐在仓库根目录放一个 **`.env`**（已 gitignore）：

```bash
cp .env.example .env
# 编辑 .env，填入你的 SSID / 密码
WLROBOT_WIFI_SSID=你的SSID
WLROBOT_WIFI_PASSWORD=你的密码
```

构建时 `main/CMakeLists.txt` 会自动读取 `.env`。也可以直接用环境变量（**优先于
`.env`**）：

```bash
WLROBOT_WIFI_SSID=你的SSID WLROBOT_WIFI_PASSWORD=你的密码 tools/ota.sh
```

都没提供时使用 `main/wifi_net.c` 里的占位默认值（`wlrobot-setup`，空密码，
连不上真实网络，只会保留 AP 兜底）。AP 也支持 `WLROBOT_AP_SSID` /
`WLROBOT_AP_PASSWORD` 覆盖。

固件只暴露**数据面**：WebSocket 在 `:81/`，另有 `/api/status`、`/api/set`、`POST /api/ota`。
控制界面在**主机侧**运行（`tools/web`）——机器人不托管网页，静态资源和渲染压力都在电脑上。

## 架构

| 任务 | 优先级 | 核心 | 周期 | 职责 |
| --- | --- | --- | --- | --- |
| `control_task` | 7 | 1 | ~0.5–1 kHz | 读 IMU、LQR 平衡 / YAW / 腿部控制、`loopFOC` + `move`、故障判定 |
| `leg_task` | 4 | 1 | 100 Hz | 从邮箱取最新腿部姿态并写 STS 舵机 |
| `servo_task` | 3 | 1 | 2 s | 舵机反馈诊断 |
| `battery_task` | 2 | 1 | 1 s | 电压采样 + LED |

- 所有应用任务绑定 core 1，core 0 留给 Wi-Fi / 系统栈。
- 跨任务共享状态（`robot_command_t`、FOC 模式/目标、`robot_state`）统一用 `portMUX` 临界区保护。
- 控制环在同一循环里完成 `loopFOC()+move()`（与原版单循环一致），只把腿部姿态写入长度 1 的邮箱，
  UART 与互斥量不会阻塞平衡环。

## 功能

- **传感器**：双 AS5600 绝对角度 + 连续角度/速度；MPU6050 陀螺零偏校准 + 互补滤波姿态。
- **电机 FOC**：基于官方组件 `espressif/esp_simplefoc`（`arduino-foc` 2.4 + `iqmath`），
  LEDC 3PWM、`BLDCMotor(7)`、电压力矩模式；参数与原版一致
  （`voltage_power_supply=8`、`voltage_sensor_align=6`、速度环 `P=0.05/I=1`）。
- **腿部舵机**：STS3032 半双工同步写，行程标定 + 安全限幅。
- **控制环**：LQR 平衡、YAW 转向、腿部高度 + roll 补偿、跳跃、失控保护。
- **运行时调参**：`pid` / `lpf` / `zero` / `yaw`，边跑边调，立即生效。
- **电源**：电压采样（EWMA 滤波）+ LED 迟滞指示 + 低压保护（去抖）。
- **网络**：WebSocket 遥控（端口 81）+ JSON API（`/api/status`、`/api/set`、`POST /api/ota`）；
  控制 UI 在主机侧（`tools/web`），机器人只做数据面。
- **OTA**：双 OTA 分区，`POST /api/ota`（body 为固件 URL）或串口 `ota <url>`，成功自动重启。
- **串口控制台**：UART0，提示符 `wlrobot>`。

## 状态与安全保护

状态机：`safe → calibrating → ready ⇄ running`，异常进入 `fault`；
`rc` 与 `/api/status` 会显示真实状态。

以下任一条件触发即 **断电并进入 `fault`**：

- 机身倾角 `|angle| > 25°`；
- 电池低于 `6.8 V`（回充到 `7.0 V` 以上才允许恢复）。

故障后需先解除原因，再**显式 `go 1`** 且保持直立 200 ms 才会重新使能。
电机对齐失败会自动断电；标定命令要求先 `go 0`。

## 构建与烧录

```bash
source "$HOME/.espressif/tools/activate_idf_v6.1.sh"
idf.py build
idf.py -p /dev/cu.usbserial-XXXX flash monitor
```

首次构建会自动拉取 `main/idf_component.yml` 声明的外部组件。

### 主机侧工具与 OTA

```bash
tools/serve_web.sh                 # 本地托管控制界面，浏览器打开提示的地址
tools/ota.sh                       # 构建 → 本机起临时 HTTP → 触发机器人 OTA 并等待重启
ROBOT=192.168.1.195 tools/ota.sh   # 指定机器人地址
```

界面连接 `ws://<host>:81/` 做实时遥控，用 `/api/set` 调参、`POST /api/ota` 升级，均带 CORS。

## 串口命令

```text
read                       读取两个舵机反馈
move <id> <pos> [speed]    移动单个舵机
sync <p1> <p2> [speed]     同步移动两个舵机
limits                     查看标定范围
cal <id|all>               自动行程标定（需先 go 0）
torque <id|all> <0|1>      舵机力矩开关
stop                       关闭舵机力矩

m_init / m_align / m_align1 <id> [v]   电机初始化 / 对齐
m_pp <1|2> <pp>            设置极对数
m_mode <disabled|torque|velocity|angle|vopen|aopen>
m_target <left> <right>    直接设定目标
m_state / m_stop           电机状态 / 停止

go <0|1>                   使能平衡输出
height <32..80>            腿部高度（内部有斜坡，避免跳变冲击）
joy <x> <y>                虚拟摇杆
dir <0..5>                 运动方向（5=jump, 4=stop）
rc                         控制状态（state/fault/terms/leg_add/zero）
enc / imu                  读取编码器 / 读取 accel,gyro,angle
rate                       测量 control/FOC 循环频率
jit [sec]                  测量 lqr_angle/roll/leg_add 峰峰值（高频抖动）

pid                        列出全部 PID
pid <name> <P> <I> [D] [limit]   改 PID（立即生效）
lpf <name> <Tf>            改低通（joyy/zeropoint/roll）
zero [deg]                 查看/设置平衡零点（基准值）
yaw <1|-1|0>                YAW 正常 / 反向 / 关闭
bat                        电池电压（含原始 ADC 值）
ota <url>                  通过 Wi-Fi 拉取固件升级，成功后自动重启
```

## 平衡零点与调参

- 本台实测机械平衡角**随腿高变化**：h32≈4.0°，h52≈3.2°，h80≈0.4°，约 **-0.075°/单位**。
- 实现：`angle_zeropoint`（基准，h38 处默认 **4.4°**）+ 高度前馈；`rc` 的 `terms ... zero=`
  显示当前生效零点。慢速自适应 `pid zeropoint` **默认关闭**（P=0）：实测它会在噪声下低频
  游走，而固定基准 + 前馈已足够准；如确需跟踪，`pid zeropoint <P>` 可开启，其调整量被硬限制
  在基准 ±`LEG_BALANCE_ZERO_ADAPT`（1°）内，绝不会因被按住/推动而跑飞。
- 控制环路：`angle`（刚度）→ `gyro`（阻尼）→ `distance` / `speed`（前后）→ `yaw_angle` /
  `yaw_gyro`（转向）；本台实测较优默认 `angle=1.1 distance=0.2 speed=0.4`，站立残摆约 0.5°。
  实测把 `distance`（位移环）增益调小可显著减小 1~2 Hz 前后极限环。
- 高度指令经软件斜坡（`LEG_HEIGHT_SLEW`）输出，切换高度时不会瞬间冲击机身。
- 调参顺序：先 `zero` 找准站立点，再 `pid angle` / `pid gyro`，最后 `pid distance` / `pid speed`。
- 脱线运行时可通过 HTTP 调参（无需 USB）：
  `http://192.168.1.195/api/set?zero=4.4`、`/api/set?pid=angle&p=1.1&i=0`、
  `/api/set?lpf=roll&tf=0.6`、`/api/set?yaw=1`；当前状态见 `/api/status`。

## 标定数据（`main/robot_config.h`）

```text
ID1 机械行程 2030..2575，ID2 1512..2077
安全区(±40)：ID1 2070..2535，ID2 1552..2037
镜像轴 4096，中性位 ID1=2302 ID2=1794
```

## 关键移植决策

- **FOC 并入控制任务**：原版是单循环里 `loopFOC()+move()`。曾拆成独立 `foc_task`，结果两个 1 kHz
  任务在同核节拍错位 + I2C 争用，FOC 实际掉到 ~300 Hz、力矩更新迟滞导致站不住；合并回单循环后恢复。
- **`move()` 在 `loopFOC()` 之前**：arduino-foc 的 `loopFOC()` 施加的是上一次 `move()` 设的目标，
  按 `loopFOC()+move()` 顺序会晚一拍（约 2 ms）施力，减少相位裕度。改为先 `move()` 再 `loopFOC()`
  让新力矩在同一拍生效。
- **LEDC 而非 MCPWM**：组件默认 MCPWM，但右电机（MCPWM 分组 1）带轮胎时对齐反复失败；
  原版 Arduino SimpleFOC 使用 LEDC，改回后带载对齐正常。
- **自定义 `BoardEncoder : Sensor`**：复用现有 `i2c_master` AS5600，避免与组件的 `i2c_bus` 争用端口。
- **右编码器与 MPU6050 共用 I2C1**：运行时 I2C 超时设为 10 ms，稳态下无读失败。
- **网页摇杆字符串**：原版网页用 `Number.toFixed()` 发送摇杆值（字符串），原版 ArduinoJson 会自动
  转数值；cJSON 不会。网页改为数值发送，固件 `json_to_int()` 同时兼容字符串。
- **UI 外置 + OTA**：固件不再内嵌网页（原版 `basic_web` 约 22 KB），只提供 WS + JSON API；
  控制界面在主机侧（`tools/web`），固件升级走 `POST /api/ota`（双 OTA 分区）。
- **启动流程**：传感器初始化（失败重试）→ FOC 初始化 → 自动对齐（失败重试 3 次）→ 启动控制。
  对齐用互斥量串行化 `loopFOC`，避免相电压被覆盖；对齐失败自动断电。

## 备注

- FreeRTOS 必须 `CONFIG_FREERTOS_HZ=1000`。
- 分区表为自定义双 OTA（`partitions.csv`）：两个 1.5 MB app 槽，固件约 1.29 MB（余约 18%）。
- 依赖 `espressif/esp_simplefoc`、`espressif/cjson`（见 `main/idf_component.yml` 与 `dependencies.lock`）。
- 启动时 `i2c.common: GPIO 23/5 not usable` 与 `ledc: GPIO xx not usable` 为组件保留检查的
  良性告警，不影响功能。
- 家庭 Wi-Fi 凭据不进仓库：构建时用 `WLROBOT_WIFI_SSID` /
  `WLROBOT_WIFI_PASSWORD`（和 `WLROBOT_AP_*`）环境变量注入，见上文。
