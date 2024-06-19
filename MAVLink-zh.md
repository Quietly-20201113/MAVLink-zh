**MAVLink 中文版**  
**AI+手工+机翻的勉强看吧,有问题联系我更新**
### MAVLink 2 数据包格式

<img src="https://mavlink.io/assets/packets/packet_mavlink_v2.jpg" alt="MAVLink v2 packet" style="zoom:150%;" />

| Byte Index                                                   | C version                  | Content                                                      | Value        |                         Explanation                          |
| ------------------------------------------------------------ | -------------------------- | ------------------------------------------------------------ | ------------ | :----------------------------------------------------------: |
| 0                                                            | `uint8_t magic`            | 数据包起始标记                                               | 0xFD         | 协议特定的文本开始 (STX) 标记，用于指示新数据包的开始。任何不理解协议版本的系统都会跳过该数据包。 |
| 1                                                            | `uint8_t len`              | Payload length                                               | 0 - 255      | 指示以下 `payload` 部分的长度。这可能会受到 [payload truncation](https://mavlink.io/en/guide/serialization.html#payload_truncation). |
| 2                                                            | `uint8_t incompat_flags`   | [Incompatibility Flags](https://mavlink.io/en/guide/serialization.html#incompat_flags) |              | 为了实现 MAVLink 兼容性必须理解的标志（如果实现不理解标志，则会丢弃数据包）。 |
| 3                                                            | `uint8_t compat_flags`     | [Compatibility Flags](https://mavlink.io/en/guide/serialization.html#compat_flags) | 0x01         | 为了实现 MAVLink 兼容性必须理解的标志（如果实现不理解标志，则会丢弃数据包）。 |
| 4                                                            | `uint8_t seq`              | Packet sequence number                                       | 0 - 255      |       用于检测数据包丢失。组件为发送的每条消息增加值。       |
| 5                                                            | `uint8_t sysid`            | System ID (sender)                                           | 1 - 255      |   发送消息的 *system* (飞机) 的 ID。 用于区分网络上的系统    |
| 6                                                            | `uint8_t compid`           | Component ID (sender)                                        | 1 - 255      | 发送消息的*组件*的 ID。用于区分*系统*中的*组件*（例如自动驾驶仪和摄像头）。在 [MAV_COMPONENT](https://mavlink.io/en/messages/common.html#MAV_COMPONENT) 中使用适当的值。请注意，广播地址“MAV_COMP_ID_ALL”不能在此字段中使用，因为它是无效的*源*地址。 |
| 7 to 9                                                       | `uint32_t msgid:24`        | Message ID (low, middle, high bytes)                         | 0 - 16777215 |  有效负载中的 *消息类型* 的 ID。用于将数据解码回消息对象。   |
| For *n*-byte payload: `n=0`: NA, `n=1`: 10, `n>=2`: 10 to (9+n) | `uint8_t payload[max 255]` | [Payload](https://mavlink.io/en/guide/serialization.html#payload) |              |        消息数据。取决于消息类型（即消息 ID）和内容。         |
| (n+10) to (n+11)                                             | `uint16_t checksum`        | [Checksum](https://mavlink.io/en/guide/serialization.html#checksum) (low byte, high byte) |              | 消息的 CRC-16/MCRF4XX（不包括“magic”字节）。包括 [CRC_EXTRA](https://mavlink.io/en/guide/serialization.html#crc_extra) 字节。 |
| (n+12) to (n+25)                                             | `uint8_t signature[13]`    | [Signature](https://mavlink.io/en/guide/message_signing.html) |              |               （可选）签名以确保链接不可篡改。               |



****

<!-- more -->

### CRC
#### Python

```python
def message_checksum(msg):
    '''calculate a 8-bit checksum of the key fields of a message, so we
       can detect incompatible XML changes'''
    from .mavcrc import x25crc
    crc = x25crc()
    crc.accumulate_str(msg.name + ' ')
    # in order to allow for extensions the crc does not include
    # any field extensions
    crc_end = msg.base_fields()
    for i in range(crc_end):
        f = msg.ordered_fields[i]
        crc.accumulate_str(f.type + ' ')
        crc.accumulate_str(f.name + ' ')
        if f.array_length:
            crc.accumulate([f.array_length])
    return (crc.crc&0xFF) ^ (crc.crc>>8)
```

#### Rust-AI

```rust
use std::vec::Vec;

// Define a struct for Field
struct Field {
    type_: String,
    name: String,
    array_length: Option<u8>,
}

// Define a struct for Message
struct Message {
    name: String,
    ordered_fields: Vec<Field>,
}

impl Message {
    fn base_fields(&self) -> usize {
        self.ordered_fields.len()
    }
}

// Define a struct for CRC with relevant methods
struct X25Crc {
    crc: u16,
}

impl X25Crc {
    fn new() -> X25Crc {
        X25Crc { crc: 0xffff }
    }

    fn accumulate(&mut self, bytes: &[u8]) {
        for &byte in bytes {
            let tmp = byte ^ (self.crc as u8);
            let tmp = (tmp ^ (tmp << 4)) as u16;
            self.crc = (self.crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
        }
    }

    fn accumulate_str(&mut self, string: &str) {
        self.accumulate(string.as_bytes());
    }
}

fn message_checksum(msg: &Message) -> u8 {
    let mut crc = X25Crc::new();
    crc.accumulate_str(&(msg.name.clone() + " "));
    
    // in order to allow for extensions the crc does not include
    // any field extensions
    let crc_end = msg.base_fields();
    for i in 0..crc_end {
        let f = &msg.ordered_fields[i];
        crc.accumulate_str(&(f.type_.clone() + " "));
        crc.accumulate_str(&(f.name.clone() + " "));
        if let Some(length) = f.array_length {
            crc.accumulate(&[length]);
        }
    }
    
    (crc.crc & 0xFF) as u8 ^ (crc.crc >> 8) as u8
}

fn main() {
    // Example usage
    let fields = vec![
        Field { type_: "int".to_string(), name: "field1".to_string(), array_length: None },
        Field { type_: "float".to_string(), name: "field2".to_string(), array_length: Some(5) }
    ];
    let msg = Message { name: "example".to_string(), ordered_fields: fields };
    
    let checksum = message_checksum(&msg);
    println!("Checksum: {}", checksum);
}

```
****

## Messages

### HEARTBEAT (0)

### SYS_STATUS (1)

一般系统状态。如果系统遵循 MAVLink 标准，则系统状态主要由三种正交状态/模式定义： 系统模式为 LOCKED（电机关闭并锁定）、MANUAL（系统由遥控控制）、GUIDED（系统具有自主位置控制功能，位置设定点由手动控制）或 AUTO（系统由路径/航点规划器引导）。NAV_MODE](#NAV_MODE)定义了当前的飞行状态： LIFTOFF（通常为开环机动）、LANDING、WAYPOINTS 或 VECTOR。这表示内部导航状态机。系统状态显示系统当前是否处于激活状态，以及是否发生了紧急情况。在 "危急 "和 "紧急 "状态下，飞行器仍被视为处于活动状态，但应自主启动紧急程序。发生故障后，它应首先从活动状态转入危急状态，以便进行人工干预，然后在一定超时后转入紧急状态。

| Field Name                                                   | Type       | Units | Values                                                       | Description                                                  |
| ------------------------------------------------------------ | ---------- | ----- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| onboard_control_sensors_present                              | `uint32_t` |       | [MAV_SYS_STATUS_SENSOR](#MAV_SYS_STATUS_SENSOR)              | 显示存在哪些机载控制器和传感器的位图。值为 0：不存在。值为 1：存在。 |
| onboard_control_sensors_enabled                              | `uint32_t` |       | [MAV_SYS_STATUS_SENSOR](#MAV_SYS_STATUS_SENSOR)              | 显示启用了哪些板载控制器和传感器的位图：  值为 0：未启用。值为 1：已启用。 |
| onboard_control_sensors_health                               | `uint32_t` |       | [MAV_SYS_STATUS_SENSOR](#MAV_SYS_STATUS_SENSOR)              | 显示哪些机载控制器和传感器出错（或正常）的位图。值为 0：错误。值为 1：健康。 |
| load                                                         | `uint16_t` | d%    |                                                              | 最大使用时间，占主循环时间的百分比。值： [0-1000] - 应始终低于 1000 |
| voltage_battery                                              | `uint16_t` | mV    | invalid:UINT16_MAX                                           | 电池电压，UINT16_MAX：自动驾驶仪未发送的电压                 |
| current_battery                                              | `int16_t`  | cA    | invalid:-1                                                   | 电池电流，-1： 自动驾驶仪未发送电流                          |
| battery_remaining                                            | `int8_t`   | %     | invalid:-1                                                   | 电池剩余能量，-1：自动驾驶仪未发送电池剩余能量               |
| drop_rate_comm                                               | `uint16_t` | c%    |                                                              | 通信丢失率（UART、I2C、SPI、CAN），所有链路上丢失的数据包（MAV 接收时损坏的数据包） |
| errors_comm                                                  | `uint16_t` |       |                                                              | 通信错误（UART、I2C、SPI、CAN），所有链路上的数据包丢失（MAV 接收时损坏的数据包） |
| errors_count1                                                | `uint16_t` |       |                                                              | 自动驾驶仪特有错误                                           |
| errors_count2                                                | `uint16_t` |       |                                                              | 自动驾驶仪特有错误                                           |
| errors_count3                                                | `uint16_t` |       |                                                              | 自动驾驶仪特有错误                                           |
| errors_count4                                                | `uint16_t` |       |                                                              | 自动驾驶仪特有错误                                           |
| <span class='ext'>onboard_control_sensors_present_extended</span> <a href='#mav2_extension_field'>++</a> | `uint32_t` |       | [MAV_SYS_STATUS_SENSOR_EXTENDED](#MAV_SYS_STATUS_SENSOR_EXTENDED) | 显示存在哪些机载控制器和传感器的位图。值为 0：不存在。值为 1：存在。 |
| <span class='ext'>onboard_control_sensors_enabled_extended</span> <a href='#mav2_extension_field'>++</a> | `uint32_t` |       | [MAV_SYS_STATUS_SENSOR_EXTENDED](#MAV_SYS_STATUS_SENSOR_EXTENDED) | 显示启用了哪些板载控制器和传感器的位图：  值为 0：未启用。值为 1：已启用。 |
| <span class='ext'>onboard_control_sensors_health_extended</span> <a href='#mav2_extension_field'>++</a> | `uint32_t` |       | [MAV_SYS_STATUS_SENSOR_EXTENDED](#MAV_SYS_STATUS_SENSOR_EXTENDED) | 显示哪些机载控制器和传感器出错（或正常）的位图。值为 0：错误。值为 1：健康。 |

### SYSTEM_TIME (2)

系统时间是主时钟的时间，通常是主板载计算机的计算机时钟。

| 字段名称       | 类型       | 单位 | 说明                         |
| -------------- | ---------- | ---- | ---------------------------- |
| time_unix_usec | `uint64_t` | us   | 时间戳（UNIX 纪元时间）。    |
| time_boot_ms   | `uint32_t` | ms   | 时间戳（系统启动后的时间）。 |

### PING (4) — [DEP]

<span class="warning">***已删除：** 被 [SYSTEM_TIME](#SYSTEM_TIME) 取代 (2011-08) - 将被移除/与 [SYSTEM_TIME](#SYSTEM_TIME) 合并）</span>。

请求或响应 ping 的 ping 消息。这允许测量系统延迟，包括串行端口、无线电调制解调器和 UDP 连接。ping 微服务的文档载于 https://mavlink.io/en/services/ping.html。

| 字段名称         | 类型       | 单位 | 说明                                                         |
| ---------------- | ---------- | ---- | ------------------------------------------------------------ |
| time_usec        | `uint64_t` | us   | 时间戳（UNIX 时间或系统启动后的时间）。接收端可以通过检查数字的大小来推断时间戳格式（自 1.1.1970 起或自系统启动起）。 |
| seq              | `uint32_t` |      | PING 序列                                                    |
| target_system    | `uint8_t`  |      | 0：请求所有接收系统 ping。如果大于 0：消息为 ping 响应，number 为请求系统的系统 ID |
| target_component | `uint8_t`  |      | 0：请求从所有接收组件 ping。如果大于 0：则消息为 ping 响应，编号为请求组件的组件 ID。 |

### CHANGE_OPERATOR_CONTROL (5) 

请求控制此 MAV

| 字段名称        | 类型       | 单位 | 说明                                                         |
| --------------- | ---------- | ---- | ------------------------------------------------------------ |
| target_system   | `uint8_t`  |      | GCS 请求控制的系统                                           |
| control_request | `uint8_t`  |      | 0：请求对该飞行器的控制，1：释放对该飞行器的控制             |
| 版本            | `uint8_t`  | rad  | 0：密钥明文，1-255：未来不同的散列/加密变体。一般来说，GCS 最初应尽可能使用最安全的模式，然后在收到 NACK 消息表明加密不匹配时，逐步降低加密级别。 |
| passkey         | `char[25]` |      | 密码/密钥，取决于明文或加密版本。25 个或更少的字符，以空格结束。字符可能包括 A-Z、a-z、0-9 和"！？ |

### CHANGE_OPERATOR_CONTROL_ACK (6) 

接受/拒绝对该飞行器的控制

| 字段名称        | 类型      | 说明                                                         |
| --------------- | --------- | ------------------------------------------------------------ |
| gcs_system_id   | `uint8_t` | 此信息的 GCS ID                                              |
| control_request | `uint8_t` | 0：请求对该飞行器的控制，1：解除对该飞行器的控制             |
| ack             | `uint8_t` | 0: ACK, 1: NACK: 密钥错误, 2: NACK: 密钥加密方法不支持, 3: NACK: 已被控制 |


### AUTH_KEY (7) 

发送加密签名/密钥，识别该系统。请注意：本协议非常简单，因此传输密钥需要加密通道，以确保真正的安全。

| 字段名称 | 类型       | 说明 |
| -------- | ---------- | ---- |
| key      | `char[32]` | key  |


### LINK_NODE_STATUS (8) — [WIP] 

<span class="warning">**WORK IN PROGRESS**: Do not use in stable production environments (it may change).</span>

通信链中每个节点生成并注入 MAVLink 数据流的状态。

| 字段名称          | 类型       | 单位    | 说明                                                   |
| ----------------- | ---------- | ------- | ------------------------------------------------------ |
| timestamp         | `uint64_t` | ms      | 时间戳（系统启动后的时间）。                           |
| tx_buf            | `uint8_t`  | %       | 剩余的可用传输缓冲空间                                 |
| rx_buf            | `uint8_t`  | %       | 剩余的可用接收缓冲区空间                               |
| tx_rate           | `uint32_t` | bytes/s | 发送速率                                               |
| rx_rate           | `uint32_t` | 字节/秒 | 接收速率                                               |
| rx_parse_err      | `uint16_t` | 字节数  | 无法正确解析的字节数。                                 |
| tx_overflows      | `uint16_t` | 字节    | 传输缓冲区溢出。该数字在达到 UINT16_MAX 时会缠绕一圈。 |
| rx_overflows      | `uint16_t` | 字节    | 接收缓冲区溢出。当达到 UINT16_MAX 时，该数字会绕一圈。 |
| messages_sent     | `uint32_t` |         | 发送的信息                                             |
| messages_received |            |         | 收到的信息（根据计数序列估计）                         |
| messages_lost     | `uint32_t` |         |                                                        |


### SET_MODE (11) — [DEP] 

<span class="warning">**DEPRECATED:** Replaced By [MAV_CMD_DO_SET_MODE](#MAV_CMD_DO_SET_MODE) (2015-12) — Use [COMMAND_LONG](#COMMAND_LONG) with [MAV_CMD_DO_SET_MODE](#MAV_CMD_DO_SET_MODE) instead)</span>

设置枚举 [MAV_MODE](#MAV_MODE)所定义的系统模式。没有目标组件 ID，因为根据定义，该模式适用于整个飞机，而不仅仅适用于某个组件。

| 字段名称       | 类型       | 值                    | 说明                                               |
| -------------- | ---------- | --------------------- | -------------------------------------------------- |
| 设置模式的系统 |            |                       |                                                    |
| base_mode      | `uint8_t`  | [MAV_MODE](#MAV_MODE) | 新的基本模式。                                     |
| custom_mode    | `uint32_t` |                       | 新的自动驾驶仪专用模式。自动驾驶仪可以忽略此字段。 |

### PARAM_REQUEST_READ (20)

请求读取带有 param_id 字符串 id 的板载参数。机载参数存储为 key[const char*] -> value[float]。这样就可以将参数发送到任何其他组件（如 GCS），而无需事先了解可能的参数名称。因此，同一个 GCS 可以为不同的自动驾驶仪存储不同的参数。有关 QGroundControl 和 IMU 代码的完整文档，请参阅 https://mavlink.io/en/services/parameter.html。

| 字段名称         | 类型       | 说明                                                         |
| ---------------- | ---------- | ------------------------------------------------------------ |
| target_system    | `uint8_t`  | 系统 ID                                                      |
| target_component | `uint8_t`  | 组件 ID                                                      |
| param_id         | `char[16]` | 板载参数 ID，如果长度小于 16 个人类可读字符，则以 NULL 结尾；如果长度正好是 16 个字符，则不以空字节（NULL）结尾 - 如果 ID 以字符串形式存储，应用程序必须提供 16+1 字节的存储空间 |
| param_index      | `int16_t`  | 参数索引。发送 -1 表示使用参数 ID 字段作为标识符（否则将忽略参数 ID） |


### PARAM_REQUEST_LIST (21) 

请求此组件的所有参数。发出请求后，所有参数将被发送。参数微服务的文档地址是 https://mavlink.io/en/services/parameter.html。

| 字段名称         | 类型      | 说明    |
| ---------------- | --------- | ------- |
| target_system    | `uint8_t` | 系统 ID |
| target_component | `uint8_t` | 组件 ID |


### PARAM_VALUE (22) 

发送机载参数值。在报文中加入 param_count 和 param_index 可让接收方跟踪收到的参数，并在丢失或超时后重新请求丢失的参数。参数微服务的文档地址为 https://mavlink.io/en/services/parameter.html。

| 字段名称    | 类型       | 值                                | 说明                                                         |
| ----------- | ---------- | --------------------------------- | ------------------------------------------------------------ |
| param_id    | `char[16]` |                                   | 板载参数 ID，如果长度小于 16 个人类可读字符，则以 NULL 结尾；如果长度正好是 16 个字符，则不以空字节（NULL）结尾--如果 ID 以字符串形式存储，应用程序必须提供 16+1 字节的存储空间 |
| param_value | `float`    |                                   | 板载参数值                                                   |
| param_type  | `uint8_t`  | [MAV_PARAM_TYPE](#MAV_PARAM_TYPE) | 板载参数类型。                                               |
| param_count | `uint16_t` |                                   | 板载参数总数                                                 |
| param_index |            |                                   | 该板载参数的索引                                             |

### PARAM_SET (23)

设置参数值（将新值写入永久存储）。

接收组件应通过广播 [PARAM_VALUE](#PARAM_VALUE)消息确认新的参数值（广播可确保多个 GCS 都拥有所有参数的最新列表）。如果发送 GCS 在超时时间内没有收到 [PARAM_VALUE](#PARAM_VALUE)，则应重新发送 [PARAM_SET](#PARAM_SET) 消息。参数微服务在 https://mavlink.io/en/services/parameter.html 有详细说明。
[PARAM_SET](#PARAM_SET) 也可以在事务（使用 [MAV_CMD_PARAM_TRANSACTION](#MAV_CMD_PARAM_TRANSACTION) 启动）中调用。在事务中，接收组件应该用 [PARAM_ACK_TRANSACTION](#PARAM_ACK_TRANSACTION)响应设置组件（而不是广播 [PARAM_VALUE](#PARAM_VALUE)），如果没有收到 ACK，应该重新发送 [PARAM_SET](#PARAM_SET)。

| 字段名称         | 类型       | 值                                | 说明                                                         |
| ---------------- | ---------- | --------------------------------- | ------------------------------------------------------------ |
| target_system    | `uint8_t`  |                                   | 系统 ID                                                      |
| target_component | `uint8_t`  |                                   | 组件 ID                                                      |
| param_id         | `char[16]` |                                   | 板载参数 ID，如果长度小于 16 个人类可读字符，则以 NULL 结尾；如果长度正好是 16 个字符，则不以空字节（NULL）结尾 - 如果 ID 以字符串形式存储，应用程序必须提供 16+1 字节的存储空间 |
| param_value      | `float`    |                                   | 板载参数值                                                   |
| param_type       | `uint8_t`  | [MAV_PARAM_TYPE](#MAV_PARAM_TYPE) | 板载参数类型。                                               |


### GPS_RAW_INT (24) 

全球定位系统 (GPS) 返回的全球位置。这

不是系统的全球位置估计值，而是传感器的原始值。有关全球位置估计值，请参阅信息 [GLOBAL_POSITION_INT](#GLOBAL_POSITION_INT)。

| 字段名称                                                     | 类型       | 单位  | 值                            | 说明                                                         |
| ------------------------------------------------------------ | ---------- | ----- | ----------------------------- | ------------------------------------------------------------ |
| time_usec                                                    | `uint64_t` | us    |                               | 时间戳（UNIX 时间或系统启动后的时间）。接收端可以通过检查数字的大小来推断时间戳格式（自 1.1.1970 起或自系统启动起）。 |
| fix_type                                                     | `uint8_t`  |       | [GPS_FIX_TYPE](#GPS_FIX_TYPE) | GPS 定位类型。                                               |
| lat                                                          | `int32_t`  | degE7 |                               | 纬度（WGS84，EGM96 椭圆形）                                  |
| lon                                                          | `int32_t`  | degE7 |                               | 经度（WGS84，EGM96 椭圆形）                                  |
| alt                                                          | `int32_t`  | mm    |                               | 高度（MSL）。正数表示向上。请注意，除了 WGS84 高度外，几乎所有 GPS 模块都提供 MSL 高度。 |
| eph                                                          | `uint16_t` |       | invalid:UINT16_MAX            | GPS HDOP 水平稀释位置（无单位 * 100）。如果未知，则设置为 UINT16_MAX |
| epv                                                          | `uint16_t` |       | invalid:UINT16_MAX            | GPS VDOP 垂直位置稀释（无单位 * 100）。If unknown, set to: UINT16_MAX |
| vel                                                          | `uint16_t` | cm/s  | invalid:UINT16_MAX            | GPS 地速。如果未知，则设置为：UINT16_MAX                     |
| cog                                                          | `uint16_t` | cdeg  | invalid:UINT16_MAX            | 地面航线（不是航向，而是移动方向），单位为度 * 100，0.0...359.99 度。如果未知，则设置为 UINT16_MAX |
| satellites_visible                                           | `uint8_t`  |       | invalid:UINT8_MAX             | 可见卫星数。如果未知，则设置为 UINT8_MAX                     |
| <span class='ext'>alt_ellipsoid</span> <a href='#mav2_extension_field'>++</a> | `int32_t`  | mm    |                               | 高度（高于 WGS84、EGM96 椭圆形）。正表示向上。               |
| <span class='ext'>h_acc</span> <a href='#mav2_extension_field'>++</a> | `uint32_t` | mm    |                               | 位置不确定。                                                 |
| <span class='ext'>v_acc</span> <a href='#mav2_extension_field'>++</a> | `uint32_t` | mm    |                               | 高度不确定。                                                 |
| <span class='ext'>vel_acc</span> <a href='#mav2_extension_field'>++</a> | `uint32_t` | mm    |                               | 速度的不确定性                                               |
| <span class='ext'>hdg_acc</span> <a href='#mav2_extension_field'>++</a> | `uint32_t` | degE5 |                               | 航向/航迹不确定性                                            |
| <span class='ext'>yaw</span> <a href='#mav2_extension_field'>++</a> | `uint16_t` | cdeg  | invalid:0                     | 从北面看大地框架的偏航。 如果 GPS 不提供偏航功能，则使用 0。如果 GPS 已配置为提供偏航，但目前无法提供，则使用 UINT16_MAX。使用 36000 表示北方。 |


### GPS_STATUS (25) 

GPS 报告的定位状态。该信息用于显示接收机可见的每颗卫星的状态信息。有关全球位置估计，请参阅信息 [GLOBAL_POSITION_INT]（#GLOBAL_POSITION_INT）。该信息最多可包含 20 颗卫星的信息。

| 字段名称            | 类型          | 单位 | 说明                               |
| ------------------- | ------------- | ---- | ---------------------------------- |
| satellites_visible  | `uint8_t`     |      | 可见卫星数                         |
| satellite_prn       | `uint8_t[20]` |      | 全球卫星 ID                        |
| satellite_used      | `uint8_t[20]` |      | 0：未使用卫星，1：用于定位         |
| satellite_elevation | `uint8_t[20]` |      | deg                                |
| satellite_azimuth   | `uint8_t[20]` | deg  | 卫星的方向，0: 0 度，255: 360 度。 |
| satellite_snr       | `uint8_t[20]` | dB   | 卫星的信噪比                       |


### SCALED_IMU (26) 

通常 9DOF 传感器设置的 RAW IMU 读数。该信息应包含按所述单位缩放的数值

| 字段名称                                                     | 类型       | 单位   | 说明                                                         |
| ------------------------------------------------------------ | ---------- | ------ | ------------------------------------------------------------ |
| time_boot_ms                                                 | `uint32_t` | ms     | 时间戳（系统启动后的时间）。                                 |
| xacc                                                         | `int16_t`  | mG     | X 加速度                                                     |
| yacc                                                         | `int16_t`  | mG     | Y 加速度                                                     |
| zacc                                                         | `int16_t`  | mG     | Z 加速度                                                     |
| xgyro                                                        | `int16_t`  | mrad/s | 绕 X 轴的角速度                                              |
| ygyro                                                        | `int16_t`  | mrad/s | 绕 Y 轴的角速度                                              |
| zgyro                                                        | `int16_t`  | mrad/s | 绕 Z 轴的角速度                                              |
| xmag                                                         | `int16_t`  | mgauss | X 磁场                                                       |
| ymag                                                         | `int16_t`  | mgauss | Y 磁场                                                       |
| zmag                                                         | `int16_t`  | mgauss | Z 磁场                                                       |
| <span class='ext'>temperature</span> <a href='#mav2_extension_field'>++</a> | `int16_t`  | cdegC  | 温度，0： IMU 不提供温度值。如果 IMU 的温度为 0C，则必须发送 1 (0.01C) |


### RAW_IMU (27) 

9DOF 传感器的原始 IMU 读数，由 id（默认为 IMU1）标识。该信息应始终包含真实的原始值，不做任何缩放，以便进行数据捕捉和系统调试。

| 字段名称                                                     | 类型       | 单位                      | 说明                                                         |
| ------------------------------------------------------------ | ---------- | ------------------------- | ------------------------------------------------------------ |
| time_usec                                                    | `uint64_t` | us                        | 时间戳（UNIX 时间或系统启动后的时间）。接收端可以通过检查数字的大小来推断时间戳格式（自 1.1.1970 起或自系统启动起）。 |
| xacc                                                         | `int16_t`  |                           | X 加速度（原始值）                                           |
| yacc                                                         | `int16_t`  |                           | Y 加速度（原始值）                                           |
| zacc                                                         | `int16_t`  | Z 加速度（原始值）        |                                                              |
| xgyro                                                        | `int16_t`  | 绕 X 轴的角速度（原始值） |                                                              |
| ygyro                                                        | `int16_t`  | 绕 Y 轴的角速度（原始值） |                                                              |
| zgyro                                                        | `int16_t`  | 绕 Z 轴的角速度（原始值） |                                                              |
| xmag                                                         | `int16_t`  |                           | X 磁场（原始值）                                             |
| Ymag                                                         | `int16_t`  |                           | Y 磁场（原始值）                                             |
| zmag                                                         | `int16_t`  |                           | Z 磁场（原始数据）                                           |
| <span class='ext'>id</span> <a href='#mav2_extension_field'>++</a> | `uint8_t`  |                           | Id。Ids 从 0 开始编号，并映射到从 1 开始编号的 IMU（例如，IMU1 将具有 id=0 的消息）<br>具有相同值的消息来自同一来源（实例）。 |
| <span class='ext'>temperature</span> <a href='#mav2_extension_field'>++</a> | `int16_t`  | cdegC                     | 温度，0： IMU 不提供温度值。如果 IMU 的温度为 0C，则必须发送 1（0.01C）。 |


### RAW_PRESSURE (28) 

一个绝对压力传感器和一个差压传感器典型设置的原始压力读数。传感器值应是未经标定的 ADC 原始值。

| 字段名称    | 类型       | 单位 | 说明                                                         |
| ----------- | ---------- | ---- | ------------------------------------------------------------ |
| time_usec   | `uint64_t` | us   | 时间戳（UNIX 时间或系统启动后的时间）。接收端可以通过检查数字的大小来推断时间戳格式（自 1.1.1970 起或自系统启动起）。 |
| press_abs   | `int16_t`  |      | 绝对压力（原始值）                                           |
| press_diff1 | `int16_t`  |      | 压差 1（原始值，不存在时为 0）                               |
| press_diff2 | `int16_t`  |      | 压差 2（原始值，不存在时为 0）                               |
| temperature | `int16_t`  |      | 原始温度测量值（原始值）                                     |


### SCALED_PRESSURE (29) 

一个绝对和差分压力传感器典型设置的压力读数。单位在每个字段中指定。

| 字段名称                                                     | 类型       | 单位  | 说明                                                      |
| ------------------------------------------------------------ | ---------- | ----- | --------------------------------------------------------- |
| time_boot_ms                                                 | `uint32_t` | ms    | 时间戳（系统启动后的时间）。                              |
| press_abs                                                    | `float`    | hPa   | 绝对压力                                                  |
| press_diff                                                   | `float`    | hPa   | 压差 1                                                    |
| temperature                                                  | `int16_t`  | cdegC | 绝对压力温度                                              |
| <span class='ext'>temperature_press_diff</span> <a href='#mav2_extension_field'>++</a> | `int16_t`  | cdegC | 压差温度（0，如果没有）。将 0（或 1）的值报告为 1 cdegC。 |


### ATTITUDE (30) 

航空框架中的姿态（右旋、Z 下、Y 右、X 前、ZYX、固有）。

| 字段名称     | 类型       | 单位  | 说明                         |
| ------------ | ---------- | ----- | ---------------------------- |
| time_boot_ms | `uint32_t` | ms    | 时间戳（系统启动后的时间）。 |
| roll         | `float`    | rad   | 滚动角度 (-pi...+pi)         |
| pitch        | `float`    | rad   | Pitch angle (-pi..+pi)       |
| yaw          | `float`    | rad   | 偏航角 (-pi..+pi)            |
| rollspeed    | `float`    | rad/s | 滚转角速度                   |
| pitchspeed   | `float`    | rad/s | 螺距角速度                   |
| yawspeed     | `float`    | rad/s | 偏航角速度                   |


### ATTITUDE_QUATERNION (31) 

航空框架中的姿态（右旋、Z 下、X 前、Y 右），用四元数表示。四元数顺序为 w、x、y、z，零旋转表示为 (1 0 0 0)。

| 字段名称                                                     | 类型       | 单位  | 说明                                                         |
| ------------------------------------------------------------ | ---------- | ----- | ------------------------------------------------------------ |
| time_boot_ms                                                 | `uint32_t` | ms    | 时间戳（系统启动后的时间）。                                 |
| q1                                                           | `float`    |       | 四元数分量 1，w（空旋转时为 1）                              |
| q2                                                           | `float`    |       | 四元数分量 2、x（空旋转时为 0）                              |
| q3                                                           | `float`    |       | 四元数分量 3、y（空旋转时为 0）                              |
| q4                                                           | `float`    |       | 四元数分量 4、z（空旋转时为 0）                              |
| rollspeed                                                    | `float`    | rad/s | 滚转角速度                                                   |
| pitchspeed                                                   | `float`    | rad/s | 螺距角速度                                                   |
| yawspeed                                                     | `float`    | rad/s | 偏航角速度                                                   |
| <span class='ext'>repr_offset_q</span> <a href='#mav2_extension_field'>++</a> | `float[4]` |       | 为便于用户显示，应旋转姿态四元数和角速度矢量的旋转偏移（四元数顺序为 [w，x，y，z]，零旋转为 [1，0，0，0]，如果不支持该字段，则发送 [0，0，0，0]）。该字段适用于飞行过程中参考姿态可能发生变化的系统。例如，尾翼 VTOL 在悬停模式和固定翼模式之间会将参考姿态旋转 90 度，因此在悬停模式下，repr_offset_q 等于 [1, 0, 0, 0]，而在固定翼模式下，repr_offset_q 等于 [0.7071, 0, 0.7071, 0]。 |


### LOCAL_POSITION_NED (32) 

滤波后的本地位置（如融合计算机视觉和加速度计）。坐标框架为右旋，Z 轴向下（航空框架，NED / 东北-向下惯例）

| 字段名称     | 类型       | 单位 | 说明                         |
| ------------ | ---------- | ---- | ---------------------------- |
| time_boot_ms | `uint32_t` | ms   | 时间戳（系统启动后的时间）。 |
| x            | `float`    | m    | X 位置                       |
| y            | `float`    | m    | Y 位置                       |
| z            | `float`    | m    | Z 位置                       |
| vx           | `float`    | m/s  | X 速度                       |
| vy           | `float`    | m/s  | Y 速度                       |
| vz           | `float`    | m/s  | Z 速度                       |


### GLOBAL_POSITION_INT (33) 

过滤后的全球位置（例如，融合 GPS 和加速度计）。位置在 GPS 框架内（右旋，Z 向上）。它

由于浮点分辨率不够，因此设计为按比例整数信息。

| 字段名称     | 类型       | 单位  | 说明                                                         |
| ------------ | ---------- | ----- | ------------------------------------------------------------ |
| time_boot_ms | `uint32_t` | ms    | 时间戳（系统启动后的时间）。                                 |
| lat          | `int32_t`  | degE7 | 表示的纬度                                                   |
| lon          | `int32_t`  | degE7 | 经度，用数字表示                                             |
| alt          | `int32_t`  | mm    | 高度（MSL）。请注意，几乎所有 GPS 模块都同时提供 WGS84 和 MSL。 |
| relative_alt | `int32_t`  | mm    | 离地高度                                                     |
| vx           | `int16_t`  | cm/s  | 地面 X 速度（纬度，正北方向）                                |
| vy           | `int16_t`  | cm/s  | 地面 Y 速度（经度，正东方向）                                |
| vz           | `int16_t`  | cm/s  | 地面 Z 速度（高度，正下方）                                  |
| hdg          | `uint16_t` | cdeg  | 车辆航向（偏航角），0.0...359.99 度。如果未知，则设置为 UINT16_MAX |


### RC_CHANNELS_SCALED (34) 

接收到的 RC 通道缩放值：(-100%) -10000，(0%) 0，(100%) 10000。不活动的通道应设置为 INT16_MAX。

| 字段名称     | 类型       | 单位 | 说明                                                         |
| ------------ | ---------- | ---- | ------------------------------------------------------------ |
| time_boot_ms | `uint32_t` | ms   | 时间戳（系统启动后的时间）。                                 |
| port         | `uint8_t`  |      | 伺服输出端口（一组 8 个输出 = 1 个端口）。在 Pixhawk 上运行的飞行堆栈应使用： 0 = 主端口，1 = 辅助端口。 |
| chan1_scaled | `int16_t`  |      |                                                              |
| chan2_scaled | `int16_t`  |      | RC 通道 2 的值已缩放。                                       |
| chan3_scaled | `int16_t`  |      | RC 通道 3 的值已缩放。                                       |
| chan4_scaled | `int16_t`  |      | RC 通道 4 的值已缩放。                                       |
| chan5_scaled | `int16_t`  |      | RC 通道 5 的值已缩放。                                       |
| chan6_scaled | `int16_t`  |      | RC 通道 6 的值已缩放。                                       |
| chan7_scaled | `int16_t`  |      | RC 通道 7 的值已缩放。                                       |
| chan8_scaled | `int16_t`  |      | RC 通道 8 的值已缩放。                                       |
| rssi         | `uint8_t`  |      | 接收信号强度指示器，单位/刻度取决于设备。值： [0-254]，UINT8_MAX：无效/未知。 |


### RC_CHANNELS_RAW (35) 

接收到的 RC 信道的 RAW 值。标准 PPM 调制方式如下： 1000 微秒 0%，2000 微秒： 100%. UINT16_MAX 的值表示通道未使用。个别接收器/发射器可能会违反此规范。

| 字段名称     | 类型       | 单位 | 说明                                                         |
| ------------ | ---------- | ---- | ------------------------------------------------------------ |
| time_boot_ms | `uint32_t` | ms   | 时间戳（系统启动后的时间）。                                 |
| port         | `uint8_t`  |      | 伺服输出端口（一组 8 个输出 = 1 个端口）。在 Pixhawk 上运行的飞行堆栈应使用： 0 = 主端口，1 = 辅助端口。 |
| chan1_raw    | `uint16_t` | us   | RC 通道 1 的值。                                             |
| chan2_raw    | `uint16_t` | us   | RC 通道 2 值。                                               |
| chan3_raw    | `uint16_t` | us   | RC 通道 3 值。                                               |
| chan4_raw    | `uint16_t` | us   | RC 通道 4 值。                                               |
| chan5_raw    | `uint16_t` | us   | RC 通道 5 值。                                               |
| chan6_raw    | `uint16_t` | us   | RC 通道 6 值。                                               |
| chan7_raw    | `uint16_t` | us   | RC 通道 7 值。                                               |
| chan8_raw    | `uint16_t` | us   | RC 通道 8 值。                                               |
| rssi         | `uint8_t`  |      | 接收信号强度指示器，单位/刻度取决于设备。值： [0-254]，UINT8_MAX：无效/未知。 |


### SERVO_OUTPUT_RAW (36) 

由 [ACTUATOR_OUTPUT_STATUS](#ACTUATOR_OUTPUT_STATUS)取代。舵机输出的 RAW 值（对于来自遥控器的遥控输入，请使用 [RC_CHANNELS](#RC_CHANNELS) 信息）。标准 PPM 调制如下： 1000 微秒 0%，2000 微秒： 100%.

| 字段名称                                                     | 类型       | 单位 | 说明                                                         |
| ------------------------------------------------------------ | ---------- | ---- | ------------------------------------------------------------ |
| time_usec                                                    | `uint32_t` | us   | 时间戳（UNIX 时间或系统启动后的时间）。接收端可以通过检查数字的大小来推断时间戳格式（自 1.1.1970 起或自系统启动起）。 |
| port                                                         | `uint8_t`  |      | 伺服输出端口（一组 8 个输出 = 1 个端口）。在 Pixhawk 上运行的飞行堆栈应使用： 0 = 主端口，1 = 辅助端口。 |
| servo1_raw                                                   | `uint16_t` | us   | 伺服输出 1 的值                                              |
| servo2_raw                                                   | `uint16_t` | us   | 伺服输出 2 值                                                |
| servo3_raw                                                   | `uint16_t` | us   | 伺服输出 3 值                                                |
| servo4_raw                                                   | `uint16_t` | us   | 伺服输出 4 值                                                |
| servo5_raw                                                   | `uint16_t` | us   | 伺服输出 5 值                                                |
| servo6_raw                                                   | `uint16_t` | us   | 伺服输出 6 值                                                |
| servo7_raw                                                   | `uint16_t` | us   | 伺服输出 7 值                                                |
| servo8_raw                                                   | `uint16_t` | us   | 伺服输出 8 值                                                |
| <span class='ext'>servo9_raw</span> <a href='#mav2_extension_field'>++</a> | `uint16_t` | us   | 伺服输出 9 值                                                |
| <span class='ext'>servo10_raw</span> <a href='#mav2_extension_field'>++</a> | `uint16_t` | us   | 伺服输出 10 值                                               |
| <span class='ext'>servo11_raw</span> <a href='#mav2_extension_field'>++</a> | `uint16_t` | us   | 伺服输出 11 value                                            |
| <span class='ext'>servo12_raw</span> <a href='#mav2_extension_field'>++</a> | `uint16_t` | us   | 伺服输出 12 value                                            |
| <span class='ext'>servo13_raw</span> <a href='#mav2_extension_field'>++</a> | `uint16_t` | us   | 伺服输出 13 value                                            |
| <span class='ext'>servo14_raw</span> <a href='#mav2_extension_field'>++</a> | `uint16_t` | us   | 伺服输出 14 value                                            |
| <span class='ext'>servo15_raw</span> <a href='#mav2_extension_field'>++</a> | `uint16_t` | us   | 伺服输出 15 value                                            |
| <span class='ext'>servo16_raw</span> <a href='#mav2_extension_field'>++</a> | `uint16_t` | us   | 伺服输出 16 value                                            |


### MISSION_REQUEST_PARTIAL_LIST (37) 

要求系统/组件提供部分任务项目列表。https://mavlink.io/en/services/mission.html。如果起点和终点索引相同，只需发送一个航点。

| 字段名称                                                     | 类型      | 值                                    | 说明                                                         |
| ------------------------------------------------------------ | --------- | ------------------------------------- | ------------------------------------------------------------ |
| target_system                                                | `uint8_t` |                                       | 系统 ID                                                      |
| target_component                                             | `uint8_t` |                                       | 组件 ID                                                      |
| start_index                                                  | `int16_t` |                                       | 起始索引                                                     |
| end_index                                                    | `int16_t` |                                       | 结束索引，默认为-1（-1：发送列表到结束）。否则为列表的有效索引 |
| <span class='ext'>mission_type</span> <a href='#mav2_extension_field'>++</a> | `uint8_t` | [MAV_MISSION_TYPE](#MAV_MISSION_TYPE) | 任务类型                                                     |


### MISSION_WRITE_PARTIAL_LIST (38) 

该信息将发送至 MAV，以写入部分列表。如果开始索引 == 结束索引，则只传输/更新一个项目。如果起始索引不为 0 且大于当前列表大小，则应拒绝此请求！

| 字段名称                                                     | 类型      | 值                                    | 说明                                             |
| ------------------------------------------------------------ | --------- | ------------------------------------- | ------------------------------------------------ |
| target_system                                                | `uint8_t` |                                       | 系统 ID                                          |
| target_component                                             | `uint8_t` |                                       | 组件 ID                                          |
| start_index                                                  | `int16_t` |                                       | 起始索引。必须小于或等于当前板载列表的最大索引。 |
| end_index                                                    | `int16_t` |                                       | 终止索引，等于或大于起始索引。                   |
| <span class='ext'>mission_type</span> <a href='#mav2_extension_field'>++</a> | `uint8_t` | [MAV_MISSION_TYPE](#MAV_MISSION_TYPE) | 任务类型                                         |


### MISSION_ITEM (39) — [DEP] 

<span class="warning">**DEPRECATED:** Replaced By [MISSION_ITEM_INT](#MISSION_ITEM_INT) (2020-06)</span>

对任务项目进行编码的信息。该信息用于宣布

任务项目的存在，并在系统中设置任务项目。任务项目的单位可以是 x、y、z 米（类型：LOCAL）或 x:lat、y:lon、z:altitude。本地帧为 Z 向下、右手（NED），全局帧为 Z 向上、右手（ENU）。可使用 NaN 表示可选值/默认值（例如，使用系统当前的纬度或偏航而非特定值）。另请参阅 https://mavlink.io/en/services/mission.html。

| 字段名称                                                     | 类型       | 值                                    | 说明                                                         |
| ------------------------------------------------------------ | ---------- | ------------------------------------- | ------------------------------------------------------------ |
| target_system                                                | `uint8_t`  |                                       | 系统 ID                                                      |
| target_component                                             | `uint8_t`  |                                       | 组件 ID                                                      |
| seq                                                          | `uint16_t` |                                       | 序列                                                         |
| frame                                                        | `uint8_t`  | [MAV_FRAME](#MAV_FRAME)               | 航点的坐标系。                                               |
| command                                                      | `uint16_t` | [MAV_CMD](#mav_commands)              | 航点的预定操作。                                             |
| current                                                      | `uint8_t`  |                                       | false:0, true:1                                              |
| autocontinue                                                 | `uint8_t`  |                                       | 自动继续下一个航点。0：false，1：true。设置为 false 会在项目完成后暂停任务。 |
| param1                                                       | `float`    |                                       | PARAM1，参见 [MAV_CMD](#mav_commands)枚举                    |
| param2                                                       | `float`    |                                       | PARAM2，参见 [MAV_CMD](#mav_commands)枚举                    |
| param3                                                       | `float`    |                                       | PARAM3，参见 [MAV_CMD](#mav_commands)枚举                    |
| param4                                                       | `float`    |                                       | PARAM4，参见 [MAV_CMD](#mav_commands)枚举                    |
| x                                                            | `float`    |                                       | PARAM5 / 本地： X 坐标，全局：纬度                           |
| y                                                            | `float`    |                                       | PARAM6 / 局部：Y 坐标，全局：经度： Y 坐标，全局：经度       |
| z                                                            | `float`    |                                       | PARAM7 / 局部： Z 坐标，全局：高度（相对或绝对，取决于帧）。 |
| <span class='ext'>mission_type</span> <a href='#mav2_extension_field'>++</a> | `uint8_t`  | [MAV_MISSION_TYPE](#MAV_MISSION_TYPE) | 任务类型                                                     |


### MISSION_REQUEST (40) — [DEP] 

<span class="warning">**DEPRECATED:** Replaced By [MISSION_REQUEST_INT](#MISSION_REQUEST_INT) (2020-06) — A system that gets this request should respond with [MISSION_ITEM_INT](#MISSION_ITEM_INT) (as though [MISSION_REQUEST_INT](#MISSION_REQUEST_INT) was received).)</span>

请求获取序列号为 seq 的任务项目信息。系统对此信息的响应应为 [MISSION_ITEM]（#MISSION_ITEM）信息。https://mavlink.io/en/services/mission.html。

| 字段名称                                                     | 类型       | 值                                    | 说明     |
| ------------------------------------------------------------ | ---------- | ------------------------------------- | -------- |
| target_system                                                | `uint8_t`  |                                       | 系统 ID  |
| target_component                                             | `uint8_t`  |                                       | 组件 ID  |
| seq                                                          | `uint16_t` |                                       | 序列     |
| <span class='ext'>mission_type</span> <a href='#mav2_extension_field'>++</a> | `uint8_t`  | [MAV_MISSION_TYPE](#MAV_MISSION_TYPE) | 任务类型 |


### MISSION_SET_CURRENT (41) — [DEP] 

<span class="warning">**DEPRECATED:** Replaced By [MAV_CMD_DO_SET_MISSION_CURRENT](#MAV_CMD_DO_SET_MISSION_CURRENT) (2022-08)</span>

将序列号为 seq 的任务项目设为当前项目，并发出 [MISSION_CURRENT](#MISSION_CURRENT)（无论任务编号是否改变）。
如果当前正在执行任务，系统将以最短路径继续执行这个新任务项目，跳过任何中间任务项目。
请注意，任务跳转重复计数器不会被重置（参见 [MAV_CMD_DO_JUMP](#MAV_CMD_DO_JUMP) param2）。

在某些系统上，此信息可能会触发任务状态机的改变：例如从[MISSION_STATE_NOT_STARTED](#MISSION_STATE_NOT_STARTED)或[MISSION_STATE_PAUSED](#MISSION_STATE_PAUSED)到[MISSION_STATE_ACTIVE](#MISSION_STATE_ACTIVE)。
如果系统处于任务模式，在这些系统上，该命令可能会因此启动、重启或恢复任务。
如果系统未处于任务模式，则该信息不得触发任务模式切换。

| 字段名称         | 类型       | 说明    |
| ---------------- | ---------- | ------- |
| target_system    | `uint8_t`  | 系统 ID |
| target_component | `uint8_t`  | 组件 ID |
| seq              | `uint16_t` | 序列    |


### MISSION_CURRENT (42) 

公布当前目标任务项目序列号的信息（当任务运行时，系统将飞向/执行该项目）。
该信息应一直以流式传输（通常为 1Hz）。
该信息应在调用 [MAV_CMD_DO_SET_MISSION_CURRENT](#MAV_CMD_DO_SET_MISSION_CURRENT) 或 [SET_MISSION_CURRENT](#SET_MISSION_CURRENT) 之后发出。

| 字段名称                                                     | 类型       | 值                                        | 说明                                                         |
| ------------------------------------------------------------ | ---------- | ----------------------------------------- | ------------------------------------------------------------ |
| seq                                                          | `uint16_t` |                                           | 序列                                                         |
| <span class='ext'>total</span> <a href='#mav2_extension_field'>++</a> | `uint16_t` | invalid:UINT16_MAX                        | 飞行器上的任务项目总数（最后一个项目，序列 == 总数）。如果自动驾驶仪将其原点位置存储为任务的一部分，则不计入总数。0：不支持，如果飞行器上没有任务，则为 UINT16_MAX。 |
| <span class='ext'>mission_state</span> <a href='#mav2_extension_field'>++</a> | `uint8_t`  | invalid:0 [MISSION_STATE](#MISSION_STATE) | 任务状态机状态。如果不支持状态报告，则显示 [MISSION_STATE_UNKNOWN](#MISSION_STATE_UNKNOWN)。 |
| <span class='ext'>mission_mode</span> <a href='#mav2_extension_field'>++</a> | `uint8_t`  | invalid:0                                 | 车辆处于可执行任务项目的模式还是暂停模式。0：未知，1：处于任务模式，2：暂停（不处于任务模式）。 |
| <span class='ext'>mission_id</span> <a href='#mav2_extension_field'>++</a> | `uint32_t` | invalid:0                                 | 当前车载任务计划的 ID，如果不支持 ID 或未加载任务，则为 0。GCS 可以用它来跟踪任务计划类型的变化。任务上传时（在 [MISSION_ACK](#MISSION_ACK)）也会返回相同的值。 |
| <span class='ext'>fence_id</span> <a href='#mav2_extension_field'>++</a> | `uint32_t` | invalid:0                                 | 当前车载围栏计划的 ID，如果不支持 ID 或未加载围栏，则为 0。GCS 可以用它来跟踪围栏计划类型的变化。围栏上传时（[MISSION_ACK](#MISSION_ACK)中）也会返回相同的值。 |
| <span class='ext'>rally_points_id</span> <a href='#mav2_extension_field'>++</a> | `uint32_t` | invalid:0                                 | 当前车载集结点计划的 ID，如果不支持 ID 或未加载集结点，则为 0。GCS 可以用它来跟踪集结点计划类型的变化。集结点上传时（在 [MISSION_ACK](#MISSION_ACK)）也会返回相同的值。 |


### MISSION_REQUEST_LIST (43) 

要求系统/组件提供任务项目总清单。

| 字段名称                                                     | 类型      | 值                                    | 说明          |
| ------------------------------------------------------------ | --------- | ------------------------------------- | ------------- |
| target_system                                                | `uint8_t` |                                       | 系统 ID       |
| target_component                                             | `uint8_t` |                                       | 组件 ID       |
| <span class='ext'>mission_type</span> <a href='#mav2_extension_field'>++</a> | `uint8_t` | [MAV_MISSION_TYPE](#MAV_MISSION_TYPE) | Mission type. |


### MISSION_COUNT (44) 

该信息是 MAV 对 [MISSION_REQUEST_LIST](#MISSION_REQUEST_LIST)的响应，并启动写入事务。然后，GCS 可根据航点总数请求单个任务项目。

| 字段名称                                                     | 类型       | 值                                    | 说明          |
| ------------------------------------------------------------ | ---------- | ------------------------------------- | ------------- |
| target_system                                                | `uint8_t`  |                                       | 系统 ID       |
| target_component                                             | `uint8_t`  |                                       | 组件 ID       |
| count                                                        | `uint16_t` | 序列中的任务项目数                    |               |
| <span class='ext'>mission_type</span> <a href='#mav2_extension_field'>++</a> | `uint8_t`  | [MAV_MISSION_TYPE](#MAV_MISSION_TYPE) | Mission type. |
| <span class='ext'>opaque_id</span> <a href='#mav2_extension_field'>++</a> | `uint32_t` | invalid:0                             |               |
| 612 / 5,000                                                  |            |                                       |               |
| 当前车载任务、围栏或集合点计划的 ID（从车辆下载时）。<br>此字段在将计划从车辆下载到 GCS 时使用。<br>从 GCS 上传到车辆时为 0。<br>如果不支持计划 ID，则为 0。<br>当前车载计划 ID 在 `[MISSION_CURRENT](#MISSION_CURRENT)` 中流式传输，允许 GCS 确定计划的任何部分是否已更改并需要重新上传。<br>当车载计划的任何部分发生变化时，车辆会重新计算 ID（当上传新计划时，车辆会在 [MISSION_ACK](#MISSION_ACK)) 中将新 ID 返回给 GCS）。 |            |                                       |               |


### MISSION_CLEAR_ALL (45) 

一次性删除所有任务项目。

| 字段名称                                                     | 类型      | 值                                    | 说明          |
| ------------------------------------------------------------ | --------- | ------------------------------------- | ------------- |
| target_system                                                | `uint8_t` |                                       | 系统 ID       |
| target_component                                             | `uint8_t` |                                       | 组件 ID       |
| <span class='ext'>mission_type</span> <a href='#mav2_extension_field'>++</a> | `uint8_t` | [MAV_MISSION_TYPE](#MAV_MISSION_TYPE) | Mission type. |


### MISSION_ITEM_REACHED (46) 

已到达某个任务项目。系统将保持该位置（或在轨道上绕圈），或者（如果在 WP 上设置了自动继续）继续前往下一个航点。

| 字段名称 | 类型       | 说明 |
| -------- | ---------- | ---- |
| seq      | `uint16_t` | 序列 |


### MISSION_ACK (47) 

航点处理过程中的确认信息。类型字段说明该信息是正回执（类型=0）还是发生了错误（类型=非零）。

| 字段名称                                                     | 类型       | 值                                        | 说明                                                         |
| ------------------------------------------------------------ | ---------- | ----------------------------------------- | ------------------------------------------------------------ |
| target_system                                                | `uint8_t`  |                                           | 系统 ID                                                      |
| target_component                                             | `uint8_t`  |                                           | 组件 ID                                                      |
| type                                                         | `uint8_t`  | [MAV_MISSION_RESULT](#MAV_MISSION_RESULT) | Mission result.                                              |
| <span class='ext'>mission_type</span> <a href='#mav2_extension_field'>++</a> | `uint8_t`  | [MAV_MISSION_TYPE](#MAV_MISSION_TYPE)     | Mission type.                                                |
| <span class='ext'>opaque_id</span> <a href='#mav2_extension_field'>++</a> | `uint32_t` | invalid:0                                 | 新的车载任务、围栏或集合点计划的 ID（上传到车辆时）。<br>当 GCS 上传新计划时，车辆会计算并返回此 ID。<br>对 ID 的唯一要求是，当车载计划类型发生任何变化时，它必须更改（不要求 ID 必须是全局唯一的）。<br>从车辆下载到 GCS 时为 0（下载时，ID 设置在 [MISSION_COUNT](#MISSION_COUNT))。<br>如果不支持计划 ID，则为 0。<br>当前车载计划 ID 在 `[MISSION_CURRENT](#MISSION_CURRENT)` 中流式传输，允许 GCS 确定计划的任何部分是否已更改并需要重新上传。 |


### SET_GPS_GLOBAL_ORIGIN (48) 

设置车辆本地原点（0,0,0）位置的 GPS 坐标。无论原点是否改变，车辆都应发出 [GPS_GLOBAL_ORIGIN](#GPS_GLOBAL_ORIGIN)。这样就可以在本地坐标框架和全球（GPS）坐标框架之间进行转换，当（例如）室内和室外设置相连，MAV 需要从室内移动到室外时，这可能是必要的。

| 字段名称                                                     | 类型       | 单位  | 说明                                                         |
| ------------------------------------------------------------ | ---------- | ----- | ------------------------------------------------------------ |
| target_system                                                | `uint8_t`  |       | 系统 ID                                                      |
| latitude                                                     | `int32_t`  | degE7 | 纬度 (WGS84)                                                 |
| 经度                                                         | `int32_t`  | degE7 | 经度 (WGS84)                                                 |
| 高度                                                         | `int32_t`  | mm    | 高度（MSL）。正数表示向上。                                  |
| <span class='ext'>time_usec</span> <a href='#mav2_extension_field'>++</a> | `uint64_t` | us    | 时间戳（UNIX 纪元时间或系统启动后的时间）。接收端可通过检查数字的大小来推断时间戳格式（自 1.1.1970 起或自系统启动起）。 |


### GPS_GLOBAL_ORIGIN (49) 

发布车辆本地原点（0,0,0）位置的 GPS 坐标。每当请求或设置新的 GPS 本地位置映射（如 [SET_GPS_GLOBAL_ORIGIN](#SET_GPS_GLOBAL_ORIGIN)消息之后）时发出。

| 字段名称                                                     | 类型       | 单位  | 说明                                                         |
| ------------------------------------------------------------ | ---------- | ----- | ------------------------------------------------------------ |
| latitude                                                     | `int32_t`  | degE7 | Latitude (WGS84)                                             |
| 经度                                                         | `int32_t`  | degE7 | 经度 (WGS84)                                                 |
| 高度                                                         | `int32_t`  | mm    | 高度（MSL）。正数表示向上。                                  |
| <span class='ext'>time_usec</span> <a href='#mav2_extension_field'>++</a> | `uint64_t` | us    | 时间戳（UNIX 纪元时间或系统启动后的时间）。接收端可通过检查数字的大小来推断时间戳格式（自 1.1.1970 起或自系统启动起）。 |


### PARAM_MAP_RC (50) 

将 RC 通道与参数绑定。参数应根据 RC 通道的值变化。

| 字段名称                   | 类型       | 说明                                                         |
| -------------------------- | ---------- | ------------------------------------------------------------ |
| target_system              | `uint8_t`  | 系统 ID                                                      |
| target_component           | `uint8_t`  | 组件 ID                                                      |
| param_id                   | `char[16]` | 板载参数 ID，如果长度小于 16 个人类可读字符，则以 NULL 结尾；如果长度正好是 16 个字符，则不以空字节（NULL）结尾 - 如果 ID 以字符串形式存储，应用程序必须提供 16+1 字节的存储空间 |
| param_index                | `int16_t`  | 参数索引。发送 -1 表示使用参数 ID 字段作为标识符（否则参数 ID 将被忽略），发送 -2 表示禁用此 rc_channel_index 的任何现有映射。 |
| parameter_rc_channel_index | `uint8_t`  | 参数 RC 通道的索引。不等于 RC 通道 ID。通常对应 RC 上的电位器旋钮。 |
| param_value0               | `float`    | 初始参数值                                                   |
| scale                      | `float`    | 比例，将 RC 范围 [-1, 1] 映射为参数值                        |
| param_value_min            | `float`    | 最小参数值。协议未定义是否覆盖板载最小值。(取决于实现）      |
| param_value_max            | `float`    | 最大参数值。协议未定义是否覆盖板载最大值。(取决于实现）      |


### MISSION_REQUEST_INT (51) 

请求获取序列号为 seq 的任务项目信息。系统对此报文的响应应为 [MISSION_ITEM_INT]（#MISSION_ITEM_INT）报文。 https://mavlink.io/en/services/mission.html

| 字段名称                                                     | 类型       | 值                                    | 说明          |
| ------------------------------------------------------------ | ---------- | ------------------------------------- | ------------- |
| target_system                                                | `uint8_t`  |                                       | 系统 ID       |
| target_component                                             | `uint8_t`  |                                       | 组件 ID       |
| seq                                                          | `uint16_t` | 序列                                  |               |
| <span class='ext'>mission_type</span> <a href='#mav2_extension_field'>++</a> | `uint8_t`  | [MAV_MISSION_TYPE](#MAV_MISSION_TYPE) | Mission type. |


### SAFETY_SET_ALLOWED_AREA (54) 

设置安全区域（体积），由立方体的两个角定义。该信息可用于告诉 MAV 接受哪些设定点/航路点，拒绝哪些设定点/航路点。安全区域通常由国家或比赛规定强制执行。

| 字段名称         | 类型      | 单位 | 值                      | 说明                                                         |
| ---------------- | --------- | ---- | ----------------------- | ------------------------------------------------------------ |
| target_system    | `uint8_t` |      |                         | 系统 ID                                                      |
| target_component | `uint8_t` |      |                         | 组件 ID                                                      |
| frame            | `uint8_t` |      | [MAV_FRAME](#MAV_FRAME) | 坐标框架。可以是全局、GPS、右旋、Z 轴向上，也可以是局部、右旋、Z 轴向下。 |
| p1x              | `float`   | m    |                         |                                                              |
| p1y              | `float`   | m    |                         | y 位置 1 / 经度 1                                            |
| p1z              | `float`   | m    |                         | z 位置 1 / 高度 1                                            |
| p2x              | `float`   | m    |                         | x 位置 2 / 纬度 2                                            |
| p2y              | `float`   | m    |                         | y 位置 2 / 经度 2                                            |
| p2z              | `float`   | m    |                         | z 位置 2 / 高度 2                                            |


### SAFETY_ALLOWED_AREA (55) 

读出 MAV 当前所处的安全区域。

| 字段名称 | 类型      | 单位 | 值                      | 说明                                                         |
| -------- | --------- | ---- | ----------------------- | ------------------------------------------------------------ |
| frame    | `uint8_t` |      | [MAV_FRAME](#MAV_FRAME) | 坐标框架。可以是全局、GPS、右旋且 Z 轴向上，也可以是局部、右旋且 Z 轴向下。 |
| p1x      | `float`   | m    |                         |                                                              |
| p1y      | `float`   | m    |                         | y 位置 1 / 经度 1                                            |
| p1z      | `float`   | m    |                         | z 位置 1 / 高度 1                                            |
| p2x      | `float`   | m    |                         | x 位置 2 / 纬度 2                                            |
| p2y      | `float`   | m    |                         | y 位置 2 / 经度 2                                            |
| p2z      | `float`   | m    |                         | z 位置 2 / 高度 2                                            |


### ATTITUDE_QUATERNION_COV (61) 

航空框架中的姿态（右旋、Z 下、X 前、Y 右），用四元数表示。四元数顺序为 w、x、y、z，零旋转表示为 (1 0 0 0)。

| 字段名称   | 类型       | 单位  | 说明                                                         |
| ---------- | ---------- | ----- | ------------------------------------------------------------ |
| time_usec  | `uint64_t` | us    | 时间戳（UNIX 时间或系统启动后的时间）。接收端可以通过检查数字的大小来推断时间戳格式（自 1.1.1970 起或自系统启动起）。 |
| q          | `float[4]` |       | 四元数分量、w、x、y、z（1 0 0 0 为空旋转）                   |
| rollspeed  | `float`    | rad/s | 滚转角速度                                                   |
| pitchspeed | `float`    | rad/s | 螺距角速度                                                   |
| yawspeed   | `float`    | rad/s | 偏航角速度                                                   |
| covariance | `float[9]` |       | 3x3姿态协方差矩阵（状态：滚转、俯仰、偏航；前三项为第一行，后三项为第二行，以此类推）的行主表示。如果未知，则为数组中的第一个元素赋 NaN 值。 |


### NAV_CONTROLLER_OUTPUT (62) 

导航和定位控制器的状态。

| 字段名称       | 类型       | 单位 | 说明                          |
| -------------- | ---------- | ---- | ----------------------------- |
| nav_roll       | `float`    | deg  | 当前期望的滚转度              |
| nav_pitch      | `float`    | deg  | 当前希望的俯仰角              |
| nav_bearing    | `int16_t`  | deg  | 当前期望航向                  |
| target_bearing | `int16_t`  | deg  | 当前航点/目标的方位           |
| wp_dist        | `uint16_t` | m    | 与当前航点的距离              |
| alt_error      | `float`    | m    | 当前高度误差                  |
| aspd_error     | `float`    | m/s  | 当前空速误差                  |
| xtrack_error   | `float`    | m    | 当前 x-y 平面上的串行轨迹误差 |


### GLOBAL_POSITION_INT_COV (63) 

过滤后的全球位置（例如，融合 GPS 和加速度计）。位置为 GPS 帧（右旋，Z 向上）。由于浮点分辨率不够，因此设计为按比例整数报文。注意：该信息适用于机载网络/配套计算机和带宽较高的链路，并对准确性和完整性进行了优化。请使用[GLOBAL_POSITION_INT](#GLOBAL_POSITION_INT)报文获取最小子集。

| 字段名称       | 类型        | 单位           | 值                                                           | 说明                                                         |
| -------------- | ----------- | -------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| time_usec      | `uint64_t`  | us             |                                                              | 时间戳（UNIX 时间或系统启动后的时间）。接收端可以通过检查数字的大小来推断时间戳格式（自 1.1.1970 起或自系统启动起）。 |
| estimator_type | `uint8_t`   |                | [MAV_ESTIMATOR_TYPE](#MAV_ESTIMATOR_TYPE)                    | 此估计值来源于估计器的类 id。                                |
| lat            | `int32_t`   | degE7          |                                                              | 纬度                                                         |
| lon            | `int32_t`   | degE7          |                                                              | 经度                                                         |
| alt            | `int32_t`   | mm             |                                                              | 高度，以 MSL 以上米数为单位                                  |
| relative_alt   | `int32_t`   | mm             |                                                              | 离地面高度                                                   |
| vx             | `float`     | m/s            |                                                              | 地面 X 速度（纬度）                                          |
| vy             | `float`     |                | 地面 Y 速度（经度）                                          |                                                              |
| vz             | `float`     | m/s            |                                                              | 地面 Z 速度（高度）                                          |
| covariance     | `float[36]` | invalid:[NaN:] | 6x6 位置和速度 6x6 交叉协方差矩阵（状态：lat、lon、alt、vx、vy、vz；前六项为第一行，后六项为第二行，以此类推）的主行表示。如果未知，则为数组中的第一个元素赋 NaN 值。 |                                                              |

### LOCAL_POSITION_NED_COV (64) 

滤波后的本地位置（如融合计算机视觉和加速度计）。坐标框架为右旋，Z 轴向下（航空框架，NED / 东北-向下惯例）

| 字段名称       | 类型        | 单位 | 数值                                      | 说明                                                         |
| -------------- | ----------- | ---- | ----------------------------------------- | ------------------------------------------------------------ |
| time_usec      | `uint64_t`  | us   |                                           | 时间戳（UNIX 时间或系统启动后的时间）。接收端可以通过检查数字的大小来推断时间戳格式（自 1.1.1970 起或自系统启动起）。 |
| estimator_type | `uint8_t`   |      | [MAV_ESTIMATOR_TYPE](#MAV_ESTIMATOR_TYPE) | 此估计值来源于估计器的类 id。                                |
| x              | `float`     | m    |                                           |                                                              |
| y              | `float`     | m    |                                           | Y 位置                                                       |
| z              | `float`     | m    |                                           | Z 位置                                                       |
| vx             | `float`     | m/s  |                                           | X 速度                                                       |
| vy             | `float`     | m/s  |                                           | Y 速度                                                       |
| vz             | `float`     | m/s  |                                           | Z 速度                                                       |
| ax             | `float`     | m/s  |                                           | X 加速度                                                     |
| ay             | `float`     | m/s  |                                           | Y 加速度                                                     |
| az             | `float`     | m/s  |                                           | Z 加速度                                                     |
| covariance     | `float[45]` |      | invalid:[NaN:]                            | 位置、速度和加速度的 9x9 交叉协方差矩阵右上角三角形（状态：x, y, z, vx, vy, vz, ax, ay, az；前 9 个条目为第一行，后 8 个条目为第二行，以此类推）。如果未知，则为数组中的第一个元素赋 NaN 值。 |


### RC_CHANNELS (65) 

接收到的 RC 信道的 PPM 值。标准 PPM 调制方式如下： 1000 微秒 0%，2000 微秒 100%.  UINT16_MAX 的值表示通道未使用。个别接收器/发射器可能会违反此规范。

| 字段名称     | 类型       | 单位 | 说明                                                         |
| ------------ | ---------- | ---- | ------------------------------------------------------------ |
| time_boot_ms | `uint32_t` | ms   | 时间戳（系统启动后的时间）。                                 |
| chancount    | `uint8_t`  |      | 接收到的 RC 信道总数。该值可以大于 18，表示有更多可用信道，但未在此报文中给出。如果没有可用的 RC 信道，该值应为 0。 |
| chan1_raw    | `uint16_t` | us   | RC 信道 1 的值。                                             |
| chan2_raw    | `uint16_t` | us   | RC 通道 2 的值。                                             |
| chan3_raw    | `uint16_t` | us   | RC 通道 3 的值。                                             |
| chan4_raw    | `uint16_t` | us   | RC 通道 4 值。                                               |
| chan5_raw    | `uint16_t` | us   | RC 通道 5 值。                                               |
| chan6_raw    | `uint16_t` | us   | RC 通道 6 值。                                               |
| chan7_raw    | `uint16_t` | us   | RC 通道 7 值。                                               |
| chan8_raw    | `uint16_t` | us   | RC 通道 8 值。                                               |
| chan9_raw    | `uint16_t` | us   | RC 通道 9 的值。                                             |
| chan10_raw   | `uint16_t` | us   | RC 通道 10 的值。                                            |
| chan11_raw   | `uint16_t` | us   | RC 通道 11 的值。                                            |
| chan12_raw   | `uint16_t` | us   | RC 通道 12 值。                                              |
| chan13_raw   | `uint16_t` | us   | RC 通道 13 值。                                              |
| chan14_raw   | `uint16_t` | us   | RC 通道 14 值。                                              |
| chan15_raw   | `uint16_t` | us   | RC 通道 15 值。                                              |
| chan16_raw   | `uint16_t` | us   | RC 通道 16 值。                                              |
| chan17_raw   | `uint16_t` | us   | RC 通道 17 值。                                              |
| chan18_raw   | `uint16_t` | us   | RC 通道 18 值。                                              |
| rssi         | `uint8_t`  |      | 接收信号强度指示器，单位/刻度取决于设备。值： [0-254]，UINT8_MAX：无效/未知。 |


### REQUEST_DATA_STREAM (66) — [DEP] 

<span class="warning">**DEPRECATED:** Replaced By [MAV_CMD_SET_MESSAGE_INTERVAL](#MAV_CMD_SET_MESSAGE_INTERVAL)  (2015-08)</span>

请求数据流。

| 字段名称         | 类型       | 单位 | 说明                             |
| ---------------- | ---------- | ---- | -------------------------------- |
| target_system    | `uint8_t`  |      | 请求发送消息流的目标。           |
| target_component | `uint8_t`  |      | 请求发送数据流的目标。           |
| req_stream_id    | `uint8_t`  |      | 请求数据流的 ID                  |
| req_message_rate | `uint16_t` |      | Hz                               |
| start_stop       | `uint8_t`  |      | 1 表示开始发送，0 表示停止发送。 |


### DATA_STREAM (67) — [DEP] 

<span class="warning">**DEPRECATED:** Replaced By [MESSAGE_INTERVAL](#MESSAGE_INTERVAL) (2015-08)</span>

数据流状态信息。

| 字段名称                             | 类型 | 单位 | 说明 |
| ------------------------------------ | ---- | ---- | ---- |
| 请求数据流的 ID                      |      |      |      |
| 消息速率                             |      |      |      |
| 1 表示启用数据流，0 表示停止数据流。 |      |      |      |


### MANUAL_CONTROL (69) 

该信息提供了一个应用程序接口，用于使用标准操纵杆轴命名法和类似操纵杆的输入设备对车辆进行手动控制。未使用的轴可被禁用，按钮状态以位掩码的单个开/关位的形式传输。

| 字段名称                                                     | 类型       | 说明                                                         |
| ------------------------------------------------------------ | ---------- | ------------------------------------------------------------ |
| target                                                       | `uint8_t`  | 要控制的系统。                                               |
| x                                                            | `int16_t`  | X 轴，归一化为 [-1000,1000] 范围。如果值为 INT16_MAX，则表示此轴无效。一般相当于操纵杆上的向前（1000）-向后（-1000）移动和车辆的俯仰。 |
| y                                                            | `int16_t`  | Y 轴，归一化为 [-1000,1000] 范围。如果值为 INT16_MAX，则表示此轴无效。一般对应操纵杆的左（-1000）-右（1000）移动和车辆的翻滚。 |
| z                                                            | `int16_t`  | Z 轴，归一化为 [-1000,1000] 范围。如果值为 INT16_MAX，则表示此轴无效。一般对应于操纵杆上最大为 1000、最小为 -1000 的单独滑块移动和车辆的推力。正值表示正推力，负值表示负推力。 |
| r                                                            | `int16_t`  | R 轴，归一化为 [-1000,1000] 范围。如果值为 INT16_MAX，则表示该轴无效。一般相当于操纵杆的扭转，逆时针为 1000，顺时针为-1000，也相当于车辆的偏航。 |
| buttons                                                      | `uint16_t` | 与操纵杆按钮 0-15 当前状态相对应的位字段，1 表示按下，0 表示松开。最低位对应按钮 1。 |
| <span class='ext'>buttons2</span> <a href='#mav2_extension_field'>++</a> | `uint16_t` | 一个比特字段，对应操纵杆按钮 16-31 的当前状态，1 表示按下，0 表示松开。最低位对应按钮 16。 |
| <span class='ext'>enabled_extensions</span> <a href='#mav2_extension_field'>++</a> | `uint8_t`  | 将位设置为 1 表示下列扩展字段中哪个包含有效数据：位 0：俯仰，位 1：滚动，位 2：辅助 1，位 3：辅助 2，位 4：辅助 3，位 5：辅助 4，位 6：辅助 5，位 7：辅助 6 |
| <span class='ext'>s</span> <a href='#mav2_extension_field'>++</a> | `int16_t`  | 仅俯仰轴，归一化范围为 [-1000,1000]。通常对应于具有额外自由度的车辆的俯仰。如果 enabled_extensions 字段的第 0 位被设置，则该值有效。如果无效，则设置为 0。 |
| <span class='ext'>t</span> <a href='#mav2_extension_field'>++</a> | `int16_t`  | 仅滚动轴，归一化为 [-1000,1000]。一般对应于具有额外自由度的车辆滚动。如果 enabled_extensions 字段的第 1 位被设置，则有效。如果未设置，则设置为 0。 |
| <span class='ext'>aux1</span> <a href='#mav2_extension_field'>++</a> | `int16_t`  | 辅助连续输入字段 1。规范化范围 [-1000,1000]。由接收方定义。如果 enabled_extensions 字段的第 2 位已设置，则数据有效。如果第 2 位未设置，则为 0。 |
| <span class='ext'>aux2</span> <a href='#mav2_extension_field'>++</a> | `int16_t`  | 辅助连续输入字段 2。规范化范围 [-1000,1000]。由接收方定义。如果 enabled_extensions 字段的第 3 位已设置，则数据有效。如果第 3 位未设置，则为 0。 |
| <span class='ext'>aux3</span> <a href='#mav2_extension_field'>++</a> | `int16_t`  | 辅助连续输入字段 3。规范化范围 [-1000,1000]。由接收方定义。如果 enabled_extensions 字段的第 4 位已设置，则数据有效。如果第 4 位未设置，则为 0。 |
| <span class='ext'>aux4</span> <a href='#mav2_extension_field'>++</a> | `int16_t`  | 辅助连续输入字段 4。规范化范围 [-1000,1000]。由接收方定义。如果 enabled_extensions 字段的第 5 位已设置，则数据有效。如果第 5 位未设置，则为 0。 |
| <span class='ext'>aux5</span> <a href='#mav2_extension_field'>++</a> | `int16_t`  | 辅助连续输入字段 5。规范化范围 [-1000,1000]。由接收方定义。如果 enabled_extensions 字段的第 6 位已设置，则数据有效。如果第 6 位未设置，则为 0。 |
| <span class='ext'>aux6</span> <a href='#mav2_extension_field'>++</a> | `int16_t`  | 辅助连续输入字段 6。归一化范围 [-1000,1000]。由接收方定义。如果 enabled_extensions 字段的第 7 位已设置，则数据有效。如果第 7 位未设置，则为 0。 |


### RC_CHANNELS_OVERRIDE (70) 

发送至 MAV 的遥控通道 RAW 值将覆盖从遥控无线电接收到的信息。标准 PPM 调制如下： 1000 微秒 0%，2000 微秒： 100%. 个别接收器/发射器可能会违反这一规定。 请注意前 8 个信道与后续信道之间的语义差异

| 字段名称                                                     | 类型       | 单位 | 说明                                                         |
| ------------------------------------------------------------ | ---------- | ---- | ------------------------------------------------------------ |
| target_system                                                | `uint8_t`  |      | 系统 ID                                                      |
| target_component                                             | `uint8_t`  |      | 组件 ID                                                      |
| chan1_raw                                                    | `uint16_t` | us   | RC 通道 1 的值。UINT16_MAX 表示忽略此字段。值为 0 表示将此信道释放回 RC 无线电。 |
| chan2_raw                                                    | `uint16_t` | us   | RC 通道 2 值。UINT16_MAX 表示忽略此字段。值为 0 表示将此信道释放回 RC 无线电。 |
| chan3_raw                                                    | `uint16_t` | us   | RC 通道 3 值。UINT16_MAX 表示忽略此字段。值为 0 表示将此信道释放回 RC 无线电。 |
| chan4_raw                                                    | `uint16_t` | us   | RC 通道 4 值。UINT16_MAX 表示忽略此字段。值为 0 表示将此信道释放回 RC 无线电。 |
| chan5_raw                                                    | `uint16_t` | us   | RC 通道 5 值。UINT16_MAX 表示忽略此字段。值为 0 表示将此信道释放回 RC 无线电。 |
| chan6_raw                                                    | `uint16_t` | us   | RC 通道 6 值。UINT16_MAX 表示忽略此字段。值为 0 表示将此信道释放回 RC 无线电。 |
| chan7_raw                                                    | `uint16_t` | us   | RC 通道 7 值。UINT16_MAX 表示忽略此字段。值为 0 表示将此信道释放回 RC 无线电。 |
| chan8_raw                                                    | `uint16_t` | us   | RC 通道 8 值。UINT16_MAX 表示忽略此字段。值为 0 表示将此信道释放回 RC 无线电。 |
| <span class='ext'>chan9_raw</span> <a href='#mav2_extension_field'>++</a> | `uint16_t` | us   | RC 通道 9 值。值为 0 或 UINT16_MAX 表示忽略该字段。UINT16_MAX-1 表示将该信道释放回 RC 无线电。 |
| <span class='ext'>chan10_raw</span> <a href='#mav2_extension_field'>++</a> | `uint16_t` | us   | RC 通道 10 值。值为 0 或 UINT16_MAX 表示忽略该字段。UINT16_MAX-1 表示将该信道释放回 RC 无线电。 |
| <span class='ext'>chan11_raw</span> <a href='#mav2_extension_field'>++</a> | `uint16_t` | us   | RC 通道 11 值。值为 0 或 UINT16_MAX 表示忽略该字段。UINT16_MAX-1 表示将该信道释放回 RC 无线电。 |
| <span class='ext'>chan12_raw</span> <a href='#mav2_extension_field'>++</a> | `uint16_t` | us   | RC 通道 12 值。值为 0 或 UINT16_MAX 表示忽略该字段。UINT16_MAX-1 表示将该信道释放回 RC 无线电。 |
| <span class='ext'>chan13_raw</span> <a href='#mav2_extension_field'>++</a> | `uint16_t` | us   | RC 通道 13 值。值为 0 或 UINT16_MAX 表示忽略该字段。UINT16_MAX-1 表示将该信道释放回 RC 无线电。 |
| <span class='ext'>chan14_raw</span> <a href='#mav2_extension_field'>++</a> | `uint16_t` | us   | RC 通道 14 值。值为 0 或 UINT16_MAX 表示忽略该字段。UINT16_MAX-1 表示将该信道释放回 RC 无线电。 |
| <span class='ext'>chan15_raw</span> <a href='#mav2_extension_field'>++</a> | `uint16_t` | us   | RC 通道 15 值。值为 0 或 UINT16_MAX 表示忽略该字段。UINT16_MAX-1 表示将该信道释放回 RC 无线电。 |
| <span class='ext'>chan16_raw</span> <a href='#mav2_extension_field'>++</a> | `uint16_t` | us   | RC 通道 16 值。值为 0 或 UINT16_MAX 表示忽略该字段。UINT16_MAX-1 表示将该信道释放回 RC 无线电。 |
| <span class='ext'>chan17_raw</span> <a href='#mav2_extension_field'>++</a> | `uint16_t` | us   | RC 通道 17 值。值为 0 或 UINT16_MAX 表示忽略该字段。UINT16_MAX-1 表示将该信道释放回 RC 无线电。 |
| <span class='ext'>chan18_raw</span> <a href='#mav2_extension_field'>++</a> | `uint16_t` | us   | RC 通道 18 值。值为 0 或 UINT16_MAX 表示忽略该字段。UINT16_MAX-1 表示将该信道释放回 RC 无线电。 |


### MISSION_ITEM_INT (73) 

对任务项目进行编码的信息。该信息用于宣布

任务项目的存在，并在系统中设置任务项目。任务项目的单位可以是 x、y、z 米（类型：LOCAL）或 x:lat、y:lon、z:altitude。本地帧为 Z 向下、右手（NED），全局帧为 Z 向上、右手（ENU）。浮点数/整数参数（分别）中可以使用 NaN 或 INT32_MAX，表示可选值/默认值（例如，使用组件的当前纬度、偏航而不是特定值）。另请参阅 https://mavlink.io/en/services/mission.html。

| 字段名称                                                     | 类型       | 值                                    | 说明                                                         |
| ------------------------------------------------------------ | ---------- | ------------------------------------- | ------------------------------------------------------------ |
| target_system                                                | `uint8_t`  |                                       | 系统 ID                                                      |
| target_component                                             | `uint8_t`  |                                       | 组件 ID                                                      |
| seq                                                          | `uint16_t` |                                       | 航点 ID（序列号）。从 0 开始。每个航点单调递增，序列中无间隙（0,1,2,3,4）。 |
| frame                                                        | `uint8_t`  | [MAV_FRAME](#MAV_FRAME)               | 航点的坐标系。                                               |
| command                                                      | `uint16_t` | [MAV_CMD](#mav_commands)              | 航点的计划动作。                                             |
| current                                                      | `uint8_t`  |                                       | false:0, true:1                                              |
| autocontinue                                                 | `uint8_t`  |                                       | 自动继续下一个航点。0：false，1：true。设置为 false 会在项目完成后暂停任务。 |
| param1                                                       | `float`    |                                       | PARAM1，参见 [MAV_CMD](#mav_commands)枚举                    |
| param2                                                       | `float`    |                                       | PARAM2，见 [MAV_CMD](#mav_commands)枚举                      |
| param3                                                       | `float`    |                                       | PARAM3，参见 [MAV_CMD](#mav_commands)枚举                    |
| param4                                                       | `float`    |                                       | PARAM4，参见 [MAV_CMD](#mav_commands)枚举                    |
| x                                                            | `int32_t`  |                                       | PARAM5 / 局部：X 位置，单位为米 * 1e4，全局：纬度，单位为度 * 10^7 |
| y                                                            | `int32_t`  |                                       | PARAM6 / y 位置：本地：X 位置（单位：米*1e4），全局：经度（单位：度*10^7 |
| z                                                            | `float`    |                                       | PARAM7 / z 位置：全局：高度，以米为单位（相对或绝对，取决于帧。 |
| <span class='ext'>mission_type</span> <a href='#mav2_extension_field'>++</a> | `uint8_t`  | [MAV_MISSION_TYPE](#MAV_MISSION_TYPE) | Mission type.                                                |


### VFR_HUD (74) 

通常显示在固定翼飞机 HUD 上的指标。

| 字段名称    | 类型       | 单位 | 说明                                                         |
| ----------- | ---------- | ---- | ------------------------------------------------------------ |
| airspeed    | `float`    | m/s  | 以适合飞行器类型的形式显示的飞行速度。对于标准飞机，这通常是校准空速（CAS）或指示空速（IAS）--飞行员可使用其中任何一种来估计失速速度。 |
| groundspeed | `float`    | m/s  | 当前地面速度。                                               |
| heading     | `int16_t`  | deg  | 当前航向，罗盘单位（0-360，0=北）。                          |
| throttle    | `uint16_t` | %    | 当前节流阀设置（0 至 100）。                                 |
| alt         | `float`    | m    | 当前高度（MSL）。                                            |
| climb       | `float`    | m/s  | 当前爬升率。                                                 |


### COMMAND_INT (75) 

向 MAV 发送最多包含七个参数的命令，其中参数 5 和 6 为整数，其他值为浮点数。与 [COMMAND_LONG](#COMMAND_LONG)相比，[COMMAND_INT](#COMMAND_INT) 更受青睐，因为它允许指定 [MAV_FRAME](#MAV_FRAME)，用于解释高度等位置信息。在发送参数 5 和 6 中的经度和纬度数据时，[COMMAND_INT](#COMMAND_INT) 也是首选，因为它可以获得更高的精度。参数 5 和 6 将位置数据编码为缩放整数，缩放程度取决于实际命令值。浮点数/整数参数（分别）中可使用 NaN 或 INT32_MAX，以表示可选值/默认值（例如，使用组件的当前纬度、偏航而不是特定值）。命令微服务的文档见 https://mavlink.io/en/services/command.html。

| 字段名称         | 类型       | 值                       | 说明                                                         |
| ---------------- | ---------- | ------------------------ | ------------------------------------------------------------ |
| target_system    | `uint8_t`  |                          | 系统 ID                                                      |
| target_component | `uint8_t`  |                          | 组件 ID                                                      |
| frame            | `uint8_t`  | [MAV_FRAME](#MAV_FRAME)  | COMMAND 的坐标系。                                           |
| command          | `uint16_t` | [MAV_CMD](#mav_commands) | 任务项目的预定动作。                                         |
| current          | `uint8_t`  |                          | 未使用。                                                     |
| autocontinue     | `uint8_t`  | 不使用（设置为 0）。     |                                                              |
| param1           | `float`    | 无效：NaN                | PARAM1，参见 [MAV_CMD](#mav_commands)枚举                    |
| param2           | `float`    | invalid:NaN              | PARAM2，参见 [MAV_CMD](#mav_commands)枚举                    |
| param3           | `float`    | invalid:NaN              | PARAM3，参见 [MAV_CMD](#mav_commands)枚举                    |
| param4           | `float`    | invalid:NaN              | PARAM4，参见 [MAV_CMD](#mav_commands)枚举                    |
| x                | `int32_t`  | invalid:INT32_MAX        | PARAM5 / 本地：X 位置，单位为米 * 1e4，全局：纬度，单位为度 * 10^7 |
| y                | `int32_t`  | invalid:INT32_MAX        | PARAM6 / 本地：y 位置（单位：米 * 1e4），全局：经度（单位：度 * 10^7 |
| z                | `float`    | invalid:NaN              | PARAM7 / z 位置：全局：高度，以米为单位（相对或绝对，取决于帧）。 |


### COMMAND_LONG (76) 

向 MAV 发送最多包含七个参数的命令。在发送包含位置信息的[MAV_CMD](#mav_commands)命令时，通常首选[COMMAND_INT](#COMMAND_INT)；它提供了更高的精度，并允许指定[MAV_FRAME](#MAV_FRAME)（否则可能会含糊不清，尤其是高度）。命令微服务的文档地址为 https://mavlink.io/en/services/command.html。

| 字段名称         | 类型       | 值                       | 说明                                               |
| ---------------- | ---------- | ------------------------ | -------------------------------------------------- |
| target_system    | `uint8_t`  |                          | 执行命令的系统                                     |
| target_component |            |                          |                                                    |
| command          | `uint16_t` | [MAV_CMD](#mav_commands) | 命令 ID（要发送的命令）。                          |
| confirmation     | `uint8_t`  |                          | 0：首次发送此命令。1-255: 确认发送（如杀死命令）。 |
| param1           | `float`    | invalid:NaN              | 参数 1（用于特定命令）。                           |
| param2           | `float`    | invalid:NaN              | 参数 2（用于特定命令）。                           |
| param3           | `float`    | invalid:NaN              | 参数 3（用于特定命令）。                           |
| param4           | `float`    | invalid:NaN              | 参数 4（用于特定命令）。                           |
| param5           | `float`    | invalid:NaN              | 参数 5（用于特定命令）。                           |
| param6           | `float`    | invalid:NaN              | 参数 6（用于特定命令）。                           |
| param7           | `float`    | invalid:NaN              | 参数 7（用于特定命令）。                           |


### COMMAND_ACK (77) 

报告命令状态。包括命令是否已执行的反馈。命令微服务的记录在 https://mavlink.io/en/services/command.html。

| 字段名称                                                     | 类型       | 单位 | 值                        | 说明                                                         |
| ------------------------------------------------------------ | ---------- | ---- | ------------------------- | ------------------------------------------------------------ |
| command                                                      | `uint16_t` |      | [MAV_CMD](#mav_commands)  | 命令 ID（确认的命令）。                                      |
| result                                                       | `uint8_t`  |      | [MAV_RESULT](#MAV_RESULT) | 命令的结果。                                                 |
| <span class='ext'>progress</span> <a href='#mav2_extension_field'>++</a> | `uint8_t`  | %    | invalid:UINT8_MAX         | 结果为 [MAV_RESULT_IN_PROGRESS](#MAV_RESULT_IN_PROGRESS) 时的进度百分比。值： [0-100]，如果进度未知，则为 UINT8_MAX。 |
| <span class='ext'>result_param2</span> <a href='#mav2_extension_field'>++</a> | `int32_t`  |      |                           | 附加结果信息。可与特定于命令的枚举一起设置，该枚举包含特定于命令的错误原因，说明为什么命令可能会被拒绝。如果使用，相关枚举必须记录在相应的 [MAV_CMD](#mav_commands)（该枚举的值应为 0，表示 "未使用 "或 "未知"）中。 |
| <span class='ext'>target_system</span> <a href='#mav2_extension_field'>++</a> | `uint8_t`  |      |                           | 目标接收者的系统 ID。这是发送 [COMMAND_ACK](#COMMAND_ACK)为确认的命令的系统 ID。 |
| <span class='ext'>target_component </span> <a href='#mav2_extension_field'>++</a> | `uint8_t`  |      |                           | 目标接收者的组件 ID。这是发送 [COMMAND_ACK](#COMMAND_ACK)为确认的命令的系统 ID。 |


### COMMAND_CANCEL (80) — [WIP] 

<span class="warning">**WORK IN PROGRESS**: Do not use in stable production environments (it may change).</span>

取消长期运行的命令。如果长时间运行的进程被取消，目标系统应向原始命令响应一个 [COMMAND_ACK](#COMMAND_ACK)，其结果=MAV_RESULT_CANCELLED。如果进程已经完成，则可以忽略取消操作。取消操作可以重试，直到收到对原始命令的某种确认。命令微服务的文档地址是 https://mavlink.io/en/services/command.html。

| 字段名称         | 类型       | 值                       | 描述                                   |
| ---------------- | ---------- | ------------------------ | -------------------------------------- |
| target_system    | `uint8_t`  |                          | 执行长期运行命令的系统。不应广播 (0)。 |
| target_component | `uint8_t`  |                          | 执行长命令的组件。                     |
| command          | `uint16_t` | [MAV_CMD](#mav_commands) | 命令 ID（要取消的命令的 ID）。         |


### MANUAL_SETPOINT (81) 

来自操作员的滚动、俯仰、偏航和推力设定值

| 字段名称               | 类型       | 单位  | 说明                         |
| ---------------------- | ---------- | ----- | ---------------------------- |
| time_boot_ms           | `uint32_t` | ms    | 时间戳（系统启动后的时间）。 |
| roll                   | `float`    | rad/s | 预期滚动率                   |
| pitch                  | `float`    | rad/s | 预期俯仰率                   |
| yaw                    | `float`    | rad/s | 预期偏航率                   |
| thrust                 | `float`    |       | 集合推力，归一化为 0 ... 1   |
| mode_switch            | `uint8_t`  |       | 飞行模式开关位置，0...255    |
| manual_override_switch | `uint8_t`  |       | 覆盖模式开关位置，0...255    |


### SET_ATTITUDE_TARGET (82) 

设置所需的飞行器姿态。由外部控制器用于指挥飞行器（手动控制器或其他系统）。

| 字段名称                                                     | 类型       | 单位  | 数值                                                  | 说明                                                         |
| ------------------------------------------------------------ | ---------- | ----- | ----------------------------------------------------- | ------------------------------------------------------------ |
| time_boot_ms                                                 | `uint32_t` | ms    |                                                       | 时间戳（系统启动后的时间）。                                 |
| target_system                                                | `uint8_t`  |       |                                                       | 系统 ID                                                      |
| target_component                                             | `uint8_t`  |       |                                                       | 组件 ID                                                      |
| type_mask                                                    | `uint8_t`  |       | [ATTITUDE_TARGET_TYPEMASK](#ATTITUDE_TARGET_TYPEMASK) | 表示车辆应忽略哪些尺寸的位图。                               |
| q                                                            | `float[4]` |       |                                                       | 从[MAV_FRAME_LOCAL_NED](#MAV_FRAME_LOCAL_NED)到[MAV_FRAME_BODY_FRD](#MAV_FRAME_BODY_FRD)的姿态四元数（w、x、y、z 顺序，零旋转为 1, 0, 0, 0）。 |
| body_roll_rate                                               | `float`    | rad/s |                                                       |                                                              |
| body_pitch_rate                                              | `float`    | rad/s |                                                       | 机身俯仰率                                                   |
| body_yaw_rate                                                | `float`    | rad/s |                                                       | 身体偏航率                                                   |
| thrust                                                       | `float`    |       |                                                       | 集体推力，归一化为 0 ... 1（-1 ... 1 适用于可反向信任的飞行器） |
| <span class='ext'>thrust_body</span> <a href='#mav2_extension_field'>++</a> | `float[3]` |       |                                                       | 车身 NED 框架中的 3D 推力设定点，归一化为 -1 ... 1           |


### ATTITUDE_TARGET (83) 

报告自动驾驶仪指定的飞行器当前指令姿态。如果以这种方式控制飞行器，则应与 [SET_ATTITUDE_TARGET](#SET_ATTITUDE_TARGET) 信息中发送的命令一致。

| 字段名称                                                     | 类型       | 单位  | 值                                                    | 说明                                                         |
| ------------------------------------------------------------ | ---------- | ----- | ----------------------------------------------------- | ------------------------------------------------------------ |
| time_boot_ms                                                 | `uint32_t` | ms    |                                                       | 时间戳（系统启动后的时间）。                                 |
| type_mask                                                    | `uint8_t`  |       | [ATTITUDE_TARGET_TYPEMASK](#ATTITUDE_TARGET_TYPEMASK) | 表示车辆应忽略哪些尺寸的位图。                               |
| q                                                            | `float[4]` |       |                                                       | 姿态四元数（w、x、y、z 顺序，零旋转为 1、0、0、0）           |
| body_roll_rate                                               | `float`    | rad/s |                                                       | 身体滚动速率                                                 |
| body_pitch_rate                                              | `float`    | rad/s |                                                       | 身体俯仰率                                                   |
| body_yaw_rate                                                | `float`    | rad/s |                                                       | 身体偏航率                                                   |
| thrust                                                       | `float`    |       |                                                       | 集体推力，归一化为 0 ... 1（-1 ... 1 适用于可反向信任的飞行器） |
| <span class='ext'>thrust_body</span> <a href='#mav2_extension_field'>++</a> | `float[3]` |       |                                                       | 身体 NED 框架中的 3D 推力设定点，标准化为 -1 .. 1            |


### SET_POSITION_TARGET_LOCAL_NED (84) 

在当地的东北-东南坐标框架内设置所需的车辆位置。由外部控制器用于指挥车辆（手动控制器或其他系统）。

| 字段名称         | 类型       | 单位  | 数值                                                  | 说明                                                         |
| ---------------- | ---------- | ----- | ----------------------------------------------------- | ------------------------------------------------------------ |
| time_boot_ms     | `uint32_t` | ms    |                                                       | 时间戳（系统启动后的时间）。                                 |
| target_system    | `uint8_t`  |       |                                                       | 系统 ID                                                      |
| target_component | `uint8_t`  |       |                                                       | 组件 ID                                                      |
| coordinate_frame | `uint8_t`  |       | [MAV_FRAME](#MAV_FRAME)                               | 有效选项为： [mav_frame_local_ned](#mav_frame_local_ned) = 1, [mav_frame_local_offset_ned](#mav_frame_local_offset_ned) = 7, [mav_frame_body_ned](#mav_frame_body_ned) = 8, [mav_frame_body_offset_ned](#mav_frame_body_offset_ned) = 9 |
| type_mask        | `uint16_t` |       | [POSITION_TARGET_TYPEMASK](#POSITION_TARGET_TYPEMASK) | 用于指示车辆应忽略哪些尺寸的位图。                           |
| x                | `float`    | m     |                                                       | 在 NED 帧中的 X 位置                                         |
| y                | `float`    | m     |                                                       | NED 帧中的 Y 位置                                            |
| z                | `float`    | m     |                                                       | Z 在 NED 帧中的位置（注意，高度在 NED 中为负值）             |
| vx               | `float`    | m/s   |                                                       | NED 帧中的 X 速度                                            |
| vy               | `float`    | m/s   |                                                       | NED 帧中的 Y 速度                                            |
| vz               | `float`    | m/s   |                                                       | NED 帧中的 Z 速度                                            |
| afx              | `float`    | m/s/s |                                                       | NED 帧中的 X 加速度或力（如果设置了 type_mask 的第 10 位），单位为米/秒^2 或牛顿（N |
| afy              | `float`    | m/s/s |                                                       | Y 加速度或力（如果设置了 type_mask 的第 10 位），在 NED 帧中，单位为米/秒^2 或 N |
| afz              | `float`    | m/s/s |                                                       | Z 加速度或力（如果设置了 type_mask 的第 10 位），在 NED 帧中，单位为米/秒^2 或牛顿。 |
| yaw              | `float`    | rad   |                                                       | 偏航设定点                                                   |
| yaw_rate         | `float`    | rad/s |                                                       | 偏航率设定值                                                 |


### POSITION_TARGET_LOCAL_NED (85) 

报告自动驾驶仪指定的当前指令车辆位置、速度和加速度。如果以这种方式控制飞行器，则应与 [SET_POSITION_TARGET_LOCAL_NED]（#SET_POSITION_TARGET_LOCAL_NED）中发送的命令一致。

| 字段名称         | 类型       | 单位  | 值                                                    | 说明                                                         |
| ---------------- | ---------- | ----- | ----------------------------------------------------- | ------------------------------------------------------------ |
| time_boot_ms     | `uint32_t` | ms    |                                                       | 时间戳（系统启动后的时间）。                                 |
| coordinate_frame | `uint8_t`  |       | [MAV_FRAME](#MAV_FRAME)                               | 有效选项为： [mav_frame_local_ned](#mav_frame_local_ned) = 1, [mav_frame_local_offset_ned](#mav_frame_local_offset_ned) = 7, [mav_frame_body_ned](#mav_frame_body_ned) = 8, [mav_frame_body_offset_ned](#mav_frame_body_offset_ned) = 9 |
| type_mask        | `uint16_t` |       | [POSITION_TARGET_TYPEMASK](#POSITION_TARGET_TYPEMASK) | 用于指示车辆应忽略哪些尺寸的位图。                           |
| x                | `float`    | m     |                                                       | 在 NED 帧中的 X 位置                                         |
| y                | `float`    | m     |                                                       | NED 帧中的 Y 位置                                            |
| z                | `float`    | m     |                                                       | Z 在 NED 帧中的位置（注意，高度在 NED 中为负值）             |
| vx               | `float`    | m/s   |                                                       | NED 帧中的 X 速度                                            |
| vy               | `float`    | m/s   |                                                       | NED 帧中的 Y 速度                                            |
| vz               | `float`    | m/s   |                                                       | NED 帧中的 Z 速度                                            |
| afx              | `float`    | m/s/s |                                                       | NED 帧中的 X 加速度或力（如果设置了 type_mask 的第 10 位），单位为米/秒^2 或牛顿（N |
| afy              | `float`    | m/s/s |                                                       | Y 加速度或力（如果设置了 type_mask 的第 10 位），在 NED 帧中，单位为米/秒^2 或 N |
| afz              | `float`    | m/s/s |                                                       | Z 加速度或力（如果设置了 type_mask 的第 10 位），在 NED 帧中，单位为米/秒^2 或牛顿。 |
| yaw              | `float`    | rad   |                                                       | 偏航设定点                                                   |
| yaw_rate         | `float`    | rad/s |                                                       | 偏航率设定值                                                 |


### SET_POSITION_TARGET_GLOBAL_INT (86) 

在全球坐标系（WGS84）中设置所需的车辆位置、速度和/或加速度。由外部控制器用于指挥车辆（手动控制器或其他系统）。

| 字段名称         | 类型       | 单位       | 数值                    | 说明                                                         |
| ---------------- | ---------- | ---------- | ----------------------- | ------------------------------------------------------------ |
| time_boot_ms     | `uint32_t` | ms         |                         | 时间戳（系统启动后的时间）。在设定点中设置时间戳的理由是允许系统补偿设定点的传输延迟。这允许系统补偿处理延迟。 |
| target_system    | `uint8_t`  |            |                         | 系统 ID                                                      |
| target_component | `uint8_t`  |            |                         | 组件 ID                                                      |
| coordinate_frame | `uint8_t`  |            | [MAV_FRAME](#MAV_FRAME) | 有效选项为： [mav_frame_global](#mav_frame_global) = 0, [mav_frame_global_relative_alt](#mav_frame_global_relative_alt) = 3, [mav_frame_global_terrain_alt](#mav_frame_global_terrain_alt) = 10 ([mav_frame_global_int](#mav_frame_global_int)、 [MAV_FRAME_GLOBAL_RELATIVE_ALT_INT](#MAV_FRAME_GLOBAL_RELATIVE_ALT_INT)、[MAV_FRAME_GLOBAL_TERRAIN_ALT_INT](#MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) 是允许使用的同义词，但已被弃用) |
| type_mask        |            | `uint16_t` |                         | [POSITION_TARGET_TYPEMASK](#POSITION_TARGET_TYPEMASK)        |
| lat_int          | `int32_t`  | degE7      |                         | WGS84 框架中的纬度                                           |
| lon_int          | `int32_t`  | degE7      |                         | WGS84 框架中的经度                                           |
| alt              | `float`    | m          |                         | 高度（MSL、与原点的相对高度或 AGL - 取决于框架）             |
| vx               | `float`    | m/s        |                         | NED 帧中的 X 速度                                            |
| vy               | `float`    | m/s        |                         | NED 帧中的 Y 速度                                            |
| vz               | `float`    | m/s        |                         | NED 帧中的 Z 速度                                            |
| afx              | `float`    | m/s/s      |                         | NED 帧中的 X 加速度或力（如果设置了 type_mask 的第 10 位），单位为米/秒^2 或牛顿（N |
| afy              | `float`    | m/s/s      |                         | Y 加速度或力（如果设置了 type_mask 的第 10 位），在 NED 帧中，单位为米/秒^2 或 N |
| afz              | `float`    | m/s/s      |                         | Z 加速度或力（如果设置了 type_mask 的第 10 位），在 NED 帧中，单位为米/秒^2 或牛顿。 |
| yaw              | `float`    | rad        |                         | 偏航设定点                                                   |
| yaw_rate         | `float`    | rad/s      |                         | 偏航率设定值                                                 |


### POSITION_TARGET_GLOBAL_INT (87) 

报告自动驾驶仪指定的当前指令车辆位置、速度和加速度。如果以这种方式控制飞行器，则应与 [SET_POSITION_TARGET_GLOBAL_INT]（#SET_POSITION_TARGET_GLOBAL_INT）中发送的命令一致。

| 字段名称         | 类型       | 单位       | 值                      | 说明                                                         |
| ---------------- | ---------- | ---------- | ----------------------- | ------------------------------------------------------------ |
| time_boot_ms     | `uint32_t` | ms         |                         | 时间戳（系统启动后的时间）。在设定点中设置时间戳的理由是允许系统补偿设定点的传输延迟。这样，系统就可以补偿处理延迟。 |
| coordinate_frame | `uint8_t`  |            | [MAV_FRAME](#MAV_FRAME) | 有效选项如下： [mav_frame_global](#mav_frame_global) = 0, [mav_frame_global_relative_alt](#mav_frame_global_relative_alt) = 3, [mav_frame_global_terrain_alt](#mav_frame_global_terrain_alt) = 10 ([mav_frame_global_int](#mav_frame_global_int)、 [MAV_FRAME_GLOBAL_RELATIVE_ALT_INT](#MAV_FRAME_GLOBAL_RELATIVE_ALT_INT)、[MAV_FRAME_GLOBAL_TERRAIN_ALT_INT](#MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) 是允许使用的同义词，但已被弃用) |
| type_mask        |            | `uint16_t` |                         | [POSITION_TARGET_TYPEMASK](#POSITION_TARGET_TYPEMASK)        |
| lat_int          | `int32_t`  | degE7      |                         | WGS84 框架中的纬度                                           |
| lon_int          | `int32_t`  | degE7      |                         | WGS84 框架中的经度                                           |
| alt              | `float`    | m          |                         | 高度（MSL、AGL 或相对于原点高度，取决于框架）                |
| vx               | `float`    | m/s        |                         | NED 帧中的 X 速度                                            |
| vy               | `float`    | m/s        |                         | NED 帧中的 Y 速度                                            |
| vz               | `float`    | m/s        |                         | NED 帧中的 Z 速度                                            |
| afx              | `float`    | m/s/s      |                         | NED 帧中的 X 加速度或力（如果设置了 type_mask 的第 10 位），单位为米/秒^2 或牛顿（N |
| afy              | `float`    | m/s/s      |                         | Y 加速度或力（如果设置了 type_mask 的第 10 位），在 NED 帧中，单位为米/秒^2 或 N |
| afz              | `float`    | m/s/s      |                         | Z 加速度或力（如果设置了 type_mask 的第 10 位），在 NED 帧中，单位为米/秒^2 或牛顿。 |
| yaw              | `float`    | rad        |                         | 偏航设定点                                                   |
| yaw_rate         | `float`    | rad/s      |                         | 偏航率设定值                                                 |


### LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET (89) 

MAV X 的 [LOCAL_POSITION_NED]（#LOCAL_POSITION_NED）信息与 NED 坐标中的全局坐标框架之间在 X、Y、Z 和偏航方面的偏移。坐标框架为右旋，Z 轴向下（航空框架，NED / 东北向下惯例）

| 字段名称     | 类型       | 单位 | 说明                         |
| ------------ | ---------- | ---- | ---------------------------- |
| time_boot_ms | `uint32_t` | ms   | 时间戳（系统启动后的时间）。 |
| x            | `float`    | m    | X 位置                       |
| y            | `float`    | m    | Y 位置                       |
| z            | `float`    | m    | Z 位置                       |
| roll         | `float`    | rad  | Roll                         |
| pitch        | `float`    | rad  | Pitch                        |
| yaw          | `float`    | rad  | 偏航                         |


### HIL_STATE (90) — [DEP] 

<span class="warning">**DEPRECATED:** Replaced By [HIL_STATE_QUATERNION](#HIL_STATE_QUATERNION) (2013-07) — Suffers from missing airspeed fields and singularities due to Euler angles)</span>

从模拟发送到自动驾驶仪。该数据包适用于高吞吐量应用，如硬件在环仿真。

| 字段名称               | 类型       | 单位  | 说明                                                         |
| ---------------------- | ---------- | ----- | ------------------------------------------------------------ |
| time_usec              | `uint64_t` | us    | 时间戳（UNIX 时间或系统启动后的时间）。接收端可以通过检查数字的大小来推断时间戳格式（自 1.1.1970 起或自系统启动起）。 |
| roll                   | `float`    | rad   | 滚转角度                                                     |
| pitch                  | `float`    | rad   | Pitch angle（俯仰角                                          |
| yaw                    | `float`    | rad   | 偏航角                                                       |
| rollspeed              | `float`    | rad/s | 机身滚动/phi 角速度                                          |
| pitchspeed（俯仰速度） | `float`    | rad/s | 机身俯仰/θ角速度                                             |
| yawspeed               | `float`    | rad/s | 车体框架偏航/π角速度                                         |
| lat                    | `int32_t`  | degE7 | 纬度                                                         |
| lon                    | `int32_t`  | degE7 | 经度                                                         |
| alt                    | `int32_t`  | mm    | 高度                                                         |
| vx                     | `int16_t`  | cm/s  | 地面 X 速度（纬度）                                          |
| vy                     | `int16_t`  | cm/s  | 地面 Y 速度（经度）                                          |
| vz                     | `int16_t`  | cm/s  | 地面 Z 速度（高度）                                          |
| xacc                   | `int16_t`  | mG    | X 加速度                                                     |
| yacc                   | `int16_t`  | mG    | Y 加速度                                                     |
| zacc                   | `int16_t`  | mG    | Z 加速度                                                     |


### HIL_CONTROLS (91) 

从自动驾驶仪发送到模拟器。硬件在环控制输出

| 字段名称       | 类型       | 单位 | 值                    | 说明                                                         |
| -------------- | ---------- | ---- | --------------------- | ------------------------------------------------------------ |
| time_usec      | `uint64_t` | us   |                       | 时间戳（UNIX 时间或系统启动后的时间）。接收端可以通过检查数字的大小来推断时间戳格式（自 1.1.1970 起或自系统启动起）。 |
| roll_ailerons  | `float`    |      |                       | 控制输出 -1 ... 1                                            |
| pitch_elevator | `float`    |      |                       | 控制输出 -1 ... 1                                            |
| yaw_rudder     | `float`    |      |                       | 控制输出 -1 ... 1                                            |
| throttle       | `float`    |      |                       | 节流阀 0 .. 1                                                |
| aux1           | `float`    |      |                       | 辅助 1, -1 .                                                 |
| aux2           | `float`    |      |                       | 辅助 2, -1 ... 1                                             |
| aux3           | `float`    |      |                       | 辅助 3, -1 ... 1                                             |
| aux4           | `float`    |      |                       | 辅助 4, -1 .. 1                                              |
| mode           | `uint8_t`  |      | [MAV_MODE](#MAV_MODE) | 系统模式。                                                   |
| nav_mode       | `uint8_t`  |      |                       | 导航模式 ([MAV_NAV_MODE](#MAV_NAV_MODE))                     |


### HIL_RC_INPUTS_RAW (92) 

从模拟器发送到自动驾驶仪。接收到的遥控通道 RAW 值。标准 PPM 调制如下： 1000 微秒 0%，2000 微秒： 100%. 个别接收机/发射机可能会违反此规范。

| 字段名称   | 类型       | 单位 | 说明                                                         |
| ---------- | ---------- | ---- | ------------------------------------------------------------ |
| time_usec  | `uint64_t` | us   | 时间戳（UNIX 时间或系统启动后的时间）。接收端可以通过检查数字的大小来推断时间戳格式（自 1.1.1970 起或自系统启动起）。 |
| chan1_raw  | `uint16_t` | us   | RC 通道 1 值                                                 |
| chan2_raw  | `uint16_t` | us   | RC 通道 2 值                                                 |
| chan3_raw  | `uint16_t` | us   | RC 通道 3 值                                                 |
| chan4_raw  | `uint16_t` | us   | RC 通道 4 值                                                 |
| chan5_raw  | `uint16_t` | us   | RC 通道 5 值                                                 |
| chan6_raw  | `uint16_t` | us   | RC 通道 6 值                                                 |
| chan7_raw  | `uint16_t` | us   | RC 通道 7 值                                                 |
| chan8_raw  | `uint16_t` | us   | RC 通道 8 值                                                 |
| chan9_raw  | `uint16_t` | us   | RC 通道 9 值                                                 |
| chan10_raw | `uint16_t` | us   | RC 通道 10 值                                                |
| chan11_raw | `uint16_t` | us   | RC 通道 11 值                                                |
| chan12_raw | `uint16_t` | us   | RC 通道 12 值                                                |
| rssi       | `uint8_t`  |      | 接收信号强度指示器，单位/刻度取决于设备。值： [0-254]，UINT8_MAX：无效/未知。 |


### HIL_ACTUATOR_CONTROLS (93) 

从自动驾驶仪发送到模拟。硬件在环控制输出（取代 [HIL_CONTROLS](#HIL_CONTROLS)

| 字段名称  | 类型        | 单位 | 值                              | 说明                                                         |
| --------- | ----------- | ---- | ------------------------------- | ------------------------------------------------------------ |
| time_usec | `uint64_t`  | us   |                                 | 时间戳（UNIX 时间或系统启动后的时间）。接收端可以通过检查数字的大小来推断时间戳格式（自 1.1.1970 起或自系统启动起）。 |
| controls  | `float[16]` |      |                                 | 控制输出 -1 ... 1。 通道分配取决于模拟硬件。                 |
| mode      | `uint8_t`   |      | [MAV_MODE_FLAG](#MAV_MODE_FLAG) | 系统模式。包括布防状态。                                     |
| flags     | `uint64_t`  |      |                                 | 标志为位字段，1：表示使用 lockstep 进行仿真。                |


### OPTICAL_FLOW (100) 

来自流量传感器（如光学鼠标传感器）的光流量

| 字段名称                                                     | 类型       | 单位  | 描述                                                         |
| ------------------------------------------------------------ | ---------- | ----- | ------------------------------------------------------------ |
| time_usec                                                    | `uint64_t` | us    | 时间戳（UNIX 时间或系统启动后的时间）。接收端可以通过检查数字的大小来推断时间戳格式（自 1.1.1970 起或自系统启动起）。 |
| sensor_id                                                    | `uint8_t`  |       | 传感器 ID                                                    |
| flow_x                                                       | `int16_t`  | dpix  | x 传感器方向的流量                                           |
| flow_y                                                       | `int16_t`  | dpix  | y 传感器方向的流量                                           |
| flow_comp_m_x                                                | `float`    | m/s   | x 传感器方向的流量，角速度补偿                               |
| flow_comp_m_y                                                | `float`    | m/s   | Y 传感器方向的流量，角速度补偿                               |
| quality                                                      | `uint8_t`  |       | 光流质量/置信度。0：差，255：最高质量                        |
| ground_distance                                              | `float`    | m     | 地面距离。正值：已知距离。负值： 未知距离                    |
| <span class='ext'>flow_rate_x</span> <a href='#mav2_extension_field'>++</a> | `float`    | rad/s | X 轴流速                                                     |
| <span class='ext'>flow_rate_y</span> <a href='#mav2_extension_field'>++</a> | `float`    | rad/s | 绕 Y 轴的流速                                                |


### GLOBAL_VISION_POSITION_ESTIMATE (101) 

来自视觉源的全球位置/姿态估计值。

| 字段名称                                                     | 类型        | 单位 | 说明                                                         |
| ------------------------------------------------------------ | ----------- | ---- | ------------------------------------------------------------ |
| usec                                                         | `uint64_t`  | us   | 时间戳（UNIX 时间或系统启动后的时间）                        |
| x                                                            | `float`     | m    | 全局 X 位置                                                  |
| y                                                            | `float`     | m    | 全局 Y 位置                                                  |
| z                                                            | `float`     | m    | 全局 Z 位置                                                  |
| roll                                                         | `float`     | rad  | 滚动角                                                       |
| 俯仰                                                         | `float`     | rad  | 俯仰角                                                       |
| yaw                                                          | `float`     | rad  | 偏航角                                                       |
| <span class='ext'>协方差</span> <a href='#mav2_extension_field'>++</a> | `float[21]` |      | 姿态 6x6 交叉协方差矩阵右上角三角形的行-主表示（状态：x_global、y_global、z_global、roll、pitch、yaw；前六个条目为第一行，后五个条目为第二行，以此类推）。如果未知，则将 NaN 值赋值给数组中的第一个元素。 |
| <span class='ext'>重置计数器</span> <a href='#mav2_extension_field'>++</a> | `uint8_t`   |      | 估计值重置计数器。当估计值在任何维度（位置、速度、姿态、角速度）重置时，该计数器都应递增。该计数器用于外部 SLAM 系统检测到环路闭合和估计值跳变等情况。 |


### VISION_POSITION_ESTIMATE (102) 

来自视觉源的本地位置/姿态估计值。

| 字段名称                                                     | 类型        | 单位 | 说明                                                         |
| ------------------------------------------------------------ | ----------- | ---- | ------------------------------------------------------------ |
| usec                                                         | `uint64_t`  | us   | 时间戳（UNIX 时间或系统启动后的时间）                        |
| x                                                            | `float`     | m    | 本地 X 位置                                                  |
| y                                                            | `float`     | m    | 本地 Y 位置                                                  |
| z                                                            | `float`     | m    | 本地 Z 位置                                                  |
| roll                                                         | `float`     | rad  | 滚动角                                                       |
| pitch                                                        | `float`     | rad  | Pitch angle                                                  |
| yaw                                                          | `float`     | rad  | 偏航角                                                       |
| <span class='ext'>covariance</span> <a href='#mav2_extension_field'>++</a> | `float[21]` |      | 姿态 6x6 交叉协方差矩阵右上角三角形的行-主表示（状态：x、y、z、roll、pitch、yaw；前六个条目为第一行，后五个条目为第二行，以此类推）。如果未知，则将 NaN 值赋值给数组中的第一个元素。 |
| <span class='ext'>reset_counter</span> <a href='#mav2_extension_field'>++</a> | `uint8_t`   |      | 估计值重置计数器。当估计值在任何维度（位置、速度、姿态、角速度）重置时，该计数器都应递增。该计数器用于外部 SLAM 系统检测到环路闭合和估计值跳变等情况。 |


### VISION_SPEED_ESTIMATE (103) 

来自视觉源的速度估计值。

| 字段名称                                                     | 类型       | 单位 | 说明                                                         |
| ------------------------------------------------------------ | ---------- | ---- | ------------------------------------------------------------ |
| usec                                                         | `uint64_t` | us   | 时间戳（UNIX 时间或系统启动后的时间）                        |
| x                                                            | `float`    | m/s  | 全局 X 速度                                                  |
| y                                                            | `float`    | m/s  | 全局 Y 速度                                                  |
| z                                                            | `float`    | m/s  | 全局 Z 速度                                                  |
| <span class='ext'>covariance</span> <a href='#mav2_extension_field'>++</a> | `float[9]` |      | 3x3 线性速度协方差矩阵的行-主表示（状态：vx、vy、vz；前三个条目 - 第一行，等等）。如果未知，则为数组中的第一个元素赋 NaN 值。 |
| <span class='ext'>reset_counter</span> <a href='#mav2_extension_field'>++</a> | `uint8_t`  |      | 估算重置计数器。当估计值在任何维度（位置、速度、姿态、角速度）重置时，该计数器都应递增。当外部 SLAM 系统检测到循环闭合和估计值跳跃等情况时，就会使用该计数器。 |


### VICON_POSITION_ESTIMATE (104) 

来自 Vicon 运动系统信号源的全局位置估计值。

| 字段名称                                                     | 类型        | 单位 | 说明                                                         |
| ------------------------------------------------------------ | ----------- | ---- | ------------------------------------------------------------ |
| usec                                                         | `uint64_t`  | us   | 时间戳（UNIX 时间或系统启动后的时间）                        |
| x                                                            | `float`     | m    | 全局 X 位置                                                  |
| y                                                            | `float`     | m    | 全局 Y 位置                                                  |
| z                                                            | `float`     | m    | 全局 Z 位置                                                  |
| roll                                                         | `float`     | rad  | 滚动角                                                       |
| pitch                                                        | `float`     | rad  | 俯仰角                                                       |
| yaw                                                          | `float`     | rad  | 偏航角                                                       |
| <span class='ext'>covariance</span> <a href='#mav2_extension_field'>++</a> | `float[21]` |      | 6x6 姿态交叉协方差矩阵右上角三角形的行-主表示（状态：x、y、z、roll、pitch、yaw；前六个条目为第一行，后五个条目为第二行，以此类推）。如果未知，则为数组中的第一个元素赋值 NaN。 |


### HIGHRES_IMU (105) 

在 NED 车身框架内以 SI 单位表示的 IMU 读数

| 字段名称                                                     | 类型       | 单位  | 值                                                      | 说明                                                         |
| ------------------------------------------------------------ | ---------- | ----- | ------------------------------------------------------- | ------------------------------------------------------------ |
| time_usec                                                    | `uint64_t` | us    |                                                         | 时间戳（UNIX 时间或系统启动后的时间）。接收端可以通过检查数字的大小来推断时间戳格式（自 1.1.1970 起或自系统启动起）。 |
| xacc                                                         | `float`    | m/s/s |                                                         | X 加速度                                                     |
| yacc                                                         | `float`    | m/s/s |                                                         | Y 加速度                                                     |
| zacc                                                         | `float`    | m/s/s |                                                         | Z 加速度                                                     |
| xgyro                                                        | `float`    | rad/s |                                                         | 绕 X 轴的角速度                                              |
| ygyro                                                        | `float`    | rad/s |                                                         | 绕 Y 轴的角速度                                              |
| zgyro                                                        | `float`    | rad/s |                                                         | 绕 Z 轴的角速度                                              |
| xmag                                                         | `float`    | 高斯  |                                                         | X 磁场                                                       |
| ymag                                                         | `float`    | 高斯  |                                                         | Y 磁场                                                       |
| zmag                                                         | `float`    | gauss |                                                         | Z 磁场                                                       |
| abs_pressure                                                 | `float`    | hPa   |                                                         | 绝对压力                                                     |
| diff_pressure                                                | `float`    | hPa   |                                                         | 差压                                                         |
| pressure_alt                                                 | `float`    |       |                                                         | 根据压力计算出的高度                                         |
| temperature                                                  | `float`    | degC  |                                                         | 温度                                                         |
| fields_updated                                               | `uint16_t` |       | [HIGHRES_IMU_UPDATED_FLAGS](#HIGHRES_IMU_UPDATED_FLAGS) | 自上次消息后更新的字段的位图                                 |
| <span class='ext'>id</span> <a href='#mav2_extension_field'>++</a> | `uint8_t`  |       |                                                         | Id。Ids 从 0 开始编号，并映射到从 1 开始编号的 IMU（例如，IMU1 将具有 id=0 的消息）<br>具有相同值的消息来自同一来源（实例）。 |


### OPTICAL_FLOW_RAD (106) 

来自角速率流量传感器（例如 PX4FLOW 或鼠标传感器）的光流

| 字段名称               | 类型       | 单位  | 说明                                                         |
| ---------------------- | ---------- | ----- | ------------------------------------------------------------ |
| time_usec              | `uint64_t` | us    | 时间戳（UNIX 纪元时间或自系统启动以来的时间）。接收端可以通过检查数字的大小来推断时间戳格式（自 1.1.1970 或自系统启动以来）。 |
| sensor_id              | `uint8_t`  |       | 传感器 ID<br>具有相同值的消息来自同一来源（实例）。          |
| integration_time_us    | `uint32_t` | us    | 积分时间。将 integration_x 和 integration_y 除以积分时间以获得平均流量。积分时间还表示。 |
| integration_x          | `float`    | rad   | 绕 X 轴的流动（传感器 RH 绕 X 轴旋转会产生正流动。传感器沿正 Y 轴的线性运动会产生负流动。） |
| integrated_y           | `float`    | rad   | 绕 Y 轴的流动（传感器 RH 绕 Y 轴旋转会产生正流动。传感器沿正 X 轴的线性运动会产生正流动。） |
| integrated_xgyro       | `float`    | rad   | 绕 X 轴的 RH 旋转                                            |
| integrated_ygyro       | `float`    | rad   | 绕 Y 轴的 RH 旋转                                            |
| integrated_zgyro       | `float`    | rad   | 绕 Z 轴的 RH 旋转                                            |
| temperature            | `int16_t`  | cdegC | 温度                                                         |
| quality                | `uint8_t`  |       | 光流质量/置信度。0：无有效流量，255：最大质量                |
| time_delta_distance_us | `uint32_t` | us    | 自采样距离以来的时间。                                       |
| distance               | `float`    | m     | 距流场中心的距离。正值（包括零）：距离已知。负值：距离未知。 |


### HIL_SENSOR (107) 

在 NED 车身框架内以 SI 单位表示的 IMU 读数

| 字段名称                                                     | 类型       | 单位  | 值                                                    | 说明                                                         |
| ------------------------------------------------------------ | ---------- | ----- | ----------------------------------------------------- | ------------------------------------------------------------ |
| time_usec                                                    | `uint64_t` | us    |                                                       | 时间戳（UNIX 时间或系统启动后的时间）。接收端可以通过检查数字的大小来推断时间戳格式（自 1.1.1970 起或自系统启动起）。 |
| xacc                                                         | `float`    | m/s/s |                                                       | X 加速度                                                     |
| yacc                                                         | `float`    | m/s/s |                                                       | Y 加速度                                                     |
| zacc                                                         | `float`    | m/s/s |                                                       | Z 加速度                                                     |
| xgyro                                                        | `float`    | rad/s |                                                       | 身体框架内绕 X 轴的角速度                                    |
| ygyro                                                        | `float`    | rad/s |                                                       | 在本体框架内绕 Y 轴的角速度                                  |
| Zgyro                                                        | `float`    | rad/s |                                                       | 车身框架中 Z 轴的角速度                                      |
| Xmag                                                         | `float`    | gauss |                                                       | X 磁场                                                       |
| ymag                                                         | `float`    | gauss |                                                       | Y 磁场                                                       |
| zmag                                                         | `float`    | gauss |                                                       | Z 磁场                                                       |
| abs_pressure                                                 | `float`    | hPa   |                                                       | 绝对压力                                                     |
| diff_pressure                                                | `float`    | hPa   |                                                       | 压差（空速）                                                 |
| pressure_alt                                                 | `float`    |       |                                                       | 根据压力计算出的高度                                         |
| temperature                                                  | `float`    | degC  |                                                       | 温度                                                         |
| fields_updated                                               | `uint32_t` |       | [HIL_SENSOR_UPDATED_FLAGS](#HIL_SENSOR_UPDATED_FLAGS) | 自上次发送信息以来已更新字段的位图                           |
| <span class='ext'>id</span> <a href='#mav2_extension_field'>++</a> | `uint8_t`  |       |                                                       | 传感器 ID（零索引）。用于多个传感器输入                      |


### SIM_STATE (108) 

模拟环境的状态（如果使用

| 字段名称                                                     | 类型      | 单位  | 说明                                                         |
| ------------------------------------------------------------ | --------- | ----- | ------------------------------------------------------------ |
| q1                                                           | `float`   |       | 真姿态四元数分量 1，w（空旋转时为 1）                        |
| q2                                                           | `float`   |       | 真姿态四元数分量 2，x（空旋转时为 0）                        |
| q3                                                           | `float`   |       | 真姿态四元数分量 3, y（空旋转时为 0）                        |
| q4                                                           | `float`   |       | 真姿态四元数分量 4，z（空旋转时为 0）                        |
| roll                                                         | `float`   | rad   | 以欧拉角表示的姿态滚动，不建议使用，除非输出结果可供人阅读   |
| pitch                                                        | `float`   | rad   | 以欧拉角表示的姿态俯仰，不建议使用，除非输出结果可供用户阅读 |
| yaw                                                          | `float`   | rad   | 以欧拉角表示的姿态偏航，除人工可读输出外，不推荐使用。       |
| xacc                                                         | `float`   | m/s/s | X 加速度                                                     |
| yacc                                                         | `float`   | m/s/s | Y 加速度                                                     |
| zacc                                                         | `float`   | m/s/s | Z 加速度                                                     |
| xgyro                                                        | `float`   | rad/s | 绕 X 轴的角速度                                              |
| ygyro                                                        | `float`   | rad/s | 绕 Y 轴的角速度                                              |
| zgyro                                                        | `float`   | rad/s | 绕 Z 轴的角速度                                              |
| lat                                                          | `float`   | deg   | 纬度（精度较低）。该字段和 lat_int 字段都应设置。            |
| lon                                                          | `float`   | deg   | 经度（精度较低）。此字段和 lon_int 字段都应设置。            |
| alt                                                          | `float`   | m     | 高度                                                         |
| std_dev_horz                                                 | `float`   |       | 水平位置标准偏差                                             |
| std_dev_vert                                                 | `float`   |       | 垂直位置标准偏差                                             |
| vn                                                           | `float`   | m/s   | 在地球固定的 NED 框架内北向的真实速度                        |
| ve                                                           | `float`   | m/s   | 固定在地球上的 NED 框架中的东向真实速度                      |
| vd                                                           | `float`   | m/s   | 在地球固定的 NED 框架中向下方向的真实速度                    |
| <span class='ext'>lat_int</span> <a href='#mav2_extension_field'>++</a> | `int32_t` | degE7 | 纬度（精度更高）。如果为 0，收件人应使用 lat 字段值（否则首选此字段）。 |
| <span class='ext'>lon_int</span> <a href='#mav2_extension_field'>++</a> | `int32_t` | degE7 | 经度（精度更高）。如果为 0，收件人应使用 lon 字段值（否则首选此字段）。 |


### RADIO_STATUS (109) 

由无线电生成并注入 MAVLink 数据流的状态。

| 字段名称 | 类型       | 单位 | 说明                                                         |
| -------- | ---------- | ---- | ------------------------------------------------------------ |
| rssi     | `uint8_t`  |      | 本地（信息发送方）接收到的信号强度指示，单位/刻度取决于设备。值： [0-254]，UINT8_MAX：无效/未知。 |
| remrssi  | `uint8_t`  |      | 远程（信息接收器）信号强度指示，单位/刻度取决于设备。值： [0-254]，UINT8_MAX：无效/未知。 |
| txbuf    | `uint8_t`  | %    | 剩余的空闲发送缓冲空间。                                     |
| noise    | `uint8_t`  |      | 本地背景噪声电平。这些是与设备相关的 RSSI 值（在 SiK 无线电设备上约为 2x dB）。值： [0-254]，UINT8_MAX：无效/未知。 |
| remnoise | `uint8_t`  |      | 远程背景噪声电平。这些是取决于设备的 RSSI 值（在 SiK 无线电设备上约为 2x dB）。值： [0-254]，UINT8_MAX：无效/未知。 |
| rxerrors | `uint16_t` |      | 无线电数据包接收错误计数（自启动以来）。                     |
| fixed    | `uint16_t` |      | 纠正错误的无线电数据包计数（自启动以来）。                   |


### FILE_TRANSFER_PROTOCOL (110) 

文件传输协议信息： https://mavlink.io/en/services/ftp.html。

| 字段名称         | 类型           | 说明                                                         |
| ---------------- | -------------- | ------------------------------------------------------------ |
| target_network   | `uint8_t`      | 网络 ID（0 用于广播）                                        |
| target_system    | `uint8_t`      | 系统 ID（0 用于广播）                                        |
| target_component | `uint8_t`      | 组件 ID（广播时为 0）                                        |
| payload          | `uint8_t[251]` | 长度可变的有效载荷。长度由减去报文头和其他字段后的剩余报文长度决定。该块的内容/格式在 https://mavlink.io/en/services/ftp.html 中定义。 |


### TIMESYNC (111) 

时间同步信息。
该消息用于时间同步请求和响应。
请求以 `ts1=syncing 组件时间戳` 和 `tc1=0` 的形式发送，可以广播或针对特定系统/组件。
发送响应时会使用 `ts1=syncing 组件时间戳`（镜像返回不变）和 `tc1=responding 组件时间戳`，并将 `target_system` 和 `target_component` 设置为原始请求的 id。
系统可根据 `tc` 的值确定自己收到的是请求还是响应。
如果响应中的 `target_system==target_component==0` 表示远程系统尚未更新以使用组件 ID，因此无法进行可靠的时间同步；请求者可能会报错。
时间戳是以纳秒为单位的 UNIX 纪元时间或系统启动后的时间（时间戳格式可通过检查数字的大小来推断；一般来说，这并不重要，因为只使用偏移量）。
报文序列会重复多次，并对结果进行过滤/平均，以估算偏移量。

| 字段名称                                                     | 类型      | 单位 | 说明                                                         |
| ------------------------------------------------------------ | --------- | ---- | ------------------------------------------------------------ |
| tc1                                                          | `int64_t` | ns   | 时间同步时间戳 1. 同步：0： 0. 响应： 响应组件的时间戳。     |
| ts1                                                          | `int64_t` | ns   | 时间同步时间戳 2。同步组件的时间戳（在响应中镜像）。         |
| <span class='ext'>target_system</span> <a href='#mav2_extension_field'>++</a> | `uint8_t` |      | 目标系统 ID。请求：0（广播）或特定系统 ID。响应必须包含请求组件的系统 ID。 |
| <span class='ext'>target_component</span> <a href='#mav2_extension_field'>++</a> | `uint8_t` |      | 目标组件 ID。请求：0（广播）或特定组件的 ID。响应必须包含请求组件的组件 ID。 |


### CAMERA_TRIGGER (112) 

摄像机-IMU 触发和同步信息。

| 字段名称  | 类型       | 单位 | 说明                                                         |
| --------- | ---------- | ---- | ------------------------------------------------------------ |
| time_usec | `uint64_t` | us   | 图像帧的时间戳（UNIX 时间或系统启动后的时间）。接收端可以通过检查数字的大小来推断时间戳格式（自 1.1.1970 起或自系统启动起）。 |
| seq       | `uint32_t` |      | 图像帧序列                                                   |


### HIL_GPS (113) 

全球定位系统 (GPS) 返回的全球位置。这

不是系统的全球位置估计值，而是传感器的原始值。有关全球位置估计值，请参阅信息 [GLOBAL_POSITION_INT](#GLOBAL_POSITION_INT)。

| 字段名称                                                     | 类型       | 单位  | 说明                                                         |
| ------------------------------------------------------------ | ---------- | ----- | ------------------------------------------------------------ |
| time_usec                                                    | `uint64_t` | us    | 时间戳（UNIX 时间或系统启动后的时间）。接收端可以通过检查数字的大小来推断时间戳格式（自 1.1.1970 起或自系统启动起）。 |
| fix_type                                                     | `uint8_t`  |       | 0-1: no fix, 2: 2D fix, 3: 3D fix. 某些应用程序不会使用该字段的值，除非它至少是 2，因此请务必正确填写固定值。 |
| lat                                                          | `int32_t`  | degE7 | 纬度（WGS84）                                                |
| lon                                                          | `int32_t`  | degE7 | 经度（WGS84）                                                |
| alt                                                          | `int32_t`  | mm    | 高度（MSL）。正数表示向上。                                  |
| eph                                                          | `uint16_t` |       | GPS HDOP 位置水平稀释（无单位 * 100）。如果未知，则设置为 UINT16_MAX |
| epv                                                          | `uint16_t` |       | GPS VDOP 垂直位置稀释（无单位 * 100）。如果未知，则设置为 UINT16_MAX |
| vel                                                          | `uint16_t` | cm/s  | GPS 地面速度。如果未知，则设置为 UINT16_MAX                  |
| vn                                                           | `int16_t`  | cm/s  | 在地球固定的 NED 框架中 GPS 的北向速度                       |
| ve                                                           | `int16_t`  | cm/s  | 在地球固定的 NED 帧中 GPS 的东向速度                         |
| vd                                                           | `int16_t`  | cm/s  | 在地球固定的 NED 框架中 GPS 下行方向的速度                   |
| cog                                                          | `uint16_t` | cdeg  | 地面航线（不是航向，而是运动方向），0.0...359.99 度。如果未知，则设置为 UINT16_MAX |
| satellites_visible                                           | `uint8_t`  |       | 可见卫星数。如果未知，则设置为 UINT8_MAX                     |
| <span class='ext'>id</span> <a href='#mav2_extension_field'>++</a> | `uint8_t`  |       | GPS ID（零索引）。用于多个 GPS 输入                          |
| <span class='ext'>yaw</span> <a href='#mav2_extension_field'>++</a> | `uint16_t` | cdeg  | 车辆相对于地球北方的偏航，0 表示不可用，用 36000 表示北方    |


### HIL_OPTICAL_FLOW (114) 

来自流量传感器（如 PX4FLOW 或光学鼠标传感器）的模拟光流量

| 字段名称               | 类型       | 单位  | 说明                                                         |
| ---------------------- | ---------- | ----- | ------------------------------------------------------------ |
| time_usec              | `uint64_t` | us    | 时间戳（UNIX 时间或系统启动后的时间）。接收端可以通过检查数字的大小来推断时间戳格式（自 1.1.1970 起或自系统启动起）。 |
| sensor_id              | `uint8_t`  |       | 传感器 ID                                                    |
| integration_time_us    | `uint32_t` | us    | 积分时间。用积分时间除以 integrated_x 和 integrated_y，即可得到平均流量。积分时间还表示 |
| integrated_x           | `float`    | rad   | 绕 X 轴以弧度为单位的流量（传感器绕 X 轴的 RH 旋转会产生正流量。传感器沿 Y 轴正向线性运动会产生负流量）。 |
| integrated_y           | `float`    | rad   | 绕 Y 轴以弧度为单位的流量（传感器绕 Y 轴的 RH 旋转会产生正流量。传感器沿 X 轴正方向线性运动会产生正流量）。 |
| integrated_xgyro       | `float`    | rad   | 绕 X 轴的 RH 旋转                                            |
| integrated_ygyro       | `float`    | rad   | 绕 Y 轴的 RH 旋转                                            |
| integrated_zgyro       | `float`    | rad   | 相对湿度绕 Z 轴旋转                                          |
| temperature            | `int16_t`  | cdegC | 温度                                                         |
| quality                | `uint8_t`  |       | 光流质量/置信度。0：无有效光流，255：最高质量                |
| time_delta_distance_us | `uint32_t` | us    | 距离采样后的时间。                                           |
| distance               | `float`    | m     | 到流场中心的距离。正值（包括零）：已知距离。负值： 未知距离。 |


### HIL_STATE_QUATERNION (115) 

从模拟发送到自动驾驶仪，避免与 [HIL_STATE](#HIL_STATE) 奇异点形成对比。该数据包适用于高吞吐量应用，如硬件在环仿真。

| 字段名称                          | 类型       | 单位  | 说明                                                         |
| --------------------------------- | ---------- | ----- | ------------------------------------------------------------ |
| time_usec                         | `uint64_t` | us    | 时间戳（UNIX 时间或系统启动后的时间）。接收端可以通过检查数字的大小来推断时间戳格式（自 1.1.1970 起或自系统启动起）。 |
| 姿态四元数（attitude_quaternion） | `float[4]` |       | 以 w、x、y、z 顺序归一化四元数表示的飞行器姿态（1 0 0 0 为空旋转）。 |
| rollspeed                         | `float`    | rad/s | 车体框架滚动 / phi 角速度                                    |
| pitchspeed                        | `float`    | rad/s | 车体框架俯仰/θ角速度                                         |
| yawspeed                          | `float`    | rad/s | 车体框架偏航/π角速度                                         |
| lat                               | `int32_t`  | degE7 | 纬度                                                         |
| lon                               | `int32_t`  | degE7 | 经度                                                         |
| alt                               | `int32_t`  | mm    | 高度                                                         |
| vx                                | `int16_t`  | cm/s  | 地面 X 速度（纬度）                                          |
| vy                                | `int16_t`  | cm/s  | 地面 Y 速度（经度）                                          |
| vz                                | `int16_t`  | cm/s  | 地面 Z 速度（高度）                                          |
| ind_airspeed                      | `uint16_t` | cm/s  | 指示空速                                                     |
| true_airspeed                     | `uint16_t` | cm/s  | 真实空速                                                     |
| xacc                              | `int16_t`  | mG    | X 加速度                                                     |
| yacc                              | `int16_t`  | mG    | Y 加速度                                                     |
| zacc                              | `int16_t`  | mG    | Z 加速度                                                     |


### SCALED_IMU2 (116) 

辅助 9DOF 传感器设置的 RAW IMU 读数。该信息应包含按所述单位缩放的数值

| 字段名称                                                     | 类型       | 单位   | 说明                                                         |
| ------------------------------------------------------------ | ---------- | ------ | ------------------------------------------------------------ |
| time_boot_ms                                                 | `uint32_t` | ms     | 时间戳（系统启动后的时间）。                                 |
| xacc                                                         | `int16_t`  | mG     | X 加速度                                                     |
| yacc                                                         | `int16_t`  | mG     | Y 加速度                                                     |
| zacc                                                         | `int16_t`  | mG     | Z 加速度                                                     |
| xgyro                                                        | `int16_t`  | mrad/s | 绕 X 轴的角速度                                              |
| ygyro                                                        | `int16_t`  | mrad/s | 绕 Y 轴的角速度                                              |
| zgyro                                                        | `int16_t`  | mrad/s | 绕 Z 轴的角速度                                              |
| xmag                                                         | `int16_t`  | mgauss | X 磁场                                                       |
| ymag                                                         | `int16_t`  | mgauss | Y 磁场                                                       |
| zmag                                                         | `int16_t`  | mgauss | Z 磁场                                                       |
| <span class='ext'>temperature</span> <a href='#mav2_extension_field'>++</a> | `int16_t`  | cdegC  | 温度，0： IMU 不提供温度值。如果 IMU 的温度为 0C，则必须发送 1 (0.01C)。 |


### LOG_REQUEST_LIST (117) 

请求可用日志列表。在某些系统中，调用该请求可能会停止板载日志记录，直到调用 [LOG_REQUEST_END](#LOG_REQUEST_END)。如果没有可用的日志文件，则将以 id = 0 和 num_logs = 0 的 [LOG_ENTRY](#LOG_ENTRY) 消息回答此请求。

| 字段名称         | 类型       | 说明                                     |
| ---------------- | ---------- | ---------------------------------------- |
| target_system    | `uint8_t`  | 系统 ID                                  |
| target_component | `uint8_t`  | 组件 ID                                  |
| start            | `uint16_t` | 第一个日志 ID（0 表示第一个可用日志 ID） |
| end              | `uint16_t` | 最后日志 ID（0xffff 表示最后可用）       |


### LOG_ENTRY (118) 

回复 [LOG_REQUEST_LIST](#LOG_REQUEST_LIST)

| 字段名称     | 类型       | 单位  | 说明                                              |
| ------------ | ---------- | ----- | ------------------------------------------------- |
| id           | `uint16_t` |       | Log id                                            |
| num_logs     | `uint16_t` |       | 日志总数                                          |
| last_log_num | `uint16_t` |       | 高日志数                                          |
| time_utc     | `uint32_t` | s     | 自 1970 年以来日志的 UTC 时间戳，如果没有，则为 0 |
| size         | `uint32_t` | bytes | 日志大小（可能是近似值）                          |


### LOG_REQUEST_DATA (119) 

请求日志块

| 字段名称         | 类型       | 单位  | 描述                                        |
| ---------------- | ---------- | ----- | ------------------------------------------- |
| target_system    | `uint8_t`  |       | 系统 ID                                     |
| target_component | `uint8_t`  |       | 组件 ID                                     |
| id               | `uint16_t` |       | Log id (来自 [LOG_ENTRY](#LOG_ENTRY) reply) |
| os               | `uint32_t` |       | 日志中的偏移量                              |
| count            | `uint32_t` | bytes | 字节数                                      |


### LOG_DATA (120) 

回复 [LOG_REQUEST_DATA](#LOG_REQUEST_DATA)

| 字段名称 | 类型          | 单位  | 说明                                         |
| -------- | ------------- | ----- | -------------------------------------------- |
| id       | `uint16_t`    |       | Log id（来自 [LOG_ENTRY](#LOG_ENTRY) reply） |
| os       | `uint32_t`    |       | 日志中的偏移量                               |
| count    | `uint8_t`     | bytes | 字节数（日志结束时为零）                     |
| data     | `uint8_t[90]` |       | 日志数据                                     |


### LOG_ERASE (121) 

删除所有日志

| 字段名称         | 类型      | 说明    |
| ---------------- | --------- | ------- |
| target_system    | `uint8_t` | 系统 ID |
| target_component | `uint8_t` | 组件 ID |


### LOG_REQUEST_END (122) 

停止日志传输并恢复正常日志记录

| 字段名称         | 类型      | 说明    |
| ---------------- | --------- | ------- |
| target_system    | `uint8_t` | 系统 ID |
| target_component | `uint8_t` | 组件 ID |


### GPS_INJECT_DATA (123) — [DEP] 

<span class="warning">**DEPRECATED:** Replaced By [GPS_RTCM_DATA](#GPS_RTCM_DATA) (2022-05)</span>

注入机载 GPS 的数据（用于 DGPS）

| 字段名称         | 类型           | 单位  | 说明                                           |
| ---------------- | -------------- | ----- | ---------------------------------------------- |
| target_system    | `uint8_t`      |       | 系统 ID                                        |
| target_component | `uint8_t`      |       | 组件 ID                                        |
| len              | `uint8_t`      | bytes | 数据长度                                       |
| data             | `uint8_t[110]` |       | 原始数据（110 字节足够 RTCMv2 的 12 颗卫星）。 |


### GPS2_RAW (124) 

第二个 GPS 数据。

| 字段名称                                                     | 类型       | 单位      | 值                            | 说明                                                         |
| ------------------------------------------------------------ | ---------- | --------- | ----------------------------- | ------------------------------------------------------------ |
| time_usec                                                    | `uint64_t` | us        |                               | 时间戳（UNIX 时间或系统启动后的时间）。接收端可以通过检查数字的大小来推断时间戳格式（自 1.1.1970 起或自系统启动起）。 |
| fix_type                                                     | `uint8_t`  |           | [GPS_FIX_TYPE](#GPS_FIX_TYPE) | GPS 定位类型。                                               |
| lat                                                          | `int32_t`  | degE7     |                               | 纬度 (WGS84)                                                 |
| lon                                                          | `int32_t`  | degE7     |                               | 经度 (WGS84)                                                 |
| alt                                                          | `int32_t`  | mm        |                               | 高度（MSL）。正数表示向上。                                  |
| eph                                                          | `uint16_t` |           | invalid:UINT16_MAX            | GPS HDOP 水平稀释位置（无单位 * 100）。如果未知，则设置为 UINT16_MAX |
| epv                                                          | `uint16_t` |           | invalid:UINT16_MAX            | GPS VDOP 垂直位置稀释（无单位 * 100）。如果未知，则设置为 UINT16_MAX |
| vel                                                          | `uint16_t` | cm/s      | invalid:UINT16_MAX            | GPS 地面速度。如果未知，则设置为 UINT16_MAX                  |
| cog                                                          | `uint16_t` | cdeg      | invalid:UINT16_MAX            | 地面航线（不是航向，而是运动方向）： 0.0..359.99 度。如果未知，则设置为 UINT16_MAX |
| satellites_visible                                           |            | `uint8_t` |                               | invalid:UINT8_MAX                                            |
| dgps_numch                                                   | `uint8_t`  |           |                               | DGPS 卫星数                                                  |
| dgps_age                                                     | `uint32_t` | ms        |                               | DGPS 信息的年龄                                              |
| <span class='ext'>yaw</span> <a href='#mav2_extension_field'>++</a> | `uint16_t` | cdeg      | invalid:0                     | 从北开始在地球帧中偏航。如果 GPS 不提供偏航功能，则使用 0。如果 GPS 已配置为提供偏航，但目前无法提供，则使用 UINT16_MAX。使用 36000 表示北纬。 |
| <span class='ext'>alt_ellipsoid</span> <a href='#mav2_extension_field'>++</a> | `int32_t`  | mm        |                               | 高度（WGS84、EGM96 椭圆体之上）。正表示向上。                |
| <span class='ext'>h_acc</span> <a href='#mav2_extension_field'>++</a> | `uint32_t` | mm        |                               | 位置不确定。                                                 |
| <span class='ext'>v_acc</span> <a href='#mav2_extension_field'>++</a> | `uint32_t` | mm        |                               | 高度不确定。                                                 |
| <span class='ext'>vel_acc</span> <a href='#mav2_extension_field'>++</a> | `uint32_t` | mm        |                               | 速度的不确定性                                               |
| <span class='ext'>hdg_acc</span> <a href='#mav2_extension_field'>++</a> | `uint32_t` | degE5     |                               | 航向/航迹不确定性                                            |


### POWER_STATUS (125) 

电源状态

| 字段名称 | 类型       | 单位 | 值                                    | 说明               |
| -------- | ---------- | ---- | ------------------------------------- | ------------------ |
| Vcc      | `uint16_t` | mV   |                                       | 5V 轨电压。        |
| Vservo   | `uint16_t` | mV   |                                       | 伺服轨电压。       |
| flags    | `uint16_t` |      | [MAV_POWER_STATUS](#MAV_POWER_STATUS) | 电源状态标志位图。 |

### SERIAL_CONTROL (126) 

控制串行端口。它可用于原始访问机载串行外设，如 GPS 或遥测无线电。其目的是通过 MAVLink 信息更新设备固件或更改设备设置。零字节的报文仅可用于更改波特率。

| 字段名称                                                     | 类型          | 单位   | 值                                          | 说明                     |
| ------------------------------------------------------------ | ------------- | ------ | ------------------------------------------- | ------------------------ |
| device                                                       | `uint8_t`     |        | [SERIAL_CONTROL_DEV](#SERIAL_CONTROL_DEV)   | 串行控制设备类型。       |
| flags                                                        | `uint8_t`     |        | [SERIAL_CONTROL_FLAG](#SERIAL_CONTROL_FLAG) | 串行控制标志的位图。     |
| timeout                                                      | `uint16_t`    | ms     |                                             | 响应数据超时             |
| baudrate                                                     | `uint32_t`    | bits/s |                                             | 传输波特率。零表示不变。 |
| count                                                        | `uint8_t`     | bytes  |                                             | 此次传输的字节数         |
| data                                                         | `uint8_t[70]` |        |                                             | 串行数据                 |
| <span class='ext'>target_system</span> <a href='#mav2_extension_field'>++</a> | `uint8_t`     |        |                                             | System ID                |
| <span class='ext'>target_component</span> <a href='#mav2_extension_field'>++</a> | `uint8_t`     |        |                                             | Component ID             |


### GPS_RTK (127) 

RTK GPS 数据。提供 GPS 报告的相对基线计算信息

| 字段名称              | 类型       | 单位 | 值                                                           | 说明                                        |
| --------------------- | ---------- | ---- | ------------------------------------------------------------ | ------------------------------------------- |
| time_last_baseline_ms | `uint32_t` | ms   |                                                              | 上次收到基线信息的启动时间。                |
| rtk_receiver_id       | `uint8_t`  |      |                                                              | 连接的 RTK 接收机的标识。                   |
| wn                    | `uint16_t` |      |                                                              | 上次基线的 GPS 周号                         |
| tow                   | `uint32_t` | ms   |                                                              | 上次基线的 GPS 周时间                       |
| rtk_health            | `uint8_t`  |      |                                                              | RTK 数据的 GPS 特定健康状况报告。           |
| rtk_rate              | `uint8_t`  | Hz   |                                                              | GPS 接收基线信息的速率                      |
| nsats                 | `uint8_t`  |      |                                                              | 用于 RTK 计算的当前卫星数。                 |
| baseline_coords_type  | `uint8_t`  |      | [RTK_BASELINE_COORDINATE_SYSTEM](#RTK_BASELINE_COORDINATE_SYSTEM) | 基线坐标系                                  |
| baseline_a_mm         | `int32_t`  | mm   |                                                              | 以 ECEF x 或 NED north 分量表示的当前基线。 |
| baseline_b_mm         | `int32_t`  | mm   |                                                              | 当前基线（ECEF y 或 NED 东分量）。          |
| baseline_c_mm         | `int32_t`  | mm   |                                                              | ECEF z 或 NED 下分量的当前基线。            |
| accuracy              | `uint32_t` |      |                                                              | 当前基线精度估计值。                        |
| iar_num_hypotheses    | `int32_t`  |      |                                                              | 当前整数歧义假设数。                        |


### GPS2_RTK (128) 

RTK GPS 数据。提供 GPS 报告的相对基线计算信息

| 字段名称              | 类型       | 单位 | 值                                                           | 说明                                        |
| --------------------- | ---------- | ---- | ------------------------------------------------------------ | ------------------------------------------- |
| time_last_baseline_ms | `uint32_t` | ms   |                                                              | 上次收到基线信息的启动时间。                |
| rtk_receiver_id       | `uint8_t`  |      |                                                              | 连接的 RTK 接收机的标识。                   |
| wn                    | `uint16_t` |      |                                                              | 上次基线的 GPS 周号                         |
| tow                   | `uint32_t` | ms   |                                                              | 上次基线的 GPS 周时间                       |
| rtk_health            | `uint8_t`  |      |                                                              | RTK 数据的 GPS 特定健康状况报告。           |
| rtk_rate              | `uint8_t`  | Hz   |                                                              | GPS 接收基线信息的速率                      |
| nsats                 | `uint8_t`  |      |                                                              | 用于 RTK 计算的当前卫星数。                 |
| baseline_coords_type  | `uint8_t`  |      | [RTK_BASELINE_COORDINATE_SYSTEM](#RTK_BASELINE_COORDINATE_SYSTEM) | 基线坐标系                                  |
| baseline_a_mm         | `int32_t`  | mm   |                                                              | 以 ECEF x 或 NED north 分量表示的当前基线。 |
| baseline_b_mm         | `int32_t`  | mm   |                                                              | 当前基线（ECEF y 或 NED 东分量）。          |
| baseline_c_mm         | `int32_t`  | mm   |                                                              | ECEF z 或 NED 下分量的当前基线。            |
| accuracy              | `uint32_t` |      |                                                              | 当前基线精度估计值。                        |
| iar_num_hypotheses    | `int32_t`  |      |                                                              | 当前整数歧义假设数。                        |


### SCALED_IMU3 (129) 

第 3 次 9DOF 传感器设置的 RAW IMU 读数。该信息应包含按所述单位缩放的数值

| 字段名称                                                     | 类型       | 单位   | 说明                                                         |
| ------------------------------------------------------------ | ---------- | ------ | ------------------------------------------------------------ |
| time_boot_ms                                                 | `uint32_t` | ms     | 时间戳（系统启动后的时间）。                                 |
| xacc                                                         | `int16_t`  | mG     | X 加速度                                                     |
| yacc                                                         | `int16_t`  | mG     | Y 加速度                                                     |
| zacc                                                         | `int16_t`  | mG     | Z 加速度                                                     |
| xgyro                                                        | `int16_t`  | mrad/s | 绕 X 轴的角速度                                              |
| ygyro                                                        | `int16_t`  | mrad/s | 绕 Y 轴的角速度                                              |
| zgyro                                                        | `int16_t`  | mrad/s | 绕 Z 轴的角速度                                              |
| xmag                                                         | `int16_t`  | mgauss | X 磁场                                                       |
| ymag                                                         | `int16_t`  | mgauss | Y 磁场                                                       |
| zmag                                                         | `int16_t`  | mgauss | Z 磁场                                                       |
| <span class='ext'>temperature</span> <a href='#mav2_extension_field'>++</a> | `int16_t`  | cdegC  | 温度，0： IMU 不提供温度值。如果 IMU 的温度为 0C，则必须发送 1 (0.01C)。 |


### DATA_TRANSMISSION_HANDSHAKE (130) 

握手信息，用于在使用图像传输协议时启动、控制和停止图像流：https://mavlink.io/en/services/image_transmission.html。

| 字段名称    | 类型       | 单位  | 值                                                    | 说明                                                         |
| ----------- | ---------- | ----- | ----------------------------------------------------- | ------------------------------------------------------------ |
| type        | `uint8_t`  |       | [MAVLINK_DATA_STREAM_TYPE](#MAVLINK_DATA_STREAM_TYPE) | 请求/确认的数据类型。                                        |
| size        | `uint32_t` | bytes |                                                       | 总数据大小（仅在 ACK 时设置）。                              |
| width       | `uint16_t` |       |                                                       | 矩阵或图像的宽度。                                           |
| height      | `uint16_t` |       |                                                       | 矩阵或图像的高度。                                           |
| packets     | `uint16_t` |       |                                                       | 发送的数据包数量（仅在 ACK 时设置）。                        |
| payload     | `uint8_t`  | bytes |                                                       | 每个数据包的有效载荷大小（通常为 253 字节，请参阅消息 [ENCAPSULATED_DATA](#ENCAPSULATED_DATA) 中的 DATA 字段大小）（仅在 ACK 时设置）。 |
| jpg_quality | `uint8_t`  | %     |                                                       | JPEG 质量。值： [1-100].                                     |


### ENCAPSULATED_DATA (131) 

使用图像传输协议发送的图像数据包：https://mavlink.io/en/services/image_transmission.html。

| 字段名称 | 类型           | 说明                          |
| -------- | -------------- | ----------------------------- |
| seqnr    | `uint16_t`     | 序列号（每次传输都从 0 开始） |
| data     | `uint8_t[253]` | 图像数据字节数                |


### DISTANCE_SENSOR (132) 

板载测距仪的距离传感器信息。

| 字段名称                                                     | 类型       | 单位 | 值                                                | 说明                                                         |
| ------------------------------------------------------------ | ---------- | ---- | ------------------------------------------------- | ------------------------------------------------------------ |
| time_boot_ms                                                 | `uint32_t` | ms   |                                                   | 时间戳（自系统启动以来的时间）。                             |
| min_distance                                                 | `uint16_t` | cm   |                                                   | 传感器可以测量的最小距离                                     |
| max_distance                                                 | `uint16_t` | cm   |                                                   | 传感器可以测量的最大距离                                     |
| current_distance                                             | `uint16_t` | cm   |                                                   | 当前距离读数                                                 |
| type                                                         | `uint8_t`  |      | [MAV_DISTANCE_SENSOR](#MAV_DISTANCE_SENSOR)       | 距离传感器的类型。                                           |
| id                                                           | `uint8_t`  |      |                                                   | 传感器的板载 ID<br>具有相同值的消息来自同一来源（实例）。    |
| orientation                                                  | `uint8_t`  |      | [MAV_SENSOR_ORIENTATION](#MAV_SENSOR_ORIENTATION) | 传感器朝向。朝下：[ROTATION_PITCH_270](#ROTATION_PITCH_270)，朝上：[ROTATION_PITCH_90](#ROTATION_PITCH_90)，朝后：[ROTATION_PITCH_180](#ROTATION_PITCH_180)，朝前：[ROTATION_NONE](#ROTATION_NONE)，朝左：[ROTATION_YAW_90](#ROTATION_YAW_90)，朝右：[ROTATION_YAW_270](#ROTATION_YAW_270) |
| 协方差                                                       | `uint8_t`  | cm^2 | invalid:UINT8_MAX                                 | 测量方差。最大标准偏差为 6cm。如果未知，则为 UINT8_MAX。     |
| <span class='ext'>horizontal_fov</span> <a href='#mav2_extension_field'>++</a> | `float`    | rad  | invalid:0                                         | 水平视场角（角度），在距离测量有效且视场已知的情况下。否则设为 0。 |
| <span class='ext'>vertical_fov</span> <a href='#mav2_extension_field'>++</a> | `float`    | rad  | invalid:0                                         | 垂直视场角（角度），在距离测量有效且视场已知的情况下。否则设为 0。 |
| <span class='ext'>quaternion</span> <a href='#mav2_extension_field'>++</a> | `float[4]` |      | invalid:[0]                                       | 传感器在车体框架中的方向四元数（w、x、y、z 顺序，零旋转为 1、0、0、0）。零旋转是沿车体 x 轴。如果方向设置为 [MAV_SENSOR_ROTATION_CUSTOM]（#MAV_SENSOR_ROTATION_CUSTOM），则必须填写此字段。如果无效，则设为 0。 |
| <span class='ext'>signal_quality</span> <a href='#mav2_extension_field'>++</a> | `uint8_t`  | %    | invalid:0                                         | 传感器的信号质量。针对每种传感器类型，表示信号强度与目标反射率、距离、大小或长宽的关系，但以百分比表示。0 = 信号质量未知/不确定，1 = 信号无效，100 = 信号完美。 |


### TERRAIN_REQUEST (133) 

请求提供地形数据和地形状态。请参阅地形协议文档： https://mavlink.io/en/services/terrain.html

| 字段名称     | 类型       | 单位  | 说明                                                |
| ------------ | ---------- | ----- | --------------------------------------------------- |
| lat          | `int32_t`  | degE7 | 第一个网格西南角的纬度                              |
| lon          | `int32_t`  | degE7 | 第一个网格西南角的经度                              |
| grid_spacing | `uint16_t` | m     | 网格间距                                            |
| mask         | `uint64_t` |       | 请求的 4x4 网格的位掩码（行主 8x7 网格数组，56 位） |


### TERRAIN_DATA (134) 

从 GCS 发送的地形数据。纬度/水平线和网格间距必须与 [TERRAIN_REQUEST](#TERRAIN_REQUEST) 的纬度/水平线相同。请参阅地形协议文档： https://mavlink.io/en/services/terrain.html

| 字段名称                   | 类型          | 单位  | 说明                   |
| -------------------------- | ------------- | ----- | ---------------------- |
| lat                        | `int32_t`     | degE7 | 第一个网格西南角的纬度 |
| 第一个网格西南角的经度 lon | `int32_t`     | degE7 |                        |
| grid_spacing               | `uint16_t`    | m     | 网格间距               |
| gridbit                    | `uint8_t`     |       | 地形请求掩码中的位     |
| data                       | `int16_t[16]` | m     | 地形数据 MSL           |


### TERRAIN_CHECK (135) 

请求飞行器报告给定位置的地形高度（预期响应为 [TERRAIN_REPORT](#TERRAIN_REPORT)）。GCS 用来检查飞行器是否拥有任务所需的所有地形数据。

| 字段名称 | 类型      | 单位  | 说明 |
| -------- | --------- | ----- | ---- |
| lat      | `int32_t` | degE7 | 纬度 |
| lon      | `int32_t` | degE7 | 经度 |


### TERRAIN_REPORT (136) 

从无人机流式传输，用于报告地形图下载进度（由 [TERRAIN_REQUEST](#TERRAIN_REQUEST)），或作为对 [TERRAIN_CHECK](#TERRAIN_CHECK)请求的响应发送。请参阅地形协议文档： https://mavlink.io/en/services/terrain.html

| 字段名称       | 类型       | 单位  | 说明                                       |
| -------------- | ---------- | ----- | ------------------------------------------ |
| lat            | `int32_t`  | degE7 | 纬度                                       |
| lon            | `int32_t`  | degE7 | 经度                                       |
| spacing        | `uint16_t` |       | 网格间距（如果该位置的地形不可用，则为 0） |
| terrain_height | `float`    | m     | 地形高度 MSL                               |
| current_height | `float`    | m     | 车辆在纬度/纵向地形高度之上的当前高度      |
| pending        | `uint16_t` |       | 等待接收或从磁盘读取的 4x4 地形块数量      |
| loaded         | `uint16_t` |       | 内存中的 4x4 地形块数量                    |


### SCALED_PRESSURE2 (137) 

第 2 个气压计的气压读数

| 字段名称                                                     | 类型       | 单位  | 说明                                                      |
| ------------------------------------------------------------ | ---------- | ----- | --------------------------------------------------------- |
| time_boot_ms                                                 | `uint32_t` | ms    | 时间戳（系统启动后的时间）。                              |
| press_abs                                                    | `float`    | hPa   | 绝对压力                                                  |
| press_diff                                                   | `float`    | hPa   | 压差                                                      |
| temperature                                                  | `int16_t`  | cdegC | 绝对压力温度                                              |
| <span class='ext'>temperature_press_diff</span> <a href='#mav2_extension_field'>++</a> | `int16_t`  | cdegC | 压差温度（0，如果没有）。将 0（或 1）的值报告为 1 cdegC。 |


### ATT_POS_MOCAP (138) 

运动捕捉姿态和位置

| 字段名称                                                     | 类型        | 单位 | 说明                                                         |
| ------------------------------------------------------------ | ----------- | ---- | ------------------------------------------------------------ |
| time_usec                                                    | `uint64_t`  | us   | 时间戳（UNIX 时间或系统启动后的时间）。接收端可以通过检查数字的大小来推断时间戳格式（自 1.1.1970 起或自系统启动起）。 |
| q                                                            | `float[4]`  |      | 姿态四元数（w、x、y、z 顺序，零旋转为 1, 0, 0, 0）           |
| x                                                            | `float`     | m    | X 位置（NED）                                                |
| y                                                            | `float`     | m    | Y 位置 (NED)                                                 |
| z                                                            | `float`     | m    | Z 位置 (NED)                                                 |
| <span class='ext'>covariance</span> <a href='#mav2_extension_field'>++</a> | `float[21]` |      | 姿势 6x6 交叉协方差矩阵右上角三角形的行-主表示（状态：x、y、z、roll、pitch、yaw；前六个条目为第一行，后五个条目为第二行，以此类推）。如果未知，则将 NaN 值赋值给数组中的第一个元素。 |


### SET_ACTUATOR_CONTROL_TARGET (139) 

设置车辆姿态和车身角速度。

| 字段名称         | 类型       | 单位 | 说明                                                         |
| ---------------- | ---------- | ---- | ------------------------------------------------------------ |
| time_usec        | `uint64_t` | us   | 时间戳（UNIX 时间或系统启动后的时间）。接收端可以通过检查数字的大小来推断时间戳格式（自 1.1.1970 起或自系统启动起）。 |
| group_mlx        | `uint8_t`  |      | 执行器组。_mlx "表示这是一条多实例报文，MAVLink 分析器应使用该字段区分实例。 |
| target_system    | `uint8_t`  |      | 系统 ID                                                      |
| target_component | `uint8_t`  |      | 组件 ID                                                      |
| controls         | `float[8]` |      | 执行器控制。取值范围为-1...+1，其中 0 为中位。单向旋转电机的节流范围为 0...1，反向旋转电机的节流范围为负数。姿态控制（0 组）的标准映射：（索引 0-7）：滚转、俯仰、偏航、油门、襟翼、扰流板、空气制动器、起落架。加载直通混合器可将它们重新用作通用输出。 |


### ACTUATOR_CONTROL_TARGET (140) 

设置车辆姿态和车身角速度。

| 字段名称  | 类型       | 单位 | 说明                                                         |
| --------- | ---------- | ---- | ------------------------------------------------------------ |
| time_usec | `uint64_t` | us   | 时间戳（UNIX 时间或系统启动后的时间）。接收端可以通过检查数字的大小来推断时间戳格式（自 1.1.1970 起或自系统启动起）。 |
| group_mlx | `uint8_t`  |      | 执行器组。_mlx "表示这是一条多实例报文，MAVLink 分析器应使用该字段区分实例。 |
| controls  | `float[8]` |      | 执行器控制。取值范围为-1...+1，其中 0 为中位。单向旋转电机的节流范围为 0...1，反向旋转电机的节流范围为负数。姿态控制（0 组）的标准映射：（索引 0-7）：滚转、俯仰、偏航、油门、襟翼、扰流板、空气制动器、起落架。加载直通混合器可将它们重新用作通用输出。 |


### ALTITUDE (141) 

当前系统高度。

| 字段名称           | 类型       | 单位 | 说明                                                         |
| ------------------ | ---------- | ---- | ------------------------------------------------------------ |
| time_usec          | `uint64_t` | us   | 时间戳（UNIX 时间或系统启动后的时间）。接收端可以通过检查数字的大小来推断时间戳格式（自 1.1.1970 起或自系统启动起）。 |
| altitude_monotonic | `float`    | m    | 该高度值在系统启动时初始化，并且是单调的（永不重置，但代表本地高度变化）。该字段的唯一保证是永远不会被重置，并且在一次飞行中保持一致。该字段的推荐值是启动时未经校正的气压高度。该高度在不同的飞行中也会漂移和变化。 |
| altitude_amsl      | `float`    | m    | 此高度值严格高于平均海平面，可能是非单调的（可能会在 GPS 锁定或设置新的 QNH 值时重置）。它应该是全球高度航点与之比较的高度。请注意，它**不是 GPS 高度，不过，大多数 GPS 模块已经默认输出 MSL 而不是 WGS84 高度。 |
| altitude_local     | `float`    | m    | 这是本地坐标框架中的本地高度。它不是原点上方的高度，而是参照坐标原点（0, 0, 0）的高度。它是上正值。 |
| altitude_relative  | `float`    | m    | 这是原点上方的高度。每次改变当前的原点位置时，它都会重置。   |
| altitude_terrain   | `float`    | m    | 这是高于地形的高度。它可能由地形数据库或高度计提供。小于 -1000 的值应解释为未知值。 |
| bottom_clearance   | `float`    | m    | 这不是高度，而是根据融合净空估算得出的系统下方净空。一般来说，它的最大值应为激光高度计等的最大测距。它通常是一个移动目标。负值表示没有可用的测量值。 |


### RESOURCE_REQUEST (142) 

自动驾驶仪请求资源（文件、二进制数据或其他数据类型）

| 字段名称      | 类型           | 描述                                                         |
| ------------- | -------------- | ------------------------------------------------------------ |
| request_id    | `uint8_t`      | 请求 ID。回传 URI 内容时应重复使用此 ID                      |
| uri_type      | `uint8_t`      | 请求的 URI 类型。0 = 通过 URL 发送的文件。1 = UAVCAN 二进制文件 |
| uri           | `uint8_t[120]` | 请求的唯一资源标识符（URI）。不一定是直接的域名（取决于 URI 类型枚举） |
| transfer_type | `uint8_t`      | 自动驾驶仪接收 URI 的方式。0 = MAVLink FTP。1 = 二进制流。   |
| storage       | `uint8_t[120]` | 自动驾驶仪希望存储 URI 的存储路径。只有当传输类型（transfer_type）与存储相关联（例如 MAVLink FTP）时才有效。 |


### SCALED_PRESSURE3 (143) 

第三气压计的气压读数

| 字段名称                                                     | 类型       | 单位     | 说明                                                      |
| ------------------------------------------------------------ | ---------- | -------- | --------------------------------------------------------- |
| time_boot_ms                                                 | `uint32_t` | ms       | 时间戳（系统启动后的时间）。                              |
| press_abs                                                    | `float`    | 绝对压力 |                                                           |
| press_diff                                                   | `float`    | hPa      | 压差                                                      |
| temperature                                                  | `int16_t`  | cdegC    | 绝对压力温度                                              |
| <span class='ext'>temperature_press_diff</span> <a href='#mav2_extension_field'>++</a> | `int16_t`  | cdegC    | 压差温度（0，如果没有）。将 0（或 1）的值报告为 1 cdegC。 |


### FOLLOW_TARGET (144) 

来自指定系统的当前运动信息

| 字段名称         | 类型       | 单位  | 说明                                                         |
| ---------------- | ---------- | ----- | ------------------------------------------------------------ |
| timestamp        | `uint64_t` | ms    | 时间戳（系统启动后的时间）。                                 |
| est_capabilities | `uint8_t`  |       | 追踪器报告能力的位位置（POS = 0、VEL = 1、ACCEL = 2、ATT + RATES = 3） |
| lat              | `int32_t`  | degE7 | 纬度 (WGS84)                                                 |
| lon              | `int32_t`  | degE7 | 经度（WGS84）                                                |
| alt              | `float`    | m     | 高度（MSL）                                                  |
| vel              | `float[3]` | m/s   | 未知目标速度 (0,0,0)                                         |
| acc              | `float[3]` | m/s/s | 未知目标线性加速度 (0,0,0)                                   |
| attitude_q       | `float[4]` |       | （0 0 0 0，为未知数）                                        |
| rates            | `float[3]` |       | (0 0 0 未知)                                                 |
| position_cov     | `float[3]` |       | eph epv                                                      |
| custom_state     | `uint64_t` |       | 追踪器设备的按钮状态或开关                                   |


### CONTROL_SYSTEM_STATE (146) 

平滑、单调的系统状态，用于为系统控制回路提供信息。

| 字段名称     | 类型       | 单位  | 说明                                                         |
| ------------ | ---------- | ----- | ------------------------------------------------------------ |
| time_usec    | `uint64_t` | us    | 时间戳（UNIX 时间或系统启动后的时间）。接收端可以通过检查数字的大小来推断时间戳格式（自 1.1.1970 起或自系统启动起）。 |
| x_acc        | `float`    | m/s/s | 体帧中的 X 加速度                                            |
| y_acc        | `float`    | m/s/s | 车身框架内的 Y 加速度                                        |
| Z_acc        | `float`    | m/s/s | 车身框架中的 Z 加速度                                        |
| x_vel        | `float`    | m/s   | 身体框架中的 X 速度                                          |
| y_vel        | `float`    | m/s   | 身体框架中的 Y 速度                                          |
| Z_vel        | `float`    | m/s   | 车身框架中的 Z 速度                                          |
| x_pos        | `float`    | m     | 本帧中的 X 位置                                              |
| y_pos        | `float`    | m     | 本帧中的 Y 位置                                              |
| z_pos        | `float`    | m     | 本帧中的 Z 位置                                              |
| airspeed     | `float`    | m/s   | 空速，如果未知，则设为 -1                                    |
| vel_variance | `float[3]` |       | 身体速度估计值的方差                                         |
| pos_variance | `float[3]` |       | 本地位置的方差                                               |
| q            | `float[4]` |       | 以四元数表示的姿态                                           |
| roll_rate    | `float`    | rad/s | 滚轴角速度                                                   |
| pitch_rate   | `float`    | rad/s | 在俯仰轴上的角速度                                           |
| yaw_rate     | `float`    | rad/s | 偏航轴的角速度                                               |


### BATTERY_STATUS (147) 

电池信息。向 GCS 更新飞行控制器的电池状态。智能电池也使用此信息，但可能会额外发送 [BATTERY_INFO](#BATTERY_INFO)。

| 字段名称                                                     | 类型           | 单位  | 值                                                    | 说明                                                         |
| ------------------------------------------------------------ | -------------- | ----- | ----------------------------------------------------- | ------------------------------------------------------------ |
| id                                                           | `uint8_t`      |       |                                                       | Battery ID<br>具有相同值的信息来自同一来源（实例）。         |
| battery_function                                             | `uint8_t`      |       | [MAV_BATTERY_FUNCTION](#MAV_BATTERY_FUNCTION)         | 电池的功能                                                   |
| type                                                         | `uint8_t`      |       | [MAV_BATTERY_TYPE](#MAV_BATTERY_TYPE)                 | 电池的类型（化学）。                                         |
| temperature                                                  | `int16_t`      | cdegC | invalid:INT16_MAX                                     | 电池的温度。INT16_MAX表示未知温度。                          |
| voltages（电压）                                             | `uint16_t[10]` | mV    | invalid:[UINT16_MAX]                                  | 1 至 10 号电池单元的电池电压（11 至 14 号电池单元参见 voltages_ext）。该字段中高于电池有效节数的电池单元应具有 UINT16_MAX 值。如果该电池的单个电池单元电压未知或未测量，则应将电池总电压填入 0 号单元，其他单元均设置为 UINT16_MAX。如果电池电压大于 (UINT16_MAX -1)，则 0 号电池单元应设置为 (UINT16_MAX -1)，1 号电池单元应设置为剩余电压。如果总电压大于 2 * (UINT16_MAX - 1)，则可以扩展到多个电池单元。 |
| current_battery                                              | `int16_t`      | cA    | invalid:-1                                            | 电池电流，-1：自动驾驶仪不测量电流。                         |
| current_consumed                                             | `int32_t`      | mAh   | invalid:-1                                            | 消耗的电量，-1：自动驾驶仪不提供消耗估计值                   |
| energy_consumed                                              | `int32_t`      | hJ    | invalid:-1                                            | 已消耗能量，-1：自动驾驶仪不提供能量消耗估计值               |
| battery_remaining                                            | `int8_t`       | %     | invalid:-1                                            | 剩余电池能量。值： [0-100]，-1：自动驾驶仪不估算剩余电量。   |
| <span class='ext'>time_remaining</span> <a href='#mav2_extension_field'>++</a> | `int32_t`      | s     | invalid:0                                             | 剩余电池时间，0：自动驾驶仪不提供剩余电池时间估计值          |
| <span class='ext'>charge_state</span> <a href='#mav2_extension_field'>++</a> | `uint8_t`      |       | [MAV_BATTERY_CHARGE_STATE](#MAV_BATTERY_CHARGE_STATE) | 放电范围状态，由自动驾驶仪提供，用于警告或外部反应           |
| <span class='ext'>voltages_ext</span> <a href='#mav2_extension_field'>++</a> | `uint16_t[4]`  | mV    | invalid:[0]                                           | 11 至 14 号电池的电池电压。高于该电池有效电池数的电池单元应为 0，0 表示不支持（注意，这与电压字段不同，允许空字节截断）。如果测量值为 0，则应发送 1。 |
| <span class='ext'>mode</span> <a href='#mav2_extension_field'>++</a> | `uint8_t`      |       | [MAV_BATTERY_MODE](#MAV_BATTERY_MODE)                 | 电池模式。默认值（0）为不支持电池模式报告或电池处于正常使用模式。 |
| <span class='ext'>fault_bitmask</span> <a href='#mav2_extension_field'>++</a> | `uint32_t`     |       | [MAV_BATTERY_FAULT](#MAV_BATTERY_FAULT)               | 故障/健康指示。当 charge_state 为 [MAV_BATTERY_CHARGE_STATE_FAILED](#MAV_BATTERY_CHARGE_STATE_FAILED) 或 [MAV_BATTERY_CHARGE_STATE_UNHEALTHY](#MAV_BATTERY_CHARGE_STATE_UNHEALTHY) 时，应设置这些指示（否则不支持故障报告）。 |


### AUTOPILOT_VERSION (148) 

自动驾驶软件的版本和功能。应在收到 [MAV_CMD_REQUEST_MESSAGE]（#MAV_CMD_REQUEST_MESSAGE）请求时发出。

| 字段名称                                                     | 类型          | 值                                                  | 说明                                                         |
| ------------------------------------------------------------ | ------------- | --------------------------------------------------- | ------------------------------------------------------------ |
| capabilities                                                 | `uint64_t`    | [MAV_PROTOCOL_CAPABILITY](#MAV_PROTOCOL_CAPABILITY) | 能力的位图                                                   |
| flight_sw_version                                            | `uint32_t`    |                                                     | 固件版本号                                                   |
| middleware_sw_version                                        | `uint32_t`    |                                                     | 中间件版本号                                                 |
| os_sw_version                                                | `uint32_t`    |                                                     | 操作系统版本号                                               |
| board_version                                                | `uint32_t`    |                                                     | 硬件/电路板版本（最后 8 位应是硅 ID（如有））。该字段的前 16 位指定 https://github.com/PX4/PX4-Bootloader/blob/master/board_types.txt |
| flight_custom_version                                        | `uint8_t[8]`  |                                                     | 自定义版本字段，通常是 git 哈希值的前 8 个字节。这不是唯一标识符，但即使对于非常大的代码库，也能使用主版本号识别提交。 |
| middleware_custom_version                                    | `uint8_t[8]`  |                                                     | 自定义版本字段，通常是 git 哈希值的前 8 个字节。这不是唯一标识符，但即使对于非常大的代码库，也能使用主版本号识别提交。 |
| os_custom_version                                            | `uint8_t[8]`  |                                                     | 自定义版本字段，通常是 git 哈希值的前 8 个字节。这不是唯一标识符，但即使对于非常大的代码库，也能使用主版本号识别提交。 |
| vendor_id                                                    | `uint16_t`    |                                                     | 板卡供应商的 ID                                              |
| product_id                                                   | `uint16_t`    |                                                     | 产品 ID                                                      |
| uid                                                          | `uint64_t`    |                                                     | UID（如果硬件提供）（参见 uid2                               |
| <span class='ext'>uid2</span> <a href='#mav2_extension_field'>++</a> | `uint8_t[18]` |                                                     | UID（如果硬件提供）（取代 uid 字段。如果该字段非零，则使用该字段，否则使用 uid) |


### LANDING_TARGET (149) 

着陆目标的位置。参见： https://mavlink.io/en/services/landing_target.html

| 字段名称                                                     | 类型       | 单位 | 值                                          | 说明                                                         |
| ------------------------------------------------------------ | ---------- | ---- | ------------------------------------------- | ------------------------------------------------------------ |
| time_usec                                                    | `uint64_t` | us   |                                             | 时间戳（UNIX 时间或系统启动后的时间）。接收端可以通过检查数字的大小来推断时间戳格式（自 1.1.1970 起或自系统启动起）。 |
| target_num                                                   | `uint8_t`  |      |                                             | 如果存在多个目标，则是目标的 ID                              |
| frame                                                        | `uint8_t`  |      | [MAV_FRAME](#MAV_FRAME)                     | 用于以下字段的坐标框架。                                     |
| angle_x                                                      | `float`    | rad  |                                             | 目标物与图像中心的 X 轴角度偏移量                            |
| angle_y                                                      | `float`    | rad  |                                             | 目标物与图像中心的 Y 轴角度偏移                              |
| distance                                                     | `float`    | m    |                                             | 目标与车辆的距离                                             |
| size_x                                                       | `float`    | rad  |                                             | 目标沿 x 轴的尺寸                                            |
| size_y                                                       | `float`    | rad  |                                             | 沿 y 轴的目标尺寸                                            |
| <span class='ext'>x</span> <a href='#mav2_extension_field'>++</a> | `float`    | m    |                                             | X 着陆目标在 [MAV_FRAME](#MAV_FRAME) 中的位置                |
| <span class='ext'>y</span> <a href='#mav2_extension_field'>++</a> | `float`    | m    |                                             | Y 着陆目标在 [MAV_FRAME](#MAV_FRAME) 中的位置                |
| <span class='ext'>z</span> <a href='#mav2_extension_field'>++</a> | `float`    | m    |                                             | 着陆目标在 [MAV_FRAME](#MAV_FRAME) 中的 Z 位置               |
| <span class='ext'>q</span> <a href='#mav2_extension_field'>++</a> | `float[4]` |      |                                             | 着陆目标方向的四元数（w、x、y、z 顺序，零旋转为 1、0、0、0） |
| <span class='ext'>type</span> <a href='#mav2_extension_field'>++</a> | `uint8_t`  |      | [LANDING_TARGET_TYPE](#LANDING_TARGET_TYPE) | 着陆目标类型                                                 |
| <span class='ext'>position_valid</span> <a href='#mav2_extension_field'>++</a> | `uint8_t`  |      | invalid:0                                   | 布尔值，表示位置字段（x、y、z、q、类型）是否包含有效的目标位置信息（有效：1，无效：0）。默认为 0（无效）。 |


### FENCE_STATUS (162) 

地理围栏的状态。启用围栏时在扩展状态流中发送。

| 字段名称                                                     | 类型       | 单位 | 值                                | 说明                                                     |
| ------------------------------------------------------------ | ---------- | ---- | --------------------------------- | -------------------------------------------------------- |
| breach_status                                                | `uint8_t`  |      |                                   | 突破状态（如果当前在围栏内则为 0，如果在围栏外则为 1）。 |
| breach_count                                                 | `uint16_t` |      |                                   | 栅栏被破坏的次数。                                       |
| breach_type                                                  | `uint8_t`  |      | [FENCE_BREACH](#FENCE_BREACH)     | 最后违反类型。                                           |
| breach_time                                                  | `uint32_t` | ms   |                                   | 上次破坏的时间（启动后）。                               |
| <span class='ext'>breach_mitigation</span> <a href='#mav2_extension_field'>++</a> | `uint8_t`  |      | [FENCE_MITIGATE](#FENCE_MITIGATE) | 防止围栏被破坏的主动行动                                 |


### MAG_CAL_REPORT (192) 

报告已完成的指南针校准结果。发送直至收到 [MAG_CAL_ACK](#MAG_CAL_ACK)。

| 字段名称                                                     | 类型      | 单位   | 值                                                | 说明                                                         |
| ------------------------------------------------------------ | --------- | ------ | ------------------------------------------------- | ------------------------------------------------------------ |
| compass_id                                                   | `uint8_t` |        |                                                   | 正在校准的指南针。<br>具有相同值的消息来自同一来源（实例）。 |
| cal_mask                                                     | `uint8_t` |        |                                                   | 正在校准的指南针的位掩码。                                   |
| cal_status                                                   | `uint8_t` |        | [MAG_CAL_STATUS](#MAG_CAL_STATUS)                 | 校准状态。                                                   |
| autosaved                                                    | `uint8_t` |        |                                                   | 0=需要 [MAV_CMD_DO_ACCEPT_MAG_CAL](#MAV_CMD_DO_ACCEPT_MAG_CAL)，1=保存到参数。 |
| fitness                                                      | `float`   | mgauss |                                                   | RMS 毫高斯残差。                                             |
| ofs_x                                                        | `float`   |        |                                                   | X 偏移。                                                     |
| ofs_y                                                        | `float`   |        |                                                   | Y 偏移。                                                     |
| ofs_z                                                        | `float`   |        |                                                   | Z 偏移。                                                     |
| diag_x                                                       | `float`   |        |                                                   | X 对角线（矩阵 11）。                                        |
| diag_y                                                       | `float`   |        |                                                   | Y 对角线（矩阵 22）。                                        |
| diag_z                                                       | `float`   |        |                                                   | Z 对角线（矩阵 33）。                                        |
| offdiag_x                                                    | `float`   |        |                                                   | X 非对角线（矩阵 12 和 21）。                                |
| offdiag_y                                                    | `float`   |        |                                                   | Y 非对角线（矩阵 13 和 31）。                                |
| offdiag_z                                                    | `float`   |        |                                                   | Z 非对角线（矩阵 32 和 23）。                                |
| <span class='ext'>orientation_confidence</span> <a href='#mav2_extension_field'>++</a> | `float`   |        |                                                   | 对方向的信心（越高越好）。                                   |
| <span class='ext'>old_orientation</span> <a href='#mav2_extension_field'>++</a> | `uint8_t` |        | [MAV_SENSOR_ORIENTATION](#MAV_SENSOR_ORIENTATION) | 校准前的方向。                                               |
| <span class='ext'>new_orientation</span> <a href='#mav2_extension_field'>++</a> | `uint8_t` |        | [MAV_SENSOR_ORIENTATION](#MAV_SENSOR_ORIENTATION) | 校准后的方向。                                               |
| <span class='ext'>scale_factor</span> <a href='#mav2_extension_field'>++</a> | `float`   |        |                                                   | 场半径修正系数                                               |


### EFI_STATUS (225) 

EFI 状态输出

| 字段名称                                                     | 类型      | 单位     | 说明                                                         |
| ------------------------------------------------------------ | --------- | -------- | ------------------------------------------------------------ |
| health                                                       | `uint8_t` |          | EFI 健康状态                                                 |
| ecu_index                                                    |           | `float`  |                                                              |
| rpm                                                          | `float`   |          | RPM                                                          |
| fuel_consumed                                                | `float`   | cm^3     | 耗油量                                                       |
| fuel_flow                                                    | `float`   | cm^3/min | 燃油流量                                                     |
| engine_load                                                  | `float`   | %        | 发动机负载                                                   |
| throttle_position                                            | `float`   | %        | 节气门位置                                                   |
| spark_dwell_time                                             | `float`   | ms       | 火花停留时间                                                 |
| barometric_pressure                                          | `float`   | kPa      | 气压                                                         |
| intake_manifold_pressure                                     | `float`   | kPa      | 进气歧管压力（                                               |
| intake_manifold_temperature                                  | `float`   | degC     | 进气歧管温度                                                 |
| cylinder_head_temperature                                    | `float`   | degC     | 气缸盖温度                                                   |
| ignition_timing                                              | `float`   | deg      | 点火时间（曲柄角度度数）                                     |
| injection_time                                               | `float`   | ms       | 喷油时间                                                     |
| exhaust_gas_temperature                                      | `float`   | degC     | 废气温度                                                     |
| throttle_out                                                 | `float`   | %        | 输出节流阀                                                   |
| pt_compensation                                              | `float`   |          | 压力/温度补偿                                                |
| <span class='ext'>ignition_voltage</span> <a href='#mav2_extension_field'>++</a> | `float`   | V        | 电喷火花系统的供电电压。 该值中的零表示 "未知"，因此如果电源电压确实为零，则使用 0.0001 代替。 |
| <span class='ext'>fuel_pressure</span> <a href='#mav2_extension_field'>++</a> | `float`   | kPa      | 燃油压力。该值中的 0 表示 "未知"，因此如果燃油压力确实为 0 kPa，请使用 0.0001。 |


### ESTIMATOR_STATUS (230) 

估计器状态信息，包括标志、创新测试比率和估计精度。标志信息是一个整数位掩码，包含哪些 EKF 输出有效的信息。更多信息请参阅 [ESTIMATOR_STATUS_FLAGS](#ESTIMATOR_STATUS_FLAGS) 枚举定义。创新测试比率显示传感器创新的幅度除以创新检查阈值。在正常运行情况下，创新测试比率应低于 0.5，偶尔会达到 1.0。在正常运行情况下，大于 1.0 的值应该很少见，这表示滤波器拒绝接受测量。如果记录的创新测试比率大于 1.0，则应通知用户。对于 0.5 至 1.0 之间的数值，用户可以选择是否发出通知。

| 字段名称           | 类型       | 单位 | 数值                                              | 说明                                                         |
| ------------------ | ---------- | ---- | ------------------------------------------------- | ------------------------------------------------------------ |
| time_usec          | `uint64_t` | us   |                                                   | 时间戳（UNIX 时间或系统启动后的时间）。接收端可以通过检查数字的大小来推断时间戳格式（自 1.1.1970 起或自系统启动起）。 |
| flags              | `uint16_t` |      | [ESTIMATOR_STATUS_FLAGS](#ESTIMATOR_STATUS_FLAGS) | 表示哪些 EKF 输出有效的位图。                                |
| vel_ratio          | `float`    |      |                                                   | 速度创新测试比率                                             |
| pos_horiz_ratio    |            |      |                                                   | 水平位置创新测试比率                                         |
| pos_vert_ratio     | `float`    |      | 垂直位置创新测试比率                              |                                                              |
| mag_ratio          | `float`    |      |                                                   | 磁强计创新测试比率                                           |
| hagl_ratio         | `float`    |      |                                                   | 高于地形的高度 创新测试比率                                  |
| tas_ratio          | `float`    |      |                                                   | 真实空速创新测试比                                           |
| pos_horiz_accuracy | `float`    | m    |                                                   | 相对于 EKF 本地原点的水平位置 1-STD 精确度                   |
| pos_vert_accuracy  | `float`    | m    |                                                   | 相对于 EKF 本地原点的垂直位置 1-STD 精确度                   |


### WIND_COV (231) 

车辆风速估计值。请注意，尽管名称如此，该信息实际上并不包含任何协方差，而是包含以标准偏差（1-STD）表示的变异性和准确性字段。

| 字段名称       | 类型       | 单位 | 说明                                                         |
| -------------- | ---------- | ---- | ------------------------------------------------------------ |
| time_usec      | `uint64_t` | us   | 时间戳（UNIX 时间或系统启动后的时间）。接收端可以通过检查数字的大小来推断时间戳格式（自 1.1.1970 起或自系统启动起）。 |
| wind_x         | `float`    | m/s  | 北风（NED）方向的风速（未知时为 NAN）                        |
| wind_y         | `float`    | m/s  | 东风（NED）方向（未知时为 NAN）                              |
| wind_z         | `float`    | m/s  | 下风（NED）方向（未知时为 NAN）                              |
| var_horiz      | `float`    | m/s  | 根据 1 Hz 低通滤波风速估计值估算的 XY、1-STD 方向风速变化率（如未知，则为 NAN） |
| var_vert       | `float`    | m/s  | Z 方向风速变率，根据 1 Hz 低通滤波风速估算得出的 1-STD 值（未知时为 NAN 值） |
| wind_alt       | `float`    | m    | 本次测量的高度（MSL）（未知时为 NAN）                        |
| horiz_accuracy | `float`    | m/s  | 水平速度 1-STD 精确度（未知时为 0）                          |
| vert_accuracy  | `float`    | m/s  | 垂直速度 1-STD 精确度（未知时为 0）                          |


### GPS_INPUT (232) 

GPS 传感器输入信息。 这是 GPS 发送的原始传感器值。这不是系统的全球位置估计值。

| 字段名称                                                     | 类型       | 单位  | 值                                                | 说明                                                         |
| ------------------------------------------------------------ | ---------- | ----- | ------------------------------------------------- | ------------------------------------------------------------ |
| time_usec                                                    | `uint64_t` | us    |                                                   | 时间戳（UNIX 时间或系统启动后的时间）。接收端可以通过检查数字的大小来推断时间戳格式（自 1.1.1970 起或自系统启动起）。 |
| gps_id                                                       | `uint8_t`  |       |                                                   | 多个 GPS 输入的 GPS ID<br>具有相同值的信息来自同一来源（实例）。 |
| ignore_flags                                                 | `uint16_t` |       | [GPS_INPUT_IGNORE_FLAGS](#GPS_INPUT_IGNORE_FLAGS) | 表示要忽略哪些 GPS 输入标志字段的位图。 必须提供所有其他字段。 |
| time_week_ms                                                 | `uint32_t` | ms    |                                                   | GPS 时间（从 GPS 周开始计算）                                |
| time_week                                                    | `uint16_t` |       |                                                   | GPS 周数                                                     |
| fix_type                                                     | `uint8_t`  |       |                                                   | 0-1：无固定值，2：2D 固定值，3：3D 固定值。4: 3D with DGPS. 5: 3D with RTK |
| lat                                                          | `int32_t`  | degE7 |                                                   | 纬度 (WGS84)                                                 |
| lon                                                          | `int32_t`  | degE7 |                                                   | 经度 (WGS84)                                                 |
| alt                                                          | `float`    | m     |                                                   | 高度（MSL）。正数表示向上。                                  |
| hdop                                                         | `float`    |       | invalid:UINT16_MAX                                | GPS HDOP 水平稀释位置（无单位）。如果未知，则设置为 UINT16_MAX |
| vdop                                                         | `float`    |       | invalid:UINT16_MAX                                | GPS VDOP 垂直位置稀释（无单位）。如果未知，则设置为 UINT16_MAX |
| vn                                                           | `float`    | m/s   |                                                   | GPS 在地球固定的 NED 框架中的北向速度                        |
| ve                                                           | `float`    | m/s   |                                                   | 在地球固定的 NED 框架中 GPS 的东向速度                       |
| vd                                                           | `float`    | m/s   |                                                   | 在地球固定的 NED 框架中 GPS 下行方向的速度                   |
| speed_accuracy                                               | `float`    | m/s   |                                                   | GPS 速度精度                                                 |
| horiz_accuracy                                               | `float`    | m     |                                                   | GPS 水平精度                                                 |
| vert_accuracy                                                | `float`    | m     |                                                   | GPS 垂直精度                                                 |
| satellites_visible                                           | `uint8_t`  |       |                                                   | 可见卫星数。                                                 |
| <span class='ext'>yaw</span> <a href='#mav2_extension_field'>++</a> | `uint16_t` | cdeg  |                                                   | 车辆相对于地球北方的偏航，0 表示不可用，使用 36000 表示北方  |


### GPS_RTCM_DATA (233) 

注入板载 GPS 的 RTCM 信息（用于 DGPS）

| 字段名称 | 类型           | 单位  | 说明                                                         |
| -------- | -------------- | ----- | ------------------------------------------------------------ |
| flags    | `uint8_t`      |       | LSB：1 表示报文已分片，接下来的 2 位是片段 ID，其余 5 位用于序列 ID。只有在自动驾驶仪上重建了整个报文后，才能将报文刷新到 GPS 上。片段 ID 规定了片段组装到缓冲区的顺序，而序列 ID 则用于检测不同缓冲区之间的不匹配。当所有 4 个片段都存在，或收到第一个非完整有效载荷片段之前的所有片段时，缓冲区就被视为完全重建。该管理用于确保正常的 GPS 操作不会损坏 RTCM 数据，并从不可靠的传输交付命令中恢复。 |
| len      | `uint8_t`      | bytes | 数据长度                                                     |
| data     | `uint8_t[180]` |       | RTCM 信息（可能会被分片）                                    |


### HIGH_LATENCY (234) — [DEP] 

<span class="warning">**DEPRECATED:** Replaced By [HIGH_LATENCY2](#HIGH_LATENCY2) (2020-10)</span>

适合铱星等高延迟连接的信息

| 字段名称          | 类型       | 单位      | 值                                    | 说明                                                         |
| ----------------- | ---------- | --------- | ------------------------------------- | ------------------------------------------------------------ |
| base_mode         | `uint8_t`  |           | [MAV_MODE_FLAG](#MAV_MODE_FLAG)       | 已启用系统模式的位图。                                       |
| custom_mode       | `uint32_t` |           |                                       | 用于自动驾驶仪特定标志的位域。                               |
| landed_state      | `uint8_t`  |           | [MAV_LANDED_STATE](#MAV_LANDED_STATE) | 着陆状态。如果着陆状态未知，则设置为 [MAV_LANDED_STATE_UNDEFINED](#MAV_LANDED_STATE_UNDEFINED)。 |
| roll              | `int16_t`  | cdeg      |                                       | roll                                                         |
| pitch             | `int16_t`  | cdeg      |                                       | pitch                                                        |
| heading           | `uint16_t` | cdeg      |                                       | heading                                                      |
| throttle          | `int8_t`   | %         |                                       | 节流阀（百分比）                                             |
| heading_sp        | `int16_t`  | cdeg      |                                       | 航向设定点                                                   |
| latitude          | `int32_t`  | degE7     |                                       | 纬度                                                         |
| longitude         | `int32_t`  | degE7     |                                       | 经度                                                         |
| altitude_amsl     | `int16_t`  | m         |                                       | 平均海拔高度                                                 |
| altitude_sp       | `int16_t`  | m         |                                       | 相对于原点的高度设定值                                       |
| airspeed          | `uint8_t`  | m/s       |                                       | 空速                                                         |
| airspeed_sp       | `uint8_t`  | m/s       |                                       | 空速设置点                                                   |
| groundspeed       | `uint8_t`  | m/s       |                                       | 地面速度                                                     |
| climb_rate        | `int8_t`   | m/s       |                                       | 爬升率                                                       |
| gps_nsat          | `uint8_t`  |           | invalid:UINT8_MAX                     | 可见卫星数。如果未知，则设置为 UINT8_MAX                     |
| gps_fix_type      |            | `uint8_t` |                                       | [GPS_FIX_TYPE](#GPS_FIX_TYPE)                                |
| battery_remaining | `uint8_t`  | %         |                                       | 剩余电量（百分比）                                           |
| temperature       | `int8_t`   | degC      |                                       | 自动驾驶仪温度（摄氏度）                                     |
| temperature_air   | `int8_t`   | degC      |                                       | 空气温度（摄氏度），来自空速传感器                           |
| failsafe          | `uint8_t`  |           |                                       | 故障安全（每一位代表一个故障安全，0=ok，1=故障安全激活（bit0:RC，bit1:batt，bit2:GPS，bit3:GCS，bit4:fence) |
| wp_num            | `uint8_t`  |           |                                       | 当前航点编号                                                 |
| wp_distance       | `uint16_t` | m         |                                       | 目标距离                                                     |


### HIGH_LATENCY2 (235) 

适合高延迟连接（如铱星（版本 2））的信息

| 字段名称        | 类型       | 单位  | 值                                  | 说明                                                         |
| --------------- | ---------- | ----- | ----------------------------------- | ------------------------------------------------------------ |
| timestamp       | `uint32_t` | ms    |                                     | 时间戳（自启动或 Unix 时间起的毫秒数）                       |
| type            | `uint8_t`  |       | [MAV_TYPE](#MAV_TYPE)               | MAV 的类型（四旋翼、直升机等）                               |
| autopilot       | `uint8_t`  |       | [MAV_AUTOPILOT](#MAV_AUTOPILOT)     | 自动驾驶类型/类别。对于非飞行控制器的组件，请使用 [MAV_AUTOPILOT_INVALID](#MAV_AUTOPILOT_INVALID)。 |
| custom_mode     | `uint16_t` |       |                                     | 用于自动驾驶仪特定标志的位域（2 字节版本）。                 |
| latitude        | `int32_t`  | degE7 |                                     | 纬度                                                         |
| longitude       | `int32_t`  | degE7 |                                     | 经度                                                         |
| altitude        | `int16_t`  | m     |                                     | 平均海平面以上高度                                           |
| target_altitude | `int16_t`  | m     |                                     | 高度设置点                                                   |
| heading         | `uint8_t`  | deg/2 |                                     | 航向                                                         |
| target_heading  | `uint8_t`  | deg/2 |                                     | 航向设置点                                                   |
| target_distance | `uint16_t` | dam   |                                     | 目标航点或位置的距离                                         |
| throttle        | `uint8_t`  | %     |                                     | 节流阀                                                       |
| airspeed        | `uint8_t`  | m/s*5 |                                     | 空速                                                         |
| airspeed_sp     | `uint8_t`  | m/s*5 |                                     | 空速设定值                                                   |
| groundspeed     | `uint8_t`  | m/s*5 |                                     | 地面速度                                                     |
| windspeed       | `uint8_t`  | m/s*5 |                                     | 风速                                                         |
| wind_heading    | `uint8_t`  | deg/2 |                                     | 风向                                                         |
| eph             | `uint8_t`  | dm    |                                     | 自上次发送信息以来水平位置的最大误差                         |
| epv             | `uint8_t`  | dm    |                                     | 自上次发送信息以来垂直位置的最大误差                         |
| temperature_air | `int8_t`   | degC  |                                     | 空速传感器显示的空气温度                                     |
| climb_rate      | `int8_t`   | dm/s  |                                     | 自上次发送信息以来的最大爬升率幅度                           |
| battery         | `int8_t`   | %     | invalid:-1                          | 电池电量（如果未提供字段，则为-1）。                         |
| wp_num          | `uint16_t` |       |                                     | 当前航点编号                                                 |
| failure_flags   | `uint16_t` |       | [HL_FAILURE_FLAG](#HL_FAILURE_FLAG) | 故障标志位图。                                               |
| custom0         | `int8_t`   |       |                                     | 自定义有效载荷字段。                                         |
| custom1         | `int8_t`   |       |                                     | 自定义有效负载的字段。                                       |
| custom2         | `int8_t`   |       |                                     | 自定义有效载荷的字段。                                       |


### VIBRATION (241) 

振动级别和加速度计削波

| 字段名称    | 类型       | 单位 | 说明                                                         |
| ----------- | ---------- | ---- | ------------------------------------------------------------ |
| time_usec   | `uint64_t` | us   | 时间戳（UNIX 时间或系统启动后的时间）。接收端可以通过检查数字的大小来推断时间戳格式（自 1.1.1970 起或自系统启动起）。 |
| vibration_x | `float`    |      | X 轴上的振动级别                                             |
| vibration_y | `float`    |      | Y 轴上的振动级别                                             |
| vibration_z | `float`    |      | Z 轴上的振动级别                                             |
| clipping_0  | `uint32_t` |      | 第一个加速度计剪辑计数                                       |
| clipping_1  | `uint32_t` |      | 第二加速度计剪辑计数                                         |
| clipping_2  | `uint32_t` |      | 第三加速度计剪辑计数                                         |


### HOME_POSITION (242) 

包含原点位置。
原点是系统返回和着陆的默认位置。
该位置必须由系统在起飞时自动设置，也可以使用 [MAV_CMD_DO_SET_HOME]（#MAV_CMD_DO_SET_HOME）进行显式设置。
全局位置和局部位置表示在各自坐标系中的位置，而 q 参数表示表面的方向。
在正常情况下，它描述了航向和地形坡度，飞机可利用它们调整进场。
进场 3D 矢量描述了系统在正常飞行模式下应飞到的点，然后沿该矢量执行着陆序列。
注意： 可以通过发送参数 1=242 的 [MAV_CMD_REQUEST_MESSAGE](#MAV_CMD_REQUEST_MESSAGE)（或已废弃的 [MAV_CMD_GET_HOME_POSITION](#MAV_CMD_GET_HOME_POSITION)命令）来请求此信息。

| 字段名称                                                     | 类型       | 单位  | 说明                                                         |
| ------------------------------------------------------------ | ---------- | ----- | ------------------------------------------------------------ |
| latitude                                                     | `int32_t`  | degE7 | 纬度 (WGS84)                                                 |
| longitude                                                    | `int32_t`  | degE7 | 经度 (WGS84)                                                 |
| altitude                                                     | `int32_t`  | mm    | 高度（MSL）。正数表示向上。                                  |
| x                                                            | `float`    | m     | 该位置在本地坐标系（NED）中的本地 X 位置                     |
| y                                                            | `float`    | m     | 该位置在本地坐标系（NED）中的本地 Y 位置                     |
| z                                                            | `float`    | m     | 此位置在本地坐标系中的本地 Z 位置（NED：正 "下 "位）         |
| q                                                            | `float[4]` |       | 表示起飞位置的世界到表面法线和航向变换的四元数。<br>用于指示地面的方向和坡度。<br>如果无法提供精确的航向和表面坡度四元数，则应将所有字段设置为 NaN。 |
| approach_x                                                   | `float`    | m     | 接近矢量末端的本地 X 位置。多旋翼飞机应根据起飞路径设置该位置。在草地上着陆的固定翼飞机的设置方法与多旋翼飞机相同。在跑道上着陆的固定翼飞机应将其设置为起飞的相反方向，假设起飞是从门槛/着陆区开始的。 |
| approach_y                                                   | `float`    | m     | 进场矢量末端的本地 Y 位置。多旋翼飞机应根据其起飞路径设置该位置。草地着陆固定翼飞机的设置方法与多旋翼飞机相同。在跑道上着陆的固定翼飞机应将其设置为起飞的相反方向，假设起飞是从门槛/着陆区开始的。 |
| approach_z                                                   | `float`    | m     | 进场矢量末端的本地 Z 位置。多旋翼飞机应根据其起飞路径设置该位置。在草地上着陆的固定翼飞机的设置方法与多旋翼飞机相同。在跑道上着陆的固定翼飞机应将其设置为起飞的相反方向，假设起飞是从门槛/着陆区开始的。 |
| <span class='ext'>time_usec</span> <a href='#mav2_extension_field'>++</a> | `uint64_t` | us    | 时间戳（UNIX 纪元时间或系统启动后的时间）。接收端可通过检查数字的大小来推断时间戳格式（自 1.1.1970 起或自系统启动起）。 |


### SET_HOME_POSITION (243) — [DEP] 

<span class="warning">**DEPRECATED:** Replaced By [MAV_CMD_DO_SET_HOME](#MAV_CMD_DO_SET_HOME) (2022-02) — The command protocol version ([MAV_CMD_DO_SET_HOME](#MAV_CMD_DO_SET_HOME)) allows a GCS to detect when setting the home position has failed.)</span>

设置原点位置。
原点是系统返回和着陆的默认位置。
该位置由系统在起飞时自动设置（也可使用此信息设置）。
全局位置和局部位置编码各自坐标系中的位置，而 q 参数编码表面的方向。
在正常情况下，它描述了航向和地形坡度，飞机可利用它们来调整进近。
进场 3D 矢量描述了系统在正常飞行模式下应飞到的点，然后沿该矢量执行着陆序列。
注意： 当前的原点位置可根据请求（使用参数 1=242 的 [MAV_CMD_REQUEST_MESSAGE](#MAV_CMD_REQUEST_MESSAGE)）在 [HOME_POSITION](#HOME_POSITION) 消息中发送。

| 字段名称                                                     | 类型       | 单位  | 说明                                                         |
| ------------------------------------------------------------ | ---------- | ----- | ------------------------------------------------------------ |
| target_system                                                | `uint8_t`  |       | 系统 ID。                                                    |
| latitude                                                     | `int32_t`  | degE7 | 纬度 (WGS84)                                                 |
| longitude                                                    | `int32_t`  | degE7 | 经度 (WGS84)                                                 |
| altitude                                                     | `int32_t`  | mm    | 高度（MSL）。正数表示向上。                                  |
| x                                                            | `float`    | m     | 该位置在本地坐标系（NED）中的本地 X 位置                     |
| y                                                            | `float`    | m     | 该位置在本地坐标系（NED）中的本地 Y 位置                     |
| z                                                            | `float`    | m     | 此位置在本地坐标框架中的本地 Z 位置（NED：正 "下"）。        |
| q                                                            | `float[4]` |       | 起飞位置的世界到表面法线和航向变换。用于指示地面的航向和坡度 |
| approach_x                                                   | `float`    | m     | 接近矢量末端的本地 X 位置。多旋翼飞机应根据其起飞路径设置该位置。草地着陆固定翼飞机的设置方法与多旋翼飞机相同。在跑道上着陆的固定翼飞机应将其设置为起飞的相反方向，假设起飞是从门槛/着陆区开始的。 |
| approach_y                                                   | `float`    | m     | 进场矢量末端的本地 Y 位置。多旋翼飞机应根据其起飞路径设置该位置。草地着陆固定翼飞机的设置方法与多旋翼飞机相同。在跑道上着陆的固定翼飞机应将其设置为起飞的相反方向，假设起飞是从门槛/着陆区开始的。 |
| approach_z                                                   | `float`    | m     | 进场矢量末端的本地 Z 位置。多旋翼飞机应根据其起飞路径设置该位置。在草地上着陆的固定翼飞机的设置方法与多旋翼飞机相同。在跑道上着陆的固定翼飞机应将其设置为起飞的相反方向，假设起飞是从门槛/着陆区开始的。 |
| <span class='ext'>time_usec</span> <a href='#mav2_extension_field'>++</a> | `uint64_t` | us    | 时间戳（UNIX 纪元时间或系统启动后的时间）。接收端可通过检查数字的大小来推断时间戳格式（自 1.1.1970 起或自系统启动起）。 |


### MESSAGE_INTERVAL (244) 

特定 MAVLink 报文 ID 的报文间隔。
该信息是对 [MAV_CMD_REQUEST_MESSAGE](#MAV_CMD_REQUEST_MESSAGE)命令的响应，其中参数 1=244（此信息）和参数 2=message_id（需要间隔的信息 ID）。
它也可以作为对[MAV_CMD_GET_MESSAGE_INTERVAL](#MAV_CMD_GET_MESSAGE_INTERVAL)的响应发送。
该接口取代 [DATA_STREAM](#DATA_STREAM)。

| 字段名称    | 类型       | 单位 | 说明                                                         |
| ----------- | ---------- | ---- | ------------------------------------------------------------ |
| message_id  | `uint16_t` |      | 请求的 MAVLink 消息的 ID，V1.0 版限制为 254 条消息。         |
| interval_us | `int32_t`  | us   | 两条信息之间的间隔时间。值为 -1 表示禁用该信息流，0 表示不可用，> 0 表示发送信息的时间间隔。 |


### EXTENDED_SYS_STATE (245) 

为附加功能提供状态

| 字段名称     | 类型      | 值                                    | 说明                                                         |
| ------------ | --------- | ------------------------------------- | ------------------------------------------------------------ |
| vtol_state   | `uint8_t` | [MAV_VTOL_STATE](#MAV_VTOL_STATE)     | VTOL 状态（如果适用）。如果无人机未进行 VTOL 配置，则设置为 [MAV_VTOL_STATE_UNDEFINED](#MAV_VTOL_STATE_UNDEFINED)。 |
| landed_state | `uint8_t` | [MAV_LANDED_STATE](#MAV_LANDED_STATE) | 降落状态。如果着陆状态未知，则设置为 [MAV_LANDED_STATE_UNDEFINED](#MAV_LANDED_STATE_UNDEFINED)。 |


### ADSB_VEHICLE (246) 

ADSB 车辆的位置和信息

| 字段名称      | 类型       | 单位  | 值                                        | 说明                                   |
| ------------- | ---------- | ----- | ----------------------------------------- | -------------------------------------- |
| ICAO_address  | `uint32_t` |       |                                           | ICAO 地址                              |
| lat           | `int32_t`  | degE7 |                                           | 纬度                                   |
| lon           | `int32_t`  | degE7 |                                           | 经度                                   |
| altitude_type | `uint8_t`  |       | [ADSB_ALTITUDE_TYPE](#ADSB_ALTITUDE_TYPE) | ADSB 高度类型。                        |
| altitude      | `int32_t`  | mm    |                                           | Altitude(ASL)                          |
| heading       | `uint16_t` | cdeg  |                                           | 地面航线                               |
| hor_velocity  | `uint16_t` | cm/s  |                                           | 水平速度                               |
| ver_velocity  | `int16_t`  | cm/s  |                                           | 垂直速度。正值表示向上                 |
| callign       | `char[9]`  |       |                                           | 呼号，8+null                           |
| emitter_type  | `uint8_t`  |       | [ADSB_EMITTER_TYPE](#ADSB_EMITTER_TYPE)   | ADSB 发射器类型。                      |
| tslc          | `uint8_t`  | s     |                                           | 上次通信后的时间，以秒为单位           |
| flags         | `uint16_t` |       | [ADSB_FLAGS](#ADSB_FLAGS)                 | 表示各种状态（包括有效数据字段）的位图 |
| Squawk        | `uint16_t` |       |                                           |                                        |


### COLLISION (247) 

有关潜在碰撞的信息

| 字段名称                 | 类型       | 单位 | 值                                                        | 说明                         |
| ------------------------ | ---------- | ---- | --------------------------------------------------------- | ---------------------------- |
| src                      | `uint8_t`  |      | [MAV_COLLISION_SRC](#MAV_COLLISION_SRC)                   | 碰撞数据源                   |
| id                       | `uint32_t` |      |                                                           | 唯一标识符，域基于 src 字段  |
| action                   | `uint8_t`  |      | [MAV_COLLISION_ACTION](#MAV_COLLISION_ACTION)             | 为避免碰撞而采取的行动       |
| threat_level             | `uint8_t`  |      | [MAV_COLLISION_THREAT_LEVEL](#MAV_COLLISION_THREAT_LEVEL) | 飞机对此次碰撞的担忧程度     |
| time_too_minimum_delta   | `float`    | s    |                                                           | 撞击发生前的估计时间         |
| altitude_minimum_delta   | `float`    | m    |                                                           | 车辆与物体之间最近的垂直距离 |
| horizontal_minimum_delta | `float`    | m    |                                                           | 车辆和物体之间最近的水平距离 |


### V2_EXTENSION (248) 

在 V1 帧中执行 V2 有效载荷规格部分的报文，用于过渡支持。

| 字段名称         | 类型           | 说明                                                         |
| ---------------- | -------------- | ------------------------------------------------------------ |
| target_network   | `uint8_t`      | 网络 ID（0 用于广播）                                        |
| target_system    | `uint8_t`      | 系统 ID（广播时为 0）                                        |
| target_component | `uint8_t`      | 组件 ID（广播时为 0）                                        |
| message_type     | `uint16_t`     | 标识理解此信息的软件组件的代码（类似于 USB 设备类或 mime 类型字符串）。如果该代码小于 32768，则视为 "已注册 "协议扩展，相应条目应添加到 https://github.com/mavlink/mavlink/definition_files/extension_message_ids.xml。软件创建者可根据需要注册信息 ID 块（对 GCS 特定元数据等有用）。大于 32767 的消息类型（Message_types）将被视为本地实验，不应在任何广泛传播的代码库中进行检查。 |
| payload          | `uint8_t[249]` | 长度可变的有效载荷。作为报文类型协议的一部分，必须在有效载荷中对长度进行编码，例如将长度作为有效载荷数据，或以非零标记结束有效载荷数据。为了重建被 MAVLink 2 空字节截断（或以其他方式被截断）的零结尾有效载荷，必须这样做。除非了解编码信息类型，否则有效载荷块的整个内容都是不透明的。所使用的特定编码可能是针对特定扩展的，可能并不总是作为 MAVLink 规范的一部分进行记录。 |


### MEMORY_VECT (249) 

发送原始控制器内存。不鼓励在正常数据包中使用此信息，但它是测试新信息和获取实验性调试输出的有效方法。

| 字段名称 | 类型         | 说明                                                         |
| -------- | ------------ | ------------------------------------------------------------ |
| address  | `uint16_t`   | 调试变量的起始地址                                           |
| ver      | `uint8_t`    | 类型变量的版本代码。0=未知，类型忽略并假定为 int16_t。1=如下所示 |
| type     | `uint8_t`    | 内存变量的类型代码。对于 ver = 1：0=16 x int16_t，1=16 x uint16_t，2=16 x Q15，3=16 x 1Q14 |
| value    | `int8_t[32]` | 指定地址处的内存内容                                         |


### DEBUG_VECT (250) 

使用已命名的 3D 矢量调试。

| 字段名称  | 类型       | 单位 | 说明                                                         |
| --------- | ---------- | ---- | ------------------------------------------------------------ |
| name      | `char[10]` |      | 名称<br>具有相同值的报文来自同一来源（实例）。               |
| time_usec | `uint64_t` | us   | 时间戳（UNIX 时间或系统启动后的时间）。接收端可以通过检查数字的大小来推断时间戳格式（自 1.1.1970 起或自系统启动起）。 |
| x         | `float`    |      | x                                                            |
| y         | `float`    |      | y                                                            |
| z         | `float`    |      | z                                                            |


### NAMED_VALUE_FLOAT (251) 

以浮点形式发送键值对。不鼓励在正常数据包中使用此报文，但它是测试新报文和获取实验性调试输出的有效方法。

| 字段名称     | 类型       | 单位 | 说明                                                     |
| ------------ | ---------- | ---- | -------------------------------------------------------- |
| time_boot_ms | `uint32_t` | ms   | 时间戳（系统启动后的时间）。                             |
| name         | `char[10]` |      | 调试变量的名称<br>具有相同值的信息来自同一来源（实例）。 |
| value        | `float`    |      | 浮点数值                                                 |


### NAMED_VALUE_INT (252) 

以整数形式发送键值对。不鼓励在正常数据包中使用此报文，但它是测试新报文和获取实验性调试输出的有效方法。

| 字段名称     | 类型       | 单位 | 说明                                                     |
| ------------ | ---------- | ---- | -------------------------------------------------------- |
| time_boot_ms | `uint32_t` | ms   | 时间戳（系统启动后的时间）。                             |
| name         | `char[10]` |      | 调试变量的名称<br>具有相同值的报文来自同一来源（实例）。 |
| value        | `int32_t`  |      | 有符号整数值                                             |


### STATUSTEXT (253) 

状态文本信息。这些信息会以黄色打印在 QGroundControl 的 COMM 控制台中。警告：这些信息会占用相当大的带宽，因此只能用于重要的状态和错误信息。如果执行得当，这些信息会在 MCU 上缓冲，并以有限的速率（如 10 Hz）发送。

| 字段名称                                                     | 类型       | 值                            | 说明                                                         |
| ------------------------------------------------------------ | ---------- | ----------------------------- | ------------------------------------------------------------ |
| severity                                                     | `uint8_t`  | [MAV_SEVERITY](#MAV_SEVERITY) | 状态严重性。依赖于 RFC-5424 中的定义。                       |
| text                                                         | `char[50]` |                               | 状态文本信息，不包含空结束符                                 |
| <span class='ext'>id</span> <a href='#mav2_extension_field'>++</a> | `uint16_t` |                               | 该规约文本报文的唯一（不透明）标识符。 该标识符可用于将逻辑长文本报文从一个数据块序列中重新组合。 如果值为 0，则表示这是序列中唯一的一个块，可以立即发送信息。 |
| <span class='ext'>chunk_seq</span> <a href='#mav2_extension_field'>++</a> | `uint8_t`  |                               | 该数据块的序列号；索引从 0 开始。 文本字段中的任何空字符都表示这是最后一个数据块。 |


### DEBUG (254) 

发送调试值。索引用于区分不同的值。这些值在 QGroundControl 的绘图中显示为 DEBUG N。

| 字段名称     | 类型       | 单位 | 说明                         |
| ------------ | ---------- | ---- | ---------------------------- |
| time_boot_ms | `uint32_t` | ms   | 时间戳（系统启动后的时间）。 |
| ind          | `uint8_t`  |      | 调试变量的索引               |
| value        | `float`    |      | DEBUG 值                     |


### SETUP_SIGNING (256) 

设置 MAVLink2 签名密钥。如果调用时 secret_key 全为零且 initial_timestamp 为零，则将禁用签名。

| 字段名称          | 类型          | 说明          |
| ----------------- | ------------- | ------------- |
| target_system     | `uint8_t`     | 目标机系统 ID |
| target_component  | `uint8_t`     | 目标的组件 ID |
| secret_key        | `uint8_t[32]` | 签名密钥      |
| initial_timestamp | `uint64_t`    | 初始时间戳    |


### BUTTON_CHANGE (257) 

报告按钮状态更改。

| 字段名称       | 类型       | 单位 | 说明                         |
| -------------- | ---------- | ---- | ---------------------------- |
| time_boot_ms   | `uint32_t` | ms   | 时间戳（系统启动后的时间）。 |
| last_change_ms | `uint32_t` | ms   | 上次改变按钮状态的时间。     |
| state          | `uint8_t`  |      | 按钮状态的位图。             |


### PLAY_TUNE (258) — [DEP] 

<span class="warning">**DEPRECATED:** Replaced By [PLAY_TUNE_V2](#PLAY_TUNE_V2) (2019-10) — New version explicitly defines format. More interoperable.)</span>

控制车辆音调生成（蜂鸣器）。

| 字段名称                                                     | 类型        | 说明                  |
| ------------------------------------------------------------ | ----------- | --------------------- |
| target_system                                                | `uint8_t`   | 系统 ID               |
| target_component                                             | `uint8_t`   | 组件 ID               |
| tune                                                         | `char[30]`  | 电路板特定格式的 tune |
| <span class='ext'>tune2</span> <a href='#mav2_extension_field'>++</a> | `char[200]` | 曲调扩展              |


### CAMERA_INFORMATION (259) 

Information about a camera. Can be requested with a [MAV_CMD_REQUEST_MESSAGE](#MAV_CMD_REQUEST_MESSAGE) command.

| 字段名称                                                     | 类型          | 单位        | 值                                    | 说明                                                         |
| ------------------------------------------------------------ | ------------- | ----------- | ------------------------------------- | ------------------------------------------------------------ |
| time_boot_ms                                                 | `uint32_t`    | ms          |                                       | 时间戳（系统启动后的时间）。                                 |
| vendor_name                                                  | `uint8_t[32]` |             |                                       | 摄像机供应商名称                                             |
| model_name                                                   | `uint8_t[32]` |             |                                       | 摄像机型号名称                                               |
| firmware_version                                             | `uint32_t`    |             | invalid:0                             | 摄像机固件的版本，编码为： (Dev & 0xff) << 24                |
| focal_length                                                 | `float`       | mm          | invalid:NaN                           | 焦距。如果不知道，请使用 NaN。                               |
| sensor_size_h                                                | `float`       | mm          | invalid:NaN                           | 水平图像传感器尺寸。如果不知道，请使用 NaN。                 |
| sensor_size_v                                                | `float`       | mm          | invalid:NaN                           | 垂直图像传感器尺寸。如果不知道，请使用 NaN。                 |
| Resolution_h                                                 | `uint16_t`    | pix         | invalid:0                             | 水平图像分辨率。如果不知道，请使用 0。                       |
| resolution_v                                                 | `uint16_t`    | pix         | invalid:0                             | 垂直图像分辨率。如果不知道，请使用 0。                       |
| lens_id                                                      | `uint8_t`     |             | invalid:0                             | 保留镜头 ID。 如果不知道，请使用 0。                         |
| flags                                                        | `uint32_t`    |             | [CAMERA_CAP_FLAGS](#CAMERA_CAP_FLAGS) | 摄像机功能标志的位图。                                       |
| cam_definition_version                                       |               |             |                                       | 摄像机定义版本（迭代）。 如果不知道，请使用 0。              |
| cam_definition_uri                                           |               | `char[140]` |                                       | 相机定义 URI（如果有，否则只能使用基本功能）。允许使用 HTTP- (http://) 和 MAVLink FTP- (mavlinkftp://) 格式的 URI（执行摄像机协议的任何 GCS 都必须支持这两种格式）。定义文件可以是 xz 压缩文件，文件扩展名为 .xml.xz（执行该协议的 GCS 必须支持文件解压缩）。字符串必须以零结尾。 如果不知道，请使用长度为零的字符串。 |
| <span class='ext'>gimbal_device_id</span> <a href='#mav2_extension_field'>++</a> | `uint8_t`     |             | invalid:0                             | 与此摄像机关联的云台的云台 ID。这是云台设备的组件 ID，对于非 mavlink 云台则为 1-6。如果摄像机没有关联云台，则使用 0。 |


### CAMERA_SETTINGS (260) 

Settings of a camera. Can be requested with a [MAV_CMD_REQUEST_MESSAGE](#MAV_CMD_REQUEST_MESSAGE) command.

| 字段名称                                                     | 类型       | 单位 | 值                          | 说明                                                         |
| ------------------------------------------------------------ | ---------- | ---- | --------------------------- | ------------------------------------------------------------ |
| time_boot_ms                                                 | `uint32_t` | ms   |                             | 时间戳（系统启动后的时间）。                                 |
| mode_id                                                      | `uint8_t`  |      | [CAMERA_MODE](#CAMERA_MODE) | 摄像机模式                                                   |
| <span class='ext'>zoomLevel</span> <a href='#mav2_extension_field'>++</a> | `float`    |      | invalid:NaN                 | 当前缩放级别占整个范围的百分比（0.0 至 100.0，未知则为 NaN） |
| <span class='ext'>focusLevel</span> <a href='#mav2_extension_field'>++</a> | `float`    |      | invalid:NaN                 | 当前聚焦水平占整个范围的百分比（0.0 至 100.0，未知则为 NaN） |


### STORAGE_INFORMATION (261) 

有关存储介质的信息。此消息是响应带有 [MAV_CMD_REQUEST_MESSAGE](#MAV_CMD_REQUEST_MESSAGE) 的请求以及每当存储状态发生变化 ([STORAGE_STATUS](#STORAGE_STATUS)) 时发送的。使用 [MAV_CMD_REQUEST_MESSAGE](#MAV_CMD_REQUEST_MESSAGE).param2 指示请求存储的索引/ID：0 表示全部，1 表示第一个，2 表示第二个，等等。

| 字段名称                                                     | 类型       | 单位  | 值                                        | 说明                                                         |
| ------------------------------------------------------------ | ---------- | ----- | ----------------------------------------- | ------------------------------------------------------------ |
| time_boot_ms                                                 | `uint32_t` | ms    |                                           | 时间戳（自系统启动以来的时间）。                             |
| storage_id                                                   | `uint8_t`  |       |                                           | 存储 ID（1 表示第一个，2 表示第二个，等等）<br>具有相同值的消息来自同一来源（实例）。 |
| storage_count                                                | `uint8_t`  |       |                                           | 存储设备数量                                                 |
| status                                                       | `uint8_t`  |       | [STORAGE_STATUS](#STORAGE_STATUS)         | 存储状态                                                     |
| total_capacity                                               | `float`    | MiB   |                                           | 总容量。如果存储尚未准备好（[STORAGE_STATUS_READY](#STORAGE_STATUS_READY))，值将被忽略。 |
| used_capacity                                                | `float`    | MiB   |                                           | 已用容量。如果存储尚未准备好（[STORAGE_STATUS_READY](#STORAGE_STATUS_READY))，值将被忽略。 |
| available_capacity                                           | `float`    | MiB   |                                           | 可用存储容量。如果存储尚未准备好（[STORAGE_STATUS_READY](#STORAGE_STATUS_READY))，值将被忽略。 |
| read_speed                                                   | `float`    | MiB/s |                                           | 读取速度。                                                   |
| write_speed                                                  | `float`    | MiB/s |                                           | 写入速度。                                                   |
| <span class='ext'>type</span> <a href='#mav2_extension_field'>++</a> | `uint8_t`  |       | [STORAGE_TYPE](#STORAGE_TYPE)             | 存储类型                                                     |
| <span class='ext'>name</span> <a href='#mav2_extension_field'>++</a> | `char[32]` |       |                                           | 将在用户界面中使用的文本存储名称（microSD 1、内部存储器等），这是一个以 NULL 结尾的字符串。如果长度正好为 32 个字符，则添加一个以 NULL 结尾的字符串。如果该字符串为空，则向用户显示通用类型。 |
| <span class='ext'>storage_usage</span> <a href='#mav2_extension_field'>++</a> | `uint8_t`  |       | [STORAGE_USAGE_FLAG](#STORAGE_USAGE_FLAG) | 表示此实例是否优先存储照片、视频等的标志。<br注意：实施时应首先在用于保存介质的系统默认存储 ID 上设置标记（如果可能/支持的话）。<br>然后可以使用以下方法覆盖此设置 [MAV_CMD_SET_STORAGE_USAGE](#MAV_CMD_SET_STORAGE_USAGE).<br>如果未设置介质使用标志，GCS 可能会认为存储 ID 1 是所有介质类型的默认存储。 |


### CAMERA_CAPTURE_STATUS (262) 

有关捕获状态的信息。可用 [MAV_CMD_REQUEST_MESSAGE](#MAV_CMD_REQUEST_MESSAGE) 命令请求。

| 字段名称                                                     | 类型       | 单位 | 说明                                                         |
| ------------------------------------------------------------ | ---------- | ---- | ------------------------------------------------------------ |
| time_boot_ms                                                 | `uint32_t` | ms   | 时间戳（系统启动后的时间）。                                 |
| image_status                                                 | `uint8_t`  |      | 图像捕获的当前状态（0：空闲，1：捕获中，2：间隔已设置但空闲，3：间隔已设置且捕获中） |
| video_status                                                 | `uint8_t`  |      | 视频捕捉的当前状态（0: 空闲，1: 捕捉中）                     |
| image_interval                                               | `float`    | s    | 图像捕捉间隔                                                 |
| recording_time_ms                                            | `uint32_t` | ms   | 录制开始后的耗时（0：不支持/不可用）。GCS 应计算记录时间，并使用此字段的非零值来纠正任何差异。 |
| available_capacity                                           | `float`    | MiB  | 可用存储容量。                                               |
| <span class='ext'>image_count</span> <a href='#mav2_extension_field'>++</a> | `int32_t`  |      | 捕获的图像总数（"永久"，或使用 [MAV_CMD_STORAGE_FORMAT](#MAV_CMD_STORAGE_FORMAT)重置为止）。 |


### CAMERA_IMAGE_CAPTURED (263) 

有关捕获图像的信息。每次捕获信息时都会发出该信息。

[MAV_CMD_REQUEST_MESSAGE](#MAV_CMD_REQUEST_MESSAGE)可用于为特定序列号或序列号范围（重新）请求此信息：
[MAV_CMD_REQUEST_MESSAGE](#MAV_CMD_REQUEST_MESSAGE).param2表示要发送的第一个图像的序列号，或者设置为-1来发送所有序列号的信息。
[MAV_CMD_REQUEST_MESSAGE](#MAV_CMD_REQUEST_MESSAGE).param3 用于指定要发送的信息范围：
设置为 0（默认），只发送参数 2 中序列号的信息、
设置为-1，则发送参数 2 中序列号的报文和后面所有序列号的报文、
设置为范围内最后一条报文的序列号。

| 字段名称       | 类型        | 单位  | 说明                                                         |
| -------------- | ----------- | ----- | ------------------------------------------------------------ |
| time_boot_ms   | `uint32_t`  | ms    | 时间戳（系统启动后的时间）。                                 |
| time_utc       | `uint64_t`  | us    | 以 UTC 为单位的时间戳（UNIX 元年以来的时间）。0 表示未知。   |
| camera_id      | `uint8_t`   |       | 过时/未使用。组件 ID 用于区分多个摄像机。                    |
| lat            | `int32_t`   | degE7 | 图像拍摄地点的纬度                                           |
| lon            | `int32_t`   | degE7 | 拍摄地点的经度                                               |
| alt            | `int32_t`   | mm    | 拍摄图像的高度（MSL）                                        |
| relative_alt   | `int32_t`   | mm    | 离地高度                                                     |
| q              | `float[4]`  |       | 摄像机方向的四元数（w、x、y、z 顺序，零旋转为 1, 0, 0, 0）   |
| image_index    | `int32_t`   |       | 此图像的零基索引（即新图像的索引 [CAMERA_CAPTURE_STATUS](#CAMERA_CAPTURE_STATUS).image count -1） |
| capture_result | `int8_t`    |       | 表示捕捉图像成功（1）或失败（0）的布尔值。                   |
| file_url       | `char[205]` |       | 拍摄图像的 URL。如果摄像机提供 HTTP 接口，可选择本地存储或 http://foo.jpg。 |


### FLIGHT_INFORMATION (264) 

飞行信息。

其中包括启动、起飞和着陆后的时间以及航班号。
起飞和着陆值在启动时重置为零。
可使用 [MAV_CMD_REQUEST_MESSAGE](#MAV_CMD_REQUEST_MESSAGE)请求获取。
请注意，有些字段的名称有误--时间戳来自开机（而非UTC），flight_uuid 是序列号。

| 字段名称                                                     | 类型       | 单位 | 说明                                                         |
| ------------------------------------------------------------ | ---------- | ---- | ------------------------------------------------------------ |
| time_boot_ms                                                 | `uint32_t` | ms   | 时间戳（系统启动后的时间）。                                 |
| arming_time_utc                                              | `uint64_t` | us   | 启动时的时间戳（系统启动后的时间）。启动时设置为 0。启动时设置为 0。注意，该字段被误称为 UTC。 |
| takeoff_time_utc                                             | `uint64_t` | us   | 启动时的时间戳（自系统启动起）。启动和布防时设置为 0。注意，该字段被错误命名为 UTC。 |
| flight_uuid                                                  | `uint64_t` |      | 航班号。注意，字段名称 UUID 有误。                           |
| <span class='ext'>landing_time</span> <a href='#mav2_extension_field'>++</a> | `uint32_t` | ms   | 着陆时的时间戳（系统启动后的毫秒数）。启动和布防时设置为 0。 |


### MOUNT_ORIENTATION (265) — [DEP] 

<span class="warning">**DEPRECATED:** Replaced By [MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW](#MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW) (2020-01) — This message is being superseded by [MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW](#MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW). The message can still be used to communicate with legacy gimbals implementing it.)</span>

镶样的方向

| 字段名称                                                     | 类型       | 单位    | 说明                                                       |
| ------------------------------------------------------------ | ---------- | ------- | ---------------------------------------------------------- |
| time_boot_ms                                                 | `uint32_t` | ms      | 时间戳（系统启动后的时间）。                               |
| roll                                                         |            | `float` | deg                                                        |
| pitch                                                        |            | `float` | deg                                                        |
| yaw                                                          | `float`    | deg     | 相对于车辆的偏航（无效时设置为 NaN）。                     |
| <span class='ext'>yaw_absolute</span> <a href='#mav2_extension_field'>++</a> | `float`    | deg     | 相对于地球正北的绝对帧偏航，正北为 0（无效时设置为 NaN）。 |


### LOGGING_DATA (266) 

包含记录数据的信息（另请参阅 [MAV_CMD_LOGGING_START](#MAV_CMD_LOGGING_START)）。

| 字段名称             | 类型           | 单位  | 说明                                                         |
| -------------------- | -------------- | ----- | ------------------------------------------------------------ |
| target_system        | `uint8_t`      |       | 目标机的系统 ID                                              |
| target_component     | `uint8_t`      |       | 目标的组件 ID                                                |
| sequence             | `uint16_t`     |       | 序列号（可以换行）                                           |
| length               | `uint8_t`      | bytes | 数据长度                                                     |
| first_message_offset | `uint8_t`      | bytes | 第一个信息开始的数据偏移量。当前一条信息丢失时，该偏移可用于恢复（如果不存在起始位置，则设置为 UINT8_MAX）。 |
| data                 | `uint8_t[249]` |       | 日志数据                                                     |


### LOGGING_DATA_ACKED (267) 

包含记录数据的报文，需要发送 [LOGGING_ACK](#LOGGING_ACK)回传

| 字段名称             | 类型           | 单位  | 说明                                                         |
| -------------------- | -------------- | ----- | ------------------------------------------------------------ |
| target_system        | `uint8_t`      |       | 目标机的系统 ID                                              |
| target_component     | `uint8_t`      |       | 目标的组件 ID                                                |
| sequence             | `uint16_t`     |       | 序列号（可以换行）                                           |
| 长度                 | `uint8_t`      | bytes | 数据长度                                                     |
| first_message_offset | `uint8_t`      | bytes | 第一个信息开始的数据偏移量。当前一条信息丢失时，该偏移可用于恢复（如果不存在起始位置，则设置为 UINT8_MAX）。 |
| data                 | `uint8_t[249]` |       | 日志数据                                                     |


### LOGGING_ACK (268) 

对 [LOGGING_DATA_ACKED](#LOGGING_DATA_ACKED) 消息的应答

| 字段名称         | 类型       | 描述                                                         |
| ---------------- | ---------- | ------------------------------------------------------------ |
| target_system    | `uint8_t`  | 目标系统 ID                                                  |
| target_component | `uint8_t`  | 目标的组件 ID                                                |
| sequence         | `uint16_t` | 序列号（必须与 [LOGGING_DATA_ACKED](#LOGGING_DATA_ACKED) 中的序列号一致） |


### VIDEO_STREAM_INFORMATION (269) 

有关视频流的信息。可以使用 [MAV_CMD_REQUEST_MESSAGE](#MAV_CMD_REQUEST_MESSAGE) 请求，其中 param2 表示视频流 ID：0 表示所有流，1 表示第一个，2 表示第二个，等等。

| 字段名称     | 类型        | 单位   | 值                                                      | 说明                                                         |
| ------------ | ----------- | ------ | ------------------------------------------------------- | ------------------------------------------------------------ |
| stream_id    | `uint8_t`   |        |                                                         | 视频流 ID（1 表示第一个，2 表示第二个，等等）<br>具有相同值的消息来自同一源（实例）。 |
| count        | `uint8_t`   |        |                                                         | 可用流的数量。                                               |
| type         | `uint8_t`   |        | [VIDEO_STREAM_TYPE](#VIDEO_STREAM_TYPE)                 | 流的类型。                                                   |
| flags        | `uint16_t`  |        | [VIDEO_STREAM_STATUS_FLAGS](#VIDEO_STREAM_STATUS_FLAGS) | 流状态标志的位图。                                           |
| framerate    | `float`     | Hz     |                                                         | 帧速率。                                                     |
| resolution_h | `uint16_t`  | pix    |                                                         | 水平分辨率。                                                 |
| resolution_v | `uint16_t`  | pix    |                                                         | 垂直分辨率。                                                 |
| bitrate      | `uint32_t`  | bits/s |                                                         | 比特率。                                                     |
| rotation     | `uint16_t`  | deg    |                                                         | 视频图像顺时针旋转。                                         |
| hfov         | `uint16_t`  | deg    |                                                         | 水平视野。                                                   |
| name         | `char[32]`  |        |                                                         | 流名称。                                                     |
| uri          | `char[160]` |        |                                                         | 视频流 URI（地面站应连接到的 TCP 或 RTSP URI）或端口号（地面站应监听的 UDP 端口）。 |


### VIDEO_STREAM_STATUS (270) 

有关视频流状态的信息。可以使用 [MAV_CMD_REQUEST_MESSAGE](#MAV_CMD_REQUEST_MESSAGE) 请求。

| 字段名称     | 类型       | 单位   | 值                                                      | 说明                                                         |
| ------------ | ---------- | ------ | ------------------------------------------------------- | ------------------------------------------------------------ |
| stream_id    | `uint8_t`  |        |                                                         | 视频流 ID（1 表示第一个，2 表示第二个，等等）<br>具有相同值的消息来自同一源（实例）。 |
| flags        | `uint16_t` |        | [VIDEO_STREAM_STATUS_FLAGS](#VIDEO_STREAM_STATUS_FLAGS) | 流状态标志的位图                                             |
| framerate    | `float`    | Hz     |                                                         | 帧速率                                                       |
| resolution_h | `uint16_t` | pix    |                                                         | 水平分辨率                                                   |
| resolution_v | `uint16_t` | pix    |                                                         | 垂直分辨率                                                   |
| bitrate      | `uint32_t` | bits/s |                                                         | 比特率                                                       |
| 旋转         | `uint16_t` | 度     |                                                         | 视频图像顺时针旋转                                           |
| hfov         | `uint16_t` | 度     |                                                         | 水平视野                                                     |


### CAMERA_FOV_STATUS (271) 

有关摄像机视野的信息。可用 [MAV_CMD_REQUEST_MESSAGE](#MAV_CMD_REQUEST_MESSAGE) 命令请求。

| 字段名称     | 类型       | 单位  | 说明                                                         |
| ------------ | ---------- | ----- | ------------------------------------------------------------ |
| time_boot_ms | `uint32_t` | ms    | 时间戳（系统启动后的时间）。                                 |
| lat_camera   | `int32_t`  | degE7 | 摄像机的纬度（如果未知，则为 INT32_MAX）。                   |
| lon_camera   | `int32_t`  | degE7 | 摄像机的经度（如果未知，则为 INT32_MAX）。                   |
| alt_camera   | `int32_t`  | mm    | 摄像机的高度（MSL）（未知时为 INT32_MAX）。                  |
| lat_image    | `int32_t`  | degE7 | 图像中心的纬度（如果未知，则为 INT32_MAX；如果在无限远处，不与地平线相交，则为 INT32_MIN）。 |
| lon_image    | `int32_t`  | degE7 | 图像中心的经度（如果未知，则为 INT32_MAX；如果在无限远处，且不与地平线相交，则为 INT32_MIN）。 |
| alt_image    | `int32_t`  | mm    | 图像中心的高度（MSL）（如果未知，则为 INT32_MAX；如果在无限远处，且不与地平线相交，则为 INT32_MIN）。 |
| q            | `float[4]` |       | 摄像机方向的四元数（w、x、y、z 顺序，零旋转为 1, 0, 0, 0）   |
| hfov         | `float`    | deg   | 水平视场（未知时为 NaN）。                                   |
| vfov         | `float`    | deg   | 垂直视场（未知时为 NaN）。                                   |


### CAMERA_TRACKING_IMAGE_STATUS (275) 

摄像机跟踪状态，在主动跟踪时发送。使用 [MAV_CMD_SET_MESSAGE_INTERVAL]（#MAV_CMD_SET_MESSAGE_INTERVAL）定义信息间隔。

| 字段名称        | 类型      | 值                                                           | 说明                                                         |
| --------------- | --------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| tracking_status | `uint8_t` | [CAMERA_TRACKING_STATUS_FLAGS](#CAMERA_TRACKING_STATUS_FLAGS) | 当前跟踪状态                                                 |
| tracking_mode   | `uint8_t` | [CAMERA_TRACKING_MODE](#CAMERA_TRACKING_MODE)                | 当前跟踪模式                                                 |
| target_data     | `uint8_t` | [CAMERA_TRACKING_TARGET_DATA](#CAMERA_TRACKING_TARGET_DATA)  | 定义目标数据的位置                                           |
| point_x         | `float`   | invalid:NaN                                                  | 如果 [CAMERA_TRACKING_MODE_POINT](#CAMERA_TRACKING_MODE_POINT)，当前跟踪点 x 值（归一化为 0...1，0 表示左，1 表示右），如果未知，则为 NAN |
| point_y         | `float`   | invalid:NaN                                                  | 当前跟踪点的 y 值，如果 [CAMERA_TRACKING_MODE_POINT](#CAMERA_TRACKING_MODE_POINT)（归一化为 0...1，0 表示顶部，1 表示底部），未知时为 NAN。 |
| radius          | `float`   | invalid:NaN                                                  | 当前跟踪半径，如果 [CAMERA_TRACKING_MODE_POINT](#CAMERA_TRACKING_MODE_POINT)（归一化为 0...1，0 表示图像左侧，1 表示图像右侧），如果未知，则为 NAN |
| rec_top_x       | `float`   | invalid:NaN                                                  | 如果 [CAMERA_TRACKING_MODE_RECTANGLE](#CAMERA_TRACKING_MODE_RECTANGLE)（归一化为 0...1，0 表示图像左侧，1 表示图像右侧），则当前跟踪矩形的顶部 x 值为 NAN（如果未知）。 |
| rec_top_y       | `float`   | invalid:NaN                                                  | 如果 [CAMERA_TRACKING_MODE_RECTANGLE](#CAMERA_TRACKING_MODE_RECTANGLE)（归一化为 0...1，0 表示顶部，1 表示底部），则当前跟踪矩形的顶部 y 值，如果未知，则为 NAN。 |
| rec_bottom_x    | `float`   | invalid:NaN                                                  | 如果 [CAMERA_TRACKING_MODE_RECTANGLE](#CAMERA_TRACKING_MODE_RECTANGLE)（归一化为 0...1，0 表示左，1 表示右），则当前跟踪矩形的底部 x 值，如果未知，则为 NAN。 |
| rec_bottom_y    | `float`   | invalid:NaN                                                  | 如果 [CAMERA_TRACKING_MODE_RECTANGLE](#CAMERA_TRACKING_MODE_RECTANGLE)（归一化为 0...1，0 表示顶部，1 表示底部），则当前跟踪矩形的底部 y 值为 NAN（如果未知）。 |


### CAMERA_TRACKING_GEO_STATUS (276) 

摄像机跟踪状态，在主动跟踪时发送。使用 [MAV_CMD_SET_MESSAGE_INTERVAL]（#MAV_CMD_SET_MESSAGE_INTERVAL）定义信息间隔。

| 字段名称        | 类型      | 单位  | 值                                                           | 说明                                               |
| --------------- | --------- | ----- | ------------------------------------------------------------ | -------------------------------------------------- |
| tracking_status | `uint8_t` |       | [CAMERA_TRACKING_STATUS_FLAGS](#CAMERA_TRACKING_STATUS_FLAGS) | 当前跟踪状态                                       |
| lat             | `int32_t` | degE7 |                                                              | 被跟踪物体的纬度                                   |
| lon             | `int32_t` | degE7 |                                                              | 被跟踪物体的经度                                   |
| alt             | `float`   | m     |                                                              | 被跟踪物体的高度（AMLL, WGS84）                    |
| h_acc           | `float`   | m     | invalid:NaN                                                  | 水平精度。如果未知，则为 NAN                       |
| v_acc           | `float`   | m     | invalid:NaN                                                  | 垂直精度。如果未知，则为 NAN                       |
| vel_n           | `float`   | m/s   | invalid:NaN                                                  | 被跟踪物体的北向速度。如果未知，则为 NAN           |
| vel_e           | `float`   | m/s   | invalid:NaN                                                  | 被跟踪物体的东移速度。如果未知，则为 NAN           |
| vel_d           | `float`   | m/s   | invalid:NaN                                                  | 被跟踪物体的向下速度。如果未知，则为 NAN           |
| vel_acc         | `float`   | m/s   | invalid:NaN                                                  | 速度精度。如果未知，则为 NAN                       |
| dist            | `float`   | m     | invalid:NaN                                                  | 摄像机与被跟踪物体之间的距离。如果未知，则为 NAN   |
| hdg             | `float`   | rad   | invalid:NaN                                                  | 以弧度为单位的航向，单位为 NED。如果未知，则为 NAN |
| hdg_acc         | `float`   | rad   | invalid:NaN                                                  | 航向精度，单位 NED。如果未知，则为 NAN             |


### GIMBAL_MANAGER_INFORMATION (280) 

有关高级万向节管理器的信息。地面站应使用 [MAV_CMD_REQUEST_MESSAGE](#MAV_CMD_REQUEST_MESSAGE) 请求此消息。

| 字段名称         | 类型       | 单位 | 值                                                    | 说明                                                         |
| ---------------- | ---------- | ---- | ----------------------------------------------------- | ------------------------------------------------------------ |
| time_boot_ms     | `uint32_t` | ms   |                                                       | 时间戳（自系统启动以来的时间）。                             |
| cap_flags        | `uint32_t` |      | [GIMBAL_MANAGER_CAP_FLAGS](#GIMBAL_MANAGER_CAP_FLAGS) | 万向节功能标志的位图。                                       |
| gimbal_device_id | `uint8_t`  |      |                                                       | 此万向节管理器负责的万向节设备 ID。万向节设备的组件 ID（或非 MAVLink 万向节的 1-6）。<br>具有相同值的消息来自同一来源（实例）。 |
| roll_min         | `float`    | rad  |                                                       | 最小硬件滚动角度（正数：向右滚动，负数：向左滚动）           |
| roll_max         | `float`    | rad  |                                                       | 最大硬件滚动角度（正数：向右滚动，负数：向左滚动）           |
| pitch_min        | `float`    | rad  |                                                       | 最小俯仰角度（正数：向上，负数：向下）                       |
| pitch_max        | `float`    | rad  |                                                       | 最大俯仰角度（正数：向上，负数：向下）                       |
| yaw_min          | `float`    | rad  |                                                       | 最小偏航角度（正数：向右，负数：向左）                       |
| yaw_max          | `float`    | rad  |                                                       | 最大偏航角度（正数：向右，负数：向左）                       |


### GIMBAL_MANAGER_STATUS (281) 

有关高级万向节管理器的当前状态。此消息应以较低的常规速率（例如 5Hz）广播。

| 字段名称                 | 类型       | 单位 | 值                                            | 说明                                                         |
| ------------------------ | ---------- | ---- | --------------------------------------------- | ------------------------------------------------------------ |
| time_boot_ms             | `uint32_t` | ms   |                                               | 时间戳（自系统启动以来的时间）。                             |
| flags                    | `uint32_t` |      | [GIMBAL_MANAGER_FLAGS](#GIMBAL_MANAGER_FLAGS) | 当前应用的高级万向节管理器标志。                             |
| gimbal_device_id         | `uint8_t`  |      |                                               | 此万向节管理器负责的万向节设备 ID。万向节设备的组件 ID（或非 MAVLink 万向节的 1-6）。<br>具有相同值的消息来自同一来源（实例）。 |
| primary_control_sysid    | `uint8_t`  |      |                                               | 具有主要控制的 MAVLink 组件的系统 ID，0 表示无。             |
| primary_control_compid   | `uint8_t`  |      |                                               | 具有主要控制的 MAVLink 组件的组件 ID，0 表示无。             |
| secondary_control_sysid  | `uint8_t`  |      |                                               | 具有辅助控制的 MAVLink 组件的系统 ID，0 表示无。             |
| secondary_control_compid | `uint8_t`  |      |                                               | 具有辅助控制的 MAVLink 组件的组件 ID，0 表示无。             |


### GIMBAL_MANAGER_SET_ATTITUDE (282) 

用于控制万向节姿态的高级消息。此消息将发送到万向节管理器（例如从地面站发送）。角度和速率可以根据使用情况设置为 NaN。

| 字段名称           | 类型       | 单位  | 值                                            | 说明                                                         |
| ------------------ | ---------- | ----- | --------------------------------------------- | ------------------------------------------------------------ |
| target_system      | `uint8_t`  |       |                                               | 系统 ID                                                      |
| target_component   | `uint8_t`  |       |                                               | 组件 ID                                                      |
| flags              | `uint32_t` |       | [GIMBAL_MANAGER_FLAGS](#GIMBAL_MANAGER_FLAGS) | 要使用的高级万向节管理器标志。                               |
| gimbal_device_id   | `uint8_t`  |       |                                               | 要寻址的万向节设备的组件 ID（对于非 MAVLink 万向节，为 1-6），对于所有万向节设备组件，为 0。为多个万向节（但不是所有万向节）多次发送命令。<br>具有相同值的消息来自同一来源（实例）。 |
| q                  | `float[4]` |       |                                               | 四元数分量，w、x、y、z（1 0 0 0 为零旋转，框架取决于是否设置了标志 [GIMBAL_MANAGER_FLAGS_YAW_LOCK](#GIMBAL_MANAGER_FLAGS_YAW_LOCK)） |
| angular_velocity_x | `float`    | rad/s | invalid:NaN                                   | 角速度的 X 分量，正数为向右滚动，NaN 被忽略。                |
| angular_velocity_y | `float`    | rad/s | invalid:NaN                                   | 角速度的 Y 分量，正数为向上俯仰，NaN 被忽略。                |
| angular_velocity_z | `float`    | rad/s | invalid:NaN                                   | 角速度的 Z 分量，正数表示向右偏航，NaN 表示被忽略。          |


### GIMBAL_DEVICE_INFORMATION (283) 

有关低级别云台的信息。该信息应由云台管理器或地面站使用 [MAV_CMD_REQUEST_MESSAGE]（#MAV_CMD_REQUEST_MESSAGE）进行请求。最大角度和速率是硬件限制。但是，所使用软件的限制可能不同/较小，并取决于模式/设置等。

| 字段名称                                                     | 类型       | 单位 | 值                                                  | 说明                                                         |
| ------------------------------------------------------------ | ---------- | ---- | --------------------------------------------------- | ------------------------------------------------------------ |
| time_boot_ms                                                 | `uint32_t` | ms   |                                                     | 时间戳（系统启动后的时间）。                                 |
| vendor_name                                                  | `char[32]` |      |                                                     | 云台供应商名称。                                             |
| model_name                                                   | `char[32]` |      |                                                     | 云台型号名称。                                               |
| custom_name                                                  | `char[32]` |      |                                                     | 用户自定义的云台名称。                                       |
| firmware_version                                             | `uint32_t` |      |                                                     | 云台固件的版本，编码为： (Dev & 0xff) << 24                  |
| hardware_version                                             | `uint32_t` |      |                                                     | 云台硬件的版本，编码为： (Dev & 0xff) << 24 (Patch & 0xff) << 16 (Minor & 0xff) << 8 (Major & 0xff)： (Dev & 0xff) << 24 |
| uid                                                          | `uint64_t` |      | invalid:0                                           | 万向节硬件的 UID（如果未知，则为 0）。                       |
| cap_flags                                                    | `uint16_t` |      | [GIMBAL_DEVICE_CAP_FLAGS](#GIMBAL_DEVICE_CAP_FLAGS) | 万向节功能标志的位图。                                       |
| custom_cap_flags                                             | `uint16_t` |      |                                                     | 用于万向节特定功能标志的位图。                               |
| roll_min                                                     | `float`    | rad  | invalid:NaN                                         | 硬件最小滚动角度（正：向右滚动，负：向左滚动）。如果未知，则为 NAN。 |
| Roll_max                                                     | `float`    | rad  | invalid:NaN                                         | 最大硬件滚动角度（正：向右滚动，负：向左滚动）。如果未知，则为 NAN。 |
| pitch_min                                                    | `float`    | rad  | invalid:NaN                                         | 最小硬件俯仰角（正：向上，负：向下）。如果未知，则为 NAN。   |
| pitch_max                                                    | `float`    | rad  | invalid:NaN                                         | 最大硬件俯仰角（正：向上，负：向下）。如果未知，则为 NAN。   |
| yaw_min                                                      | `float`    | rad  | invalid:NaN                                         | 硬件偏航最小角度（正：向右，负：向左）。如果未知，则为 NAN。 |
| yaw_max                                                      | `float`    | rad  | invalid:NaN                                         | 最大硬件偏航角度（正：向右，负：向左）。如果未知，则为 NAN。 |
| <span class='ext'>gimbal_device_id</span> <a href='#mav2_extension_field'>++</a> | `uint8_t`  |      | invalid:0                                           | 如果万向节管理器和万向节设备是同一个组件，因此组件 ID 相同，则使用该字段。该字段可设置为 1-6 之间的数字。如果组件 ID 是独立的，则不需要此字段，必须设置为 0。 |


### GIMBAL_DEVICE_SET_ATTITUDE (284) 

用于控制万向节设备姿态的低级信息。

该信息由万向节管理器发送至万向节设备组件。
四元数和角速度可根据使用情况设置为 NaN。
对于四元数和角速度中编码的角度：
如果标志 [GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME]（#GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME）被设置，则它们相对于车辆航向（车辆帧）。
如果设置了 [GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME]（#GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME）标志，则它们是相对于绝对北方（地球框架）的。
如果这两个标志都未设置，则（为了向后兼容）保持不变：
如果设置了[GIMBAL_DEVICE_FLAGS_YAW_LOCK](#GIMBAL_DEVICE_FLAGS_YAW_LOCK)标志，则它们是相对于绝对北方（地球帧）的、
否则就是相对于车辆航向（车辆框架）。
不允许同时设置 [GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME]（#GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME）和 [GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME]（#GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME）。
这些规则是为了确保向后兼容性。
新实现应始终设置 [GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME](#GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME)或 [GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME](#GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME)。

| 字段名称           | 类型       | 单位  | 值                                          | 说明                                                         |
| ------------------ | ---------- | ----- | ------------------------------------------- | ------------------------------------------------------------ |
| target_system      | `uint8_t`  |       |                                             | 系统 ID                                                      |
| target_component   | `uint8_t`  |       |                                             | 组件 ID                                                      |
| flags              | `uint16_t` |       | [GIMBAL_DEVICE_FLAGS](#GIMBAL_DEVICE_FLAGS) | 低级万向节标志。                                             |
| q                  | `float[4]` |       | invalid:[NaN]                               | 四元数分量，w、x、y、z（1 0 0 0 为空旋转）。信息描述中描述了该帧。将字段设置为 NaN 将被忽略。 |
| angular_velocity_x | `float`    | rad/s | invalid:NaN                                 | 角速度的 X 分量（正值：向右滚动）。该帧在信息描述中进行了说明。NaN 将被忽略。 |
| angular_velocity_y | `float`    | rad/s | invalid:NaN                                 | 角速度的 Y 分量（正值：向上翻滚）。该帧已在信息描述中说明。NaN 将被忽略。 |
| angular_velocity_z | `float`    | rad/s | invalid:NaN                                 | 角速度的 Z 分量（正值：向右偏航）。该帧已在信息描述中说明。NaN 将被忽略。 |


### GIMBAL_DEVICE_ATTITUDE_STATUS (285) 

报告万向节设备状态的信息。

该信息应由云台设备组件以较低的频率（如 5 Hz）广播。
对于用四元数编码的角度和角速度来说，该信息有效：
如果[GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME]（#GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME）标志被设置，则它们相对于飞行器航向（飞行器帧）。
如果设置了 [GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME]（#GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME）标志，则它们是相对于绝对北方（地球框架）的。
如果这两个标志都未设置，则（为了向后兼容）保持不变：
如果设置了[GIMBAL_DEVICE_FLAGS_YAW_LOCK](#GIMBAL_DEVICE_FLAGS_YAW_LOCK)标志，则它们相对于绝对北方（地球帧）、
否则为相对于车辆航向（车辆框架）。
不允许使用其他标志条件。
根据 delta_yaw 和 delta_yaw_velocity 可以计算出其他帧中的四元数和角速度，计算公式为
q_earth = q_delta_yaw * q_vehicle，w_earth = w_delta_yaw_velocity + w_vehicle（如果不为 NaN）。
如果既未设置 [GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME](#GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME) 标志，也未设置 [GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME](#GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME) 标志、
那么（为了向后兼容）delta_yaw 和 delta_yaw_velocity 字段中的数据将被忽略。
新实现应始终设置[GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME](#GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME)或[GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME](#GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME)、
并始终将 delta_yaw 和 delta_yaw_velocity 设置为合适的值或 NaN。

| 字段名称                                                     | 类型       | 单位  | 值                                                      | 说明                                                         |
| ------------------------------------------------------------ | ---------- | ----- | ------------------------------------------------------- | ------------------------------------------------------------ |
| target_system                                                | `uint8_t`  |       |                                                         | 系统 ID                                                      |
| target_component                                             | `uint8_t`  |       |                                                         | 组件 ID                                                      |
| time_boot_ms                                                 | `uint32_t` | ms    |                                                         | 时间戳（系统启动后的时间）。                                 |
| flags                                                        | `uint16_t` |       | [GIMBAL_DEVICE_FLAGS](#GIMBAL_DEVICE_FLAGS)             | 当前设置的万向节标志。                                       |
| q                                                            | `float[4]` |       |                                                         | 四元数分量，w、x、y、z（1 0 0 0 为空旋转）。帧在信息描述中进行了说明。 |
| angular_velocity_x                                           | `float`    | rad/s | invalid:NaN                                             | 角速度的 X 分量（正值：向右滚动）。帧描述见信息描述。如果未知，则为 NaN。 |
| angular_velocity_y                                           | `float`    | rad/s | invalid:NaN                                             | 角速度的 Y 分量（正值：向上翻滚）。该帧已在信息描述中说明。如果未知，则为 NaN。 |
| angular_velocity_z                                           | `float`    | rad/s | invalid:NaN                                             | 角速度的 Z 分量（正值：向右偏航）。帧在信息描述中说明。如果未知，则为 NaN。 |
| failure_flags                                                | `uint32_t` |       | [GIMBAL_DEVICE_ERROR_FLAGS](#GIMBAL_DEVICE_ERROR_FLAGS) | 故障标志（0 表示无故障）。                                   |
| <span class='ext'>delta_yaw</span> <a href='#mav2_extension_field'>++</a> | `float`    | rad   | invalid:NAN                                             | 与地球帧和主体帧四元数相关的偏航角（见信息描述）。如果未知，则为 NaN。 |
| <span class='ext'>delta_yaw_velocity</span> <a href='#mav2_extension_field'>++</a> | `float`    | rad/s | invalid:NAN                                             | 偏航角速度（Yaw angular velocity），与地球和车身框架中的角速度有关（见信息描述）。如果未知，则为 NaN。 |
| <span class='ext'>gimbal_device_id</span> <a href='#mav2_extension_field'>++</a> | `uint8_t`  |       | invalid:0                                               | 如果万向节管理器和万向节设备是同一个组件，因此组件 ID 相同，则使用该字段。该字段设置为 1-6 之间的数字。如果组件 ID 是独立的，则不需要此字段，必须设置为 0。 |


### AUTOPILOT_STATE_FOR_GIMBAL_DEVICE (286) 

包含与万向节设备相关的自动驾驶仪状态的低级别信息。该报文由自动驾驶仪发送至万向节组件。该报文的数据用于万向节设备的估算器修正，特别是地平补偿，以及显示自动驾驶仪的控制意图，例如 Z 轴的前馈角度控制。

| 字段名称                                                     | 类型       | 单位  | 数值                                                         | 说明                                                         |
| ------------------------------------------------------------ | ---------- | ----- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| target_system                                                | `uint8_t`  |       |                                                              | 系统 ID                                                      |
| target_component                                             | `uint8_t`  |       |                                                              | 组件 ID                                                      |
| time_boot_us                                                 | `uint64_t` | us    |                                                              | 时间戳（系统启动后的时间）。                                 |
| q                                                            | `float[4]` |       | 自动驾驶仪姿态的四元数分量：w、x、y、z（1 0 0 0 为空旋转，汉密尔顿惯例）。 |                                                              |
| q_estimated_delay_us                                         | `uint32_t` | us    | invalid:0                                                    | 姿态数据的估计延迟。如果未知，则为 0。                       |
| vx                                                           | `float`    | m/s   | invalid:NaN                                                  | X 速度，单位 NED（北、东、下）。未知时为 NAN。               |
| vy                                                           | `float`    | m/s   | invalid:NaN                                                  | Y 速度，NED（北、东、南）。如果未知，则为 NAN。              |
| vz                                                           | `float`    | m/s   | invalid:NaN                                                  | Z 速度，NED（北、东、南）。如果未知，则为 NAN。              |
| v_estimated_delay_us                                         | `uint32_t` | us    | invalid:0                                                    | 速度数据的估计延迟。如果未知，则为 0。                       |
| feed_forward_angular_velocity_z                              | `float`    | rad/s | invalid:NaN                                                  | 角速度的前馈 Z 分量（正：向右偏航）。NaN 将被忽略。这表示自动驾驶仪是否在主动偏航。 |
| estimator_status                                             | `uint16_t` |       | [ESTIMATOR_STATUS_FLAGS](#ESTIMATOR_STATUS_FLAGS)            | 表示哪些估计器输出有效的位图。                               |
| landed_state                                                 | `uint8_t`  |       | invalid:MAV_LANDED_STATE_UNDEFINED [MAV_LANDED_STATE](#MAV_LANDED_STATE) | 着陆状态。如果着陆状态未知，则设置为 [MAV_LANDED_STATE_UNDEFINED](#MAV_LANDED_STATE_UNDEFINED)。 |
| <span class='ext'>angular_velocity_z</span> <a href='#mav2_extension_field'>++</a> | `float`    | rad/s | invalid:NaN                                                  | NED（北、东、下）角速度的 Z 分量。如果未知，则为 NaN。       |


### GIMBAL_MANAGER_SET_PITCHYAW (287) 

设置万向节管理器俯仰和偏航角度（高速率消息）。此消息将发送到万向节管理器（例如从地面站发送），并将被万向节设备忽略。角度和速率可以根据使用情况设置为 NaN。使用 [MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW](#MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW) 进行需要确认的低速率调整。

| 字段名称         | 类型       | 单位  | 值                                            | 说明                                                         |
| ---------------- | ---------- | ----- | --------------------------------------------- | ------------------------------------------------------------ |
| target_system    | `uint8_t`  |       |                                               | 系统 ID                                                      |
| target_component | `uint8_t`  |       |                                               | 组件 ID                                                      |
| flags            | `uint32_t` |       | [GIMBAL_MANAGER_FLAGS](#GIMBAL_MANAGER_FLAGS) | 要使用的高级万向节管理器标志。                               |
| gimbal_device_id | `uint8_t`  |       |                                               | 要寻址的万向节设备的组件 ID（非 MAVLink 万向节为 1-6），所有万向节设备组件为 0。对多个万向节（但不是所有万向节）多次发送命令。<br>具有相同值的消息来自同一来源（实例）。 |
| pitch            | `float`    | rad   | invalid:NaN                                   | 俯仰角（正数：向上，负数：向下，NaN 将被忽略）。             |
| yaw              | `float`    | rad   | invalid:NaN                                   | 偏航角（正数：向右，负数：向左，NaN 将被忽略）。             |
| pitch_rate       | `float`    | rad/s | invalid:NaN                                   | 俯仰角速率（正数：向上，负数：向下，NaN 将被忽略）。         |
| yaw_rate         | `float`    | rad/s | invalid:NaN                                   | 偏航角速率（正数：向右，负数：向左，NaN 表示忽略）。         |


### GIMBAL_MANAGER_SET_MANUAL_CONTROL (288) 

用于手动控制万向节的高级消息。角度或角速率无单位；实际速率将取决于内部万向节管理器设置/配置（例如，通过参数设置）。此消息将发送到万向节管理器（例如，从地面站）。角度和速率可以根据使用情况设置为 NaN。

| 字段名称         | 类型       | 值                                            | 说明                                                         |
| ---------------- | ---------- | --------------------------------------------- | ------------------------------------------------------------ |
| target_system    | `uint8_t`  |                                               | 系统 ID                                                      |
| target_component | `uint8_t`  |                                               | 组件 ID                                                      |
| flags            | `uint32_t` | [GIMBAL_MANAGER_FLAGS](#GIMBAL_MANAGER_FLAGS) | 高级万向节管理器标志。                                       |
| gimbal_device_id | `uint8_t`  |                                               | 要寻址的万向节设备的组件 ID（对于非 MAVLink 万向节，为 1-6），对于所有万向节设备组件，为 0。为多个万向节（但不是所有万向节）多次发送命令。<br>具有相同值的消息来自同一来源（实例）。 |
| pitch            | `float`    | invalid:NaN                                   | 俯仰角无单位（-1..1，正数：向上，负数：向下，NaN 将被忽略）。 |
| yaw              | `float`    | invalid:NaN                                   | 偏航角无单位（-1..1，正数：向右，负数：向左，NaN 将被忽略）。 |
| pitch_rate       | `float`    | invalid:NaN                                   | 俯仰角速率无单位（-1..1，正数：向上，负数：向下，NaN 将被忽略）。 |
| yaw_rate         | `float`    | invalid:NaN                                   | 偏航角速率无单位（-1..1，正数：向右，负数：向左，NaN 将被忽略）。 |


### ESC_INFO (290) — [WIP] 

<span class="warning">**WORK IN PROGRESS**: Do not use in stable production environments (it may change).</span>

低速率流式传输的 ESC 信息。建议的流式传输速率为 1Hz。有关高速率 ESC 数据，请参阅 [ESC_STATUS](#ESC_STATUS)。

| 字段名称        | 类型          | 单位  | 值                                          | 说明                                                         |
| --------------- | ------------- | ----- | ------------------------------------------- | ------------------------------------------------------------ |
| index           | `uint8_t`     |       |                                             | 此消息中第一个 ESC 的索引。minValue = 0，maxValue = 60，increment = 4。<br>具有相同值的消息来自同一源（实例）。 |
| time_usec       | `uint64_t`    | us    |                                             | 时间戳（UNIX 纪元时间或自系统启动以来的时间）。接收端可以通过检查数字的大小来推断时间戳格式（自 1970 年 1 月 1 日或自系统启动以来）。 |
| counter         | `uint16_t`    |       |                                             | 收到的数据包的计数器。                                       |
| count           | `uint8_t`     |       |                                             | 此类型的所有消息中的 ESC 总数。索引高于此值的消息字段应被忽略，因为它们包含无效数据。 |
| connection_type | `uint8_t`     |       | [ESC_CONNECTION_TYPE](#ESC_CONNECTION_TYPE) | 所有 ESC 的连接类型协议。                                    |
| info            | `uint8_t`     |       |                                             | 有关每个 ESC 的在线/离线状态的信息。                         |
| failure_flags   | `uint16_t[4]` |       | [ESC_FAILURE_FLAGS](#ESC_FAILURE_FLAGS)     | ESC 故障标志的位图。                                         |
| error_count     | `uint32_t[4]` |       |                                             |                                                              |
| temperature     | `int16_t[4]`  | cdegC | invalid:[INT16_MAX]                         | 每个 ESC 的温度。INT16_MAX：如果 ESC 未提供数据。            |


### ESC_STATUS (291) — [WIP] 

<span class="warning">**WORK IN PROGRESS**: Do not use in stable production environments (it may change).</span>

用于更高速率流式传输的 ESC 信息。建议的流式传输速率为 ~10 Hz。变化较慢的信息在 [ESC_INFO](#ESC_INFO) 中发送。它通常只应在高带宽链路上流式传输（即流式传输到配套计算机）。

| 字段名称  | 类型         | 单位 | 说明                                                         |
| --------- | ------------ | ---- | ------------------------------------------------------------ |
| index     | `uint8_t`    |      | 此消息中第一个 ESC 的索引。minValue = 0，maxValue = 60，increment = 4。<br>具有相同值的消息来自同一来源（实例）。 |
| time_usec | `uint64_t`   | us   | 时间戳（UNIX 纪元时间或自系统启动以来的时间）。接收端可以通过检查数字的大小来推断时间戳格式（自 1.1.1970 或自系统启动以来）。 |
| rpm       | `int32_t[4]` | rpm  | 每个 ESC 报告的电机 RPM（反向旋转为负）。                    |
| 电压      | `float[4]`   | V    | 从每个 ESC 测量的电压。                                      |
| 电流      | `float[4]`   | A    | 从每个 ESC 测量的电流。                                      |


### WIFI_CONFIG_AP (299) 

配置 WiFi 接入点 SSID、密码和模式。AP 会将此信息作为确认信息重新发送。也可使用 [MAV_CMD_REQUEST_MESSAGE]（#MAV_CMD_REQUEST_MESSAGE）明确请求该信息。

| 字段名称                                                     | 类型       | 值                                                           | 说明                                                         |
| ------------------------------------------------------------ | ---------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| ssid                                                         | `char[32]` |                                                              | Wi-Fi 网络名称（SSID）。空白表示设置时保持不变。当前 SSID 将作为响应发送回来。 |
| password                                                     | `char[64]` | 密码。空白表示 AP 处于打开状态。信息作为响应发回时的 MD5 哈希值。 |                                                              |
| <span class='ext'>mode</span> <a href='#mav2_extension_field'>++</a> | `int8_t`   | [WIFI_CONFIG_AP_MODE](#WIFI_CONFIG_AP_MODE)                  | WiFi 模式。                                                  |
| <span class='ext'>response</span> <a href='#mav2_extension_field'>++</a> | `int8_t`   | [WIFI_CONFIG_AP_RESPONSE](#WIFI_CONFIG_AP_RESPONSE)          | 信息接受回复（发回 GS）。                                    |


### PROTOCOL_VERSION (300) — \[from: [minimal](../messages/minimal.md#PROTOCOL_VERSION)\] [WIP] 

### AIS_VESSEL (301) 

AIS 船只的位置和信息

| 字段名称            | 类型       | 单位   | 值                                | 描述                                     |
| ------------------- | ---------- | ------ | --------------------------------- | ---------------------------------------- |
| MMSI                | `uint32_t` |        |                                   | 移动海事服务标识符，9 位十进制数字       |
| lat                 | `int32_t`  | degE7  |                                   |                                          |
| lon                 | `int32_t`  | degE7  |                                   | 经度                                     |
| COG                 | `uint16_t` | cdeg   |                                   | 地面航线                                 |
| heading             | `uint16_t` | cdeg   |                                   | 真航向                                   |
| velocity            | `uint16_t` | cm/s   |                                   | 地面速度                                 |
| turn_rate           | `int8_t`   | cdeg/s |                                   | 转弯速率                                 |
| navigational_status | `uint8_t`  |        | [AIS_NAV_STATUS](#AIS_NAV_STATUS) | 导航状态                                 |
| type                | `uint8_t`  |        | [AIS_TYPE](#AIS_TYPE)             | 船舶类型                                 |
| dimension_bow       | `uint16_t` | m      |                                   |                                          |
| dimension_stern     | `uint16_t` | m      |                                   | 从纬度/长位置到船尾的距离                |
| dimension_port      | `uint8_t`  | m      |                                   | 纬度/长线位置到左舷的距离                |
| dimension_starboard | `uint8_t`  | m      |                                   | 从纬/列位置到右舷的距离                  |
| callsign            | `char[7]`  |        |                                   | 船舶呼号                                 |
| name                | `char[20]` |        |                                   | 船名                                     |
| tslc                | `uint16_t` | s      |                                   | 上次通信后的时间，以秒为单位             |
| flags               | `uint16_t` |        | [AIS_FLAGS](#AIS_FLAGS)           | 表示各种状态（包括有效数据字段）的位掩码 |


### UAVCAN_NODE_STATUS (310) 

UAVCAN 节点的一般状态信息。有关背景信息，请参阅 UAVCAN 消息 "uavcan.protocol.NodeStatus "的定义。UAVCAN 规范见 http://uavcan.org。

| 字段名称                    | 类型       | 单位 | 值                                        | 说明                                                         |
| --------------------------- | ---------- | ---- | ----------------------------------------- | ------------------------------------------------------------ |
| time_usec                   | `uint64_t` | us   |                                           | 时间戳（UNIX 时间或系统启动后的时间）。接收端可以通过检查数字的大小来推断时间戳格式（自 1.1.1970 起或自系统启动起）。 |
| uptime_sec                  | `uint32_t` | s    |                                           | 节点启动后的时间。                                           |
| health                      | `uint8_t`  |      | [UAVCAN_NODE_HEALTH](#UAVCAN_NODE_HEALTH) | 通用节点健康状态。                                           |
| mode                        | `uint8_t`  |      | [UAVCAN_NODE_MODE](#UAVCAN_NODE_MODE)     | 通用工作模式。                                               |
| sub_mode                    | `uint8_t`  |      |                                           | 当前未使用。                                                 |
| vendor_specific_status_code | `uint16_t` |      |                                           | 特定于供应商的状态信息。                                     |


### UAVCAN_NODE_INFO (311) 

描述特定 UAVCAN 节点的一般信息。有关背景信息，请参阅 UAVCAN 服务 "uavcan.protocol.GetNodeInfo "的定义。每当有新节点上线或现有节点重启时，系统都会发出这条信息。此外，当 MAVLink 信道（参见 [MAV_CMD_UAVCAN_GET_NODE_INFO](#MAV_CMD_UAVCAN_GET_NODE_INFO)）的另一端提出请求时，系统也会发出该消息。此外，也不禁止无条件地低频发射该信息。UAVCAN 规范见 http://uavcan.org。

| 字段名称         | 类型          | 单位 | 说明                                                         |
| ---------------- | ------------- | ---- | ------------------------------------------------------------ |
| time_usec        | `uint64_t`    | us   | 时间戳（UNIX 时间或系统启动后的时间）。接收端可以通过检查数字的大小来推断时间戳格式（自 1.1.1970 起或自系统启动起）。 |
| uptime_sec       | `uint32_t`    | s    | 节点启动后的时间。                                           |
| name             | `char[80]`    |      | 节点名称字符串。例如，"sapog.px4.io"。                       |
| hw_version_major | `uint8_t`     |      | 硬件主版本号。                                               |
| hw_version_minor | `uint8_t`     |      | 硬件次版本号。                                               |
| hw_unique_id     | `uint8_t[16]` |      | 硬件唯一的 128 位 ID。                                       |
| sw_version_major | `uint8_t`     |      | 软件主版本号。                                               |
| sw_version_minor | `uint8_t`     |      | 软件次版本号。                                               |
| sw_vcs_commit    | `uint32_t`    |      | 版本控制系统（VCS）版本标识符（例如 git 短提交哈希值）。未知时为 0。 |


### PARAM_EXT_REQUEST_READ (320) 

请求读取带有 param_id 字符串 id 或 param_index 的参数值。[PARAM_EXT_VALUE](#PARAM_EXT_VALUE)应作为响应。

| 字段名称         | 类型       | 说明                                                         |
| ---------------- | ---------- | ------------------------------------------------------------ |
| target_system    | `uint8_t`  | 系统 ID                                                      |
| target_component | `uint8_t`  | 组件 ID                                                      |
| param_id         | `char[16]` | 参数 ID，如果长度小于 16 个人类可读字符，则以 NULL 结尾；如果长度正好是 16 个字符，则不以空字节（NULL）结尾 - 如果 ID 以字符串形式存储，应用程序必须提供 16+1 字节的存储空间 |
| param_index      | `int16_t`  | 参数索引。设置为 -1，将使用参数 ID 字段作为标识符（否则 param_id 将被忽略）。 |


### PARAM_EXT_REQUEST_LIST (321) 

请求此组件的所有参数。所有参数都应以 [PARAM_EXT_VALUE](#PARAM_EXT_VALUE)的形式发送。

| 字段名称         | 类型      | 说明    |
| ---------------- | --------- | ------- |
| target_system    | `uint8_t` | 系统 ID |
| target_component | `uint8_t` | 组件 ID |


### PARAM_EXT_VALUE (322) 

发送参数值。在报文中加入 param_count 和 param_index 可使收件人跟踪已收到的参数，并允许他们在丢失或超时后重新请求丢失的参数。

| 字段名称    | 类型        | 值                                        | 说明                                                         |
| ----------- | ----------- | ----------------------------------------- | ------------------------------------------------------------ |
| param_id    | `char[16]`  |                                           | 参数 ID，如果长度小于 16 个人类可读字符，则以 NULL 结尾；如果长度正好是 16 个字符，则不以空字节（NULL）结尾 - 如果 ID 以字符串形式存储，应用程序必须提供 16+1 字节的存储空间 |
| param_value | `char[128]` |                                           | 参数值                                                       |
| param_type  | `uint8_t`   | [MAV_PARAM_EXT_TYPE](#MAV_PARAM_EXT_TYPE) | 参数类型。                                                   |
| param_count | `uint16_t`  | 参数总数                                  |                                                              |
| param_index | `uint16_t`  | 该参数的索引                              |                                                              |


### PARAM_EXT_SET (323) 

设置参数值 为了处理消息丢失（以及 [PARAM_EXT_SET](#PARAM_EXT_SET)的重传），在设置参数值且新值与当前值相同时，将立即收到 [PARAM_ACK_ACCEPTED](#PARAM_ACK_ACCEPTED) 响应。如果当前状态是[PARAM_ACK_IN_PROGRESS](#PARAM_ACK_IN_PROGRESS)，则会相应地收到[PARAM_ACK_IN_PROGRESS](#PARAM_ACK_IN_PROGRESS)响应。

| 字段名称         | 类型        | 值                                                           | 说明       |
| ---------------- | ----------- | ------------------------------------------------------------ | ---------- |
| target_system    | `uint8_t`   |                                                              | 系统 ID    |
| target_component | `uint8_t`   |                                                              | 组件 ID    |
| param_id         | `char[16]`  | 参数 ID，如果长度小于 16 个人类可读字符，则以 NULL 结尾；如果长度正好是 16 个字符，则不以空字节（NULL）结尾 - 如果 ID 以字符串形式存储，应用程序必须提供 16+1 字节的存储空间 |            |
| param_value      | `char[128]` |                                                              | 参数值     |
| param_type       | `uint8_t`   | [MAV_PARAM_EXT_TYPE](#MAV_PARAM_EXT_TYPE)                    | 参数类型。 |


### PARAM_EXT_ACK (324) 

PARAM_EXT_SET](#PARAM_EXT_SET) 消息的响应。

| 字段名称     | 类型       | 值                                        | 说明                                                         |
| ------------ | ---------- | ----------------------------------------- | ------------------------------------------------------------ |
| param_id     | `char[16]` |                                           | 参数 ID，如果长度小于 16 个人类可读字符，则以 NULL 结尾；如果长度正好是 16 个字符，则不以空字节（NULL）结尾 - 如果 ID 以字符串形式存储，应用程序必须提供 16+1 字节的存储空间 |
| param_value  | `cha[128]` |                                           | 参数值（如果 [PARAM_ACK_ACCEPTED](#PARAM_ACK_ACCEPTED) 为新值，否则为当前值 |
| param_type   | `uint8_t`  | [MAV_PARAM_EXT_TYPE](#MAV_PARAM_EXT_TYPE) | 参数类型。                                                   |
| param_result | `uint8_t`  | [PARAM_ACK](#PARAM_ACK)                   | 结果代码。                                                   |


### OBSTACLE_DISTANCE (330) 

传感器前方的障碍物距离，从左边开始向右递增

| 字段名称                                                     | 类型           | 单位 | 值                                          | 说明                                                         |
| ------------------------------------------------------------ | -------------- | ---- | ------------------------------------------- | ------------------------------------------------------------ |
| time_usec                                                    | `uint64_t`     | us   |                                             | 时间戳（UNIX 时间或系统启动后的时间）。接收端可以通过检查数字的大小来推断时间戳格式（自 1.1.1970 起或自系统启动起）。 |
| sensor_type                                                  | `uint8_t`      |      | [MAV_DISTANCE_SENSOR](#MAV_DISTANCE_SENSOR) | 距离传感器类型的类 id。                                      |
| distances                                                    | `uint16_t[72]` | cm   | invalid:[UINT16_MAX]                        | 车辆周围障碍物的距离，除非帧中另有规定，否则索引 0 对应于 north + angle_offset。值为 0 有效，表示障碍物几乎接触到传感器。max_distance +1 的值表示没有障碍物。UINT16_MAX 表示未知/未使用。在数组元素中，一个单位对应 1 厘米。 |
| increment                                                    | `uint8_t`      | deg  |                                             | 每个数组元素的角度宽度（度）。增量方向为顺时针。如果 increment_f 非零，则忽略此字段。 |
| min_distance                                                 | `uint16_t`     | cm   |                                             | 传感器可以测量的最小距离。                                   |
| max_distance                                                 | `uint16_t`     | cm   |                                             | 传感器可以测量的最大距离。                                   |
| <span class='ext'>increment_f</span> <a href='#mav2_extension_field'>++</a> | `float`        | deg  |                                             | 每个数组元素的角度宽度（以度为单位），单位为浮点数。如果非零，则使用该值代替 uint8_t 增量字段。正值表示顺时针方向，负值表示逆时针方向。 |
| <span class='ext'>angle_offset</span> <a href='#mav2_extension_field'>++</a> | `float`        | deg  |                                             | 距离数组中 0 索引元素的相对角度偏移。0 表示向前。正值表示顺时针方向，负值表示逆时针方向。 |
| <span class='ext'>frame</span> <a href='#mav2_extension_field'>++</a> | `uint8_t`      |      | [MAV_FRAME](#MAV_FRAME)                     | 传感器数据偏航旋转和偏移的参照坐标系。默认为 [MAV_FRAME_GLOBAL](#MAV_FRAME_GLOBAL)，即向北对齐。对于车身安装的传感器，使用[MAV_FRAME_BODY_FRD](#MAV_FRAME_BODY_FRD)，即车辆前方对齐。 |


### ODOMETRY (331) 

飞行高度信息，用于与外部接口交流飞行高度信息。符合 ROS REP 147 航空飞行器标准 (http://www.ros.org/reps/rep-0147.html)。

| 字段名称                                                     | 类型        | 单位  | 值                                        | 说明                                                         |
| ------------------------------------------------------------ | ----------- | ----- | ----------------------------------------- | ------------------------------------------------------------ |
| time_usec                                                    | `uint64_t`  | us    |                                           | 时间戳（UNIX 时间或系统启动后的时间）。接收端可以通过检查数字的大小来推断时间戳格式（自 1.1.1970 起或自系统启动起）。 |
| frame_id                                                     | `uint8_t`   |       | [MAV_FRAME](#MAV_FRAME)                   | 姿态数据的参照坐标系。                                       |
| child_frame_id                                               | `uint8_t`   |       | [MAV_FRAME](#MAV_FRAME)                   | 自由空间速度（扭曲）数据的参照坐标系。                       |
| x                                                            | `float`     | m     |                                           |                                                              |
| y                                                            | `float`     | m     |                                           | Y 位置                                                       |
| z                                                            | `float`     | m     |                                           | Z 位置                                                       |
| q                                                            | `float[4]`  |       |                                           | 四元数分量，w、x、y、z（1 0 0 0 为空旋转）                   |
| vx                                                           | `float`     | m/s   |                                           | X线速度                                                      |
| vy                                                           | `float`     | m/s   |                                           | Y 线速度                                                     |
| vz                                                           | `float`     | m/s   |                                           | Z 线速度                                                     |
| rollspeed                                                    | `float`     | rad/s |                                           | 滚动角速度                                                   |
| pitchspeed                                                   | `float`     | rad/s |                                           | 螺距角速度                                                   |
| yawspeed                                                     | `float`     | rad/s |                                           | 偏航角速度                                                   |
| pose_covariance                                              | `float[21]` |       | invalid:[NaN:]                            | 6x6 pose 交叉协方差矩阵右上角三角形的行主表示（状态：x, y, z, roll, pitch, yaw；前六项为第一行，后五项为第二行，以此类推）。如果未知，则给数组中的第一个元素赋 NaN 值。 |
| velocity_covariance                                          | `float[21]` |       | invalid:[NaN:]                            | 6x6 速度交叉协方差矩阵右上角三角形的行主表示（状态：vx、vy、vz、rollspeed、pitchspeed、yawspeed；前六个条目为第一行，后五个条目为第二行，等等）。如果未知，则将 NaN 值赋值给数组中的第一个元素。 |
| <span class='ext'>reset_counter</span> <a href='#mav2_extension_field'>++</a> | `uint8_t`   |       |                                           | 估计值重置计数器。当估计值在任何维度（位置、速度、姿态、角速度）重置时，该计数器都应递增。该计数器用于外部 SLAM 系统检测到环路闭合和估计值跳变等情况。 |
| <span class='ext'>estimator_type</span> <a href='#mav2_extension_field'>++</a> | `uint8_t`   |       | [MAV_ESTIMATOR_TYPE](#MAV_ESTIMATOR_TYPE) | 提供轨迹测量的估算器类型。                                   |
| <span class='ext'>quality</span> <a href='#mav2_extension_field'>++</a> | `int8_t`    | %     | invalid:0                                 | 以百分比表示的可选里程测量质量指标。-1 = 测距失败，0 = 质量未知/不确定，1 = 质量最差，100 = 质量最好 |


### TRAJECTORY_REPRESENTATION_WAYPOINTS (332) 

使用本地帧（[MAV_FRAME_LOCAL_NED](#MAV_FRAME_LOCAL_NED)）中最多 5 个航点的数组来描述轨迹。

| 字段名称     | 类型          | 单位  | 值                                            | 说明                                                         |
| ------------ | ------------- | ----- | --------------------------------------------- | ------------------------------------------------------------ |
| time_usec    | `uint64_t`    | us    |                                               | 时间戳（UNIX 时间或系统启动后的时间）。接收端可以通过检查数字的大小来推断时间戳格式（自 1.1.1970 起或自系统启动起）。 |
| valid_points | `uint8_t`     |       |                                               | 有效点数（最多可有 5 个航点）                                |
| pos_x        | `float[5]`    | m     | invalid:[NaN]                                 | 航点的 X 坐标，如果不使用，则设为 NaN                        |
| pos_y        | `float[5]`    | m     | invalid:[NaN]                                 | 航点的 Y 坐标，如果未使用，则设置为 NaN                      |
| pos_z        | `float[5]`    | m     | invalid:[NaN]                                 | 航点的 Z 坐标，如果未使用，则设置为 NaN                      |
| vel_x        | `float[5]`    | m/s   | invalid:[NaN]                                 | 航点的 X 速度，如果未使用，则设置为 NaN                      |
| vel_y        | `float[5]`    | m/s   | invalid:[NaN]                                 | 航点的 Y 速度，如果未使用，则设置为 NaN                      |
| vel_z        | `float[5]`    | m/s   | invalid:[NaN]                                 | 航点的 Z 速度，如果未使用，则设置为 NaN                      |
| acc_x        | `float[5]`    | m/s/s | invalid:[NaN]                                 | 航点的 X 加速，如果未使用，则设置为 NaN                      |
| acc_y        | `float[5]`    | m/s/s | invalid:[NaN]                                 | 航点的 Y 加速度，如果未使用，则设置为 NaN                    |
| acc_z        | `float[5]`    | m/s/s | invalid:[NaN]                                 | 航点的 Z 加速，如果未使用，则设置为 NaN。                    |
| pos_yaw      | `float[5]`    | rad   | invalid:[NaN]                                 | 偏航角度，如果未使用，则设置为 NaN                           |
| vel_yaw      | `float[5]`    | rad/s | invalid:[NaN]                                 | 偏航速率，如果未使用，则设置为 NaN                           |
| command      | `uint16_t[5]` |       | invalid:[UINT16_MAX] [MAV_CMD](#mav_commands) | [MAV_CMD](#mav_commands) 航点命令 ID，如果未使用，则设置为 UINT16_MAX。 |


### TRAJECTORY_REPRESENTATION_BEZIER (333) 

使用本地帧（[MAV_FRAME_LOCAL_NED](#MAV_FRAME_LOCAL_NED)）中最多 5 个贝塞尔控制点的数组来描述轨迹。

| 字段名称     | 类型       | 单位 | 说明                                                         |
| ------------ | ---------- | ---- | ------------------------------------------------------------ |
| time_usec    | `uint64_t` | us   | 时间戳（UNIX 时间或系统启动后的时间）。接收端可以通过检查数字的大小来推断时间戳格式（自 1.1.1970 起或自系统启动起）。 |
| valid_points | `uint8_t`  |      | 有效控制点的数量（最多可有 5 个控制点）                      |
| pos_x        | `float[5]` | m    | 贝塞尔控制点的 X 坐标。如果不使用，则设置为 NaN              |
| pos_y        | `float[5]` | m    | 贝塞尔控制点的 Y 坐标。如果不使用，则设置为 NaN              |
| pos_z        | `float[5]` | m    | 贝塞尔控制点的 Z 坐标。如果不使用，则设置为 NaN              |
| delta        | `float[5]` | s    | 贝塞尔时间范围。如果不包含速度/加速度，则设置为 NaN          |
| pos_yaw      | `float[5]` | rad  | 偏航。不变时设置为 NaN                                       |


### CELLULAR_STATUS (334) 

报告当前使用的蜂窝网络状态

| 字段名称       | 类型       | 值                                                           | 说明                                                         |
| -------------- | ---------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| status         | `uint8_t`  | [CELLULAR_STATUS_FLAG](#CELLULAR_STATUS_FLAG)                | 蜂窝调制解调器状态                                           |
| failure_reason | `uint8_t`  | [CELLULAR_NETWORK_FAILED_REASON](#CELLULAR_NETWORK_FAILED_REASON) | 状态在 [CELLULAR_STATUS_FLAG_FAILED](#CELLULAR_STATUS_FLAG_FAILED) 时的失败原因 |
| type           | `uint8_t`  | [CELLULAR_NETWORK_RADIO_TYPE](#CELLULAR_NETWORK_RADIO_TYPE)  | 蜂窝网络无线电类型：GSM、CDMA、LTE...                        |
| quality        | `uint8_t`  | invalid:UINT8_MAX                                            | 信号质量（百分比）。如果未知，则设置为 UINT8_MAX             |
| mcc            | `uint16_t` | invalid:UINT16_MAX                                           | 移动国家代码。如果未知，则设置为 UINT16_MAX                  |
| mnc            | `uint16_t` | invalid:UINT16_MAX                                           | 移动网络代码。如果未知，则设置为 UINT16_MAX                  |
| lac            | `uint16_t` | invalid:0                                                    | 所在地区代码。如果未知，则设置为 0                           |

### ISBD_LINK_STATUS (335) 

铱星 SBD 链路的状态。

| 字段名称            | 类型       | 单位 | 说明                                                         |
| ------------------- | ---------- | ---- | ------------------------------------------------------------ |
| timestamp           | `uint64_t` | us   | 时间戳（UNIX 纪元时间或系统启动后的时间）。接收端可以通过检查数字的大小来推断时间戳格式（自 1.1.1970 起或自系统启动起）。 |
| last_heartbeat      | `uint64_t` | us   | sbd 会话最后一次成功的时间戳。接收端可以通过检查数字的大小来推断时间戳格式（自 1.1.1970 起或自系统启动起）。 |
| failed_sessions     | `uint16_t` |      | 失败的 SBD 会话数。                                          |
| successful_sessions | `uint16_t` |      | 成功的 SBD 会话数。                                          |
| signal_quality      | `uint8_t`  |      | 信号质量等于 ISU 信号强度指示器上显示的条数。范围为 0 到 5，其中 0 表示无信号，5 表示最大信号强度。 |
| ring_pending        | `uint8_t`  |      | 1: Ring call pending, 0: No call pending.                    |
| tx_session_pending  | `uint8_t`  |      | 1：传输会话挂起，0：无传输会话挂起。                         |
| rx_session_pending  | `uint8_t`  |      | 1: 接收会话挂起，0: 无接收会话挂起。                         |


### CELLULAR_CONFIG (336) 

配置蜂窝调制解调器。

调制解调器会重新发送该信息作为确认。
也可以使用 [MAV_CMD_REQUEST_MESSAGE](#MAV_CMD_REQUEST_MESSAGE)明确请求该信息。

| 字段名称   | 类型       | 值                                                    | 说明                                                         |
| ---------- | ---------- | ----------------------------------------------------- | ------------------------------------------------------------ |
| enable_lte | `uint8_t`  |                                                       | 启用/禁用 LTE。0：设置不变，1：禁用，2：启用。当前设置作为响应发送回来。 |
| enable_pin | `uint8_t`  |                                                       | 启用/禁用 SIM 卡上的 PIN 码。0：设置不变，1：禁用，2：启用。当前设置作为响应发回。 |
| pin        | `char[16]` |                                                       | 发送到 SIM 卡的 PIN 码。禁用 PIN 时为空。信息作为响应发回时为空。 |
| new_pin    | `char[16]` |                                                       | 更改 PIN 码时的新 PIN 码。空白表示保持不变。当信息作为回复发回时为空。 |
| apn        | `char[32]` |                                                       | 蜂窝 APN 名称。空白，保持不变。当信息作为回复发回时为当前 APN。 |
| PUK        | `char[16]` |                                                       | 必要的 PUK 码，以防用户使用 PIN 验证 3 次失败。信息作为回复发回时为空。 |
| roaming    | `uint8_t`  |                                                       | 启用/禁用漫游。0：设置不变，1：禁用，2：启用。当前设置作为回复发送。 |
| response   | `uint8_t`  | [CELLULAR_CONFIG_RESPONSE](#CELLULAR_CONFIG_RESPONSE) | 消息接受响应（发回 GS）。                                    |


### RAW_RPM (339) 

转速传感器数据信息。

| 字段名称 | 类型      | 单位 | 说明                          |
| -------- | --------- | ---- | ----------------------------- |
| index    | `uint8_t` |      | 该 RPM 传感器的索引（0-索引） |
| 频率     | `float`   | rpm  | 指示速率                      |


### UTM_GLOBAL_POSITION (340) 

GPS 和传感器融合后的全球位置。

| 字段名称     | 类型          | 单位  | 值                                            | 说明                                                       |
| ------------ | ------------- | ----- | --------------------------------------------- | ---------------------------------------------------------- |
| time         | `uint64_t`    | us    |                                               | 定位的适用时间（自 UNIX epoch 起的微秒数）。               |
| uas_id       | `uint8_t[18]` |       |                                               | 唯一的 UAS ID。                                            |
| lat          | `int32_t`     | degE7 |                                               |                                                            |
| lon          | `int32_t`     | degE7 |                                               | 经度 (WGS84)                                               |
| alt          | `int32_t`     | mm    |                                               | 高度 (WGS84)                                               |
| relative_alt | `int32_t`     | mm    |                                               | 离地高度                                                   |
| vx           | `int16_t`     | cm/s  |                                               | 地面 X 速度（纬度，正北方向）                              |
| vy           | `int16_t`     | cm/s  |                                               | 地面 Y 速度（经度，正东方向）                              |
| vz           | `int16_t`     | cm/s  |                                               | 地面 Z 速度（高度，正下方）                                |
| h_acc        | `uint16_t`    | mm    |                                               | 水平位置不确定性（标准偏差）                               |
| v_acc        | `uint16_t`    | mm    |                                               | 高度不确定性（标准偏差）                                   |
| vel_acc      | `uint16_t`    | cm/s  |                                               | 速度不确定性（标准偏差）                                   |
| next_lat     | `int32_t`     | degE7 |                                               | 下一个航点，纬度 (WGS84)                                   |
| next_lon     | `int32_t`     | degE7 |                                               | 下一个航点，经度 (WGS84)                                   |
| next_alt     | `int32_t`     | mm    |                                               | 下一个航点，高度 (WGS84)                                   |
| update_rate  | `uint16_t`    | cs    | invalid:0                                     | 距离下次更新的时间。如果未知或处于数据驱动模式，则设为 0。 |
| flight_state | `uint8_t`     |       | [UTM_FLIGHT_STATE](#UTM_FLIGHT_STATE)         | 飞行状态                                                   |
| flags        | `uint8_t`     |       | [UTM_DATA_AVAIL_FLAGS](#UTM_DATA_AVAIL_FLAGS) | 数据可用标志的位或组合。                                   |


### DEBUG_FLOAT_ARRAY (350) 

大型调试/原型阵列。消息使用最大可用有效载荷作为数据。array_id 和 name 字段分别用于区分代码中的信息和用户界面中的信息。请勿在生产代码中使用。

| 字段名称                                                     | 类型        | 单位 | 说明                                                         |
| ------------------------------------------------------------ | ----------- | ---- | ------------------------------------------------------------ |
| time_usec                                                    | `uint64_t`  | us   | 时间戳（UNIX 时间或系统启动后的时间）。接收端可以通过检查数字的大小来推断时间戳格式（自 1.1.1970 起或自系统启动起）。 |
| name                                                         | `char[10]`  |      | 名称，用于地面控制站的人性化显示                             |
| array_id                                                     | `uint16_t`  |      | 用于区分数组的唯一 ID<br>具有相同值的信息来自同一来源（实例）。 |
| <span class='ext'>data</span> <a href='#mav2_extension_field'>++</a> | `float[58]` |      | data                                                         |


### ORBIT_EXECUTION_STATUS (360) — [WIP] 

<span class="warning">**WORK IN PROGRESS**: Do not use in stable production environments (it may change).</span>

在轨道执行过程中发送的车辆状态报告（参见 [MAV_CMD_DO_ORBIT](#MAV_CMD_DO_ORBIT)）。

| 字段名称  | 类型       | 单位 | 值                      | 说明                                                         |
| --------- | ---------- | ---- | ----------------------- | ------------------------------------------------------------ |
| time_usec | `uint64_t` | us   |                         | 时间戳（UNIX 时间或系统启动后的时间）。接收端可以通过检查数字的大小来推断时间戳格式（自 1.1.1970 起或自系统启动起）。 |
| radius    | `float`    | m    |                         | 轨道圆的半径。正值为顺时针，负值为逆时针。                   |
| frame     | `uint8_t`  |      | [MAV_FRAME](#MAV_FRAME) | 域的坐标系：x、y、z。                                        |
| x         | `int32_t`  |      |                         | 中心点的 X 坐标。坐标系取决于帧字段：本地 = x 位置（米）* 1e4，全局 = 纬度（度）* 1e7。 |
| y         | `int32_t`  |      |                         | 中心点的 Y 坐标。 坐标系取决于帧域：本地 = x 位置（单位：米）* 1e4，全局 = 纬度（单位：度）* 1e7。 |
| z         | `float`    | m    |                         | 中心点的高度。坐标系取决于帧场。                             |


### BATTERY_INFO (370) — [WIP] 

<span class="warning">**WORK IN PROGRESS**: Do not use in stable production environments (it may change).</span>

电池信息是静态的，或者不需要频繁更新。
此消息应使用 [MAV_CMD_REQUEST_MESSAGE](#MAV_CMD_REQUEST_MESSAGE) 请求和/或以非常低的速率传输。
[BATTERY_STATUS_V2](#BATTERY_STATUS_V2) 用于更高速率的电池状态信息。

| 字段名称                        | 类型       | 单位 | 值                                            | 描述                                                         |
| ------------------------------- | ---------- | ---- | --------------------------------------------- | ------------------------------------------------------------ |
| id                              | `uint8_t`  |      |                                               | 电池 ID<br>具有相同值的消息来自同一来源（实例）。            |
| battery_function                | `uint8_t`  |      | [MAV_BATTERY_FUNCTION](#MAV_BATTERY_FUNCTION) | 电池的功能。                                                 |
| type                            | `uint8_t`  |      | [MAV_BATTERY_TYPE](#MAV_BATTERY_TYPE)         | 电池的类型（化学性质）。                                     |
| state_of_health                 | `uint8_t`  | %    | invalid:UINT8_MAX                             | 健康状态（SOH）估计值。制造时通常为 100%，会随着时间和使用而降低。-1：未提供字段。 |
| cells_in_series                 | `uint8_t`  |      | invalid:0                                     | 串联电池单元数。0：未提供字段。                              |
| cycle_count                     | `uint16_t` |      | invalid:UINT16_MAX                            | 充电/放电循环次数的使用寿命计数（https://en.wikipedia.org/wiki/Charge_cycle）。UINT16_MAX：未提供字段。 |
| weight                          | `uint16_t` | g    | invalid:0                                     | 电池重量。0：未提供字段。                                    |
| discharge_minimum_voltage       | `float`    | V    | invalid:0                                     | 放电时的最小单电池电压。0：未提供字段。                      |
| charging_minimum_voltage        | `float`    | V    | invalid:0                                     | 充电时每个电池的最小电压。0：未提供字段。                    |
| resting_minimum_voltage         | `float`    | V    | invalid:0                                     | 静止时每个电池的最小电压。0：未提供字段。                    |
| Charging_maximum_voltage        | `float`    | V    | invalid:0                                     | 充电时每个电池的最大电压。0：未提供字段。                    |
| Charging_maximum_current        | `float`    | A    | invalid:0                                     | 最大电池组连续充电电流。0：未提供字段。                      |
| Nominal_voltage                 | `float`    | V    | invalid:0                                     | 电池标称电压。用于 Wh 和 Ah 之间的转换。0：未提供字段。      |
| discharge_maximum_current       | `float`    | A    | invalid:0                                     | 最大电池组放电电流。0：未提供字段。                          |
| discharge_maximum_burst_current | `float`    | A    | invalid:0                                     | 最大电池组放电突发电流。0：未提供字段。                      |
| design_capacity                 | `float`    | Ah   | invalid:0                                     | 充满电时的设计容量。0：未提供字段。                          |
| full_charge_capacity            | `float`    | Ah   | invalid:NaN                                   | 充满电时预测的电池容量（考虑到电池退化）。NAN：未提供字段。  |
| manufacture_date                | `char[9]`  |      | invalid:[0]                                   | 以 ASCII 字符表示的制造日期 (DDMMYYYY)，以 0 结尾。全部 0：未提供字段。 |
| serial_number                   | `char[32]` |      | invalid:[0]                                   | 以 ASCII 字符表示的序列号，以 0 结尾。全部 0：未提供字段。   |
| name                            | `char[50]` |      | invalid:[0]                                   | 电池设备名称。格式为制造商名称然后是产品名称，用下划线分隔（以 ASCII 字符表示），以 0 结尾。全部 0：未提供字段。 |


### GENERATOR_STATUS (373) 

发电系统遥测。交流发电机或机械发电机。

| 字段名称               | 类型       | 单位 | 值                                                      | 说明                                                         |
| ---------------------- | ---------- | ---- | ------------------------------------------------------- | ------------------------------------------------------------ |
| status                 | `uint64_t` |      | [MAV_GENERATOR_STATUS_FLAG](#MAV_GENERATOR_STATUS_FLAG) | 状态标志。                                                   |
| generator_speed        | `uint16_t` | rpm  | invalid:UINT16_MAX                                      | 发电机或交流发电机的转速。UINT16_MAX：未提供字段。           |
| battery_current        | `float`    | A    | invalid:NaN                                             | 流入/流出电池的电流。正表示输出。负表示输入。NaN：未提供字段。 |
| load_current           | `float`    | A    | invalid:NaN                                             | 进入 UAV 的电流。如果没有电池电流，则为来自发电机的直流电流。正表示输出。负表示输入。NaN：未提供字段 |
| power_generated        | `float`    | W    | invalid:NaN                                             | 生成的功率。NaN：未提供字段                                  |
| bus_voltage            | `float`    | V    |                                                         | 在发电机上看到的总线电压，如果电池总线由发电机控制，且电压与主总线不同，则为电池总线电压。 |
| rectifier_temperature  | `int16_t`  | degC | invalid:INT16_MAX                                       | 整流器或功率转换器的温度。INT16_MAX：未提供字段。            |
| bat_current_setpoint   | `float`    | A    | invalid:NaN                                             | 目标电池电流。正值表示输出。负值表示输入。NaN：未提供字段    |
| generator_temperature  | `int16_t`  | degC | invalid:INT16_MAX                                       | 机械发动机、燃料电池芯或发电机的温度。INT16_MAX：未提供字段。 |
| runtime                | `uint32_t` | s    | invalid:UINT32_MAX                                      | 电源箱重启后的运行秒数。UINT32_MAX：未提供字段。             |
| time_until_maintenance | `int32_t`  | s    | invalid:INT32_MAX                                       | 该电源箱需要维护前的秒数。 负值表示维护时间已过。INT32_MAX：未提供字段。 |


### ACTUATOR_OUTPUT_STATUS (375) 

执行器输出的原始值（例如 Pixhawk 上的 MAIN、AUX 端口）。该信息取代 [SERVO_OUTPUT_RAW]（#SERVO_OUTPUT_RAW）。

| 字段名称  | 类型        | 单位 | 说明                                        |
| --------- | ----------- | ---- | ------------------------------------------- |
| time_usec | `uint64_t`  | us   | 时间戳（系统启动后）。                      |
| active    | `uint32_t`  |      | 有效输出                                    |
| actuator  | `float[32]` |      | 伺服/电机输出阵列值。零值表示未使用的通道。 |


### TIME_ESTIMATE_TO_TARGET (380) — [WIP] 

<span class="warning">**WORK IN PROGRESS**: Do not use in stable production environments (it may change).</span>

在当前车辆状态和位置下，各种事件和操作的时间/持续时间估计值。

| 字段名称          | 类型      | 单位 | 说明                                                         |
| ----------------- | --------- | ---- | ------------------------------------------------------------ |
| safe_return       | `int32_t` | s    | 从当前位置（如 RTL、Smart RTL 等）完成车辆配置的 "安全返回 "操作的估计时间。-1表示飞行器已着陆或无时间估计。 |
| land              | `int32_t` | s    | 车辆从当前位置完成 "着陆 "操作的预计时间。-1表示车辆已着陆，或没有时间估计值。 |
| mission_next_item | `int32_t` | s    | 到达/完成当前活动任务项目的预计时间。-1表示无时间估计。      |
| mission_end       | `int32_t` | s    | 完成当前任务的预计时间。-1表示无任务和/或无估计时间。        |
| commanded_action  | `int32_t` | s    | 完成当前指令动作（即前往、起飞、着陆等）的预计时间。-1表示当前没有行动和/或没有估计时间。 |


### TUNNEL (385) 

用于将 "任意 "变长数据从一个组件传输到另一个组件的信息（不禁止广播，但不鼓励）。数据编码通常是针对特定扩展的，即由数据源决定，通常不作为 MAVLink 规范的一部分进行记录。

| 字段名称         | 类型           | 值                                                  | 描述                                                         |
| ---------------- | -------------- | --------------------------------------------------- | ------------------------------------------------------------ |
| target_system    | `uint8_t`      |                                                     | 系统 ID（广播时可以为 0，但不鼓励这样做）                    |
| target_component | `uint8_t`      |                                                     | 组件 ID（广播时可以为 0，但不鼓励这样做）                    |
| payload_type     | `uint16_t`     | [MAV_TUNNEL_PAYLOAD_TYPE](#MAV_TUNNEL_PAYLOAD_TYPE) | 识别有效负载内容的代码（0 表示未知，这是默认值）。如果该代码小于 32768，则表示 "已注册 "有效载荷类型，相应代码应添加到[MAV_TUNNEL_PAYLOAD_TYPE](#MAV_TUNNEL_PAYLOAD_TYPE)枚举中。软件创建者可根据需要注册类型块。大于 32767 的代码将被视为本地试验，不应在任何广泛传播的代码库中使用。 |
| payload_length   | `uint8_t`      |                                                     | 有效载荷中传输数据的长度                                     |
| payload          | `uint8_t[128]` |                                                     | 有效载荷长度可变。有效载荷长度由 payload_length 定义。除非了解由 payload_type 指定的编码，否则该数据块的全部内容都是不透明的。 |


### CAN_FRAME (386) 

由 [MAV_CMD_CAN_FORWARD](#MAV_CMD_CAN_FORWARD)请求转发的 CAN 帧。

| 字段名称         | 类型         | 说明      |
| ---------------- | ------------ | --------- |
| target_system    | `uint8_t`    | 系统 ID。 |
| target_component | `uint8_t`    | 组件 ID。 |
| bus              | `uint8_t`    | 总线编号  |
| len              | `uint8_t`    | 帧长度    |
| id               | `uint32_t`   | 帧 ID     |
| 数据             | `uint8_t[8]` | 帧数据    |


### CANFD_FRAME (387) 

由[MAV_CMD_CAN_FORWARD](#MAV_CMD_CAN_FORWARD)请求转发的 CANFD 帧。这些帧与 [CAN_FRAME](#CAN_FRAME)是分开的，因为它们需要不同的处理（例如 TAO 处理）。

| 字段名称         | 类型          | 说明      |
| ---------------- | ------------- | --------- |
| target_system    | `uint8_t`     | 系统 ID。 |
| target_component | `uint8_t`     | 组件 ID。 |
| bus              | `uint8_t`     | 总线编号  |
| len              | `uint8_t`     | 帧长度    |
| id               | `uint32_t`    | 帧 ID     |
| 数据             | `uint8_t[64]` | 帧数据    |


### CAN_FILTER_MODIFY (388) 

修改通过 mavlink 转发 CAN 信息的过滤器。这可用于使 CAN 转发在低带宽链路上运行良好。过滤应用于 CAN ID 的第 8 至 24 位（第 2 和第 3 个字节），该 ID 与 DroneCAN 的 DroneCAN 报文 ID 相对应。可通过发送多个 [CAN_FILTER_MODIFY](#CAN_FILTER_MODIFY) 报文来构建 ID 超过 16 个的过滤器。

| 字段名称         | 类型           | 值                              | 说明                                                         |
| ---------------- | -------------- | ------------------------------- | ------------------------------------------------------------ |
| target_system    | `uint8_t`      |                                 | 系统 ID。                                                    |
| target_component | `uint8_t`      |                                 | 组件 ID。                                                    |
| bus              | `uint8_t`      |                                 | 总线编号                                                     |
| operation        | `uint8_t`      | [CAN_FILTER_OP](#CAN_FILTER_OP) | 对过滤器列表执行的操作。参见 [CAN_FILTER_OP](#CAN_FILTER_OP) 枚举。 |
| num_ids          | `uint8_t`      |                                 | 过滤器列表中的 ID 数量                                       |
| ids              | `uint16_t[16]` |                                 | 过滤器 ID，长度 num_ids                                      |


### ONBOARD_COMPUTER_STATUS (390) — [WIP] 

<span class="warning">**WORK IN PROGRESS**: Do not use in stable production environments (it may change).</span>

机载计算机发送的硬件状态。

| 字段名称          | 类型          | 单位  | 说明                                                         |
| ----------------- | ------------- | ----- | ------------------------------------------------------------ |
| time_usec         | `uint64_t`    | us    | 时间戳（UNIX 时间或系统启动后的时间）。接收端可以通过检查数字的大小来推断时间戳格式（自 1.1.1970 起或自系统启动起）。 |
| uptime            | `uint32_t`    | ms    | 系统启动后的时间。                                           |
| type              | `uint8_t`     |       | 机载计算机类型： 0：主要任务计算机，1：备用任务计算机1，2：备用任务计算机2，3：计算节点，4-5：备用计算节点，6-9：有效载荷计算机： 有效载荷计算机。 |
| cpu_cores         | `uint8_t[8]`  |       | 组件的 CPU 使用率，单位为百分比（100 - 空闲）。UINT8_MAX 表示该字段未使用。 |
| cpu_combined      | `uint8_t[10]` |       | 100 个 MS 中最后 10 个片段的 CPU 使用率组合（直方图）。这样就可以识别出使系统负荷达到最大、但仅持续很短时间的负载峰值。UINT8_MAX 表示该字段未使用。 |
| gpu_cores         | `uint8_t[4]`  |       | GPU 在组件上的使用百分比（100 - 空闲）。如果值为 UINT8_MAX，则表示该字段未使用。 |
| gpu_combined      | `uint8_t[10]` |       | GPU 使用率的组合，即 100 MS 的最后 10 个片段（直方图）。这样就可以识别出使系统达到最大负荷的尖峰负荷，但这种负荷只会持续很短的时间。值为 UINT8_MAX 表示该字段未使用。 |
| temperature_board | `int8_t`      | degC  | 电路板的温度。如果值为 INT8_MAX，则表示该字段未使用。        |
| temperature_core  | `int8_t[8]`   | degC  | CPU 内核的温度。INT8_MAX 表示该字段未用。                    |
| fan_speed         | `int16_t[4]`  | rpm   | 风扇转速。INT16_MAX 表示该字段未使用。                       |
| ram_usage         | `uint32_t`    | MiB   | 组件系统中已使用的 RAM 数量。UINT32_MAX 表示该字段未使用。   |
| ram_total         | `uint32_t`    | MiB   | 组件系统的 RAM 总量。如果值为 UINT32_MAX，则表示该字段未使用。 |
| storage_type      | `uint32_t[4]` |       | 存储类型： 0：HDD，1：SSD，2：EMMC，3：SD 卡（不可拆卸），4：SD 卡（可拆卸）。值为 UINT32_MAX 表示该字段未使用。 |
| storage_usage     | `uint32_t[4]` | MiB   | 组件系统中已用存储空间的大小。如果值为 UINT32_MAX，则表示该字段未使用。 |
| storage_total     | `uint32_t[4]` | MiB   | 组件系统中的存储空间总量。如果值为 UINT32_MAX，则表示该字段未使用。 |
| link_type         | `uint32_t[6]` |       | 链接类型： 0-9: UART, 10-19: 有线网络, 20-29： Wifi，30-39： 点对点专有网络，40-49：网状专有网络 |
| link_tx_rate      | `uint32_t[6]` | KiB/s | 来自组件系统的网络流量。UINT32_MAX 表示未使用该字段。        |
| link_rx_rate      | `uint32_t[6]` | KiB/s | 连接到组件系统的网络流量。UINT32_MAX 表示该字段未使用。      |
| link_tx_max       | `uint32_t[6]` | KiB/s | 来自组件系统的网络流量。UINT32_MAX 表示该字段未使用。        |
| link_rx_max       | `uint32_t[6]` | KiB/s | 连接到组件系统的网络容量。UINT32_MAX 表示该字段未使用。      |


### COMPONENT_INFORMATION (395) — [DEP] 

<span class="warning">**DEPRECATED:** Replaced By [COMPONENT_METADATA](#COMPONENT_METADATA) (2022-04)</span>

组件信息消息，可使用 [MAV_CMD_REQUEST_MESSAGE](#MAV_CMD_REQUEST_MESSAGE)请求。

| 字段名称                      | 类型        | 单位 | 说明                                                         |
| ----------------------------- | ----------- | ---- | ------------------------------------------------------------ |
| time_boot_ms                  | `uint32_t`  | ms   | 时间戳（系统启动后的时间）。                                 |
| general_metadata_file_crc     | `uint32_t`  |      | 一般元数据文件（general_metadata_uri）的 CRC32。             |
| general_metadata_uri          | `char[100]` |      | 一般元数据文件（[COMP_METADATA_TYPE_GENERAL](#COMP_METADATA_TYPE_GENERAL)）的 MAVLink FTP URI，可使用 xz 压缩。该文件包含一般组件元数据，还可能包含其他元数据的 URI 链接（请参阅 [COMP_METADATA_TYPE](#COMP_METADATA_TYPE)）。这些信息在启动时是静态的，也可能在编译时生成。字符串必须以 0 结尾。 |
| peripherals_metadata_file_crc | `uint32_t`  |      | 外围设备元数据文件（peripherals_metadata_uri）的 CRC32。     |
| peripherals_metadata_uri      | `char[100]` |      | （可选）外围设备元数据文件（[COMP_METADATA_TYPE_PERIPHERALS](#COMP_METADATA_TYPE_PERIPHERALS)）的 MAVLink FTP URI，可使用 xz 压缩。其中包含 UAVCAN 节点等 "附加组件 "的数据。由于外围设备的信息必须在运行时动态生成，因此外围设备的信息被放在一个单独的文件中。字符串必须以 0 结尾。 |


### COMPONENT_INFORMATION_BASIC (396) 

基本组件信息数据。应在启动时或需要时使用 [MAV_CMD_REQUEST_MESSAGE](#MAV_CMD_REQUEST_MESSAGE) 请求。

| 字段名称           | 类型       | 单位 | 值                                                  | 说明                                                         |
| ------------------ | ---------- | ---- | --------------------------------------------------- | ------------------------------------------------------------ |
| time_boot_ms       | `uint32_t` | ms   |                                                     | 时间戳（自系统启动以来的时间）。                             |
| capabilities       | `uint64_t` |      | [MAV_PROTOCOL_CAPABILITY](#MAV_PROTOCOL_CAPABILITY) | 组件功能标志                                                 |
| time_manufacture_s | `uint32_t` | s    | invalid:0                                           | 制造日期为 UNIX 纪元时间（自 1970 年 1 月 1 日以来），以秒为单位。 |
| vendor_name        | `char[32]` |      |                                                     | 组件供应商的名称。需要以零结尾。该字段是可选的，可以为空或全零。 |
| model_name         | `char[32]` |      |                                                     | 组件模型的名称。需要以零结尾。该字段是可选的，可以为空或全零。 |
| software_version   | `char[24]` |      |                                                     | 软件版本。建议的格式为 SEMVER: 'major.minor.patch'（可以使用任何格式）。如果该字段有值，则必须以零结尾。该字段是可选的，可以为空或全零。 |
| hardware_version   | `char[24]` |      |                                                     | 硬件版本。建议的格式为 SEMVER: 'major.minor.patch'（可以使用任何格式）。如果该字段有值，则必须以零结尾。该字段是可选的，可以为空或全零。 |
| serial_number      | `char[32]` |      |                                                     | 硬件序列号。如果该字段有值，则必须以零结尾。该字段是可选的，可以为空或全零。 |


### COMPONENT_METADATA (397) — [WIP] 

<span class="warning">**WORK IN PROGRESS**: Do not use in stable production environments (it may change).</span>


1,388 / 5,000
组件元数据消息，可使用 [MAV_CMD_REQUEST_MESSAGE](#MAV_CMD_REQUEST_MESSAGE) 请求。

它包含组件通用元数据文件的 MAVLink FTP URI 和 CRC。
该文件必须托管在组件上，并且可以是 xz 压缩的。
文件 CRC 可用于文件缓存。

可以读取通用元数据文件以获取其他元数据文件 ([COMP_METADATA_TYPE](#COMP_METADATA_TYPE)) 和翻译的位置，这些文件可以托管在车辆或互联网上。
有关更多信息，请参阅：https://mavlink.io/en/services/component_information.html。

注意：相机组件应改用 [CAMERA_INFORMATION](#CAMERA_INFORMATION)，自动驾驶仪可以同时使用此消息和 [AUTOPILOT_VERSION](#AUTOPILOT_VERSION)。

| 字段名称     | 类型        | 单位 | 说明                                                         |
| ------------ | ----------- | ---- | ------------------------------------------------------------ |
| time_boot_ms | `uint32_t`  | ms   | 时间戳（自系统启动以来的时间）。                             |
| file_crc     | `uint32_t`  |      | 通用元数据文件的 CRC32。                                     |
| uri          | `char[100]` |      | 通用元数据文件 ([COMP_METADATA_TYPE_GENERAL](#COMP_METADATA_TYPE_GENERAL)) 的 MAVLink FTP URI，可以使用 xz 压缩。该文件包含通用组件元数据，可能包含其他元数据的 URI 链接（参见 [COMP_METADATA_TYPE](#COMP_METADATA_TYPE)）。该信息从启动时就是静态的，可以在编译时生成。该字符串需要以零结尾。 |


### PLAY_TUNE_V2 (400) 

播放车辆音调/调谐（蜂鸣器）。取代消息 [PLAY_TUNE](#PLAY_TUNE)。

| 字段名称         | 类型        | 值                          | 说明                             |
| ---------------- | ----------- | --------------------------- | -------------------------------- |
| target_system    | `uint8_t`   |                             | 系统 ID                          |
| target_component | `uint8_t`   |                             | 组件 ID                          |
| format           | `uint32_t`  | [TUNE_FORMAT](#TUNE_FORMAT) | 调谐格式                         |
| tune             | `char[248]` |                             | 调谐定义为以 NULL 结尾的字符串。 |


### SUPPORTED_TUNES (401) 

车辆支持的调谐格式。这应作为对 [MAV_CMD_REQUEST_MESSAGE](#MAV_CMD_REQUEST_MESSAGE) 的响应发出。

| 字段名称         | 类型       | 值                          | 说明                     |
| ---------------- | ---------- | --------------------------- | ------------------------ |
| target_system    | `uint8_t`  |                             | 系统 ID                  |
| target_component | `uint8_t`  |                             | 组件 ID                  |
| format           | `uint32_t` | [TUNE_FORMAT](#TUNE_FORMAT) | 支持的调谐格式的位字段。 |


### EVENT (410) — [WIP] 

<span class="warning">**WORK IN PROGRESS**: Do not use in stable production environments (it may change).</span>

事件消息。来自特定组件的每个新事件都会获得一个新的序列号。如果（重新）请求，则可能会多次发送相同的消息。大多数事件都是广播的，有些事件可以特定于目标组件（因为接收器会跟踪错过的事件的序列，所以所有事件都需要广播。因此我们使用 destination_component 而不是 target_component）。

| 字段名称              | 类型          | 单位 | 描述                                                         |
| --------------------- | ------------- | ---- | ------------------------------------------------------------ |
| destination_component | `uint8_t`     |      | 组件 ID                                                      |
| destination_system    | `uint8_t`     |      | 系统 ID                                                      |
| id                    | `uint32_t`    |      | 事件 ID（如组件元数据中所定义）                              |
| event_time_boot_ms    | `uint32_t`    | ms   | 时间戳（自系统启动以来发生事件的时间）。                     |
| sequence              | `uint16_t`    |      | 序列号。                                                     |
| log_levels            | `uint8_t`     |      | 日志级别：4 位 MSB：内部（用于日志记录目的），4 位 LSB：外部。级别：紧急 = 0、警报 = 1、严重 = 2、错误 = 3、警告 = 4、通知 = 5、信息 = 6、调试 = 7、协议 = 8、禁用 = 9 |
| 参数                  | `uint8_t[40]` |      | 参数（取决于事件 ID）。                                      |


### CURRENT_EVENT_SEQUENCE (411) — [WIP] 

<span class="warning">**WORK IN PROGRESS**: Do not use in stable production environments (it may change).</span>

定期广播组件的当前最新事件序列号。这用于检查丢弃的事件。

| 字段名称 | 类型       | 值                                                           | 说明       |
| -------- | ---------- | ------------------------------------------------------------ | ---------- |
| sequence | `uint16_t` |                                                              | 序列号。   |
| flags    | `uint8_t`  | [MAV_EVENT_CURRENT_SEQUENCE_FLAGS](#MAV_EVENT_CURRENT_SEQUENCE_FLAGS) | 标志位集。 |


### REQUEST_EVENT (412) — [WIP] 

<span class="warning">**WORK IN PROGRESS**: Do not use in stable production environments (it may change).</span>

请求（重新）发送一个或多个事件。如果 first_sequence==last_sequence，则仅请求单个事件。请注意，first_sequence 可以大于 last_sequence（因为序列号可以换行）。每个序列都会触发 EVENT 或 [EVENT_ERROR](#EVENT_ERROR) 响应。

| 字段名称         | 类型       | 说明                       |
| ---------------- | ---------- | -------------------------- |
| target_system    | `uint8_t`  | 系统 ID                    |
| target_component | `uint8_t`  | 组件 ID                    |
| first_sequence   | `uint16_t` | 请求事件的第一个序列号。   |
| last_sequence    | `uint16_t` | 请求事件的最后一个序列号。 |


### RESPONSE_EVENT_ERROR (413) — [WIP] 

<span class="warning">**WORK IN PROGRESS**: Do not use in stable production environments (it may change).</span>

发生错误时对 [REQUEST_EVENT](#REQUEST_EVENT) 的响应（例如，事件不再可用）。

| 字段名称                  | 类型       | 值                                                | 说明                                                         |
| ------------------------- | ---------- | ------------------------------------------------- | ------------------------------------------------------------ |
| target_system             | `uint8_t`  |                                                   | 系统 ID                                                      |
| target_component          | `uint8_t`  |                                                   | 组件 ID                                                      |
| sequence                  | `uint16_t` |                                                   | 序列号。                                                     |
| sequence_oldest_available | `uint16_t` |                                                   | 在 [REQUEST_EVENT](#REQUEST_EVENT) 中设置的序列之后仍然可用的最旧序列号。 |
| reason                    | `uint8_t`  | [MAV_EVENT_ERROR_REASON](#MAV_EVENT_ERROR_REASON) | 错误原因。                                                   |


### ILLUMINATOR_STATUS (440) 

照明器状态

| 字段名称          | 类型       | 单位 | 值                                                  | 说明                                   |
| ----------------- | ---------- | ---- | --------------------------------------------------- | -------------------------------------- |
| uptime_ms         | `uint32_t` | ms   |                                                     | 自照明器启动以来的时间（以毫秒为单位） |
| enable            | `uint8_t`  |      |                                                     | 0：照明器关闭，1：照明器打开           |
| mode_bitmask      | `uint8_t`  |      | [ILLUMINATOR_MODE](#ILLUMINATOR_MODE)               | 支持的照明器模式                       |
| error_status      | `uint32_t` |      | [ILLUMINATOR_ERROR_FLAGS](#ILLUMINATOR_ERROR_FLAGS) | 错误                                   |
| mode              | `uint8_t`  |      | [ILLUMINATOR_MODE](#ILLUMINATOR_MODE)               | 照明器模式                             |
| brightness        | `float`    | %    |                                                     | 照明器亮度                             |
| strobe_period     | `float`    | s    |                                                     | 照明器频闪周期（以秒为单位）           |
| strobe_duty_cycle | `float`    | %    |                                                     | 照明器频闪占空比                       |
| temp_c            | `float`    |      |                                                     | 温度（以摄氏度为单位）                 |
| min_strobe_period | `float`    | s    |                                                     | 最小频闪周期（以秒为单位）             |
| max_strobe_period | `float`    | s    |                                                     | 最大频闪周期（以秒为单位）             |


### WHEEL_DISTANCE (9000) 

每个报告的车轮累计行驶距离。

| 字段名称  | 类型         | 单位 | 说明                                                         |
| --------- | ------------ | ---- | ------------------------------------------------------------ |
| time_usec | `uint64_t`   | us   | 时间戳（同步到 UNIX 时间或自系统启动以来）。                 |
| count     | `uint8_t`    |      | 报告的车轮数量。                                             |
| distance  | `double[16]` | m    | 各个车轮编码器报告的距离。正向旋转会增加值，反向旋转会减少值。并非所有车轮都必须有车轮编码器；编码器到车轮位置的映射必须由端点同意/理解。 |


### WINCH_STATUS (9005) 

绞盘状态。

| 字段名称    | 类型       | 单位 | 值                                              | 说明                                                         |
| ----------- | ---------- | ---- | ----------------------------------------------- | ------------------------------------------------------------ |
| time_usec   | `uint64_t` | us   |                                                 | 时间戳（同步到 UNIX 时间或自系统启动以来）。                 |
| line_length | `float`    | m    | invalid:NaN                                     | 释放的线的长度。如果未知则为 NaN                             |
| speed       | `float`    | m/s  | invalid:NaN                                     | 线被释放或缩回的速度。如果被释放则为正值，如果被缩回则为负值，如果未知则为 NaN |
| tension     | `float`    | kg   | invalid:NaN                                     | 线上的张力。如果未知则为 NaN                                 |
| voltage     | `float`    | V    | invalid:NaN                                     | 为绞盘供电的电池电压。如果未知则为 NaN                       |
| current     | `float`    | A    | invalid:NaN                                     | 绞盘的电流消耗。如果未知则为 NaN                             |
| temperature | `int16_t`  | degC | invalid:INT16_MAX                               | 电机温度。如果未知则为 INT16_MAX                             |
| 状态        | `uint32_t` |      | [MAV_WINCH_STATUS_FLAG](#MAV_WINCH_STATUS_FLAG) | 状态标志                                                     |


### OPEN_DRONE_ID_BASIC_ID (12900) 

用于填充 OpenDroneID 基本 ID 消息的数据。此消息和以下消息主要用于向 OpenDroneID 实现提供数据或从 OpenDroneID 实现提供数据。例如 https://github.com/opendroneid/opendroneid-core-c。这些消息与 ASTM F3411 远程 ID 标准和 ASD-STAN prEN 4709-002 直接远程 ID 标准兼容。这些消息的其他信息和用法记录在 https://mavlink.io/en/services/opendroneid.html。

| 字段名称         | 类型          | 值                                    | 描述                                                         |
| ---------------- | ------------- | ------------------------------------- | ------------------------------------------------------------ |
| target_system    | `uint8_t`     |                                       | 系统 ID（0 表示广播）。                                      |
| target_component | `uint8_t`     |                                       | 组件 ID（0 表示广播）。                                      |
| id_or_mac        | `uint8_t[20]` |                                       | 仅用于从其他 UA 接收的无人机 ID 数据。详细说明见https://mavlink.io/en/services/opendroneid.html。 |
| id_type          | `uint8_t`     | [MAV_ODID_ID_TYPE](#MAV_ODID_ID_TYPE) | 表示此消息的 uas_id 字段的格式。                             |
| ua_type          | `uint8_t`     | [MAV_ODID_UA_TYPE](#MAV_ODID_UA_TYPE) | 表示 UA（无人机）的类型。                                    |
| uas_id           | `uint8_t[20]` |                                       | UAS（无人机系统）ID 遵循 id_type 指定的格式。字段未使用的部分应填充为空值。 |


### OPEN_DRONE_ID_LOCATION (12901) 

用于填充OpenDroneID Location消息的数据。浮点数据类型为32位IEEE 754。Location消息提供无人机的位置、高度、方向和速度。

| 字段名称            | 类型          | 单位  | 值                                          | 说明                                                         |
| ------------------- | ------------- | ----- | ------------------------------------------- | ------------------------------------------------------------ |
| target_system       | `uint8_t`     |       |                                             | 系统ID（0表示广播）。                                        |
| target_component    | `uint8_t`     |       |                                             | 组件ID（0表示广播）。                                        |
| id_or_mac           | `uint8_t[20]` |       |                                             | 仅用于从其他UA接收的无人机ID数据。详细说明见https://mavlink.io/en/services/opendroneid.html。 |
| status              | `uint8_t`     |       | [MAV_ODID_STATUS](#MAV_ODID_STATUS)         | 表示无人机是在地面还是在空中。                               |
| direction           | `uint16_t`    | cdeg  | invalid:36100                               | 从真北方向顺时针测量的地面方向（不是航向，而是移动方向）：0 - 35999 摄氏度。如果未知：36100 摄氏度。 |
| speed_horizontal    | `uint16_t`    | cm/s  |                                             | 地速。只能为正数。如果未知：25500 cm/s。如果速度大于 25425 cm/s，则使用 25425 cm/s。 |
| speed_vertical      | `int16_t`     | cm/s  |                                             | 垂直速度。向上为正数。如果未知：6300 cm/s。如果速度大于 6200 cm/s，则使用 6200 cm/s。如果低于 -6200 cm/s，则使用 -6200 cm/s。 |
| latitude            | `int32_t`     | degE7 | invalid:0                                   | 无人机的当前纬度。如果未知：0（纬度/经度）。                 |
| 经度                | `int32_t`     | degE7 | 无效：0                                     | 无人机的当前经度。 如果未知：0（纬度/经度）。                |
| 高度_气压           | `float`       | m     | 无效：-1000                                 | 根据气压计算的高度。 参考值为 29.92inHg 或 1013.2mb。 如果未知：-1000 米。 |
| 高度_大地高度       | `float`       | m     | 无效：-1000                                 | WGS84 定义的大地高度。 如果未知：-1000 米。                  |
| 高度_参考           | `uint8_t`     |       | [MAV_ODID_HEIGHT_REF](#MAV_ODID_HEIGHT_REF) | 表示高度字段的参考点。                                       |
| 高度                | `float`       | m     | 无效：-1000                                 | 无人机当前距起飞位置或地面的高度，由 height_reference 指示。如果未知：-1000 米。 |
| horizontal_accuracy | `uint8_t`     |       | [MAV_ODID_HOR_ACC](#MAV_ODID_HOR_ACC)       | 水平位置的精度。                                             |
| vertical_accuracy   | `uint8_t`     |       | [MAV_ODID_VER_ACC](#MAV_ODID_VER_ACC)       | 垂直位置的精度。                                             |
| barometer_accuracy  | `uint8_t`     |       | [MAV_ODID_VER_ACC](#MAV_ODID_VER_ACC)       | 气压高度的精度。                                             |
| speed_accuracy      | `uint8_t`     |       | [MAV_ODID_SPEED_ACC](#MAV_ODID_SPEED_ACC)   | 水平和垂直速度的精度。                                       |
| timestamp           | `float`       | s     | invalid:0xFFFF                              | 以 UTC 时间为参考，整点后的秒数。通常，GPS 会以毫秒为单位输出星期值。首先将其转换为 UTC，然后使用 ((float) (time_week_ms % (60*60*1000))) / 1000 转换此字段。如果未知：0xFFFF。 |
| timestamp_accuracy  | `uint8_t`     |       | [MAV_ODID_TIME_ACC](#MAV_ODID_TIME_ACC)     | 时间戳的准确性。                                             |


### OPEN_DRONE_ID_AUTHENTICATION (12902) 

用于填充 OpenDroneID 身份验证消息的数据。身份验证消息定义了一个字段，该字段可以为 UAS（无人机系统）的身份提供真实性。身份验证消息可以有两种不同的格式。对于数据页 0，字段 PageCount、Length 和 TimeStamp 存在，AuthData 仅为 17 个字节。对于数据页 1 至 15，PageCount、Length 和 TimeStamp 不存在，AuthData 的大小为 23 个字节。

| 字段名称            | 类型          | 单位  | 值                                        | 说明                                                         |
| ------------------- | ------------- | ----- | ----------------------------------------- | ------------------------------------------------------------ |
| target_system       | `uint8_t`     |       |                                           | 系统 ID（0 表示广播）。                                      |
| target_component    | `uint8_t`     |       |                                           | 组件 ID（0 表示广播）。                                      |
| id_or_mac           | `uint8_t[20]` |       |                                           | 仅用于从其他 UA 接收的无人机 ID 数据。请参阅 https://mavlink.io/en/services/opendroneid.html 上的详细说明。 |
| authentication_type | `uint8_t`     |       | [MAV_ODID_AUTH_TYPE](#MAV_ODID_AUTH_TYPE) | 表示身份验证的类型。                                         |
| data_page           | `uint8_t`     |       |                                           | 允许的范围是 0 - 15。                                        |
| last_page_index     | `uint8_t`     |       |                                           | 此字段仅存在于第 0 页。允许的范围是 0 - 15。请参阅 https://github.com/opendroneid/opendroneid-core-c/blob/master/libopendroneid/opendroneid.h 上 struct ODID_Auth_data 的描述。 |
| length              | `uint8_t`     | bytes |                                           | 此字段仅存在于第 0 页。来自所有数据页的 authentication_data 总字节数。请参阅 https://github.com/opendroneid/opendroneid-core-c/blob/master/libopendroneid/opendroneid.h 上 struct ODID_Auth_data 的描述。 |
| timestamp           | `uint32_t`    | s     |                                           | 此字段仅存在于第 0 页。自 2019 年 1 月 1 日 00:00:00 以来的 32 位 Unix 时间戳（以秒为单位）。 |
| authentication_data | `uint8_t[23]` |       |                                           | 不透明的身份验证数据。对于第 0 页，大小只有 17 个字节。对于其他页面，大小为 23 个字节。应在字段未使用的部分填充空值。 |


### OPEN_DRONE_ID_SELF_ID (12903) 

用于填充 OpenDroneID 自我识别消息的数据。自我识别消息是操作员（可选）声明其身份和飞行目的的机会。此消息可以提供其他信息，从而降低在特定区域或以特定方式飞行的 UA（无人机）的威胁概况。此消息还可用于在紧急/远程 ID 系统故障情况下提供可选的额外说明。

| 字段名称         | 类型          | 值                                        | 说明                                                         |
| ---------------- | ------------- | ----------------------------------------- | ------------------------------------------------------------ |
| target_system    | `uint8_t`     |                                           | 系统 ID（0 表示广播）。                                      |
| target_component | `uint8_t`     |                                           | 组件 ID（0 表示广播）。                                      |
| id_or_mac        | `uint8_t[20]` |                                           | 仅用于从其他 UA 接收的无人机 ID 数据。请参阅 https://mavlink.io/en/services/opendroneid.html 上的详细说明。 |
| description_type | `uint8_t`     | [MAV_ODID_DESC_TYPE](#MAV_ODID_DESC_TYPE) | 表示描述字段的类型。                                         |
| description      | `char[23]`    |                                           | 以 ASCII 字符表示的文本描述或数值。字段未使用的部分应填充为空值。 |


### OPEN_DRONE_ID_SYSTEM (12904) 


2,268 / 5,000
用于填充 OpenDroneID 系统消息的数据。系统消息包含一般系统信息，包括操作员位置/高度和可能的飞机组和/或类别/等级信息。

| 字段名称               | 类型          | 单位  | 值                                                           | 说明                                                         |
| ---------------------- | ------------- | ----- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| target_system          | `uint8_t`     |       |                                                              | 系统 ID（0 表示广播）。                                      |
| target_component       | `uint8_t`     |       |                                                              | 组件 ID（0 表示广播）。                                      |
| id_or_mac              | `uint8_t[20]` |       |                                                              | 仅用于从其他 UA 接收的无人机 ID 数据。请参阅 https://mavlink.io/en/services/opendroneid.html 上的详细说明。 |
| operator_location_type | `uint8_t`     |       | [MAV_ODID_OPERATOR_LOCATION_TYPE](#MAV_ODID_OPERATOR_LOCATION_TYPE) | 指定操作员位置类型。                                         |
| category_type          | `uint8_t`     |       | [MAV_ODID_CLASSIFICATION_TYPE](#MAV_ODID_CLASSIFICATION_TYPE) | 指定 UA 的分类类型。                                         |
| operator_latitude      | `int32_t`     | degE7 | invalid:0                                                    | 操作员的纬度。如果未知：0（纬度/经度）。                     |
| operator_longitude     | `int32_t`     | degE7 | invalid:0                                                    | 操作员的经度。如果未知：0（纬度/经度）。                     |
| area_count             | `uint16_t`    |       |                                                              | 区域、群组或编队中的飞机数量（默认值为 1）。仅用于群集/多个 UA。 |
| area_radius            | `uint16_t`    | m     |                                                              | 群组或编队的圆柱形区域的半径（默认值为 0）。仅用于群集/多个 UA。 |
| area_ceiling           | `float`       | m     | invalid:-1000                                                | 相对于 WGS84 的区域操作上限。如果未知：-1000 米。仅用于群体/多个 UA。 |
| area_floor             | `float`       | m     | invalid:-1000                                                | 相对于 WGS84 的区域操作下限。如果未知：-1000 米。仅用于群体/多个 UA。 |
| category_eu            | `uint8_t`     |       | [MAV_ODID_CATEGORY_EU](#MAV_ODID_CATEGORY_EU)                | 当 category_type 为 [MAV_ODID_CLASSIFICATION_TYPE_EU](#MAV_ODID_CLASSIFICATION_TYPE_EU) 时，指定 UA 的类别。 |
| class_eu               | `uint8_t`     |       | [MAV_ODID_CLASS_EU](#MAV_ODID_CLASS_EU)                      | 当classification_type为[MAV_ODID_CLASSIFICATION_TYPE_EU](#MAV_ODID_CLASSIFICATION_TYPE_EU)时，指定UA的类别。 |
| operator_altitude_geo  | `float`       | m     | invalid:-1000                                                | 操作员相对于WGS84的大地高度。 如果未知：-1000 m。            |
| timestamp              | `uint32_t`    | s     |                                                              | 自2019年1月1日 00:00:00以来的32位Unix时间戳（以秒为单位）。  |


### OPEN_DRONE_ID_OPERATOR_ID (12905) 

用于填充 OpenDroneID 操作员 ID 消息的数据，其中包含 CAA（民航局）颁发的操作员 ID。

| 字段名称         | 类型          | 值                                                      | 说明                                                         |
| ---------------- | ------------- | ------------------------------------------------------- | ------------------------------------------------------------ |
| target_system    | `uint8_t`     |                                                         | 系统 ID（0 表示广播）。                                      |
| target_component | `uint8_t`     |                                                         | 组件 ID（0 表示广播）。                                      |
| id_or_mac        | `uint8_t[20]` |                                                         | 仅用于从其他 UA 接收的无人机 ID 数据。请参阅 https://mavlink.io/en/services/opendroneid.html 上的详细说明。 |
| operator_id_type | `uint8_t`     | [MAV_ODID_OPERATOR_ID_TYPE](#MAV_ODID_OPERATOR_ID_TYPE) | 指示 operator_id 字段的类型。                                |
| operator_id      | `char[20]`    |                                                         | 以 ASCII 字符表示的文本描述或数值。应在字段未使用的部分填充空值。 |


### OPEN_DRONE_ID_MESSAGE_PACK (12915) 

OpenDroneID 消息包是多个编码的 OpenDroneID 消息的容器（即不是上述消息描述中给出的格式，而是编码为压缩的 OpenDroneID 字节格式）。例如，在蓝牙 5.0 长距离/扩展广告或 WiFi 邻居感知网络或 WiFi 信标上传输时使用。

| 字段名称            | 类型           | 单位 | 描述                                                         |
| ------------------- | -------------- | ---- | ------------------------------------------------------------ |
| target_system       | `uint8_t`      |      | 系统 ID（0 表示广播）。                                      |
| target_component    | `uint8_t`      |      | 组件 ID（0 表示广播）。                                      |
| id_or_mac           | `uint8_t[20]`  |      | 仅用于从其他 UA 接收的无人机 ID 数据。请参阅 https://mavlink.io/en/services/opendroneid.html 上的详细说明。 |
| single_message_size | `uint8_t`      | 字节 | 此字段当前必须始终等于 25（字节），因为所有编码的 OpenDroneID 消息都指定为具有此长度。 |
| msg_pack_size       | `uint8_t`      |      | 包中的编码消息数（不是字节数）。允许的范围是 1 - 9。         |
| messages            | `uint8_t[225]` |      | 编码的 OpenDroneID 消息的连接。应在字段的未使用部分填充空值。 |


### OPEN_DRONE_ID_ARM_STATUS (12918) 

发射器（远程 ID 系统）已启用并准备好开始发送位置和其他所需信息。这由发射器传输。飞行控制器使用它作为布防条件。

| 字段名称 | 类型       | 值                                          | 说明                                                         |
| -------- | ---------- | ------------------------------------------- | ------------------------------------------------------------ |
| status   | `uint8_t`  | [MAV_ODID_ARM_STATUS](#MAV_ODID_ARM_STATUS) | 指示是否允许布防的状态级别。                                 |
| error    | `char[50]` |                                             | 文本错误消息，如果状态适合布防，则应为空。在未使用的部分用空值填充。 |


### OPEN_DRONE_ID_SYSTEM_UPDATE (12919) 

使用新的位置信息更新 [OPEN_DRONE_ID_SYSTEM](#OPEN_DRONE_ID_SYSTEM) 消息中的数据。当 SYSTEM 消息中的其他信息均未发生变化时，可以发送此消息来更新操作员的位置信息。此消息允许在具有有限上行链路带宽的无线电链路上高效运行，同时满足操作员位置更新频率的要求。

| 字段名称              | 类型       | 单位  | 说明                                                         |
| --------------------- | ---------- | ----- | ------------------------------------------------------------ |
| target_system         | `uint8_t`  |       | 系统 ID（0 表示广播）。                                      |
| target_component      | `uint8_t`  |       | 组件 ID（0 表示广播）。                                      |
| operator_latitude     | `int32_t`  | degE7 | 操作员的纬度。如果未知：0（纬度/经度）。                     |
| operator_longitude    | `int32_t`  | degE7 | 操作员的经度。如果未知：0（纬度/经度）。                     |
| operator_altitude_geo | `float`    | m     | 操作员相对于 WGS84 的大地测量高度。如果未知：-1000 米。      |
| 时间戳                | `uint32_t` | s     | 自 2019 年 1 月 1 日 00:00:00 以来的 32 位 Unix 时间戳（以秒为单位）。 |


### HYGROMETER_SENSOR (12920) 

来自湿度计的温度和湿度。

| 字段名称 | 类型       | 单位  | 说明                                                |
| -------- | ---------- | ----- | --------------------------------------------------- |
| id       | `uint8_t`  |       | 湿度计 ID<br>具有相同值的消息来自同一来源（实例）。 |
| 温度     | `int16_t`  | cdegC | 温度                                                |
| 湿度     | `uint16_t` | c%    | 湿度                                                |


## Enumerated Types

### FIRMWARE_VERSION_TYPE 

这些值定义固件版本的类型。这些值表示此类型的第一个版本或发行版。例如，第一个 alpha 版本将是 64，第二个将是 65。

| 值                                             | 名称                                                         | 说明         |
| ---------------------------------------------- | ------------------------------------------------------------ | ------------ |
| <a id='FIRMWARE_VERSION_TYPE_DEV'></a>0        | [FIRMWARE_VERSION_TYPE_DEV](#FIRMWARE_VERSION_TYPE_DEV)      | 开发版本     |
| <a id='FIRMWARE_VERSION_TYPE_ALPHA'></a>64     | [FIRMWARE_VERSION_TYPE_ALPHA](#FIRMWARE_VERSION_TYPE_ALPHA)  | alpha 版本   |
| <a id='FIRMWARE_VERSION_TYPE_BETA'></a>128     | [FIRMWARE_VERSION_TYPE_BETA](#FIRMWARE_VERSION_TYPE_BETA)    | 测试版本     |
| <a id='FIRMWARE_VERSION_TYPE_RC'></a>192       | [FIRMWARE_VERSION_TYPE_RC](#FIRMWARE_VERSION_TYPE_RC)        | 发布候选     |
| <a id='FIRMWARE_VERSION_TYPE_OFFICIAL'></a>255 | [FIRMWARE_VERSION_TYPE_OFFICIAL](#FIRMWARE_VERSION_TYPE_OFFICIAL) | 发布正式稳定 |

### HL_FAILURE_FLAG 

（位掩码）用于报告高延迟遥测故障情况的标志。

| 值                                                  | 名称                                                         | 说明                                 |
| --------------------------------------------------- | ------------------------------------------------------------ | ------------------------------------ |
| <a id='HL_FAILURE_FLAG_GPS'></a>1                   | [HL_FAILURE_FLAG_GPS](#HL_FAILURE_FLAG_GPS)                  | GPS 故障。                           |
| <a id='HL_FAILURE_FLAG_DIFFERENTIAL_PRESSURE'></a>2 | [HL_FAILURE_FLAG_DIFFERENTIAL_PRESSURE](#HL_FAILURE_FLAG_DIFFERENTIAL_PRESSURE) | 差压传感器故障。                     |
| <a id='HL_FAILURE_FLAG_ABSOLUTE_PRESSURE'></a>4     | [HL_FAILURE_FLAG_ABSOLUTE_PRESSURE](#HL_FAILURE_FLAG_ABSOLUTE_PRESSURE) | 绝对压力传感器故障。                 |
| <a id='HL_FAILURE_FLAG_3D_ACCEL'></a>8              | [HL_FAILURE_FLAG_3D_ACCEL](#HL_FAILURE_FLAG_3D_ACCEL)        | 加速度计传感器故障。                 |
| <a id='HL_FAILURE_FLAG_3D_GYRO'></a>16              | [HL_FAILURE_FLAG_3D_GYRO](#HL_FAILURE_FLAG_3D_GYRO)          | 陀螺仪传感器故障。                   |
| <a id='HL_FAILURE_FLAG_3D_MAG'></a>32               | [HL_FAILURE_FLAG_3D_MAG](#HL_FAILURE_FLAG_3D_MAG)            | 磁力计传感器故障。                   |
| <a id='HL_FAILURE_FLAG_TERRAIN'></a>64              | [HL_FAILURE_FLAG_TERRAIN](#HL_FAILURE_FLAG_TERRAIN)          | 地形子系统故障。                     |
| <a id='HL_FAILURE_FLAG_BATTERY'></a>128             | [HL_FAILURE_FLAG_BATTERY](#HL_FAILURE_FLAG_BATTERY)          | 电池故障/电池电量严重不足。          |
| <a id='HL_FAILURE_FLAG_RC_RECEIVER'></a>256         | [HL_FAILURE_FLAG_RC_RECEIVER](#HL_FAILURE_FLAG_RC_RECEIVER)  | RC 接收器故障/无 RC 连接。           |
| <a id='HL_FAILURE_FLAG_OFFBOARD_LINK'></a>512       | [HL_FAILURE_FLAG_OFFBOARD_LINK](#HL_FAILURE_FLAG_OFFBOARD_LINK) | 车外链路故障。                       |
| <a id='HL_FAILURE_FLAG_ENGINE'></a>1024             | [HL_FAILURE_FLAG_ENGINE](#HL_FAILURE_FLAG_ENGINE)            | 发动机故障。                         |
| <a id='HL_FAILURE_FLAG_GEOFENCE'></a>2048           | [HL_FAILURE_FLAG_GEOFENCE](#HL_FAILURE_FLAG_GEOFENCE)        | 地理围栏违规。                       |
| <a id='HL_FAILURE_FLAG_ESTIMATOR'></a>4096          | [HL_FAILURE_FLAG_ESTIMATOR](#HL_FAILURE_FLAG_ESTIMATOR)      | 估计器失败，例如测量拒绝或方差较大。 |
| <a id='HL_FAILURE_FLAG_MISSION'></a>8192            | [HL_FAILURE_FLAG_MISSION](#HL_FAILURE_FLAG_MISSION)          | 任务失败。                           |

### MAV_GOTO 

可在 [MAV_CMD_OVERRIDE_GOTO](#MAV_CMD_OVERRIDE_GOTO) 中指定的操作以覆盖任务执行。

| 值                                                | 名称                                                         | 描述                                            |
| ------------------------------------------------- | ------------------------------------------------------------ | ----------------------------------------------- |
| <a id='MAV_GOTO_DO_HOLD'></a>0                    | [MAV_GOTO_DO_HOLD](#MAV_GOTO_DO_HOLD)                        | 保持在当前位置。                                |
| <a id='MAV_GOTO_DO_CONTINUE'></a>1                | [MAV_GOTO_DO_CONTINUE](#MAV_GOTO_DO_CONTINUE)                | 继续执行任务的下一项。                          |
| <a id='MAV_GOTO_HOLD_AT_CURRENT_POSITION'></a>2   | [MAV_GOTO_HOLD_AT_CURRENT_POSITION](#MAV_GOTO_HOLD_AT_CURRENT_POSITION) | 保持在系统当前位置                              |
| <a id='MAV_GOTO_HOLD_AT_SPECIFIED_POSITION'></a>3 | [MAV_GOTO_HOLD_AT_SPECIFIED_POSITION](#MAV_GOTO_HOLD_AT_SPECIFIED_POSITION) | 在 [DO_HOLD](#DO_HOLD) 动作的参数指定的位置保持 |

### MAV_MODE 

这些定义是预定义的或组合模式标志。无需使用此枚举中的值，但它

简化了模式标志的使用。请注意，在所有模式下都启用手动输入作为安全覆盖。

| 值                                         | 名称                                                        | 说明                                                         |
| ------------------------------------------ | ----------------------------------------------------------- | ------------------------------------------------------------ |
| <a id='MAV_MODE_PREFLIGHT'></a>0           | [MAV_MODE_PREFLIGHT](#MAV_MODE_PREFLIGHT)                   | 系统尚未准备好飞行、启动、校准等。未设置标志。               |
| <a id='MAV_MODE_MANUAL_DISARMED'></a>64    | [MAV_MODE_MANUAL_DISARMED](#MAV_MODE_MANUAL_DISARMED)       | 系统允许处于活动状态，在手动（RC）控制下，无稳定性           |
| <a id='MAV_MODE_TEST_DISARMED'></a>66      | [MAV_MODE_TEST_DISARMED](#MAV_MODE_TEST_DISARMED)           | 未定义模式。这完全取决于自动驾驶仪 - 请谨慎使用，仅供开发人员使用。 |
| <a id='MAV_MODE_STABILIZE_DISARMED'></a>80 | [MAV_MODE_STABILIZE_DISARMED](#MAV_MODE_STABILIZE_DISARMED) | 在辅助 RC 控制下，系统可以处于活动状态。                     |
| <a id='MAV_MODE_GUIDED_DISARMED'></a>88    | [MAV_MODE_GUIDED_DISARMED](#MAV_MODE_GUIDED_DISARMED)       | 系统可以处于活动状态，自主控制，手动设定                     |
| <a id='MAV_MODE_AUTO_DISARMED'></a>92      | [MAV_MODE_AUTO_DISARMED](#MAV_MODE_AUTO_DISARMED)           | 系统可以处于活动状态，自主控制和导航（轨迹由机上决定，而不是由航路点预先编程） |
| <a id='MAV_MODE_MANUAL_ARMED'></a>192      | [MAV_MODE_MANUAL_ARMED](#MAV_MODE_MANUAL_ARMED)             | 系统允许处于活动状态，在手动（RC）控制下，无稳定性           |
| <a id='MAV_MODE_TEST_ARMED'></a>194        | [MAV_MODE_TEST_ARMED](#MAV_MODE_TEST_ARMED)                 | 未定义模式。这完全取决于自动驾驶仪 - 请谨慎使用，仅供开发人员使用。 |
| <a id='MAV_MODE_STABILIZE_ARMED'></a>208   | [MAV_MODE_STABILIZE_ARMED](#MAV_MODE_STABILIZE_ARMED)       | 在辅助 RC 控制下，系统可以处于活动状态。                     |
| <a id='MAV_MODE_GUIDED_ARMED'></a>216      | [MAV_MODE_GUIDED_ARMED](#MAV_MODE_GUIDED_ARMED)             | 系统可以处于活动状态，自主控制，手动设定                     |
| <a id='MAV_MODE_AUTO_ARMED'></a>220        | [MAV_MODE_AUTO_ARMED](#MAV_MODE_AUTO_ARMED)                 | 系统可以处于活动状态，自主控制和导航（轨迹由机上决定，而不是由航路点预先编程） |

### MAV_SYS_STATUS_SENSOR 

（位掩码）这些编码对传感器进行编码，传感器的状态作为 [SYS_STATUS](#SYS_STATUS) 消息的一部分发送。

| 值                                                           | 名称                                                         | 描述                                                         |
| ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |
| <a id='MAV_SYS_STATUS_SENSOR_3D_GYRO'></a>1                  | [MAV_SYS_STATUS_SENSOR_3D_GYRO](#MAV_SYS_STATUS_SENSOR_3D_GYRO) | 0x01 3D 陀螺仪                                               |
| <a id='MAV_SYS_STATUS_SENSOR_3D_ACCEL'></a>2                 | [MAV_SYS_STATUS_SENSOR_3D_ACCEL](#MAV_SYS_STATUS_SENSOR_3D_ACCEL) | 0x02 3D 加速度计                                             |
| <a id='MAV_SYS_STATUS_SENSOR_3D_MAG'></a>4                   | [MAV_SYS_STATUS_SENSOR_3D_MAG](#MAV_SYS_STATUS_SENSOR_3D_MAG) | 0x04 3D 磁力仪                                               |
| <a id='MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE'></a>8        | [MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE](#MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE) | 0x08 绝对压力                                                |
| <a id='MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE'></a>16   | [MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE](#MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE) | 0x10 不同的压力                                              |
| <a id='MAV_SYS_STATUS_SENSOR_GPS'></a>32                     | [MAV_SYS_STATUS_SENSOR_GPS](#MAV_SYS_STATUS_SENSOR_GPS)      | 0x20 GPS                                                     |
| <a id='MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW'></a>64            | [MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW](#MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW) | 0x40 optical flow                                            |
| <a id='MAV_SYS_STATUS_SENSOR_VISION_POSITION'></a>128        | [MAV_SYS_STATUS_SENSOR_VISION_POSITION](#MAV_SYS_STATUS_SENSOR_VISION_POSITION) | 0x80 计算机视觉职位                                          |
| <a id='MAV_SYS_STATUS_SENSOR_LASER_POSITION'></a>256         | [MAV_SYS_STATUS_SENSOR_LASER_POSITION](#MAV_SYS_STATUS_SENSOR_LASER_POSITION) | 0x100 激光定位                                               |
| <a id='MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH'></a>512  | [MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH](#MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH) | 0x200 外部地面实况（Vicon 或 Leica）                         |
| <a id='MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL'></a>1024  | [MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL](#MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL) | 0x400 3D 角速率控制                                          |
| <a id='MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION'></a>2048 | [MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION](#MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION) | 0x800 姿态稳定                                               |
| <a id='MAV_SYS_STATUS_SENSOR_YAW_POSITION'></a>4096          | [MAV_SYS_STATUS_SENSOR_YAW_POSITION](#MAV_SYS_STATUS_SENSOR_YAW_POSITION) | 0x1000 偏航位置                                              |
| <a id='MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL'></a>8192    | [MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL](#MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL) | 0x2000 z/高度控制                                            |
| <a id='MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL'></a>16384  | [MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL](#MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL) | 0x4000 X/Y 位置控制                                          |
| <a id='MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS'></a>32768        | [MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS](#MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS) | 0x8000 电机输出/控制                                         |
| <a id='MAV_SYS_STATUS_SENSOR_RC_RECEIVER'></a>65536          | [MAV_SYS_STATUS_SENSOR_RC_RECEIVER](#MAV_SYS_STATUS_SENSOR_RC_RECEIVER) | 0x10000 遥控接收器                                           |
| <a id='MAV_SYS_STATUS_SENSOR_3D_GYRO2'></a>131072            | [MAV_SYS_STATUS_SENSOR_3D_GYRO2](#MAV_SYS_STATUS_SENSOR_3D_GYRO2) | 0x20000 第 2 个 3D 陀螺仪                                    |
| <a id='MAV_SYS_STATUS_SENSOR_3D_ACCEL2'></a>262144           | [MAV_SYS_STATUS_SENSOR_3D_ACCEL2](#MAV_SYS_STATUS_SENSOR_3D_ACCEL2) | 0x40000 第 2 个 3D 加速计                                    |
| <a id='MAV_SYS_STATUS_SENSOR_3D_MAG2'></a>524288             | [MAV_SYS_STATUS_SENSOR_3D_MAG2](#MAV_SYS_STATUS_SENSOR_3D_MAG2) | 0x80000 第 2 个 3D 磁力仪                                    |
| <a id='MAV_SYS_STATUS_GEOFENCE'></a>1048576                  | [MAV_SYS_STATUS_GEOFENCE](#MAV_SYS_STATUS_GEOFENCE)          | 0x100000 地理围栏                                            |
| <a id='MAV_SYS_STATUS_AHRS'></a>2097152                      | [MAV_SYS_STATUS_AHRS](#MAV_SYS_STATUS_AHRS)                  | 0x200000 AHRS 子系统运行状况                                 |
| <a id='MAV_SYS_STATUS_TERRAIN'></a>4194304                   | [MAV_SYS_STATUS_TERRAIN](#MAV_SYS_STATUS_TERRAIN)            | 0x400000 地形子系统状况                                      |
| <a id='MAV_SYS_STATUS_REVERSE_MOTOR'></a>8388608             | [MAV_SYS_STATUS_REVERSE_MOTOR](#MAV_SYS_STATUS_REVERSE_MOTOR) | 0x800000 电机反转                                            |
| <a id='MAV_SYS_STATUS_LOGGING'></a>16777216                  | [MAV_SYS_STATUS_LOGGING](#MAV_SYS_STATUS_LOGGING)            | 0x1000000 日志记录                                           |
| <a id='MAV_SYS_STATUS_SENSOR_BATTERY'></a>33554432           | [MAV_SYS_STATUS_SENSOR_BATTERY](#MAV_SYS_STATUS_SENSOR_BATTERY) | 0x2000000 电池                                               |
| <a id='MAV_SYS_STATUS_SENSOR_PROXIMITY'></a>67108864         | [MAV_SYS_STATUS_SENSOR_PROXIMITY](#MAV_SYS_STATUS_SENSOR_PROXIMITY) | 0x4000000 邻近地区                                           |
| <a id='MAV_SYS_STATUS_SENSOR_SATCOM'></a>134217728           | [MAV_SYS_STATUS_SENSOR_SATCOM](#MAV_SYS_STATUS_SENSOR_SATCOM) | 0x8000000 卫星通信                                           |
| <a id='MAV_SYS_STATUS_PREARM_CHECK'></a>268435456            | [MAV_SYS_STATUS_PREARM_CHECK](#MAV_SYS_STATUS_PREARM_CHECK)  | 0x10000000 布防前检查状态。布防时始终处于健康状态            |
| <a id='MAV_SYS_STATUS_OBSTACLE_AVOIDANCE'></a>536870912      | [MAV_SYS_STATUS_OBSTACLE_AVOIDANCE](#MAV_SYS_STATUS_OBSTACLE_AVOIDANCE) | 0x20000000 避免/预防碰撞                                     |
| <a id='MAV_SYS_STATUS_SENSOR_PROPULSION'></a>1073741824      | [MAV_SYS_STATUS_SENSOR_PROPULSION](#MAV_SYS_STATUS_SENSOR_PROPULSION) | 0x40000000 推进器（推杆、擒纵器、电机或推进器）              |
| <a id='MAV_SYS_STATUS_EXTENSION_USED'></a>2147483648         | [MAV_SYS_STATUS_EXTENSION_USED](#MAV_SYS_STATUS_EXTENSION_USED) | 0x80000000 扩展位域用于更多传感器状态位（仅需在 onboard_control_sensors_present 中设置）。 |

### MAV_SYS_STATUS_SENSOR_EXTENDED 

(位掩码）对传感器进行编码，其状态将作为 [SYS_STATUS](#SYS_STATUS)报文的一部分在扩展字段中发送。

| 值                                           | 名称                                                         | 说明                                      |
| -------------------------------------------- | ------------------------------------------------------------ | ----------------------------------------- |
| <a id='MAV_SYS_STATUS_RECOVERY_SYSTEM'></a>1 | [MAV_SYS_STATUS_RECOVERY_SYSTEM](#MAV_SYS_STATUS_RECOVERY_SYSTEM) | 0x01 回收系统（降落伞、气球、收放装置等） |

### MAV_FRAME 

MAVLink 使用的坐标框架。并非所有命令、信息或飞行器都支持所有帧。


全局帧使用以下命名规则：
- "GLOBAL"（全球）： 全局坐标框架，默认为 WGS84 经纬度和平均海平面（MSL）正高度。
以下修饰符可与 "GLOBAL "一起使用：
- "relative_alt"（相对高度）： 海拔高度相对于车辆原点而非 MSL。
- "terrain_alt"（地形高度）： 相对于地面高度，而非 MSL。
- INT"： 纬度/经度（单位：度）按比例乘以 1E7。

本地帧使用以下命名规则：
- "LOCAL"（本地）： 本地框架的原点相对于地球是固定的。除非另有说明，否则该原点就是车辆位置估算器（"EKF"）的原点。
- 车身"： 本地框架的原点随车辆移动。注意，"BODY（车身）"并不表示车架轴线与车辆姿态对齐。
- 偏移"： 已废弃的 "BODY "同义词（原点随车辆移动）。不得用于新框架。

某些已废弃的帧不遵循这些约定（例如 [MAV_FRAME_BODY_NED]（#MAV_FRAME_BODY_NED）和 [MAV_FRAME_BODY_OFFSET_NED]（#MAV_FRAME_BODY_OFFSET_NED））。

| 值                                              | 名称                                                         | 说明                                                         |
| ----------------------------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| <a id='MAV_FRAME_GLOBAL'></a>0                  | [MAV_FRAME_GLOBAL](#MAV_FRAME_GLOBAL)                        | 全球（WGS84）坐标框架+相对于平均海平面（MSL）的高度。        |
| <a id='MAV_FRAME_LOCAL_NED'></a>1               | [MAV_FRAME_LOCAL_NED](#MAV_FRAME_LOCAL_NED)                  | NED 局部切线框架（x：北，y：东，z：下），原点相对于地球固定。 |
| <a id='MAV_FRAME_MISSION'></a>2                 | [MAV_FRAME_MISSION](#MAV_FRAME_MISSION)                      | 不是坐标框架，表示任务指令。                                 |
| <a id='MAV_FRAME_GLOBAL_RELATIVE_ALT'></a>3     | [MAV_FRAME_GLOBAL_RELATIVE_ALT](#MAV_FRAME_GLOBAL_RELATIVE_ALT) | 全球（WGS84）坐标系 + 相对于原点的高度。                     |
| <a id='MAV_FRAME_LOCAL_ENU'></a>4               | [MAV_FRAME_LOCAL_ENU](#MAV_FRAME_LOCAL_ENU)                  | ENU 本地切线框架（x：东，y：北，z：上），原点相对于地球固定。 |
| <a id='MAV_FRAME_GLOBAL_INT'></a>5              | [MAV_FRAME_GLOBAL_INT](#MAV_FRAME_GLOBAL_INT)                | 全球（WGS84）坐标框架（按比例）+ 相对于平均海平面（MSL）的高度。<b><span class="warning">**DEPRECATED:** Replaced By [MAV_FRAME_GLOBAL](#MAV_FRAME_GLOBAL) (2024-03) — Use [MAV_FRAME_GLOBAL](#MAV_FRAME_GLOBAL) in [COMMAND_INT](#COMMAND_INT) (和其他地方）作为同义替换）。</span> |
| <a id='MAV_FRAME_GLOBAL_RELATIVE_ALT_INT'></a>6 | [MAV_FRAME_GLOBAL_RELATIVE_ALT_INT](#MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) | 全球（WGS84）坐标系（缩放）+ 相对于原点的高度。<b><span class="warning">**DEPRECATED:** Replaced By [MAV_FRAME_GLOBAL_RELATIVE_ALT](#MAV_FRAME_GLOBAL_RELATIVE_ALT) (2024-03) — Use [MAV_FRAME_GLOBAL_RELATIVE_ALT](#MAV_FRAME_GLOBAL_RELATIVE_ALT) in [COMMAND_INT](#COMMAND_INT) (和其他地方）作为同义替换）。</span> |
| <a id='MAV_FRAME_LOCAL_OFFSET_NED'></a>7        | [MAV_FRAME_LOCAL_OFFSET_NED](#MAV_FRAME_LOCAL_OFFSET_NED)    | NED 本地切线框架（x：北，y：东，z：下），其原点与车辆一起移动。 |
| <a id='MAV_FRAME_BODY_NED'></a>8                | [MAV_FRAME_BODY_NED](#MAV_FRAME_BODY_NED)                    | Same as [MAV_FRAME_LOCAL_NED](#MAV_FRAME_LOCAL_NED) 用于表示位置值时。用于表示速度/加速度值时，与 [MAV_FRAME_BODY_FRD]（#MAV_FRAME_BODY_FRD）相同。<b><span class="warning">**DEPRECATED:** Replaced By [MAV_FRAME_BODY_FRD](#MAV_FRAME_BODY_FRD) (2019-08)</span> |
| <a id='MAV_FRAME_BODY_OFFSET_NED'></a>9         | [MAV_FRAME_BODY_OFFSET_NED](#MAV_FRAME_BODY_OFFSET_NED)      | 这与 [MAV_FRAME_BODY_FRD] 相同。(#MAV_FRAME_BODY_FRD).<b><span class="warning">**DEPRECATED:** Replaced By [MAV_FRAME_BODY_FRD](#MAV_FRAME_BODY_FRD) (2019-08)</span> |
| <a id='MAV_FRAME_GLOBAL_TERRAIN_ALT'></a>10     | [MAV_FRAME_GLOBAL_TERRAIN_ALT](#MAV_FRAME_GLOBAL_TERRAIN_ALT) | 全球（WGS84）坐标框架，AGL 高度（地面高度）。                |
| <a id='MAV_FRAME_GLOBAL_TERRAIN_ALT_INT'></a>11 | [MAV_FRAME_GLOBAL_TERRAIN_ALT_INT](#MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) | 全球（WGS84）坐标框架（按比例）与 AGL 高度（地面高度）。<b><span class="warning">**DEPRECATED:** Replaced By [MAV_FRAME_GLOBAL_TERRAIN_ALT](#MAV_FRAME_GLOBAL_TERRAIN_ALT) (2024-03) — Use [MAV_FRAME_GLOBAL_TERRAIN_ALT](#MAV_FRAME_GLOBAL_TERRAIN_ALT) in [COMMAND_INT](#COMMAND_INT)  (和其他地方）作为同义替换）。</span> |
| <a id='MAV_FRAME_BODY_FRD'></a>12               | [MAV_FRAME_BODY_FRD](#MAV_FRAME_BODY_FRD)                    | FRD 本地帧与车辆姿态（x：向前，y：向右，z：向下）对齐，原点随车辆移动。 |
| <a id='MAV_FRAME_RESERVED_13'></a>13            | [MAV_FRAME_RESERVED_13](#MAV_FRAME_RESERVED_13)              | [MAV_FRAME_BODY_FLU](#MAV_FRAME_BODY_FLU) - 身体固定参照系，Z 向上（x：向前，y：向左，z：向上）。<b><span class="warning">**DEPRECATED:**(2019-04)</span> |
| <a id='MAV_FRAME_RESERVED_14'></a>14            | [MAV_FRAME_RESERVED_14](#MAV_FRAME_RESERVED_14)              | [MAV_FRAME_MOCAP_NED](#MAV_FRAME_MOCAP_NED) - 由动作捕捉系统提供的数据的局部坐标系，Z-down（x：北，y：东，z：下）。<b><span class="warning">**DEPRECATED:** Replaced By [MAV_FRAME_LOCAL_FRD](#MAV_FRAME_LOCAL_FRD) (2019-04)</span> |
| <a id='MAV_FRAME_RESERVED_15'></a>15            | [MAV_FRAME_RESERVED_15](#MAV_FRAME_RESERVED_15)              | [MAV_FRAME_MOCAP_ENU](#MAV_FRAME_MOCAP_ENU) - 运动捕捉系统提供的数据的局部坐标系，Z-up（x：东，y：北，z：上）。<b><span class="warning">**DEPRECATED:** Replaced By [MAV_FRAME_LOCAL_FLU](#MAV_FRAME_LOCAL_FLU) (2019-04)</span> |
| <a id='MAV_FRAME_RESERVED_16'></a>16            | [MAV_FRAME_RESERVED_16](#MAV_FRAME_RESERVED_16)              | [MAV_FRAME_VISION_NED](#MAV_FRAME_VISION_NED) -  由视觉估算系统提供的视场角局部坐标系数据，Z-down（x：北，y：东，z：下）。<b><span class="warning">**DEPRECATED:** Replaced By [MAV_FRAME_LOCAL_FRD](#MAV_FRAME_LOCAL_FRD) (2019-04)</span> |
| <a id='MAV_FRAME_RESERVED_17'></a>17            | [MAV_FRAME_RESERVED_17](#MAV_FRAME_RESERVED_17)              | [MAV_FRAME_VISION_ENU](#MAV_FRAME_VISION_ENU) - 由视觉估算系统提供的视场角本地坐标系数据，Z-up（x：东，y：北，z：上）。<b><span class="warning">**DEPRECATED:** Replaced By [MAV_FRAME_LOCAL_FLU](#MAV_FRAME_LOCAL_FLU) (2019-04)</span> |
| <a id='MAV_FRAME_RESERVED_18'></a>18            | [MAV_FRAME_RESERVED_18](#MAV_FRAME_RESERVED_18)              | [MAV_FRAME_ESTIM_NED](#MAV_FRAME_ESTIM_NED) - 由车载估算器提供的视距测量本地坐标系数据，Z-down（x：北，y：东，z：下）。<b><span class="warning">**DEPRECATED:** Replaced By [MAV_FRAME_LOCAL_FRD](#MAV_FRAME_LOCAL_FRD) (2019-04)</span> |
| <a id='MAV_FRAME_RESERVED_19'></a>19            | [MAV_FRAME_RESERVED_19](#MAV_FRAME_RESERVED_19)              | [MAV_FRAME_ESTIM_ENU](#MAV_FRAME_ESTIM_ENU) - 由车载估算器提供的运动轨迹本地坐标系数据，Z-up（x：东，y：北，z：上）。<b><span class="warning">**DEPRECATED:** Replaced By [MAV_FRAME_LOCAL_FLU](#MAV_FRAME_LOCAL_FLU) (2019-04)</span> |
| <a id='MAV_FRAME_LOCAL_FRD'></a>20              | [MAV_FRAME_LOCAL_FRD](#MAV_FRAME_LOCAL_FRD)                  | FRD 本地切线框架（x：向前，y：向右，z：向下），原点相对于地球固定。前轴在水平面内对准飞行器的前方。 |
| <a id='MAV_FRAME_LOCAL_FLU'></a>21              | [MAV_FRAME_LOCAL_FLU](#MAV_FRAME_LOCAL_FLU)                  | FLU 本地切线框架（x：向前，y：向左，z：向上），原点相对于地球固定。前轴在水平面内对准飞行器的前方。 |

### MAVLINK_DATA_STREAM_TYPE 

| Value                                        | Name                                                         | Description |
| -------------------------------------------- | ------------------------------------------------------------ | ----------- |
| <a id='MAVLINK_DATA_STREAM_IMG_JPEG'></a>0   | [MAVLINK_DATA_STREAM_IMG_JPEG](#MAVLINK_DATA_STREAM_IMG_JPEG) |             |
| <a id='MAVLINK_DATA_STREAM_IMG_BMP'></a>1    | [MAVLINK_DATA_STREAM_IMG_BMP](#MAVLINK_DATA_STREAM_IMG_BMP)  |             |
| <a id='MAVLINK_DATA_STREAM_IMG_RAW8U'></a>2  | [MAVLINK_DATA_STREAM_IMG_RAW8U](#MAVLINK_DATA_STREAM_IMG_RAW8U) |             |
| <a id='MAVLINK_DATA_STREAM_IMG_RAW32U'></a>3 | [MAVLINK_DATA_STREAM_IMG_RAW32U](#MAVLINK_DATA_STREAM_IMG_RAW32U) |             |
| <a id='MAVLINK_DATA_STREAM_IMG_PGM'></a>4    | [MAVLINK_DATA_STREAM_IMG_PGM](#MAVLINK_DATA_STREAM_IMG_PGM)  |             |
| <a id='MAVLINK_DATA_STREAM_IMG_PNG'></a>5    | [MAVLINK_DATA_STREAM_IMG_PNG](#MAVLINK_DATA_STREAM_IMG_PNG)  |             |

### FENCE_ACTION 

违反地理围栏后采取的行动。

| 值                                         | 名称                                                         | 说明                                                         |
| ------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |
| <a id='FENCE_ACTION_NONE'></a>0            | [FENCE_ACTION_NONE](#FENCE_ACTION_NONE)                      | 禁用栅栏模式。如果在计划中使用，这意味着下一个栅栏将被禁用。 |
| <a id='FENCE_ACTION_GUIDED'></a>1          | [FENCE_ACTION_GUIDED](#FENCE_ACTION_GUIDED)                  | 在 GUIDED 模式下飞向地理围栏 [MAV_CMD_NAV_FENCE_RETURN_POINT](#MAV_CMD_NAV_FENCE_RETURN_POINT)。注意：此操作仅受 ArduPlane 支持，所有版本可能都不支持。 |
| <a id='FENCE_ACTION_REPORT'></a>2          | [FENCE_ACTION_REPORT](#FENCE_ACTION_REPORT)                  | 报告栅栏漏洞，但不采取行动                                   |
| <a id='FENCE_ACTION_GUIDED_THR_PASS'></a>3 | [FENCE_ACTION_GUIDED_THR_PASS](#FENCE_ACTION_GUIDED_THR_PASS) | 在 GUIDED 模式下手动控制油门飞至地理围栏 [MAV_CMD_NAV_FENCE_RETURN_POINT](#MAV_CMD_NAV_FENCE_RETURN_POINT)。注意：此操作仅受 ArduPlane 支持，所有版本可能都不支持。 |
| <a id='FENCE_ACTION_RTL'></a>4             | [FENCE_ACTION_RTL](#FENCE_ACTION_RTL)                        | 返回/RTL 模式。                                              |
| <a id='FENCE_ACTION_HOLD'></a>5            | [FENCE_ACTION_HOLD](#FENCE_ACTION_HOLD)                      | 保持在当前位置。                                             |
| <a id='FENCE_ACTION_TERMINATE'></a>6       | [FENCE_ACTION_TERMINATE](#FENCE_ACTION_TERMINATE)            | 终止故障保护。电机关闭（某些飞行堆栈可能会触发其他故障保护动作）。 |
| <a id='FENCE_ACTION_LAND'></a>7            | [FENCE_ACTION_LAND](#FENCE_ACTION_LAND)                      | 当前位置的土地。                                             |

### FENCE_BREACH 

| Value                               | Name                                            | Description                     |
| ----------------------------------- | ----------------------------------------------- | ------------------------------- |
| <a id='FENCE_BREACH_NONE'></a>0     | [FENCE_BREACH_NONE](#FENCE_BREACH_NONE)         | 没有最后一个围栏缺口(没有闭合?) |
| <a id='FENCE_BREACH_MINALT'></a>1   | [FENCE_BREACH_MINALT](#FENCE_BREACH_MINALT)     | 突破最低飞行高度                |
| <a id='FENCE_BREACH_MAXALT'></a>2   | [FENCE_BREACH_MAXALT](#FENCE_BREACH_MAXALT)     | 突破最大飞行高度                |
| <a id='FENCE_BREACH_BOUNDARY'></a>3 | [FENCE_BREACH_BOUNDARY](#FENCE_BREACH_BOUNDARY) | 突破围栏边界                    |

### FENCE_MITIGATE 

为减少/防止栅栏破坏而采取的行动

| 值                                     | 名称                                                  | 说明                       |
| -------------------------------------- | ----------------------------------------------------- | -------------------------- |
| <a id='FENCE_MITIGATE_UNKNOWN'></a>0   | [FENCE_MITIGATE_UNKNOWN](#FENCE_MITIGATE_UNKNOWN)     | 未知                       |
| <a id='FENCE_MITIGATE_NONE'></a>1      | [FENCE_MITIGATE_NONE](#FENCE_MITIGATE_NONE)           | 未采取任何行动             |
| <a id='FENCE_MITIGATE_VEL_LIMIT'></a>2 | [FENCE_MITIGATE_VEL_LIMIT](#FENCE_MITIGATE_VEL_LIMIT) | 主动限制速度以防止违规行为 |

### FENCE_TYPE 

(比特掩码) 

| Value                            | Name                                      | Description  |
| -------------------------------- | ----------------------------------------- | ------------ |
| <a id='FENCE_TYPE_ALL'></a>0     | [FENCE_TYPE_ALL](#FENCE_TYPE_ALL)         | 所有围栏类型 |
| <a id='FENCE_TYPE_ALT_MAX'></a>1 | [FENCE_TYPE_ALT_MAX](#FENCE_TYPE_ALT_MAX) | 最大高度围栏 |
| <a id='FENCE_TYPE_CIRCLE'></a>2  | [FENCE_TYPE_CIRCLE](#FENCE_TYPE_CIRCLE)   | 圆形围栏     |
| <a id='FENCE_TYPE_POLYGON'></a>4 | [FENCE_TYPE_POLYGON](#FENCE_TYPE_POLYGON) | 多边形围栏   |
| <a id='FENCE_TYPE_ALT_MIN'></a>8 | [FENCE_TYPE_ALT_MIN](#FENCE_TYPE_ALT_MIN) | 最低高度围栏 |

### MAV_MOUNT_MODE — [DEP] 

<span class="warning">**DEPRECATED:** Replaced By [GIMBAL_MANAGER_FLAGS](#GIMBAL_MANAGER_FLAGS) (2020-01)</span>

可能的安装操作模式枚举。该信息用于过时/废弃的云台信息。

| 值                                             | 名称                                                         | 说明                                                         |
| ---------------------------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| <a id='MAV_MOUNT_MODE_RETRACT'></a>0           | [MAV_MOUNT_MODE_RETRACT](#MAV_MOUNT_MODE_RETRACT)            | 从永久记忆中加载并保持安全位置（滚动、俯仰、偏航），并停止稳定功能 |
| <a id='MAV_MOUNT_MODE_NEUTRAL'></a>1           | [MAV_MOUNT_MODE_NEUTRAL](#MAV_MOUNT_MODE_NEUTRAL)            | 从永久记忆中加载并保持中立位置（滚动、俯仰、偏航）。         |
| <a id='MAV_MOUNT_MODE_MAVLINK_TARGETING'></a>2 | [MAV_MOUNT_MODE_MAVLINK_TARGETING](#MAV_MOUNT_MODE_MAVLINK_TARGETING) | 加载中立位置并启动 MAVLink 滚转、俯仰、偏航稳定控制          |
| <a id='MAV_MOUNT_MODE_RC_TARGETING'></a>3      | [MAV_MOUNT_MODE_RC_TARGETING](#MAV_MOUNT_MODE_RC_TARGETING)  | 加载中立位置并启动带稳定功能的遥控滚动、俯仰和偏航控制       |
| <a id='MAV_MOUNT_MODE_GPS_POINT'></a>4         | [MAV_MOUNT_MODE_GPS_POINT](#MAV_MOUNT_MODE_GPS_POINT)        | 加载中立位置并开始指向纬度、经度和高度                       |
| <a id='MAV_MOUNT_MODE_SYSID_TARGET'></a>5      | [MAV_MOUNT_MODE_SYSID_TARGET](#MAV_MOUNT_MODE_SYSID_TARGET)  | 万向节通过指定的系统 ID 跟踪系统                             |
| <a id='MAV_MOUNT_MODE_HOME_LOCATION'></a>6     | [MAV_MOUNT_MODE_HOME_LOCATION](#MAV_MOUNT_MODE_HOME_LOCATION) | 云台跟踪原点                                                 |

### GIMBAL_DEVICE_CAP_FLAGS 

(位图）万向节设备（低级）能力标志（位图）。

| 值                                                           | 名称                                                         | 说明                                                         |
| ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |
| <a id='GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT'></a>1            | [GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT](#GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT) | 万向节装置支持缩回位置。                                     |
| <a id='GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL'></a>2            | [GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL](#GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL) | 云台装置支持水平、前视、稳定位置。                           |
| <a id='GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS'></a>4          | [GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS](#GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS) | 云台装置支持绕滚动轴旋转。                                   |
| <a id='GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW'></a>8        | [GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW](#GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW) | 云台装置支持跟踪相对于车辆的滚动角度。                       |
| <a id='GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK'></a>16         | [GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK](#GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK) | 云台设备支持锁定滚动角度（一般来说，滚动稳定的默认值就是这个角度）。 |
| <a id='GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS'></a>32        | [GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS](#GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS) | 云台装置支持绕俯仰轴旋转。                                   |
| <a id='GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW'></a>64      | [GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW](#GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW) | 云台装置支持跟踪相对于飞行器的俯仰角度。                     |
| <a id='GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK'></a>128       | [GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK](#GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK) | 云台设备支持锁定俯仰角度（一般情况下，俯仰稳定装置默认锁定俯仰角度）。 |
| <a id='GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS'></a>256         | [GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS](#GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS) | 云台装置支持绕偏航轴旋转。                                   |
| <a id='GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW'></a>512       | [GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW](#GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW) | 云台设备支持相对于车辆的偏航角度（通常为默认值）。           |
| <a id='GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK'></a>1024        | [GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK](#GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK) | 云台设备支持锁定绝对航向，即相对于北（地球框架，通常是一个可用选项）的偏航角。 |
| <a id='GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW'></a>2048 | [GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW](#GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW) | 云台装置支持无限偏航/摇摆（例如使用滑动盘）。                |
| <a id='GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_YAW_IN_EARTH_FRAME'></a>4096 | [GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_YAW_IN_EARTH_FRAME](#GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_YAW_IN_EARTH_FRAME) | 云台设备支持相对于北方（地球框架）的偏航角和角速度。这通常需要自动驾驶仪通过 [AUTOPILOT_STATE_FOR_GIMBAL_DEVICE]（#AUTOPILOT_STATE_FOR_GIMBAL_DEVICE）提供支持。在运行期间，支持可以开启或关闭，这通过标志[GIMBAL_DEVICE_FLAGS_CAN_ACCEPT_YAW_IN_EARTH_FRAME](#GIMBAL_DEVICE_FLAGS_CAN_ACCEPT_YAW_IN_EARTH_FRAME)进行报告。 |
| <a id='GIMBAL_DEVICE_CAP_FLAGS_HAS_RC_INPUTS'></a>8192       | [GIMBAL_DEVICE_CAP_FLAGS_HAS_RC_INPUTS](#GIMBAL_DEVICE_CAP_FLAGS_HAS_RC_INPUTS) | 云台设备支持无线电控制输入，作为控制万向节方向的替代输入。   |

### GIMBAL_MANAGER_CAP_FLAGS 

(位掩码）万向节管理器高级功能标志（位图）。前 16 位与 [GIMBAL_DEVICE_CAP_FLAGS](#GIMBAL_DEVICE_CAP_FLAGS) 相同。不过，万向节管理器不需要复制万向节的标志，也可以增强功能，从而添加标志。

| 值                                                           | 名称                                                         | 说明                                                         |
| ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |
| <a id='GIMBAL_MANAGER_CAP_FLAGS_HAS_RETRACT'></a>1           | [GIMBAL_MANAGER_CAP_FLAGS_HAS_RETRACT](#GIMBAL_MANAGER_CAP_FLAGS_HAS_RETRACT) | Based on [GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT](#GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT). |
| <a id='GIMBAL_MANAGER_CAP_FLAGS_HAS_NEUTRAL'></a>2           | [GIMBAL_MANAGER_CAP_FLAGS_HAS_NEUTRAL](#GIMBAL_MANAGER_CAP_FLAGS_HAS_NEUTRAL) | Based on [GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL](#GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL). |
| <a id='GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_AXIS'></a>4         | [GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_AXIS](#GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_AXIS) | Based on [GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS](#GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS). |
| <a id='GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_FOLLOW'></a>8       | [GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_FOLLOW](#GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_FOLLOW) | Based on [GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW](#GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW). |
| <a id='GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_LOCK'></a>16        | [GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_LOCK](#GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_LOCK) | Based on [GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK](#GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK). |
| <a id='GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_AXIS'></a>32       | [GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_AXIS](#GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_AXIS) | Based on [GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS](#GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS). |
| <a id='GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_FOLLOW'></a>64     | [GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_FOLLOW](#GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_FOLLOW) | Based on [GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW](#GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW). |
| <a id='GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_LOCK'></a>128      | [GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_LOCK](#GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_LOCK) | Based on [GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK](#GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK). |
| <a id='GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_AXIS'></a>256        | [GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_AXIS](#GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_AXIS) | Based on [GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS](#GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS). |
| <a id='GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_FOLLOW'></a>512      | [GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_FOLLOW](#GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_FOLLOW) | Based on [GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW](#GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW). |
| <a id='GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_LOCK'></a>1024       | [GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_LOCK](#GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_LOCK) | Based on [GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK](#GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK). |
| <a id='GIMBAL_MANAGER_CAP_FLAGS_SUPPORTS_INFINITE_YAW'></a>2048 | [GIMBAL_MANAGER_CAP_FLAGS_SUPPORTS_INFINITE_YAW](#GIMBAL_MANAGER_CAP_FLAGS_SUPPORTS_INFINITE_YAW) | Based on [GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW](#GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW). |
| <a id='GIMBAL_MANAGER_CAP_FLAGS_SUPPORTS_YAW_IN_EARTH_FRAME'></a>4096 | [GIMBAL_MANAGER_CAP_FLAGS_SUPPORTS_YAW_IN_EARTH_FRAME](#GIMBAL_MANAGER_CAP_FLAGS_SUPPORTS_YAW_IN_EARTH_FRAME) | Based on [GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_YAW_IN_EARTH_FRAME](#GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_YAW_IN_EARTH_FRAME). |
| <a id='GIMBAL_MANAGER_CAP_FLAGS_HAS_RC_INPUTS'></a>8192      | [GIMBAL_MANAGER_CAP_FLAGS_HAS_RC_INPUTS](#GIMBAL_MANAGER_CAP_FLAGS_HAS_RC_INPUTS) | Based on [GIMBAL_DEVICE_CAP_FLAGS_HAS_RC_INPUTS](#GIMBAL_DEVICE_CAP_FLAGS_HAS_RC_INPUTS). |
| <a id='GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_LOCAL'></a>65536 | [GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_LOCAL](#GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_LOCAL) | 云台管理器支持指向本地位置。                                 |
| <a id='GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_GLOBAL'></a>131072 | [GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_GLOBAL](#GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_GLOBAL) | 云台管理器支持指向全球经纬度和高度位置。                     |

### GIMBAL_DEVICE_FLAGS 

(位掩码）万向节设备（下位）运行标志。

| 值                                                           | 名称                                                         | 说明                                                         |
| ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |
| <a id='GIMBAL_DEVICE_FLAGS_RETRACT'></a>1                    | [GIMBAL_DEVICE_FLAGS_RETRACT](#GIMBAL_DEVICE_FLAGS_RETRACT)  | 设置为缩回安全位置（无稳定功能），优先于所有其他标志         |
| <a id='GIMBAL_DEVICE_FLAGS_NEUTRAL'></a>2                    | [GIMBAL_DEVICE_FLAGS_NEUTRAL](#GIMBAL_DEVICE_FLAGS_NEUTRAL)  | 设置为中位/默认位置，优先于除 RETRACT 以外的所有其他标志。中位通常是朝前的水平位置（滚转=俯仰=偏航=0），但也可以是任何方向。 |
| <a id='GIMBAL_DEVICE_FLAGS_ROLL_LOCK'></a>4                  | [GIMBAL_DEVICE_FLAGS_ROLL_LOCK](#GIMBAL_DEVICE_FLAGS_ROLL_LOCK) | 将滚转角度锁定为相对地平线（而非相对飞行器）的绝对角度。这通常是稳定云台的默认设置。 |
| <a id='GIMBAL_DEVICE_FLAGS_PITCH_LOCK'></a>8                 | [GIMBAL_DEVICE_FLAGS_PITCH_LOCK](#GIMBAL_DEVICE_FLAGS_PITCH_LOCK) | 将俯仰角锁定为相对地平线的绝对角度（而非相对飞行器的角度）。这通常是稳定云台的默认设置。 |
| <a id='GIMBAL_DEVICE_FLAGS_YAW_LOCK'></a>16                  | [GIMBAL_DEVICE_FLAGS_YAW_LOCK](#GIMBAL_DEVICE_FLAGS_YAW_LOCK) | 将偏航角锁定为相对于北（而非相对于车辆）的绝对角度。如果设置了该标记，则偏航角和角速度的 z 分量相对于北方（地球框架，X 轴指向北方），否则它们相对于车辆航向（车辆框架，地球框架旋转后 X 轴指向前方）。 |
| <a id='GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME'></a>32      | [GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME](#GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME) | 偏航角和角速度的 Z 分量是相对于飞行器航向（飞行器框架，地球框架旋转后，X 轴指向前方）而言的。 |
| <a id='GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME'></a>64        | [GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME](#GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME) | 偏航角和角速度的 Z 分量是相对于北方的（地球框架，X 轴指向北方）。 |
| <a id='GIMBAL_DEVICE_FLAGS_ACCEPTS_YAW_IN_EARTH_FRAME'></a>128 | [GIMBAL_DEVICE_FLAGS_ACCEPTS_YAW_IN_EARTH_FRAME](#GIMBAL_DEVICE_FLAGS_ACCEPTS_YAW_IN_EARTH_FRAME) | 云台设备可以接受相对于北方（地球帧）的偏航角输入。该标记仅用于报告（设置该标记的尝试将被忽略）。 |
| <a id='GIMBAL_DEVICE_FLAGS_RC_EXCLUSIVE'></a>256             | [GIMBAL_DEVICE_FLAGS_RC_EXCLUSIVE](#GIMBAL_DEVICE_FLAGS_RC_EXCLUSIVE) | 万向节方向完全由馈送至万向节无线电控制输入端的遥控信号设置。设置云台方向的 MAVLink 信息（[GIMBAL_DEVICE_SET_ATTITUDE](#GIMBAL_DEVICE_SET_ATTITUDE)）将被忽略。 |
| <a id='GIMBAL_DEVICE_FLAGS_RC_MIXED'></a>512                 | [GIMBAL_DEVICE_FLAGS_RC_MIXED](#GIMBAL_DEVICE_FLAGS_RC_MIXED) | 万向节的方向由馈送至万向节无线电控制输入端的遥控信号和用于设置万向节方向的 MAVLink 信息（[GIMBAL_DEVICE_SET_ATTITUDE](#GIMBAL_DEVICE_SET_ATTITUDE)）的组合/混合决定。如何将这两种控制方式结合或混合在一起，协议中并没有规定，而是由实现方式自行决定。 |

### GIMBAL_MANAGER_FLAGS 

(前 16 位与 [GIMBAL_DEVICE_FLAGS]（#GIMBAL_DEVICE_FLAGS）相同。）

| Value                                                        | Name                                                         | Description                                                  |
| ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |
| <a id='GIMBAL_MANAGER_FLAGS_RETRACT'></a>1                   | [GIMBAL_MANAGER_FLAGS_RETRACT](#GIMBAL_MANAGER_FLAGS_RETRACT) | Based on [GIMBAL_DEVICE_FLAGS_RETRACT](#GIMBAL_DEVICE_FLAGS_RETRACT). |
| <a id='GIMBAL_MANAGER_FLAGS_NEUTRAL'></a>2                   | [GIMBAL_MANAGER_FLAGS_NEUTRAL](#GIMBAL_MANAGER_FLAGS_NEUTRAL) | Based on [GIMBAL_DEVICE_FLAGS_NEUTRAL](#GIMBAL_DEVICE_FLAGS_NEUTRAL). |
| <a id='GIMBAL_MANAGER_FLAGS_ROLL_LOCK'></a>4                 | [GIMBAL_MANAGER_FLAGS_ROLL_LOCK](#GIMBAL_MANAGER_FLAGS_ROLL_LOCK) | Based on [GIMBAL_DEVICE_FLAGS_ROLL_LOCK](#GIMBAL_DEVICE_FLAGS_ROLL_LOCK). |
| <a id='GIMBAL_MANAGER_FLAGS_PITCH_LOCK'></a>8                | [GIMBAL_MANAGER_FLAGS_PITCH_LOCK](#GIMBAL_MANAGER_FLAGS_PITCH_LOCK) | Based on [GIMBAL_DEVICE_FLAGS_PITCH_LOCK](#GIMBAL_DEVICE_FLAGS_PITCH_LOCK). |
| <a id='GIMBAL_MANAGER_FLAGS_YAW_LOCK'></a>16                 | [GIMBAL_MANAGER_FLAGS_YAW_LOCK](#GIMBAL_MANAGER_FLAGS_YAW_LOCK) | Based on [GIMBAL_DEVICE_FLAGS_YAW_LOCK](#GIMBAL_DEVICE_FLAGS_YAW_LOCK). |
| <a id='GIMBAL_MANAGER_FLAGS_YAW_IN_VEHICLE_FRAME'></a>32     | [GIMBAL_MANAGER_FLAGS_YAW_IN_VEHICLE_FRAME](#GIMBAL_MANAGER_FLAGS_YAW_IN_VEHICLE_FRAME) | Based on [GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME](#GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME). |
| <a id='GIMBAL_MANAGER_FLAGS_YAW_IN_EARTH_FRAME'></a>64       | [GIMBAL_MANAGER_FLAGS_YAW_IN_EARTH_FRAME](#GIMBAL_MANAGER_FLAGS_YAW_IN_EARTH_FRAME) | Based on [GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME](#GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME). |
| <a id='GIMBAL_MANAGER_FLAGS_ACCEPTS_YAW_IN_EARTH_FRAME'></a>128 | [GIMBAL_MANAGER_FLAGS_ACCEPTS_YAW_IN_EARTH_FRAME](#GIMBAL_MANAGER_FLAGS_ACCEPTS_YAW_IN_EARTH_FRAME) | Based on [GIMBAL_DEVICE_FLAGS_ACCEPTS_YAW_IN_EARTH_FRAME](#GIMBAL_DEVICE_FLAGS_ACCEPTS_YAW_IN_EARTH_FRAME). |
| <a id='GIMBAL_MANAGER_FLAGS_RC_EXCLUSIVE'></a>256            | [GIMBAL_MANAGER_FLAGS_RC_EXCLUSIVE](#GIMBAL_MANAGER_FLAGS_RC_EXCLUSIVE) | Based on [GIMBAL_DEVICE_FLAGS_RC_EXCLUSIVE](#GIMBAL_DEVICE_FLAGS_RC_EXCLUSIVE). |
| <a id='GIMBAL_MANAGER_FLAGS_RC_MIXED'></a>512                | [GIMBAL_MANAGER_FLAGS_RC_MIXED](#GIMBAL_MANAGER_FLAGS_RC_MIXED) | Based on [GIMBAL_DEVICE_FLAGS_RC_MIXED](#GIMBAL_DEVICE_FLAGS_RC_MIXED). |

### GIMBAL_DEVICE_ERROR_FLAGS 

(位图）万向节设备（低电平）错误标志（位图，0 表示无错误）

| 值                                                           | 名称                                                         | 说明                             |
| ------------------------------------------------------------ | ------------------------------------------------------------ | -------------------------------- |
| <a id='GIMBAL_DEVICE_ERROR_FLAGS_AT_ROLL_LIMIT'></a>1        | [GIMBAL_DEVICE_ERROR_FLAGS_AT_ROLL_LIMIT](#GIMBAL_DEVICE_ERROR_FLAGS_AT_ROLL_LIMIT) | 云台装置受硬件滚动限制。         |
| <a id='GIMBAL_DEVICE_ERROR_FLAGS_AT_PITCH_LIMIT'></a>2       | [GIMBAL_DEVICE_ERROR_FLAGS_AT_PITCH_LIMIT](#GIMBAL_DEVICE_ERROR_FLAGS_AT_PITCH_LIMIT) | 万向节装置受硬件间距限制。       |
| <a id='GIMBAL_DEVICE_ERROR_FLAGS_AT_YAW_LIMIT'></a>4         | [GIMBAL_DEVICE_ERROR_FLAGS_AT_YAW_LIMIT](#GIMBAL_DEVICE_ERROR_FLAGS_AT_YAW_LIMIT) | 万向节装置受硬件偏航限制。       |
| <a id='GIMBAL_DEVICE_ERROR_FLAGS_ENCODER_ERROR'></a>8        | [GIMBAL_DEVICE_ERROR_FLAGS_ENCODER_ERROR](#GIMBAL_DEVICE_ERROR_FLAGS_ENCODER_ERROR) | 万向节编码器出错。               |
| <a id='GIMBAL_DEVICE_ERROR_FLAGS_POWER_ERROR'></a>16         | [GIMBAL_DEVICE_ERROR_FLAGS_POWER_ERROR](#GIMBAL_DEVICE_ERROR_FLAGS_POWER_ERROR) | 云台电源出现错误。               |
| <a id='GIMBAL_DEVICE_ERROR_FLAGS_MOTOR_ERROR'></a>32         | [GIMBAL_DEVICE_ERROR_FLAGS_MOTOR_ERROR](#GIMBAL_DEVICE_ERROR_FLAGS_MOTOR_ERROR) | 万向节电机出现错误。             |
| <a id='GIMBAL_DEVICE_ERROR_FLAGS_SOFTWARE_ERROR'></a>64      | [GIMBAL_DEVICE_ERROR_FLAGS_SOFTWARE_ERROR](#GIMBAL_DEVICE_ERROR_FLAGS_SOFTWARE_ERROR) | 云台软件出错。                   |
| <a id='GIMBAL_DEVICE_ERROR_FLAGS_COMMS_ERROR'></a>128        | [GIMBAL_DEVICE_ERROR_FLAGS_COMMS_ERROR](#GIMBAL_DEVICE_ERROR_FLAGS_COMMS_ERROR) | 云台通信出错。                   |
| <a id='GIMBAL_DEVICE_ERROR_FLAGS_CALIBRATION_RUNNING'></a>256 | [GIMBAL_DEVICE_ERROR_FLAGS_CALIBRATION_RUNNING](#GIMBAL_DEVICE_ERROR_FLAGS_CALIBRATION_RUNNING) | 云台设备正在校准。               |
| <a id='GIMBAL_DEVICE_ERROR_FLAGS_NO_MANAGER'></a>512         | [GIMBAL_DEVICE_ERROR_FLAGS_NO_MANAGER](#GIMBAL_DEVICE_ERROR_FLAGS_NO_MANAGER) | 万向节设备未分配给万向节管理器。 |

### GRIPPER_ACTIONS 

抓取动作。

| Value                                | Name                                              | Description      |
| ------------------------------------ | ------------------------------------------------- | ---------------- |
| <a id='GRIPPER_ACTION_RELEASE'></a>0 | [GRIPPER_ACTION_RELEASE](#GRIPPER_ACTION_RELEASE) | 夹持器释放货物。 |
| <a id='GRIPPER_ACTION_GRAB'></a>1    | [GRIPPER_ACTION_GRAB](#GRIPPER_ACTION_GRAB)       | 抓手抓住货物。   |

### WINCH_ACTIONS 

绞盘操作。

| Value                                       | Name                                                         | Description                                                  |
| ------------------------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| <a id='WINCH_RELAXED'></a>0                 | [WINCH_RELAXED](#WINCH_RELAXED)                              | 让电机自由转动。                                             |
| <a id='WINCH_RELATIVE_LENGTH_CONTROL'></a>1 | [WINCH_RELATIVE_LENGTH_CONTROL](#WINCH_RELATIVE_LENGTH_CONTROL) | 收卷或放卷指定长度的线路，可选择使用指定速率。               |
| <a id='WINCH_RATE_CONTROL'></a>2            | [WINCH_RATE_CONTROL](#WINCH_RATE_CONTROL)                    | 以指定速度卷绕或松开缆线。                                   |
| <a id='WINCH_LOCK'></a>3                    | [WINCH_LOCK](#WINCH_LOCK)                                    | 当电机处于完全缩回位置时，执行锁定顺序以释放电机。仅使用动作和实例命令参数，其他参数忽略不计。 |
| <a id='WINCH_DELIVER'></a>4                 | [WINCH_DELIVER](#WINCH_DELIVER)                              | 下降、减速、触地、卷起、锁定的顺序。仅使用动作和实例命令参数，其他参数忽略不计。 |
| <a id='WINCH_HOLD'></a>5                    | [WINCH_HOLD](#WINCH_HOLD)                                    | 启动电机并保持当前位置。仅使用动作和实例命令参数，其他参数忽略不计。 |
| <a id='WINCH_RETRACT'></a>6                 | [WINCH_RETRACT](#WINCH_RETRACT)                              | 将卷盘返回到完全缩回位置。仅使用动作和实例命令参数，其他参数忽略不计。 |
| <a id='WINCH_LOAD_LINE'></a>7               | [WINCH_LOAD_LINE](#WINCH_LOAD_LINE)                          | 将缆线装入卷轴。绞盘会计算装载的总长度，并在张力超过临界值时停止。仅使用动作和实例命令参数，其他参数忽略不计。 |
| <a id='WINCH_ABANDON_LINE'></a>8            | [WINCH_ABANDON_LINE](#WINCH_ABANDON_LINE)                    | Spool 输出整行长度。仅使用动作和实例命令参数，其他参数忽略不计。 |
| <a id='WINCH_LOAD_PAYLOAD'></a>9            | [WINCH_LOAD_PAYLOAD](#WINCH_LOAD_PAYLOAD)                    | 向用户提供足够的钩子来加载有效载荷。只使用动作和实例命令参数，其他参数将被忽略 |

### UAVCAN_NODE_HEALTH 

通用 UAVCAN 节点健康状况

| Value                                     | Name                                                        | Description                          |
| ----------------------------------------- | ----------------------------------------------------------- | ------------------------------------ |
| <a id='UAVCAN_NODE_HEALTH_OK'></a>0       | [UAVCAN_NODE_HEALTH_OK](#UAVCAN_NODE_HEALTH_OK)             | 节点运行正常。                       |
| <a id='UAVCAN_NODE_HEALTH_WARNING'></a>1  | [UAVCAN_NODE_HEALTH_WARNING](#UAVCAN_NODE_HEALTH_WARNING)   | 关键参数超出范围或节点出现轻微故障。 |
| <a id='UAVCAN_NODE_HEALTH_ERROR'></a>2    | [UAVCAN_NODE_HEALTH_ERROR](#UAVCAN_NODE_HEALTH_ERROR)       | 节点遇到重大故障。                   |
| <a id='UAVCAN_NODE_HEALTH_CRITICAL'></a>3 | [UAVCAN_NODE_HEALTH_CRITICAL](#UAVCAN_NODE_HEALTH_CRITICAL) | 节点发生致命故障。                   |

### UAVCAN_NODE_MODE 

通用 UAVCAN 节点模式

| Value                                          | Name                                                         | Description                            |
| ---------------------------------------------- | ------------------------------------------------------------ | -------------------------------------- |
| <a id='UAVCAN_NODE_MODE_OPERATIONAL'></a>0     | [UAVCAN_NODE_MODE_OPERATIONAL](#UAVCAN_NODE_MODE_OPERATIONAL) | 节点正在执行其主要功能。               |
| <a id='UAVCAN_NODE_MODE_INITIALIZATION'></a>1  | [UAVCAN_NODE_MODE_INITIALIZATION](#UAVCAN_NODE_MODE_INITIALIZATION) | 节点正在初始化；启动后立即进入该模式。 |
| <a id='UAVCAN_NODE_MODE_MAINTENANCE'></a>2     | [UAVCAN_NODE_MODE_MAINTENANCE](#UAVCAN_NODE_MODE_MAINTENANCE) | 节点正在维护中。                       |
| <a id='UAVCAN_NODE_MODE_SOFTWARE_UPDATE'></a>3 | [UAVCAN_NODE_MODE_SOFTWARE_UPDATE](#UAVCAN_NODE_MODE_SOFTWARE_UPDATE) | 节点正在更新软件。                     |
| <a id='UAVCAN_NODE_MODE_OFFLINE'></a>7         | [UAVCAN_NODE_MODE_OFFLINE](#UAVCAN_NODE_MODE_OFFLINE)        | 该节点已不再在线提供。                 |

### ESC_CONNECTION_TYPE 

显示电调连接类型。

| Value                                     | Name                                                        | Description          |
| ----------------------------------------- | ----------------------------------------------------------- | -------------------- |
| <a id='ESC_CONNECTION_TYPE_PPM'></a>0     | [ESC_CONNECTION_TYPE_PPM](#ESC_CONNECTION_TYPE_PPM)         | 传统的 PPM ESC。     |
| <a id='ESC_CONNECTION_TYPE_SERIAL'></a>1  | [ESC_CONNECTION_TYPE_SERIAL](#ESC_CONNECTION_TYPE_SERIAL)   | 串行总线连接的电调。 |
| <a id='ESC_CONNECTION_TYPE_ONESHOT'></a>2 | [ESC_CONNECTION_TYPE_ONESHOT](#ESC_CONNECTION_TYPE_ONESHOT) | 单发 PPM ESC。       |
| <a id='ESC_CONNECTION_TYPE_I2C'></a>3     | [ESC_CONNECTION_TYPE_I2C](#ESC_CONNECTION_TYPE_I2C)         | I2C 电子调速器。     |
| <a id='ESC_CONNECTION_TYPE_CAN'></a>4     | [ESC_CONNECTION_TYPE_CAN](#ESC_CONNECTION_TYPE_CAN)         | CAN 总线电调。       |
| <a id='ESC_CONNECTION_TYPE_DSHOT'></a>5   | [ESC_CONNECTION_TYPE_DSHOT](#ESC_CONNECTION_TYPE_DSHOT)     | DShot 电子调速器。   |

### ESC_FAILURE_FLAGS 

(位掩码） 报告 ESC 故障的标志。

| Value                                       | Name                                                         | Description          |
| ------------------------------------------- | ------------------------------------------------------------ | -------------------- |
| <a id='ESC_FAILURE_NONE'></a>0              | [ESC_FAILURE_NONE](#ESC_FAILURE_NONE)                        | 无电调故障。         |
| <a id='ESC_FAILURE_OVER_CURRENT'></a>1      | [ESC_FAILURE_OVER_CURRENT](#ESC_FAILURE_OVER_CURRENT)        | 过流故障。           |
| <a id='ESC_FAILURE_OVER_VOLTAGE'></a>2      | [ESC_FAILURE_OVER_VOLTAGE](#ESC_FAILURE_OVER_VOLTAGE)        | 过压故障。           |
| <a id='ESC_FAILURE_OVER_TEMPERATURE'></a>4  | [ESC_FAILURE_OVER_TEMPERATURE](#ESC_FAILURE_OVER_TEMPERATURE) | 温度过高故障。       |
| <a id='ESC_FAILURE_OVER_RPM'></a>8          | [ESC_FAILURE_OVER_RPM](#ESC_FAILURE_OVER_RPM)                | 超转速故障。         |
| <a id='ESC_FAILURE_INCONSISTENT_CMD'></a>16 | [ESC_FAILURE_INCONSISTENT_CMD](#ESC_FAILURE_INCONSISTENT_CMD) | 命令不一致，即越界。 |
| <a id='ESC_FAILURE_MOTOR_STUCK'></a>32      | [ESC_FAILURE_MOTOR_STUCK](#ESC_FAILURE_MOTOR_STUCK)          | 电机卡死故障。       |
| <a id='ESC_FAILURE_GENERIC'></a>64          | [ESC_FAILURE_GENERIC](#ESC_FAILURE_GENERIC)                  | 一般电调故障。       |

### STORAGE_STATUS 

用于指示摄像机存储状态的标记。

| Value                                      | Name                                                         | Description                                                  |
| ------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |
| <a id='STORAGE_STATUS_EMPTY'></a>0         | [STORAGE_STATUS_EMPTY](#STORAGE_STATUS_EMPTY)                | 缺少存储空间（例如未加载 microSD 卡）。                      |
| <a id='STORAGE_STATUS_UNFORMATTED'></a>1   | [STORAGE_STATUS_UNFORMATTED](#STORAGE_STATUS_UNFORMATTED)    | 存储空间存在，但未格式化。                                   |
| <a id='STORAGE_STATUS_READY'></a>2         | [STORAGE_STATUS_READY](#STORAGE_STATUS_READY)                | 储藏室已准备就绪。                                           |
| <a id='STORAGE_STATUS_NOT_SUPPORTED'></a>3 | [STORAGE_STATUS_NOT_SUPPORTED](#STORAGE_STATUS_NOT_SUPPORTED) | 摄像机不提供存储状态信息。STORAGE_INFORMATION](#STORAGE_INFORMATION) 字段中的容量信息将被忽略。 |

### STORAGE_TYPE 

表示存储类型的标志。

| Value                                | Name                                              | Description                    |
| ------------------------------------ | ------------------------------------------------- | ------------------------------ |
| <a id='STORAGE_TYPE_UNKNOWN'></a>0   | [STORAGE_TYPE_UNKNOWN](#STORAGE_TYPE_UNKNOWN)     | 存储类型未知。                 |
| <a id='STORAGE_TYPE_USB_STICK'></a>1 | [STORAGE_TYPE_USB_STICK](#STORAGE_TYPE_USB_STICK) | 存储类型为 USB 设备。          |
| <a id='STORAGE_TYPE_SD'></a>2        | [STORAGE_TYPE_SD](#STORAGE_TYPE_SD)               | 存储类型为 SD 卡。             |
| <a id='STORAGE_TYPE_MICROSD'></a>3   | [STORAGE_TYPE_MICROSD](#STORAGE_TYPE_MICROSD)     | 存储类型为微型 SD 卡。         |
| <a id='STORAGE_TYPE_CF'></a>4        | [STORAGE_TYPE_CF](#STORAGE_TYPE_CF)               | 存储类型为 CFast。             |
| <a id='STORAGE_TYPE_CFE'></a>5       | [STORAGE_TYPE_CFE](#STORAGE_TYPE_CFE)             | 存储类型为 CFexpress。         |
| <a id='STORAGE_TYPE_XQD'></a>6       | [STORAGE_TYPE_XQD](#STORAGE_TYPE_XQD)             | 存储类型为 XQD。               |
| <a id='STORAGE_TYPE_HD'></a>7        | [STORAGE_TYPE_HD](#STORAGE_TYPE_HD)               | 存储类型是 HD 大容量存储类型。 |
| <a id='STORAGE_TYPE_OTHER'></a>254   | [STORAGE_TYPE_OTHER](#STORAGE_TYPE_OTHER)         | 存储类型为其他，未列出。       |

### STORAGE_USAGE_FLAG 

用于指示特定存储空间使用情况的标志（请参阅 [STORAGE_INFORMATION](#STORAGE_INFORMATION).storage_usage 和 [MAV_CMD_SET_STORAGE_USAGE](#MAV_CMD_SET_STORAGE_USAGE)）。

| Value                                  | Name                                                  | Description                                                  |
| -------------------------------------- | ----------------------------------------------------- | ------------------------------------------------------------ |
| <a id='STORAGE_USAGE_FLAG_SET'></a>1   | [STORAGE_USAGE_FLAG_SET](#STORAGE_USAGE_FLAG_SET)     | 始终设为 1（表示支持 [STORAGE_INFORMATION](#STORAGE_INFORMATION).storage_usage）。 |
| <a id='STORAGE_USAGE_FLAG_PHOTO'></a>2 | [STORAGE_USAGE_FLAG_PHOTO](#STORAGE_USAGE_FLAG_PHOTO) | 用于保存照片的存储器。                                       |
| <a id='STORAGE_USAGE_FLAG_VIDEO'></a>4 | [STORAGE_USAGE_FLAG_VIDEO](#STORAGE_USAGE_FLAG_VIDEO) | 用于保存视频的存储器。                                       |
| <a id='STORAGE_USAGE_FLAG_LOGS'></a>8  | [STORAGE_USAGE_FLAG_LOGS](#STORAGE_USAGE_FLAG_LOGS)   | 用于保存日志的存储器。                                       |

### ORBIT_YAW_BEHAVIOUR 

轨道飞行期间的偏航行为

| Value                                                        | Name                                                         | Description                          |
| ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------ |
| <a id='ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TO_CIRCLE_CENTER'></a>0 | [ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TO_CIRCLE_CENTER](#ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TO_CIRCLE_CENTER) | 车辆前部指向中央（默认）。           |
| <a id='ORBIT_YAW_BEHAVIOUR_HOLD_INITIAL_HEADING'></a>1       | [ORBIT_YAW_BEHAVIOUR_HOLD_INITIAL_HEADING](#ORBIT_YAW_BEHAVIOUR_HOLD_INITIAL_HEADING) | 收到信息时，车辆前部保持航向。       |
| <a id='ORBIT_YAW_BEHAVIOUR_UNCONTROLLED'></a>2               | [ORBIT_YAW_BEHAVIOUR_UNCONTROLLED](#ORBIT_YAW_BEHAVIOUR_UNCONTROLLED) | 偏航失控                             |
| <a id='ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TANGENT_TO_CIRCLE'></a>3 | [ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TANGENT_TO_CIRCLE](#ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TANGENT_TO_CIRCLE) | 车辆前部沿飞行路径飞行（与圆相切）。 |
| <a id='ORBIT_YAW_BEHAVIOUR_RC_CONTROLLED'></a>4              | [ORBIT_YAW_BEHAVIOUR_RC_CONTROLLED](#ORBIT_YAW_BEHAVIOUR_RC_CONTROLLED) | 偏航由 RC 输入控制。                 |

### WIFI_CONFIG_AP_RESPONSE 

可能来自 [WIFI_CONFIG_AP](#WIFI_CONFIG_AP) 的回复信息。

| Value                                                | Name                                                         | Description                            |
| ---------------------------------------------------- | ------------------------------------------------------------ | -------------------------------------- |
| <a id='WIFI_CONFIG_AP_RESPONSE_UNDEFINED'></a>0      | [WIFI_CONFIG_AP_RESPONSE_UNDEFINED](#WIFI_CONFIG_AP_RESPONSE_UNDEFINED) | 未定义响应。可能表明系统不支持此请求。 |
| <a id='WIFI_CONFIG_AP_RESPONSE_ACCEPTED'></a>1       | [WIFI_CONFIG_AP_RESPONSE_ACCEPTED](#WIFI_CONFIG_AP_RESPONSE_ACCEPTED) | 接受更改。                             |
| <a id='WIFI_CONFIG_AP_RESPONSE_REJECTED'></a>2       | [WIFI_CONFIG_AP_RESPONSE_REJECTED](#WIFI_CONFIG_AP_RESPONSE_REJECTED) | 拒绝更改。                             |
| <a id='WIFI_CONFIG_AP_RESPONSE_MODE_ERROR'></a>3     | [WIFI_CONFIG_AP_RESPONSE_MODE_ERROR](#WIFI_CONFIG_AP_RESPONSE_MODE_ERROR) | 无效模式。                             |
| <a id='WIFI_CONFIG_AP_RESPONSE_SSID_ERROR'></a>4     | [WIFI_CONFIG_AP_RESPONSE_SSID_ERROR](#WIFI_CONFIG_AP_RESPONSE_SSID_ERROR) | SSID 无效。                            |
| <a id='WIFI_CONFIG_AP_RESPONSE_PASSWORD_ERROR'></a>5 | [WIFI_CONFIG_AP_RESPONSE_PASSWORD_ERROR](#WIFI_CONFIG_AP_RESPONSE_PASSWORD_ERROR) | 密码无效。                             |

### CELLULAR_CONFIG_RESPONSE 

可能来自  [CELLULAR_CONFIG](#CELLULAR_CONFIG) 的回复信息。

| Value                                              | Name                                                         | Description           |
| -------------------------------------------------- | ------------------------------------------------------------ | --------------------- |
| <a id='CELLULAR_CONFIG_RESPONSE_ACCEPTED'></a>0    | [CELLULAR_CONFIG_RESPONSE_ACCEPTED](#CELLULAR_CONFIG_RESPONSE_ACCEPTED) | 接受更改。            |
| <a id='CELLULAR_CONFIG_RESPONSE_APN_ERROR'></a>1   | [CELLULAR_CONFIG_RESPONSE_APN_ERROR](#CELLULAR_CONFIG_RESPONSE_APN_ERROR) | 无效 APN。            |
| <a id='CELLULAR_CONFIG_RESPONSE_PIN_ERROR'></a>2   | [CELLULAR_CONFIG_RESPONSE_PIN_ERROR](#CELLULAR_CONFIG_RESPONSE_PIN_ERROR) | 无效 PIN 码。         |
| <a id='CELLULAR_CONFIG_RESPONSE_REJECTED'></a>3    | [CELLULAR_CONFIG_RESPONSE_REJECTED](#CELLULAR_CONFIG_RESPONSE_REJECTED) | 拒绝更改。            |
| <a id='CELLULAR_CONFIG_BLOCKED_PUK_REQUIRED'></a>4 | [CELLULAR_CONFIG_BLOCKED_PUK_REQUIRED](#CELLULAR_CONFIG_BLOCKED_PUK_REQUIRED) | 解锁 SIM 卡需要 PUK。 |

### WIFI_CONFIG_AP_MODE 

WiFi 模式。

| Value                                       | Name                                                         | Description                               |
| ------------------------------------------- | ------------------------------------------------------------ | ----------------------------------------- |
| <a id='WIFI_CONFIG_AP_MODE_UNDEFINED'></a>0 | [WIFI_CONFIG_AP_MODE_UNDEFINED](#WIFI_CONFIG_AP_MODE_UNDEFINED) | 未定义 WiFi 模式。                        |
| <a id='WIFI_CONFIG_AP_MODE_AP'></a>1        | [WIFI_CONFIG_AP_MODE_AP](#WIFI_CONFIG_AP_MODE_AP)            | WiFi 被配置为接入点。                     |
| <a id='WIFI_CONFIG_AP_MODE_STATION'></a>2   | [WIFI_CONFIG_AP_MODE_STATION](#WIFI_CONFIG_AP_MODE_STATION)  | WiFi 配置为连接到现有本地 WiFi 网络的站。 |
| <a id='WIFI_CONFIG_AP_MODE_DISABLED'></a>3  | [WIFI_CONFIG_AP_MODE_DISABLED](#WIFI_CONFIG_AP_MODE_DISABLED) | 禁用 WiFi。                               |

### COMP_METADATA_TYPE 

支持的组件元数据类型。这些元数据用于 [COMPONENT_METADATA](#COMPONENT_METADATA)返回的 "常规 "元数据文件，以提供有关支持的元数据类型的信息。这些类型不会直接用于 MAVLink 信息。

| Value                                        | Name                                                         | Description                                                  |
| -------------------------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| <a id='COMP_METADATA_TYPE_GENERAL'></a>0     | [COMP_METADATA_TYPE_GENERAL](#COMP_METADATA_TYPE_GENERAL)    | 组件的一般信息。一般元数据包括组件支持的其他元数据类型的信息。必须支持此类型的文件，且必须可使用 MAVLink FTP URI 从车辆上下载。 |
| <a id='COMP_METADATA_TYPE_PARAMETER'></a>1   | [COMP_METADATA_TYPE_PARAMETER](#COMP_METADATA_TYPE_PARAMETER) | 参数元数据。                                                 |
| <a id='COMP_METADATA_TYPE_COMMANDS'></a>2    | [COMP_METADATA_TYPE_COMMANDS](#COMP_METADATA_TYPE_COMMANDS)  | 指定车辆支持哪些命令和命令参数的元数据。（WIP）              |
| <a id='COMP_METADATA_TYPE_PERIPHERALS'></a>3 | [COMP_METADATA_TYPE_PERIPHERALS](#COMP_METADATA_TYPE_PERIPHERALS) | 指定外部非 MAVLink 外围设备的元数据。                        |
| <a id='COMP_METADATA_TYPE_EVENTS'></a>4      | [COMP_METADATA_TYPE_EVENTS](#COMP_METADATA_TYPE_EVENTS)      | 事件接口的元数据。                                           |
| <a id='COMP_METADATA_TYPE_ACTUATORS'></a>5   | [COMP_METADATA_TYPE_ACTUATORS](#COMP_METADATA_TYPE_ACTUATORS) | 执行器配置（电机、伺服和车辆几何形状）和测试的元数据。       |

### ACTUATOR_CONFIGURATION 

执行器配置，用于更改执行器的设置。组件信息元数据可用于了解哪些输出支持哪些命令。

| Value                                                | Name                                                         | Description                                                  |
| ---------------------------------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| <a id='ACTUATOR_CONFIGURATION_NONE'></a>0            | [ACTUATOR_CONFIGURATION_NONE](#ACTUATOR_CONFIGURATION_NONE)  | 什么也别做                                                   |
| <a id='ACTUATOR_CONFIGURATION_BEEP'></a>1            | [ACTUATOR_CONFIGURATION_BEEP](#ACTUATOR_CONFIGURATION_BEEP)  | 命令执行机构发出蜂鸣声。                                     |
| <a id='ACTUATOR_CONFIGURATION_3D_MODE_ON'></a>2      | [ACTUATOR_CONFIGURATION_3D_MODE_ON](#ACTUATOR_CONFIGURATION_3D_MODE_ON) | 将推杆（电调）永久设置为 3D 模式（可逆推力）。               |
| <a id='ACTUATOR_CONFIGURATION_3D_MODE_OFF'></a>3     | [ACTUATOR_CONFIGURATION_3D_MODE_OFF](#ACTUATOR_CONFIGURATION_3D_MODE_OFF) | 将推杆（ESC）永久设置为非 3D 模式（不可逆推力）。            |
| <a id='ACTUATOR_CONFIGURATION_SPIN_DIRECTION1'></a>4 | [ACTUATOR_CONFIGURATION_SPIN_DIRECTION1](#ACTUATOR_CONFIGURATION_SPIN_DIRECTION1) | 将执行机构（ESC）永久设置为旋转方向 1（可以是顺时针或逆时针）。 |
| <a id='ACTUATOR_CONFIGURATION_SPIN_DIRECTION2'></a>5 | [ACTUATOR_CONFIGURATION_SPIN_DIRECTION2](#ACTUATOR_CONFIGURATION_SPIN_DIRECTION2) | 将执行机构（ESC）永久设置为旋转方向 2（与方向 1 相反）。     |

### ACTUATOR_OUTPUT_FUNCTION 

执行器输出功能。大于或等于 1000 的值为自动驾驶仪专用值。

| Value                                           | Name                                                         | Description        |
| ----------------------------------------------- | ------------------------------------------------------------ | ------------------ |
| <a id='ACTUATOR_OUTPUT_FUNCTION_NONE'></a>0     | [ACTUATOR_OUTPUT_FUNCTION_NONE](#ACTUATOR_OUTPUT_FUNCTION_NONE) | 无功能（已禁用）。 |
| <a id='ACTUATOR_OUTPUT_FUNCTION_MOTOR1'></a>1   | [ACTUATOR_OUTPUT_FUNCTION_MOTOR1](#ACTUATOR_OUTPUT_FUNCTION_MOTOR1) | 电机 1             |
| <a id='ACTUATOR_OUTPUT_FUNCTION_MOTOR2'></a>2   | [ACTUATOR_OUTPUT_FUNCTION_MOTOR2](#ACTUATOR_OUTPUT_FUNCTION_MOTOR2) | Motor 2            |
| <a id='ACTUATOR_OUTPUT_FUNCTION_MOTOR3'></a>3   | [ACTUATOR_OUTPUT_FUNCTION_MOTOR3](#ACTUATOR_OUTPUT_FUNCTION_MOTOR3) | Motor 3            |
| <a id='ACTUATOR_OUTPUT_FUNCTION_MOTOR4'></a>4   | [ACTUATOR_OUTPUT_FUNCTION_MOTOR4](#ACTUATOR_OUTPUT_FUNCTION_MOTOR4) | Motor 4            |
| <a id='ACTUATOR_OUTPUT_FUNCTION_MOTOR5'></a>5   | [ACTUATOR_OUTPUT_FUNCTION_MOTOR5](#ACTUATOR_OUTPUT_FUNCTION_MOTOR5) | Motor 5            |
| <a id='ACTUATOR_OUTPUT_FUNCTION_MOTOR6'></a>6   | [ACTUATOR_OUTPUT_FUNCTION_MOTOR6](#ACTUATOR_OUTPUT_FUNCTION_MOTOR6) | Motor 6            |
| <a id='ACTUATOR_OUTPUT_FUNCTION_MOTOR7'></a>7   | [ACTUATOR_OUTPUT_FUNCTION_MOTOR7](#ACTUATOR_OUTPUT_FUNCTION_MOTOR7) | Motor 7            |
| <a id='ACTUATOR_OUTPUT_FUNCTION_MOTOR8'></a>8   | [ACTUATOR_OUTPUT_FUNCTION_MOTOR8](#ACTUATOR_OUTPUT_FUNCTION_MOTOR8) | Motor 8            |
| <a id='ACTUATOR_OUTPUT_FUNCTION_MOTOR9'></a>9   | [ACTUATOR_OUTPUT_FUNCTION_MOTOR9](#ACTUATOR_OUTPUT_FUNCTION_MOTOR9) | Motor 9            |
| <a id='ACTUATOR_OUTPUT_FUNCTION_MOTOR10'></a>10 | [ACTUATOR_OUTPUT_FUNCTION_MOTOR10](#ACTUATOR_OUTPUT_FUNCTION_MOTOR10) | Motor 10           |
| <a id='ACTUATOR_OUTPUT_FUNCTION_MOTOR11'></a>11 | [ACTUATOR_OUTPUT_FUNCTION_MOTOR11](#ACTUATOR_OUTPUT_FUNCTION_MOTOR11) | Motor 11           |
| <a id='ACTUATOR_OUTPUT_FUNCTION_MOTOR12'></a>12 | [ACTUATOR_OUTPUT_FUNCTION_MOTOR12](#ACTUATOR_OUTPUT_FUNCTION_MOTOR12) | Motor 12           |
| <a id='ACTUATOR_OUTPUT_FUNCTION_MOTOR13'></a>13 | [ACTUATOR_OUTPUT_FUNCTION_MOTOR13](#ACTUATOR_OUTPUT_FUNCTION_MOTOR13) | Motor 13           |
| <a id='ACTUATOR_OUTPUT_FUNCTION_MOTOR14'></a>14 | [ACTUATOR_OUTPUT_FUNCTION_MOTOR14](#ACTUATOR_OUTPUT_FUNCTION_MOTOR14) | Motor 14           |
| <a id='ACTUATOR_OUTPUT_FUNCTION_MOTOR15'></a>15 | [ACTUATOR_OUTPUT_FUNCTION_MOTOR15](#ACTUATOR_OUTPUT_FUNCTION_MOTOR15) | Motor 15           |
| <a id='ACTUATOR_OUTPUT_FUNCTION_MOTOR16'></a>16 | [ACTUATOR_OUTPUT_FUNCTION_MOTOR16](#ACTUATOR_OUTPUT_FUNCTION_MOTOR16) | Motor 16           |
| <a id='ACTUATOR_OUTPUT_FUNCTION_SERVO1'></a>33  | [ACTUATOR_OUTPUT_FUNCTION_SERVO1](#ACTUATOR_OUTPUT_FUNCTION_SERVO1) | 伺服 1             |
| <a id='ACTUATOR_OUTPUT_FUNCTION_SERVO2'></a>34  | [ACTUATOR_OUTPUT_FUNCTION_SERVO2](#ACTUATOR_OUTPUT_FUNCTION_SERVO2) | Servo 2            |
| <a id='ACTUATOR_OUTPUT_FUNCTION_SERVO3'></a>35  | [ACTUATOR_OUTPUT_FUNCTION_SERVO3](#ACTUATOR_OUTPUT_FUNCTION_SERVO3) | Servo 3            |
| <a id='ACTUATOR_OUTPUT_FUNCTION_SERVO4'></a>36  | [ACTUATOR_OUTPUT_FUNCTION_SERVO4](#ACTUATOR_OUTPUT_FUNCTION_SERVO4) | Servo 4            |
| <a id='ACTUATOR_OUTPUT_FUNCTION_SERVO5'></a>37  | [ACTUATOR_OUTPUT_FUNCTION_SERVO5](#ACTUATOR_OUTPUT_FUNCTION_SERVO5) | Servo 5            |
| <a id='ACTUATOR_OUTPUT_FUNCTION_SERVO6'></a>38  | [ACTUATOR_OUTPUT_FUNCTION_SERVO6](#ACTUATOR_OUTPUT_FUNCTION_SERVO6) | Servo 6            |
| <a id='ACTUATOR_OUTPUT_FUNCTION_SERVO7'></a>39  | [ACTUATOR_OUTPUT_FUNCTION_SERVO7](#ACTUATOR_OUTPUT_FUNCTION_SERVO7) | Servo 7            |
| <a id='ACTUATOR_OUTPUT_FUNCTION_SERVO8'></a>40  | [ACTUATOR_OUTPUT_FUNCTION_SERVO8](#ACTUATOR_OUTPUT_FUNCTION_SERVO8) | Servo 8            |
| <a id='ACTUATOR_OUTPUT_FUNCTION_SERVO9'></a>41  | [ACTUATOR_OUTPUT_FUNCTION_SERVO9](#ACTUATOR_OUTPUT_FUNCTION_SERVO9) | Servo 9            |
| <a id='ACTUATOR_OUTPUT_FUNCTION_SERVO10'></a>42 | [ACTUATOR_OUTPUT_FUNCTION_SERVO10](#ACTUATOR_OUTPUT_FUNCTION_SERVO10) | Servo 10           |
| <a id='ACTUATOR_OUTPUT_FUNCTION_SERVO11'></a>43 | [ACTUATOR_OUTPUT_FUNCTION_SERVO11](#ACTUATOR_OUTPUT_FUNCTION_SERVO11) | Servo 11           |
| <a id='ACTUATOR_OUTPUT_FUNCTION_SERVO12'></a>44 | [ACTUATOR_OUTPUT_FUNCTION_SERVO12](#ACTUATOR_OUTPUT_FUNCTION_SERVO12) | Servo 12           |
| <a id='ACTUATOR_OUTPUT_FUNCTION_SERVO13'></a>45 | [ACTUATOR_OUTPUT_FUNCTION_SERVO13](#ACTUATOR_OUTPUT_FUNCTION_SERVO13) | Servo 13           |
| <a id='ACTUATOR_OUTPUT_FUNCTION_SERVO14'></a>46 | [ACTUATOR_OUTPUT_FUNCTION_SERVO14](#ACTUATOR_OUTPUT_FUNCTION_SERVO14) | Servo 14           |
| <a id='ACTUATOR_OUTPUT_FUNCTION_SERVO15'></a>47 | [ACTUATOR_OUTPUT_FUNCTION_SERVO15](#ACTUATOR_OUTPUT_FUNCTION_SERVO15) | Servo 15           |
| <a id='ACTUATOR_OUTPUT_FUNCTION_SERVO16'></a>48 | [ACTUATOR_OUTPUT_FUNCTION_SERVO16](#ACTUATOR_OUTPUT_FUNCTION_SERVO16) | Servo 16           |

### AUTOTUNE_AXIS 

(位屏蔽）启用将通过自动调整功能进行调整的轴。在 [MAV_CMD_DO_AUTOTUNE_ENABLE]（#MAV_CMD_DO_AUTOTUNE_ENABLE）中使用。

| Value                               | Name                                            | Description                        |
| ----------------------------------- | ----------------------------------------------- | ---------------------------------- |
| <a id='AUTOTUNE_AXIS_DEFAULT'></a>0 | [AUTOTUNE_AXIS_DEFAULT](#AUTOTUNE_AXIS_DEFAULT) | 飞行堆栈根据默认设置对轴进行调整。 |
| <a id='AUTOTUNE_AXIS_ROLL'></a>1    | [AUTOTUNE_AXIS_ROLL](#AUTOTUNE_AXIS_ROLL)       | 自动调整滚动轴                     |
| <a id='AUTOTUNE_AXIS_PITCH'></a>2   | [AUTOTUNE_AXIS_PITCH](#AUTOTUNE_AXIS_PITCH)     | 自动调整俯仰轴                     |
| <a id='AUTOTUNE_AXIS_YAW'></a>4     | [AUTOTUNE_AXIS_YAW](#AUTOTUNE_AXIS_YAW)         | 自动调整偏航轴                     |

### PREFLIGHT_STORAGE_PARAMETER_ACTION 

使用 [MAV_CMD_PREFLIGHT_STORAGE](#MAV_CMD_PREFLIGHT_STORAGE)时在持久存储和易失性存储之间读/写参数的操作。
(通常在启动时将参数从持久存储器（闪存/EEPROM）载入易失性存储器（RAM），并在参数发生变化时将其写回）。

| Value                                    | Name                                                      | Description                                                  |
| ---------------------------------------- | --------------------------------------------------------- | ------------------------------------------------------------ |
| <a id='PARAM_READ_PERSISTENT'></a>0      | [PARAM_READ_PERSISTENT](#PARAM_READ_PERSISTENT)           | 从持久存储中读取所有参数。替换易失性存储中的值。             |
| <a id='PARAM_WRITE_PERSISTENT'></a>1     | [PARAM_WRITE_PERSISTENT](#PARAM_WRITE_PERSISTENT)         | 将所有参数值写入持久存储器（闪存/EEPROM）                    |
| <a id='PARAM_RESET_CONFIG_DEFAULT'></a>2 | [PARAM_RESET_CONFIG_DEFAULT](#PARAM_RESET_CONFIG_DEFAULT) | 将所有用户可配置参数重置为默认值（包括机身选择、传感器校准数据、安全设置等）。不会重置包含操作计数器和飞行器计算统计数据的值。 |
| <a id='PARAM_RESET_SENSOR_DEFAULT'></a>3 | [PARAM_RESET_SENSOR_DEFAULT](#PARAM_RESET_SENSOR_DEFAULT) | 仅将传感器校准参数重置为出厂默认设置（或固件默认设置，如果不可用） |
| <a id='PARAM_RESET_ALL_DEFAULT'></a>4    | [PARAM_RESET_ALL_DEFAULT](#PARAM_RESET_ALL_DEFAULT)       | 将所有参数（包括操作计数器）重置为默认值                     |

### PREFLIGHT_STORAGE_MISSION_ACTION 

使用[MAV_CMD_PREFLIGHT_STORAGE](#MAV_CMD_PREFLIGHT_STORAGE)时，在持久存储和易失性存储之间读写计划信息（任务、集结点、地理围栏）的操作。
(通常任务会在启动时从持久存储（闪存/EEPROM）加载到易失性存储（RAM）中，并在任务更改时写回）。

| Value                                  | Name                                                  | Description                                                |
| -------------------------------------- | ----------------------------------------------------- | ---------------------------------------------------------- |
| <a id='MISSION_READ_PERSISTENT'></a>0  | [MISSION_READ_PERSISTENT](#MISSION_READ_PERSISTENT)   | 从持久存储中读取当前任务数据                               |
| <a id='MISSION_WRITE_PERSISTENT'></a>1 | [MISSION_WRITE_PERSISTENT](#MISSION_WRITE_PERSISTENT) | 将当前任务数据写入持久存储                                 |
| <a id='MISSION_RESET_DEFAULT'></a>2    | [MISSION_RESET_DEFAULT](#MISSION_RESET_DEFAULT)       | 清除飞行器上存储的所有任务数据（包括持久存储和易失性存储） |

### MAV_DATA_STREAM — [DEP] 

<span class="warning">**DEPRECATED:** Replaced By [MESSAGE_INTERVAL](#MESSAGE_INTERVAL) (2015-06)</span>

数据流不是一组固定的信息，而是

对自动驾驶软件的建议。个别自动驾驶仪可能会也可能不会遵守
建议的信息。

| Value                                         | Name                                                         | Description                                                  |
| --------------------------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| <a id='MAV_DATA_STREAM_ALL'></a>0             | [MAV_DATA_STREAM_ALL](#MAV_DATA_STREAM_ALL)                  | 启用所有数据流                                               |
| <a id='MAV_DATA_STREAM_RAW_SENSORS'></a>1     | [MAV_DATA_STREAM_RAW_SENSORS](#MAV_DATA_STREAM_RAW_SENSORS)  | 启用 [IMU_RAW](#IMU_RAW), [GPS_RAW](#GPS_RAW), [GPS_STATUS](#GPS_STATUS) 数据包. |
| <a id='MAV_DATA_STREAM_EXTENDED_STATUS'></a>2 | [MAV_DATA_STREAM_EXTENDED_STATUS](#MAV_DATA_STREAM_EXTENDED_STATUS) | Enable [GPS_STATUS](#GPS_STATUS), [CONTROL_STATUS](#CONTROL_STATUS), [AUX_STATUS](#AUX_STATUS) |
| <a id='MAV_DATA_STREAM_RC_CHANNELS'></a>3     | [MAV_DATA_STREAM_RC_CHANNELS](#MAV_DATA_STREAM_RC_CHANNELS)  | Enable [RC_CHANNELS_SCALED](#RC_CHANNELS_SCALED), [RC_CHANNELS_RAW](#RC_CHANNELS_RAW), [SERVO_OUTPUT_RAW](#SERVO_OUTPUT_RAW) |
| <a id='MAV_DATA_STREAM_RAW_CONTROLLER'></a>4  | [MAV_DATA_STREAM_RAW_CONTROLLER](#MAV_DATA_STREAM_RAW_CONTROLLER) | Enable [ATTITUDE_CONTROLLER_OUTPUT](#ATTITUDE_CONTROLLER_OUTPUT), [POSITION_CONTROLLER_OUTPUT](#POSITION_CONTROLLER_OUTPUT), [NAV_CONTROLLER_OUTPUT](#NAV_CONTROLLER_OUTPUT). |
| <a id='MAV_DATA_STREAM_POSITION'></a>6        | [MAV_DATA_STREAM_POSITION](#MAV_DATA_STREAM_POSITION)        | Enable [LOCAL_POSITION](#LOCAL_POSITION), [GLOBAL_POSITION_INT](#GLOBAL_POSITION_INT) messages. |
| <a id='MAV_DATA_STREAM_EXTRA1'></a>10         | [MAV_DATA_STREAM_EXTRA1](#MAV_DATA_STREAM_EXTRA1)            | 视自动驾驶仪而定                                             |
| <a id='MAV_DATA_STREAM_EXTRA2'></a>11         | [MAV_DATA_STREAM_EXTRA2](#MAV_DATA_STREAM_EXTRA2)            | 视自动驾驶仪而定                                             |
| <a id='MAV_DATA_STREAM_EXTRA3'></a>12         | [MAV_DATA_STREAM_EXTRA3](#MAV_DATA_STREAM_EXTRA3)            | 视自动驾驶仪而定                                             |

### MAV_ROI — [DEP] 

<span class="warning">**DEPRECATED:** Replaced By MAV_CMD_DO_SET_ROI_* (2018-01)</span>

车辆的 ROI（感兴趣区域）。这可以

可被飞行器用于摄像机/飞行器姿态校准（见
[mav_cmd_nav_roi](#mav_cmd_nav_roi))。

| Value                          | Name                                  | Description                                |
| ------------------------------ | ------------------------------------- | ------------------------------------------ |
| <a id='MAV_ROI_NONE'></a>0     | [MAV_ROI_NONE](#MAV_ROI_NONE)         | 无相关区域。                               |
| <a id='MAV_ROI_WPNEXT'></a>1   | [MAV_ROI_WPNEXT](#MAV_ROI_WPNEXT)     | 指向下一个航点，可选择俯仰/滚动/偏航偏移。 |
| <a id='MAV_ROI_WPINDEX'></a>2  | [MAV_ROI_WPINDEX](#MAV_ROI_WPINDEX)   | 指向给定的航点。                           |
| <a id='MAV_ROI_LOCATION'></a>3 | [MAV_ROI_LOCATION](#MAV_ROI_LOCATION) | 指向固定位置。                             |
| <a id='MAV_ROI_TARGET'></a>4   | [MAV_ROI_TARGET](#MAV_ROI_TARGET)     | 指向给定 id。                              |

### MAV_PARAM_TYPE 

指定 MAVLink 参数的数据类型。

| Value                                | Name                                            | Description     |
| ------------------------------------ | ----------------------------------------------- | --------------- |
| <a id='MAV_PARAM_TYPE_UINT8'></a>1   | [MAV_PARAM_TYPE_UINT8](#MAV_PARAM_TYPE_UINT8)   | 8 位无符号整数  |
| <a id='MAV_PARAM_TYPE_INT8'></a>2    | [MAV_PARAM_TYPE_INT8](#MAV_PARAM_TYPE_INT8)     | 8 位有符号整数  |
| <a id='MAV_PARAM_TYPE_UINT16'></a>3  | [MAV_PARAM_TYPE_UINT16](#MAV_PARAM_TYPE_UINT16) | 16 位无符号整数 |
| <a id='MAV_PARAM_TYPE_INT16'></a>4   | [MAV_PARAM_TYPE_INT16](#MAV_PARAM_TYPE_INT16)   | 16 位有符号整数 |
| <a id='MAV_PARAM_TYPE_UINT32'></a>5  | [MAV_PARAM_TYPE_UINT32](#MAV_PARAM_TYPE_UINT32) | 32 位无符号整数 |
| <a id='MAV_PARAM_TYPE_INT32'></a>6   | [MAV_PARAM_TYPE_INT32](#MAV_PARAM_TYPE_INT32)   | 32 位有符号整数 |
| <a id='MAV_PARAM_TYPE_UINT64'></a>7  | [MAV_PARAM_TYPE_UINT64](#MAV_PARAM_TYPE_UINT64) | 64 位无符号整数 |
| <a id='MAV_PARAM_TYPE_INT64'></a>8   | [MAV_PARAM_TYPE_INT64](#MAV_PARAM_TYPE_INT64)   | 64 位有符号整数 |
| <a id='MAV_PARAM_TYPE_REAL32'></a>9  | [MAV_PARAM_TYPE_REAL32](#MAV_PARAM_TYPE_REAL32) | 32 位浮点运算   |
| <a id='MAV_PARAM_TYPE_REAL64'></a>10 | [MAV_PARAM_TYPE_REAL64](#MAV_PARAM_TYPE_REAL64) | 64 位浮点运算   |

### MAV_PARAM_EXT_TYPE 

指定 MAVLink 扩展参数的数据类型。

| Value                                    | Name                                                    | Description             |
| ---------------------------------------- | ------------------------------------------------------- | ----------------------- |
| <a id='MAV_PARAM_EXT_TYPE_UINT8'></a>1   | [MAV_PARAM_EXT_TYPE_UINT8](#MAV_PARAM_EXT_TYPE_UINT8)   | 8 位无符号整数          |
| <a id='MAV_PARAM_EXT_TYPE_INT8'></a>2    | [MAV_PARAM_EXT_TYPE_INT8](#MAV_PARAM_EXT_TYPE_INT8)     | 8 位有符号整数          |
| <a id='MAV_PARAM_EXT_TYPE_UINT16'></a>3  | [MAV_PARAM_EXT_TYPE_UINT16](#MAV_PARAM_EXT_TYPE_UINT16) | 16-bit unsigned integer |
| <a id='MAV_PARAM_EXT_TYPE_INT16'></a>4   | [MAV_PARAM_EXT_TYPE_INT16](#MAV_PARAM_EXT_TYPE_INT16)   | 16-bit signed integer   |
| <a id='MAV_PARAM_EXT_TYPE_UINT32'></a>5  | [MAV_PARAM_EXT_TYPE_UINT32](#MAV_PARAM_EXT_TYPE_UINT32) | 32-bit unsigned integer |
| <a id='MAV_PARAM_EXT_TYPE_INT32'></a>6   | [MAV_PARAM_EXT_TYPE_INT32](#MAV_PARAM_EXT_TYPE_INT32)   | 32-bit signed integer   |
| <a id='MAV_PARAM_EXT_TYPE_UINT64'></a>7  | [MAV_PARAM_EXT_TYPE_UINT64](#MAV_PARAM_EXT_TYPE_UINT64) | 64-bit unsigned integer |
| <a id='MAV_PARAM_EXT_TYPE_INT64'></a>8   | [MAV_PARAM_EXT_TYPE_INT64](#MAV_PARAM_EXT_TYPE_INT64)   | 64-bit signed integer   |
| <a id='MAV_PARAM_EXT_TYPE_REAL32'></a>9  | [MAV_PARAM_EXT_TYPE_REAL32](#MAV_PARAM_EXT_TYPE_REAL32) | 32-bit floating-point   |
| <a id='MAV_PARAM_EXT_TYPE_REAL64'></a>10 | [MAV_PARAM_EXT_TYPE_REAL64](#MAV_PARAM_EXT_TYPE_REAL64) | 64-bit floating-point   |
| <a id='MAV_PARAM_EXT_TYPE_CUSTOM'></a>11 | [MAV_PARAM_EXT_TYPE_CUSTOM](#MAV_PARAM_EXT_TYPE_CUSTOM) | 自定义类型              |

### MAV_RESULT 

MAVLink 命令的结果 ([MAV_CMD](#mav_commands))

| Value                                                  | Name                                                         | Description                                                  |
| ------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |
| <a id='MAV_RESULT_ACCEPTED'></a>0                      | [MAV_RESULT_ACCEPTED](#MAV_RESULT_ACCEPTED)                  | 命令有效（受支持且参数有效），并已执行。                     |
| <a id='MAV_RESULT_TEMPORARILY_REJECTED'></a>1          | [MAV_RESULT_TEMPORARILY_REJECTED](#MAV_RESULT_TEMPORARILY_REJECTED) | 命令有效，但此时无法执行。该命令用于表示只需等待即可解决的问题（如状态机繁忙、因未锁定 GPS 而无法布防等）。稍后重试应该可以解决。 |
| <a id='MAV_RESULT_DENIED'></a>2                        | [MAV_RESULT_DENIED](#MAV_RESULT_DENIED)                      | 命令无效（支持但参数无效）。重试相同的命令和参数将不起作用。 |
| <a id='MAV_RESULT_UNSUPPORTED'></a>3                   | [MAV_RESULT_UNSUPPORTED](#MAV_RESULT_UNSUPPORTED)            | 不支持该命令（未知）。                                       |
| <a id='MAV_RESULT_FAILED'></a>4                        | [MAV_RESULT_FAILED](#MAV_RESULT_FAILED)                      | 命令有效，但执行失败。用于表示任何非临时或意外问题，即在命令成功/重试之前必须解决的问题。例如，内存不足时试图写入文件，传感器未校准时试图布防等。 |
| <a id='MAV_RESULT_IN_PROGRESS'></a>5                   | [MAV_RESULT_IN_PROGRESS](#MAV_RESULT_IN_PROGRESS)            | 命令有效并正在执行。随后会有进一步的进度更新，即组件可能会发送更多带有结果 [MAV_RESULT_IN_PROGRESS](#MAV_RESULT_IN_PROGRESS) 的 [COMMAND_ACK](#COMMAND_ACK) 消息（速度由实现决定），并且必须通过发送带有操作最终结果的 [COMMAND_ACK](#COMMAND_ACK) 消息来终止。可以使用 [COMMAND_ACK](#COMMAND_ACK).progress 字段来指示操作的进度。 |
| <a id='MAV_RESULT_CANCELLED'></a>6                     | [MAV_RESULT_CANCELLED](#MAV_RESULT_CANCELLED)                | 命令已取消（由于收到 [COMMAND_CANCEL](#COMMAND_CANCEL)信息）。 |
| <a id='MAV_RESULT_COMMAND_LONG_ONLY'></a>7             | [MAV_RESULT_COMMAND_LONG_ONLY](#MAV_RESULT_COMMAND_LONG_ONLY) | 只有以 [COMMAND_LONG]（#COMMAND_LONG）形式发送时，才接受该命令。 |
| <a id='MAV_RESULT_COMMAND_INT_ONLY'></a>8              | [MAV_RESULT_COMMAND_INT_ONLY](#MAV_RESULT_COMMAND_INT_ONLY)  | 只有以 [COMMAND_INT]（#COMMAND_INT）形式发送时，才接受该命令。 |
| <a id='MAV_RESULT_COMMAND_UNSUPPORTED_MAV_FRAME'></a>9 | [MAV_RESULT_COMMAND_UNSUPPORTED_MAV_FRAME](#MAV_RESULT_COMMAND_UNSUPPORTED_MAV_FRAME) | 命令无效，因为需要帧，但不支持指定的帧。                     |

### MAV_MISSION_RESULT 

任务运行结果（在 [MISSION_ACK]（#MISSION_ACK）报文中）。

| Value                                          | Name                                                         | Description                                |
| ---------------------------------------------- | ------------------------------------------------------------ | ------------------------------------------ |
| <a id='MAV_MISSION_ACCEPTED'></a>0             | [MAV_MISSION_ACCEPTED](#MAV_MISSION_ACCEPTED)                | 接受任务                                   |
| <a id='MAV_MISSION_ERROR'></a>1                | [MAV_MISSION_ERROR](#MAV_MISSION_ERROR)                      | 通用错误/现在完全不接受任务指令。          |
| <a id='MAV_MISSION_UNSUPPORTED_FRAME'></a>2    | [MAV_MISSION_UNSUPPORTED_FRAME](#MAV_MISSION_UNSUPPORTED_FRAME) | 不支持坐标系。                             |
| <a id='MAV_MISSION_UNSUPPORTED'></a>3          | [MAV_MISSION_UNSUPPORTED](#MAV_MISSION_UNSUPPORTED)          | 不支持该命令。                             |
| <a id='MAV_MISSION_NO_SPACE'></a>4             | [MAV_MISSION_NO_SPACE](#MAV_MISSION_NO_SPACE)                | 任务物品超出存储空间。                     |
| <a id='MAV_MISSION_INVALID'></a>5              | [MAV_MISSION_INVALID](#MAV_MISSION_INVALID)                  | 其中一个参数值无效。                       |
| <a id='MAV_MISSION_INVALID_PARAM1'></a>6       | [MAV_MISSION_INVALID_PARAM1](#MAV_MISSION_INVALID_PARAM1)    | param1 的值无效。                          |
| <a id='MAV_MISSION_INVALID_PARAM2'></a>7       | [MAV_MISSION_INVALID_PARAM2](#MAV_MISSION_INVALID_PARAM2)    | param2 has an invalid value.               |
| <a id='MAV_MISSION_INVALID_PARAM3'></a>8       | [MAV_MISSION_INVALID_PARAM3](#MAV_MISSION_INVALID_PARAM3)    | param3 has an invalid value.               |
| <a id='MAV_MISSION_INVALID_PARAM4'></a>9       | [MAV_MISSION_INVALID_PARAM4](#MAV_MISSION_INVALID_PARAM4)    | param4 has an invalid value.               |
| <a id='MAV_MISSION_INVALID_PARAM5_X'></a>10    | [MAV_MISSION_INVALID_PARAM5_X](#MAV_MISSION_INVALID_PARAM5_X) | x / param5 的值无效。                      |
| <a id='MAV_MISSION_INVALID_PARAM6_Y'></a>11    | [MAV_MISSION_INVALID_PARAM6_Y](#MAV_MISSION_INVALID_PARAM6_Y) | y / param6 has an invalid value.           |
| <a id='MAV_MISSION_INVALID_PARAM7'></a>12      | [MAV_MISSION_INVALID_PARAM7](#MAV_MISSION_INVALID_PARAM7)    | z / param7 has an invalid value.           |
| <a id='MAV_MISSION_INVALID_SEQUENCE'></a>13    | [MAV_MISSION_INVALID_SEQUENCE](#MAV_MISSION_INVALID_SEQUENCE) | 任务项目收到的顺序不对                     |
| <a id='MAV_MISSION_DENIED'></a>14              | [MAV_MISSION_DENIED](#MAV_MISSION_DENIED)                    | 不接受来自该通讯伙伴的任何任务指令。       |
| <a id='MAV_MISSION_OPERATION_CANCELLED'></a>15 | [MAV_MISSION_OPERATION_CANCELLED](#MAV_MISSION_OPERATION_CANCELLED) | 当前任务操作取消（如任务上传、任务下载）。 |

### MAV_SEVERITY 

表示严重性级别，一般用于状态信息，以表示其相对紧迫性。基于 RFC-5424，使用以下扩展定义： http://www.kiwisyslog.com/kb/info:-syslog-message-levels/.

| Value                                | Name                                              | Description                                                  |
| ------------------------------------ | ------------------------------------------------- | ------------------------------------------------------------ |
| <a id='MAV_SEVERITY_EMERGENCY'></a>0 | [MAV_SEVERITY_EMERGENCY](#MAV_SEVERITY_EMERGENCY) | 系统无法使用。这是一种 "恐慌 "状态。                         |
| <a id='MAV_SEVERITY_ALERT'></a>1     | [MAV_SEVERITY_ALERT](#MAV_SEVERITY_ALERT)         | 应立即采取行动。表示非关键系统出错。                         |
| <a id='MAV_SEVERITY_CRITICAL'></a>2  | [MAV_SEVERITY_CRITICAL](#MAV_SEVERITY_CRITICAL)   | 必须立即采取行动。表示主系统出现故障。                       |
| <a id='MAV_SEVERITY_ERROR'></a>3     | [MAV_SEVERITY_ERROR](#MAV_SEVERITY_ERROR)         | 表示辅助/冗余系统出错。                                      |
| <a id='MAV_SEVERITY_WARNING'></a>4   | [MAV_SEVERITY_WARNING](#MAV_SEVERITY_WARNING)     | 如果在给定时间内未解决，则表示未来可能出现错误。例如低电量警告。 |
| <a id='MAV_SEVERITY_NOTICE'></a>5    | [MAV_SEVERITY_NOTICE](#MAV_SEVERITY_NOTICE)       | 发生了异常事件，尽管不是错误状况。应调查其根本原因。         |
| <a id='MAV_SEVERITY_INFO'></a>6      | [MAV_SEVERITY_INFO](#MAV_SEVERITY_INFO)           | 正常运行信息。用于记录。无需对这些信息采取任何措施。         |
| <a id='MAV_SEVERITY_DEBUG'></a>7     | [MAV_SEVERITY_DEBUG](#MAV_SEVERITY_DEBUG)         | 有助于调试的有用的非运行信息。正常运行时不应出现这些信息。   |

### MAV_POWER_STATUS 

(位掩码） 电源状态标志（位掩码）

| Value                                                      | Name                                                         | Description                    |
| ---------------------------------------------------------- | ------------------------------------------------------------ | ------------------------------ |
| <a id='MAV_POWER_STATUS_BRICK_VALID'></a>1                 | [MAV_POWER_STATUS_BRICK_VALID](#MAV_POWER_STATUS_BRICK_VALID) | 主砖供电有效                   |
| <a id='MAV_POWER_STATUS_SERVO_VALID'></a>2                 | [MAV_POWER_STATUS_SERVO_VALID](#MAV_POWER_STATUS_SERVO_VALID) | FMU 的主伺服电源有效           |
| <a id='MAV_POWER_STATUS_USB_CONNECTED'></a>4               | [MAV_POWER_STATUS_USB_CONNECTED](#MAV_POWER_STATUS_USB_CONNECTED) | USB 电源已连接                 |
| <a id='MAV_POWER_STATUS_PERIPH_OVERCURRENT'></a>8          | [MAV_POWER_STATUS_PERIPH_OVERCURRENT](#MAV_POWER_STATUS_PERIPH_OVERCURRENT) | 外围供电处于过流状态           |
| <a id='MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT'></a>16 | [MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT](#MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT) | 高功率外围设备电源处于过流状态 |
| <a id='MAV_POWER_STATUS_CHANGED'></a>32                    | [MAV_POWER_STATUS_CHANGED](#MAV_POWER_STATUS_CHANGED)        | 启动后电源状态已更改           |

### SERIAL_CONTROL_DEV 

[SERIAL_CONTROL](#SERIAL_CONTROL) 设备类型

| Value                                   | Name                                                    | Description     |
| --------------------------------------- | ------------------------------------------------------- | --------------- |
| <a id='SERIAL_CONTROL_DEV_TELEM1'></a>0 | [SERIAL_CONTROL_DEV_TELEM1](#SERIAL_CONTROL_DEV_TELEM1) | 第一个遥测端口  |
| <a id='SERIAL_CONTROL_DEV_TELEM2'></a>1 | [SERIAL_CONTROL_DEV_TELEM2](#SERIAL_CONTROL_DEV_TELEM2) | 第二个遥测端口  |
| <a id='SERIAL_CONTROL_DEV_GPS1'></a>2   | [SERIAL_CONTROL_DEV_GPS1](#SERIAL_CONTROL_DEV_GPS1)     | 第一个 GPS 端口 |
| <a id='SERIAL_CONTROL_DEV_GPS2'></a>3   | [SERIAL_CONTROL_DEV_GPS2](#SERIAL_CONTROL_DEV_GPS2)     | 第二个 GPS 端口 |
| <a id='SERIAL_CONTROL_DEV_SHELL'></a>10 | [SERIAL_CONTROL_DEV_SHELL](#SERIAL_CONTROL_DEV_SHELL)   | 系统外壳        |
| <a id='SERIAL_CONTROL_SERIAL0'></a>100  | [SERIAL_CONTROL_SERIAL0](#SERIAL_CONTROL_SERIAL0)       | 序号 0          |
| <a id='SERIAL_CONTROL_SERIAL1'></a>101  | [SERIAL_CONTROL_SERIAL1](#SERIAL_CONTROL_SERIAL1)       | SERIAL1         |
| <a id='SERIAL_CONTROL_SERIAL2'></a>102  | [SERIAL_CONTROL_SERIAL2](#SERIAL_CONTROL_SERIAL2)       | SERIAL2         |
| <a id='SERIAL_CONTROL_SERIAL3'></a>103  | [SERIAL_CONTROL_SERIAL3](#SERIAL_CONTROL_SERIAL3)       | SERIAL3         |
| <a id='SERIAL_CONTROL_SERIAL4'></a>104  | [SERIAL_CONTROL_SERIAL4](#SERIAL_CONTROL_SERIAL4)       | SERIAL4         |
| <a id='SERIAL_CONTROL_SERIAL5'></a>105  | [SERIAL_CONTROL_SERIAL5](#SERIAL_CONTROL_SERIAL5)       | SERIAL5         |
| <a id='SERIAL_CONTROL_SERIAL6'></a>106  | [SERIAL_CONTROL_SERIAL6](#SERIAL_CONTROL_SERIAL6)       | SERIAL6         |
| <a id='SERIAL_CONTROL_SERIAL7'></a>107  | [SERIAL_CONTROL_SERIAL7](#SERIAL_CONTROL_SERIAL7)       | SERIAL7         |
| <a id='SERIAL_CONTROL_SERIAL8'></a>108  | [SERIAL_CONTROL_SERIAL8](#SERIAL_CONTROL_SERIAL8)       | SERIAL8         |
| <a id='SERIAL_CONTROL_SERIAL9'></a>109  | [SERIAL_CONTROL_SERIAL9](#SERIAL_CONTROL_SERIAL9)       | SERIAL9         |

### SERIAL_CONTROL_FLAG 

(Bitmask) [SERIAL_CONTROL](#SERIAL_CONTROL) 标志 (Bitmask)

| Value                                       | Name                                                         | Description                                                  |
| ------------------------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| <a id='SERIAL_CONTROL_FLAG_REPLY'></a>1     | [SERIAL_CONTROL_FLAG_REPLY](#SERIAL_CONTROL_FLAG_REPLY)      | 设置是否为回复                                               |
| <a id='SERIAL_CONTROL_FLAG_RESPOND'></a>2   | [SERIAL_CONTROL_FLAG_RESPOND](#SERIAL_CONTROL_FLAG_RESPOND)  | 设置发送方是否希望接收方以另一条 [SERIAL_CONTROL]（#SERIAL_CONTROL）报文的形式发送回复 |
| <a id='SERIAL_CONTROL_FLAG_EXCLUSIVE'></a>4 | [SERIAL_CONTROL_FLAG_EXCLUSIVE](#SERIAL_CONTROL_FLAG_EXCLUSIVE) | 设置是否应从当前使用串行端口的任何驱动程序中移除对该端口的访问，从而使 [SERIAL_CONTROL](#SERIAL_CONTROL) 协议独占访问串行端口。在未设置该标志的情况下，可通过发送请求交还端口 |
| <a id='SERIAL_CONTROL_FLAG_BLOCKING'></a>8  | [SERIAL_CONTROL_FLAG_BLOCKING](#SERIAL_CONTROL_FLAG_BLOCKING) | 阻止对串行端口的写入                                         |
| <a id='SERIAL_CONTROL_FLAG_MULTI'></a>16    | [SERIAL_CONTROL_FLAG_MULTI](#SERIAL_CONTROL_FLAG_MULTI)      | 发送多个回复，直到端口耗尽                                   |

### MAV_DISTANCE_SENSOR 

距离传感器类型枚举

| Value                                        | Name                                                         | Description                                         |
| -------------------------------------------- | ------------------------------------------------------------ | --------------------------------------------------- |
| <a id='MAV_DISTANCE_SENSOR_LASER'></a>0      | [MAV_DISTANCE_SENSOR_LASER](#MAV_DISTANCE_SENSOR_LASER)      | 激光测距仪，如 LightWare SF02/F 或 PulsedLight 设备 |
| <a id='MAV_DISTANCE_SENSOR_ULTRASOUND'></a>1 | [MAV_DISTANCE_SENSOR_ULTRASOUND](#MAV_DISTANCE_SENSOR_ULTRASOUND) | 超声波测距仪，如 MaxBotix 设备                      |
| <a id='MAV_DISTANCE_SENSOR_INFRARED'></a>2   | [MAV_DISTANCE_SENSOR_INFRARED](#MAV_DISTANCE_SENSOR_INFRARED) | 红外线测距仪，如夏普设备                            |
| <a id='MAV_DISTANCE_SENSOR_RADAR'></a>3      | [MAV_DISTANCE_SENSOR_RADAR](#MAV_DISTANCE_SENSOR_RADAR)      | 雷达类型，例如 u 着陆装置                           |
| <a id='MAV_DISTANCE_SENSOR_UNKNOWN'></a>4    | [MAV_DISTANCE_SENSOR_UNKNOWN](#MAV_DISTANCE_SENSOR_UNKNOWN)  | 损坏或未知类型，例如模拟装置                        |

### MAV_SENSOR_ORIENTATION 

根据传感器的旋转枚举传感器方向

| Value                                                       | Name                                                         | Description                   |
| ----------------------------------------------------------- | ------------------------------------------------------------ | ----------------------------- |
| <a id='MAV_SENSOR_ROTATION_NONE'></a>0                      | [MAV_SENSOR_ROTATION_NONE](#MAV_SENSOR_ROTATION_NONE)        | Roll: 0, Pitch: 0, Yaw: 0     |
| <a id='MAV_SENSOR_ROTATION_YAW_45'></a>1                    | [MAV_SENSOR_ROTATION_YAW_45](#MAV_SENSOR_ROTATION_YAW_45)    | Roll: 0, Pitch: 0, Yaw: 45    |
| <a id='MAV_SENSOR_ROTATION_YAW_90'></a>2                    | [MAV_SENSOR_ROTATION_YAW_90](#MAV_SENSOR_ROTATION_YAW_90)    | Roll: 0, Pitch: 0, Yaw: 90    |
| <a id='MAV_SENSOR_ROTATION_YAW_135'></a>3                   | [MAV_SENSOR_ROTATION_YAW_135](#MAV_SENSOR_ROTATION_YAW_135)  | Roll: 0, Pitch: 0, Yaw: 135   |
| <a id='MAV_SENSOR_ROTATION_YAW_180'></a>4                   | [MAV_SENSOR_ROTATION_YAW_180](#MAV_SENSOR_ROTATION_YAW_180)  | Roll: 0, Pitch: 0, Yaw: 180   |
| <a id='MAV_SENSOR_ROTATION_YAW_225'></a>5                   | [MAV_SENSOR_ROTATION_YAW_225](#MAV_SENSOR_ROTATION_YAW_225)  | Roll: 0, Pitch: 0, Yaw: 225   |
| <a id='MAV_SENSOR_ROTATION_YAW_270'></a>6                   | [MAV_SENSOR_ROTATION_YAW_270](#MAV_SENSOR_ROTATION_YAW_270)  | Roll: 0, Pitch: 0, Yaw: 270   |
| <a id='MAV_SENSOR_ROTATION_YAW_315'></a>7                   | [MAV_SENSOR_ROTATION_YAW_315](#MAV_SENSOR_ROTATION_YAW_315)  | Roll: 0, Pitch: 0, Yaw: 315   |
| <a id='MAV_SENSOR_ROTATION_ROLL_180'></a>8                  | [MAV_SENSOR_ROTATION_ROLL_180](#MAV_SENSOR_ROTATION_ROLL_180) | Roll: 180, Pitch: 0, Yaw: 0   |
| <a id='MAV_SENSOR_ROTATION_ROLL_180_YAW_45'></a>9           | [MAV_SENSOR_ROTATION_ROLL_180_YAW_45](#MAV_SENSOR_ROTATION_ROLL_180_YAW_45) | Roll: 180, Pitch: 0, Yaw: 45  |
| <a id='MAV_SENSOR_ROTATION_ROLL_180_YAW_90'></a>10          | [MAV_SENSOR_ROTATION_ROLL_180_YAW_90](#MAV_SENSOR_ROTATION_ROLL_180_YAW_90) | Roll: 180, Pitch: 0, Yaw: 90  |
| <a id='MAV_SENSOR_ROTATION_ROLL_180_YAW_135'></a>11         | [MAV_SENSOR_ROTATION_ROLL_180_YAW_135](#MAV_SENSOR_ROTATION_ROLL_180_YAW_135) | Roll: 180, Pitch: 0, Yaw: 135 |
| <a id='MAV_SENSOR_ROTATION_PITCH_180'></a>12                | [MAV_SENSOR_ROTATION_PITCH_180](#MAV_SENSOR_ROTATION_PITCH_180) | Roll: 0, Pitch: 180, Yaw: 0   |
| <a id='MAV_SENSOR_ROTATION_ROLL_180_YAW_225'></a>13         | [MAV_SENSOR_ROTATION_ROLL_180_YAW_225](#MAV_SENSOR_ROTATION_ROLL_180_YAW_225) | Roll: 180, Pitch: 0, Yaw: 225 |
| <a id='MAV_SENSOR_ROTATION_ROLL_180_YAW_270'></a>14         | [MAV_SENSOR_ROTATION_ROLL_180_YAW_270](#MAV_SENSOR_ROTATION_ROLL_180_YAW_270) | Roll: 180, Pitch: 0, Yaw: 270 |
| <a id='MAV_SENSOR_ROTATION_ROLL_180_YAW_315'></a>15         | [MAV_SENSOR_ROTATION_ROLL_180_YAW_315](#MAV_SENSOR_ROTATION_ROLL_180_YAW_315) | Roll: 180, Pitch: 0, Yaw: 315 |
| <a id='MAV_SENSOR_ROTATION_ROLL_90'></a>16                  | [MAV_SENSOR_ROTATION_ROLL_90](#MAV_SENSOR_ROTATION_ROLL_90)  | Roll: 90, Pitch: 0, Yaw: 0    |
| <a id='MAV_SENSOR_ROTATION_ROLL_90_YAW_45'></a>17           | [MAV_SENSOR_ROTATION_ROLL_90_YAW_45](#MAV_SENSOR_ROTATION_ROLL_90_YAW_45) | Roll: 90, Pitch: 0, Yaw: 45   |
| <a id='MAV_SENSOR_ROTATION_ROLL_90_YAW_90'></a>18           | [MAV_SENSOR_ROTATION_ROLL_90_YAW_90](#MAV_SENSOR_ROTATION_ROLL_90_YAW_90) | Roll: 90, Pitch: 0, Yaw: 90   |
| <a id='MAV_SENSOR_ROTATION_ROLL_90_YAW_135'></a>19          | [MAV_SENSOR_ROTATION_ROLL_90_YAW_135](#MAV_SENSOR_ROTATION_ROLL_90_YAW_135) | Roll: 90, Pitch: 0, Yaw: 135  |
| <a id='MAV_SENSOR_ROTATION_ROLL_270'></a>20                 | [MAV_SENSOR_ROTATION_ROLL_270](#MAV_SENSOR_ROTATION_ROLL_270) | Roll: 270, Pitch: 0, Yaw: 0   |
| <a id='MAV_SENSOR_ROTATION_ROLL_270_YAW_45'></a>21          | [MAV_SENSOR_ROTATION_ROLL_270_YAW_45](#MAV_SENSOR_ROTATION_ROLL_270_YAW_45) | Roll: 270, Pitch: 0, Yaw: 45  |
| <a id='MAV_SENSOR_ROTATION_ROLL_270_YAW_90'></a>22          | [MAV_SENSOR_ROTATION_ROLL_270_YAW_90](#MAV_SENSOR_ROTATION_ROLL_270_YAW_90) | Roll: 270, Pitch: 0, Yaw: 90  |
| <a id='MAV_SENSOR_ROTATION_ROLL_270_YAW_135'></a>23         | [MAV_SENSOR_ROTATION_ROLL_270_YAW_135](#MAV_SENSOR_ROTATION_ROLL_270_YAW_135) | Roll: 270, Pitch: 0, Yaw: 135 |
| <a id='MAV_SENSOR_ROTATION_PITCH_90'></a>24                 | [MAV_SENSOR_ROTATION_PITCH_90](#MAV_SENSOR_ROTATION_PITCH_90) | Roll: 0, Pitch: 90, Yaw: 0    |
| <a id='MAV_SENSOR_ROTATION_PITCH_270'></a>25                | [MAV_SENSOR_ROTATION_PITCH_270](#MAV_SENSOR_ROTATION_PITCH_270) | Roll: 0, Pitch: 270, Yaw: 0   |
| <a id='MAV_SENSOR_ROTATION_PITCH_180_YAW_90'></a>26         | [MAV_SENSOR_ROTATION_PITCH_180_YAW_90](#MAV_SENSOR_ROTATION_PITCH_180_YAW_90) | Roll: 0, Pitch: 180, Yaw: 90  |
| <a id='MAV_SENSOR_ROTATION_PITCH_180_YAW_270'></a>27        | [MAV_SENSOR_ROTATION_PITCH_180_YAW_270](#MAV_SENSOR_ROTATION_PITCH_180_YAW_270) | Roll: 0, Pitch: 180, Yaw: 270 |
| <a id='MAV_SENSOR_ROTATION_ROLL_90_PITCH_90'></a>28         | [MAV_SENSOR_ROTATION_ROLL_90_PITCH_90](#MAV_SENSOR_ROTATION_ROLL_90_PITCH_90) | Roll: 90, Pitch: 90, Yaw: 0   |
| <a id='MAV_SENSOR_ROTATION_ROLL_180_PITCH_90'></a>29        | [MAV_SENSOR_ROTATION_ROLL_180_PITCH_90](#MAV_SENSOR_ROTATION_ROLL_180_PITCH_90) | Roll: 180, Pitch: 90, Yaw: 0  |
| <a id='MAV_SENSOR_ROTATION_ROLL_270_PITCH_90'></a>30        | [MAV_SENSOR_ROTATION_ROLL_270_PITCH_90](#MAV_SENSOR_ROTATION_ROLL_270_PITCH_90) | Roll: 270, Pitch: 90, Yaw: 0  |
| <a id='MAV_SENSOR_ROTATION_ROLL_90_PITCH_180'></a>31        | [MAV_SENSOR_ROTATION_ROLL_90_PITCH_180](#MAV_SENSOR_ROTATION_ROLL_90_PITCH_180) | Roll: 90, Pitch: 180, Yaw: 0  |
| <a id='MAV_SENSOR_ROTATION_ROLL_270_PITCH_180'></a>32       | [MAV_SENSOR_ROTATION_ROLL_270_PITCH_180](#MAV_SENSOR_ROTATION_ROLL_270_PITCH_180) | Roll: 270, Pitch: 180, Yaw: 0 |
| <a id='MAV_SENSOR_ROTATION_ROLL_90_PITCH_270'></a>33        | [MAV_SENSOR_ROTATION_ROLL_90_PITCH_270](#MAV_SENSOR_ROTATION_ROLL_90_PITCH_270) | Roll: 90, Pitch: 270, Yaw: 0  |
| <a id='MAV_SENSOR_ROTATION_ROLL_180_PITCH_270'></a>34       | [MAV_SENSOR_ROTATION_ROLL_180_PITCH_270](#MAV_SENSOR_ROTATION_ROLL_180_PITCH_270) | Roll: 180, Pitch: 270, Yaw: 0 |
| <a id='MAV_SENSOR_ROTATION_ROLL_270_PITCH_270'></a>35       | [MAV_SENSOR_ROTATION_ROLL_270_PITCH_270](#MAV_SENSOR_ROTATION_ROLL_270_PITCH_270) | Roll: 270, Pitch: 270, Yaw: 0 |
| <a id='MAV_SENSOR_ROTATION_ROLL_90_PITCH_180_YAW_90'></a>36 | [MAV_SENSOR_ROTATION_ROLL_90_PITCH_180_YAW_90](#MAV_SENSOR_ROTATION_ROLL_90_PITCH_180_YAW_90) | Roll: 90, Pitch: 180, Yaw: 90 |
| <a id='MAV_SENSOR_ROTATION_ROLL_90_YAW_270'></a>37          | [MAV_SENSOR_ROTATION_ROLL_90_YAW_270](#MAV_SENSOR_ROTATION_ROLL_90_YAW_270) | Roll: 90, Pitch: 0, Yaw: 270  |
| <a id='MAV_SENSOR_ROTATION_ROLL_90_PITCH_68_YAW_293'></a>38 | [MAV_SENSOR_ROTATION_ROLL_90_PITCH_68_YAW_293](#MAV_SENSOR_ROTATION_ROLL_90_PITCH_68_YAW_293) | Roll: 90, Pitch: 68, Yaw: 293 |
| <a id='MAV_SENSOR_ROTATION_PITCH_315'></a>39                | [MAV_SENSOR_ROTATION_PITCH_315](#MAV_SENSOR_ROTATION_PITCH_315) | Pitch: 315                    |
| <a id='MAV_SENSOR_ROTATION_ROLL_90_PITCH_315'></a>40        | [MAV_SENSOR_ROTATION_ROLL_90_PITCH_315](#MAV_SENSOR_ROTATION_ROLL_90_PITCH_315) | Roll: 90, Pitch: 315          |
| <a id='MAV_SENSOR_ROTATION_CUSTOM'></a>100                  | [MAV_SENSOR_ROTATION_CUSTOM](#MAV_SENSOR_ROTATION_CUSTOM)    | 自定义方向                    |

### MAV_PROTOCOL_CAPABILITY 

(位掩码）自动驾驶仪功能（可选）的位掩码（64 位）。如果某位被设置，则自动驾驶仪支持该功能。

| Value                                                        | Name                                                         | Description                                                  |
| ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |
| <a id='MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT'></a>1          | [MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT](#MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT) | 自动驾驶仪支持[MISSION_ITEM](#MISSION_ITEM)浮点信息类型。<br>请注意，[MISSION_ITEM](#MISSION_ITEM)已被弃用，自动驾驶仪应改用[MISSION_INT](#MISSION_INT)。 |
| <a id='MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT'></a>2            | [MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT](#MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT) | 自动驾驶仪支持新的参数浮动报文类型<b><span class="warning">**DEPRECATED:** Replaced By [MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_C_CAST](#MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_C_CAST) (2022-03)</span> |
| <a id='MAV_PROTOCOL_CAPABILITY_MISSION_INT'></a>4            | [MAV_PROTOCOL_CAPABILITY_MISSION_INT](#MAV_PROTOCOL_CAPABILITY_MISSION_INT) | 自动驾驶仪支持[MISSION_ITEM_INT](#MISSION_ITEM_INT)缩放整数消息类型。<br>注意，如果支持任务，则必须始终设置此标记，因为任务必须始终使用[MISSION_ITEM_INT](#MISSION_ITEM_INT)（而不是已废弃的[MISSION_ITEM](#MISSION_ITEM)）。 |
| <a id='MAV_PROTOCOL_CAPABILITY_COMMAND_INT'></a>8            | [MAV_PROTOCOL_CAPABILITY_COMMAND_INT](#MAV_PROTOCOL_CAPABILITY_COMMAND_INT) | 自动驾驶仪支持 [COMMAND_INT]（#COMMAND_INT）缩放整数信息类型。 |
| <a id='MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE'></a>16 | [MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE](#MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE) | 参数协议将参数值按字节编码到 param_value（浮点）字段中： https://mavlink.io/en/services/parameter.html#parameter-encoding.<br>请注意，该标记或 [MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_C_CAST](#MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_C_CAST) 如果支持参数协议，则应设置该参数。 |
| <a id='MAV_PROTOCOL_CAPABILITY_FTP'></a>32                   | [MAV_PROTOCOL_CAPABILITY_FTP](#MAV_PROTOCOL_CAPABILITY_FTP)  | Autopilot 支持文件传输协议 v1： https://mavlink.io/en/services/ftp.html. |
| <a id='MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET'></a>64   | [MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET](#MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET) | 自动驾驶仪支持在机外指挥姿态。                               |
| <a id='MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED'></a>128 | [MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED](#MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED) | 自动驾驶仪支持在本地 NED 框架内指挥位置和速度目标。          |
| <a id='MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT'></a>256 | [MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT](#MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT) | 自动驾驶仪支持以全局缩放整数为单位的位置和速度目标指令。     |
| <a id='MAV_PROTOCOL_CAPABILITY_TERRAIN'></a>512              | [MAV_PROTOCOL_CAPABILITY_TERRAIN](#MAV_PROTOCOL_CAPABILITY_TERRAIN) | 自动驾驶仪支持地形协议/数据处理。                            |
| <a id='MAV_PROTOCOL_CAPABILITY_RESERVED3'></a>1024           | [MAV_PROTOCOL_CAPABILITY_RESERVED3](#MAV_PROTOCOL_CAPABILITY_RESERVED3) | 留待将来使用。                                               |
| <a id='MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION'></a>2048  | [MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION](#MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION) | 自动驾驶仪支持 [MAV_CMD_DO_FLIGHTTERMINATION]（#MAV_CMD_DO_FLIGHTTERMINATION）命令（飞行终止）。 |
| <a id='MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION'></a>4096 | [MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION](#MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION) | 自动驾驶仪支持机载罗盘校准。                                 |
| <a id='MAV_PROTOCOL_CAPABILITY_MAVLINK2'></a>8192            | [MAV_PROTOCOL_CAPABILITY_MAVLINK2](#MAV_PROTOCOL_CAPABILITY_MAVLINK2) | 自动驾驶仪支持 MAVLink 版本 2。                              |
| <a id='MAV_PROTOCOL_CAPABILITY_MISSION_FENCE'></a>16384      | [MAV_PROTOCOL_CAPABILITY_MISSION_FENCE](#MAV_PROTOCOL_CAPABILITY_MISSION_FENCE) | 自动驾驶仪支持任务围栏协议。                                 |
| <a id='MAV_PROTOCOL_CAPABILITY_MISSION_RALLY'></a>32768      | [MAV_PROTOCOL_CAPABILITY_MISSION_RALLY](#MAV_PROTOCOL_CAPABILITY_MISSION_RALLY) | 自动驾驶仪支持任务集结点协议。                               |
| <a id='MAV_PROTOCOL_CAPABILITY_RESERVED2'></a>65536          | [MAV_PROTOCOL_CAPABILITY_RESERVED2](#MAV_PROTOCOL_CAPABILITY_RESERVED2) | 留待将来使用。                                               |
| <a id='MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_C_CAST'></a>131072 | [MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_C_CAST](#MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_C_CAST) | 参数协议使用参数值的 C-cast，来设置 param_value（浮点）字段： https://mavlink.io/en/services/parameter.html#parameter-encoding.<br>请注意，该标记或 [MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE](#MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE) 如果支持参数协议，则应设置该参数。 |

### MAV_MISSION_TYPE 

任务协议中要求/发送的任务项目类型。

| Value                                  | Name                                                  | Description                                                  |
| -------------------------------------- | ----------------------------------------------------- | ------------------------------------------------------------ |
| <a id='MAV_MISSION_TYPE_MISSION'></a>0 | [MAV_MISSION_TYPE_MISSION](#MAV_MISSION_TYPE_MISSION) | 项目是主要任务的任务指令。                                   |
| <a id='MAV_MISSION_TYPE_FENCE'></a>1   | [MAV_MISSION_TYPE_FENCE](#MAV_MISSION_TYPE_FENCE)     | 指定 GeoFence 区域。项目为 MAV_CMD_NAV_FENCE_ GeoFence 项目。 |
| <a id='MAV_MISSION_TYPE_RALLY'></a>2   | [MAV_MISSION_TYPE_RALLY](#MAV_MISSION_TYPE_RALLY)     | 指定车辆的集结点。集结点是备选的 RTL 点。项目为 [MAV_CMD_NAV_RALLY_POINT](#MAV_CMD_NAV_RALLY_POINT)集结点项目。 |
| <a id='MAV_MISSION_TYPE_ALL'></a>255   | [MAV_MISSION_TYPE_ALL](#MAV_MISSION_TYPE_ALL)         | 仅用于 [MISSION_CLEAR_ALL](#MISSION_CLEAR_ALL)，以清除所有任务类型 |

### MAV_ESTIMATOR_TYPE 

估算器类型枚举

| Value                                      | Name                                                         | Description                                  |
| ------------------------------------------ | ------------------------------------------------------------ | -------------------------------------------- |
| <a id='MAV_ESTIMATOR_TYPE_UNKNOWN'></a>0   | [MAV_ESTIMATOR_TYPE_UNKNOWN](#MAV_ESTIMATOR_TYPE_UNKNOWN)    | 估算器的未知类型。                           |
| <a id='MAV_ESTIMATOR_TYPE_NAIVE'></a>1     | [MAV_ESTIMATOR_TYPE_NAIVE](#MAV_ESTIMATOR_TYPE_NAIVE)        | 这是一个没有任何实际协方差反馈的天真估计值。 |
| <a id='MAV_ESTIMATOR_TYPE_VISION'></a>2    | [MAV_ESTIMATOR_TYPE_VISION](#MAV_ESTIMATOR_TYPE_VISION)      | 基于计算机视觉的估算。可能符合比例尺。       |
| <a id='MAV_ESTIMATOR_TYPE_VIO'></a>3       | [MAV_ESTIMATOR_TYPE_VIO](#MAV_ESTIMATOR_TYPE_VIO)            | 视觉惯性估算。                               |
| <a id='MAV_ESTIMATOR_TYPE_GPS'></a>4       | [MAV_ESTIMATOR_TYPE_GPS](#MAV_ESTIMATOR_TYPE_GPS)            | 普通 GPS 估算值。                            |
| <a id='MAV_ESTIMATOR_TYPE_GPS_INS'></a>5   | [MAV_ESTIMATOR_TYPE_GPS_INS](#MAV_ESTIMATOR_TYPE_GPS_INS)    | 集成 GPS 和惯性传感的估算器。                |
| <a id='MAV_ESTIMATOR_TYPE_MOCAP'></a>6     | [MAV_ESTIMATOR_TYPE_MOCAP](#MAV_ESTIMATOR_TYPE_MOCAP)        | 来自外部运动捕捉系统的估计值。               |
| <a id='MAV_ESTIMATOR_TYPE_LIDAR'></a>7     | [MAV_ESTIMATOR_TYPE_LIDAR](#MAV_ESTIMATOR_TYPE_LIDAR)        | 基于激光雷达传感器输入的估算器。             |
| <a id='MAV_ESTIMATOR_TYPE_AUTOPILOT'></a>8 | [MAV_ESTIMATOR_TYPE_AUTOPILOT](#MAV_ESTIMATOR_TYPE_AUTOPILOT) | 自动驾驶估算器                               |

### MAV_BATTERY_TYPE 

电池类型枚举

| Value                                  | Name                                                  | Description  |
| -------------------------------------- | ----------------------------------------------------- | ------------ |
| <a id='MAV_BATTERY_TYPE_UNKNOWN'></a>0 | [MAV_BATTERY_TYPE_UNKNOWN](#MAV_BATTERY_TYPE_UNKNOWN) | 未说明。     |
| <a id='MAV_BATTERY_TYPE_LIPO'></a>1    | [MAV_BATTERY_TYPE_LIPO](#MAV_BATTERY_TYPE_LIPO)       | 锂聚合物电池 |
| <a id='MAV_BATTERY_TYPE_LIFE'></a>2    | [MAV_BATTERY_TYPE_LIFE](#MAV_BATTERY_TYPE_LIFE)       | 磷酸铁锂电池 |
| <a id='MAV_BATTERY_TYPE_LION'></a>3    | [MAV_BATTERY_TYPE_LION](#MAV_BATTERY_TYPE_LION)       | 锂离子电池   |
| <a id='MAV_BATTERY_TYPE_NIMH'></a>4    | [MAV_BATTERY_TYPE_NIMH](#MAV_BATTERY_TYPE_NIMH)       | 镍氢电池     |

### MAV_BATTERY_FUNCTION 

电池功能枚举

| Value                                         | Name                                                         | Description          |
| --------------------------------------------- | ------------------------------------------------------------ | -------------------- |
| <a id='MAV_BATTERY_FUNCTION_UNKNOWN'></a>0    | [MAV_BATTERY_FUNCTION_UNKNOWN](#MAV_BATTERY_FUNCTION_UNKNOWN) | 电池功能未知         |
| <a id='MAV_BATTERY_FUNCTION_ALL'></a>1        | [MAV_BATTERY_FUNCTION_ALL](#MAV_BATTERY_FUNCTION_ALL)        | 电池支持所有飞行系统 |
| <a id='MAV_BATTERY_FUNCTION_PROPULSION'></a>2 | [MAV_BATTERY_FUNCTION_PROPULSION](#MAV_BATTERY_FUNCTION_PROPULSION) | 推进系统电池         |
| <a id='MAV_BATTERY_FUNCTION_AVIONICS'></a>3   | [MAV_BATTERY_FUNCTION_AVIONICS](#MAV_BATTERY_FUNCTION_AVIONICS) | 航空电子设备电池     |
| <a id='MAV_BATTERY_FUNCTION_PAYLOAD'></a>4    | [MAV_BATTERY_FUNCTION_PAYLOAD](#MAV_BATTERY_FUNCTION_PAYLOAD) | 有效载荷电池         |

### MAV_BATTERY_CHARGE_STATE 

电池充电状态枚举。

| Value                                            | Name                                                         | Description                                                  |
| ------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |
| <a id='MAV_BATTERY_CHARGE_STATE_UNDEFINED'></a>0 | [MAV_BATTERY_CHARGE_STATE_UNDEFINED](#MAV_BATTERY_CHARGE_STATE_UNDEFINED) | 不提供低电量状态                                             |
| <a id='MAV_BATTERY_CHARGE_STATE_OK'></a>1        | [MAV_BATTERY_CHARGE_STATE_OK](#MAV_BATTERY_CHARGE_STATE_OK)  | 电池电量不低。正常运行。                                     |
| <a id='MAV_BATTERY_CHARGE_STATE_LOW'></a>2       | [MAV_BATTERY_CHARGE_STATE_LOW](#MAV_BATTERY_CHARGE_STATE_LOW) | 电池电量不足，发出警告并关闭监控器。                         |
| <a id='MAV_BATTERY_CHARGE_STATE_CRITICAL'></a>3  | [MAV_BATTERY_CHARGE_STATE_CRITICAL](#MAV_BATTERY_CHARGE_STATE_CRITICAL) | 电池状态危急，请立即返回或终止。                             |
| <a id='MAV_BATTERY_CHARGE_STATE_EMERGENCY'></a>4 | [MAV_BATTERY_CHARGE_STATE_EMERGENCY](#MAV_BATTERY_CHARGE_STATE_EMERGENCY) | 电池电量过低，无法执行普通终止程序。执行最快的紧急停止操作，以防损坏。 |
| <a id='MAV_BATTERY_CHARGE_STATE_FAILED'></a>5    | [MAV_BATTERY_CHARGE_STATE_FAILED](#MAV_BATTERY_CHARGE_STATE_FAILED) | 电池故障，损坏不可避免。可能的原因（故障）列于 [MAV_BATTERY_FAULT](#MAV_BATTERY_FAULT)。 |
| <a id='MAV_BATTERY_CHARGE_STATE_UNHEALTHY'></a>6 | [MAV_BATTERY_CHARGE_STATE_UNHEALTHY](#MAV_BATTERY_CHARGE_STATE_UNHEALTHY) | 电池被诊断为故障或发生错误，不鼓励/禁止使用。可能的原因（故障）列于 [MAV_BATTERY_FAULT](#MAV_BATTERY_FAULT)。 |
| <a id='MAV_BATTERY_CHARGE_STATE_CHARGING'></a>7  | [MAV_BATTERY_CHARGE_STATE_CHARGING](#MAV_BATTERY_CHARGE_STATE_CHARGING) | 电池正在充电。                                               |

### MAV_BATTERY_MODE 

电池模式。注意，正常运行模式（即飞行时）应报告为 [MAV_BATTERY_MODE_UNKNOWN]（#MAV_BATTERY_MODE_UNKNOWN），以便在正常飞行时进行信息微调。

| Value                                           | Name                                                         | Description                                                  |
| ----------------------------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| <a id='MAV_BATTERY_MODE_UNKNOWN'></a>0          | [MAV_BATTERY_MODE_UNKNOWN](#MAV_BATTERY_MODE_UNKNOWN)        | 不支持电池模式/未知电池模式/正常操作。                       |
| <a id='MAV_BATTERY_MODE_AUTO_DISCHARGING'></a>1 | [MAV_BATTERY_MODE_AUTO_DISCHARGING](#MAV_BATTERY_MODE_AUTO_DISCHARGING) | 电池正在自动放电（接近储存水平）。                           |
| <a id='MAV_BATTERY_MODE_HOT_SWAP'></a>2         | [MAV_BATTERY_MODE_HOT_SWAP](#MAV_BATTERY_MODE_HOT_SWAP)      | 热插拔模式下的电池（电流受限，以防止可能损坏敏感电路的尖峰电流）。 |

### MAV_BATTERY_FAULT 

(智能电池供电状态/故障标志（位掩码），用于指示健康状况。如果设置了[MAV_BATTERY_CHARGE_STATE_FAILED](#MAV_BATTERY_CHARGE_STATE_FAILED)或[MAV_BATTERY_CHARGE_STATE_UNHEALTHY](#MAV_BATTERY_CHARGE_STATE_UNHEALTHY)，电池还必须报告这两个标志。

| Value                                                        | Name                                                         | Description                                                  |
| ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |
| <a id='MAV_BATTERY_FAULT_DEEP_DISCHARGE'></a>1               | [MAV_BATTERY_FAULT_DEEP_DISCHARGE](#MAV_BATTERY_FAULT_DEEP_DISCHARGE) | 电池深度放电。                                               |
| <a id='MAV_BATTERY_FAULT_SPIKES'></a>2                       | [MAV_BATTERY_FAULT_SPIKES](#MAV_BATTERY_FAULT_SPIKES)        | 电压峰值。                                                   |
| <a id='MAV_BATTERY_FAULT_CELL_FAIL'></a>4                    | [MAV_BATTERY_FAULT_CELL_FAIL](#MAV_BATTERY_FAULT_CELL_FAIL)  | 一个或多个电池失效。电池也应报告 [MAV_BATTERY_CHARGE_STATE_FAILE]（#MAV_BATTERY_CHARGE_STATE_FAILE）（并且不应使用）。 |
| <a id='MAV_BATTERY_FAULT_OVER_CURRENT'></a>8                 | [MAV_BATTERY_FAULT_OVER_CURRENT](#MAV_BATTERY_FAULT_OVER_CURRENT) | 过流故障。                                                   |
| <a id='MAV_BATTERY_FAULT_OVER_TEMPERATURE'></a>16            | [MAV_BATTERY_FAULT_OVER_TEMPERATURE](#MAV_BATTERY_FAULT_OVER_TEMPERATURE) | 过热故障。                                                   |
| <a id='MAV_BATTERY_FAULT_UNDER_TEMPERATURE'></a>32           | [MAV_BATTERY_FAULT_UNDER_TEMPERATURE](#MAV_BATTERY_FAULT_UNDER_TEMPERATURE) | 温度过低故障。                                               |
| <a id='MAV_BATTERY_FAULT_INCOMPATIBLE_VOLTAGE'></a>64        | [MAV_BATTERY_FAULT_INCOMPATIBLE_VOLTAGE](#MAV_BATTERY_FAULT_INCOMPATIBLE_VOLTAGE) | 车辆电压与此电池不兼容（同一电源轨上的电池应具有相似的电压）。 |
| <a id='MAV_BATTERY_FAULT_INCOMPATIBLE_FIRMWARE'></a>128      | [MAV_BATTERY_FAULT_INCOMPATIBLE_FIRMWARE](#MAV_BATTERY_FAULT_INCOMPATIBLE_FIRMWARE) | 电池固件与当前的自动驾驶仪固件不兼容。                       |
| <a id='BATTERY_FAULT_INCOMPATIBLE_CELLS_CONFIGURATION'></a>256 | [BATTERY_FAULT_INCOMPATIBLE_CELLS_CONFIGURATION](#BATTERY_FAULT_INCOMPATIBLE_CELLS_CONFIGURATION) | 由于电池配置（如 5s1p 而车辆要求 6s），电池不兼容。          |

### MAV_GENERATOR_STATUS_FLAG 

(位掩码）报告发电机状态/故障情况的标志（用于 [GENERATOR_STATUS](#GENERATOR_STATUS)）。请注意，"故障 "是指导致发电机发生故障的情况。警告是下次使用前需要注意的情况（表示系统运行不正常）。

| Value                                                        | Name                                                         | Description                                                  |
| ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |
| <a id='MAV_GENERATOR_STATUS_FLAG_OFF'></a>1                  | [MAV_GENERATOR_STATUS_FLAG_OFF](#MAV_GENERATOR_STATUS_FLAG_OFF) | 发电机已关闭。                                               |
| <a id='MAV_GENERATOR_STATUS_FLAG_READY'></a>2                | [MAV_GENERATOR_STATUS_FLAG_READY](#MAV_GENERATOR_STATUS_FLAG_READY) | 发电机已准备好开始发电。                                     |
| <a id='MAV_GENERATOR_STATUS_FLAG_GENERATING'></a>4           | [MAV_GENERATOR_STATUS_FLAG_GENERATING](#MAV_GENERATOR_STATUS_FLAG_GENERATING) | 发电机正在发电。                                             |
| <a id='MAV_GENERATOR_STATUS_FLAG_CHARGING'></a>8             | [MAV_GENERATOR_STATUS_FLAG_CHARGING](#MAV_GENERATOR_STATUS_FLAG_CHARGING) | 发电机正在为蓄电池充电（产生足够的电能为蓄电池充电并提供负载）。 |
| <a id='MAV_GENERATOR_STATUS_FLAG_REDUCED_POWER'></a>16       | [MAV_GENERATOR_STATUS_FLAG_REDUCED_POWER](#MAV_GENERATOR_STATUS_FLAG_REDUCED_POWER) | 发电机以降低的最大功率运行。                                 |
| <a id='MAV_GENERATOR_STATUS_FLAG_MAXPOWER'></a>32            | [MAV_GENERATOR_STATUS_FLAG_MAXPOWER](#MAV_GENERATOR_STATUS_FLAG_MAXPOWER) | 发电机提供最大输出功率。                                     |
| <a id='MAV_GENERATOR_STATUS_FLAG_OVERTEMP_WARNING'></a>64    | [MAV_GENERATOR_STATUS_FLAG_OVERTEMP_WARNING](#MAV_GENERATOR_STATUS_FLAG_OVERTEMP_WARNING) | 发电机接近最高工作温度，冷却不足。                           |
| <a id='MAV_GENERATOR_STATUS_FLAG_OVERTEMP_FAULT'></a>128     | [MAV_GENERATOR_STATUS_FLAG_OVERTEMP_FAULT](#MAV_GENERATOR_STATUS_FLAG_OVERTEMP_FAULT) | 发电机达到最高工作温度后停机。                               |
| <a id='MAV_GENERATOR_STATUS_FLAG_ELECTRONICS_OVERTEMP_WARNING'></a>256 | [MAV_GENERATOR_STATUS_FLAG_ELECTRONICS_OVERTEMP_WARNING](#MAV_GENERATOR_STATUS_FLAG_ELECTRONICS_OVERTEMP_WARNING) | 电力电子设备接近最高工作温度，冷却不足。                     |
| <a id='MAV_GENERATOR_STATUS_FLAG_ELECTRONICS_OVERTEMP_FAULT'></a>512 | [MAV_GENERATOR_STATUS_FLAG_ELECTRONICS_OVERTEMP_FAULT](#MAV_GENERATOR_STATUS_FLAG_ELECTRONICS_OVERTEMP_FAULT) | 电力电子设备达到最高工作温度并关闭。                         |
| <a id='MAV_GENERATOR_STATUS_FLAG_ELECTRONICS_FAULT'></a>1024 | [MAV_GENERATOR_STATUS_FLAG_ELECTRONICS_FAULT](#MAV_GENERATOR_STATUS_FLAG_ELECTRONICS_FAULT) | 电力电子设备出现故障并关闭。                                 |
| <a id='MAV_GENERATOR_STATUS_FLAG_POWERSOURCE_FAULT'></a>2048 | [MAV_GENERATOR_STATUS_FLAG_POWERSOURCE_FAULT](#MAV_GENERATOR_STATUS_FLAG_POWERSOURCE_FAULT) | 为发电机供电的动力源出现故障，例如机械发电机停止工作、系绳不再提供动力、太阳能电池处于阴暗状态、氢气反应不再发生。 |
| <a id='MAV_GENERATOR_STATUS_FLAG_COMMUNICATION_WARNING'></a>4096 | [MAV_GENERATOR_STATUS_FLAG_COMMUNICATION_WARNING](#MAV_GENERATOR_STATUS_FLAG_COMMUNICATION_WARNING) | 发电机控制器出现通信问题。                                   |
| <a id='MAV_GENERATOR_STATUS_FLAG_COOLING_WARNING'></a>8192   | [MAV_GENERATOR_STATUS_FLAG_COOLING_WARNING](#MAV_GENERATOR_STATUS_FLAG_COOLING_WARNING) | 电力电子或发电机冷却系统错误。                               |
| <a id='MAV_GENERATOR_STATUS_FLAG_POWER_RAIL_FAULT'></a>16384 | [MAV_GENERATOR_STATUS_FLAG_POWER_RAIL_FAULT](#MAV_GENERATOR_STATUS_FLAG_POWER_RAIL_FAULT) | 发电机控制器电源轨出现故障。                                 |
| <a id='MAV_GENERATOR_STATUS_FLAG_OVERCURRENT_FAULT'></a>32768 | [MAV_GENERATOR_STATUS_FLAG_OVERCURRENT_FAULT](#MAV_GENERATOR_STATUS_FLAG_OVERCURRENT_FAULT) | 发电机控制器超过过流阈值并关闭，以防止损坏。                 |
| <a id='MAV_GENERATOR_STATUS_FLAG_BATTERY_OVERCHARGE_CURRENT_FAULT'></a>65536 | [MAV_GENERATOR_STATUS_FLAG_BATTERY_OVERCHARGE_CURRENT_FAULT](#MAV_GENERATOR_STATUS_FLAG_BATTERY_OVERCHARGE_CURRENT_FAULT) | 发电机控制器检测到进入蓄电池的电流过大，为防止蓄电池损坏而关闭。 |
| <a id='MAV_GENERATOR_STATUS_FLAG_OVERVOLTAGE_FAULT'></a>131072 | [MAV_GENERATOR_STATUS_FLAG_OVERVOLTAGE_FAULT](#MAV_GENERATOR_STATUS_FLAG_OVERVOLTAGE_FAULT) | 发电机控制器超过过压阈值，为防止超过额定电压而关闭。         |
| <a id='MAV_GENERATOR_STATUS_FLAG_BATTERY_UNDERVOLT_FAULT'></a>262144 | [MAV_GENERATOR_STATUS_FLAG_BATTERY_UNDERVOLT_FAULT](#MAV_GENERATOR_STATUS_FLAG_BATTERY_UNDERVOLT_FAULT) | 蓄电池电压不足（发电机无法启动）。                           |
| <a id='MAV_GENERATOR_STATUS_FLAG_START_INHIBITED'></a>524288 | [MAV_GENERATOR_STATUS_FLAG_START_INHIBITED](#MAV_GENERATOR_STATUS_FLAG_START_INHIBITED) | 例如，通过安全开关抑制发电机启动。                           |
| <a id='MAV_GENERATOR_STATUS_FLAG_MAINTENANCE_REQUIRED'></a>1048576 | [MAV_GENERATOR_STATUS_FLAG_MAINTENANCE_REQUIRED](#MAV_GENERATOR_STATUS_FLAG_MAINTENANCE_REQUIRED) | 发电机需要维护。                                             |
| <a id='MAV_GENERATOR_STATUS_FLAG_WARMING_UP'></a>2097152     | [MAV_GENERATOR_STATUS_FLAG_WARMING_UP](#MAV_GENERATOR_STATUS_FLAG_WARMING_UP) | 发电机尚未准备好发电。                                       |
| <a id='MAV_GENERATOR_STATUS_FLAG_IDLE'></a>4194304           | [MAV_GENERATOR_STATUS_FLAG_IDLE](#MAV_GENERATOR_STATUS_FLAG_IDLE) | 发电机处于闲置状态。                                         |

### MAV_VTOL_STATE 

VTOL 状态枚举

| Value                                         | Name                                                         | Description                         |
| --------------------------------------------- | ------------------------------------------------------------ | ----------------------------------- |
| <a id='MAV_VTOL_STATE_UNDEFINED'></a>0        | [MAV_VTOL_STATE_UNDEFINED](#MAV_VTOL_STATE_UNDEFINED)        | 飞行器未配置为 VTOL                 |
| <a id='MAV_VTOL_STATE_TRANSITION_TO_FW'></a>1 | [MAV_VTOL_STATE_TRANSITION_TO_FW](#MAV_VTOL_STATE_TRANSITION_TO_FW) | VTOL 正从多旋翼飞机向固定翼飞机过渡 |
| <a id='MAV_VTOL_STATE_TRANSITION_TO_MC'></a>2 | [MAV_VTOL_STATE_TRANSITION_TO_MC](#MAV_VTOL_STATE_TRANSITION_TO_MC) | VTOL 正从固定翼飞机向多旋翼飞机过渡 |
| <a id='MAV_VTOL_STATE_MC'></a>3               | [MAV_VTOL_STATE_MC](#MAV_VTOL_STATE_MC)                      | VTOL 处于多旋翼状态                 |
| <a id='MAV_VTOL_STATE_FW'></a>4               | [MAV_VTOL_STATE_FW](#MAV_VTOL_STATE_FW)                      | VTOL 处于固定翼状态                 |

### MAV_LANDED_STATE 

落地探测器状态枚举

| Value                                    | Name                                                      | Description              |
| ---------------------------------------- | --------------------------------------------------------- | ------------------------ |
| <a id='MAV_LANDED_STATE_UNDEFINED'></a>0 | [MAV_LANDED_STATE_UNDEFINED](#MAV_LANDED_STATE_UNDEFINED) | 飞行器着陆状态未知       |
| <a id='MAV_LANDED_STATE_ON_GROUND'></a>1 | [MAV_LANDED_STATE_ON_GROUND](#MAV_LANDED_STATE_ON_GROUND) | 无人飞行器已着陆（地面） |
| <a id='MAV_LANDED_STATE_IN_AIR'></a>2    | [MAV_LANDED_STATE_IN_AIR](#MAV_LANDED_STATE_IN_AIR)       | 飞行器在空中             |
| <a id='MAV_LANDED_STATE_TAKEOFF'></a>3   | [MAV_LANDED_STATE_TAKEOFF](#MAV_LANDED_STATE_TAKEOFF)     | 飞行器正在起飞           |
| <a id='MAV_LANDED_STATE_LANDING'></a>4   | [MAV_LANDED_STATE_LANDING](#MAV_LANDED_STATE_LANDING)     | 飞行器正在着陆           |

### ADSB_ALTITUDE_TYPE 

ADSB 高度计类型枚举

| Value                                         | Name                                                         | Description                           |
| --------------------------------------------- | ------------------------------------------------------------ | ------------------------------------- |
| <a id='ADSB_ALTITUDE_TYPE_PRESSURE_QNH'></a>0 | [ADSB_ALTITUDE_TYPE_PRESSURE_QNH](#ADSB_ALTITUDE_TYPE_PRESSURE_QNH) | 使用 QNH 参考从 Baro 信号源报告的高度 |
| <a id='ADSB_ALTITUDE_TYPE_GEOMETRIC'></a>1    | [ADSB_ALTITUDE_TYPE_GEOMETRIC](#ADSB_ALTITUDE_TYPE_GEOMETRIC) | 全球导航卫星系统信号源报告的高度      |

### ADSB_EMITTER_TYPE 

发出应答器信号的车辆类型的 ADSB 分类

| Value                                              | Name                                                         | Description |
| -------------------------------------------------- | ------------------------------------------------------------ | ----------- |
| <a id='ADSB_EMITTER_TYPE_NO_INFO'></a>0            | [ADSB_EMITTER_TYPE_NO_INFO](#ADSB_EMITTER_TYPE_NO_INFO)      |             |
| <a id='ADSB_EMITTER_TYPE_LIGHT'></a>1              | [ADSB_EMITTER_TYPE_LIGHT](#ADSB_EMITTER_TYPE_LIGHT)          |             |
| <a id='ADSB_EMITTER_TYPE_SMALL'></a>2              | [ADSB_EMITTER_TYPE_SMALL](#ADSB_EMITTER_TYPE_SMALL)          |             |
| <a id='ADSB_EMITTER_TYPE_LARGE'></a>3              | [ADSB_EMITTER_TYPE_LARGE](#ADSB_EMITTER_TYPE_LARGE)          |             |
| <a id='ADSB_EMITTER_TYPE_HIGH_VORTEX_LARGE'></a>4  | [ADSB_EMITTER_TYPE_HIGH_VORTEX_LARGE](#ADSB_EMITTER_TYPE_HIGH_VORTEX_LARGE) |             |
| <a id='ADSB_EMITTER_TYPE_HEAVY'></a>5              | [ADSB_EMITTER_TYPE_HEAVY](#ADSB_EMITTER_TYPE_HEAVY)          |             |
| <a id='ADSB_EMITTER_TYPE_HIGHLY_MANUV'></a>6       | [ADSB_EMITTER_TYPE_HIGHLY_MANUV](#ADSB_EMITTER_TYPE_HIGHLY_MANUV) |             |
| <a id='ADSB_EMITTER_TYPE_ROTOCRAFT'></a>7          | [ADSB_EMITTER_TYPE_ROTOCRAFT](#ADSB_EMITTER_TYPE_ROTOCRAFT)  |             |
| <a id='ADSB_EMITTER_TYPE_UNASSIGNED'></a>8         | [ADSB_EMITTER_TYPE_UNASSIGNED](#ADSB_EMITTER_TYPE_UNASSIGNED) |             |
| <a id='ADSB_EMITTER_TYPE_GLIDER'></a>9             | [ADSB_EMITTER_TYPE_GLIDER](#ADSB_EMITTER_TYPE_GLIDER)        |             |
| <a id='ADSB_EMITTER_TYPE_LIGHTER_AIR'></a>10       | [ADSB_EMITTER_TYPE_LIGHTER_AIR](#ADSB_EMITTER_TYPE_LIGHTER_AIR) |             |
| <a id='ADSB_EMITTER_TYPE_PARACHUTE'></a>11         | [ADSB_EMITTER_TYPE_PARACHUTE](#ADSB_EMITTER_TYPE_PARACHUTE)  |             |
| <a id='ADSB_EMITTER_TYPE_ULTRA_LIGHT'></a>12       | [ADSB_EMITTER_TYPE_ULTRA_LIGHT](#ADSB_EMITTER_TYPE_ULTRA_LIGHT) |             |
| <a id='ADSB_EMITTER_TYPE_UNASSIGNED2'></a>13       | [ADSB_EMITTER_TYPE_UNASSIGNED2](#ADSB_EMITTER_TYPE_UNASSIGNED2) |             |
| <a id='ADSB_EMITTER_TYPE_UAV'></a>14               | [ADSB_EMITTER_TYPE_UAV](#ADSB_EMITTER_TYPE_UAV)              |             |
| <a id='ADSB_EMITTER_TYPE_SPACE'></a>15             | [ADSB_EMITTER_TYPE_SPACE](#ADSB_EMITTER_TYPE_SPACE)          |             |
| <a id='ADSB_EMITTER_TYPE_UNASSGINED3'></a>16       | [ADSB_EMITTER_TYPE_UNASSGINED3](#ADSB_EMITTER_TYPE_UNASSGINED3) |             |
| <a id='ADSB_EMITTER_TYPE_EMERGENCY_SURFACE'></a>17 | [ADSB_EMITTER_TYPE_EMERGENCY_SURFACE](#ADSB_EMITTER_TYPE_EMERGENCY_SURFACE) |             |
| <a id='ADSB_EMITTER_TYPE_SERVICE_SURFACE'></a>18   | [ADSB_EMITTER_TYPE_SERVICE_SURFACE](#ADSB_EMITTER_TYPE_SERVICE_SURFACE) |             |
| <a id='ADSB_EMITTER_TYPE_POINT_OBSTACLE'></a>19    | [ADSB_EMITTER_TYPE_POINT_OBSTACLE](#ADSB_EMITTER_TYPE_POINT_OBSTACLE) |             |

### ADSB_FLAGS 

(位掩码） 这些标志指示各数据源的数据有效性等状态。设置 = 数据有效

| Value                                              | Name                                                         | Description |
| -------------------------------------------------- | ------------------------------------------------------------ | ----------- |
| <a id='ADSB_FLAGS_VALID_COORDS'></a>1              | [ADSB_FLAGS_VALID_COORDS](#ADSB_FLAGS_VALID_COORDS)          |             |
| <a id='ADSB_FLAGS_VALID_ALTITUDE'></a>2            | [ADSB_FLAGS_VALID_ALTITUDE](#ADSB_FLAGS_VALID_ALTITUDE)      |             |
| <a id='ADSB_FLAGS_VALID_HEADING'></a>4             | [ADSB_FLAGS_VALID_HEADING](#ADSB_FLAGS_VALID_HEADING)        |             |
| <a id='ADSB_FLAGS_VALID_VELOCITY'></a>8            | [ADSB_FLAGS_VALID_VELOCITY](#ADSB_FLAGS_VALID_VELOCITY)      |             |
| <a id='ADSB_FLAGS_VALID_CALLSIGN'></a>16           | [ADSB_FLAGS_VALID_CALLSIGN](#ADSB_FLAGS_VALID_CALLSIGN)      |             |
| <a id='ADSB_FLAGS_VALID_SQUAWK'></a>32             | [ADSB_FLAGS_VALID_SQUAWK](#ADSB_FLAGS_VALID_SQUAWK)          |             |
| <a id='ADSB_FLAGS_SIMULATED'></a>64                | [ADSB_FLAGS_SIMULATED](#ADSB_FLAGS_SIMULATED)                |             |
| <a id='ADSB_FLAGS_VERTICAL_VELOCITY_VALID'></a>128 | [ADSB_FLAGS_VERTICAL_VELOCITY_VALID](#ADSB_FLAGS_VERTICAL_VELOCITY_VALID) |             |
| <a id='ADSB_FLAGS_BARO_VALID'></a>256              | [ADSB_FLAGS_BARO_VALID](#ADSB_FLAGS_BARO_VALID)              |             |
| <a id='ADSB_FLAGS_SOURCE_UAT'></a>32768            | [ADSB_FLAGS_SOURCE_UAT](#ADSB_FLAGS_SOURCE_UAT)              |             |

### MAV_DO_REPOSITION_FLAGS 

（位掩码） 位图，用于显示 [MAV_CMD_DO_REPOSITION](#MAV_CMD_DO_REPOSITION)

| Value                                             | Name                                                         | Description                                          |
| ------------------------------------------------- | ------------------------------------------------------------ | ---------------------------------------------------- |
| <a id='MAV_DO_REPOSITION_FLAGS_CHANGE_MODE'></a>1 | [MAV_DO_REPOSITION_FLAGS_CHANGE_MODE](#MAV_DO_REPOSITION_FLAGS_CHANGE_MODE) | 飞机应立即过渡到制导状态。这不应设置为 "跟随我 "应用 |

### SPEED_TYPE 

 [MAV_CMD_DO_CHANGE_SPEED](#MAV_CMD_DO_CHANGE_SPEED)中使用的速度设定点类型

| Value                                  | Name                                                  | Description |
| -------------------------------------- | ----------------------------------------------------- | ----------- |
| <a id='SPEED_TYPE_AIRSPEED'></a>0      | [SPEED_TYPE_AIRSPEED](#SPEED_TYPE_AIRSPEED)           | 飞行速度    |
| <a id='SPEED_TYPE_GROUNDSPEED'></a>1   | [SPEED_TYPE_GROUNDSPEED](#SPEED_TYPE_GROUNDSPEED)     | 地面速度    |
| <a id='SPEED_TYPE_CLIMB_SPEED'></a>2   | [SPEED_TYPE_CLIMB_SPEED](#SPEED_TYPE_CLIMB_SPEED)     | 爬升速度    |
| <a id='SPEED_TYPE_DESCENT_SPEED'></a>3 | [SPEED_TYPE_DESCENT_SPEED](#SPEED_TYPE_DESCENT_SPEED) | 下降速度    |

### ESTIMATOR_STATUS_FLAGS 

(位掩码） [ESTIMATOR_STATUS]（#ESTIMATOR_STATUS）报文中的标记

| Value                                        | Name                                                         | Description                                                  |
| -------------------------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| <a id='ESTIMATOR_ATTITUDE'></a>1             | [ESTIMATOR_ATTITUDE](#ESTIMATOR_ATTITUDE)                    | 如果态度估计良好，则为真                                     |
| <a id='ESTIMATOR_VELOCITY_HORIZ'></a>2       | [ESTIMATOR_VELOCITY_HORIZ](#ESTIMATOR_VELOCITY_HORIZ)        | 如果水平速度估计良好，则为真                                 |
| <a id='ESTIMATOR_VELOCITY_VERT'></a>4        | [ESTIMATOR_VELOCITY_VERT](#ESTIMATOR_VELOCITY_VERT)          | 如果垂直速度估计良好，则为 True                              |
| <a id='ESTIMATOR_POS_HORIZ_REL'></a>8        | [ESTIMATOR_POS_HORIZ_REL](#ESTIMATOR_POS_HORIZ_REL)          | 如果水平位置（相对）估计良好，则为 True                      |
| <a id='ESTIMATOR_POS_HORIZ_ABS'></a>16       | [ESTIMATOR_POS_HORIZ_ABS](#ESTIMATOR_POS_HORIZ_ABS)          | 如果水平位置（绝对值）估计良好，则为 True                    |
| <a id='ESTIMATOR_POS_VERT_ABS'></a>32        | [ESTIMATOR_POS_VERT_ABS](#ESTIMATOR_POS_VERT_ABS)            | 如果垂直位置（绝对值）估计良好，则为 True                    |
| <a id='ESTIMATOR_POS_VERT_AGL'></a>64        | [ESTIMATOR_POS_VERT_AGL](#ESTIMATOR_POS_VERT_AGL)            | 如果垂直位置（地面以上）估计良好，则为 True                  |
| <a id='ESTIMATOR_CONST_POS_MODE'></a>128     | [ESTIMATOR_CONST_POS_MODE](#ESTIMATOR_CONST_POS_MODE)        | 如果 EKF 处于恒定位置模式，且不使用外部测量（如 GPS 或光流），则为 True |
| <a id='ESTIMATOR_PRED_POS_HORIZ_REL'></a>256 | [ESTIMATOR_PRED_POS_HORIZ_REL](#ESTIMATOR_PRED_POS_HORIZ_REL) | 如果 EKF 有足够的数据，可以进入提供（相对）位置估计值的模式，则为真 |
| <a id='ESTIMATOR_PRED_POS_HORIZ_ABS'></a>512 | [ESTIMATOR_PRED_POS_HORIZ_ABS](#ESTIMATOR_PRED_POS_HORIZ_ABS) | 如果 EKF 有足够的数据可以进入提供（绝对）位置估计值的模式，则为真 |
| <a id='ESTIMATOR_GPS_GLITCH'></a>1024        | [ESTIMATOR_GPS_GLITCH](#ESTIMATOR_GPS_GLITCH)                | 如果 EKF 检测到 GPS 故障，则为 True                          |
| <a id='ESTIMATOR_ACCEL_ERROR'></a>2048       | [ESTIMATOR_ACCEL_ERROR](#ESTIMATOR_ACCEL_ERROR)              | 如果 EKF 检测到不良加速度计数据，则为 True                   |

### MOTOR_TEST_ORDER 

使用 [MAV_CMD_DO_MOTOR_TEST](#MAV_CMD_DO_MOTOR_TEST) 时测试电机的顺序。

| Value                                   | Name                                                    | Description                                        |
| --------------------------------------- | ------------------------------------------------------- | -------------------------------------------------- |
| <a id='MOTOR_TEST_ORDER_DEFAULT'></a>0  | [MOTOR_TEST_ORDER_DEFAULT](#MOTOR_TEST_ORDER_DEFAULT)   | 默认自动驾驶仪电机测试方法。                       |
| <a id='MOTOR_TEST_ORDER_SEQUENCE'></a>1 | [MOTOR_TEST_ORDER_SEQUENCE](#MOTOR_TEST_ORDER_SEQUENCE) | 在预定义的车辆特定序列中，电机编号被指定为其索引。 |
| <a id='MOTOR_TEST_ORDER_BOARD'></a>2    | [MOTOR_TEST_ORDER_BOARD](#MOTOR_TEST_ORDER_BOARD)       | 电机编号指定为电路板上标注的输出。                 |

### MOTOR_TEST_THROTTLE_TYPE 

定义在 [MAV_CMD_DO_MOTOR_TEST]（#MAV_CMD_DO_MOTOR_TEST）中如何表示节流阀值。

| Value                                     | Name                                                        | Description                                  |
| ----------------------------------------- | ----------------------------------------------------------- | -------------------------------------------- |
| <a id='MOTOR_TEST_THROTTLE_PERCENT'></a>0 | [MOTOR_TEST_THROTTLE_PERCENT](#MOTOR_TEST_THROTTLE_PERCENT) | 节气门百分比（0 ~ 100）                      |
| <a id='MOTOR_TEST_THROTTLE_PWM'></a>1     | [MOTOR_TEST_THROTTLE_PWM](#MOTOR_TEST_THROTTLE_PWM)         | 节流为绝对 PWM 值（通常在 1000~2000 之间）。 |
| <a id='MOTOR_TEST_THROTTLE_PILOT'></a>2   | [MOTOR_TEST_THROTTLE_PILOT](#MOTOR_TEST_THROTTLE_PILOT)     | 飞行员发射机的油门直通。                     |
| <a id='MOTOR_TEST_COMPASS_CAL'></a>3      | [MOTOR_TEST_COMPASS_CAL](#MOTOR_TEST_COMPASS_CAL)           | 每个电机罗盘校准测试。                       |

### GPS_INPUT_IGNORE_FLAGS 

(Bitmask) 

| Value                                                    | Name                                                         | Description                  |
| -------------------------------------------------------- | ------------------------------------------------------------ | ---------------------------- |
| <a id='GPS_INPUT_IGNORE_FLAG_ALT'></a>1                  | [GPS_INPUT_IGNORE_FLAG_ALT](#GPS_INPUT_IGNORE_FLAG_ALT)      | 忽略高度区域                 |
| <a id='GPS_INPUT_IGNORE_FLAG_HDOP'></a>2                 | [GPS_INPUT_IGNORE_FLAG_HDOP](#GPS_INPUT_IGNORE_FLAG_HDOP)    | 忽略 hdop 字段               |
| <a id='GPS_INPUT_IGNORE_FLAG_VDOP'></a>4                 | [GPS_INPUT_IGNORE_FLAG_VDOP](#GPS_INPUT_IGNORE_FLAG_VDOP)    | 忽略 vdop 字段               |
| <a id='GPS_INPUT_IGNORE_FLAG_VEL_HORIZ'></a>8            | [GPS_INPUT_IGNORE_FLAG_VEL_HORIZ](#GPS_INPUT_IGNORE_FLAG_VEL_HORIZ) | 不考虑水平速度场（vn 和 ve） |
| <a id='GPS_INPUT_IGNORE_FLAG_VEL_VERT'></a>16            | [GPS_INPUT_IGNORE_FLAG_VEL_VERT](#GPS_INPUT_IGNORE_FLAG_VEL_VERT) | 忽略垂直速度场 (vd)          |
| <a id='GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY'></a>32      | [GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY](#GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY) | 忽略速度精度字段             |
| <a id='GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY'></a>64 | [GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY](#GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY) | 忽略水平精度区域             |
| <a id='GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY'></a>128  | [GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY](#GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY) | 忽略垂直精度区域             |

### MAV_COLLISION_ACTION 

飞机为避免碰撞可能采取的行动。

| Value                                                 | Name                                                         | Description                  |
| ----------------------------------------------------- | ------------------------------------------------------------ | ---------------------------- |
| <a id='MAV_COLLISION_ACTION_NONE'></a>0               | [MAV_COLLISION_ACTION_NONE](#MAV_COLLISION_ACTION_NONE)      | 忽略任何可能发生的碰撞       |
| <a id='MAV_COLLISION_ACTION_REPORT'></a>1             | [MAV_COLLISION_ACTION_REPORT](#MAV_COLLISION_ACTION_REPORT)  | 报告可能发生的碰撞           |
| <a id='MAV_COLLISION_ACTION_ASCEND_OR_DESCEND'></a>2  | [MAV_COLLISION_ACTION_ASCEND_OR_DESCEND](#MAV_COLLISION_ACTION_ASCEND_OR_DESCEND) | 上升或下降以避免威胁         |
| <a id='MAV_COLLISION_ACTION_MOVE_HORIZONTALLY'></a>3  | [MAV_COLLISION_ACTION_MOVE_HORIZONTALLY](#MAV_COLLISION_ACTION_MOVE_HORIZONTALLY) | 水平移动以避开威胁           |
| <a id='MAV_COLLISION_ACTION_MOVE_PERPENDICULAR'></a>4 | [MAV_COLLISION_ACTION_MOVE_PERPENDICULAR](#MAV_COLLISION_ACTION_MOVE_PERPENDICULAR) | 飞机垂直于碰撞的速度矢量移动 |
| <a id='MAV_COLLISION_ACTION_RTL'></a>5                | [MAV_COLLISION_ACTION_RTL](#MAV_COLLISION_ACTION_RTL)        | 飞机直接飞回发射点           |
| <a id='MAV_COLLISION_ACTION_HOVER'></a>6              | [MAV_COLLISION_ACTION_HOVER](#MAV_COLLISION_ACTION_HOVER)    | 飞机原地停止                 |

### MAV_COLLISION_THREAT_LEVEL 

飞机将面临这种威胁带来的危险。

| Value                                         | Name                                                         | Description                            |
| --------------------------------------------- | ------------------------------------------------------------ | -------------------------------------- |
| <a id='MAV_COLLISION_THREAT_LEVEL_NONE'></a>0 | [MAV_COLLISION_THREAT_LEVEL_NONE](#MAV_COLLISION_THREAT_LEVEL_NONE) | 不构成威胁                             |
| <a id='MAV_COLLISION_THREAT_LEVEL_LOW'></a>1  | [MAV_COLLISION_THREAT_LEVEL_LOW](#MAV_COLLISION_THREAT_LEVEL_LOW) | 克拉夫特对这一威胁略感担忧             |
| <a id='MAV_COLLISION_THREAT_LEVEL_HIGH'></a>2 | [MAV_COLLISION_THREAT_LEVEL_HIGH](#MAV_COLLISION_THREAT_LEVEL_HIGH) | 飞行器惊慌失措，可能会采取行动避免威胁 |

### MAV_COLLISION_SRC 

有关此次碰撞的信息来源。

| Value                                                  | Name                                                         | Description                                     |
| ------------------------------------------------------ | ------------------------------------------------------------ | ----------------------------------------------- |
| <a id='MAV_COLLISION_SRC_ADSB'></a>0                   | [MAV_COLLISION_SRC_ADSB](#MAV_COLLISION_SRC_ADSB)            | ID 字段引用 [ADSB_VEHICLE](#ADSB_VEHICLE)数据包 |
| <a id='MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT'></a>1 | [MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT](#MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT) | ID 字段引用 MAVLink SRC ID                      |

### GPS_FIX_TYPE 

GPS 定位类型

| Value                                | Name                                              | Description            |
| ------------------------------------ | ------------------------------------------------- | ---------------------- |
| <a id='GPS_FIX_TYPE_NO_GPS'></a>0    | [GPS_FIX_TYPE_NO_GPS](#GPS_FIX_TYPE_NO_GPS)       | 未连接 GPS             |
| <a id='GPS_FIX_TYPE_NO_FIX'></a>1    | [GPS_FIX_TYPE_NO_FIX](#GPS_FIX_TYPE_NO_FIX)       | 无位置信息，已连接 GPS |
| <a id='GPS_FIX_TYPE_2D_FIX'></a>2    | [GPS_FIX_TYPE_2D_FIX](#GPS_FIX_TYPE_2D_FIX)       | 二维位置               |
| <a id='GPS_FIX_TYPE_3D_FIX'></a>3    | [GPS_FIX_TYPE_3D_FIX](#GPS_FIX_TYPE_3D_FIX)       | 三维位置               |
| <a id='GPS_FIX_TYPE_DGPS'></a>4      | [GPS_FIX_TYPE_DGPS](#GPS_FIX_TYPE_DGPS)           | DGPS/SBAS 辅助 3D 定位 |
| <a id='GPS_FIX_TYPE_RTK_FLOAT'></a>5 | [GPS_FIX_TYPE_RTK_FLOAT](#GPS_FIX_TYPE_RTK_FLOAT) | RTK 浮点、3D 定位      |
| <a id='GPS_FIX_TYPE_RTK_FIXED'></a>6 | [GPS_FIX_TYPE_RTK_FIXED](#GPS_FIX_TYPE_RTK_FIXED) | RTK 固定、3D 定位      |
| <a id='GPS_FIX_TYPE_STATIC'></a>7    | [GPS_FIX_TYPE_STATIC](#GPS_FIX_TYPE_STATIC)       | 静态固定，通常用于基站 |
| <a id='GPS_FIX_TYPE_PPP'></a>8       | [GPS_FIX_TYPE_PPP](#GPS_FIX_TYPE_PPP)             | PPP，3D 位置。         |

### RTK_BASELINE_COORDINATE_SYSTEM 

RTK GPS 基线坐标系，用于 RTK 校正

| Value                                             | Name                                                         | Description                    |
| ------------------------------------------------- | ------------------------------------------------------------ | ------------------------------ |
| <a id='RTK_BASELINE_COORDINATE_SYSTEM_ECEF'></a>0 | [RTK_BASELINE_COORDINATE_SYSTEM_ECEF](#RTK_BASELINE_COORDINATE_SYSTEM_ECEF) | 以地球为中心，固定在地球上     |
| <a id='RTK_BASELINE_COORDINATE_SYSTEM_NED'></a>1  | [RTK_BASELINE_COORDINATE_SYSTEM_NED](#RTK_BASELINE_COORDINATE_SYSTEM_NED) | RTK 基站居中、向北、向东、向下 |

### LANDING_TARGET_TYPE 

降落目标类型

| Value                                             | Name                                                         | Description                                                  |
| ------------------------------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| <a id='LANDING_TARGET_TYPE_LIGHT_BEACON'></a>0    | [LANDING_TARGET_TYPE_LIGHT_BEACON](#LANDING_TARGET_TYPE_LIGHT_BEACON) | 着陆目标由灯塔发出信号（例如：IR-LOCK）。                    |
| <a id='LANDING_TARGET_TYPE_RADIO_BEACON'></a>1    | [LANDING_TARGET_TYPE_RADIO_BEACON](#LANDING_TARGET_TYPE_RADIO_BEACON) | 通过无线电信标指示着陆目标（例如：ILS、NDB）                 |
| <a id='LANDING_TARGET_TYPE_VISION_FIDUCIAL'></a>2 | [LANDING_TARGET_TYPE_VISION_FIDUCIAL](#LANDING_TARGET_TYPE_VISION_FIDUCIAL) | 用靶标（例如：ARTag）表示着陆目标                            |
| <a id='LANDING_TARGET_TYPE_VISION_OTHER'></a>3    | [LANDING_TARGET_TYPE_VISION_OTHER](#LANDING_TARGET_TYPE_VISION_OTHER) | 着陆目标由预定义的视觉形状/特征表示（例如：X 标记、H 标记、正方形） |

### VTOL_TRANSITION_HEADING 

VTOL 过渡方向

| Value                                                 | Name                                                         | Description                                                  |
| ----------------------------------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| <a id='VTOL_TRANSITION_HEADING_VEHICLE_DEFAULT'></a>0 | [VTOL_TRANSITION_HEADING_VEHICLE_DEFAULT](#VTOL_TRANSITION_HEADING_VEHICLE_DEFAULT) | 遵守车辆的航向配置。                                         |
| <a id='VTOL_TRANSITION_HEADING_NEXT_WAYPOINT'></a>1   | [VTOL_TRANSITION_HEADING_NEXT_WAYPOINT](#VTOL_TRANSITION_HEADING_NEXT_WAYPOINT) | 使用指向下一个航点的航向。                                   |
| <a id='VTOL_TRANSITION_HEADING_TAKEOFF'></a>2         | [VTOL_TRANSITION_HEADING_TAKEOFF](#VTOL_TRANSITION_HEADING_TAKEOFF) | 起飞时使用航向（坐在地面上时）。                             |
| <a id='VTOL_TRANSITION_HEADING_SPECIFIED'></a>3       | [VTOL_TRANSITION_HEADING_SPECIFIED](#VTOL_TRANSITION_HEADING_SPECIFIED) | 使用参数 4 中指定的标题。                                    |
| <a id='VTOL_TRANSITION_HEADING_ANY'></a>4             | [VTOL_TRANSITION_HEADING_ANY](#VTOL_TRANSITION_HEADING_ANY)  | 在达到起飞高度时使用当前航向（在天气变化时可能会迎风飞行）。 |

### CAMERA_CAP_FLAGS 

(位图） 相机功能标志（位图）

| Value                                                        | Name                                                         | Description                                                  |
| ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |
| <a id='CAMERA_CAP_FLAGS_CAPTURE_VIDEO'></a>1                 | [CAMERA_CAP_FLAGS_CAPTURE_VIDEO](#CAMERA_CAP_FLAGS_CAPTURE_VIDEO) | 摄像头可录制视频                                             |
| <a id='CAMERA_CAP_FLAGS_CAPTURE_IMAGE'></a>2                 | [CAMERA_CAP_FLAGS_CAPTURE_IMAGE](#CAMERA_CAP_FLAGS_CAPTURE_IMAGE) | 摄像头能够拍摄图像                                           |
| <a id='CAMERA_CAP_FLAGS_HAS_MODES'></a>4                     | [CAMERA_CAP_FLAGS_HAS_MODES](#CAMERA_CAP_FLAGS_HAS_MODES)    | 相机具有独立的视频和图像/照片模式 ([MAV_CMD_SET_CAMERA_MODE](#MAV_CMD_SET_CAMERA_MODE)) |
| <a id='CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE'></a>8 | [CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE](#CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE) | 相机可在视频模式下拍摄图像                                   |
| <a id='CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE'></a>16 | [CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE](#CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE) | 相机可在照片/图像模式下拍摄视频                              |
| <a id='CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE'></a>32        | [CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE](#CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE) | 相机具有图像测量模式 ([MAV_CMD_SET_CAMERA_MODE](#MAV_CMD_SET_CAMERA_MODE)) |
| <a id='CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM'></a>64               | [CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM](#CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM) | 相机具有基本的变焦控制功能 ([MAV_CMD_SET_CAMERA_ZOOM](#MAV_CMD_SET_CAMERA_ZOOM)) |
| <a id='CAMERA_CAP_FLAGS_HAS_BASIC_FOCUS'></a>128             | [CAMERA_CAP_FLAGS_HAS_BASIC_FOCUS](#CAMERA_CAP_FLAGS_HAS_BASIC_FOCUS) | 相机具有基本的对焦控制功能 ([MAV_CMD_SET_CAMERA_FOCUS](#MAV_CMD_SET_CAMERA_FOCUS)) |
| <a id='CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM'></a>256            | [CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM](#CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM) | 摄像机具有视频流功能（通过 [MAV_CMD_REQUEST_MESSAGE](#MAV_CMD_REQUEST_MESSAGE)请求 [VIDEO_STREAM_INFORMATION](#VIDEO_STREAM_INFORMATION)，以获取视频流信息） |
| <a id='CAMERA_CAP_FLAGS_HAS_TRACKING_POINT'></a>512          | [CAMERA_CAP_FLAGS_HAS_TRACKING_POINT](#CAMERA_CAP_FLAGS_HAS_TRACKING_POINT) | 摄像机支持跟踪摄像机视图上的一个点。                         |
| <a id='CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE'></a>1024     | [CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE](#CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE) | 摄像头支持在摄像头视图上跟踪选择矩形。                       |
| <a id='CAMERA_CAP_FLAGS_HAS_TRACKING_GEO_STATUS'></a>2048    | [CAMERA_CAP_FLAGS_HAS_TRACKING_GEO_STATUS](#CAMERA_CAP_FLAGS_HAS_TRACKING_GEO_STATUS) | 摄像头支持跟踪地理位置 ([CAMERA_TRACKING_GEO_STATUS](#CAMERA_TRACKING_GEO_STATUS)). |

### VIDEO_STREAM_STATUS_FLAGS 

位屏蔽） 数据流状态标志（位图）

| Value                                           | Name                                                         | Description                  |
| ----------------------------------------------- | ------------------------------------------------------------ | ---------------------------- |
| <a id='VIDEO_STREAM_STATUS_FLAGS_RUNNING'></a>1 | [VIDEO_STREAM_STATUS_FLAGS_RUNNING](#VIDEO_STREAM_STATUS_FLAGS_RUNNING) | 数据流处于活动状态（运行中） |
| <a id='VIDEO_STREAM_STATUS_FLAGS_THERMAL'></a>2 | [VIDEO_STREAM_STATUS_FLAGS_THERMAL](#VIDEO_STREAM_STATUS_FLAGS_THERMAL) | 热成像流                     |

### VIDEO_STREAM_TYPE 

视频流类型

| Value                                        | Name                                                         | Description                                     |
| -------------------------------------------- | ------------------------------------------------------------ | ----------------------------------------------- |
| <a id='VIDEO_STREAM_TYPE_RTSP'></a>0         | [VIDEO_STREAM_TYPE_RTSP](#VIDEO_STREAM_TYPE_RTSP)            | 数据流为 RTSP                                   |
| <a id='VIDEO_STREAM_TYPE_RTPUDP'></a>1       | [VIDEO_STREAM_TYPE_RTPUDP](#VIDEO_STREAM_TYPE_RTPUDP)        | 数据流为 RTP UDP（URI 提供端口号）              |
| <a id='VIDEO_STREAM_TYPE_TCP_MPEG'></a>2     | [VIDEO_STREAM_TYPE_TCP_MPEG](#VIDEO_STREAM_TYPE_TCP_MPEG)    | 数据流为 TCP 上的 MPEG                          |
| <a id='VIDEO_STREAM_TYPE_MPEG_TS_H264'></a>3 | [VIDEO_STREAM_TYPE_MPEG_TS_H264](#VIDEO_STREAM_TYPE_MPEG_TS_H264) | 数据流为 MPEG TS 上的 h.264（URI 提供了端口号） |

### CAMERA_TRACKING_STATUS_FLAGS 

摄像机跟踪状态标志

| Value                                             | Name                                                         | Description            |
| ------------------------------------------------- | ------------------------------------------------------------ | ---------------------- |
| <a id='CAMERA_TRACKING_STATUS_FLAGS_IDLE'></a>0   | [CAMERA_TRACKING_STATUS_FLAGS_IDLE](#CAMERA_TRACKING_STATUS_FLAGS_IDLE) | 相机无法跟踪           |
| <a id='CAMERA_TRACKING_STATUS_FLAGS_ACTIVE'></a>1 | [CAMERA_TRACKING_STATUS_FLAGS_ACTIVE](#CAMERA_TRACKING_STATUS_FLAGS_ACTIVE) | 摄像头正在跟踪         |
| <a id='CAMERA_TRACKING_STATUS_FLAGS_ERROR'></a>2  | [CAMERA_TRACKING_STATUS_FLAGS_ERROR](#CAMERA_TRACKING_STATUS_FLAGS_ERROR) | 错误状态下的摄像机跟踪 |

### CAMERA_TRACKING_MODE 

摄像机跟踪模式

| Value                                        | Name                                                         | Description    |
| -------------------------------------------- | ------------------------------------------------------------ | -------------- |
| <a id='CAMERA_TRACKING_MODE_NONE'></a>0      | [CAMERA_TRACKING_MODE_NONE](#CAMERA_TRACKING_MODE_NONE)      | 不跟踪         |
| <a id='CAMERA_TRACKING_MODE_POINT'></a>1     | [CAMERA_TRACKING_MODE_POINT](#CAMERA_TRACKING_MODE_POINT)    | 目标为一个点   |
| <a id='CAMERA_TRACKING_MODE_RECTANGLE'></a>2 | [CAMERA_TRACKING_MODE_RECTANGLE](#CAMERA_TRACKING_MODE_RECTANGLE) | 目标是一个矩形 |

### CAMERA_TRACKING_TARGET_DATA 

(位掩码） 摄像机跟踪目标数据（显示图像中被跟踪目标的位置）

| Value                                               | Name                                                         | Description                      |
| --------------------------------------------------- | ------------------------------------------------------------ | -------------------------------- |
| <a id='CAMERA_TRACKING_TARGET_DATA_NONE'></a>0      | [CAMERA_TRACKING_TARGET_DATA_NONE](#CAMERA_TRACKING_TARGET_DATA_NONE) | 无目标数据                       |
| <a id='CAMERA_TRACKING_TARGET_DATA_EMBEDDED'></a>1  | [CAMERA_TRACKING_TARGET_DATA_EMBEDDED](#CAMERA_TRACKING_TARGET_DATA_EMBEDDED) | 嵌入图像数据的目标数据（专有）   |
| <a id='CAMERA_TRACKING_TARGET_DATA_RENDERED'></a>2  | [CAMERA_TRACKING_TARGET_DATA_RENDERED](#CAMERA_TRACKING_TARGET_DATA_RENDERED) | 在图像中呈现目标数据             |
| <a id='CAMERA_TRACKING_TARGET_DATA_IN_STATUS'></a>4 | [CAMERA_TRACKING_TARGET_DATA_IN_STATUS](#CAMERA_TRACKING_TARGET_DATA_IN_STATUS) | 状态信息中的目标数据（点或矩形） |

### CAMERA_ZOOM_TYPE 

MAV_CMD_SET_CAMERA_ZOOM](#MAV_CMD_SET_CAMERA_ZOOM) 的变焦类型

| Value                                  | Name                                                  | Description                                                  |
| -------------------------------------- | ----------------------------------------------------- | ------------------------------------------------------------ |
| <a id='ZOOM_TYPE_STEP'></a>0           | [ZOOM_TYPE_STEP](#ZOOM_TYPE_STEP)                     | 变焦一步增量（-1 表示广角，1 表示远摄）                      |
| <a id='ZOOM_TYPE_CONTINUOUS'></a>1     | [ZOOM_TYPE_CONTINUOUS](#ZOOM_TYPE_CONTINUOUS)         | 连续向上/向下变焦直至停止（-1 表示广角，1 表示远摄，0 表示停止变焦） |
| <a id='ZOOM_TYPE_RANGE'></a>2          | [ZOOM_TYPE_RANGE](#ZOOM_TYPE_RANGE)                   | 变焦值占整个相机范围的比例（0.0 至 100.0 之间的百分比值）    |
| <a id='ZOOM_TYPE_FOCAL_LENGTH'></a>3   | [ZOOM_TYPE_FOCAL_LENGTH](#ZOOM_TYPE_FOCAL_LENGTH)     | 以毫米为单位的变焦值/可变焦距。请注意，没有信息可用于获取摄像机的有效变焦范围，因此这种类型只能用于已知变焦范围的摄像机（这意味着不能在任意摄像机的 GCS 中可靠地使用这种类型）。 |
| <a id='ZOOM_TYPE_HORIZONTAL_FOV'></a>4 | [ZOOM_TYPE_HORIZONTAL_FOV](#ZOOM_TYPE_HORIZONTAL_FOV) | 缩放值为水平视场角（度）。                                   |

### SET_FOCUS_TYPE 

MAV_CMD_SET_CAMERA_FOCUS](#MAV_CMD_SET_CAMERA_FOCUS) 的对焦类型

| Value                                    | Name                                                      | Description                                                  |
| ---------------------------------------- | --------------------------------------------------------- | ------------------------------------------------------------ |
| <a id='FOCUS_TYPE_STEP'></a>0            | [FOCUS_TYPE_STEP](#FOCUS_TYPE_STEP)                       | 对焦一步增量（-1 表示向内对焦，1 表示向无限远处对焦）。      |
| <a id='FOCUS_TYPE_CONTINUOUS'></a>1      | [FOCUS_TYPE_CONTINUOUS](#FOCUS_TYPE_CONTINUOUS)           | 连续向上/向下对焦直至停止（-1 表示向内对焦，1 表示向无限远处对焦，0 表示停止对焦） |
| <a id='FOCUS_TYPE_RANGE'></a>2           | [FOCUS_TYPE_RANGE](#FOCUS_TYPE_RANGE)                     | 对焦值占整个相机对焦范围的比例（数值介于 0.0 和 100.0 之间） |
| <a id='FOCUS_TYPE_METERS'></a>3          | [FOCUS_TYPE_METERS](#FOCUS_TYPE_METERS)                   | 以米为单位的焦距值。请注意，没有信息可以获取摄像机的有效焦距范围，因此这种类型只能用于已知焦距范围的摄像机（这意味着不能可靠地用于任意摄像机的 GCS）。 |
| <a id='FOCUS_TYPE_AUTO'></a>4            | [FOCUS_TYPE_AUTO](#FOCUS_TYPE_AUTO)                       | 自动对焦。                                                   |
| <a id='FOCUS_TYPE_AUTO_SINGLE'></a>5     | [FOCUS_TYPE_AUTO_SINGLE](#FOCUS_TYPE_AUTO_SINGLE)         | 单次自动对焦。主要用于静态图像。通常缩写为 AF-S。            |
| <a id='FOCUS_TYPE_AUTO_CONTINUOUS'></a>6 | [FOCUS_TYPE_AUTO_CONTINUOUS](#FOCUS_TYPE_AUTO_CONTINUOUS) | 连续自动对焦。主要用于动态场景。简称 AF-C。                  |

### CAMERA_SOURCE 

MAV_CMD_SET_CAMERA_SOURCE](#MAV_CMD_SET_CAMERA_SOURCE) 的相机源

| Value                               | Name                                            | Description        |
| ----------------------------------- | ----------------------------------------------- | ------------------ |
| <a id='CAMERA_SOURCE_DEFAULT'></a>0 | [CAMERA_SOURCE_DEFAULT](#CAMERA_SOURCE_DEFAULT) | 默认摄像机信号源。 |
| <a id='CAMERA_SOURCE_RGB'></a>1     | [CAMERA_SOURCE_RGB](#CAMERA_SOURCE_RGB)         | RGB 摄像机信号源。 |
| <a id='CAMERA_SOURCE_IR'></a>2      | [CAMERA_SOURCE_IR](#CAMERA_SOURCE_IR)           | 红外摄像机光源。   |
| <a id='CAMERA_SOURCE_NDVI'></a>3    | [CAMERA_SOURCE_NDVI](#CAMERA_SOURCE_NDVI)       | NDVI 相机信号源。  |

### PARAM_ACK 

PARAM_EXT_SET](#PARAM_EXT_SET) 消息（或事务中的 [PARAM_SET](#PARAM_SET)）的结果。

| Value                                     | Name                                                        | Description                                                  |
| ----------------------------------------- | ----------------------------------------------------------- | ------------------------------------------------------------ |
| <a id='PARAM_ACK_ACCEPTED'></a>0          | [PARAM_ACK_ACCEPTED](#PARAM_ACK_ACCEPTED)                   | 参数值 ACCEPTED 和设置                                       |
| <a id='PARAM_ACK_VALUE_UNSUPPORTED'></a>1 | [PARAM_ACK_VALUE_UNSUPPORTED](#PARAM_ACK_VALUE_UNSUPPORTED) | 参数值未知/不支持                                            |
| <a id='PARAM_ACK_FAILED'></a>2            | [PARAM_ACK_FAILED](#PARAM_ACK_FAILED)                       | 参数设置失败                                                 |
| <a id='PARAM_ACK_IN_PROGRESS'></a>3       | [PARAM_ACK_IN_PROGRESS](#PARAM_ACK_IN_PROGRESS)             | 已收到参数值，但尚未设置/接受。操作完成后，将随即返回包含最终结果的 [PARAM_ACK_TRANSACTION](#PARAM_ACK_TRANSACTION) 或 [PARAM_EXT_ACK](#PARAM_EXT_ACK)。对于设置时间较长的参数，将立即返回，表明参数已收到，无需重新设置。 |

### CAMERA_MODE 

相机模式。

| Value                                  | Name                                                  | Description                                                  |
| -------------------------------------- | ----------------------------------------------------- | ------------------------------------------------------------ |
| <a id='CAMERA_MODE_IMAGE'></a>0        | [CAMERA_MODE_IMAGE](#CAMERA_MODE_IMAGE)               | 相机处于图像/照片捕捉模式。                                  |
| <a id='CAMERA_MODE_VIDEO'></a>1        | [CAMERA_MODE_VIDEO](#CAMERA_MODE_VIDEO)               | 相机处于视频拍摄模式。                                       |
| <a id='CAMERA_MODE_IMAGE_SURVEY'></a>2 | [CAMERA_MODE_IMAGE_SURVEY](#CAMERA_MODE_IMAGE_SURVEY) | 摄像机处于图像勘测捕捉模式。它允许相机控制器为勘测进行特定设置。 |

### MAV_ARM_AUTH_DENIED_REASON 

| Value                                                     | Name                                                         | Description                                                  |
| --------------------------------------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| <a id='MAV_ARM_AUTH_DENIED_REASON_GENERIC'></a>0          | [MAV_ARM_AUTH_DENIED_REASON_GENERIC](#MAV_ARM_AUTH_DENIED_REASON_GENERIC) | 没有具体原因                                                 |
| <a id='MAV_ARM_AUTH_DENIED_REASON_NONE'></a>1             | [MAV_ARM_AUTH_DENIED_REASON_NONE](#MAV_ARM_AUTH_DENIED_REASON_NONE) | 授权程序将以字符串形式向 GCS 发送错误信息                    |
| <a id='MAV_ARM_AUTH_DENIED_REASON_INVALID_WAYPOINT'></a>2 | [MAV_ARM_AUTH_DENIED_REASON_INVALID_WAYPOINT](#MAV_ARM_AUTH_DENIED_REASON_INVALID_WAYPOINT) | 至少有一个航点的值无效                                       |
| <a id='MAV_ARM_AUTH_DENIED_REASON_TIMEOUT'></a>3          | [MAV_ARM_AUTH_DENIED_REASON_TIMEOUT](#MAV_ARM_AUTH_DENIED_REASON_TIMEOUT) | 授权程序的超时（取决于网络的情况下）                         |
| <a id='MAV_ARM_AUTH_DENIED_REASON_AIRSPACE_IN_USE'></a>4  | [MAV_ARM_AUTH_DENIED_REASON_AIRSPACE_IN_USE](#MAV_ARM_AUTH_DENIED_REASON_AIRSPACE_IN_USE) | 另一飞行器正在使用的飞行任务空域，第二个结果参数可以是导致其被拒绝的航点 ID。 |
| <a id='MAV_ARM_AUTH_DENIED_REASON_BAD_WEATHER'></a>5      | [MAV_ARM_AUTH_DENIED_REASON_BAD_WEATHER](#MAV_ARM_AUTH_DENIED_REASON_BAD_WEATHER) | 天气不适合飞行                                               |

### RC_TYPE 

RC 型

| Value                               | Name                                            | Description   |
| ----------------------------------- | ----------------------------------------------- | ------------- |
| <a id='RC_TYPE_SPEKTRUM_DSM2'></a>0 | [RC_TYPE_SPEKTRUM_DSM2](#RC_TYPE_SPEKTRUM_DSM2) | Spektrum DSM2 |
| <a id='RC_TYPE_SPEKTRUM_DSMX'></a>1 | [RC_TYPE_SPEKTRUM_DSMX](#RC_TYPE_SPEKTRUM_DSMX) | Spektrum DSMX |

### POSITION_TARGET_TYPEMASK 

(位掩码） 用于指示车辆应忽略哪些维度的位图：0b0000000000000000 或 0b0000001000000000 表示不忽略任何设定点维度。如果设置了第 9 位，浮点数 afx afy afz 将被解释为力而不是加速度。

| Value                                                     | Name                                                         | Description    |
| --------------------------------------------------------- | ------------------------------------------------------------ | -------------- |
| <a id='POSITION_TARGET_TYPEMASK_X_IGNORE'></a>1           | [POSITION_TARGET_TYPEMASK_X_IGNORE](#POSITION_TARGET_TYPEMASK_X_IGNORE) | 忽略位置 x     |
| <a id='POSITION_TARGET_TYPEMASK_Y_IGNORE'></a>2           | [POSITION_TARGET_TYPEMASK_Y_IGNORE](#POSITION_TARGET_TYPEMASK_Y_IGNORE) | 忽略位置 y     |
| <a id='POSITION_TARGET_TYPEMASK_Z_IGNORE'></a>4           | [POSITION_TARGET_TYPEMASK_Z_IGNORE](#POSITION_TARGET_TYPEMASK_Z_IGNORE) | 忽略位置 z     |
| <a id='POSITION_TARGET_TYPEMASK_VX_IGNORE'></a>8          | [POSITION_TARGET_TYPEMASK_VX_IGNORE](#POSITION_TARGET_TYPEMASK_VX_IGNORE) | 忽略速度 x     |
| <a id='POSITION_TARGET_TYPEMASK_VY_IGNORE'></a>16         | [POSITION_TARGET_TYPEMASK_VY_IGNORE](#POSITION_TARGET_TYPEMASK_VY_IGNORE) | 忽略速度 y     |
| <a id='POSITION_TARGET_TYPEMASK_VZ_IGNORE'></a>32         | [POSITION_TARGET_TYPEMASK_VZ_IGNORE](#POSITION_TARGET_TYPEMASK_VZ_IGNORE) | 忽略速度 z     |
| <a id='POSITION_TARGET_TYPEMASK_AX_IGNORE'></a>64         | [POSITION_TARGET_TYPEMASK_AX_IGNORE](#POSITION_TARGET_TYPEMASK_AX_IGNORE) | 忽略加速度 x   |
| <a id='POSITION_TARGET_TYPEMASK_AY_IGNORE'></a>128        | [POSITION_TARGET_TYPEMASK_AY_IGNORE](#POSITION_TARGET_TYPEMASK_AY_IGNORE) | 忽略加速度 y   |
| <a id='POSITION_TARGET_TYPEMASK_AZ_IGNORE'></a>256        | [POSITION_TARGET_TYPEMASK_AZ_IGNORE](#POSITION_TARGET_TYPEMASK_AZ_IGNORE) | 忽略加速度 z   |
| <a id='POSITION_TARGET_TYPEMASK_FORCE_SET'></a>512        | [POSITION_TARGET_TYPEMASK_FORCE_SET](#POSITION_TARGET_TYPEMASK_FORCE_SET) | 用力代替加速度 |
| <a id='POSITION_TARGET_TYPEMASK_YAW_IGNORE'></a>1024      | [POSITION_TARGET_TYPEMASK_YAW_IGNORE](#POSITION_TARGET_TYPEMASK_YAW_IGNORE) | 忽略偏航       |
| <a id='POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE'></a>2048 | [POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE](#POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE) | 忽略偏航率     |

### ATTITUDE_TARGET_TYPEMASK 

(位掩码） 表示车辆应忽略哪些维度的位图：0b00000000 表示不忽略任何设定点维度。

| Value                                                        | Name                                                         | Description                      |
| ------------------------------------------------------------ | ------------------------------------------------------------ | -------------------------------- |
| <a id='ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE'></a>1 | [ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE](#ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE) | 忽略车身滚动率                   |
| <a id='ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE'></a>2 | [ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE](#ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE) | 忽略身体俯仰率                   |
| <a id='ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE'></a>4  | [ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE](#ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE) | 忽略机身偏航率                   |
| <a id='ATTITUDE_TARGET_TYPEMASK_THRUST_BODY_SET'></a>32      | [ATTITUDE_TARGET_TYPEMASK_THRUST_BODY_SET](#ATTITUDE_TARGET_TYPEMASK_THRUST_BODY_SET) | 使用 3D 机身推力设定点代替节流阀 |
| <a id='ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE'></a>64      | [ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE](#ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE) | 忽略节流阀                       |
| <a id='ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE'></a>128     | [ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE](#ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE) | 无视态度                         |

### UTM_FLIGHT_STATE 

无人机系统的空中状态。

| Value                                     | Name                                                      | Description                  |
| ----------------------------------------- | --------------------------------------------------------- | ---------------------------- |
| <a id='UTM_FLIGHT_STATE_UNKNOWN'></a>1    | [UTM_FLIGHT_STATE_UNKNOWN](#UTM_FLIGHT_STATE_UNKNOWN)     | 飞行状态无法确定。           |
| <a id='UTM_FLIGHT_STATE_GROUND'></a>2     | [UTM_FLIGHT_STATE_GROUND](#UTM_FLIGHT_STATE_GROUND)       | 地面无人机系统。             |
| <a id='UTM_FLIGHT_STATE_AIRBORNE'></a>3   | [UTM_FLIGHT_STATE_AIRBORNE](#UTM_FLIGHT_STATE_AIRBORNE)   | 无人机系统升空。             |
| <a id='UTM_FLIGHT_STATE_EMERGENCY'></a>16 | [UTM_FLIGHT_STATE_EMERGENCY](#UTM_FLIGHT_STATE_EMERGENCY) | 无人机系统处于紧急飞行状态。 |
| <a id='UTM_FLIGHT_STATE_NOCTRL'></a>32    | [UTM_FLIGHT_STATE_NOCTRL](#UTM_FLIGHT_STATE_NOCTRL)       | UAS 没有主动控制。           |

### UTM_DATA_AVAIL_FLAGS 

(位掩码） 全局位置报告的标志。

| Value                                                        | Name                                                         | Description                                        |
| ------------------------------------------------------------ | ------------------------------------------------------------ | -------------------------------------------------- |
| <a id='UTM_DATA_AVAIL_FLAGS_TIME_VALID'></a>1                | [UTM_DATA_AVAIL_FLAGS_TIME_VALID](#UTM_DATA_AVAIL_FLAGS_TIME_VALID) | 字段时间包含有效数据。                             |
| <a id='UTM_DATA_AVAIL_FLAGS_UAS_ID_AVAILABLE'></a>2          | [UTM_DATA_AVAIL_FLAGS_UAS_ID_AVAILABLE](#UTM_DATA_AVAIL_FLAGS_UAS_ID_AVAILABLE) | 字段 uas_id 包含有效数据。                         |
| <a id='UTM_DATA_AVAIL_FLAGS_POSITION_AVAILABLE'></a>4        | [UTM_DATA_AVAIL_FLAGS_POSITION_AVAILABLE](#UTM_DATA_AVAIL_FLAGS_POSITION_AVAILABLE) | 字段 lat、lon 和 h_acc 包含有效数据。              |
| <a id='UTM_DATA_AVAIL_FLAGS_ALTITUDE_AVAILABLE'></a>8        | [UTM_DATA_AVAIL_FLAGS_ALTITUDE_AVAILABLE](#UTM_DATA_AVAIL_FLAGS_ALTITUDE_AVAILABLE) | alt 和 v_acc 字段包含有效数据。                    |
| <a id='UTM_DATA_AVAIL_FLAGS_RELATIVE_ALTITUDE_AVAILABLE'></a>16 | [UTM_DATA_AVAIL_FLAGS_RELATIVE_ALTITUDE_AVAILABLE](#UTM_DATA_AVAIL_FLAGS_RELATIVE_ALTITUDE_AVAILABLE) | relative_alt 字段包含有效数据。                    |
| <a id='UTM_DATA_AVAIL_FLAGS_HORIZONTAL_VELO_AVAILABLE'></a>32 | [UTM_DATA_AVAIL_FLAGS_HORIZONTAL_VELO_AVAILABLE](#UTM_DATA_AVAIL_FLAGS_HORIZONTAL_VELO_AVAILABLE) | 字段 vx 和 vy 包含有效数据。                       |
| <a id='UTM_DATA_AVAIL_FLAGS_VERTICAL_VELO_AVAILABLE'></a>64  | [UTM_DATA_AVAIL_FLAGS_VERTICAL_VELO_AVAILABLE](#UTM_DATA_AVAIL_FLAGS_VERTICAL_VELO_AVAILABLE) | 字段 vz 包含有效数据。                             |
| <a id='UTM_DATA_AVAIL_FLAGS_NEXT_WAYPOINT_AVAILABLE'></a>128 | [UTM_DATA_AVAIL_FLAGS_NEXT_WAYPOINT_AVAILABLE](#UTM_DATA_AVAIL_FLAGS_NEXT_WAYPOINT_AVAILABLE) | 字段 next_lat、next_lon 和 next_alt 包含有效数据。 |

### CELLULAR_STATUS_FLAG 

这些标志表示蜂窝网络状态

| Value                                             | Name                                                         | Description                                                  |
| ------------------------------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| <a id='CELLULAR_STATUS_FLAG_UNKNOWN'></a>0        | [CELLULAR_STATUS_FLAG_UNKNOWN](#CELLULAR_STATUS_FLAG_UNKNOWN) | 状态未知或无法报告。                                         |
| <a id='CELLULAR_STATUS_FLAG_FAILED'></a>1         | [CELLULAR_STATUS_FLAG_FAILED](#CELLULAR_STATUS_FLAG_FAILED)  | 调制解调器无法使用                                           |
| <a id='CELLULAR_STATUS_FLAG_INITIALIZING'></a>2   | [CELLULAR_STATUS_FLAG_INITIALIZING](#CELLULAR_STATUS_FLAG_INITIALIZING) | 调制解调器正在初始化                                         |
| <a id='CELLULAR_STATUS_FLAG_LOCKED'></a>3         | [CELLULAR_STATUS_FLAG_LOCKED](#CELLULAR_STATUS_FLAG_LOCKED)  | 调制解调器被锁定                                             |
| <a id='CELLULAR_STATUS_FLAG_DISABLED'></a>4       | [CELLULAR_STATUS_FLAG_DISABLED](#CELLULAR_STATUS_FLAG_DISABLED) | 调制解调器未启用且已关机                                     |
| <a id='CELLULAR_STATUS_FLAG_DISABLING'></a>5      | [CELLULAR_STATUS_FLAG_DISABLING](#CELLULAR_STATUS_FLAG_DISABLING) | 调制解调器当前正过渡到 [CELLULAR_STATUS_FLAG_DISABLED](#CELLULAR_STATUS_FLAG_DISABLED) 状态 |
| <a id='CELLULAR_STATUS_FLAG_ENABLING'></a>6       | [CELLULAR_STATUS_FLAG_ENABLING](#CELLULAR_STATUS_FLAG_ENABLING) | 调制解调器当前正过渡到 [CELLULAR_STATUS_FLAG_ENABLED]（#CELLULAR_STATUS_FLAG_ENABLED）状态 |
| <a id='CELLULAR_STATUS_FLAG_ENABLED'></a>7        | [CELLULAR_STATUS_FLAG_ENABLED](#CELLULAR_STATUS_FLAG_ENABLED) | 调制解调器已启用并接通电源，但未在网络提供商处注册，无法进行数据连接 |
| <a id='CELLULAR_STATUS_FLAG_SEARCHING'></a>8      | [CELLULAR_STATUS_FLAG_SEARCHING](#CELLULAR_STATUS_FLAG_SEARCHING) | 调制解调器正在寻找网络提供商进行注册                         |
| <a id='CELLULAR_STATUS_FLAG_REGISTERED'></a>9     | [CELLULAR_STATUS_FLAG_REGISTERED](#CELLULAR_STATUS_FLAG_REGISTERED) | 调制解调器已在网络提供商处注册，可以使用数据连接和信息传输功能 |
| <a id='CELLULAR_STATUS_FLAG_DISCONNECTING'></a>10 | [CELLULAR_STATUS_FLAG_DISCONNECTING](#CELLULAR_STATUS_FLAG_DISCONNECTING) | 调制解调器正在断开并停用最后一个活动分组数据承载。如果有一个以上的分组数据承载处于活动状态，且其中一个活动承载被停用，则不会进入此状态 |
| <a id='CELLULAR_STATUS_FLAG_CONNECTING'></a>11    | [CELLULAR_STATUS_FLAG_CONNECTING](#CELLULAR_STATUS_FLAG_CONNECTING) | 调制解调器正在激活和连接第一个分组数据承载。当另一承载已激活时，随后的承载激活不会导致进入此状态 |
| <a id='CELLULAR_STATUS_FLAG_CONNECTED'></a>12     | [CELLULAR_STATUS_FLAG_CONNECTED](#CELLULAR_STATUS_FLAG_CONNECTED) | 一个或多个分组数据承载处于活动状态并已连接                   |

### CELLULAR_NETWORK_FAILED_REASON 

这些标志用于诊断 [CELLULAR_STATUS]（#CELLULAR_STATUS）的故障状态

| Value                                                    | Name                                                         | Description                   |
| -------------------------------------------------------- | ------------------------------------------------------------ | ----------------------------- |
| <a id='CELLULAR_NETWORK_FAILED_REASON_NONE'></a>0        | [CELLULAR_NETWORK_FAILED_REASON_NONE](#CELLULAR_NETWORK_FAILED_REASON_NONE) | No error                      |
| <a id='CELLULAR_NETWORK_FAILED_REASON_UNKNOWN'></a>1     | [CELLULAR_NETWORK_FAILED_REASON_UNKNOWN](#CELLULAR_NETWORK_FAILED_REASON_UNKNOWN) | 错误状态未知                  |
| <a id='CELLULAR_NETWORK_FAILED_REASON_SIM_MISSING'></a>2 | [CELLULAR_NETWORK_FAILED_REASON_SIM_MISSING](#CELLULAR_NETWORK_FAILED_REASON_SIM_MISSING) | 调制解调器需要 SIM 卡，但缺失 |
| <a id='CELLULAR_NETWORK_FAILED_REASON_SIM_ERROR'></a>3   | [CELLULAR_NETWORK_FAILED_REASON_SIM_ERROR](#CELLULAR_NETWORK_FAILED_REASON_SIM_ERROR) | SIM 卡可用，但无法连接        |

### CELLULAR_NETWORK_RADIO_TYPE 

蜂窝网络无线电类型

| Value                                           | Name                                                         | Description |
| ----------------------------------------------- | ------------------------------------------------------------ | ----------- |
| <a id='CELLULAR_NETWORK_RADIO_TYPE_NONE'></a>0  | [CELLULAR_NETWORK_RADIO_TYPE_NONE](#CELLULAR_NETWORK_RADIO_TYPE_NONE) |             |
| <a id='CELLULAR_NETWORK_RADIO_TYPE_GSM'></a>1   | [CELLULAR_NETWORK_RADIO_TYPE_GSM](#CELLULAR_NETWORK_RADIO_TYPE_GSM) |             |
| <a id='CELLULAR_NETWORK_RADIO_TYPE_CDMA'></a>2  | [CELLULAR_NETWORK_RADIO_TYPE_CDMA](#CELLULAR_NETWORK_RADIO_TYPE_CDMA) |             |
| <a id='CELLULAR_NETWORK_RADIO_TYPE_WCDMA'></a>3 | [CELLULAR_NETWORK_RADIO_TYPE_WCDMA](#CELLULAR_NETWORK_RADIO_TYPE_WCDMA) |             |
| <a id='CELLULAR_NETWORK_RADIO_TYPE_LTE'></a>4   | [CELLULAR_NETWORK_RADIO_TYPE_LTE](#CELLULAR_NETWORK_RADIO_TYPE_LTE) |             |

### PRECISION_LAND_MODE 

精确陆地模式（用于 [MAV_CMD_NAV_LAND](#MAV_CMD_NAV_LAND)）。

| Value                                           | Name                                                         | Description                                                  |
| ----------------------------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| <a id='PRECISION_LAND_MODE_DISABLED'></a>0      | [PRECISION_LAND_MODE_DISABLED](#PRECISION_LAND_MODE_DISABLED) | 正常（非精确）着陆。                                         |
| <a id='PRECISION_LAND_MODE_OPPORTUNISTIC'></a>1 | [PRECISION_LAND_MODE_OPPORTUNISTIC](#PRECISION_LAND_MODE_OPPORTUNISTIC) | Use precision landing if beacon detected when land command accepted, otherwise land normally. |
| <a id='PRECISION_LAND_MODE_REQUIRED'></a>2      | [PRECISION_LAND_MODE_REQUIRED](#PRECISION_LAND_MODE_REQUIRED) | 如果接受着陆指令时检测到信标，则使用精确着陆，否则正常着陆。 |

### PARACHUTE_ACTION 

降落伞动作。触发释放和启用/禁用自动释放。

| Value                           | Name                                    | Description                                        |
| ------------------------------- | --------------------------------------- | -------------------------------------------------- |
| <a id='PARACHUTE_DISABLE'></a>0 | [PARACHUTE_DISABLE](#PARACHUTE_DISABLE) | 禁用降落伞自动释放功能（即由碰撞探测器触发释放）。 |
| <a id='PARACHUTE_ENABLE'></a>1  | [PARACHUTE_ENABLE](#PARACHUTE_ENABLE)   | 启用降落伞自动释放功能。                           |
| <a id='PARACHUTE_RELEASE'></a>2 | [PARACHUTE_RELEASE](#PARACHUTE_RELEASE) | 释放降落伞并关闭发动机。                           |

### MAV_TUNNEL_PAYLOAD_TYPE 

| Value                                                     | Name                                                         | Description                               |
| --------------------------------------------------------- | ------------------------------------------------------------ | ----------------------------------------- |
| <a id='MAV_TUNNEL_PAYLOAD_TYPE_UNKNOWN'></a>0             | [MAV_TUNNEL_PAYLOAD_TYPE_UNKNOWN](#MAV_TUNNEL_PAYLOAD_TYPE_UNKNOWN) | 有效载荷编码未知。                        |
| <a id='MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED0'></a>200 | [MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED0](#MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED0) | 注册用于 STorM32 万向节控制器。           |
| <a id='MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED1'></a>201 | [MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED1](#MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED1) | 已注册 STorM32 万向节控制器。             |
| <a id='MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED2'></a>202 | [MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED2](#MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED2) | 已注册 STorM32 万向节控制器。             |
| <a id='MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED3'></a>203 | [MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED3](#MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED3) | 已注册 STorM32 万向节控制器。             |
| <a id='MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED4'></a>204 | [MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED4](#MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED4) | 已注册 STorM32 万向节控制器。             |
| <a id='MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED5'></a>205 | [MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED5](#MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED5) | Registered for STorM32 gimbal controller. |
| <a id='MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED6'></a>206 | [MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED6](#MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED6) | Registered for STorM32 gimbal controller. |
| <a id='MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED7'></a>207 | [MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED7](#MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED7) | Registered for STorM32 gimbal controller. |
| <a id='MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED8'></a>208 | [MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED8](#MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED8) | Registered for STorM32 gimbal controller. |
| <a id='MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED9'></a>209 | [MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED9](#MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED9) | 已注册 STorM32 万向节控制器。             |

### MAV_ODID_ID_TYPE 

| Value                                              | Name                                                         | Description                                                  |
| -------------------------------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| <a id='MAV_ODID_ID_TYPE_NONE'></a>0                | [MAV_ODID_ID_TYPE_NONE](#MAV_ODID_ID_TYPE_NONE)              | 未定义类型。                                                 |
| <a id='MAV_ODID_ID_TYPE_SERIAL_NUMBER'></a>1       | [MAV_ODID_ID_TYPE_SERIAL_NUMBER](#MAV_ODID_ID_TYPE_SERIAL_NUMBER) | 制造商序列号（ANSI/CTA-2063 格式）。                         |
| <a id='MAV_ODID_ID_TYPE_CAA_REGISTRATION_ID'></a>2 | [MAV_ODID_ID_TYPE_CAA_REGISTRATION_ID](#MAV_ODID_ID_TYPE_CAA_REGISTRATION_ID) | CAA（民航局）注册 ID。格式： [国际民航组织国家代码].[民航局指定标识]. |
| <a id='MAV_ODID_ID_TYPE_UTM_ASSIGNED_UUID'></a>3   | [MAV_ODID_ID_TYPE_UTM_ASSIGNED_UUID](#MAV_ODID_ID_TYPE_UTM_ASSIGNED_UUID) | UTM（无人机流量管理）分配的 UUID（RFC4122）。                |
| <a id='MAV_ODID_ID_TYPE_SPECIFIC_SESSION_ID'></a>4 | [MAV_ODID_ID_TYPE_SPECIFIC_SESSION_ID](#MAV_ODID_ID_TYPE_SPECIFIC_SESSION_ID) | 特定航班/时段的 20 个字节 ID。确切的 ID 类型由 uas_id 的第一个字节表示，这些类型值由国际民航组织管理。 |

### MAV_ODID_UA_TYPE 

| Value                                                     | Name                                                         | Description                                |
| --------------------------------------------------------- | ------------------------------------------------------------ | ------------------------------------------ |
| <a id='MAV_ODID_UA_TYPE_NONE'></a>0                       | [MAV_ODID_UA_TYPE_NONE](#MAV_ODID_UA_TYPE_NONE)              | 未定义 UA（无人驾驶飞机）类型。            |
| <a id='MAV_ODID_UA_TYPE_AEROPLANE'></a>1                  | [MAV_ODID_UA_TYPE_AEROPLANE](#MAV_ODID_UA_TYPE_AEROPLANE)    | 飞机 固定翼飞机                            |
| <a id='MAV_ODID_UA_TYPE_HELICOPTER_OR_MULTIROTOR'></a>2   | [MAV_ODID_UA_TYPE_HELICOPTER_OR_MULTIROTOR](#MAV_ODID_UA_TYPE_HELICOPTER_OR_MULTIROTOR) | 直升机或多旋翼飞行器                       |
| <a id='MAV_ODID_UA_TYPE_GYROPLANE'></a>3                  | [MAV_ODID_UA_TYPE_GYROPLANE](#MAV_ODID_UA_TYPE_GYROPLANE)    | Gyroplane. (陀螺飞机)                      |
| <a id='MAV_ODID_UA_TYPE_HYBRID_LIFT'></a>4                | [MAV_ODID_UA_TYPE_HYBRID_LIFT](#MAV_ODID_UA_TYPE_HYBRID_LIFT) | VTOL（垂直起降）。可垂直起飞的固定翼飞机。 |
| <a id='MAV_ODID_UA_TYPE_ORNITHOPTER'></a>5                | [MAV_ODID_UA_TYPE_ORNITHOPTER](#MAV_ODID_UA_TYPE_ORNITHOPTER) | Ornithopter(鸟类直升机).                   |
| <a id='MAV_ODID_UA_TYPE_GLIDER'></a>6                     | [MAV_ODID_UA_TYPE_GLIDER](#MAV_ODID_UA_TYPE_GLIDER)          | 滑翔机                                     |
| <a id='MAV_ODID_UA_TYPE_KITE'></a>7                       | [MAV_ODID_UA_TYPE_KITE](#MAV_ODID_UA_TYPE_KITE)              | 风筝.                                      |
| <a id='MAV_ODID_UA_TYPE_FREE_BALLOON'></a>8               | [MAV_ODID_UA_TYPE_FREE_BALLOON](#MAV_ODID_UA_TYPE_FREE_BALLOON) | 自由气艇.                                  |
| <a id='MAV_ODID_UA_TYPE_CAPTIVE_BALLOON'></a>9            | [MAV_ODID_UA_TYPE_CAPTIVE_BALLOON](#MAV_ODID_UA_TYPE_CAPTIVE_BALLOON) | 被囚禁的气球                               |
| <a id='MAV_ODID_UA_TYPE_AIRSHIP'></a>10                   | [MAV_ODID_UA_TYPE_AIRSHIP](#MAV_ODID_UA_TYPE_AIRSHIP)        | 飞艇。如飞艇。                             |
| <a id='MAV_ODID_UA_TYPE_FREE_FALL_PARACHUTE'></a>11       | [MAV_ODID_UA_TYPE_FREE_FALL_PARACHUTE](#MAV_ODID_UA_TYPE_FREE_FALL_PARACHUTE) | 自由落体/降落伞（无动力）。                |
| <a id='MAV_ODID_UA_TYPE_ROCKET'></a>12                    | [MAV_ODID_UA_TYPE_ROCKET](#MAV_ODID_UA_TYPE_ROCKET)          | 火箭.                                      |
| <a id='MAV_ODID_UA_TYPE_TETHERED_POWERED_AIRCRAFT'></a>13 | [MAV_ODID_UA_TYPE_TETHERED_POWERED_AIRCRAFT](#MAV_ODID_UA_TYPE_TETHERED_POWERED_AIRCRAFT) | 系留动力飞行器                             |
| <a id='MAV_ODID_UA_TYPE_GROUND_OBSTACLE'></a>14           | [MAV_ODID_UA_TYPE_GROUND_OBSTACLE](#MAV_ODID_UA_TYPE_GROUND_OBSTACLE) | 地面障碍物。                               |
| <a id='MAV_ODID_UA_TYPE_OTHER'></a>15                     | [MAV_ODID_UA_TYPE_OTHER](#MAV_ODID_UA_TYPE_OTHER)            | 前面未列出的其他类型的飞机。               |

### MAV_ODID_STATUS 

| Value                                                  | Name                                                         | Description                        |
| ------------------------------------------------------ | ------------------------------------------------------------ | ---------------------------------- |
| <a id='MAV_ODID_STATUS_UNDECLARED'></a>0               | [MAV_ODID_STATUS_UNDECLARED](#MAV_ODID_STATUS_UNDECLARED)    | (UA) 无人驾驶飞机的状态未定。      |
| <a id='MAV_ODID_STATUS_GROUND'></a>1                   | [MAV_ODID_STATUS_GROUND](#MAV_ODID_STATUS_GROUND)            | UA 在地面上。                      |
| <a id='MAV_ODID_STATUS_AIRBORNE'></a>2                 | [MAV_ODID_STATUS_AIRBORNE](#MAV_ODID_STATUS_AIRBORNE)        | UA 就在空中。                      |
| <a id='MAV_ODID_STATUS_EMERGENCY'></a>3                | [MAV_ODID_STATUS_EMERGENCY](#MAV_ODID_STATUS_EMERGENCY)      | UA 出了紧急情况。                  |
| <a id='MAV_ODID_STATUS_REMOTE_ID_SYSTEM_FAILURE'></a>4 | [MAV_ODID_STATUS_REMOTE_ID_SYSTEM_FAILURE](#MAV_ODID_STATUS_REMOTE_ID_SYSTEM_FAILURE) | 远程身份识别系统出现故障或不可靠。 |

### MAV_ODID_HEIGHT_REF 

| Value                                          | Name                                                         | Description              |
| ---------------------------------------------- | ------------------------------------------------------------ | ------------------------ |
| <a id='MAV_ODID_HEIGHT_REF_OVER_TAKEOFF'></a>0 | [MAV_ODID_HEIGHT_REF_OVER_TAKEOFF](#MAV_ODID_HEIGHT_REF_OVER_TAKEOFF) | 高度区域与起飞位置相对。 |
| <a id='MAV_ODID_HEIGHT_REF_OVER_GROUND'></a>1  | [MAV_ODID_HEIGHT_REF_OVER_GROUND](#MAV_ODID_HEIGHT_REF_OVER_GROUND) | 高度区域相对于地面。     |

### MAV_ODID_HOR_ACC 

| Value                                    | Name                                                    | Description                       |
| ---------------------------------------- | ------------------------------------------------------- | --------------------------------- |
| <a id='MAV_ODID_HOR_ACC_UNKNOWN'></a>0   | [MAV_ODID_HOR_ACC_UNKNOWN](#MAV_ODID_HOR_ACC_UNKNOWN)   | 水平精度未知。                    |
| <a id='MAV_ODID_HOR_ACC_10NM'></a>1      | [MAV_ODID_HOR_ACC_10NM](#MAV_ODID_HOR_ACC_10NM)         | 水平精度小于 10 海里。18.52 千米  |
| <a id='MAV_ODID_HOR_ACC_4NM'></a>2       | [MAV_ODID_HOR_ACC_4NM](#MAV_ODID_HOR_ACC_4NM)           | 水平精度小于 4 海里。7.408 千米   |
| <a id='MAV_ODID_HOR_ACC_2NM'></a>3       | [MAV_ODID_HOR_ACC_2NM](#MAV_ODID_HOR_ACC_2NM)           | 水平精度小于 2 海里。3.704 千米   |
| <a id='MAV_ODID_HOR_ACC_1NM'></a>4       | [MAV_ODID_HOR_ACC_1NM](#MAV_ODID_HOR_ACC_1NM)           | 水平精度小于 1 海里。1.852 千米。 |
| <a id='MAV_ODID_HOR_ACC_0_5NM'></a>5     | [MAV_ODID_HOR_ACC_0_5NM](#MAV_ODID_HOR_ACC_0_5NM)       | 水平精度小于 0.5 海里。926 m.     |
| <a id='MAV_ODID_HOR_ACC_0_3NM'></a>6     | [MAV_ODID_HOR_ACC_0_3NM](#MAV_ODID_HOR_ACC_0_3NM)       | 水平精度小于 0.3 海里。555.6 m.   |
| <a id='MAV_ODID_HOR_ACC_0_1NM'></a>7     | [MAV_ODID_HOR_ACC_0_1NM](#MAV_ODID_HOR_ACC_0_1NM)       | 水平精度小于 0.1 海里。185.2 m.   |
| <a id='MAV_ODID_HOR_ACC_0_05NM'></a>8    | [MAV_ODID_HOR_ACC_0_05NM](#MAV_ODID_HOR_ACC_0_05NM)     | 水平精度小于 0.05 海里。92.6 m.   |
| <a id='MAV_ODID_HOR_ACC_30_METER'></a>9  | [MAV_ODID_HOR_ACC_30_METER](#MAV_ODID_HOR_ACC_30_METER) | 水平精度小于 30 米。              |
| <a id='MAV_ODID_HOR_ACC_10_METER'></a>10 | [MAV_ODID_HOR_ACC_10_METER](#MAV_ODID_HOR_ACC_10_METER) | 水平精度小于 10 米。              |
| <a id='MAV_ODID_HOR_ACC_3_METER'></a>11  | [MAV_ODID_HOR_ACC_3_METER](#MAV_ODID_HOR_ACC_3_METER)   | 水平精度小于 3 米。               |
| <a id='MAV_ODID_HOR_ACC_1_METER'></a>12  | [MAV_ODID_HOR_ACC_1_METER](#MAV_ODID_HOR_ACC_1_METER)   | 水平精度小于 1 米。               |

### MAV_ODID_VER_ACC 

| Value                                    | Name                                                      | Description                                     |
| ---------------------------------------- | --------------------------------------------------------- | ----------------------------------------------- |
| <a id='MAV_ODID_VER_ACC_UNKNOWN'></a>0   | [MAV_ODID_VER_ACC_UNKNOWN](#MAV_ODID_VER_ACC_UNKNOWN)     | 垂直精度未知。                                  |
| <a id='MAV_ODID_VER_ACC_150_METER'></a>1 | [MAV_ODID_VER_ACC_150_METER](#MAV_ODID_VER_ACC_150_METER) | 垂直精度小于 150 米。                           |
| <a id='MAV_ODID_VER_ACC_45_METER'></a>2  | [MAV_ODID_VER_ACC_45_METER](#MAV_ODID_VER_ACC_45_METER)   | 垂直精度小于 45 米。                            |
| <a id='MAV_ODID_VER_ACC_25_METER'></a>3  | [MAV_ODID_VER_ACC_25_METER](#MAV_ODID_VER_ACC_25_METER)   | The vertical accuracy is smaller than 25 meter. |
| <a id='MAV_ODID_VER_ACC_10_METER'></a>4  | [MAV_ODID_VER_ACC_10_METER](#MAV_ODID_VER_ACC_10_METER)   | The vertical accuracy is smaller than 10 meter. |
| <a id='MAV_ODID_VER_ACC_3_METER'></a>5   | [MAV_ODID_VER_ACC_3_METER](#MAV_ODID_VER_ACC_3_METER)     | The vertical accuracy is smaller than 3 meter.  |
| <a id='MAV_ODID_VER_ACC_1_METER'></a>6   | [MAV_ODID_VER_ACC_1_METER](#MAV_ODID_VER_ACC_1_METER)     | The vertical accuracy is smaller than 1 meter.  |

### MAV_ODID_SPEED_ACC 

| Value                                                  | Name                                                         | Description               |
| ------------------------------------------------------ | ------------------------------------------------------------ | ------------------------- |
| <a id='MAV_ODID_SPEED_ACC_UNKNOWN'></a>0               | [MAV_ODID_SPEED_ACC_UNKNOWN](#MAV_ODID_SPEED_ACC_UNKNOWN)    | 速度精度未知。            |
| <a id='MAV_ODID_SPEED_ACC_10_METERS_PER_SECOND'></a>1  | [MAV_ODID_SPEED_ACC_10_METERS_PER_SECOND](#MAV_ODID_SPEED_ACC_10_METERS_PER_SECOND) | 速度精度小于每秒 10 米。  |
| <a id='MAV_ODID_SPEED_ACC_3_METERS_PER_SECOND'></a>2   | [MAV_ODID_SPEED_ACC_3_METERS_PER_SECOND](#MAV_ODID_SPEED_ACC_3_METERS_PER_SECOND) | 速度精度小于每秒 3 米。   |
| <a id='MAV_ODID_SPEED_ACC_1_METERS_PER_SECOND'></a>3   | [MAV_ODID_SPEED_ACC_1_METERS_PER_SECOND](#MAV_ODID_SPEED_ACC_1_METERS_PER_SECOND) | 速度精度小于每秒 1 米。   |
| <a id='MAV_ODID_SPEED_ACC_0_3_METERS_PER_SECOND'></a>4 | [MAV_ODID_SPEED_ACC_0_3_METERS_PER_SECOND](#MAV_ODID_SPEED_ACC_0_3_METERS_PER_SECOND) | 速度精度小于每秒 0.3 米。 |

### MAV_ODID_TIME_ACC 

| Value                                       | Name                                                         | Description                                                  |
| ------------------------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| <a id='MAV_ODID_TIME_ACC_UNKNOWN'></a>0     | [MAV_ODID_TIME_ACC_UNKNOWN](#MAV_ODID_TIME_ACC_UNKNOWN)      | 时间戳准确性未知。                                           |
| <a id='MAV_ODID_TIME_ACC_0_1_SECOND'></a>1  | [MAV_ODID_TIME_ACC_0_1_SECOND](#MAV_ODID_TIME_ACC_0_1_SECOND) | 时间戳精度小于或等于 0.1 秒。                                |
| <a id='MAV_ODID_TIME_ACC_0_2_SECOND'></a>2  | [MAV_ODID_TIME_ACC_0_2_SECOND](#MAV_ODID_TIME_ACC_0_2_SECOND) | 时间戳精度小于或等于 0.2 秒。                                |
| <a id='MAV_ODID_TIME_ACC_0_3_SECOND'></a>3  | [MAV_ODID_TIME_ACC_0_3_SECOND](#MAV_ODID_TIME_ACC_0_3_SECOND) | 时间戳精度小于或等于 0.3 秒。                                |
| <a id='MAV_ODID_TIME_ACC_0_4_SECOND'></a>4  | [MAV_ODID_TIME_ACC_0_4_SECOND](#MAV_ODID_TIME_ACC_0_4_SECOND) | 时间戳精度小于或等于 0.4 秒。                                |
| <a id='MAV_ODID_TIME_ACC_0_5_SECOND'></a>5  | [MAV_ODID_TIME_ACC_0_5_SECOND](#MAV_ODID_TIME_ACC_0_5_SECOND) | The timestamp accuracy is smaller than or equal to 0.5 second. |
| <a id='MAV_ODID_TIME_ACC_0_6_SECOND'></a>6  | [MAV_ODID_TIME_ACC_0_6_SECOND](#MAV_ODID_TIME_ACC_0_6_SECOND) | The timestamp accuracy is smaller than or equal to 0.6 second. |
| <a id='MAV_ODID_TIME_ACC_0_7_SECOND'></a>7  | [MAV_ODID_TIME_ACC_0_7_SECOND](#MAV_ODID_TIME_ACC_0_7_SECOND) | The timestamp accuracy is smaller than or equal to 0.7 second. |
| <a id='MAV_ODID_TIME_ACC_0_8_SECOND'></a>8  | [MAV_ODID_TIME_ACC_0_8_SECOND](#MAV_ODID_TIME_ACC_0_8_SECOND) | The timestamp accuracy is smaller than or equal to 0.8 second. |
| <a id='MAV_ODID_TIME_ACC_0_9_SECOND'></a>9  | [MAV_ODID_TIME_ACC_0_9_SECOND](#MAV_ODID_TIME_ACC_0_9_SECOND) | The timestamp accuracy is smaller than or equal to 0.9 second. |
| <a id='MAV_ODID_TIME_ACC_1_0_SECOND'></a>10 | [MAV_ODID_TIME_ACC_1_0_SECOND](#MAV_ODID_TIME_ACC_1_0_SECOND) | The timestamp accuracy is smaller than or equal to 1.0 second. |
| <a id='MAV_ODID_TIME_ACC_1_1_SECOND'></a>11 | [MAV_ODID_TIME_ACC_1_1_SECOND](#MAV_ODID_TIME_ACC_1_1_SECOND) | 时间戳精度小于或等于 1.1 秒。                                |
| <a id='MAV_ODID_TIME_ACC_1_2_SECOND'></a>12 | [MAV_ODID_TIME_ACC_1_2_SECOND](#MAV_ODID_TIME_ACC_1_2_SECOND) | The timestamp accuracy is smaller than or equal to 1.2 second. |
| <a id='MAV_ODID_TIME_ACC_1_3_SECOND'></a>13 | [MAV_ODID_TIME_ACC_1_3_SECOND](#MAV_ODID_TIME_ACC_1_3_SECOND) | The timestamp accuracy is smaller than or equal to 1.3 second. |
| <a id='MAV_ODID_TIME_ACC_1_4_SECOND'></a>14 | [MAV_ODID_TIME_ACC_1_4_SECOND](#MAV_ODID_TIME_ACC_1_4_SECOND) | The timestamp accuracy is smaller than or equal to 1.4 second. |
| <a id='MAV_ODID_TIME_ACC_1_5_SECOND'></a>15 | [MAV_ODID_TIME_ACC_1_5_SECOND](#MAV_ODID_TIME_ACC_1_5_SECOND) | The timestamp accuracy is smaller than or equal to 1.5 second. |

### MAV_ODID_AUTH_TYPE 

| Value                                                    | Name                                                         | Description                                                  |
| -------------------------------------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| <a id='MAV_ODID_AUTH_TYPE_NONE'></a>0                    | [MAV_ODID_AUTH_TYPE_NONE](#MAV_ODID_AUTH_TYPE_NONE)          | 未指定验证类型。                                             |
| <a id='MAV_ODID_AUTH_TYPE_UAS_ID_SIGNATURE'></a>1        | [MAV_ODID_AUTH_TYPE_UAS_ID_SIGNATURE](#MAV_ODID_AUTH_TYPE_UAS_ID_SIGNATURE) | 无人驾驶航空器系统（UAS）ID 的签名。                         |
| <a id='MAV_ODID_AUTH_TYPE_OPERATOR_ID_SIGNATURE'></a>2   | [MAV_ODID_AUTH_TYPE_OPERATOR_ID_SIGNATURE](#MAV_ODID_AUTH_TYPE_OPERATOR_ID_SIGNATURE) | 操作员 ID 的签名。                                           |
| <a id='MAV_ODID_AUTH_TYPE_MESSAGE_SET_SIGNATURE'></a>3   | [MAV_ODID_AUTH_TYPE_MESSAGE_SET_SIGNATURE](#MAV_ODID_AUTH_TYPE_MESSAGE_SET_SIGNATURE) | 整个信息集的签名。                                           |
| <a id='MAV_ODID_AUTH_TYPE_NETWORK_REMOTE_ID'></a>4       | [MAV_ODID_AUTH_TYPE_NETWORK_REMOTE_ID](#MAV_ODID_AUTH_TYPE_NETWORK_REMOTE_ID) | 身份验证由网络远程 ID 提供。                                 |
| <a id='MAV_ODID_AUTH_TYPE_SPECIFIC_AUTHENTICATION'></a>5 | [MAV_ODID_AUTH_TYPE_SPECIFIC_AUTHENTICATION](#MAV_ODID_AUTH_TYPE_SPECIFIC_AUTHENTICATION) | 确切的认证类型由认证数据的第一个字节表示，这些类型值由国际民航组织管理。 |

### MAV_ODID_DESC_TYPE 

| Value                                            | Name                                                         | Description                                                  |
| ------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |
| <a id='MAV_ODID_DESC_TYPE_TEXT'></a>0            | [MAV_ODID_DESC_TYPE_TEXT](#MAV_ODID_DESC_TYPE_TEXT)          | 关于飞行目的的可选自由格式文本描述。                         |
| <a id='MAV_ODID_DESC_TYPE_EMERGENCY'></a>1       | [MAV_ODID_DESC_TYPE_EMERGENCY](#MAV_ODID_DESC_TYPE_EMERGENCY) | 当状态 == [MAV_ODID_STATUS_EMERGENCY]（#MAV_ODID_STATUS_EMERGENCY）时，可选择附加说明。 |
| <a id='MAV_ODID_DESC_TYPE_EXTENDED_STATUS'></a>2 | [MAV_ODID_DESC_TYPE_EXTENDED_STATUS](#MAV_ODID_DESC_TYPE_EXTENDED_STATUS) | 当状态 != [MAV_ODID_STATUS_EMERGENCY]（#MAV_ODID_STATUS_EMERGENCY）时，可选择附加说明。 |

### MAV_ODID_OPERATOR_LOCATION_TYPE 

| Value                                                   | Name                                                         | Description                                           |
| ------------------------------------------------------- | ------------------------------------------------------------ | ----------------------------------------------------- |
| <a id='MAV_ODID_OPERATOR_LOCATION_TYPE_TAKEOFF'></a>0   | [MAV_ODID_OPERATOR_LOCATION_TYPE_TAKEOFF](#MAV_ODID_OPERATOR_LOCATION_TYPE_TAKEOFF) | 操作员的位置/高度与起飞位置相同。                     |
| <a id='MAV_ODID_OPERATOR_LOCATION_TYPE_LIVE_GNSS'></a>1 | [MAV_ODID_OPERATOR_LOCATION_TYPE_LIVE_GNSS](#MAV_ODID_OPERATOR_LOCATION_TYPE_LIVE_GNSS) | 操作员的位置/高度是动态的。例如，基于实时 GNSS 数据。 |
| <a id='MAV_ODID_OPERATOR_LOCATION_TYPE_FIXED'></a>2     | [MAV_ODID_OPERATOR_LOCATION_TYPE_FIXED](#MAV_ODID_OPERATOR_LOCATION_TYPE_FIXED) | 操作员的位置/高度是固定值。                           |

### MAV_ODID_CLASSIFICATION_TYPE 

| Value                                                 | Name                                                         | Description                       |
| ----------------------------------------------------- | ------------------------------------------------------------ | --------------------------------- |
| <a id='MAV_ODID_CLASSIFICATION_TYPE_UNDECLARED'></a>0 | [MAV_ODID_CLASSIFICATION_TYPE_UNDECLARED](#MAV_ODID_CLASSIFICATION_TYPE_UNDECLARED) | 未申报 UA 的分类类型。            |
| <a id='MAV_ODID_CLASSIFICATION_TYPE_EU'></a>1         | [MAV_ODID_CLASSIFICATION_TYPE_EU](#MAV_ODID_CLASSIFICATION_TYPE_EU) | UA 的分类类型遵循欧盟（EU）规范。 |

### MAV_ODID_CATEGORY_EU 

| Value                                         | Name                                                         | Description                                  |
| --------------------------------------------- | ------------------------------------------------------------ | -------------------------------------------- |
| <a id='MAV_ODID_CATEGORY_EU_UNDECLARED'></a>0 | [MAV_ODID_CATEGORY_EU_UNDECLARED](#MAV_ODID_CATEGORY_EU_UNDECLARED) | 根据欧盟规范，UA 的类别是未申报的。          |
| <a id='MAV_ODID_CATEGORY_EU_OPEN'></a>1       | [MAV_ODID_CATEGORY_EU_OPEN](#MAV_ODID_CATEGORY_EU_OPEN)      | 根据欧盟规范，统一用户协议的类别是开放类别。 |
| <a id='MAV_ODID_CATEGORY_EU_SPECIFIC'></a>2   | [MAV_ODID_CATEGORY_EU_SPECIFIC](#MAV_ODID_CATEGORY_EU_SPECIFIC) | 根据欧盟规范，UA 的类别是特定类别。          |
| <a id='MAV_ODID_CATEGORY_EU_CERTIFIED'></a>3  | [MAV_ODID_CATEGORY_EU_CERTIFIED](#MAV_ODID_CATEGORY_EU_CERTIFIED) | 根据欧盟规范，UA 的类别是认证类别。          |

### MAV_ODID_CLASS_EU 

| Value                                      | Name                                                         | Description                       |
| ------------------------------------------ | ------------------------------------------------------------ | --------------------------------- |
| <a id='MAV_ODID_CLASS_EU_UNDECLARED'></a>0 | [MAV_ODID_CLASS_EU_UNDECLARED](#MAV_ODID_CLASS_EU_UNDECLARED) | 根据欧盟规范，UA 的类别尚未公布。 |
| <a id='MAV_ODID_CLASS_EU_CLASS_0'></a>1    | [MAV_ODID_CLASS_EU_CLASS_0](#MAV_ODID_CLASS_EU_CLASS_0)      | 根据欧盟规范，UA 的等级为 0 级。  |
| <a id='MAV_ODID_CLASS_EU_CLASS_1'></a>2    | [MAV_ODID_CLASS_EU_CLASS_1](#MAV_ODID_CLASS_EU_CLASS_1)      | 根据欧盟规范，UA 的等级为 1 级。  |
| <a id='MAV_ODID_CLASS_EU_CLASS_2'></a>3    | [MAV_ODID_CLASS_EU_CLASS_2](#MAV_ODID_CLASS_EU_CLASS_2)      | 根据欧盟规范，UA 的等级为 2 级。  |
| <a id='MAV_ODID_CLASS_EU_CLASS_3'></a>4    | [MAV_ODID_CLASS_EU_CLASS_3](#MAV_ODID_CLASS_EU_CLASS_3)      | 根据欧盟规范，UA 的等级为 3 级。  |
| <a id='MAV_ODID_CLASS_EU_CLASS_4'></a>5    | [MAV_ODID_CLASS_EU_CLASS_4](#MAV_ODID_CLASS_EU_CLASS_4)      | 根据欧盟规范，UA 的等级为 4 级。  |
| <a id='MAV_ODID_CLASS_EU_CLASS_5'></a>6    | [MAV_ODID_CLASS_EU_CLASS_5](#MAV_ODID_CLASS_EU_CLASS_5)      | 根据欧盟规范，UA 的等级为 5 级。  |
| <a id='MAV_ODID_CLASS_EU_CLASS_6'></a>7    | [MAV_ODID_CLASS_EU_CLASS_6](#MAV_ODID_CLASS_EU_CLASS_6)      | 根据欧盟规范，UA 的等级为 6 级。  |

### MAV_ODID_OPERATOR_ID_TYPE 

| Value                                       | Name                                                         | Description                  |
| ------------------------------------------- | ------------------------------------------------------------ | ---------------------------- |
| <a id='MAV_ODID_OPERATOR_ID_TYPE_CAA'></a>0 | [MAV_ODID_OPERATOR_ID_TYPE_CAA](#MAV_ODID_OPERATOR_ID_TYPE_CAA) | CAA（民航局）注册运营商 ID。 |

### MAV_ODID_ARM_STATUS 

| Value                                                  | Name                                                         | Description                          |
| ------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------ |
| <a id='MAV_ODID_ARM_STATUS_GOOD_TO_ARM'></a>0          | [MAV_ODID_ARM_STATUS_GOOD_TO_ARM](#MAV_ODID_ARM_STATUS_GOOD_TO_ARM) | 通过武装检查。                       |
| <a id='MAV_ODID_ARM_STATUS_PRE_ARM_FAIL_GENERIC'></a>1 | [MAV_ODID_ARM_STATUS_PRE_ARM_FAIL_GENERIC](#MAV_ODID_ARM_STATUS_PRE_ARM_FAIL_GENERIC) | 通用布防失败，详情请查看错误字符串。 |

### TUNE_FORMAT 

调谐格式（用于生成车辆蜂鸣器/音调）。

| Value                                | Name                                              | Description                                                  |
| ------------------------------------ | ------------------------------------------------- | ------------------------------------------------------------ |
| <a id='TUNE_FORMAT_QBASIC1_1'></a>1  | [TUNE_FORMAT_QBASIC1_1](#TUNE_FORMAT_QBASIC1_1)   | 格式为 QBasic 1.1 播放：https://www.qbasic.net/en/reference/qb11/Statement/PLAY-006.htm。 |
| <a id='TUNE_FORMAT_MML_MODERN'></a>2 | [TUNE_FORMAT_MML_MODERN](#TUNE_FORMAT_MML_MODERN) | 格式为现代音乐标记语言 (MML)：https://en.wikipedia.org/wiki/Music_Macro_Language#Modern_MML。 |

### AIS_TYPE 

TAIS 船只类型，枚举，与 AIS 标准重复，https://gpsd.gitlab.io/gpsd/AIVDM.html

| Value                                         | Name                                                         | Description                             |
| --------------------------------------------- | ------------------------------------------------------------ | --------------------------------------- |
| <a id='AIS_TYPE_UNKNOWN'></a>0                | [AIS_TYPE_UNKNOWN](#AIS_TYPE_UNKNOWN)                        | 不可用（默认）。                        |
| <a id='AIS_TYPE_RESERVED_1'></a>1             | [AIS_TYPE_RESERVED_1](#AIS_TYPE_RESERVED_1)                  |                                         |
| <a id='AIS_TYPE_RESERVED_2'></a>2             | [AIS_TYPE_RESERVED_2](#AIS_TYPE_RESERVED_2)                  |                                         |
| <a id='AIS_TYPE_RESERVED_3'></a>3             | [AIS_TYPE_RESERVED_3](#AIS_TYPE_RESERVED_3)                  |                                         |
| <a id='AIS_TYPE_RESERVED_4'></a>4             | [AIS_TYPE_RESERVED_4](#AIS_TYPE_RESERVED_4)                  |                                         |
| <a id='AIS_TYPE_RESERVED_5'></a>5             | [AIS_TYPE_RESERVED_5](#AIS_TYPE_RESERVED_5)                  |                                         |
| <a id='AIS_TYPE_RESERVED_6'></a>6             | [AIS_TYPE_RESERVED_6](#AIS_TYPE_RESERVED_6)                  |                                         |
| <a id='AIS_TYPE_RESERVED_7'></a>7             | [AIS_TYPE_RESERVED_7](#AIS_TYPE_RESERVED_7)                  |                                         |
| <a id='AIS_TYPE_RESERVED_8'></a>8             | [AIS_TYPE_RESERVED_8](#AIS_TYPE_RESERVED_8)                  |                                         |
| <a id='AIS_TYPE_RESERVED_9'></a>9             | [AIS_TYPE_RESERVED_9](#AIS_TYPE_RESERVED_9)                  |                                         |
| <a id='AIS_TYPE_RESERVED_10'></a>10           | [AIS_TYPE_RESERVED_10](#AIS_TYPE_RESERVED_10)                |                                         |
| <a id='AIS_TYPE_RESERVED_11'></a>11           | [AIS_TYPE_RESERVED_11](#AIS_TYPE_RESERVED_11)                |                                         |
| <a id='AIS_TYPE_RESERVED_12'></a>12           | [AIS_TYPE_RESERVED_12](#AIS_TYPE_RESERVED_12)                |                                         |
| <a id='AIS_TYPE_RESERVED_13'></a>13           | [AIS_TYPE_RESERVED_13](#AIS_TYPE_RESERVED_13)                |                                         |
| <a id='AIS_TYPE_RESERVED_14'></a>14           | [AIS_TYPE_RESERVED_14](#AIS_TYPE_RESERVED_14)                |                                         |
| <a id='AIS_TYPE_RESERVED_15'></a>15           | [AIS_TYPE_RESERVED_15](#AIS_TYPE_RESERVED_15)                |                                         |
| <a id='AIS_TYPE_RESERVED_16'></a>16           | [AIS_TYPE_RESERVED_16](#AIS_TYPE_RESERVED_16)                |                                         |
| <a id='AIS_TYPE_RESERVED_17'></a>17           | [AIS_TYPE_RESERVED_17](#AIS_TYPE_RESERVED_17)                |                                         |
| <a id='AIS_TYPE_RESERVED_18'></a>18           | [AIS_TYPE_RESERVED_18](#AIS_TYPE_RESERVED_18)                |                                         |
| <a id='AIS_TYPE_RESERVED_19'></a>19           | [AIS_TYPE_RESERVED_19](#AIS_TYPE_RESERVED_19)                |                                         |
| <a id='AIS_TYPE_WIG'></a>20                   | [AIS_TYPE_WIG](#AIS_TYPE_WIG)                                | 翼在地面效果                            |
| <a id='AIS_TYPE_WIG_HAZARDOUS_A'></a>21       | [AIS_TYPE_WIG_HAZARDOUS_A](#AIS_TYPE_WIG_HAZARDOUS_A)        |                                         |
| <a id='AIS_TYPE_WIG_HAZARDOUS_B'></a>22       | [AIS_TYPE_WIG_HAZARDOUS_B](#AIS_TYPE_WIG_HAZARDOUS_B)        |                                         |
| <a id='AIS_TYPE_WIG_HAZARDOUS_C'></a>23       | [AIS_TYPE_WIG_HAZARDOUS_C](#AIS_TYPE_WIG_HAZARDOUS_C)        |                                         |
| <a id='AIS_TYPE_WIG_HAZARDOUS_D'></a>24       | [AIS_TYPE_WIG_HAZARDOUS_D](#AIS_TYPE_WIG_HAZARDOUS_D)        |                                         |
| <a id='AIS_TYPE_WIG_RESERVED_1'></a>25        | [AIS_TYPE_WIG_RESERVED_1](#AIS_TYPE_WIG_RESERVED_1)          |                                         |
| <a id='AIS_TYPE_WIG_RESERVED_2'></a>26        | [AIS_TYPE_WIG_RESERVED_2](#AIS_TYPE_WIG_RESERVED_2)          |                                         |
| <a id='AIS_TYPE_WIG_RESERVED_3'></a>27        | [AIS_TYPE_WIG_RESERVED_3](#AIS_TYPE_WIG_RESERVED_3)          |                                         |
| <a id='AIS_TYPE_WIG_RESERVED_4'></a>28        | [AIS_TYPE_WIG_RESERVED_4](#AIS_TYPE_WIG_RESERVED_4)          |                                         |
| <a id='AIS_TYPE_WIG_RESERVED_5'></a>29        | [AIS_TYPE_WIG_RESERVED_5](#AIS_TYPE_WIG_RESERVED_5)          |                                         |
| <a id='AIS_TYPE_FISHING'></a>30               | [AIS_TYPE_FISHING](#AIS_TYPE_FISHING)                        |                                         |
| <a id='AIS_TYPE_TOWING'></a>31                | [AIS_TYPE_TOWING](#AIS_TYPE_TOWING)                          |                                         |
| <a id='AIS_TYPE_TOWING_LARGE'></a>32          | [AIS_TYPE_TOWING_LARGE](#AIS_TYPE_TOWING_LARGE)              | 牵引：长度超过 200 米或宽度超过 25 米。 |
| <a id='AIS_TYPE_DREDGING'></a>33              | [AIS_TYPE_DREDGING](#AIS_TYPE_DREDGING)                      | 清淤或其他水下作业。                    |
| <a id='AIS_TYPE_DIVING'></a>34                | [AIS_TYPE_DIVING](#AIS_TYPE_DIVING)                          |                                         |
| <a id='AIS_TYPE_MILITARY'></a>35              | [AIS_TYPE_MILITARY](#AIS_TYPE_MILITARY)                      |                                         |
| <a id='AIS_TYPE_SAILING'></a>36               | [AIS_TYPE_SAILING](#AIS_TYPE_SAILING)                        |                                         |
| <a id='AIS_TYPE_PLEASURE'></a>37              | [AIS_TYPE_PLEASURE](#AIS_TYPE_PLEASURE)                      |                                         |
| <a id='AIS_TYPE_RESERVED_20'></a>38           | [AIS_TYPE_RESERVED_20](#AIS_TYPE_RESERVED_20)                |                                         |
| <a id='AIS_TYPE_RESERVED_21'></a>39           | [AIS_TYPE_RESERVED_21](#AIS_TYPE_RESERVED_21)                |                                         |
| <a id='AIS_TYPE_HSC'></a>40                   | [AIS_TYPE_HSC](#AIS_TYPE_HSC)                                | 高速飞行器                              |
| <a id='AIS_TYPE_HSC_HAZARDOUS_A'></a>41       | [AIS_TYPE_HSC_HAZARDOUS_A](#AIS_TYPE_HSC_HAZARDOUS_A)        |                                         |
| <a id='AIS_TYPE_HSC_HAZARDOUS_B'></a>42       | [AIS_TYPE_HSC_HAZARDOUS_B](#AIS_TYPE_HSC_HAZARDOUS_B)        |                                         |
| <a id='AIS_TYPE_HSC_HAZARDOUS_C'></a>43       | [AIS_TYPE_HSC_HAZARDOUS_C](#AIS_TYPE_HSC_HAZARDOUS_C)        |                                         |
| <a id='AIS_TYPE_HSC_HAZARDOUS_D'></a>44       | [AIS_TYPE_HSC_HAZARDOUS_D](#AIS_TYPE_HSC_HAZARDOUS_D)        |                                         |
| <a id='AIS_TYPE_HSC_RESERVED_1'></a>45        | [AIS_TYPE_HSC_RESERVED_1](#AIS_TYPE_HSC_RESERVED_1)          |                                         |
| <a id='AIS_TYPE_HSC_RESERVED_2'></a>46        | [AIS_TYPE_HSC_RESERVED_2](#AIS_TYPE_HSC_RESERVED_2)          |                                         |
| <a id='AIS_TYPE_HSC_RESERVED_3'></a>47        | [AIS_TYPE_HSC_RESERVED_3](#AIS_TYPE_HSC_RESERVED_3)          |                                         |
| <a id='AIS_TYPE_HSC_RESERVED_4'></a>48        | [AIS_TYPE_HSC_RESERVED_4](#AIS_TYPE_HSC_RESERVED_4)          |                                         |
| <a id='AIS_TYPE_HSC_UNKNOWN'></a>49           | [AIS_TYPE_HSC_UNKNOWN](#AIS_TYPE_HSC_UNKNOWN)                |                                         |
| <a id='AIS_TYPE_PILOT'></a>50                 | [AIS_TYPE_PILOT](#AIS_TYPE_PILOT)                            |                                         |
| <a id='AIS_TYPE_SAR'></a>51                   | [AIS_TYPE_SAR](#AIS_TYPE_SAR)                                | 搜索和救援船只。                        |
| <a id='AIS_TYPE_TUG'></a>52                   | [AIS_TYPE_TUG](#AIS_TYPE_TUG)                                |                                         |
| <a id='AIS_TYPE_PORT_TENDER'></a>53           | [AIS_TYPE_PORT_TENDER](#AIS_TYPE_PORT_TENDER)                |                                         |
| <a id='AIS_TYPE_ANTI_POLLUTION'></a>54        | [AIS_TYPE_ANTI_POLLUTION](#AIS_TYPE_ANTI_POLLUTION)          | 防污染设备。                            |
| <a id='AIS_TYPE_LAW_ENFORCEMENT'></a>55       | [AIS_TYPE_LAW_ENFORCEMENT](#AIS_TYPE_LAW_ENFORCEMENT)        |                                         |
| <a id='AIS_TYPE_SPARE_LOCAL_1'></a>56         | [AIS_TYPE_SPARE_LOCAL_1](#AIS_TYPE_SPARE_LOCAL_1)            |                                         |
| <a id='AIS_TYPE_SPARE_LOCAL_2'></a>57         | [AIS_TYPE_SPARE_LOCAL_2](#AIS_TYPE_SPARE_LOCAL_2)            |                                         |
| <a id='AIS_TYPE_MEDICAL_TRANSPORT'></a>58     | [AIS_TYPE_MEDICAL_TRANSPORT](#AIS_TYPE_MEDICAL_TRANSPORT)    |                                         |
| <a id='AIS_TYPE_NONECOMBATANT'></a>59         | [AIS_TYPE_NONECOMBATANT](#AIS_TYPE_NONECOMBATANT)            | 根据 RR 第 18 号决议，为非战斗舰船。    |
| <a id='AIS_TYPE_PASSENGER'></a>60             | [AIS_TYPE_PASSENGER](#AIS_TYPE_PASSENGER)                    |                                         |
| <a id='AIS_TYPE_PASSENGER_HAZARDOUS_A'></a>61 | [AIS_TYPE_PASSENGER_HAZARDOUS_A](#AIS_TYPE_PASSENGER_HAZARDOUS_A) |                                         |
| <a id='AIS_TYPE_PASSENGER_HAZARDOUS_B'></a>62 | [AIS_TYPE_PASSENGER_HAZARDOUS_B](#AIS_TYPE_PASSENGER_HAZARDOUS_B) |                                         |
| <a id='AIS_TYPE_PASSENGER_HAZARDOUS_C'></a>63 | [AIS_TYPE_PASSENGER_HAZARDOUS_C](#AIS_TYPE_PASSENGER_HAZARDOUS_C) |                                         |
| <a id='AIS_TYPE_PASSENGER_HAZARDOUS_D'></a>64 | [AIS_TYPE_PASSENGER_HAZARDOUS_D](#AIS_TYPE_PASSENGER_HAZARDOUS_D) |                                         |
| <a id='AIS_TYPE_PASSENGER_RESERVED_1'></a>65  | [AIS_TYPE_PASSENGER_RESERVED_1](#AIS_TYPE_PASSENGER_RESERVED_1) |                                         |
| <a id='AIS_TYPE_PASSENGER_RESERVED_2'></a>66  | [AIS_TYPE_PASSENGER_RESERVED_2](#AIS_TYPE_PASSENGER_RESERVED_2) |                                         |
| <a id='AIS_TYPE_PASSENGER_RESERVED_3'></a>67  | [AIS_TYPE_PASSENGER_RESERVED_3](#AIS_TYPE_PASSENGER_RESERVED_3) |                                         |
| <a id='AIS_TYPE_PASSENGER_RESERVED_4'></a>68  | [AIS_TYPE_PASSENGER_RESERVED_4](#AIS_TYPE_PASSENGER_RESERVED_4) |                                         |
| <a id='AIS_TYPE_PASSENGER_UNKNOWN'></a>69     | [AIS_TYPE_PASSENGER_UNKNOWN](#AIS_TYPE_PASSENGER_UNKNOWN)    |                                         |
| <a id='AIS_TYPE_CARGO'></a>70                 | [AIS_TYPE_CARGO](#AIS_TYPE_CARGO)                            |                                         |
| <a id='AIS_TYPE_CARGO_HAZARDOUS_A'></a>71     | [AIS_TYPE_CARGO_HAZARDOUS_A](#AIS_TYPE_CARGO_HAZARDOUS_A)    |                                         |
| <a id='AIS_TYPE_CARGO_HAZARDOUS_B'></a>72     | [AIS_TYPE_CARGO_HAZARDOUS_B](#AIS_TYPE_CARGO_HAZARDOUS_B)    |                                         |
| <a id='AIS_TYPE_CARGO_HAZARDOUS_C'></a>73     | [AIS_TYPE_CARGO_HAZARDOUS_C](#AIS_TYPE_CARGO_HAZARDOUS_C)    |                                         |
| <a id='AIS_TYPE_CARGO_HAZARDOUS_D'></a>74     | [AIS_TYPE_CARGO_HAZARDOUS_D](#AIS_TYPE_CARGO_HAZARDOUS_D)    |                                         |
| <a id='AIS_TYPE_CARGO_RESERVED_1'></a>75      | [AIS_TYPE_CARGO_RESERVED_1](#AIS_TYPE_CARGO_RESERVED_1)      |                                         |
| <a id='AIS_TYPE_CARGO_RESERVED_2'></a>76      | [AIS_TYPE_CARGO_RESERVED_2](#AIS_TYPE_CARGO_RESERVED_2)      |                                         |
| <a id='AIS_TYPE_CARGO_RESERVED_3'></a>77      | [AIS_TYPE_CARGO_RESERVED_3](#AIS_TYPE_CARGO_RESERVED_3)      |                                         |
| <a id='AIS_TYPE_CARGO_RESERVED_4'></a>78      | [AIS_TYPE_CARGO_RESERVED_4](#AIS_TYPE_CARGO_RESERVED_4)      |                                         |
| <a id='AIS_TYPE_CARGO_UNKNOWN'></a>79         | [AIS_TYPE_CARGO_UNKNOWN](#AIS_TYPE_CARGO_UNKNOWN)            |                                         |
| <a id='AIS_TYPE_TANKER'></a>80                | [AIS_TYPE_TANKER](#AIS_TYPE_TANKER)                          |                                         |
| <a id='AIS_TYPE_TANKER_HAZARDOUS_A'></a>81    | [AIS_TYPE_TANKER_HAZARDOUS_A](#AIS_TYPE_TANKER_HAZARDOUS_A)  |                                         |
| <a id='AIS_TYPE_TANKER_HAZARDOUS_B'></a>82    | [AIS_TYPE_TANKER_HAZARDOUS_B](#AIS_TYPE_TANKER_HAZARDOUS_B)  |                                         |
| <a id='AIS_TYPE_TANKER_HAZARDOUS_C'></a>83    | [AIS_TYPE_TANKER_HAZARDOUS_C](#AIS_TYPE_TANKER_HAZARDOUS_C)  |                                         |
| <a id='AIS_TYPE_TANKER_HAZARDOUS_D'></a>84    | [AIS_TYPE_TANKER_HAZARDOUS_D](#AIS_TYPE_TANKER_HAZARDOUS_D)  |                                         |
| <a id='AIS_TYPE_TANKER_RESERVED_1'></a>85     | [AIS_TYPE_TANKER_RESERVED_1](#AIS_TYPE_TANKER_RESERVED_1)    |                                         |
| <a id='AIS_TYPE_TANKER_RESERVED_2'></a>86     | [AIS_TYPE_TANKER_RESERVED_2](#AIS_TYPE_TANKER_RESERVED_2)    |                                         |
| <a id='AIS_TYPE_TANKER_RESERVED_3'></a>87     | [AIS_TYPE_TANKER_RESERVED_3](#AIS_TYPE_TANKER_RESERVED_3)    |                                         |
| <a id='AIS_TYPE_TANKER_RESERVED_4'></a>88     | [AIS_TYPE_TANKER_RESERVED_4](#AIS_TYPE_TANKER_RESERVED_4)    |                                         |
| <a id='AIS_TYPE_TANKER_UNKNOWN'></a>89        | [AIS_TYPE_TANKER_UNKNOWN](#AIS_TYPE_TANKER_UNKNOWN)          |                                         |
| <a id='AIS_TYPE_OTHER'></a>90                 | [AIS_TYPE_OTHER](#AIS_TYPE_OTHER)                            |                                         |
| <a id='AIS_TYPE_OTHER_HAZARDOUS_A'></a>91     | [AIS_TYPE_OTHER_HAZARDOUS_A](#AIS_TYPE_OTHER_HAZARDOUS_A)    |                                         |
| <a id='AIS_TYPE_OTHER_HAZARDOUS_B'></a>92     | [AIS_TYPE_OTHER_HAZARDOUS_B](#AIS_TYPE_OTHER_HAZARDOUS_B)    |                                         |
| <a id='AIS_TYPE_OTHER_HAZARDOUS_C'></a>93     | [AIS_TYPE_OTHER_HAZARDOUS_C](#AIS_TYPE_OTHER_HAZARDOUS_C)    |                                         |
| <a id='AIS_TYPE_OTHER_HAZARDOUS_D'></a>94     | [AIS_TYPE_OTHER_HAZARDOUS_D](#AIS_TYPE_OTHER_HAZARDOUS_D)    |                                         |
| <a id='AIS_TYPE_OTHER_RESERVED_1'></a>95      | [AIS_TYPE_OTHER_RESERVED_1](#AIS_TYPE_OTHER_RESERVED_1)      |                                         |
| <a id='AIS_TYPE_OTHER_RESERVED_2'></a>96      | [AIS_TYPE_OTHER_RESERVED_2](#AIS_TYPE_OTHER_RESERVED_2)      |                                         |
| <a id='AIS_TYPE_OTHER_RESERVED_3'></a>97      | [AIS_TYPE_OTHER_RESERVED_3](#AIS_TYPE_OTHER_RESERVED_3)      |                                         |
| <a id='AIS_TYPE_OTHER_RESERVED_4'></a>98      | [AIS_TYPE_OTHER_RESERVED_4](#AIS_TYPE_OTHER_RESERVED_4)      |                                         |
| <a id='AIS_TYPE_OTHER_UNKNOWN'></a>99         | [AIS_TYPE_OTHER_UNKNOWN](#AIS_TYPE_OTHER_UNKNOWN)            |                                         |

### AIS_NAV_STATUS 

AIS 船只的导航状态，从 AIS 标准中复制的枚举，https://gpsd.gitlab.io/gpsd/AIVDM.html

| Value                                             | Name                                                         | Description      |
| ------------------------------------------------- | ------------------------------------------------------------ | ---------------- |
| <a id='UNDER_WAY'></a>0                           | [UNDER_WAY](#UNDER_WAY)                                      | 正在使用发动机。 |
| <a id='AIS_NAV_ANCHORED'></a>1                    | [AIS_NAV_ANCHORED](#AIS_NAV_ANCHORED)                        |                  |
| <a id='AIS_NAV_UN_COMMANDED'></a>2                | [AIS_NAV_UN_COMMANDED](#AIS_NAV_UN_COMMANDED)                |                  |
| <a id='AIS_NAV_RESTRICTED_MANOEUVERABILITY'></a>3 | [AIS_NAV_RESTRICTED_MANOEUVERABILITY](#AIS_NAV_RESTRICTED_MANOEUVERABILITY) |                  |
| <a id='AIS_NAV_DRAUGHT_CONSTRAINED'></a>4         | [AIS_NAV_DRAUGHT_CONSTRAINED](#AIS_NAV_DRAUGHT_CONSTRAINED)  |                  |
| <a id='AIS_NAV_MOORED'></a>5                      | [AIS_NAV_MOORED](#AIS_NAV_MOORED)                            |                  |
| <a id='AIS_NAV_AGROUND'></a>6                     | [AIS_NAV_AGROUND](#AIS_NAV_AGROUND)                          |                  |
| <a id='AIS_NAV_FISHING'></a>7                     | [AIS_NAV_FISHING](#AIS_NAV_FISHING)                          |                  |
| <a id='AIS_NAV_SAILING'></a>8                     | [AIS_NAV_SAILING](#AIS_NAV_SAILING)                          |                  |
| <a id='AIS_NAV_RESERVED_HSC'></a>9                | [AIS_NAV_RESERVED_HSC](#AIS_NAV_RESERVED_HSC)                |                  |
| <a id='AIS_NAV_RESERVED_WIG'></a>10               | [AIS_NAV_RESERVED_WIG](#AIS_NAV_RESERVED_WIG)                |                  |
| <a id='AIS_NAV_RESERVED_1'></a>11                 | [AIS_NAV_RESERVED_1](#AIS_NAV_RESERVED_1)                    |                  |
| <a id='AIS_NAV_RESERVED_2'></a>12                 | [AIS_NAV_RESERVED_2](#AIS_NAV_RESERVED_2)                    |                  |
| <a id='AIS_NAV_RESERVED_3'></a>13                 | [AIS_NAV_RESERVED_3](#AIS_NAV_RESERVED_3)                    |                  |
| <a id='AIS_NAV_AIS_SART'></a>14                   | [AIS_NAV_AIS_SART](#AIS_NAV_AIS_SART)                        | 搜救应答器。     |
| <a id='AIS_NAV_UNKNOWN'></a>15                    | [AIS_NAV_UNKNOWN](#AIS_NAV_UNKNOWN)                          | 不可用（默认）。 |

### AIS_FLAGS 

(位掩码）这些标志用于 [AIS_VESSEL](#AIS_VESSEL).fields 位掩码，以指示其他报文字段中数据的有效性。设置时，数据有效。

| Value                                                | Name                                                         | Description                                                  |
| ---------------------------------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| <a id='AIS_FLAGS_POSITION_ACCURACY'></a>1            | [AIS_FLAGS_POSITION_ACCURACY](#AIS_FLAGS_POSITION_ACCURACY)  | 1 = 定位精度小于 10 米，0 = 定位精度大于 10 米。             |
| <a id='AIS_FLAGS_VALID_COG'></a>2                    | [AIS_FLAGS_VALID_COG](#AIS_FLAGS_VALID_COG)                  |                                                              |
| <a id='AIS_FLAGS_VALID_VELOCITY'></a>4               | [AIS_FLAGS_VALID_VELOCITY](#AIS_FLAGS_VALID_VELOCITY)        |                                                              |
| <a id='AIS_FLAGS_HIGH_VELOCITY'></a>8                | [AIS_FLAGS_HIGH_VELOCITY](#AIS_FLAGS_HIGH_VELOCITY)          | 1 = 速度超过 52.5765 米/秒（102.2 海里/小时）                |
| <a id='AIS_FLAGS_VALID_TURN_RATE'></a>16             | [AIS_FLAGS_VALID_TURN_RATE](#AIS_FLAGS_VALID_TURN_RATE)      |                                                              |
| <a id='AIS_FLAGS_TURN_RATE_SIGN_ONLY'></a>32         | [AIS_FLAGS_TURN_RATE_SIGN_ONLY](#AIS_FLAGS_TURN_RATE_SIGN_ONLY) | 只有返回的转动速率值的符号有效，即大于 5deg/30s 或小于 -5deg/30s |
| <a id='AIS_FLAGS_VALID_DIMENSIONS'></a>64            | [AIS_FLAGS_VALID_DIMENSIONS](#AIS_FLAGS_VALID_DIMENSIONS)    |                                                              |
| <a id='AIS_FLAGS_LARGE_BOW_DIMENSION'></a>128        | [AIS_FLAGS_LARGE_BOW_DIMENSION](#AIS_FLAGS_LARGE_BOW_DIMENSION) | 船首距离大于 511 米                                          |
| <a id='AIS_FLAGS_LARGE_STERN_DIMENSION'></a>256      | [AIS_FLAGS_LARGE_STERN_DIMENSION](#AIS_FLAGS_LARGE_STERN_DIMENSION) | 到船尾的距离大于 511 米                                      |
| <a id='AIS_FLAGS_LARGE_PORT_DIMENSION'></a>512       | [AIS_FLAGS_LARGE_PORT_DIMENSION](#AIS_FLAGS_LARGE_PORT_DIMENSION) | 左舷距离大于 63 米                                           |
| <a id='AIS_FLAGS_LARGE_STARBOARD_DIMENSION'></a>1024 | [AIS_FLAGS_LARGE_STARBOARD_DIMENSION](#AIS_FLAGS_LARGE_STARBOARD_DIMENSION) | 与右舷的距离大于 63 米                                       |
| <a id='AIS_FLAGS_VALID_CALLSIGN'></a>2048            | [AIS_FLAGS_VALID_CALLSIGN](#AIS_FLAGS_VALID_CALLSIGN)        |                                                              |
| <a id='AIS_FLAGS_VALID_NAME'></a>4096                | [AIS_FLAGS_VALID_NAME](#AIS_FLAGS_VALID_NAME)                |                                                              |

### FAILURE_UNIT 

可注入故障的可能单元列表。

| Value                                              | Name                                                         | Description |
| -------------------------------------------------- | ------------------------------------------------------------ | ----------- |
| <a id='FAILURE_UNIT_SENSOR_GYRO'></a>0             | [FAILURE_UNIT_SENSOR_GYRO](#FAILURE_UNIT_SENSOR_GYRO)        |             |
| <a id='FAILURE_UNIT_SENSOR_ACCEL'></a>1            | [FAILURE_UNIT_SENSOR_ACCEL](#FAILURE_UNIT_SENSOR_ACCEL)      |             |
| <a id='FAILURE_UNIT_SENSOR_MAG'></a>2              | [FAILURE_UNIT_SENSOR_MAG](#FAILURE_UNIT_SENSOR_MAG)          |             |
| <a id='FAILURE_UNIT_SENSOR_BARO'></a>3             | [FAILURE_UNIT_SENSOR_BARO](#FAILURE_UNIT_SENSOR_BARO)        |             |
| <a id='FAILURE_UNIT_SENSOR_GPS'></a>4              | [FAILURE_UNIT_SENSOR_GPS](#FAILURE_UNIT_SENSOR_GPS)          |             |
| <a id='FAILURE_UNIT_SENSOR_OPTICAL_FLOW'></a>5     | [FAILURE_UNIT_SENSOR_OPTICAL_FLOW](#FAILURE_UNIT_SENSOR_OPTICAL_FLOW) |             |
| <a id='FAILURE_UNIT_SENSOR_VIO'></a>6              | [FAILURE_UNIT_SENSOR_VIO](#FAILURE_UNIT_SENSOR_VIO)          |             |
| <a id='FAILURE_UNIT_SENSOR_DISTANCE_SENSOR'></a>7  | [FAILURE_UNIT_SENSOR_DISTANCE_SENSOR](#FAILURE_UNIT_SENSOR_DISTANCE_SENSOR) |             |
| <a id='FAILURE_UNIT_SENSOR_AIRSPEED'></a>8         | [FAILURE_UNIT_SENSOR_AIRSPEED](#FAILURE_UNIT_SENSOR_AIRSPEED) |             |
| <a id='FAILURE_UNIT_SYSTEM_BATTERY'></a>100        | [FAILURE_UNIT_SYSTEM_BATTERY](#FAILURE_UNIT_SYSTEM_BATTERY)  |             |
| <a id='FAILURE_UNIT_SYSTEM_MOTOR'></a>101          | [FAILURE_UNIT_SYSTEM_MOTOR](#FAILURE_UNIT_SYSTEM_MOTOR)      |             |
| <a id='FAILURE_UNIT_SYSTEM_SERVO'></a>102          | [FAILURE_UNIT_SYSTEM_SERVO](#FAILURE_UNIT_SYSTEM_SERVO)      |             |
| <a id='FAILURE_UNIT_SYSTEM_AVOIDANCE'></a>103      | [FAILURE_UNIT_SYSTEM_AVOIDANCE](#FAILURE_UNIT_SYSTEM_AVOIDANCE) |             |
| <a id='FAILURE_UNIT_SYSTEM_RC_SIGNAL'></a>104      | [FAILURE_UNIT_SYSTEM_RC_SIGNAL](#FAILURE_UNIT_SYSTEM_RC_SIGNAL) |             |
| <a id='FAILURE_UNIT_SYSTEM_MAVLINK_SIGNAL'></a>105 | [FAILURE_UNIT_SYSTEM_MAVLINK_SIGNAL](#FAILURE_UNIT_SYSTEM_MAVLINK_SIGNAL) |             |

### FAILURE_TYPE 

要注入的可能故障类型列表。

| Value                                   | Name                                                    | Description                                |
| --------------------------------------- | ------------------------------------------------------- | ------------------------------------------ |
| <a id='FAILURE_TYPE_OK'></a>0           | [FAILURE_TYPE_OK](#FAILURE_TYPE_OK)                     | 未注入故障，用于重置之前的故障。           |
| <a id='FAILURE_TYPE_OFF'></a>1          | [FAILURE_TYPE_OFF](#FAILURE_TYPE_OFF)                   | 设置设备关闭，因此完全没有反应。           |
| <a id='FAILURE_TYPE_STUCK'></a>2        | [FAILURE_TYPE_STUCK](#FAILURE_TYPE_STUCK)               | 设备被卡住，例如一直报告相同的值。         |
| <a id='FAILURE_TYPE_GARBAGE'></a>3      | [FAILURE_TYPE_GARBAGE](#FAILURE_TYPE_GARBAGE)           | 单位报告完全是垃圾。                       |
| <a id='FAILURE_TYPE_WRONG'></a>4        | [FAILURE_TYPE_WRONG](#FAILURE_TYPE_WRONG)               | 单位一直都错了。                           |
| <a id='FAILURE_TYPE_SLOW'></a>5         | [FAILURE_TYPE_SLOW](#FAILURE_TYPE_SLOW)                 | 单位速度慢，因此，例如，报告速度比预期的慢 |
| <a id='FAILURE_TYPE_DELAYED'></a>6      | [FAILURE_TYPE_DELAYED](#FAILURE_TYPE_DELAYED)           | 单元数据在时间上有延迟。                   |
| <a id='FAILURE_TYPE_INTERMITTENT'></a>7 | [FAILURE_TYPE_INTERMITTENT](#FAILURE_TYPE_INTERMITTENT) | 设备有时工作，有时不工作。                 |

### NAV_VTOL_LAND_OPTIONS 

| Value                                             | Name                                                         | Description                                                  |
| ------------------------------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| <a id='NAV_VTOL_LAND_OPTIONS_DEFAULT'></a>0       | [NAV_VTOL_LAND_OPTIONS_DEFAULT](#NAV_VTOL_LAND_OPTIONS_DEFAULT) | 默认自动驾驶仪着陆行为。                                     |
| <a id='NAV_VTOL_LAND_OPTIONS_FW_DESCENT'></a>1    | [NAV_VTOL_LAND_OPTIONS_FW_DESCENT](#NAV_VTOL_LAND_OPTIONS_FW_DESCENT) | 以固定翼模式下降，接近地面时转换为多旋翼模式进行垂直着陆。<br>固定翼下降模式由飞行器自行决定（例如过渡高度、盘旋方向、半径和速度等）。 |
| <a id='NAV_VTOL_LAND_OPTIONS_HOVER_DESCENT'></a>2 | [NAV_VTOL_LAND_OPTIONS_HOVER_DESCENT](#NAV_VTOL_LAND_OPTIONS_HOVER_DESCENT) | 到达着陆坐标后，以多旋翼模式着陆（整个着陆过程采用 "悬停下降 "方式）。 |

### MAV_WINCH_STATUS_FLAG 

(位掩码）[WINCH_STATUS](#WINCH_STATUS) 中使用的绞盘状态标记

| Value                                          | Name                                                         | Description                                                  |
| ---------------------------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| <a id='MAV_WINCH_STATUS_HEALTHY'></a>1         | [MAV_WINCH_STATUS_HEALTHY](#MAV_WINCH_STATUS_HEALTHY)        | 绞盘健康                                                     |
| <a id='MAV_WINCH_STATUS_FULLY_RETRACTED'></a>2 | [MAV_WINCH_STATUS_FULLY_RETRACTED](#MAV_WINCH_STATUS_FULLY_RETRACTED) | 绞车线完全收回                                               |
| <a id='MAV_WINCH_STATUS_MOVING'></a>4          | [MAV_WINCH_STATUS_MOVING](#MAV_WINCH_STATUS_MOVING)          | 绞盘电机正在移动                                             |
| <a id='MAV_WINCH_STATUS_CLUTCH_ENGAGED'></a>8  | [MAV_WINCH_STATUS_CLUTCH_ENGAGED](#MAV_WINCH_STATUS_CLUTCH_ENGAGED) | 绞盘离合器已接合，允许电机自由移动。                         |
| <a id='MAV_WINCH_STATUS_LOCKED'></a>16         | [MAV_WINCH_STATUS_LOCKED](#MAV_WINCH_STATUS_LOCKED)          | 绞盘由锁定装置锁住。                                         |
| <a id='MAV_WINCH_STATUS_DROPPING'></a>32       | [MAV_WINCH_STATUS_DROPPING](#MAV_WINCH_STATUS_DROPPING)      | 绞盘在重力作用下将有效载荷抛下。                             |
| <a id='MAV_WINCH_STATUS_ARRESTING'></a>64      | [MAV_WINCH_STATUS_ARRESTING](#MAV_WINCH_STATUS_ARRESTING)    | 绞盘正在阻止有效载荷下降。                                   |
| <a id='MAV_WINCH_STATUS_GROUND_SENSE'></a>128  | [MAV_WINCH_STATUS_GROUND_SENSE](#MAV_WINCH_STATUS_GROUND_SENSE) | 绞盘利用扭矩测量来感测地面。                                 |
| <a id='MAV_WINCH_STATUS_RETRACTING'></a>256    | [MAV_WINCH_STATUS_RETRACTING](#MAV_WINCH_STATUS_RETRACTING)  | 绞盘正在返回完全缩回位置。                                   |
| <a id='MAV_WINCH_STATUS_REDELIVER'></a>512     | [MAV_WINCH_STATUS_REDELIVER](#MAV_WINCH_STATUS_REDELIVER)    | 绞盘正在重新输送有效载荷。如果在回卷过程中缆线张力超过临界值，这是一种故障切换状态。 |
| <a id='MAV_WINCH_STATUS_ABANDON_LINE'></a>1024 | [MAV_WINCH_STATUS_ABANDON_LINE](#MAV_WINCH_STATUS_ABANDON_LINE) | 绞盘放弃了缆线，可能还放弃了有效载荷。绞盘释放整个计算出的线路长度。如果尝试次数超过阈值，这是从 REDELIVER 的故障切换状态。 |
| <a id='MAV_WINCH_STATUS_LOCKING'></a>2048      | [MAV_WINCH_STATUS_LOCKING](#MAV_WINCH_STATUS_LOCKING)        | 绞盘正在啮合锁定装置。                                       |
| <a id='MAV_WINCH_STATUS_LOAD_LINE'></a>4096    | [MAV_WINCH_STATUS_LOAD_LINE](#MAV_WINCH_STATUS_LOAD_LINE)    | 绞盘已上线。                                                 |
| <a id='MAV_WINCH_STATUS_LOAD_PAYLOAD'></a>8192 | [MAV_WINCH_STATUS_LOAD_PAYLOAD](#MAV_WINCH_STATUS_LOAD_PAYLOAD) | 绞盘正在装载有效载荷。                                       |

### MAG_CAL_STATUS 

| Value                                  | Name                                                  | Description |
| -------------------------------------- | ----------------------------------------------------- | ----------- |
| <a id='MAG_CAL_NOT_STARTED'></a>0      | [MAG_CAL_NOT_STARTED](#MAG_CAL_NOT_STARTED)           |             |
| <a id='MAG_CAL_WAITING_TO_START'></a>1 | [MAG_CAL_WAITING_TO_START](#MAG_CAL_WAITING_TO_START) |             |
| <a id='MAG_CAL_RUNNING_STEP_ONE'></a>2 | [MAG_CAL_RUNNING_STEP_ONE](#MAG_CAL_RUNNING_STEP_ONE) |             |
| <a id='MAG_CAL_RUNNING_STEP_TWO'></a>3 | [MAG_CAL_RUNNING_STEP_TWO](#MAG_CAL_RUNNING_STEP_TWO) |             |
| <a id='MAG_CAL_SUCCESS'></a>4          | [MAG_CAL_SUCCESS](#MAG_CAL_SUCCESS)                   |             |
| <a id='MAG_CAL_FAILED'></a>5           | [MAG_CAL_FAILED](#MAG_CAL_FAILED)                     |             |
| <a id='MAG_CAL_BAD_ORIENTATION'></a>6  | [MAG_CAL_BAD_ORIENTATION](#MAG_CAL_BAD_ORIENTATION)   |             |
| <a id='MAG_CAL_BAD_RADIUS'></a>7       | [MAG_CAL_BAD_RADIUS](#MAG_CAL_BAD_RADIUS)             |             |

### MAV_EVENT_ERROR_REASON 

事件错误响应的原因。

| Value                                            | Name                                                         | Description                    |
| ------------------------------------------------ | ------------------------------------------------------------ | ------------------------------ |
| <a id='MAV_EVENT_ERROR_REASON_UNAVAILABLE'></a>0 | [MAV_EVENT_ERROR_REASON_UNAVAILABLE](#MAV_EVENT_ERROR_REASON_UNAVAILABLE) | 请求的事件不可用（不再可用）。 |

### MAV_EVENT_CURRENT_SEQUENCE_FLAGS 

CURRENT_EVENT_SEQUENCE](#CURRENT_EVENT_SEQUENCE) 的标志。

| Value                                                | Name                                                         | Description                  |
| ---------------------------------------------------- | ------------------------------------------------------------ | ---------------------------- |
| <a id='MAV_EVENT_CURRENT_SEQUENCE_FLAGS_RESET'></a>1 | [MAV_EVENT_CURRENT_SEQUENCE_FLAGS_RESET](#MAV_EVENT_CURRENT_SEQUENCE_FLAGS_RESET) | 发生序列重置（如车辆重启）。 |

### HIL_SENSOR_UPDATED_FLAGS 

(位掩码） [HIL_SENSOR]（#HIL_SENSOR）报文中的标志指示自上次报文以来哪些字段已更新

| Value                                             | Name                                                         | Description                                            |
| ------------------------------------------------- | ------------------------------------------------------------ | ------------------------------------------------------ |
| <a id='HIL_SENSOR_UPDATED_NONE'></a>0             | [HIL_SENSOR_UPDATED_NONE](#HIL_SENSOR_UPDATED_NONE)          | None of the fields in [HIL_SENSOR](#HIL_SENSOR) 已更新 |
| <a id='HIL_SENSOR_UPDATED_XACC'></a>1             | [HIL_SENSOR_UPDATED_XACC](#HIL_SENSOR_UPDATED_XACC)          | xacc 字段中的值已更新                                  |
| <a id='HIL_SENSOR_UPDATED_YACC'></a>2             | [HIL_SENSOR_UPDATED_YACC](#HIL_SENSOR_UPDATED_YACC)          | yacc 字段中的值已更新                                  |
| <a id='HIL_SENSOR_UPDATED_ZACC'></a>4             | [HIL_SENSOR_UPDATED_ZACC](#HIL_SENSOR_UPDATED_ZACC)          | zacc 字段中的值已更新                                  |
| <a id='HIL_SENSOR_UPDATED_XGYRO'></a>8            | [HIL_SENSOR_UPDATED_XGYRO](#HIL_SENSOR_UPDATED_XGYRO)        | xgyro 字段中的值已更新                                 |
| <a id='HIL_SENSOR_UPDATED_YGYRO'></a>16           | [HIL_SENSOR_UPDATED_YGYRO](#HIL_SENSOR_UPDATED_YGYRO)        | ygyro 字段中的值已更新                                 |
| <a id='HIL_SENSOR_UPDATED_ZGYRO'></a>32           | [HIL_SENSOR_UPDATED_ZGYRO](#HIL_SENSOR_UPDATED_ZGYRO)        | zgyro 字段中的值已更新                                 |
| <a id='HIL_SENSOR_UPDATED_XMAG'></a>64            | [HIL_SENSOR_UPDATED_XMAG](#HIL_SENSOR_UPDATED_XMAG)          | xmag 字段中的值已更新                                  |
| <a id='HIL_SENSOR_UPDATED_YMAG'></a>128           | [HIL_SENSOR_UPDATED_YMAG](#HIL_SENSOR_UPDATED_YMAG)          | ymag 字段中的值已更新                                  |
| <a id='HIL_SENSOR_UPDATED_ZMAG'></a>256           | [HIL_SENSOR_UPDATED_ZMAG](#HIL_SENSOR_UPDATED_ZMAG)          | Tzmag 字段中的值已更新                                 |
| <a id='HIL_SENSOR_UPDATED_ABS_PRESSURE'></a>512   | [HIL_SENSOR_UPDATED_ABS_PRESSURE](#HIL_SENSOR_UPDATED_ABS_PRESSURE) | abs_pressure 字段中的值已更新                          |
| <a id='HIL_SENSOR_UPDATED_DIFF_PRESSURE'></a>1024 | [HIL_SENSOR_UPDATED_DIFF_PRESSURE](#HIL_SENSOR_UPDATED_DIFF_PRESSURE) | 已更新 diff_pressure 字段中的值                        |
| <a id='HIL_SENSOR_UPDATED_PRESSURE_ALT'></a>2048  | [HIL_SENSOR_UPDATED_PRESSURE_ALT](#HIL_SENSOR_UPDATED_PRESSURE_ALT) | 已更新 pressure_alt 字段中的值                         |
| <a id='HIL_SENSOR_UPDATED_TEMPERATURE'></a>4096   | [HIL_SENSOR_UPDATED_TEMPERATURE](#HIL_SENSOR_UPDATED_TEMPERATURE) | 温度字段中的值已更新                                   |
| <a id='HIL_SENSOR_UPDATED_RESET'></a>2147483648   | [HIL_SENSOR_UPDATED_RESET](#HIL_SENSOR_UPDATED_RESET)        | 在模拟（第 31 位）中对姿态/位置/速度等进行了全面重置。 |

### HIGHRES_IMU_UPDATED_FLAGS 

(位掩码）[HIGHRES_IMU]（#HIGHRES_IMU）报文中的标志指示自上次报文以来哪些字段已更新

| Value                                              | Name                                                         | Description                                      |
| -------------------------------------------------- | ------------------------------------------------------------ | ------------------------------------------------ |
| <a id='HIGHRES_IMU_UPDATED_NONE'></a>0             | [HIGHRES_IMU_UPDATED_NONE](#HIGHRES_IMU_UPDATED_NONE)        | 没有更新 [HIGHRES_IMU](#HIGHRES_IMU)中的任何字段 |
| <a id='HIGHRES_IMU_UPDATED_XACC'></a>1             | [HIGHRES_IMU_UPDATED_XACC](#HIGHRES_IMU_UPDATED_XACC)        | xacc 字段中的值已更新                            |
| <a id='HIGHRES_IMU_UPDATED_YACC'></a>2             | [HIGHRES_IMU_UPDATED_YACC](#HIGHRES_IMU_UPDATED_YACC)        | yacc 字段中的值已更新                            |
| <a id='HIGHRES_IMU_UPDATED_ZACC'></a>4             | [HIGHRES_IMU_UPDATED_ZACC](#HIGHRES_IMU_UPDATED_ZACC)        | zacc字段中的值已更新                             |
| <a id='HIGHRES_IMU_UPDATED_XGYRO'></a>8            | [HIGHRES_IMU_UPDATED_XGYRO](#HIGHRES_IMU_UPDATED_XGYRO)      | xgyro 字段中的值已更新                           |
| <a id='HIGHRES_IMU_UPDATED_YGYRO'></a>16           | [HIGHRES_IMU_UPDATED_YGYRO](#HIGHRES_IMU_UPDATED_YGYRO)      | ygyro 字段中的值已更新                           |
| <a id='HIGHRES_IMU_UPDATED_ZGYRO'></a>32           | [HIGHRES_IMU_UPDATED_ZGYRO](#HIGHRES_IMU_UPDATED_ZGYRO)      | zgyro 字段中的值已更新                           |
| <a id='HIGHRES_IMU_UPDATED_XMAG'></a>64            | [HIGHRES_IMU_UPDATED_XMAG](#HIGHRES_IMU_UPDATED_XMAG)        | xmag 字段中的值已更新                            |
| <a id='HIGHRES_IMU_UPDATED_YMAG'></a>128           | [HIGHRES_IMU_UPDATED_YMAG](#HIGHRES_IMU_UPDATED_YMAG)        | ymag 字段中的值已更新                            |
| <a id='HIGHRES_IMU_UPDATED_ZMAG'></a>256           | [HIGHRES_IMU_UPDATED_ZMAG](#HIGHRES_IMU_UPDATED_ZMAG)        | zmag 字段中的值已更新                            |
| <a id='HIGHRES_IMU_UPDATED_ABS_PRESSURE'></a>512   | [HIGHRES_IMU_UPDATED_ABS_PRESSURE](#HIGHRES_IMU_UPDATED_ABS_PRESSURE) | abs_pressure 字段中的值已更新                    |
| <a id='HIGHRES_IMU_UPDATED_DIFF_PRESSURE'></a>1024 | [HIGHRES_IMU_UPDATED_DIFF_PRESSURE](#HIGHRES_IMU_UPDATED_DIFF_PRESSURE) | 已更新 diff_pressure 字段中的值                  |
| <a id='HIGHRES_IMU_UPDATED_PRESSURE_ALT'></a>2048  | [HIGHRES_IMU_UPDATED_PRESSURE_ALT](#HIGHRES_IMU_UPDATED_PRESSURE_ALT) | 已更新 pressure_alt 字段中的值                   |
| <a id='HIGHRES_IMU_UPDATED_TEMPERATURE'></a>4096   | [HIGHRES_IMU_UPDATED_TEMPERATURE](#HIGHRES_IMU_UPDATED_TEMPERATURE) | 温度字段中的值已更新                             |
| <a id='HIGHRES_IMU_UPDATED_ALL'></a>65535          | [HIGHRES_IMU_UPDATED_ALL](#HIGHRES_IMU_UPDATED_ALL)          | 已更新 [HIGHRES_IMU](#HIGHRES_IMU)中的所有字段。 |

### CAN_FILTER_OP 

| Value                            | Name                                      | Description |
| -------------------------------- | ----------------------------------------- | ----------- |
| <a id='CAN_FILTER_REPLACE'></a>0 | [CAN_FILTER_REPLACE](#CAN_FILTER_REPLACE) |             |
| <a id='CAN_FILTER_ADD'></a>1     | [CAN_FILTER_ADD](#CAN_FILTER_ADD)         |             |
| <a id='CAN_FILTER_REMOVE'></a>2  | [CAN_FILTER_REMOVE](#CAN_FILTER_REMOVE)   |             |

### MAV_FTP_ERR 

MAV FTP 错误代码 (https://mavlink.io/en/services/ftp.html)

| Value                                         | Name                                                         | Description                                                  |
| --------------------------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| <a id='MAV_FTP_ERR_NONE'></a>0                | [MAV_FTP_ERR_NONE](#MAV_FTP_ERR_NONE)                        | None： 无错误                                                |
| <a id='MAV_FTP_ERR_FAIL'></a>1                | [MAV_FTP_ERR_FAIL](#MAV_FTP_ERR_FAIL)                        | Fail： 未知故障                                              |
| <a id='MAV_FTP_ERR_FAILERRNO'></a>2           | [MAV_FTP_ERR_FAILERRNO](#MAV_FTP_ERR_FAILERRNO)              | FailErrno：命令失败，错误编号在 PayloadHeader.data[1] 中发回。<br>这是服务器操作系统理解的文件系统错误编号。 |
| <a id='MAV_FTP_ERR_INVALIDDATASIZE'></a>3     | [MAV_FTP_ERR_INVALIDDATASIZE](#MAV_FTP_ERR_INVALIDDATASIZE)  | InvalidDataSize： 有效负载大小无效                           |
| <a id='MAV_FTP_ERR_INVALIDSESSION'></a>4      | [MAV_FTP_ERR_INVALIDSESSION](#MAV_FTP_ERR_INVALIDSESSION)    | InvalidSession： 会话当前未打开                              |
| <a id='MAV_FTP_ERR_NOSESSIONSAVAILABLE'></a>5 | [MAV_FTP_ERR_NOSESSIONSAVAILABLE](#MAV_FTP_ERR_NOSESSIONSAVAILABLE) | NoSessionsAvailable： 所有可用会话已被使用                   |
| <a id='MAV_FTP_ERR_EOF'></a>6                 | [MAV_FTP_ERR_EOF](#MAV_FTP_ERR_EOF)                          | EOF：ListDirectory 和 ReadFile 命令的文件末尾偏移量          |
| <a id='MAV_FTP_ERR_UNKNOWNCOMMAND'></a>7      | [MAV_FTP_ERR_UNKNOWNCOMMAND](#MAV_FTP_ERR_UNKNOWNCOMMAND)    | UnknownCommand: 未知命令/操作码                              |
| <a id='MAV_FTP_ERR_FILEEXISTS'></a>8          | [MAV_FTP_ERR_FILEEXISTS](#MAV_FTP_ERR_FILEEXISTS)            | FileExists: 文件/目录已存在                                  |
| <a id='MAV_FTP_ERR_FILEPROTECTED'></a>9       | [MAV_FTP_ERR_FILEPROTECTED](#MAV_FTP_ERR_FILEPROTECTED)      | FileProtected: 文件/目录受写入保护                           |
| <a id='MAV_FTP_ERR_FILENOTFOUND'></a>10       | [MAV_FTP_ERR_FILENOTFOUND](#MAV_FTP_ERR_FILENOTFOUND)        | FileNotFound: 未找到文件/目录                                |

### MAV_FTP_OPCODE 

MAV FTP 操作码： https://mavlink.io/en/services/ftp.html

| Value                                         | Name                                                         | Description                                       |
| --------------------------------------------- | ------------------------------------------------------------ | ------------------------------------------------- |
| <a id='MAV_FTP_OPCODE_NONE'></a>0             | [MAV_FTP_OPCODE_NONE](#MAV_FTP_OPCODE_NONE)                  | None. 忽略，始终 ACK                              |
| <a id='MAV_FTP_OPCODE_TERMINATESESSION'></a>1 | [MAV_FTP_OPCODE_TERMINATESESSION](#MAV_FTP_OPCODE_TERMINATESESSION) | TerminateSession: 终止开放式 "阅读 "会话          |
| <a id='MAV_FTP_OPCODE_RESETSESSION'></a>2     | [MAV_FTP_OPCODE_RESETSESSION](#MAV_FTP_OPCODE_RESETSESSION)  | ResetSessions: 终止所有打开的读取会话             |
| <a id='MAV_FTP_OPCODE_LISTDIRECTORY'></a>3    | [MAV_FTP_OPCODE_LISTDIRECTORY](#MAV_FTP_OPCODE_LISTDIRECTORY) | ListDirectory. 从偏移量开始列出路径中的文件和目录 |
| <a id='MAV_FTP_OPCODE_OPENFILERO'></a>4       | [MAV_FTP_OPCODE_OPENFILERO](#MAV_FTP_OPCODE_OPENFILERO)      | OpenFileRO: 打开路径上的文件供读取，返回会话      |
| <a id='MAV_FTP_OPCODE_READFILE'></a>5         | [MAV_FTP_OPCODE_READFILE](#MAV_FTP_OPCODE_READFILE)          | ReadFile: 从会话中的偏移量读取大小字节            |
| <a id='MAV_FTP_OPCODE_CREATEFILE'></a>6       | [MAV_FTP_OPCODE_CREATEFILE](#MAV_FTP_OPCODE_CREATEFILE)      | CreateFile: 在路径上创建文件以供写入，返回会话    |
| <a id='MAV_FTP_OPCODE_WRITEFILE'></a>7        | [MAV_FTP_OPCODE_WRITEFILE](#MAV_FTP_OPCODE_WRITEFILE)        | WriteFile: 将大小字节写入会话中的偏移量           |
| <a id='MAV_FTP_OPCODE_REMOVEFILE'></a>8       | [MAV_FTP_OPCODE_REMOVEFILE](#MAV_FTP_OPCODE_REMOVEFILE)      | RemoveFile: 删除路径上的文件                      |
| <a id='MAV_FTP_OPCODE_CREATEDIRECTORY'></a>9  | [MAV_FTP_OPCODE_CREATEDIRECTORY](#MAV_FTP_OPCODE_CREATEDIRECTORY) | CreateDirectory: 在路径下创建目录                 |
| <a id='MAV_FTP_OPCODE_REMOVEDIRECTORY'></a>10 | [MAV_FTP_OPCODE_REMOVEDIRECTORY](#MAV_FTP_OPCODE_REMOVEDIRECTORY) | RemoveDirectory: 删除路径下的目录。目录必须为空。 |
| <a id='MAV_FTP_OPCODE_OPENFILEWO'></a>11      | [MAV_FTP_OPCODE_OPENFILEWO](#MAV_FTP_OPCODE_OPENFILEWO)      | OpenFileWO: 打开路径上的文件以供写入，返回会话    |
| <a id='MAV_FTP_OPCODE_TRUNCATEFILE'></a>12    | [MAV_FTP_OPCODE_TRUNCATEFILE](#MAV_FTP_OPCODE_TRUNCATEFILE)  | TruncateFile: 截断文件的路径偏移长度              |
| <a id='MAV_FTP_OPCODE_RENAME'></a>13          | [MAV_FTP_OPCODE_RENAME](#MAV_FTP_OPCODE_RENAME)              | Rename: 将路径 1 重命名为路径 2                   |
| <a id='MAV_FTP_OPCODE_CALCFILECRC'></a>14     | [MAV_FTP_OPCODE_CALCFILECRC](#MAV_FTP_OPCODE_CALCFILECRC)    | CalcFileCRC32:  计算路径上文件的 CRC32            |
| <a id='MAV_FTP_OPCODE_BURSTREADFILE'></a>15   | [MAV_FTP_OPCODE_BURSTREADFILE](#MAV_FTP_OPCODE_BURSTREADFILE) | BurstReadFile:  突发下载会话文件                  |
| <a id='MAV_FTP_OPCODE_ACK'></a>128            | [MAV_FTP_OPCODE_ACK](#MAV_FTP_OPCODE_ACK)                    | ACK: ACK 响应                                     |
| <a id='MAV_FTP_OPCODE_NAK'></a>129            | [MAV_FTP_OPCODE_NAK](#MAV_FTP_OPCODE_NAK)                    | NAK: NAK 响应                                     |

### MISSION_STATE 

任务状态机的状态。
请注意，这些状态与任务是否处于可以执行任务项目的模式（暂停）无关。
它们不一定对所有飞行器都适用。

| Value                                   | Name                                                    | Description                                                |
| --------------------------------------- | ------------------------------------------------------- | ---------------------------------------------------------- |
| <a id='MISSION_STATE_UNKNOWN'></a>0     | [MISSION_STATE_UNKNOWN](#MISSION_STATE_UNKNOWN)         | 不支持任务状态报告。                                       |
| <a id='MISSION_STATE_NO_MISSION'></a>1  | [MISSION_STATE_NO_MISSION](#MISSION_STATE_NO_MISSION)   | 车上没有任务。                                             |
| <a id='MISSION_STATE_NOT_STARTED'></a>2 | [MISSION_STATE_NOT_STARTED](#MISSION_STATE_NOT_STARTED) | 任务尚未开始。这种情况发生在任务已上传但尚未开始执行之后。 |
| <a id='MISSION_STATE_ACTIVE'></a>3      | [MISSION_STATE_ACTIVE](#MISSION_STATE_ACTIVE)           | 任务已激活，在自动模式下将执行任务项目。                   |
| <a id='MISSION_STATE_PAUSED'></a>4      | [MISSION_STATE_PAUSED](#MISSION_STATE_PAUSED)           | 任务在自动模式下暂停。                                     |
| <a id='MISSION_STATE_COMPLETE'></a>5    | [MISSION_STATE_COMPLETE](#MISSION_STATE_COMPLETE)       | 任务已执行所有任务项目。                                   |

### SAFETY_SWITCH_STATE 

可能的安全开关状态。

| Value                                       | Name                                                         | Description                                              |
| ------------------------------------------- | ------------------------------------------------------------ | -------------------------------------------------------- |
| <a id='SAFETY_SWITCH_STATE_SAFE'></a>0      | [SAFETY_SWITCH_STATE_SAFE](#SAFETY_SWITCH_STATE_SAFE)        | 安全开关已启动，车辆应可安全接近。                       |
| <a id='SAFETY_SWITCH_STATE_DANGEROUS'></a>1 | [SAFETY_SWITCH_STATE_DANGEROUS](#SAFETY_SWITCH_STATE_DANGEROUS) | 安全开关未接通，电机、螺旋桨和其他执行器应视为激活状态。 |

### ILLUMINATOR_MODE 

照明器的模式

| Value                                           | Name                                                         | Description                                                  |
| ----------------------------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| <a id='ILLUMINATOR_MODE_UNKNOWN'></a>0          | [ILLUMINATOR_MODE_UNKNOWN](#ILLUMINATOR_MODE_UNKNOWN)        | 照明器模式未指定/未知                                        |
| <a id='ILLUMINATOR_MODE_INTERNAL_CONTROL'></a>1 | [ILLUMINATOR_MODE_INTERNAL_CONTROL](#ILLUMINATOR_MODE_INTERNAL_CONTROL) | 照明器行为由 [MAV_CMD_DO_ILLUMINATOR_CONFIGURE]（#MAV_CMD_DO_ILLUMINATOR_CONFIGURE）设置控制。 |
| <a id='ILLUMINATOR_MODE_EXTERNAL_SYNC'></a>2    | [ILLUMINATOR_MODE_EXTERNAL_SYNC](#ILLUMINATOR_MODE_EXTERNAL_SYNC) | 照明器的行为受外部因素控制：例如外部硬件信号                 |

### ILLUMINATOR_ERROR_FLAGS 

（位图） 照明器模块错误标志（位图，0 表示无错误）

| Value                                                        | Name                                                         | Description          |
| ------------------------------------------------------------ | ------------------------------------------------------------ | -------------------- |
| <a id='ILLUMINATOR_ERROR_FLAGS_THERMAL_THROTTLING'></a>1     | [ILLUMINATOR_ERROR_FLAGS_THERMAL_THROTTLING](#ILLUMINATOR_ERROR_FLAGS_THERMAL_THROTTLING) | 照明器热节流错误。   |
| <a id='ILLUMINATOR_ERROR_FLAGS_OVER_TEMPERATURE_SHUTDOWN'></a>2 | [ILLUMINATOR_ERROR_FLAGS_OVER_TEMPERATURE_SHUTDOWN](#ILLUMINATOR_ERROR_FLAGS_OVER_TEMPERATURE_SHUTDOWN) | 照明器超温关机错误。 |
| <a id='ILLUMINATOR_ERROR_FLAGS_THERMISTOR_FAILURE'></a>4     | [ILLUMINATOR_ERROR_FLAGS_THERMISTOR_FAILURE](#ILLUMINATOR_ERROR_FLAGS_THERMISTOR_FAILURE) | 照明器热敏电阻故障。 |

### MAV_AUTOPILOT — \[from: [minimal](../messages/minimal.md#MAV_AUTOPILOT)\] 

### MAV_TYPE — \[from: [minimal](../messages/minimal.md#MAV_TYPE)\] 

### MAV_MODE_FLAG — \[from: [minimal](../messages/minimal.md#MAV_MODE_FLAG)\] 

### MAV_MODE_FLAG_DECODE_POSITION — \[from: [minimal](../messages/minimal.md#MAV_MODE_FLAG_DECODE_POSITION)\] 

### MAV_STATE — \[from: [minimal](../messages/minimal.md#MAV_STATE)\] 

### MAV_COMPONENT — \[from: [minimal](../messages/minimal.md#MAV_COMPONENT)\] 

## Commands (MAV_CMD) 

### MAV_CMD_NAV_WAYPOINT (16) 

导航至航点。该功能用于任务中（任务外的引导指令请使用 [MAV_CMD_DO_REPOSITION](#MAV_CMD_DO_REPOSITION)）。

| 参数（标签）                | 说明                                                         | 数值       | 单位 |
| --------------------------- | ------------------------------------------------------------ | ---------- | ---- |
| 1 Hold（保持）              | 保持时间。(固定翼忽略，旋转翼停留在航点的时间）              | 最小值： 0 | s    |
| 2 Accept Radius（接受半径） | 接受半径（如果击中具有此半径的球体，则航点视为已到达）       | 最小： 0   | m    |
| 3 Pass Radius（通过半径）   | 半径为 0 时通过 WP，半径大于 0 时通过 WP。正值表示顺时针轨道，负值表示逆时针轨道。允许轨迹控制。 |            | m    |
| 4 (Yaw)                     | 预期航点偏航角（旋翼）。NaN 表示使用当前系统的偏航航向模式（如偏向下一个航点、偏向原点等）。 |            | deg  |
| 5 Latitude（纬度            |                                                              |            |      |
| 6 Longitude（经度）         | 经度                                                         |            |      |
| 7 Altitude(高度)            | 高度                                                         |            | m    |


### MAV_CMD_NAV_LOITER_UNLIM (17) 

在该航点附近无限时停泊

| 参数（标签）       | 说明                                                         | 单位 |
| ------------------ | ------------------------------------------------------------ | ---- |
| 1                  | 空                                                           |      |
| 2                  | 空                                                           |      |
| 3 (Radius)         | 仅向前移动的车辆（非多旋翼飞行器）在航点周围的停机半径。如果为正值，则顺时针停放，否则逆时针停放 | m    |
| 4 (Yaw)            | 预期偏航角度。NaN 表示使用当前系统的偏航航向模式（例如，偏航至下一个航点、偏航至原点等）。 | deg  |
| 5 (Latitude)       | 纬度                                                         |      |
| 6 (Longitude)      | 经度                                                         |      |
| 7 Altitude（高度） | 高度                                                         | m    |


### MAV_CMD_NAV_LOITER_TURNS (18) 

在该航点周围停泊 X 个回合

| 参数（标签）         | 说明                                                         | 数值                  | 单位 |
| -------------------- | ------------------------------------------------------------ | --------------------- | ---- |
| 1 Turns(转数)        | 转数。                                                       | 最小值：0             |      |
| 2 (Heading Required) | 只有在朝向下一个航点时才离开停机圈（0 = False）              | min: 0 max： 1 inc: 1 |      |
| 3 (Radius)           | 仅向前行驶的车辆（非多旋翼飞行器）围绕航点的停机半径。如果为正值，则顺时针停放，否则逆时针停放 |                       | m    |
| 4 (Xtrack Location)  | 仅向前移动的车辆（非多旋翼飞行器）的停放圈出口位置和/或通往下一个航点的路径（"xtrack"）。0 表示车辆离开停机坪时向中心 XTRACK（当前航点和下一个航点中心之间的直线）靠拢，1 表示向车辆从停机坪半径出口位置到下一个航点之间的直线靠拢。否则，车辆必须在徘徊半径切线与中心 X 轨迹之间的角度（单位：度）离开徘徊半径（并向中心 X 轨迹靠拢）。如果为 NaN，则使用当前系统默认的 X 轴行为。 |                       |      |
| 5（Latitude纬度）    | 纬度                                                         |                       |      |
| 6 (Longitude)        | 经度                                                         |                       |      |
| 7 Altitude(高度)     | 高度                                                         |                       | m    |


### MAV_CMD_NAV_LOITER_TIME (19) 

在指定的经纬度和高度停留一定时间。多旋翼飞行器停在该点（在特定飞行器的接受半径内）。仅向前移动的飞行器（如固定翼飞行器）以指定的半径/方向环绕该点。如果 "所需航向 "参数 (2) 不为零，则前向飞行器只有在飞往下一个航点时才会离开停机坪。

| 参数（标签）         | 说明                                                         | 数值                  | 单位 |
| -------------------- | ------------------------------------------------------------ | --------------------- | ---- |
| 1（Time时间）        | Loiter time（仅在到达纬度、经度和高度后才开始）。            | 最小值：0             | s    |
| 2 (Heading Required) | 只有在飞往下一个航点时才离开停机圈（0 = False）              | min: 0 max： 1 inc: 1 |      |
| 3 (Radius)           | 仅向前行驶的车辆（非多旋翼飞行器）围绕航点的停机半径。如果为正，则按顺时针方向停放，否则按逆时针方向停放。 |                       | m    |
| 4 (Xtrack Location)  | 仅向前移动的车辆（非多旋翼飞行器）的停放圈出口位置和/或通往下一个航点的路径（"xtrack"）。0 表示车辆离开停机坪时向中心 XTRACK（当前航点和下一个航点中心之间的直线）靠拢，1 表示向车辆从停机坪半径出口位置到下一个航点之间的直线靠拢。否则，车辆必须在徘徊半径切线与中心 X 轨迹之间的角度（单位：度）离开徘徊半径（并向中心 X 轨迹靠拢）。如果为 NaN，则使用当前系统默认的 X 轴行为。 |                       |      |
| 5（Latitude纬度）    | 纬度                                                         |                       |      |
| 6 (Longitude)        | 经度                                                         |                       |      |
| 7 Altitude(高度)     | 高度                                                         |                       | m    |


### MAV_CMD_NAV_RETURN_TO_LAUNCH (20) 

返回发射地点

| Param (Label) | Description |
| ------------- | ----------- |
| 1             | Empty       |
| 2             | Empty       |
| 3             | Empty       |
| 4             | Empty       |
| 5             | Empty       |
| 6             | Empty       |
| 7             | Empty       |


### MAV_CMD_NAV_LAND (21) 

地点的土地。

| 参数（标签）            | 描述                                                         | 数值                | 单位 |
| ----------------------- | ------------------------------------------------------------ | ------------------- | ---- |
| 1 (Abort Alt)           | 中止着陆时的最小目标高度（0 = 未定义/使用系统默认值）。      |                     | m    |
| 2 Land Mode（着陆模式） | 精确着陆模式。                                               | PRECISION_LAND_MODE |      |
| 3                       | 空。                                                         |                     |      |
| 4 (Yaw Angle)           | 预期偏航角度。NaN 表示使用当前系统的偏航航向模式（例如，偏航至下一个航点、偏航至原点等）。 |                     | deg  |
| 5 Latitude（纬度）      | 纬度。                                                       |                     |      |
| 6 Longitude（经度）     | 经度。                                                       |                     |      |
| 7 Altitude(高度)        | 着陆高度（当前帧中的地面高度）。                             |                     | m    |


### MAV_CMD_NAV_TAKEOFF (22) 

从地面/手部起飞。支持多种起飞模式的飞行器（如 VTOL 四翼机）应使用当前配置的模式起飞。

| 参数（标签）       | 说明                                                         | 单位 |
| ------------------ | ------------------------------------------------------------ | ---- |
| 1 Pitch（螺距）    | 最小螺距（如果存在空速传感器），无传感器时的期望螺距         | deg  |
| 2                  | 空                                                           |      |
| 3                  | 空                                                           |      |
| 4 (Yaw)            | 偏航角（如果有磁力计），无磁力计时忽略。NaN 表示使用当前系统的偏航航向模式（如偏航至下一个航点、偏航至原点等）。 | deg  |
| 5 （Latitude纬度） | 纬度                                                         | 纬度 |
| 6 (Longitude)      | 经度                                                         |      |
| 7 Altitude（高度） | 高度                                                         | m    |


### MAV_CMD_NAV_LAND_LOCAL (23) 

在本地位置着陆（仅限本地框架）

| 参数（标签）               | 说明                                                         | 数值              | 单位       |
| -------------------------- | ------------------------------------------------------------ | ----------------- | ---------- |
| 1 (Target)                 | 着陆目标编号（如果有）                                       | 最小： 0 最大： 1 | 2 (Offset) |
| 2 Offset（偏移量）         | 预期着陆位置的最大可接受偏移量 - 根据球形坐标计算出的大小：d = sqrt(x^2 + y^2 + z^2)，它给出了预期着陆位置与飞行器即将着陆位置之间的最大可接受距离 | 最小： 0          | m          |
| 3 Descend Rate（下降速率） | 着陆下降速率                                                 |                   | m/s        |
| 4 (Yaw)                    | 预期偏航角度                                                 |                   | rad        |
| 5 (Y Position)             | Y 轴位置                                                     |                   | m          |
| 6 (X 位置)                 | X 轴位置                                                     |                   | m          |
| 7 (Z 位置)                 | Z 轴/地面位置                                                |                   | m          |


### MAV_CMD_NAV_TAKEOFF_LOCAL (24) 

从本地位置起飞（仅限本地框架）

| 参数（标签）              | 说明                                                         | 单位  |
| ------------------------- | ------------------------------------------------------------ | ----- |
| 1 (Pitch)                 | 最小螺距（如果存在空速传感器），无传感器时的期望螺距         | rad   |
| 2                         | 空                                                           |       |
| 3 Ascend Rate（上升速率） | 起飞上升速率                                                 | 米/秒 |
| 4 Yaw（偏航）             | 偏航角（如果有磁力计或其他偏航估计源），没有这些估计源时忽略 | rad   |
| 5 (Y Position)            | Y 轴位置                                                     | m     |
| 6 (X Position)            | X 轴位置                                                     | m     |
| 7 (Z Position)            | Z 轴位置                                                     | m     |


### MAV_CMD_NAV_FOLLOW (25) 

车辆跟随，即该航点代表移动车辆的位置

| 参数（标签）               | 说明                                                         | 数值   | 单位          |
| -------------------------- | ------------------------------------------------------------ | ------ | ------------- |
| 1 Following（跟随）        | 使用的跟随逻辑（如闲逛或正弦波跟随） - 取决于具体的自动驾驶仪实现 | inc: 1 | 2（地面速度） |
| 2 Ground Speed（地面速度） | 跟随车辆的地面速度                                           |        | m/s           |
| 3 (Radius)                 | 航点周围的半径。如果为正值，则顺时针巡航，否则逆时针巡航     |        | m             |
| 4 (Yaw)                    | 预期偏航角度。                                               |        | 度            |
| 5 Latitude（纬度）         | 纬度                                                         |        |               |
| 6 (Longitude)              | 经度                                                         |        |               |
| 7 (高度)                   | 高度                                                         |        | 米            |


### MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT (30) 

继续当前航向并爬升/下降至指定高度。 达到指定高度后，继续执行下一条指令（即在达到所需高度之前不执行下一条指令）。

| 参数（标签）     | 说明                                                         | 数值                 | 单位 |
| ---------------- | ------------------------------------------------------------ | -------------------- | ---- |
| 1 Action(动作)   | 爬升或下降（0 = 中立，当在该指令高度的 5 米范围内时指令完成，1 = 爬升，当在该指令高度或以上时指令完成，2 = 下降，当在该指令高度或以下时指令完成）。 | min: 0 max: 2 inc: 1 |      |
| 2                | 空                                                           |                      |      |
| 3                | 空                                                           |                      |      |
| 4                | 空                                                           |                      |      |
| 5                | 空                                                           |                      |      |
| 6                | 空                                                           |                      |      |
| 7 Altitude(高度) |                                                              |                      | m    |


### MAV_CMD_NAV_LOITER_TO_ALT (31) 

在指定的纬度和经度处开始徘徊。 如果纬度=经度=0，则在当前位置闲逛。 在达到高度之前，不要认为导航指令已经完成（不要离开徘徊）。此外，如果 "所需航向 "参数不为零，则飞机在飞往下一个航点之前不会离开徘徊状态。

| 参数（标签）         | 说明                                                         | 数值                     | 单位 |
| -------------------- | ------------------------------------------------------------ | ------------------------ | ---- |
| 1 (Heading Required) | 只有在驶向下一个航点时才离开停机坪（0 = False）              | min: 0 max： 1 inc: 1    |      |
| 2 Radius（半径）     | 仅向前移动的车辆（非多旋翼飞行器）围绕航点的停机半径。如果为正值，则顺时针停放；如果为负值，则逆时针停放；0 表示不改变标准停放。 |                          | m    |
| 3                    | 空                                                           |                          |      |
| 4 (Xtrack Location)  | 仅向前移动的车辆（非多旋翼飞行器）的停机坪出口位置和/或通往下一个航点的路径（"xtrack"）。0 表示车辆在离开停机坪时向中心 XTRACK（当前航点和下一个航点中心之间的直线）靠拢，1 表示向车辆从停机坪半径出口位置到下一个航点之间的直线靠拢。否则，车辆必须在徘徊半径切线与中心 X 轨迹之间的角度（单位：度）离开徘徊半径（并向中心 X 轨迹靠拢）。如果使用 NaN 表示使用当前系统默认的 X 轴行为。 | 最小：0 最大：1 1 inc: 1 |      |
| 5 (Latitude)         | 纬度                                                         |                          |      |
| 6 (Longitude)        | 经度                                                         |                          |      |
| 7 (Altitude)         | 高度                                                         | m                        |      |


### MAV_CMD_DO_FOLLOW (32) 

开始跟踪目标

| 参数（标签）                  | 说明                                                         | 数值                         | 单位 |
| ----------------------------- | ------------------------------------------------------------ | ---------------------------- | ---- |
| 1 (System ID)                 | 系统 ID（FOLLOW_TARGET 信标的系统 ID）。发送 0 将禁用 follow-me，并返回默认位置保持模式。 | 最小值：0 最大值：255 inc: 1 |      |
| 2                             | 保留                                                         |                              |      |
| 3 保留                        |                                                              |                              |      |
| 4 （高度模式）                | 高度模式： 0：保持当前高度；1：保持与目标之间的高度差；2：进入原点上方的固定高度。 | 最小：0 最大：2 收入：1      |      |
| 5 (高度)                      | 原点上方的高度。(模式=2 时使用）                             |                              | m    |
| 6 保留                        |                                                              |                              |      |
| 7 (Time to Land) （着陆时间） | 在信息 RX 超时后，MAV 应转入默认位置保持模式的着陆时间。     | 最小值：0                    | 秒   |


### MAV_CMD_DO_FOLLOW_REPOSITION (33) 

发送跟随目标指令后重新定位 MAV

| 参数（标签）   | 说明                                             | 单位          |
| -------------- | ------------------------------------------------ | ------------- |
| 1 (Camera Q1)  | Camera q1（其中 0 位于从相机到跟踪设备的射线上） |               |
| 2（摄像机 Q2） | 摄像机 q2                                        |               |
| 3（摄像机 Q3） | 摄像机 q3                                        |               |
| 4 (Camera Q4)  | Camera q4                                        | 4 (Camera Q5) |
| 5 (高度偏移)   | 与目标的高度偏移                                 | m             |
| 6 (X 偏移)     | X 偏离目标                                       | m             |
| 7 (Y 偏移)     | 偏离目标的 Y 偏移                                | m             |


### MAV_CMD_DO_ORBIT (34) — [WIP] 

<span class="warning">**WORK IN PROGRESS**: 请勿在稳定的生产环境中使用（可能会发生变化）</span>

在参数定义的圆周上开始运行。将数值设置为 NaN/INT32_MAX（视情况而定）将导致使用默认值。

| 参数（标签）    | 说明                                                         | 数值                              | 单位 |
| --------------- | ------------------------------------------------------------ | --------------------------------- | ---- |
| 1 (Radius)      | 圆的半径。正数：顺时针轨道。负数：逆时针运行。NaN：使用车辆默认半径，如果已在运行，则使用当前半径。 |                                   | m    |
| 2（速度）       | 切向速度。NaN：使用车辆默认速度，如果已进入轨道，则使用当前速度。 | m/s                               |      |
| 3（偏航行为）   | 飞行器的偏航行为。                                           | 3 （偏航行为） 飞行器的偏航行为。 |      |
| 4 (Orbits)      | 围绕中心点运行多少弧度（例如，四分之三轨道设置为 270*Pi/180）。0：永远运行。NaN：使用车辆默认值，如果已经在运行，则使用当前值。 | min: 0                            | rad  |
| 5 (Latitude/X)  | 中心点纬度（如果未指定 MAV_FRAME）/根据 MAV_FRAME 确定的 X 坐标。INT32_MAX（如果在 COMMAND_LONG 中发送，则为 NaN）： 使用当前飞行器位置，如果已在轨道上，则使用当前中心位置。 |                                   |      |
| 6 (Longitude/Y) | 中心点经度（如果未指定 MAV_FRAME）/根据 MAV_FRAME 确定的 Y 坐标。INT32_MAX（如果在 COMMAND_LONG 中发送，则为 NaN）： 使用当前飞行器位置，如果已在轨道上，则使用当前中心位置。 |                                   |      |
| 7 (Altitude/Z)  | 中心点高度（MSL）（如果未指定 MAV_FRAME）/根据 MAV_FRAME 确定的 Z 坐标。NaN: 使用当前飞行器高度。 |                                   |      |


### MAV_CMD_NAV_ROI (80) — [DEP] 

<span class="warning">**DEPRECATED:** Replaced By MAV_CMD_DO_SET_ROI_* (2018-01)</span>

为传感器集或车辆本身设置感兴趣区域（ROI）。然后，飞行器的控制系统就可以用它来控制飞行器的姿态和各种传感器（如摄像头）的姿态。

| 参数（标签）   | 说明                                   | 数值                |
| -------------- | -------------------------------------- | ------------------- |
| 1 （ROI 模式） | 感兴趣区域模式。                       | [mav_roi](#mav_roi) |
| 2 (WP Index)   | 航点索引/目标 ID。(参见 MAV_ROI 枚举） | 最小： 0 最大： 1   |
| 3 (ROI Index)  | ROI 索引（允许车辆管理多个 ROI）       | min: 0 inc: 1       |
| 4              | 空                                     |                     |
| 5 (X)          | x 固定 ROI 的位置（请参阅 MAV_FRAME）  | 6 (Y)               |
| 6 (Y)          | y                                      |                     |
| 7 (Z)          | z                                      |                     |


### MAV_CMD_NAV_PATHPLANNING (81) 

控制 MAV 的自主路径规划。

| 参数（标签）    | 说明                                                         | 数值                  | 单位 |
| --------------- | ------------------------------------------------------------ | --------------------- | ---- |
| 1 (Local Ctrl)  | 0: 禁用本地避障/本地路径规划（不重置地图），1: 启用本地路径规划，2: 启用并重置本地路径规划 | min: 0 max: 2 inc: 1  |      |
| 2 (Global Ctrl) | 0：禁用完整路径规划（不重置地图），1：启用，2：启用并重置地图/占位网格，3：启用并重置规划路线，但不重置占位网格 | min: 0 max： 3 inc: 1 |      |
| 3               | 空                                                           |                       |      |
| 4 （偏航）      | 目标处的偏航角度                                             |                       | 度   |
| 5 （纬度/X）    | 目标的纬度/X                                                 |                       |      |
| 6 (Longitude/Y) | 球门的经度/Y                                                 |                       |      |
| 7 (Altitude/Z)  | 球门高度/Z                                                   |                       |      |


### MAV_CMD_NAV_SPLINE_WAYPOINT (82) 

使用样条路径导航到航点。

| 参数（标签）    | 说明                                            | 数值       | 单位 |
| --------------- | ----------------------------------------------- | ---------- | ---- |
| 1 (保持)        | 保持时间。(固定翼忽略，旋转翼停留在航点的时间） | 最小值： 0 | 秒   |
| 2               | 空                                              |            |      |
| 3               | 空                                              |            |      |
| 4               | 空                                              |            |      |
| 5 （纬度/X）    | 目标的纬度/X                                    |            |      |
| 6 (Longitude/Y) | 球门的经度/Y                                    |            |      |
| 7 (Altitude/Z)  | 球门高度/Z                                      |            |      |


### MAV_CMD_NAV_VTOL_TAKEOFF (84) 

使用 VTOL 模式从地面起飞，并按指定航向转为向前飞行。不支持 VTOL 和固定翼飞行的飞行器（多旋翼飞行器、船只等）应忽略该命令。

| 参数（标签）                                                 | 说明         | 数值                                                | 单位 |
| ------------------------------------------------------------ | ------------ | --------------------------------------------------- | ---- |
| 1                                                            | 空           |                                                     |      |
| 2 （过渡航向）                                               | 前过渡航向。 | [vtol_transition_heading](#vtol_transition_heading) | 3    |
| 3                                                            | 空           |                                                     |      |
| 4 (Yaw Angle) 偏航角度。NaN 表示使用当前系统偏航航向模式（例如，偏航至下一个航点、偏航至原点等）。 |              | 度                                                  |      |
| 5 （纬度）                                                   | 纬度         |                                                     |      |
| 6 （经度）                                                   | 经度         |                                                     |      |
| 7 (高度)                                                     | 高度         |                                                     | 米   |


### MAV_CMD_NAV_VTOL_LAND (85) 

使用 VTOL 模式着陆

| 参数（标签）          | 说明                                                         | 数值                                            | 单位 |
| --------------------- | ------------------------------------------------------------ | ----------------------------------------------- | ---- |
| 1 （着陆选项）        | 着陆行为。                                                   | [nav_vtol_land_options](#nav_vtol_land_options) | 2    |
| 2                     | 空。                                                         |                                                 |      |
| 3 (Approach Altitude) | 接近高度（与 "高度 "字段的参考值相同）。如果未指定，则为 NaN。 |                                                 | m    |
| 4 (Yaw)               | 偏航角。NaN 表示使用当前系统的偏航航向模式（例如，偏航至下一个航点、偏航至原点等）。 |                                                 | 度   |
| 5 （纬度）            | 纬度                                                         |                                                 |      |
| 6 （经度）            | 经度                                                         |                                                 |      |
| 7（地面高度）         | 相对于当前坐标系的高度（地面高度）。NaN 表示使用系统默认着陆高度（忽略值）。 |                                                 | m    |


### MAV_CMD_NAV_GUIDED_ENABLE (92) 

将控制权交给外部控制器

| Param (Label) | Description          | Values               |
| ------------- | -------------------- | -------------------- |
| 1 (Enable)    | On / Off (> 0.5f on) | min: 0 max: 1 inc: 1 |
| 2             | Empty                |                      |
| 3             | Empty                |                      |
| 4             | Empty                |                      |
| 5             | Empty                |                      |
| 6             | Empty                |                      |
| 7             | Empty                |                      |


### MAV_CMD_NAV_DELAY (93) 

将下一条导航命令延迟若干秒或指定时间

| 参数（标签） | 说明                                   | 数值                    | 单位 |
| ------------ | -------------------------------------- | ----------------------- | ---- |
| 1（延迟）    | 延迟（-1 时启用日期时间字段）          | 最小值：-1 英寸：1      | 秒   |
| 2 (Hour)     | 小时（24 小时格式，UTC，-1 忽略）      | min: -1 max: 23 inc: 1  |      |
| 3 (Minute)   | minute (24h format, UTC, -1 to ignore) | min: -1 max： 59 inc: 1 |      |
| 4 (Second)   | 秒（24 小时格式，UTC，-1 可忽略）      | min: -1 max： 59 inc: 1 |      |
| 5            | 空                                     |                         |      |
| 6            | 空                                     |                         |      |
| 7            | 空                                     |                         |      |


### MAV_CMD_NAV_PAYLOAD_PLACE (94) 

下降并放置有效载荷。飞行器移动到指定位置，下降直到检测到悬挂的有效载荷已到达地面，然后释放有效载荷。如果在达到最大下降值（参数 1）之前未检测到地面，则命令将在不释放有效载荷的情况下完成。

| 参数（标签）       | 说明           | 数值      | 单位 |
| ------------------ | -------------- | --------- | ---- |
| 1 （最大下降距离） | 最大下降距离。 | 最小值：0 | 米   |
| 2                  | 空             |           |      |
| 3                  | 空             |           |      |
| 4                  | 空             |           |      |
| 5 （纬度）         | 纬度           |           |      |
| 6 (经度)           | 经度           |           |      |
| 7 (高度)           | 高度           |           | m    |


### MAV_CMD_NAV_LAST (95) 

NOP - 该命令仅用于标记枚举中 NAV/ACTION 命令的上限。

| Param (Label) | Description |
| ------------- | ----------- |
| 1             | Empty       |
| 2             | Empty       |
| 3             | Empty       |
| 4             | Empty       |
| 5             | Empty       |
| 6             | Empty       |
| 7             | Empty       |


### MAV_CMD_CONDITION_DELAY (112) 

延迟任务状态机。

| Param (Label) | Description | Values | Units |
| ------------- | ----------- | ------ | ----- |
| 1 (Delay)     | Delay       | min: 0 | s     |
| 2             | Empty       |        |       |
| 3             | Empty       |        |       |
| 4             | Empty       |        |       |
| 5             | Empty       |        |       |
| 6             | Empty       |        |       |
| 7             | Empty       |        |       |


### MAV_CMD_CONDITION_CHANGE_ALT (113) 

以指定速度上升/下降到目标高度。延迟任务状态机，直到达到预期高度。

| Param (Label) | Description    | Units |
| ------------- | -------------- | ----- |
| 1 (Rate)      | 下降/上升速度. | m/s   |
| 2             | Empty          |       |
| 3             | Empty          |       |
| 4             | Empty          |       |
| 5             | Empty          |       |
| 6             | Empty          |       |
| 7 (Altitude)  | 目标高度       | m     |


### MAV_CMD_CONDITION_DISTANCE (114) 

延迟任务状态机，直到与下一个导航点保持在所需距离内。

| Param (Label) | Description | Values | Units |
| ------------- | ----------- | ------ | ----- |
| 1 (Distance)  | 距离.       | min: 0 | m     |
| 2             | Empty       |        |       |
| 3             | Empty       |        |       |
| 4             | Empty       |        |       |
| 5             | Empty       |        |       |
| 6             | Empty       |        |       |
| 7             | Empty       |        |       |


### MAV_CMD_CONDITION_YAW (115) 

达到某一目标角度。

| 参数（标签） | 说明                                                         | 数值                          | 单位  |
| ------------ | ------------------------------------------------------------ | ----------------------------- | ----- |
| 1 (Angle)    | 目标角度 [0-360]。绝对角度：0 为正北。相对角度：0 为初始偏航。方向由参数 3 设置。 | 最小： 0 最大： 360 360       | 度    |
| 2（角速度）  | 角速度                                                       | 最小： 0                      | 度/秒 |
| 3（方向）    | 方向：-1：逆时针，0：最短方向，1：顺时针                     | 最小值：-1 最大值：1 1 inc: 1 |       |
| 4 (Relative) | 0：绝对角度，1：相对偏移                                     | min: 0 max： 1 inc: 1         |       |
| 5            | 空                                                           |                               |       |
| 6            | 空                                                           |                               |       |
| 7            | 空                                                           |                               |       |


### MAV_CMD_CONDITION_LAST (159) 

NOP - 该命令仅用于标记枚举中 CONDITION 命令的上限

| Param (Label) | Description |
| ------------- | ----------- |
| 1             | Empty       |
| 2             | Empty       |
| 3             | Empty       |
| 4             | Empty       |
| 5             | Empty       |
| 6             | Empty       |
| 7             | Empty       |


### MAV_CMD_DO_SET_MODE (176) 

设置系统模式。

| 参数（标签）       | 说明                                                         | 值                    |
| ------------------ | ------------------------------------------------------------ | --------------------- |
| 1 （模式）         | 模式                                                         | [MAV_MODE](#MAV_MODE) |
| 2 (Custom Mode)    | 自定义模式 - 这与系统有关，详情请参阅各自动驾驶仪规格。      |                       |
| 3 (Custom Submode) | 自定义子模式 - 此项与系统相关，详情请参见各个自动驾驶仪的规格说明。 |                       |
| 4                  | Empty                                                        |                       |
| 5                  | Empty                                                        |                       |
| 6                  | Empty                                                        |                       |
| 7                  | Empty                                                        |                       |


### MAV_CMD_DO_JUMP (177) 


在任务列表中跳转到所需的命令。 只能重复指定次数的操作

| 参数（标签） | 说明     | 数值              |
| ------------ | -------- | ----------------- |
| 1（编号）    | 序列号   | 最小： 0 最大： 1 |
| 2 (Repeat)   | 重复次数 | min: 0 inc: 1     |
| 3            | Empty    |                   |
| 4            | Empty    |                   |
| 5            | Empty    |                   |
| 6            | Empty    |                   |
| 7            | Empty    |                   |


### MAV_CMD_DO_CHANGE_SPEED (178) 

更改速度和/或油门设置点。该值会一直存在，直到被覆盖或模式发生变化。

| 参数（标签）  | 说明                                             | 数值                      | 单位 |
| ------------- | ------------------------------------------------ | ------------------------- | ---- |
| 1（速度类型） | 参数 2 中所设值的速度类型（如空速、地速等）      | [SPEED_TYPE](#SPEED_TYPE) |      |
| 2（速度）     | 速度（-1 表示不变，-2 表示返回默认车速）         | 最小值：-2                | m/s  |
| 3 (Throttle)  | 节流（-1 表示不更改，-2 表示返回默认车辆节流值） | 最小值：-2                | %    |
| 4             |                                                  |                           |      |
| 5             |                                                  |                           |      |
| 6             |                                                  |                           |      |
| 7             |                                                  |                           |      |


### MAV_CMD_DO_SET_HOME (179) 

将原点设置为当前位置或指定位置。
原点是系统返回和着陆的默认位置。
该位置由系统在起飞时自动设置（也可使用此命令设置）。
注意： 当前的原点位置可根据请求（使用参数 1=242 的 [MAV_CMD_REQUEST_MESSAGE](#MAV_CMD_REQUEST_MESSAGE)）在 [HOME_POSITION](#HOME_POSITION) 消息中发布。

| 参数（标签）      | 说明                                                         | 值                               | 单位 |
| ----------------- | ------------------------------------------------------------ | -------------------------------- | ---- |
| 1（使用当前位置） | 使用当前位置（1=使用当前位置，0=使用指定位置）               | 最小： 0 最大： 1 inc： 1 inc: 1 |      |
| 2 (Roll)          | （表面）滚动角度。范围： -180...180 度。NAN 或 0 表示未设置值。0.01 表示零滚动。 | 最小值：-180 最大值：180 180     | 度   |
| 3（俯仰角）       | 俯仰角（表面）。范围： -90...90 度。NAN 或 0 表示未设置值。0.01 表示间距为零。 | 最小： -90 最大： 90 90          | 度   |
| 4 (Yaw)           | 偏航角度。NaN 表示使用默认航向。范围： -180...180 度。最小： -180 最大： 180 | 度 180                           | 度   |
| 5 (Latitude)      | 纬度                                                         |                                  |      |
| 6 (Longitude)     | 经度                                                         |                                  |      |
| 7 (高度)          | 高度                                                         |                                  | 米   |


### MAV_CMD_DO_SET_PARAMETER (180) — [DEP] 

<span class="warning">**DEPRECATED:** Replaced By [PARAM_SET](#PARAM_SET) (2024-04)</span>

设置系统参数。 注意！ 使用此命令需要了解参数的数字枚举值。

| 参数（标签） | 说明     | 数值              |
| ------------ | -------- | ----------------- |
| 1 （编号）   | 参数编号 | 最小： 0 最大： 1 |
| 2 (Value)    | 参数值   |                   |
| 3            | Empty    |                   |
| 4            | Empty    |                   |
| 5            | Empty    |                   |
| 6            | Empty    |                   |
| 7            | Empty    |                   |


### MAV_CMD_DO_SET_RELAY (181) 

将继电器设置为条件。

| 参数（标签） | 说明                                        | 值                |
| ------------ | ------------------------------------------- | ----------------- |
| 1 (Instance) | 继电器实例编号。                            | 最小： 0 最大： 1 |
| 2 （设置）   | 设置。(1=开，0=关，其他可能取决于系统硬件） | 最小： 0 最大： 1 |
| 3            | Empty                                       |                   |
| 4            | Empty                                       |                   |
| 5            | Empty                                       |                   |
| 6            | Empty                                       |                   |
| 7            | Empty                                       |                   |


### MAV_CMD_DO_REPEAT_RELAY (182) 

在所需周期内按所需次数循环开关继电器。

| 参数（标签） | 说明             | 数值                 | 单位 |
| ------------ | ---------------- | -------------------- | ---- |
| 1 (Instance) | 继电器实例编号。 | 最小值：0，最大值：1 |      |
| 2 (Count)    | 循环计数         | min: 1 inc: 1。      |      |
| 3 (Time)     | 循环时间。       | 最小值：0            |      |
| 4            | Empty            |                      |      |
| 5            | Empty            |                      |      |
| 6            | Empty            |                      |      |
| 7            | Empty            |                      |      |


### MAV_CMD_DO_SET_SERVO (183) 

将伺服设置为所需的 PWM 值。

| 参数（标签） | 说明           | 数值            | 单位 |
| ------------ | -------------- | --------------- | ---- |
| 1 (Instance) | 伺服实例编号。 | 最小：0 最大：1 |      |
| 2 (PWM)      | 脉冲宽度调制。 | min: 0 inc: 1   | us   |
| 3            | Empty          |                 |      |
| 4            | Empty          |                 |      |
| 5            | Empty          |                 |      |
| 6            | Empty          |                 |      |
| 7            | Empty          |                 |      |


### MAV_CMD_DO_REPEAT_SERVO (184) 

在标称设置和所需 PWM 之间循环，以所需周期数循环。

| 参数（标签） | 说明           | 数值                   | 单位 |
| ------------ | -------------- | ---------------------- | ---- |
| 1 (Instance) | 伺服实例编号。 | 最小：0 最大：1        |      |
| 2 (PWM)      | 脉冲宽度调制。 | 最小值：0，最大值：1。 |      |
| 3 (Count)    | 周期计数。     | min: 1 inc: 1          | us   |
| 4 (Time)     | 循环时间。     | 最小值：0              |      |
| 5            | Empty          |                        |      |
| 6            | Empty          |                        |      |
| 7            | Empty          |                        |      |


### MAV_CMD_DO_FLIGHTTERMINATION (185) 

立即终止飞行。

飞行终止立即不可逆转地终止当前飞行，使飞行器返回地面。
飞行器将忽略遥控或其他输入，直到电源重新启动。
终止飞行可能会触发安全措施，包括：在多旋翼飞行器上禁用发动机和展开降落伞，在固定翼飞行器上设置飞行表面以启动着陆模式。）
在没有降落伞的多旋翼飞行器上，可能会触发迫降。
可使用协议位测试对该命令的支持： [mav_protocol_capability_flight_termination]（#mav_protocol_capability_flight_termination）。
也可通过发送参数 1=0 (< 0.5) 的命令来测试对该命令的支持；ACK 应为 [MAV_RESULT_FAILED](#MAV_RESULT_FAILED) 或 [MAV_RESULT_UNSUPPORTED](#MAV_RESULT_UNSUPPORTED)。

| 参数（标签）  | 说明                                                         | 值                         |
| ------------- | ------------------------------------------------------------ | -------------------------- |
| 1 (Terminate) | 如果 > 0.5，则激活飞行终止。否则不激活，并以 MAV_RESULT_FAILED 发送 ACK。 | 最小： 0 最大： 1 1 inc: 1 |
| 2             | Empty                                                        |                            |
| 3             | Empty                                                        |                            |
| 4             | Empty                                                        |                            |
| 5             | Empty                                                        |                            |
| 6             | Empty                                                        |                            |
| 7             | Empty                                                        |                            |


### MAV_CMD_DO_CHANGE_ALTITUDE (186) 

更改高度设置点。

| 参数（标签） | 说明         | 数值                    | 单位 |
| ------------ | ------------ | ----------------------- | ---- |
| 1 (Altitude) | 高度。       |                         | m    |
| 2 (Frame)    | 新高度的帧。 | [mav_frame](#mav_frame) |      |
| 3            | Empty        |                         |      |
| 4            | Empty        |                         |      |
| 5            | Empty        |                         |      |
| 6            | Empty        |                         |      |
| 7            | Empty        |                         |      |


### MAV_CMD_DO_SET_ACTUATOR (187) 

将执行器（如舵机）设置为所需值。执行器编号通过飞行堆栈特定的机制（即参数）映射到特定的输出端（例如主站、辅助 PWM 或 UAVCAN 上的输出端）。

| 参数（标签）   | 说明                                                         | 数值                   |
| -------------- | ------------------------------------------------------------ | ---------------------- |
| 1 (Actuator 1) | 执行器 1 值，从 [-1 到 1] 缩放。忽略 NaN。                   | 最小值：-1 最大值：1 1 |
| 2 (Actuator 2) | 执行机构 2 的值，从 [-1 到 1] 缩放。忽略 NaN。               | 最小值：-1 最大值：1 1 |
| 3 (Actuator 3) | 执行机构 3 的值，刻度范围 [-1 to 1]。忽略 NaN。              | 最小值：-1 最大值：1 1 |
| 4 (Actuator 4) | 执行机构 4 的值，从 [-1 到 1] 缩放。忽略 NaN。               | 最小值：-1 最大值：1 1 |
| 5 (Actuator 5) | 执行机构 5 的值，从 [-1 到 1] 缩放。忽略 NaN。               | 最小值：-1 最大值：1 1 |
| 6 (Actuator 6) | 执行机构 6 的值，刻度范围 [-1 to 1]。忽略 NaN。              | 最小值：-1 最大值：1 1 |
| 7 (Index)      | 执行机构设置的索引（例如，如果设置为 1，则执行机构 1 变为执行机构 7） | min: 0 inc: 1          |


### MAV_CMD_DO_RETURN_PATH_START (188) — [WIP] 

<span class="warning">**WORK IN PROGRESS**: Do not use in stable production environments (it may change).</span>

任务项，用于指定故障安全/着陆返回路径段的起点（该路径段的终点为下一个 [MAV_CMD_DO_LAND_START]（#MAV_CMD_DO_LAND_START）项）。

使用任务进行着陆的飞行器（例如在返回模式下）将在返回路径段的最近路径上加入任务（而不是 [MAV_CMD_DO_LAND_START](#MAV_CMD_DO_LAND_START) 或最近的航点）。
主要用途是在走廊任务中尽量减少故障安全飞行路径，在走廊任务中，入站/出站路径（通过地理围栏）受限于相同的特定路径。
MAV_CMD_NAV_RETURN_PATH_START](#MAV_CMD_NAV_RETURN_PATH_START) 将被置于返回路径的起点。
如果出站路径上发生故障保护，飞行器将移动到返回路径上最近的点（此类任务的返回路径是平行的），实际上是掉头并沿着最短的路径着陆。
如果故障安全发生在进站路径上，飞行器已经在返回段上，将继续着陆。
纬度/经度/纬度是可选项，如果不需要，可设置为 0。
如果指定，该项将定义返回航段的起始航点。
如果作为命令发送，飞行器将执行任务着陆（如果定义了着陆段，则使用着陆段）；如果不支持任务着陆或未定义任务着陆，则拒绝该命令。作为指令发送时，指令中的任何位置信息都将被忽略。

| Param (Label) | Description            | Units |
| ------------- | ---------------------- | ----- |
| 1             | Empty                  |       |
| 2             | Empty                  |       |
| 3             | Empty                  |       |
| 4             | Empty                  |       |
| 5（纬度）     | Latitudee。0：未使用。 |       |
| 6 (Longitude) | 经度。0：未使用。      |       |
| 7 (Altitude)  | 高度。0：未使用。      | m     |


### MAV_CMD_DO_LAND_START (189) 

执行着陆的任务指令。这是任务中的一个标记，用于告诉自动驾驶仪代表着陆的任务项目序列从哪里开始。

也可以通过 [COMMAND_LONG]（#COMMAND_LONG）发送来触发着陆，在这种情况下，将使用任务中最近（地理位置上）的着陆序列。
纬度/经度/纬度是可选项，如果不需要，可设置为 0。如果指定，则将用于帮助查找最近的着陆序列。

| 参数（标签）  | 说明      | 单位 |
| ------------- | --------- | ---- |
| 1             | Empty     |      |
| 2             | Empty     |      |
| 3             | Empty     |      |
| 4             | Empty     |      |
| 5 (Latitude)  | Latitude  |      |
| 6 (Longitude) | Longitude |      |
| 7 (Altitude)  | Altitude  | m    |


### MAV_CMD_DO_RALLY_LAND (190) 


从集结点着陆的任务指令。

| 参数（标签） | 说明     | 单位  |
| ------------ | -------- | ----- |
| 1 （高度）   | 降落高度 | 米    |
| 2 （速度）   | 着陆速度 | 米/秒 |
| 3            | Empty    |       |
| 4            | Empty    |       |
| 5            | Empty    |       |
| 6            | Empty    |       |
| 7            | Empty    |       |


### MAV_CMD_DO_GO_AROUND (191) 

安全中止自主着陆的任务指令。

| Param (Label) | Description | Units |
| ------------- | ----------- | ----- |
| 1 (Altitude)  | Altitude    | m     |
| 2             | Empty       |       |
| 3             | Empty       |       |
| 4             | Empty       |       |
| 5             | Empty       |       |
| 6             | Empty       |       |
| 7             | Empty       |       |


### MAV_CMD_DO_REPOSITION (192) 

将飞行器重新定位到特定的 WGS84 全局位置。该命令适用于制导命令（执行任务时使用 [MAV_CMD_NAV_WAYPOINT](#MAV_CMD_NAV_WAYPOINT)）。

| 参数（标签） | 说明                                                         | 数值                    | 单位 |
| ------------ | ------------------------------------------------------------ | ----------------------- | ---- |
| 1 (Speed)    | 地面速度，默认小于 0 (-1)                                    | 最小值：-1              | m/s  |
| 2 (Bitmask)  | 选项标志的位掩码。                                           | MAV_DO_REPOSITION_FLAGS |      |
| 3 (Radius)   | 飞机的装载半径。仅为正值，方向由偏航值控制。零值或 NaN 值将被忽略。 |                         | m    |
| 4 (Yaw)      | 偏航方向。NaN表示使用当前系统的偏航航向模式（例如偏航至下一个航点、偏航至原点等）。对飞机而言，表示闲逛方向（0：顺时针，1：逆时针）。 |                         |      |
| 5（纬度）    | 纬度                                                         |                         |      |
| 6 (经度)     | 经度                                                         |                         |      |
| 7 （高度）   | 高度                                                         |                         | 米   |


### MAV_CMD_DO_PAUSE_CONTINUE (193) 

如果处于 GPS 控制定位模式，则保持当前位置或继续定位。

| 参数（标签） | 说明                                                         | 数值                       |
| ------------ | ------------------------------------------------------------ | -------------------------- |
| 1（继续）    | 0： 暂停当前任务或重新定位命令，保持当前位置。1: 继续执行任务。具有 VTOL 功能的飞行器应进入悬停模式（多旋翼飞行器和 VTOL 飞机）。飞机应以默认的闲逛半径闲逛。 | 最小： 0 最大： 1 1 inc: 1 |
| 2            | Reserved                                                     |                            |
| 3            | Reserved                                                     |                            |
| 4            | Reserved                                                     |                            |
| 5            | Reserved                                                     |                            |
| 6            | Reserved                                                     |                            |
| 7            | Reserved                                                     |                            |


### MAV_CMD_DO_SET_REVERSE (194) 

将移动方向设置为前进或后退。

| 参数（标签） | 说明                   | 数值                             |
| ------------ | ---------------------- | -------------------------------- |
| 1 (Reverse)  | 方向（0=前进，1=后退） | 最小： 0 最大： 1 inc： 1 inc: 1 |
| 2            | Empty                  |                                  |
| 3            | Empty                  |                                  |
| 4            | Empty                  |                                  |
| 5            | Empty                  |                                  |
| 6            | Empty                  |                                  |
| 7            | Empty                  |                                  |


### MAV_CMD_DO_SET_ROI_LOCATION (195) 

将感兴趣区域（ROI）设置为一个位置。然后，飞行器的控制系统就可以利用它来控制飞行器的姿态和各种传感器（如摄像头）的姿态。该命令可发送至万向节管理器，但不能发送至万向节设备。万向节不会对该信息做出反应。

| 参数（标签）       | 说明                                                         | 单位  |
| ------------------ | ------------------------------------------------------------ | ----- |
| 1（万向节设备 ID） | 要寻址的万向节设备的组件 ID（或 1-6 用于非 MAVLink 万向节），0 用于所有万向节设备组件。为多个万向节（但不是所有万向节）多次发送命令。 |       |
| 2                  | 空                                                           |       |
| 3                  | 空                                                           |       |
| 4                  | 空                                                           |       |
| 5 (Latitude)       | ROI 位置的纬度                                               | degE7 |
| 6 (Longitude)      | ROI 所在位置的经度                                           | degE7 |
| 7 (高度)           | ROI 所在位置的高度                                           | 米    |


### MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET (196) 

将感兴趣区域（ROI）设置为下一个航点，并可选择俯仰/滚动/偏航偏移。然后，飞行器的控制系统就可以用它来控制飞行器的姿态和各种传感器（如摄像头）的姿态。该命令可以发送给万向节管理器，但不能发送给万向节设备。万向节设备不会对该信息做出反应。

| 参数（标签）       | 说明                                                         | 单位 |
| ------------------ | ------------------------------------------------------------ | ---- |
| 1（万向节设备 ID） | 要寻址的万向节设备的组件 ID（或 1-6 用于非 MAVLink 万向节），0 用于所有万向节设备组件。为多个万向节（但不是所有万向节）多次发送命令。 |      |
| 2                  | 空                                                           |      |
| 3                  | 空                                                           |      |
| 4                  | 空                                                           |      |
| 5（俯仰偏移）      | 下一个航点的俯仰偏移，正俯仰向上                             | 度   |
| 6（滚动偏移）      | 下一个航点的滚动偏移，向右正向滚动                           | 度   |
| 7 (Yaw Offset)     | 偏航偏移，从下一个航点开始，向右正偏航                       | 度   |


### MAV_CMD_DO_SET_ROI_NONE (197) 

取消之前的任何 ROI 命令，使飞行器/传感器返回默认飞行特性。然后，飞行器的控制系统就可以用它来控制飞行器的姿态和各种传感器（如摄像头）的姿态。该命令可发送至万向节管理器，但不能发送至万向节设备。万向节设备不会对该信息做出反应。发出该命令后，如果有手动输入，云台管理器应返回手动输入，否则应保持中立位置。

| 参数（标签）       | 说明                                                         |
| ------------------ | ------------------------------------------------------------ |
| 1（万向节设备 ID） | 要寻址的万向节设备的组件 ID（或 1-6 用于非 MAVLink 万向节），0 用于所有万向节设备组件。为多个万向节（但不是所有万向节）发送多次命令。 |
| 2                  | Empty                                                        |
| 3                  | Empty                                                        |
| 4                  | Empty                                                        |
| 5                  | Empty                                                        |
| 6                  | Empty                                                        |
| 7                  | Empty                                                        |


### MAV_CMD_DO_SET_ROI_SYSID (198) 

使用指定的系统 ID 安装跟踪系统。可以使用 [GLOBAL_POSITION_INT](#GLOBAL_POSITION_INT)或其他方法确定目标飞行器的位置。该命令可以发送给万向节管理器，但不能发送给万向节设备。万向节设备不会对该信息做出任何反应。

| 参数（标签）         | 说明                                                         | 值                         |
| -------------------- | ------------------------------------------------------------ | -------------------------- |
| 1 （系统 ID）        | 系统 ID                                                      | 最小： 1 最大： 255 inc: 1 |
| 2 (Gimbal device ID) | 要寻址的万向节设备的组件 ID（或 1-6 用于非 MAVLink 万向节），0 用于所有万向节设备组件。为多个万向节（但不是所有万向节）多次发送命令。 |                            |


### MAV_CMD_DO_CONTROL_VIDEO (200) 

控制车载摄像系统。

| 参数（标签） | 说明                                          | 数值                   | 单位             |
| ------------ | --------------------------------------------- | ---------------------- | ---------------- |
| 1 (ID)       | 摄像机 ID（全部为-1）                         | 最小值：-1，最大值：1  | 2 (Transmission) |
| 2（传输）    | 传输：0：禁用，1：启用压缩，2：启用原始       | 最小：0 最大：2 inc: 1 | 3（间隔）        |
| 3 (Interval) | 传输模式： 0：视频流，>0：每 n 秒传输一个图像 | min: 0                 | s                |
| 4 (录制)     | 录制： 0：禁用，1：启用压缩，2：启用原始数据  | min: 0 max: 2 inc: 1   | s                |
| 5            | Empty                                         |                        |                  |
| 6            | Empty                                         |                        |                  |
| 7            | Empty                                         |                        |                  |


### MAV_CMD_DO_SET_ROI (201) — [DEP] 

<span class="warning">**DEPRECATED:** Replaced By MAV_CMD_DO_SET_ROI_* (2018-01)</span>

为传感器集或车辆本身设置感兴趣区域（ROI）。然后，飞行器的控制系统就可以用它来控制飞行器的姿态和各种传感器（如摄像头）的姿态。

| 参数（标签）   | 说明                                                         | 数值                |
| -------------- | ------------------------------------------------------------ | ------------------- |
| 1 （ROI 模式） | 感兴趣区域模式。                                             | [mav_roi](#mav_roi) |
| 2 (WP Index)   | 航点索引/目标 ID（取决于参数 1）。                           | 最小：0 最大：1     |
| 3 (ROI Index)  | 感兴趣区域索引。(允许车辆管理多个 ROI）。                    |                     |
| 4              | 空                                                           |                     |
| 5              | MAV_ROI_WPNEXT：下一个航点的俯仰偏移，MAV_ROI_LOCATION：纬度 |                     |
| 6              | MAV_ROI_WPNEXT：下一个航点的滚动偏移，MAV_ROI_LOCATION：经度 |                     |
| 7              | MAV_ROI_WPNEXT：下一个航点的偏航偏移量，MAV_ROI_LOCATION：高度 |                     |


### MAV_CMD_DO_DIGICAM_CONFIGURE (202) 

配置数码相机。对于尚未执行 [PARAM_EXT_XXX](#PARAM_EXT_XXX) 信息和摄像机定义文件的系统，这是一条备用信息（请参阅 https://mavlink.io/en/services/camera_def.html ）。

| 参数（标签）                                                 | 描述                     | 值                   | 单位 |
| ------------------------------------------------------------ | ------------------------ | -------------------- | ---- |
| 1 （模式）                                                   | 模式： P、TV、AV、M 等。 | 最小值：0 最大值：1  |      |
| 2 (Shutter Speed) （快门速度）                               | 快门速度： 一秒的除数。  | 最小值：0，最大值：1 |      |
| 3 (Aperture) 光圈： F 光圈值。                               | 最小：0                  |                      |      |
| 4 (ISO) ISO 数字，如 80、100、200 等。                       | 最小：0 最大：1          |                      |      |
| 5 (Exposure) 曝光类型枚举器。                                |                          |                      |      |
| 6 (Command Identity) 命令标识。                              |                          |                      |      |
| 7 (Engine Cut-off) 主引擎在摄像机触发前的截止时间。(0 表示无截止时间） | min: 0 inc: 1            | ds                   |      |


### MAV_CMD_DO_DIGICAM_CONTROL (203) 

控制数码相机。对于尚未执行 [PARAM_EXT_XXX](#PARAM_EXT_XXX) 信息和摄像机定义文件的系统，这是一条备用信息（请参阅 https://mavlink.io/en/services/camera_def.html ）。

| 参数（标签）                  | 说明                                                         |
| ----------------------------- | ------------------------------------------------------------ |
| 1（会话控制）                 | 会话控制，例如显示/隐藏镜头                                  |
| 2 （绝对缩放）                | 缩放的绝对位置                                               |
| 3 (Zoom Relative（相对缩放）) | 从当前位置偏移缩放的缩放步长值                               |
| 4（对焦）                     | 对焦锁定、解锁或重新锁定                                     |
| 5（拍摄命令）                 | 拍摄命令                                                     |
| 6 （命令标识）                | 命令标识                                                     |
| 7 (Shot ID)                   | 测试镜头标识符。如果设置为 1，则仅拍摄图像，但不计入内部帧数。 |

### MAV_CMD_DO_MOUNT_CONFIGURE (204) — [DEP] 

<span class="warning">**已删除：** 被 [MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE](#MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE) 取代 (2020-01) - 此信息已被 [MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE](#MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE) 取代。该信息仍可用于与使用该信息的传统云台进行通信（#MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE)。</span>

任务命令，用于配置摄像机或天线支架

| 参数（标签）                                                 | 说明                                                         | 值                                |
| ------------------------------------------------------------ | ------------------------------------------------------------ | --------------------------------- |
| 1 (Mode)                                                     | 安装操作模式                                                 | [MAV_MOUNT_MODE](#MAV_MOUNT_MODE) |
| 2 (Stabilize Roll)                                           | 稳定滚动？(1 = 是，0 = 否)                                   | min: 0 max： 1 inc: 1             |
| 3 (Stabilize Pitch)                                          | 稳定俯仰角？1 inc: 1                                         |                                   |
| 4 (Stabilize Yaw)                                            | 稳定偏航？(1 = 是，0 = 否)                                   | min: 0 max： 1 inc: 1             |
| 5 (滚动输入模式)                                             | 滚动输入（0 = 角体帧，1 = 角速率，2 = 角绝对帧）             | 6 (俯仰输入模式)                  |
| 6 (俯仰输入模式)                                             | 俯仰输入（0 = 角体帧，1 = 角速率，2 = 绝对角帧）             | 7 (偏航输入模式)                  |
| 7 (Yaw Input Mode) 偏航输入（0 = 角度体帧，1 = 角度速率，2 = 绝对角度帧） | 7 (Yaw Input Mode) 偏航输入（0 = 角度体帧，1 = 角度速率，2 = 绝对角度帧 |                                   |


### MAV_CMD_DO_MOUNT_CONTROL (205) — [DEP] 

<span class="warning">**已删除：** 被 [MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW](#MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW) 所取代 (2020-01) - 此信息含糊不清且不一致。它已被 [MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW](#MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW) 和 MAV_CMD_DO_SET_ROI_* 所取代。该信息仍可用于与实现该功能的传统云台进行通信）。</span>

控制摄像机或天线座的任务命令

| 参数（标签） | 说明                                                  | 数值                | 单位 |
| ------------ | ----------------------------------------------------- | ------------------- | ---- |
| 1 (俯仰角)   | 俯仰角取决于安装模式（度或度/秒，取决于俯仰角输入）。 |                     |      |
| 2（滚动）    | 滚动取决于悬置模式（度或度/秒，取决于滚动输入）。     |                     |      |
| 3（偏航）    | 偏航取决于悬停模式（度或度/秒，取决于偏航输入）。     |                     |      |
| 4 (高度)     | 高度取决于悬停模式。                                  |                     | m    |
| 5（纬度）    | 纬度，根据相应的安装模式设置。                        |                     |      |
| 6 (经度)     | 经度，根据相应的安装模式设置。                        |                     |      |
| 7 (Mode)     | 载入模式。                                            | 7 (Mode) 安装模式。 |      |


### MAV_CMD_DO_SET_CAM_TRIGG_DIST (206) 

任务指令，为本次飞行设置相机触发距离。每次超过该距离时都会触发照相机。该命令还可用于设置摄像机的快门积分时间。

| 参数（标签） | 说明                                        | 数值                  | 单位 |
| ------------ | ------------------------------------------- | --------------------- | ---- |
| 1 (Distance) | 摄像机触发距离。0 表示停止触发。            | 最小值：0             | m    |
| 2 (Shutter)  | 摄像机快门积分时间。-1或0忽略。             |                       |      |
| 3 (Trigger)  | 立即触发摄像机一次。(0 = 不触发，1 = 触发） | min: 0 max： 1 inc: 1 |      |
| 4            | Empty                                       |                       |      |
| 5            | Empty                                       |                       |      |
| 6            | Empty                                       |                       |      |
| 7            | Empty                                       |                       |      |


### MAV_CMD_DO_FENCE_ENABLE (207) 

启用地理围栏。
这可以在任务中使用，也可以通过命令协议使用。
设置的持续时间/寿命未定义。
根据飞行堆栈的执行情况，它可能会一直存在直到被取代，也可能在任务结束时恢复为系统默认设置。
飞行堆栈通常会在重启时将设置重置为系统默认值。

| 参数（标签） | 说明                                                         | 值                            |
| ------------ | ------------------------------------------------------------ | ----------------------------- |
| 1（启用）    | 启用？(0=禁用，1=启用，2=禁用仅地板）                        | 最小值：0 最大值：2 终止值：1 |
| 2 (Types)    | 以位掩码形式启用或禁用栅栏类型。值为 0 表示启用或禁用所有栅栏。如果参数 1 的值为 2，则该参数被忽略 | [FENCE_TYPE](#FENCE_TYPE)     |
| 3            | Empty                                                        |                               |
| 4            | Empty                                                        |                               |
| 5            | Empty                                                        |                               |
| 6            | Empty                                                        |                               |
| 7            | Empty                                                        |                               |


### MAV_CMD_DO_PARACHUTE (208) 

释放降落伞或启用/禁用自动释放的任务项目/命令。

| 参数（标签） | 说明  | 数值                                  |
| ------------ | ----- | ------------------------------------- |
| 1 （动作）   | 动作  | [PARACHUTE_ACTION](#PARACHUTE_ACTION) |
| 2            | Empty |                                       |
| 3            | Empty |                                       |
| 4            | Empty |                                       |
| 5            | Empty |                                       |
| 6            | Empty |                                       |
| 7            | Empty |                                       |


### MAV_CMD_DO_MOTOR_TEST (209) 

执行电机测试的命令。

| 参数（标签）            | 说明                                                         | 数值                                                  | 单位 |
| ----------------------- | ------------------------------------------------------------ | ----------------------------------------------------- | ---- |
| 1 (Instance)            | 电机实例编号（从 1 到车辆上电机的最大数量）。                | 最小值：1，最大值：1。                                |      |
| 2（节流类型）           | 节流类型（参数 3 中的节流值是否为百分比、PWM 值等）          | [MOTOR_TEST_THROTTLE_TYPE](#MOTOR_TEST_THROTTLE_TYPE) |      |
| 3 (Throttle) （节流阀） | 节流阀值。                                                   |                                                       |      |
| 4 (Timeout)             | 按顺序运行的测试之间的超时。                                 | 最小值：0                                             | 秒   |
| 5 (Motor Count)         | 电机数量。依次测试的电机数量： 0/1= 一个电机，2= 两个电机，等等。超时（参数 4）用于两次测试之间。 | 最小值：0，最大值：1。                                |      |
| 6 （测试顺序）          | 电机测试顺序。                                               | [motor_test_order](#motor_test_order)                 |      |
| 7                       | 空                                                           |                                                       |      |


### MAV_CMD_DO_INVERTED_FLIGHT (210) 

改变为/从反转飞行。

| 参数（标签） | 说明                        | 数值                             |
| ------------ | --------------------------- | -------------------------------- |
| 1 (反转)     | 反转飞行。(0=正常，1=倒转） | 最小： 0 最大： 1 inc： 1 inc: 1 |
| 2            | Empty                       |                                  |
| 3            | Empty                       |                                  |
| 4            | Empty                       |                                  |
| 5            | Empty                       |                                  |
| 6            | Empty                       |                                  |
| 7            | Empty                       |                                  |


### MAV_CMD_DO_GRIPPER (211) 

操作机械手的任务指令。

| 参数（标签） | 说明               | 数值                                |
| ------------ | ------------------ | ----------------------------------- |
| 1 (Instance) | 机械手实例编号。   | 最小：1，最大：1                    |
| 2 (Action)   | 要执行的抓取动作。 | [gripper_actions](#gripper_actions) |
| 3            | Empty              |                                     |
| 4            | Empty              |                                     |
| 5            | Empty              |                                     |
| 6            | Empty              |                                     |
| 7            | Empty              |                                     |


### MAV_CMD_DO_AUTOTUNE_ENABLE (212) 

启用/禁用自动调整。

| 参数（标签） | 说明                                         | 值                           |
| ------------ | -------------------------------------------- | ---------------------------- |
| 1 (Enable)   | 启用（1：启用，0：禁用）。                   | 最小：0 最大：1 1 inc: 1     |
| 2 (Axis)     | 指定自动调整的轴。0 表示自动驾驶仪默认设置。 | [自动调整轴](#autotune_axis) |
| 3            | Empty.                                       |                              |
| 4            | Empty.                                       |                              |
| 5            | Empty.                                       |                              |
| 6            | Empty.                                       |                              |
| 7            | Empty.                                       |                              |


### MAV_CMD_NAV_SET_YAW_SPEED (213) 

设置所需的车辆转弯角度和速度变化。

| 参数（标签）     | 说明                                | 数值                             | 单位 |
| ---------------- | ----------------------------------- | -------------------------------- | ---- |
| 1 (Yaw)          | 用偏航角调整转向。                  | 度                               |      |
| 2 (Speed) 速度。 | 米/秒                               |                                  |      |
| 3 (Angle)        | 最终角度。(0=绝对角度，1=相对角度） | 最小： 0 最大： 1 inc： 1 inc: 1 |      |
| 4                | Empty                               |                                  |      |
| 5                | Empty                               |                                  |      |
| 6                | Empty                               |                                  |      |
| 7                | Empty                               |                                  |      |


### MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL (214) 

任务命令，为本次飞行设置摄像机触发间隔。如果启用了触发功能，则每次触发时间间隔结束时都会触发摄像机。该命令还可用于设置摄像机的快门积分时间。

| 参数（标签）            | 说明                                          | 值                    | 单位 |
| ----------------------- | --------------------------------------------- | --------------------- | ---- |
| 1 (Trigger Cycle)       | 摄像机触发周期时间。-1或0忽略。               | 最小：-1 最大：1 毫秒 |      |
| 2 (Shutter Integration) | 快门积分时间。应小于触发周期时间。-1或0忽略。 | 最小值：-1，最大值：1 | 毫秒 |
| 3                       | Empty                                         |                       |      |
| 4                       | Empty                                         |                       |      |
| 5                       | Empty                                         |                       |      |
| 6                       | Empty                                         |                       |      |
| 7                       | Empty                                         |                       |      |


### MAV_CMD_DO_MOUNT_CONTROL_QUAT (220) — [DEP] 

<span class="warning">**DEPRECATED:** Replaced By [MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW](#MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW) (2020-01)</span>

任务指令，用于以四元数为基准控制摄像机或天线支架。

| 参数（标签） | 说明                             |
| ------------ | -------------------------------- |
| 1 (Q1)       | 四元数参数 q1, w（空旋转时为 1） |
| 2 (Q2)       | 四元数参数 q2, x（空旋转时为 0） |
| 3 (Q3)       | 四元数参数 q3, y（空旋转时为 0） |
| 4 (Q4)       | 四元数参数 q4, z（空旋转时为 0） |
| 5            | Empty                            |
| 6            | Empty                            |
| 7            | Empty                            |


### MAV_CMD_DO_GUIDED_MASTER (221) 

设置主控制器 ID

| 参数（标签）  | 说明    | 值                         |
| ------------- | ------- | -------------------------- |
| 1 (System ID) | 系统 ID | 最小： 0 最大： 255 inc: 1 |
| 2 （组件 ID） | 组件 ID | 最小： 0 最大： 255 inc: 1 |
| 3             | Empty   |                            |
| 4             | Empty   |                            |
| 5             | Empty   |                            |
| 6             | Empty   |                            |
| 7             | Empty   |                            |


### MAV_CMD_DO_GUIDED_LIMITS (222) 

为外部控制设置限值

| 参数（标签）      | 说明                                                         | 数值      | 单位 |
| ----------------- | ------------------------------------------------------------ | --------- | ---- |
| 1（超时）         | 超时 - 允许外部控制器控制车辆的最长时间。0 表示无超时。      | 最小：0   | 秒   |
| 2（最低高度）     | 最低高度（MSL） - 如果飞行器的高度低于此高度，命令将被终止，任务将继续执行。0 表示没有高度下限。 |           | m    |
| 3（最大高度）     | 高度（MSL） max - 如果飞行器的高度超过此高度，命令将被终止，任务将继续执行。0 表示没有高度上限。 |           | m    |
| 4（水平移动限制） | 水平移动限制 - 如果飞行器从执行命令时的位置移动超过此距离，命令将被终止，任务将继续执行。0 表示没有水平移动限制。 | 最小值：0 | m    |
| 5                 | 空                                                           |           |      |
| 6                 | 空                                                           |           |      |
| 7                 | 空                                                           |           |      |


### MAV_CMD_DO_ENGINE_CONTROL (223) 

控制车辆发动机。车辆发动机控制器将对此进行解释，以改变发动机的目标状态。适用于装有内燃机的车辆。

| 参数（标签）     | 说明                                                         | 数值                     | 单位        |
| ---------------- | ------------------------------------------------------------ | ------------------------ | ----------- |
| 1 (Start Engine) | 0: Stop engine, 1:Start Engine                               | min: 0 max： 1 inc: 1    |             |
| 2 （冷启动）     | 0：热启动，1：冷启动。控制扼流圈的使用（如适用）             | 最小： 0 最大： 1 inc: 1 | 2（冷启动） |
| 3（高度延时）    | 高度延时。用于在飞行器达到指定高度后才命令启动发动机。用于 VTOL 飞行器起飞时，在飞机离地后启动发动机。0 表示无延迟。 | 最小值：0                | m           |
| 4                | 空                                                           |                          |             |
| 5                | 空                                                           |                          |             |
| 6                | 空                                                           |                          |             |
| 7                | 空                                                           |                          |             |


### MAV_CMD_DO_SET_MISSION_CURRENT (224) 

将序列号为 seq 的任务项目设为当前项目，并发出 [MISSION_CURRENT](#MISSION_CURRENT)（无论任务编号是否改变）。
如果当前正在执行任务，系统将以最短路径继续执行这个新任务项目，跳过任何中间任务项目。
请注意，除非设置了参数 2，否则任务跳转重复计数器不会重置（参见 [MAV_CMD_DO_JUMP](#MAV_CMD_DO_JUMP)参数 2）。

在某些系统上，该命令可能会触发任务状态机的改变：例如从[MISSION_STATE_NOT_STARTED](#MISSION_STATE_NOT_STARTED)或[MISSION_STATE_PAUSED](#MISSION_STATE_PAUSED)到[MISSION_STATE_ACTIVE](#MISSION_STATE_ACTIVE)。
如果系统处于任务模式，在这些系统上，该命令可能会因此启动、重启或恢复任务。
如果系统未处于任务模式，则该命令不得触发任务模式的切换。

使用参数 2 可以 "重置 "任务。
重置会将跳转计数器设为初始值（要重置计数器而不改变当前任务项目，可将参数 1 设为"-1"）。
重置也会明确地将任务状态从[MISSION_STATE_COMPLETE](#MISSION_STATE_COMPLETE)变为[MISSION_STATE_PAUSED](#MISSION_STATE_PAUSED)或[MISSION_STATE_ACTIVE](#MISSION_STATE_ACTIVE)，从而有可能在（下一次）进入任务模式时恢复。

如果序列号超出范围（包括没有任务项目），命令将以 [MAV_RESULT_FAILED](#MAV_RESULT_FAILED)作为 ACK。

| 参数（标签）      | 说明                                                         | 值                       |
| ----------------- | ------------------------------------------------------------ | ------------------------ |
| 1 （数字）        | 要设置的任务序列值。-1表示当前任务项目（用于重置任务而不改变当前任务项目）。 | 最小值：-1，最大值：1    |
| 2 (Reset Mission) | 重置任务。1: true，0: false。将跳跃计数器重置为初始值，并将任务状态 "已完成 "更改为 "激活 "或 "暂停"。 | 最小：0 最大：1 1 inc: 1 |
| 3                 | Empty                                                        |                          |
| 4                 | Empty                                                        |                          |
| 5                 | Empty                                                        |                          |
| 6                 | Empty                                                        |                          |
| 7                 | Empty                                                        |                          |


### MAV_CMD_DO_LAST (240) 

NOP - 该命令仅用于标记枚举中 DO 命令的上限

| Param (Label) | Description |
| ------------- | ----------- |
| 1             | Empty       |
| 2             | Empty       |
| 3             | Empty       |
| 4             | Empty       |
| 5             | Empty       |
| 6             | Empty       |
| 7             | Empty       |


### MAV_CMD_PREFLIGHT_CALIBRATION (241) 

触发校准。只有在预飞行模式下才接受该命令。除温度校准外，单条信息中只能设置一个传感器，其他传感器均应为零。

| 参数（标签）     | 说明                                                         | 数值                              |
| ---------------- | ------------------------------------------------------------ | --------------------------------- |
| 1（陀螺仪温度）  | 1：陀螺仪校准，3：陀螺仪温度校准                             | 最小： 0 最大： 3 inc： 3 inc: 1  |
| 2 (Magnetometer) | 1：磁力计校准                                                | min: 0 max： 1 inc: 1             |
| 3（地面压力）    | 1：地面压力校准                                              | 最小： 0 最大： 1 inc: 1 1 inc: 1 |
| 4（遥控）        | 1：无线电遥控校准，2：遥控微调校准                           | 最小： 0 最大： 1 inc: 1 1 inc: 1 |
| 5 (加速度计)     | 1：加速度计校准，2：板级校准，3：加速度计温度校准，4：简单加速度计校准 | 最小： 0 最大： 4 inc: 1 4 inc: 1 |
| 6（罗盘或空速）  | 1：APM：罗盘/电机干扰校准（PX4：空速校准，已废弃），2：空速校准 | 最小： 0 最大： 2 inc: 1          |
| 7（电调或气压）  | 1：电调校准，3：气压计温度校准                               | 最小： 0 最大： 3 inc: 1 3 inc: 1 |


### MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS (242) 

设置传感器偏移。只有在预飞行模式下才接受此命令。

| 参数（标签）     | 说明                                                         | 值                    |
| ---------------- | ------------------------------------------------------------ | --------------------- |
| 1 （传感器类型） | 用于调整偏移的传感器： 0: 陀螺仪，1: 加速计，2: 磁力计，3: 气压计，4: 光流计，5: 第二磁力计，6: 第三磁力计 | min: 0 max： 6 inc: 1 |
| 2 (X 偏移)       | X 轴偏移（或通用维度 1），以传感器的原始单位表示             |                       |
| 3 (Y 偏移)       | Y 轴偏移（或通用维度 2），以传感器原始单位表示               |                       |
| 4（Z 偏移）      | Z 轴偏移（或通用维度 3），以传感器的原始单位表示             |                       |
| 5（第 4 维）     | 通用维度 4，以传感器的原始单位表示                           |                       |
| 6（第 5 维）     | 通用第 5 维，以传感器的原始单位表示                          |                       |
| 7 (第 6 维)      | 通用第 6 维，在传感器的原始单位中                            |                       |


### MAV_CMD_PREFLIGHT_UAVCAN (243) 

触发 UAVCAN 配置（执行器 ID 分配和方向映射）。请注意，这与传统的 UAVCAN v0 功能 [UAVCAN_ENUMERATE](#UAVCAN_ENUMERATE)相对应，该功能仅用于在初始飞行器配置时执行一次（它不是正常的飞行前命令，且名称不妥）。

| 参数（标签）   | 说明                                           |
| -------------- | ---------------------------------------------- |
| 1（执行器 ID） | 1：触发执行器 ID 分配和方向映射。0：取消命令。 |
| 2              | 保留                                           |
| 3              | 保留                                           |
| 4              | 保留                                           |
| 5              | 保留                                           |
| 6              | 保留                                           |
| 7              | 保留                                           |


### MAV_CMD_PREFLIGHT_STORAGE (245) 

要求存储不同的参数值和日志。只有在预飞行模式下才接受此命令。

| 参数（标签） | 说明 | 数值 | 单位 |
| ------------ | ---- | ---- | ---- |

2 （任务存储） | 对持久任务存储执行的操作 | [PREFLIGHT_STORAGE_MISSION_ACTION](#PREFLIGHT_STORAGE_MISSION_ACTION) | [PREFLIGHT_STORAGE_MISSION_ACTION](#PREFLIGHT_STORAGE_MISSION_ACTION)   
3（日志记录速率） | 机载日志记录：0：忽略，1：启动默认速率日志记录，-1：停止日志记录，>1：日志记录： 停止记录，> 1：记录速率（例如，设置为 1000 可记录 1000 Hz） | 最小：-1，最大：1 | Hz 
4 | 保留   
5 | 空 | | | 6 | 空   
6 | 空   
7 | 空   


### MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN (246) 

请求重启或关闭系统组件。

| 参数（标签）     | 说明                                                         | 值                       |
| ---------------- | ------------------------------------------------------------ | ------------------------ |
| 1 (Autopilot)    | 0: 不对自动驾驶仪做任何操作，1: 重启自动驾驶仪，2: 关闭自动驾驶仪，3: 重启自动驾驶仪并将其保留在引导加载程序中直至升级。 | 最小：0 最大：3 3 inc: 1 |
| 2 (Companion)    | 0: 对机载计算机不做任何操作，1: 重启机载计算机，2: 关闭机载计算机，3: 重启机载计算机并将其保留在引导加载程序中直至升级。 | 最小：0 最大：3 3 inc: 1 |
| 3 (组件操作)     | 0: 不对组件做任何操作，1: 重启组件，2: 关闭组件，3: 重启组件并将其保留在引导加载程序中直至升级 | min: 0 max： 3 inc: 1    |
| 4 (Component ID) | MAVLink Component ID（MAVLink 组件 ID），参数 3 中的目标值（所有组件均为 0）。 | 最小：0 最大：255 inc: 1 |
| 5                | 保留（设置为 0                                               |                          |
| 6                | 保留（设置为 0                                               |                          |
| 7                | WIP： ID（例如，摄像机 ID -1 适用于所有 ID）。               |                          |


### MAV_CMD_OVERRIDE_GOTO (252) 

用暂停任务、暂停任务并移动到位置、继续/恢复任务的命令覆盖当前任务。当参数 1 表示任务暂停（[MAV_GOTO_DO_HOLD](#MAV_GOTO_DO_HOLD)）时，参数 2 定义任务是原地不动还是移动到其他位置。

| 参数（标签）    | 说明                                                         | 数值                                                  | 单位    |
| --------------- | ------------------------------------------------------------ | ----------------------------------------------------- | ------- |
| 1（继续）       | MAV_GOTO_DO_HOLD：暂停任务并保持或移动到指定位置（取决于参数 2），MAV_GOTO_DO_CONTINUE：继续任务。 | MAV_GOTO_DO_CONTINUE: 恢复任务。                      |         |
| 2 (位置)        | MAV_GOTO_HOLD_AT_CURRENT_POSITION：保持在当前位置，MAV_GOTO_HOLD_AT_SPECIFIED_POSITION：保持在指定位置。 | MAV_GOTO_HOLD_AT_SPECIFIED_POSITION: 保持在指定位置。 |         |
| 3 (Frame)       | 保持点的坐标帧。                                             | [mav_frame](#mav_frame)                               | 4 (Yaw) |
| 4 (Yaw)         | 预期偏航角度。                                               |                                                       | 度      |
| 5 (Latitude/X)  | 纬度/X 位置。                                                |                                                       |         |
| 6 (Longitude/Y) | 经度/Y 位置。                                                |                                                       |         |
| 7 (Altitude/Z)  | 高度/Z 位置。                                                |                                                       |         |


### MAV_CMD_OBLIQUE_SURVEY (260) 

任务命令，用于设置摄像机自动安装枢轴旋转斜测（取代 [CAM_TRIGG_DIST](#CAM_TRIGG_DIST)）。每次超过此距离都会触发摄像机，然后卡口移动到下一个位置。参数 4~6 设置了斜测的角度限制和位置数量，在斜测过程中，支持支架的车辆会在两次拍摄之间自动滚动相机，以模拟斜测相机设置（提供更大的高频视场角）。该命令还可用于设置相机的快门积分时间。

| 参数（标签）               | 说明                                                         | 数值                             | 单位 |
| -------------------------- | ------------------------------------------------------------ | -------------------------------- | ---- |
| 1 (Distance)               | 摄像机触发距离。0 表示停止触发。                             | 最小值：0                        | m    |
| 2 (Shutter)                | 摄像机快门积分时间。0 至忽略                                 | 最小：0 英寸：1                  | 毫秒 |
| 3 （最小间隔）             | 摄像机能够重复拍摄后续图像的最小间隔。0 至忽略。             | 最小：0 最大：10000 10000 inc: 1 | ms   |
| 4 (Positions) （位置）     | 摄像机将拍摄照片的卷轴位置总数（在参数 5 定义的限制范围内平均拍摄图像）。 | 最小：2，最大：1。               |      |
| 5 （滚动角度）             | 摄像机可向中心左右滚动的角度限制。                           | 最小：0                          | 度   |
| 6 (Pitch Angle) （俯仰角） | 如果在俯仰轴上启动支架，摄像机在斜角模式下将保持的固定俯仰角。 | 最小：-180 最大：180 180         | 度   |
| 7                          | 空                                                           |                                  | 空   |


### MAV_CMD_MISSION_START (300) 

开始执行任务

| 参数（标签）     | 说明                                                         | 数值          |
| ---------------- | ------------------------------------------------------------ | ------------- |
| 1 （第一个项目） | first_item：要运行的第一个任务项目                           | min: 0 inc: 1 |
| 2 （最后一项）   | last_item：要运行的最后一个任务项目（该项目运行后，任务结束） | min: 0 inc: 1 |


### MAV_CMD_ACTUATOR_TEST (310) 

致动器测试指令。该命令与 [MAV_CMD_DO_MOTOR_TEST](#MAV_CMD_DO_MOTOR_TEST)类似，但在输出功能层面上进行操作，即可以测试电机 1，而不受配置在哪个输出上的影响。自动驾驶仪通常在布防时拒绝接受此命令。

| 参数（标签）   | 说明                                                         | 数值                                                  | 单位           |
| -------------- | ------------------------------------------------------------ | ----------------------------------------------------- | -------------- |
| 1（值）        | 输出值：1 表示最大正输出，0 表示中心舵机或最小电机推力（预期旋转），-1 表示最大负输出（如果电机不支持，即电机不可逆，小于 0 则映射为 NaN）。NaN 则表示解除（停止电机）。 | 最小：-1 最大：1 1                                    |                |
| 2（超时）      | 超时后，测试命令失效，输出恢复为之前的值。出于安全考虑，必须设置超时。超时为 0 表示立即恢复先前的值。 | 最小：0 最大：3 3                                     | s              |
| 3              |                                                              |                                                       |                |
| 4              |                                                              |                                                       |                |
| 5 （输出功能） | 执行器输出功能                                               | [ACTUATOR_OUTPUT_FUNCTION](#ACTUATOR_OUTPUT_FUNCTION) | 执行器输出功能 |
| 6              |                                                              |                                                       |                |
| 7              |                                                              |                                                       |                |


### MAV_CMD_CONFIGURE_ACTUATOR (311) 

执行机构配置命令。

| 参数（标签）   | 说明             | 值                                                    |
| -------------- | ---------------- | ----------------------------------------------------- |
| 1 （配置）     | 执行机构配置操作 | [ACTUATOR_CONFIGURATION](#ACTUATOR_CONFIGURATION)     |
| 2              |                  |                                                       |
| 3              |                  |                                                       |
| 4              |                  |                                                       |
| 5 （输出功能） | 执行器输出功能   | [ACTUATOR_OUTPUT_FUNCTION](#ACTUATOR_OUTPUT_FUNCTION) |
| 6              |                  |                                                       |
| 7              |                  |                                                       |


### MAV_CMD_COMPONENT_ARM_DISARM (400) 

布防/撤防组件

| 参数（标签） | 说明                                                         | 数值                             |
| ------------ | ------------------------------------------------------------ | -------------------------------- |
| 1 (Arm)      | 0: disarm, 1: arm                                            | min: 0 max： 1 inc: 1            |
| 2（强制）    | 0：布防-撤防，除非安全检查阻止（如着陆时），21196：强制布防/撤防（例如，允许布防覆盖飞行前检查并在飞行中撤防） | 最小： 0 最大： 21196 inc: 21196 |


### MAV_CMD_RUN_PREARM_CHECKS (401) 

指示目标系统运行布防前检查。

这允许按需运行飞行前检查，对于通常以较低速率运行检查或在可布防状态发生变化时不触发检查的系统可能有用。
如果执行检查，该命令应返回 [MAV_RESULT_ACCEPTED]（#MAV_RESULT_ACCEPTED）。
检查结果通常会在 [SYS_STATUS](#SYS_STATUS) 消息中报告（这与系统有关）。
如果系统已经布防，命令应返回 [MAV_RESULT_TEMPORARILY_REJECTED](#MAV_RESULT_TEMPORARILY_REJECTED)。

| 参数（标签） | 说明 |
| ------------ | ---- |


### MAV_CMD_ILLUMINATOR_ON_OFF (405) 

打开/关闭照明器。照明器是用于照亮系统外部黑暗区域的光源，如手电筒或探照灯（与照亮系统本身的光源相反，如指示灯）。

| 参数（标签） | 说明                         | 值                               |
| ------------ | ---------------------------- | -------------------------------- |
| 1 (Enable)   | 0: 照明器关闭，1: 照明器开启 | 最小： 0 最大： 1 inc： 1 inc: 1 |


### MAV_CMD_DO_ILLUMINATOR_CONFIGURE (406) 

配置照明器设置。照明器是用于照亮系统外部黑暗区域的光源，如手电筒或探照灯（与照亮系统本身的光源相反，如指示灯）。

| 参数（标签）    | 说明                                            | 数值                                  | 单位 |
| --------------- | ----------------------------------------------- | ------------------------------------- | ---- |
| 1 （模式）      | 模式                                            | [ILLUMINATOR_MODE](#ILLUMINATOR_MODE) |      |
| 2 (Brightness)  | 0%： 关闭，100%： 0%: 关闭，100%: 最大亮度      | min: 0 max： 100                      | %    |
| 3 （频闪周期）  | 频闪周期（秒），0 表示不使用频闪                | 最小： 0                              | 秒   |
| 4 (Strobe Duty) | 闪烁占空比，100% 表示持续亮起，0 表示不使用频闪 | min: 0 max： 100                      | %    |


### MAV_CMD_GET_HOME_POSITION (410) — [DEP] 

<span class="warning">**DEPRECATED:** Replaced By [MAV_CMD_REQUEST_MESSAGE](#MAV_CMD_REQUEST_MESSAGE) (2022-04)</span>

向车辆请求原点位置。

车辆将 ACK 该命令，然后发出 [HOME_POSITION](#HOME_POSITION) 信息。

| 参数（标签） | 说明     |
| ------------ | -------- |
| 1            | 保留     |
| 2            | Reserved |
| 3            | Reserved |
| 4            | Reserved |
| 5            | Reserved |
| 6            | Reserved |
| 7            | Reserved |


### MAV_CMD_INJECT_FAILURE (420) 

为测试目的注入人为故障。请注意，自动驾驶仪在接受此命令（如特定参数设置）前应实施额外保护。

| 参数（标签）     | 说明                             | 值                            |
| ---------------- | -------------------------------- | ----------------------------- |
| 1 （故障单元）   | 受故障影响的单元。               | [failure_unit](#failure_unit) |
| 2 (Failure type) | 故障类型。                       | [failure_type](#failure_type) |
| 3 (Instance)     | 受故障影响的实例（0 表示全部）。 |                               |


### MAV_CMD_START_RX_PAIR (500) 

开始接收机配对。

| 参数（标签） | 说明        | 数值                |
| ------------ | ----------- | ------------------- |
| 1 (Spektrum) | 0:Spektrum. |                     |
| 2 (RC Type)  | RC 类型。   | [rc_type](#rc_type) |


### MAV_CMD_GET_MESSAGE_INTERVAL (510) — [DEP] 

<span class="warning">**DEPRECATED:** Replaced By [MAV_CMD_REQUEST_MESSAGE](#MAV_CMD_REQUEST_MESSAGE) (2022-04)</span>

请求特定 MAVLink 信息 ID 的信息间隔。
接收方应 ACK 该命令，然后在[MESSAGE_INTERVAL]（#MESSAGE_INTERVAL）消息中发出响应。

| 参数（标签）   | 说明            | 值                                                 |
| -------------- | --------------- | -------------------------------------------------- |
| 1 (Message ID) | MAVLink 消息 ID | 最小值： 0 最大值： 16777215 inc： 16777215 inc: 1 |


### MAV_CMD_SET_MESSAGE_INTERVAL (511) 

设置特定 MAVLink 信息 ID 的信息间隔。此接口取代 [REQUEST_DATA_STREAM]（#REQUEST_DATA_STREAM）。

| 参数（标签）   | 说明                                                         | 值                                             | 单位         |
| -------------- | ------------------------------------------------------------ | ---------------------------------------------- | ------------ |
| 1 (Message ID) | MAVLink 消息 ID                                              | 最小： 0 最大： 16777215 inc： 16777215 inc: 1 | 2 (Interval) |
| 2 (Interval)   | 两个信息之间的间隔。-1：禁用。0：请求默认速率（可能为零）。  | 最小：-1 inc: 1                                | us           |
| 7（响应目标）  | 信息流的目标地址（如果信息有目标地址字段）。0：Flight-stack 默认（推荐），1：请求者地址，2：广播。 | min: 0 max: 2 inc: 1                           |              |


### MAV_CMD_REQUEST_MESSAGE (512) 

请求目标系统发送指定消息的单个实例（即 [MAV_CMD_SET_MESSAGE_INTERVAL](#MAV_CMD_SET_MESSAGE_INTERVAL) 的 "一次性 "版本）。

| 参数（标签）          | 说明                                                         | 值                                     |
| --------------------- | ------------------------------------------------------------ | -------------------------------------- |
| 1 (Message ID)        | 请求信息的 MAVLink 信息 ID。                                 | 最小：0 最大：16777215 16777215 inc: 1 |
| 2 (Req Param 1)       | 如果需要，用于索引 ID。否则，必须在请求的报文中定义该参数（如有）的用途。默认为不使用（0）。 |                                        |
| 3 (Req Param 2)       | 必须在请求的报文中定义该参数（如有）的用途。默认情况下假设未使用（0）。 |                                        |
| 4 (Req Param 3)       | 必须在请求的报文中定义该参数（如有）的用途。默认情况下假设未使用（0）。 |                                        |
| 5 (Req Param 4)       | 必须在请求的报文中定义该参数（如有）的用途。默认情况下假设未使用（0）。 |                                        |
| 6 (Req Param 5)       | 必须在请求的报文中定义该参数（如有）的用途。默认情况下假设未使用（0）。 |                                        |
| 7 （Response Target） | 请求报文的目标地址（如果报文有目标地址字段）。0：Flight-stack 默认值，1：请求者地址，2：广播。 | 最小：0 最大：2 收入：1                |


### MAV_CMD_REQUEST_PROTOCOL_VERSION (519) — [DEP] 

<span class="warning">**DEPRECATED:** Replaced By [MAV_CMD_REQUEST_MESSAGE](#MAV_CMD_REQUEST_MESSAGE) (2019-08)</span>

请求兼容 MAVLink 协议版本。所有接收器都应 ACK 该命令，然后在 [PROTOCOL_VERSION](#PROTOCOL_VERSION) 消息中发出它们的能力。

| 参数（标签） | 说明                                | 值                               |
| ------------ | ----------------------------------- | -------------------------------- |
| 1（协议）    | 1：请求网络上所有节点支持的协议版本 | 最小： 0 最大： 1 inc： 1 inc: 1 |
| 2            | 保留（所有剩余参数                  |                                  |


### MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES (520) — [DEP] 

<span class="warning">**DEPRECATED:** Replaced By [MAV_CMD_REQUEST_MESSAGE](#MAV_CMD_REQUEST_MESSAGE) (2019-08)</span>

请求自动驾驶功能。接收器应 ACK 该命令，然后在[AUTOPILOT_VERSION]（#AUTOPILOT_VERSION）报文中发布其功能。

| 参数（标签） | 说明                  | 值                               |
| ------------ | --------------------- | -------------------------------- |
| 1（版本）    | 1：请求自动驾驶仪版本 | 最小： 0 最大： 1 inc： 1 inc: 1 |
| 2            | 保留（所有剩余参数    |                                  |


### MAV_CMD_REQUEST_CAMERA_INFORMATION (521) — [DEP] 

<span class="warning">**DEPRECATED:** Replaced By [MAV_CMD_REQUEST_MESSAGE](#MAV_CMD_REQUEST_MESSAGE) (2019-08)</span>

请求摄像机信息（[CAMERA_INFORMATION](#CAMERA_INFORMATION)）。

| 参数（标签）     | 说明                                        | 值                    |
| ---------------- | ------------------------------------------- | --------------------- |
| 1 (Capabilities) | 0: No action 1: Request camera capabilities | min: 0 max： 1 inc: 1 |
| 2                | 保留（所有剩余参数                          |                       |


### MAV_CMD_REQUEST_CAMERA_SETTINGS (522) — [DEP] 

<span class="warning">**DEPRECATED:** Replaced By [MAV_CMD_REQUEST_MESSAGE](#MAV_CMD_REQUEST_MESSAGE) (2019-08)</span>

请求设置摄像机（[CAMERA_SETTINGS](#CAMERA_SETTINGS)）。

| 参数（标签） | 说明                                    | 值                    |
| ------------ | --------------------------------------- | --------------------- |
| 1 (Settings) | 0: No Action 1: Request camera settings | min: 0 max： 1 inc: 1 |
| 2            | 保留（所有剩余参数                      |                       |


### MAV_CMD_REQUEST_STORAGE_INFORMATION (525) — [DEP] 

<span class="warning">**DEPRECATED:** Replaced By [MAV_CMD_REQUEST_MESSAGE](#MAV_CMD_REQUEST_MESSAGE) (2019-08)</span>

请求设置摄像机（[CAMERA_SETTINGS](#CAMERA_SETTINGS)）。

| 参数（标签） | 说明                                    | 值                    |
| ------------ | --------------------------------------- | --------------------- |
| 1 (Settings) | 0: No Action 1: Request camera settings | min: 0 max： 1 inc: 1 |
| 2            | 保留（所有剩余参数                      |                       |


### MAV_CMD_STORAGE_FORMAT (526) 

格式化存储介质。格式化完成后，将发送一条 [STORAGE_INFORMATION](#STORAGE_INFORMATION) 信息。使用该命令的 target_component 来指定特定组件的存储介质。

| 参数（标签）        | 说明                                                         | 值                                |
| ------------------- | ------------------------------------------------------------ | --------------------------------- |
| 1 (Storage ID)      | 存储 ID（1 表示第一个，2 表示第二个等）                      | 最小： 0 最大： 1                 |
| 2 (Format)          | 格式化存储（并重置图像日志）。0: 无操作 1: 格式化存储        | 最小： 0 最大： 1 inc: 1 1 inc: 1 |
| 3 (Reset Image Log) | 重置图像日志（不格式化存储介质）。这将重置 CAMERA_CAPTURE_STATUS.image_count 和 CAMERA_IMAGE_CAPTURED.image_index。0: 无操作 1: 重置图像日志 | 最小： 0 最大： 1 inc: 1 1 inc: 1 |
| 4                   | 保留（所有剩余参数                                           |                                   |


### MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS (527) — [DEP] 

<span class="warning">**DEPRECATED:** Replaced By [MAV_CMD_REQUEST_MESSAGE](#MAV_CMD_REQUEST_MESSAGE) (2019-08)</span>

请求摄像机捕捉状态（[CAMERA_CAPTURE_STATUS](#CAMERA_CAPTURE_STATUS)

| 参数（标签）       | 说明                                          | 值                    |
| ------------------ | --------------------------------------------- | --------------------- |
| 1 (Capture Status) | 0: No Action 1: Request camera capture status | min: 0 max： 1 inc: 1 |
| 2                  | 保留（所有其余参数                            |                       |


### MAV_CMD_REQUEST_FLIGHT_INFORMATION (528) — [DEP] 

<span class="warning">**DEPRECATED:** Replaced By [MAV_CMD_REQUEST_MESSAGE](#MAV_CMD_REQUEST_MESSAGE) (2019-08)</span>

请求航班信息（[FLIGHT_INFORMATION](#FLIGHT_INFORMATION)

| 参数（标签）  | 说明               | 值                               |
| ------------- | ------------------ | -------------------------------- |
| 1（航班信息） | 1：请求航班信息    | 最小： 0 最大： 1 inc： 1 inc: 1 |
| 2             | 保留（所有其余参数 |                                  |


### MAV_CMD_RESET_CAMERA_SETTINGS (529) 

将所有摄像机设置重置为出厂默认设置

| 参数（标签） | 说明                      | 值                               |
| ------------ | ------------------------- | -------------------------------- |
| 1 (重置)     | 0: 无操作 1: 重置所有设置 | 最小： 0 最大： 1 inc： 1 inc: 1 |
| 2            | 保留（所有其余参数        |                                  |


### MAV_CMD_SET_CAMERA_MODE (530) 

设置摄像机运行模式。保留值使用 NaN。如果摄像机支持视频流，GCS 将在模式更改后发送 [MAV_CMD_REQUEST_VIDEO_STREAM_STATUS]（#MAV_CMD_REQUEST_VIDEO_STREAM_STATUS）命令。

| 参数（标签）   | 说明                                                         | 值                          |
| -------------- | ------------------------------------------------------------ | --------------------------- |
| 1 (id)         | 目标摄像机 ID。7 至 255： MAVLink 摄像机组件 ID。1 至 6 表示没有明确组件 ID 的摄像机（例如自动驾驶仪连接的摄像机）。0：所有摄像机。用于专门针对自动驾驶仪连接的摄像机或多传感器 MAVLink 摄像机中的单个传感器。在任务中使用 MAV_CMD 时，它还可用于定位特定的摄像机。 |                             |
| 2 （相机模式） | 相机模式                                                     | [CAMERA_MODE](#CAMERA_MODE) |
| 3              |                                                              |                             |
| 4              |                                                              |                             |
| 7              |                                                              |                             |


### MAV_CMD_SET_CAMERA_ZOOM (531) 

设置摄像机变焦。摄像机必须响应 [CAMERA_SETTINGS](#CAMERA_SETTINGS)（成功时）信息。

| 参数（标签）   | 说明                               | 值                                    |
| -------------- | ---------------------------------- | ------------------------------------- |
| 1 （缩放类型） | 缩放类型                           | [CAMERA_ZOOM_TYPE](#CAMERA_ZOOM_TYPE) |
| 2 (Zoom Value) | 缩放值。有效值范围取决于变焦类型。 |                                       |
| 3              |                                    |                                       |
| 4              |                                    |                                       |
| 7              |                                    |                                       |


### MAV_CMD_SET_CAMERA_FOCUS (532) 

设置摄像机对焦。摄像机必须响应 [CAMERA_SETTINGS](#CAMERA_SETTINGS)（成功时）信息。

| 参数（标签）    | 说明     | 值                                |
| --------------- | -------- | --------------------------------- |
| 1 （对焦类型）  | 对焦类型 | [SET_FOCUS_TYPE](#SET_FOCUS_TYPE) |
| 2 (Focus Value) | 焦点值   |                                   |
| 3               |          |                                   |
| 4               |          |                                   |
| 7               |          |                                   |


### MAV_CMD_SET_STORAGE_USAGE (533) 

设置特定存储设备为保存照片、视频和/或其他媒体的首选位置（例如，设置 SD 卡用于存储视频）。

每种特定媒体类型只能有一个首选保存位置：如果在任何其他存储设备上设置了媒体使用标志，则该标志将被清除/重置。
如果未设置标记，系统将使用默认存储。
目标机系统可以选择始终使用默认存储，在这种情况下，它应该以 [MAV_RESULT_UNSUPPORTED](#MAV_RESULT_UNSUPPORTED) ACK 命令。
目标系统可以选择不允许将特定存储设备设置为首选存储设备，在这种情况下，目标系统应使用 [MAV_RESULT_DENIED](#MAV_RESULT_DENIED)来 ACK 命令。

| 参数（标签）   | 说明                                        | 值                                        |
| -------------- | ------------------------------------------- | ----------------------------------------- |
| 1 (Storage ID) | 存储 ID（1 表示第一个，2 表示第二个，等等） | 最小值：0，最大值：1                      |
| 2 (Usage)      | 使用标志                                    | [STORAGE_USAGE_FLAG](#STORAGE_USAGE_FLAG) |


### MAV_CMD_SET_CAMERA_SOURCE (534) 

设置摄像机信号源。在具有多个图像传感器的摄像机上更改摄像机的活动信号源。

| 参数（标签）    | 说明                                                         | 值                              |
| --------------- | ------------------------------------------------------------ | ------------------------------- |
| 1（设备 ID）    | 要寻址的摄像机的组件 ID，非 MAVLink 摄像机为 1-6，所有摄像机为 0。 |                                 |
| 2（主信号源）   | 主信号源                                                     | [CAMERA_SOURCE](#CAMERA_SOURCE) |
| 3（二级信号源） | 二级信号源。如果非零，第二个信号源将显示为画中画。           | [CAMERA_SOURCE](#CAMERA_SOURCE) |


### MAV_CMD_JUMP_TAG (600) 

标记的跳转目标。可使用 [MAV_CMD_DO_JUMP_TAG](#MAV_CMD_DO_JUMP_TAG)跳转。

| 参数（标签） | 说明 | 值                  |
| ------------ | ---- | ------------------- |
| 1 (Tag)      | Tag. | 最小值：0 最大值：1 |


### MAV_CMD_DO_JUMP_TAG (601) 

跳转到任务列表中的匹配标签。重复此操作指定次数。任务的每次跳转都应包含一个匹配标签。如果不是这种情况，则跳转到缺失的标签时应完成任务，而跳转到有多个匹配标签时应始终选择任务序列号最低的标签。

| 参数（标签） | 说明               | 数值                 |
| ------------ | ------------------ | -------------------- |
| 1 (Tag)      | 跳转到的目标标签。 | 最小值：0，最大值：1 |
| 2 (Repeat)   | 重复次数。         | 最小值：0 最大值：1  |


### MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW (1000) 

设置万向节管理器俯仰/偏航设定点（低速率指令）。可以设置以下数值的组合。例如，可以使用角度和所需的角速度来以一定的角速度达到该角度，或者仅使用角速度将导致连续转动。NaN 表示未设置。注意：只有万向节管理器会对该命令做出反应，万向节设备将忽略该命令。如果需要更高速度的俯仰/偏航设置点流，请使用 [GIMBAL_MANAGER_SET_PITCHYAW]（#GIMBAL_MANAGER_SET_PITCHYAW）。

| 参数（标签）             | 说明                                                         | 数值                                          | 单位            |
| ------------------------ | ------------------------------------------------------------ | --------------------------------------------- | --------------- |
| 1（俯仰角）              | 俯仰角（向上俯仰时为正值，"跟随 "模式下相对于车辆，"锁定 "模式下相对于地平线）。 | 最小：-180 最大：180 180                      | 度              |
| 2（偏航角）              | 偏航角（向右偏航的正偏航角，在 FOLLOW（跟随）模式下相对于车辆，在 LOCK（锁定）模式下绝对偏向北方）。 | 最小： -180 最大： 180 180                    | 度              |
| 3（俯仰速率）            | 俯仰速率（正向俯仰）。                                       | 度/秒                                         |                 |
| 4 （偏航率）             | 偏航率（向右偏航为正）。                                     | 度/秒                                         |                 |
| 5 (Gimbal Manager flags) | 要使用的云台管理器标志。                                     | [gimbal_manager_flags](#gimbal_manager_flags) | 7 (云台设备 ID) |
| 7（万向节设备 ID）       | 要寻址的万向节设备组件 ID（或 1-6 用于非 MAVLink 万向节），0 用于所有万向节设备组件。为多个万向节（但不是所有万向节）多次发送命令。 |                                               |                 |


### MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE (1001) 

万向节配置，用于设置哪个 sysid/compid 是主控和副控。

| 参数（标签）                 | 说明                                                         |
| ---------------------------- | ------------------------------------------------------------ |
| 1 (sysid primary control)    | 主控制的 Sysid（0：无人控制；-1：保持不变；-2：设置自身为控制方（用于自身 sysid 仍然未知的任务）；-3：如果当前为控制方，则取消控制）。 |
| 2 (compid primary control)   | 主要控制的 Compid（0：无人控制，-1：保持不变，-2：设置自身为控制方（用于自身 sysid 仍然未知的任务），-3：如果当前为控制方，则取消控制）。 |
| 3 (sysid secondary control)  | 二级控制的 Sysid（0：无人控制，-1：保持不变，-2：将自身设置为控制状态（用于自身 sysid 仍然未知的任务），-3：如果当前处于控制状态，则取消控制）。 |
| 4 (compid secondary control) | 二级控制的 Compid（0：无人控制，-1：保持不变，-2：自行设置为控制（用于自身 sysid 仍然未知的任务），-3：如果当前处于控制中，则移除控制）。 |
| 7（万向节设备 ID）           | 要寻址的万向节设备的组件 ID（或 1-6 用于非 MAVLink 万向节），0 用于所有万向节设备组件。为多个万向节（但不是所有万向节）发送多次命令。 |


### MAV_CMD_IMAGE_START_CAPTURE (2000) 

开始图像捕捉序列。每次捕获后必须发出 [CAMERA_IMAGE_CAPTURED](#CAMERA_IMAGE_CAPTURED)。


参数 1（id）可用于指定目标摄像机： 0：所有摄像机，1 至 6：自动驾驶仪连接的摄像机，7-255： MAVLink 摄像机组件 ID。
需要使用它来指定与自动驾驶仪连接的特定摄像机或多传感器摄像机中的特定传感器（这两种摄像机都没有明确的 MAVLink 组件 ID）。
在任务中指定目标摄像机时也需要它。

在任务中使用时，自动驾驶仪应针对指定的本地摄像机（param1 = 1-6）执行[MAV_CMD](#mav_commands)，如果是针对 MAVLink 摄像机（param1 = 7-255），则将其作为命令重新发送，并将命令的 target_component 设置为 param1 值（并将命令中的 param1 设置为 0）。
如果参数 1 为 0，自动驾驶仪将同时执行这两项操作。

当命令发送时，目标 MAVLink 地址通过 target_component 设置。
如果是专门发送给自动驾驶仪：则参数 1 的使用方式应与任务相同（不过，如果指定的本地摄像机不存在，则命令会以 [MAV_RESULT_DENIED]（#MAV_RESULT_DENIED）进行 NACK）。
如果寻址到 MAVLink 摄像机，参数 1 可用于寻址所有摄像机（0），或分别寻址 1 至 7 个传感器。其他值应使用 [MAV_RESULT_DENIED](#MAV_RESULT_DENIED)退回。
如果命令是广播式的（target_component 为 0），则参数 1 应设置为 0（任何其他值都应使用 [MAV_RESULT_DENIED](#MAV_RESULT_DENIED)）。自动驾驶仪将触发任何本地摄像机并将命令转发到所有通道。

| 参数（标签）          | 描述                                                         | 值                     | 单位 |
| --------------------- | ------------------------------------------------------------ | ---------------------- | ---- |
| 1 (id)                | 目标摄像机 ID。7 至 255： MAVLink 摄像机组件 ID。1 至 6 表示没有明确组件 ID 的摄像机（例如自动驾驶仪连接的摄像机）。0：所有摄像机。用于专门针对自动驾驶仪连接的摄像机或多传感器 MAVLink 摄像机中的单个传感器。在任务中使用 MAV_CMD 时，它还可用于锁定特定的摄像机。 |                        |      |
| 2 (Interval) （间隔） | 两张连续图像之间的预期经过时间（秒）。最小值取决于硬件（通常大于 2 秒）。 | 最小值：0              | 秒   |
| 3（图像总数）         | 要捕捉的图像总数。0 表示永久捕获/直到 MAV_CMD_IMAGE_STOP_CAPTURE。 | 最小值：0，最大值：1。 |      |
| 4 (Sequence Number)   | 捕获序列号，从 1 开始，仅对单次捕获有效（param3 ==1），否则设为 0。 增加每次捕获命令的捕获 ID，以防止命令重新传输时重复捕获。 | 最小值：1，最大值：1。 |      |
| 5                     |                                                              |                        |      |
| 6                     |                                                              |                        |      |
| 7                     |                                                              |                        |      |


### MAV_CMD_IMAGE_STOP_CAPTURE (2001) 

停止图像捕捉序列。


参数 1（id）可用于指定目标摄像机： 0：所有摄像机，1 至 6：自动驾驶仪连接的摄像机，7-255： MAVLink 摄像机组件 ID。
需要使用它来指定与自动驾驶仪连接的特定摄像机或多传感器摄像机中的特定传感器（这两种摄像机都没有明确的 MAVLink 组件 ID）。
在任务中指定目标摄像机时也需要它。

在任务中使用时，自动驾驶仪应针对指定的本地摄像机（param1 = 1-6）执行[MAV_CMD](#mav_commands)，如果是针对 MAVLink 摄像机（param1 = 7-255），则将其作为命令重新发送，并将命令的 target_component 设置为 param1 值（并将命令中的 param1 设置为 0）。
如果参数 1 为 0，自动驾驶仪将同时执行这两项操作。

当命令发送时，目标 MAVLink 地址通过 target_component 设置。
如果是专门发送给自动驾驶仪：则参数 1 的使用方式应与任务相同（不过，如果指定的本地摄像机不存在，则命令会以 [MAV_RESULT_DENIED]（#MAV_RESULT_DENIED）进行 NACK）。
如果寻址到 MAVLink 摄像机，param1 可用于寻址所有摄像机（0），或分别寻址 1 至 7 个传感器。其他值应使用 [MAV_RESULT_DENIED](#MAV_RESULT_DENIED)进行 NACK。
如果命令是广播式的（target_component 为 0），则参数 1 应设置为 0（任何其他值都应使用 [MAV_RESULT_DENIED](#MAV_RESULT_DENIED)）。自动驾驶仪将触发任何本地摄像机并将命令转发到所有通道。

| 参数（标签） | 说明                                                         | 值   |
| ------------ | ------------------------------------------------------------ | ---- |
| 1 (id)       | 目标摄像机 ID。7 至 255： MAVLink 摄像机组件 ID。1 至 6 表示没有明确组件 ID 的摄像机（例如自动驾驶仪连接的摄像机）。0：所有摄像机。用于专门针对自动驾驶仪连接的摄像机或多传感器 MAVLink 摄像机中的单个传感器。在任务中使用 MAV_CMD 时，它还可用于定位特定的摄像机。 |      |
| 2            |                                                              |      |
| 3            |                                                              |      |
| 4            |                                                              |      |
| 5            |                                                              |      |
| 6            |                                                              |      |
| 7            |                                                              |      |


### MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE (2002) — [DEP] 

<span class="warning">**DEPRECATED:** Replaced By [MAV_CMD_REQUEST_MESSAGE](#MAV_CMD_REQUEST_MESSAGE) (2019-08)</span>

重新请求一条 [CAMERA_IMAGE_CAPTURED](#CAMERA_IMAGE_CAPTURED) 信息。

| 参数（标签） | 说明                                    | 值                |
| ------------ | --------------------------------------- | ----------------- |
| 1（编号）    | 缺少 CAMERA_IMAGE_CAPTURED 消息的序列号 | 最小： 0 最大： 1 |
| 2            |                                         |                   |
| 3            |                                         |                   |
| 4            |                                         |                   |
| 5            |                                         |                   |
| 6            |                                         |                   |
| 7            |                                         |                   |


### MAV_CMD_DO_TRIGGER_CONTROL (2003) 

启用或禁用板载摄像机触发系统。

| 参数（标签） | 说明                                                   | 值                             |
| ------------ | ------------------------------------------------------ | ------------------------------ |
| 1 (Enable)   | 启用/禁用触发器（0 表示禁用，1 表示启动），-1 表示忽略 | 最小值：-1 最大值：-1 1 inc: 1 |
| 2 (Reset)    | 1 表示重置触发顺序，-1 或 0 表示忽略                   | min: -1 max： 1 inc: 1         |
| 3 (Pause)    | 1 用于暂停触发，但不会关闭或收回摄像机。-1忽略         | 最小：-1 最大：1 1 inc: 2      |


### MAV_CMD_CAMERA_TRACK_POINT (2004) 

如果摄像机支持点视觉跟踪（已设置[CAMERA_CAP_FLAGS_HAS_TRACKING_POINT](#CAMERA_CAP_FLAGS_HAS_TRACKING_POINT)），则可以使用此命令启动跟踪。

| 参数（标签） | 说明                                                         | 值                    |
| ------------ | ------------------------------------------------------------ | --------------------- |
| 1 (Point x)  | 跟踪点 x 值（归一化为 0...1，0 为左，1 为右）。              | 最小值：0 最大值：1 1 |
| 2 (Point y)  | 指向轨迹 y 值（归一化为 0...1，0 表示顶部，1 表示底部）。    | 最小： 0 最大： 1 1   |
| 3 (Radius)   | 点半径（归一化 0...1，0 表示一个像素，1 表示整个图像宽度）。 | 最小：0 最大：1 1     |


### MAV_CMD_CAMERA_TRACK_RECTANGLE (2005) 

如果摄像机支持矩形视觉跟踪（已设置[CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE](#CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE)），则可以使用此命令启动跟踪。

| 参数（标签）              | 说明                                                        | 值              |
| ------------------------- | ----------------------------------------------------------- | --------------- |
| 1 (Top Left Corner x)     | 矩形左上角 x 值（归一化为 0...1，0 为左，1 为右）。         | 最小：0 最大：1 |
| 2 (左上角 y)              | 矩形左上角 y 值（规范化为 0...1，0 表示顶部，1 表示底部）。 | 最小：0 最大：1 |
| 3 (Bottom right corner x) | 矩形右下角 x 值（规范化为 0...1，0 表示左，1 表示右）。     | 最小：0 最大：1 |
| 4 (Bottom right corner y) | 矩形右下角 y 值（规范化为 0...1，0 表示顶部，1 表示底部）。 | 最小：0 最大：1 |


### MAV_CMD_CAMERA_STOP_TRACKING (2010) 

Stops ongoing tracking.

| Param (Label) | Description |
| ------------- | ----------- |


### MAV_CMD_VIDEO_START_CAPTURE (2500) 

开始视频捕捉（录制）。

| 参数（标签）         | 说明                                                         | 数值                | 单位                 |
| -------------------- | ------------------------------------------------------------ | ------------------- | -------------------- |
| 1 (Stream ID)        | 视频流 ID（所有视频流均为 0）                                | 最小值：0 最大值：1 | 2 (Status Frequency) |
| 2 (Status Frequency) | 录制时发送 CAMERA_CAPTURE_STATUS 消息的频率（0 表示无消息，否则为频率） | min: 0              | Hz                   |
| 3                    |                                                              |                     |                      |
| 4                    |                                                              |                     |                      |
| 5                    |                                                              |                     |                      |
| 6                    |                                                              |                     |                      |
| 7                    |                                                              |                     |                      |


### MAV_CMD_VIDEO_STOP_CAPTURE (2501) 

停止当前视频采集（录制）。

| 参数（标签）    | 说明                          | 值                |
| --------------- | ----------------------------- | ----------------- |
| 1 （视频流 ID） | 视频流 ID（所有视频流均为 0） | 最小： 0 最大： 1 |
| 2               |                               |                   |
| 3               |                               |                   |
| 4               |                               |                   |
| 5               |                               |                   |
| 6               |                               |                   |
| 7               |                               |                   |


### MAV_CMD_VIDEO_START_STREAMING (2502) 

开始视频流

| 参数（标签）    | 说明                                                         | 值                |
| --------------- | ------------------------------------------------------------ | ----------------- |
| 1 （视频流 ID） | 视频流 ID（0 表示所有视频流，1 表示第一视频流，2 表示第二视频流等） | 最小： 0 最大： 1 |


### MAV_CMD_VIDEO_STOP_STREAMING (2503) 

停止给定的视频流

| 参数（标签）    | 说明                                                         | 值                |
| --------------- | ------------------------------------------------------------ | ----------------- |
| 1 （视频流 ID） | 视频流 ID（0 表示所有视频流，1 表示第一视频流，2 表示第二视频流等） | 最小： 0 最大： 1 |


### MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION (2504) — [DEP] 

<span class="warning">**DEPRECATED:** Replaced By [MAV_CMD_REQUEST_MESSAGE](#MAV_CMD_REQUEST_MESSAGE) (2019-08)</span>

请求视频流信息（[VIDEO_STREAM_INFORMATION](#VIDEO_STREAM_INFORMATION)

| 参数（标签）    | 说明                                                         | 值                |
| --------------- | ------------------------------------------------------------ | ----------------- |
| 1 （视频流 ID） | 视频流 ID（0 表示所有视频流，1 表示第一视频流，2 表示第二视频流等） | 最小： 0 最大： 1 |


### MAV_CMD_REQUEST_VIDEO_STREAM_STATUS (2505) — [DEP] 

<span class="warning">**DEPRECATED:** Replaced By [MAV_CMD_REQUEST_MESSAGE](#MAV_CMD_REQUEST_MESSAGE) (2019-08)</span>

请求视频流状态（[VIDEO_STREAM_STATUS](#VIDEO_STREAM_STATUS)

| 参数（标签）    | 说明                                                         | 值                |
| --------------- | ------------------------------------------------------------ | ----------------- |
| 1 （视频流 ID） | 视频流 ID（0 表示所有视频流，1 表示第一视频流，2 表示第二视频流等） | 最小： 0 最大： 1 |


### MAV_CMD_LOGGING_START (2510) 

请求开始通过 MAVLink 流式传输日志数据（另见 [LOGGING_DATA](#LOGGING_DATA) 消息）

| 参数（标签） | 描述           | 值            |
| ------------ | -------------- | ------------- |
| 1 （格式）   | 格式： 0: ULog | min: 0 inc: 1 |
| 2            | 保留（设置为 0 |               |
| 3            | 保留（设置为 0 |               |
| 4            | 保留（设置为 0 |               |
| 5            | 保留（设置为 0 |               |
| 6            | 保留（设置为 0 |               |
| 7            | 保留（设置为 0 |               |


### MAV_CMD_LOGGING_STOP (2511) 

请求停止通过 MAVLink 流式传输日志数据

| 参数（标签） | 说明             |
| ------------ | ---------------- |
| 1            | 保留（设为 0）   |
| 2            | 保留（设置为 0） |
| 3            | 保留（设置为 0） |
| 4            | 保留（设置为 0） |
| 5            | 保留（设置为 0） |
| 6            | 保留（设置为 0） |
| 7            | 保留（设置为 0） |


### MAV_CMD_AIRFRAME_CONFIGURATION (2520) 

| 参数（标签）    | 描述                                           | 值                                           |
| --------------- | ---------------------------------------------- | -------------------------------------------- |
| 1 （起落架 ID） | 起落架 ID（默认值：0，全部为-1）               | 最小值：-1，最大值：1                        |
| 2 (起落架位置)  | 起落架位置（向下：0，向上：1，NaN 表示无变化） | 起落架位置（向下：0，向上：1，NaN 表示无变化 |
| 3               |                                                |                                              |
| 4               |                                                |                                              |
| 5               |                                                |                                              |
| 6               |                                                |                                              |
| 7               |                                                |                                              |


### MAV_CMD_CONTROL_HIGH_LATENCY (2600) 

请求开始/停止高延迟遥测传输

| 参数（标签） | 说明                                       | 值                               |
| ------------ | ------------------------------------------ | -------------------------------- |
| 1 (Enable)   | 控制通过高延迟遥测传输（0: 停止，1: 开始） | 最小： 0 最大： 1 inc： 1 inc: 1 |
| 2            | Empty                                      |                                  |
| 3            | Empty                                      |                                  |
| 4            | Empty                                      |                                  |
| 5            | Empty                                      |                                  |
| 6            | Empty                                      |                                  |
| 7            | Empty                                      |                                  |


### MAV_CMD_PANORAMA_CREATE (2800) 

在当前位置创建全景图

| 参数（标签）       | 说明                            | 单位  |
| ------------------ | ------------------------------- | ----- |
| 1（水平角）        | 全景的水平视角（+- 0.5 总角度） | 度    |
| 2 (Vertical Angle) | 全景的垂直视角。                | 度    |
| 3 （水平速度）     | 水平旋转速度。                  | 度/秒 |
| 4 (Vertical Speed) | 垂直旋转速度。                  | 度/秒 |


### MAV_CMD_DO_VTOL_TRANSITION (3000) 

请求 VTOL 过渡

| 参数（标签） | 说明                                                         | 数值                              |
| ------------ | ------------------------------------------------------------ | --------------------------------- |
| 1（状态）    | 目标 VTOL 状态。对于正常转换，只能使用 MAV_VTOL_STATE_MC 和 MAV_VTOL_STATE_FW。 | [mav_vtol_state](#mav_vtol_state) |
| 2（立即）    | 强制立即过渡到指定的 MAV_VTOL_STATE。1：强制立即转换，0：正常转换。例如，可用于触发紧急 "Quadchute"。注意： 可能会造成危险/损坏飞行器，具体取决于自动驾驶仪对该命令的执行情况。 |                                   |


### MAV_CMD_ARM_AUTHORIZATION_REQUEST (3001) 

请求授权外部实体布防车辆，布防授权人负责在授权或拒绝请求前请求车辆提供所需的所有数据。

如果批准，[COMMAND_ACK](#COMMAND_ACK) 信息进度栏应设置该授权的有效期（以秒为单位）。
如果拒绝授权，[COMMAND_ACK](#COMMAND_ACK).result_param2 应设置为 [ARM_AUTH_DENIED_REASON](#ARM_AUTH_DENIED_REASON)中的原因之一。

| 参数（标签） | 说明                                                  | 值                         |
| ------------ | ----------------------------------------------------- | -------------------------- |
| 1（系统 ID） | 车辆系统 ID，这样地面站就可以代表任何车辆请求布防授权 | 最小： 0 最大： 255 inc: 1 |


### MAV_CMD_SET_GUIDED_SUBMODE_STANDARD (4000) 

当飞行器处于制导模式时，该命令将子模式设置为标准制导模式。飞行器将保持位置和高度，用户可沿所有三个轴输入所需的速度。

| Param (Label) | Description |
| ------------- | ----------- |


### MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE (4001) 

当飞行器处于制导模式时，该命令设置子模式圆。飞行器沿圆周飞行，面向圆心。用户可以输入沿圆飞行的速度并改变半径。如果没有输入，飞行器将保持位置不变。

| 参数（标签）  | 说明                           | 单位                   |
| ------------- | ------------------------------ | ---------------------- |
| 1 (Radius)    | CIRCLE_MODE 模式下所需圆的半径 | m                      |
| 2             | 用户定义                       |                        |
| 3             | 用户定义                       |                        |
| 4             | 用户定义                       |                        |
| 5 (Latitude)  | CIRCLE_MODE 中圆心的目标纬度   | degE7                  |
| 6 (Longitude) | CIRCLE_MODE                    | degE7 中圆心的目标经度 |


### MAV_CMD_CONDITION_GATE (4501) — [WIP] 

<span class="warning">**WORK IN PROGRESS**: Do not use in stable production environments (it may change).</span>

延迟任务状态机，直到达到门。

| 参数（标签）    | 说明                                               | 数值                   | 单位 |
| --------------- | -------------------------------------------------- | ---------------------- | ---- |
| 1（几何形状）   | 几何形状： 0：与上一个和下一个航点之间的路径正交。 | 最小值：0，最大值：1。 |      |
| 2 (UseAltitude) | 高度：0：忽略高度： 1 inc: 1                       |                        |      |
| 3               | 空                                                 |                        |      |
| 4               | 空                                                 |                        |      |
| 5 (Latitude)    | 纬度                                               |                        |      |
| 6 (Longitude)   | 经度                                               |                        |      |
| 7 (高度)        | 高度                                               |                        | 米   |


### MAV_CMD_NAV_FENCE_RETURN_POINT (5000) 

栅栏返回点（一个地理栅栏定义中只能有一个这样的点）。如果支持集结点，则应使用集结点。

| 参数（标签） | 说明 | 单位 |
| ------------ | ---- | ---- |
| 1            | 保留 |      |
| 2            | 保留 |      |
| 3            | 保留 |      |
| 4            | 保留 |      |
| 5（纬度）    | 纬度 | 经度 |
| 6 (经度)     | 经度 |      |
| 7 (高度)     | 高度 | 米   |


### MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION (5001) 

包含多边形的栅栏顶点（多边形不得自交）。车辆必须停留在该区域内。至少需要 3 个顶点。

| 参数（标签）                  | 说明                                                         | 数值                   |
| ----------------------------- | ------------------------------------------------------------ | ---------------------- |
| 1 （顶点数）                  | 多边形顶点数                                                 | 最少：3 个，最多：1 个 |
| 2 (Inclusion Group（包含组）) | 车辆必须位于单个组中的所有包含区内，车辆必须位于至少一个组内，每个多边形中的所有点必须相同 | min: 0 inc: 1          |
| 3                             | 保留                                                         |                        |
| 4                             | 保留                                                         |                        |
| 5 (Latitude)                  | 纬度                                                         |                        |
| 6 (Longitude)                 | 经度                                                         |                        |
| 7                             | 保留                                                         |                        |


### MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION (5002) 

栅栏顶点，用于隔离多边形（多边形不得自交）。车辆必须停留在该区域之外。至少需要 3 个顶点。

| 参数（标签）  | 说明         | 数值                   |
| ------------- | ------------ | ---------------------- |
| 1 （顶点数）  | 多边形顶点数 | 最少：3 个，最多：1 个 |
| 2             | 保留         |                        |
| 3             | 保留         |                        |
| 4             | 保留         |                        |
| 5 (Latitude)  | 纬度         |                        |
| 6 (Longitude) | 经度         |                        |
| 7             | 保留         |                        |


### MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION (5003) 

圆形围栏区域。车辆必须停留在此区域内。

| Param (Label)       | Description                                                  | Values        | Units |
| ------------------- | ------------------------------------------------------------ | ------------- | ----- |
| 1 (Radius)          | Radius.                                                      |               | m     |
| 2 (Inclusion Group) | 车辆必须位于单个组内的所有包含区域内，车辆必须位于至少一个组内 | min: 0 inc: 1 |       |
| 3                   | Reserved                                                     |               |       |
| 4                   | Reserved                                                     |               |       |
| 5 (Latitude)        | Latitude                                                     |               |       |
| 6 (Longitude)       | Longitude                                                    |               |       |
| 7                   | Reserved                                                     |               |       |


### MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION (5004) 

圆形围栏区域。车辆必须停留在此区域之外。

| Param (Label) | Description | Units |
| ------------- | ----------- | ----- |
| 1 (Radius)    | Radius.     | m     |
| 2             | Reserved    |       |
| 3             | Reserved    |       |
| 4             | Reserved    |       |
| 5 (Latitude)  | Latitude    |       |
| 6 (Longitude) | Longitude   |       |
| 7             | Reserved    |       |


### MAV_CMD_NAV_RALLY_POINT (5100) 

集合点。您可以定义多个集合点。

| Param (Label) | Description | Units |
| ------------- | ----------- | ----- |
| 1             | Reserved    |       |
| 2             | Reserved    |       |
| 3             | Reserved    |       |
| 4             | Reserved    |       |
| 5 (Latitude)  | Latitude    |       |
| 6 (Longitude) | Longitude   |       |
| 7 (Altitude)  | Altitude    | m     |


### MAV_CMD_UAVCAN_GET_NODE_INFO (5200) 

命令车辆以一系列消息 [UAVCAN_NODE_INFO](#UAVCAN_NODE_INFO) 进行响应，每个在线的 UAVCAN 节点发送一条消息。请注意，部分响应消息可能会丢失，接收方可以通过检查每个收到的 [UAVCAN_NODE_STATUS](#UAVCAN_NODE_STATUS) 是否具有先前收到的匹配消息 [UAVCAN_NODE_INFO](#UAVCAN_NODE_INFO) 来轻松检测；如果没有，则应再次发送此命令以请求重新传输节点信息消息。

| Param (Label) | Description         |
| ------------- | ------------------- |
| 1             | Reserved (set to 0) |
| 2             | Reserved (set to 0) |
| 3             | Reserved (set to 0) |
| 4             | Reserved (set to 0) |
| 5             | Reserved (set to 0) |
| 6             | Reserved (set to 0) |
| 7             | Reserved (set to 0) |


### MAV_CMD_DO_SET_SAFETY_SWITCH_STATE (5300) 

改变安全开关的状态。

| Param (Label)     | Description        | Values                                      |
| ----------------- | ------------------ | ------------------------------------------- |
| 1 (Desired State) | 新的安全开关状态。 | [SAFETY_SWITCH_STATE](#SAFETY_SWITCH_STATE) |
| 2                 | Empty.             |                                             |
| 3                 | Empty.             |                                             |
| 4                 | Empty              |                                             |
| 5                 | Empty.             |                                             |
| 6                 | Empty.             |                                             |
| 7                 | Empty.             |                                             |


### MAV_CMD_DO_ADSB_OUT_IDENT (10001) 

触发 ADSB-out IDENT 的启动。只有在管制空域内空中交通管制员要求时才可使用。启动 IDENT 后，硬件通常会根据模式 A、C 和 S 应答机规格将 IDENT 保持 18 秒。

| Param (Label) | Description         |
| ------------- | ------------------- |
| 1             | Reserved (set to 0) |
| 2             | Reserved (set to 0) |
| 3             | Reserved (set to 0) |
| 4             | Reserved (set to 0) |
| 5             | Reserved (set to 0) |
| 6             | Reserved (set to 0) |
| 7             | Reserved (set to 0) |


### MAV_CMD_PAYLOAD_PREPARE_DEPLOY (30001) — [DEP] 

<span class="warning">**DEPRECATED:**(2021-06)</span>

在纬度/经度/海拔位置部署有效载荷。这包括到达所需释放位置和速度的导航。

| 参数（标签）  | 说明                                                         | 值                          | 单位  |
| ------------- | ------------------------------------------------------------ | --------------------------- | ----- |
| 1（操作模式） | 操作模式。0：准备单个有效载荷部署（覆盖以前的请求），但不执行。1：立即执行有效载荷部署（执行期间拒绝进一步部署命令，但允许中止）。2：将有效载荷部署添加到现有部署列表。 | 最小值：0 最大值：2 增量：1 |       |
| 2（进近矢量） | 罗盘航向中的所需进近矢量。负值表示系统可以随意定义进近矢量。 | 最小值：-1 最大值：360      | 度    |
| 3（地速）     | 释放时的所需地速。如果需要满足最低空速，机身可以覆盖此值。负值表示系统可以随意定义地速。 | 最小值：-1                  |       |
| 4 (高度间隙)  | 到释放位置的最小高度间隙。负值表示系统可以随意定义间隙。     | min: -1                     | m     |
| 5 (纬度)      | 纬度。                                                       |                             | degE7 |
| 6 (经度)      | 经度。                                                       |                             | degE7 |
| 7 (高度)      | 高度 (MSL)                                                   |                             | m     |


### MAV_CMD_PAYLOAD_CONTROL_DEPLOY (30002) — [DEP] 

<span class="warning">**DEPRECATED:**(2021-06)</span>

控制有效载荷部署。

| 参数（标签）  | 说明                                                         | 值                            |
| ------------- | ------------------------------------------------------------ | ----------------------------- |
| 1（操作模式） | 操作模式。0：中止部署，继续正常任务。1：切换到有效载荷部署模式。100：删除第一个有效载荷部署请求。101：删除所有有效载荷部署请求。 | 最小值：0 最大值：101 增量：1 |
| 2             | Reserved                                                     |                               |
| 3             | Reserved                                                     |                               |
| 4             | Reserved                                                     |                               |
| 5             | Reserved                                                     |                               |
| 6             | Reserved                                                     |                               |
| 7             | Reserved                                                     |                               |


### MAV_CMD_WAYPOINT_USER_1 (31000) 

用户定义的航路点项目。地面站将显示车辆飞过该项目。

| Param (Label) | Description        | Units |
| ------------- | ------------------ | ----- |
| 1             | User defined       |       |
| 2             | User defined       |       |
| 3             | User defined       |       |
| 4             | User defined       |       |
| 5 (Latitude)  | Latitude unscaled  |       |
| 6 (Longitude) | Longitude unscaled |       |
| 7 (Altitude)  | Altitude (MSL)     | m     |


### MAV_CMD_WAYPOINT_USER_2 (31001) 

用户定义的航路点项目。地面站将显示车辆飞过该项目。

| Param (Label) | Description        | Units |
| ------------- | ------------------ | ----- |
| 1             | User defined       |       |
| 2             | User defined       |       |
| 3             | User defined       |       |
| 4             | User defined       |       |
| 5 (Latitude)  | Latitude unscaled  |       |
| 6 (Longitude) | Longitude unscaled |       |
| 7 (Altitude)  | Altitude (MSL)     | m     |


### MAV_CMD_WAYPOINT_USER_3 (31002) 

用户定义的航点项目。地面站将显示飞行器飞过该项目。

| Param (Label) | Description        | Units |
| ------------- | ------------------ | ----- |
| 1             | User defined       |       |
| 2             | User defined       |       |
| 3             | User defined       |       |
| 4             | User defined       |       |
| 5 (Latitude)  | Latitude unscaled  |       |
| 6 (Longitude) | Longitude unscaled |       |
| 7 (Altitude)  | Altitude (MSL)     | m     |


### MAV_CMD_WAYPOINT_USER_4 (31003) 

用户定义的航点项目。地面站将显示飞行器飞过该项目。

| Param (Label) | Description        | Units |
| ------------- | ------------------ | ----- |
| 1             | User defined       |       |
| 2             | User defined       |       |
| 3             | User defined       |       |
| 4             | User defined       |       |
| 5 (Latitude)  | Latitude unscaled  |       |
| 6 (Longitude) | Longitude unscaled |       |
| 7 (Altitude)  | Altitude (MSL)     | m     |


### MAV_CMD_WAYPOINT_USER_5 (31004) 

用户定义的航点项目。地面站将显示飞行器飞过该项目。

| Param (Label) | Description        | Units |
| ------------- | ------------------ | ----- |
| 1             | User defined       |       |
| 2             | User defined       |       |
| 3             | User defined       |       |
| 4             | User defined       |       |
| 5 (Latitude)  | Latitude unscaled  |       |
| 6 (Longitude) | Longitude unscaled |       |
| 7 (Altitude)  | Altitude (MSL)     | m     |


### MAV_CMD_SPATIAL_USER_1 (31005) 

用户定义的航点项目。地面站将显示飞行器飞过该项目。

| Param (Label) | Description        | Units |
| ------------- | ------------------ | ----- |
| 1             | User defined       |       |
| 2             | User defined       |       |
| 3             | User defined       |       |
| 4             | User defined       |       |
| 5 (Latitude)  | Latitude unscaled  |       |
| 6 (Longitude) | Longitude unscaled |       |
| 7 (Altitude)  | Altitude (MSL)     | m     |


### MAV_CMD_SPATIAL_USER_2 (31006) 

用户定义的航点项目。地面站将显示飞行器飞过该项目。

| Param (Label) | Description        | Units |
| ------------- | ------------------ | ----- |
| 1             | User defined       |       |
| 2             | User defined       |       |
| 3             | User defined       |       |
| 4             | User defined       |       |
| 5 (Latitude)  | Latitude unscaled  |       |
| 6 (Longitude) | Longitude unscaled |       |
| 7 (Altitude)  | Altitude (MSL)     | m     |


### MAV_CMD_SPATIAL_USER_3 (31007) 

用户定义的航点项目。地面站将显示飞行器飞过该项目。

| Param (Label) | Description        | Units |
| ------------- | ------------------ | ----- |
| 1             | User defined       |       |
| 2             | User defined       |       |
| 3             | User defined       |       |
| 4             | User defined       |       |
| 5 (Latitude)  | Latitude unscaled  |       |
| 6 (Longitude) | Longitude unscaled |       |
| 7 (Altitude)  | Altitude (MSL)     | m     |


### MAV_CMD_SPATIAL_USER_4 (31008) 

用户定义的航点项目。地面站将显示飞行器飞过该项目。

| Param (Label) | Description        | Units |
| ------------- | ------------------ | ----- |
| 1             | User defined       |       |
| 2             | User defined       |       |
| 3             | User defined       |       |
| 4             | User defined       |       |
| 5 (Latitude)  | Latitude unscaled  |       |
| 6 (Longitude) | Longitude unscaled |       |
| 7 (Altitude)  | Altitude (MSL)     | m     |


### MAV_CMD_SPATIAL_USER_5 (31009) 

用户定义的航点项目。地面站将显示飞行器飞过该项目。

| Param (Label) | Description        | Units |
| ------------- | ------------------ | ----- |
| 1             | User defined       |       |
| 2             | User defined       |       |
| 3             | User defined       |       |
| 4             | User defined       |       |
| 5 (Latitude)  | Latitude unscaled  |       |
| 6 (Longitude) | Longitude unscaled |       |
| 7 (Altitude)  | Altitude (MSL)     | m     |


### MAV_CMD_USER_1 (31010) 

用户自定义指令。地面站将不会显示飞行器通过此项目飞行。举例说明： [MAV_CMD_DO_SET_PARAMETER](#MAV_CMD_DO_SET_PARAMETER) 项目。

| Param (Label) | Description  |
| ------------- | ------------ |
| 1             | 用户定义     |
| 2             | User defined |
| 3             | User defined |
| 4             | User defined |
| 5             | User defined |
| 6             | User defined |
| 7             | User defined |


### MAV_CMD_USER_2 (31011) 

用户自定义指令。地面站将不会显示飞行器通过此项目飞行。举例说明： [MAV_CMD_DO_SET_PARAMETER](#MAV_CMD_DO_SET_PARAMETER) 项目。

| Param (Label) | Description  |
| ------------- | ------------ |
| 1             | User defined |
| 2             | User defined |
| 3             | User defined |
| 4             | User defined |
| 5             | User defined |
| 6             | User defined |
| 7             | User defined |


### MAV_CMD_USER_3 (31012) 

用户自定义指令。地面站将不会显示飞行器通过此项目飞行。举例说明： [MAV_CMD_DO_SET_PARAMETER](#MAV_CMD_DO_SET_PARAMETER) 项目。

| Param (Label) | Description  |
| ------------- | ------------ |
| 1             | User defined |
| 2             | User defined |
| 3             | User defined |
| 4             | User defined |
| 5             | User defined |
| 6             | User defined |
| 7             | User defined |


### MAV_CMD_USER_4 (31013) 

用户自定义指令。地面站将不会显示飞行器通过此项目飞行。举例说明： [MAV_CMD_DO_SET_PARAMETER](#MAV_CMD_DO_SET_PARAMETER) 项目。

| Param (Label) | Description  |
| ------------- | ------------ |
| 1             | User defined |
| 2             | User defined |
| 3             | User defined |
| 4             | User defined |
| 5             | User defined |
| 6             | User defined |
| 7             | User defined |


### MAV_CMD_USER_5 (31014) 

用户自定义指令。地面站将不会显示飞行器通过此项目飞行。举例说明： [MAV_CMD_DO_SET_PARAMETER](#MAV_CMD_DO_SET_PARAMETER) 项目。

| Param (Label) | Description  |
| ------------- | ------------ |
| 1             | User defined |
| 2             | User defined |
| 3             | User defined |
| 4             | User defined |
| 5             | User defined |
| 6             | User defined |
| 7             | User defined |


### MAV_CMD_CAN_FORWARD (32000) 

请求将 CAN 数据包从给定的 CAN 总线转发到该组件。CAN 帧使用 [CAN_FRAME](#CAN_FRAME) 和 [CANFD_FRAME](#CANFD_FRAME) 报文发送。

| Param (Label) | Description                                                  |
| ------------- | ------------------------------------------------------------ |
| 1 (bus)       | Bus number (0 to disable forwarding, 1 for first bus, 2 for 2nd bus, 3 for 3rd bus). |
| 2             | Empty.                                                       |
| 3             | Empty.                                                       |
| 4             | Empty.                                                       |
| 5             | Empty.                                                       |
| 6             | Empty.                                                       |
| 7             | Empty.                                                       |


### MAV_CMD_FIXED_MAG_CAL_YAW (42006) 

基于已知偏航进行磁力计校准。这样就可以仅根据已知的车辆偏航情况，使用车辆中的 WMM 场表进行快速校准。如果纬度和经度均为零，则使用当前车辆位置。

| Param (Label)   | Description                    | Units |
| --------------- | ------------------------------ | ----- |
| 1 (Yaw)         | Yaw of vehicle in earth frame. | deg   |
| 2 (CompassMask) | CompassMask, 0 for all.        |       |
| 3 (Latitude)    | Latitude.                      | deg   |
| 4 (Longitude)   | Longitude.                     | deg   |
| 5               | Empty.                         |       |
| 6               | Empty.                         |       |
| 7               | Empty.                         |       |


### MAV_CMD_DO_WINCH (42600) 

指挥操作绞盘。

| Param (Label) | Description          | Values                          | Units |
| ------------- | -------------------- | ------------------------------- | ----- |
| 1 (Instance)  | 绞车实例编号。       | min: 1 inc: 1                   |       |
| 2 (Action)    | 要执行的操作。       | [WINCH_ACTIONS](#WINCH_ACTIONS) |       |
| 3 (Length)    | 放线长度（负风向）。 |                                 | m     |
| 4 (Rate)      | 释放率（负风向）。   |                                 | m/s   |
| 5             | Empty.               |                                 |       |
| 6             | Empty.               |                                 |       |
| 7             | Empty.               |                                 |       |

### MAV_CMD_EXTERNAL_POSITION_ESTIMATE (43003)

提供外部位置估计值，供死循环时使用。该功能用于外部系统（如通过视频链接使用地标的远程飞行员）偶尔提供的位置重设。

| Param (Label)         | Description                                                  | Units |
| --------------------- | ------------------------------------------------------------ | ----- |
| 1 (transmission_time) | 以发送器时域中的时间表示的信息发送时间戳。发送方应根据应用所需的时间精度和 32 位浮点运算的限制，将时间归零。例如，将时间换算为 10 小时，精度约为 1 毫秒。收件人必须在应用于该字段的任何定时抖动校正中处理时间包络。包络滚动时间不应超过 250 秒，这将提供约 10 微秒的精度。 | s     |
| 2 (processing_time)   | 处理作为该位置基础的传感器数据所花费的时间。接收者可以用它来改进数据的时间校准。如果不知道，则设为零. | s     |
| 3 (accuracy)          | 估计测量精度的一个标准差。如果不知道，则设置为 NaN。         |       |
| 4                     | Empty                                                        |       |
| 5 (Latitude)          | Latitude                                                     |       |
| 6 (Longitude)         | Longitude                                                    |       |
| 7 (Altitude)          | 高度，未使用。应以 NaN 发送。本报文未来版本可能支持。        | m     |