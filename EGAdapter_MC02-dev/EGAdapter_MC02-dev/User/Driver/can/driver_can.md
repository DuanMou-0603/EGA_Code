# CAN 通信模块（driver_can）

## 1. 模块简介
本模块封装了 STM32H723 的 FDCAN 外设驱动，提供面向对象的 CAN 通信接口。  

主要设计目的为：
- 提供简洁稳定的 CAN 发送/接收抽象接口
- 支持多实例，每实例绑定各自 CAN ID
- 自动配置硬件过滤器并启用中断回调
- 基于 HAL，零动态内存申请，适用于嵌入式系统

核心功能：
- 标准帧（11bit）收发
- RX 中断回调
- 自动滤波器管理
- 使用H7自带FIFO发送
- 支持 FDCAN1/2/3 三个总线

## 2. 使用前准备

### 2.1 CubeMX 配置要求

> 待修改。下面这段是AI生成的。

| 项目    | 配置要求                    |
|-------|-------------------------|
| 模式    | Classic CAN (非 CAN FD)  |
| ID 类型 | Standard ID (11-bit)    |
| FIFO  | RX FIFO0 使能             |
| 中断    | 使能 RX FIFO0 New Message |
| 滤波器   | 数量 ≥ 计划实例数              |
| 时钟    | 建议 80MHz（或满足数据率需求）      |


### 2.2 依赖
- HAL FDCAN 驱动
- C++17 编译环境


## 3. 配置说明

### 3.1 配置结构体

典型配置：绑定一个 TX ID 和一个 RX ID，并绑定一个函数名作为接收回调
```cpp
CANInstance::Config config = {
    .can_handle  = &hfdcan1,
    .tx_id       = 0x100,
    .rx_id       = 0x101,
    .rx_callback = onCanRx, // void onCanRx(const uint8_t* data, uint8_t len)
};
```

无回调（仅发送）
```cpp
CANInstance::Config tx_only_cfg = {
    .can_handle = &hfdcan2,
    .tx_id = 0x200,
    .rx_id = 0x201,         //可不填
    .rx_callback = nullptr, //可不填
};
```

### 3.2 配置参数说明

| 参数            | 说明               | 可选值            | 注意事项                 |
|---------------|------------------|----------------|----------------------|
| `can_handle`  | HAL FDCAN 句柄     | `&hfdcan1/2/3` | 必填                   |
| `tx_id`       | 发送使用的 CAN 标准帧 ID | 0x001 ~ 0x7FF  | 不可为 0，不能和 `rx_id` 相同 |
| `rx_id`       | 接收过滤目标 ID        | 0x001 ~ 0x7FF  | 需保证滤波器数量足够           |
| `rx_callback` | 接收回调函数           | 函数指针 / null    | 参数 `(data, len)`     |


## 4. 使用方法

### 4.1 定义回调

```cpp
void onCanRx(const uint8_t* data, uint8_t len) {
    // 处理接收到的数据
}
```

### 4.2 初始化实例

```cpp
CANInstance can1({
    .can_handle  = &hfdcan1,
    .tx_id       = 0x100,
    .rx_id       = 0x101,
    .rx_callback = onCanRx,
});
```

### 4.3 发送数据

```cpp
can_msg_t msg;
msg.data[0] = 0x55;
msg.length = 8; //也可以不提供，默认为8

can1.transmit(msg);  // 默认超时 150µs
```

使用指定 ID 发送（测试用途，不推荐写进上层应用中）

```cpp
uint8_t buf[8] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11, 0x22};
can1.transmit(0x102, buf, 8);
```

## 5. API 接口

### 5.1 类静态接口

| 接口                           | 功能描述                               |
|------------------------------|------------------------------------|
| `static CANRxFifoCallback()` | RX FIFO0 中断处理入口，用于重载HAL回调，用户无需手动调用 |

### 5.2 实例接口

| 接口                                 | 功能描述                 |
|------------------------------------|----------------------|
| `CANInstance(config)`              | 创建实例，配置滤波器并启动 CAN 服务 |
| `transmit(msg, timeout_us)`        | 发送 CAN 帧             |
| `transmit(id, data, len, timeout)` | 指定 ID 发送（测试用途）       |
| `setDataLength(len)`               | 设置帧长度（1~8字节）         |
| `getCanHandle()`                   | 获取底层 HAL 句柄          |
| `...`                              | ...                  |


## 6. 注意事项
- 默认仅支持 Standard ID
- 发送默认调用FIFO0, 如果FIFO0已满将阻塞等待
- `rx_callback` 仅回调数据和长度，**ID 已由实例绑定**
- 每个实例需占用硬件滤波器的一个索引。
- 同一条can线上实例总数最大值由CubeMX设置的`Std Filters Nbr`决定，使用时请注意不要超出。


## 7. 更新日志

| 日期         | 更新内容        |
|------------|-------------|
| 2025-11-04 | 补全说明文档      |
| 2025-09-25 | 初版 CAN 驱动实现 |
