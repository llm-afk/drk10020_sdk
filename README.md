# drk10020_sdk

DRK10020 电源 USB 虚拟串口 Python SDK，Python 3.11+，115200／8N1。
常规收发最高固定 20 Hz，相邻发送至少 50 ms，无后台控制线程。

```powershell
python -m pip install .
python -m unittest discover -s tests -v
```

```python
import drk10020_sdk

info = drk10020_sdk.open()  # 自动筛选 CH340 并查询型号；多台时指定 open("COM9")
try:
    drk10020_sdk.disable()
    drk10020_sdk.exchange(5.0, 0.1)  # V、限流 A，保持当前输出开关状态
    drk10020_sdk.enable()
    for _ in range(100):
        sample = drk10020_sdk.exchange(5.0, 0.1)
        print(sample["voltage"], sample["current"], sample["fault"])
        events = drk10020_sdk.drain_events()  # 由调用脚本选择、完整保存
finally:
    try:
        drk10020_sdk.disable()  # 显式请求关闭输出，异常应交给调用方处理
    finally:
        drk10020_sdk.close()
        remaining_events = drk10020_sdk.drain_events()
```

| 接口 | 行为 |
| --- | --- |
| `open(port=None, timeout=1.0)` | 只查询型号、固件、设置、主从状态，不开启或关闭输出 |
| `settings()` | 用 ATRCON 读取实际设置，不是实时输出测量 |
| `enable()`／`disable()` | 显式开启／关闭输出，读取测量并核对设置；保留其他参数 |
| `exchange(voltage, current)` | 一次设置电压／限流并读取一个测量回帧 |
| `send(voltage, current)`＋`read()` | 同线程配对调用；read 只接收，不发送 |
| `drain_events()` | 转移全部待交付收发记录，关闭串口后仍可取尾部 |
| `stats()` | 当前连接的字节／事件计数、待取队列、缓冲和不确定状态 |
| `close()` | 只关闭串口，输出保持原状态 |

电压设置范围 0–100 V、限流 0.001–20 A。这是保守软件输入范围，两个上限不代表可以同时输出；实际功率档位仍由电源决定。单位为 V、A、W、℃。

反馈包含完整 `raw`（288 字节）、三组各 22 个样本、平均电压／电流、逐点乘积平均功率、温度、故障位图、原始状态和主机接收时间／序号。故障仅交付，不自动停机、清错、重试或重新使能。没有设备时间戳或事务号，不把测量回帧当作全部目标参数生效的证明。

不清空接收缓冲。完整帧、串口返回的原始字节块、异常帧、杂字节和关闭时未完成尾部均通过事件交付。事件队列可增长，不覆盖旧记录或因积压自动结束；调用方需持续 `drain_events()` 并保存。软件只能保留已由串口返回的数据，不承诺驱动层／线缆此前零丢失或无限内存。

控制部分写入、反馈超时等引发 `WriteUncertainError` 后，常规发送和使能锁定，不自动重发。只读查询与显式失能仍可用；检查设备后重新打开可开始新会话。SDK 应独占串口；同一控制循环中不要由面板或其他程序同时改参数。

协议与验证边界见 [来源](docs/protocol_sources.md)和[验证](docs/validation.md)。测试目录中的模拟设备只用于离线协议测试，不作为真实设备证据。
