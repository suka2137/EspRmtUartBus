## RMT UART (Draft release) v0.0
- Uses RMT for sending and receiving customizable UART signals (Any GPIO, 300-3.2M baud, 4-9 data bits, optional parity bit, 1/1.5/2 stops).
- Accurate with poor signals - basic software glitch filtering and slow edge compensation in addition to RMT's HW filter
- It can work without any extra electronic components.
- Reasonably simple and efficient for what it is - RMT TX is offloaded to hardware after the encoding, which runs at megabits/s
- The decoding runs in short bursts in ISR, but it could be offloaded to a task (not currently supported).
- Single-wire half-duplex mode using RMT TX and RX channels at once - up to 4 instances




|Supported SoC|Framework|RMTs|Receive notes|RMT Sleep Retention|
|---|---|---|---|---|
|ESP32-S3|ESP-IDF 6|4 TX + 4 RX| |Unsupported by HW|
|ESP32|ESP-IDF 6|8 TX/RX|**No RX thres int - limited to 12B / RMT Mem Block**|Unsupported by HW|
|ESP32-C3|ESP-IDF 6|2 TX + 2 RX| |Unsupported by HW|

It should also be possible to run in arduino.

RMT is not ideal for high speed UART, but 250k+ should be easily achieveable.

## current limitations
### putting this on github to guilt trip myself into fixing it
- No threading (Run only on main task's and RMT driver's core)
- Barely works, unsafe code
- No hibernation / power management for entering sleep - full deinit

### Usage
It can be used as a component or included in sources.
See the header for function descriptions





license: MIT (my creative input only)
