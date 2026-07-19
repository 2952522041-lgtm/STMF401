# 四电机底层驱动说明

## 硬件映射

| 车轮 | TB6612 方向引脚 | PWM | 编码器 |
| --- | --- | --- | --- |
| 左前 FL | AIN1 / AIN2 | TIM5 CH1 | TIM1 |
| 右前 FR | BIN1 / BIN2 | TIM5 CH2 | TIM2 |
| 左后 BL | CIN1 / CIN2 | TIM5 CH3 | TIM3 |
| 右后 BR | DIN1 / DIN2 | TIM5 CH4 | TIM4 |

TIM9 保持为 HAL 的 1 kHz 系统时基。TIM10 使用 84-1 分频和 10000-1 自动重装值，在 84 MHz 定时器时钟下直接产生 100 Hz 中断，作为应用层控制信号。

电机方向系数在 `tb6612.h` 中设置，编码器方向系数、PPR 和减速比在 `encode.h` 中设置。上车前应逐轮悬空测试正负方向。

## UARTMODEL 协议

USART6 参数沿用 CubeMX 配置：115200、8N1。

USART6 RX 使用 DMA2 Stream1 / Channel 5 的 Receive-to-IDLE 接收方式。DMA 缓冲区为 64 字节，空闲事件或缓冲区接收完成后，将本次数据块交给协议状态机解析并立即重新启动 DMA。

帧格式：`AA 55 TYPE LENGTH PAYLOAD CHECKSUM`。校验字节是 `TYPE`、`LENGTH` 和全部负载字节的异或值，多字节数据均为小端序。

- `TYPE=0x01`：设置四路目标转速。负载依次为 FL、FR、BL、BR 四个 `int16_t`，单位 0.1 RPM，共 8 字节。
- `TYPE=0x02`：四轮停止，负载长度为 0。
- `TYPE=0x81`：100 Hz 遥测。负载依次为四路目标 RPM、四路实测 RPM（均为 `int16_t`，单位 0.1 RPM）及四路编码器原始计数（`int32_t`），共 32 字节。

## 速度环

`app_task.c` 使用 100 Hz 固定周期执行四路独立速度控制。四路控制器均使用 `Lib/CMSIS-DSP-1.17.0` 中的 `arm_pid_instance_f32`，PID 输出直接作为电机转速控制输出；`SPEED_PID_KP/KI/KD` 是底盘实测后需要调整的参数。
