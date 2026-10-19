# FPGA-to-TouchDesigner Ultrasonic Sensor Pipeline

Real-time distance sensing on a Cyclone V SoC. Two HC-SR04 ultrasonic sensors are measured by custom Verilog state machines in the FPGA fabric, exposed to embedded Linux through a custom **Avalon-MM** peripheral on the **HPS lightweight bridge**, and streamed to **TouchDesigner** as **OSC over UDP**.

Target hardware: Terasic **DE1-SoC** (Intel Cyclone V SoC 5CSEMA5F31C6: FPGA fabric + dual-core ARM Cortex-A9 HPS).

## Hardware Setup

- Terasic DE1-SoC board (Cyclone V SoC)
- 2x HC-SR04 ultrasonic sensors
- 2x 1 kΩ and 2x 2 kΩ resistors (Echo voltage dividers)
- Breadboard and jumper wires
- microSD card with a Linux image
- Ethernet cable and router
- USB cable for the on-board USB-Blaster
- A PC running TouchDesigner

---

## Highlights

- **Hardware/software co-design across the full stack:** sensor physics, RTL, bus interconnect, Linux userspace, and network protocol, all built and debugged on real hardware.
- **Custom Verilog FSMs** that generate trigger pulses and measure echo pulse widths with 1 µs resolution, including timeout guards and a `busy` handshake for safe multi-sensor scheduling.
- **Custom Avalon Memory-Mapped slave** packaged as a Platform Designer component and wired to the HPS lightweight HPS-to-FPGA bridge.
- **Platform Designer system built from scratch**: HPS configuration, clocking, exports, address map, and generated interconnect.
- **Linux userspace register access** via `mmap()` on `/dev/mem`, with no kernel driver.


---

### FPGA (Verilog)

| File | Description |
|---|---|
| `RTL/top.v` | Top-level module. Instantiates both sensor drivers, the sequential scheduler, the us-to-cm conversion, the 7-segment display logic, and the Platform Designer `soc_system`. Declares the HPS DDR3, Ethernet, SD, USB, and UART pin ports. |
| `RTL/hcsr04_driver.v` | Per-sensor driver FSM: 10 µs trigger pulse, echo rise detection, echo-width measurement in microseconds, timeout guards, recovery delay, and `busy`/`done` handshake signals. |
| `RTL/sensor_avalon_slave.v` | Custom Avalon-MM slave exposing the two distance values as read-only 32-bit registers (word 0 = sensor 1, word 1 = sensor 2). |
| `RTL/seg7_decode.v` | BCD-to-7-segment decoder (active low) for the DE1-SoC HEX displays. |

### Platform Designer / Quartus

| File | Description |
|---|---|
| `soc_system.qsys` | Platform Designer system: 50 MHz clock source, Cyclone V HPS, 10-bit PIO for the LEDs, and the custom sensor slave, with the address map and exports. |
| `sensor_avalon_slave_hw.tcl` | Component descriptor created by the Component Editor, defining the sensor slave's clock, reset, Avalon-MM slave, and conduit interfaces. |
| `TD_project.qpf` / `TD_project.qsf` | Quartus project and settings (device, top-level entity, pin assignments, file list). |

### Linux userspace (C, runs on the HPS)

| File | Description |
|---|---|
| `osc_data.c` | The production program. Polls both sensor registers over the lightweight bridge and sends `/sensor1` and `/sensor2` OSC messages to TouchDesigner at about 10 Hz. Usage: `sudo ./osc_data <TD_IP> <port>`. |
| `sensor_read_test.c` | Bring-up tool that prints both distance registers once per second, to verify the FPGA-to-Linux path before adding networking. |
| `mmap_test.c` | Bridge sanity check: writes patterns to the LED PIO through `mmap()` and confirms the LEDs respond. |

---

## System Architecture

```
  HC-SR04 x2   (Trig out, Echo in through 1k/2k dividers: 5 V -> ~3.3 V)
       |
       v
  +----------------------------- FPGA fabric (50 MHz) -----------------------------+
  |  1 us tick (clock enable)                                                       |
  |        |                                                                        |
  |  hcsr04_driver x2  (trigger / echo-width FSMs, timeouts, busy)                  |
  |        |                                                                        |
  |  sequential scheduler (sensor 1, then sensor 2, gated on !busy)                 |
  |        |                                                                        |
  |  us -> cm --> dist1_cm, dist2_cm (10-bit)                                       |
  |        |                         |                                              |
  |        |                         +--> seg7_decode x6 --> HEX0-HEX5              |
  |        v                                                                        |
  |  sensor_avalon_slave  (Avalon-MM slave, 32-bit read-only registers)             |
  +-----------|---------------------------------------------------------------------+
              |    Platform Designer generated interconnect
              v
  HPS lightweight HPS-to-FPGA bridge  (AXI-3 master, 32-bit, base 0xFF200000)
              |
              v
  Linux on ARM Cortex-A9:  osc_data
      mmap(/dev/mem) -> read registers -> build OSC packet -> UDP socket
              |
              v   Ethernet, OSC over UDP (/sensor1, /sensor2 as float32, cm)
  TouchDesigner:  OSC In CHOP
```

---

## Project Overview

### 1. Sensing layer

An HC-SR04 measures distance by emitting an ultrasonic burst when its Trig pin receives a 10 µs pulse, then raising its Echo pin for a duration proportional to the round-trip time of the sound. With sound travelling about 340 m/s, the echo high-time in microseconds divided by 58 gives distance in centimetres.

**Electrical interface:** the sensors run at 5 V and their Echo output is 5 V, but the Cyclone V GPIO is 3.3 V only. Each Echo line passes through its own 1 kΩ / 2 kΩ resistor divider (about 3.3 V after division) before reaching the FPGA. The 3.3 V Trig output from the FPGA is above the sensor's logic-high threshold, so Trig connects directly. The sensors are powered from the DE1-SoC's 5 V GPIO header pin, with a common ground.

### 2. FPGA fabric design

**Timing base.** A counter divides 50 MHz down to a **1 µs clock enable** (a single-cycle pulse every 50 cycles) rather than a derived clock. Counters throughout the design advance on that enable, so microsecond-resolution timing is simple while the design stays fully synchronous.

**Per-sensor FSM (`hcsr04_driver`).** Six states:

| State | Behaviour |
|---|---|
| `IDLE` | Waits for `start`. Drives `trig` low and `busy` low. |
| `TRIG` | Holds `trig` high for 10 µs. |
| `WAIT_RISE` | Waits for Echo to rise, with a 30 ms timeout for a missing or out-of-range echo. |
| `MEASURE` | Counts microseconds while Echo is high, with a 30 ms timeout guard against a stuck-high Echo. |
| `LATCH` | Registers the result (0 on a timeout), pulses `done`. |
| `RECOVER` | Waits 60 ms (the sensor's recommended settling time) before allowing another trigger. |

`busy` is high from `start` until the FSM is back in `IDLE`. The measured width is a 17-bit microsecond count, converted to a 10-bit centimetre value in `top.v`.

**Two-sensor scheduler.** A four-state FSM (`S_START1`, `S_WAIT1`, `S_START2`, `S_WAIT2`) triggers the sensors one at a time so one sensor's ultrasonic pulse cannot be mistaken for the other's echo. It only issues a `start` pulse when the target sensor reports `!busy`, then waits for `done` and latches the result. Each sensor's reading refreshes roughly 6 to 8 times per second, limited mainly by the mandatory recovery interval.

**7 Seg display** The latest distance values are shown to 3 digits each on DE1-SoC's six 7-segment displays. This served as the standalone verification stage before any Linux integration.

### 3. Platform Designer system

The HPS and the fabric are tied together by a Platform Designer (Qsys) system, `soc_system`, built from scratch:

| Component | Role |
|---|---|
| `clk_0` | 50 MHz clock source (exported as the system clock and reset). |
| `hps_0` | Cyclone V Hard Processor System. |
| `pio_0` | 10-bit output PIO driving the red LEDs, used as a known-good bridge sanity check. |
| `sensor_avalon_slave_0` | The custom sensor register component. |

**HPS configuration.** The lightweight HPS-to-FPGA bridge is enabled at 32 bits. The full-width HPS-to-FPGA bridge, the FPGA-to-HPS bridge, and the FPGA-to-SDRAM ports are all set to unused, which keeps the generated interconnect small. The HPS peripherals the board uses (Gigabit Ethernet, SD/MMC, USB, UART) and the DDR3 interface are configured and exported so they map to the dedicated HPS pins.

**Bus connectivity.** The HPS's lightweight bridge master (`h2f_lw_axi_master`, AXI-3) connects to the Avalon-MM slave ports of the PIO and the sensor component. Platform Designer generates the interconnect that handles address decoding, protocol translation, and clock/reset distribution.

**Address map.** Addresses are offsets from the bridge base `0xFF200000`:

| Peripheral | Offset | Physical address | Contents |
|---|---|---|---|
| LED PIO | `0x0000` | `0xFF200000` | LED data register |
| Sensor slave, word 0 | `0x0100` | `0xFF200100` | Sensor 1 distance (cm) |
| Sensor slave, word 1 | `0x0104` | `0xFF200104` | Sensor 2 distance (cm) |

**The custom Avalon-MM slave (`sensor_avalon_slave`).** The interface is minimal: `clk`, `reset`, a 1-bit word `address`, `read`, and a 32-bit `readdata`. The two sensor distance values enter as exported **conduits** (`dist1`, `dist2`, 10 bits each) and are zero-extended into 32-bit words. The slave is read-only with a **fixed read latency of 1** and no `waitrequest`: `readdata` is registered when `read` is asserted. The component declares word-based address units, so word address 1 appears on the bus as byte offset `0x4`. All 32 bits are used as the register width because that is the native width of the lightweight bridge and of the ARM's 32-bit loads, so no width adaptation is needed.

**Top-level integration.** `top.v` instantiates `soc_system`, connects the board clock, feeds `dist1_cm` and `dist2_cm` into the sensor conduits, drives `LEDR` from the PIO, and connects the HPS DDR3 and peripheral ports. The HPS reset input is deliberately tied inactive rather than to a pushbutton: `KEY[0]` resets only the sensor logic, so resetting the FSMs never reboots the running Linux system.

**Physical design details.** After Analysis and Synthesis, the generated `hps_sdram_p0_pin_assignments.tcl` script assigns the dedicated HPS DDR3 pin locations and on-chip termination settings before fitting.

### 4. HPS and embedded Linux

The HPS boots Linux from a microSD card image (Ubuntu/Linaro-based, with GCC and Python, kernel 3.18) and is reachable over Ethernet and SSH. The FPGA fabric is configured over JTAG with the Quartus Programmer after Linux has booted, which replaces the board's default reference design in the fabric without disturbing the running HPS.

**Register access with `mmap()`.** Linux user processes cannot touch physical addresses directly, so the program maps the bridge window into its own virtual address space:

1. `open("/dev/mem", O_RDWR | O_SYNC)`: opens physical memory (needs root); `O_SYNC` requests uncached, device-style access.
2. `mmap(NULL, SPAN, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0xFF200000)`: maps the bridge's physical window. The base is page-aligned, and the mapping is rounded up to whole 4 KB pages.
3. Registers are read through `volatile unsigned long *` pointers at `virtual_base + 0x100` and `+ 0x104`. `volatile` forces a real bus read each time, so the compiler cannot cache a value that hardware changes asynchronously.

Each read travels through the bridge and interconnect to the sensor slave and comes back as a plain integer.

### 5. Network layer: OSC over UDP

`osc_data` encodes OSC messages by hand. Each message contains:

- the **address pattern** (`/sensor1` or `/sensor2`), null-terminated and padded to a 4-byte boundary;
- the **type tag string** `,f` (one float argument), also padded;
- a **32-bit IEEE 754 float** holding the distance in centimetres, in **big-endian** byte order.

Because C does not permit byte-order operations on a `float`, the program reinterprets the float's bits as an integer through a union, converts the byte order with `htonl()`, and copies the 4 bytes into the packet. Packets are sent through a UDP socket (`sendto`) to the TouchDesigner machine, roughly 10 times per second.

### 6. TouchDesigner

An **OSC In CHOP** listening on the chosen UDP port receives the two channels, `sensor1` and `sensor2`, which can drive any parameter in a TouchDesigner network.


---


## Future Work

- Possibly faster sensor read updates. They are limited in timing because we dont need them to poll as fast as possible to minimize crosstalk, but we could push
timings to the limit. Could also trigger the sensors in parallel to roughly double the update rate.
- Better TouchDesigner Visuals. 