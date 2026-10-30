# CLAUDE.md

Guidance for Claude Code when working in this repository.

## Project overview

A pool pump and diverter valve controller that manages:
- **Main pool pump** (240 VAC, switched by an SSR)
- **Boost pump** (for the pool cleaner, SSR)
- **Diverter valve** (motorized 3-way valve sending water to the roof solar panels or back to the pool;
  24 VAC transformer via SSR, direction via relay)
- Integration with the separate **solarthermal** controller, which requests water to the roof

Since 2026-10-29 the controller is a **Waveshare ESP32-S3-POE-ETH-8DI-8DO** (`poolcontrol_esp32/`), on
branch `esp32`. The original Arduino version (`poolpumpcontrol.ino`, SparkFun RedBoard, USB serial to
homecontrol, gobble scripts) is kept for reference only; it is no longer running.

## Hardware (ESP32 version)

| Part | Details |
|---|---|
| Main board | Waveshare ESP32-S3-POE-ETH-8DI-8DO: ESP32-S3 (16 MB flash, 8 MB PSRAM), W5500 Ethernet with 802.3af PoE, 8 isolated inputs, 8 isolated Darlington outputs (500 mA sink, built-in flyback diodes), PCF85063 RTC with CR1220 backup cell, isolated RS485 |
| Analog | Waveshare Modbus RTU Analog Input 8CH, address 1, 9600 8N1, on the RS485 port (A to A, B to B). Channel jumpers removed (voltage mode); the firmware sets data type 0 (0-5 V) on CH1-4 at start |
| Network | PoE over the CAT5 run to the data closet PoE switch (the run used to carry USB over Ethernet). static **192.168.1.7** set in the firmware (`poolpumpcontrol` in /etc/hosts; solartracker and solarthermal will be .8 and .9), mDNS `poolpumpcontrol.local` |
| Sensor power | The existing 5 V buck converter; sensor grounds tied to the analog module's power ground |

### Board pins (8DO variant)
RS485 TX 17, RX 18, **DE/RTS 21** (must be set up with `setPins(-1,-1,-1,21)` and
`UART_MODE_RS485_HALF_DUPLEX`, or nothing is ever received; the 8RO wiki doesn't mention it). I2C SDA 42,
SCL 41: TCA9554 0x20 (EXIO1-8 = DO1-8, **latch bit 0 = output on**), PCF85063 0x51. W5500 CS 16, INT 12,
SCLK 15, MOSI 13, MISO 14, RST 39. DI1-8 = GPIO4-11 (INPUT_PULLUP, low = active). RGB LED 38, buzzer 46.

### Wiring
| Terminal | Connected to |
|---|---|
| DO1 | Main pump SSR (SSR + to +12 V, SSR - to DO1) |
| DO2 | Boost pump SSR |
| DO3 | Diverter 24 VAC transformer SSR |
| DO4 | Diverter direction SPDT relay coil (+12 V to coil to DO4); **not installed yet** (issue), needed only for solar heating |
| DO GND | 12 V supply negative. DO COM should go to +12 V for the flyback diodes (issue) |
| DI1 | Mechanical timer switch, dry contact to the DI ground |
| DI2 | Solarthermal pool-heat request: 12 VDC when asserted, + to DI2, - to the DI ground |
| DI ground | **Isolated: only the timer switch and the solarthermal pair.** Do not tie it to the box's common ground; that bonds the box to solarthermal's ground over the 100'+ run (ground loop, noise; DI2 followed the timer switch while it was tied) |
| Analog CH1, CH2 | Pool LM36s (same probe) |
| Analog CH3 | Filter pressure sensor (about 0.27 V at 0 psi) |
| Analog CH4 | Outside LM36 |

## Software (`poolcontrol_esp32/poolcontrol_esp32.ino`)

The Arduino version's control logic, with new I/O, clock and telemetry. TaskScheduler tasks: sensors 500 ms
(16-sample trimmed mean), main pump and boost pump 1 s, diverter 6 s, inputs 100 ms, telemetry 1 s, relay
check 10 s.

### Control rules
- **Timer switch on**: main pump on immediately, regardless of anything else (no drain-down wait, no time
  limit, no overpressure or midnight shutoff).
- **Boost pump**: on as soon as pressure is >= 5 psi with the main pump on, if the timer switch or the
  periodic run wants it; off when neither does or the main pump stops; **never more than 10 s below 5 psi,
  in any mode**; no time limit while the timer switch is on (otherwise 1 h, 45 min periodic).
- When the timer switch turns off, the pumps keep running only for another request (button/manual, noon
  filtering, solar diverter).
- Noon filtering: 2 h main pump plus a 45 min boost run, daily. Main pump limit 10 h (except timer).
- Overpressure: 23 psi in pool mode, 25 psi in roof mode (except timer).
- Drain-down: 5 min after the pump stops in roof mode before the valve returns to pool or the pump restarts.
- Diverter transformer power is limited to 5 min per move.
- Test modes (pump, boost, 24vac, diverter) time out back to Normal after 10 min. A '+' in pump mode
  sets the manual request, which persists into Normal; clear it with `poolctl button`.

### Clock
NTP (pool.ntp.org, time.apple.com), written to the RTC hourly. At power-up the RTC sets the clock
(verified 2026-10-29 after 5 min unpowered). RTC years outside 2025-2040 are treated as unset. Telemetry
is held until the clock is valid.

### Web interface (port 80)
`GET /status` (JSON, including `analog_mv` raw channel millivolts), `GET /log` (ring buffer),
`POST /cmd` (text/plain, `X-Token`), `POST /update` (firmware, `X-Token`).

### Telemetry
Same line format as the Arduino (`Date Time Pl1 Pl2 Out PSI Pmp Bst Req Roof Mode Runtime Boot`), sent on
a change or every 10 minutes, POSTed over HTTPS to `https://bedichek.org/home/api/ingest/poolpumpcontrol`
(`server/home_ingest.py`), which appends to `/var/www/html/home/poolpumpcontrol.txt` and sends lines
containing "alert" to Pushover. Uploads are switched with `poolctl upload on|off` (kept in flash; on since
2026-10-29).

### poolctl
```
poolctl build | update | status | log [-f] [BYTES]
poolctl mode normal|pump|boost|24vac|diverter
poolctl + | - | button | telemetry | reboot | upload on|off
poolctl flash [PORT]     # USB, first load only
```
**All updates go over Ethernet** (`poolctl update`); there is no USB host at the pool. A new image must
run a minute with the output expander answering **and have served `/status` over the network** (poolctl
update polls it) before it is marked good, or the bootloader rolls back. If you change the address, fetch
`/status` at the new address within that minute (`POOLCTL_HOST` points poolctl at the old one).

Build: arduino-cli, FQBN `esp32:esp32:esp32s3:FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,PSRAM=opi,CDCOnBoot=cdc`,
core esp32 3.3.x.

### Secrets
`poolcontrol_esp32/secrets.h` (git-ignored) is written by `poolctl` from `~/.config/poolctl/lan_token` and
`ingest_token`; the server's copy is `/etc/home-ingest/tokens`. Never print them.

## Other directories
- `esp32_bringup/` - bench test sketch for the board (I2C scan, RS485/Modbus probes, output and input
  tests). Not for the pool.
- `server/` - the bedichek.org ingest service (systemd unit and nginx snippet).
- `poolpumpcontrol.wiki/` - the GitHub wiki, its own git repo.
- `gobble_poolpumpcontrol.py`, `watch_serial_*`, start/stop scripts, `com.bedichek.watch_gobble_poolpumpcontrol.plist` -
  the Arduino-era logger on homecontrol, retired 2026-10-29 (plist moved to `~/Library/LaunchAgents/disabled/`).
- `fo` - an old ARP cache snapshot, not part of the project.
