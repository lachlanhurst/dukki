# Microduck Unitree: expression outputs (eye LED)

How to bring up the eye: one WS2812-type RGB LED in the head that shows what the robot is doing, and the `expressd` daemon that drives it. This is Dukki's addition; Pollen's robot has no software-controlled LED. Follows on from `compute-setup.md`.

Date: 03/10/2026. Written against vendor kernel 6.1.115-vendor-rk35xx, board device tree `rk3576-radxa-cm4-io.dtb`. Verified the same day: the test script shows red, green, blue and white in the right order with the data line driven straight from the 3.3 V header pin, and `expressd` follows robotd's state, takes mood and identify requests, and falls back to "asleep" when robotd stops.

## 1. The short version

1. Wire the breakout's input side with the board powered off. The output side (DO, 5V, GND) is for chaining a second LED and stays unconnected.

| Breakout | NANO-A header pin | Signal |
|---|---|---|
| 5V | 2 or 4 | 5 V from the head buck |
| GND | 20 or 25, or any ground | ground |
| DI | 19 | SPI1_MOSI_M0 |

2. From the Mac, install the overlay and udev rule, then reboot:

```
scp -r scripts/compute/setup-express.sh scripts/compute/express duck@microduck.local:
ssh -t duck@microduck.local sudo bash setup-express.sh
ssh -t duck@microduck.local sudo reboot
```

3. After the reboot, check the node and the LED:

```
ssh -t duck@microduck.local sudo bash setup-express.sh --check     # /dev/spidev-eye -> spidev1.0, group robot
ssh duck@microduck.local python3 express/eye-test.py               # red, green, blue, white, off
```

4. Install the daemon from the Mac: `scripts/compute/push-daemons.sh expressd`.

## 2. Hardware

The breakout is a generic red board sold as "WS2811 Breakout": a 5050 LED with the controller built in (nothing on the back), so it behaves as a WS2812, colour order GRB. It runs at 5 V and nominally wants 0.7 x VDD = 3.5 V for a logic high, which the RK3576's 3.3 V pins do not reach. This one works driven directly. If a replacement flickers or shows the wrong colours, put a diode (1N4148 or 1N5817) in series with the breakout's 5V: the LED then runs at about 4.3 V and 3.3 V clears its threshold. A 74AHCT1G125 buffer powered from 5 V is the proper fix. DI is an input, so nothing at 5 V reaches the SoC either way. Full white draws about 60 mA.

Keep the LED out of the camera's view and behind a diffuser; glare upsets the IMX219's auto exposure.

## 3. Why SPI

The LED's single-wire protocol is a pulse train at 800 kHz with about 150 ns of tolerance, which Linux cannot bit-bang from a GPIO. The STM32 bridge could, but it is in the trunk and would need another wire through the neck. Clocked out of SPI MOSI at 2.4 MHz, three SPI bits make one LED bit (100 for a 0, 110 for a 1), giving highs of 0.42 and 0.83 µs against the WS2812's 0.4 and 0.8 µs. Each frame is 9 data bytes with 96 zero bytes either side for the reset gap, sent as one `write()`, so the controller holds the timing.

The `microduck-nano-a-spi1` overlay enables SPI1 M0 with a `rockchip,spidev` device on CS0. Its pin group claims header pins 19 (MOSI), 21 (MISO) and 23 (SCLK); the chip-select pins 24 and 26 stay free. `setup-express.sh` installs it as an Armbian user overlay, the same way as the I2C8 overlay, and adds a udev rule that names the node `/dev/spidev-eye` (matched by the controller's address, `2ad00000.spi`) and gives it to the `robot` group.

## 4. The daemon

`expressd` is a new crate, `express`, on the `dukki` branch of `../microduck`. It touches no upstream crate: its methods are its own, not added to `duck-ipc-proto`. It runs as user `expressd` with the `robot` group.

What it shows is decided by a stack of layers. The highest one that is set wins.

| Layer | Set by | Look |
|---|---|---|
| privacy | a client | whatever the client asks; meant for "the camera is being watched", and nothing below can hide it |
| fault | robotd's state | fallen: red, blinking at 2 Hz |
| identify | `expressd identify` | white, fast blink, for 10 s by default |
| mood | a client | whatever the client asks |
| ambient | robotd's state | see below |

The ambient look follows `robot.state`, subscribed at 10 Hz:

| State | Look |
|---|---|
| no robotd, or no frame for 2 s | asleep: dim white, slow breathe (4 s) |
| limp (torque off) | violet, breathe (3 s) |
| running, no policy (`held`) | blue, breathe (3 s) |
| stand policy, or walk with nothing commanded | blue, steady |
| walking | cyan, steady |

Clients cannot set or clear `fault` or `ambient`; that would let them hide a fall or fake walking. Brightness is capped at 15 % (`--brightness`) and applied through a gamma curve. Animations render at about 30 Hz; a steady look is written on change and refreshed once a second. On stop the LED is turned off.

From a shell on the robot (any user in the `robot` group):

```
expressd status
expressd set mood 0 255 0 --pattern breathe --period 3
expressd set mood 255 120 0 --pattern pulse --ttl 30
expressd clear mood
expressd identify --seconds 5
```

Patterns: `solid`, `breathe`, `blink`, `pulse`. `expressd` is in `/opt/robot/daemon/current/bin/`.

Over the socket, `/run/expressd/express.sock`, one JSON-RPC 2.0 request per line, as many per connection as a client likes:

```
{"jsonrpc":"2.0","id":1,"method":"express.set","params":{"layer":"mood","colour":[0,255,0],"pattern":"breathe","period_s":3.0,"ttl_s":30}}
{"jsonrpc":"2.0","id":2,"method":"express.clear","params":{"layer":"mood"}}
{"jsonrpc":"2.0","id":3,"method":"express.identify","params":{"seconds":5}}
{"jsonrpc":"2.0","id":4,"method":"express.status"}
```

## 5. Open items

- The camera indicator. mediad has no call that says whether anyone is watching, and robotd's `robot.remoteSessionActive` always answers false. Until mediad sets the `privacy` layer itself when a viewer connects, nothing lights it.
- A mood source. Upstream's behaviour layer (`docs/ideas/autonomous_behavior.md` in `../microduck`) is where moods would come from; for now, set `mood` by hand or from a script.
- `identify` over BLE, before authentication, which is what `app-path-design.md` wants it for. Needs a route in `btd`.
- Colours and periods are constants in `eye/src/lib.rs`. Move them to config if they get tuned often.
