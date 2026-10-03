# Microduck Unitree: expression outputs (eye LED and fan)

How to bring up the robot's two expressive outputs and the `expressd` daemon that drives them: one WS2812-type RGB LED in the head (the eye), and the head fan, whose first job is cooling the compute module and whose second is expression. This is Dukki's addition; Pollen's robot has neither. Follows on from `compute-setup.md`.

Date: 03/10/2026. Written against vendor kernel 6.1.115-vendor-rk35xx, board device tree `rk3576-radxa-cm4-io.dtb`. Verified the same day:
- Eye: the test script shows red, green, blue and white in the right order with the data line driven straight from the 3.3 V header pin. `expressd` follows robotd's state, takes mood and identify requests, and falls back to "asleep" when robotd stops.
- Fan: the PWM output on header pin 11 was verified with an LED and 1 kΩ resistor standing in for the MOSFET (the MOSFET is not fitted yet). It held 100 %, ran a burst pattern, and settled at the cooling floor.

## 1. The short version

1. Wire with the board powered off.

The eye breakout's input side (its output side, DO/5V/GND, is for chaining a second LED and stays unconnected):

| Eye breakout | NANO-A header pin | Signal |
|---|---|---|
| 5V | 2 or 4 | 5 V from the head buck |
| GND | 20 or 25, or any ground | ground |
| DI | 19 | SPI1_MOSI_M0 |

The fan, through a logic-level N-channel MOSFET on its ground side (section 3):

| From | To |
|---|---|
| header pin 11 | 100 Ω, then the MOSFET gate |
| MOSFET gate | 100 kΩ to ground |
| MOSFET source | ground, header pin 9 |
| MOSFET drain | fan negative (black) |
| fan positive (red) | 5 V, header pin 2 or 4 |
| diode (1N4148 or 1N5817) | across the fan, stripe to the 5 V side |

2. From the Mac, install the overlays and udev rules, then reboot:

```
scp -r scripts/compute/setup-express.sh scripts/compute/express duck@microduck.local:
ssh -t duck@microduck.local sudo bash setup-express.sh
ssh -t duck@microduck.local sudo reboot
```

3. After the reboot, check the nodes and the LED:

```
ssh -t duck@microduck.local sudo bash setup-express.sh --check     # /dev/spidev-eye, the fan pwmchip
ssh duck@microduck.local python3 express/eye-test.py               # red, green, blue, white, off
```

4. Install the daemon from the Mac: `scripts/compute/push-daemons.sh expressd`.

## 2. The eye

The breakout is a generic red board sold as "WS2811 Breakout": a 5050 LED with the controller built in (nothing on the back), so it behaves as a WS2812, colour order GRB. It runs at 5 V and nominally wants 0.7 x VDD = 3.5 V for a logic high, which the RK3576's 3.3 V pins do not reach. This one works driven directly. If a replacement flickers or shows the wrong colours, put a diode (1N4148 or 1N5817) in series with the breakout's 5V: the LED then runs at about 4.3 V and 3.3 V clears its threshold. A 74AHCT1G125 buffer powered from 5 V is the proper fix. DI is an input, so nothing at 5 V reaches the SoC either way. Full white draws about 60 mA.

Keep the LED out of the camera's view and behind a diffuser; glare upsets the IMX219's auto exposure.

Why SPI: the LED's single-wire protocol is a pulse train at 800 kHz with about 150 ns of tolerance, which Linux cannot bit-bang from a GPIO. The STM32 bridge could, but it is in the trunk and would need another wire through the neck. Clocked out of SPI MOSI at 2.4 MHz, three SPI bits make one LED bit (100 for a 0, 110 for a 1), giving highs of 0.42 and 0.83 µs against the WS2812's 0.4 and 0.8 µs. Each frame is 9 data bytes with 96 zero bytes either side for the reset gap, sent as one `write()`, so the controller holds the timing.

The `microduck-nano-a-spi1` overlay enables SPI1 M0 with a `rockchip,spidev` device on CS0. Its pin group claims header pins 19 (MOSI), 21 (MISO) and 23 (SCLK); the chip-select pins 24 and 26 stay free. `setup-express.sh` installs it as an Armbian user overlay and adds a udev rule that names the node `/dev/spidev-eye` (matched by the controller's address, `2ad00000.spi`) and gives it to the `robot` group.

## 3. The fan

A 2-wire 5 V axial fan (5 V and ground only, no tachometer), visible on the robot. Its speed is the duty of a hardware PWM channel switching the fan's ground through a MOSFET.

Which pin. The fan needs a header pin with a hardware PWM function: software PWM from Linux is jittery and audible. Header pin 11 carries GPIO1_C2, whose alternate function is PWM1 channel 2. **Radxa's device tree labels its header pins wrongly for at least this pair**: it calls GPIO1_C2 `PIN_33` and GPIO1_C4 `PIN_11`, but on this module and carrier they are the other way round. Measured 03/10/2026: GPIO1_C2 driven high left header pin 33 at 0 V, and a resistor to ground on header pin 11 pulled GPIO1_C2 low. `PIN_7` is right (the first on/off test used it). Measure any other header pin before relying on its label. Header pin 33 (GPIO1_C4) has no PWM function.

**Do not scan GPIO lines in bulk to find a pin.** Requesting a line on the RK3576, even read-only as an input, switches that pin to GPIO whatever else owns it. A scan of every line with no GPIO consumer remuxed the eMMC's pins and the root filesystem went read-only until a power cycle (a forced `fsck` afterwards found no damage). The safe search reads only lines that are named after a header pin and show `(MUX UNCLAIMED)` in `/sys/kernel/debug/pinctrl/pinctrl-rockchip-pinctrl/pinmux-pins` (pin number = bank x 32 + line). Hold the header pin to ground, then to 3.3 V, through 1 kΩ, and see which line follows.

The MOSFET. A logic-level N-channel part that is fully on at 3.3 V gate drive, such as an AO3400 or IRLML6344; a 2N7000 is marginal at 3.3 V. The 100 Ω gate resistor limits the pin's switching current at 25 kHz; the 100 kΩ pull-down keeps the fan off while the pin is not driven. The diode across the fan catches the motor's back-EMF when the MOSFET switches off.

The PWM. The `microduck-nano-a-pwm-fan` overlay enables `pwm1_6ch_2` (controller `2add2000.pwm`) with the `pwm1m1_ch2` pin state, which the vendor driver names `active`. `setup-express.sh` installs it and a udev rule that gives the channel's sysfs files to the `robot` group, on `change` as well as `add` because exporting a channel announces its new directory as a change on the chip. The vendor driver has two quirks that `expressd` works around: a fresh channel has a period of 0 and refuses every write, even `enable`, until it has one; and its polarity starts `inversed` and can only change while disabled.

Speed control on a 2-wire fan. Below some duty the fan's own electronics brown out and it stalls, so running duties are kept at or above `--fan-min` (default 30 %), and a start from rest runs at full for half a second first. The default frequency is 25 kHz, above hearing. Some 2-wire fans behave better switched slowly, around 100 Hz (`--fan-hz 100`): try that if the real fan whines, or does not slow down as the duty falls. Find its real stall point and set `--fan-min` to just above it.

## 4. The daemon

`expressd` is the crate `express` on the `dukki` branch of `../microduck`. It touches no upstream crate: its methods are its own, not added to `duck-ipc-proto`. It runs as user `expressd` with the `robot` group.

What each output shows is decided by a stack of layers. The highest one that is set wins.

| Layer | Set by | Eye | Fan |
|---|---|---|---|
| privacy | a client | whatever the client asks; meant for "the camera is being watched", and nothing below can hide it | whatever the client asks |
| fault | robotd's state | fallen: red, blinking at 2 Hz | not used |
| identify | `expressd identify` | white, fast blink, for 10 s by default | not used |
| mood | a client | whatever the client asks | whatever the client asks |
| ambient | robotd's state | see below | not used |

The eye's ambient look follows `robot.state`, subscribed at 10 Hz:

| State | Look |
|---|---|
| no robotd, or no frame for 2 s | asleep: dim white, slow breathe (4 s) |
| limp (torque off) | violet, breathe (3 s) |
| running, no policy (`held`) | blue, breathe (3 s) |
| stand policy, or walk with nothing commanded | blue, steady |
| walking | cyan, steady |

Clients cannot set or clear `fault` or `ambient`; that would let them hide a fall or fake walking. Eye brightness is capped at 15 % (`--brightness`) and applied through a gamma curve.

The fan has a floor that no layer can go below: cooling. `expressd` reads the hottest thermal zone every 2 s. The fan is off until the SoC reaches 60 °C (`--fan-on-c`), then runs from the minimum speed at 60 °C up to full at 75 °C (`--fan-full-c`), and stops again only below 50 °C (`--fan-off-c`), so a SoC sitting near a threshold does not cycle it. A layer's fan request is applied above that floor. The fan fails on: an unreadable temperature runs it full, and so does `expressd` stopping. Between the PWM driver claiming the pin at boot and `expressd` starting, the fan is not driven; that is a few seconds, and the SoC's own throttling covers it. `--no-fan` leaves the PWM alone; a fan PWM that cannot be opened is logged and the eye runs without it.

Both outputs animate at about 30 Hz while either is animated; a steady state is written on change and refreshed once a second. On stop the eye turns off and the fan goes to full.

From a shell on the robot (any user in the `robot` group):

```
expressd status
expressd set mood --eye 0,255,0 --pattern breathe --period 3
expressd set mood --eye 255,120,0 --pattern pulse --fan 0.8 --fan-pattern blink --fan-period 4 --ttl 30
expressd set mood --fan 0.6
expressd clear mood
expressd identify --seconds 5
```

Patterns, for both outputs: `solid`, `breathe`, `blink`, `pulse`. On the fan, `breathe` swells and settles, `blink` is bursts and `pulse` is a short puff each period; the fan's inertia smooths periods under a couple of seconds into a lower steady speed. `expressd` is in `/opt/robot/daemon/current/bin/`.

Over the socket, `/run/expressd/express.sock`, one JSON-RPC 2.0 request per line, as many per connection as a client likes. `express.set` takes `eye`, `fan` or both:

```
{"jsonrpc":"2.0","id":1,"method":"express.set","params":{"layer":"mood","eye":{"colour":[0,255,0],"pattern":"breathe","period_s":3.0},"fan":{"level":0.6,"pattern":"blink","period_s":4.0},"ttl_s":30}}
{"jsonrpc":"2.0","id":2,"method":"express.clear","params":{"layer":"mood"}}
{"jsonrpc":"2.0","id":3,"method":"express.identify","params":{"seconds":5}}
{"jsonrpc":"2.0","id":4,"method":"express.status"}
```

`express.status` reports, for the fan, the duty being driven, the cooling floor and the temperature.

## 5. Open items

- The fan MOSFET is not fitted yet. When it is: confirm the fan starts from rest after the kick, find its stall duty and set `--fan-min`, and choose `--fan-hz`.
- The fan's state at boot, before `expressd` runs, has not been measured. The 100 kΩ gate pull-down should hold it off.
- The camera indicator. mediad has no call that says whether anyone is watching, and robotd's `robot.remoteSessionActive` always answers false. Until mediad sets the `privacy` layer itself when a viewer connects, nothing lights it.
- A mood source. Upstream's behaviour layer (`docs/ideas/autonomous_behavior.md` in `../microduck`) is where moods would come from; for now, set `mood` by hand or from a script.
- `identify` over BLE, before authentication, which is what `app-path-design.md` wants it for. Needs a route in `btd`.
- Colours, periods and the fan curve are constants or flags in `express/src/`. Move them to config if they get tuned often.
