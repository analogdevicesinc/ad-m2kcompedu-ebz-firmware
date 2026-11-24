# AD-M2KCOMPEDU-EBZ Firmware

Arduino firmware for the AD-M2KCOMPEDU-EBZ, an educational add-on board for the ADALM2000. It has a signal generator (sine, square, triangle, DC) with adjustable amplitude, frequency, and DC offset, and a PWM generator with adjustable frequency and duty cycle. Both can also be controlled remotely over the USB port using [libiio](https://github.com/analogdevicesinc/libiio).

![Board](doc/board.jpg)

# Example Usage

Make sure that your board has the firmware uploaded, if not see Building and Flashing the board section

## Generating a wave

1. Connect the power supply in the USB-C power port of the board and move the Power Swich to the on possition.
2. Using the rotary encoder navigate to Signal Gen -> WaveForm (you can select DC, Sine, Square or Triangle). Then select the peak to peak amplitude frequency and offset

    ![Generate Wave](doc/usage/gen_sine_wave.gif)

3. You can view the resulting waveform by connecting a scope to any of the SGOUT test points or header. As a ground reference you can use any of the test points or headers labled as GND

    ![Generate Wave](doc/usage/sig_gen_tp.png)  
    ![WaveWave](doc/usage/sine_wave.png)

## Generating a PWM signal

1. Connect the power supply in the USB-C power port of the board and move the Power Swich to the on possition.
2. Using the rotary encoder navigate to PWN Gen -> PWN Out set to On. Then you can select Frequency and Duty Cycle

    ![Generate Wave](doc/usage/gen_pwm.gif)

3. You can view the resulting pwm signal by connecting a scope to any of the CLK labled test points or header. As a ground reference you can use any of the test points or headers labled as GND

    ![Generate Wave](doc/usage/pwn_gen_tp.png)   
    ![WaveWave](doc/usage/pwn_wave.png)

## Audio Capture

Pins A2 (pre-amp) and A3 (post-amp) are sampled at 8 kHz by a timer ISR using ping-pong double buffering. The raw samples are streamed over serial as binary frames so the Python demo can plot them in real time.

Audio streaming, IIO and CAN/SLCAN all share the same serial port, so only one can be active at a time — the firmware's bus arbiter handles the switching.

### Streaming with the Python demo

1. On the OLED, navigate to **Audio Monitor** and press the encoder — this starts the capture and streams binary frames over serial
2. Launch `python demos/ebz_demo.py`, open the **Audio Amplifier** tab, pick the COM port and hit **Connect**
3. You should see the pre-amp (and post-amp) waveforms updating live
4. The filter section lets you apply biquad filters (low-pass, high-pass, band-pass, etc.) and view the FFT
5. Long-press the encoder on the board to stop streaming and exit Audio Monitor

Binary frame format: `0xAA 0x55 <channels:1> <preAmp:256×int16> [postAmp:256×int16]`

### Reading audio levels via IIO

Once you exit Audio Monitor (long-press), serial switches back to IIO and you can query the last captured levels:

```
iio_attr -u serial:/dev/ttyACM0,921600,8n1 -c m2kcompedu-ebz -i voltage2 raw
```

You can also start/stop capture remotely without the OLED menu:

```
iio_attr -u serial:/dev/ttyACM0,921600,8n1 -c m2kcompedu-ebz -i voltage2 enable 1
```

## Controlling the board remotely

The firmware runs an IIOD server over the UART (accessible through the USB port of the Uno R4), so the board can also be controlled remotely with any [libiio](https://github.com/analogdevicesinc/libiio) client instead of the rotary encoder/menu. Connect using the serial backend, at 921600 baud:

```
iio_info -u serial:/dev/ttyACM0,921600,8n1
```

This exposes a single IIO device, `m2kcompedu-ebz`, with output and input channels:

- `altvoltage0` : the signal generator: `wavetype`, `amplitude`, `offset`, `frequency`
- `altvoltage1` : the PWM generator: `enable`, `duty_cycle`, `frequency`
- `voltage0`, `voltage1` : voltmeter inputs (A2, A3): `raw`, `scale`
- `voltage2`, `voltage3` : audio capture inputs (A2, A3): `raw`, `scale`, `enable`

`wavetype` selects the signal generator's waveform and is an index into `OFF (0)`, `DC (1)`, `Sine (2)`, `Square (3)`, `Triangle (4)` e.g. the `wavetype value: 2` in the sample below means the generator is producing a sine wave.

`amplitude` and `offset` are raw millivolt (mV) values, e.g. `amplitude value: 2000` is 2000 mV (2 Vpp), and `offset value: 0` is 0 mV.

```
IIO context has 1 devices:
        m2kcompedu-ebz: m2kcompedu-ebz
                2 channels found:
                        altvoltage0:  (output)
                        4 channel-specific attributes found:
                                attr  0: wavetype value: 2
                                attr  1: amplitude value: 2000
                                attr  2: offset value: 0
                                attr  3: frequency value: 1234.000
                        altvoltage1:  (output)
                        3 channel-specific attributes found:
                                attr  0: enable value: 1
                                attr  1: duty_cycle value: 85
                                attr  2: frequency value: 1235.000
```

Each attribute can be read or written directly with `iio_attr`, e.g. to set the signal generator frequency to 2 kHz:

```
iio_attr -u serial:/dev/ttyACM0,921600,8n1 -c m2kcompedu-ebz altvoltage0 frequency 2000
```


# Building and Flashing the board

## Arduino IDE

The Arduino IDE does not read the build profile, so libraries must be installed
manually via **Sketch > Include Library > Manage Libraries**. Install the exact
versions below for a build that matches the pinned toolchain:

| Library | Version | Purpose |
|---------|---------|---------|
| `Adafruit SH110X` | 2.1.14 | OLED display driver |
| `Adafruit GFX Library` | 1.12.6 | Graphics primitives (used by SH110X) |
| `Adafruit BusIO` | 1.17.4 | I2C/SPI abstraction (dependency of the Adafruit libraries) |
| `AD9833` | 0.4.5 | AD9833 DDS waveform generator driver |


Then:

1. Open `AD-M2KCOMPEDU-EBZ-firmware.ino` in the Arduino IDE

   ![Open the sketch](doc/ide_instructions/open_ino.png)

2. Install the libraries listed above. In the Library Manager, select the
   package (1), pick the exact version from the dropdown (2), and click
   **Install** (3):

   ![Selecting a library version](doc/ide_instructions/lib_version_select_example.png)

3. Install the **Arduino UNO R4 Boards** core, version **1.5.3**, via
   **Tools > Board > Boards Manager** (the version dropdown lets you pick 1.5.3),
   then select **Arduino UNO R4 WiFi** via **Tools > Board**

   ![Selecting the UNO R4 core version](doc/ide_instructions/uno_core_version_select.png)

4. Select the correct port via **Tools > Port**
5. Click **Upload** (or Ctrl+U)

## Arduino CLI

The repository includes a `sketch.yaml` build profile that pins the board core
and all library versions. This is the recommended path — it installs the exact
pinned versions into a sketch-local location and requires no manual library
setup.

From the root of the project directory:

```bash
# Compile using the pinned profile (installs the pinned core and libraries)
arduino-cli compile --profile unor4wifi AD-M2KCOMPEDU-EBZ-firmware.ino

# Upload to the connected board
arduino-cli upload --profile unor4wifi --port <port> AD-M2KCOMPEDU-EBZ-firmware.ino
```

Replace `<port>` with the serial port (e.g. `COM3` or `/dev/ttyUSB0`).


UART is configured at **921600 baud**.

## Flashing the Board

### Before Flashing

Before uploading the firmware, make sure the board is powered and connected:

- **Power supply connected** to the board
- **USB-C cable plugged into the Arduino**
- **Power switch in the ON position**

![Board ready for flashing](doc/flash_instructions/before_flash.jpg)

### After Flashing

Once the upload completes, the firmware boots automatically. You should see:

- The **menu appear on the OLED display**
- The **Analog Devices animation on the UNO R4 LED matrix** — the wedge logo is
  revealed, then the "Analog Devices" text scrolls across

![Board after flashing](doc/flash_instructions/after_flash.gif)

# Further information

## Signal Chain

```
AD9833 (DDS)  -->  AD5443 (MDAC)  -->  OUT_SG
                                        ^
                                        |
                             AD5625R (DC Offset DAC)
```

- **AD9833** generates the base waveform at the set frequency
- **AD5443** MDAC attenuates the AD9833 output by `Data / 4096` (does not generate voltage on its own)
- **AD5625R** generates DC offset via differential pair: `Vout ~ (DAC_A - DAC_B)`

### Amplitude Conversion (V to LSB)

| Waveform | Gain Constant | Example |
|----------|--------------|---------|
| Sine / Triangle | `SinGain_VperLsb = 0.0053311` V/LSB | 1 Vpp = 188 LSB, 5 Vpp = 938 LSB |
| Square | `SqwGain_VperLsb = 0.02914` V/LSB | 1 Vpp = 34 LSB, 5 Vpp = 172 LSB |

### DC Offset Computation

```
DC_LSB = round(DC_voltage / DcOffperLSB) + LSBOFF
DAC_A  = 2048 + DC_LSB / 2
DAC_B  = 2048 - DC_LSB / 2
```

Where `DcOffperLSB = 0.0031640` V/LSB and `LSBOFF` is the calibration correction from EEPROM.

## Software Architecture

The firmware is an Arduino sketch following a **Model-View-Controller** pattern:

```
AD-M2KCOMPEDU-EBZ-firmware.ino                   Entry point (setup + 1ms loop)
src/
  app/
    Controller.cpp/.h       FSM controller, menu tree, state management
    Menu.cpp/.h             Menu rendering with custom markup formatting
    Peripherals.cpp/.h      Hardware abstraction, refreshFlags-based updates
    CalibrationWizard.cpp/.h  Multi-step interactive calibration
    Audio.cpp/.h            Two-channel ADC capture with ping-pong ISR buffering
    Messenger.cpp/.h        Shared state and flags between app modules
  drivers/
    AD5443.cpp/.h           SPI driver for amplitude MDAC
    AD5625R.cpp/.h          I2C driver for DC offset quad-DAC
    Display.cpp/.h          SH1107 OLED driver (via Adafruit_SH110X)
    Encoder.cpp/.h          Rotary encoder with debounce and event detection
    LedMatrix.cpp/.h        UNO R4 12x8 LED matrix startup/idle animation
  iio/
    iiod.c, iiod.h,
    iiod_private.h          Symlinks into external/no-OS/iio/ (vendored no-OS source, never edited in place)
    iiod_cxx.h              extern "C" wrapper so .cpp files can include iiod.h (compiled as C)
    iio.h, no_os_error.h,
    no_os_util.h            Minimal shims satisfying iiod.c's own includes, without vendoring all of no-OS
    iio_server.cpp          glue code connecting the IIOD server to the adjustable parameters
external/
  no-OS/iio/                Unmodified copy of no-OS's iio/ directory, kept in sync by a scheduled GitHub Action
```

### Vendored no-OS IIOD Server

The IIOD server (`iiod.c`, `iiod.h`, `iiod_private.h`) isn't maintained in this
repo — it's vendored unmodified from the `iio/` directory of
[analogdevicesinc/no-OS](https://github.com/analogdevicesinc/no-OS). The real
source lives in `external/no-OS/iio/`, kept current by a scheduled GitHub
Action; `src/iio/iiod.c`, `iiod.h`, and `iiod_private.h` are symlinks into
that directory, so the vendored code is never edited in place.

Since this board only needs the IIOD protocol/server (not no-OS's drivers,
network backends, etc.), a few minimal shim headers live alongside the
symlinks in `src/iio/` to satisfy `iiod.c`'s own includes without vendoring
all of no-OS:

- `iio.h` — just the `physical_link_type` enum
- `no_os_error.h` — `NO_OS_IS_ERR_VALUE`
- `no_os_util.h` — `no_os_min` and `NO_OS_ARRAY_SIZE`

`iiod_cxx.h` is a separate `extern "C"` wrapper: `iiod.c` compiles as C, but
`iio_server.cpp` is C++, so C++ translation units should include
`iiod_cxx.h` instead of `iiod.h` directly to get correct linkage.

### Main Loop

The main loop runs on a 1 ms tick. Each iteration:

1. Polls the rotary encoder for events
2. Dispatches events to either the CalibrationWizard (if active) or the Controller FSM
3. Renders the menu to the OLED display
4. Pushes changed parameters to hardware (only dirty channels via `refreshFlags[]`)

Independently of the 1 ms tick, every loop iteration also calls `iio_server_poll()`,
which steps the IIOD server (`iiod_conn_step()`) to service remote IIO client
requests over UART — reading/writing the same `Param`/`refreshFlags[]` state used
by the Controller FSM.

![Main Loop Activity Diagram](doc/call_graph.png)

### UI State Machine

Three top-level states:
- **Navigating** - Browse menus (Root / Submenu), cursor moves with encoder turns
- **Editing** - Adjust a parameter value (single param or multi-field value group)
- **Calibrating** - Multi-step wizard for offset and amplitude calibration

![UI State Machine](doc/state_diagram.png)

Encoder input mapping:
- **Rotate** - Navigate between menu items (in Navigating) or adjust values (in Editing/Calibrating)
- **Short press** - Enter a sub-menu, enter editing mode, or advance calibration step
- **Long press** - Return to root menu from any state

### Menu Structure

```
Main
 |-- Signal Gen
 |     |-- WaveForm:   OFF / DC / Sine / Square / Triangle
 |     |-- Vampl-pp:   0.0 - 5.0 V (step 0.1 V)
 |     |-- Frequency:  0.1 Hz - 2 MHz (value + order: Hz/KHz/MHz)
 |     |-- DC Offset:  -2.5 V - +2.5 V (step 0.1 V)
 |-- PWM Gen
 |     |-- PWM Out:    ON / OFF
 |     |-- Frequency:  value + order
 |     |-- Duty Cycle: 0 - 100%
 |-- Audio Monitor          (live signal level on OLED, long-press to exit)
 |-- Calibrate
       |-- Calibrate Offset   (multi-step wizard)
       |-- Calibrate Sine Amp (2 steps: 1 Vpp and 10 Vpp)
       |-- Calibrate SQW Amp  (2 steps: 1 Vpp and 10 Vpp)
```

## Calibration System

Three procedures correct for component tolerances. Each is a step-by-step wizard: press advances steps, rotate adjusts values. Results are stored in EEPROM at address `0x10`.

### Offset Calibration

Corrects the DC offset zero-point by adjusting `LSBOFF`.

1. **Press** to start: firmware forces DC mode with DC = 0.0 V. OLED displays: "Connect Voltmeter to Sig Out, Rotate until Vout ~ 0, then Press"
2. **Rotate** to adjust `LSBOFF` until the voltmeter reads 0 V
3. **Press** to store: new `LSBOFF` value saved to EEPROM

### Sine Amplitude Calibration

Corrects sine amplitude gain at two reference points:

1. **Press** to start: firmware sets Sine, 1 KHz, 1 Vpp, DC = 0 V
2. **Rotate** to adjust `LsbCal_SineAmp1V` until scope reads exactly 1 Vpp
3. **Press** to advance: amplitude changes to target 10 Vpp
4. **Rotate** to adjust `LsbCal_SineAmp10V` until scope reads exactly 10 Vpp
5. **Press** to store

### Square Wave Amplitude Calibration

Same procedure as sine calibration, but uses `SqwGain_VperLsb` and adjusts `LsbCal_SqWAmp1V` / `LsbCal_SqWAmp10V`.
