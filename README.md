# pico-bno055-rs

Have a [Raspberry Pi Pico](https://www.raspberrypi.com/products/raspberry-pi-pico/) get its realtime orientation data from a Bosch BNO055 [Inertial Measurement Unit](https://en.wikipedia.org/wiki/Inertial_measurement_unit) and write the to the host computer via serial USB connection.

## Get Started

1. Install the latest version of Rust and the `thumbv6m-none-eabi` target. This is the primary language we use to write programs for the Pico

```
https://www.rust-lang.org/tools/install
rustup self update
rustup update stable
rustup target add thumbv6m-none-eabi
```

2. Install the Rust port of ELF2UF2. This is used for flashing the program to the board

```
cargo install elf2uf2-rs
```

3. Install [Clippy](https://rust-lang.github.io/rust-clippy/). This is used to help lint the Rust code

```
rustup component add clippy
```

4. Install [just](https://github.com/casey/just). This is used for running certain tasks, (`just build` will build the program, for example)

```
cargo install just
```

5. Build the project to make sure everything's setup

```
just build
```

And you're good to go! If you want to do some debugging as well, do the following _optional_ steps.

6. Install [Python](https://www.python.org/)

7. Install the [pySerial](https://pyserial.readthedocs.io/en/latest/index.html) library

## Flashing the program to the Pico

To flash the program to the Pico:

1. Plug in the Pico to your host machine in bootloader mode (with the on-board button held down)
2. Run `just flash`

## View [Quaternion](https://en.wikipedia.org/wiki/Quaternion) Data Live

1. With the program flashed to the Pico, plug in the Pico via USB to the host computer
2. Locate the serial port on the host computer (COM... on Windows, /dev/tty... on POSIX)
3. Edit `debug-quaternion.py` to use the serial port to which the Pico is connected
4. Run `python scripts/debug-quaternion.py`

## Additional Resources

- The [rp-pico](https://github.com/rp-rs/rp-hal/tree/main/boards/rp-pico) crate
- The [rp-hal](https://github.com/rp-rs/rp-hal) crate
- The [bno055](https://github.com/eupn/bno055) crate
