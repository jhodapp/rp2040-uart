# A Rust-based UART echo server for the Raspberry Pi Pico

## To Run
`cargo run`

For more details see the following article on getting started:
https://reltech.substack.com/p/getting-started-with-rust-on-a-raspberry

On a Mac, to run minicom: `minicom -D /dev/tty.usbmodem14201 -b 115200`. Not that you'll most likely
need to find the current /dev link assigned to the Pico UART for your particular machine.

## Requirements
- The standard Rust tooling (cargo, rustup) which you can install from https://rustup.rs/

- Toolchain support for the cortex-m0+ processors in the rp2040 (thumbv6m-none-eabi)

- flip-link - this allows you to detect stack-overflows on the first core, which is the only supported target for now.

- probe-run. Upstream support for RP2040 is not finished yet, so this template uses `probe-run-rp` for now.
  `probe-run-rp` is a version of `probe-run` using a `probe-rs` fork with support for the RP2040 chip.
  Note that this installs the binary with name `probe-run-rp`, so you can still have the original `probe-run` installed in parallel.

  This is important because `probe-run-rp` ONLY works with the RP2040 chip.

- A CMSIS-DAP probe. (JLink probes sort of work but are very unstable. Other probes won't work at all)

  You can use a second Pico as a CMSIS-DAP debug probe by installing the following firmware on it:
  https://github.com/majbthrd/DapperMime/releases/download/20210225/raspberry_pi_pico-DapperMime.uf2

  More details on supported debug probes can be found in [debug_probes.md](debug_probes.md)

## Installation of development dependencies
```
rustup target install thumbv6m-none-eabi
cargo install --git https://github.com/rp-rs/probe-run --branch rp2040-support
cargo install flip-link
```

## Running

For a debug build
```
cargo run
```
For a release build
```
cargo run --release
```
  
## License

This project is licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  http://www.apache.org/licenses/LICENSE-2.0)

- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.
