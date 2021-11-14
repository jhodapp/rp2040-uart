//! # UART Echo Server Example
//!
//! This application demonstrates how to use the UART Driver to talk to a serial
//! connection, echoing all key presses back to the host machine.
//!
//! Based on the UART example from the rp-rs HAL project.
//!
//! To run: cargo run
//! See the following series on how to use OpenOCD, gdb and two Pico boards to
//! run this example:
//! https://reltech.substack.com/p/getting-started-with-rust-on-a-raspberry
//!
//! See the `Cargo.toml` file for Copyright and licence details.

#![no_std]
#![no_main]

// The macro for our start-up function
use cortex_m_rt::entry;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Alias for our HAL crate
use rp2040_hal as hal;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use hal::pac;

// Some traits we need
use core::fmt::Write;
use cortex_m::prelude::_embedded_hal_serial_Read;
use cortex_m::prelude::_embedded_hal_serial_Write;

/// The linker will place this boot block at the start of our program image. We
// need this to help the ROM bootloader get our code up and running.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then writes to the UART in
/// an inifinite loop.
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::watchdog::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::sio::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut uart = hal::uart::UartPeripheral::<_, _>::enable(
        pac.UART0,
        &mut pac.RESETS,
        hal::uart::common_configs::_115200_8_N_1,
        clocks.peripheral_clock.into(),
    )
    .unwrap();

    // UART TX (characters sent from RP2040) on pin 1 (GPIO0)
    let _tx_pin = pins.gpio0.into_mode::<hal::gpio::FunctionUart>();
    // UART RX (characters reveived by RP2040) on pin 2 (GPIO1)
    let _rx_pin = pins.gpio1.into_mode::<hal::gpio::FunctionUart>();

    uart.write_full_blocking(b"UART echo server example\r\n");

    loop {
        // Read a single byte from the UART at a time
        let res = uart.read();
        match res {
            Ok(byte) => {
                match byte {
                    // If we press enter (carriage return), make sure the output
                    // moves to a new line
                    0xD => { uart.write_full_blocking(b"\r\n"); },
                    // If we press any other key, transmit it back to the host
                    _ => { 
                        // We succeeded, write the read buffer back out (echo)
                        let _x = uart.write(byte);
                        writeln!(uart, " (ascii: {:?})\r", &byte).ok().unwrap();
                        continue;
                    },
                }
            }
            Err(_) => continue,
        }
    }
}

// End of file