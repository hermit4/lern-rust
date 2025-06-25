#![no_std]
#![no_main]

use bsp::entry;
use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    fugit::RateExtU32,
    pac,
    sio::Sio,
    uart::{DataBits, StopBits, UartConfig},
    watchdog::Watchdog,
    I2C,
};
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::InputPin;
use embedded_hal::digital::OutputPin;
use embedded_hal::i2c::I2c;
use panic_probe as _;
use rp_pico as bsp;

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let _core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let sda = pins
        .gpio6
        .into_pull_up_input()
        .into_function::<bsp::hal::gpio::FunctionI2C>();
    let scl = pins
        .gpio7
        .into_pull_up_input()
        .into_function::<bsp::hal::gpio::FunctionI2C>();

    let mut i2c = I2C::i2c1(
        pac.I2C1,
        sda,
        scl,
        100_000.Hz(),
        &mut pac.RESETS,
        &clocks.peripheral_clock,
    );
    let mut rst = pins.gpio22.into_push_pull_output();
    rst.set_high().unwrap();

    let mut irq = pins.gpio21.into_pull_up_input();

    let uart_pins = (pins.gpio0.into_function(), pins.gpio1.into_function());
    let uart = bsp::hal::uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(9600.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();
    uart.write_full_blocking(b"CST816S investigation\n");

    rst.set_low().unwrap();
    cortex_m::asm::delay(12_000_000);
    rst.set_high().unwrap();
    cortex_m::asm::delay(12_000_000);

    let mut buf = [0u8; 1];
    let status = i2c.write_read(0x15u8, &[0xA7], &mut buf);
    match status {
        Ok(_) => {
            defmt::info!("CST816S version: 0x{:02X}", buf[0]);
        }
        Err(_e) => {
            defmt::warn!("I2C error");
        }
    }

    loop {
        if irq.is_low().unwrap() {
            let mut touch_data = [0u8; 6];
            if i2c.write_read(0x15u8, &[0x01], &mut touch_data).is_ok() {
                let gesture = touch_data[0];
                let x = ((touch_data[2] as u16 & 0x0F) << 8) | touch_data[3] as u16;
                let y = ((touch_data[4] as u16 & 0x0F) << 8) | touch_data[5] as u16;
                defmt::info!("Gesture: {}, X: {}, Y: {}", gesture, x, y);
            }
        }
    }
}
