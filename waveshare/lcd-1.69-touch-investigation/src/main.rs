#![no_std]
#![no_main]

use crate::pac::interrupt;
use bsp::entry;
use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    fugit::RateExtU32,
    gpio::{self, Interrupt as GpioInterrupt},
    pac,
    sio::Sio,
    uart::{DataBits, StopBits, UartConfig},
    watchdog::Watchdog,
    I2C,
};
use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, Ordering};
use cortex_m::interrupt::Mutex;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::i2c::I2c;
use panic_probe as _;
use rp_pico as bsp;

type IrqPin = gpio::Pin<gpio::bank0::Gpio21, gpio::FunctionSio<gpio::SioInput>, gpio::PullUp>;
static IRQ_PIN: Mutex<RefCell<Option<IrqPin>>> = Mutex::new(RefCell::new(None));
static TOUCH_IRQ_PENDING: AtomicBool = AtomicBool::new(false);

const CST816S_ADDR: u8 = 0x15;
const REG_VERSION: u8 = 0xA7;
const REG_DATA: u8 = 0x01;

#[entry]
fn main() -> ! {
    info!("Program start");

    let mut pac = pac::Peripherals::take().expect("PAC not available");
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

    let irq = pins.gpio21.into_pull_up_input();

    cortex_m::interrupt::free(|cs| {
        IRQ_PIN.borrow(cs).replace(Some(irq));
        IRQ_PIN
            .borrow(cs)
            .borrow()
            .as_ref()
            .unwrap()
            .set_interrupt_enabled(GpioInterrupt::LevelLow, true);
    });

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
    let status = i2c.write_read(CST816S_ADDR, &[REG_VERSION], &mut buf);
    match status {
        Ok(_) => {
            defmt::info!("CST816S version: 0x{:02X}", buf[0]);
        }
        Err(e) => {
            defmt::warn!("I2C error: {:?}", defmt::Debug2Format(&e));
        }
    }

    unsafe {
        pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
    }

    loop {
        cortex_m::interrupt::free(|cs| {
            if TOUCH_IRQ_PENDING.load(Ordering::Acquire) {
                TOUCH_IRQ_PENDING.store(false, Ordering::Release);
                let mut touch_data = [0u8; 6];
                if i2c
                    .write_read(CST816S_ADDR, &[REG_DATA], &mut touch_data)
                    .is_ok()
                {
                    let gesture = touch_data[0];
                    let x = ((touch_data[2] as u16 & 0x0F) << 8) | touch_data[3] as u16;
                    let y = ((touch_data[4] as u16 & 0x0F) << 8) | touch_data[5] as u16;
                    defmt::info!("Gesture: {}, X: {}, Y: {}", gesture, x, y);
                }
                IRQ_PIN
                    .borrow(cs)
                    .borrow()
                    .as_ref()
                    .unwrap()
                    .set_interrupt_enabled(GpioInterrupt::LevelLow, true);
            }
        });
        cortex_m::asm::wfe();
    }
}

#[interrupt]
fn IO_IRQ_BANK0() {
    cortex_m::interrupt::free(|cs| {
        let mut pin = IRQ_PIN.borrow(cs).borrow_mut();
        let pin = pin.as_mut().unwrap();
        if pin.is_low().unwrap_or(false) {
            TOUCH_IRQ_PENDING.store(true, Ordering::Release);
        }
        pin.set_interrupt_enabled(GpioInterrupt::LevelLow, false);
        pin.clear_interrupt(GpioInterrupt::LevelLow);
    });
}
