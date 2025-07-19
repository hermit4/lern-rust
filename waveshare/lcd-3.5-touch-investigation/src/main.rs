#![no_std]
#![no_main]

#[link_section = ".boot2"]
#[used]
static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

use crate::pac::interrupt;
use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, Ordering};
use cortex_m::interrupt::Mutex;
use defmt_rtt as _;
use embedded_hal::{
    digital::{InputPin, OutputPin},
    spi::SpiBus,
};
use hal::{
    clocks::{init_clocks_and_plls, Clock},
    entry,
    fugit::{Hertz, RateExtU32},
    gpio::{bank0::*, Interrupt as GpioInterrupt, Pin, PullUp},
    pac,
    sio::Sio,
    watchdog::Watchdog,
    Spi, Timer,
};
use panic_probe as _;
use rp2040_hal as hal;

const SPI_ILI9488_MAX_FREQ: Hertz<u32> = Hertz::<u32>::Hz(33_300_000);
const SCREEN_HEIGHT: usize = 480;
const BAND_HEIGHT: usize = 96;

pub struct ILI9488Interface<SPI, CS, DC> {
    spi: SPI,
    cs: CS,
    dc: DC,
}

impl<SPI, CS, DC> ILI9488Interface<SPI, CS, DC>
where
    SPI: SpiBus,
    CS: OutputPin,
    DC: OutputPin,
{
    pub fn new(spi: SPI, cs: CS, dc: DC) -> Self {
        Self { spi, cs, dc }
    }
    pub fn write_command(&mut self, cmd: u8) {
        self.cs.set_low().ok();
        self.dc.set_low().ok();
        self.spi.write(&[cmd]).ok();
        self.cs.set_high().ok();
    }

    pub fn write_data(&mut self, data: &[u8]) {
        self.cs.set_low().ok();
        self.dc.set_high().ok();
        self.spi.write(data).ok();
        self.cs.set_high().ok();
    }
}

fn redraw_screen<SPI, CS, DC>(
    ili9488: &mut ILI9488Interface<SPI, CS, DC>,
    timer: &mut Timer,
    offset: usize,
    test_colors: &[u16],
    pixel_data: &mut [u8],
) where
    SPI: SpiBus,
    CS: OutputPin,
    DC: OutputPin,
{
    for y in 0..SCREEN_HEIGHT {
        let logical_y = (offset + y) % SCREEN_HEIGHT;
        let band = logical_y / BAND_HEIGHT;
        let color = test_colors[band];
        let hi = !(color >> 8) as u8;
        let lo = !(color & 0xFF) as u8;
        for px in pixel_data.chunks_exact_mut(2) {
            px[0] = hi;
            px[1] = lo;
        }
        ili9488.write_data(pixel_data);
    }
}

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
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
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let lcd_dc = pins.gpio8.into_push_pull_output();
    let lcd_cs = pins.gpio9.into_push_pull_output();
    let lcd_clk = pins.gpio10.into_function::<hal::gpio::FunctionSpi>();
    let mosi = pins.gpio11.into_function::<hal::gpio::FunctionSpi>();
    let miso = pins.gpio12.into_function::<hal::gpio::FunctionSpi>();
    let mut lcd_bl = pins.gpio13.into_push_pull_output();
    let mut lcd_rst = pins.gpio15.into_push_pull_output();
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    lcd_rst.set_low().unwrap();
    delay.delay_ms(100);
    lcd_rst.set_high().unwrap();
    delay.delay_ms(200);

    let spi = Spi::<_, _, _, 8>::new(pac.SPI1, (mosi, lcd_clk)).init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        SPI_ILI9488_MAX_FREQ,
        embedded_hal::spi::MODE_3,
    );
    let mut ili9488 = ILI9488Interface::new(spi, lcd_cs, lcd_dc);

    ili9488.write_command(0x01); // software reset
    delay.delay_ms(100);
    ili9488.write_command(0x11); // sleep out
    delay.delay_ms(100);
    ili9488.write_command(0x3A); // pixel format
    ili9488.write_data(&[0x55]); // RGB565
    ili9488.write_command(0x36); // MADCTL
    ili9488.write_data(&[0x00]); // BGR, no rotet
    delay.delay_ms(100);
    ili9488.write_command(0x29); // Display on
    delay.delay_ms(100);

    ili9488.write_command(0x2A); // Column
    ili9488.write_data(&[0x00, 0x00, 0x00, 0xEF]);
    ili9488.write_command(0x2B); // Row
    ili9488.write_data(&[0x00, 0x14, 0x01, 0x2B]);
    ili9488.write_command(0x2C);
    delay.delay_ms(100);

    let test_colors = [
        0xF800, // Red
        0x07E0, // Green
        0x001F, // Blue
        0xFFFF, // White
        0x0000, // Black
    ];
    let mut pixel_data = [0u8; 320 * 2];
    let mut prev_y: u16 = (SCREEN_HEIGHT + 1) as u16;
    let mut scroll_offset = 0;
    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    redraw_screen(
        &mut ili9488,
        &mut timer,
        scroll_offset,
        &test_colors,
        &mut pixel_data,
    );
    lcd_bl.set_high().unwrap();
    loop {
        cortex_m::asm::wfe();
    }
}
