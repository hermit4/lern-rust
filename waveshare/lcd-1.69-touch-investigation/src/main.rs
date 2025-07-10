#![no_std]
#![no_main]

#[link_section = ".boot2"]
#[used]
static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

extern crate alloc;
mod display;
use crate::display::{DisplayRotation, St7789Interface};
use crate::pac::interrupt;
use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, AtomicU16, Ordering};
use cortex_m::interrupt::Mutex;
use defmt::*;
use defmt_rtt as _;
use embedded_alloc::LlffHeap as Heap;
use embedded_hal::{
    digital::{InputPin, OutputPin},
    i2c::I2c,
    spi::SpiBus,
};
use hal::multicore::{Multicore, Stack};
use hal::{
    clocks::{init_clocks_and_plls, Clock},
    dma::{ChannelIndex, DMAExt},
    entry,
    fugit::{Hertz, RateExtU32},
    gpio::{bank0::*, FunctionI2C, Interrupt as GpioInterrupt, Pin, PullUp},
    pac,
    sio::Sio,
    watchdog::Watchdog,
    Spi, Timer, I2C,
};
use panic_probe as _;
use rp2040_hal as hal;

#[global_allocator]
static ALLOCATOR: Heap = Heap::empty();

const HEAP_SIZE: usize = 200 * 1024;
const SPI_ST7789VW_MAX_FREQ: Hertz<u32> = Hertz::<u32>::Hz(33_300_000);
const CST816S_ADDR: u8 = 0x15;
const REG_VERSION: u8 = 0xA7;
const REG_DATA: u8 = 0x01;
const BAND_HEIGHT: usize = 56;

type IrqPin = Pin<Gpio21, hal::gpio::FunctionSio<hal::gpio::SioInput>, PullUp>;
type SharedI2C = I2C<
    pac::I2C1,
    (
        Pin<Gpio6, FunctionI2C, PullUp>,
        Pin<Gpio7, FunctionI2C, PullUp>,
    ),
>;
static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];
static IRQ_PIN: Mutex<RefCell<Option<IrqPin>>> = Mutex::new(RefCell::new(None));
static TOUCH_IRQ_PENDING: AtomicBool = AtomicBool::new(false);
static TOUCH_X: AtomicU16 = AtomicU16::new(0);
static TOUCH_Y: AtomicU16 = AtomicU16::new(0);
static mut CORE1_STACK: Stack<4096> = Stack::new();
static G_I2C: Mutex<RefCell<Option<SharedI2C>>> = Mutex::new(RefCell::new(None));

fn redraw_screen<SPI, CS, DC, RST, CHI>(
    st7789: &mut St7789Interface<SPI, CS, DC, RST, CHI>,
    timer: &mut Timer,
    offset: usize,
    colors: &[u16],
) where
    SPI: SpiBus<u8> + rp2040_hal::dma::WriteTarget<TransmittedWord = u8>,
    CS: OutputPin,
    DC: OutputPin,
    RST: OutputPin,
    CHI: ChannelIndex,
{
    let start = timer.get_counter();
    for y in 0..st7789.height() {
        let logical_y = (offset + y) % st7789.height();
        let band = logical_y / BAND_HEIGHT;
        let color = colors[band];
        let range = { 0..st7789.width() };
        st7789.process_line(y, range, |buf| {
            for px in buf.iter_mut() {
                *px = color;
            }
        });
    }
    let end = timer.get_counter();
    let micros = (end - start).to_micros();
    defmt::info!("draw: {}us (FPS = {})", micros, 1_000_000 / micros);
}

fn touch_task() {
    loop {
        cortex_m::interrupt::free(|cs| {
            if TOUCH_IRQ_PENDING.load(Ordering::Acquire) {
                TOUCH_IRQ_PENDING.store(false, Ordering::Release);
                let mut i2c_guard = G_I2C.borrow(cs).borrow_mut();
                if let Some(ref mut i2c) = *i2c_guard {
                    let mut touch_data = [0u8; 6];
                    if i2c
                        .write_read(CST816S_ADDR, &[REG_DATA], &mut touch_data)
                        .is_ok()
                    {
                        let finger = touch_data[1];
                        let x = ((touch_data[2] as u16 & 0x0F) << 8) | touch_data[3] as u16;
                        let y = ((touch_data[4] as u16 & 0x0F) << 8) | touch_data[5] as u16;
                        if finger > 0 {
                            TOUCH_X.store(x, Ordering::Release);
                            TOUCH_Y.store(y, Ordering::Release);
                        } else {
                            TOUCH_X.store(0xFFFF, Ordering::Release);
                            TOUCH_Y.store(0xFFFF, Ordering::Release);
                        }
                    }
                }
            }
        });
        cortex_m::asm::wfe();
    }
}

#[entry]
fn main() -> ! {
    info!("Program start");
    unsafe { ALLOCATOR.init(core::ptr::addr_of_mut!(HEAP) as usize, HEAP_SIZE) }
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let mut sio = Sio::new(pac.SIO);
    let mut multicore = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
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

    let tp_sda = pins
        .gpio6
        .into_pull_up_input()
        .into_function::<hal::gpio::FunctionI2C>();
    let tp_scl = pins
        .gpio7
        .into_pull_up_input()
        .into_function::<hal::gpio::FunctionI2C>();
    let mut tp_rst = pins.gpio22.into_push_pull_output();
    let tp_irq = pins
        .gpio21
        .into_pull_up_input()
        .into_function::<hal::gpio::FunctionSio<hal::gpio::SioInput>>();
    cortex_m::interrupt::free(|cs| {
        IRQ_PIN.borrow(cs).replace(Some(tp_irq));
        IRQ_PIN
            .borrow(cs)
            .borrow()
            .as_ref()
            .unwrap()
            .set_interrupt_enabled(GpioInterrupt::LevelLow, true);
    });
    let mut i2c = I2C::i2c1(
        pac.I2C1,
        tp_sda,
        tp_scl,
        300.kHz(),
        &mut pac.RESETS,
        &clocks.peripheral_clock,
    );
    let lcd_dc = pins.gpio8.into_push_pull_output();
    let lcd_cs = pins.gpio9.into_push_pull_output();
    let lcd_clk = pins.gpio10.into_function::<hal::gpio::FunctionSpi>();
    let lcd_din = pins.gpio11.into_function::<hal::gpio::FunctionSpi>();
    let lcd_rst = pins.gpio13.into_push_pull_output();
    let mut lcd_bl = pins.gpio15.into_push_pull_output();
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    tp_rst.set_low().unwrap();
    delay.delay_ms(100);
    tp_rst.set_high().unwrap();
    delay.delay_ms(200);

    let spi = Spi::<_, _, _, 8>::new(pac.SPI1, (lcd_din, lcd_clk)).init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        SPI_ST7789VW_MAX_FREQ,
        embedded_hal::spi::MODE_3,
    );

    let dma_ch = pac.DMA.split(&mut pac.RESETS).ch0;
    let mut st7789 =
        St7789Interface::new(spi, lcd_cs, lcd_dc, lcd_rst, dma_ch, DisplayRotation::Deg0);
    st7789.init(&mut delay);

    let mut buf = [0u8; 1];
    let status = i2c.write_read(CST816S_ADDR, &[REG_VERSION], &mut buf);
    match status {
        Ok(_) => defmt::info!("CST816S version: 0x{:02X}", buf[0]),
        Err(e) => defmt::warn!("I2C error: {:?}", defmt::Debug2Format(&e)),
    }
    let _free = cortex_m::interrupt::free(|cs| {
        *G_I2C.borrow(cs).borrow_mut() = Some(i2c);
    });

    unsafe {
        pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
    }
    let cores = multicore.cores();
    let core1 = &mut cores[1];
    let _ = core1.spawn(unsafe { CORE1_STACK.take().unwrap() }, touch_task);

    let test_colors = [
        0xF800, // Red
        0x07E0, // Green
        0x001F, // Blue
        0xFFFF, // White
        0x0000, // Black
    ];
    let mut prev_y: u16 = (st7789.height() + 1) as u16;
    let mut scroll_offset = 0;
    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    redraw_screen(&mut st7789, &mut timer, scroll_offset, &test_colors);
    lcd_bl.set_high().unwrap();
    loop {
        let mut x = TOUCH_X.load(Ordering::Acquire);
        let mut y = TOUCH_Y.load(Ordering::Acquire);
        (x, y) = st7789.convert_point(x, y);
        let mut scroll_offset_changed = false;
        if y < st7789.height() as u16 {
            if prev_y > st7789.height() as u16 {
                prev_y = y;
            } else if prev_y.abs_diff(y) > 1 {
                if prev_y > y {
                    scroll_offset = (scroll_offset + prev_y.abs_diff(y) as usize) % st7789.height();
                } else {
                    scroll_offset = (scroll_offset - prev_y.abs_diff(y) as usize) % st7789.height();
                }
                scroll_offset_changed = true;
                prev_y = y;
            }
        } else {
            prev_y = (st7789.height() + 1) as u16;
        }

        if scroll_offset_changed {
            redraw_screen(&mut st7789, &mut timer, scroll_offset, &test_colors);
        }
        cortex_m::asm::wfe();
    }
}

#[interrupt]
fn IO_IRQ_BANK0() {
    cortex_m::interrupt::free(|cs| {
        if let Some(tp_irq) = IRQ_PIN.borrow(cs).borrow_mut().as_mut() {
            if tp_irq.is_low().unwrap_or(false) {
                TOUCH_IRQ_PENDING.store(true, Ordering::Release);
            }
            tp_irq.clear_interrupt(GpioInterrupt::LevelLow);
        }
    });
    cortex_m::asm::sev();
}
