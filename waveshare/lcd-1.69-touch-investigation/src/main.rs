#![no_std]
#![no_main]

#[link_section = ".boot2"]
#[used]
static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

use crate::pac::interrupt;
use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, AtomicU16, Ordering};
use cortex_m::interrupt::Mutex;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::{
    digital::{InputPin, OutputPin},
    i2c::I2c,
    spi::SpiBus,
};
use hal::multicore::{Multicore, Stack};
use hal::{
    clocks::{init_clocks_and_plls, Clock},
    entry,
    fugit::{Hertz, RateExtU32},
    dma::{DMAExt ,ChannelIndex},
    gpio::{bank0::*, FunctionI2C, Interrupt as GpioInterrupt, Pin, PullUp},
    pac,
    sio::Sio,
    watchdog::Watchdog,
    Spi, Timer, I2C,
};
use panic_probe as _;
use rp2040_hal as hal;

const SPI_ST7789VW_MAX_FREQ: Hertz<u32> = Hertz::<u32>::Hz(33_300_000);
const CST816S_ADDR: u8 = 0x15;
const REG_VERSION: u8 = 0xA7;
const REG_DATA: u8 = 0x01;
const SCREEN_HEIGHT: usize = 280;
const BAND_HEIGHT: usize = 56;
const TRANSFER_SIZE: usize = 240 * 2;

type IrqPin = Pin<Gpio21, hal::gpio::FunctionSio<hal::gpio::SioInput>, PullUp>;
type SharedI2C = I2C<
    pac::I2C1,
    (
        Pin<Gpio6, FunctionI2C, PullUp>,
        Pin<Gpio7, FunctionI2C, PullUp>,
    ),
>;
static IRQ_PIN: Mutex<RefCell<Option<IrqPin>>> = Mutex::new(RefCell::new(None));
static TOUCH_IRQ_PENDING: AtomicBool = AtomicBool::new(false);
static TOUCH_X: AtomicU16 = AtomicU16::new(0);
static TOUCH_Y: AtomicU16 = AtomicU16::new(0);
static mut CORE1_STACK: Stack<4096> = Stack::new();
static G_I2C: Mutex<RefCell<Option<SharedI2C>>> = Mutex::new(RefCell::new(None));
static mut TX_BUFFER: [u8; TRANSFER_SIZE] = [0; TRANSFER_SIZE];
pub struct St7789Interface<SPI, CS, DC> {
    spi: SPI,
    cs: CS,
    dc: DC,
}

impl<SPI, CS, DC> St7789Interface<SPI, CS, DC>
where
    SPI: SpiBus + rp2040_hal::dma::WriteTarget<TransmittedWord = u8>,
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
    pub fn write_dma_blocking<CHI>(
        &mut self,
        ch : &mut rp2040_hal::dma::Channel<CHI>,
        buf: &'static [u8],
    ) where
        CHI: rp2040_hal::dma::ChannelIndex,
    {
        use rp2040_hal::dma::single_buffer::Config;
        use core::ptr;

        self.cs.set_low().ok();
        self.dc.set_high().ok();

        let spi_owned     = unsafe { ptr::read(&self.spi) };
        let channel_owned = unsafe { ptr::read(ch) };
        let (ch_back, _buf_back, spi_back) = unsafe {
            Config::new(channel_owned, buf, spi_owned)
                .start()
                .wait()
        };
        unsafe {
            ptr::write(ch, ch_back);
            ptr::write(&mut self.spi, spi_back);
        }
        self.spi.flush().ok();
        self.cs.set_high().ok();
    }
}


fn redraw_screen<SPI, CS, DC, CHI>(
    st7789 : &mut St7789Interface<SPI, CS, DC>,
    dma_ch : &mut rp2040_hal::dma::Channel<CHI>,
    timer  : &mut Timer,
    offset : usize,
    colors : &[u16],
) where
    SPI: SpiBus<u8> + rp2040_hal::dma::WriteTarget<TransmittedWord = u8>,
    CS : OutputPin,
    DC : OutputPin,
    CHI: ChannelIndex,
{
    let start = timer.get_counter();
    for y in 0..SCREEN_HEIGHT {
        let logical_y = (offset + y) % SCREEN_HEIGHT;
        let band = logical_y / BAND_HEIGHT;
        let color = colors[band];
        let hi = (color >> 8) as u8;
        let lo = (color & 0xFF) as u8;
        unsafe {
            for px in TX_BUFFER.chunks_exact_mut(2) {
                px[0] = hi;
                px[1] = lo;
            }
        }
        st7789.write_dma_blocking(dma_ch, unsafe { &TX_BUFFER });
        //st7789.write_data(unsafe{&TX_BUFFER});
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
    let mut lcd_rst = pins.gpio13.into_push_pull_output();
    let mut lcd_bl = pins.gpio15.into_push_pull_output();
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    tp_rst.set_low().unwrap();
    lcd_rst.set_low().unwrap();
    delay.delay_ms(100);
    tp_rst.set_high().unwrap();
    lcd_rst.set_high().unwrap();
    delay.delay_ms(200);

    let spi = Spi::<_, _, _, 8>::new(pac.SPI1, (lcd_din, lcd_clk)).init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        SPI_ST7789VW_MAX_FREQ,
        embedded_hal::spi::MODE_3,
    );

    let mut dma_ch0 = pac.DMA.split(&mut pac.RESETS).ch0;
    let mut st7789 = St7789Interface::new(spi, lcd_cs, lcd_dc);

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

    st7789.write_command(0x01); // software reset
    delay.delay_ms(120);
    st7789.write_command(0x11); // sleep out
    delay.delay_ms(120);
    st7789.write_command(0x3A); // pixel format
    st7789.write_data(&[0x55]); // RGB565
    st7789.write_command(0x36); // MADCTL
    st7789.write_data(&[0x00]); // BGR, no rotet
    st7789.write_command(0x21); // INVON
    st7789.write_command(0x29); // Display on
    delay.delay_ms(20);

    st7789.write_command(0x2A); // Column
    st7789.write_data(&[0x00, 0x00, 0x00, 0xEF]);
    st7789.write_command(0x2B); // Row
    st7789.write_data(&[0x00, 0x14, 0x01, 0x2B]);
    st7789.write_command(0x2C);

    let test_colors = [
        0xF800, // Red
        0x07E0, // Green
        0x001F, // Blue
        0xFFFF, // White
        0x0000, // Black
    ];
    let mut prev_y: u16 = (SCREEN_HEIGHT + 1) as u16;
    let mut scroll_offset = 0;
    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    redraw_screen(
        &mut st7789,
        &mut dma_ch0,
        &mut timer,
        scroll_offset,
        &test_colors,
    );
    lcd_bl.set_high().unwrap();
    loop {
        let y = TOUCH_Y.load(Ordering::Acquire);
        let mut scroll_offset_changed = false;
        if y < SCREEN_HEIGHT as u16 {
            if prev_y > SCREEN_HEIGHT as u16 {
                prev_y = y;
            } else if prev_y.abs_diff(y) > 1 {
                if prev_y > y {
                    scroll_offset = (scroll_offset + prev_y.abs_diff(y) as usize) % SCREEN_HEIGHT;
                } else {
                    scroll_offset = (scroll_offset - prev_y.abs_diff(y) as usize) % SCREEN_HEIGHT;
                }
                scroll_offset_changed = true;
                prev_y = y;
            }
        } else {
            prev_y = (SCREEN_HEIGHT + 1) as u16;
        }

        if scroll_offset_changed {
            redraw_screen(
                &mut st7789,
                &mut dma_ch0,
                &mut timer,
                scroll_offset,
                &test_colors,
            );
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
