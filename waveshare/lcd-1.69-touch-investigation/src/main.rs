#![no_std]
#![no_main]

#[link_section = ".boot2"]
#[used]
static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

extern crate alloc;
mod display;
mod touch;
use alloc::rc::Rc;
use alloc::boxed::Box;
use crate::{
    display::{DisplayRotation, St7789Interface},
    touch::CST816SInterface,
    touch::TouchState,
    touch::TouchStatus,
};
use defmt::*;
use defmt_rtt as _;
use embedded_alloc::LlffHeap as Heap;
use embedded_hal::digital::OutputPin;
use hal::{
    clocks::{init_clocks_and_plls, Clock},
    dma::DMAExt,
    entry,
    fugit::{Hertz, RateExtU32},
    pac,
    sio::Sio,
    watchdog::Watchdog,
    Spi, I2C, Timer,
};
use panic_probe as _;
use rp2040_hal as hal;
use slint::platform::software_renderer::MinimalSoftwareWindow;
use slint::platform::Platform;
use slint::platform::WindowEvent;
use slint::LogicalPosition;
use slint::platform::PointerEventButton;

const HEAP_SIZE: usize = 200 * 1024;
const SPI_ST7789VW_MAX_FREQ: Hertz<u32> = Hertz::<u32>::Hz(33_300_000);

#[global_allocator]
static ALLOCATOR: Heap = Heap::empty();
static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];

slint::include_modules!();

struct MyPlatform {
    window: Rc<MinimalSoftwareWindow>,
    timer: hal::Timer,
}

impl Platform for MyPlatform {
    fn create_window_adapter(&self) -> Result<Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError> {
        Ok(self.window.clone())
    }
    fn duration_since_start(&self) -> core::time::Duration {
        core::time::Duration::from_micros(self.timer.get_counter().ticks())
    }
}

#[entry]
fn main() -> ! {
    info!("Program start");
    unsafe { ALLOCATOR.init(core::ptr::addr_of_mut!(HEAP) as usize, HEAP_SIZE) }
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
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

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
    let tp_rst = pins.gpio22.into_push_pull_output();
    let tp_irq = pins
        .gpio21
        .into_pull_up_input()
        .into_function::<hal::gpio::FunctionSio<hal::gpio::SioInput>>();
    let i2c = I2C::i2c1(
        pac.I2C1,
        tp_sda,
        tp_scl,
        300.kHz(),
        &mut pac.RESETS,
        &clocks.peripheral_clock,
    );
    let mut cst816s = CST816SInterface::new(i2c, tp_rst, tp_irq).unwrap();

    let lcd_dc = pins.gpio8.into_push_pull_output();
    let lcd_cs = pins.gpio9.into_push_pull_output();
    let lcd_clk = pins.gpio10.into_function::<hal::gpio::FunctionSpi>();
    let lcd_din = pins.gpio11.into_function::<hal::gpio::FunctionSpi>();
    let lcd_rst = pins.gpio13.into_push_pull_output();
    let mut lcd_bl = pins.gpio15.into_push_pull_output();
    let spi = Spi::<_, _, _, 8>::new(pac.SPI1, (lcd_din, lcd_clk)).init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        SPI_ST7789VW_MAX_FREQ,
        embedded_hal::spi::MODE_3,
    );

    let dma_ch = pac.DMA.split(&mut pac.RESETS).ch0;
    let mut st7789 = St7789Interface::new(
        spi,
        lcd_cs,
        lcd_dc,
        lcd_rst,
        dma_ch,
        DisplayRotation::Deg270,
    );
    st7789.init(&mut delay);
    lcd_bl.set_high().ok();

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    let window = MinimalSoftwareWindow::new(Default::default());
    window.set_size(st7789.size());
    slint::platform::set_platform(Box::new(MyPlatform {
        window: window.clone(),
        timer: timer,
    })).unwrap();

    let ui = AppWindow::new().unwrap();
    ui.on_request_increase_value({
        let ui_handle = ui.as_weak();
        move || {
            let ui = ui_handle.unwrap();
            ui.set_counter(ui.get_counter() + 1);
        }
    });
    ui.run().ok();

    let mut touch_state = TouchState::None;
    loop {
        slint::platform::update_timers_and_animations();
        window.draw_if_needed(|renderer| {
            renderer.render_by_line(&mut st7789);
        });

        if let Ok(status) = cst816s.read_touch_status(&mut touch_state) {
            match status {
                TouchStatus::TouchDown { x, y } => {
                    let (x,y) = st7789.convert_point(x,y);
                    window.dispatch_event(WindowEvent::PointerPressed {
                        position: LogicalPosition::new(x.into(),y.into()),
                        button: PointerEventButton::Left,
                    });
                }
                TouchStatus::TouchUp { x, y } => {
                    let (x,y) = st7789.convert_point(x,y);
                    window.dispatch_event(WindowEvent::PointerReleased {
                        position: LogicalPosition::new(x.into(),y.into()),
                        button: PointerEventButton::Left,
                    });
                }
                TouchStatus::TouchMove { x, y } => {
                    let (x,y) = st7789.convert_point(x,y);
                    window.dispatch_event(WindowEvent::PointerMoved {
                        position: LogicalPosition::new(x.into(),y.into()),
                    });
                }
                TouchStatus::None => {}
            }
        }
        cortex_m::asm::wfe();
    }
}
