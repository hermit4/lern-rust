#![no_std]
#![no_main]

#[link_section = ".boot2"]
#[used]
static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

extern crate alloc;
mod display;
mod touch;
use crate::pac::interrupt;
use crate::{
    display::{DisplayRotation, St7789Interface},
    touch::CST816SInterface,
    touch::TouchState,
    touch::TouchStatus,
};
use alloc::boxed::Box;
use alloc::rc::Rc;
use alloc::vec::Vec;
use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use defmt_rtt as _;
use embedded_alloc::LlffHeap as Heap;
use embedded_hal::digital::OutputPin;
use hal::{
    clocks::{init_clocks_and_plls, Clock},
    dma::DMAExt,
    entry,
    fugit::{self, Hertz, RateExtU32},
    pac,
    sio::Sio,
    timer::{Alarm, Alarm0},
    watchdog::Watchdog,
    Spi, Timer, I2C,
};
use panic_probe as _;
use rp2040_hal as hal;
use slint::platform::software_renderer::MinimalSoftwareWindow;
use slint::platform::Platform;
use slint::platform::PointerEventButton;
use slint::platform::WindowEvent;
use slint::LogicalPosition;
use slint::Model;

const HEAP_SIZE: usize = 180 * 1024;
const SPI_ST7789VW_MAX_FREQ: Hertz<u32> = Hertz::<u32>::Hz(33_300_000);

#[global_allocator]
static ALLOCATOR: Heap = Heap::empty();
static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];

static ALARM0: Mutex<RefCell<Option<Alarm0>>> = Mutex::new(RefCell::new(None));

slint::include_modules!();

struct MyPlatform {
    window: Rc<MinimalSoftwareWindow>,
    timer: hal::Timer,
}

impl Platform for MyPlatform {
    fn create_window_adapter(
        &self,
    ) -> Result<Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError> {
        Ok(self.window.clone())
    }
    fn duration_since_start(&self) -> core::time::Duration {
        core::time::Duration::from_micros(self.timer.get_counter().ticks())
    }
}

struct PrinterQueueData {
    data: Rc<slint::VecModel<PrinterQueueItem>>,
    print_progress_timer: slint::Timer,
}

impl PrinterQueueData {
    fn push_job(&self, title: slint::SharedString) {
        self.data.push(PrinterQueueItem {
            status: JobStatus::Waiting,
            progress: 0,
            title,
            owner: env!("CARGO_PKG_AUTHORS").into(),
            pages: 1,
            size: "100kB".into(),
            submission_date: "".into(),
        })
    }
}

#[entry]
fn main() -> ! {
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
    let mut led = pins.gpio25.into_push_pull_output();
    let spi = Spi::<_, _, _, 8>::new(pac.SPI1, (lcd_din, lcd_clk)).init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        SPI_ST7789VW_MAX_FREQ,
        embedded_hal::spi::MODE_3,
    );

    let dma_ch = pac.DMA.split(&mut pac.RESETS).ch0;
    let mut st7789 =
        St7789Interface::new(spi, lcd_cs, lcd_dc, lcd_rst, dma_ch, DisplayRotation::Deg90);
    st7789.init(&mut delay);
    lcd_bl.set_high().ok();
    led.set_high().ok();

    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    let mut alarm0 = timer.alarm_0().unwrap();
    alarm0.enable_interrupt();
    cortex_m::interrupt::free(|cs| {
        ALARM0.borrow(cs).replace(Some(alarm0));
    });
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::TIMER_IRQ_0);
    }

    let window = MinimalSoftwareWindow::new(Default::default());
    window.set_size(st7789.size());
    slint::platform::set_platform(Box::new(MyPlatform {
        window: window.clone(),
        timer: timer,
    }))
    .unwrap();

    let main_window = MainWindow::new().unwrap();
    main_window.set_ink_levels(
        [
            InkLevel {
                color: slint::Color::from_rgb_u8(0, 255, 255),
                level: 0.40,
            },
            InkLevel {
                color: slint::Color::from_rgb_u8(255, 0, 255),
                level: 0.20,
            },
            InkLevel {
                color: slint::Color::from_rgb_u8(255, 255, 0),
                level: 0.50,
            },
            InkLevel {
                color: slint::Color::from_rgb_u8(0, 0, 0),
                level: 0.80,
            },
        ]
        .into(),
    );

    let default_queue: Vec<PrinterQueueItem> = main_window
        .global::<PrinterQueue>()
        .get_printer_queue()
        .iter()
        .collect();
    let printer_queue = Rc::new(PrinterQueueData {
        data: Rc::new(slint::VecModel::from(default_queue.clone())),
        print_progress_timer: Default::default(),
    });
    main_window
        .global::<PrinterQueue>()
        .set_printer_queue(printer_queue.data.clone().into());

    main_window.on_quit(move || {
        #[cfg(not(target_arch = "wasm32"))]
        slint::quit_event_loop().unwrap();
    });

    let printer_queue_copy = printer_queue.clone();
    main_window
        .global::<PrinterQueue>()
        .on_start_job(move |title| {
            printer_queue_copy.push_job(title);
        });

    let printer_queue_copy = printer_queue.clone();
    main_window
        .global::<PrinterQueue>()
        .on_cancel_job(move |idx| {
            printer_queue_copy.data.remove(idx as usize);
        });

    let printer_queue_weak = Rc::downgrade(&printer_queue);
    printer_queue.print_progress_timer.start(
        slint::TimerMode::Repeated,
        core::time::Duration::from_secs(1),
        move || {
            if let Some(printer_queue) = printer_queue_weak.upgrade() {
                if printer_queue.data.row_count() > 0 {
                    let mut top_item = printer_queue.data.row_data(0).unwrap();
                    top_item.progress += 1;
                    top_item.status = JobStatus::Printing;
                    if top_item.progress > 100 {
                        printer_queue.data.remove(0);
                        if printer_queue.data.row_count() == 0 {
                            return;
                        }
                        top_item = printer_queue.data.row_data(0).unwrap();
                    }
                    printer_queue.data.set_row_data(0, top_item);
                } else {
                    printer_queue.data.set_vec(default_queue.clone());
                }
            }
        },
    );
    // main_window.run().unwrap();

    let mut touch_state = TouchState::None;
    loop {
        slint::platform::update_timers_and_animations();
        window.draw_if_needed(|renderer| {
            renderer.render_by_line(&mut st7789);
        });

        if let Ok(status) = cst816s.read_touch_status(&mut touch_state) {
            match status {
                TouchStatus::TouchDown { x, y } => {
                    let (x, y) = st7789.convert_point(x, y);
                    window.dispatch_event(WindowEvent::PointerPressed {
                        position: LogicalPosition::new(x.into(), y.into()),
                        button: PointerEventButton::Left,
                    });
                }
                TouchStatus::TouchUp { x, y } => {
                    let (x, y) = st7789.convert_point(x, y);
                    window.dispatch_event(WindowEvent::PointerReleased {
                        position: LogicalPosition::new(x.into(), y.into()),
                        button: PointerEventButton::Left,
                    });
                }
                TouchStatus::TouchMove { x, y } => {
                    let (x, y) = st7789.convert_point(x, y);
                    window.dispatch_event(WindowEvent::PointerMoved {
                        position: LogicalPosition::new(x.into(), y.into()),
                    });
                }
                TouchStatus::None => {}
            }
        }
        if window.has_active_animations() {
            continue;
        }
        let sleep_duration = match slint::platform::duration_until_next_timer_update() {
            None => None,
            Some(d) => {
                let micros = d.as_micros() as u32;
                if micros < 10 {
                    // Cannot wait for less than 10µs, or `schedule()` panics
                    continue;
                } else {
                    Some(fugit::MicrosDurationU32::micros(micros))
                }
            }
        };
        cortex_m::interrupt::free(|cs| {
            if let Some(duration) = sleep_duration {
                ALARM0
                    .borrow(cs)
                    .borrow_mut()
                    .as_mut()
                    .unwrap()
                    .schedule(duration)
                    .unwrap();
            }
        });
        cortex_m::asm::wfe();
    }
}

#[interrupt]
fn TIMER_IRQ_0() {
    cortex_m::interrupt::free(|cs| {
        ALARM0
            .borrow(cs)
            .borrow_mut()
            .as_mut()
            .unwrap()
            .clear_interrupt();
    });
}
