use crate::pac::interrupt;
use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, Ordering};
use cortex_m::interrupt::Mutex;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::i2c::I2c;
use hal::gpio::{bank0::*, Interrupt as GpioInterrupt, Pin, PullUp};
use hal::pac;
use panic_probe as _;
use rp2040_hal as hal;
type IrqPin = Pin<Gpio21, hal::gpio::FunctionSio<hal::gpio::SioInput>, PullUp>;

const CST816S_ADDR: u8 = 0x15;
const REG_DATA: u8 = 0x01;

static IRQ_PIN: Mutex<RefCell<Option<IrqPin>>> = Mutex::new(RefCell::new(None));
static TOUCH_IRQ_PENDING: AtomicBool = AtomicBool::new(false);

#[derive(Copy, Clone, PartialEq, Eq)]
pub enum TouchState {
    None,
    Touching,
}

pub enum TouchStatus {
    None,
    TouchUp { x: u16, y: u16 },
    TouchDown { x: u16, y: u16 },
    TouchMove { x: u16, y: u16 },
}

pub struct CST816SInterface<I2C, RST> {
    i2c: Option<I2C>,
    _rst: RST,
}

impl<I2C, RST, E> CST816SInterface<I2C, RST>
where
    I2C: I2c<Error = E>,
    RST: OutputPin,
{
    pub fn new(mut i2c: I2C, mut rst: RST, irq: IrqPin) -> Result<Self, E> {
        rst.set_low().ok();
        cortex_m::asm::delay(10_000);
        rst.set_high().ok();
        cortex_m::asm::delay(100_000);

        i2c.write(CST816S_ADDR, &[0xEC, 0x00])?; // ignore gesture
        i2c.write(CST816S_ADDR, &[0xEE, 0x05])?; // scan timing
        i2c.write(CST816S_ADDR, &[0xF9, 0x00])?; // Auto sleep time = 0
        i2c.write(CST816S_ADDR, &[0xFE, 0x01])?; // Disable auto sleep
        i2c.write(CST816S_ADDR, &[0xFA, 0x60])?; // interrupt setting
        cortex_m::interrupt::free(|cs| {
            IRQ_PIN.borrow(cs).replace(Some(irq));
            IRQ_PIN
                .borrow(cs)
                .borrow()
                .as_ref()
                .unwrap()
                .set_interrupt_enabled(GpioInterrupt::LevelLow, true);
        });
        unsafe {
            pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
        }
        Ok(Self {
            i2c: Some(i2c),
            _rst: rst,
        })
    }

    pub fn read_touch_status(&mut self, state: &mut TouchState) -> Result<TouchStatus, E> {
        if !TOUCH_IRQ_PENDING.load(Ordering::Acquire) {
            return Ok(TouchStatus::None);
        }

        TOUCH_IRQ_PENDING.store(false, Ordering::Release);
        let mut touch_data = [0u8; 6];
        self.i2c
            .as_mut()
            .unwrap()
            .write_read(CST816S_ADDR, &[REG_DATA], &mut touch_data)?;
        let finger = touch_data[1];
        let x = ((touch_data[2] as u16 & 0x0F) << 8) | touch_data[3] as u16;
        let y = ((touch_data[4] as u16 & 0x0F) << 8) | touch_data[5] as u16;
        let event = {
            if finger == 0 {
                match *state {
                    TouchState::None => TouchStatus::None,
                    TouchState::Touching => {
                        *state = TouchState::None;
                        TouchStatus::TouchUp { x, y }
                    }
                }
            } else {
                match *state {
                    TouchState::None => {
                        *state = TouchState::Touching;
                        TouchStatus::TouchDown { x, y }
                    }
                    TouchState::Touching => TouchStatus::TouchMove { x, y },
                }
            }
        };
        return Ok(event);
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
}
