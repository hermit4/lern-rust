extern crate alloc;
use alloc::boxed::Box;
use alloc::vec;
use core::ops::Range;
use embedded_hal::digital::OutputPin;
use embedded_hal::spi::SpiBus;
use panic_probe as _;
use rp2040_hal::dma::single_buffer::Config;
use rp2040_hal::dma::{Channel, ChannelIndex, WriteTarget};
use rp2040_hal::pac::SPI1;
use slint::platform::software_renderer::{LineBufferProvider, Rgb565Pixel};

#[derive(Copy, Clone)]
pub enum DisplayRotation {
    Deg0,
    Deg90,
    Deg180,
    Deg270,
}

const SCREEN_WIDTH: usize = 240;
const SCREEN_HEIGHT: usize = 280;
const SCREEN_MAX: usize = if SCREEN_WIDTH > SCREEN_HEIGHT {
    SCREEN_WIDTH
} else {
    SCREEN_HEIGHT
};
const SCREEN_OFFSET: usize = 0x14;

fn madctl_value(rotation: DisplayRotation) -> u8 {
    match rotation {
        DisplayRotation::Deg0 => 0x00,
        DisplayRotation::Deg90 => 0xA0,
        DisplayRotation::Deg180 => 0xC0, // MX, MY
        DisplayRotation::Deg270 => 0x60, // MX, MV
    }
}

fn create_tx_buf(size: usize) -> &'static mut [u8] {
    let buf: Box<[u8]> = vec![0u8; size].into_boxed_slice();
    Box::leak(buf)
}

fn convert_rgb565_le_to_be_u8(dst: &mut [u8], src: &[Rgb565Pixel]) {
    for (chunk, &Rgb565Pixel(val)) in dst.chunks_exact_mut(2).zip(src.iter()) {
        let swapped = val.swap_bytes();
        chunk.copy_from_slice(&swapped.to_ne_bytes());
    }
}

pub struct St7789Interface<SPI, CS, DC, RST, CHI>
where
    CHI: ChannelIndex,
{
    spi: Option<SPI>,
    cs: CS,
    dc: DC,
    rst: RST,
    dma_ch: Option<Channel<CHI>>,
    tx_buf: &'static mut [u8],
    rotation: DisplayRotation,
}

impl<SPI, CS, DC, RST, CHI> St7789Interface<SPI, CS, DC, RST, CHI>
where
    SPI: SpiBus + WriteTarget<TransmittedWord = u8>,
    CS: OutputPin,
    DC: OutputPin,
    RST: OutputPin,
    CHI: ChannelIndex,
{
    pub fn new(
        spi: SPI,
        cs: CS,
        dc: DC,
        rst: RST,
        dma_ch: Channel<CHI>,
        rotation: DisplayRotation,
    ) -> Self {
        let width = match rotation {
            DisplayRotation::Deg0 | DisplayRotation::Deg180 => SCREEN_WIDTH,
            DisplayRotation::Deg90 | DisplayRotation::Deg270 => SCREEN_HEIGHT,
        };
        let tx_buf = create_tx_buf(width * 2);
        Self {
            spi: Some(spi),
            cs,
            dc,
            rst,
            dma_ch: Some(dma_ch),
            tx_buf,
            rotation,
        }
    }

    pub fn init(&mut self, delay: &mut cortex_m::delay::Delay) {
        // hardware reset
        let modctl = madctl_value(self.rotation);

        self.rst.set_low().unwrap();
        delay.delay_ms(10);
        self.rst.set_high().unwrap();
        delay.delay_ms(120);

        self.write_command(0x01); // software reset
        delay.delay_ms(120);
        self.write_command(0x11); // sleep out
        delay.delay_ms(120);
        self.write_command(0x3A); // pixel format
        self.write_data(&[0x55]); // RGB565
        self.write_command(0x21); // INVON
        self.write_command(0x2A); // Column
        self.write_data(&[0x00, 0x00, 0x00, 0xEF]);
        self.write_command(0x2B); // Row
        self.write_data(&[0x00, 0x14, 0x01, 0x2B]);
        self.write_command(0x2C); // data
        let pixel_data = [0u8; 240 * 2];
        for _ in 0..SCREEN_HEIGHT {
            self.write_data(&pixel_data);
        }
        self.write_command(0x36); // MADCTL
        self.write_data(&[modctl]); // BGR, no rotet
        self.write_command(0x29); // Display on
        delay.delay_ms(20);
    }

    pub fn write_command(&mut self, cmd: u8) {
        self.cs.set_low().ok();
        self.dc.set_low().ok();
        if let Some(spi) = &mut self.spi {
            spi.write(&[cmd]).ok();
            unsafe { while (*SPI1::ptr()).sspsr().read().bsy().bit_is_set() {} }
        }
        self.cs.set_high().ok();
    }

    pub fn write_data(&mut self, data: &[u8]) {
        self.cs.set_low().ok();
        self.dc.set_high().ok();
        if let Some(spi) = &mut self.spi {
            spi.write(data).ok();
            unsafe { while (*SPI1::ptr()).sspsr().read().bsy().bit_is_set() {} }
        }
        self.cs.set_high().ok();
    }

    pub fn write_dma_blocking(&mut self, buf: &'static [u8], row: usize, range: Range<usize>) {
        self.cs.set_low().ok();

        let mut spi = self.spi.take().unwrap();
        let dma_ch = self.dma_ch.take().unwrap();

        self.dc.set_low().ok();
        cortex_m::asm::delay(10);
        spi.write(&[0x2A]).ok();
        unsafe { while (*SPI1::ptr()).sspsr().read().bsy().bit_is_set() {} }

        self.dc.set_high().ok();
        cortex_m::asm::delay(10);
        spi.write(&[
            (range.start >> 8) as u8,
            (range.start & 0xff) as u8,
            ((range.end - 1) >> 8) as u8,
            ((range.end - 1) & 0xff) as u8,
        ])
        .ok();
        unsafe { while (*SPI1::ptr()).sspsr().read().bsy().bit_is_set() {} }

        self.dc.set_low().ok();
        cortex_m::asm::delay(10);
        spi.write(&[0x2B]).ok();
        unsafe { while (*SPI1::ptr()).sspsr().read().bsy().bit_is_set() {} }

        self.dc.set_high().ok();
        cortex_m::asm::delay(10);
        spi.write(&[
            (row >> 8) as u8,
            (row & 0xff) as u8,
            (row >> 8) as u8,
            (row & 0xff) as u8,
        ])
        .ok();
        unsafe { while (*SPI1::ptr()).sspsr().read().bsy().bit_is_set() {} }

        self.dc.set_low().ok();
        cortex_m::asm::delay(10);
        spi.write(&[0x2C]).ok();
        unsafe { while (*SPI1::ptr()).sspsr().read().bsy().bit_is_set() {} }

        self.dc.set_high().ok();
        cortex_m::asm::delay(10);

        let (ch_back, _buf_back, spi_back) = Config::new(dma_ch, buf, spi).start().wait();
        self.dma_ch = Some(ch_back);
        self.spi = Some(spi_back);

        unsafe { while (*SPI1::ptr()).sspsr().read().bsy().bit_is_set() {} }
        self.cs.set_high().ok();
    }

    pub fn convert_point(&mut self, x: u16, y: u16) -> (u16, u16) {
        let ret = match self.rotation {
            DisplayRotation::Deg0 => (x, y),
            DisplayRotation::Deg90 => (SCREEN_HEIGHT as u16 - y - 1, x),
            DisplayRotation::Deg180 => (SCREEN_WIDTH as u16 - x - 1, SCREEN_HEIGHT as u16 - y - 1),
            DisplayRotation::Deg270 => (y, SCREEN_WIDTH as u16 - x - 1),
        };
        ret
    }
    pub fn height(&mut self) -> usize {
        match self.rotation {
            DisplayRotation::Deg0 | DisplayRotation::Deg180 => SCREEN_HEIGHT,
            DisplayRotation::Deg90 | DisplayRotation::Deg270 => SCREEN_WIDTH,
        }
    }
    pub fn width(&mut self) -> usize {
        match self.rotation {
            DisplayRotation::Deg0 | DisplayRotation::Deg180 => SCREEN_WIDTH,
            DisplayRotation::Deg90 | DisplayRotation::Deg270 => SCREEN_HEIGHT,
        }
    }

    pub fn size(&mut self) -> slint::PhysicalSize {
        let width = self.width() as u32;
        let height = self.height() as u32;
        slint::PhysicalSize::new(width, height)
    }
}

impl<SPI, CS, DC, RST, CHI> LineBufferProvider for &mut St7789Interface<SPI, CS, DC, RST, CHI>
where
    SPI: SpiBus + WriteTarget<TransmittedWord = u8>,
    CS: OutputPin,
    DC: OutputPin,
    RST: OutputPin,
    CHI: ChannelIndex,
{
    type TargetPixel = Rgb565Pixel;
    fn process_line(
        &mut self,
        line: usize,
        range: core::ops::Range<usize>,
        render_fn: impl FnOnce(&mut [Rgb565Pixel]),
    ) {
        let mut line_buf = [Rgb565Pixel(0); SCREEN_MAX];
        let width = range.len();
        render_fn(&mut line_buf[..width]);
        let tx_buf = &mut self.tx_buf[..width * 2];
        convert_rgb565_le_to_be_u8(tx_buf, &line_buf[..width]);
        let tx_buf_static: &'static [u8] =
            unsafe { core::slice::from_raw_parts(tx_buf.as_ptr(), tx_buf.len()) };
        let row = match self.rotation {
            DisplayRotation::Deg0 | DisplayRotation::Deg180 => line + SCREEN_OFFSET,
            DisplayRotation::Deg90 | DisplayRotation::Deg270 => line,
        };
        let drange = match self.rotation {
            DisplayRotation::Deg0 | DisplayRotation::Deg180 => range,
            DisplayRotation::Deg90 | DisplayRotation::Deg270 => {
                range.start + SCREEN_OFFSET..range.end + SCREEN_OFFSET
            }
        };
        self.write_dma_blocking(tx_buf_static, row, drange);
    }
}
