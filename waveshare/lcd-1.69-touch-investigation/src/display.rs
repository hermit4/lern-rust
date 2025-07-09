use core::ops::Range;
use embedded_hal::digital::OutputPin;
use embedded_hal::spi::SpiBus;
use rp2040_hal::dma::single_buffer::Config;
use rp2040_hal::dma::{Channel, ChannelIndex, WriteTarget};
use rp2040_hal::pac::SPI1;

pub struct St7789Interface<SPI, CS, DC, RST, CHI>
where
    CHI: ChannelIndex,
{
    spi: Option<SPI>,
    cs: CS,
    dc: DC,
    rst: RST,
    dma_ch: Option<Channel<CHI>>,
}

impl<SPI, CS, DC, RST, CHI> St7789Interface<SPI, CS, DC, RST, CHI>
where
    SPI: SpiBus + WriteTarget<TransmittedWord = u8>,
    CS: OutputPin,
    DC: OutputPin,
    RST: OutputPin,
    CHI: ChannelIndex,
{
    pub fn new(spi: SPI, cs: CS, dc: DC, rst: RST, dma_ch: Channel<CHI>) -> Self {
        Self {
            spi: Some(spi),
            cs,
            dc,
            rst,
            dma_ch: Some(dma_ch),
        }
    }

    pub fn init(&mut self, delay: &mut cortex_m::delay::Delay) {
        // hardware reset
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
        self.write_command(0x36); // MADCTL
        self.write_data(&[0x00]); // BGR, no rotet
        self.write_command(0x21); // INVON
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

    fn convert_u16_le_to_be_u8(&mut self, dst: &mut [u8], src: &[u16]) {
        for (chunk, &val) in dst.chunks_exact_mut(2).zip(src.iter()) {
            let swapped = val.swap_bytes();
            chunk.copy_from_slice(&swapped.to_ne_bytes());
        }
    }

    pub fn process_line(
        &mut self,
        line: usize,
        range: Range<usize>,
        render_fn: impl FnOnce(&mut [u16]),
    ) {
        static mut TX_BUF: [u8; 240 * 2] = [0; 240 * 2];
        let width = range.len();

        let mut line_buf = [0; 240];

        render_fn(&mut line_buf[..width]);

        const ROW_OFFSET: usize = 0x14;
        let row = line + ROW_OFFSET;
        unsafe {
            self.convert_u16_le_to_be_u8(&mut TX_BUF[..width * 2], &line_buf[..width]);
            self.write_dma_blocking(&TX_BUF[..width * 2], row, range);
        }
    }
}
