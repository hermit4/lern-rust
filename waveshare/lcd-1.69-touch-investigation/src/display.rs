use embedded_hal::digital::OutputPin;
use embedded_hal::spi::SpiBus;
use rp2040_hal::dma::single_buffer::Config;
use rp2040_hal::dma::{Channel, ChannelIndex, WriteTarget};

pub struct St7789Interface<SPI, CS, DC, CHI>
where
    CHI: ChannelIndex,
{
    pub spi: Option<SPI>,
    pub cs: CS,
    pub dc: DC,
    pub dma_ch: Option<Channel<CHI>>,
}

impl<SPI, CS, DC, CHI> St7789Interface<SPI, CS, DC, CHI>
where
    SPI: SpiBus + WriteTarget<TransmittedWord = u8>,
    CS: OutputPin,
    DC: OutputPin,
    CHI: ChannelIndex,
{
    pub fn new(spi: SPI, cs: CS, dc: DC, dma_ch: Channel<CHI>) -> Self {
        Self {
            spi: Some(spi),
            cs,
            dc,
            dma_ch: Some(dma_ch),
        }
    }

    pub fn write_command(&mut self, cmd: u8) {
        self.cs.set_low().ok();
        self.dc.set_low().ok();
        if let Some(spi) = &mut self.spi {
            spi.write(&[cmd]).ok();
        }
        self.cs.set_high().ok();
    }

    pub fn write_data(&mut self, data: &[u8]) {
        self.cs.set_low().ok();
        self.dc.set_high().ok();
        if let Some(spi) = &mut self.spi {
            spi.write(data).ok();
        }
        self.cs.set_high().ok();
    }

    pub fn write_dma_blocking(&mut self, buf: &'static [u8]) {
        self.cs.set_low().ok();
        self.dc.set_high().ok();

        let spi = self.spi.take().unwrap();
        let dma_ch = self.dma_ch.take().unwrap();

        let (ch_back, _buf_back, spi_back) = Config::new(dma_ch, buf, spi).start().wait();

        self.spi = Some(spi_back);
        self.dma_ch = Some(ch_back);

        if let Some(spi) = &mut self.spi {
            spi.flush().ok();
        }

        self.cs.set_high().ok();
    }
}
