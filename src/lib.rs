//! Simple SPI driver for the GDEH0154D67 E-Paper display.
//! This crate is a `no_std` library that provides an interface compatible with [embedded-hal-1.0.0-alpha.8](https://docs.rs/embedded-hal/1.0.0-alpha.8/embedded_hal/).
//! It is also designed to be used together with [embedded-graphics](https://docs.rs/embedded-graphics/latest/embedded_graphics/).
//! It ensures a correct initialization and a consistent state at every moment by enforcing design
//! constrains at compile time using zero cost abstractions.
#![no_std]

use bitvec::prelude::*;
use embedded_graphics::{
    draw_target::DrawTarget, geometry::OriginDimensions, geometry::Size, pixelcolor::BinaryColor,
    Pixel,
};
use embedded_hal::delay::DelayUs;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::spi::{SpiBus, SpiDevice};

use command::*;
use error::Error;

mod command;
pub mod error;

// Original Waveforms from Waveshare for 1in54_V2
pub(crate) const LUT_FULL_UPDATE: [u8; 159] = [
    0x80, 0x48, 0x40, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x40, 0x48, 0x80, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x80, 0x48, 0x40, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x40, 0x48, 0x80, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xA, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x8, 0x1, 0x0, 0x8, 0x1, 0x0,
    0x2, 0xA, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x22, 0x22, 0x22, 0x22,
    0x22, 0x22, 0x0, 0x0, 0x0, 0x22, 0x17, 0x41, 0x0, 0x32, 0x20,
];

pub(crate) const LUT_PARTIAL_UPDATE: [u8; 159] = [
    0x0, 0x40, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x80, 0x80, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x40, 0x40, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x80, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0xF, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x22, 0x22, 0x22, 0x22, 0x22,
    0x22, 0x0, 0x0, 0x0, 0x02, 0x17, 0x41, 0xB0, 0x32, 0x28,
];

/// Main structure of the library, used to initialize and control de display.
pub struct GDEH0154D67<SPI, DC, RST, BSY, DLY, S> {
    spi: SPI,
    buffer: Buffer,
    dc: DC,
    rst: RST,
    busy: BSY,
    delay: DLY,
    #[allow(dead_code)]
    state: S,

    lut_partial: bool,
}

/// Sets the display as initialized, only after calling the method `init()`
/// the display is considered initialized.
pub struct Initialized;

/// Sets the display as not initialized. This state occurs when acquiring
/// a new instance of `GDEH0154D67` or after doing a `reset()` of the display.
pub struct NotInitialized;

/// Struct that stores the binary color of the pixels that will be set when the
/// next display update is performed
struct Buffer {
    pixels: BitArray<[u8; 5000], Msb0>,
    flags: BitArray<[u8; 5000]>,
}

impl<SPI, DC, RST, BSY, DLY> GDEH0154D67<SPI, DC, RST, BSY, DLY, NotInitialized>
where
    SPI: SpiDevice,
    SPI::Bus: SpiBus,
    DC: OutputPin,
    RST: OutputPin,
    BSY: InputPin,
    DLY: DelayUs,
{
    /// Acquires the SPI interface and the control GPIO pins. It also performs
    /// a hardware reset on the device.
    pub fn new(interface: SPI, dc: DC, reset: RST, busy: BSY, delay: DLY) -> Result<Self, Error> {
        let disp = Self {
            spi: interface,
            buffer: Buffer {
                pixels: bitarr![u8, Msb0; 0; 40000],
                flags: bitarr![u8, LocalBits; 0; 40000],
            },
            dc,
            rst: reset,
            busy,
            delay,
            state: NotInitialized,
            lut_partial: false,
        };
        disp.reset()
    }

    /// Sets the display into an initialized state. It is required to call this
    /// method once before any display update can be done.
    pub fn init(mut self) -> Result<GDEH0154D67<SPI, DC, RST, BSY, DLY, Initialized>, Error> {
        // 000 -> normal
        // 011 -> upside down
        DriverOutputControl([0xc7, 0x00, 0b000])
            .send(&mut self.spi, &mut self.dc)
            .unwrap();
        DataEntryModeSetting([0b011])
            .send(&mut self.spi, &mut self.dc)
            .unwrap();
        SetRamXAddressStartEndPosition([0x00, 0x18])
            .send(&mut self.spi, &mut self.dc)
            .unwrap();
        SetRamYAddressStartEndPosition([0x00, 0x00, 0xc7, 0x00])
            .send(&mut self.spi, &mut self.dc)
            .unwrap();
        TemperatureSensorWrite([0x43, 0x20])
            .send(&mut self.spi, &mut self.dc)
            .unwrap();
        DisplayUpdateControl2([0xb1])
            .send(&mut self.spi, &mut self.dc)
            .unwrap();
        MasterActivation.send(&mut self.spi, &mut self.dc).unwrap();
        self.busy_block().unwrap();

        Ok(GDEH0154D67 {
            spi: self.spi,
            buffer: self.buffer,
            dc: self.dc,
            rst: self.rst,
            busy: self.busy,
            delay: self.delay,
            state: Initialized,
            lut_partial: self.lut_partial,
        })
    }

    /// Releases SPI interface and control pins.
    pub fn release(self) -> Result<(SPI, DC, RST, BSY), Error> {
        Ok((self.spi, self.dc, self.rst, self.busy))
    }
}

impl<SPI, DC, RST, BSY, DLY> GDEH0154D67<SPI, DC, RST, BSY, DLY, Initialized>
where
    SPI: SpiDevice,
    SPI::Bus: SpiBus,
    DC: OutputPin,
    RST: OutputPin,
    BSY: InputPin,
    DLY: DelayUs,
{
    /// Writes into the displays' RAM only those contents of `Buffer`
    /// that have been modified.
    pub fn partial_update(&mut self) -> Result<(), Error> {
        if !self.lut_partial {
            self.set_lut_partial()?;
        }

        for (idx, val) in self
            .buffer
            .flags
            .chunks_mut(8)
            .enumerate()
            .filter(|(_, val)| val.any())
        {
            let (x, y) = index2address(idx);
            SetRamXAddressPosition([x]).send(&mut self.spi, &mut self.dc)?;
            SetRamYAddressPosition([y, 0x00]).send(&mut self.spi, &mut self.dc)?;
            let raw_data = self.buffer.pixels.as_raw_slice();
            WriteRam(&[raw_data[idx]]).send(&mut self.spi, &mut self.dc)?;

            // self.buffer.pixels[(idx * 8)..((idx + 1) * 8)].store(0);
            val.store(0);
        }

        DisplayUpdateControl2([0xC7]).send(&mut self.spi, &mut self.dc)?;
        MasterActivation.send(&mut self.spi, &mut self.dc)?;
        self.busy_block()?;
        Ok(())
    }

    /// Writes all the contents of the `Buffer` into the RAM of the display and
    /// performs a display update.
    pub fn full_update(&mut self) -> Result<(), Error> {
        if self.lut_partial {
            self.set_lut_full()?;
        }
        SetRamXAddressPosition([0x00]).send(&mut self.spi, &mut self.dc)?;
        SetRamYAddressPosition([0x00, 0x00]).send(&mut self.spi, &mut self.dc)?;
        WriteRam(self.buffer.pixels.as_raw_slice()).send(&mut self.spi, &mut self.dc)?;
        DisplayUpdateControl2([0xCF]).send(&mut self.spi, &mut self.dc)?;
        MasterActivation.send(&mut self.spi, &mut self.dc)?;
        self.busy_block().unwrap();
        Ok(())
    }

    pub fn write_raw_lut(&mut self, lut: &[u8]) -> Result<(), Error> {
        WriteLutRegister(&lut).send(&mut self.spi, &mut self.dc)
    }

    pub fn set_lut(&mut self, lut: &[u8]) -> Result<(), Error> {
        self.busy_block()?;

        WriteLutRegister(&lut[0..153]).send(&mut self.spi, &mut self.dc)?;
        WriteLutRegisterEnd([lut[153]]).send(&mut self.spi, &mut self.dc)?;
        self.busy_block()?;

        GateDrivingVoltage([lut[154]]).send(&mut self.spi, &mut self.dc)?;
        SourceDrivingVoltage([lut[155], lut[156], lut[157]]).send(&mut self.spi, &mut self.dc)?;
        WriteVcomRegister([lut[158]]).send(&mut self.spi, &mut self.dc)?;

        Ok(())
    }

    pub fn set_lut_full(&mut self) -> Result<(), Error> {
        self.set_lut(&LUT_FULL_UPDATE)?;
        self.lut_partial = false;
        Ok(())
    }

    pub fn set_lut_partial(&mut self) -> Result<(), Error> {
        self.set_lut(&LUT_PARTIAL_UPDATE)?;

        WriteOtpSelection(&[0x0, 0x0, 0x0, 0x0, 0x0, 0x40, 0x0, 0x0, 0x0, 0x0])
            .send(&mut self.spi, &mut self.dc)
            .unwrap();

        BorderWaveformControl([0x80])
            .send(&mut self.spi, &mut self.dc)
            .unwrap();

        DisplayUpdateControl2([0xc0])
            .send(&mut self.spi, &mut self.dc)
            .unwrap();

        MasterActivation.send(&mut self.spi, &mut self.dc).unwrap();

        Nop.send(&mut self.spi, &mut self.dc).unwrap();
        self.lut_partial = true;
        Ok(())
    }
    /// Releases SPI interface and control pins.
    pub fn release(self) -> Result<(SPI, DC, RST, BSY), Error> {
        let display = self.turn_off().unwrap();
        Ok((display.spi, display.dc, display.rst, display.busy))
    }
}

impl<SPI, DC, RST, BSY, DLY, S> GDEH0154D67<SPI, DC, RST, BSY, DLY, S>
where
    SPI: SpiDevice,
    SPI::Bus: SpiBus,
    DC: OutputPin,
    RST: OutputPin,
    BSY: InputPin,
    DLY: DelayUs,
{
    /// Sends the display into deep sleep mode.
    pub fn turn_off(
        mut self,
    ) -> Result<GDEH0154D67<SPI, DC, RST, BSY, DLY, NotInitialized>, Error> {
        DeepSleepMode([0x1])
            .send(&mut self.spi, &mut self.dc)
            .unwrap();

        Ok(GDEH0154D67 {
            spi: self.spi,
            buffer: self.buffer,
            dc: self.dc,
            rst: self.rst,
            busy: self.busy,
            delay: self.delay,
            state: NotInitialized,
            lut_partial: self.lut_partial,
        })
    }

    /// Performs a hardware reset of the display.
    fn reset(mut self) -> Result<GDEH0154D67<SPI, DC, RST, BSY, DLY, NotInitialized>, Error> {
        self.rst.set_low().unwrap();
        DelayUs::delay_ms(&mut self.delay, 10).unwrap();
        self.rst.set_high().unwrap();
        DelayUs::delay_ms(&mut self.delay, 10).unwrap();
        SwReset.send(&mut self.spi, &mut self.dc).unwrap();
        self.busy_block().unwrap();
        Ok(GDEH0154D67 {
            spi: self.spi,
            buffer: self.buffer,
            dc: self.dc,
            rst: self.rst,
            busy: self.busy,
            delay: self.delay,
            state: NotInitialized,
            lut_partial: false,
        })
    }

    /// Performs a busy block while the display is updating.
    fn busy_block(&mut self) -> Result<(), Error> {
        while self.busy.is_high().unwrap() {
            DelayUs::delay_ms(&mut self.delay, 10).unwrap();
        }
        Ok(())
    }
}

impl<SPI, DC, RST, BSY, DLY> DrawTarget for GDEH0154D67<SPI, DC, RST, BSY, DLY, Initialized>
where
    SPI: SpiDevice,
    SPI::Bus: SpiBus,
    DC: OutputPin,
    RST: OutputPin,
    BSY: InputPin,
    DLY: DelayUs,
{
    type Color = BinaryColor;
    type Error = Error;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(coord, color) in pixels.into_iter() {
            // Check if the pixel coordinates are out of bounds (negative or greater than
            // (200, 200)). `DrawTarget` implementation are required to discard any out of bounds
            // pixels without returning an error or causing a panic.
            if let Ok((x @ 0..=199, y @ 0..=199)) = coord.try_into() {
                // Calculate the index in the buffer.
                let index = pixel2buffer(x, y) as usize;
                self.buffer.pixels.set(index, color.is_on());
                self.buffer.flags.set(index, true);
            }
        }
        Ok(())
    }
}

impl<SPI, DC, RST, BSY, DLY> OriginDimensions for GDEH0154D67<SPI, DC, RST, BSY, DLY, Initialized> {
    fn size(&self) -> Size {
        Size::new(200, 200)
    }
}

fn pixel2buffer(x: u32, y: u32) -> usize {
    (x + y * 200).try_into().unwrap()
}

fn index2address(i: usize) -> (u8, u8) {
    ((i % 25).try_into().unwrap(), (i / 25).try_into().unwrap())
}
