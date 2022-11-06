//! This module contains the commands and all the logic necessary to send them to the display.
//! Currently is only possible to communicate with the display through SPI protocol. 
use crate::error::Error;

use embedded_hal::spi::blocking::{SpiBus, SpiDevice};
use embedded_hal::digital::blocking::OutputPin;

/// Trait that every command must implement. It takes care of sending commands and writing data
/// to the display. Reading from the display is currently not implemented.
pub trait Command {
    /// Sends the command and associated argument (if any) to the display.
    fn send<SPI: SpiDevice>(&self, display: &mut SPI, dc: &mut impl OutputPin) -> Result<(), Error> where SPI::Bus: SpiBus;

    fn send_command<SPI>(cmd: &[u8], display: &mut SPI, dc: &mut  impl OutputPin) -> Result<(), Error>
    where
        SPI: SpiDevice,
        SPI::Bus: SpiBus,
    {
        dc.set_low().unwrap();
        display.write(cmd).unwrap();
        Ok(())
    }

    fn send_data<SPI>(cmd: &[u8], display: &mut SPI, dc: &mut impl OutputPin) -> Result<(), Error>
    where
        SPI: SpiDevice,
        SPI::Bus: SpiBus,
    {
        dc.set_high().unwrap();
        display.write(cmd).unwrap();
        Ok(())
    }
}

/// Convenience macro for implementing the `Command` trait for commands with no arguments.
macro_rules! cmd {
    ($t:ty, $l:literal) => {
        impl Command for $t {
            fn send<SPI>(&self, display: &mut SPI, dc: &mut impl OutputPin) -> Result<(), Error> 
            where
            SPI: SpiDevice,
            SPI::Bus: SpiBus,
            {
                Self::send_command(&[$l], display, dc)?;
                Ok(())
            }
        }
    };
}

/// Convenience macro for implementing the `Command` trait for commands with arguments.
macro_rules! cmd_arg {
    ($t:ty, $l:literal) => {
        impl Command for $t {
            fn send<SPI>(&self, display: &mut SPI, dc: &mut impl OutputPin) -> Result<(), Error>
            where
            SPI: SpiDevice,
            SPI::Bus: SpiBus,
            {
                Self::send_command(&[$l], display, dc)?;
                Self::send_data(&self.0, display, dc)?;
                Ok(())
            }
        }
    };
}

/// Driver Output control.
// 0x01
//   3 Data bytes:
//     A\[7:0\]
//     0.. A\[8\]
//     0.. B\[2:0\]
//   Default: Set A\[8:0\] = 0xc7 and B[2:0\] = 0x0
pub struct DriverOutputControl(/** 3 bytes command argument. */ pub [u8; 3]);
/// Gate Driving Voltage.
// 0x03
pub struct GateDrivingVoltage;
/// Source Driving Voltage.
// 0x04
pub struct SourceDrivingVoltage;
/// Initial Code Setting.
// 0x08
pub struct InitialCodeSetting;
/// Booster Soft start control.
// 0x0c
//   4 Data bytes:
//     1.. A\[6:0\]
//     1.. B\[6:0\]
//     1.. C\[6:0\]
//   Default: A\[7:0\] = 0x8b, B\[7:0\] = 0x9c, C\[7:0\] = 0x96, D\[7:0\] = 0x0f
pub struct BoosterSoftStartControl(/** 4 bytes command argument. */pub [u8; 4]);
/// Deep Sleep Mode Control.
// 0x10
//   1 Data byte:
//   0.. A\[1:0\]
//   Values:
//     A\[1:0\] = 0x0: Normal Mode (POR)
//     A\[1:0\] = 0x1: Enter Deep Sleep Mode 1
//     A\[1:0\] = 0x3: Enter Deep Sleep Mode 2
pub struct DeepSleepMode(/** 1 byte command argument. */ pub [u8; 1]);
/// Data Entry mode setting.
// 0x11
//   1 Data byte: 0.. A\[3:0\]
//   Values:
//     A\[2:0\] = 0x00: (X-, Y-)
//     A\[2:0\] = 0x01: (X+, Y-)
//     A\[2:0\] = 0x02: (X-, Y+)
//     A\[2:0\] = 0x03: (X+, Y+)
pub struct DataEntryModeSetting(/** 1 byte command argument. */ pub [u8; 1]);
/// Software reset.
// 0x12
pub struct SwReset;
/// Temperature Sensor Selection.
// 0x18
pub struct TemperatureSensorSelection;
/// Temperature Sensor Write.
// 0x1a
pub struct TemperatureSensorWrite(/** 2 bytes command argument. */pub [u8; 2]);
/// Display update.
// 0x20
pub struct MasterActivation;
/// Display update control 1.
// 0x21
//   1 Data byte: 0.. A\[3:0\]
//   Values:
//     A\[3:0\] = 0x0: Normal
//     A\[3:0\] = 0x4: Bypass RAM content as 0
//     A\[3:0\] = 0x8: Inverse RAM content
pub struct DisplayUpdateControl1(/** 1 byte command argument. */pub [u8; 1]);
/// Display update control 2.
// 0x22
//   1 Data byte: A\[7:0\]
//   Values:
//     A\[7:0\] = 0xb1: Load LUT with display mode 1
//     A\[7:0\] = 0xc7: Display with DISPLAY Mode 1
//     A\[7:0\] = 0xcf: DISPLAY with DISPLAY Mode 2
pub struct DisplayUpdateControl2(/** 1 byte command argument. */pub [u8; 1]);
/// Write RAM.
// 0x24
// White pixel: 0x01
// Black pixel: 0x00
pub struct WriteRam<'a>(/** Slice command argument. */pub &'a[u8]);
/// Ram-X Address Position.
// 0x44
//   2 Data byte: A\[5:0\], B\[5:0\]
//   Default Values:
//     A\[5:0\] = 0x00: Start
//     B\[5:0\] = 0x15: End
pub struct SetRamXAddressStartEndPosition(/** 2 bytes command argument. */pub [u8; 2]);
/// Ram-Y Address Position.
// 0x45
//   4 Data byte: A\[5:0\], B\[5:0\]
//   Default Values:
//     A\[8:0\] = \[0x00, 0x00\]: Start
//     B\[8:0\] = \[0xc7, 0x00\]: End
pub struct SetRamYAddressStartEndPosition(/** 4 bytes command argument. */pub [u8; 4]);
/// Set Ram X-Address Position.
// 0x4E
pub struct SetRamXAddressPosition(/** 1 byte command argument. */pub [u8; 1]);
/// Set Ram Y-Address Position
// 0x4F
pub struct SetRamYAddressPosition(/** 2 bytes command argument. */pub [u8; 2]);
/// Empty command.
// 0x7f
// Can be used to terminate RAM write/read sequences.
pub struct Nop;

cmd_arg!(DriverOutputControl, 0x01);
cmd!(GateDrivingVoltage, 0x03);
cmd!(SourceDrivingVoltage, 0x04);
cmd!(InitialCodeSetting, 0x08);
cmd_arg!(BoosterSoftStartControl, 0x0c);
cmd_arg!(DeepSleepMode, 0x10);
cmd_arg!(DataEntryModeSetting, 0x11);
cmd!(SwReset, 0x12);
cmd!(TemperatureSensorSelection, 0x18);
cmd!(TemperatureSensorWrite, 0x1a);
cmd!(MasterActivation, 0x20);
cmd_arg!(DisplayUpdateControl1, 0x21);
cmd_arg!(DisplayUpdateControl2, 0x22);
cmd_arg!(WriteRam<'_>, 0x24);
cmd_arg!(SetRamXAddressStartEndPosition, 0x44);
cmd_arg!(SetRamYAddressStartEndPosition, 0x45);
cmd_arg!(SetRamXAddressPosition, 0x4e);
cmd_arg!(SetRamYAddressPosition, 0x4f);
cmd!(Nop, 0x7f);
