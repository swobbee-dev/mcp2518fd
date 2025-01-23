//! MCP2518FD driver for Classic CAN 2.0 via SPI

#![no_std]

mod registers;

use embedded_hal::spi::{Operation, SpiDevice};
use embedded_can::{Frame, StandardId, ExtendedId};
use registers::Register8;

#[derive(Debug)]
pub enum MCPError<SpiErr> {
    /// SPI transfer error from underlying bus.
    SpiError(SpiErr),
    /// Device did not reach the requested mode within the allowed tries.
    ModeFailure,
    /// Cannot configure the device with given parameters.
    ConfigurationFailure,
    /// Baud rate not supported by this minimal example.
    BaudRateNotSupported,
    /// Attempted to transmit a frame but it failed or timed out.
    TransmitFailure,
    /// Received a CAN-FD frame, which is not supported in this driver.
    CanFdNotSupported,
    /// The requested DLC is bigger than 8 bytes (FD-only).
    InvalidDlc,
    /// General unknown error or unexpected device state.
    Unknown,
}

/// Classic CAN (non-FD) data rates for demonstration.
/// In production, you might want a more flexible approach (custom BRP, TSEG).
#[derive(Clone, Copy, Debug)]
pub enum BaudRate {
    Bps250k,
    Bps500k,
    // ... add others as needed
}

/// MCP2518FD operation modes
#[derive(Clone, Copy, Debug)]
pub enum OperationMode {
    Configuration,
    Normal,
    Sleep,
    ListenOnly,
    LoopbackInternal,
    LoopbackExternal,
    // ...
}

/// Configuration object for MCP2518FD
#[derive(Clone, Copy, Debug)]
pub struct Config {
    mode: OperationMode,
    baudrate: BaudRate,
}

#[derive(Clone, Debug)]
pub struct CanFrame {
    extended: bool,
    id: u32,
    dlc: u8,
    data: [u8; 8],
}

fn build_header(opcode: u8, addr: u16) -> [u8; 2] {
    let high = (opcode << 4) | ((addr >> 8) as u8 & 0x0F);
    let low = addr as u8;
    [high, low]
}

impl embedded_can::Frame for CanFrame {
    fn id(&self) -> embedded_can::Id {
        if self.extended {
            embedded_can::Id::Extended(embedded_can::ExtendedId::new(self.id).unwrap())
        } else {
            embedded_can::Id::Standard(embedded_can::StandardId::new(self.id as u16).unwrap())
        }
    }

    fn data(&self) -> &[u8] {
        &self.data[..self.dlc as usize]
    }

    fn dlc(&self) -> usize {
        self.dlc as usize
    }

    fn is_remote_frame(&self) -> bool {
        false
    }
    
    fn new(id: impl Into<embedded_can::Id>, data: &[u8]) -> Option<Self> {
        let (extended, id) = match id.into() {
            embedded_can::Id::Standard(id) => (false, id.as_raw() as u32),
            embedded_can::Id::Extended(id) => (true, id.as_raw()),
        };
        let dlc = data.len() as u8;
        let mut frame = CanFrame {
            extended,
            id,
            dlc,
            data: [0; 8],
        };
        frame.data[..dlc as usize].copy_from_slice(data);
        Some(frame)
    }
    
    fn new_remote(_id: impl Into<embedded_can::Id>, _dlc: usize) -> Option<Self> {
        unimplemented!("Remote frames are not supported in this driver.")
    }
    
    fn is_extended(&self) -> bool {
        self.extended
    }
}

pub struct Mcp2518fd<SPI> {
    spi: SPI,
}

impl<SPI> Mcp2518fd<SPI> {
    pub fn new(spi: SPI) -> Self {
        Self { spi }
    }

    pub fn release(self) -> SPI {
        self.spi
    }
}

impl<SPI, BusErr> Mcp2518fd<SPI>
where
    SPI: SpiDevice<Error = BusErr>,
{
    pub fn test(&mut self) -> Result<(), MCPError<BusErr>> {
        self.reset()?;

        let mut iocon = registers::Iocon0::new();
        iocon.set_xcr_stby_enable(true);
        self.write_subregister(registers::Iocon0::ADDRESS, iocon.into())?;

        Ok(())
    }



    //==========================
    // Low-level register access
    //==========================


    /// Send a RESET command.
    fn reset(&mut self) -> Result<(), MCPError<BusErr>> {
        // CMD = RESET => first byte = 0x00
        let header = build_header(0b0000, 0);
        self.spi.write(&header).map_err(MCPError::SpiError)
    }

    /// Read 8-bit value from a subregister.
    fn read_subregister(&mut self, address: u16) -> Result<u8, MCPError<BusErr>> {
        let header = build_header(0b0011, address);
        
        let mut read_buf = [0u8; 1];
        self.spi.transaction(&mut [
            Operation::Write(&header),
            Operation::Read(&mut read_buf),
        ]).map_err(MCPError::SpiError)?;

        Ok(read_buf[0])
    }

    /// Write 8-bit value to a subregister.
    fn write_subregister(&mut self, address: u16, value: u8) -> Result<(), MCPError<BusErr>> {
        let header = build_header(0b0010, address);
        self.spi.transaction(&mut [
            Operation::Write(&header),
            Operation::Write(&[value]),
        ]).map_err(MCPError::SpiError)
    }

    /// Read 32-bit value from a register.
    fn read_register(&mut self, address: u16) -> Result<u32, MCPError<BusErr>> {
        let header = build_header(0b0011, address);
        
        let mut read_buf = [0u8; 4];
        self.spi.transaction(&mut [
            Operation::Write(&header),
            Operation::Read(&mut read_buf),
        ]).map_err(MCPError::SpiError)?;

        Ok(u32::from_le_bytes(read_buf))
    }

    /// Write 32-bit value to a register.
    fn write_register(&mut self, address: u16, value: u32) -> Result<(), MCPError<BusErr>> {
        let header = build_header(0b0010, address);
        self.spi.transaction(&mut [
            Operation::Write(&header),
            Operation::Write(&value.to_le_bytes()),
        ]).map_err(MCPError::SpiError)
    }

    /// Read a block of RAM from the device.
    fn read_ram(&mut self, address: u16, buffer: &mut [u8]) -> Result<(), MCPError<BusErr>> {
        let header = build_header(0b0011, address);
        self.spi.transaction(&mut [
            Operation::Write(&header),
            Operation::Read(buffer),
        ]).map_err(MCPError::SpiError)
    }

    /// Write a block of RAM to the device.
    fn write_ram(&mut self, address: u16, buffer: &[u8]) -> Result<(), MCPError<BusErr>> {
        let header = build_header(0b0010, address);
        self.spi.transaction(&mut [
            Operation::Write(&header),
            Operation::Write(buffer),
        ]).map_err(MCPError::SpiError)
    }
}

