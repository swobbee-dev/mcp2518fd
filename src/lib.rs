//! MCP2518FD driver for Classic CAN 2.0 via SPI

#![no_std]
#![allow(clippy::new_without_default)]

mod registers;

use core::{
    cell::RefCell,
    marker::PhantomData,
    task::{Poll, Waker},
};

use embedded_can::asynch::{CanRx, CanTx};
use embedded_hal::spi::{Operation, SpiDevice};
use registers::{FromBytes, OperationMode, Register, ToBytes};

#[derive(Debug)]
pub enum MCPError<SpiErr> {
    /// SPI transfer error from underlying bus.
    SpiError(SpiErr),
    /// Configuration could not be applied.
    ConfigurationErrror,
    /// Device is not configured.
    NotConfiguredError,
}

impl<BusErr: core::fmt::Debug> embedded_can::Error for MCPError<BusErr> {
    fn kind(&self) -> embedded_can::ErrorKind {
        use embedded_can::ErrorKind;
        ErrorKind::Other
    }
}

#[derive(Debug)]
pub enum Baudrate {
    Bps250k,
    Bps500k,
}

/// Configuration object for MCP2518FD
#[derive(Debug)]
pub struct Config {
    baudrate: Baudrate,
}

impl Config {
    pub fn new(baudrate: Baudrate) -> Self {
        Self { baudrate }
    }
}

pub struct SharedDriver<SPI>(RefCell<Driver<SPI>>);

impl<SPI> SharedDriver<SPI> {
    pub fn new(driver: Driver<SPI>) -> Self {
        Self(RefCell::new(driver))
    }

    fn with<R>(&self, f: impl FnOnce(&mut Driver<SPI>) -> R) -> R {
        f(&mut self.0.borrow_mut())
    }
}

/// Transmit-only handle
pub struct McpTx<'a, SPI, F> {
    shared: &'a SharedDriver<SPI>,
    _frame: PhantomData<F>,
}

impl<'a, SPI, F> McpTx<'a, SPI, F> {
    /// Constructor for internal use only
    pub fn new(shared: &'a SharedDriver<SPI>) -> Self {
        Self {
            shared,
            _frame: PhantomData,
        }
    }

    pub fn is_message_available(&mut self) -> Result<bool, MCPError<SPI::Error>>
    where
        SPI: SpiDevice,
    {
        self.shared.with(|drv| drv.is_message_available())
    }
}

impl<'a, SPI, FRAME, ERR> CanTx for McpTx<'a, SPI, FRAME>
where
    FRAME: embedded_can::Frame,
    SPI: SpiDevice<Error = ERR>,
    ERR: core::fmt::Debug,
{
    type Frame = FRAME;
    type Error = MCPError<ERR>;

    async fn transmit(
        &mut self,
        frame: &<Self as CanTx>::Frame,
    ) -> Result<(), <Self as CanTx>::Error> {
        self.shared.with(|drv| drv.transmit(frame))
    }
}

/// Receive-only handle
pub struct McpRx<'a, SPI, F> {
    shared: &'a SharedDriver<SPI>,
    _frame: PhantomData<F>,
}

impl<'a, SPI, FRAME> McpRx<'a, SPI, FRAME> {
    pub fn new(shared: &'a SharedDriver<SPI>) -> Self {
        Self {
            shared,
            _frame: PhantomData,
        }
    }

    pub fn is_message_available(&mut self) -> Result<bool, MCPError<SPI::Error>>
    where
        SPI: SpiDevice,
    {
        self.shared.with(|drv| drv.is_message_available())
    }
}

impl<'a, SPI, FRAME, ERR> CanRx for McpRx<'a, SPI, FRAME>
where
    FRAME: embedded_can::Frame,
    SPI: SpiDevice<Error = ERR>,
    ERR: core::fmt::Debug,
{
    type Frame = FRAME;
    type Error = MCPError<ERR>;

    async fn receive(&mut self) -> Result<<Self as CanRx>::Frame, <Self as CanRx>::Error> {
        core::future::poll_fn(|cx| {
            self.shared
                .with(|drv| drv.poll_receive::<FRAME>(cx.waker()))
        })
        .await
    }
}

/// Handle to call when the interrupt pin is triggered.
pub struct InterruptHandle<'a, SPI> {
    shared: &'a SharedDriver<SPI>,
}

impl<'a, SPI> InterruptHandle<'a, SPI>
where
    SPI: SpiDevice,
{
    fn new(shared: &'a SharedDriver<SPI>) -> Self {
        Self { shared }
    }

    pub fn handle_interrupt(&mut self) {
        self.shared.with(|drv| drv.handle_interrupt());
    }
}

pub struct Mcp2518fd<SPI, FRAME> {
    shared: SharedDriver<SPI>,
    _frame: PhantomData<FRAME>,
}

impl<SPI, FRAME> Mcp2518fd<SPI, FRAME> {
    pub fn new(spi: SPI) -> Self {
        let driver = Driver::new(spi);
        Self {
            shared: SharedDriver::new(driver),
            _frame: PhantomData,
        }
    }

    pub fn configure(&mut self, config: &Config) -> Result<(), MCPError<SPI::Error>>
    where
        SPI: SpiDevice,
    {
        self.shared.with(|drv| drv.configure(config))
    }

    /// Split into separate TX/RX handles that implement the embedded-can "asynch" traits
    pub fn split(&mut self) -> (McpTx<'_, SPI, FRAME>, McpRx<'_, SPI, FRAME>, InterruptHandle<'_, SPI>)
    where
        SPI: SpiDevice,
    {
        (
            McpTx::new(&self.shared),
            McpRx::new(&self.shared),
            InterruptHandle::new(&self.shared),
        )
    }
}

fn build_header(opcode: u8, addr: u16) -> [u8; 2] {
    let high = (opcode << 4) | ((addr >> 8) as u8 & 0x0F);
    let low = addr as u8;
    [high, low]
}

pub struct Driver<SPI> {
    spi: SPI,
    configured: bool,
    rx_waker: RefCell<Option<Waker>>,
}

impl<SPI> Driver<SPI> {
    pub fn new(spi: SPI) -> Self {
        Self {
            spi,
            configured: false,
            rx_waker: RefCell::new(None),
        }
    }

    pub fn release(self) -> SPI {
        self.spi
    }
}

impl<SPI, BusErr> Driver<SPI>
where
    SPI: SpiDevice<Error = BusErr>,
{
    pub fn configure(&mut self, config: &Config) -> Result<(), MCPError<BusErr>> {
        // Device will be reset to configuration mode
        self.reset()?;
        assert!(self.read_mode()? == OperationMode::Configuration);

        // Disable Clock Output Divisor for 20mHz crystal
        self.write_register(registers::OscConfig::new().with_clkodiv(0))?;
        assert!(self.read_register::<registers::OscStatus>()?.osc_ready());

        // Configure GPIOs
        let mut iocon = registers::Iocon3::new();
        iocon.set_pin_mode0(true); // use as GPIO0
        iocon.set_pin_mode1(true); // use as GPIO1
        self.write_register(iocon)?;

        // Configure CAN control register
        let mut con2 = registers::CiCon2::new();
        con2.set_restrict_re_tx_attempts(true);
        self.write_register(con2)?;

        let mut con0 = registers::CiCon0::new();
        con0.set_protocol_exception_event_disable(true);
        con0.set_iso_crc_enable(true);
        self.write_register(con0)?;

        // Setup FIFOs
        self.configure_fifo_rx(0)?;
        self.configure_fifo_tx(1)?;

        for filter_index in 0..32 {
            self.write_register_with_index(registers::CiFltcon::new(), filter_index)?;
            self.write_register_with_index(registers::CiMask::new(), filter_index)?;
            self.write_register_with_index(registers::CiFltobj::new(), filter_index)?;
        }

        // Configure FILTER0 to take move all received messages into FIFO 1
        self.write_register_with_index(
            registers::CiFltcon::new()
                .with_buffer_pointer(1)
                .with_enable(true),
            0,
        )?;

        // Enable RX interrupt
        self.write_register(registers::CanIntEnables::new().with_rxie(true))?;

        // Set the baudrate
        let (tseg1, tseg2) = match config.baudrate {
            Baudrate::Bps250k => (62, 15), // BT = 4us, (1 + (1+TSEG1) + (1+TSEG2)) = BT/TQ = 80
            Baudrate::Bps500k => (30, 7),  // BT = 2us, (1 + (1+TSEG1) + (1+TSEG2)) = BT/TQ = 40
        };
        let sjw = tseg2; // maximize SJW while SJW <= min(TSEG1, TSEG2)

        let nbtcfg = registers::CiNbtcfg::new()
            .with_sjw(sjw)
            .with_tseg1(tseg1)
            .with_tseg2(tseg2)
            .with_brp(0);
        self.write_register(nbtcfg)?;

        let dbtcfg = registers::CiDbtcfg::from(u32::from(nbtcfg)); // NBTCFG and DBTCFG are the same in CAN 2.0
        self.write_register(dbtcfg)?;

        self.write_register(
            registers::CiTdc::new()
                .with_tdc_mode(2) // Transmitter Delay Compensation Mode Auto
                .with_tdc_offset(tseg1) // Transmitter Delay Compensation Offset bits
                .with_edge_filter_enable(true), // Enable Edge Filtering during Bus Integration state
        )?;

        self.set_mode(OperationMode::NormalCan2_0)?;
        self.configured = true;
        Ok(())
    }

    pub fn transmit(&mut self, msg: &impl embedded_can::Frame) -> Result<(), MCPError<BusErr>> {
        let ram_addr = u32::from(self.read_register_with_index::<registers::CiFIFOUA>(1)?) as u16;

        let (t0, mut t1) = match msg.id() {
            embedded_can::Id::Standard(id) => (id.as_raw() as u32 & 0x7FF, 0),
            embedded_can::Id::Extended(id) => (
                (id.as_raw() >> 18 & 0x7FF) | (id.as_raw() & 0x3FFFF) << 11,
                1 << 4,
            ),
        };
        t1 |= msg.dlc() as u32;

        let buf = {
            let mut buf = [0u8; 16];
            buf[..4].copy_from_slice(&t0.to_le_bytes());
            buf[4..8].copy_from_slice(&t1.to_le_bytes());
            buf[8..8 + msg.dlc()].copy_from_slice(msg.data());
            buf
        };

        self.write_ram(ram_addr, &buf)?;

        self.write_register_with_index(
            registers::CiFifoCon1::new()
                .with_uinc(true)
                .with_txreq(true),
            1,
        )?;

        Ok(())
    }

    pub fn is_message_available(&mut self) -> Result<bool, MCPError<BusErr>> {
        let fifosta = self.read_register::<registers::CiFifoStaRx0>()?;
        Ok(fifosta.rx_not_empty_if())
    }

    pub fn receive<FRAME>(&mut self) -> Result<FRAME, MCPError<BusErr>>
    where
        FRAME: embedded_can::Frame,
    {
        let ram_addr = u32::from(self.read_register_with_index::<registers::CiFIFOUA>(0)?) as u16;
        let mut buf = [0u8; 16];
        self.read_ram(ram_addr, &mut buf)?;

        let r0 = u32::from_le_bytes(buf[..4].try_into().unwrap());
        let r1 = u32::from_le_bytes(buf[4..8].try_into().unwrap());
        let dlc = r1 as usize & 0xF;

        let id = if r1 & (1 << 4) != 0 {
            embedded_can::Id::Extended(
                embedded_can::ExtendedId::new((r0 >> 11) & 0x3FFFF | (r0 & 0x7FF) << 18).unwrap(),
            )
        } else {
            embedded_can::Id::Standard(embedded_can::StandardId::new(r0 as u16).unwrap())
        };
        let data = &buf[8..8 + dlc];
        let frame = FRAME::new(id, data).unwrap();

        self.write_register_with_index::<registers::CiFifoCon1>(
            registers::CiFifoCon1::new().with_uinc(true)
        , 0)?; // increment FIFO pointer

        Ok(frame)
    }

    //==========================
    // Private helper functions
    //==========================

    fn configure_fifo_tx(&mut self, fifo_idx: u16) -> Result<(), MCPError<BusErr>> {
        let mut fifo_con = registers::CiFifoConTx::new();
        fifo_con.set_fifo_size(0b11111);
        fifo_con.set_tx_enable(true);
        self.write_register_with_index(fifo_con, fifo_idx)?;
        Ok(())
    }

    fn configure_fifo_rx(&mut self, fifo_idx: u16) -> Result<(), MCPError<BusErr>> {
        let mut fifo_con = registers::CiFifoConRx::new();
        fifo_con.set_fifo_size(0b11111);
        fifo_con.set_rx_not_empty_ie(true);
        self.write_register_with_index(fifo_con, fifo_idx)?;
        Ok(())
    }

    /// Read the current operation mode
    pub fn read_mode(&mut self) -> Result<OperationMode, MCPError<BusErr>> {
        // The OPMOD bits are bits [5..7] in CiCON+2 (read-only).
        let cicon2 = self.read_register::<registers::CiCon2>()?;
        Ok(cicon2.op_mode())
    }

    /// Set the operation mode
    fn set_mode(&mut self, mode: OperationMode) -> Result<(), MCPError<BusErr>> {
        let cicon3 = registers::CiCon3::new().with_request_op_mode(mode.into());
        self.write_register(cicon3)?;
        if self.read_mode()? == mode {
            Ok(())
        } else {
            Err(MCPError::ConfigurationErrror)
        }
    }

    //==========================
    // Low-level register access
    //==========================

    /// Send a RESET command.
    pub fn reset(&mut self) -> Result<(), MCPError<BusErr>> {
        // CMD = RESET => first byte = 0x00
        let header = build_header(0b0000, 0);
        self.spi.write(&header).map_err(MCPError::SpiError)
    }

    fn read_register<R: Register>(&mut self) -> Result<R, MCPError<BusErr>> {
        self.read_register_with_index::<R>(0)
    }

    fn read_register_with_index<R: Register>(&mut self, index: u16) -> Result<R, MCPError<BusErr>> {
        let header = build_header(0b0011, R::address(index));
        let mut read_buf = [0u8; 4];
        let byte_count = core::mem::size_of::<R::V>();
        self.spi
            .transaction(&mut [
                Operation::Write(&header),
                Operation::Read(&mut read_buf[..byte_count]),
            ])
            .map_err(MCPError::SpiError)?;

        let value = R::V::from_bytes(&read_buf);

        Ok(R::from(value))
    }

    /// Read 8-bit value from a register.
    fn write_register<R: Register>(&mut self, reg: R) -> Result<(), MCPError<BusErr>> {
        self.write_register_with_index::<R>(reg, 0)
    }

    /// Write 32-bit value to a register with offset.
    fn write_register_with_index<R: Register>(
        &mut self,
        reg: R,
        index: u16,
    ) -> Result<(), MCPError<BusErr>> {
        let header = build_header(0b0010, R::address(index));
        self.spi
            .transaction(&mut [
                Operation::Write(&header),
                Operation::Write(reg.into().to_bytes().as_ref()),
            ])
            .map_err(MCPError::SpiError)
    }

    /// Read a block of RAM from the device.
    fn read_ram(&mut self, address: u16, buffer: &mut [u8]) -> Result<(), MCPError<BusErr>> {
        assert!(buffer.len().is_multiple_of(4));
        let header = build_header(0b0011, 0x400 + address);
        self.spi
            .transaction(&mut [Operation::Write(&header), Operation::Read(buffer)])
            .map_err(MCPError::SpiError)
    }

    /// Write a block of RAM to the device.
    fn write_ram(&mut self, address: u16, buffer: &[u8]) -> Result<(), MCPError<BusErr>> {
        assert!(buffer.len().is_multiple_of(4));
        let header = build_header(0b0010, 0x400 + address);
        self.spi
            .transaction(&mut [Operation::Write(&header), Operation::Write(buffer)])
            .map_err(MCPError::SpiError)
    }

    pub fn poll_receive<FRAME>(&mut self, waker: &Waker) -> Poll<Result<FRAME, MCPError<BusErr>>>
    where
        FRAME: embedded_can::Frame,
    {
        // First, check if there's a message ready:
        match self.is_message_available() {
            Ok(true) => {
                // If we have a message, read it right away.
                let frame = self.receive()?;
                Poll::Ready(Ok(frame))
            }
            Ok(false) => {
                // No message yet, store the waker and return Pending.
                *self.rx_waker.borrow_mut() = Some(waker.clone());
                Poll::Pending
            }
            Err(e) => Poll::Ready(Err(e)),
        }
    }

    /// Called from an ISR or elsewhere when the interrupt pin is triggered.
    /// Wakes the stored waker (if any).
    pub fn handle_interrupt(&mut self) {
        if let Some(waker) = self.rx_waker.get_mut().take() {
            waker.wake();
        }
    }
}
