use modular_bitfield::prelude::*;

/// A 32-bit register trait
pub trait Register32: From<u32> + Into<u32> {
    const ADDRESS: u16;
}

/// An 8-bit register trait
pub trait Register8: From<u8> + Into<u8> {
    const ADDRESS: u16;
}

/// A 16-bit register trait
pub trait Register16: From<u16> + Into<u16> {
    const ADDRESS: u16;
}

// -----------------------------------------------------------------------------
// OperationMode – 3-bit enum

#[derive(Clone, Copy, Debug, PartialEq, BitfieldSpecifier)]
pub enum OperationMode {
    NormalCanFd = 0,
    Sleep = 1,
    LoopbackInt = 2,
    ListenOnly = 3,
    Configuration = 4,
    LoopbackExt = 5,
    NormalCan2_0 = 6,
    Restricted = 7,
}

impl Into<u8> for OperationMode {
    fn into(self) -> u8 {
        self as u8
    }
}

// -----------------------------------------------------------------------------
// CiCON – CAN Control Register

#[bitfield(bits = 8)]
#[derive(BitfieldSpecifier)]
#[repr(u8)]
pub struct CiCon0 {
    pub d_net_filter_count: B5,
    pub iso_crc_enable: bool,
    pub protocol_exception_event_disable: bool,
    #[skip]
    __: B1,
}

#[bitfield(bits = 8)]
#[derive(BitfieldSpecifier)]
#[repr(u8)]
pub struct CiCon1 {
    pub wake_up_filter_enable: bool,
    pub wake_up_filter_time: B2,
    #[skip]
    __: B1,
    pub bit_rate_switch_disable: bool,
    #[skip]
    __: B3,
}

#[bitfield(bits = 8)]
#[derive(BitfieldSpecifier)]
#[repr(u8)]
pub struct CiCon2 {
    pub restrict_re_tx_attempts: bool,
    pub esi_in_gateway_mode: bool,
    pub system_error_to_listen_only: bool,
    pub store_in_tef: bool,
    pub txq_enable: bool,
    #[bits = 3]
    pub op_mode: OperationMode,
}

#[bitfield(bits = 8)]
#[derive(BitfieldSpecifier)]
#[repr(u8)]
pub struct CiCon3 {
    #[bits = 3]
    pub request_op_mode: B3,
    pub abort_all_tx: bool,
    #[bits = 4]
    pub tx_band_width_sharing: B4,
}

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct CiCon {
    pub con0: CiCon0,
    pub con1: CiCon1,
    pub con2: CiCon2,
    pub con3: CiCon3,
}

impl Register32 for CiCon {
    const ADDRESS: u16 = 0x0000;
}

impl Register8 for CiCon0 {
    const ADDRESS: u16 = CiCon::ADDRESS;
}
impl Register8 for CiCon1 {
    const ADDRESS: u16 = CiCon::ADDRESS + 1;
}
impl Register8 for CiCon2 {
    const ADDRESS: u16 = CiCon::ADDRESS + 2;
}
impl Register8 for CiCon3 {
    const ADDRESS: u16 = CiCon::ADDRESS + 3;
}

// -----------------------------------------------------------------------------
// CiNBTCFG – Nominal Bit Time Configuration Register

#[bitfield(bits = 8)]
#[derive(BitfieldSpecifier)]
#[repr(u8)]
pub struct CiNbtcfg0 {
    #[bits = 7]
    pub sjw: B7,
    #[skip]
    __: B1,
}

#[bitfield(bits = 8)]
#[derive(BitfieldSpecifier)]
#[repr(u8)]
pub struct CiNbtcfg1 {
    #[bits = 7]
    pub tseg2: B7,
    #[skip]
    __: B1,
}

#[bitfield(bits = 8)]
#[derive(BitfieldSpecifier)]
#[repr(u8)]
pub struct CiNbtcfg2 {
    #[bits = 8]
    pub tseg1: u8,
}

#[bitfield(bits = 8)]
#[derive(BitfieldSpecifier)]
#[repr(u8)]
pub struct CiNbtcfg3 {
    #[bits = 8]
    pub brp: u8,
}

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct CiNbtcfg {
    pub nbtcfg0: CiNbtcfg0,
    pub nbtcfg1: CiNbtcfg1,
    pub nbtcfg2: CiNbtcfg2,
    pub nbtcfg3: CiNbtcfg3,
}

impl Register32 for CiNbtcfg {
    const ADDRESS: u16 = 0x0004;
}

impl Register8 for CiNbtcfg0 {
    const ADDRESS: u16 = CiNbtcfg::ADDRESS;
}
impl Register8 for CiNbtcfg1 {
    const ADDRESS: u16 = CiNbtcfg::ADDRESS + 1;
}
impl Register8 for CiNbtcfg2 {
    const ADDRESS: u16 = CiNbtcfg::ADDRESS + 2;
}
impl Register8 for CiNbtcfg3 {
    const ADDRESS: u16 = CiNbtcfg::ADDRESS + 3;
}

// -----------------------------------------------------------------------------
// CiDBTCFG – Data Bit Time Configuration Register (32 bits)

#[bitfield(bits = 8)]
#[derive(BitfieldSpecifier)]
#[repr(u8)]
pub struct CiDbtcfg0 {
    #[bits = 4]
    pub sjw: B4,
    #[skip]
    __: B4,
}

#[bitfield(bits = 8)]
#[derive(BitfieldSpecifier)]
#[repr(u8)]
pub struct CiDbtcfg1 {
    #[bits = 4]
    pub tseg2: B4,
    #[skip]
    __: B4,
}

#[bitfield(bits = 8)]
#[derive(BitfieldSpecifier)]
#[repr(u8)]
pub struct CiDbtcfg2 {
    #[bits = 5]
    pub tseg1: B5,
    #[skip]
    __: B3,
}

#[bitfield(bits = 8)]
#[derive(BitfieldSpecifier)]
#[repr(u8)]
pub struct CiDbtcfg3 {
    #[bits = 8]
    pub brp: u8,
}

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct CiDbtcfg {
    pub dbtcfg0: CiDbtcfg0,
    pub dbtcfg1: CiDbtcfg1,
    pub dbtcfg2: CiDbtcfg2,
    pub dbtcfg3: CiDbtcfg3,
}

impl Register32 for CiDbtcfg {
    const ADDRESS: u16 = 0x0008;
}

impl Register8 for CiDbtcfg0 {
    const ADDRESS: u16 = CiDbtcfg::ADDRESS;
}
impl Register8 for CiDbtcfg1 {
    const ADDRESS: u16 = CiDbtcfg::ADDRESS + 1;
}
impl Register8 for CiDbtcfg2 {
    const ADDRESS: u16 = CiDbtcfg::ADDRESS + 2;
}
impl Register8 for CiDbtcfg3 {
    const ADDRESS: u16 = CiDbtcfg::ADDRESS + 3;
}

// -----------------------------------------------------------------------------
// CiTDC – Transmitter Delay Compensation Register (32 bits)

#[bitfield(bits = 8)]
#[derive(BitfieldSpecifier)]
#[repr(u8)]
pub struct CiTdc0 {
    #[bits = 6]
    pub tdc_value: B6,
    #[skip]
    __: B2,
}

#[bitfield(bits = 8)]
#[derive(BitfieldSpecifier)]
#[repr(u8)]
pub struct CiTdc1 {
    #[bits = 7]
    pub tdc_offset: B7,
    #[skip]
    __: B1,
}

#[bitfield(bits = 8)]
#[derive(BitfieldSpecifier)]
#[repr(u8)]
pub struct CiTdc2 {
    #[bits = 2]
    pub tdc_mode: B2,
    #[skip]
    __: B6,
}

#[bitfield(bits = 8)]
#[derive(BitfieldSpecifier)]
#[repr(u8)]
pub struct CiTdc3 {
    pub sid11_enable: bool,
    pub edge_filter_enable: bool,
    #[skip]
    __: B6,
}

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct CiTdc {
    pub tdc0: CiTdc0,
    pub tdc1: CiTdc1,
    pub tdc2: CiTdc2,
    pub tdc3: CiTdc3,
}

impl Register32 for CiTdc {
    const ADDRESS: u16 = 0x000C;
}

impl Register8 for CiTdc0 {
    const ADDRESS: u16 = CiTdc::ADDRESS;
}
impl Register8 for CiTdc1 {
    const ADDRESS: u16 = CiTdc::ADDRESS + 1;
}
impl Register8 for CiTdc2 {
    const ADDRESS: u16 = CiTdc::ADDRESS + 2;
}
impl Register8 for CiTdc3 {
    const ADDRESS: u16 = CiTdc::ADDRESS + 3;
}

// -----------------------------------------------------------------------------
// CiTSCON – Time Stamp Configuration Register (32 bits)
//
// The tbc_prescaler is 10 bits, which crosses a single byte boundary.
// So we can represent the first 16 bits as one 16-bit subfield,
// and the second 16 bits as another 16-bit subfield.

#[bitfield(bits = 16)]
#[derive(BitfieldSpecifier)]
#[repr(u16)]
pub struct CiTscon0 {
    #[bits = 10]
    pub tbc_prescaler: B10,
    #[skip]
    __: B6,
}

#[bitfield(bits = 16)]
#[derive(BitfieldSpecifier)]
#[repr(u16)]
pub struct CiTscon1 {
    pub tbc_enable: bool,
    pub time_stamp_eof: bool,
    #[skip]
    __: B14,
}

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct CiTscon {
    pub tscon0: CiTscon0,
    pub tscon1: CiTscon1,
}

impl Register32 for CiTscon {
    const ADDRESS: u16 = 0x0014;
}

impl Register16 for CiTscon0 {
    const ADDRESS: u16 = CiTscon::ADDRESS;
}
impl Register16 for CiTscon1 {
    const ADDRESS: u16 = CiTscon::ADDRESS + 2;
}

// -----------------------------------------------------------------------------
// CiVEC – Interrupt Vector Register (32 bits)

#[bitfield(bits = 8)]
#[derive(BitfieldSpecifier)]
#[repr(u8)]
pub struct CiVec0 {
    #[bits = 7]
    pub icode: B7,
    #[skip]
    __: B1,
}

#[bitfield(bits = 8)]
#[derive(BitfieldSpecifier)]
#[repr(u8)]
pub struct CiVec1 {
    #[bits = 5]
    pub filter_hit: B5,
    #[skip]
    __: B3,
}

#[bitfield(bits = 8)]
#[derive(BitfieldSpecifier)]
#[repr(u8)]
pub struct CiVec2 {
    #[bits = 7]
    pub txcode: B7,
    #[skip]
    __: B1,
}

#[bitfield(bits = 8)]
#[derive(BitfieldSpecifier)]
#[repr(u8)]
pub struct CiVec3 {
    #[bits = 7]
    pub rxcode: B7,
    #[skip]
    __: B1,
}

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct CiVec {
    pub vec0: CiVec0,
    pub vec1: CiVec1,
    pub vec2: CiVec2,
    pub vec3: CiVec3,
}

impl Register32 for CiVec {
    const ADDRESS: u16 = 0x0018;
}

impl Register8 for CiVec0 {
    const ADDRESS: u16 = CiVec::ADDRESS;
}
impl Register8 for CiVec1 {
    const ADDRESS: u16 = CiVec::ADDRESS + 1;
}
impl Register8 for CiVec2 {
    const ADDRESS: u16 = CiVec::ADDRESS + 2;
}
impl Register8 for CiVec3 {
    const ADDRESS: u16 = CiVec::ADDRESS + 3;
}

// -----------------------------------------------------------------------------
// CAN_INT_FLAGS (16 bits)

#[bitfield(bits = 16)]
#[derive(BitfieldSpecifier)]
#[repr(u16)]
pub struct CanIntFlags {
    pub txif: bool,
    pub rxif: bool,
    pub tbcif: bool,
    pub modif: bool,
    pub tefif: bool,
    #[skip]
    __: B3,
    pub eccif: bool,
    pub spicrcif: bool,
    pub txatif: bool,
    pub rxovif: bool,
    pub serrif: bool,
    pub cerrif: bool,
    pub wakif: bool,
    pub ivmif: bool,
}

// -----------------------------------------------------------------------------
// CAN_INT_ENABLES (16 bits)

#[bitfield(bits = 16)]
#[derive(BitfieldSpecifier)]
#[repr(u16)]
pub struct CanIntEnables {
    pub txie: bool,
    pub rxie: bool,
    pub tbcie: bool,
    pub modie: bool,
    pub tefie: bool,
    #[skip]
    __: B3,
    pub eccie: bool,
    pub spicrcie: bool,
    pub txatie: bool,
    pub rxovie: bool,
    pub serrie: bool,
    pub cerrie: bool,
    pub wakie: bool,
    pub ivmie: bool,
}

// -----------------------------------------------------------------------------
// CiINT – Interrupt Register (32 bits), combining FLAGS + ENABLES

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct CiInt {
    #[bits = 16]
    pub iflags: CanIntFlags,
    #[bits = 16]
    pub ienable: CanIntEnables,
}

impl Register32 for CiInt {
    const ADDRESS: u16 = 0x001C;
}

impl Register16 for CanIntFlags {
    const ADDRESS: u16 = CiInt::ADDRESS;
}

impl Register16 for CanIntEnables {
    const ADDRESS: u16 = CiInt::ADDRESS + 2;
}

// -----------------------------------------------------------------------------
// CiTREC – Transmit/Receive Error Count Register (32 bits)
//   bits 0..7 => rx_error_count
//   bits 8..15 => tx_error_count
//   bits 16..31 => error state bits

#[bitfield(bits = 16)]
#[derive(BitfieldSpecifier)]
#[repr(u16)]
pub struct CiTrecState {
    pub error_state_warning: bool,
    pub rx_error_state_warning: bool,
    pub tx_error_state_warning: bool,
    pub rx_error_state_passive: bool,
    pub tx_error_state_passive: bool,
    pub tx_error_state_bus_off: bool,
    #[skip]
    __: B10,
}

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct CiTrec {
    pub rx_error_count: u8,
    pub tx_error_count: u8,
    pub state: CiTrecState,
}

impl Register32 for CiTrec {
    const ADDRESS: u16 = 0x0034;
}

impl Register16 for CiTrecState {
    const ADDRESS: u16 = CiTrec::ADDRESS + 2;
}

// -----------------------------------------------------------------------------
// CiBDIAG0 – Diagnostic Register 0

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct CiBdiag0 {
    #[bits = 8]
    pub n_rx_error_count: u8,
    #[bits = 8]
    pub n_tx_error_count: u8,
    #[bits = 8]
    pub d_rx_error_count: u8,
    #[bits = 8]
    pub d_tx_error_count: u8,
}

impl Register32 for CiBdiag0 {
    const ADDRESS: u16 = 0x0038;
}

// -----------------------------------------------------------------------------
// CiBDIAG1 – Diagnostic Register 1

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct CiBdiag1 {
    #[bits = 16]
    pub error_free_msg_count: u16,

    pub n_bit0_error: bool,
    pub n_bit1_error: bool,
    pub n_ack_error: bool,
    pub n_form_error: bool,
    pub n_stuff_error: bool,
    pub n_crc_error: bool,
    #[skip]
    __: B1,
    pub txbo_error: bool,
    pub d_bit0_error: bool,
    pub d_bit1_error: bool,
    pub d_ack_error: bool,
    pub d_form_error: bool,
    pub d_stuff_error: bool,
    pub d_crc_error: bool,
    pub esi: bool,
    #[skip]
    __: B1,
}

impl Register32 for CiBdiag1 {
    const ADDRESS: u16 = 0x003C;
}

// -----------------------------------------------------------------------------
// CiTEFCON – Transmit Event FIFO Control Register (32 bits)

#[bitfield(bits = 8)]
#[derive(BitfieldSpecifier)]
#[repr(u8)]
pub struct CiTefcon0 {
    pub tefneie: bool,
    pub tefhfie: bool,
    pub teffulie: bool,
    pub tefovie: bool,
    #[skip]
    __: B1,
    pub time_stamp_enable: bool,
    #[skip]
    __: B2,
}

#[bitfield(bits = 8)]
#[derive(BitfieldSpecifier)]
#[repr(u8)]
pub struct CiTefcon1 {
    pub uinc: bool,
    #[skip]
    __: B1,
    pub freset: bool,
    #[skip]
    __: B5,
}

#[bitfield(bits = 8)]
#[derive(BitfieldSpecifier)]
#[repr(u8)]
pub struct CiTefcon2 {
    #[skip]
    __: B8,
}

#[bitfield(bits = 8)]
#[derive(BitfieldSpecifier)]
#[repr(u8)]
pub struct CiTefcon3 {
    #[bits = 5]
    pub fifo_size: B5,
    #[skip]
    __: B3,
}

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct CiTefcon {
    pub citefcon0: CiTefcon0,
    pub citefcon1: CiTefcon1,
    pub citefcon2: CiTefcon2,
    pub citefcon3: CiTefcon3,
}

impl Register32 for CiTefcon {
    const ADDRESS: u16 = 0x0040;
}

impl Register8 for CiTefcon0 {
    const ADDRESS: u16 = CiTefcon::ADDRESS;
}

impl Register8 for CiTefcon1 {
    const ADDRESS: u16 = CiTefcon::ADDRESS + 1;
}

impl Register8 for CiTefcon2 {
    const ADDRESS: u16 = CiTefcon::ADDRESS + 2;
}

impl Register8 for CiTefcon3 {
    const ADDRESS: u16 = CiTefcon::ADDRESS + 3;
}

// -----------------------------------------------------------------------------
// CiTEFSTA – Transmit Event FIFO Status Register (32 bits, mostly bits in first byte)

#[bitfield(bits = 8)]
#[derive(BitfieldSpecifier)]
#[repr(u8)]
pub struct CiTefsta0 {
    pub tef_not_empty_if: bool,
    pub tef_half_full_if: bool,
    pub tef_full_if: bool,
    pub tefovif: bool,
    #[skip]
    __: B4,
}

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct CiTefsta {
    pub sta0: CiTefsta0,
    #[skip]
    __: B24,
}

impl Register32 for CiTefsta {
    const ADDRESS: u16 = 0x0044;
}

impl Register8 for CiTefsta0 {
    const ADDRESS: u16 = CiTefsta::ADDRESS;
}

// -----------------------------------------------------------------------------
// CiTXQCON – Transmit Queue Control Register (32 bits)

#[bitfield(bits = 8)]
#[derive(BitfieldSpecifier)]
#[repr(u8)]
pub struct CiTxqcon0 {
    pub tx_not_full_ie: bool,
    #[skip]
    __: B1,
    pub tx_empty_ie: bool,
    #[skip]
    __: B1,
    pub tx_attempt_ie: bool,
    #[skip]
    __: B2,
    pub tx_enable: bool,
}

#[bitfield(bits = 8)]
#[derive(BitfieldSpecifier)]
#[repr(u8)]
pub struct CiTxqcon1 {
    pub uinc: bool,
    pub tx_request: bool,
    pub freset: bool,
    #[skip]
    __: B5,
}

#[bitfield(bits = 8)]
#[derive(BitfieldSpecifier)]
#[repr(u8)]
pub struct CiTxqcon2 {
    #[bits = 5]
    pub tx_priority: B5,
    #[bits = 2]
    pub tx_attempts: B2,
    #[skip]
    __: B1,
}

#[bitfield(bits = 8)]
#[derive(BitfieldSpecifier)]
#[repr(u8)]
pub struct CiTxqcon3 {
    #[bits = 5]
    pub fifo_size: B5,
    #[bits = 3]
    pub payload_size: B3,
}

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct CiTxqcon {
    pub txqcon0: CiTxqcon0,
    pub txqcon1: CiTxqcon1,
    pub txqcon2: CiTxqcon2,
    pub txqcon3: CiTxqcon3,
}

impl Register32 for CiTxqcon {
    const ADDRESS: u16 = 0x0050;
}

impl Register8 for CiTxqcon0 {
    const ADDRESS: u16 = CiTxqcon::ADDRESS;
}

impl Register8 for CiTxqcon1 {
    const ADDRESS: u16 = CiTxqcon::ADDRESS + 1;
}

impl Register8 for CiTxqcon2 {
    const ADDRESS: u16 = CiTxqcon::ADDRESS + 2;
}

impl Register8 for CiTxqcon3 {
    const ADDRESS: u16 = CiTxqcon::ADDRESS + 3;
}

// -----------------------------------------------------------------------------
// CiTXQSTA – Transmit Queue Status Register (32 bits)

#[bitfield(bits = 8)]
#[derive(BitfieldSpecifier)]
#[repr(u8)]
pub struct CiTxqsta0 {
    pub tx_not_full_if: bool,
    #[skip]
    __: B1,
    pub tx_empty_if: bool,
    #[skip]
    __: B1,
    pub tx_attempt_if: bool,
    pub tx_error: bool,
    pub tx_lost_arbitration: bool,
    pub tx_aborted: bool,
}

#[bitfield(bits = 8)]
#[derive(BitfieldSpecifier)]
#[repr(u8)]
pub struct CiTxqsta1 {
    #[bits = 5]
    pub fifo_index: B5,
    #[skip]
    __: B3,
}

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct CiTxqsta {
    pub txqsta0: CiTxqsta0,
    pub txqsta1: CiTxqsta1,
    #[skip]
    __: B16,
}

impl Register32 for CiTxqsta {
    const ADDRESS: u16 = 0x0054;
}

// -----------------------------------------------------------------------------
// CiFIFOCON – FIFO Control Register
// (Separate Rx vs Tx structs)

// Rx version

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct CiFifoConRx {
    pub rx_not_empty_ie: bool,
    pub rx_half_full_ie: bool,
    pub rx_full_ie: bool,
    pub rx_over_flow_ie: bool,
    #[skip]
    __: B1,
    pub rx_time_stamp_enable: bool,
    #[skip]
    __: B1,
    pub tx_enable: bool, // same bit used for "TxEnable" or "RTREnable" in TX version
    pub uinc: bool,
    #[skip]
    __: B1,
    pub freset: bool,
    #[skip]
    __: B13,
    #[bits = 5]
    pub fifo_size: B5,
    #[bits = 3]
    pub payload_size: B3,
}

impl Register32 for CiFifoConRx {
    const ADDRESS: u16 = 0x005C;
}

// Tx version

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct CiFifoConTx {
    pub tx_not_full_ie: bool,
    pub tx_half_full_ie: bool,
    pub tx_empty_ie: bool,
    #[skip]
    __: B1,
    pub tx_attempt_ie: bool,
    #[skip]
    __: B1,
    pub rtr_enable: bool,
    pub tx_enable: bool,
    pub uinc: bool,
    pub tx_request: bool,
    pub freset: bool,
    #[skip]
    __: B5,
    #[bits = 5]
    pub tx_priority: B5,
    #[bits = 2]
    pub tx_attempts: B2,
    #[skip]
    __: B1,
    #[bits = 5]
    pub fifo_size: B5,
    #[bits = 3]
    pub payload_size: B3,
}

impl Register32 for CiFifoConTx {
    const ADDRESS: u16 = CiFifoConRx::ADDRESS;
}

// -----------------------------------------------------------------------------
// CiFIFOSTA – FIFO Status Register (Rx vs Tx both at same address range)

#[bitfield(bits = 8)]
#[derive(BitfieldSpecifier)]
#[repr(u8)]
pub struct CiFifoStaRx0 {
    pub rx_not_empty_if: bool,
    pub rx_half_full_if: bool,
    pub rx_full_if: bool,
    pub rx_over_flow_if: bool,
    #[skip]
    __: B4,
}

#[bitfield(bits = 8)]
#[derive(BitfieldSpecifier)]
#[repr(u8)]
pub struct CiFifoStaRx1 {
    #[bits = 5]
    pub fifo_index: B5,
    #[skip]
    __: B3,
}

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct CiFifoStaRx {
    pub rx0: CiFifoStaRx0,
    pub rx1: CiFifoStaRx1,
    #[skip]
    __: B16,
}

impl Register32 for CiFifoStaRx {
    const ADDRESS: u16 = 0x0060;
}

// Tx version

#[bitfield(bits = 8)]
#[derive(BitfieldSpecifier)]
#[repr(u8)]
pub struct CiFifoStaTx0 {
    pub tx_not_full_if: bool,
    pub tx_half_full_if: bool,
    pub tx_empty_if: bool,
    #[skip]
    __: B1,
    pub tx_attempt_if: bool,
    pub tx_error: bool,
    pub tx_lost_arbitration: bool,
    pub tx_aborted: bool,
}

#[bitfield(bits = 8)]
#[derive(BitfieldSpecifier)]
#[repr(u8)]
pub struct CiFifoStaTx1 {
    #[bits = 5]
    pub fifo_index: B5,
    #[skip]
    __: B3,
}

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct CiFifoStaTx {
    pub tx0: CiFifoStaTx0,
    pub tx1: CiFifoStaTx1,
    #[skip]
    __: B16,
}

impl Register32 for CiFifoStaTx {
    const ADDRESS: u16 = CiFifoStaRx::ADDRESS;
}

// -----------------------------------------------------------------------------
// CiFLTCON – Filter Control Register (32 bits) => 4 bytes, each 8 bits

#[bitfield(bits = 8)]
#[derive(BitfieldSpecifier)]
#[repr(u8)]
pub struct CiFltcon8 {
    #[bits = 5]
    pub buffer_pointer: B5,
    #[skip]
    __: B2,
    pub enable: bool,
}

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct CiFltcon {
    pub fltcon0: CiFltcon8,
    pub fltcon1: CiFltcon8,
    pub fltcon2: CiFltcon8,
    pub fltcon3: CiFltcon8,
}

impl Register32 for CiFltcon {
    const ADDRESS: u16 = 0x01D0;
}

// -----------------------------------------------------------------------------
// CiFLTOBJ & CiMASK – just placeholders with 32 bits, no subfields

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct CiFltobj {
    #[bits = 32]
    pub raw: u32,
}

impl Register32 for CiFltobj {
    const ADDRESS: u16 = 0x01F0;
}

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct CiMask {
    #[bits = 32]
    pub raw: u32,
}

impl Register32 for CiMask {
    const ADDRESS: u16 = 0x01F4;
}

// -----------------------------------------------------------------------------
// OSC – Oscillator Control Register (MCP2518FD variant) – 32 bits

#[bitfield(bits = 8)]
#[derive(BitfieldSpecifier)]
#[repr(u8)]
pub struct Osc0 {
    pub pll_enable: bool,
    #[skip]
    __: B1,
    pub osc_disable: bool,
    pub low_power_mode_enable: bool,
    pub sclkdiv: bool,
    #[bits = 2]
    pub clkodiv: B2,
    #[skip]
    __: B1,
}

#[bitfield(bits = 8)]
#[derive(BitfieldSpecifier)]
#[repr(u8)]
pub struct Osc1 {
    pub pll_ready: bool,
    #[skip]
    __: B1,
    pub osc_ready: bool,
    #[skip]
    __: B1,
    pub sclk_ready: bool,
    #[skip]
    __: B3,
}

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct Osc {
    pub osc0: Osc0,
    pub osc1: Osc1,
    #[skip]
    __: B16,
}

impl Register32 for Osc {
    const ADDRESS: u16 = 0x0E00;
}

// -----------------------------------------------------------------------------
// IOCON – I/O Control Register - **must be written using single data byte SFR WRITE instructions**

#[bitfield(bits = 8)]
#[repr(u8)]
pub struct Iocon0 {
    pub tris0: bool,
    pub tris1: bool,
    #[skip]
    __: B4,
    pub xcr_stby_enable: bool,
    #[skip]
    __: B1,
}

#[bitfield(bits = 8)]
#[repr(u8)]
pub struct Iocon1 {
    pub lat0: bool,
    pub lat1: bool,
    #[skip]
    __: B6,
}

#[bitfield(bits = 8)]
#[repr(u8)]
pub struct Iocon2 {
    pub gpio0: bool,
    pub gpio1: bool,
    #[skip]
    __: B6,
}

#[bitfield(bits = 8)]
#[repr(u8)]
pub struct Iocon3 {
    pub pin_mode0: bool,
    pub pin_mode1: bool,
    #[skip]
    __: B2,
    pub txcan_open_drain: bool,
    pub sof_output_enable: bool,
    pub int_pin_open_drain: bool,
    #[skip]
    __: B1,
}

impl Register8 for Iocon0 {
    const ADDRESS: u16 = 0x0E04;
}
impl Register8 for Iocon1 {
    const ADDRESS: u16 = 0x0E04 + 1;
}
impl Register8 for Iocon2 {
    const ADDRESS: u16 = 0x0E04 + 2;
}
impl Register8 for Iocon3 {
    const ADDRESS: u16 = 0x0E04 + 3;
}

// -----------------------------------------------------------------------------
// CRC – CRC Register

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct Crc {
    #[bits = 16]
    pub crc: u16, // bits 0..15
    pub crcerrif: bool, // bit 16
    pub ferrif: bool,   // bit 17
    #[skip]
    __: B6,  // bits 18..23
    pub crcerrie: bool, // bit 24
    pub ferrie: bool,   // bit 25
    #[skip]
    __: B6,  // bits 26..31
}

impl Register32 for Crc {
    const ADDRESS: u16 = 0x0E08;
}

// -----------------------------------------------------------------------------
// ECCCON – ECC Control Register

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct Ecccon {
    pub ecc_en: bool, // bit 0
    pub secie: bool,  // bit 1
    pub dedie: bool,  // bit 2
    #[skip]
    __: B5, // bits 3..7
    #[bits = 7]
    pub parity: B7,
    #[skip]
    __: B17, // bits 15..31
}

impl Register32 for Ecccon {
    const ADDRESS: u16 = 0x0E0C;
}

// -----------------------------------------------------------------------------
// ECCSTA – ECC Status Register

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct Eccsta {
    #[skip]
    __: B1, // bit 0 (unimplemented)
    pub secif: bool, // bit 1
    pub dedif: bool, // bit 2
    #[skip]
    __: B13, // bits 3..15
    #[bits = 12]
    pub error_address: B12,
    #[skip]
    __: B4, // bits 28..31
}

impl Register32 for Eccsta {
    const ADDRESS: u16 = 0x0E10;
}

// -----------------------------------------------------------------------------
// DEVID – Device ID Register

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct Devid {
    #[bits = 4]
    pub rev: B4,
    #[bits = 4]
    pub dev: B4,
    #[skip]
    __: B24, // bits 8..31
}

impl Register32 for Devid {
    const ADDRESS: u16 = 0x0E14;
}

// -----------------------------------------------------------------------------
