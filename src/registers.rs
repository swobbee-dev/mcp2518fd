use modular_bitfield::prelude::*;

pub trait Register: From<Self::V> + Into<Self::V> + Sized {
    type V: FromBytes + ToBytes;

    fn address(index: u16) -> u16;
}

pub trait FromBytes {
    fn from_bytes(bytes: &[u8]) -> Self;
}

pub trait ToBytes {
    type Bytes: AsRef<[u8]>;

    fn to_bytes(&self) -> Self::Bytes;
}

impl FromBytes for u8 {
    fn from_bytes(bytes: &[u8]) -> Self {
        bytes[0]
    }
}

impl ToBytes for u8 {
    type Bytes = [u8; 1];
    fn to_bytes(&self) -> Self::Bytes {
        [*self]
    }
}

impl FromBytes for u16 {
    fn from_bytes(bytes: &[u8]) -> Self {
        let mut array = [0u8; 2];
        array.copy_from_slice(bytes);
        u16::from_le_bytes(array)
    }
}

impl ToBytes for u16 {
    type Bytes = [u8; 2];
    fn to_bytes(&self) -> Self::Bytes {
        self.to_le_bytes()
    }
}

impl FromBytes for u32 {
    fn from_bytes(bytes: &[u8]) -> Self {
        let mut array = [0u8; 4];
        array.copy_from_slice(bytes);
        u32::from_le_bytes(array)
    }
}

impl ToBytes for u32 {
    type Bytes = [u8; 4];
    fn to_bytes(&self) -> Self::Bytes {
        self.to_le_bytes()
    }
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

impl From<OperationMode> for u8 {
    fn from(val: OperationMode) -> Self {
        val as u8
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

impl Register for CiCon {
    type V = u32;
    fn address(_index: u16) -> u16 {
        0x0000
    }
}

impl Register for CiCon0 {
    type V = u8;
    fn address(_index: u16) -> u16 {
        CiCon::address(_index)
    }
}
impl Register for CiCon1 {
    type V = u8;
    fn address(_index: u16) -> u16 {
        CiCon::address(_index) + 1
    }
}
impl Register for CiCon2 {
    type V = u8;
    fn address(_index: u16) -> u16 {
        CiCon::address(_index) + 2
    }
}
impl Register for CiCon3 {
    type V = u8;
    fn address(_index: u16) -> u16 {
        CiCon::address(_index) + 3
    }
}

// -----------------------------------------------------------------------------
// CiNBTCFG – Nominal Bit Time Configuration Register

#[bitfield(bits = 32)]
#[derive(Clone, Copy)]
#[repr(u32)]
pub struct CiNbtcfg {
    #[bits = 7]
    pub sjw: B7,
    #[skip]
    __: B1,
    #[bits = 7]
    pub tseg2: B7,
    #[skip]
    __: B1,
    #[bits = 8]
    pub tseg1: u8,
    #[bits = 8]
    pub brp: u8,
}

impl Register for CiNbtcfg {
    type V = u32;
    fn address(_index: u16) -> u16 {
        0x0004
    }
}

// -----------------------------------------------------------------------------
// CiDBTCFG – Data Bit Time Configuration Register (32 bits)

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct CiDbtcfg {
    #[bits = 7]
    pub sjw: B7,
    #[skip]
    __: B1,
    #[bits = 7]
    pub tseg2: B7,
    #[skip]
    __: B1,
    #[bits = 8]
    pub tseg1: u8,
    #[bits = 8]
    pub brp: u8,
}

impl Register for CiDbtcfg {
    type V = u32;
    fn address(_index: u16) -> u16 {
        0x0008
    }
}

// -----------------------------------------------------------------------------
// CiTDC – Transmitter Delay Compensation Register (32 bits)

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct CiTdc {
    #[bits = 6]
    pub tdc_value: B6,
    #[skip]
    __: B2,
    #[bits = 7]
    pub tdc_offset: B7,
    #[skip]
    __: B1,
    #[bits = 2]
    pub tdc_mode: B2,
    #[skip]
    __: B6,
    pub sid11_enable: bool,
    pub edge_filter_enable: bool,
    #[skip]
    __: B6,
}

impl Register for CiTdc {
    type V = u32;
    fn address(_index: u16) -> u16 {
        0x000C
    }
}

// -----------------------------------------------------------------------------
// CiTSCON – Time Stamp Configuration Register (32 bits)

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

impl Register for CiTscon {
    type V = u32;
    fn address(_index: u16) -> u16 {
        0x0014
    }
}

impl Register for CiTscon0 {
    type V = u16;
    fn address(_index: u16) -> u16 {
        CiTscon::address(_index)
    }
}
impl Register for CiTscon1 {
    type V = u16;
    fn address(_index: u16) -> u16 {
        CiTscon::address(_index) + 2
    }
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

impl Register for CiVec {
    type V = u32;
    fn address(_index: u16) -> u16 {
        0x0018
    }
}

impl Register for CiVec0 {
    type V = u8;
    fn address(_index: u16) -> u16 {
        CiVec::address(_index)
    }
}
impl Register for CiVec1 {
    type V = u8;
    fn address(_index: u16) -> u16 {
        CiVec::address(_index) + 1
    }
}
impl Register for CiVec2 {
    type V = u8;
    fn address(_index: u16) -> u16 {
        CiVec::address(_index) + 2
    }
}
impl Register for CiVec3 {
    type V = u8;
    fn address(_index: u16) -> u16 {
        CiVec::address(_index) + 3
    }
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

impl Register for CiInt {
    type V = u32;
    fn address(_index: u16) -> u16 {
        0x001C
    }
}

impl Register for CanIntFlags {
    type V = u16;
    fn address(_index: u16) -> u16 {
        CiInt::address(_index)
    }
}

impl Register for CanIntEnables {
    type V = u16;
    fn address(_index: u16) -> u16 {
        CiInt::address(_index) + 2
    }
}

// -----------------------------------------------------------------------------
// CiTREC – Transmit/Receive Error Count Register (32 bits)

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

impl Register for CiTrec {
    type V = u32;
    fn address(_index: u16) -> u16 {
        0x0034
    }
}

impl Register for CiTrecState {
    type V = u16;
    fn address(_index: u16) -> u16 {
        CiTrec::address(_index) + 2
    }
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

impl Register for CiBdiag0 {
    type V = u32;
    fn address(_index: u16) -> u16 {
        0x0038
    }
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

impl Register for CiBdiag1 {
    type V = u32;
    fn address(_index: u16) -> u16 {
        0x003C
    }
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

impl Register for CiTefcon {
    type V = u32;
    fn address(_index: u16) -> u16 {
        0x0040
    }
}

impl Register for CiTefcon0 {
    type V = u8;
    fn address(_index: u16) -> u16 {
        CiTefcon::address(_index)
    }
}

impl Register for CiTefcon1 {
    type V = u8;
    fn address(_index: u16) -> u16 {
        CiTefcon::address(_index) + 1
    }
}

impl Register for CiTefcon2 {
    type V = u8;
    fn address(_index: u16) -> u16 {
        CiTefcon::address(_index) + 2
    }
}

impl Register for CiTefcon3 {
    type V = u8;
    fn address(_index: u16) -> u16 {
        CiTefcon::address(_index) + 3
    }
}

// -----------------------------------------------------------------------------
// CiTEFSTA – Transmit Event FIFO Status Register (32 bits)

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

impl Register for CiTefsta {
    type V = u32;
    fn address(_index: u16) -> u16 {
        0x0044
    }
}

impl Register for CiTefsta0 {
    type V = u8;
    fn address(_index: u16) -> u16 {
        CiTefsta::address(_index)
    }
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

impl Register for CiTxqcon {
    type V = u32;
    fn address(_index: u16) -> u16 {
        0x0050
    }
}

impl Register for CiTxqcon0 {
    type V = u8;
    fn address(_index: u16) -> u16 {
        CiTxqcon::address(_index)
    }
}

impl Register for CiTxqcon1 {
    type V = u8;
    fn address(_index: u16) -> u16 {
        CiTxqcon::address(_index) + 1
    }
}

impl Register for CiTxqcon2 {
    type V = u8;
    fn address(_index: u16) -> u16 {
        CiTxqcon::address(_index) + 2
    }
}

impl Register for CiTxqcon3 {
    type V = u8;
    fn address(_index: u16) -> u16 {
        CiTxqcon::address(_index) + 3
    }
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

impl Register for CiTxqsta {
    type V = u32;
    fn address(_index: u16) -> u16 {
        0x0054
    }
}

// -----------------------------------------------------------------------------
// CiFIFOCON – FIFO Control Register (Separate Rx vs Tx)

#[bitfield(bits = 8)]
#[derive(BitfieldSpecifier)]
#[repr(u8)]
pub struct CiFifoCon1 {
    pub uinc: bool,
    pub txreq: bool,
    pub freset: bool,
    #[skip]
    __: B5,
}

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
    pub tx_enable: bool,
    pub cififocon1: CiFifoCon1,
    #[skip]
    __: B8,
    #[bits = 5]
    pub fifo_size: B5,
    #[bits = 3]
    pub payload_size: B3,
}

impl Register for CiFifoConRx {
    type V = u32;
    fn address(index: u16) -> u16 {
        0x005C + index * 12
    }
}

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
    pub cififocon1: CiFifoCon1,
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

impl Register for CiFifoConTx {
    type V = u32;
    fn address(index: u16) -> u16 {
        CiFifoConRx::address(index)
    }
}

impl Register for CiFifoCon1 {
    type V = u8;
    fn address(index: u16) -> u16 {
        CiFifoConTx::address(index) + 1
    }
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

impl Register for CiFifoStaRx {
    type V = u32;
    fn address(index: u16) -> u16 {
        0x0060 + index * 12
    }
}

impl Register for CiFifoStaRx0 {
    type V = u8;
    fn address(index: u16) -> u16 {
        CiFifoStaRx::address(index)
    }
}

impl Register for CiFifoStaRx1 {
    type V = u8;
    fn address(index: u16) -> u16 {
        CiFifoStaRx::address(index) + 1
    }
}

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

impl Register for CiFifoStaTx {
    type V = u32;
    fn address(index: u16) -> u16 {
        CiFifoStaRx::address(index)
    }
}

impl Register for CiFifoStaTx0 {
    type V = u8;
    fn address(index: u16) -> u16 {
        CiFifoStaTx::address(index)
    }
}

impl Register for CiFifoStaTx1 {
    type V = u8;
    fn address(index: u16) -> u16 {
        CiFifoStaTx::address(index) + 1
    }
}

// -----------------------------------------------------------------------------
// CiFFIFOUA – FIFO USER ADDRESS REGISTER (32 bits)

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct CiFIFOUA {
    pub fifo_user_address: u32,
}

impl Register for CiFIFOUA {
    type V = u32;
    fn address(index: u16) -> u16 {
        0x0064 + index * 12
    }
}

// -----------------------------------------------------------------------------
// CiFltcon – Filter Control Register (32 bits)

#[bitfield(bits = 8)]
#[derive(BitfieldSpecifier)]
#[repr(u8)]
pub struct CiFltcon {
    #[bits = 5]
    pub buffer_pointer: B5,
    #[skip]
    __: B2,
    pub enable: bool,
}

impl Register for CiFltcon {
    type V = u8;
    fn address(index: u16) -> u16 {
        0x01D0 + index
    }
}

// -----------------------------------------------------------------------------
// CiFLTOBJ & CiMASK – placeholders

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct CiFltobj {
    #[bits = 32]
    pub raw: u32,
}

impl Register for CiFltobj {
    type V = u32;
    fn address(index: u16) -> u16 {
        0x01F0 + 8 * index
    }
}

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct CiMask {
    #[bits = 32]
    pub raw: u32,
}

impl Register for CiMask {
    type V = u32;
    fn address(index: u16) -> u16 {
        0x01F4 + 8 * index
    }
}

// -----------------------------------------------------------------------------
// OSC – Oscillator Control Register (MCP2518FD variant) – 32 bits

#[bitfield(bits = 8)]
#[repr(u8)]
pub struct OscConfig {
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
#[repr(u8)]
pub struct OscStatus {
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

impl Register for OscConfig {
    type V = u8;
    fn address(_index: u16) -> u16 {
        0x0E00
    }
}

impl Register for OscStatus {
    type V = u8;
    fn address(_index: u16) -> u16 {
        OscConfig::address(_index) + 1
    }
}

// -----------------------------------------------------------------------------
// IOCON – I/O Control Register

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

impl Register for Iocon0 {
    type V = u8;
    fn address(_index: u16) -> u16 {
        0x0E04
    }
}
impl Register for Iocon1 {
    type V = u8;
    fn address(_index: u16) -> u16 {
        0x0E04 + 1
    }
}
impl Register for Iocon2 {
    type V = u8;
    fn address(_index: u16) -> u16 {
        0x0E04 + 2
    }
}
impl Register for Iocon3 {
    type V = u8;
    fn address(_index: u16) -> u16 {
        0x0E04 + 3
    }
}

// -----------------------------------------------------------------------------
// CRC – CRC Register

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct Crc {
    #[bits = 16]
    pub crc: u16,
    pub crcerrif: bool,
    pub ferrif: bool,
    #[skip]
    __: B6,
    pub crcerrie: bool,
    pub ferrie: bool,
    #[skip]
    __: B6,
}

impl Register for Crc {
    type V = u32;
    fn address(_index: u16) -> u16 {
        0x0E08
    }
}

// -----------------------------------------------------------------------------
// ECCCON – ECC Control Register

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct Ecccon {
    pub ecc_en: bool,
    pub secie: bool,
    pub dedie: bool,
    #[skip]
    __: B5,
    #[bits = 7]
    pub parity: B7,
    #[skip]
    __: B17,
}

impl Register for Ecccon {
    type V = u32;
    fn address(_index: u16) -> u16 {
        0x0E0C
    }
}

// -----------------------------------------------------------------------------
// ECCSTA – ECC Status Register

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct Eccsta {
    #[skip]
    __: B1,
    pub secif: bool,
    pub dedif: bool,
    #[skip]
    __: B13,
    #[bits = 12]
    pub error_address: B12,
    #[skip]
    __: B4,
}

impl Register for Eccsta {
    type V = u32;
    fn address(_index: u16) -> u16 {
        0x0E10
    }
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
    __: B24,
}

impl Register for Devid {
    type V = u32;
    fn address(_index: u16) -> u16 {
        0x0E14
    }
}

// -----------------------------------------------------------------------------
