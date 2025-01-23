use modular_bitfield::prelude::*;

pub trait Register: From<u32> + Into<u32> {
    const ADDRESS: u16;
}

// -----------------------------------------------------------------------------
// CiCON – CAN Control Register

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct CiCon {
    pub d_net_filter_count: B5,                   // bits 0..4
    pub iso_crc_enable: bool,                     // bit 5
    pub protocol_exception_event_disable: bool,   // bit 6
    #[skip] __: B1,                               // bit 7 (unimplemented)
    pub wake_up_filter_enable: bool,              // bit 8
    pub wake_up_filter_time: B2,                  // bits 9..10
    #[skip] __: B1,                               // bit 11 (unimplemented)
    pub bit_rate_switch_disable: bool,            // bit 12
    #[skip] __: B3,                               // bits 13..15 (unimplemented)
    pub restrict_re_tx_attempts: bool,            // bit 16
    pub esi_in_gateway_mode: bool,                // bit 17
    pub system_error_to_listen_only: bool,        // bit 18
    pub store_in_tef: bool,                       // bit 19
    pub txq_enable: bool,                         // bit 20
    #[bits = 3]
    pub op_mode: B3,                              // bits 21..23
    #[bits = 3]
    pub request_op_mode: B3,                      // bits 24..26
    pub abort_all_tx: bool,                       // bit 27
    #[bits = 4]
    pub tx_band_width_sharing: B4,                // bits 28..31
}

impl Register for CiCon {
    const ADDRESS: u16 = 0x0000;
}

// -----------------------------------------------------------------------------
// CiNBTCFG – Nominal Bit Time Configuration Register

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct CiNbtcfg {
    #[bits = 7]
    pub sjw: B7,      // bits 0..6
    #[skip] __: B1,   // bit 7 (unimplemented)
    #[bits = 7]
    pub tseg2: B7,    // bits 8..14
    #[skip] __: B1,   // bit 15 (unimplemented)
    #[bits = 8]
    pub tseg1: u8,    // bits 16..23
    #[bits = 8]
    pub brp: u8,      // bits 24..31
}

impl Register for CiNbtcfg {
    const ADDRESS: u16 = 0x0004;
}

// -----------------------------------------------------------------------------
// CiDBTCFG – Data Bit Time Configuration Register

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct CiDbtcfg {
    #[bits = 4]
    pub sjw: B4,         // bits 0..3
    #[skip] __: B4,      // bits 4..7 (unimplemented)
    #[bits = 4]
    pub tseg2: B4,       // bits 8..11
    #[skip] __: B4,      // bits 12..15 (unimplemented)
    #[bits = 5]
    pub tseg1: B5,       // bits 16..20
    #[skip] __: B3,      // bits 21..23 (unimplemented)
    #[bits = 8]
    pub brp: u8,         // bits 24..31
}

impl Register for CiDbtcfg {
    const ADDRESS: u16 = 0x0008;
}

// -----------------------------------------------------------------------------
// CiTDC – Transmitter Delay Compensation Register

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct CiTdc {
    #[bits = 6]
    pub tdc_value: B6,    // bits 0..5
    #[skip] __: B2,       // bits 6..7 (unimplemented)
    #[bits = 7]
    pub tdc_offset: B7,   // bits 8..14
    #[skip] __: B1,       // bit 15 (unimplemented)
    #[bits = 2]
    pub tdc_mode: B2,     // bits 16..17
    #[skip] __: B6,       // bits 18..23 (unimplemented)
    pub sid11_enable: bool,      // bit 24
    pub edge_filter_enable: bool, // bit 25
    #[skip] __: B6,       // bits 26..31 (unimplemented)
}

impl Register for CiTdc {
    const ADDRESS: u16 = 0x000C;
}

// -----------------------------------------------------------------------------
// CiTSCON – Time Stamp Configuration Register

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct CiTscon {
    #[bits = 10]
    pub tbc_prescaler: B10,  // bits 0..9
    #[skip] __: B6,          // bits 10..15 (unimplemented)
    pub tbc_enable: bool,    // bit 16
    pub time_stamp_eof: bool,// bit 17
    #[skip] __: B14,         // bits 18..31 (unimplemented)
}

impl Register for CiTscon {
    const ADDRESS: u16 = 0x0014;
}

// -----------------------------------------------------------------------------
// CiVEC – Interrupt Vector Register

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct CiVec {
    #[bits = 7]
    pub icode: B7,     // bits 0..6
    #[skip] __: B1,    // bit 7 (unimplemented)
    #[bits = 5]
    pub filter_hit: B5,// bits 8..12
    #[skip] __: B3,    // bits 13..15 (unimplemented)
    #[bits = 7]
    pub txcode: B7,    // bits 16..22
    #[skip] __: B1,    // bit 23 (unimplemented)
    #[bits = 7]
    pub rxcode: B7,    // bits 24..30
    #[skip] __: B1,    // bit 31 (unimplemented)
}

impl Register for CiVec {
    const ADDRESS: u16 = 0x0018;
}

// -----------------------------------------------------------------------------
// CAN_INT_FLAGS (16 bits)

#[bitfield(bits = 16)]
#[derive(BitfieldSpecifier)]
#[repr(u16)]
pub struct CanIntFlags {
    pub txif: bool,    // bit 0
    pub rxif: bool,    // bit 1
    pub tbcif: bool,   // bit 2
    pub modif: bool,   // bit 3
    pub tefif: bool,   // bit 4
    #[skip] __: B3,    // bits 5..7 (unimplemented)
    pub eccif: bool,   // bit 8
    pub spicrcif: bool,// bit 9
    pub txatif: bool,  // bit 10
    pub rxovif: bool,  // bit 11
    pub serrif: bool,  // bit 12
    pub cerrif: bool,  // bit 13
    pub wakif: bool,   // bit 14
    pub ivmif: bool,   // bit 15
}

// -----------------------------------------------------------------------------
// CAN_INT_ENABLES (16 bits)

#[bitfield(bits = 16)]
#[derive(BitfieldSpecifier)]
#[repr(u16)]
pub struct CanIntEnables {
    pub txie: bool,   // bit 0
    pub rxie: bool,   // bit 1
    pub tbcie: bool,  // bit 2
    pub modie: bool,  // bit 3
    pub tefie: bool,  // bit 4
    #[skip] __: B3,   // bits 5..7 (unimplemented)
    pub eccie: bool,  // bit 8
    pub spicrcie: bool,// bit 9
    pub txatie: bool, // bit 10
    pub rxovie: bool, // bit 11
    pub serrie: bool, // bit 12
    pub cerrie: bool, // bit 13
    pub wakie: bool,  // bit 14
    pub ivmie: bool,  // bit 15
}

// -----------------------------------------------------------------------------
// CiINT – Interrupt Register (32 bits), combining FLAGS + ENABLES

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct CiInt {
    #[bits = 16]
    pub iflags: CanIntFlags,   // lower 16 bits
    #[bits = 16]
    pub ienable: CanIntEnables // upper 16 bits
}

impl Register for CiInt {
    const ADDRESS: u16 = 0x001C;
}

// -----------------------------------------------------------------------------
// CiTREC – Transmit/Receive Error Count Register

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct CiTrec {
    #[bits = 8]
    pub rx_error_count: u8,
    #[bits = 8]
    pub tx_error_count: u8,
    pub error_state_warning: bool,
    pub rx_error_state_warning: bool,
    pub tx_error_state_warning: bool,
    pub rx_error_state_passive: bool,
    pub tx_error_state_passive: bool,
    pub tx_error_state_bus_off: bool,
    #[skip] __: B10,
}

impl Register for CiTrec {
    const ADDRESS: u16 = 0x0034;
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
    #[skip] __: B1, // unimplemented1
    pub txbo_error: bool,
    pub d_bit0_error: bool,
    pub d_bit1_error: bool,
    pub d_ack_error: bool,
    pub d_form_error: bool,
    pub d_stuff_error: bool,
    pub d_crc_error: bool,
    pub esi: bool,
    #[skip] __: B1, // unimplemented2
}

impl Register for CiBdiag1 {
    const ADDRESS: u16 = 0x003C;
}

// -----------------------------------------------------------------------------
// CiTEFCON – Transmit Event FIFO Control Register

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct CiTefcon {
    pub tefneie: bool,
    pub tefhfie: bool,
    pub teffulie: bool,
    pub tefovie: bool,
    #[skip] __: B1,
    pub time_stamp_enable: bool,
    #[skip] __: B2,
    pub uinc: bool,
    #[skip] __: B1,
    pub freset: bool,
    #[skip] __: B13,
    #[bits = 5]
    pub fifo_size: B5,  
    #[skip] __: B3,
}

impl Register for CiTefcon {
    const ADDRESS: u16 = 0x0040;
}

// -----------------------------------------------------------------------------
// CiTEFSTA – Transmit Event FIFO Status Register

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct CiTefsta {
    pub tef_not_empty_if: bool,
    pub tef_half_full_if: bool,
    pub tef_full_if: bool,
    pub tefovif: bool,
    #[skip] __: B28,
}

impl Register for CiTefsta {
    const ADDRESS: u16 = 0x0044;
}

// -----------------------------------------------------------------------------
// CiTXQCON – Transmit Queue Control Register

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct CiTxqcon {
    pub tx_not_full_ie: bool,
    #[skip] __: B1,
    pub tx_empty_ie: bool,
    #[skip] __: B1,
    pub tx_attempt_ie: bool,
    #[skip] __: B2,
    pub tx_enable: bool,
    pub uinc: bool,
    pub tx_request: bool,
    pub freset: bool,
    #[skip] __: B5,
    #[bits = 5]
    pub tx_priority: B5, 
    #[bits = 2]
    pub tx_attempts: B2, 
    #[skip] __: B1,
    #[bits = 5]
    pub fifo_size: B5,   
    #[bits = 3]
    pub payload_size: B3,
}

impl Register for CiTxqcon {
    const ADDRESS: u16 = 0x0050;
}

// -----------------------------------------------------------------------------
// CiTXQSTA – Transmit Queue Status Register

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct CiTxqsta {
    pub tx_not_full_if: bool,
    #[skip] __: B1,
    pub tx_empty_if: bool,
    #[skip] __: B1,
    pub tx_attempt_if: bool,
    pub tx_error: bool,
    pub tx_lost_arbitration: bool,
    pub tx_aborted: bool,
    #[bits = 5]
    pub fifo_index: B5, 
    #[skip] __: B19,
}

impl Register for CiTxqsta {
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
    #[skip] __: B1,
    pub rx_time_stamp_enable: bool,
    #[skip] __: B1,
    pub tx_enable: bool, // same bit used for "TxEnable" or "RTREnable" in TX version
    pub uinc: bool,
    #[skip] __: B1,
    pub freset: bool,
    #[skip] __: B13,
    #[bits = 5]
    pub fifo_size: B5,     
    #[bits = 3]
    pub payload_size: B3,  
}

impl Register for CiFifoConRx {
    const ADDRESS: u16 = 0x005C;
}

// Tx version

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct CiFifoConTx {
    pub tx_not_full_ie: bool,
    pub tx_half_full_ie: bool,
    pub tx_empty_ie: bool,
    #[skip] __: B1,
    pub tx_attempt_ie: bool,
    #[skip] __: B1,
    pub rtr_enable: bool,
    pub tx_enable: bool,
    pub uinc: bool,
    pub tx_request: bool,
    pub freset: bool,
    #[skip] __: B5,
    #[bits = 5]
    pub tx_priority: B5, 
    #[bits = 2]
    pub tx_attempts: B2, 
    #[skip] __: B1,
    #[bits = 5]
    pub fifo_size: B5,   
    #[bits = 3]
    pub payload_size: B3,
}

impl Register for CiFifoConTx {
    const ADDRESS: u16 = CiFifoConRx::ADDRESS;
}

// -----------------------------------------------------------------------------
// CiFIFOSTA – FIFO Status Register (Rx vs Tx)

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct CiFifoStaRx {
    pub rx_not_empty_if: bool,
    pub rx_half_full_if: bool,
    pub rx_full_if: bool,
    pub rx_over_flow_if: bool,
    #[skip] __: B4,
    #[bits = 5]
    pub fifo_index: B5, 
    #[skip] __: B19,
}

impl Register for CiFifoStaRx {
    const ADDRESS: u16 = 0x0060;
}

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct CiFifoStaTx {
    pub tx_not_full_if: bool,
    pub tx_half_full_if: bool,
    pub tx_empty_if: bool,
    #[skip] __: B1,
    pub tx_attempt_if: bool,
    pub tx_error: bool,
    pub tx_lost_arbitration: bool,
    pub tx_aborted: bool,
    #[bits = 5]
    pub fifo_index: B5, 
    #[skip] __: B19,
}

impl Register for CiFifoStaTx {
    const ADDRESS: u16 = CiFifoStaRx::ADDRESS;
}

// -----------------------------------------------------------------------------
// CiFLTCON_BYTE – Filter Control Register (8 bits)

#[bitfield(bits = 8)]
#[derive(BitfieldSpecifier)]
#[repr(u8)]
pub struct CiFltconByte {
    #[bits = 5]
    pub buffer_pointer: B5, 
    #[skip] __: B2,
    pub enable: bool,
}

// CiFLTCON – Filter Control Register

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct CiFltcon {
    #[bits = 8]
    pub fltcon0: CiFltconByte, // bits 0..7
    #[bits = 8]
    pub fltcon1: CiFltconByte, // bits 8..15
    #[bits = 8]
    pub fltcon2: CiFltconByte, // bits 16..23
    #[bits = 8]
    pub fltcon3: CiFltconByte, // bits 24..31
}

impl Register for CiFltcon {
    const ADDRESS: u16 = 0x01D0;
}

// -----------------------------------------------------------------------------
// CiFLTOBJ & CiMASK – Generic placeholders

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct CiFltobj {
    #[bits = 32]
    pub raw: u32,
}

impl Register for CiFltobj {
    const ADDRESS: u16 = 0x01F0;
}

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct CiMask {
    #[bits = 32]
    pub raw: u32,
}

impl Register for CiMask {
    const ADDRESS: u16 = 0x01F4;
}

// -----------------------------------------------------------------------------
// OSC – Oscillator Control Register (MCP2518FD variant)

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct Osc {
    pub pll_enable: bool,             // bit 0
    #[skip] __: B1,                   // bit 1 (unimplemented on MCP2518)
    pub osc_disable: bool,            // bit 2
    pub low_power_mode_enable: bool,  // bit 3
    pub sclkdiv: bool,                // bit 4
    #[bits = 2]
    pub clkodiv: B2,                  
    #[skip] __: B1,                   // bit 7
    pub pll_ready: bool,              // bit 8
    #[skip] __: B1,                   // bit 9
    pub osc_ready: bool,              // bit 10
    #[skip] __: B1,                   // bit 11
    pub sclk_ready: bool,             // bit 12
    #[skip] __: B19,                  // bits 13..31
}

impl Register for Osc {
    const ADDRESS: u16 = 0x0E00;
}

// -----------------------------------------------------------------------------
// IOCON – I/O Control Register

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct Iocon {
    pub tris0: bool,                       // bit 0
    pub tris1: bool,                       // bit 1
    #[skip] __: B2,                        // bits 2..3 (unimplemented)
    pub clear_auto_sleep_on_match: bool,   // bit 4
    pub auto_sleep_enable: bool,           // bit 5
    pub xcr_stby_enable: bool,             // bit 6
    #[skip] __: B1,                        // bit 7
    pub lat0: bool,                        // bit 8
    pub lat1: bool,                        // bit 9
    #[skip] __: B5,                        // bits 10..14 (unimplemented)
    pub hvdetsel: bool,                    // bit 15
    pub gpio0: bool,                       // bit 16
    pub gpio1: bool,                       // bit 17
    #[skip] __: B6,                        // bits 18..23
    pub pin_mode0: bool,                   // bit 24
    pub pin_mode1: bool,                   // bit 25
    #[skip] __: B2,                        // bits 26..27
    pub txcan_open_drain: bool,            // bit 28
    pub sof_output_enable: bool,           // bit 29
    pub int_pin_open_drain: bool,          // bit 30
    #[skip] __: B1,                        // bit 31
}

impl Register for Iocon {
    const ADDRESS: u16 = 0x0E04;
}

// -----------------------------------------------------------------------------
// CRC – CRC Register

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct Crc {
    #[bits = 16]
    pub crc: u16,      // bits 0..15
    pub crcerrif: bool,// bit 16
    pub ferrif: bool,  // bit 17
    #[skip] __: B6,    // bits 18..23
    pub crcerrie: bool,// bit 24
    pub ferrie: bool,  // bit 25
    #[skip] __: B6,    // bits 26..31
}

impl Register for Crc {
    const ADDRESS: u16 = 0x0E08;
}

// -----------------------------------------------------------------------------
// ECCCON – ECC Control Register

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct Ecccon {
    pub ecc_en: bool,  // bit 0
    pub secie: bool,   // bit 1
    pub dedie: bool,   // bit 2
    #[skip] __: B5,    // bits 3..7
    #[bits = 7]
    pub parity: B7,    
    #[skip] __: B17,   // bits 15..31
}

impl Register for Ecccon {
    const ADDRESS: u16 = 0x0E0C;
}

// -----------------------------------------------------------------------------
// ECCSTA – ECC Status Register

#[bitfield(bits = 32)]
#[repr(u32)]
pub struct Eccsta {
    #[skip] __: B1,    // bit 0 (unimplemented)
    pub secif: bool,   // bit 1
    pub dedif: bool,   // bit 2
    #[skip] __: B13,   // bits 3..15
    #[bits = 12]
    pub error_address: B12, 
    #[skip] __: B4,    // bits 28..31
}

impl Register for Eccsta {
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
    #[skip] __: B24,// bits 8..31
}

impl Register for Devid {
    const ADDRESS: u16 = 0x0E14;
}

// -----------------------------------------------------------------------------