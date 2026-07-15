pub mod codec;
pub mod error;
#[cfg(feature = "json")]
pub mod mavlink_json;
pub mod rust_mavlink_compatibility;
pub mod signing;
pub mod v1;
pub mod v2;

use bytes::Bytes;

use v1::{V1Packet, V1PacketRef, V1_STX};
use v2::{V2Packet, V2PacketRef, V2_STX};

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(u8)]
pub enum Packet {
    V1(V1Packet) = V1_STX,
    V2(V2Packet) = V2_STX,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum PacketRef<'a> {
    V1(V1PacketRef<'a>),
    V2(V2PacketRef<'a>),
}

impl Packet {
    #[inline(always)]
    pub fn bytes(&self) -> &Bytes {
        match self {
            Packet::V1(v1_packet) => v1_packet.bytes(),
            Packet::V2(v2_packet) => v2_packet.bytes(),
        }
    }

    #[inline(always)]
    pub fn as_slice(&self) -> &[u8] {
        match self {
            Packet::V1(v1_packet) => v1_packet.as_slice(),
            Packet::V2(v2_packet) => v2_packet.as_slice(),
        }
    }

    /// Zero-copy borrowed view of this owned frame.
    ///
    /// Prefer operating on [`PacketRef`] (field access, typed parse) so work stays
    /// in place on the underlying buffer.
    #[inline(always)]
    pub fn as_ref(&self) -> PacketRef<'_> {
        match self {
            Packet::V1(v1_packet) => PacketRef::V1(v1_packet.as_ref()),
            Packet::V2(v2_packet) => PacketRef::V2(v2_packet.as_ref()),
        }
    }

    #[inline(always)]
    pub fn header(&self) -> &[u8] {
        match self {
            Packet::V1(v1_packet) => v1_packet.header(),
            Packet::V2(v2_packet) => v2_packet.header(),
        }
    }

    #[inline(always)]
    pub fn payload(&self) -> &[u8] {
        match self {
            Packet::V1(v1_packet) => v1_packet.payload(),
            Packet::V2(v2_packet) => v2_packet.payload(),
        }
    }

    /// Zero-copy view of the payload as a reference-counted [`Bytes`] slice of the frame buffer.
    #[inline(always)]
    pub fn payload_bytes(&self) -> Bytes {
        match self {
            Packet::V1(v1_packet) => v1_packet.payload_bytes(),
            Packet::V2(v2_packet) => v2_packet.payload_bytes(),
        }
    }

    /// Zero-copy view of the header as a reference-counted [`Bytes`] slice of the frame buffer.
    #[inline(always)]
    pub fn header_bytes(&self) -> Bytes {
        match self {
            Packet::V1(v1_packet) => v1_packet.header_bytes(),
            Packet::V2(v2_packet) => v2_packet.header_bytes(),
        }
    }

    #[inline(always)]
    pub fn checksum(&self) -> u16 {
        match self {
            Packet::V1(v1_packet) => v1_packet.checksum(),
            Packet::V2(v2_packet) => v2_packet.checksum(),
        }
    }

    #[inline(always)]
    pub fn checksum_data(&self) -> &[u8] {
        match self {
            Packet::V1(v1_packet) => v1_packet.checksum_data(),
            Packet::V2(v2_packet) => v2_packet.checksum_data(),
        }
    }

    #[inline(always)]
    pub fn packet_size(&self) -> usize {
        match self {
            Packet::V1(v1_packet) => v1_packet.packet_size(),
            Packet::V2(v2_packet) => v2_packet.packet_size(),
        }
    }

    #[inline(always)]
    pub fn stx(&self) -> &u8 {
        match self {
            Packet::V1(v1_packet) => v1_packet.stx(),
            Packet::V2(v2_packet) => v2_packet.stx(),
        }
    }

    #[inline(always)]
    pub fn payload_length(&self) -> &u8 {
        match self {
            Packet::V1(v1_packet) => v1_packet.payload_length(),
            Packet::V2(v2_packet) => v2_packet.payload_length(),
        }
    }

    #[inline(always)]
    pub fn sequence(&self) -> &u8 {
        match self {
            Packet::V1(v1_packet) => v1_packet.sequence(),
            Packet::V2(v2_packet) => v2_packet.sequence(),
        }
    }

    #[inline(always)]
    pub fn system_id(&self) -> &u8 {
        match self {
            Packet::V1(v1_packet) => v1_packet.system_id(),
            Packet::V2(v2_packet) => v2_packet.system_id(),
        }
    }

    #[inline(always)]
    pub fn component_id(&self) -> &u8 {
        match self {
            Packet::V1(v1_packet) => v1_packet.component_id(),
            Packet::V2(v2_packet) => v2_packet.component_id(),
        }
    }

    #[inline(always)]
    pub fn message_id(&self) -> u32 {
        match self {
            Packet::V1(v1_packet) => *v1_packet.message_id() as u32,
            Packet::V2(v2_packet) => v2_packet.message_id(),
        }
    }
}

impl<'a> PacketRef<'a> {
    #[inline(always)]
    pub fn new(buffer: &'a [u8]) -> Option<Self> {
        match *buffer.first()? {
            V2_STX => V2PacketRef::new(buffer).map(PacketRef::V2),
            V1_STX => V1PacketRef::new(buffer).map(PacketRef::V1),
            _ => None,
        }
    }

    #[inline(always)]
    pub fn as_slice(&self) -> &'a [u8] {
        match self {
            PacketRef::V1(packet) => packet.as_slice(),
            PacketRef::V2(packet) => packet.as_slice(),
        }
    }

    #[inline(always)]
    pub fn header(&self) -> &'a [u8] {
        match self {
            PacketRef::V1(packet) => packet.header(),
            PacketRef::V2(packet) => packet.header(),
        }
    }

    #[inline(always)]
    pub fn payload(&self) -> &'a [u8] {
        match self {
            PacketRef::V1(packet) => packet.payload(),
            PacketRef::V2(packet) => packet.payload(),
        }
    }

    #[inline(always)]
    pub fn checksum(&self) -> u16 {
        match self {
            PacketRef::V1(packet) => packet.checksum(),
            PacketRef::V2(packet) => packet.checksum(),
        }
    }

    #[inline(always)]
    pub fn checksum_data(&self) -> &'a [u8] {
        match self {
            PacketRef::V1(packet) => packet.checksum_data(),
            PacketRef::V2(packet) => packet.checksum_data(),
        }
    }

    #[inline(always)]
    pub fn packet_size(&self) -> usize {
        match self {
            PacketRef::V1(packet) => packet.packet_size(),
            PacketRef::V2(packet) => packet.packet_size(),
        }
    }

    #[inline(always)]
    pub fn stx(&self) -> &u8 {
        match self {
            PacketRef::V1(packet) => packet.stx(),
            PacketRef::V2(packet) => packet.stx(),
        }
    }

    #[inline(always)]
    pub fn payload_length(&self) -> &u8 {
        match self {
            PacketRef::V1(packet) => packet.payload_length(),
            PacketRef::V2(packet) => packet.payload_length(),
        }
    }

    #[inline(always)]
    pub fn sequence(&self) -> &u8 {
        match self {
            PacketRef::V1(packet) => packet.sequence(),
            PacketRef::V2(packet) => packet.sequence(),
        }
    }

    #[inline(always)]
    pub fn system_id(&self) -> &u8 {
        match self {
            PacketRef::V1(packet) => packet.system_id(),
            PacketRef::V2(packet) => packet.system_id(),
        }
    }

    #[inline(always)]
    pub fn component_id(&self) -> &u8 {
        match self {
            PacketRef::V1(packet) => packet.component_id(),
            PacketRef::V2(packet) => packet.component_id(),
        }
    }

    #[inline(always)]
    pub fn message_id(&self) -> u32 {
        match self {
            PacketRef::V1(packet) => *packet.message_id() as u32,
            PacketRef::V2(packet) => packet.message_id(),
        }
    }

    /// Copies this borrowed frame into an owned [`Packet`].
    #[inline]
    pub fn to_owned(&self) -> Packet {
        match self {
            PacketRef::V1(packet) => Packet::V1(packet.to_owned()),
            PacketRef::V2(packet) => Packet::V2(packet.to_owned()),
        }
    }
}

#[cfg(test)]
mod packet_ref_test {
    use super::*;
    use v1::V1Packet;
    use v2::V2Packet;

    const COMMAND_LONG: &[u8] = &[
        253, 30, 0, 0, 0, 0, 50, 76, 0, 0, 0, 0, 230, 66, 0, 64, 156, 69, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 255, 1, 188, 195,
    ];

    const HEARTBEAT_V1: &[u8] = &[254, 9, 239, 1, 2, 0, 5, 0, 0, 0, 2, 3, 89, 3, 3, 31, 80];

    #[test]
    fn packet_ref_v2_command_long() {
        let packet = PacketRef::new(COMMAND_LONG).expect("valid v2 frame");
        assert!(matches!(packet, PacketRef::V2(_)));
        assert_eq!(packet.message_id(), 76);
        assert_eq!(*packet.system_id(), 0);
        assert_eq!(*packet.component_id(), 50);
        assert_eq!(*packet.sequence(), 0);
        assert_eq!(packet.payload().len(), 30);
        assert_eq!(
            packet.payload(),
            &COMMAND_LONG[V2Packet::STX_SIZE + V2Packet::HEADER_SIZE
                ..V2Packet::STX_SIZE + V2Packet::HEADER_SIZE + 30]
        );
    }

    #[test]
    fn packet_ref_v1_heartbeat() {
        let packet = PacketRef::new(HEARTBEAT_V1).expect("valid v1 frame");
        assert!(matches!(packet, PacketRef::V1(_)));
        assert_eq!(packet.message_id(), 0);
        assert_eq!(*packet.system_id(), 1);
        assert_eq!(*packet.component_id(), 2);
        assert_eq!(*packet.sequence(), 239);
        assert_eq!(packet.payload().len(), 9);
        assert_eq!(
            packet.payload(),
            &HEARTBEAT_V1[V1Packet::STX_SIZE + V1Packet::HEADER_SIZE
                ..V1Packet::STX_SIZE + V1Packet::HEADER_SIZE + 9]
        );
    }

    #[test]
    fn packet_ref_rejects_invalid() {
        assert!(PacketRef::new(&[]).is_none());
        assert!(PacketRef::new(&[0x00, 0x01]).is_none());
        assert!(PacketRef::new(&[253, 30, 0, 0, 0, 0, 50, 76, 0, 0]).is_none());
    }
}

/// Creates a `MavlinkCodec` with compile-time configuration.
///
/// # Parameters
///
/// - `accept_v1`: Whether to accept MAVLink V1 messages.
/// - `accept_v2`: Whether to accept MAVLink V2 messages.
/// - `drop_invalid_sysid`: Whether to drop messages with zeroed System ID
/// - `drop_invalid_compid`: Whether to drop messages with zeroed Component ID
/// - `skip_crc_validation`: Whether to skip the CRC validation
/// - `drop_incompatible`: Whether to drop messages with unknown Incompatibility Flags
/// - `verify_signature`: Whether to require a valid MAVLink2 signature
/// - `accept_unknown_msgid`: Whether to forward (unvalidated) frames whose message id is
///   absent from the compiled dialect, instead of dropping them (router use case)
///
/// # Example
///
/// ```
/// use mavlink_codec::{mavlink_codec, codec::MavlinkCodec};
///
/// let codec = mavlink_codec! {
///     accept_v1: true,
///     accept_v2: true,
///     drop_invalid_sysid: false,
///     drop_invalid_compid: false,
///     skip_crc_validation: false,
///     drop_incompatible: false,
///     verify_signature: false,
///     accept_unknown_msgid: false,
/// };
///
/// // Which is equivallent to:
/// let codec = MavlinkCodec::<true, true, false, false, false, false, false, false>::default();
/// ```
#[macro_export]
macro_rules! mavlink_codec {
    (
        // Whether to accept MAVLink V1 messages
        accept_v1: $accept_v1:expr,
        // Whether to accept MAVLink V2 messages
        accept_v2: $accept_v2:expr,
        /// Whether to drop messages with zeroed System ID
        drop_invalid_sysid: $drop_invalid_sysid:expr,
        /// Whether to drop messages with zeroed Component ID
        drop_invalid_compid: $drop_invalid_compid:expr,
        /// Whether to skip the CRC validation
        skip_crc_validation: $skip_crc_validation:expr,
        /// Whether to drop messages with unknown Incompatibility Flags
        drop_incompatible: $drop_incompatible:expr,
        /// Whether to require a valid MAVLink2 signature
        verify_signature: $verify_signature:expr,
        /// Whether to forward frames with a message id absent from the compiled dialect
        accept_unknown_msgid: $accept_unknown_msgid:expr,
    ) => {
        $crate::codec::MavlinkCodec::<
            { $accept_v1 },
            { $accept_v2 },
            { $drop_invalid_sysid },
            { $drop_invalid_compid },
            { $skip_crc_validation },
            { $drop_incompatible },
            { $verify_signature },
            { $accept_unknown_msgid },
        >::default()
    };
}
