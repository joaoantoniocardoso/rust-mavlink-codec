//! rust-mavlink interoperability for owned and borrowed frames.
//!
//! Prefer borrowed views ([`PacketRef`], [`V1PacketRef`], [`V2PacketRef`]) so header/payload
//! reads and typed parses stay in place on the caller's buffer. Owned [`Packet`] values
//! convert with [`Packet::as_ref`] without copying; only call [`PacketRef::to_owned`] (or
//! copy into `MAVLinkV*MessageRaw`) when you truly need an owned buffer.

use bytes::Bytes;
use mavlink::{calculate_crc, MavHeader, MavlinkVersion, Message};

use crate::{
    error::DecoderError,
    v1::{V1Packet, V1PacketRef},
    v2::{V2Packet, V2PacketRef},
    Packet, PacketRef,
};

impl V1PacketRef<'_> {
    /// Parses header fields and payload in place into a typed message.
    ///
    /// No frame allocation or CRC validation. Prefer this over copying into
    /// `MAVLinkV1MessageRaw` when you only need the typed form.
    #[inline]
    pub fn to_mav_message<M: Message>(
        &self,
    ) -> Result<(MavHeader, M), mavlink::error::ParserError> {
        let header = MavHeader {
            system_id: *self.system_id(),
            component_id: *self.component_id(),
            sequence: *self.sequence(),
        };
        let message = M::parse(
            MavlinkVersion::V1,
            u32::from(*self.message_id()),
            self.payload(),
        )?;
        Ok((header, message))
    }

    /// Validates the frame checksum in place against the `extra_crc` of the message id
    /// in dialect `M`, without allocating, copying, or instantiating a codec.
    #[inline]
    pub fn try_validate<M: Message>(&self) -> Result<(), DecoderError> {
        let extra_crc = M::extra_crc(u32::from(*self.message_id()));
        let calculated_crc = calculate_crc(self.checksum_data(), extra_crc);
        let expected_crc = self.checksum();
        if calculated_crc != expected_crc {
            return Err(DecoderError::InvalidCRC {
                expected_crc,
                calculated_crc,
            });
        }
        Ok(())
    }
}

impl V2PacketRef<'_> {
    /// Parses header fields and payload in place into a typed message.
    ///
    /// No frame allocation or CRC validation. Prefer this over copying into
    /// `MAVLinkV2MessageRaw` when you only need the typed form.
    #[inline]
    pub fn to_mav_message<M: Message>(
        &self,
    ) -> Result<(MavHeader, M), mavlink::error::ParserError> {
        let header = MavHeader {
            system_id: *self.system_id(),
            component_id: *self.component_id(),
            sequence: *self.sequence(),
        };
        let message = M::parse(MavlinkVersion::V2, self.message_id(), self.payload())?;
        Ok((header, message))
    }

    /// Validates the frame checksum in place against the `extra_crc` of the message id
    /// in dialect `M`, without allocating, copying, or instantiating a codec.
    #[inline]
    pub fn try_validate<M: Message>(&self) -> Result<(), DecoderError> {
        let extra_crc = M::extra_crc(self.message_id());
        let calculated_crc = calculate_crc(self.checksum_data(), extra_crc);
        let expected_crc = self.checksum();
        if calculated_crc != expected_crc {
            return Err(DecoderError::InvalidCRC {
                expected_crc,
                calculated_crc,
            });
        }
        Ok(())
    }
}

impl PacketRef<'_> {
    /// Parses header fields and payload in place into a typed message.
    ///
    /// No frame allocation or CRC validation. Prefer this over copying into
    /// `MAVLinkV*MessageRaw` when you only need the typed form.
    #[inline]
    pub fn to_mav_message<M: Message>(
        &self,
    ) -> Result<(MavHeader, M), mavlink::error::ParserError> {
        match self {
            PacketRef::V1(packet) => packet.to_mav_message(),
            PacketRef::V2(packet) => packet.to_mav_message(),
        }
    }

    /// Validates the frame checksum in place against the `extra_crc` of the message id
    /// in dialect `M`, without allocating, copying, or instantiating a codec.
    #[inline]
    pub fn try_validate<M: Message>(&self) -> Result<(), DecoderError> {
        match self {
            PacketRef::V1(packet) => packet.try_validate::<M>(),
            PacketRef::V2(packet) => packet.try_validate::<M>(),
        }
    }
}

impl From<mavlink::MAVLinkV1MessageRaw> for Packet {
    fn from(value: mavlink::MAVLinkV1MessageRaw) -> Self {
        Self::V1(V1Packet::from(value))
    }
}

impl From<mavlink::MAVLinkV2MessageRaw> for Packet {
    fn from(value: mavlink::MAVLinkV2MessageRaw) -> Self {
        Self::V2(V2Packet::from(value))
    }
}

impl From<mavlink::MAVLinkV1MessageRaw> for V1Packet {
    fn from(value: mavlink::MAVLinkV1MessageRaw) -> Self {
        Self {
            buffer: Bytes::copy_from_slice(value.raw_bytes()),
        }
    }
}

impl From<mavlink::MAVLinkV2MessageRaw> for V2Packet {
    fn from(value: mavlink::MAVLinkV2MessageRaw) -> Self {
        Self {
            buffer: Bytes::copy_from_slice(value.raw_bytes()),
        }
    }
}

impl TryFrom<V1PacketRef<'_>> for mavlink::MAVLinkV1MessageRaw {
    type Error = mavlink::error::MessageReadError;

    fn try_from(value: V1PacketRef<'_>) -> Result<Self, Self::Error> {
        Ok(raw_v1_from_slice(value.as_slice()))
    }
}

impl TryFrom<V2PacketRef<'_>> for mavlink::MAVLinkV2MessageRaw {
    type Error = mavlink::error::MessageReadError;

    fn try_from(value: V2PacketRef<'_>) -> Result<Self, Self::Error> {
        Ok(raw_v2_from_slice(value.as_slice()))
    }
}

impl TryFrom<PacketRef<'_>> for mavlink::MAVLinkV1MessageRaw {
    type Error = mavlink::error::MessageReadError;

    fn try_from(value: PacketRef<'_>) -> Result<Self, Self::Error> {
        match value {
            PacketRef::V1(packet) => Self::try_from(packet),
            PacketRef::V2(_) => Err(mavlink::error::MessageReadError::Io(std::io::Error::new(
                std::io::ErrorKind::Unsupported,
                "Expected V1 Message",
            ))),
        }
    }
}

impl TryFrom<PacketRef<'_>> for mavlink::MAVLinkV2MessageRaw {
    type Error = mavlink::error::MessageReadError;

    fn try_from(value: PacketRef<'_>) -> Result<Self, Self::Error> {
        match value {
            PacketRef::V1(_) => Err(mavlink::error::MessageReadError::Io(std::io::Error::new(
                std::io::ErrorKind::Unsupported,
                "Expected V2 Message",
            ))),
            PacketRef::V2(packet) => Self::try_from(packet),
        }
    }
}

impl TryFrom<V1Packet> for mavlink::MAVLinkV1MessageRaw {
    type Error = mavlink::error::MessageReadError;

    /// Copies into rust-mavlink’s fixed raw buffer.
    ///
    /// Prefer [`TryFrom<V1PacketRef>`] via [`V1Packet::as_ref`] when you already have borrowed
    /// bytes, or [`PacketRef::to_mav_message`] when you only need the typed message.
    fn try_from(value: V1Packet) -> Result<Self, Self::Error> {
        Self::try_from(value.as_ref())
    }
}

impl TryFrom<V2Packet> for mavlink::MAVLinkV2MessageRaw {
    type Error = mavlink::error::MessageReadError;

    /// Copies into rust-mavlink’s fixed raw buffer.
    ///
    /// Prefer [`TryFrom<V2PacketRef>`] via [`V2Packet::as_ref`] when you already have borrowed
    /// bytes, or [`PacketRef::to_mav_message`] when you only need the typed message.
    fn try_from(value: V2Packet) -> Result<Self, Self::Error> {
        Self::try_from(value.as_ref())
    }
}

impl TryFrom<Packet> for mavlink::MAVLinkV1MessageRaw {
    type Error = mavlink::error::MessageReadError;

    /// Copies into rust-mavlink’s fixed raw buffer.
    ///
    /// Prefer [`TryFrom<PacketRef>`] via [`Packet::as_ref`], or [`PacketRef::to_mav_message`]
    /// when you only need the typed message.
    fn try_from(value: Packet) -> Result<Self, Self::Error> {
        Self::try_from(value.as_ref())
    }
}

impl TryFrom<Packet> for mavlink::MAVLinkV2MessageRaw {
    type Error = mavlink::error::MessageReadError;

    /// Copies into rust-mavlink’s fixed raw buffer.
    ///
    /// Prefer [`TryFrom<PacketRef>`] via [`Packet::as_ref`], or [`PacketRef::to_mav_message`]
    /// when you only need the typed message.
    fn try_from(value: Packet) -> Result<Self, Self::Error> {
        Self::try_from(value.as_ref())
    }
}

/// Builds a raw v1 message from a frame slice with a single copy.
///
/// A free function rather than `TryFrom<&[u8]>` because the orphan rule forbids implementing a
/// foreign trait for a foreign type over a non-local slice argument.
fn raw_v1_from_slice(src_s: &[u8]) -> mavlink::MAVLinkV1MessageRaw {
    let src_s_ptr = src_s.as_ptr();
    let src_s_len = src_s.len();

    let mut message = std::mem::MaybeUninit::<mavlink::MAVLinkV1MessageRaw>::uninit();
    let dst_s_ptr = message.as_mut_ptr() as *mut u8;

    unsafe {
        let remaining_len = 263 - src_s_len;
        if remaining_len > 0 {
            std::ptr::write_bytes(dst_s_ptr.add(src_s_len), 0, remaining_len);
        }

        std::ptr::copy_nonoverlapping(src_s_ptr, dst_s_ptr, src_s_len);
        message.assume_init()
    }
}

/// Builds a raw v2 message from a frame slice with a single copy.
///
/// A free function rather than `TryFrom<&[u8]>` because the orphan rule forbids implementing a
/// foreign trait for a foreign type over a non-local slice argument.
fn raw_v2_from_slice(src_s: &[u8]) -> mavlink::MAVLinkV2MessageRaw {
    let src_s_ptr = src_s.as_ptr();
    let src_s_len = src_s.len();

    let mut message = std::mem::MaybeUninit::<mavlink::MAVLinkV2MessageRaw>::uninit();
    let dst_s_ptr = message.as_mut_ptr() as *mut u8;

    unsafe {
        let remaining_len = 280 - src_s_len;
        if remaining_len > 0 {
            std::ptr::write_bytes(dst_s_ptr.add(src_s_len), 0, remaining_len);
        }

        std::ptr::copy_nonoverlapping(src_s_ptr, dst_s_ptr, src_s_len);
        message.assume_init()
    }
}
