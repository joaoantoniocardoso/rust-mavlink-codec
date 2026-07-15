use bytes::{Buf, BufMut, BytesMut};
use log::trace;
use mavlink::calculate_crc;
use tokio_util::codec::{Decoder, Encoder};

use crate::{
    error::DecoderError,
    v1::{self, V1Packet, V1_STX},
    v2::{self, V2Packet, MAVLINK_SUPPORTED_IFLAGS, V2_STX},
    Packet, PacketRef,
};

/// MAVLink packet codec whose behavior is selected at compile time through
/// const-generic toggles.
///
/// The toggles are, in order:
///
/// * `ACCEPT_V1` -- accept MAVLink v1 frames.
/// * `ACCEPT_V2` -- accept MAVLink v2 frames.
/// * `DROP_INVALID_SYSID` -- reject frames whose system id equals zero.
/// * `DROP_INVALID_COMPID` -- reject frames whose component id equals zero.
/// * `SKIP_CRC_VALIDATION` -- skip **only** the CRC computation step.
/// * `DROP_INCOMPATIBLE` -- reject v2 frames with unsupported incompat flags.
/// * `VERIFY_SIGNATURE` -- require a valid MAVLink2 signature on accepted v2 frames; reject all v1 frames.
/// * `ACCEPT_UNKNOWN_MSGID` -- forward frames whose message id is absent from the compiled
///   dialect instead of dropping them (router use case). Such frames cannot be CRC-validated
///   (their `extra_crc` is unknown), so they are forwarded unvalidated; defaults to `false`.
#[derive(Default)]
pub struct MavlinkCodec<
    const ACCEPT_V1: bool,
    const ACCEPT_V2: bool,
    const DROP_INVALID_SYSID: bool,
    const DROP_INVALID_COMPID: bool,
    const SKIP_CRC_VALIDATION: bool,
    const DROP_INCOMPATIBLE: bool,
    const VERIFY_SIGNATURE: bool,
    const ACCEPT_UNKNOWN_MSGID: bool = false,
> {
    pub state: CodecState,
    signing: Option<crate::signing::SigningData>,
}

impl<
        const ACCEPT_V1: bool,
        const ACCEPT_V2: bool,
        const DROP_INVALID_SYSID: bool,
        const DROP_INVALID_COMPID: bool,
        const SKIP_CRC_VALIDATION: bool,
        const DROP_INCOMPATIBLE: bool,
        const VERIFY_SIGNATURE: bool,
        const ACCEPT_UNKNOWN_MSGID: bool,
    > std::fmt::Debug
    for MavlinkCodec<
        ACCEPT_V1,
        ACCEPT_V2,
        DROP_INVALID_SYSID,
        DROP_INVALID_COMPID,
        SKIP_CRC_VALIDATION,
        DROP_INCOMPATIBLE,
        VERIFY_SIGNATURE,
        ACCEPT_UNKNOWN_MSGID,
    >
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("MavlinkCodec")
            .field("state", &self.state)
            .field("signing", &self.signing.as_ref().map(|_| ".."))
            .finish()
    }
}

impl<
        const ACCEPT_V1: bool,
        const ACCEPT_V2: bool,
        const DROP_INVALID_SYSID: bool,
        const DROP_INVALID_COMPID: bool,
        const SKIP_CRC_VALIDATION: bool,
        const DROP_INCOMPATIBLE: bool,
        const VERIFY_SIGNATURE: bool,
        const ACCEPT_UNKNOWN_MSGID: bool,
    >
    MavlinkCodec<
        ACCEPT_V1,
        ACCEPT_V2,
        DROP_INVALID_SYSID,
        DROP_INVALID_COMPID,
        SKIP_CRC_VALIDATION,
        DROP_INCOMPATIBLE,
        VERIFY_SIGNATURE,
        ACCEPT_UNKNOWN_MSGID,
    >
{
    pub fn with_signing(signing: crate::signing::SigningData) -> Self {
        Self {
            state: CodecState::default(),
            signing: Some(signing),
        }
    }

    /// Validates a complete MAVLink frame in place and returns a borrowed view.
    ///
    /// `buf` must begin at the frame STX. Extra trailing bytes are ignored; the returned
    /// [`PacketRef`] borrows only the first complete frame. Applies the same policy toggles as
    /// [`Decoder::decode`] (CRC, sysid/compid, incompat flags, signature, unknown msgid).
    pub fn try_validate<'a>(&mut self, buf: &'a [u8]) -> Result<PacketRef<'a>, DecoderError> {
        let Some(&stx) = buf.first() else {
            return Err(DecoderError::Incomplete);
        };

        match stx {
            V1_STX if ACCEPT_V1 => self.try_validate_v1(buf),
            V2_STX if ACCEPT_V2 => self.try_validate_v2(buf),
            _ => Err(DecoderError::InvalidStx { stx }),
        }
    }

    fn try_validate_v1<'a>(&mut self, buf: &'a [u8]) -> Result<PacketRef<'a>, DecoderError> {
        if buf.len() < V1Packet::STX_SIZE + V1Packet::HEADER_SIZE {
            return Err(DecoderError::Incomplete);
        }

        let packet_size = v1::packet_size(&buf);
        if buf.len() < packet_size {
            return Err(DecoderError::Incomplete);
        }
        let frame = &buf[..packet_size];

        if DROP_INVALID_SYSID {
            let sysid = *v1::sysid(&frame);
            if sysid == 0 {
                return Err(DecoderError::InvalidSystemID { sysid });
            }
        }

        if DROP_INVALID_COMPID {
            let compid = *v1::compid(&frame);
            if compid == 0 {
                return Err(DecoderError::InvalidComponentID { compid });
            }
        }

        if !SKIP_CRC_VALIDATION {
            let msgid = u32::from(*v1::msgid(&frame));
            match get_extra_crc(msgid) {
                None => return Err(DecoderError::UnknownMessageID { msgid }),
                Some(extra_crc) => {
                    let checksum_data = v1::checksum_data(&frame);
                    let calculated_crc = calculate_crc(checksum_data, extra_crc);
                    let expected_crc = v1::checksum(&frame);
                    if calculated_crc != expected_crc
                        && !(ACCEPT_UNKNOWN_MSGID && !is_known_msgid(msgid))
                    {
                        return Err(DecoderError::InvalidCRC {
                            expected_crc,
                            calculated_crc,
                        });
                    }
                }
            }
        }

        if VERIFY_SIGNATURE {
            return Err(DecoderError::InvalidSignature);
        }

        Ok(PacketRef::V1(crate::v1::V1PacketRef::from_buffer(frame)))
    }

    fn try_validate_v2<'a>(&mut self, buf: &'a [u8]) -> Result<PacketRef<'a>, DecoderError> {
        if buf.len() < V2Packet::STX_SIZE + V2Packet::HEADER_SIZE {
            return Err(DecoderError::Incomplete);
        }

        let packet_size = v2::packet_size(&buf);
        if buf.len() < packet_size {
            return Err(DecoderError::Incomplete);
        }
        let frame = &buf[..packet_size];

        if DROP_INCOMPATIBLE {
            let incompat_flags = *v2::incompat_flags(&frame);
            if incompat_flags & !MAVLINK_SUPPORTED_IFLAGS > 0 {
                return Err(DecoderError::Incompatible { incompat_flags });
            }
        }

        if DROP_INVALID_SYSID {
            let sysid = *v2::sysid(&frame);
            if sysid == 0 {
                return Err(DecoderError::InvalidSystemID { sysid });
            }
        }

        if DROP_INVALID_COMPID {
            let compid = *v2::compid(&frame);
            if compid == 0 {
                return Err(DecoderError::InvalidComponentID { compid });
            }
        }

        if !SKIP_CRC_VALIDATION {
            let msgid = v2::msgid(&frame);
            match get_extra_crc(msgid) {
                None => return Err(DecoderError::UnknownMessageID { msgid }),
                Some(extra_crc) => {
                    let checksum_data = v2::checksum_data(&frame);
                    let calculated_crc = calculate_crc(checksum_data, extra_crc);
                    let expected_crc = v2::checksum(&frame);
                    if calculated_crc != expected_crc
                        && !(ACCEPT_UNKNOWN_MSGID && !is_known_msgid(msgid))
                    {
                        return Err(DecoderError::InvalidCRC {
                            expected_crc,
                            calculated_crc,
                        });
                    }
                }
            }
        }

        if VERIFY_SIGNATURE {
            let signature_ok = self
                .signing
                .as_mut()
                .is_some_and(|signing| signing.verify_signature(frame));
            if !signature_ok {
                return Err(DecoderError::InvalidSignature);
            }
        }

        Ok(PacketRef::V2(crate::v2::V2PacketRef::from_buffer(frame)))
    }
}

#[derive(Debug, Default)]
pub enum CodecState {
    #[default]
    WaitingForStx,
    WaitingV1PacketHeader,
    WaitingV2PacketHeader,
    ValidatingV1Packet {
        packet_size: usize,
    },
    ValidatingV2Packet {
        packet_size: usize,
    },
    CopyV1Packet {
        packet_size: usize,
    },
    CopyV2Packet {
        packet_size: usize,
    },
    /// Resync state entered when a frame is rejected (bad CRC, policy drop, bad signature,
    /// unsupported incompat flags).
    ///
    /// The whole declared frame is discarded (`remaining = packet_size`) rather than
    /// rescanning for the next STX from the byte after the marker. This is a deliberate
    /// security default: refusing to resync inside a rejected frame closes packet-in-packet
    /// injection, where a forged outer frame hides a valid inner frame in its payload (see
    /// the `packet_in_packet` and `desync_liveness` exploit suites, and rust-mavlink PR #508).
    ///
    /// The trade-off is liveness: a corrupt declared `len` can over-skip and drop a real
    /// subsequent frame. A forward-rescan toggle (`RESYNC_FROM_NEXT_STX`) was considered and
    /// rejected as a default because it reopens that injection vector; see `SECURITY.md`.
    Discarding {
        remaining: usize,
    },
}

impl<
        const ACCEPT_V1: bool,
        const ACCEPT_V2: bool,
        const DROP_INVALID_SYSID: bool,
        const DROP_INVALID_COMPID: bool,
        const SKIP_CRC_VALIDATION: bool,
        const DROP_INCOMPATIBLE: bool,
        const VERIFY_SIGNATURE: bool,
        const ACCEPT_UNKNOWN_MSGID: bool,
    > Decoder
    for MavlinkCodec<
        ACCEPT_V1,
        ACCEPT_V2,
        DROP_INVALID_SYSID,
        DROP_INVALID_COMPID,
        SKIP_CRC_VALIDATION,
        DROP_INCOMPATIBLE,
        VERIFY_SIGNATURE,
        ACCEPT_UNKNOWN_MSGID,
    >
{
    type Item = Result<Packet, DecoderError>;
    type Error = std::io::Error;

    fn decode(&mut self, buf: &mut BytesMut) -> Result<Option<Self::Item>, Self::Error> {
        trace!("Decoding: {:?}", &buf[..]);

        loop {
            match self.state {
                CodecState::WaitingForStx => {
                    trace!("Waitig for STX...");

                    if buf.is_empty() {
                        trace!(
                            "Not enough data, buf.len: {:?}, buf.capacity: {:?}",
                            buf.len(),
                            buf.capacity()
                        );
                        return Ok(None);
                    }

                    match buf[0] {
                        V1_STX if ACCEPT_V1 => self.state = CodecState::WaitingV1PacketHeader,
                        V2_STX if ACCEPT_V2 => self.state = CodecState::WaitingV2PacketHeader,
                        _ => {
                            trace!("Invalid STX byte: {}", buf[0]);
                            buf.advance(V1Packet::STX_SIZE);
                            continue;
                        }
                    }
                }
                // V1 Codec
                CodecState::WaitingV1PacketHeader if ACCEPT_V1 => {
                    if buf.len() < V1Packet::HEADER_SIZE {
                        trace!(
                            "Not enough data, buf.len: {:?}, buf.capacity: {:?}",
                            buf.len(),
                            buf.capacity()
                        );
                        return Ok(None);
                    }

                    let packet_size = v1::packet_size(buf);
                    self.state = CodecState::ValidatingV1Packet { packet_size };
                }
                CodecState::ValidatingV1Packet { packet_size } if ACCEPT_V1 => {
                    if buf.len() < packet_size {
                        trace!(
                            "Not enough data, buf.len: {:?}, buf.capacity: {:?}",
                            buf.len(),
                            buf.capacity()
                        );
                        return Ok(None);
                    }

                    // System ID validation
                    if DROP_INVALID_SYSID {
                        let sysid = *v1::sysid(buf);
                        if sysid == 0 {
                            trace!("Invalid SystemID: {sysid:?}. Data: {:?}", &buf[..]);

                            self.state = CodecState::Discarding {
                                remaining: packet_size,
                            };

                            return Ok(Some(Err(DecoderError::InvalidSystemID { sysid })));
                        }
                    }

                    // Component ID validation
                    if DROP_INVALID_COMPID {
                        let compid = *v1::compid(buf);
                        if compid == 0 {
                            trace!("Invalid SystemID: {compid:?}. Data: {:?}", &buf[..]);

                            self.state = CodecState::Discarding {
                                remaining: packet_size,
                            };

                            return Ok(Some(Err(DecoderError::InvalidComponentID { compid })));
                        }
                    }

                    // CRC Validation
                    if !SKIP_CRC_VALIDATION {
                        let msgid = *v1::msgid(buf) as u32;
                        let Some(extra_crc) = get_extra_crc(msgid) else {
                            trace!("Unknown message ID {msgid:?}. Data: {:?}", &buf[..]);

                            self.state = CodecState::Discarding {
                                remaining: packet_size,
                            };

                            return Ok(Some(Err(DecoderError::UnknownMessageID { msgid })));
                        };
                        let checksum_data = v1::checksum_data(buf);
                        let calculated_crc = calculate_crc(checksum_data, extra_crc);

                        let expected_crc = v1::checksum(buf);
                        if calculated_crc.ne(&expected_crc) {
                            // An unknown message id always fails CRC here because we lack its
                            // `extra_crc`. In router mode, forward such frames unvalidated
                            // instead of dropping them; genuinely corrupt known frames are
                            // still rejected.
                            if ACCEPT_UNKNOWN_MSGID && !is_known_msgid(msgid) {
                                trace!(
                                    "Unknown message ID {msgid:?}; forwarding unvalidated frame."
                                );

                                self.state = CodecState::CopyV1Packet { packet_size };
                                continue;
                            }

                            trace!(
                                "Invalid CRC: expected: {expected_crc:?}, calculated: {calculated_crc:?}. checksum_data: {checksum_data:?}"
                            );

                            self.state = CodecState::Discarding {
                                remaining: packet_size,
                            };

                            return Ok(Some(Err(DecoderError::InvalidCRC {
                                expected_crc,
                                calculated_crc,
                            })));
                        }
                    } else {
                        trace!("CRC Validation skipped.");
                    }

                    // Signature Verification
                    if VERIFY_SIGNATURE {
                        self.state = CodecState::Discarding {
                            remaining: packet_size,
                        };

                        return Ok(Some(Err(DecoderError::InvalidSignature)));
                    } else {
                        trace!("Signature Verification skipped.");
                    }

                    self.state = CodecState::CopyV1Packet { packet_size };
                }
                CodecState::CopyV1Packet { packet_size } if ACCEPT_V1 => {
                    let buf_packet = buf.split_to(packet_size);
                    let packet = V1Packet {
                        buffer: buf_packet.freeze(),
                    };

                    self.state = CodecState::WaitingForStx;
                    return Ok(Some(Ok(Packet::V1(packet))));
                }
                // V2 Codec
                CodecState::WaitingV2PacketHeader if ACCEPT_V2 => {
                    if buf.len() < V2Packet::HEADER_SIZE {
                        trace!(
                            "Not enough data, buf.len: {:?}, buf.capacity: {:?}",
                            buf.len(),
                            buf.capacity()
                        );
                        return Ok(None);
                    }

                    let packet_size = v2::packet_size(buf);

                    if DROP_INCOMPATIBLE {
                        let incompat_flags = *v2::incompat_flags(buf);
                        if incompat_flags & !MAVLINK_SUPPORTED_IFLAGS > 0 {
                            self.state = CodecState::Discarding {
                                remaining: packet_size,
                            };

                            return Ok(Some(Err(DecoderError::Incompatible { incompat_flags })));
                        }
                    }

                    self.state = CodecState::ValidatingV2Packet { packet_size };
                }
                CodecState::ValidatingV2Packet { packet_size } if ACCEPT_V2 => {
                    if buf.len() < packet_size {
                        trace!(
                            "Not enough data, buf.len: {:?}, buf.capacity: {:?}",
                            buf.len(),
                            buf.capacity()
                        );
                        return Ok(None);
                    }

                    // System ID validation
                    if DROP_INVALID_SYSID {
                        let sysid = *v2::sysid(buf);
                        if sysid == 0 {
                            trace!("Invalid SystemID: {sysid:?}. Data: {:?}", &buf[..]);

                            self.state = CodecState::Discarding {
                                remaining: packet_size,
                            };

                            return Ok(Some(Err(DecoderError::InvalidSystemID { sysid })));
                        }
                    }

                    // Component ID validation
                    if DROP_INVALID_COMPID {
                        let compid = *v2::compid(buf);
                        if compid == 0 {
                            trace!("Invalid SystemID: {compid:?}. Data: {:?}", &buf[..]);

                            self.state = CodecState::Discarding {
                                remaining: packet_size,
                            };

                            return Ok(Some(Err(DecoderError::InvalidComponentID { compid })));
                        }
                    }

                    // CRC Validation
                    if !SKIP_CRC_VALIDATION {
                        let msgid = v2::msgid(buf);
                        let Some(extra_crc) = get_extra_crc(msgid) else {
                            trace!("Unknown message ID {msgid:?}. Data: {:?}", &buf[..]);

                            self.state = CodecState::Discarding {
                                remaining: packet_size,
                            };

                            return Ok(Some(Err(DecoderError::UnknownMessageID { msgid })));
                        };
                        let checksum_data = v2::checksum_data(buf);
                        let calculated_crc = calculate_crc(checksum_data, extra_crc);

                        let expected_crc = v2::checksum(buf);
                        if calculated_crc.ne(&expected_crc) {
                            // An unknown message id always fails CRC here because we lack its
                            // `extra_crc`. In router mode, forward such frames unvalidated
                            // instead of dropping them; genuinely corrupt known frames are
                            // still rejected.
                            if ACCEPT_UNKNOWN_MSGID && !is_known_msgid(msgid) {
                                trace!(
                                    "Unknown message ID {msgid:?}; forwarding unvalidated frame."
                                );

                                self.state = CodecState::CopyV2Packet { packet_size };
                                continue;
                            }

                            trace!(
                                "Invalid CRC: expected: {expected_crc:?}, calculated: {calculated_crc:?}. checksum_data: {checksum_data:?}"
                            );

                            self.state = CodecState::Discarding {
                                remaining: packet_size,
                            };

                            return Ok(Some(Err(DecoderError::InvalidCRC {
                                expected_crc,
                                calculated_crc,
                            })));
                        }
                    } else {
                        trace!("CRC Validation skipped.");
                    }

                    // Signature Verification
                    if VERIFY_SIGNATURE {
                        // Verify in place over the buffered frame bytes (zero-copy).
                        let signature_ok = self
                            .signing
                            .as_mut()
                            .is_some_and(|signing| signing.verify_signature(&buf[..packet_size]));
                        if !signature_ok {
                            self.state = CodecState::Discarding {
                                remaining: packet_size,
                            };

                            return Ok(Some(Err(DecoderError::InvalidSignature)));
                        }
                    } else {
                        trace!("Signature Verification skipped.");
                    }

                    self.state = CodecState::CopyV2Packet { packet_size };
                }
                CodecState::CopyV2Packet { packet_size } if ACCEPT_V2 => {
                    let buf_packet = buf.split_to(packet_size);

                    let packet = V2Packet {
                        buffer: buf_packet.freeze(),
                    };

                    self.state = CodecState::WaitingForStx;
                    return Ok(Some(Ok(Packet::V2(packet))));
                }
                CodecState::Discarding { remaining } => {
                    let to_discard = remaining.min(buf.len());
                    buf.advance(to_discard);

                    let left = remaining - to_discard;
                    if left > 0 {
                        trace!("Discarding rejected frame, {left} bytes still pending");
                        self.state = CodecState::Discarding { remaining: left };
                        return Ok(None);
                    }

                    self.state = CodecState::WaitingForStx;
                }
                _ => {
                    unreachable!()
                }
            }
        }
    }
}

impl<
        const ACCEPT_V1: bool,
        const ACCEPT_V2: bool,
        const DROP_INVALID_SYSID: bool,
        const DROP_INVALID_COMPID: bool,
        const SKIP_CRC_VALIDATION: bool,
        const DROP_INCOMPATIBLE: bool,
        const VERIFY_SIGNATURE: bool,
        const ACCEPT_UNKNOWN_MSGID: bool,
    > Encoder<PacketRef<'_>>
    for MavlinkCodec<
        ACCEPT_V1,
        ACCEPT_V2,
        DROP_INVALID_SYSID,
        DROP_INVALID_COMPID,
        SKIP_CRC_VALIDATION,
        DROP_INCOMPATIBLE,
        VERIFY_SIGNATURE,
        ACCEPT_UNKNOWN_MSGID,
    >
{
    type Error = std::io::Error;

    fn encode(&mut self, packet: PacketRef<'_>, buf: &mut BytesMut) -> Result<(), Self::Error> {
        trace!("encoding...");
        match packet {
            PacketRef::V1(v1_packet) if ACCEPT_V1 => {
                trace!("v1 package written");
                buf.put(v1_packet.as_slice());
            }
            PacketRef::V2(v2_packet) if ACCEPT_V2 => {
                trace!("v2 package written");
                buf.put(v2_packet.as_slice());
            }
            _ => {
                trace!("unsupported package version");
                return Err(std::io::Error::new(
                    std::io::ErrorKind::InvalidData,
                    "Unsupported packet version",
                ));
            }
        }

        Ok(())
    }
}

impl<
        const ACCEPT_V1: bool,
        const ACCEPT_V2: bool,
        const DROP_INVALID_SYSID: bool,
        const DROP_INVALID_COMPID: bool,
        const SKIP_CRC_VALIDATION: bool,
        const DROP_INCOMPATIBLE: bool,
        const VERIFY_SIGNATURE: bool,
        const ACCEPT_UNKNOWN_MSGID: bool,
    > Encoder<Packet>
    for MavlinkCodec<
        ACCEPT_V1,
        ACCEPT_V2,
        DROP_INVALID_SYSID,
        DROP_INVALID_COMPID,
        SKIP_CRC_VALIDATION,
        DROP_INCOMPATIBLE,
        VERIFY_SIGNATURE,
        ACCEPT_UNKNOWN_MSGID,
    >
{
    type Error = std::io::Error;

    fn encode(&mut self, packet: Packet, buf: &mut BytesMut) -> Result<(), Self::Error> {
        Encoder::<PacketRef<'_>>::encode(self, packet.as_ref(), buf)
    }
}

#[inline(always)]
pub fn get_extra_crc(msgid: u32) -> Option<u8> {
    use mavlink::Message;

    Some(mavlink::dialects::ardupilotmega::MavMessage::extra_crc(
        msgid,
    ))
}

/// Returns whether `msgid` exists in the compiled dialect.
///
/// Used on the CRC-failure path to distinguish a frame carrying an unknown message id
/// (which we cannot CRC-validate, lacking its `extra_crc`) from a genuinely corrupt
/// known frame. This runs only after a CRC mismatch, so it stays off the hot path.
#[inline(always)]
pub fn is_known_msgid(msgid: u32) -> bool {
    use mavlink::Message;

    mavlink::dialects::ardupilotmega::MavMessage::default_message_from_id(msgid).is_some()
}

#[cfg(test)]
mod test_encode {
    use super::*;
    use mavlink::{
        dialects::ardupilotmega::MavMessage, MAVLinkV1MessageRaw, MAVLinkV2MessageRaw, MavHeader,
        Message,
    };

    #[test]
    fn test_encode_v1() {
        let mut codec = MavlinkCodec::<true, true, false, false, false, false, false>::default();

        let v1_packet = {
            let header = MavHeader {
                system_id: 1,
                component_id: 1,
                sequence: 0,
            };

            let message_data = MavMessage::default_message_from_id(0).unwrap(); // Heartbeat message
            let mut raw_v1_message = MAVLinkV1MessageRaw::new();
            raw_v1_message.serialize_message(header, &message_data);
            V1Packet::from(raw_v1_message)
        };
        let mut buf = BytesMut::with_capacity(V1Packet::MAX_PACKET_SIZE);

        codec
            .encode(Packet::V1(v1_packet.clone()), &mut buf)
            .unwrap();

        assert_eq!(&buf[..v1_packet.packet_size()], v1_packet.as_slice())
    }

    #[test]
    fn test_encode_v2() {
        let mut codec = MavlinkCodec::<true, true, false, false, false, false, false>::default();

        let v2_packet = {
            let header = MavHeader {
                system_id: 1,
                component_id: 1,
                sequence: 0,
            };

            let message_data = MavMessage::default_message_from_id(0).unwrap(); // Heartbeat message
            let mut raw_v2_message = MAVLinkV2MessageRaw::new();
            raw_v2_message.serialize_message(header, &message_data);
            V2Packet::from(raw_v2_message)
        };

        let mut buf = BytesMut::with_capacity(V2Packet::MAX_PACKET_SIZE);

        codec
            .encode(Packet::V2(v2_packet.clone()), &mut buf)
            .unwrap();

        assert_eq!(&buf[..v2_packet.packet_size()], v2_packet.as_slice())
    }

    #[test]
    fn test_encode_packet_ref_v2() {
        let mut codec = MavlinkCodec::<true, true, false, false, false, false, false>::default();

        let v2_packet = {
            let header = MavHeader {
                system_id: 1,
                component_id: 1,
                sequence: 0,
            };

            let message_data = MavMessage::default_message_from_id(0).unwrap(); // Heartbeat message
            let mut raw_v2_message = MAVLinkV2MessageRaw::new();
            raw_v2_message.serialize_message(header, &message_data);
            V2Packet::from(raw_v2_message)
        };

        let mut buf = BytesMut::with_capacity(V2Packet::MAX_PACKET_SIZE);

        codec
            .encode(PacketRef::V2(v2_packet.as_ref()), &mut buf)
            .unwrap();

        assert_eq!(&buf[..v2_packet.packet_size()], v2_packet.as_slice())
    }
}

#[cfg(test)]
mod test_decode {
    use super::*;
    use mavlink::{
        dialects::ardupilotmega::MavMessage, MAVLinkV1MessageRaw, MAVLinkV2MessageRaw, MavHeader,
        Message,
    };

    #[test]
    fn test_decode_v1() {
        let mut codec = MavlinkCodec::<true, false, false, false, false, false, false>::default();

        let mut buf = BytesMut::with_capacity(V1Packet::MAX_PACKET_SIZE);

        let expected_packet = {
            let header = MavHeader {
                system_id: 1,
                component_id: 1,
                sequence: 0,
            };

            let message_data = MavMessage::default_message_from_id(0).unwrap(); // Heartbeat message
            let mut raw_v1_message = MAVLinkV1MessageRaw::new();
            raw_v1_message.serialize_message(header, &message_data);

            buf.put(raw_v1_message.raw_bytes());

            Packet::V1(V1Packet::from(raw_v1_message))
        };
        assert!(!buf.is_empty());

        let packet = codec.decode(&mut buf).unwrap().unwrap().unwrap();

        assert_eq!(packet, expected_packet);
    }

    #[test]
    fn test_decode_v2() {
        let mut codec = MavlinkCodec::<false, true, false, false, false, false, false>::default();

        let mut buf = BytesMut::with_capacity(V1Packet::MAX_PACKET_SIZE);

        let expected_packet = {
            let header = MavHeader {
                system_id: 1,
                component_id: 1,
                sequence: 0,
            };

            let message_data = MavMessage::default_message_from_id(0).unwrap(); // Heartbeat message
            let mut raw_v2_message = MAVLinkV2MessageRaw::new();
            raw_v2_message.serialize_message(header, &message_data);

            buf.put(raw_v2_message.raw_bytes());

            Packet::V2(V2Packet::from(raw_v2_message))
        };
        assert!(!buf.is_empty());

        let packet = codec.decode(&mut buf).unwrap().unwrap().unwrap();

        assert_eq!(packet, expected_packet);
    }
}

#[cfg(test)]
mod test_try_validate {
    use super::*;
    use mavlink::{dialects::ardupilotmega::MavMessage, MAVLinkV2MessageRaw, MavHeader, Message};

    fn heartbeat_v2_bytes() -> Vec<u8> {
        let header = MavHeader {
            system_id: 1,
            component_id: 1,
            sequence: 0,
        };
        let message_data = MavMessage::default_message_from_id(0).unwrap();
        let mut raw = MAVLinkV2MessageRaw::new();
        raw.serialize_message(header, &message_data);
        raw.raw_bytes().to_vec()
    }

    #[test]
    fn try_validate_accepts_valid_v2() {
        let mut codec = MavlinkCodec::<true, true, false, false, false, false, false>::default();
        let bytes = heartbeat_v2_bytes();
        let packet = codec.try_validate(&bytes).unwrap();
        assert_eq!(packet.message_id(), 0);
        assert_eq!(*packet.system_id(), 1);
    }

    #[test]
    fn try_validate_rejects_bad_crc() {
        let mut codec = MavlinkCodec::<true, true, false, false, false, false, false>::default();
        let mut bytes = heartbeat_v2_bytes();
        let last = bytes.len() - 1;
        bytes[last] ^= 0xff;
        assert!(matches!(
            codec.try_validate(&bytes),
            Err(DecoderError::InvalidCRC { .. })
        ));
    }

    #[test]
    fn try_validate_rejects_incomplete() {
        let mut codec = MavlinkCodec::<true, true, false, false, false, false, false>::default();
        let bytes = heartbeat_v2_bytes();
        assert!(matches!(
            codec.try_validate(&bytes[..5]),
            Err(DecoderError::Incomplete)
        ));
    }
}
