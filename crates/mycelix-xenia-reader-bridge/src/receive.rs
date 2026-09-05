use xenia_peer_core::{
    AuthenticatedPeerApplicationChannelErrorV1, AuthenticatedPeerApplicationChannelV1,
    transport::Transport,
};
use mycelix_nix_cache::NixStoreHashV1;
use thiserror::Error;

use crate::{
    CONTENT_FABRIC_READER_PAYLOAD_TYPE_V1, CONTENT_FABRIC_RESPONSE_DATA_MAX_V1,
    CONTENT_FABRIC_RESPONSE_FRAME_HEADER_LEN_V1, CONTENT_FABRIC_RESPONSE_FRAME_SCHEMA_V1,
    NixReaderOperationV1, encode_nix_reader_request_v1,
};

const CONTENT_FABRIC_RESPONSE_FRAME_MAGIC_V1: [u8; 4] = *b"MCFS";
const RESPONSE_FRAME_KIND_DATA_V1: u8 = 1;
const RESPONSE_FRAME_KIND_END_V1: u8 = 2;

/// Hard receiver-side ceiling for one complete NarInfo response.
///
/// NarInfo is metadata and should remain small. This bound prevents an
/// authenticated peer from turning a valid response stream into unbounded
/// metadata delivery when there is no prior exact NarInfo size commitment.
pub const MAX_NARINFO_RESPONSE_BYTES_V1: u64 = 1024 * 1024;

/// Fail-closed failures while sending one MCFR request and receiving its serial
/// MCFS response over the same authenticated Xenia application channel.
#[derive(Debug, Error)]
pub enum XeniaNixResponseReceiveErrorV1 {
    /// The supplied Xenia channel is not the Content Fabric application domain.
    #[error("unexpected Xenia application payload type: expected 0x{expected:02x}, got 0x{actual:02x}")]
    UnexpectedPayloadType {
        /// Required Content Fabric payload-domain byte.
        expected: u8,
        /// Actual channel/opened-payload byte.
        actual: u8,
    },
    /// Raw NAR receive requires a non-zero NarSize from previously validated
    /// metadata.
    #[error("expected raw NAR size must be non-zero")]
    InvalidExpectedNarSize,
    /// The receiver has already failed and can no longer release its channel.
    #[error("Content Fabric Xenia response receiver is terminal")]
    Terminal,
    /// A caller attempted another receive after a valid End frame.
    #[error("Content Fabric Xenia response is already complete")]
    AlreadyCompleted,
    /// `finish()` was attempted before a valid End frame completed the stream.
    #[error("Content Fabric Xenia response ended without a valid End frame")]
    IncompleteResponse,
    /// An MCFS frame was shorter than its fixed v1 header.
    #[error("Content Fabric response frame too short: {0} bytes")]
    FrameTooShort(usize),
    /// The response-family magic was not exactly `MCFS`.
    #[error("invalid Content Fabric response frame magic")]
    InvalidMagic,
    /// The response schema is not supported by this implementation.
    #[error("unsupported Content Fabric response schema version {0}")]
    UnsupportedSchemaVersion(u16),
    /// The response frame kind is not assigned by schema v1.
    #[error("unsupported Content Fabric response frame kind {0}")]
    UnsupportedFrameKind(u8),
    /// The response operation is not assigned by schema v1.
    #[error("unsupported Content Fabric response operation {0}")]
    UnsupportedOperation(u8),
    /// A response attempted to switch operation relative to the exact request.
    #[error("Content Fabric response operation does not match request")]
    OperationMismatch,
    /// A response attempted to switch store hash relative to the exact request.
    #[error("Content Fabric response store hash does not match request")]
    StoreHashMismatch,
    /// The next response-frame sequence was not exact.
    #[error("Content Fabric response sequence mismatch: expected {expected}, got {actual}")]
    SequenceMismatch {
        /// Required next sequence.
        expected: u64,
        /// Received sequence.
        actual: u64,
    },
    /// The declared payload length disagreed with the exact frame length.
    #[error("Content Fabric response payload length mismatch: declared {declared}, actual {actual}")]
    PayloadLengthMismatch {
        /// Header-declared payload bytes.
        declared: usize,
        /// Bytes actually present after the header.
        actual: usize,
    },
    /// One data frame exceeded the frozen v1 payload ceiling.
    #[error("Content Fabric response data frame too large: {0} bytes")]
    FramePayloadTooLarge(usize),
    /// A Data frame carried no representation bytes.
    #[error("Content Fabric response Data frame must not be empty")]
    EmptyDataFrame,
    /// A Data frame tried to use the End-only total-byte field.
    #[error("Content Fabric response Data frame total-byte field must be zero")]
    DataFrameTotalNonZero,
    /// An End frame contained representation payload bytes.
    #[error("Content Fabric response End frame must not contain payload bytes")]
    EndFrameHasPayload,
    /// Accumulated response bytes exceeded the operation-specific receive bound.
    #[error("Content Fabric response exceeds receive bound: limit={limit}, attempted={attempted}")]
    ResponseTooLarge {
        /// Maximum bytes permitted for this response.
        limit: u64,
        /// Accumulated bytes after applying the candidate frame.
        attempted: u64,
    },
    /// The End frame's committed total did not equal accumulated Data bytes.
    #[error("Content Fabric response End total mismatch: accumulated={accumulated}, end={end_total}")]
    EndTotalMismatch {
        /// Bytes accumulated from valid Data frames.
        accumulated: u64,
        /// Total committed by End.
        end_total: u64,
    },
    /// A raw NAR response did not end at the exact NarSize supplied when the
    /// request was sent.
    #[error("raw NAR response size mismatch: expected={expected}, actual={actual}")]
    ExpectedNarSizeMismatch {
        /// Previously validated expected NarSize.
        expected: u64,
        /// Complete MCFS Data byte count.
        actual: u64,
    },
    /// Response byte accounting overflowed.
    #[error("Content Fabric response byte accounting overflow")]
    ByteCountOverflow,
    /// Response sequence arithmetic overflowed.
    #[error("Content Fabric response sequence exhausted")]
    SequenceOverflow,
    /// Same-channel Xenia request send or response receive/open failed.
    #[error(transparent)]
    Xenia(#[from] AuthenticatedPeerApplicationChannelErrorV1),
}

/// One authenticated, structurally validated MCFS Data payload from an
/// in-progress response.
///
/// This value is deliberately non-`Clone` and has no `Debug` implementation so
/// potentially restricted representation bytes are not copied or logged by
/// convenience. A chunk is **not** proof that the response completed: consumers
/// must require [`CompletedXeniaNixResponseV1`] before treating the stream as a
/// complete representation.
pub struct ReceivedXeniaNixResponseChunkV1 {
    bytes: Vec<u8>,
    sequence: u64,
    cumulative_bytes: u64,
}

impl ReceivedXeniaNixResponseChunkV1 {
    /// Exact authenticated representation bytes in this response frame.
    pub fn bytes(&self) -> &[u8] {
        &self.bytes
    }

    /// Consume the transient chunk and return its representation bytes.
    pub fn into_bytes(self) -> Vec<u8> {
        self.bytes
    }

    /// Exact zero-based MCFS Data-frame sequence.
    pub const fn sequence(&self) -> u64 {
        self.sequence
    }

    /// Total validated Data bytes through this frame.
    pub const fn cumulative_bytes(&self) -> u64 {
        self.cumulative_bytes
    }
}

/// One event produced by the serial MCFS receiver.
pub enum XeniaNixResponseEventV1 {
    /// One authenticated and structurally valid Data frame.
    Data(ReceivedXeniaNixResponseChunkV1),
    /// A valid terminal End frame was received.
    End,
}

/// Audit-only proof that one serial MCFS stream reached a valid End frame.
///
/// This proves response framing/sequence/length completion only. It does not
/// prove a raw NAR digest, Nix signature, trusted-public-key policy, or install
/// authorization. Raw NAR integrity still belongs to the subsequent CF-03 CAS
/// ingest/verification boundary.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CompletedXeniaNixResponseV1 {
    operation: NixReaderOperationV1,
    store_hash: NixStoreHashV1,
    total_bytes: u64,
    data_frames: u64,
}

impl CompletedXeniaNixResponseV1 {
    /// Exact operation originally sent in MCFR and repeated by every MCFS frame.
    pub const fn operation(&self) -> NixReaderOperationV1 {
        self.operation
    }

    /// Exact store hash originally requested and repeated by every MCFS frame.
    pub const fn store_hash(&self) -> &NixStoreHashV1 {
        &self.store_hash
    }

    /// Exact accumulated representation byte count committed by End.
    pub const fn total_bytes(&self) -> u64 {
        self.total_bytes
    }

    /// Number of validated Data frames before End.
    pub const fn data_frames(&self) -> u64 {
        self.data_frames
    }
}

/// Same-channel serial response receiver created only by successfully sending
/// one exact MCFR request through that channel.
///
/// There is deliberately no public constructor from an arbitrary channel. The
/// channel is owned by this receiver and can be recovered only through
/// [`Self::finish`] after a valid End frame. Any Xenia/framing/state failure
/// makes the receiver terminal; dropping or failing the receiver drops the
/// authority-bearing channel.
pub struct XeniaNixResponseReceiverV1<T: Transport> {
    channel: Option<AuthenticatedPeerApplicationChannelV1<T>>,
    state: ResponseStateV1,
    terminal: bool,
}

impl<T: Transport> XeniaNixResponseReceiverV1<T> {
    /// Receive and validate the next authenticated MCFS response frame from the
    /// exact channel used to send this request.
    pub async fn recv_next(
        &mut self,
    ) -> Result<XeniaNixResponseEventV1, XeniaNixResponseReceiveErrorV1> {
        if self.terminal {
            return Err(XeniaNixResponseReceiveErrorV1::Terminal);
        }
        if self.state.completed {
            return Err(XeniaNixResponseReceiveErrorV1::AlreadyCompleted);
        }

        let channel = self
            .channel
            .as_mut()
            .ok_or(XeniaNixResponseReceiveErrorV1::Terminal)?;
        let opened = match channel.recv_opened_payload().await {
            Ok(opened) => opened,
            Err(error) => {
                self.terminal = true;
                return Err(error.into());
            }
        };
        let actual_payload_type = opened.payload_type().value();
        if actual_payload_type != CONTENT_FABRIC_READER_PAYLOAD_TYPE_V1 {
            self.terminal = true;
            return Err(XeniaNixResponseReceiveErrorV1::UnexpectedPayloadType {
                expected: CONTENT_FABRIC_READER_PAYLOAD_TYPE_V1,
                actual: actual_payload_type,
            });
        }

        match self.state.apply_frame(opened.plaintext()) {
            Ok(ParsedResponseEventV1::Data {
                bytes,
                sequence,
                cumulative_bytes,
            }) => Ok(XeniaNixResponseEventV1::Data(
                ReceivedXeniaNixResponseChunkV1 {
                    bytes,
                    sequence,
                    cumulative_bytes,
                },
            )),
            Ok(ParsedResponseEventV1::End) => Ok(XeniaNixResponseEventV1::End),
            Err(error) => {
                self.terminal = true;
                Err(error)
            }
        }
    }

    /// Consume a completed receiver and recover its authenticated Xenia channel
    /// together with an audit-only completion record.
    ///
    /// Calling this before a valid End frame fails and drops the channel. There
    /// is no recovery API for a terminal or incomplete response.
    pub fn finish(
        mut self,
    ) -> Result<
        (
            AuthenticatedPeerApplicationChannelV1<T>,
            CompletedXeniaNixResponseV1,
        ),
        XeniaNixResponseReceiveErrorV1,
    > {
        if self.terminal {
            return Err(XeniaNixResponseReceiveErrorV1::Terminal);
        }
        if !self.state.completed {
            return Err(XeniaNixResponseReceiveErrorV1::IncompleteResponse);
        }
        let completion = self.state.completion();
        let channel = self
            .channel
            .take()
            .ok_or(XeniaNixResponseReceiveErrorV1::Terminal)?;
        Ok((channel, completion))
    }
}

/// Send one NarInfo MCFR request and return a receiver that owns the exact same
/// authenticated Xenia channel until the matching MCFS response completes.
pub async fn send_nix_narinfo_request_over_xenia_v1<T: Transport>(
    channel: AuthenticatedPeerApplicationChannelV1<T>,
    store_hash: NixStoreHashV1,
) -> Result<XeniaNixResponseReceiverV1<T>, XeniaNixResponseReceiveErrorV1> {
    send_request_v1(
        channel,
        NixReaderOperationV1::NarInfo,
        store_hash,
        MAX_NARINFO_RESPONSE_BYTES_V1,
        None,
    )
    .await
}

/// Send one raw-NAR MCFR request and return a receiver that owns the exact same
/// authenticated Xenia channel until the matching MCFS response completes.
///
/// `expected_nar_size` must come from previously validated NarInfo metadata. It
/// provides an exact response-size bound before any raw NAR bytes are accepted.
pub async fn send_nix_nar_request_over_xenia_v1<T: Transport>(
    channel: AuthenticatedPeerApplicationChannelV1<T>,
    store_hash: NixStoreHashV1,
    expected_nar_size: u64,
) -> Result<XeniaNixResponseReceiverV1<T>, XeniaNixResponseReceiveErrorV1> {
    if expected_nar_size == 0 {
        return Err(XeniaNixResponseReceiveErrorV1::InvalidExpectedNarSize);
    }
    send_request_v1(
        channel,
        NixReaderOperationV1::Nar,
        store_hash,
        expected_nar_size,
        Some(expected_nar_size),
    )
    .await
}

async fn send_request_v1<T: Transport>(
    mut channel: AuthenticatedPeerApplicationChannelV1<T>,
    operation: NixReaderOperationV1,
    store_hash: NixStoreHashV1,
    max_total_bytes: u64,
    expected_total_bytes: Option<u64>,
) -> Result<XeniaNixResponseReceiverV1<T>, XeniaNixResponseReceiveErrorV1> {
    let actual_payload_type = channel.payload_type().value();
    if actual_payload_type != CONTENT_FABRIC_READER_PAYLOAD_TYPE_V1 {
        return Err(XeniaNixResponseReceiveErrorV1::UnexpectedPayloadType {
            expected: CONTENT_FABRIC_READER_PAYLOAD_TYPE_V1,
            actual: actual_payload_type,
        });
    }

    let request = encode_nix_reader_request_v1(operation, &store_hash);
    channel.send_payload(&request).await?;

    Ok(XeniaNixResponseReceiverV1 {
        channel: Some(channel),
        state: ResponseStateV1::new(
            operation,
            store_hash,
            max_total_bytes,
            expected_total_bytes,
        ),
        terminal: false,
    })
}

struct ResponseStateV1 {
    operation: NixReaderOperationV1,
    store_hash: NixStoreHashV1,
    max_total_bytes: u64,
    expected_total_bytes: Option<u64>,
    next_sequence: u64,
    accumulated_bytes: u64,
    completed: bool,
}

impl ResponseStateV1 {
    fn new(
        operation: NixReaderOperationV1,
        store_hash: NixStoreHashV1,
        max_total_bytes: u64,
        expected_total_bytes: Option<u64>,
    ) -> Self {
        Self {
            operation,
            store_hash,
            max_total_bytes,
            expected_total_bytes,
            next_sequence: 0,
            accumulated_bytes: 0,
            completed: false,
        }
    }

    fn apply_frame(
        &mut self,
        frame: &[u8],
    ) -> Result<ParsedResponseEventV1, XeniaNixResponseReceiveErrorV1> {
        if self.completed {
            return Err(XeniaNixResponseReceiveErrorV1::AlreadyCompleted);
        }
        if frame.len() < CONTENT_FABRIC_RESPONSE_FRAME_HEADER_LEN_V1 {
            return Err(XeniaNixResponseReceiveErrorV1::FrameTooShort(frame.len()));
        }
        if frame[..4] != CONTENT_FABRIC_RESPONSE_FRAME_MAGIC_V1 {
            return Err(XeniaNixResponseReceiveErrorV1::InvalidMagic);
        }

        let schema = u16::from_be_bytes([frame[4], frame[5]]);
        if schema != CONTENT_FABRIC_RESPONSE_FRAME_SCHEMA_V1 {
            return Err(XeniaNixResponseReceiveErrorV1::UnsupportedSchemaVersion(
                schema,
            ));
        }
        let kind = frame[6];
        if kind != RESPONSE_FRAME_KIND_DATA_V1 && kind != RESPONSE_FRAME_KIND_END_V1 {
            return Err(XeniaNixResponseReceiveErrorV1::UnsupportedFrameKind(kind));
        }
        let operation = match frame[7] {
            1 => NixReaderOperationV1::NarInfo,
            2 => NixReaderOperationV1::Nar,
            other => return Err(XeniaNixResponseReceiveErrorV1::UnsupportedOperation(other)),
        };
        if operation != self.operation {
            return Err(XeniaNixResponseReceiveErrorV1::OperationMismatch);
        }
        if &frame[8..40] != self.store_hash.as_str().as_bytes() {
            return Err(XeniaNixResponseReceiveErrorV1::StoreHashMismatch);
        }

        let sequence = u64::from_be_bytes(frame[40..48].try_into().expect("fixed slice"));
        if sequence != self.next_sequence {
            return Err(XeniaNixResponseReceiveErrorV1::SequenceMismatch {
                expected: self.next_sequence,
                actual: sequence,
            });
        }
        let payload_len =
            u32::from_be_bytes(frame[48..52].try_into().expect("fixed slice")) as usize;
        if payload_len > CONTENT_FABRIC_RESPONSE_DATA_MAX_V1 {
            return Err(XeniaNixResponseReceiveErrorV1::FramePayloadTooLarge(
                payload_len,
            ));
        }
        let actual_payload_len = frame.len() - CONTENT_FABRIC_RESPONSE_FRAME_HEADER_LEN_V1;
        if payload_len != actual_payload_len {
            return Err(XeniaNixResponseReceiveErrorV1::PayloadLengthMismatch {
                declared: payload_len,
                actual: actual_payload_len,
            });
        }
        let total_bytes = u64::from_be_bytes(frame[52..60].try_into().expect("fixed slice"));
        let payload = &frame[CONTENT_FABRIC_RESPONSE_FRAME_HEADER_LEN_V1..];

        match kind {
            RESPONSE_FRAME_KIND_DATA_V1 => {
                if payload.is_empty() {
                    return Err(XeniaNixResponseReceiveErrorV1::EmptyDataFrame);
                }
                if total_bytes != 0 {
                    return Err(XeniaNixResponseReceiveErrorV1::DataFrameTotalNonZero);
                }
                let attempted = self
                    .accumulated_bytes
                    .checked_add(payload.len() as u64)
                    .ok_or(XeniaNixResponseReceiveErrorV1::ByteCountOverflow)?;
                if attempted > self.max_total_bytes {
                    return Err(XeniaNixResponseReceiveErrorV1::ResponseTooLarge {
                        limit: self.max_total_bytes,
                        attempted,
                    });
                }
                let emitted_sequence = self.next_sequence;
                self.next_sequence = self
                    .next_sequence
                    .checked_add(1)
                    .ok_or(XeniaNixResponseReceiveErrorV1::SequenceOverflow)?;
                self.accumulated_bytes = attempted;
                Ok(ParsedResponseEventV1::Data {
                    bytes: payload.to_vec(),
                    sequence: emitted_sequence,
                    cumulative_bytes: attempted,
                })
            }
            RESPONSE_FRAME_KIND_END_V1 => {
                if !payload.is_empty() {
                    return Err(XeniaNixResponseReceiveErrorV1::EndFrameHasPayload);
                }
                if total_bytes != self.accumulated_bytes {
                    return Err(XeniaNixResponseReceiveErrorV1::EndTotalMismatch {
                        accumulated: self.accumulated_bytes,
                        end_total: total_bytes,
                    });
                }
                if let Some(expected) = self.expected_total_bytes {
                    if self.accumulated_bytes != expected {
                        return Err(XeniaNixResponseReceiveErrorV1::ExpectedNarSizeMismatch {
                            expected,
                            actual: self.accumulated_bytes,
                        });
                    }
                }
                self.completed = true;
                Ok(ParsedResponseEventV1::End)
            }
            _ => unreachable!("frame kind validated above"),
        }
    }

    fn completion(&self) -> CompletedXeniaNixResponseV1 {
        debug_assert!(self.completed);
        CompletedXeniaNixResponseV1 {
            operation: self.operation,
            store_hash: self.store_hash.clone(),
            total_bytes: self.accumulated_bytes,
            data_frames: self.next_sequence,
        }
    }
}

enum ParsedResponseEventV1 {
    Data {
        bytes: Vec<u8>,
        sequence: u64,
        cumulative_bytes: u64,
    },
    End,
}

#[cfg(test)]
mod tests {
    use super::*;

    const STORE_HASH: &str = "00000000000000000000000000000000";
    const OTHER_HASH: &str = "11111111111111111111111111111111";

    fn store_hash(value: &str) -> NixStoreHashV1 {
        NixStoreHashV1::parse(value).unwrap()
    }

    fn frame(
        kind: u8,
        operation: NixReaderOperationV1,
        hash: &str,
        sequence: u64,
        total: u64,
        payload: &[u8],
    ) -> Vec<u8> {
        let operation_tag = match operation {
            NixReaderOperationV1::NarInfo => 1,
            NixReaderOperationV1::Nar => 2,
        };
        let mut out = Vec::with_capacity(CONTENT_FABRIC_RESPONSE_FRAME_HEADER_LEN_V1 + payload.len());
        out.extend_from_slice(b"MCFS");
        out.extend_from_slice(&CONTENT_FABRIC_RESPONSE_FRAME_SCHEMA_V1.to_be_bytes());
        out.push(kind);
        out.push(operation_tag);
        out.extend_from_slice(hash.as_bytes());
        out.extend_from_slice(&sequence.to_be_bytes());
        out.extend_from_slice(&(payload.len() as u32).to_be_bytes());
        out.extend_from_slice(&total.to_be_bytes());
        out.extend_from_slice(payload);
        out
    }

    #[test]
    fn serial_data_then_exact_end_completes() {
        let mut state = ResponseStateV1::new(
            NixReaderOperationV1::Nar,
            store_hash(STORE_HASH),
            5,
            Some(5),
        );
        assert!(matches!(
            state.apply_frame(&frame(
                RESPONSE_FRAME_KIND_DATA_V1,
                NixReaderOperationV1::Nar,
                STORE_HASH,
                0,
                0,
                b"abc",
            )),
            Ok(ParsedResponseEventV1::Data { sequence: 0, cumulative_bytes: 3, .. })
        ));
        assert!(matches!(
            state.apply_frame(&frame(
                RESPONSE_FRAME_KIND_DATA_V1,
                NixReaderOperationV1::Nar,
                STORE_HASH,
                1,
                0,
                b"de",
            )),
            Ok(ParsedResponseEventV1::Data { sequence: 1, cumulative_bytes: 5, .. })
        ));
        assert!(matches!(
            state.apply_frame(&frame(
                RESPONSE_FRAME_KIND_END_V1,
                NixReaderOperationV1::Nar,
                STORE_HASH,
                2,
                5,
                b"",
            )),
            Ok(ParsedResponseEventV1::End)
        ));
        let completion = state.completion();
        assert_eq!(completion.total_bytes(), 5);
        assert_eq!(completion.data_frames(), 2);
    }

    #[test]
    fn operation_and_store_hash_cannot_change() {
        let mut state = ResponseStateV1::new(
            NixReaderOperationV1::Nar,
            store_hash(STORE_HASH),
            10,
            Some(10),
        );
        assert!(matches!(
            state.apply_frame(&frame(
                RESPONSE_FRAME_KIND_DATA_V1,
                NixReaderOperationV1::NarInfo,
                STORE_HASH,
                0,
                0,
                b"x",
            )),
            Err(XeniaNixResponseReceiveErrorV1::OperationMismatch)
        ));

        let mut state = ResponseStateV1::new(
            NixReaderOperationV1::Nar,
            store_hash(STORE_HASH),
            10,
            Some(10),
        );
        assert!(matches!(
            state.apply_frame(&frame(
                RESPONSE_FRAME_KIND_DATA_V1,
                NixReaderOperationV1::Nar,
                OTHER_HASH,
                0,
                0,
                b"x",
            )),
            Err(XeniaNixResponseReceiveErrorV1::StoreHashMismatch)
        ));
    }

    #[test]
    fn duplicate_or_reordered_sequence_is_rejected() {
        let mut state = ResponseStateV1::new(
            NixReaderOperationV1::NarInfo,
            store_hash(STORE_HASH),
            MAX_NARINFO_RESPONSE_BYTES_V1,
            None,
        );
        state
            .apply_frame(&frame(
                RESPONSE_FRAME_KIND_DATA_V1,
                NixReaderOperationV1::NarInfo,
                STORE_HASH,
                0,
                0,
                b"a",
            ))
            .unwrap();
        assert!(matches!(
            state.apply_frame(&frame(
                RESPONSE_FRAME_KIND_DATA_V1,
                NixReaderOperationV1::NarInfo,
                STORE_HASH,
                0,
                0,
                b"b",
            )),
            Err(XeniaNixResponseReceiveErrorV1::SequenceMismatch { expected: 1, actual: 0 })
        ));
    }

    #[test]
    fn declared_length_and_frame_ceiling_are_enforced() {
        let mut state = ResponseStateV1::new(
            NixReaderOperationV1::NarInfo,
            store_hash(STORE_HASH),
            MAX_NARINFO_RESPONSE_BYTES_V1,
            None,
        );
        let mut bad = frame(
            RESPONSE_FRAME_KIND_DATA_V1,
            NixReaderOperationV1::NarInfo,
            STORE_HASH,
            0,
            0,
            b"abc",
        );
        bad[48..52].copy_from_slice(&4_u32.to_be_bytes());
        assert!(matches!(
            state.apply_frame(&bad),
            Err(XeniaNixResponseReceiveErrorV1::PayloadLengthMismatch { .. })
        ));

        let mut state = ResponseStateV1::new(
            NixReaderOperationV1::NarInfo,
            store_hash(STORE_HASH),
            u64::MAX,
            None,
        );
        let payload = vec![0_u8; CONTENT_FABRIC_RESPONSE_DATA_MAX_V1 + 1];
        assert!(matches!(
            state.apply_frame(&frame(
                RESPONSE_FRAME_KIND_DATA_V1,
                NixReaderOperationV1::NarInfo,
                STORE_HASH,
                0,
                0,
                &payload,
            )),
            Err(XeniaNixResponseReceiveErrorV1::FramePayloadTooLarge(_))
        ));
    }

    #[test]
    fn data_total_and_end_payload_are_closed_world() {
        let mut state = ResponseStateV1::new(
            NixReaderOperationV1::NarInfo,
            store_hash(STORE_HASH),
            MAX_NARINFO_RESPONSE_BYTES_V1,
            None,
        );
        assert!(matches!(
            state.apply_frame(&frame(
                RESPONSE_FRAME_KIND_DATA_V1,
                NixReaderOperationV1::NarInfo,
                STORE_HASH,
                0,
                1,
                b"x",
            )),
            Err(XeniaNixResponseReceiveErrorV1::DataFrameTotalNonZero)
        ));

        let mut state = ResponseStateV1::new(
            NixReaderOperationV1::NarInfo,
            store_hash(STORE_HASH),
            MAX_NARINFO_RESPONSE_BYTES_V1,
            None,
        );
        assert!(matches!(
            state.apply_frame(&frame(
                RESPONSE_FRAME_KIND_END_V1,
                NixReaderOperationV1::NarInfo,
                STORE_HASH,
                0,
                0,
                b"x",
            )),
            Err(XeniaNixResponseReceiveErrorV1::EndFrameHasPayload)
        ));
    }

    #[test]
    fn end_total_and_expected_nar_size_must_both_match() {
        let mut state = ResponseStateV1::new(
            NixReaderOperationV1::Nar,
            store_hash(STORE_HASH),
            4,
            Some(4),
        );
        state
            .apply_frame(&frame(
                RESPONSE_FRAME_KIND_DATA_V1,
                NixReaderOperationV1::Nar,
                STORE_HASH,
                0,
                0,
                b"abc",
            ))
            .unwrap();
        assert!(matches!(
            state.apply_frame(&frame(
                RESPONSE_FRAME_KIND_END_V1,
                NixReaderOperationV1::Nar,
                STORE_HASH,
                1,
                4,
                b"",
            )),
            Err(XeniaNixResponseReceiveErrorV1::EndTotalMismatch { accumulated: 3, end_total: 4 })
        ));

        let mut state = ResponseStateV1::new(
            NixReaderOperationV1::Nar,
            store_hash(STORE_HASH),
            4,
            Some(4),
        );
        state
            .apply_frame(&frame(
                RESPONSE_FRAME_KIND_DATA_V1,
                NixReaderOperationV1::Nar,
                STORE_HASH,
                0,
                0,
                b"abc",
            ))
            .unwrap();
        assert!(matches!(
            state.apply_frame(&frame(
                RESPONSE_FRAME_KIND_END_V1,
                NixReaderOperationV1::Nar,
                STORE_HASH,
                1,
                3,
                b"",
            )),
            Err(XeniaNixResponseReceiveErrorV1::ExpectedNarSizeMismatch { expected: 4, actual: 3 })
        ));
    }

    #[test]
    fn response_bound_is_enforced_before_more_bytes_are_released() {
        let mut state = ResponseStateV1::new(
            NixReaderOperationV1::Nar,
            store_hash(STORE_HASH),
            3,
            Some(3),
        );
        assert!(matches!(
            state.apply_frame(&frame(
                RESPONSE_FRAME_KIND_DATA_V1,
                NixReaderOperationV1::Nar,
                STORE_HASH,
                0,
                0,
                b"abcd",
            )),
            Err(XeniaNixResponseReceiveErrorV1::ResponseTooLarge { limit: 3, attempted: 4 })
        ));
    }

    #[test]
    fn frame_after_end_is_rejected() {
        let mut state = ResponseStateV1::new(
            NixReaderOperationV1::NarInfo,
            store_hash(STORE_HASH),
            MAX_NARINFO_RESPONSE_BYTES_V1,
            None,
        );
        state
            .apply_frame(&frame(
                RESPONSE_FRAME_KIND_END_V1,
                NixReaderOperationV1::NarInfo,
                STORE_HASH,
                0,
                0,
                b"",
            ))
            .unwrap();
        assert!(matches!(
            state.apply_frame(&frame(
                RESPONSE_FRAME_KIND_END_V1,
                NixReaderOperationV1::NarInfo,
                STORE_HASH,
                0,
                0,
                b"",
            )),
            Err(XeniaNixResponseReceiveErrorV1::AlreadyCompleted)
        ));
    }
}
