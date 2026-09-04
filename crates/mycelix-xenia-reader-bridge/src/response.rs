use std::{
    io::{self, Read, Write},
    sync::Arc,
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};

use mycelix_content_node::LocalCasV1;
use mycelix_nix_cache::NixStoreHashV1;
use thiserror::Error;
use tokio::{sync::mpsc, task, time};
use xenia_peer_core::{
    AuthenticatedPeerApplicationChannelErrorV1, AuthenticatedPeerApplicationChannelV1,
    transport::Transport,
};

use crate::{
    AuthorizedNarInfoV1, AuthorizedNarReaderV1, AuthorizedNixReadV1,
    CONTENT_FABRIC_READER_PAYLOAD_TYPE_V1, NixReaderOperationV1, PreparedNixReadV1,
    VerifiedReadServeErrorV1, prepare_authorized_nix_read_v1,
};

/// Fixed Content Fabric Xenia response-frame header length.
pub const CONTENT_FABRIC_RESPONSE_FRAME_HEADER_LEN_V1: usize = 60;
/// Current Content Fabric Xenia response-frame schema version.
pub const CONTENT_FABRIC_RESPONSE_FRAME_SCHEMA_V1: u16 = 1;
/// Maximum representation bytes carried by one response data frame.
pub const CONTENT_FABRIC_RESPONSE_DATA_MAX_V1: usize = 64 * 1024;

const CONTENT_FABRIC_RESPONSE_FRAME_MAGIC_V1: [u8; 4] = *b"MCFS";
const RESPONSE_FRAME_KIND_DATA_V1: u8 = 1;
const RESPONSE_FRAME_KIND_END_V1: u8 = 2;
const NARINFO_PRODUCER_QUEUE_DEPTH_V1: usize = 1;

/// Fail-closed failures while carrying one authorized Nix representation over
/// the exact authenticated Xenia Content Fabric application channel.
#[derive(Debug, Error)]
pub enum XeniaNixResponseStreamErrorV1 {
    /// The supplied authenticated application channel is not the Content Fabric
    /// reader/response domain reserved by CF-07C2.
    #[error("unexpected Xenia application payload type: expected 0x{expected:02x}, got 0x{actual:02x}")]
    UnexpectedPayloadType {
        /// Required Content Fabric application-domain byte.
        expected: u8,
        /// Actual channel application-domain byte.
        actual: u8,
    },
    /// The serving horizon expired before a new response frame could safely be
    /// started or before an in-flight frame send completed.
    #[error("Content Fabric Xenia response serving authority expired")]
    AuthorizationExpired,
    /// The trusted system clock was before the Unix epoch.
    #[error("system clock is before the Unix epoch")]
    ClockBeforeUnixEpoch,
    /// The trusted system clock exceeded the supported u64 millisecond range.
    #[error("system clock exceeds the supported Unix-millisecond range")]
    ClockOverflow,
    /// The remaining serving horizon could not be represented by a monotonic
    /// deadline.
    #[error("serving horizon exceeds the monotonic clock range")]
    DeadlineOverflow,
    /// Response sequence arithmetic overflowed.
    #[error("Content Fabric response frame sequence overflow")]
    SequenceOverflow,
    /// Response byte accounting overflowed.
    #[error("Content Fabric response byte accounting overflow")]
    ByteCountOverflow,
    /// A response data frame exceeded the frozen v1 payload ceiling.
    #[error("Content Fabric response data frame too large: {0} bytes")]
    FramePayloadTooLarge(usize),
    /// The one-shot NarInfo producer and transmitted byte count disagreed.
    #[error("NarInfo producer/transmit length mismatch: producer={producer}, transmitted={transmitted}")]
    NarInfoLengthMismatch {
        /// Bytes reported by CF-07C4's one-shot writer.
        producer: u64,
        /// Bytes actually accepted into response data frames.
        transmitted: u64,
    },
    /// The raw-NAR stream emitted a byte count different from the verified
    /// authorized size.
    #[error("NAR response length mismatch: expected={expected}, transmitted={transmitted}")]
    NarLengthMismatch {
        /// Exact verified NAR size inherited from CF-07C4.
        expected: u64,
        /// Bytes actually accepted into response data frames.
        transmitted: u64,
    },
    /// A blocking preparation/read task panicked or was cancelled.
    #[error("Content Fabric response blocking task failed: {0}")]
    TaskJoin(String),
    /// CF-07C4 representation preparation failed.
    #[error(transparent)]
    Prepare(#[from] VerifiedReadServeErrorV1),
    /// CF-07C4 one-shot/stream read failed.
    #[error(transparent)]
    Io(#[from] io::Error),
    /// Xenia seal/carrier send failed.
    #[error(transparent)]
    Xenia(#[from] AuthenticatedPeerApplicationChannelErrorV1),
}

/// Consume one CF-07C3 authorization and one authenticated Xenia application
/// channel, prepare the exact CF-07C4 representation, and send one complete
/// serial response stream.
///
/// The channel is returned **only** after every data frame and the terminal
/// `End` frame have been sent successfully. Any preparation, deadline, read,
/// framing, seal, or carrier failure consumes/drops the channel so a partial
/// response cannot be followed by ordinary channel reuse.
///
/// The authority-bearing monotonic deadline is derived before potentially
/// expensive CF-03 verification. That deadline is therefore conservative with
/// respect to CF-07C4's later preparation deadline and remains immune to a wall
/// clock rollback during verification.
pub async fn send_authorized_nix_read_over_xenia_v1<T: Transport>(
    mut channel: AuthenticatedPeerApplicationChannelV1<T>,
    authorized: AuthorizedNixReadV1,
    cas: Arc<LocalCasV1>,
) -> Result<AuthenticatedPeerApplicationChannelV1<T>, XeniaNixResponseStreamErrorV1> {
    let actual_payload_type = channel.payload_type().value();
    if actual_payload_type != CONTENT_FABRIC_READER_PAYLOAD_TYPE_V1 {
        return Err(XeniaNixResponseStreamErrorV1::UnexpectedPayloadType {
            expected: CONTENT_FABRIC_READER_PAYLOAD_TYPE_V1,
            actual: actual_payload_type,
        });
    }

    let serve_until_unix_ms = authorized.serve_until_unix_ms();
    let delivery_deadline = monotonic_deadline_from_unix(serve_until_unix_ms)?;
    ensure_delivery_deadline(delivery_deadline, serve_until_unix_ms)?;

    let prepared = task::spawn_blocking(move || prepare_authorized_nix_read_v1(authorized, &cas))
        .await
        .map_err(|error| XeniaNixResponseStreamErrorV1::TaskJoin(error.to_string()))??;

    ensure_delivery_deadline(delivery_deadline, serve_until_unix_ms)?;
    send_prepared_response_v1(
        &mut channel,
        prepared,
        delivery_deadline,
        serve_until_unix_ms,
    )
    .await?;
    Ok(channel)
}

async fn send_prepared_response_v1<S: ResponsePayloadSink>(
    sink: &mut S,
    prepared: PreparedNixReadV1,
    delivery_deadline: Instant,
    serve_until_unix_ms: u64,
) -> Result<(), XeniaNixResponseStreamErrorV1> {
    let operation = prepared.audit().operation();
    let store_hash = prepared.audit().store_hash().clone();
    match prepared {
        PreparedNixReadV1::NarInfo(value) => {
            send_narinfo_response_v1(
                sink,
                value,
                operation,
                &store_hash,
                delivery_deadline,
                serve_until_unix_ms,
            )
            .await
        }
        PreparedNixReadV1::Nar(value) => {
            send_nar_response_v1(
                sink,
                value,
                operation,
                &store_hash,
                delivery_deadline,
                serve_until_unix_ms,
            )
            .await
        }
    }
}

async fn send_narinfo_response_v1<S: ResponsePayloadSink>(
    sink: &mut S,
    value: AuthorizedNarInfoV1,
    operation: NixReaderOperationV1,
    store_hash: &NixStoreHashV1,
    delivery_deadline: Instant,
    serve_until_unix_ms: u64,
) -> Result<(), XeniaNixResponseStreamErrorV1> {
    let (tx, mut rx) = mpsc::channel::<Vec<u8>>(NARINFO_PRODUCER_QUEUE_DEPTH_V1);
    let producer = task::spawn_blocking(move || {
        let mut writer = NarInfoChunkWriterV1 { tx };
        value.write_to(&mut writer)
    });

    let mut sequence = 0_u64;
    let mut transmitted = 0_u64;
    while let Some(chunk) = rx.recv().await {
        if chunk.is_empty() {
            continue;
        }
        let frame = encode_response_frame_v1(
            RESPONSE_FRAME_KIND_DATA_V1,
            operation,
            store_hash,
            sequence,
            0,
            &chunk,
        )?;
        send_frame_before_deadline_v1(
            sink,
            &frame,
            delivery_deadline,
            serve_until_unix_ms,
        )
        .await?;
        transmitted = transmitted
            .checked_add(chunk.len() as u64)
            .ok_or(XeniaNixResponseStreamErrorV1::ByteCountOverflow)?;
        sequence = sequence
            .checked_add(1)
            .ok_or(XeniaNixResponseStreamErrorV1::SequenceOverflow)?;
    }

    let producer_bytes = producer
        .await
        .map_err(|error| XeniaNixResponseStreamErrorV1::TaskJoin(error.to_string()))??;
    if producer_bytes != transmitted {
        return Err(XeniaNixResponseStreamErrorV1::NarInfoLengthMismatch {
            producer: producer_bytes,
            transmitted,
        });
    }

    let end = encode_response_frame_v1(
        RESPONSE_FRAME_KIND_END_V1,
        operation,
        store_hash,
        sequence,
        transmitted,
        &[],
    )?;
    send_frame_before_deadline_v1(
        sink,
        &end,
        delivery_deadline,
        serve_until_unix_ms,
    )
    .await
}

async fn send_nar_response_v1<S: ResponsePayloadSink>(
    sink: &mut S,
    mut value: AuthorizedNarReaderV1,
    operation: NixReaderOperationV1,
    store_hash: &NixStoreHashV1,
    delivery_deadline: Instant,
    serve_until_unix_ms: u64,
) -> Result<(), XeniaNixResponseStreamErrorV1> {
    let expected_total = value.remaining_bytes();
    let mut sequence = 0_u64;
    let mut transmitted = 0_u64;

    while value.remaining_bytes() != 0 {
        ensure_delivery_deadline(delivery_deadline, serve_until_unix_ms)?;
        let (next_value, chunk) = read_nar_chunk_v1(value).await?;
        value = next_value;
        ensure_delivery_deadline(delivery_deadline, serve_until_unix_ms)?;

        let frame = encode_response_frame_v1(
            RESPONSE_FRAME_KIND_DATA_V1,
            operation,
            store_hash,
            sequence,
            0,
            &chunk,
        )?;
        send_frame_before_deadline_v1(
            sink,
            &frame,
            delivery_deadline,
            serve_until_unix_ms,
        )
        .await?;
        transmitted = transmitted
            .checked_add(chunk.len() as u64)
            .ok_or(XeniaNixResponseStreamErrorV1::ByteCountOverflow)?;
        sequence = sequence
            .checked_add(1)
            .ok_or(XeniaNixResponseStreamErrorV1::SequenceOverflow)?;
    }

    if transmitted != expected_total {
        return Err(XeniaNixResponseStreamErrorV1::NarLengthMismatch {
            expected: expected_total,
            transmitted,
        });
    }

    let end = encode_response_frame_v1(
        RESPONSE_FRAME_KIND_END_V1,
        operation,
        store_hash,
        sequence,
        transmitted,
        &[],
    )?;
    send_frame_before_deadline_v1(
        sink,
        &end,
        delivery_deadline,
        serve_until_unix_ms,
    )
    .await
}

async fn read_nar_chunk_v1(
    value: AuthorizedNarReaderV1,
) -> Result<(AuthorizedNarReaderV1, Vec<u8>), XeniaNixResponseStreamErrorV1> {
    task::spawn_blocking(move || -> io::Result<(AuthorizedNarReaderV1, Vec<u8>)> {
        let mut value = value;
        let mut chunk = vec![0_u8; CONTENT_FABRIC_RESPONSE_DATA_MAX_V1];
        let read = value.read(&mut chunk)?;
        chunk.truncate(read);
        Ok((value, chunk))
    })
    .await
    .map_err(|error| XeniaNixResponseStreamErrorV1::TaskJoin(error.to_string()))?
    .map_err(Into::into)
}

fn encode_response_frame_v1(
    kind: u8,
    operation: NixReaderOperationV1,
    store_hash: &NixStoreHashV1,
    sequence: u64,
    total_bytes: u64,
    payload: &[u8],
) -> Result<Vec<u8>, XeniaNixResponseStreamErrorV1> {
    if payload.len() > CONTENT_FABRIC_RESPONSE_DATA_MAX_V1 {
        return Err(XeniaNixResponseStreamErrorV1::FramePayloadTooLarge(
            payload.len(),
        ));
    }
    let payload_len = u32::try_from(payload.len())
        .map_err(|_| XeniaNixResponseStreamErrorV1::FramePayloadTooLarge(payload.len()))?;
    let operation_tag = match operation {
        NixReaderOperationV1::NarInfo => 1_u8,
        NixReaderOperationV1::Nar => 2_u8,
    };

    let mut out = Vec::with_capacity(CONTENT_FABRIC_RESPONSE_FRAME_HEADER_LEN_V1 + payload.len());
    out.extend_from_slice(&CONTENT_FABRIC_RESPONSE_FRAME_MAGIC_V1);
    out.extend_from_slice(&CONTENT_FABRIC_RESPONSE_FRAME_SCHEMA_V1.to_be_bytes());
    out.push(kind);
    out.push(operation_tag);
    out.extend_from_slice(store_hash.as_str().as_bytes());
    out.extend_from_slice(&sequence.to_be_bytes());
    out.extend_from_slice(&payload_len.to_be_bytes());
    out.extend_from_slice(&total_bytes.to_be_bytes());
    debug_assert_eq!(out.len(), CONTENT_FABRIC_RESPONSE_FRAME_HEADER_LEN_V1);
    out.extend_from_slice(payload);
    Ok(out)
}

async fn send_frame_before_deadline_v1<S: ResponsePayloadSink>(
    sink: &mut S,
    frame: &[u8],
    delivery_deadline: Instant,
    serve_until_unix_ms: u64,
) -> Result<(), XeniaNixResponseStreamErrorV1> {
    ensure_delivery_deadline(delivery_deadline, serve_until_unix_ms)?;
    let deadline = time::Instant::from_std(delivery_deadline);
    match time::timeout_at(deadline, sink.send_response_payload(frame)).await {
        Ok(result) => result?,
        Err(_) => return Err(XeniaNixResponseStreamErrorV1::AuthorizationExpired),
    }
    ensure_delivery_deadline(delivery_deadline, serve_until_unix_ms)
}

fn monotonic_deadline_from_unix(
    serve_until_unix_ms: u64,
) -> Result<Instant, XeniaNixResponseStreamErrorV1> {
    let monotonic_now = Instant::now();
    let unix_now_ms = unix_now_ms()?;
    if unix_now_ms >= serve_until_unix_ms {
        return Err(XeniaNixResponseStreamErrorV1::AuthorizationExpired);
    }
    monotonic_now
        .checked_add(Duration::from_millis(serve_until_unix_ms - unix_now_ms))
        .ok_or(XeniaNixResponseStreamErrorV1::DeadlineOverflow)
}

fn ensure_delivery_deadline(
    deadline: Instant,
    serve_until_unix_ms: u64,
) -> Result<(), XeniaNixResponseStreamErrorV1> {
    if Instant::now() >= deadline || unix_now_ms()? >= serve_until_unix_ms {
        return Err(XeniaNixResponseStreamErrorV1::AuthorizationExpired);
    }
    Ok(())
}

fn unix_now_ms() -> Result<u64, XeniaNixResponseStreamErrorV1> {
    let duration = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_err(|_| XeniaNixResponseStreamErrorV1::ClockBeforeUnixEpoch)?;
    u64::try_from(duration.as_millis()).map_err(|_| XeniaNixResponseStreamErrorV1::ClockOverflow)
}

struct NarInfoChunkWriterV1 {
    tx: mpsc::Sender<Vec<u8>>,
}

impl Write for NarInfoChunkWriterV1 {
    fn write(&mut self, buffer: &[u8]) -> io::Result<usize> {
        if buffer.is_empty() {
            return Ok(0);
        }
        self.tx.blocking_send(buffer.to_vec()).map_err(|_| {
            io::Error::new(
                io::ErrorKind::BrokenPipe,
                "Content Fabric Xenia NarInfo response consumer closed",
            )
        })?;
        Ok(buffer.len())
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

#[allow(async_fn_in_trait)]
trait ResponsePayloadSink {
    async fn send_response_payload(
        &mut self,
        payload: &[u8],
    ) -> Result<(), XeniaNixResponseStreamErrorV1>;
}

impl<T: Transport> ResponsePayloadSink for AuthenticatedPeerApplicationChannelV1<T> {
    async fn send_response_payload(
        &mut self,
        payload: &[u8],
    ) -> Result<(), XeniaNixResponseStreamErrorV1> {
        self.send_payload(payload).await.map_err(Into::into)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const STORE_HASH: &str = "00000000000000000000000000000000";

    #[derive(Default)]
    struct RecordingSink {
        payloads: Vec<Vec<u8>>,
    }

    impl ResponsePayloadSink for RecordingSink {
        async fn send_response_payload(
            &mut self,
            payload: &[u8],
        ) -> Result<(), XeniaNixResponseStreamErrorV1> {
            self.payloads.push(payload.to_vec());
            Ok(())
        }
    }

    #[test]
    fn response_data_frame_encoding_is_canonical() {
        let store_hash = NixStoreHashV1::parse(STORE_HASH).unwrap();
        let frame = encode_response_frame_v1(
            RESPONSE_FRAME_KIND_DATA_V1,
            NixReaderOperationV1::Nar,
            &store_hash,
            7,
            0,
            b"abc",
        )
        .unwrap();

        assert_eq!(&frame[0..4], b"MCFS");
        assert_eq!(&frame[4..6], &1_u16.to_be_bytes());
        assert_eq!(frame[6], RESPONSE_FRAME_KIND_DATA_V1);
        assert_eq!(frame[7], 2);
        assert_eq!(&frame[8..40], STORE_HASH.as_bytes());
        assert_eq!(&frame[40..48], &7_u64.to_be_bytes());
        assert_eq!(&frame[48..52], &3_u32.to_be_bytes());
        assert_eq!(&frame[52..60], &0_u64.to_be_bytes());
        assert_eq!(&frame[60..], b"abc");
    }

    #[test]
    fn response_end_frame_commits_total_transmitted_bytes() {
        let store_hash = NixStoreHashV1::parse(STORE_HASH).unwrap();
        let frame = encode_response_frame_v1(
            RESPONSE_FRAME_KIND_END_V1,
            NixReaderOperationV1::NarInfo,
            &store_hash,
            3,
            12_345,
            &[],
        )
        .unwrap();
        assert_eq!(frame.len(), CONTENT_FABRIC_RESPONSE_FRAME_HEADER_LEN_V1);
        assert_eq!(frame[6], RESPONSE_FRAME_KIND_END_V1);
        assert_eq!(frame[7], 1);
        assert_eq!(&frame[40..48], &3_u64.to_be_bytes());
        assert_eq!(&frame[48..52], &0_u32.to_be_bytes());
        assert_eq!(&frame[52..60], &12_345_u64.to_be_bytes());
    }

    #[test]
    fn oversized_response_frame_is_rejected() {
        let store_hash = NixStoreHashV1::parse(STORE_HASH).unwrap();
        let payload = vec![0_u8; CONTENT_FABRIC_RESPONSE_DATA_MAX_V1 + 1];
        assert!(matches!(
            encode_response_frame_v1(
                RESPONSE_FRAME_KIND_DATA_V1,
                NixReaderOperationV1::Nar,
                &store_hash,
                0,
                0,
                &payload,
            ),
            Err(XeniaNixResponseStreamErrorV1::FramePayloadTooLarge(_))
        ));
    }

    #[tokio::test]
    async fn expired_deadline_refuses_frame_without_touching_sink() {
        let mut sink = RecordingSink::default();
        let deadline = Instant::now();
        let result = send_frame_before_deadline_v1(&mut sink, b"secret", deadline, u64::MAX).await;
        assert!(matches!(
            result,
            Err(XeniaNixResponseStreamErrorV1::AuthorizationExpired)
        ));
        assert!(sink.payloads.is_empty());
    }
}
