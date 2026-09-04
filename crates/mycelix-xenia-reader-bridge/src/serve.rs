use std::{
    fs::File,
    io::{self, Read, Write},
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};

use mycelix_content_core::{BlobDescriptorV1, ContentDigestV1, DigestAlgorithmV1};
use mycelix_content_node::{CasErrorV1, LocalCasV1};
use mycelix_infrastructure_types::StableIdV1;
use mycelix_nix_cache::NixStoreHashV1;
use thiserror::Error;

use crate::{AuthorizedNixReadV1, NixReaderOperationV1};

const MAX_NAR_READ_CHUNK_V1: usize = 64 * 1024;
const MAX_NARINFO_WRITE_CHUNK_V1: usize = 4 * 1024;

/// Preparation failures while converting CF-07C3 authority into a verified,
/// deadline-bound representation stream.
#[derive(Debug, Error)]
pub enum VerifiedReadServeErrorV1 {
    /// The CF-07A serving horizon expired before a representation could be
    /// prepared safely.
    #[error("authorized Nix read expired before verified serving preparation completed")]
    AuthorizationExpired,
    /// The local system clock could not be represented as Unix milliseconds.
    #[error("system clock is before the Unix epoch")]
    ClockBeforeUnixEpoch,
    /// The local system clock exceeded the u64 Unix-millisecond range.
    #[error("system clock exceeds the supported Unix-millisecond range")]
    ClockOverflow,
    /// The remaining serving horizon could not be represented by a monotonic
    /// deadline.
    #[error("serving deadline exceeds the monotonic clock range")]
    DeadlineOverflow,
    /// CF-03 CAS verification/open failed.
    #[error(transparent)]
    Cas(#[from] CasErrorV1),
}

/// Cloneable, non-authorizing audit facts attached to one prepared read.
///
/// This structure intentionally contains no reader principal/groups, CAS path,
/// NAR digest, raw NAR size, or representation bytes. Cloning it does not clone
/// serving authority.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PreparedNixReadAuditV1 {
    operation: NixReaderOperationV1,
    store_hash: NixStoreHashV1,
    serving_snapshot_id: StableIdV1,
    serving_projection_id: StableIdV1,
    policy_id: StableIdV1,
    authorized_at_unix_ms: u64,
    serve_until_unix_ms: u64,
    registry_id: StableIdV1,
    enrollment_id: StableIdV1,
    credential_id: StableIdV1,
    transcript_hash: [u8; 32],
    negotiated_context_hash: Option<[u8; 32]>,
    carrier_receive_sequence: u64,
}

impl PreparedNixReadAuditV1 {
    /// Exact representation operation authorized by CF-07C3.
    pub const fn operation(&self) -> NixReaderOperationV1 {
        self.operation
    }

    /// Exact Nix store hash requested and authorized.
    pub const fn store_hash(&self) -> &NixStoreHashV1 {
        &self.store_hash
    }

    /// Exact CF-07A serving snapshot commitment.
    pub const fn serving_snapshot_id(&self) -> StableIdV1 {
        self.serving_snapshot_id
    }

    /// Exact CF-07A exposure projection commitment.
    pub const fn serving_projection_id(&self) -> StableIdV1 {
        self.serving_projection_id
    }

    /// Exact remote-exposure policy commitment.
    pub const fn policy_id(&self) -> StableIdV1 {
        self.policy_id
    }

    /// Trusted server time at which CF-07C3 admitted the request.
    pub const fn authorized_at_unix_ms(&self) -> u64 {
        self.authorized_at_unix_ms
    }

    /// Exclusive CF-07A serving horizon inherited by this prepared read.
    pub const fn serve_until_unix_ms(&self) -> u64 {
        self.serve_until_unix_ms
    }

    /// Exact pinned reader-enrollment registry commitment.
    pub const fn registry_id(&self) -> StableIdV1 {
        self.registry_id
    }

    /// Exact enrollment selected for the authenticated peer.
    pub const fn enrollment_id(&self) -> StableIdV1 {
        self.enrollment_id
    }

    /// Exact hybrid credential commitment selected by CF-07C1.
    pub const fn credential_id(&self) -> StableIdV1 {
        self.credential_id
    }

    /// Xenia handshake transcript generation.
    pub const fn transcript_hash(&self) -> [u8; 32] {
        self.transcript_hash
    }

    /// Xenia negotiated context commitment, when present.
    pub const fn negotiated_context_hash(&self) -> Option<[u8; 32]> {
        self.negotiated_context_hash
    }

    /// Same-carrier receive sequence from the Xenia transport wrapper.
    pub const fn carrier_receive_sequence(&self) -> u64 {
        self.carrier_receive_sequence
    }
}

/// One prepared representation that preserves CF-07C3 operation authority.
///
/// Neither variant is `Clone`, serializable, or convertible into a raw file or
/// reusable body buffer.
pub enum PreparedNixReadV1 {
    /// One-shot `.narinfo` write authority.
    NarInfo(AuthorizedNarInfoV1),
    /// Sequential verified raw-NAR read authority.
    Nar(AuthorizedNarReaderV1),
}

impl PreparedNixReadV1 {
    /// Audit-only facts for this prepared representation.
    pub const fn audit(&self) -> &PreparedNixReadAuditV1 {
        match self {
            Self::NarInfo(value) => value.audit(),
            Self::Nar(value) => value.audit(),
        }
    }
}

/// One-shot, deadline-bound `.narinfo` writer.
///
/// The metadata bytes remain private. Serving consumes this object and writes
/// bounded chunks while checking both the private monotonic deadline and the
/// original Unix serving deadline before and after each write. It cannot be
/// converted into a reusable `String`/`Vec`.
pub struct AuthorizedNarInfoV1 {
    audit: PreparedNixReadAuditV1,
    body: Vec<u8>,
    deadline: Instant,
}

impl AuthorizedNarInfoV1 {
    /// Audit-only facts for this metadata representation.
    pub const fn audit(&self) -> &PreparedNixReadAuditV1 {
        &self.audit
    }

    /// Consume this one-shot authority and write the exact authorized NarInfo
    /// representation.
    ///
    /// Both clocks are checked before and after every bounded write. A writer
    /// that blocks inside one OS/application write cannot be preempted by this
    /// synchronous abstraction, but no later chunk is emitted after expiry.
    pub fn write_to<W: Write>(self, writer: &mut W) -> io::Result<u64> {
        let mut offset = 0_usize;
        while offset < self.body.len() {
            ensure_deadline(self.deadline, self.audit.serve_until_unix_ms)?;
            let end = self
                .body
                .len()
                .min(offset.saturating_add(MAX_NARINFO_WRITE_CHUNK_V1));
            let written = writer.write(&self.body[offset..end])?;
            if written == 0 {
                return Err(io::Error::new(
                    io::ErrorKind::WriteZero,
                    "NarInfo writer made no progress",
                ));
            }
            offset = offset.saturating_add(written);
            ensure_deadline(self.deadline, self.audit.serve_until_unix_ms)?;
        }
        Ok(offset as u64)
    }
}

/// Sequential reader over the exact CF-03 file handle that was fully verified
/// before release.
///
/// There is deliberately no `Seek`, `Clone`, or `into_file` surface. Reads are
/// capped at 64 KiB, check both monotonic and Unix serving deadlines before and
/// after each file read, and terminalize on expiry, clock failure, I/O error, or
/// unexpected early EOF. If expiry occurs while a file read is in progress, the
/// newly-read bytes are zeroed in the caller buffer before returning an error.
pub struct AuthorizedNarReaderV1 {
    audit: PreparedNixReadAuditV1,
    file: File,
    remaining_bytes: u64,
    deadline: Instant,
    terminal: bool,
}

impl AuthorizedNarReaderV1 {
    fn new(
        audit: PreparedNixReadAuditV1,
        file: File,
        size_bytes: u64,
        deadline: Instant,
    ) -> Self {
        Self {
            audit,
            file,
            remaining_bytes: size_bytes,
            deadline,
            terminal: false,
        }
    }

    /// Audit-only facts for this raw-NAR representation.
    pub const fn audit(&self) -> &PreparedNixReadAuditV1 {
        &self.audit
    }

    /// Number of authorized bytes not yet returned by this sequential reader.
    pub const fn remaining_bytes(&self) -> u64 {
        self.remaining_bytes
    }

    /// Whether an expiry/clock/I/O/truncation fault permanently terminalized
    /// the stream before its expected end.
    pub const fn is_terminal(&self) -> bool {
        self.terminal
    }
}

impl Read for AuthorizedNarReaderV1 {
    fn read(&mut self, buffer: &mut [u8]) -> io::Result<usize> {
        if buffer.is_empty() || self.remaining_bytes == 0 {
            return Ok(0);
        }
        if self.terminal {
            return Err(io::Error::new(
                io::ErrorKind::BrokenPipe,
                "authorized NAR reader is terminal",
            ));
        }
        if let Err(error) = ensure_deadline(self.deadline, self.audit.serve_until_unix_ms) {
            self.terminal = true;
            return Err(error);
        }

        let remaining_cap = usize::try_from(self.remaining_bytes).unwrap_or(usize::MAX);
        let read_len = buffer
            .len()
            .min(remaining_cap)
            .min(MAX_NAR_READ_CHUNK_V1);
        let read = match self.file.read(&mut buffer[..read_len]) {
            Ok(read) => read,
            Err(error) => {
                self.terminal = true;
                return Err(error);
            }
        };

        if read == 0 {
            self.terminal = true;
            return Err(io::Error::new(
                io::ErrorKind::UnexpectedEof,
                "verified NAR file ended before its authorized size",
            ));
        }

        if let Err(error) = ensure_deadline(self.deadline, self.audit.serve_until_unix_ms) {
            buffer[..read].fill(0);
            self.terminal = true;
            return Err(error);
        }

        self.remaining_bytes -= read as u64;
        Ok(read)
    }
}

/// Consume one CF-07C3 read authorization and prepare its exact representation.
///
/// No digest, size, path, store hash, operation, or deadline is supplied beside
/// the authorization object. For `Nar`, this function constructs the exact
/// SHA-256 `BlobDescriptorV1` from CF-07C3 authority and calls CF-03
/// `LocalCasV1::open_verified`, which validates size and re-hashes the same file
/// handle it returns. The authorization is rechecked after that potentially
/// expensive verification before a dual-clock stream deadline is minted.
pub fn prepare_authorized_nix_read_v1(
    authorized: AuthorizedNixReadV1,
    cas: &LocalCasV1,
) -> Result<PreparedNixReadV1, VerifiedReadServeErrorV1> {
    let operation = authorized.operation();
    let audit = audit_from_authorized(&authorized);

    match operation {
        NixReaderOperationV1::NarInfo => {
            let (now_unix_ms, deadline) =
                current_monotonic_deadline(authorized.serve_until_unix_ms())?;
            let body = authorized
                .render_narinfo_at(now_unix_ms)
                .ok_or(VerifiedReadServeErrorV1::AuthorizationExpired)?;
            Ok(PreparedNixReadV1::NarInfo(AuthorizedNarInfoV1 {
                audit,
                body: body.into_bytes(),
                deadline,
            }))
        }
        NixReaderOperationV1::Nar => {
            let now_unix_ms = unix_now_ms()?;
            let digest_bytes = authorized
                .nar_sha256_digest_at(now_unix_ms)
                .ok_or(VerifiedReadServeErrorV1::AuthorizationExpired)?;
            let size_bytes = authorized
                .nar_size_at(now_unix_ms)
                .ok_or(VerifiedReadServeErrorV1::AuthorizationExpired)?;
            let descriptor = BlobDescriptorV1 {
                digest: ContentDigestV1 {
                    algorithm: DigestAlgorithmV1::Sha256,
                    bytes: digest_bytes,
                },
                size_bytes,
                media_type: None,
            };

            let file = cas.open_verified(&descriptor)?;

            // CAS hashing may be expensive. Re-read trusted wall time only after
            // verification, then convert the *remaining* authority horizon to a
            // private monotonic deadline. Runtime reads subsequently check both
            // clocks: monotonic time prevents rollback extension, while Unix
            // time makes a forward wall-clock jump fail immediately.
            let (after_verify_unix_ms, deadline) =
                current_monotonic_deadline(authorized.serve_until_unix_ms())?;
            if !authorized.is_valid_at(after_verify_unix_ms) {
                return Err(VerifiedReadServeErrorV1::AuthorizationExpired);
            }

            Ok(PreparedNixReadV1::Nar(AuthorizedNarReaderV1::new(
                audit, file, size_bytes, deadline,
            )))
        }
    }
}

fn audit_from_authorized(authorized: &AuthorizedNixReadV1) -> PreparedNixReadAuditV1 {
    PreparedNixReadAuditV1 {
        operation: authorized.operation(),
        store_hash: authorized.store_hash().clone(),
        serving_snapshot_id: authorized.serving_snapshot_id(),
        serving_projection_id: authorized.serving_projection_id(),
        policy_id: authorized.policy_id(),
        authorized_at_unix_ms: authorized.authorized_at_unix_ms(),
        serve_until_unix_ms: authorized.serve_until_unix_ms(),
        registry_id: authorized.registry_id(),
        enrollment_id: authorized.enrollment_id(),
        credential_id: authorized.credential_id(),
        transcript_hash: authorized.transcript_hash(),
        negotiated_context_hash: authorized.negotiated_context_hash(),
        carrier_receive_sequence: authorized.carrier_receive_sequence(),
    }
}

fn unix_now_ms() -> Result<u64, VerifiedReadServeErrorV1> {
    let duration = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_err(|_| VerifiedReadServeErrorV1::ClockBeforeUnixEpoch)?;
    u64::try_from(duration.as_millis()).map_err(|_| VerifiedReadServeErrorV1::ClockOverflow)
}

fn current_monotonic_deadline(
    serve_until_unix_ms: u64,
) -> Result<(u64, Instant), VerifiedReadServeErrorV1> {
    let monotonic_now = Instant::now();
    let unix_now_ms = unix_now_ms()?;
    if unix_now_ms >= serve_until_unix_ms {
        return Err(VerifiedReadServeErrorV1::AuthorizationExpired);
    }
    let remaining = Duration::from_millis(serve_until_unix_ms - unix_now_ms);
    let deadline = monotonic_now
        .checked_add(remaining)
        .ok_or(VerifiedReadServeErrorV1::DeadlineOverflow)?;
    Ok((unix_now_ms, deadline))
}

fn ensure_deadline(deadline: Instant, serve_until_unix_ms: u64) -> io::Result<()> {
    if Instant::now() >= deadline {
        return Err(expired_io_error());
    }
    let unix_now_ms = unix_now_ms().map_err(|_| expired_io_error())?;
    if unix_now_ms >= serve_until_unix_ms {
        return Err(expired_io_error());
    }
    Ok(())
}

fn expired_io_error() -> io::Error {
    io::Error::new(
        io::ErrorKind::PermissionDenied,
        "Content Fabric serving authority expired or trusted clock is unavailable",
    )
}

#[cfg(test)]
mod tests {
    use std::io::{Read as _, Seek, SeekFrom};

    use tempfile::NamedTempFile;

    use super::*;

    const STORE_HASH: &str = "00000000000000000000000000000000";

    fn stable(byte: u8) -> StableIdV1 {
        StableIdV1([byte; 32])
    }

    fn audit(operation: NixReaderOperationV1) -> PreparedNixReadAuditV1 {
        PreparedNixReadAuditV1 {
            operation,
            store_hash: NixStoreHashV1::parse(STORE_HASH).unwrap(),
            serving_snapshot_id: stable(1),
            serving_projection_id: stable(2),
            policy_id: stable(3),
            authorized_at_unix_ms: 0,
            serve_until_unix_ms: u64::MAX,
            registry_id: stable(4),
            enrollment_id: stable(5),
            credential_id: stable(6),
            transcript_hash: [0xAA; 32],
            negotiated_context_hash: Some([0xBB; 32]),
            carrier_receive_sequence: 7,
        }
    }

    fn file_with(bytes: &[u8]) -> File {
        let mut temp = NamedTempFile::new().unwrap();
        temp.write_all(bytes).unwrap();
        temp.as_file_mut().seek(SeekFrom::Start(0)).unwrap();
        temp.reopen().unwrap()
    }

    #[test]
    fn narinfo_is_one_shot_and_operation_audit_only() {
        let body = b"StorePath: /nix/store/test\n".to_vec();
        let value = AuthorizedNarInfoV1 {
            audit: audit(NixReaderOperationV1::NarInfo),
            body: body.clone(),
            deadline: Instant::now() + Duration::from_secs(1),
        };
        let mut output = Vec::new();
        let written = value.write_to(&mut output).unwrap();
        assert_eq!(written, body.len() as u64);
        assert_eq!(output, body);
    }

    #[test]
    fn expired_narinfo_refuses_to_write() {
        let value = AuthorizedNarInfoV1 {
            audit: audit(NixReaderOperationV1::NarInfo),
            body: b"secret metadata".to_vec(),
            deadline: Instant::now(),
        };
        let mut output = Vec::new();
        let error = value.write_to(&mut output).unwrap_err();
        assert_eq!(error.kind(), io::ErrorKind::PermissionDenied);
        assert!(output.is_empty());
    }

    #[test]
    fn nar_reader_is_sequential_exact_length_and_not_terminal_on_success() {
        let bytes = vec![0x5A; 100_000];
        let file = file_with(&bytes);
        let mut reader = AuthorizedNarReaderV1::new(
            audit(NixReaderOperationV1::Nar),
            file,
            bytes.len() as u64,
            Instant::now() + Duration::from_secs(1),
        );
        let mut output = Vec::new();
        reader.read_to_end(&mut output).unwrap();
        assert_eq!(output, bytes);
        assert_eq!(reader.remaining_bytes(), 0);
        assert!(!reader.is_terminal());
    }

    #[test]
    fn nar_reader_refuses_after_deadline_before_disclosing_bytes() {
        let file = file_with(b"secret");
        let mut reader = AuthorizedNarReaderV1::new(
            audit(NixReaderOperationV1::Nar),
            file,
            6,
            Instant::now(),
        );
        let mut buffer = [0xA5; 6];
        let error = reader.read(&mut buffer).unwrap_err();
        assert_eq!(error.kind(), io::ErrorKind::PermissionDenied);
        assert_eq!(buffer, [0xA5; 6]);
        assert!(reader.is_terminal());
    }

    #[test]
    fn unexpected_early_eof_terminalizes_stream() {
        let file = file_with(b"ab");
        let mut reader = AuthorizedNarReaderV1::new(
            audit(NixReaderOperationV1::Nar),
            file,
            3,
            Instant::now() + Duration::from_secs(1),
        );
        let mut output = Vec::new();
        let error = reader.read_to_end(&mut output).unwrap_err();
        assert_eq!(error.kind(), io::ErrorKind::UnexpectedEof);
        assert_eq!(output, b"ab");
        assert!(reader.is_terminal());
    }

    #[test]
    fn one_read_never_exceeds_bounded_chunk_size() {
        let bytes = vec![0x11; MAX_NAR_READ_CHUNK_V1 + 10];
        let file = file_with(&bytes);
        let mut reader = AuthorizedNarReaderV1::new(
            audit(NixReaderOperationV1::Nar),
            file,
            bytes.len() as u64,
            Instant::now() + Duration::from_secs(1),
        );
        let mut buffer = vec![0u8; bytes.len()];
        let read = reader.read(&mut buffer).unwrap();
        assert_eq!(read, MAX_NAR_READ_CHUNK_V1);
        assert_eq!(reader.remaining_bytes(), 10);
    }
}
