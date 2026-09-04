use mycelix_infrastructure_types::StableIdV1;
use mycelix_nix_cache::{NixCacheEntryV1, NixCacheErrorV1, NixStoreHashV1};
use mycelix_nix_exposure::{
    RemoteReaderV1, RemoteServingSnapshotErrorV1, RemoteServingSnapshotV1,
};
use thiserror::Error;

use crate::EnrollmentBoundOpenedPayloadV1;

/// Fixed byte length of one CF-07C3 reader request.
pub const CONTENT_FABRIC_READER_REQUEST_LEN_V1: usize = 40;
/// Current fixed binary request schema version.
pub const CONTENT_FABRIC_READER_REQUEST_SCHEMA_V1: u16 = 1;
const CONTENT_FABRIC_READER_REQUEST_MAGIC_V1: [u8; 4] = *b"MCFR";

/// Read operation admitted by the CF-07C3 Content Fabric request schema.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum NixReaderOperationV1 {
    /// Request the authorized `.narinfo` representation for one store hash.
    NarInfo,
    /// Request the authorized raw NAR bytes for one store hash.
    Nar,
}

impl NixReaderOperationV1 {
    const fn tag(self) -> u8 {
        match self {
            Self::NarInfo => 1,
            Self::Nar => 2,
        }
    }

    fn from_tag(tag: u8) -> Result<Self, NixReaderRequestErrorV1> {
        match tag {
            1 => Ok(Self::NarInfo),
            2 => Ok(Self::Nar),
            _ => Err(NixReaderRequestErrorV1::UnsupportedOperation(tag)),
        }
    }
}

/// Fail-closed request parsing and serving-authorization failures for CF-07C3.
#[derive(Debug, Error)]
pub enum NixReaderRequestErrorV1 {
    /// The opened request did not have the one exact fixed v1 length.
    #[error("invalid Content Fabric reader request length: expected {expected}, got {actual}")]
    InvalidLength {
        /// Required v1 length.
        expected: usize,
        /// Received length.
        actual: usize,
    },
    /// The fixed request-family magic was not `MCFR`.
    #[error("invalid Content Fabric reader request magic")]
    InvalidMagic,
    /// The binary schema version is not supported by this implementation.
    #[error("unsupported Content Fabric reader request schema version {0}")]
    UnsupportedSchemaVersion(u16),
    /// The operation byte is not assigned in request schema v1.
    #[error("unsupported Content Fabric reader request operation {0}")]
    UnsupportedOperation(u8),
    /// Reserved request bits must remain zero so future semantics cannot be
    /// smuggled into a v1 decoder.
    #[error("Content Fabric reader request reserved byte must be zero, got 0x{0:02x}")]
    ReservedByteNonZero(u8),
    /// The fixed 32-byte Nix store hash field was not UTF-8/ASCII text.
    #[error("Content Fabric reader request store hash is not valid UTF-8")]
    InvalidStoreHashEncoding,
    /// The exact Nix store-hash parser rejected the request field.
    #[error(transparent)]
    NixCache(#[from] NixCacheErrorV1),
    /// The serving-safe CF-07A snapshot was stale, not-yet-valid, or otherwise
    /// refused the lookup.
    #[error(transparent)]
    Serving(#[from] RemoteServingSnapshotErrorV1),
}

/// One resource read authorized against an exact serving-safe CF-07A snapshot.
///
/// This value deliberately has no `Debug`, `Clone`, `Copy`, serialization, or
/// public constructor surface. It binds the requested operation to the exact
/// cache entry returned by `RemoteServingSnapshotV1::entry_for_reader_at` and
/// carries the snapshot's exclusive serving deadline. Downstream serving code
/// never receives either a detachable `RemoteReaderV1` or a generic cache-entry
/// accessor that could erase the NarInfo-vs-Nar operation distinction.
#[derive(PartialEq, Eq)]
pub struct AuthorizedNixReadV1 {
    operation: NixReaderOperationV1,
    entry: NixCacheEntryV1,
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

impl AuthorizedNixReadV1 {
    /// Exact read operation authorized by the parsed request.
    pub const fn operation(&self) -> NixReaderOperationV1 {
        self.operation
    }

    /// Exact Nix store hash that survived request parsing and CF-07A lookup.
    pub fn store_hash(&self) -> &NixStoreHashV1 {
        self.entry.store_path().hash()
    }

    /// Render the exact authorized `.narinfo` body only when this request
    /// authorized NarInfo and the CF-07A serving horizon is still current.
    pub fn render_narinfo_at(&self, now_unix_ms: u64) -> Option<String> {
        if self.operation == NixReaderOperationV1::NarInfo && self.is_valid_at(now_unix_ms) {
            Some(self.entry.render_narinfo())
        } else {
            None
        }
    }

    /// Exact SHA-256 digest of raw NAR bytes, exposed only when this request
    /// authorized Nar and the CF-07A serving horizon is still current.
    pub fn nar_sha256_digest_at(&self, now_unix_ms: u64) -> Option<[u8; 32]> {
        if self.operation == NixReaderOperationV1::Nar && self.is_valid_at(now_unix_ms) {
            Some(self.entry.nar_digest().bytes)
        } else {
            None
        }
    }

    /// Exact raw NAR byte length, exposed only when this request authorized Nar
    /// and the CF-07A serving horizon is still current.
    pub fn nar_size_at(&self, now_unix_ms: u64) -> Option<u64> {
        if self.operation == NixReaderOperationV1::Nar && self.is_valid_at(now_unix_ms) {
            Some(self.entry.nar_size())
        } else {
            None
        }
    }

    /// CF-07A serving-snapshot commitment used for the lookup.
    pub const fn serving_snapshot_id(&self) -> StableIdV1 {
        self.serving_snapshot_id
    }

    /// Underlying remote-exposure projection commitment.
    pub const fn serving_projection_id(&self) -> StableIdV1 {
        self.serving_projection_id
    }

    /// Exact remote-exposure policy commitment enforced by the snapshot.
    pub const fn policy_id(&self) -> StableIdV1 {
        self.policy_id
    }

    /// Trusted server-side request time used for snapshot-validity admission.
    pub const fn authorized_at_unix_ms(&self) -> u64 {
        self.authorized_at_unix_ms
    }

    /// Exclusive serving-authority deadline inherited from CF-07A.
    ///
    /// A downstream byte-serving adapter must stop using this authorization
    /// when its trusted current time is greater than or equal to this value.
    pub const fn serve_until_unix_ms(&self) -> u64 {
        self.serve_until_unix_ms
    }

    /// Whether this request-scoped authority is still inside its CF-07A serving
    /// horizon at the supplied trusted server time.
    pub const fn is_valid_at(&self, now_unix_ms: u64) -> bool {
        self.authorized_at_unix_ms <= now_unix_ms && now_unix_ms < self.serve_until_unix_ms
    }

    /// Exact CF-07C1 registry commitment that mapped the authenticated peer.
    pub const fn registry_id(&self) -> StableIdV1 {
        self.registry_id
    }

    /// Exact enrollment selected for the authenticated hybrid peer key pair.
    pub const fn enrollment_id(&self) -> StableIdV1 {
        self.enrollment_id
    }

    /// Exact hybrid credential commitment authenticated by Xenia and CF-07C1.
    pub const fn credential_id(&self) -> StableIdV1 {
        self.credential_id
    }

    /// Xenia hybrid-handshake transcript generation for the request.
    pub const fn transcript_hash(&self) -> [u8; 32] {
        self.transcript_hash
    }

    /// Negotiated Xenia context commitment, when present.
    pub const fn negotiated_context_hash(&self) -> Option<[u8; 32]> {
        self.negotiated_context_hash
    }

    /// Same-carrier successful receive sequence from Xenia.
    pub const fn carrier_receive_sequence(&self) -> u64 {
        self.carrier_receive_sequence
    }
}

/// Canonically encode one CF-07C3 reader request for a Xenia Content Fabric
/// application channel.
///
/// Wire format, exactly 40 bytes:
///
/// ```text
/// 0..4   magic      = "MCFR"
/// 4..6   schema     = u16 big-endian, value 1
/// 6      operation  = 1 narinfo | 2 nar
/// 7      reserved   = 0
/// 8..40  store hash = exact 32-byte Nix base32 text
/// ```
///
/// Identity, groups, registry IDs, serving snapshot IDs, timestamps, and other
/// authority facts are intentionally absent from the request bytes.
pub fn encode_nix_reader_request_v1(
    operation: NixReaderOperationV1,
    store_hash: &NixStoreHashV1,
) -> [u8; CONTENT_FABRIC_READER_REQUEST_LEN_V1] {
    let mut out = [0u8; CONTENT_FABRIC_READER_REQUEST_LEN_V1];
    out[..4].copy_from_slice(&CONTENT_FABRIC_READER_REQUEST_MAGIC_V1);
    out[4..6].copy_from_slice(&CONTENT_FABRIC_READER_REQUEST_SCHEMA_V1.to_be_bytes());
    out[6] = operation.tag();
    out[7] = 0;
    out[8..].copy_from_slice(store_hash.as_str().as_bytes());
    out
}

/// Consume one CF-07C2 enrollment-bound opened payload, parse exactly one
/// bounded request schema, and immediately authorize that exact operation/store
/// hash against a serving-safe CF-07A snapshot.
///
/// The trusted `request_time_unix_ms` is supplied by server-side runtime state;
/// request plaintext cannot choose the time at which authority is evaluated.
///
/// Unauthorized and absent store hashes intentionally both return `Ok(None)`,
/// preserving CF-07A's non-enumerating lookup behavior. No public
/// `RemoteReaderV1` or parsed-but-unbound authority object is released.
pub fn authorize_bound_nix_read_v1(
    bound: EnrollmentBoundOpenedPayloadV1,
    serving: &RemoteServingSnapshotV1,
    request_time_unix_ms: u64,
) -> Result<Option<AuthorizedNixReadV1>, NixReaderRequestErrorV1> {
    let facts = BoundReaderFactsRefV1 {
        plaintext: bound.plaintext(),
        reader: bound.reader(),
        registry_id: bound.registry_id(),
        enrollment_id: bound.enrollment_id(),
        credential_id: bound.credential_id(),
        transcript_hash: bound.transcript_hash(),
        negotiated_context_hash: bound.negotiated_context_hash(),
        carrier_receive_sequence: bound.carrier_receive_sequence(),
    };
    authorize_reader_facts_v1(facts, serving, request_time_unix_ms)
}

struct ParsedNixReaderRequestV1 {
    operation: NixReaderOperationV1,
    store_hash: NixStoreHashV1,
}

struct BoundReaderFactsRefV1<'a> {
    plaintext: &'a [u8],
    reader: &'a RemoteReaderV1,
    registry_id: StableIdV1,
    enrollment_id: StableIdV1,
    credential_id: StableIdV1,
    transcript_hash: [u8; 32],
    negotiated_context_hash: Option<[u8; 32]>,
    carrier_receive_sequence: u64,
}

fn authorize_reader_facts_v1(
    facts: BoundReaderFactsRefV1<'_>,
    serving: &RemoteServingSnapshotV1,
    request_time_unix_ms: u64,
) -> Result<Option<AuthorizedNixReadV1>, NixReaderRequestErrorV1> {
    let request = parse_nix_reader_request_v1(facts.plaintext)?;
    let Some(entry) = serving
        .entry_for_reader_at(&request.store_hash, facts.reader, request_time_unix_ms)?
        .cloned()
    else {
        return Ok(None);
    };

    // Defense in depth: the serving lookup is keyed by the requested hash, but
    // pin the returned entry's own parsed store hash before releasing authority
    // in case CF-07A lookup internals are refactored later.
    if entry.store_path().hash() != &request.store_hash {
        return Ok(None);
    }

    Ok(Some(AuthorizedNixReadV1 {
        operation: request.operation,
        entry,
        serving_snapshot_id: serving.id(),
        serving_projection_id: serving.projection_id(),
        policy_id: serving.policy_id(),
        authorized_at_unix_ms: request_time_unix_ms,
        serve_until_unix_ms: serving.serve_until_unix_ms(),
        registry_id: facts.registry_id,
        enrollment_id: facts.enrollment_id,
        credential_id: facts.credential_id,
        transcript_hash: facts.transcript_hash,
        negotiated_context_hash: facts.negotiated_context_hash,
        carrier_receive_sequence: facts.carrier_receive_sequence,
    }))
}

fn parse_nix_reader_request_v1(
    bytes: &[u8],
) -> Result<ParsedNixReaderRequestV1, NixReaderRequestErrorV1> {
    if bytes.len() != CONTENT_FABRIC_READER_REQUEST_LEN_V1 {
        return Err(NixReaderRequestErrorV1::InvalidLength {
            expected: CONTENT_FABRIC_READER_REQUEST_LEN_V1,
            actual: bytes.len(),
        });
    }
    if bytes[..4] != CONTENT_FABRIC_READER_REQUEST_MAGIC_V1 {
        return Err(NixReaderRequestErrorV1::InvalidMagic);
    }
    let schema = u16::from_be_bytes([bytes[4], bytes[5]]);
    if schema != CONTENT_FABRIC_READER_REQUEST_SCHEMA_V1 {
        return Err(NixReaderRequestErrorV1::UnsupportedSchemaVersion(schema));
    }
    let operation = NixReaderOperationV1::from_tag(bytes[6])?;
    if bytes[7] != 0 {
        return Err(NixReaderRequestErrorV1::ReservedByteNonZero(bytes[7]));
    }
    let store_hash_text = std::str::from_utf8(&bytes[8..])
        .map_err(|_| NixReaderRequestErrorV1::InvalidStoreHashEncoding)?;
    let store_hash = NixStoreHashV1::parse(store_hash_text)?;
    Ok(ParsedNixReaderRequestV1 {
        operation,
        store_hash,
    })
}

#[cfg(test)]
mod tests {
    use mycelix_content_core::{ContentDigestV1, DigestAlgorithmV1};
    use mycelix_infrastructure_types::PartyIdV1;
    use mycelix_nix_cache::NixCacheCatalogV1;
    use mycelix_nix_exposure::{
        ExposureAssuranceV1, ExposureAudienceV1, ExposureEndpointIdV1,
        ExposureEvidenceCoverageV1, RemoteExposureGrantEvidenceV1, RemoteExposurePolicyV1,
        project_remote_serving_snapshot_v1,
    };

    use super::*;

    const STORE_HASH: &str = "00000000000000000000000000000000";
    const MISSING_HASH: &str = "11111111111111111111111111111111";

    fn party(byte: u8) -> PartyIdV1 {
        PartyIdV1([byte; 32])
    }

    fn stable(byte: u8) -> StableIdV1 {
        StableIdV1([byte; 32])
    }

    fn cache_entry() -> NixCacheEntryV1 {
        let digest = ContentDigestV1::compute(DigestAlgorithmV1::Sha256, b"test-nar");
        NixCacheEntryV1::new(
            &format!("/nix/store/{STORE_HASH}-cf-test"),
            digest,
            8,
            vec![],
            None,
            vec![],
            None,
        )
        .unwrap()
    }

    fn serving_snapshot() -> RemoteServingSnapshotV1 {
        let authority = party(0xA0);
        let reader = party(0x07);
        let endpoint = ExposureEndpointIdV1::derive(authority, "cf07c3-test").unwrap();
        let entry = cache_entry();
        let catalog = NixCacheCatalogV1::new(vec![entry.clone()]).unwrap();
        let grant = RemoteExposureGrantEvidenceV1::for_entry(
            authority,
            endpoint,
            &entry,
            ExposureAudienceV1::AuthenticatedPrincipal(reader),
            1_000,
            1_000,
            10_000,
            ExposureAssuranceV1::CryptographicallyVerified,
        )
        .unwrap();
        let policy = RemoteExposurePolicyV1::strict(endpoint, 20_000, 20_000).unwrap();
        project_remote_serving_snapshot_v1(
            &catalog,
            &[grant],
            &[],
            2_000,
            ExposureEvidenceCoverageV1::CompleteForAuthority,
            &policy,
            1_000,
        )
        .unwrap()
    }

    fn facts<'a>(plaintext: &'a [u8], reader: &'a RemoteReaderV1) -> BoundReaderFactsRefV1<'a> {
        BoundReaderFactsRefV1 {
            plaintext,
            reader,
            registry_id: stable(1),
            enrollment_id: stable(2),
            credential_id: stable(3),
            transcript_hash: [0xAA; 32],
            negotiated_context_hash: Some([0xBB; 32]),
            carrier_receive_sequence: 9,
        }
    }

    #[test]
    fn canonical_request_is_exactly_fixed_length_and_round_trips() {
        let hash = NixStoreHashV1::parse(STORE_HASH).unwrap();
        let encoded = encode_nix_reader_request_v1(NixReaderOperationV1::Nar, &hash);
        assert_eq!(encoded.len(), CONTENT_FABRIC_READER_REQUEST_LEN_V1);
        let parsed = parse_nix_reader_request_v1(&encoded).unwrap();
        assert_eq!(parsed.operation, NixReaderOperationV1::Nar);
        assert_eq!(parsed.store_hash, hash);
    }

    #[test]
    fn trailing_bytes_are_rejected_instead_of_ignored() {
        let hash = NixStoreHashV1::parse(STORE_HASH).unwrap();
        let mut encoded = encode_nix_reader_request_v1(NixReaderOperationV1::NarInfo, &hash).to_vec();
        encoded.push(0);
        assert!(matches!(
            parse_nix_reader_request_v1(&encoded),
            Err(NixReaderRequestErrorV1::InvalidLength { .. })
        ));
    }

    #[test]
    fn magic_schema_operation_and_reserved_byte_are_closed_world() {
        let hash = NixStoreHashV1::parse(STORE_HASH).unwrap();
        let base = encode_nix_reader_request_v1(NixReaderOperationV1::NarInfo, &hash);

        let mut wrong_magic = base;
        wrong_magic[0] ^= 1;
        assert!(matches!(
            parse_nix_reader_request_v1(&wrong_magic),
            Err(NixReaderRequestErrorV1::InvalidMagic)
        ));

        let mut wrong_schema = base;
        wrong_schema[5] = 2;
        assert!(matches!(
            parse_nix_reader_request_v1(&wrong_schema),
            Err(NixReaderRequestErrorV1::UnsupportedSchemaVersion(2))
        ));

        let mut wrong_operation = base;
        wrong_operation[6] = 3;
        assert!(matches!(
            parse_nix_reader_request_v1(&wrong_operation),
            Err(NixReaderRequestErrorV1::UnsupportedOperation(3))
        ));

        let mut reserved = base;
        reserved[7] = 1;
        assert!(matches!(
            parse_nix_reader_request_v1(&reserved),
            Err(NixReaderRequestErrorV1::ReservedByteNonZero(1))
        ));
    }

    #[test]
    fn invalid_nix_store_hash_is_rejected_by_existing_exact_parser() {
        let hash = NixStoreHashV1::parse(STORE_HASH).unwrap();
        let mut encoded = encode_nix_reader_request_v1(NixReaderOperationV1::Nar, &hash);
        encoded[8] = b'e';
        assert!(matches!(
            parse_nix_reader_request_v1(&encoded),
            Err(NixReaderRequestErrorV1::NixCache(_))
        ));
    }

    #[test]
    fn authorized_request_binds_operation_snapshot_deadline_and_identity_audit_chain() {
        let serving = serving_snapshot();
        let reader = RemoteReaderV1::authenticated(
            party(0x07),
            std::iter::empty::<StableIdV1>(),
        )
        .unwrap();
        let hash = NixStoreHashV1::parse(STORE_HASH).unwrap();
        let request = encode_nix_reader_request_v1(NixReaderOperationV1::Nar, &hash);
        let authorized = authorize_reader_facts_v1(facts(&request, &reader), &serving, 2_500)
            .unwrap()
            .unwrap();

        assert_eq!(authorized.operation(), NixReaderOperationV1::Nar);
        assert_eq!(authorized.store_hash(), &hash);
        assert_eq!(authorized.nar_size_at(2_500), Some(8));
        assert!(authorized.nar_sha256_digest_at(2_500).is_some());
        assert!(authorized.render_narinfo_at(2_500).is_none());
        assert_eq!(authorized.serving_snapshot_id(), serving.id());
        assert_eq!(authorized.serving_projection_id(), serving.projection_id());
        assert_eq!(authorized.policy_id(), serving.policy_id());
        assert_eq!(authorized.authorized_at_unix_ms(), 2_500);
        assert_eq!(authorized.serve_until_unix_ms(), 3_000);
        assert!(authorized.is_valid_at(2_999));
        assert!(!authorized.is_valid_at(3_000));
        assert_eq!(authorized.nar_size_at(3_000), None);
        assert_eq!(authorized.nar_sha256_digest_at(3_000), None);
        assert_eq!(authorized.registry_id(), stable(1));
        assert_eq!(authorized.enrollment_id(), stable(2));
        assert_eq!(authorized.credential_id(), stable(3));
        assert_eq!(authorized.transcript_hash(), [0xAA; 32]);
        assert_eq!(authorized.negotiated_context_hash(), Some([0xBB; 32]));
        assert_eq!(authorized.carrier_receive_sequence(), 9);
    }

    #[test]
    fn narinfo_authority_cannot_be_used_as_raw_nar_capability() {
        let serving = serving_snapshot();
        let reader = RemoteReaderV1::authenticated(
            party(0x07),
            std::iter::empty::<StableIdV1>(),
        )
        .unwrap();
        let hash = NixStoreHashV1::parse(STORE_HASH).unwrap();
        let request = encode_nix_reader_request_v1(NixReaderOperationV1::NarInfo, &hash);
        let authorized = authorize_reader_facts_v1(facts(&request, &reader), &serving, 2_500)
            .unwrap()
            .unwrap();

        assert!(authorized.render_narinfo_at(2_500).is_some());
        assert_eq!(authorized.nar_sha256_digest_at(2_500), None);
        assert_eq!(authorized.nar_size_at(2_500), None);
        assert_eq!(authorized.render_narinfo_at(3_000), None);
    }

    #[test]
    fn unauthorized_and_absent_entries_are_indistinguishable_none() {
        let serving = serving_snapshot();
        let wrong_reader = RemoteReaderV1::authenticated(
            party(0x08),
            std::iter::empty::<StableIdV1>(),
        )
        .unwrap();
        let allowed_hash = NixStoreHashV1::parse(STORE_HASH).unwrap();
        let allowed_request =
            encode_nix_reader_request_v1(NixReaderOperationV1::NarInfo, &allowed_hash);
        assert!(
            authorize_reader_facts_v1(facts(&allowed_request, &wrong_reader), &serving, 2_500)
                .unwrap()
                .is_none()
        );

        let reader = RemoteReaderV1::authenticated(
            party(0x07),
            std::iter::empty::<StableIdV1>(),
        )
        .unwrap();
        let missing_hash = NixStoreHashV1::parse(MISSING_HASH).unwrap();
        let missing_request =
            encode_nix_reader_request_v1(NixReaderOperationV1::NarInfo, &missing_hash);
        assert!(
            authorize_reader_facts_v1(facts(&missing_request, &reader), &serving, 2_500)
                .unwrap()
                .is_none()
        );
    }

    #[test]
    fn stale_serving_snapshot_fails_before_authority_is_released() {
        let serving = serving_snapshot();
        let reader = RemoteReaderV1::authenticated(
            party(0x07),
            std::iter::empty::<StableIdV1>(),
        )
        .unwrap();
        let hash = NixStoreHashV1::parse(STORE_HASH).unwrap();
        let request = encode_nix_reader_request_v1(NixReaderOperationV1::Nar, &hash);
        assert!(matches!(
            authorize_reader_facts_v1(facts(&request, &reader), &serving, 3_000),
            Err(NixReaderRequestErrorV1::Serving(
                RemoteServingSnapshotErrorV1::SnapshotNotValidAt { .. }
            ))
        ));
    }
}
