use mycelix_infrastructure_types::StableIdV1;
use mycelix_nix_exposure::RemoteReaderV1;
use mycelix_reader_enrollment::{
    ReaderEnrollmentRegistryV1, XeniaHybridReaderCredentialV1,
};
use xenia_peer_core::OpenedPeerApplicationPayloadV1;

use crate::XeniaReaderBridgeErrorV1;

/// Xenia application payload-type byte reserved by CF-07C2 for Content Fabric
/// reader traffic.
///
/// This is only a cryptographic/replay stream domain. It does not replace an
/// inner versioned Content Fabric request schema.
pub const CONTENT_FABRIC_READER_PAYLOAD_TYPE_V1: u8 = 0xCF;

/// Cryptographically opened bytes bound to an exact pinned CF-07C1 enrollment
/// and the resulting CF-07A reader identity facts.
///
/// This value intentionally does not implement `Debug`, `Clone`, `Copy`, or
/// serialization traits. Its plaintext may be sensitive, and the value is
/// request-scoped evidence rather than a durable credential.
///
/// The plaintext has **not** yet been decoded or validated as a Content Fabric
/// request. For that reason neither the plaintext nor `RemoteReaderV1` is
/// publicly extractable in CF-07C2. A later typed request parser in this crate
/// must consume this value before releasing any public resource-operation
/// authority.
#[derive(PartialEq, Eq)]
pub struct EnrollmentBoundOpenedPayloadV1 {
    plaintext: Vec<u8>,
    reader: RemoteReaderV1,
    registry_id: StableIdV1,
    enrollment_id: StableIdV1,
    credential_id: StableIdV1,
    transcript_hash: [u8; 32],
    negotiated_context_hash: Option<[u8; 32]>,
    carrier_receive_sequence: u64,
}

impl EnrollmentBoundOpenedPayloadV1 {
    pub(crate) fn plaintext(&self) -> &[u8] {
        &self.plaintext
    }

    pub(crate) const fn reader(&self) -> &RemoteReaderV1 {
        &self.reader
    }

    /// Stable ID of the exact CF-07C1 registry snapshot used for the mapping.
    pub const fn registry_id(&self) -> StableIdV1 {
        self.registry_id
    }

    /// Stable ID of the exact enrollment record selected by hybrid-key lookup.
    pub const fn enrollment_id(&self) -> StableIdV1 {
        self.enrollment_id
    }

    /// Stable ID of the exact authenticated Ed25519 + ML-DSA-65 credential.
    pub const fn credential_id(&self) -> StableIdV1 {
        self.credential_id
    }

    /// Xenia handshake transcript generation bound to the opened payload.
    pub const fn transcript_hash(&self) -> [u8; 32] {
        self.transcript_hash
    }

    /// Negotiated Xenia session-context commitment, when present.
    pub const fn negotiated_context_hash(&self) -> Option<[u8; 32]> {
        self.negotiated_context_hash
    }

    /// Process-local successful carrier-receive sequence from the Xenia
    /// authenticated transport wrapper.
    pub const fn carrier_receive_sequence(&self) -> u64 {
        self.carrier_receive_sequence
    }
}

/// Bind one sealed Xenia opened-payload proof to one explicitly pinned CF-07C1
/// enrollment registry snapshot.
///
/// The Xenia proof is consumed by value. The exact authenticated Ed25519 +
/// ML-DSA-65 pair must exist in `registry`, and `registry.id()` must equal
/// `expected_registry_id` supplied by higher-level policy/configuration.
///
/// This function constructs `RemoteReaderV1` only from that exact enrollment;
/// it never derives a Mycelix principal from a key hash and never accepts a
/// caller-supplied PartyId/group assertion. The resulting reader remains sealed
/// inside [`EnrollmentBoundOpenedPayloadV1`] until typed request validation is
/// added by the next tranche.
pub fn bind_xenia_opened_reader_v1(
    opened: OpenedPeerApplicationPayloadV1,
    registry: &ReaderEnrollmentRegistryV1,
    expected_registry_id: StableIdV1,
) -> Result<EnrollmentBoundOpenedPayloadV1, XeniaReaderBridgeErrorV1> {
    let facts = OpenedPeerFactsRefV1 {
        plaintext: opened.plaintext(),
        payload_type: opened.payload_type().value(),
        peer_ed25519_public_key: opened.peer_ed25519_public_key(),
        peer_ml_dsa_65_public_key: opened.peer_ml_dsa_65_public_key(),
        transcript_hash: opened.transcript_hash(),
        negotiated_context_hash: opened.negotiated_context_hash(),
        carrier_receive_sequence: opened.carrier_receive_sequence(),
    };
    bind_opened_facts_v1(facts, registry, expected_registry_id)
}

struct OpenedPeerFactsRefV1<'a> {
    plaintext: &'a [u8],
    payload_type: u8,
    peer_ed25519_public_key: [u8; 32],
    peer_ml_dsa_65_public_key: &'a [u8],
    transcript_hash: [u8; 32],
    negotiated_context_hash: Option<[u8; 32]>,
    carrier_receive_sequence: u64,
}

fn bind_opened_facts_v1(
    facts: OpenedPeerFactsRefV1<'_>,
    registry: &ReaderEnrollmentRegistryV1,
    expected_registry_id: StableIdV1,
) -> Result<EnrollmentBoundOpenedPayloadV1, XeniaReaderBridgeErrorV1> {
    if facts.payload_type != CONTENT_FABRIC_READER_PAYLOAD_TYPE_V1 {
        return Err(XeniaReaderBridgeErrorV1::WrongPayloadType {
            expected: CONTENT_FABRIC_READER_PAYLOAD_TYPE_V1,
            actual: facts.payload_type,
        });
    }

    let actual_registry_id = registry.id();
    if actual_registry_id != expected_registry_id {
        return Err(XeniaReaderBridgeErrorV1::RegistryCommitmentMismatch {
            expected: expected_registry_id,
            actual: actual_registry_id,
        });
    }

    let credential = XeniaHybridReaderCredentialV1::new(
        facts.peer_ed25519_public_key,
        facts.peer_ml_dsa_65_public_key,
    )?;
    let enrollment = registry
        .lookup_keys(
            &facts.peer_ed25519_public_key,
            facts.peer_ml_dsa_65_public_key,
        )?
        .ok_or(XeniaReaderBridgeErrorV1::PeerNotEnrolled)?;

    // CF-07C1 lookup rechecks the raw key bytes after stable-ID lookup. Keep the
    // selected credential commitment explicit in this evidence object as an
    // audit link between Xenia authentication and application enrollment.
    if enrollment.credential().id() != credential.id() {
        return Err(XeniaReaderBridgeErrorV1::PeerNotEnrolled);
    }

    let reader = RemoteReaderV1::authenticated(
        enrollment.principal(),
        enrollment.groups().iter().copied(),
    )?;

    Ok(EnrollmentBoundOpenedPayloadV1 {
        plaintext: facts.plaintext.to_vec(),
        reader,
        registry_id: actual_registry_id,
        enrollment_id: enrollment.id(),
        credential_id: credential.id(),
        transcript_hash: facts.transcript_hash,
        negotiated_context_hash: facts.negotiated_context_hash,
        carrier_receive_sequence: facts.carrier_receive_sequence,
    })
}

#[cfg(test)]
mod tests {
    use mycelix_infrastructure_types::{PartyIdV1, StableIdV1};
    use mycelix_reader_enrollment::{
        ReaderEnrollmentV1, XENIA_ML_DSA_65_PUBLIC_KEY_LEN_V1,
    };

    use super::*;

    fn ml(byte: u8) -> Vec<u8> {
        vec![byte; XENIA_ML_DSA_65_PUBLIC_KEY_LEN_V1]
    }

    fn party(byte: u8) -> PartyIdV1 {
        PartyIdV1([byte; 32])
    }

    fn group(byte: u8) -> StableIdV1 {
        StableIdV1([byte; 32])
    }

    fn registry(
        ed: u8,
        pq: u8,
        principal: u8,
        groups: impl IntoIterator<Item = StableIdV1>,
    ) -> ReaderEnrollmentRegistryV1 {
        let credential = XeniaHybridReaderCredentialV1::new([ed; 32], ml(pq)).unwrap();
        let enrollment = ReaderEnrollmentV1::new(credential, party(principal), groups).unwrap();
        ReaderEnrollmentRegistryV1::from_enrollments([enrollment]).unwrap()
    }

    fn facts<'a>(plaintext: &'a [u8], ml_key: &'a [u8]) -> OpenedPeerFactsRefV1<'a> {
        OpenedPeerFactsRefV1 {
            plaintext,
            payload_type: CONTENT_FABRIC_READER_PAYLOAD_TYPE_V1,
            peer_ed25519_public_key: [1; 32],
            peer_ml_dsa_65_public_key: ml_key,
            transcript_hash: [0xAA; 32],
            negotiated_context_hash: Some([0xBB; 32]),
            carrier_receive_sequence: 7,
        }
    }

    #[test]
    fn exact_hybrid_enrollment_projects_remote_reader_and_audit_links() {
        let ml_key = ml(2);
        let registry = registry(1, 2, 7, [group(9), group(8), group(9)]);
        let bound = bind_opened_facts_v1(facts(b"opaque-request", &ml_key), &registry, registry.id())
            .unwrap();

        assert_eq!(bound.plaintext(), b"opaque-request");
        assert_eq!(bound.reader().principal(), Some(party(7)));
        assert_eq!(bound.reader().groups(), &[group(8), group(9)]);
        assert_eq!(bound.registry_id(), registry.id());
        assert_eq!(bound.transcript_hash(), [0xAA; 32]);
        assert_eq!(bound.negotiated_context_hash(), Some([0xBB; 32]));
        assert_eq!(bound.carrier_receive_sequence(), 7);
    }

    #[test]
    fn wrong_application_domain_fails_before_enrollment() {
        let ml_key = ml(2);
        let registry = registry(1, 2, 7, []);
        let mut facts = facts(b"request", &ml_key);
        facts.payload_type = 0xCE;

        assert!(matches!(
            bind_opened_facts_v1(facts, &registry, registry.id()),
            Err(XeniaReaderBridgeErrorV1::WrongPayloadType {
                expected: CONTENT_FABRIC_READER_PAYLOAD_TYPE_V1,
                actual: 0xCE,
            })
        ));
    }

    #[test]
    fn registry_snapshot_substitution_is_explicitly_rejected() {
        let ml_key = ml(2);
        let registry = registry(1, 2, 7, []);
        let wrong_expected = StableIdV1([0xEE; 32]);

        assert!(matches!(
            bind_opened_facts_v1(facts(b"request", &ml_key), &registry, wrong_expected),
            Err(XeniaReaderBridgeErrorV1::RegistryCommitmentMismatch { .. })
        ));
    }

    #[test]
    fn same_ed25519_with_different_ml_dsa_does_not_authenticate_enrollment() {
        let presented_ml = ml(3);
        let registry = registry(1, 2, 7, []);

        assert!(matches!(
            bind_opened_facts_v1(facts(b"request", &presented_ml), &registry, registry.id()),
            Err(XeniaReaderBridgeErrorV1::PeerNotEnrolled)
        ));
    }

    #[test]
    fn same_ml_dsa_with_different_ed25519_does_not_authenticate_enrollment() {
        let ml_key = ml(2);
        let registry = registry(4, 2, 7, []);

        assert!(matches!(
            bind_opened_facts_v1(facts(b"request", &ml_key), &registry, registry.id()),
            Err(XeniaReaderBridgeErrorV1::PeerNotEnrolled)
        ));
    }

    #[test]
    fn group_membership_comes_only_from_pinned_enrollment() {
        let ml_key = ml(2);
        let first = registry(1, 2, 7, [group(8)]);
        let second = registry(1, 2, 7, [group(9)]);

        let bound = bind_opened_facts_v1(facts(b"request", &ml_key), &first, first.id()).unwrap();
        assert_eq!(bound.reader().groups(), &[group(8)]);

        assert!(matches!(
            bind_opened_facts_v1(facts(b"request", &ml_key), &second, first.id()),
            Err(XeniaReaderBridgeErrorV1::RegistryCommitmentMismatch { .. })
        ));
    }
}
