// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Sovereign Intelligence Fabric (SIF) v0.1 shared protocol types.
//!
//! The protocol is intentionally small and dependency-light. Mycelix owns policy,
//! notification, and accountability semantics; Xenia and Symthaea bind into these
//! types through opaque commitments rather than becoming dependencies of this crate.
//!
//! # Core invariant
//!
//! A person-linked lookup attempt must produce a [`SubjectAccessReceipt`]. The
//! receipt is subject-visible immediately unless a separately authorized, valid,
//! time-bounded [`DelayedNotificationGrant`] temporarily suppresses delivery.
//! Missing, malformed, expired, or mismatched grants fail open *to notification*.

use serde::{Deserialize, Serialize};

/// Wire/schema version for the first SIF protocol tranche.
pub const SIF_PROTOCOL_VERSION: u16 = 1;

/// Milliseconds since Unix epoch.
///
/// We intentionally keep the core protocol independent from any wall-clock crate.
/// Callers are responsible for sourcing trustworthy time where that matters.
pub type UnixMillis = i64;

/// A domain-separated BLAKE3 commitment used to bind data across Mycelix, Xenia,
/// Symthaea, witnesses, and proof systems without requiring those systems to share
/// internal representations.
#[derive(
    Clone, Copy, Debug, Default, Deserialize, Eq, Hash, Ord, PartialEq, PartialOrd, Serialize,
)]
pub struct Commitment(pub [u8; 32]);

impl Commitment {
    /// All-zero commitment, useful only as an explicit sentinel in tests/migrations.
    pub const ZERO: Self = Self([0; 32]);

    /// Commit to already-canonical bytes under a SIF domain tag.
    pub fn from_canonical_bytes(domain: &str, bytes: &[u8]) -> Self {
        let mut hasher = blake3::Hasher::new();
        hasher.update(b"mycelix-sif-v0.1\0");
        hasher.update(&(domain.len() as u64).to_le_bytes());
        hasher.update(domain.as_bytes());
        hasher.update(&(bytes.len() as u64).to_le_bytes());
        hasher.update(bytes);
        Self(*hasher.finalize().as_bytes())
    }
}

/// Result of the policy decision for a lookup attempt.
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub enum QueryDecision {
    Allowed,
    Denied,
    Partial,
}

impl QueryDecision {
    fn code(self) -> u8 {
        match self {
            Self::Allowed => 1,
            Self::Denied => 2,
            Self::Partial => 3,
        }
    }
}

/// What the requester was allowed to learn, without embedding the disclosed values.
#[derive(Clone, Debug, Default, Deserialize, Eq, PartialEq, Serialize)]
pub struct DisclosureSummary {
    /// Semantic classes such as `plate_match`, `address`, `medical_range`, etc.
    pub categories: Vec<String>,
    /// Concrete field names disclosed, when applicable.
    pub fields: Vec<String>,
    /// True only when the capability authorized release of underlying raw evidence.
    pub raw_evidence_included: bool,
}

/// Opaque binding to an authenticated session/operator context.
///
/// Xenia can construct this from its signed session/operator provenance. Other
/// transports can define different `kind` values without changing SIF policy types.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct ProvenanceBinding {
    /// Namespaced producer, e.g. `xenia/session-provenance`.
    pub kind: String,
    /// Producer-specific schema version.
    pub version: u16,
    /// Commitment to the authenticated provenance record.
    pub digest: Commitment,
    /// Optional revocation/authorization epoch committed by the producer.
    pub authorization_epoch: Option<u64>,
}

/// Opaque binding to a verifiable-computation result.
///
/// Symthaea can use this to bind a ZK/verifiable-search result to the SIF query and
/// policy commitments while keeping the proof implementation outside this crate.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct ProofBinding {
    /// Namespaced proof system/circuit identifier.
    pub scheme: String,
    /// Version of the proof statement/circuit contract.
    pub version: u16,
    /// Commitment to the proof/attestation bytes or externally stored proof object.
    pub digest: Commitment,
}

/// Purpose-bound authority to attempt a person-linked query.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct QueryCapability {
    pub query_id: String,
    pub requester_commitment: Commitment,
    pub organization_commitment: Option<Commitment>,
    pub subject_commitment: Commitment,
    pub purpose: String,
    pub authority_ref: String,
    pub policy_commitment: Commitment,
    /// Set-like semantic disclosure classes. Canonical commitment sorting makes
    /// ordering irrelevant.
    pub allowed_disclosures: Vec<String>,
    pub valid_from_ms: UnixMillis,
    pub valid_until_ms: UnixMillis,
    pub budget_class: String,
    pub provenance_required: bool,
}

impl QueryCapability {
    /// Whether the capability is temporally valid at `now_ms`.
    pub fn is_temporally_valid(&self, now_ms: UnixMillis) -> bool {
        self.valid_from_ms <= now_ms && now_ms <= self.valid_until_ms
    }

    /// Stable, domain-separated commitment for cross-stack binding.
    pub fn commitment(&self) -> Commitment {
        let mut w = CanonicalWriter::default();
        w.u16(SIF_PROTOCOL_VERSION);
        w.string(&self.query_id);
        w.commitment(self.requester_commitment);
        w.opt_commitment(self.organization_commitment);
        w.commitment(self.subject_commitment);
        w.string(&self.purpose);
        w.string(&self.authority_ref);
        w.commitment(self.policy_commitment);
        w.string_set(&self.allowed_disclosures);
        w.i64(self.valid_from_ms);
        w.i64(self.valid_until_ms);
        w.string(&self.budget_class);
        w.bool(self.provenance_required);
        Commitment::from_canonical_bytes("query-capability", &w.finish())
    }
}

/// Notification state carried by a receipt.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub enum NotificationState {
    /// Default: visible to the subject immediately.
    Immediate,
    /// Temporarily withheld only if the referenced grant independently validates.
    Delayed { grant_id: String },
    /// Explicitly marked as delivered/revealed by the subject ledger.
    Revealed { revealed_at_ms: UnixMillis },
}

/// A separately authorized, time-bounded exception to immediate notification.
///
/// This is intentionally not a `secret: bool`. A delay must name the query, reason,
/// authority, approvers, reveal time, hard expiry, and authorization commitment.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct DelayedNotificationGrant {
    pub grant_id: String,
    pub query_id: String,
    pub authority_ref: String,
    pub reason_category: String,
    pub approver_commitments: Vec<Commitment>,
    pub issued_at_ms: UnixMillis,
    pub reveal_after_ms: UnixMillis,
    pub hard_expiry_ms: UnixMillis,
    pub policy_commitment: Commitment,
    /// Commitment to the signature, threshold authorization, warrant object, or
    /// other authorization artifact. Validation of that artifact is adapter-owned.
    pub authorization_commitment: Commitment,
}

impl DelayedNotificationGrant {
    /// Structural validity independent of cryptographic authorization verification.
    pub fn is_structurally_valid(&self) -> bool {
        !self.grant_id.is_empty()
            && !self.query_id.is_empty()
            && !self.authority_ref.is_empty()
            && !self.reason_category.is_empty()
            && !self.approver_commitments.is_empty()
            && self.issued_at_ms <= self.reveal_after_ms
            && self.reveal_after_ms <= self.hard_expiry_ms
            && self.authorization_commitment != Commitment::ZERO
    }

    /// Whether this grant may suppress notification for exactly `query_id` now.
    ///
    /// Invalid/mismatched/expired grants return false: notification fails open.
    pub fn suppresses_notification(&self, query_id: &str, now_ms: UnixMillis) -> bool {
        self.is_structurally_valid()
            && self.query_id == query_id
            && self.issued_at_ms <= now_ms
            && now_ms < self.reveal_after_ms
            && now_ms < self.hard_expiry_ms
    }

    /// Stable commitment to the delay grant.
    pub fn commitment(&self) -> Commitment {
        let mut w = CanonicalWriter::default();
        w.u16(SIF_PROTOCOL_VERSION);
        w.string(&self.grant_id);
        w.string(&self.query_id);
        w.string(&self.authority_ref);
        w.string(&self.reason_category);
        w.commitment_set(&self.approver_commitments);
        w.i64(self.issued_at_ms);
        w.i64(self.reveal_after_ms);
        w.i64(self.hard_expiry_ms);
        w.commitment(self.policy_commitment);
        w.commitment(self.authorization_commitment);
        Commitment::from_canonical_bytes("delayed-notification-grant", &w.finish())
    }
}

/// Accountability record for a person-linked lookup attempt.
///
/// Both allowed and denied attempts use the same receipt type. The receipt contains
/// commitments and disclosure metadata, not necessarily the underlying sensitive data.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct SubjectAccessReceipt {
    pub receipt_id: String,
    pub query_id: String,
    pub requester_commitment: Commitment,
    pub organization_commitment: Option<Commitment>,
    pub subject_commitment: Commitment,
    pub purpose: String,
    pub authority_ref: String,
    pub policy_commitment: Commitment,
    pub decision: QueryDecision,
    pub disclosure: DisclosureSummary,
    pub query_commitment: Commitment,
    pub result_commitment: Option<Commitment>,
    pub session_provenance: Option<ProvenanceBinding>,
    pub computation_proof: Option<ProofBinding>,
    pub created_at_ms: UnixMillis,
    pub notification: NotificationState,
    pub retention_class: String,
    pub previous_receipt_commitment: Option<Commitment>,
}

impl SubjectAccessReceipt {
    /// Construct a receipt with the constitutional default: immediate notification.
    #[allow(clippy::too_many_arguments)]
    pub fn immediate(
        receipt_id: impl Into<String>,
        capability: &QueryCapability,
        decision: QueryDecision,
        disclosure: DisclosureSummary,
        result_commitment: Option<Commitment>,
        session_provenance: Option<ProvenanceBinding>,
        computation_proof: Option<ProofBinding>,
        created_at_ms: UnixMillis,
        retention_class: impl Into<String>,
    ) -> Self {
        Self {
            receipt_id: receipt_id.into(),
            query_id: capability.query_id.clone(),
            requester_commitment: capability.requester_commitment,
            organization_commitment: capability.organization_commitment,
            subject_commitment: capability.subject_commitment,
            purpose: capability.purpose.clone(),
            authority_ref: capability.authority_ref.clone(),
            policy_commitment: capability.policy_commitment,
            decision,
            disclosure,
            query_commitment: capability.commitment(),
            result_commitment,
            session_provenance,
            computation_proof,
            created_at_ms,
            notification: NotificationState::Immediate,
            retention_class: retention_class.into(),
            previous_receipt_commitment: None,
        }
    }

    /// Apply a delay reference to a receipt. This does not itself validate the grant.
    /// [`is_subject_visible`] always re-validates the supplied grant and fails open.
    pub fn with_delayed_notification(mut self, grant_id: impl Into<String>) -> Self {
        self.notification = NotificationState::Delayed {
            grant_id: grant_id.into(),
        };
        self
    }

    /// Whether the receipt must be visible to the subject at `now_ms`.
    ///
    /// A delayed state suppresses visibility *only* when a matching valid grant is
    /// supplied. Missing, mismatched, malformed, revealed, or expired grants make the
    /// receipt visible.
    pub fn is_subject_visible(
        &self,
        now_ms: UnixMillis,
        grant: Option<&DelayedNotificationGrant>,
    ) -> bool {
        match &self.notification {
            NotificationState::Immediate | NotificationState::Revealed { .. } => true,
            NotificationState::Delayed { grant_id } => {
                let Some(grant) = grant else {
                    return true;
                };
                if &grant.grant_id != grant_id {
                    return true;
                }
                !grant.suppresses_notification(&self.query_id, now_ms)
            }
        }
    }

    /// Stable commitment to the accountability record.
    pub fn commitment(&self) -> Commitment {
        let mut w = CanonicalWriter::default();
        w.u16(SIF_PROTOCOL_VERSION);
        w.string(&self.receipt_id);
        w.string(&self.query_id);
        w.commitment(self.requester_commitment);
        w.opt_commitment(self.organization_commitment);
        w.commitment(self.subject_commitment);
        w.string(&self.purpose);
        w.string(&self.authority_ref);
        w.commitment(self.policy_commitment);
        w.u8(self.decision.code());
        w.string_set(&self.disclosure.categories);
        w.string_set(&self.disclosure.fields);
        w.bool(self.disclosure.raw_evidence_included);
        w.commitment(self.query_commitment);
        w.opt_commitment(self.result_commitment);
        w.opt_provenance(self.session_provenance.as_ref());
        w.opt_proof(self.computation_proof.as_ref());
        w.i64(self.created_at_ms);
        w.notification(&self.notification);
        w.string(&self.retention_class);
        w.opt_commitment(self.previous_receipt_commitment);
        Commitment::from_canonical_bytes("subject-access-receipt", &w.finish())
    }
}

/// Privacy-preserving witness that an independent party observed a receipt commitment.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct WitnessReceipt {
    pub witness_commitment: Commitment,
    pub receipt_commitment: Commitment,
    pub observed_at_ms: UnixMillis,
    pub witness_scheme: String,
    pub attestation_commitment: Commitment,
}

impl WitnessReceipt {
    pub fn commitment(&self) -> Commitment {
        let mut w = CanonicalWriter::default();
        w.u16(SIF_PROTOCOL_VERSION);
        w.commitment(self.witness_commitment);
        w.commitment(self.receipt_commitment);
        w.i64(self.observed_at_ms);
        w.string(&self.witness_scheme);
        w.commitment(self.attestation_commitment);
        Commitment::from_canonical_bytes("witness-receipt", &w.finish())
    }
}

#[derive(Default)]
struct CanonicalWriter {
    bytes: Vec<u8>,
}

impl CanonicalWriter {
    fn finish(self) -> Vec<u8> {
        self.bytes
    }

    fn u8(&mut self, value: u8) {
        self.bytes.push(value);
    }

    fn u16(&mut self, value: u16) {
        self.bytes.extend_from_slice(&value.to_le_bytes());
    }

    fn u64(&mut self, value: u64) {
        self.bytes.extend_from_slice(&value.to_le_bytes());
    }

    fn i64(&mut self, value: i64) {
        self.bytes.extend_from_slice(&value.to_le_bytes());
    }

    fn bool(&mut self, value: bool) {
        self.u8(u8::from(value));
    }

    fn raw_bytes(&mut self, value: &[u8]) {
        self.u64(value.len() as u64);
        self.bytes.extend_from_slice(value);
    }

    fn string(&mut self, value: &str) {
        self.raw_bytes(value.as_bytes());
    }

    fn commitment(&mut self, value: Commitment) {
        self.bytes.extend_from_slice(&value.0);
    }

    fn opt_commitment(&mut self, value: Option<Commitment>) {
        match value {
            Some(value) => {
                self.u8(1);
                self.commitment(value);
            }
            None => self.u8(0),
        }
    }

    fn string_set(&mut self, values: &[String]) {
        let mut canonical = values.to_vec();
        canonical.sort();
        canonical.dedup();
        self.u64(canonical.len() as u64);
        for value in canonical {
            self.string(&value);
        }
    }

    fn commitment_set(&mut self, values: &[Commitment]) {
        let mut canonical = values.to_vec();
        canonical.sort();
        canonical.dedup();
        self.u64(canonical.len() as u64);
        for value in canonical {
            self.commitment(value);
        }
    }

    fn opt_provenance(&mut self, value: Option<&ProvenanceBinding>) {
        let Some(value) = value else {
            self.u8(0);
            return;
        };
        self.u8(1);
        self.string(&value.kind);
        self.u16(value.version);
        self.commitment(value.digest);
        match value.authorization_epoch {
            Some(epoch) => {
                self.u8(1);
                self.u64(epoch);
            }
            None => self.u8(0),
        }
    }

    fn opt_proof(&mut self, value: Option<&ProofBinding>) {
        let Some(value) = value else {
            self.u8(0);
            return;
        };
        self.u8(1);
        self.string(&value.scheme);
        self.u16(value.version);
        self.commitment(value.digest);
    }

    fn notification(&mut self, value: &NotificationState) {
        match value {
            NotificationState::Immediate => self.u8(1),
            NotificationState::Delayed { grant_id } => {
                self.u8(2);
                self.string(grant_id);
            }
            NotificationState::Revealed { revealed_at_ms } => {
                self.u8(3);
                self.i64(*revealed_at_ms);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn c(seed: u8) -> Commitment {
        Commitment([seed; 32])
    }

    fn capability() -> QueryCapability {
        QueryCapability {
            query_id: "q-001".into(),
            requester_commitment: c(1),
            organization_commitment: Some(c(2)),
            subject_commitment: c(3),
            purpose: "stolen-vehicle-investigation".into(),
            authority_ref: "case:29418".into(),
            policy_commitment: c(4),
            allowed_disclosures: vec!["plate_match".into()],
            valid_from_ms: 1_000,
            valid_until_ms: 10_000,
            budget_class: "person-linked-standard".into(),
            provenance_required: true,
        }
    }

    fn receipt(decision: QueryDecision) -> SubjectAccessReceipt {
        SubjectAccessReceipt::immediate(
            "r-001",
            &capability(),
            decision,
            DisclosureSummary {
                categories: vec!["plate_match".into()],
                fields: vec![],
                raw_evidence_included: false,
            },
            Some(c(5)),
            Some(ProvenanceBinding {
                kind: "xenia/session-provenance".into(),
                version: 1,
                digest: c(6),
                authorization_epoch: Some(9),
            }),
            Some(ProofBinding {
                scheme: "symthaea/verifiable-predicate".into(),
                version: 1,
                digest: c(7),
            }),
            2_000,
            "accountability-standard",
        )
    }

    fn delay() -> DelayedNotificationGrant {
        DelayedNotificationGrant {
            grant_id: "delay-001".into(),
            query_id: "q-001".into(),
            authority_ref: "warrant:abc".into(),
            reason_category: "active-kidnapping-investigation".into(),
            approver_commitments: vec![c(20), c(21)],
            issued_at_ms: 1_500,
            reveal_after_ms: 5_000,
            hard_expiry_ms: 6_000,
            policy_commitment: c(4),
            authorization_commitment: c(22),
        }
    }

    #[test]
    fn immediate_notification_is_the_constructor_default() {
        let receipt = receipt(QueryDecision::Allowed);
        assert!(matches!(receipt.notification, NotificationState::Immediate));
        assert!(receipt.is_subject_visible(2_000, None));
    }

    #[test]
    fn denied_attempt_is_still_subject_visible() {
        let receipt = receipt(QueryDecision::Denied);
        assert_eq!(receipt.decision, QueryDecision::Denied);
        assert!(receipt.is_subject_visible(2_000, None));
    }

    #[test]
    fn valid_delay_suppresses_only_until_reveal_time() {
        let receipt = receipt(QueryDecision::Allowed).with_delayed_notification("delay-001");
        let grant = delay();

        assert!(!receipt.is_subject_visible(2_500, Some(&grant)));
        assert!(receipt.is_subject_visible(5_000, Some(&grant)));
        assert!(receipt.is_subject_visible(6_001, Some(&grant)));
    }

    #[test]
    fn missing_mismatched_or_invalid_delay_fails_open_to_notification() {
        let receipt = receipt(QueryDecision::Allowed).with_delayed_notification("delay-001");
        assert!(receipt.is_subject_visible(2_500, None));

        let mut mismatched = delay();
        mismatched.query_id = "another-query".into();
        assert!(receipt.is_subject_visible(2_500, Some(&mismatched)));

        let mut malformed = delay();
        malformed.approver_commitments.clear();
        assert!(receipt.is_subject_visible(2_500, Some(&malformed)));

        let mut bad_times = delay();
        bad_times.reveal_after_ms = 7_000;
        bad_times.hard_expiry_ms = 6_000;
        assert!(receipt.is_subject_visible(2_500, Some(&bad_times)));
    }

    #[test]
    fn receipt_commitment_changes_when_protected_semantics_change() {
        let a = receipt(QueryDecision::Allowed);
        let mut b = a.clone();
        b.purpose = "different-purpose".into();
        assert_ne!(a.commitment(), b.commitment());

        let mut c_receipt = a.clone();
        c_receipt.decision = QueryDecision::Denied;
        assert_ne!(a.commitment(), c_receipt.commitment());
    }

    #[test]
    fn set_like_fields_have_order_independent_commitments() {
        let mut a = capability();
        a.allowed_disclosures = vec!["match".into(), "count".into(), "match".into()];
        let mut b = a.clone();
        b.allowed_disclosures = vec!["count".into(), "match".into()];
        assert_eq!(a.commitment(), b.commitment());

        let mut grant_a = delay();
        grant_a.approver_commitments = vec![c(21), c(20), c(20)];
        let mut grant_b = grant_a.clone();
        grant_b.approver_commitments = vec![c(20), c(21)];
        assert_eq!(grant_a.commitment(), grant_b.commitment());
    }

    #[test]
    fn capability_time_window_is_closed_and_explicit() {
        let capability = capability();
        assert!(!capability.is_temporally_valid(999));
        assert!(capability.is_temporally_valid(1_000));
        assert!(capability.is_temporally_valid(10_000));
        assert!(!capability.is_temporally_valid(10_001));
    }

    #[test]
    fn serde_roundtrip_preserves_receipt_commitment() {
        let original = receipt(QueryDecision::Partial);
        let json = serde_json::to_string(&original).unwrap();
        let decoded: SubjectAccessReceipt = serde_json::from_str(&json).unwrap();
        assert_eq!(original, decoded);
        assert_eq!(original.commitment(), decoded.commitment());
    }
}
