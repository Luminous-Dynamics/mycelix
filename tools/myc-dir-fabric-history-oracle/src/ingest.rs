//! Authenticated source-manifest ingestion for the independent history oracle.
//!
//! This module derives completeness from externally authenticated source
//! manifests and exact event coverage. Callers cannot set the completeness
//! booleans on HistoryEvidence directly; the fields are crate-private.
//!
//! Cryptographic verification and trust-root policy are provided by the
//! EvidenceAuthenticator implementation. The ingest layer checks that those
//! proofs bind the exact source, scope, event set, owner, inventory and barrier.

use std::collections::{BTreeMap, BTreeSet};

use crate::{Digest, EventKind, EventRecord, HistoryEvidence};

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SourceStreamManifest {
    pub source_id: String,
    pub stream_id: String,
    /// Some(owner ID) for an effect-owner stream; None for the coordinator stream.
    pub owner: Option<String>,
    pub scope_digest: Digest,
    pub schema_version: String,
    pub source_build_digest: Digest,
    pub first_cursor: String,
    pub last_cursor: String,
    pub expected_event_ids: BTreeSet<String>,
    pub expected_event_count: usize,
    pub event_set_digest: Digest,
    pub source_receipt_digest: Digest,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RawHistoryBundle {
    pub scope_digest: Digest,
    pub events: Vec<EventRecord>,
    pub source_manifests: Vec<SourceStreamManifest>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum VerificationStatus {
    Verified,
    Invalid,
    Unavailable,
}

/// The verifier is a deliberate cryptographic trust boundary. Implementations
/// must verify source signatures, verifier policy, inventory-closure proof and
/// recompute the canonical sorted owner-ack-set commitment from the exact
/// OwnerAcknowledged event receipts. A boolean supplied by the caller is not
/// accepted as proof.
pub trait EvidenceAuthenticator {
    fn verify_source_manifest(&self, manifest: &SourceStreamManifest) -> VerificationStatus;

    fn verify_inventory_closure(&self, inventory_event: &EventRecord) -> VerificationStatus;

    fn verify_owner_ack_set(
        &self,
        barrier_event: &EventRecord,
        owner_ack_events: &[&EventRecord],
    ) -> VerificationStatus;
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum IngestVerdict {
    Ready,
    Violation,
    Inconclusive,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub enum IngestFindingCode {
    EmptyScopeDigest,
    EmptyEventBundle,
    EmptyEventId,
    DuplicateEventId,
    EventScopeMismatch,
    NoInventoryClosure,
    InventoryClosureInvalid,
    InventoryClosureUnavailable,
    EmptySourceManifestSet,
    EmptyManifestIdentity,
    EmptyManifestCursor,
    EmptyManifestEventSet,
    ManifestEventCountMismatch,
    ManifestZeroDigest,
    DuplicateManifestIdentity,
    ManifestScopeMismatch,
    SourceManifestInvalid,
    SourceManifestUnavailable,
    ManifestEventMissing,
    ManifestEventOwnerMismatch,
    DuplicateEventCoverage,
    UncoveredEvent,
    MissingOwnerManifest,
    UnexpectedOwnerManifest,
    OwnerAckEventMissing,
    OwnerAckWrongEventKind,
    OwnerAckSetInvalid,
    OwnerAckSetUnavailable,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct IngestFinding {
    pub code: IngestFindingCode,
    pub source_stream: Option<String>,
    pub event_id: Option<String>,
    pub detail: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct IngestReport {
    pub verdict: IngestVerdict,
    pub findings: Vec<IngestFinding>,
    history: Option<HistoryEvidence>,
}

impl IngestReport {
    pub fn history(&self) -> Option<&HistoryEvidence> {
        self.history.as_ref()
    }

    pub fn into_history(self) -> Option<HistoryEvidence> {
        self.history
    }
}

#[derive(Default)]
struct Findings {
    items: Vec<IngestFinding>,
    violation: bool,
    inconclusive: bool,
}

impl Findings {
    fn violation(
        &mut self,
        code: IngestFindingCode,
        stream: Option<&str>,
        event: Option<&str>,
        detail: impl Into<String>,
    ) {
        self.violation = true;
        self.items.push(IngestFinding {
            code,
            source_stream: stream.map(str::to_owned),
            event_id: event.map(str::to_owned),
            detail: detail.into(),
        });
    }

    fn inconclusive(
        &mut self,
        code: IngestFindingCode,
        stream: Option<&str>,
        event: Option<&str>,
        detail: impl Into<String>,
    ) {
        self.inconclusive = true;
        self.items.push(IngestFinding {
            code,
            source_stream: stream.map(str::to_owned),
            event_id: event.map(str::to_owned),
            detail: detail.into(),
        });
    }

    fn finish(mut self, history: Option<HistoryEvidence>) -> IngestReport {
        self.items.sort_by(|left, right| {
            (left.code, &left.source_stream, &left.event_id, &left.detail).cmp(&(
                right.code,
                &right.source_stream,
                &right.event_id,
                &right.detail,
            ))
        });
        let verdict = if self.violation {
            IngestVerdict::Violation
        } else if self.inconclusive {
            IngestVerdict::Inconclusive
        } else if history.is_some() {
            IngestVerdict::Ready
        } else {
            IngestVerdict::Inconclusive
        };
        IngestReport {
            verdict,
            findings: self.items,
            history: if verdict == IngestVerdict::Ready {
                history
            } else {
                None
            },
        }
    }
}

fn digest_is_nonzero(digest: &Digest) -> bool {
    digest.iter().any(|byte| *byte != 0)
}

fn checked_text(value: &str) -> bool {
    !value.trim().is_empty() && value.len() <= 512
}

fn record_verification(
    findings: &mut Findings,
    status: VerificationStatus,
    invalid_code: IngestFindingCode,
    unavailable_code: IngestFindingCode,
    stream: Option<&str>,
    event: Option<&str>,
    subject: &str,
) {
    match status {
        VerificationStatus::Verified => {}
        VerificationStatus::Invalid => findings.violation(
            invalid_code,
            stream,
            event,
            format!("{subject} failed independent authentication/verification"),
        ),
        VerificationStatus::Unavailable => findings.inconclusive(
            unavailable_code,
            stream,
            event,
            format!("{subject} verification is unavailable"),
        ),
    }
}

/// Authenticates source manifests and proves exact event coverage before
/// constructing HistoryEvidence. Only a Ready report contains an evidence
/// object accepted by the audit_history API.
pub fn ingest_history(
    bundle: RawHistoryBundle,
    authenticator: &impl EvidenceAuthenticator,
) -> IngestReport {
    let mut findings = Findings::default();

    if !digest_is_nonzero(&bundle.scope_digest) {
        findings.violation(
            IngestFindingCode::EmptyScopeDigest,
            None,
            None,
            "bundle scope digest must be nonzero",
        );
    }
    if bundle.events.is_empty() {
        findings.inconclusive(
            IngestFindingCode::EmptyEventBundle,
            None,
            None,
            "empty event bundle cannot establish a complete history",
        );
    }
    if bundle.source_manifests.is_empty() {
        findings.inconclusive(
            IngestFindingCode::EmptySourceManifestSet,
            None,
            None,
            "no source manifests were supplied",
        );
    }

    let mut events_by_id: BTreeMap<String, &EventRecord> = BTreeMap::new();
    for event in &bundle.events {
        if event.id.trim().is_empty() {
            findings.violation(
                IngestFindingCode::EmptyEventId,
                None,
                None,
                "event identifier is empty",
            );
            continue;
        }
        if events_by_id.insert(event.id.clone(), event).is_some() {
            findings.violation(
                IngestFindingCode::DuplicateEventId,
                None,
                Some(&event.id),
                "event ID is duplicated in the raw bundle",
            );
        }
        if event.scope_digest != bundle.scope_digest {
            findings.violation(
                IngestFindingCode::EventScopeMismatch,
                None,
                Some(&event.id),
                "event scope digest differs from the bundle scope",
            );
        }
    }

    let inventory_events: Vec<&EventRecord> = bundle
        .events
        .iter()
        .filter(|event| matches!(&event.kind, EventKind::InventoryClosed { .. }))
        .collect();
    if inventory_events.is_empty() {
        findings.inconclusive(
            IngestFindingCode::NoInventoryClosure,
            None,
            None,
            "bundle contains no independently verified inventory-closure event",
        );
    }

    let mut required_owners = BTreeSet::new();
    for inventory in &inventory_events {
        if let EventKind::InventoryClosed {
            required_owners: owners,
            route_owners,
            ..
        } = &inventory.kind
        {
            required_owners.extend(owners.iter().cloned());
            let route_owner_projection: BTreeSet<String> =
                route_owners.values().cloned().collect();
            if route_owner_projection != *owners || route_owners.is_empty() {
                findings.violation(
                    IngestFindingCode::InventoryClosureInvalid,
                    None,
                    Some(&inventory.id),
                    "inventory route-to-owner mapping does not project to its exact required owner set",
                );
            }
        }
        record_verification(
            &mut findings,
            authenticator.verify_inventory_closure(inventory),
            IngestFindingCode::InventoryClosureInvalid,
            IngestFindingCode::InventoryClosureUnavailable,
            None,
            Some(&inventory.id),
            "inventory closure",
        );
    }

    let mut manifest_ids = BTreeSet::new();
    let mut covered_event_ids: BTreeMap<String, String> = BTreeMap::new();
    let mut manifested_owners = BTreeSet::new();
    let mut verified_owner_streams = BTreeSet::new();

    for manifest in &bundle.source_manifests {
        let stream_label = format!("{}:{}", manifest.source_id, manifest.stream_id);
        if !checked_text(&manifest.source_id) || !checked_text(&manifest.stream_id) {
            findings.violation(
                IngestFindingCode::EmptyManifestIdentity,
                Some(&stream_label),
                None,
                "source and stream identities must be nonempty and bounded",
            );
        }
        if !checked_text(&manifest.schema_version)
            || !checked_text(&manifest.first_cursor)
            || !checked_text(&manifest.last_cursor)
        {
            findings.violation(
                IngestFindingCode::EmptyManifestCursor,
                Some(&stream_label),
                None,
                "schema version and capture cursors must be nonempty and bounded",
            );
        }
        if manifest.expected_event_ids.is_empty() {
            findings.violation(
                IngestFindingCode::EmptyManifestEventSet,
                Some(&stream_label),
                None,
                "a source manifest may not attest an empty event set",
            );
        }
        if manifest.expected_event_count != manifest.expected_event_ids.len() {
            findings.violation(
                IngestFindingCode::ManifestEventCountMismatch,
                Some(&stream_label),
                None,
                "declared event count differs from the exact event-ID set size",
            );
        }
        if !digest_is_nonzero(&manifest.scope_digest)
            || !digest_is_nonzero(&manifest.source_build_digest)
            || !digest_is_nonzero(&manifest.event_set_digest)
            || !digest_is_nonzero(&manifest.source_receipt_digest)
        {
            findings.violation(
                IngestFindingCode::ManifestZeroDigest,
                Some(&stream_label),
                None,
                "manifest scope/build/event-set/source-receipt commitments must be nonzero",
            );
        }
        if manifest.scope_digest != bundle.scope_digest {
            findings.violation(
                IngestFindingCode::ManifestScopeMismatch,
                Some(&stream_label),
                None,
                "source manifest scope differs from the exact bundle scope",
            );
        }
        if let Some(owner) = &manifest.owner {
            if !checked_text(owner) {
                findings.violation(
                    IngestFindingCode::EmptyManifestIdentity,
                    Some(&stream_label),
                    None,
                    "owner identity must be nonempty and bounded",
                );
            }
            manifested_owners.insert(owner.clone());
        }
        if !manifest_ids.insert((manifest.source_id.clone(), manifest.stream_id.clone())) {
            findings.violation(
                IngestFindingCode::DuplicateManifestIdentity,
                Some(&stream_label),
                None,
                "source/stream manifest identity is duplicated",
            );
        }

        record_verification(
            &mut findings,
            authenticator.verify_source_manifest(manifest),
            IngestFindingCode::SourceManifestInvalid,
            IngestFindingCode::SourceManifestUnavailable,
            Some(&stream_label),
            None,
            "source stream manifest",
        );

        for event_id in &manifest.expected_event_ids {
            let Some(event) = events_by_id.get(event_id) else {
                findings.inconclusive(
                    IngestFindingCode::ManifestEventMissing,
                    Some(&stream_label),
                    Some(event_id),
                    "signed source manifest declares an event missing from the supplied bundle",
                );
                continue;
            };
            if event.scope_digest != manifest.scope_digest || event.owner != manifest.owner {
                findings.violation(
                    IngestFindingCode::ManifestEventOwnerMismatch,
                    Some(&stream_label),
                    Some(event_id),
                    "event owner or scope differs from its source manifest",
                );
            }
            if covered_event_ids
                .insert(event_id.clone(), stream_label.clone())
                .is_some()
            {
                findings.violation(
                    IngestFindingCode::DuplicateEventCoverage,
                    Some(&stream_label),
                    Some(event_id),
                    "event is claimed by more than one source manifest",
                );
            }
        }
        if manifest.expected_event_ids.iter().all(|event_id| {
            events_by_id
                .get(event_id)
                .is_some_and(|event| event.owner == manifest.owner)
        }) {
            if let Some(owner) = &manifest.owner {
                verified_owner_streams.insert(owner.clone());
            }
        }
    }

    for event_id in events_by_id.keys() {
        if !covered_event_ids.contains_key(event_id) {
            findings.inconclusive(
                IngestFindingCode::UncoveredEvent,
                None,
                Some(event_id),
                "event is not covered by any authenticated source manifest",
            );
        }
    }

    for owner in &required_owners {
        if !verified_owner_streams.contains(owner) {
            findings.inconclusive(
                IngestFindingCode::MissingOwnerManifest,
                None,
                None,
                format!("required effect owner {owner} has no complete authenticated source manifest"),
            );
        }
    }
    for owner in &manifested_owners {
        if !required_owners.contains(owner) {
            findings.violation(
                IngestFindingCode::UnexpectedOwnerManifest,
                None,
                None,
                format!("source manifest includes unexpected owner {owner} outside the verified inventory set"),
            );
        }
    }

    for event in &bundle.events {
        if let EventKind::GlobalBarrierAcknowledged {
            owner_ack_event_ids,
            ..
        } = &event.kind
        {
            let mut owner_ack_events = Vec::with_capacity(owner_ack_event_ids.len());
            for ack_id in owner_ack_event_ids {
                let Some(ack_event) = events_by_id.get(ack_id).copied() else {
                    findings.inconclusive(
                        IngestFindingCode::OwnerAckEventMissing,
                        None,
                        Some(&event.id),
                        format!("global barrier references missing owner acknowledgement {ack_id}"),
                    );
                    continue;
                };
                if !matches!(&ack_event.kind, EventKind::OwnerAcknowledged { .. }) {
                    findings.violation(
                        IngestFindingCode::OwnerAckWrongEventKind,
                        None,
                        Some(ack_id),
                        "global barrier owner-ack reference does not identify an OwnerAcknowledged event",
                    );
                    continue;
                }
                owner_ack_events.push(ack_event);
            }
            record_verification(
                &mut findings,
                authenticator.verify_owner_ack_set(event, &owner_ack_events),
                IngestFindingCode::OwnerAckSetInvalid,
                IngestFindingCode::OwnerAckSetUnavailable,
                None,
                Some(&event.id),
                "global owner acknowledgement set",
            );
        }
    }

    if findings.violation || findings.inconclusive {
        return findings.finish(None);
    }

    let history = HistoryEvidence {
        scope_digest: bundle.scope_digest,
        event_set_complete: true,
        complete_owner_streams: required_owners,
        events: bundle.events,
    };
    findings.finish(Some(history))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{FencePosition, FindingCode};

    struct TestAuthenticator {
        source: VerificationStatus,
        inventory: VerificationStatus,
        owner_ack_set: VerificationStatus,
    }

    impl EvidenceAuthenticator for TestAuthenticator {
        fn verify_source_manifest(&self, _manifest: &SourceStreamManifest) -> VerificationStatus {
            self.source
        }

        fn verify_inventory_closure(&self, _inventory_event: &EventRecord) -> VerificationStatus {
            self.inventory
        }

        fn verify_owner_ack_set(
            &self,
            _barrier_event: &EventRecord,
            _owner_ack_events: &[&EventRecord],
        ) -> VerificationStatus {
            self.owner_ack_set
        }
    }

    fn digest(byte: u8) -> Digest {
        [byte; 32]
    }

    fn record(
        id: &str,
        owner: Option<&str>,
        owner_sequence: Option<u64>,
        kind: EventKind,
    ) -> EventRecord {
        EventRecord {
            id: id.into(),
            scope_digest: digest(1),
            parents: Vec::new(),
            owner: owner.map(str::to_owned),
            owner_sequence,
            kind,
        }
    }

    fn manifest(
        source: &str,
        stream: &str,
        owner: Option<&str>,
        event_id: &str,
    ) -> SourceStreamManifest {
        SourceStreamManifest {
            source_id: source.into(),
            stream_id: stream.into(),
            owner: owner.map(str::to_owned),
            scope_digest: digest(1),
            schema_version: "myc-dir-history-v1".into(),
            source_build_digest: digest(2),
            first_cursor: "cursor-1".into(),
            last_cursor: "cursor-1".into(),
            expected_event_ids: [event_id.to_owned()].into_iter().collect(),
            expected_event_count: 1,
            event_set_digest: digest(3),
            source_receipt_digest: digest(4),
        }
    }

    fn bundle() -> RawHistoryBundle {
        let required_owners = ["owner-a"].into_iter().map(str::to_owned).collect();
        let inventory = record(
            "inventory",
            None,
            None,
            EventKind::InventoryClosed {
                revocation_digest: digest(5),
                inventory_epoch: 7,
                inventory_manifest_digest: digest(6),
                route_set_digest: digest(7),
                required_owners,
                route_owners: [("route-a".into(), "owner-a".into())]
                    .into_iter()
                    .collect(),
            },
        );
        let bootstrap = record(
            "bootstrap-a",
            Some("owner-a"),
            Some(1),
            EventKind::OwnerBootstrapped {
                position: FencePosition::new(1, 1).unwrap(),
                bootstrap_digest: digest(8),
            },
        );
        RawHistoryBundle {
            scope_digest: digest(1),
            events: vec![inventory, bootstrap],
            source_manifests: vec![
                manifest("coordinator", "control", None, "inventory"),
                manifest("owner-a", "effects", Some("owner-a"), "bootstrap-a"),
            ],
        }
    }

    fn good_authenticator() -> TestAuthenticator {
        TestAuthenticator {
            source: VerificationStatus::Verified,
            inventory: VerificationStatus::Verified,
            owner_ack_set: VerificationStatus::Verified,
        }
    }

    #[test]
    fn complete_authenticated_bundle_produces_history_evidence() {
        let report = ingest_history(bundle(), &good_authenticator());
        assert_eq!(report.verdict, IngestVerdict::Ready);
        let history = report.history().unwrap();
        assert_eq!(history.event_count(), 2);
        assert_eq!(history.complete_owner_ids().collect::<Vec<_>>(), vec!["owner-a"]);
    }

    #[test]
    fn missing_required_owner_manifest_is_inconclusive() {
        let mut raw = bundle();
        raw.source_manifests.retain(|manifest| manifest.owner.is_none());
        let report = ingest_history(raw, &good_authenticator());
        assert_eq!(report.verdict, IngestVerdict::Inconclusive);
        assert!(report.findings.iter().any(|finding| {
            finding.code == IngestFindingCode::MissingOwnerManifest
        }));
    }

    #[test]
    fn source_manifest_signature_failure_is_a_violation() {
        let auth = TestAuthenticator {
            source: VerificationStatus::Invalid,
            ..good_authenticator()
        };
        let report = ingest_history(bundle(), &auth);
        assert_eq!(report.verdict, IngestVerdict::Violation);
        assert!(report.findings.iter().any(|finding| {
            finding.code == IngestFindingCode::SourceManifestInvalid
        }));
    }

    #[test]
    fn unavailable_source_verifier_is_inconclusive() {
        let auth = TestAuthenticator {
            source: VerificationStatus::Unavailable,
            ..good_authenticator()
        };
        let report = ingest_history(bundle(), &auth);
        assert_eq!(report.verdict, IngestVerdict::Inconclusive);
        assert!(report.findings.iter().any(|finding| {
            finding.code == IngestFindingCode::SourceManifestUnavailable
        }));
    }

    #[test]
    fn missing_event_from_signed_source_manifest_is_inconclusive() {
        let mut raw = bundle();
        raw.source_manifests[1].expected_event_ids =
            ["bootstrap-a".to_owned(), "missing-event".to_owned()]
                .into_iter()
                .collect();
        raw.source_manifests[1].expected_event_count = 2;
        let report = ingest_history(raw, &good_authenticator());
        assert_eq!(report.verdict, IngestVerdict::Inconclusive);
        assert!(report.findings.iter().any(|finding| {
            finding.code == IngestFindingCode::ManifestEventMissing
        }));
    }

    #[test]
    fn overlapping_manifest_coverage_is_a_violation() {
        let mut raw = bundle();
        raw.source_manifests.push(manifest(
            "owner-a-replica",
            "effects-copy",
            Some("owner-a"),
            "bootstrap-a",
        ));
        let report = ingest_history(raw, &good_authenticator());
        assert_eq!(report.verdict, IngestVerdict::Violation);
        assert!(report.findings.iter().any(|finding| {
            finding.code == IngestFindingCode::DuplicateEventCoverage
        }));
    }

    #[test]
    fn owner_substitution_in_source_manifest_is_a_violation() {
        let mut raw = bundle();
        raw.source_manifests[1].owner = Some("owner-b".into());
        let report = ingest_history(raw, &good_authenticator());
        assert_eq!(report.verdict, IngestVerdict::Violation);
        assert!(report.findings.iter().any(|finding| {
            finding.code == IngestFindingCode::ManifestEventOwnerMismatch
        }));
    }

    #[test]
    fn invalid_inventory_closure_proof_is_a_violation() {
        let auth = TestAuthenticator {
            inventory: VerificationStatus::Invalid,
            ..good_authenticator()
        };
        let report = ingest_history(bundle(), &auth);
        assert_eq!(report.verdict, IngestVerdict::Violation);
        assert!(report.findings.iter().any(|finding| {
            finding.code == IngestFindingCode::InventoryClosureInvalid
        }));
    }
}
