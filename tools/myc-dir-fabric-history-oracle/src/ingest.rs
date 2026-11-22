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

/// Canonical encoding failures. Event-set digests must never be calculated
/// over an ambiguous or duplicate-ID event collection.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CanonicalEventError {
    FieldTooLarge,
    DuplicateEventId,
}

fn append_len(bytes: &mut Vec<u8>, len: usize) -> Result<(), CanonicalEventError> {
    let len = u32::try_from(len).map_err(|_| CanonicalEventError::FieldTooLarge)?;
    bytes.extend_from_slice(&len.to_be_bytes());
    Ok(())
}

fn append_bytes(bytes: &mut Vec<u8>, value: &[u8]) -> Result<(), CanonicalEventError> {
    append_len(bytes, value.len())?;
    bytes.extend_from_slice(value);
    Ok(())
}

fn append_text(bytes: &mut Vec<u8>, value: &str) -> Result<(), CanonicalEventError> {
    append_bytes(bytes, value.as_bytes())
}

fn append_optional_text(
    bytes: &mut Vec<u8>,
    value: Option<&str>,
) -> Result<(), CanonicalEventError> {
    match value {
        Some(value) => {
            bytes.push(1);
            append_text(bytes, value)?;
        }
        None => bytes.push(0),
    }
    Ok(())
}

fn append_digest(bytes: &mut Vec<u8>, value: &Digest) {
    bytes.extend_from_slice(value);
}

fn append_position(bytes: &mut Vec<u8>, value: crate::FencePosition) {
    bytes.extend_from_slice(&value.owner_epoch.to_be_bytes());
    bytes.extend_from_slice(&value.generation.to_be_bytes());
}

fn append_text_set(
    bytes: &mut Vec<u8>,
    values: impl IntoIterator<Item = String>,
) -> Result<(), CanonicalEventError> {
    let values: Vec<String> = values.into_iter().collect();
    append_len(bytes, values.len())?;
    for value in values {
        append_text(bytes, &value)?;
    }
    Ok(())
}

/// Versioned canonical bytes for one complete typed history event.
/// Parent IDs are sorted because causal parent order has set semantics.
pub fn canonical_event_bytes(event: &EventRecord) -> Result<Vec<u8>, CanonicalEventError> {
    let mut bytes = b"MYC-DIR-HISTORY-EVENT\\0\\x01".to_vec();
    append_text(&mut bytes, &event.id)?;
    append_digest(&mut bytes, &event.scope_digest);
    append_optional_text(&mut bytes, event.owner.as_deref())?;
    match event.owner_sequence {
        Some(sequence) => {
            bytes.push(1);
            bytes.extend_from_slice(&sequence.to_be_bytes());
        }
        None => bytes.push(0),
    }

    let mut parents = event.parents.clone();
    parents.sort();
    append_len(&mut bytes, parents.len())?;
    for parent in parents {
        append_text(&mut bytes, &parent)?;
    }

    match &event.kind {
        EventKind::OwnerBootstrapped { position, bootstrap_digest } => {
            bytes.push(1);
            append_position(&mut bytes, *position);
            append_digest(&mut bytes, bootstrap_digest);
        }
        EventKind::WorkerClaimInstalled { operation_id, worker_id, claim_epoch } => {
            bytes.push(2);
            append_text(&mut bytes, operation_id)?;
            append_text(&mut bytes, worker_id)?;
            bytes.extend_from_slice(&claim_epoch.to_be_bytes());
        }
        EventKind::FenceCommitted {
            previous, installed, revocation_digest, inventory_epoch,
            inventory_manifest_digest, route_set_digest, owner_commit_digest,
        } => {
            bytes.push(3);
            append_position(&mut bytes, *previous);
            append_position(&mut bytes, *installed);
            append_digest(&mut bytes, revocation_digest);
            bytes.extend_from_slice(&inventory_epoch.to_be_bytes());
            append_digest(&mut bytes, inventory_manifest_digest);
            append_digest(&mut bytes, route_set_digest);
            append_digest(&mut bytes, owner_commit_digest);
        }
        EventKind::FenceReadBack {
            fence_commit_event_id, position, durable_readback_digest,
        } => {
            bytes.push(4);
            append_text(&mut bytes, fence_commit_event_id)?;
            append_position(&mut bytes, *position);
            append_digest(&mut bytes, durable_readback_digest);
        }
        EventKind::OwnerAcknowledged {
            fence_commit_event_id, readback_event_id, revocation_digest,
            inventory_epoch, inventory_manifest_digest, route_set_digest,
            owner_commit_digest, durable_readback_digest,
        } => {
            bytes.push(5);
            append_text(&mut bytes, fence_commit_event_id)?;
            append_text(&mut bytes, readback_event_id)?;
            append_digest(&mut bytes, revocation_digest);
            bytes.extend_from_slice(&inventory_epoch.to_be_bytes());
            append_digest(&mut bytes, inventory_manifest_digest);
            append_digest(&mut bytes, route_set_digest);
            append_digest(&mut bytes, owner_commit_digest);
            append_digest(&mut bytes, durable_readback_digest);
        }
        EventKind::InventoryClosed {
            revocation_digest, inventory_epoch, inventory_manifest_digest,
            route_set_digest, required_owners, route_owners,
        } => {
            bytes.push(6);
            append_digest(&mut bytes, revocation_digest);
            bytes.extend_from_slice(&inventory_epoch.to_be_bytes());
            append_digest(&mut bytes, inventory_manifest_digest);
            append_digest(&mut bytes, route_set_digest);
            append_text_set(&mut bytes, required_owners.iter().cloned())?;
            append_len(&mut bytes, route_owners.len())?;
            for (route_id, owner_id) in route_owners {
                append_text(&mut bytes, route_id)?;
                append_text(&mut bytes, owner_id)?;
            }
        }
        EventKind::GlobalBarrierAcknowledged {
            inventory_event_id, revocation_digest, inventory_epoch,
            inventory_manifest_digest, route_set_digest, owner_ack_event_ids,
            owner_ack_set_digest, coordinator_epoch, coordinator_commit_digest,
        } => {
            bytes.push(7);
            append_text(&mut bytes, inventory_event_id)?;
            append_digest(&mut bytes, revocation_digest);
            bytes.extend_from_slice(&inventory_epoch.to_be_bytes());
            append_digest(&mut bytes, inventory_manifest_digest);
            append_digest(&mut bytes, route_set_digest);
            append_text_set(&mut bytes, owner_ack_event_ids.iter().cloned())?;
            append_digest(&mut bytes, owner_ack_set_digest);
            bytes.extend_from_slice(&coordinator_epoch.to_be_bytes());
            append_digest(&mut bytes, coordinator_commit_digest);
        }
        EventKind::EffectAttemptStarted {
            operation_id, attempt_id, attempt_number, worker_id, claim_epoch,
            claim_event_id, position, idempotency_digest, retry_after_outcome_event_id,
        } => {
            bytes.push(8);
            append_text(&mut bytes, operation_id)?;
            append_text(&mut bytes, attempt_id)?;
            bytes.extend_from_slice(&attempt_number.to_be_bytes());
            append_text(&mut bytes, worker_id)?;
            bytes.extend_from_slice(&claim_epoch.to_be_bytes());
            append_text(&mut bytes, claim_event_id)?;
            append_position(&mut bytes, *position);
            append_digest(&mut bytes, idempotency_digest);
            append_optional_text(&mut bytes, retry_after_outcome_event_id.as_deref())?;
        }
        EventKind::EffectCommitted {
            operation_id, attempt_id, attempt_number, worker_id, claim_epoch,
            position, idempotency_digest, effect_digest,
        } => {
            bytes.push(9);
            append_text(&mut bytes, operation_id)?;
            append_text(&mut bytes, attempt_id)?;
            bytes.extend_from_slice(&attempt_number.to_be_bytes());
            append_text(&mut bytes, worker_id)?;
            bytes.extend_from_slice(&claim_epoch.to_be_bytes());
            append_position(&mut bytes, *position);
            append_digest(&mut bytes, idempotency_digest);
            append_digest(&mut bytes, effect_digest);
        }
        EventKind::AttemptOutcomeUnknown { operation_id, attempt_id, attempt_number } => {
            bytes.push(10);
            append_text(&mut bytes, operation_id)?;
            append_text(&mut bytes, attempt_id)?;
            bytes.extend_from_slice(&attempt_number.to_be_bytes());
        }
        EventKind::AttemptReconciled {
            operation_id, attempt_id, attempt_number, unknown_event_id, outcome,
            terminal_at_effect_owner, reconciliation_evidence_digest, effect_commit_event_id,
        } => {
            bytes.push(11);
            append_text(&mut bytes, operation_id)?;
            append_text(&mut bytes, attempt_id)?;
            bytes.extend_from_slice(&attempt_number.to_be_bytes());
            append_text(&mut bytes, unknown_event_id)?;
            bytes.push(match outcome {
                crate::ReconciledOutcome::ConfirmedNoEffect => 1,
                crate::ReconciledOutcome::Committed => 2,
            });
            bytes.push(u8::from(*terminal_at_effect_owner));
            append_digest(&mut bytes, reconciliation_evidence_digest);
            append_optional_text(&mut bytes, effect_commit_event_id.as_deref())?;
        }
        EventKind::RouteStaged {
            route_id, owner, enforcement, required_position, inventory_event_id,
        } => {
            bytes.push(12);
            append_text(&mut bytes, route_id)?;
            append_text(&mut bytes, owner)?;
            bytes.push(match enforcement {
                crate::RouteEnforcement::AtomicOwnerFence => 1,
                crate::RouteEnforcement::CaepSignalOnly => 2,
                crate::RouteEnforcement::TokenExpiryOnly => 3,
                crate::RouteEnforcement::BestEffortInvalidation => 4,
            });
            append_position(&mut bytes, *required_position);
            append_text(&mut bytes, inventory_event_id)?;
        }
        EventKind::RouteActivated {
            route_id, owner, position, staged_event_id, inventory_event_id,
            owner_readback_event_id, global_barrier_event_id,
        } => {
            bytes.push(13);
            append_text(&mut bytes, route_id)?;
            append_text(&mut bytes, owner)?;
            append_position(&mut bytes, *position);
            append_text(&mut bytes, staged_event_id)?;
            append_text(&mut bytes, inventory_event_id)?;
            append_text(&mut bytes, owner_readback_event_id)?;
            append_text(&mut bytes, global_barrier_event_id)?;
        }
    }
    Ok(bytes)
}

/// Canonical, order-independent encoding of one source stream's exact event
/// set. Event IDs are sorted, duplicates reject, and every typed event payload
/// is included. The source authenticator hashes these bytes and compares the
/// result with SourceStreamManifest.event_set_digest.
pub fn canonical_event_set_bytes(
    events: &[&EventRecord],
) -> Result<Vec<u8>, CanonicalEventError> {
    let mut ordered = events.to_vec();
    ordered.sort_by(|left, right| left.id.cmp(&right.id));
    for pair in ordered.windows(2) {
        if pair[0].id == pair[1].id {
            return Err(CanonicalEventError::DuplicateEventId);
        }
    }
    let mut bytes = b"MYC-DIR-HISTORY-EVENT-SET\\0\\x01".to_vec();
    append_len(&mut bytes, ordered.len())?;
    for event in ordered {
        append_bytes(&mut bytes, &canonical_event_bytes(event)?)?;
    }
    Ok(bytes)
}

/// The verifier is a deliberate cryptographic trust boundary. Implementations
/// must verify source signatures and verifier policy, hash
/// canonical_event_set_bytes(covered_events), verify the inventory closure,
/// and recompute the canonical sorted owner-ack-set commitment from the exact
/// OwnerAcknowledged event receipts. A boolean supplied by the caller is not
/// accepted as proof.
pub trait EvidenceAuthenticator {
    /// Verify the signed manifest and independently recompute its event-set
    /// commitment over these exact records in canonical event-ID order. The
    /// adapter must also establish that the signed cursor interval is complete.
    fn verify_source_manifest(
        &self,
        manifest: &SourceStreamManifest,
        covered_events: &[&EventRecord],
    ) -> VerificationStatus;

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

        let covered_events: Vec<&EventRecord> = manifest
            .expected_event_ids
            .iter()
            .filter_map(|event_id| events_by_id.get(event_id).copied())
            .collect();
        record_verification(
            &mut findings,
            authenticator.verify_source_manifest(manifest, &covered_events),
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
    use crate::FencePosition;

    struct TestAuthenticator {
        source: VerificationStatus,
        inventory: VerificationStatus,
        owner_ack_set: VerificationStatus,
    }

    impl EvidenceAuthenticator for TestAuthenticator {
        fn verify_source_manifest(
            &self,
            _manifest: &SourceStreamManifest,
            _covered_events: &[&EventRecord],
        ) -> VerificationStatus {
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

    fn manifest_many(
        source: &str,
        stream: &str,
        owner: Option<&str>,
        event_ids: &[&str],
    ) -> SourceStreamManifest {
        SourceStreamManifest {
            source_id: source.into(),
            stream_id: stream.into(),
            owner: owner.map(str::to_owned),
            scope_digest: digest(1),
            schema_version: "myc-dir-history-v1".into(),
            source_build_digest: digest(2),
            first_cursor: "cursor-1".into(),
            last_cursor: format!("cursor-{}", event_ids.len()),
            expected_event_ids: event_ids.iter().map(|id| (*id).to_owned()).collect(),
            expected_event_count: event_ids.len(),
            event_set_digest: digest(3),
            source_receipt_digest: digest(4),
        }
    }

    fn manifest(
        source: &str,
        stream: &str,
        owner: Option<&str>,
        event_id: &str,
    ) -> SourceStreamManifest {
        manifest_many(source, stream, owner, &[event_id])
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

    fn bundle_with_global_barrier() -> RawHistoryBundle {
        let mut raw = bundle();
        raw.events.push(record(
            "owner-ack-a",
            Some("owner-a"),
            None,
            EventKind::OwnerAcknowledged {
                fence_commit_event_id: "fence-a".into(),
                readback_event_id: "readback-a".into(),
                revocation_digest: digest(5),
                inventory_epoch: 7,
                inventory_manifest_digest: digest(6),
                route_set_digest: digest(7),
                owner_commit_digest: digest(9),
                durable_readback_digest: digest(10),
            },
        ));
        raw.events.push(record(
            "global-ack",
            None,
            None,
            EventKind::GlobalBarrierAcknowledged {
                inventory_event_id: "inventory".into(),
                revocation_digest: digest(5),
                inventory_epoch: 7,
                inventory_manifest_digest: digest(6),
                route_set_digest: digest(7),
                owner_ack_event_ids: ["owner-ack-a".to_owned()].into_iter().collect(),
                owner_ack_set_digest: digest(11),
                coordinator_epoch: 1,
                coordinator_commit_digest: digest(12),
            },
        ));
        raw.source_manifests = vec![
            manifest_many("coordinator", "control", None, &["inventory", "global-ack"]),
            manifest_many("owner-a", "effects", Some("owner-a"), &["bootstrap-a", "owner-ack-a"]),
        ];
        raw
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
    fn unavailable_global_owner_ack_verifier_is_inconclusive() {
        let auth = TestAuthenticator {
            owner_ack_set: VerificationStatus::Unavailable,
            ..good_authenticator()
        };
        let report = ingest_history(bundle_with_global_barrier(), &auth);
        assert_eq!(report.verdict, IngestVerdict::Inconclusive);
        assert!(report.findings.iter().any(|finding| {
            finding.code == IngestFindingCode::OwnerAckSetUnavailable
        }));
    }

    #[test]
    fn invalid_global_owner_ack_commitment_is_a_violation() {
        let auth = TestAuthenticator {
            owner_ack_set: VerificationStatus::Invalid,
            ..good_authenticator()
        };
        let report = ingest_history(bundle_with_global_barrier(), &auth);
        assert_eq!(report.verdict, IngestVerdict::Violation);
        assert!(report.findings.iter().any(|finding| {
            finding.code == IngestFindingCode::OwnerAckSetInvalid
        }));
    }

    #[test]
    fn an_unmanifested_event_is_inconclusive() {
        let mut raw = bundle();
        raw.events.push(record(
            "uncovered-event",
            Some("owner-a"),
            Some(2),
            EventKind::WorkerClaimInstalled {
                operation_id: "op-a".into(),
                worker_id: "worker-a".into(),
                claim_epoch: 1,
            },
        ));
        let report = ingest_history(raw, &good_authenticator());
        assert_eq!(report.verdict, IngestVerdict::Inconclusive);
        assert!(report.findings.iter().any(|finding| {
            finding.code == IngestFindingCode::UncoveredEvent
        }));
    }

    #[test]
    fn scope_substitution_in_event_is_a_violation() {
        let mut raw = bundle();
        raw.events[1].scope_digest = digest(99);
        let report = ingest_history(raw, &good_authenticator());
        assert_eq!(report.verdict, IngestVerdict::Violation);
        assert!(report.findings.iter().any(|finding| {
            finding.code == IngestFindingCode::EventScopeMismatch
        }));
    }

    #[test]
    fn duplicate_source_manifest_identity_is_a_violation() {
        let mut raw = bundle();
        raw.source_manifests.push(raw.source_manifests[0].clone());
        let report = ingest_history(raw, &good_authenticator());
        assert_eq!(report.verdict, IngestVerdict::Violation);
        assert!(report.findings.iter().any(|finding| {
            finding.code == IngestFindingCode::DuplicateManifestIdentity
        }));
    }

    #[test]
    fn missing_referenced_owner_ack_event_is_inconclusive() {
        let mut raw = bundle_with_global_barrier();
        raw.events.retain(|event| event.id != "owner-ack-a");
        let report = ingest_history(raw, &good_authenticator());
        assert_eq!(report.verdict, IngestVerdict::Inconclusive);
        assert!(report.findings.iter().any(|finding| {
            finding.code == IngestFindingCode::OwnerAckEventMissing
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
