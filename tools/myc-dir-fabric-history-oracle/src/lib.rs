//! Independent, dependency-free history oracle for Mycelix Directory Fabric.
//!
//! This crate is deliberately separate from the production reference semantics.
//! It checks captured evidence; it does not authenticate signatures, establish
//! source completeness, hash bytes, or connect to databases and brokers. Those
//! trust boundaries must be completed by the evidence-ingest adapter before
//! invoking this checker.
//!
//! A missing or contradictory history is never a pass. The result is one of
//! VALID, VIOLATION, or INCONCLUSIVE.

use std::collections::{BTreeMap, BTreeSet};

pub type Digest = [u8; 32];

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct FencePosition {
    pub owner_epoch: u64,
    pub generation: u64,
}

impl FencePosition {
    pub fn new(owner_epoch: u64, generation: u64) -> Option<Self> {
        if owner_epoch == 0 || generation == 0 {
            return None;
        }
        Some(Self {
            owner_epoch,
            generation,
        })
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RouteEnforcement {
    AtomicOwnerFence,
    CaepSignalOnly,
    TokenExpiryOnly,
    BestEffortInvalidation,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ReconciledOutcome {
    ConfirmedNoEffect,
    Committed,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum EventKind {
    OwnerBootstrapped {
        position: FencePosition,
        bootstrap_digest: Digest,
    },
    WorkerClaimInstalled {
        operation_id: String,
        worker_id: String,
        claim_epoch: u64,
    },
    FenceCommitted {
        previous: FencePosition,
        installed: FencePosition,
        revocation_digest: Digest,
        inventory_epoch: u64,
        inventory_manifest_digest: Digest,
        route_set_digest: Digest,
        owner_commit_digest: Digest,
    },
    FenceReadBack {
        fence_commit_event_id: String,
        position: FencePosition,
        durable_readback_digest: Digest,
    },
    OwnerAcknowledged {
        fence_commit_event_id: String,
        readback_event_id: String,
        revocation_digest: Digest,
        inventory_epoch: u64,
        inventory_manifest_digest: Digest,
        route_set_digest: Digest,
    },
    InventoryClosed {
        revocation_digest: Digest,
        inventory_epoch: u64,
        inventory_manifest_digest: Digest,
        route_set_digest: Digest,
        required_owners: BTreeSet<String>,
        required_routes: BTreeSet<String>,
    },
    GlobalBarrierAcknowledged {
        inventory_event_id: String,
        revocation_digest: Digest,
        inventory_epoch: u64,
        inventory_manifest_digest: Digest,
        route_set_digest: Digest,
        owner_ack_event_ids: BTreeSet<String>,
        coordinator_epoch: u64,
        coordinator_commit_digest: Digest,
    },
    EffectAttemptStarted {
        operation_id: String,
        attempt_id: String,
        attempt_number: u32,
        worker_id: String,
        claim_epoch: u64,
        claim_event_id: String,
        position: FencePosition,
        idempotency_digest: Digest,
        retry_after_outcome_event_id: Option<String>,
    },
    EffectCommitted {
        operation_id: String,
        attempt_id: String,
        attempt_number: u32,
        worker_id: String,
        claim_epoch: u64,
        position: FencePosition,
        idempotency_digest: Digest,
        effect_digest: Digest,
    },
    AttemptOutcomeUnknown {
        operation_id: String,
        attempt_id: String,
        attempt_number: u32,
    },
    AttemptReconciled {
        operation_id: String,
        attempt_id: String,
        attempt_number: u32,
        unknown_event_id: String,
        outcome: ReconciledOutcome,
        effect_commit_event_id: Option<String>,
    },
    RouteStaged {
        route_id: String,
        owner: String,
        enforcement: RouteEnforcement,
        required_position: FencePosition,
        inventory_event_id: String,
    },
    RouteActivated {
        route_id: String,
        owner: String,
        position: FencePosition,
        staged_event_id: String,
        inventory_event_id: String,
        owner_readback_event_id: String,
        global_barrier_event_id: String,
    },
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct EventRecord {
    pub id: String,
    /// Direct causal predecessors. Wall-clock timestamps are intentionally not
    /// used to establish safety ordering.
    pub parents: Vec<String>,
    /// Required for owner-local sequenced events. Absent on coordinator events.
    pub owner: Option<String>,
    /// Durable, owner-local sequence/commit position. Not a Unix timestamp.
    pub owner_sequence: Option<u64>,
    pub kind: EventKind,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct HistoryEvidence {
    /// True only if the ingestion layer has verified that the event bundle is
    /// complete for the declared interval and has checked source authenticity.
    pub event_set_complete: bool,
    /// Owners whose owner-local event streams are complete for this history.
    pub complete_owner_streams: BTreeSet<String>,
    pub events: Vec<EventRecord>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Verdict {
    Valid,
    Violation,
    Inconclusive,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub enum FindingCode {
    EmptyEventId,
    DuplicateEventId,
    DuplicateParent,
    MissingCausalParent,
    CausalCycle,
    MissingOwnerIdentity,
    MissingOwnerSequence,
    ZeroOwnerSequence,
    UnexpectedOwnerSequence,
    DuplicateOwnerSequence,
    OwnerSequenceCausalOrderMismatch,
    InvalidDigest,
    InvalidPosition,
    MissingOwnerBootstrap,
    DuplicateOwnerBootstrap,
    OwnerPositionRegression,
    FencePreviousPositionMismatch,
    FencePositionNotMonotonic,
    ReadBackMissingFenceCommit,
    ReadBackOwnerMismatch,
    ReadBackPositionMismatch,
    ReadBackBeforeFenceCommit,
    ReadBackMissingCausalParent,
    OwnerAckMissingFenceCommit,
    OwnerAckMissingReadBack,
    OwnerAckBindingMismatch,
    OwnerAckMissingCausalParent,
    EffectMissingAttempt,
    EffectAttemptBindingMismatch,
    EffectMissingCausalParent,
    EffectPositionNotCurrent,
    EffectClaimNotCurrent,
    DuplicateLogicalEffect,
    DuplicateClaimEpoch,
    ClaimEpochNotMonotonic,
    AttemptNumberGap,
    DuplicateAttemptNumber,
    RetryWithoutNoEffectReconciliation,
    RetryAfterCommittedAttempt,
    UnknownAttemptUnreconciled,
    ReconciliationMissingUnknown,
    ReconciliationBindingMismatch,
    ReconciliationMissingCommit,
    ReconciliationUnexpectedCommit,
    InventoryMissing,
    InventoryOwnerStreamIncomplete,
    InventoryEmptyOwnerSet,
    GlobalBarrierMissingInventory,
    GlobalBarrierMissingOwnerAck,
    GlobalBarrierOwnerSetMismatch,
    GlobalBarrierAckBindingMismatch,
    GlobalBarrierMissingCausalParent,
    RouteMissingStage,
    RouteMissingInventory,
    RouteMissingReadBack,
    RouteMissingGlobalBarrier,
    RouteAdmissionBindingMismatch,
    RouteNonFenceable,
    RouteMissingCausalParent,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Finding {
    pub code: FindingCode,
    pub event_id: Option<String>,
    pub detail: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AuditReport {
    pub verdict: Verdict,
    pub checked_events: usize,
    pub findings: Vec<Finding>,
}

#[derive(Default)]
struct Findings {
    items: Vec<Finding>,
    has_violation: bool,
    has_inconclusive: bool,
}

impl Findings {
    fn violation(
        &mut self,
        code: FindingCode,
        event_id: Option<&str>,
        detail: impl Into<String>,
    ) {
        self.has_violation = true;
        self.items.push(Finding {
            code,
            event_id: event_id.map(str::to_owned),
            detail: detail.into(),
        });
    }

    fn inconclusive(
        &mut self,
        code: FindingCode,
        event_id: Option<&str>,
        detail: impl Into<String>,
    ) {
        self.has_inconclusive = true;
        self.items.push(Finding {
            code,
            event_id: event_id.map(str::to_owned),
            detail: detail.into(),
        });
    }

    fn report(mut self, checked_events: usize) -> AuditReport {
        self.items.sort_by(|a, b| {
            (a.code, &a.event_id, &a.detail).cmp(&(b.code, &b.event_id, &b.detail))
        });
        let verdict = if self.has_violation {
            Verdict::Violation
        } else if self.has_inconclusive {
            Verdict::Inconclusive
        } else {
            Verdict::Valid
        };
        AuditReport {
            verdict,
            checked_events,
            findings: self.items,
        }
    }
}

#[derive(Clone)]
struct FenceInfo {
    owner: String,
    sequence: u64,
    installed: FencePosition,
    revocation_digest: Digest,
    inventory_epoch: u64,
    inventory_manifest_digest: Digest,
    route_set_digest: Digest,
}

#[derive(Clone)]
struct ReadBackInfo {
    owner: String,
    sequence: u64,
    fence_commit_event_id: String,
    position: FencePosition,
    durable_readback_digest: Digest,
}

#[derive(Clone)]
struct InventoryInfo {
    revocation_digest: Digest,
    inventory_epoch: u64,
    inventory_manifest_digest: Digest,
    route_set_digest: Digest,
    required_owners: BTreeSet<String>,
    required_routes: BTreeSet<String>,
}

fn digest_is_nonzero(digest: &Digest) -> bool {
    digest.iter().any(|byte| *byte != 0)
}

fn event_requires_owner_sequence(kind: &EventKind) -> bool {
    matches!(
        kind,
        EventKind::OwnerBootstrapped { .. }
            | EventKind::WorkerClaimInstalled { .. }
            | EventKind::FenceCommitted { .. }
            | EventKind::FenceReadBack { .. }
            | EventKind::EffectCommitted { .. }
    )
}

fn event_requires_owner_identity(kind: &EventKind) -> bool {
    event_requires_owner_sequence(kind)
        || matches!(
            kind,
            EventKind::OwnerAcknowledged { .. }
                | EventKind::EffectAttemptStarted { .. }
                | EventKind::AttemptOutcomeUnknown { .. }
                | EventKind::AttemptReconciled { .. }
        )
}

fn topological_order(
    events: &BTreeMap<String, EventRecord>,
    findings: &mut Findings,
) -> Option<Vec<String>> {
    let mut indegree: BTreeMap<String, usize> =
        events.keys().map(|id| (id.clone(), 0)).collect();
    let mut outgoing: BTreeMap<String, Vec<String>> = BTreeMap::new();

    for event in events.values() {
        let mut unique_parents = BTreeSet::new();
        for parent in &event.parents {
            if !unique_parents.insert(parent) {
                findings.violation(
                    FindingCode::DuplicateParent,
                    Some(&event.id),
                    format!("duplicate direct causal parent {parent}"),
                );
                continue;
            }
            if !events.contains_key(parent) {
                findings.inconclusive(
                    FindingCode::MissingCausalParent,
                    Some(&event.id),
                    format!("required causal parent {parent} is absent from the evidence bundle"),
                );
                continue;
            }
            *indegree.get_mut(&event.id).expect("event index exists") += 1;
            outgoing
                .entry(parent.clone())
                .or_default()
                .push(event.id.clone());
        }
    }

    let mut ready: BTreeSet<String> = indegree
        .iter()
        .filter(|(_, degree)| **degree == 0)
        .map(|(id, _)| id.clone())
        .collect();
    let mut ordered = Vec::with_capacity(events.len());

    while let Some(id) = ready.pop_first() {
        ordered.push(id.clone());
        for child in outgoing.get(&id).into_iter().flatten() {
            let degree = indegree.get_mut(child).expect("child index exists");
            *degree -= 1;
            if *degree == 0 {
                ready.insert(child.clone());
            }
        }
    }

    if ordered.len() != events.len() {
        findings.violation(
            FindingCode::CausalCycle,
            None,
            "causal parent graph contains a cycle",
        );
        None
    } else {
        Some(ordered)
    }
}

fn has_parent(event: &EventRecord, parent_id: &str) -> bool {
    event.parents.iter().any(|parent| parent == parent_id)
}

/// Returns true only when the causal-parent graph proves that one event
/// happened before another. A topological sort's arbitrary tie-breaking
/// order is not itself causal evidence.
fn is_causal_ancestor(
    ancestor: &str,
    descendant: &str,
    events: &BTreeMap<String, EventRecord>,
) -> bool {
    let mut stack = vec![descendant.to_owned()];
    let mut visited = BTreeSet::new();
    while let Some(current) = stack.pop() {
        if current == ancestor {
            return true;
        }
        if let Some(event) = events.get(&current) {
            for parent in &event.parents {
                if visited.insert(parent.clone()) {
                    stack.push(parent.clone());
                }
            }
        }
    }
    false
}

fn parent_index(order: &[String]) -> BTreeMap<String, usize> {
    order
        .iter()
        .enumerate()
        .map(|(index, id)| (id.clone(), index))
        .collect()
}

/// Checks an independently captured Directory Fabric history.
///
/// This checker assumes the evidence-ingest layer has already authenticated
/// source records and established completeness flags. It verifies structural
/// bindings and the domain-specific owner/worker/barrier invariants. It does
/// not prove that the route inventory itself is complete or that a source
/// signature is authentic.
pub fn audit_history(evidence: &HistoryEvidence) -> AuditReport {
    let mut findings = Findings::default();
    let checked_events = evidence.events.len();

    if !evidence.event_set_complete {
        findings.inconclusive(
            FindingCode::MissingCausalParent,
            None,
            "evidence ingest has not established a complete event set",
        );
    }

    if evidence.events.is_empty() {
        findings.inconclusive(
            FindingCode::InventoryMissing,
            None,
            "empty history cannot establish a revocation barrier",
        );
        return findings.report(checked_events);
    }

    let mut events = BTreeMap::new();
    for event in &evidence.events {
        if event.id.trim().is_empty() {
            findings.violation(
                FindingCode::EmptyEventId,
                None,
                "event identifier is empty",
            );
            continue;
        }
        if events.insert(event.id.clone(), event.clone()).is_some() {
            findings.violation(
                FindingCode::DuplicateEventId,
                Some(&event.id),
                "event identifier is duplicated",
            );
        }
        if event_requires_owner_identity(&event.kind)
            && event.owner.as_deref().unwrap_or("").trim().is_empty()
        {
            findings.violation(
                FindingCode::MissingOwnerIdentity,
                Some(&event.id),
                "event kind requires an exact owner identity",
            );
        }
        if event_requires_owner_sequence(&event.kind) {
            match event.owner_sequence {
                None => findings.violation(
                    FindingCode::MissingOwnerSequence,
                    Some(&event.id),
                    "owner-local event is missing its durable sequence",
                ),
                Some(0) => findings.violation(
                    FindingCode::ZeroOwnerSequence,
                    Some(&event.id),
                    "owner-local sequence must be positive",
                ),
                Some(_) => {}
            }
        } else if event.owner_sequence.is_some() {
            findings.violation(
                FindingCode::UnexpectedOwnerSequence,
                Some(&event.id),
                "event is not an owner-local sequenced event but carries an owner sequence",
            );
        }
    }

    let Some(order) = topological_order(&events, &mut findings) else {
        return findings.report(checked_events);
    };
    let order_index = parent_index(&order);

    let mut owner_streams: BTreeMap<String, Vec<(u64, String)>> = BTreeMap::new();
    for event in events.values() {
        if let (Some(owner), Some(sequence)) = (&event.owner, event.owner_sequence) {
            owner_streams
                .entry(owner.clone())
                .or_default()
                .push((sequence, event.id.clone()));
        }
    }

    for (owner, stream) in owner_streams.iter_mut() {
        stream.sort_by_key(|(sequence, _)| *sequence);
        for pair in stream.windows(2) {
            if pair[0].0 == pair[1].0 {
                findings.violation(
                    FindingCode::DuplicateOwnerSequence,
                    Some(&pair[1].1),
                    format!("owner {owner} has two events at durable sequence {}", pair[1].0),
                );
            }
            if is_causal_ancestor(&pair[1].1, &pair[0].1, events) {
                findings.violation(
                    FindingCode::OwnerSequenceCausalOrderMismatch,
                    Some(&pair[1].1),
                    format!("owner {owner} causal graph puts a later durable sequence before an earlier one"),
                );
            }
        }
    }

    let inventory_events: BTreeMap<String, InventoryInfo> = events
        .iter()
        .filter_map(|(id, event)| match &event.kind {
            EventKind::InventoryClosed {
                revocation_digest,
                inventory_epoch,
                inventory_manifest_digest,
                route_set_digest,
                required_owners,
                required_routes,
            } => Some((
                id.clone(),
                InventoryInfo {
                    revocation_digest: *revocation_digest,
                    inventory_epoch: *inventory_epoch,
                    inventory_manifest_digest: *inventory_manifest_digest,
                    route_set_digest: *route_set_digest,
                    required_owners: required_owners.clone(),
                    required_routes: required_routes.clone(),
                },
            )),
            _ => None,
        })
        .collect();

    if inventory_events.is_empty() {
        findings.inconclusive(
            FindingCode::InventoryMissing,
            None,
            "no independently verified inventory-closure event is present",
        );
    }

    for (id, inventory) in &inventory_events {
        if inventory.required_owners.is_empty() {
            findings.violation(
                FindingCode::InventoryEmptyOwnerSet,
                Some(id),
                "strict revocation inventory has no required effect owners",
            );
        }
        for owner in &inventory.required_owners {
            if !evidence.complete_owner_streams.contains(owner) {
                findings.inconclusive(
                    FindingCode::InventoryOwnerStreamIncomplete,
                    Some(id),
                    format!("owner {owner} does not have a complete evidence stream"),
                );
            }
        }
        if !digest_is_nonzero(&inventory.revocation_digest)
            || !digest_is_nonzero(&inventory.inventory_manifest_digest)
            || !digest_is_nonzero(&inventory.route_set_digest)
            || inventory.inventory_epoch == 0
        {
            findings.violation(
                FindingCode::InvalidDigest,
                Some(id),
                "inventory closure contains a zero digest or zero epoch",
            );
        }
    }


    let mut fence_commits: BTreeMap<String, FenceInfo> = BTreeMap::new();
    let mut readbacks: BTreeMap<String, ReadBackInfo> = BTreeMap::new();
    let mut owner_ack_valid: BTreeMap<String, String> = BTreeMap::new();
    let mut effect_commits_by_operation: BTreeMap<String, String> = BTreeMap::new();
    let mut attempt_starts: BTreeMap<String, EventRecord> = BTreeMap::new();
    let mut attempts_by_operation: BTreeMap<String, BTreeMap<u32, String>> = BTreeMap::new();
    let mut unknown_attempts: BTreeMap<String, String> = BTreeMap::new();
    let mut reconciliations: BTreeMap<String, (String, ReconciledOutcome)> = BTreeMap::new();

    for event in events.values() {
        match &event.kind {
            EventKind::EffectAttemptStarted {
                operation_id,
                attempt_id,
                attempt_number,
                ..
            } => {
                if attempt_starts.insert(attempt_id.clone(), event.clone()).is_some() {
                    findings.violation(
                        FindingCode::DuplicateAttemptNumber,
                        Some(&event.id),
                        format!("physical attempt id {attempt_id} is reused"),
                    );
                }
                if attempts_by_operation
                    .entry(operation_id.clone())
                    .or_default()
                    .insert(*attempt_number, attempt_id.clone())
                    .is_some()
                {
                    findings.violation(
                        FindingCode::DuplicateAttemptNumber,
                        Some(&event.id),
                        format!("operation {operation_id} reuses physical attempt number {attempt_number}"),
                    );
                }
            }
            EventKind::AttemptOutcomeUnknown { attempt_id, .. } => {
                if unknown_attempts.insert(attempt_id.clone(), event.id.clone()).is_some() {
                    findings.violation(
                        FindingCode::ReconciliationBindingMismatch,
                        Some(&event.id),
                        format!("attempt {attempt_id} has more than one Unknown declaration"),
                    );
                }
            }
            EventKind::AttemptReconciled {
                attempt_id,
                unknown_event_id,
                outcome,
                ..
            } => {
                if reconciliations
                    .insert(
                        attempt_id.clone(),
                        (event.id.clone(), *outcome),
                    )
                    .is_some()
                {
                    findings.violation(
                        FindingCode::ReconciliationBindingMismatch,
                        Some(&event.id),
                        format!("attempt {attempt_id} has multiple reconciliation events"),
                    );
                }
                let Some(unknown) = events.get(unknown_event_id) else {
                    findings.inconclusive(
                        FindingCode::ReconciliationMissingUnknown,
                        Some(&event.id),
                        "reconciliation references a missing Unknown event",
                    );
                    continue;
                };
                if !has_parent(event, unknown_event_id) {
                    findings.violation(
                        FindingCode::ReconciliationBindingMismatch,
                        Some(&event.id),
                        "reconciliation must directly reference the exact Unknown event",
                    );
                }
                let same_attempt = match (&unknown.kind, &event.kind) {
                    (
                        EventKind::AttemptOutcomeUnknown {
                            operation_id: unknown_operation,
                            attempt_id: unknown_attempt,
                            attempt_number: unknown_number,
                        },
                        EventKind::AttemptReconciled {
                            operation_id,
                            attempt_id: reconciled_attempt,
                            attempt_number,
                            ..
                        },
                    ) => {
                        unknown_operation == operation_id
                            && unknown_attempt == reconciled_attempt
                            && unknown_number == attempt_number
                    }
                    _ => false,
                };
                if !same_attempt {
                    findings.violation(
                        FindingCode::ReconciliationBindingMismatch,
                        Some(&event.id),
                        "Unknown event does not identify the same operation and physical attempt",
                    );
                }
                match outcome {
                    ReconciledOutcome::ConfirmedNoEffect => {
                        if let EventKind::AttemptReconciled {
                            effect_commit_event_id: Some(_),
                            ..
                        } = &event.kind
                        {
                            findings.violation(
                                FindingCode::ReconciliationUnexpectedCommit,
                                Some(&event.id),
                                "ConfirmedNoEffect reconciliation cannot cite an effect commit",
                            );
                        }
                    }
                    ReconciledOutcome::Committed => {
                        if let EventKind::AttemptReconciled {
                            effect_commit_event_id: None,
                            ..
                        } = &event.kind
                        {
                            findings.inconclusive(
                                FindingCode::ReconciliationMissingCommit,
                                Some(&event.id),
                                "Committed reconciliation lacks the exact effect commit event",
                            );
                        }
                    }
                }
            }
            _ => {}
        }
    }

    for (operation_id, attempts) in &attempts_by_operation {
        let mut expected_number = 1u32;
        for (number, attempt_id) in attempts {
            if *number != expected_number {
                findings.inconclusive(
                    FindingCode::AttemptNumberGap,
                    attempt_starts.get(attempt_id).map(|event| event.id.as_str()),
                    format!("operation {operation_id} has a missing physical attempt number before {number}"),
                );
            }
            expected_number = number.saturating_add(1);
        }
    }



    for (attempt_id, unknown_event_id) in &unknown_attempts {
        let Some(start) = attempt_starts.get(attempt_id) else {
            findings.violation(
                FindingCode::ReconciliationBindingMismatch,
                Some(unknown_event_id),
                format!("Unknown event exists for attempt {attempt_id}, but no attempt-start event exists"),
            );
            continue;
        };
        let Some(unknown) = events.get(unknown_event_id) else {
            findings.inconclusive(
                FindingCode::ReconciliationMissingUnknown,
                Some(unknown_event_id),
                "Unknown event is missing from the indexed history",
            );
            continue;
        };
        if !has_parent(unknown, &start.id) || unknown.owner != start.owner {
            findings.violation(
                FindingCode::ReconciliationBindingMismatch,
                Some(unknown_event_id),
                "Unknown event does not causally bind to the exact owner/attempt-start event",
            );
        }
        if let (
            EventKind::EffectAttemptStarted {
                operation_id: start_operation,
                attempt_number: start_number,
                ..
            },
            EventKind::AttemptOutcomeUnknown {
                operation_id: unknown_operation,
                attempt_number: unknown_number,
                ..
            },
        ) = (&start.kind, &unknown.kind)
        {
            if start_operation != unknown_operation || start_number != unknown_number {
                findings.violation(
                    FindingCode::ReconciliationBindingMismatch,
                    Some(unknown_event_id),
                    "Unknown event changes the operation or physical attempt number",
                );
            }
        }
    }

    for (owner, stream) in &owner_streams {
        let mut current_position: Option<FencePosition> = None;
        let mut claim_epochs: BTreeMap<String, u64> = BTreeMap::new();
        let mut claims: BTreeMap<String, (u64, String, u64, String)> = BTreeMap::new();
        let mut bootstrap_seen = false;

        for (_, event_id) in stream {
            let event = events.get(event_id).expect("indexed owner event exists");
            match &event.kind {
                EventKind::OwnerBootstrapped {
                    position,
                    bootstrap_digest,
                } => {
                    if bootstrap_seen {
                        findings.violation(
                            FindingCode::DuplicateOwnerBootstrap,
                            Some(event_id),
                            format!("owner {owner} has more than one bootstrap event"),
                        );
                    }
                    bootstrap_seen = true;
                    if position.owner_epoch == 0
                        || position.generation == 0
                        || !digest_is_nonzero(bootstrap_digest)
                    {
                        findings.violation(
                            FindingCode::InvalidPosition,
                            Some(event_id),
                            "owner bootstrap has an invalid position or zero commitment",
                        );
                    }
                    current_position = Some(*position);
                }
                EventKind::WorkerClaimInstalled {
                    operation_id,
                    worker_id,
                    claim_epoch,
                } => {
                    let previous = claim_epochs.get(operation_id).copied().unwrap_or(0);
                    if *claim_epoch <= previous || *claim_epoch == 0 {
                        findings.violation(
                            FindingCode::ClaimEpochNotMonotonic,
                            Some(event_id),
                            format!("worker claim epoch for operation {operation_id} did not advance"),
                        );
                    } else {
                        claim_epochs.insert(operation_id.clone(), *claim_epoch);
                    }
                    claims.insert(
                        operation_id.clone(),
                        (
                            event.owner_sequence.unwrap_or(0),
                            worker_id.clone(),
                            *claim_epoch,
                            event_id.clone(),
                        ),
                    );
                }
                EventKind::FenceCommitted {
                    previous,
                    installed,
                    revocation_digest,
                    inventory_epoch,
                    inventory_manifest_digest,
                    route_set_digest,
                    owner_commit_digest,
                } => {
                    let Some(current) = current_position else {
                        findings.inconclusive(
                            FindingCode::MissingOwnerBootstrap,
                            Some(event_id),
                            format!("owner {owner} has no preceding bootstrap position"),
                        );
                        continue;
                    };
                    if *previous != current {
                        findings.violation(
                            FindingCode::FencePreviousPositionMismatch,
                            Some(event_id),
                            format!("fence previous position does not match owner {owner}'s current position"),
                        );
                    }
                    if installed <= previous {
                        findings.violation(
                            FindingCode::FencePositionNotMonotonic,
                            Some(event_id),
                            "installed fence position does not strictly advance",
                        );
                    }
                    if !digest_is_nonzero(revocation_digest)
                        || !digest_is_nonzero(inventory_manifest_digest)
                        || !digest_is_nonzero(route_set_digest)
                        || !digest_is_nonzero(owner_commit_digest)
                        || *inventory_epoch == 0
                    {
                        findings.violation(
                            FindingCode::InvalidDigest,
                            Some(event_id),
                            "fence commit has zero evidence digest or inventory epoch",
                        );
                    }
                    current_position = Some(*installed);
                    fence_commits.insert(
                        event_id.clone(),
                        FenceInfo {
                            owner: owner.clone(),
                            sequence: event.owner_sequence.unwrap_or(0),
                            installed: *installed,
                            revocation_digest: *revocation_digest,
                            inventory_epoch: *inventory_epoch,
                            inventory_manifest_digest: *inventory_manifest_digest,
                            route_set_digest: *route_set_digest,
                        },
                    );
                }
                EventKind::FenceReadBack {
                    fence_commit_event_id,
                    position,
                    durable_readback_digest,
                } => {
                    let Some(commit) = fence_commits.get(fence_commit_event_id) else {
                        findings.violation(
                            FindingCode::ReadBackMissingFenceCommit,
                            Some(event_id),
                            "read-back references no earlier fence commit",
                        );
                        continue;
                    };
                    if commit.owner != *owner {
                        findings.violation(
                            FindingCode::ReadBackOwnerMismatch,
                            Some(event_id),
                            "read-back owner differs from fence commit owner",
                        );
                    }
                    if commit.installed != *position {
                        findings.violation(
                            FindingCode::ReadBackPositionMismatch,
                            Some(event_id),
                            "read-back position differs from fence commit",
                        );
                    }
                    if event.owner_sequence.unwrap_or(0) <= commit.sequence {
                        findings.violation(
                            FindingCode::ReadBackBeforeFenceCommit,
                            Some(event_id),
                            "durable read-back sequence does not follow fence commit sequence",
                        );
                    }
                    if !has_parent(event, fence_commit_event_id) {
                        findings.violation(
                            FindingCode::ReadBackMissingCausalParent,
                            Some(event_id),
                            "read-back must directly reference its fence commit",
                        );
                    }
                    if !digest_is_nonzero(durable_readback_digest) {
                        findings.violation(
                            FindingCode::InvalidDigest,
                            Some(event_id),
                            "durable read-back digest is zero",
                        );
                    }
                    readbacks.insert(
                        event_id.clone(),
                        ReadBackInfo {
                            owner: owner.clone(),
                            sequence: event.owner_sequence.unwrap_or(0),
                            fence_commit_event_id: fence_commit_event_id.clone(),
                            position: *position,
                            durable_readback_digest: *durable_readback_digest,
                        },
                    );
                }
                EventKind::EffectCommitted {
                    operation_id,
                    attempt_id,
                    attempt_number,
                    worker_id,
                    claim_epoch,
                    position,
                    idempotency_digest,
                    effect_digest,
                } => {
                    let Some(attempt) = attempt_starts.get(attempt_id) else {
                        findings.violation(
                            FindingCode::EffectMissingAttempt,
                            Some(event_id),
                            format!("effect references unknown attempt {attempt_id}"),
                        );
                        continue;
                    };
                    let EventKind::EffectAttemptStarted {
                        operation_id: started_operation,
                        attempt_id: started_attempt,
                        attempt_number: started_number,
                        worker_id: started_worker,
                        claim_epoch: started_claim_epoch,
                        claim_event_id,
                        position: started_position,
                        idempotency_digest: started_idempotency,
                        ..
                    } = &attempt.kind
                    else {
                        unreachable!()
                    };
                    if started_operation != operation_id
                        || started_attempt != attempt_id
                        || started_number != attempt_number
                        || started_worker != worker_id
                        || started_claim_epoch != claim_epoch
                        || started_position != position
                        || started_idempotency != idempotency_digest
                    {
                        findings.violation(
                            FindingCode::EffectAttemptBindingMismatch,
                            Some(event_id),
                            "effect commit differs from its exact attempt binding",
                        );
                    }
                    if !has_parent(event, &attempt.id) {
                        findings.violation(
                            FindingCode::EffectMissingCausalParent,
                            Some(event_id),
                            "effect commit must directly reference the attempt-start event",
                        );
                    }
                    let attempt_index = order_index.get(&attempt.id).copied().unwrap_or(usize::MAX);
                    let commit_index = order_index.get(event_id).copied().unwrap_or(0);
                    if attempt_index >= commit_index {
                        findings.violation(
                            FindingCode::EffectMissingCausalParent,
                            Some(event_id),
                            "attempt-start does not precede effect commit in the causal graph",
                        );
                    }
                    if !has_parent(attempt, claim_event_id) {
                        findings.violation(
                            FindingCode::EffectClaimNotCurrent,
                            Some(&attempt.id),
                            "attempt-start must reference the exact worker-claim event",
                        );
                    }
                    if let Some(current_claim) = claims.get(operation_id) {
                        if current_claim.0 < event.owner_sequence.unwrap_or(0)
                            && (current_claim.1 != *worker_id
                                || current_claim.2 != *claim_epoch
                                || current_claim.3 != *claim_event_id)
                        {
                            findings.violation(
                                FindingCode::EffectClaimNotCurrent,
                                Some(event_id),
                                "worker identity or claim epoch is stale at owner commit sequence",
                            );
                        }
                    } else {
                        findings.violation(
                            FindingCode::EffectClaimNotCurrent,
                            Some(event_id),
                            "no worker claim exists in the complete owner stream before this effect",
                        );
                    }
                    if let Some(current) = current_position {
                        if *position != current {
                            findings.violation(
                                FindingCode::EffectPositionNotCurrent,
                                Some(event_id),
                                "effect commit did not use the owner position current at its linearization point",
                            );
                        }
                    } else {
                        findings.inconclusive(
                            FindingCode::MissingOwnerBootstrap,
                            Some(event_id),
                            "effect owner has no verified bootstrap position",
                        );
                    }
                    if !digest_is_nonzero(idempotency_digest) || !digest_is_nonzero(effect_digest) {
                        findings.violation(
                            FindingCode::InvalidDigest,
                            Some(event_id),
                            "effect commit contains a zero idempotency or effect digest",
                        );
                    }
                    if effect_commits_by_operation
                        .insert(operation_id.clone(), event_id.clone())
                        .is_some()
                    {
                        findings.violation(
                            FindingCode::DuplicateLogicalEffect,
                            Some(event_id),
                            format!("logical operation {operation_id} committed more than once"),
                        );
                    }
                }
                _ => {}
            }
        }

        if !bootstrap_seen {
            findings.inconclusive(
                FindingCode::MissingOwnerBootstrap,
                None,
                format!("complete stream for owner {owner} has no bootstrap event"),
            );
        }
    }

    for event in events.values() {
        match &event.kind {
            EventKind::OwnerAcknowledged {
                fence_commit_event_id,
                readback_event_id,
                revocation_digest,
                inventory_epoch,
                inventory_manifest_digest,
                route_set_digest,
            } => {
                let Some(commit) = fence_commits.get(fence_commit_event_id) else {
                    findings.violation(
                        FindingCode::OwnerAckMissingFenceCommit,
                        Some(&event.id),
                        "owner acknowledgement references no verified fence commit",
                    );
                    continue;
                };
                let Some(readback) = readbacks.get(readback_event_id) else {
                    findings.violation(
                        FindingCode::OwnerAckMissingReadBack,
                        Some(&event.id),
                        "owner acknowledgement references no verified durable read-back",
                    );
                    continue;
                };
                if event.owner.as_deref() != Some(commit.owner.as_str())
                    || readback.owner != commit.owner
                    || readback.fence_commit_event_id != *fence_commit_event_id
                    || readback.position != commit.installed
                    || !digest_is_nonzero(&readback.durable_readback_digest)
                    || readback.sequence <= commit.sequence
                {
                    findings.violation(
                        FindingCode::OwnerAckBindingMismatch,
                        Some(&event.id),
                        "owner acknowledgement, fence commit and read-back are not the same owner/position",
                    );
                }
                if *revocation_digest != commit.revocation_digest
                    || *inventory_epoch != commit.inventory_epoch
                    || *inventory_manifest_digest != commit.inventory_manifest_digest
                    || *route_set_digest != commit.route_set_digest
                {
                    findings.violation(
                        FindingCode::OwnerAckBindingMismatch,
                        Some(&event.id),
                        "owner acknowledgement changes revocation or inventory bindings",
                    );
                }
                if !has_parent(event, fence_commit_event_id)
                    || !has_parent(event, readback_event_id)
                {
                    findings.violation(
                        FindingCode::OwnerAckMissingCausalParent,
                        Some(&event.id),
                        "owner acknowledgement must directly bind both fence commit and read-back",
                    );
                }
                owner_ack_valid.insert(event.id.clone(), commit.owner.clone());
            }
            EventKind::GlobalBarrierAcknowledged {
                inventory_event_id,
                revocation_digest,
                inventory_epoch,
                inventory_manifest_digest,
                route_set_digest,
                owner_ack_event_ids,
                coordinator_epoch,
                coordinator_commit_digest,
            } => {
                let Some(inventory) = inventory_events.get(inventory_event_id) else {
                    findings.violation(
                        FindingCode::GlobalBarrierMissingInventory,
                        Some(&event.id),
                        "global acknowledgement references no inventory closure",
                    );
                    continue;
                };
                if !has_parent(event, inventory_event_id) {
                    findings.violation(
                        FindingCode::GlobalBarrierMissingCausalParent,
                        Some(&event.id),
                        "global acknowledgement must directly reference its inventory closure",
                    );
                }
                if *revocation_digest != inventory.revocation_digest
                    || *inventory_epoch != inventory.inventory_epoch
                    || *inventory_manifest_digest != inventory.inventory_manifest_digest
                    || *route_set_digest != inventory.route_set_digest
                    || *coordinator_epoch == 0
                    || !digest_is_nonzero(coordinator_commit_digest)
                {
                    findings.violation(
                        FindingCode::GlobalBarrierAckBindingMismatch,
                        Some(&event.id),
                        "global acknowledgement changes inventory binding or lacks coordinator commit evidence",
                    );
                }
                if owner_ack_event_ids.len() != inventory.required_owners.len() {
                    findings.violation(
                        FindingCode::GlobalBarrierOwnerSetMismatch,
                        Some(&event.id),
                        "owner acknowledgement count differs from the frozen owner set",
                    );
                }
                let mut acknowledged_owners = BTreeSet::new();
                for ack_id in owner_ack_event_ids {
                    if !has_parent(event, ack_id) {
                        findings.violation(
                            FindingCode::GlobalBarrierMissingCausalParent,
                            Some(&event.id),
                            format!("global acknowledgement does not directly reference owner ack {ack_id}"),
                        );
                    }
                    let Some(owner) = owner_ack_valid.get(ack_id) else {
                        findings.violation(
                            FindingCode::GlobalBarrierMissingOwnerAck,
                            Some(&event.id),
                            format!("owner acknowledgement {ack_id} is absent or invalid"),
                        );
                        continue;
                    };
                    if !acknowledged_owners.insert(owner.clone()) {
                        findings.violation(
                            FindingCode::GlobalBarrierOwnerSetMismatch,
                            Some(&event.id),
                            format!("owner {owner} appears more than once in the owner acknowledgement set"),
                        );
                    }
                }
                if acknowledged_owners != inventory.required_owners {
                    findings.violation(
                        FindingCode::GlobalBarrierOwnerSetMismatch,
                        Some(&event.id),
                        "acknowledged owner identities do not exactly equal the frozen inventory owner set",
                    );
                }
            }
            _ => {}
        }
    }

    for event in events.values() {
        match &event.kind {
            EventKind::EffectAttemptStarted {
                operation_id,
                attempt_id,
                attempt_number,
                worker_id,
                claim_epoch,
                claim_event_id,
                retry_after_outcome_event_id,
                ..
            } => {
                let Some(claim) = events.get(claim_event_id) else {
                    findings.inconclusive(
                        FindingCode::EffectClaimNotCurrent,
                        Some(&event.id),
                        "attempt references a worker claim event absent from the evidence bundle",
                    );
                    continue;
                };
                match &claim.kind {
                    EventKind::WorkerClaimInstalled {
                        operation_id: claim_operation,
                        worker_id: claim_worker,
                        claim_epoch: installed_epoch,
                    } if claim_operation == operation_id
                        && claim_worker == worker_id
                        && installed_epoch == claim_epoch
                        && claim.owner == event.owner
                        && has_parent(event, claim_event_id) => {}
                    _ => findings.violation(
                        FindingCode::EffectClaimNotCurrent,
                        Some(&event.id),
                        "attempt does not bind an exact worker claim for its operation, worker and epoch",
                    ),
                }

                if *attempt_number == 1 {
                    if retry_after_outcome_event_id.is_some() {
                        findings.violation(
                            FindingCode::RetryWithoutNoEffectReconciliation,
                            Some(&event.id),
                            "first physical attempt unexpectedly carries retry evidence",
                        );
                    }
                } else {
                    let Some(outcome_id) = retry_after_outcome_event_id else {
                        findings.violation(
                            FindingCode::RetryWithoutNoEffectReconciliation,
                            Some(&event.id),
                            "retry lacks exact prior-attempt no-effect reconciliation",
                        );
                        continue;
                    };
                    if !has_parent(event, outcome_id) {
                        findings.violation(
                            FindingCode::RetryWithoutNoEffectReconciliation,
                            Some(&event.id),
                            "retry must directly reference its prior-attempt outcome evidence",
                        );
                    }
                    let previous_attempt_id = attempts_by_operation
                        .get(operation_id)
                        .and_then(|attempts| attempts.get(&attempt_number.saturating_sub(1)));
                    match (
                        previous_attempt_id,
                        events.get(outcome_id).map(|e| &e.kind),
                    ) {
                        (
                            Some(expected_previous_id),
                            Some(EventKind::AttemptReconciled {
                                operation_id: prior_operation,
                                attempt_id: prior_attempt_id,
                                attempt_number: prior_number,
                                outcome: ReconciledOutcome::ConfirmedNoEffect,
                                ..
                            }),
                        ) if prior_operation == operation_id
                            && prior_attempt_id == expected_previous_id
                            && *prior_number + 1 == *attempt_number => {}
                        _ => findings.violation(
                            FindingCode::RetryWithoutNoEffectReconciliation,
                            Some(&event.id),
                            "retry is not preceded by an exact ConfirmedNoEffect reconciliation of the immediately prior physical attempt",
                        ),
                    }
                }

                if let Some(unknown_id) = unknown_attempts.get(attempt_id) {
                    if !reconciliations.contains_key(attempt_id) {
                        findings.inconclusive(
                            FindingCode::UnknownAttemptUnreconciled,
                            Some(unknown_id),
                            "physical attempt outcome remains Unknown",
                        );
                    }
                }
            }
            EventKind::AttemptReconciled {
                operation_id,
                attempt_id,
                attempt_number,
                outcome,
                effect_commit_event_id,
                ..
            } => match outcome {
                ReconciledOutcome::ConfirmedNoEffect => {
                    if effect_commit_event_id.is_some()
                        || events.values().any(|candidate| matches!(
                            &candidate.kind,
                            EventKind::EffectCommitted { attempt_id: committed_attempt, .. }
                                if committed_attempt == attempt_id
                        ))
                    {
                        findings.violation(
                            FindingCode::ReconciliationUnexpectedCommit,
                            Some(&event.id),
                            "ConfirmedNoEffect conflicts with an effect commit for the same attempt",
                        );
                    }
                }
                ReconciledOutcome::Committed => {
                    let Some(commit_id) = effect_commit_event_id else {
                        findings.inconclusive(
                            FindingCode::ReconciliationMissingCommit,
                            Some(&event.id),
                            "Committed reconciliation lacks an exact effect commit event",
                        );
                        continue;
                    };
                    match events.get(commit_id).map(|e| &e.kind) {
                        Some(EventKind::EffectCommitted {
                            operation_id: committed_operation,
                            attempt_id: committed_attempt,
                            attempt_number: committed_number,
                            ..
                        }) if committed_operation == operation_id
                            && committed_attempt == attempt_id
                            && committed_number == attempt_number
                            && has_parent(event, commit_id) => {}
                        _ => findings.violation(
                            FindingCode::ReconciliationBindingMismatch,
                            Some(&event.id),
                            "Committed reconciliation does not bind the exact operation/attempt owner commit",
                        ),
                    }
                }
            },
            _ => {}
        }
    }

    for event in events.values() {
        if matches!(&event.kind, EventKind::GlobalBarrierAcknowledged { .. }) {
            // Every declared inventory owner must have a complete owner stream.
            if let EventKind::GlobalBarrierAcknowledged {
                inventory_event_id, ..
            } = &event.kind
            {
                if let Some(inventory) = inventory_events.get(inventory_event_id) {
                    for owner in &inventory.required_owners {
                        if !evidence.complete_owner_streams.contains(owner) {
                            findings.inconclusive(
                                FindingCode::InventoryOwnerStreamIncomplete,
                                Some(&event.id),
                                format!("global acknowledgement lacks complete owner stream for {owner}"),
                            );
                        }
                    }
                }
            }
        }

        if let EventKind::RouteActivated {
            route_id,
            owner,
            position,
            staged_event_id,
            inventory_event_id,
            owner_readback_event_id,
            global_barrier_event_id,
        } = &event.kind
        {
            let Some(staged) = events.get(staged_event_id) else {
                findings.violation(
                    FindingCode::RouteMissingStage,
                    Some(&event.id),
                    format!("route {route_id} activation references no stage event"),
                );
                continue;
            };
            let Some(inventory) = inventory_events.get(inventory_event_id) else {
                findings.violation(
                    FindingCode::RouteMissingInventory,
                    Some(&event.id),
                    format!("route {route_id} activation references no current inventory closure"),
                );
                continue;
            };
            let Some(readback) = readbacks.get(owner_readback_event_id) else {
                findings.violation(
                    FindingCode::RouteMissingReadBack,
                    Some(&event.id),
                    format!("route {route_id} activation references no durable owner read-back"),
                );
                continue;
            };
            let Some(barrier) = events.get(global_barrier_event_id) else {
                findings.violation(
                    FindingCode::RouteMissingGlobalBarrier,
                    Some(&event.id),
                    format!("route {route_id} activation references no prior global barrier"),
                );
                continue;
            };
            let staged_info = match &staged.kind {
                EventKind::RouteStaged {
                    route_id: staged_route,
                    owner: staged_owner,
                    enforcement,
                    required_position,
                    inventory_event_id: staged_inventory,
                } => Some((
                    staged_route,
                    staged_owner,
                    enforcement,
                    required_position,
                    staged_inventory,
                )),
                _ => None,
            };
            let Some((staged_route, staged_owner, enforcement, required, staged_inventory)) =
                staged_info
            else {
                findings.violation(
                    FindingCode::RouteMissingStage,
                    Some(&event.id),
                    "staged event reference has the wrong event kind",
                );
                continue;
            };
            if *staged_route != *route_id
                || *staged_owner != *owner
                || *staged_inventory != *inventory_event_id
                || *required != *position
                || !has_parent(event, staged_event_id)
            {
                findings.violation(
                    FindingCode::RouteAdmissionBindingMismatch,
                    Some(&event.id),
                    "route activation differs from its staged identity, owner, position or inventory",
                );
            }
            if *enforcement != RouteEnforcement::AtomicOwnerFence {
                findings.violation(
                    FindingCode::RouteNonFenceable,
                    Some(&event.id),
                    "non-fenceable route was activated into strict execution",
                );
            }
            let readback_fence_matches_barrier = match (
                fence_commits.get(&readback.fence_commit_event_id),
                &barrier.kind,
            ) {
                (
                    Some(commit),
                    EventKind::GlobalBarrierAcknowledged {
                        revocation_digest, ..
                    },
                ) => commit.installed == *position
                    && commit.revocation_digest == *revocation_digest,
                _ => false,
            };
            if event.owner.as_deref() != Some(owner.as_str())
                || !inventory.required_owners.contains(owner)
                || !inventory.required_routes.contains(route_id)
                || !readback_fence_matches_barrier
                || readback.owner != *owner
                || readback.position != *position
                || !has_parent(event, inventory_event_id)
                || !has_parent(event, owner_readback_event_id)
                || !has_parent(event, global_barrier_event_id)
            {
                findings.violation(
                    FindingCode::RouteAdmissionBindingMismatch,
                    Some(&event.id),
                    "route activation is not bound to the current inventory, owner read-back and global barrier",
                );
            }
            if !matches!(&barrier.kind, EventKind::GlobalBarrierAcknowledged { .. }) {
                findings.violation(
                    FindingCode::RouteMissingGlobalBarrier,
                    Some(&event.id),
                    "route activation parent is not a global barrier acknowledgement",
                );
            }
            if order_index.get(global_barrier_event_id) >= order_index.get(&event.id) {
                findings.violation(
                    FindingCode::RouteMissingGlobalBarrier,
                    Some(&event.id),
                    "route activation does not causally follow the global barrier",
                );
            }
        }

        if let EventKind::RouteStaged {
            route_id,
            owner,
            enforcement: _,
            required_position,
            inventory_event_id,
        } = &event.kind
        {
            if let Some(inventory) = inventory_events.get(inventory_event_id) {
                if event.owner.as_deref() != Some(owner.as_str())
                    || !inventory.required_owners.contains(owner)
                    || !inventory.required_routes.contains(route_id)
                {
                    findings.violation(
                        FindingCode::RouteAdmissionBindingMismatch,
                        Some(&event.id),
                        format!("staged route {route_id} or its owner {owner} is absent from the current inventory"),
                    );
                }
                if !has_parent(event, inventory_event_id) {
                    findings.violation(
                        FindingCode::RouteMissingCausalParent,
                        Some(&event.id),
                        "route staging must directly reference its inventory closure",
                    );
                }
                if required_position.owner_epoch == 0 || required_position.generation == 0 {
                    findings.violation(
                        FindingCode::InvalidPosition,
                        Some(&event.id),
                        "staged route has a zero owner epoch or generation",
                    );
                }
            }
        }
    }

    findings.report(checked_events)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(byte: u8) -> Digest {
        [byte; 32]
    }

    fn pos(epoch: u64, generation: u64) -> FencePosition {
        FencePosition::new(epoch, generation).unwrap()
    }

    fn record(
        id: &str,
        parents: &[&str],
        owner: Option<&str>,
        sequence: Option<u64>,
        kind: EventKind,
    ) -> EventRecord {
        EventRecord {
            id: id.to_owned(),
            parents: parents.iter().map(|s| (*s).to_owned()).collect(),
            owner: owner.map(str::to_owned),
            owner_sequence: sequence,
            kind,
        }
    }

    fn base_history(effect_after_fence: bool) -> HistoryEvidence {
        let owners = ["owner-a", "owner-b"]
            .into_iter()
            .map(str::to_owned)
            .collect::<BTreeSet<_>>();
        let mut events = vec![
            record(
                "inventory",
                &[],
                None,
                None,
                EventKind::InventoryClosed {
                    revocation_digest: digest(3),
                    inventory_epoch: 7,
                    inventory_manifest_digest: digest(4),
                    route_set_digest: digest(5),
                    required_owners: owners.clone(),
                    required_routes: BTreeSet::new(),
                },
            ),
            record(
                "bootstrap-a",
                &[],
                Some("owner-a"),
                Some(1),
                EventKind::OwnerBootstrapped {
                    position: pos(1, 1),
                    bootstrap_digest: digest(6),
                },
            ),
            record(
                "claim-a",
                &["bootstrap-a"],
                Some("owner-a"),
                Some(2),
                EventKind::WorkerClaimInstalled {
                    operation_id: "op-a".into(),
                    worker_id: "worker-a".into(),
                    claim_epoch: 1,
                },
            ),
            record(
                "attempt-a",
                &["claim-a"],
                Some("owner-a"),
                None,
                EventKind::EffectAttemptStarted {
                    operation_id: "op-a".into(),
                    attempt_id: "attempt-a".into(),
                    attempt_number: 1,
                    worker_id: "worker-a".into(),
                    claim_epoch: 1,
                    claim_event_id: "claim-a".into(),
                    position: pos(1, 1),
                    idempotency_digest: digest(7),
                    retry_after_outcome_event_id: None,
                },
            ),
            record(
                "bootstrap-b",
                &[],
                Some("owner-b"),
                Some(1),
                EventKind::OwnerBootstrapped {
                    position: pos(1, 1),
                    bootstrap_digest: digest(8),
                },
            ),
        ];

        let (effect_sequence, fence_sequence, readback_sequence) = if effect_after_fence {
            (5, 3, 4)
        } else {
            (3, 4, 5)
        };

        if effect_after_fence {
            events.push(record(
                "fence-a",
                &["claim-a"],
                Some("owner-a"),
                Some(fence_sequence),
                EventKind::FenceCommitted {
                    previous: pos(1, 1),
                    installed: pos(1, 2),
                    revocation_digest: digest(3),
                    inventory_epoch: 7,
                    inventory_manifest_digest: digest(4),
                    route_set_digest: digest(5),
                    owner_commit_digest: digest(9),
                },
            ));
        }
        events.push(record(
            "effect-a",
            &["attempt-a"],
            Some("owner-a"),
            Some(effect_sequence),
            EventKind::EffectCommitted {
                operation_id: "op-a".into(),
                attempt_id: "attempt-a".into(),
                attempt_number: 1,
                worker_id: "worker-a".into(),
                claim_epoch: 1,
                position: if effect_after_fence { pos(1, 1) } else { pos(1, 1) },
                idempotency_digest: digest(7),
                effect_digest: digest(10),
            },
        ));
        if !effect_after_fence {
            events.push(record(
                "fence-a",
                &["effect-a"],
                Some("owner-a"),
                Some(fence_sequence),
                EventKind::FenceCommitted {
                    previous: pos(1, 1),
                    installed: pos(1, 2),
                    revocation_digest: digest(3),
                    inventory_epoch: 7,
                    inventory_manifest_digest: digest(4),
                    route_set_digest: digest(5),
                    owner_commit_digest: digest(9),
                },
            ));
        }
        events.extend([
            record(
                "readback-a",
                &["fence-a"],
                Some("owner-a"),
                Some(readback_sequence),
                EventKind::FenceReadBack {
                    fence_commit_event_id: "fence-a".into(),
                    position: pos(1, 2),
                    durable_readback_digest: digest(11),
                },
            ),
            record(
                "fence-b",
                &["bootstrap-b", "inventory"],
                Some("owner-b"),
                Some(2),
                EventKind::FenceCommitted {
                    previous: pos(1, 1),
                    installed: pos(1, 2),
                    revocation_digest: digest(3),
                    inventory_epoch: 7,
                    inventory_manifest_digest: digest(4),
                    route_set_digest: digest(5),
                    owner_commit_digest: digest(12),
                },
            ),
            record(
                "readback-b",
                &["fence-b"],
                Some("owner-b"),
                Some(3),
                EventKind::FenceReadBack {
                    fence_commit_event_id: "fence-b".into(),
                    position: pos(1, 2),
                    durable_readback_digest: digest(13),
                },
            ),
            record(
                "ack-a",
                &["fence-a", "readback-a"],
                Some("owner-a"),
                None,
                EventKind::OwnerAcknowledged {
                    fence_commit_event_id: "fence-a".into(),
                    readback_event_id: "readback-a".into(),
                    revocation_digest: digest(3),
                    inventory_epoch: 7,
                    inventory_manifest_digest: digest(4),
                    route_set_digest: digest(5),
                },
            ),
            record(
                "ack-b",
                &["fence-b", "readback-b"],
                Some("owner-b"),
                None,
                EventKind::OwnerAcknowledged {
                    fence_commit_event_id: "fence-b".into(),
                    readback_event_id: "readback-b".into(),
                    revocation_digest: digest(3),
                    inventory_epoch: 7,
                    inventory_manifest_digest: digest(4),
                    route_set_digest: digest(5),
                },
            ),
            record(
                "global-ack",
                &["inventory", "ack-a", "ack-b"],
                None,
                None,
                EventKind::GlobalBarrierAcknowledged {
                    inventory_event_id: "inventory".into(),
                    revocation_digest: digest(3),
                    inventory_epoch: 7,
                    inventory_manifest_digest: digest(4),
                    route_set_digest: digest(5),
                    owner_ack_event_ids: ["ack-a", "ack-b"]
                        .into_iter()
                        .map(str::to_owned)
                        .collect(),
                    coordinator_epoch: 1,
                    coordinator_commit_digest: digest(14),
                },
            ),
        ]);

        HistoryEvidence {
            event_set_complete: true,
            complete_owner_streams: owners,
            events,
        }
    }

    fn add_post_barrier_route(history: &mut HistoryEvidence, enforcement: RouteEnforcement) {
        let owners = ["owner-a"]
            .into_iter()
            .map(str::to_owned)
            .collect::<BTreeSet<_>>();
        let routes = ["worker-route-a"]
            .into_iter()
            .map(str::to_owned)
            .collect::<BTreeSet<_>>();
        history.events.extend([
            record(
                "route-inventory",
                &["global-ack"],
                None,
                None,
                EventKind::InventoryClosed {
                    revocation_digest: digest(3),
                    inventory_epoch: 8,
                    inventory_manifest_digest: digest(15),
                    route_set_digest: digest(16),
                    required_owners: owners,
                    required_routes: routes,
                },
            ),
            record(
                "route-stage",
                &["global-ack", "route-inventory"],
                Some("owner-a"),
                None,
                EventKind::RouteStaged {
                    route_id: "worker-route-a".into(),
                    owner: "owner-a".into(),
                    enforcement,
                    required_position: pos(1, 2),
                    inventory_event_id: "route-inventory".into(),
                },
            ),
            record(
                "route-active",
                &["route-stage", "route-inventory", "readback-a", "global-ack"],
                Some("owner-a"),
                None,
                EventKind::RouteActivated {
                    route_id: "worker-route-a".into(),
                    owner: "owner-a".into(),
                    position: pos(1, 2),
                    staged_event_id: "route-stage".into(),
                    inventory_event_id: "route-inventory".into(),
                    owner_readback_event_id: "readback-a".into(),
                    global_barrier_event_id: "global-ack".into(),
                },
            ),
        ]);
    }

    #[test]
    fn post_barrier_route_requires_current_inventory_and_owner_readback() {
        let mut history = base_history(false);
        add_post_barrier_route(&mut history, RouteEnforcement::AtomicOwnerFence);
        assert_eq!(audit_history(&history).verdict, Verdict::Valid);
    }

    #[test]
    fn non_fenceable_route_cannot_be_activated() {
        let mut history = base_history(false);
        add_post_barrier_route(&mut history, RouteEnforcement::BestEffortInvalidation);
        let report = audit_history(&history);
        assert_eq!(report.verdict, Verdict::Violation);
        assert!(report.findings
            .iter()
            .any(|finding| finding.code == FindingCode::RouteNonFenceable));
    }

    #[test]
    fn superseded_worker_cannot_commit_after_new_claim_install() {
        let mut history = base_history(false);
        for event in &mut history.events {
            match event.id.as_str() {
                "effect-a" => event.owner_sequence = Some(5),
                "fence-a" => event.owner_sequence = Some(6),
                "readback-a" => event.owner_sequence = Some(7),
                _ => {}
            }
        }
        history.events.push(record(
            "claim-b",
            &["claim-a", "attempt-a"],
            Some("owner-a"),
            Some(4),
            EventKind::WorkerClaimInstalled {
                operation_id: "op-a".into(),
                worker_id: "worker-b".into(),
                claim_epoch: 2,
            },
        ));
        let report = audit_history(&history);
        assert_eq!(report.verdict, Verdict::Violation);
        assert!(report.findings
            .iter()
            .any(|finding| finding.code == FindingCode::EffectClaimNotCurrent));
    }

    #[test]
    fn owner_sequence_must_not_contradict_causal_order() {
        let mut history = base_history(false);
        let effect = history.events.iter_mut().find(|event| event.id == "effect-a").unwrap();
        effect.parents.push("fence-a".into());
        let fence = history.events.iter_mut().find(|event| event.id == "fence-a").unwrap();
        fence.parents.retain(|parent| parent != "effect-a");
        let report = audit_history(&history);
        assert_eq!(report.verdict, Verdict::Violation);
        assert!(report.findings
            .iter()
            .any(|finding| finding.code == FindingCode::OwnerSequenceCausalOrderMismatch));
    }

    #[test]
    fn retry_while_previous_outcome_is_unknown_is_rejected() {
        let mut history = base_history(false);
        history.events.retain(|event| event.id != "effect-a");
        let fence = history.events.iter_mut().find(|event| event.id == "fence-a").unwrap();
        fence.parents.retain(|parent| parent != "effect-a");
        history.events.push(record(
            "unknown-a",
            &["attempt-a"],
            Some("owner-a"),
            None,
            EventKind::AttemptOutcomeUnknown {
                operation_id: "op-a".into(),
                attempt_id: "attempt-a".into(),
                attempt_number: 1,
            },
        ));
        history.events.push(record(
            "claim-b",
            &["claim-a", "unknown-a"],
            Some("owner-a"),
            Some(6),
            EventKind::WorkerClaimInstalled {
                operation_id: "op-a".into(),
                worker_id: "worker-b".into(),
                claim_epoch: 2,
            },
        ));
        history.events.push(record(
            "attempt-b",
            &["claim-b", "unknown-a", "global-ack"],
            Some("owner-a"),
            None,
            EventKind::EffectAttemptStarted {
                operation_id: "op-a".into(),
                attempt_id: "attempt-b".into(),
                attempt_number: 2,
                worker_id: "worker-b".into(),
                claim_epoch: 2,
                claim_event_id: "claim-b".into(),
                position: pos(1, 2),
                idempotency_digest: digest(7),
                retry_after_outcome_event_id: None,
            },
        ));
        let report = audit_history(&history);
        assert_eq!(report.verdict, Verdict::Violation);
        assert!(report.findings
            .iter()
            .any(|finding| finding.code == FindingCode::RetryWithoutNoEffectReconciliation));
    }

    #[test]
    fn accepts_effect_that_committed_before_owner_fence() {
        assert_eq!(audit_history(&base_history(false)).verdict, Verdict::Valid);
    }

    #[test]
    fn rejects_old_generation_effect_after_owner_fence_commit() {
        let report = audit_history(&base_history(true));
        assert_eq!(report.verdict, Verdict::Violation);
        assert!(report
            .findings
            .iter()
            .any(|f| f.code == FindingCode::EffectPositionNotCurrent));
    }

    #[test]
    fn missing_owner_stream_is_inconclusive_not_valid() {
        let mut history = base_history(false);
        history.complete_owner_streams.remove("owner-b");
        let report = audit_history(&history);
        assert_eq!(report.verdict, Verdict::Inconclusive);
        assert!(report.findings
            .iter()
            .any(|f| f.code == FindingCode::InventoryOwnerStreamIncomplete));
    }

    #[test]
    fn duplicate_event_id_is_a_violation() {
        let mut history = base_history(false);
        history.events.push(history.events[0].clone());
        let report = audit_history(&history);
        assert_eq!(report.verdict, Verdict::Violation);
        assert!(report.findings
            .iter()
            .any(|f| f.code == FindingCode::DuplicateEventId));
    }

    #[test]
    fn missing_causal_parent_is_inconclusive() {
        let mut history = base_history(false);
        history.events[0].parents.push("missing".into());
        let report = audit_history(&history);
        assert_eq!(report.verdict, Verdict::Inconclusive);
        assert!(report.findings
            .iter()
            .any(|f| f.code == FindingCode::MissingCausalParent));
    }

    #[test]
    fn duplicate_logical_effect_is_a_violation() {
        let mut history = base_history(false);
        let mut duplicate = history
            .events
            .iter()
            .find(|event| event.id == "effect-a")
            .unwrap()
            .clone();
        duplicate.id = "effect-a-duplicate".into();
        duplicate.owner_sequence = Some(6);
        duplicate.parents = vec!["attempt-a".into()];
        history.events.push(duplicate);
        let report = audit_history(&history);
        assert_eq!(report.verdict, Verdict::Violation);
        assert!(report.findings
            .iter()
            .any(|f| f.code == FindingCode::DuplicateLogicalEffect));
    }

    #[test]
    fn a_global_acknowledgement_cannot_omit_an_owner() {
        let mut history = base_history(false);
        let global = history
            .events
            .iter_mut()
            .find(|event| event.id == "global-ack")
            .unwrap();
        global.parents.retain(|parent| parent != "ack-b");
        if let EventKind::GlobalBarrierAcknowledged {
            owner_ack_event_ids, ..
        } = &mut global.kind
        {
            owner_ack_event_ids.remove("ack-b");
        }
        let report = audit_history(&history);
        assert_eq!(report.verdict, Verdict::Violation);
        assert!(report.findings
            .iter()
            .any(|f| f.code == FindingCode::GlobalBarrierOwnerSetMismatch));
    }

    #[test]
    fn unresolved_unknown_attempt_is_inconclusive() {
        let mut history = base_history(false);
        history.events.push(record(
            "unknown-a",
            &["attempt-a"],
            Some("owner-a"),
            None,
            EventKind::AttemptOutcomeUnknown {
                operation_id: "op-a".into(),
                attempt_id: "attempt-a".into(),
                attempt_number: 1,
            },
        ));
        let report = audit_history(&history);
        assert_eq!(report.verdict, Verdict::Inconclusive);
        assert!(report.findings
            .iter()
            .any(|f| f.code == FindingCode::UnknownAttemptUnreconciled));
    }
}
