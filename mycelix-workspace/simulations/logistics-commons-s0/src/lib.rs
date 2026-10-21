pub mod partition;
pub mod scenario;

use std::collections::{BTreeMap, BTreeSet};

/// A point-in-time stock claim. It is valid only inside its stated observation window.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct InventorySnapshot {
    pub hub_id: String,
    pub sku_id: String,
    pub on_hand: u64,
    pub reserved: u64,
    pub revision: u64,
    pub observed_at_ms: u64,
    pub valid_until_ms: u64,
}

impl InventorySnapshot {
    pub fn available(&self) -> Option<u64> {
        self.on_hand.checked_sub(self.reserved)
    }
}

/// A request to reserve stock against an exact observed inventory revision.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ReservationIntent {
    pub reservation_id: String,
    pub participant_id: String,
    pub hub_id: String,
    pub sku_id: String,
    pub quantity: u64,
    pub expected_revision: u64,
    pub idempotency_key: String,
    pub observed_at_ms: u64,
    pub lease_until_ms: u64,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ReservationOutcome {
    Confirmed,
    Rejected { reason: &'static str },
    UnresolvedConflict { contenders: Vec<String> },
    DuplicateOf { primary_id: String },
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ReservationDecision {
    pub reservation_id: String,
    pub outcome: ReservationOutcome,
}

impl ReservationOutcome {
    fn canonical_token(&self) -> String {
        match self {
            Self::Confirmed => "CONFIRMED".to_owned(),
            Self::Rejected { reason } => format!("REJECTED:{}", escape_field(reason)),
            Self::UnresolvedConflict { contenders } => format!(
                "UNRESOLVED_CONFLICT:{}",
                contenders
                    .iter()
                    .map(|id| escape_field(id))
                    .collect::<Vec<_>>()
                    .join(",")
            ),
            Self::DuplicateOf { primary_id } => {
                format!("DUPLICATE_OF:{}", escape_field(primary_id))
            }
        }
    }
}

/// Stable, line-oriented output used for replay comparison and CI evidence.
pub fn canonical_decisions(decisions: &[ReservationDecision]) -> String {
    let mut ordered = decisions.iter().collect::<Vec<_>>();
    ordered.sort_by(|a, b| a.reservation_id.cmp(&b.reservation_id));

    let mut output = String::from("logistics-commons-s0/decisions-v1\n");
    for decision in ordered {
        output.push_str(&escape_field(&decision.reservation_id));
        output.push('\t');
        output.push_str(&decision.outcome.canonical_token());
        output.push('\n');
    }
    output
}

pub(crate) fn escape_field(value: &str) -> String {
    let mut output = String::with_capacity(value.len());
    const HEX: &[u8; 16] = b"0123456789ABCDEF";
    for character in value.chars() {
        match character {
            '%' => output.push_str("%25"),
            '"' => output.push_str("%22"),
            '\\' => output.push_str("%5C"),
            ',' => output.push_str("%2C"),
            '\t' => output.push_str("%09"),
            '\n' => output.push_str("%0A"),
            '\r' => output.push_str("%0D"),
            control if control.is_control() => {
                let mut buffer = [0_u8; 4];
                for byte in control.encode_utf8(&mut buffer).as_bytes() {
                    output.push('%');
                    output.push(HEX[(byte >> 4) as usize] as char);
                    output.push(HEX[(byte & 0x0F) as usize] as char);
                }
            }
            ordinary => output.push(ordinary),
        }
    }
    output
}

fn same_effect_payload(a: &ReservationIntent, b: &ReservationIntent) -> bool {
    a.participant_id == b.participant_id
        && a.hub_id == b.hub_id
        && a.sku_id == b.sku_id
        && a.quantity == b.quantity
        && a.expected_revision == b.expected_revision
        && a.idempotency_key == b.idempotency_key
        && a.observed_at_ms == b.observed_at_ms
        && a.lease_until_ms == b.lease_until_ms
}

fn validate_intent(
    snapshot: &InventorySnapshot,
    intent: &ReservationIntent,
    now_ms: u64,
) -> Option<&'static str> {
    if intent.reservation_id.trim().is_empty() {
        return Some("empty-reservation-id");
    }
    if intent.participant_id.trim().is_empty() {
        return Some("empty-participant-id");
    }
    if intent.hub_id != snapshot.hub_id || intent.sku_id != snapshot.sku_id {
        return Some("resource-mismatch");
    }
    if intent.quantity == 0 {
        return Some("zero-quantity");
    }
    if intent.idempotency_key.trim().is_empty() {
        return Some("empty-idempotency-key");
    }
    if snapshot.valid_until_ms < snapshot.observed_at_ms || snapshot.reserved > snapshot.on_hand {
        return Some("malformed-inventory-snapshot");
    }
    if now_ms < snapshot.observed_at_ms
        || now_ms > snapshot.valid_until_ms
        || intent.observed_at_ms < snapshot.observed_at_ms
        || intent.observed_at_ms > now_ms
    {
        return Some("stale-or-future-inventory-observation");
    }
    if now_ms > intent.lease_until_ms {
        return Some("reservation-lease-expired");
    }
    if intent.expected_revision != snapshot.revision {
        return Some("inventory-revision-mismatch");
    }
    None
}

/// Resolve one resource/revision batch. A contested oversubscription becomes an explicit
/// unresolved conflict; this function deliberately does not invent a winner from input order.
pub fn plan_reservations(
    snapshot: &InventorySnapshot,
    intents: &[ReservationIntent],
    now_ms: u64,
) -> Vec<ReservationDecision> {
    let mut outcomes: BTreeMap<String, ReservationOutcome> = BTreeMap::new();
    let mut id_counts: BTreeMap<String, usize> = BTreeMap::new();
    for intent in intents {
        *id_counts.entry(intent.reservation_id.clone()).or_default() += 1;
    }

    let mut idempotency_groups: BTreeMap<(String, String), Vec<&ReservationIntent>> = BTreeMap::new();
    for intent in intents {
        match id_counts.get(&intent.reservation_id).copied().unwrap_or_default() {
            0 => unreachable!("every input id was counted"),
            1 if intent.reservation_id.trim().is_empty() => {
                outcomes.insert(
                    intent.reservation_id.clone(),
                    ReservationOutcome::Rejected {
                        reason: "empty-reservation-id",
                    },
                );
            }
            1 if intent.idempotency_key.trim().is_empty() => {
                outcomes.insert(
                    intent.reservation_id.clone(),
                    ReservationOutcome::Rejected {
                        reason: "empty-idempotency-key",
                    },
                );
            }
            1 => idempotency_groups
                .entry((intent.participant_id.clone(), intent.idempotency_key.clone()))
                .or_default()
                .push(intent),
            _ => {
                outcomes.insert(
                    intent.reservation_id.clone(),
                    ReservationOutcome::Rejected {
                        reason: "duplicate-reservation-id",
                    },
                );
            }
        }
    }

    let mut primary_intents: Vec<&ReservationIntent> = Vec::new();
    for (_, mut group) in idempotency_groups {
        group.sort_by(|a, b| a.reservation_id.cmp(&b.reservation_id));
        let first = group[0];
        if group.iter().all(|candidate| same_effect_payload(first, candidate)) {
            primary_intents.push(first);
            for duplicate in group.iter().skip(1) {
                outcomes.insert(
                    duplicate.reservation_id.clone(),
                    ReservationOutcome::DuplicateOf {
                        primary_id: first.reservation_id.clone(),
                    },
                );
            }
        } else {
            for conflicting in group {
                outcomes.insert(
                    conflicting.reservation_id.clone(),
                    ReservationOutcome::Rejected {
                        reason: "idempotency-key-payload-conflict",
                    },
                );
            }
        }
    }

    let mut eligible: Vec<&ReservationIntent> = Vec::new();
    for intent in primary_intents {
        if let Some(reason) = validate_intent(snapshot, intent, now_ms) {
            outcomes.insert(
                intent.reservation_id.clone(),
                ReservationOutcome::Rejected { reason },
            );
        } else {
            eligible.push(intent);
        }
    }
    eligible.sort_by(|a, b| a.reservation_id.cmp(&b.reservation_id));

    let available = snapshot.available();
    match available {
        None => {
            for intent in eligible {
                outcomes.insert(
                    intent.reservation_id.clone(),
                    ReservationOutcome::Rejected {
                        reason: "malformed-inventory-snapshot",
                    },
                );
            }
        }
        Some(capacity) => {
            let total = eligible
                .iter()
                .try_fold(0_u64, |sum, intent| sum.checked_add(intent.quantity));
            match total {
                None => {
                    for intent in eligible {
                        outcomes.insert(
                            intent.reservation_id.clone(),
                            ReservationOutcome::Rejected {
                                reason: "reservation-quantity-overflow",
                            },
                        );
                    }
                }
                Some(required) if required <= capacity => {
                    for intent in eligible {
                        outcomes.insert(
                            intent.reservation_id.clone(),
                            ReservationOutcome::Confirmed,
                        );
                    }
                }
                Some(_) if eligible.len() > 1 => {
                    let contenders = eligible
                        .iter()
                        .map(|intent| intent.reservation_id.clone())
                        .collect::<Vec<_>>();
                    for intent in eligible {
                        outcomes.insert(
                            intent.reservation_id.clone(),
                            ReservationOutcome::UnresolvedConflict {
                                contenders: contenders.clone(),
                            },
                        );
                    }
                }
                Some(_) => {
                    for intent in eligible {
                        outcomes.insert(
                            intent.reservation_id.clone(),
                            ReservationOutcome::Rejected {
                                reason: "insufficient-available-stock",
                            },
                        );
                    }
                }
            }
        }
    }

    let mut decisions = intents
        .iter()
        .map(|intent| ReservationDecision {
            reservation_id: intent.reservation_id.clone(),
            outcome: outcomes
                .get(&intent.reservation_id)
                .cloned()
                .unwrap_or(ReservationOutcome::Rejected {
                    reason: "missing-planner-outcome",
                }),
        })
        .collect::<Vec<_>>();
    decisions.sort_by(|a, b| a.reservation_id.cmp(&b.reservation_id));
    decisions
}

/// Independent, invariant-oriented checker. It does not call the planner or its validation helper
/// and it does not attempt to choose an allocation winner.
pub fn verify_reservation_plan(
    snapshot: &InventorySnapshot,
    intents: &[ReservationIntent],
    now_ms: u64,
    decisions: &[ReservationDecision],
) -> Vec<String> {
    let mut violations = BTreeSet::new();
    let mut input_by_id: BTreeMap<String, &ReservationIntent> = BTreeMap::new();
    for intent in intents {
        if input_by_id.insert(intent.reservation_id.clone(), intent).is_some() {
            violations.insert(format!("duplicate-input-id:{}", intent.reservation_id));
        }
    }

    let mut decision_by_id: BTreeMap<String, &ReservationDecision> = BTreeMap::new();
    for decision in decisions {
        if decision_by_id
            .insert(decision.reservation_id.clone(), decision)
            .is_some()
        {
            violations.insert(format!("duplicate-decision-id:{}", decision.reservation_id));
        }
    }

    for id in input_by_id.keys() {
        if !decision_by_id.contains_key(id) {
            violations.insert(format!("missing-decision:{id}"));
        }
    }
    for id in decision_by_id.keys() {
        if !input_by_id.contains_key(id) {
            violations.insert(format!("unknown-decision:{id}"));
        }
    }

    let snapshot_window_valid = snapshot.observed_at_ms <= now_ms
        && now_ms <= snapshot.valid_until_ms
        && snapshot.observed_at_ms <= snapshot.valid_until_ms;
    let available = snapshot.on_hand.checked_sub(snapshot.reserved);
    if available.is_none() {
        violations.insert("stock-reserved-exceeds-on-hand".to_owned());
    }

    let mut confirmed_quantity = 0_u64;
    let mut confirmed_idempotency = BTreeSet::new();
    let mut confirmed_ids = BTreeSet::new();

    for decision in decisions {
        let Some(intent) = input_by_id.get(&decision.reservation_id).copied() else {
            continue;
        };
        match &decision.outcome {
            ReservationOutcome::Confirmed => {
                confirmed_ids.insert(decision.reservation_id.clone());
                if !snapshot_window_valid {
                    violations.insert("confirmed-against-stale-snapshot".to_owned());
                }
                if intent.hub_id != snapshot.hub_id || intent.sku_id != snapshot.sku_id {
                    violations.insert(format!("confirmed-resource-mismatch:{}", intent.reservation_id));
                }
                if intent.expected_revision != snapshot.revision {
                    violations.insert(format!("confirmed-wrong-revision:{}", intent.reservation_id));
                }
                if intent.quantity == 0 {
                    violations.insert(format!("confirmed-zero-quantity:{}", intent.reservation_id));
                }
                if intent.participant_id.trim().is_empty() {
                    violations.insert(format!("confirmed-empty-participant:{}", intent.reservation_id));
                }
                if intent.idempotency_key.trim().is_empty() {
                    violations.insert(format!("confirmed-empty-idempotency-key:{}", intent.reservation_id));
                }
                if intent.observed_at_ms < snapshot.observed_at_ms
                    || intent.observed_at_ms > now_ms
                    || now_ms > intent.lease_until_ms
                {
                    violations.insert(format!("confirmed-outside-validity-window:{}", intent.reservation_id));
                }
                if !confirmed_idempotency.insert((intent.participant_id.clone(), intent.idempotency_key.clone())) {
                    violations.insert(format!("idempotency-double-commit:{}:{}", intent.participant_id, intent.idempotency_key));
                }
                confirmed_quantity = match confirmed_quantity.checked_add(intent.quantity) {
                    Some(sum) => sum,
                    None => {
                        violations.insert("confirmed-quantity-overflow".to_owned());
                        u64::MAX
                    }
                };
            }
            ReservationOutcome::DuplicateOf { primary_id } => {
                match (input_by_id.get(primary_id).copied(), decision_by_id.get(primary_id).copied()) {
                    (Some(primary), Some(primary_decision))
                        if primary.idempotency_key == intent.idempotency_key
                            && primary.reservation_id != intent.reservation_id
                            && primary.participant_id == intent.participant_id
                            && primary.hub_id == intent.hub_id
                            && primary.sku_id == intent.sku_id
                            && primary.quantity == intent.quantity
                            && primary.expected_revision == intent.expected_revision
                            && primary.observed_at_ms == intent.observed_at_ms
                            && primary.lease_until_ms == intent.lease_until_ms
                            && !matches!(&primary_decision.outcome, ReservationOutcome::DuplicateOf { .. }) => {}
                    _ => {
                        violations.insert(format!("invalid-idempotency-alias:{}", intent.reservation_id));
                    }
                }
            }
            ReservationOutcome::UnresolvedConflict { contenders } => {
                let mut sorted = contenders.clone();
                sorted.sort();
                sorted.dedup();
                if sorted.as_slice() != contenders.as_slice() || !contenders.contains(&decision.reservation_id) {
                    violations.insert(format!("malformed-conflict-set:{}", decision.reservation_id));
                }
                for contender in contenders {
                    match decision_by_id.get(contender) {
                        Some(other)
                            if matches!(
                                &other.outcome,
                                ReservationOutcome::UnresolvedConflict { contenders: other_set }
                                    if other_set == contenders
                            ) => {}
                        _ => {
                            violations.insert(format!("inconsistent-conflict-set:{contender}"));
                        }
                    }
                    if confirmed_ids.contains(contender) {
                        violations.insert(format!("conflicted-reservation-also-confirmed:{contender}"));
                    }
                }
            }
            ReservationOutcome::Rejected { .. } => {}
        }
    }

    if let Some(capacity) = available {
        if confirmed_quantity > capacity {
            violations.insert("confirmed-stock-exceeds-available".to_owned());
        }
    }

    violations.into_iter().collect()
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ForecastObservation {
    pub model_version: String,
    pub input_digest: String,
    pub predicted_demand_units: u64,
    /// Uncertainty in parts per million; 1_000_000 means 100%.
    pub uncertainty_ppm: u32,
    pub generated_at_ms: u64,
    pub valid_until_ms: u64,
}
// ForecastObservation is advisory data only. It is deliberately not an input to
// plan_reservations, so no forecast value can confirm stock or authorize an effect.

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ShipmentEventKind {
    PickedPacked,
    Dispatched,
    CustodyHandoff {
        from_participant: String,
        to_participant: String,
        evidence_ref: Option<String>,
    },
    DeliveryReported {
        carrier_receipt_ref: Option<String>,
    },
    RecipientAccepted {
        evidence_ref: Option<String>,
    },
    Exception,
    ReturnReceived,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ShipmentEvent {
    pub event_id: String,
    pub idempotency_key: String,
    pub sequence: u64,
    pub kind: ShipmentEventKind,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ShipmentStage {
    Reserved,
    PickedPacked,
    Dispatched,
    CustodyTransferred,
    DeliveryReported,
    RecipientAccepted,
    Exception,
    Returned,
    Unresolved,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ShipmentView {
    pub stage: ShipmentStage,
    pub applied_event_ids: Vec<String>,
    pub unresolved_reason: Option<&'static str>,
}

fn nonempty(value: &Option<String>) -> bool {
    value.as_deref().is_some_and(|text| !text.trim().is_empty())
}

fn unresolved_shipment(reason: &'static str, applied_event_ids: Vec<String>) -> ShipmentView {
    ShipmentView {
        stage: ShipmentStage::Unresolved,
        applied_event_ids,
        unresolved_reason: Some(reason),
    }
}

/// Reduces a small shipment event stream. Invalid transitions fail closed; duplicate
/// idempotency keys with identical event payloads are replayed as no-ops.
pub fn reduce_shipment(events: &[ShipmentEvent]) -> ShipmentView {
    let mut sorted = events.iter().collect::<Vec<_>>();
    sorted.sort_by(|a, b| {
        a.sequence
            .cmp(&b.sequence)
            .then_with(|| a.event_id.cmp(&b.event_id))
    });

    let mut idempotency: BTreeMap<String, ShipmentEvent> = BTreeMap::new();
    for event in sorted {
        if event.event_id.trim().is_empty() || event.idempotency_key.trim().is_empty() {
            return unresolved_shipment("empty-event-or-idempotency-id", Vec::new());
        }
        if let Some(previous) = idempotency.get(&event.idempotency_key) {
            if previous.sequence == event.sequence && previous.kind == event.kind {
                continue;
            }
            return unresolved_shipment("idempotency-key-payload-conflict", Vec::new());
        }
        idempotency.insert(event.idempotency_key.clone(), event.clone());
    }

    let mut effective = idempotency.into_values().collect::<Vec<_>>();
    effective.sort_by(|a, b| {
        a.sequence
            .cmp(&b.sequence)
            .then_with(|| a.event_id.cmp(&b.event_id))
    });

    let mut seen_ids = BTreeSet::new();
    let mut previous_sequence = None;
    let mut stage = ShipmentStage::Reserved;
    let mut applied = Vec::new();

    for event in effective {
        if !seen_ids.insert(event.event_id.clone()) {
            return unresolved_shipment("duplicate-event-id", applied);
        }
        if previous_sequence.is_some_and(|previous| event.sequence <= previous) {
            return unresolved_shipment("non-unique-or-non-monotonic-sequence", applied);
        }
        previous_sequence = Some(event.sequence);

        let next = match (stage, &event.kind) {
            (ShipmentStage::Reserved, ShipmentEventKind::PickedPacked) => {
                Some(ShipmentStage::PickedPacked)
            }
            (ShipmentStage::PickedPacked, ShipmentEventKind::Dispatched) => {
                Some(ShipmentStage::Dispatched)
            }
            (
                ShipmentStage::Dispatched | ShipmentStage::CustodyTransferred,
                ShipmentEventKind::CustodyHandoff {
                    from_participant,
                    to_participant,
                    evidence_ref,
                },
            ) if !from_participant.trim().is_empty()
                && !to_participant.trim().is_empty()
                && nonempty(evidence_ref) =>
            {
                Some(ShipmentStage::CustodyTransferred)
            }
            (
                ShipmentStage::Dispatched | ShipmentStage::CustodyTransferred,
                ShipmentEventKind::DeliveryReported { .. },
            ) => Some(ShipmentStage::DeliveryReported),
            (
                ShipmentStage::DeliveryReported,
                ShipmentEventKind::RecipientAccepted { evidence_ref },
            ) if nonempty(evidence_ref) => Some(ShipmentStage::RecipientAccepted),
            (
                ShipmentStage::Reserved
                | ShipmentStage::PickedPacked
                | ShipmentStage::Dispatched
                | ShipmentStage::CustodyTransferred
                | ShipmentStage::DeliveryReported,
                ShipmentEventKind::Exception,
            ) => Some(ShipmentStage::Exception),
            (ShipmentStage::Exception, ShipmentEventKind::ReturnReceived) => {
                Some(ShipmentStage::Returned)
            }
            _ => None,
        };

        match next {
            Some(next_stage) => {
                stage = next_stage;
                applied.push(event.event_id);
            }
            None => {
                return unresolved_shipment("invalid-shipment-transition-or-missing-evidence", applied);
            }
        }
    }

    ShipmentView {
        stage,
        applied_event_ids: applied,
        unresolved_reason: None,
    }
}

/// Separate invariant checker for a reduced shipment. It checks the evidence ordering directly
/// rather than calling the state reducer.
pub fn verify_shipment(events: &[ShipmentEvent], view: &ShipmentView) -> Vec<String> {
    let mut violations = BTreeSet::new();
    let mut by_id: BTreeMap<&str, &ShipmentEvent> = BTreeMap::new();
    for event in events {
        if by_id.insert(event.event_id.as_str(), event).is_some() {
            violations.insert(format!("duplicate-shipment-event-id:{}", event.event_id));
        }
    }
    let mut applied: Vec<&ShipmentEvent> = Vec::new();
    for event_id in &view.applied_event_ids {
        match by_id.get(event_id.as_str()).copied() {
            Some(event) => applied.push(event),
            None => { violations.insert(format!("applied-event-not-in-input:{event_id}")); }
        }
    }
    if applied.windows(2).any(|pair| pair[0].sequence >= pair[1].sequence) {
        violations.insert("applied-events-not-strictly-ordered".to_owned());
    }

    // Reconstruct only the applied prefix independently. A forged view cannot attach extra
    // events after acceptance or claim a later stage than its final applied event supports.
    let mut replayed_stage = ShipmentStage::Reserved;
    let mut applied_transition_invalid = false;
    for event in &applied {
        let next_stage = match (replayed_stage, &event.kind) {
            (ShipmentStage::Reserved, ShipmentEventKind::PickedPacked) => Some(ShipmentStage::PickedPacked),
            (ShipmentStage::PickedPacked, ShipmentEventKind::Dispatched) => Some(ShipmentStage::Dispatched),
            (ShipmentStage::Dispatched | ShipmentStage::CustodyTransferred,
                ShipmentEventKind::CustodyHandoff { from_participant, to_participant, evidence_ref })
                if !from_participant.trim().is_empty()
                    && !to_participant.trim().is_empty()
                    && nonempty(evidence_ref) => Some(ShipmentStage::CustodyTransferred),
            (ShipmentStage::Dispatched | ShipmentStage::CustodyTransferred,
                ShipmentEventKind::DeliveryReported { .. }) => Some(ShipmentStage::DeliveryReported),
            (ShipmentStage::DeliveryReported,
                ShipmentEventKind::RecipientAccepted { evidence_ref }) if nonempty(evidence_ref) =>
                    Some(ShipmentStage::RecipientAccepted),
            (ShipmentStage::Reserved | ShipmentStage::PickedPacked | ShipmentStage::Dispatched
                | ShipmentStage::CustodyTransferred | ShipmentStage::DeliveryReported,
                ShipmentEventKind::Exception) => Some(ShipmentStage::Exception),
            (ShipmentStage::Exception, ShipmentEventKind::ReturnReceived) => Some(ShipmentStage::Returned),
            _ => None,
        };
        match next_stage {
            Some(next) => replayed_stage = next,
            None => {
                violations.insert(format!("applied-event-invalid-transition:{}", event.event_id));
                applied_transition_invalid = true;
                break;
            }
        }
    }
    if view.stage != ShipmentStage::Unresolved
        && (applied_transition_invalid || replayed_stage != view.stage) {
        violations.insert("shipment-view-stage-does-not-match-applied-prefix".to_owned());
    }

    let applied_kind = |predicate: fn(&ShipmentEventKind) -> bool| {
        applied.iter().any(|event| predicate(&event.kind))
    };
    let applied_pick = applied.iter().find(|event| event.kind == ShipmentEventKind::PickedPacked).copied();
    let applied_dispatch = applied.iter().find(|event| event.kind == ShipmentEventKind::Dispatched).copied();
    let applied_delivery = applied.iter().find(|event| matches!(&event.kind, ShipmentEventKind::DeliveryReported { .. })).copied();
    let applied_acceptance = applied.iter().find(|event| matches!(
        &event.kind,
        ShipmentEventKind::RecipientAccepted { evidence_ref } if nonempty(evidence_ref)
    )).copied();
    let applied_handoff = applied.iter().find(|event| matches!(
        &event.kind,
        ShipmentEventKind::CustodyHandoff { from_participant, to_participant, evidence_ref }
            if !from_participant.trim().is_empty()
                && !to_participant.trim().is_empty()
                && nonempty(evidence_ref)
    )).copied();
    let applied_exception = applied.iter().any(|event| event.kind == ShipmentEventKind::Exception);
    let applied_return = applied.iter().find(|event| event.kind == ShipmentEventKind::ReturnReceived).copied();

    if matches!(view.stage, ShipmentStage::PickedPacked) && applied_pick.is_none() {
        violations.insert("picked-packed-stage-without-applied-pick".to_owned());
    }
    if matches!(view.stage, ShipmentStage::Dispatched | ShipmentStage::CustodyTransferred
        | ShipmentStage::DeliveryReported | ShipmentStage::RecipientAccepted)
    {
        match (applied_pick, applied_dispatch) {
            (Some(pick), Some(dispatch)) if pick.sequence < dispatch.sequence => {}
            _ => { violations.insert("shipment-stage-without-applied-pick-before-dispatch".to_owned()); }
        }
    }
    if view.stage == ShipmentStage::CustodyTransferred {
        match (applied_dispatch, applied_handoff) {
            (Some(dispatch), Some(handoff)) if dispatch.sequence < handoff.sequence => {}
            _ => { violations.insert("custody-stage-without-applied-evidenced-handoff".to_owned()); }
        }
    }
    if matches!(view.stage, ShipmentStage::DeliveryReported | ShipmentStage::RecipientAccepted) {
        match (applied_dispatch, applied_delivery) {
            (Some(dispatch), Some(delivery)) if dispatch.sequence < delivery.sequence => {}
            _ => { violations.insert("delivery-stage-without-applied-delivery-after-dispatch".to_owned()); }
        }
    }
    if view.stage == ShipmentStage::RecipientAccepted {
        match (applied_delivery, applied_acceptance) {
            (Some(delivery), Some(acceptance)) if delivery.sequence < acceptance.sequence => {}
            _ => { violations.insert("recipient-acceptance-lacks-applied-ordered-evidence".to_owned()); }
        }
    }
    if view.stage == ShipmentStage::Exception && !applied_exception {
        violations.insert("exception-stage-without-applied-exception".to_owned());
    }
    if view.stage == ShipmentStage::Returned {
        match (applied_exception, applied_return) {
            (true, Some(returned)) if applied.iter().any(|event|
                event.kind == ShipmentEventKind::Exception && event.sequence < returned.sequence) => {}
            _ => { violations.insert("returned-stage-without-applied-exception-before-return".to_owned()); }
        }
    }
    if view.stage == ShipmentStage::Reserved && !applied.is_empty() {
        violations.insert("reserved-stage-has-applied-events".to_owned());
    }
    if applied_kind(|kind| matches!(kind, ShipmentEventKind::RecipientAccepted { evidence_ref } if !nonempty(evidence_ref))) {
        violations.insert("applied-recipient-acceptance-without-evidence".to_owned());
    }
    if view.stage == ShipmentStage::Unresolved && view.unresolved_reason.is_none() {
        violations.insert("unresolved-shipment-without-reason".to_owned());
    }
    violations.into_iter().collect()
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PrivateShipmentFacts {
    pub customer_address: String,
    pub supplier_price_cents: u64,
    pub worker_route: Vec<String>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ShipmentRecord {
    pub shipment_id: String,
    pub view: ShipmentView,
    pub private_facts: PrivateShipmentFacts,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PublicShipmentSummary {
    pub shipment_id: String,
    pub stage: ShipmentStage,
}

/// Explicit public projection. Sensitive source facts are not copied to the public type.
pub fn public_projection(record: &ShipmentRecord) -> PublicShipmentSummary {
    PublicShipmentSummary {
        shipment_id: record.shipment_id.clone(),
        stage: record.view.stage,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn snapshot(on_hand: u64, reserved: u64, valid_until_ms: u64) -> InventorySnapshot {
        InventorySnapshot {
            hub_id: "coop-hub-01".into(),
            sku_id: "rice-25kg".into(),
            on_hand,
            reserved,
            revision: 7,
            observed_at_ms: 100,
            valid_until_ms,
        }
    }

    fn intent(id: &str, key: &str, quantity: u64) -> ReservationIntent {
        ReservationIntent {
            reservation_id: id.into(),
            participant_id: format!("member-{id}"),
            hub_id: "coop-hub-01".into(),
            sku_id: "rice-25kg".into(),
            quantity,
            expected_revision: 7,
            idempotency_key: key.into(),
            observed_at_ms: 110,
            lease_until_ms: 180,
        }
    }

    fn event(id: &str, sequence: u64, kind: ShipmentEventKind) -> ShipmentEvent {
        ShipmentEvent {
            event_id: id.into(),
            idempotency_key: id.into(),
            sequence,
            kind,
        }
    }

    #[test]
    fn canonical_decision_encoding_escapes_nested_delimiters_and_controls() {
        let decisions = vec![ReservationDecision {
            reservation_id: "order,a\t\0\n%\"\\tail".into(),
            outcome: ReservationOutcome::DuplicateOf {
                primary_id: "primary,one\"\\tail".into(),
            },
        }];
        let canonical = canonical_decisions(&decisions);
        assert!(canonical.contains("order%2Ca%09%00%0A%25%22%5Ctail"));
        assert!(canonical.contains("DUPLICATE_OF:primary%2Cone%22%5Ctail"));
    }

    #[test]
    fn same_frozen_input_produces_byte_identical_canonical_output() {
        let stock = snapshot(1, 0, 200);
        let requests = vec![
            intent("r-02", "idem-02", 1),
            intent("r-01", "idem-01", 1),
        ];
        let first = plan_reservations(&stock, &requests, 120);
        let second = plan_reservations(&stock, &requests, 120);

        assert_eq!(canonical_decisions(&first).as_bytes(), canonical_decisions(&second).as_bytes());
        assert!(first.iter().all(|decision| matches!(&decision.outcome,
            ReservationOutcome::UnresolvedConflict { .. }
        )));
        assert!(verify_reservation_plan(&stock, &requests, 120, &first).is_empty());
    }

    #[test]
    fn oversubscribed_last_unit_is_an_explicit_conflict_not_two_commits() {
        let stock = snapshot(1, 0, 200);
        let requests = vec![
            intent("r-a", "idem-a", 1),
            intent("r-b", "idem-b", 1),
        ];
        let decisions = plan_reservations(&stock, &requests, 120);
        let confirmed = decisions
            .iter()
            .filter(|decision| decision.outcome == ReservationOutcome::Confirmed)
            .count();
        assert_eq!(confirmed, 0);
        assert!(decisions.iter().all(|decision| matches!(&decision.outcome,
            ReservationOutcome::UnresolvedConflict { .. }
        )));
    }

    #[test]
    fn same_idempotency_payload_is_counted_once() {
        let stock = snapshot(2, 0, 200);
        let first = intent("r-a", "idem-shared", 1);
        let mut duplicate = first.clone();
        duplicate.reservation_id = "r-b".into();

        let decisions = plan_reservations(&stock, &[first.clone(), duplicate], 120);
        assert_eq!(
            decisions.iter().filter(|d| d.outcome == ReservationOutcome::Confirmed).count(),
            1
        );
        assert!(decisions.iter().any(|d| matches!(
            &d.outcome,
            ReservationOutcome::DuplicateOf { primary_id } if primary_id == "r-a"
        )));
        let mut duplicate_for_verifier = first.clone();
        duplicate_for_verifier.reservation_id = "r-b".into();
        assert!(verify_reservation_plan(
            &stock,
            &[first, duplicate_for_verifier],
            120,
            &decisions
        ).is_empty());
    }

    #[test]
    fn reused_idempotency_key_with_different_payload_fails_closed() {
        let stock = snapshot(10, 0, 200);
        // Idempotency keys are participant-scoped. Keep the participant constant so
        // this exercises a conflicting replay, not two unrelated participants.
        let first = intent("r-a", "idem-shared", 1);
        let mut conflicting = intent("r-b", "idem-shared", 2);
        conflicting.participant_id = first.participant_id.clone();
        let requests = vec![first, conflicting];
        let decisions = plan_reservations(&stock, &requests, 120);
        assert!(decisions.iter().all(|d| matches!(&d.outcome,
            ReservationOutcome::Rejected { reason: "idempotency-key-payload-conflict" }
        )));
        assert!(!decisions.iter().any(|d| d.outcome == ReservationOutcome::Confirmed));
    }

    #[test]
    fn stale_inventory_and_malformed_stock_never_confirm() {
        let requests = vec![intent("r-a", "idem-a", 1)];
        let stale = snapshot(5, 0, 119);
        let stale_decisions = plan_reservations(&stale, &requests, 120);
        assert!(!stale_decisions.iter().any(|d| d.outcome == ReservationOutcome::Confirmed));
        assert!(stale_decisions.iter().any(|d| matches!(&d.outcome,
            ReservationOutcome::Rejected { reason: "stale-or-future-inventory-observation" }
        )));

        let malformed = snapshot(1, 2, 200);
        let malformed_decisions = plan_reservations(&malformed, &requests, 120);
        assert!(!malformed_decisions.iter().any(|d| d.outcome == ReservationOutcome::Confirmed));
        assert!(verify_reservation_plan(&malformed, &requests, 120, &malformed_decisions)
            .iter()
            .any(|violation| violation == "stock-reserved-exceeds-on-hand"));
    }

    #[test]
    fn independent_verifier_detects_invented_overcommit() {
        let stock = snapshot(1, 0, 200);
        let requests = vec![
            intent("r-a", "idem-a", 1),
            intent("r-b", "idem-b", 1),
        ];
        let invented = vec![
            ReservationDecision { reservation_id: "r-a".into(), outcome: ReservationOutcome::Confirmed },
            ReservationDecision { reservation_id: "r-b".into(), outcome: ReservationOutcome::Confirmed },
        ];
        assert!(verify_reservation_plan(&stock, &requests, 120, &invented)
            .iter()
            .any(|violation| violation == "confirmed-stock-exceeds-available"));
    }

    #[test]
    fn unrelated_participants_can_reuse_idempotency_key() {
        let stock = snapshot(3, 0, 200);
        let mut a = intent("r-a", "common-opaque-key", 1);
        let mut b = intent("r-b", "common-opaque-key", 1);
        b.participant_id = "another-cooperative".into();
        a.participant_id = "first-cooperative".into();
        let requests = vec![a, b];
        let decisions = plan_reservations(&stock, &requests, 120);
        assert_eq!(decisions.iter().filter(|d| d.outcome == ReservationOutcome::Confirmed).count(), 2);
        assert!(verify_reservation_plan(&stock, &requests, 120, &decisions).is_empty());
    }

    #[test]
    fn shipment_verifier_rejects_unapplied_delivery_event() {
        let events = vec![
            event("pick", 1, ShipmentEventKind::PickedPacked),
            event("dispatch", 2, ShipmentEventKind::Dispatched),
            event("delivery-report", 3, ShipmentEventKind::DeliveryReported { carrier_receipt_ref: None }),
        ];
        let forged_view = ShipmentView {
            stage: ShipmentStage::DeliveryReported,
            applied_event_ids: vec!["pick".into(), "dispatch".into()],
            unresolved_reason: None,
        };
        assert!(verify_shipment(&events, &forged_view)
            .iter().any(|violation| violation == "delivery-stage-without-applied-delivery-after-dispatch"));
    }

    #[test]
    fn shipment_verifier_rejects_acceptance_without_applied_pick() {
        let events = vec![
            event("pick", 1, ShipmentEventKind::PickedPacked),
            event("dispatch", 2, ShipmentEventKind::Dispatched),
            event("delivery-report", 3, ShipmentEventKind::DeliveryReported { carrier_receipt_ref: None }),
            event("accept", 4, ShipmentEventKind::RecipientAccepted { evidence_ref: Some("recipient-signature".into()) }),
        ];
        let forged_view = ShipmentView {
            stage: ShipmentStage::RecipientAccepted,
            applied_event_ids: vec!["dispatch".into(), "delivery-report".into(), "accept".into()],
            unresolved_reason: None,
        };
        assert!(verify_shipment(&events, &forged_view)
            .iter().any(|violation| violation == "shipment-stage-without-applied-pick-before-dispatch"));
    }

    #[test]
    fn shipment_verifier_rejects_event_after_terminal_acceptance() {
        let events = vec![
            event("pick", 1, ShipmentEventKind::PickedPacked),
            event("dispatch", 2, ShipmentEventKind::Dispatched),
            event("delivery-report", 3, ShipmentEventKind::DeliveryReported { carrier_receipt_ref: None }),
            event("accept", 4, ShipmentEventKind::RecipientAccepted { evidence_ref: Some("recipient-signature".into()) }),
            event("late-exception", 5, ShipmentEventKind::Exception),
        ];
        let forged_view = ShipmentView {
            stage: ShipmentStage::RecipientAccepted,
            applied_event_ids: vec!["pick".into(), "dispatch".into(), "delivery-report".into(), "accept".into(), "late-exception".into()],
            unresolved_reason: None,
        };
        let violations = verify_shipment(&events, &forged_view);
        assert!(violations.iter().any(|violation| violation == "applied-event-invalid-transition:late-exception"));
        assert!(violations.iter().any(|violation| violation == "shipment-view-stage-does-not-match-applied-prefix"));
    }

    #[test]
    fn dispatch_and_delivery_report_do_not_mean_recipient_acceptance() {
        let events = vec![
            event("pick", 1, ShipmentEventKind::PickedPacked),
            event("dispatch", 2, ShipmentEventKind::Dispatched),
            event(
                "delivery-report",
                3,
                ShipmentEventKind::DeliveryReported {
                    carrier_receipt_ref: Some("carrier-proof-1".into()),
                },
            ),
        ];
        let view = reduce_shipment(&events);
        assert_eq!(view.stage, ShipmentStage::DeliveryReported);
        assert_ne!(view.stage, ShipmentStage::RecipientAccepted);
        assert!(verify_shipment(&events, &view).is_empty());
    }

    #[test]
    fn recipient_acceptance_requires_separate_evidence_after_delivery_report() {
        let events = vec![
            event("pick", 1, ShipmentEventKind::PickedPacked),
            event("dispatch", 2, ShipmentEventKind::Dispatched),
            event("delivery-report", 3, ShipmentEventKind::DeliveryReported { carrier_receipt_ref: None }),
            event("accept", 4, ShipmentEventKind::RecipientAccepted { evidence_ref: Some("recipient-signature-7".into()) }),
        ];
        let view = reduce_shipment(&events);
        assert_eq!(view.stage, ShipmentStage::RecipientAccepted);
        assert!(verify_shipment(&events, &view).is_empty());
    }

    #[test]
    fn recipient_acceptance_without_evidence_becomes_unresolved() {
        let events = vec![
            event("pick", 1, ShipmentEventKind::PickedPacked),
            event("dispatch", 2, ShipmentEventKind::Dispatched),
            event("delivery-report", 3, ShipmentEventKind::DeliveryReported { carrier_receipt_ref: None }),
            event("accept", 4, ShipmentEventKind::RecipientAccepted { evidence_ref: None }),
        ];
        let view = reduce_shipment(&events);
        assert_eq!(view.stage, ShipmentStage::Unresolved);
        assert!(view.unresolved_reason.is_some());
        assert!(verify_shipment(&events, &view).is_empty());
    }

    #[test]
    fn same_sequence_competing_shipment_events_are_not_ordered_by_id() {
        let events = vec![
            event("a", 1, ShipmentEventKind::PickedPacked),
            event("b", 1, ShipmentEventKind::Dispatched),
        ];
        let view = reduce_shipment(&events);
        assert_eq!(view.stage, ShipmentStage::Unresolved);
    }

    #[test]
    fn public_projection_omits_customer_price_and_worker_route() {
        let record = ShipmentRecord {
            shipment_id: "shipment-1".into(),
            view: ShipmentView {
                stage: ShipmentStage::Dispatched,
                applied_event_ids: vec!["dispatch".into()],
                unresolved_reason: None,
            },
            private_facts: PrivateShipmentFacts {
                customer_address: "PRIVATE-ADDRESS-42".into(),
                supplier_price_cents: 987_654,
                worker_route: vec!["PRIVATE-DEPOT-9".into()],
            },
        };
        let public = format!("{:?}", public_projection(&record));
        assert!(!public.contains("PRIVATE-ADDRESS-42"));
        assert!(!public.contains("987654"));
        assert!(!public.contains("PRIVATE-DEPOT-9"));
        assert!(public.contains("shipment-1"));
    }

    #[test]
    fn forecast_is_data_only_and_not_an_authorization_input() {
        let forecast = ForecastObservation {
            model_version: "symthaea-advisory-v0".into(),
            input_digest: "synthetic-input-digest".into(),
            predicted_demand_units: u64::MAX,
            uncertainty_ppm: 1_000_000,
            generated_at_ms: 110,
            valid_until_ms: 180,
        };
        let stock = snapshot(3, 0, 200);
        let requests = vec![intent("r-a", "idem-a", 1)];
        let decisions = plan_reservations(&stock, &requests, 120);

        assert_eq!(forecast.uncertainty_ppm, 1_000_000);
        assert_eq!(decisions[0].outcome, ReservationOutcome::Confirmed);
        assert!(verify_reservation_plan(&stock, &requests, 120, &decisions).is_empty());
    }
}
