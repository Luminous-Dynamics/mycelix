use std::collections::{BTreeMap, BTreeSet};

use crate::escape_field;

/// An offline request is a claim against a snapshot, not a globally exclusive lock.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct OfflineReservation {
    pub participant_id: String,
    pub node_id: String,
    pub reservation_id: String,
    pub idempotency_key: String,
    pub hub_id: String,
    pub sku_id: String,
    pub quantity: u64,
    pub snapshot_revision: u64,
    pub snapshot_available_units: u64,
    pub snapshot_observed_at_ms: u64,
    pub snapshot_valid_until_ms: u64,
    pub observed_at_ms: u64,
    pub lease_until_ms: u64,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ReconciliationDisposition {
    Rejected { reason: &'static str },
    DuplicateOf { primary_id: String },
    UnresolvedConflict { reason: &'static str, contenders: Vec<String> },
    /// An authoritative fresh-state check is still required; this is not a commit.
    AwaitingAuthoritativeRecheck,
}

impl ReconciliationDisposition {
    fn canonical_token(&self) -> String {
        match self {
            Self::Rejected { reason } => format!("REJECTED:{}", escape_field(reason)),
            Self::DuplicateOf { primary_id } => format!("DUPLICATE_OF:{}", escape_field(primary_id)),
            Self::UnresolvedConflict { reason, contenders } => format!(
                "UNRESOLVED_CONFLICT:{}:{}",
                escape_field(reason),
                contenders.iter().map(|value| escape_field(value)).collect::<Vec<_>>().join(",")
            ),
            Self::AwaitingAuthoritativeRecheck => "AWAITING_AUTHORITATIVE_RECHECK".to_owned(),
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ReconciliationDecision {
    pub participant_id: String,
    pub node_id: String,
    pub reservation_id: String,
    pub disposition: ReconciliationDisposition,
}

/// Stable canonical output for deterministic replay and independent review.
pub fn canonical_reconciliation(decisions: &[ReconciliationDecision]) -> String {
    let mut ordered = decisions.iter().collect::<Vec<_>>();
    ordered.sort_by(|a, b| {
        a.participant_id.cmp(&b.participant_id)
            .then_with(|| a.node_id.cmp(&b.node_id))
            .then_with(|| a.reservation_id.cmp(&b.reservation_id))
            .then_with(|| a.disposition.canonical_token().cmp(&b.disposition.canonical_token()))
    });
    let mut output = String::from("logistics-commons-s0/offline-reconciliation-v1\n");
    for decision in ordered {
        output.push_str(&escape_field(&decision.participant_id));
        output.push('\t');
        output.push_str(&escape_field(&decision.node_id));
        output.push('\t');
        output.push_str(&escape_field(&decision.reservation_id));
        output.push('\t');
        output.push_str(&decision.disposition.canonical_token());
        output.push('\n');
    }
    output
}

fn valid_at_reconciliation(request: &OfflineReservation, now_ms: u64) -> Option<&'static str> {
    if request.participant_id.trim().is_empty() { return Some("empty-participant-id"); }
    if request.node_id.trim().is_empty() { return Some("empty-node-id"); }
    if request.reservation_id.trim().is_empty() { return Some("empty-reservation-id"); }
    if request.idempotency_key.trim().is_empty() { return Some("empty-idempotency-key"); }
    if request.hub_id.trim().is_empty() || request.sku_id.trim().is_empty() {
        return Some("empty-resource-id");
    }
    if request.quantity == 0 { return Some("zero-quantity"); }
    if request.snapshot_observed_at_ms > request.snapshot_valid_until_ms
        || request.snapshot_observed_at_ms > now_ms
        || now_ms > request.snapshot_valid_until_ms
        || request.observed_at_ms < request.snapshot_observed_at_ms
        || request.observed_at_ms > now_ms
    {
        return Some("stale-or-invalid-snapshot-window");
    }
    if now_ms > request.lease_until_ms { return Some("offline-reservation-lease-expired"); }
    None
}

fn same_effect_payload(a: &OfflineReservation, b: &OfflineReservation) -> bool {
    a.participant_id == b.participant_id
        && a.idempotency_key == b.idempotency_key
        && a.hub_id == b.hub_id
        && a.sku_id == b.sku_id
        && a.quantity == b.quantity
        && a.snapshot_revision == b.snapshot_revision
        && a.snapshot_available_units == b.snapshot_available_units
        && a.snapshot_observed_at_ms == b.snapshot_observed_at_ms
        && a.snapshot_valid_until_ms == b.snapshot_valid_until_ms
        && a.observed_at_ms == b.observed_at_ms
        && a.lease_until_ms == b.lease_until_ms
}

fn same_snapshot_context(a: &OfflineReservation, b: &OfflineReservation) -> bool {
    a.snapshot_revision == b.snapshot_revision
        && a.snapshot_available_units == b.snapshot_available_units
        && a.snapshot_observed_at_ms == b.snapshot_observed_at_ms
        && a.snapshot_valid_until_ms == b.snapshot_valid_until_ms
}

/// Reconcile participant-owned offline intents. No branch returns a committed/confirmed result:
/// a non-conflicting batch must be rechecked against fresh authoritative state before any effect.
pub fn reconcile_offline_reservations(
    requests: &[OfflineReservation],
    now_ms: u64,
) -> Vec<ReconciliationDecision> {
    let mut outcomes: Vec<Option<ReconciliationDisposition>> = vec![None; requests.len()];
    let mut id_counts: BTreeMap<String, usize> = BTreeMap::new();
    for request in requests {
        *id_counts.entry(request.reservation_id.clone()).or_default() += 1;
    }

    for (index, request) in requests.iter().enumerate() {
        if id_counts.get(&request.reservation_id).copied().unwrap_or_default() > 1 {
            outcomes[index] = Some(ReconciliationDisposition::Rejected { reason: "duplicate-reservation-id" });
        } else if let Some(reason) = valid_at_reconciliation(request, now_ms) {
            outcomes[index] = Some(ReconciliationDisposition::Rejected { reason });
        }
    }

    // Idempotency keys are scoped to a participant, so unrelated organizations can use
    // the same opaque value without either silently aliasing or globally colliding.
    let mut by_idempotency: BTreeMap<(String, String), Vec<usize>> = BTreeMap::new();
    for (index, request) in requests.iter().enumerate() {
        // Detect payload collisions even when one copy is stale or otherwise ineligible.
        // Otherwise a valid-looking reuse could hide behind an invalid copy of the same key.
        let unique_reservation_id =
            id_counts.get(&request.reservation_id).copied().unwrap_or_default() == 1;
        if unique_reservation_id
            && !request.participant_id.trim().is_empty()
            && !request.idempotency_key.trim().is_empty()
        {
            by_idempotency.entry((request.participant_id.clone(), request.idempotency_key.clone()))
                .or_default().push(index);
        }
    }

    let mut primaries = Vec::new();
    for indices in by_idempotency.values_mut() {
        indices.sort_by(|a, b| requests[*a].reservation_id.cmp(&requests[*b].reservation_id)
            .then_with(|| requests[*a].node_id.cmp(&requests[*b].node_id)));
        let primary_index = indices[0];
        let primary = &requests[primary_index];
        if indices.iter().all(|index| same_effect_payload(primary, &requests[*index])) {
            primaries.push(primary_index);
            for duplicate_index in indices.iter().skip(1) {
                outcomes[*duplicate_index] = Some(ReconciliationDisposition::DuplicateOf {
                    primary_id: primary.reservation_id.clone(),
                });
            }
        } else {
            for index in indices.iter() {
                outcomes[*index] = Some(ReconciliationDisposition::Rejected {
                    reason: "idempotency-key-payload-conflict",
                });
            }
        }
    }

    let mut by_resource: BTreeMap<(String, String), Vec<usize>> = BTreeMap::new();
    for index in primaries {
        if outcomes[index].is_none() {
            let request = &requests[index];
            by_resource.entry((request.hub_id.clone(), request.sku_id.clone()))
                .or_default().push(index);
        }
    }

    for indices in by_resource.values_mut() {
        indices.sort_by(|a, b| requests[*a].reservation_id.cmp(&requests[*b].reservation_id)
            .then_with(|| requests[*a].node_id.cmp(&requests[*b].node_id)));
        let first = &requests[indices[0]];
        let contenders = indices.iter().map(|index| requests[*index].reservation_id.clone())
            .collect::<Vec<_>>();

        if indices.iter().any(|index| !same_snapshot_context(first, &requests[*index])) {
            for index in indices.iter() {
                outcomes[*index] = Some(ReconciliationDisposition::UnresolvedConflict {
                    reason: "snapshot-context-disagreement", contenders: contenders.clone(),
                });
            }
            continue;
        }

        let aggregate = indices.iter().try_fold(0_u64, |sum, index| {
            sum.checked_add(requests[*index].quantity)
        });
        match aggregate {
            None => for index in indices.iter() {
                outcomes[*index] = Some(ReconciliationDisposition::UnresolvedConflict {
                    reason: "quantity-overflow", contenders: contenders.clone(),
                });
            },
            Some(required) if required > first.snapshot_available_units && indices.len() > 1 => {
                for index in indices.iter() {
                    outcomes[*index] = Some(ReconciliationDisposition::UnresolvedConflict {
                        reason: "oversubscribed-offline-snapshot", contenders: contenders.clone(),
                    });
                }
            }
            Some(required) if required > first.snapshot_available_units => {
                for index in indices.iter() {
                    outcomes[*index] = Some(ReconciliationDisposition::Rejected {
                        reason: "insufficient-snapshot-availability",
                    });
                }
            }
            Some(_) => for index in indices.iter() {
                outcomes[*index] = Some(ReconciliationDisposition::AwaitingAuthoritativeRecheck);
            },
        }
    }

    let mut decisions = requests.iter().enumerate().map(|(index, request)| ReconciliationDecision {
        participant_id: request.participant_id.clone(),
        node_id: request.node_id.clone(),
        reservation_id: request.reservation_id.clone(),
        disposition: outcomes[index].clone().unwrap_or(
            ReconciliationDisposition::Rejected { reason: "missing-reconciliation-outcome" }
        ),
    }).collect::<Vec<_>>();
    decisions.sort_by(|a, b| a.participant_id.cmp(&b.participant_id)
        .then_with(|| a.node_id.cmp(&b.node_id))
        .then_with(|| a.reservation_id.cmp(&b.reservation_id))
        .then_with(|| a.disposition.canonical_token().cmp(&b.disposition.canonical_token())));
    decisions
}

/// Independent checker; it does not call the reconciler's validity/context helpers.
pub fn verify_reconciliation(
    requests: &[OfflineReservation],
    now_ms: u64,
    decisions: &[ReconciliationDecision],
) -> Vec<String> {
    let mut violations = BTreeSet::new();
    type Identity = (String, String, String);
    let mut request_by_id: BTreeMap<Identity, &OfflineReservation> = BTreeMap::new();
    let mut request_by_reservation_id: BTreeMap<String, &OfflineReservation> = BTreeMap::new();
    let mut requests_by_resource: BTreeMap<(String, String), Vec<&OfflineReservation>> = BTreeMap::new();
    let mut reservation_id_counts: BTreeMap<String, usize> = BTreeMap::new();
    for request in requests {
        *reservation_id_counts.entry(request.reservation_id.clone()).or_default() += 1;
        let identity = (request.participant_id.clone(), request.node_id.clone(), request.reservation_id.clone());
        if request_by_id.insert(identity, request).is_some() {
            violations.insert(format!("duplicate-request-identity:{}", request.reservation_id));
        }
        request_by_reservation_id.insert(request.reservation_id.clone(), request);
        requests_by_resource
            .entry((request.hub_id.clone(), request.sku_id.clone()))
            .or_default()
            .push(request);
    }
    for (reservation_id, count) in reservation_id_counts {
        if count > 1 {
            violations.insert(format!("duplicate-reservation-id:{reservation_id}"));
        }
    }

    let mut decision_by_id: BTreeMap<Identity, &ReconciliationDecision> = BTreeMap::new();
    for decision in decisions {
        let identity = (decision.participant_id.clone(), decision.node_id.clone(), decision.reservation_id.clone());
        if decision_by_id.insert(identity, decision).is_some() {
            violations.insert(format!("duplicate-decision-identity:{}", decision.reservation_id));
        }
    }

    for identity in request_by_id.keys() {
        if !decision_by_id.contains_key(identity) { violations.insert(format!("missing-decision:{}", identity.2)); }
    }
    for identity in decision_by_id.keys() {
        if !request_by_id.contains_key(identity) { violations.insert(format!("unknown-decision:{}", identity.2)); }
    }

    let mut pending_by_resource: BTreeMap<(String, String), Vec<&OfflineReservation>> = BTreeMap::new();
    let mut pending_keys = BTreeSet::new();
    for (identity, decision) in &decision_by_id {
        let Some(request) = request_by_id.get(identity).copied() else { continue };
        match &decision.disposition {
            ReconciliationDisposition::AwaitingAuthoritativeRecheck => {
                if request.participant_id.trim().is_empty() || request.node_id.trim().is_empty()
                    || request.reservation_id.trim().is_empty() || request.idempotency_key.trim().is_empty()
                    || request.hub_id.trim().is_empty() || request.sku_id.trim().is_empty() || request.quantity == 0
                {
                    violations.insert(format!("invalid-request-awaiting-recheck:{}", request.reservation_id));
                }
                if request.snapshot_observed_at_ms > request.snapshot_valid_until_ms
                    || request.snapshot_observed_at_ms > now_ms || now_ms > request.snapshot_valid_until_ms
                    || request.observed_at_ms < request.snapshot_observed_at_ms || request.observed_at_ms > now_ms
                    || now_ms > request.lease_until_ms
                {
                    violations.insert(format!("stale-request-awaiting-recheck:{}", request.reservation_id));
                }
                if !pending_keys.insert((request.participant_id.clone(), request.idempotency_key.clone())) {
                    violations.insert(format!("idempotency-double-count:{}", request.idempotency_key));
                }
                pending_by_resource.entry((request.hub_id.clone(), request.sku_id.clone()))
                    .or_default().push(request);
            }
            ReconciliationDisposition::DuplicateOf { primary_id } => {
                let primary = request_by_reservation_id.get(primary_id).copied()
                    .filter(|candidate| candidate.participant_id == request.participant_id);
                match primary {
                    Some(candidate) if candidate.reservation_id != request.reservation_id
                        && candidate.idempotency_key == request.idempotency_key
                        && candidate.hub_id == request.hub_id && candidate.sku_id == request.sku_id
                        && candidate.quantity == request.quantity
                        && candidate.snapshot_revision == request.snapshot_revision
                        && candidate.snapshot_available_units == request.snapshot_available_units
                        && candidate.snapshot_observed_at_ms == request.snapshot_observed_at_ms
                        && candidate.snapshot_valid_until_ms == request.snapshot_valid_until_ms
                        && candidate.observed_at_ms == request.observed_at_ms
                        && candidate.lease_until_ms == request.lease_until_ms
                        && decision_by_id.get(&(
                            candidate.participant_id.clone(),
                            candidate.node_id.clone(),
                            candidate.reservation_id.clone(),
                        )).is_some_and(|primary_decision| !matches!(
                            &primary_decision.disposition,
                            ReconciliationDisposition::DuplicateOf { .. }
                        )) => {},
                    _ => { violations.insert(format!("invalid-idempotency-alias:{}", request.reservation_id)); }
                }
            }
            ReconciliationDisposition::UnresolvedConflict { reason, contenders } => {
                let mut sorted = contenders.clone();
                sorted.sort();
                sorted.dedup();
                if sorted.as_slice() != contenders.as_slice() || !contenders.contains(&request.reservation_id) {
                    violations.insert(format!("malformed-conflict-set:{}", request.reservation_id));
                }
                for contender in contenders {
                    let matching = request_by_reservation_id.get(contender).and_then(|candidate| {
                        let candidate_identity = (
                            candidate.participant_id.clone(),
                            candidate.node_id.clone(),
                            candidate.reservation_id.clone(),
                        );
                        decision_by_id.get(&candidate_identity)
                    }).is_some_and(|other| matches!(
                        &other.disposition,
                        ReconciliationDisposition::UnresolvedConflict {
                            reason: other_reason, contenders: other_contenders
                        } if other_reason == reason && other_contenders == contenders
                    ));
                    if !matching { violations.insert(format!("inconsistent-conflict-set:{contender}")); }
                }

                // A conflict set must exhaustively name all non-aliased requests for this
                // resource which the output classifies as unresolved or awaiting recheck.
                // Otherwise an omitted request could be treated as a separate, non-conflicting
                // recheck and escape the common conflict decision.
                let resource_key = (request.hub_id.clone(), request.sku_id.clone());
                let resource_requests = requests_by_resource.get(&resource_key)
                    .map(Vec::as_slice)
                    .unwrap_or(&[]);
                let mut expected_contenders = resource_requests.iter().filter_map(|candidate| {
                    let candidate_identity = (
                        candidate.participant_id.clone(),
                        candidate.node_id.clone(),
                        candidate.reservation_id.clone(),
                    );
                    let candidate_decision = decision_by_id.get(&candidate_identity)?;
                    match &candidate_decision.disposition {
                        ReconciliationDisposition::AwaitingAuthoritativeRecheck
                        | ReconciliationDisposition::UnresolvedConflict { .. } => {
                            Some(candidate.reservation_id.clone())
                        }
                        ReconciliationDisposition::Rejected { .. }
                        | ReconciliationDisposition::DuplicateOf { .. } => None,
                    }
                }).collect::<Vec<_>>();
                expected_contenders.sort();
                expected_contenders.dedup();
                let mut declared_contenders = contenders.clone();
                declared_contenders.sort();
                declared_contenders.dedup();
                if expected_contenders != declared_contenders {
                    violations.insert(format!("non-exhaustive-conflict-set:{}", request.reservation_id));
                }

                let rows = contenders.iter()
                    .filter_map(|reservation_id| request_by_reservation_id.get(reservation_id).copied())
                    .collect::<Vec<_>>();
                if rows.len() != contenders.len() {
                    violations.insert(format!("conflict-input-cardinality-mismatch:{}", request.reservation_id));
                }
                if let Some(first) = rows.first() {
                    if rows.iter().any(|candidate| candidate.hub_id != first.hub_id
                        || candidate.sku_id != first.sku_id)
                    {
                        violations.insert(format!("conflict-spans-multiple-resources:{}", request.reservation_id));
                    }
                    let same_snapshot = rows.iter().all(|candidate|
                        candidate.snapshot_revision == first.snapshot_revision
                            && candidate.snapshot_available_units == first.snapshot_available_units
                            && candidate.snapshot_observed_at_ms == first.snapshot_observed_at_ms
                            && candidate.snapshot_valid_until_ms == first.snapshot_valid_until_ms);
                    let aggregate = rows.iter().try_fold(0_u64, |sum, candidate|
                        sum.checked_add(candidate.quantity));

                    match *reason {
                        "oversubscribed-offline-snapshot" => {
                            if !same_snapshot || !matches!(aggregate,
                                Some(total) if total > first.snapshot_available_units)
                            {
                                violations.insert(format!("unjustified-oversubscription-conflict:{}", request.reservation_id));
                            }
                        }
                        "quantity-overflow" => {
                            if !same_snapshot || aggregate.is_some() {
                                violations.insert(format!("unjustified-overflow-conflict:{}", request.reservation_id));
                            }
                        }
                        "snapshot-context-disagreement" => {
                            if same_snapshot {
                                violations.insert(format!("unjustified-snapshot-disagreement:{}", request.reservation_id));
                            }
                        }
                        _ => {
                            violations.insert(format!("unknown-conflict-reason:{}", request.reservation_id));
                        }
                    }
                }
            }
            ReconciliationDisposition::Rejected { .. } => {}
        }
    }

    for (resource, candidates) in pending_by_resource {
        let Some(first) = candidates.first() else { continue };
        if candidates.iter().any(|candidate| candidate.snapshot_revision != first.snapshot_revision
            || candidate.snapshot_available_units != first.snapshot_available_units
            || candidate.snapshot_observed_at_ms != first.snapshot_observed_at_ms
            || candidate.snapshot_valid_until_ms != first.snapshot_valid_until_ms)
        {
            violations.insert(format!("pending-snapshot-disagreement:{}:{}", resource.0, resource.1));
        }
        match candidates.iter().try_fold(0_u64, |sum, candidate| sum.checked_add(candidate.quantity)) {
            None => { violations.insert(format!("pending-quantity-overflow:{}:{}", resource.0, resource.1)); }
            Some(total) if total > first.snapshot_available_units => {
                violations.insert(format!("pending-overcommits-snapshot:{}:{}", resource.0, resource.1));
            }
            Some(_) => {}
        }
    }
    violations.into_iter().collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn offline(participant: &str, node: &str, id: &str, key: &str, quantity: u64, available: u64) -> OfflineReservation {
        OfflineReservation {
            participant_id: participant.into(), node_id: node.into(), reservation_id: id.into(),
            idempotency_key: key.into(), hub_id: "coop-hub-01".into(), sku_id: "rice-25kg".into(),
            quantity, snapshot_revision: 12, snapshot_available_units: available,
            snapshot_observed_at_ms: 100, snapshot_valid_until_ms: 200,
            observed_at_ms: 110, lease_until_ms: 180,
        }
    }

    #[test]
    fn canonical_reconciliation_escapes_nested_list_delimiters_and_controls() {
        let decisions = vec![ReconciliationDecision {
            participant_id: "coop,a\t\0".into(),
            node_id: "edge-one".into(),
            reservation_id: "reserve,one".into(),
            disposition: ReconciliationDisposition::UnresolvedConflict {
                reason: "snapshot,conflict",
                contenders: vec!["reserve,one".into(), "reserve-two".into()],
            },
        }];
        let canonical = canonical_reconciliation(&decisions);
        assert!(canonical.contains("coop%2Ca%09%00"));
        assert!(canonical.contains("reserve%2Cone"));
        assert!(canonical.contains("UNRESOLVED_CONFLICT:snapshot%2Cconflict:reserve%2Cone,reserve-two"));
    }

    #[test]
    fn split_brain_last_unit_becomes_symmetric_conflict() {
        let requests = vec![
            offline("coop-a", "edge-a", "reserve-a", "op-a", 1, 1),
            offline("coop-b", "edge-b", "reserve-b", "op-b", 1, 1),
        ];
        let decisions = reconcile_offline_reservations(&requests, 120);
        assert!(decisions.iter().all(|decision| matches!(&decision.disposition,
            ReconciliationDisposition::UnresolvedConflict {
                reason: "oversubscribed-offline-snapshot", contenders
            } if contenders == &vec!["reserve-a".to_owned(), "reserve-b".to_owned()]
        )));
        assert!(verify_reconciliation(&requests, 120, &decisions).is_empty());
    }

    #[test]
    fn fitting_partition_batch_still_needs_authoritative_recheck() {
        let requests = vec![
            offline("coop-a", "edge-a", "reserve-a", "op-a", 1, 4),
            offline("coop-b", "edge-b", "reserve-b", "op-b", 2, 4),
        ];
        let decisions = reconcile_offline_reservations(&requests, 120);
        assert!(decisions.iter().all(|decision|
            decision.disposition == ReconciliationDisposition::AwaitingAuthoritativeRecheck));
        assert!(verify_reconciliation(&requests, 120, &decisions).is_empty());
    }

    #[test]
    fn stale_snapshot_or_expired_lease_is_rejected() {
        let mut stale = offline("coop-a", "edge-a", "reserve-a", "op-a", 1, 4);
        stale.snapshot_valid_until_ms = 119;
        let mut expired = offline("coop-b", "edge-b", "reserve-b", "op-b", 1, 4);
        expired.lease_until_ms = 119;
        let requests = vec![stale, expired];
        let decisions = reconcile_offline_reservations(&requests, 120);
        assert!(decisions.iter().all(|decision| matches!(&decision.disposition,
            ReconciliationDisposition::Rejected { .. })));
        assert!(verify_reconciliation(&requests, 120, &decisions).is_empty());
    }

    #[test]
    fn different_snapshot_revisions_remain_unresolved() {
        let a = offline("coop-a", "edge-a", "reserve-a", "op-a", 1, 4);
        let mut b = offline("coop-b", "edge-b", "reserve-b", "op-b", 1, 4);
        b.snapshot_revision += 1;
        let requests = vec![a, b];
        let decisions = reconcile_offline_reservations(&requests, 120);
        assert!(decisions.iter().all(|decision| matches!(&decision.disposition,
            ReconciliationDisposition::UnresolvedConflict {
                reason: "snapshot-context-disagreement", ..
            })));
        assert!(verify_reconciliation(&requests, 120, &decisions).is_empty());
    }

    #[test]
    fn local_idempotent_replay_from_two_edges_counts_once() {
        let first = offline("coop-a", "edge-a", "reserve-a", "same-op", 1, 4);
        let mut replay = first.clone();
        replay.node_id = "edge-b".into();
        replay.reservation_id = "reserve-replay".into();
        let requests = vec![first, replay];
        let decisions = reconcile_offline_reservations(&requests, 120);
        assert_eq!(decisions.iter().filter(|decision|
            decision.disposition == ReconciliationDisposition::AwaitingAuthoritativeRecheck).count(), 1);
        assert!(decisions.iter().any(|decision| matches!(&decision.disposition,
            ReconciliationDisposition::DuplicateOf { primary_id } if primary_id == "reserve-a")));
        assert!(verify_reconciliation(&requests, 120, &decisions).is_empty());
    }

    #[test]
    fn changed_payload_under_same_local_idempotency_key_rejects_all() {
        let a = offline("coop-a", "edge-a", "reserve-a", "same-op", 1, 4);
        let b = offline("coop-a", "edge-b", "reserve-b", "same-op", 2, 4);
        let requests = vec![a, b];
        let decisions = reconcile_offline_reservations(&requests, 120);
        assert!(decisions.iter().all(|decision| matches!(&decision.disposition,
            ReconciliationDisposition::Rejected { reason: "idempotency-key-payload-conflict" })));
        assert!(verify_reconciliation(&requests, 120, &decisions).is_empty());
    }

    #[test]
    fn stale_payload_cannot_hide_conflicting_idempotency_key_reuse() {
        let mut stale = offline("coop-a", "edge-a", "reserve-stale", "same-op", 1, 4);
        stale.snapshot_valid_until_ms = 119;
        let valid_conflicting_payload = offline("coop-a", "edge-b", "reserve-valid", "same-op", 2, 4);
        let requests = vec![stale, valid_conflicting_payload];
        let decisions = reconcile_offline_reservations(&requests, 120);
        assert!(decisions.iter().all(|decision| matches!(
            &decision.disposition,
            ReconciliationDisposition::Rejected {
                reason: "idempotency-key-payload-conflict"
            }
        )));
        assert!(!decisions.iter().any(|decision|
            decision.disposition == ReconciliationDisposition::AwaitingAuthoritativeRecheck));
        assert!(verify_reconciliation(&requests, 120, &decisions).is_empty());
    }

    #[test]
    fn unrelated_participants_can_use_same_idempotency_key() {
        let requests = vec![
            offline("coop-a", "edge-a", "reserve-a", "generic-key", 1, 4),
            offline("coop-b", "edge-b", "reserve-b", "generic-key", 1, 4),
        ];
        let decisions = reconcile_offline_reservations(&requests, 120);
        assert!(decisions.iter().all(|decision|
            decision.disposition == ReconciliationDisposition::AwaitingAuthoritativeRecheck));
        assert!(verify_reconciliation(&requests, 120, &decisions).is_empty());
    }

    #[test]
    fn permutation_of_input_does_not_change_canonical_output() {
        let a = offline("coop-a", "edge-a", "reserve-a", "op-a", 1, 4);
        let b = offline("coop-b", "edge-b", "reserve-b", "op-b", 1, 4);
        let c = offline("coop-c", "edge-c", "reserve-c", "op-c", 1, 4);
        let first = reconcile_offline_reservations(&[a.clone(), b.clone(), c.clone()], 120);
        let second = reconcile_offline_reservations(&[c, a, b], 120);
        assert_eq!(canonical_reconciliation(&first), canonical_reconciliation(&second));
    }

    #[test]
    fn quantity_overflow_becomes_explicit_conflict() {
        let requests = vec![
            offline("coop-a", "edge-a", "reserve-a", "op-a", u64::MAX, u64::MAX),
            offline("coop-b", "edge-b", "reserve-b", "op-b", 1, u64::MAX),
        ];
        let decisions = reconcile_offline_reservations(&requests, 120);
        assert!(decisions.iter().all(|decision| matches!(&decision.disposition,
            ReconciliationDisposition::UnresolvedConflict { reason: "quantity-overflow", .. })));
        assert!(verify_reconciliation(&requests, 120, &decisions).is_empty());
    }

    #[test]
    fn duplicate_reservation_ids_are_rejected_and_reported_by_verifier() {
        let a = offline("coop-a", "edge-a", "same-id", "op-a", 1, 4);
        let b = offline("coop-b", "edge-b", "same-id", "op-b", 1, 4);
        let requests = vec![a, b];
        let decisions = reconcile_offline_reservations(&requests, 120);
        assert!(decisions.iter().all(|decision| matches!(&decision.disposition,
            ReconciliationDisposition::Rejected { reason: "duplicate-reservation-id" })));
        assert!(verify_reconciliation(&requests, 120, &decisions)
            .iter().any(|violation| violation == "duplicate-reservation-id:same-id"));
    }

    #[test]
    fn independent_verifier_detects_omitted_conflict_contender() {
        let requests = vec![
            offline("coop-a", "edge-a", "reserve-a", "op-a", 1, 1),
            offline("coop-b", "edge-b", "reserve-b", "op-b", 1, 1),
            offline("coop-c", "edge-c", "reserve-c", "op-c", 1, 1),
        ];
        let partial_set = vec!["reserve-a".to_owned(), "reserve-b".to_owned()];
        let injected = vec![
            ReconciliationDecision {
                participant_id: "coop-a".into(), node_id: "edge-a".into(), reservation_id: "reserve-a".into(),
                disposition: ReconciliationDisposition::UnresolvedConflict {
                    reason: "oversubscribed-offline-snapshot", contenders: partial_set.clone(),
                },
            },
            ReconciliationDecision {
                participant_id: "coop-b".into(), node_id: "edge-b".into(), reservation_id: "reserve-b".into(),
                disposition: ReconciliationDisposition::UnresolvedConflict {
                    reason: "oversubscribed-offline-snapshot", contenders: partial_set,
                },
            },
            ReconciliationDecision {
                participant_id: "coop-c".into(), node_id: "edge-c".into(), reservation_id: "reserve-c".into(),
                disposition: ReconciliationDisposition::AwaitingAuthoritativeRecheck,
            },
        ];
        assert!(verify_reconciliation(&requests, 120, &injected)
            .iter().any(|violation| violation == "non-exhaustive-conflict-set:reserve-a"));
    }

    #[test]
    fn independent_verifier_detects_injected_offline_overcommit() {
        let requests = vec![
            offline("coop-a", "edge-a", "reserve-a", "op-a", 1, 1),
            offline("coop-b", "edge-b", "reserve-b", "op-b", 1, 1),
        ];
        let injected = requests.iter().map(|request| ReconciliationDecision {
            participant_id: request.participant_id.clone(), node_id: request.node_id.clone(),
            reservation_id: request.reservation_id.clone(),
            disposition: ReconciliationDisposition::AwaitingAuthoritativeRecheck,
        }).collect::<Vec<_>>();
        assert!(verify_reconciliation(&requests, 120, &injected)
            .iter().any(|violation| violation == "pending-overcommits-snapshot:coop-hub-01:rice-25kg"));
    }
}
