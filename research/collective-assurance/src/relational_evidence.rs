//! Rights-aware relational evidence refinement before response generation.
//!
//! This pure transformation composes with the existing Mycelix evidence ledger
//! and response-constraint binder. It selects related, already authorized,
//! verified evidence under a fixed recipient/purpose/history cutoff. It does
//! not create a memory store, grant disclosure permission, authenticate the
//! referenced upstream receipts, or generate natural-language responses.

use std::collections::{BTreeMap, BTreeSet, VecDeque};

use super::{Applicability, EvidenceStatus};

fn nonempty(value: &str) -> bool {
    !value.trim().is_empty()
}

fn valid_digest(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn unique_nonempty(values: &[String]) -> bool {
    let mut seen = BTreeSet::new();
    values
        .iter()
        .all(|value| nonempty(value) && seen.insert(value.as_str()))
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RelationalDisclosurePermission {
    Shareable,
    Scoped,
    WithheldByRights,
    Unknown,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RelationalVerification {
    Verified,
    Rejected,
    Unknown,
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum EvidenceRelationKind {
    SameSubject,
    SameEvent,
    SameDecision,
    Supports,
    Contradicts,
    Supersedes,
    Updates,
    RelatedContext,
}

impl EvidenceRelationKind {
    fn is_symmetric(self) -> bool {
        matches!(
            self,
            Self::SameSubject
                | Self::SameEvent
                | Self::SameDecision
                | Self::Contradicts
                | Self::RelatedContext
        )
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RelationalEvidenceRecord {
    pub evidence_id: String,
    /// Request that established the disclosure/exposure context for this record.
    pub request_id: String,
    /// Evidence origin is deliberately distinct from the person/agent holding it.
    pub source_origin_id: String,
    pub source_holder_id: String,
    pub source_message_id: String,
    pub payload_digest: String,
    pub source_registry_receipt_digest: String,
    pub disclosure_receipt_digest: String,
    pub source_timestamp_ms: u64,
    pub recorded_at_ms: u64,
    pub subject_ids: Vec<String>,
    pub event_ids: Vec<String>,
    pub decision_ids: Vec<String>,
    pub permission: RelationalDisclosurePermission,
    pub permitted_recipient_ids: Vec<String>,
    pub permitted_purpose_ids: Vec<String>,
    pub verification: RelationalVerification,
    /// Kept intact so the existing response-constraint binder can distinguish
    /// current, superseded, disputed, expired and unknown material.
    pub status: EvidenceStatus,
    pub applicability: Applicability,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct EvidenceRelationEdge {
    pub from_evidence_id: String,
    pub to_evidence_id: String,
    pub relation: EvidenceRelationKind,
    pub verification: RelationalVerification,
    /// Receipt from the independent relation/indexing process.
    pub relation_receipt_digest: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RelationalEvidenceScope {
    pub request_id: String,
    pub recipient_id: String,
    pub purpose_id: String,
    /// Neither evidence created nor evidence recorded after this time is visible.
    pub history_cutoff_ms: u64,
    pub seed_evidence_ids: Vec<String>,
    pub max_relation_hops: u8,
    pub max_selected_evidence: usize,
    /// A retrieval service must attest that this candidate pool is complete
    /// for the declared request, recipient, purpose and history cutoff.
    pub candidate_pool_complete: bool,
    pub candidate_pool_receipt_digest: Option<String>,
    /// Relation-index completeness is separate from candidate-pool completeness.
    pub relation_index_complete: bool,
    pub relation_index_receipt_digest: Option<String>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RelationalEvidenceStatus {
    Pass,
    Fail,
    Indeterminate,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RelationalEvidenceReport {
    pub status: RelationalEvidenceStatus,
    /// Stable, sorted evidence IDs selected for the response-generation stage.
    pub selected_evidence_ids: Vec<String>,
    /// Selected IDs that were not in the original retrieval seed set.
    pub added_evidence_ids: Vec<String>,
    /// Unique evidence origins; repeated holders of one origin do not inflate this.
    pub selected_source_origin_ids: Vec<String>,
    /// Distinct holders are reported separately from source origins.
    pub selected_source_holder_ids: Vec<String>,
    /// Only verified relations whose endpoints both survived authorization/scope.
    pub selected_relation_edges: Vec<EvidenceRelationEdge>,
    pub unverified_relation_count: usize,
    pub eligible_candidate_count: usize,
    pub excluded_candidate_count: usize,
    pub reasons: Vec<String>,
}

fn empty_report(
    status: RelationalEvidenceStatus,
    reasons: Vec<String>,
    eligible_candidate_count: usize,
    excluded_candidate_count: usize,
    unverified_relation_count: usize,
) -> RelationalEvidenceReport {
    RelationalEvidenceReport {
        status,
        selected_evidence_ids: Vec::new(),
        added_evidence_ids: Vec::new(),
        selected_source_origin_ids: Vec::new(),
        selected_source_holder_ids: Vec::new(),
        selected_relation_edges: Vec::new(),
        unverified_relation_count,
        eligible_candidate_count,
        excluded_candidate_count,
        reasons,
    }
}

fn candidate_is_eligible(
    record: &RelationalEvidenceRecord,
    scope: &RelationalEvidenceScope,
) -> bool {
    matches!(
        record.permission,
        RelationalDisclosurePermission::Shareable | RelationalDisclosurePermission::Scoped
    ) && record.request_id == scope.request_id && record
        .permitted_recipient_ids
        .iter()
        .any(|recipient| recipient == &scope.recipient_id)
        && record
            .permitted_purpose_ids
            .iter()
            .any(|purpose| purpose == &scope.purpose_id)
        && record.verification == RelationalVerification::Verified
        && record.source_timestamp_ms <= scope.history_cutoff_ms
        && record.recorded_at_ms <= scope.history_cutoff_ms
}

fn seed_disposition(
    record: Option<&RelationalEvidenceRecord>,
    scope: &RelationalEvidenceScope,
) -> Option<(RelationalEvidenceStatus, String)> {
    let Some(record) = record else {
        return Some((
            RelationalEvidenceStatus::Indeterminate,
            "a retrieval seed is absent from the complete candidate pool".into(),
        ));
    };

    if record.request_id != scope.request_id {
        return Some((
            RelationalEvidenceStatus::Fail,
            "retrieval seed belongs to a different request context".into(),
        ));
    }

    match record.permission {
        RelationalDisclosurePermission::WithheldByRights => {
            return Some((
                RelationalEvidenceStatus::Fail,
                "a rights-withheld item was present in the retrieval seed set".into(),
            ));
        }
        RelationalDisclosurePermission::Unknown => {
            return Some((
                RelationalEvidenceStatus::Indeterminate,
                "a retrieval seed has unknown disclosure permission".into(),
            ));
        }
        RelationalDisclosurePermission::Shareable | RelationalDisclosurePermission::Scoped => {}
    }

    if !record
        .permitted_recipient_ids
        .iter()
        .any(|recipient| recipient == &scope.recipient_id)
        || !record
            .permitted_purpose_ids
            .iter()
            .any(|purpose| purpose == &scope.purpose_id)
    {
        return Some((
            RelationalEvidenceStatus::Fail,
            "a retrieval seed is not authorized for the declared recipient and purpose".into(),
        ));
    }

    match record.verification {
        RelationalVerification::Rejected => {
            return Some((
                RelationalEvidenceStatus::Fail,
                "a rejected evidence item was present in the retrieval seed set".into(),
            ));
        }
        RelationalVerification::Unknown => {
            return Some((
                RelationalEvidenceStatus::Indeterminate,
                "a retrieval seed has unknown verification status".into(),
            ));
        }
        RelationalVerification::Verified => {}
    }

    if record.source_timestamp_ms > scope.history_cutoff_ms
        || record.recorded_at_ms > scope.history_cutoff_ms
    {
        return Some((
            RelationalEvidenceStatus::Fail,
            "a retrieval seed violates the frozen history cutoff".into(),
        ));
    }

    for (label, digest) in [
        ("payload", record.payload_digest.as_str()),
        ("source registry receipt", record.source_registry_receipt_digest.as_str()),
        ("disclosure receipt", record.disclosure_receipt_digest.as_str()),
    ] {
        if !valid_digest(digest) {
            return Some((
                RelationalEvidenceStatus::Fail,
                format!("retrieval seed {label} digest is malformed"),
            ));
        }
    }
    None
}

/// Refines a retrieved seed set with verified related evidence.
///
/// The function is deliberately deterministic and bounded. It traverses only
/// independently verified relation edges, and only through candidate items
/// authorized for the exact recipient/purpose and available before the frozen
/// history cutoff. Rights-withheld, unauthorized, rejected, future and unknown
/// related candidates are never selected. A seed that itself violates those
/// rules is a qualification failure or indeterminate outcome.
///
/// No partial selection is returned if the candidate pool is incomplete, the
/// relation index is incomplete, or the requested traversal exceeds its bounds.
pub fn refine_relational_evidence(
    scope: &RelationalEvidenceScope,
    candidates: &[RelationalEvidenceRecord],
    relations: &[EvidenceRelationEdge],
) -> RelationalEvidenceReport {
    let mut errors = Vec::new();
    let mut indeterminate_reasons = Vec::new();

    if !nonempty(&scope.request_id)
        || !nonempty(&scope.recipient_id)
        || !nonempty(&scope.purpose_id)
    {
        errors.push("request, recipient and purpose identities must be non-empty".into());
    }
    if scope.seed_evidence_ids.is_empty() || !unique_nonempty(&scope.seed_evidence_ids) {
        errors.push("retrieval seed IDs must be non-empty and unique".into());
    }
    if !(1..=8).contains(&scope.max_relation_hops) {
        errors.push("maximum relation hops must be between 1 and 8".into());
    }
    if scope.max_selected_evidence == 0 || scope.max_selected_evidence > 256 {
        errors.push("maximum selected evidence must be between 1 and 256".into());
    }
    if !scope.candidate_pool_complete {
        indeterminate_reasons.push(
            "candidate evidence pool is not attested complete for this request/scope/cutoff".into(),
        );
    }
    match scope.candidate_pool_receipt_digest.as_deref() {
        Some(digest) if valid_digest(digest) => {}
        Some(_) => errors.push("candidate-pool receipt digest is malformed".into()),
        None => indeterminate_reasons.push("candidate-pool receipt is missing".into()),
    }
    if !scope.relation_index_complete {
        indeterminate_reasons.push(
            "relational index is not attested complete for this request/scope/cutoff".into(),
        );
    }
    match scope.relation_index_receipt_digest.as_deref() {
        Some(digest) if valid_digest(digest) => {}
        Some(_) => errors.push("relation-index receipt digest is malformed".into()),
        None => indeterminate_reasons.push("relation-index receipt is missing".into()),
    }

    let mut by_id: BTreeMap<&str, &RelationalEvidenceRecord> = BTreeMap::new();
    for candidate in candidates {
        if !nonempty(&candidate.evidence_id)
            || !nonempty(&candidate.request_id)
            || !nonempty(&candidate.source_origin_id)
            || !nonempty(&candidate.source_holder_id)
            || !nonempty(&candidate.source_message_id)
            || !unique_nonempty(&candidate.subject_ids)
            || !unique_nonempty(&candidate.event_ids)
            || !unique_nonempty(&candidate.decision_ids)
            || !unique_nonempty(&candidate.permitted_recipient_ids)
            || !unique_nonempty(&candidate.permitted_purpose_ids)
        {
            errors.push(format!(
                "candidate evidence {} has invalid identity or duplicate/empty metadata",
                candidate.evidence_id
            ));
        }
        if by_id
            .insert(candidate.evidence_id.as_str(), candidate)
            .is_some()
        {
            errors.push(format!(
                "duplicate candidate evidence ID {}",
                candidate.evidence_id
            ));
        }

        // Any candidate that could be selected must carry complete source,
        // payload and disclosure references. Non-eligible records are not
        // selected, but malformed identifiers remain a registry-integrity error.
        if matches!(
            candidate.permission,
            RelationalDisclosurePermission::Shareable | RelationalDisclosurePermission::Scoped
        ) && candidate.verification == RelationalVerification::Verified
            && candidate
                .permitted_recipient_ids
                .iter()
                .any(|id| id == &scope.recipient_id)
            && candidate
                .permitted_purpose_ids
                .iter()
                .any(|id| id == &scope.purpose_id)
            && candidate.source_timestamp_ms <= scope.history_cutoff_ms
            && candidate.recorded_at_ms <= scope.history_cutoff_ms
        {
            for (label, digest) in [
                ("payload", candidate.payload_digest.as_str()),
                ("source registry receipt", candidate.source_registry_receipt_digest.as_str()),
                ("disclosure receipt", candidate.disclosure_receipt_digest.as_str()),
            ] {
                if !valid_digest(digest) {
                    errors.push(format!(
                        "eligible candidate {} has malformed {label} digest",
                        candidate.evidence_id
                    ));
                }
            }
        }
    }

    if !errors.is_empty() {
        return empty_report(
            RelationalEvidenceStatus::Fail,
            errors,
            0,
            0,
            0,
        );
    }

    let eligible_ids: BTreeSet<&str> = candidates
        .iter()
        .filter(|candidate| candidate_is_eligible(candidate, scope))
        .map(|candidate| candidate.evidence_id.as_str())
        .collect();
    let eligible_candidate_count = eligible_ids.len();
    let excluded_candidate_count = candidates.len().saturating_sub(eligible_candidate_count);

    for seed_id in &scope.seed_evidence_ids {
        if let Some((status, reason)) = seed_disposition(by_id.get(seed_id.as_str()).copied(), scope) {
            match status {
                RelationalEvidenceStatus::Fail => {
                    errors.push(format!("seed {seed_id}: {reason}"));
                }
                RelationalEvidenceStatus::Indeterminate => {
                    indeterminate_reasons.push(format!("seed {seed_id}: {reason}"));
                }
                RelationalEvidenceStatus::Pass => {}
            }
        }
    }

    if !errors.is_empty() {
        return empty_report(
            RelationalEvidenceStatus::Fail,
            errors,
            eligible_candidate_count,
            excluded_candidate_count,
            0,
        );
    }
    if !indeterminate_reasons.is_empty() {
        return empty_report(
            RelationalEvidenceStatus::Indeterminate,
            indeterminate_reasons,
            eligible_candidate_count,
            excluded_candidate_count,
            0,
        );
    }

    let mut seen_edges = BTreeSet::new();
    let mut adjacency: BTreeMap<String, Vec<(String, usize)>> = BTreeMap::new();
    let mut unverified_relation_count = 0usize;

    for (index, edge) in relations.iter().enumerate() {
        if !nonempty(&edge.from_evidence_id)
            || !nonempty(&edge.to_evidence_id)
            || edge.from_evidence_id == edge.to_evidence_id
            || !valid_digest(&edge.relation_receipt_digest)
        {
            errors.push("relation edge has invalid endpoints or receipt digest".into());
            continue;
        }
        if !by_id.contains_key(edge.from_evidence_id.as_str())
            || !by_id.contains_key(edge.to_evidence_id.as_str())
        {
            errors.push("relation edge references an unknown evidence ID".into());
            continue;
        }
        let (canonical_from, canonical_to) = if edge.relation.is_symmetric()
            && edge.from_evidence_id > edge.to_evidence_id
        {
            (edge.to_evidence_id.as_str(), edge.from_evidence_id.as_str())
        } else {
            (edge.from_evidence_id.as_str(), edge.to_evidence_id.as_str())
        };
        if !seen_edges.insert((canonical_from, canonical_to, edge.relation)) {
            errors.push("duplicate relation edge".into());
            continue;
        }
        if edge.verification != RelationalVerification::Verified {
            unverified_relation_count += 1;
            continue;
        }
        if !eligible_ids.contains(edge.from_evidence_id.as_str())
            || !eligible_ids.contains(edge.to_evidence_id.as_str())
        {
            // The related item may be withheld, unknown, outside recipient scope,
            // rejected or beyond cutoff. It is excluded without exposing its ID.
            continue;
        }
        adjacency
            .entry(edge.from_evidence_id.clone())
            .or_default()
            .push((edge.to_evidence_id.clone(), index));
        adjacency
            .entry(edge.to_evidence_id.clone())
            .or_default()
            .push((edge.from_evidence_id.clone(), index));
    }

    if !errors.is_empty() {
        return empty_report(
            RelationalEvidenceStatus::Fail,
            errors,
            eligible_candidate_count,
            excluded_candidate_count,
            unverified_relation_count,
        );
    }

    let mut selected: BTreeSet<String> =
        scope.seed_evidence_ids.iter().cloned().collect();
    let mut queue: VecDeque<(String, u8)> = scope
        .seed_evidence_ids
        .iter()
        .cloned()
        .map(|seed| (seed, 0))
        .collect();
    let mut truncated = false;

    while let Some((current, depth)) = queue.pop_front() {
        let neighbors = adjacency.get(&current).cloned().unwrap_or_default();
        for (neighbor, _edge_index) in neighbors {
            if selected.contains(&neighbor) {
                continue;
            }
            if depth >= scope.max_relation_hops
                || selected.len() >= scope.max_selected_evidence
            {
                truncated = true;
                continue;
            }
            selected.insert(neighbor.clone());
            queue.push_back((neighbor, depth + 1));
        }
    }

    if truncated {
        return empty_report(
            RelationalEvidenceStatus::Indeterminate,
            vec![
                "related evidence closure exceeds the preregistered hop or item bound; no partial evidence bundle is emitted".into(),
            ],
            eligible_candidate_count,
            excluded_candidate_count,
            unverified_relation_count,
        );
    }

    let selected_set: BTreeSet<&str> = selected.iter().map(String::as_str).collect();
    let mut selected_edges = Vec::new();
    let mut origins = BTreeSet::new();
    let mut holders = BTreeSet::new();
    for id in &selected {
        if let Some(record) = by_id.get(id.as_str()) {
            origins.insert(record.source_origin_id.clone());
            holders.insert(record.source_holder_id.clone());
        }
    }
    for edge in relations {
        if edge.verification == RelationalVerification::Verified
            && selected_set.contains(edge.from_evidence_id.as_str())
            && selected_set.contains(edge.to_evidence_id.as_str())
            && eligible_ids.contains(edge.from_evidence_id.as_str())
            && eligible_ids.contains(edge.to_evidence_id.as_str())
        {
            selected_edges.push(edge.clone());
        }
    }
    selected_edges.sort_by(|left, right| {
        (
            left.from_evidence_id.as_str(),
            left.to_evidence_id.as_str(),
            left.relation,
        )
            .cmp(&(
                right.from_evidence_id.as_str(),
                right.to_evidence_id.as_str(),
                right.relation,
            ))
    });

    let seed_set: BTreeSet<&str> = scope
        .seed_evidence_ids
        .iter()
        .map(String::as_str)
        .collect();
    let added_evidence_ids = selected
        .iter()
        .filter(|id| !seed_set.contains(id.as_str()))
        .cloned()
        .collect();

    RelationalEvidenceReport {
        status: RelationalEvidenceStatus::Pass,
        selected_evidence_ids: selected.into_iter().collect(),
        added_evidence_ids,
        selected_source_origin_ids: origins.into_iter().collect(),
        selected_source_holder_ids: holders.into_iter().collect(),
        selected_relation_edges: selected_edges,
        unverified_relation_count,
        eligible_candidate_count,
        excluded_candidate_count,
        reasons: vec![
            "selection includes only verified evidence authorized for the exact recipient and purpose before the frozen history cutoff".into(),
            "source origins are deduplicated independently from holders; repeated holders do not create independent corroboration".into(),
            "the result is a bounded evidence-selection bundle, not a natural-language answer or a truth judgment".into(),
        ],
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(ch: char) -> String {
        std::iter::repeat_n(ch, 64).collect()
    }

    fn record(id: &str, origin: &str, holder: &str) -> RelationalEvidenceRecord {
        RelationalEvidenceRecord {
            evidence_id: id.into(),
            request_id: "request-1".into(),
            source_origin_id: origin.into(),
            source_holder_id: holder.into(),
            source_message_id: format!("message-{id}"),
            payload_digest: digest('a'),
            source_registry_receipt_digest: digest('b'),
            disclosure_receipt_digest: digest('c'),
            source_timestamp_ms: 100,
            recorded_at_ms: 110,
            subject_ids: vec!["trip-date".into()],
            event_ids: vec!["trip-2026".into()],
            decision_ids: vec!["booking-1".into()],
            permission: RelationalDisclosurePermission::Scoped,
            permitted_recipient_ids: vec!["assistant-1".into()],
            permitted_purpose_ids: vec!["trip-planning".into()],
            verification: RelationalVerification::Verified,
            status: EvidenceStatus::Current,
            applicability: Applicability::Applicable,
        }
    }

    fn scope() -> RelationalEvidenceScope {
        RelationalEvidenceScope {
            request_id: "request-1".into(),
            recipient_id: "assistant-1".into(),
            purpose_id: "trip-planning".into(),
            history_cutoff_ms: 200,
            seed_evidence_ids: vec!["e1".into()],
            max_relation_hops: 3,
            max_selected_evidence: 8,
            candidate_pool_complete: true,
            candidate_pool_receipt_digest: Some(digest('d')),
            relation_index_complete: true,
            relation_index_receipt_digest: Some(digest('e')),
        }
    }

    fn edge(from: &str, to: &str, relation: EvidenceRelationKind) -> EvidenceRelationEdge {
        EvidenceRelationEdge {
            from_evidence_id: from.into(),
            to_evidence_id: to.into(),
            relation,
            verification: RelationalVerification::Verified,
            relation_receipt_digest: digest('f'),
        }
    }

    #[test]
    fn adds_verified_related_supporting_evidence() {
        let report = refine_relational_evidence(
            &scope(),
            &[record("e1", "origin-a", "holder-a"), record("e2", "origin-b", "holder-b")],
            &[edge("e1", "e2", EvidenceRelationKind::Supports)],
        );
        assert_eq!(report.status, RelationalEvidenceStatus::Pass);
        assert_eq!(report.selected_evidence_ids, vec!["e1", "e2"]);
        assert_eq!(report.added_evidence_ids, vec!["e2"]);
    }

    #[test]
    fn follows_verified_relations_transitively_within_hop_bound() {
        let report = refine_relational_evidence(
            &scope(),
            &[
                record("e1", "origin-a", "holder-a"),
                record("e2", "origin-b", "holder-b"),
                record("e3", "origin-c", "holder-c"),
            ],
            &[
                edge("e1", "e2", EvidenceRelationKind::SameEvent),
                edge("e2", "e3", EvidenceRelationKind::Updates),
            ],
        );
        assert_eq!(report.status, RelationalEvidenceStatus::Pass);
        assert_eq!(report.selected_evidence_ids.len(), 3);
    }

    #[test]
    fn preserves_conflicting_and_superseded_evidence_for_constraint_binding() {
        let mut conflict = record("e2", "origin-b", "holder-b");
        conflict.status = EvidenceStatus::Disputed;
        let mut old = record("e3", "origin-c", "holder-c");
        old.status = EvidenceStatus::Superseded;
        let report = refine_relational_evidence(
            &scope(),
            &[
                record("e1", "origin-a", "holder-a"),
                conflict,
                old,
            ],
            &[
                edge("e1", "e2", EvidenceRelationKind::Contradicts),
                edge("e1", "e3", EvidenceRelationKind::Supersedes),
            ],
        );
        assert_eq!(report.status, RelationalEvidenceStatus::Pass);
        assert_eq!(report.selected_evidence_ids.len(), 3);
    }

    #[test]
    fn repeated_holders_do_not_inflate_independent_source_origins() {
        let report = refine_relational_evidence(
            &scope(),
            &[
                record("e1", "same-origin", "holder-a"),
                record("e2", "same-origin", "holder-b"),
            ],
            &[edge("e1", "e2", EvidenceRelationKind::SameSubject)],
        );
        assert_eq!(report.status, RelationalEvidenceStatus::Pass);
        assert_eq!(report.selected_source_origin_ids, vec!["same-origin"]);
        assert_eq!(report.selected_source_holder_ids.len(), 2);
    }

    #[test]
    fn rights_withheld_related_candidate_is_not_added() {
        let mut withheld = record("e2", "origin-b", "holder-b");
        withheld.permission = RelationalDisclosurePermission::WithheldByRights;
        withheld.permitted_recipient_ids.clear();
        withheld.permitted_purpose_ids.clear();
        let report = refine_relational_evidence(
            &scope(),
            &[record("e1", "origin-a", "holder-a"), withheld],
            &[edge("e1", "e2", EvidenceRelationKind::Supports)],
        );
        assert_eq!(report.status, RelationalEvidenceStatus::Pass);
        assert_eq!(report.selected_evidence_ids, vec!["e1"]);
    }

    #[test]
    fn rights_withheld_seed_fails_closed() {
        let mut query = scope();
        query.seed_evidence_ids = vec!["e2".into()];
        let mut withheld = record("e2", "origin-b", "holder-b");
        withheld.permission = RelationalDisclosurePermission::WithheldByRights;
        let report = refine_relational_evidence(&query, &[withheld], &[]);
        assert_eq!(report.status, RelationalEvidenceStatus::Fail);
        assert!(report.selected_evidence_ids.is_empty());
    }


    #[test]
    fn seed_from_another_request_context_fails() {
        let mut item = record("e1", "origin-a", "holder-a");
        item.request_id = "other-request".into();
        let report = refine_relational_evidence(&scope(), &[item], &[]);
        assert_eq!(report.status, RelationalEvidenceStatus::Fail);
    }

    #[test]
    fn related_candidate_from_another_request_is_not_added() {
        let mut item = record("e2", "origin-b", "holder-b");
        item.request_id = "other-request".into();
        let report = refine_relational_evidence(
            &scope(),
            &[record("e1", "origin-a", "holder-a"), item],
            &[edge("e1", "e2", EvidenceRelationKind::Supports)],
        );
        assert_eq!(report.status, RelationalEvidenceStatus::Pass);
        assert_eq!(report.selected_evidence_ids, vec!["e1"]);
    }

    #[test]
    fn unknown_permission_seed_is_indeterminate() {
        let mut query = scope();
        let mut item = record("e1", "origin-a", "holder-a");
        item.permission = RelationalDisclosurePermission::Unknown;
        let report = refine_relational_evidence(&query, &[item], &[]);
        assert_eq!(report.status, RelationalEvidenceStatus::Indeterminate);
    }

    #[test]
    fn unauthorized_related_recipient_is_not_added() {
        let mut item = record("e2", "origin-b", "holder-b");
        item.permitted_recipient_ids = vec!["someone-else".into()];
        let report = refine_relational_evidence(
            &scope(),
            &[record("e1", "origin-a", "holder-a"), item],
            &[edge("e1", "e2", EvidenceRelationKind::SameDecision)],
        );
        assert_eq!(report.status, RelationalEvidenceStatus::Pass);
        assert_eq!(report.selected_evidence_ids, vec!["e1"]);
    }

    #[test]
    fn unauthorized_seed_fails() {
        let mut item = record("e1", "origin-a", "holder-a");
        item.permitted_recipient_ids = vec!["someone-else".into()];
        let report = refine_relational_evidence(&scope(), &[item], &[]);
        assert_eq!(report.status, RelationalEvidenceStatus::Fail);
    }

    #[test]
    fn rejected_verification_seed_fails() {
        let mut item = record("e1", "origin-a", "holder-a");
        item.verification = RelationalVerification::Rejected;
        let report = refine_relational_evidence(&scope(), &[item], &[]);
        assert_eq!(report.status, RelationalEvidenceStatus::Fail);
    }

    #[test]
    fn unknown_verification_seed_is_indeterminate() {
        let mut item = record("e1", "origin-a", "holder-a");
        item.verification = RelationalVerification::Unknown;
        let report = refine_relational_evidence(&scope(), &[item], &[]);
        assert_eq!(report.status, RelationalEvidenceStatus::Indeterminate);
    }

    #[test]
    fn future_related_evidence_is_excluded_by_history_cutoff() {
        let mut future = record("e2", "origin-b", "holder-b");
        future.source_timestamp_ms = 201;
        let report = refine_relational_evidence(
            &scope(),
            &[record("e1", "origin-a", "holder-a"), future],
            &[edge("e1", "e2", EvidenceRelationKind::Updates)],
        );
        assert_eq!(report.status, RelationalEvidenceStatus::Pass);
        assert_eq!(report.selected_evidence_ids, vec!["e1"]);
    }

    #[test]
    fn future_seed_fails_history_cutoff() {
        let mut item = record("e1", "origin-a", "holder-a");
        item.recorded_at_ms = 201;
        let report = refine_relational_evidence(&scope(), &[item], &[]);
        assert_eq!(report.status, RelationalEvidenceStatus::Fail);
    }

    #[test]
    fn incomplete_candidate_pool_is_indeterminate() {
        let mut query = scope();
        query.candidate_pool_complete = false;
        let report = refine_relational_evidence(&query, &[record("e1", "origin-a", "holder-a")], &[]);
        assert_eq!(report.status, RelationalEvidenceStatus::Indeterminate);
        assert!(report.selected_evidence_ids.is_empty());
    }

    #[test]
    fn missing_candidate_pool_receipt_is_indeterminate() {
        let mut query = scope();
        query.candidate_pool_receipt_digest = None;
        let report = refine_relational_evidence(&query, &[record("e1", "origin-a", "holder-a")], &[]);
        assert_eq!(report.status, RelationalEvidenceStatus::Indeterminate);
    }

    #[test]
    fn incomplete_relation_index_is_indeterminate() {
        let mut query = scope();
        query.relation_index_complete = false;
        let report = refine_relational_evidence(&query, &[record("e1", "origin-a", "holder-a")], &[]);
        assert_eq!(report.status, RelationalEvidenceStatus::Indeterminate);
    }

    #[test]
    fn missing_relation_receipt_is_indeterminate() {
        let mut query = scope();
        query.relation_index_receipt_digest = None;
        let report = refine_relational_evidence(&query, &[record("e1", "origin-a", "holder-a")], &[]);
        assert_eq!(report.status, RelationalEvidenceStatus::Indeterminate);
    }

    #[test]
    fn unverified_relation_is_not_traversed() {
        let mut related = record("e2", "origin-b", "holder-b");
        related.verification = RelationalVerification::Verified;
        let mut relation = edge("e1", "e2", EvidenceRelationKind::Supports);
        relation.verification = RelationalVerification::Unknown;
        let report = refine_relational_evidence(
            &scope(),
            &[record("e1", "origin-a", "holder-a"), related],
            &[relation],
        );
        assert_eq!(report.status, RelationalEvidenceStatus::Pass);
        assert_eq!(report.selected_evidence_ids, vec!["e1"]);
        assert_eq!(report.unverified_relation_count, 1);
    }

    #[test]
    fn relation_to_unknown_endpoint_fails_integrity() {
        let report = refine_relational_evidence(
            &scope(),
            &[record("e1", "origin-a", "holder-a")],
            &[edge("e1", "missing", EvidenceRelationKind::Supports)],
        );
        assert_eq!(report.status, RelationalEvidenceStatus::Fail);
    }

    #[test]
    fn relation_hop_limit_never_returns_partial_bundle() {
        let mut query = scope();
        query.max_relation_hops = 1;
        let report = refine_relational_evidence(
            &query,
            &[
                record("e1", "origin-a", "holder-a"),
                record("e2", "origin-b", "holder-b"),
                record("e3", "origin-c", "holder-c"),
            ],
            &[
                edge("e1", "e2", EvidenceRelationKind::SameEvent),
                edge("e2", "e3", EvidenceRelationKind::Updates),
            ],
        );
        assert_eq!(report.status, RelationalEvidenceStatus::Indeterminate);
        assert!(report.selected_evidence_ids.is_empty());
    }

    #[test]
    fn selected_item_limit_never_returns_partial_bundle() {
        let mut query = scope();
        query.max_selected_evidence = 1;
        let report = refine_relational_evidence(
            &query,
            &[record("e1", "origin-a", "holder-a"), record("e2", "origin-b", "holder-b")],
            &[edge("e1", "e2", EvidenceRelationKind::Supports)],
        );
        assert_eq!(report.status, RelationalEvidenceStatus::Indeterminate);
        assert!(report.selected_evidence_ids.is_empty());
    }

    #[test]
    fn duplicate_evidence_ids_fail() {
        let report = refine_relational_evidence(
            &scope(),
            &[record("e1", "origin-a", "holder-a"), record("e1", "origin-b", "holder-b")],
            &[],
        );
        assert_eq!(report.status, RelationalEvidenceStatus::Fail);
    }


    #[test]
    fn reversed_duplicate_symmetric_relation_fails() {
        let report = refine_relational_evidence(
            &scope(),
            &[record("e1", "origin-a", "holder-a"), record("e2", "origin-b", "holder-b")],
            &[
                edge("e1", "e2", EvidenceRelationKind::SameSubject),
                edge("e2", "e1", EvidenceRelationKind::SameSubject),
            ],
        );
        assert_eq!(report.status, RelationalEvidenceStatus::Fail);
    }

    #[test]
    fn duplicate_relation_edges_fail() {
        let relation = edge("e1", "e2", EvidenceRelationKind::Supports);
        let report = refine_relational_evidence(
            &scope(),
            &[record("e1", "origin-a", "holder-a"), record("e2", "origin-b", "holder-b")],
            &[relation.clone(), relation],
        );
        assert_eq!(report.status, RelationalEvidenceStatus::Fail);
    }

    #[test]
    fn malformed_candidate_digest_fails() {
        let mut item = record("e1", "origin-a", "holder-a");
        item.payload_digest = "not-a-digest".into();
        let report = refine_relational_evidence(&scope(), &[item], &[]);
        assert_eq!(report.status, RelationalEvidenceStatus::Fail);
    }

    #[test]
    fn invalid_bounds_fail() {
        let mut query = scope();
        query.max_relation_hops = 0;
        let report = refine_relational_evidence(&query, &[record("e1", "origin-a", "holder-a")], &[]);
        assert_eq!(report.status, RelationalEvidenceStatus::Fail);
    }
}
