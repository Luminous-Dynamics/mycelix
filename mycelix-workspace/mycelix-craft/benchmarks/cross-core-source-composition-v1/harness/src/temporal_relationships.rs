#![forbid(unsafe_code)]

//! Provenance-carrying temporal relationship claims (experimental V1).
//!
//! OCEL remains the interoperable event-log representation. This module
//! projects relationship assertion and mutation records into a two-frontier
//! snapshot. A relationship is a claim about an edge, not proof that the edge
//! is true or authorized. Source references, digests and authority scopes are
//! carried for later independent verification.
//!
//! Validity intervals are half-open: [valid_from, valid_to). The accepted
//! sequence is an append-order knowledge frontier, not a wall-clock timestamp.
//! source_recorded_at is preserved as source metadata and is not treated as a
//! platform transaction-time attestation.

use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TemporalRelationClaimV1 {
    pub relation_id: String,
    pub subject_id: String,
    pub predicate: String,
    pub object_id: String,
    pub valid_from: String,
    pub valid_to: Option<String>,
    pub source_recorded_at: String,
    pub accepted_sequence: u64,
    pub source_ref: String,
    pub source_digest: [u8; 32],
    pub asserting_principal: String,
    pub authority_scope: String,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TemporalRelationMutationKindV1 {
    Supersede,
    Revoke,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TemporalRelationMutationV1 {
    pub event_id: String,
    pub target_relation_id: String,
    pub target_claim_digest: [u8; 32],
    pub kind: TemporalRelationMutationKindV1,
    /// When the mutation is effective in the modeled domain.
    pub effective_at: String,
    pub source_recorded_at: String,
    pub accepted_sequence: u64,
    pub source_ref: String,
    pub source_digest: [u8; 32],
    pub asserting_principal: String,
    pub authority_scope: String,
    /// Required for Supersede; absent for Revoke.
    pub replacement_relation_id: Option<String>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TemporalRelationshipCutV1 {
    pub valid_at: String,
    pub known_through_sequence: u64,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TemporalRelationshipConflictV1 {
    pub subject_id: String,
    pub predicate: String,
    pub relation_ids: Vec<String>,
    pub object_ids: Vec<String>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TemporalRelationshipReceiptV1 {
    pub valid_at: String,
    pub known_through_sequence: u64,
    pub visible_relation_ids: Vec<String>,
    pub active_relations: Vec<TemporalRelationClaimV1>,
    pub conflicts: Vec<TemporalRelationshipConflictV1>,
    pub applied_mutation_ids: Vec<String>,
    pub superseded_relation_ids: Vec<String>,
    pub revoked_relation_ids: Vec<String>,
    pub snapshot_sha256: String,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TemporalRelationshipErrorV1 {
    EmptyIdentity,
    InvalidUtcTimestamp,
    InvalidValidityInterval,
    InvalidSequence,
    ZeroSourceDigest,
    DuplicateRelationId,
    DuplicateEventId,
    DuplicateAcceptedSequence,
    UnknownMutationTarget,
    UnknownReplacementRelation,
    TargetClaimDigestMismatch,
    NonCausalMutation,
    MutationScopeMismatch,
    InvalidReplacement,
    ConflictingMutations,
}

fn non_empty(value: &str) -> bool {
    !value.trim().is_empty() && value.trim() == value
}

/// Validate canonical fixed-width UTC seconds. Lexical ordering is chronological
/// only after this representation has been checked.
fn valid_utc_second(value: &str) -> bool {
    let bytes = value.as_bytes();
    if bytes.len() != 20
        || bytes[4] != b'-'
        || bytes[7] != b'-'
        || bytes[10] != b'T'
        || bytes[13] != b':'
        || bytes[16] != b':'
        || bytes[19] != b'Z'
    {
        return false;
    }
    for range in [0..4, 5..7, 8..10, 11..13, 14..16, 17..19] {
        if !bytes[range].iter().all(u8::is_ascii_digit) {
            return false;
        }
    }

    let number = |start: usize, end: usize| {
        std::str::from_utf8(&bytes[start..end])
            .ok()
            .and_then(|part| part.parse::<u32>().ok())
    };
    let year = number(0, 4).unwrap_or(0);
    let month = number(5, 7).unwrap_or(0);
    let day = number(8, 10).unwrap_or(0);
    let hour = number(11, 13).unwrap_or(u32::MAX);
    let minute = number(14, 16).unwrap_or(u32::MAX);
    let second = number(17, 19).unwrap_or(u32::MAX);
    let leap = year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
    let days_in_month = match month {
        1 | 3 | 5 | 7 | 8 | 10 | 12 => 31,
        4 | 6 | 9 | 11 => 30,
        2 if leap => 29,
        2 => 28,
        _ => 0,
    };

    year > 0
        && day > 0
        && day <= days_in_month
        && hour <= 23
        && minute <= 59
        && second <= 60
}

fn validate_claim(claim: &TemporalRelationClaimV1) -> Result<(), TemporalRelationshipErrorV1> {
    if !non_empty(&claim.relation_id)
        || !non_empty(&claim.subject_id)
        || !non_empty(&claim.predicate)
        || !non_empty(&claim.object_id)
        || !non_empty(&claim.source_ref)
        || !non_empty(&claim.asserting_principal)
        || !non_empty(&claim.authority_scope)
    {
        return Err(TemporalRelationshipErrorV1::EmptyIdentity);
    }
    if claim.accepted_sequence == 0 {
        return Err(TemporalRelationshipErrorV1::InvalidSequence);
    }
    if claim.source_digest == [0; 32] {
        return Err(TemporalRelationshipErrorV1::ZeroSourceDigest);
    }
    if !valid_utc_second(&claim.valid_from) || !valid_utc_second(&claim.source_recorded_at) {
        return Err(TemporalRelationshipErrorV1::InvalidUtcTimestamp);
    }
    if let Some(valid_to) = claim.valid_to.as_deref() {
        if !valid_utc_second(valid_to) {
            return Err(TemporalRelationshipErrorV1::InvalidUtcTimestamp);
        }
        if valid_to <= claim.valid_from.as_str() {
            return Err(TemporalRelationshipErrorV1::InvalidValidityInterval);
        }
    }
    Ok(())
}

fn validate_mutation(
    mutation: &TemporalRelationMutationV1,
) -> Result<(), TemporalRelationshipErrorV1> {
    if !non_empty(&mutation.event_id)
        || !non_empty(&mutation.target_relation_id)
        || !non_empty(&mutation.source_ref)
        || !non_empty(&mutation.asserting_principal)
        || !non_empty(&mutation.authority_scope)
    {
        return Err(TemporalRelationshipErrorV1::EmptyIdentity);
    }
    if mutation.accepted_sequence == 0 {
        return Err(TemporalRelationshipErrorV1::InvalidSequence);
    }
    if mutation.source_digest == [0; 32] || mutation.target_claim_digest == [0; 32] {
        return Err(TemporalRelationshipErrorV1::ZeroSourceDigest);
    }
    if !valid_utc_second(&mutation.effective_at)
        || !valid_utc_second(&mutation.source_recorded_at)
    {
        return Err(TemporalRelationshipErrorV1::InvalidUtcTimestamp);
    }
    match mutation.kind {
        TemporalRelationMutationKindV1::Supersede => {
            if mutation
                .replacement_relation_id
                .as_deref()
                .is_none_or(|value| !non_empty(value))
            {
                return Err(TemporalRelationshipErrorV1::InvalidReplacement);
            }
        }
        TemporalRelationMutationKindV1::Revoke => {
            if mutation.replacement_relation_id.is_some() {
                return Err(TemporalRelationshipErrorV1::InvalidReplacement);
            }
        }
    }
    Ok(())
}

/// Stable SHA-256 commitment to the complete declared relation assertion.
/// Length-prefixing prevents ambiguous concatenations of string fields.
pub fn temporal_relation_claim_digest(claim: &TemporalRelationClaimV1) -> [u8; 32] {
    let mut hasher = Sha256::new();
    hash_field(&mut hasher, &claim.relation_id);
    hash_field(&mut hasher, &claim.subject_id);
    hash_field(&mut hasher, &claim.predicate);
    hash_field(&mut hasher, &claim.object_id);
    hash_field(&mut hasher, &claim.valid_from);
    match claim.valid_to.as_deref() {
        Some(value) => {
            hasher.update([1]);
            hash_field(&mut hasher, value);
        }
        None => hasher.update([0]),
    }
    hash_field(&mut hasher, &claim.source_recorded_at);
    hasher.update(claim.accepted_sequence.to_be_bytes());
    hash_field(&mut hasher, &claim.source_ref);
    hasher.update(claim.source_digest);
    hash_field(&mut hasher, &claim.asserting_principal);
    hash_field(&mut hasher, &claim.authority_scope);
    finish_digest(hasher)
}

fn hash_field(hasher: &mut Sha256, value: &str) {
    hasher.update((value.len() as u64).to_be_bytes());
    hasher.update(value.as_bytes());
}

fn finish_digest(hasher: Sha256) -> [u8; 32] {
    let bytes = hasher.finalize();
    let mut digest = [0; 32];
    digest.copy_from_slice(&bytes);
    digest
}

fn digest_hex(digest: &[u8; 32]) -> String {
    digest.iter().map(|byte| format!("{byte:02x}")).collect()
}

fn snapshot_digest(
    cut: &TemporalRelationshipCutV1,
    visible_relation_ids: &[String],
    active: &[TemporalRelationClaimV1],
    conflicts: &[TemporalRelationshipConflictV1],
    applied_mutations: &[&TemporalRelationMutationV1],
    superseded: &[String],
    revoked: &[String],
) -> [u8; 32] {
    let mut hasher = Sha256::new();
    hash_field(&mut hasher, &cut.valid_at);
    hasher.update(cut.known_through_sequence.to_be_bytes());

    for id in visible_relation_ids {
        hash_field(&mut hasher, id);
    }
    for claim in active {
        hasher.update(temporal_relation_claim_digest(claim));
    }
    for conflict in conflicts {
        hash_field(&mut hasher, &conflict.subject_id);
        hash_field(&mut hasher, &conflict.predicate);
        for relation_id in &conflict.relation_ids {
            hash_field(&mut hasher, relation_id);
        }
        for object_id in &conflict.object_ids {
            hash_field(&mut hasher, object_id);
        }
    }
    for mutation in applied_mutations {
        hash_field(&mut hasher, &mutation.event_id);
        hash_field(&mut hasher, &mutation.target_relation_id);
        hasher.update(mutation.target_claim_digest);
        match mutation.kind {
            TemporalRelationMutationKindV1::Supersede => hasher.update([1]),
            TemporalRelationMutationKindV1::Revoke => hasher.update([2]),
        }
        hash_field(&mut hasher, &mutation.effective_at);
        hash_field(&mut hasher, &mutation.source_recorded_at);
        hasher.update(mutation.accepted_sequence.to_be_bytes());
        hash_field(&mut hasher, &mutation.source_ref);
        hasher.update(mutation.source_digest);
        hash_field(&mut hasher, &mutation.asserting_principal);
        hash_field(&mut hasher, &mutation.authority_scope);
        match mutation.replacement_relation_id.as_deref() {
            Some(value) => {
                hasher.update([1]);
                hash_field(&mut hasher, value);
            }
            None => hasher.update([0]),
        }
    }
    for id in superseded {
        hash_field(&mut hasher, id);
    }
    for id in revoked {
        hash_field(&mut hasher, id);
    }
    finish_digest(hasher)
}

/// Derive a deterministic temporal relationship snapshot from append-only
/// assertion and mutation records. The function does not authenticate sources,
/// verify legal authority, or infer that an asserted relationship is true.
pub fn project_temporal_relationships(
    claims: &[TemporalRelationClaimV1],
    mutations: &[TemporalRelationMutationV1],
    cut: &TemporalRelationshipCutV1,
) -> Result<TemporalRelationshipReceiptV1, TemporalRelationshipErrorV1> {
    if !valid_utc_second(&cut.valid_at) {
        return Err(TemporalRelationshipErrorV1::InvalidUtcTimestamp);
    }

    let mut claim_by_id = BTreeMap::new();
    let mut accepted_sequences = BTreeSet::new();
    for claim in claims {
        validate_claim(claim)?;
        if !accepted_sequences.insert(claim.accepted_sequence) {
            return Err(TemporalRelationshipErrorV1::DuplicateAcceptedSequence);
        }
        if claim_by_id.insert(claim.relation_id.as_str(), claim).is_some() {
            return Err(TemporalRelationshipErrorV1::DuplicateRelationId);
        }
    }

    let mut event_ids = BTreeSet::new();
    for mutation in mutations {
        validate_mutation(mutation)?;
        if !event_ids.insert(mutation.event_id.as_str()) {
            return Err(TemporalRelationshipErrorV1::DuplicateEventId);
        }
        if !accepted_sequences.insert(mutation.accepted_sequence) {
            return Err(TemporalRelationshipErrorV1::DuplicateAcceptedSequence);
        }
    }

    let mut ordered_mutations: Vec<_> = mutations.iter().collect();
    ordered_mutations.sort_by_key(|mutation| mutation.accepted_sequence);
    let mut mutated_targets = BTreeSet::new();
    for mutation in &ordered_mutations {
        let target = claim_by_id
            .get(mutation.target_relation_id.as_str())
            .ok_or(TemporalRelationshipErrorV1::UnknownMutationTarget)?;

        if mutation.accepted_sequence <= target.accepted_sequence
            || mutation.effective_at < target.valid_from
        {
            return Err(TemporalRelationshipErrorV1::NonCausalMutation);
        }
        if mutation.authority_scope != target.authority_scope {
            return Err(TemporalRelationshipErrorV1::MutationScopeMismatch);
        }
        if mutation.target_claim_digest != temporal_relation_claim_digest(target) {
            return Err(TemporalRelationshipErrorV1::TargetClaimDigestMismatch);
        }
        if !mutated_targets.insert(mutation.target_relation_id.as_str()) {
            return Err(TemporalRelationshipErrorV1::ConflictingMutations);
        }

        if mutation.kind == TemporalRelationMutationKindV1::Supersede {
            let replacement_id = mutation
                .replacement_relation_id
                .as_deref()
                .ok_or(TemporalRelationshipErrorV1::InvalidReplacement)?;
            let replacement = claim_by_id
                .get(replacement_id)
                .ok_or(TemporalRelationshipErrorV1::UnknownReplacementRelation)?;
            if mutated_targets.contains(replacement_id)
                || replacement.relation_id == target.relation_id
                || replacement.accepted_sequence >= mutation.accepted_sequence
                || replacement.valid_from > mutation.effective_at
                || replacement.subject_id != target.subject_id
                || replacement.predicate != target.predicate
                || replacement.authority_scope != target.authority_scope
            {
                return Err(TemporalRelationshipErrorV1::InvalidReplacement);
            }
        }
    }

    let visible_claims: Vec<_> = claims
        .iter()
        .filter(|claim| {
            claim.accepted_sequence <= cut.known_through_sequence
                && claim.valid_from <= cut.valid_at
                && claim
                    .valid_to
                    .as_deref()
                    .is_none_or(|valid_to| cut.valid_at.as_str() < valid_to)
        })
        .collect();
    let visible_relation_ids: Vec<_> = visible_claims
        .iter()
        .map(|claim| claim.relation_id.clone())
        .collect::<BTreeSet<_>>()
        .into_iter()
        .collect();

    let mut applied: Vec<_> = mutations
        .iter()
        .filter(|mutation| {
            mutation.accepted_sequence <= cut.known_through_sequence
                && mutation.effective_at <= cut.valid_at
        })
        .collect();
    applied.sort_by_key(|mutation| mutation.accepted_sequence);
    let mut superseded = BTreeSet::new();
    let mut revoked = BTreeSet::new();
    for mutation in &applied {
        match mutation.kind {
            TemporalRelationMutationKindV1::Supersede => {
                superseded.insert(mutation.target_relation_id.clone());
            }
            TemporalRelationMutationKindV1::Revoke => {
                revoked.insert(mutation.target_relation_id.clone());
            }
        }
    }
    let retired: BTreeSet<_> = superseded.union(&revoked).cloned().collect();
    let mut active_relations: Vec<_> = visible_claims
        .into_iter()
        .filter(|claim| !retired.contains(&claim.relation_id))
        .cloned()
        .collect();
    active_relations.sort_by(|left, right| left.relation_id.cmp(&right.relation_id));

    let mut by_subject_predicate: BTreeMap<(String, String), Vec<&TemporalRelationClaimV1>> =
        BTreeMap::new();
    for claim in &active_relations {
        by_subject_predicate
            .entry((claim.subject_id.clone(), claim.predicate.clone()))
            .or_default()
            .push(claim);
    }
    let mut conflicts = Vec::new();
    for ((subject_id, predicate), group) in by_subject_predicate {
        let object_ids: BTreeSet<_> = group
            .iter()
            .map(|claim| claim.object_id.clone())
            .collect();
        if object_ids.len() > 1 {
            let mut relation_ids: Vec<_> = group
                .iter()
                .map(|claim| claim.relation_id.clone())
                .collect();
            relation_ids.sort();
            conflicts.push(TemporalRelationshipConflictV1 {
                subject_id,
                predicate,
                relation_ids,
                object_ids: object_ids.into_iter().collect(),
            });
        }
    }

    let applied_mutation_ids: Vec<_> = applied
        .iter()
        .map(|mutation| mutation.event_id.clone())
        .collect();
    let superseded_relation_ids: Vec<_> = superseded.into_iter().collect();
    let revoked_relation_ids: Vec<_> = revoked.into_iter().collect();
    let digest = snapshot_digest(
        cut,
        &visible_relation_ids,
        &active_relations,
        &conflicts,
        &applied_mutation_ids,
        &superseded_relation_ids,
        &revoked_relation_ids,
    );

    Ok(TemporalRelationshipReceiptV1 {
        valid_at: cut.valid_at.clone(),
        known_through_sequence: cut.known_through_sequence,
        visible_relation_ids,
        active_relations,
        conflicts,
        applied_mutation_ids,
        superseded_relation_ids,
        revoked_relation_ids,
        snapshot_sha256: digest_hex(&digest),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const T08: &str = "2026-11-27T08:00:00Z";
    const T10: &str = "2026-11-27T10:00:00Z";
    const T11: &str = "2026-11-27T11:00:00Z";
    const T12: &str = "2026-11-27T12:00:00Z";
    const T13: &str = "2026-11-27T13:00:00Z";

    fn claim(
        relation_id: &str,
        subject_id: &str,
        predicate: &str,
        object_id: &str,
        valid_from: &str,
        sequence: u64,
    ) -> TemporalRelationClaimV1 {
        TemporalRelationClaimV1 {
            relation_id: relation_id.to_owned(),
            subject_id: subject_id.to_owned(),
            predicate: predicate.to_owned(),
            object_id: object_id.to_owned(),
            valid_from: valid_from.to_owned(),
            valid_to: None,
            source_recorded_at: T08.to_owned(),
            accepted_sequence: sequence,
            source_ref: format!("source:{relation_id}"),
            source_digest: [sequence as u8; 32],
            asserting_principal: "principal:synthetic-operator".to_owned(),
            authority_scope: "scope:water-zone-A".to_owned(),
        }
    }

    fn mutation(
        event_id: &str,
        target: &TemporalRelationClaimV1,
        kind: TemporalRelationMutationKindV1,
        effective_at: &str,
        sequence: u64,
        replacement_relation_id: Option<&str>,
    ) -> TemporalRelationMutationV1 {
        TemporalRelationMutationV1 {
            event_id: event_id.to_owned(),
            target_relation_id: target.relation_id.clone(),
            target_claim_digest: temporal_relation_claim_digest(target),
            kind,
            effective_at: effective_at.to_owned(),
            source_recorded_at: T13.to_owned(),
            accepted_sequence: sequence,
            source_ref: format!("source:{event_id}"),
            source_digest: [sequence as u8; 32],
            asserting_principal: "principal:synthetic-reviewer".to_owned(),
            authority_scope: target.authority_scope.clone(),
            replacement_relation_id: replacement_relation_id.map(str::to_owned),
        }
    }

    fn cut(valid_at: &str, known_through_sequence: u64) -> TemporalRelationshipCutV1 {
        TemporalRelationshipCutV1 {
            valid_at: valid_at.to_owned(),
            known_through_sequence,
        }
    }

    #[test]
    fn late_recorded_relation_is_invisible_before_accepted_sequence() {
        let mut late = claim("rel-crew-wo", "crew-7", "assigned_to", "work-order-9", T08, 2);
        late.source_recorded_at = T13.to_owned();

        let before = project_temporal_relationships(&[late.clone()], &[], &cut(T11, 1)).unwrap();
        assert!(before.active_relations.is_empty());

        let after = project_temporal_relationships(&[late], &[], &cut(T11, 2)).unwrap();
        assert_eq!(after.active_relations.len(), 1);
        assert_eq!(after.active_relations[0].relation_id, "rel-crew-wo");
    }

    #[test]
    fn supersession_surfaces_conflict_until_the_mutation_is_known() {
        let original = claim(
            "rel-asset-zone-old",
            "asset-17",
            "member_of_zone",
            "zone-A",
            T08,
            1,
        );
        let replacement = claim(
            "rel-asset-zone-new",
            "asset-17",
            "member_of_zone",
            "zone-B",
            T10,
            2,
        );
        let correction = mutation(
            "event-rel-correction",
            &original,
            TemporalRelationMutationKindV1::Supersede,
            T10,
            3,
            Some("rel-asset-zone-new"),
        );

        let known_before_correction =
            project_temporal_relationships(&[original.clone(), replacement.clone()], &[correction.clone()], &cut(T11, 2))
                .unwrap();
        assert_eq!(known_before_correction.active_relations.len(), 2);
        assert_eq!(known_before_correction.conflicts.len(), 1);

        let known_after_correction =
            project_temporal_relationships(&[original, replacement], &[correction], &cut(T11, 3))
                .unwrap();
        assert_eq!(known_after_correction.active_relations.len(), 1);
        assert_eq!(known_after_correction.active_relations[0].object_id, "zone-B");
        assert!(known_after_correction.conflicts.is_empty());
        assert_eq!(
            known_after_correction.superseded_relation_ids,
            vec!["rel-asset-zone-old"]
        );
    }

    #[test]
    fn revocation_requires_both_valid_time_and_knowledge_frontier() {
        let original = claim(
            "rel-permit-crew",
            "permit-2",
            "authorizes",
            "crew-7",
            T08,
            1,
        );
        let revocation = mutation(
            "event-permit-revoked",
            &original,
            TemporalRelationMutationKindV1::Revoke,
            T12,
            2,
            None,
        );

        let before_effective_time =
            project_temporal_relationships(&[original.clone()], &[revocation.clone()], &cut(T11, 2))
                .unwrap();
        assert_eq!(before_effective_time.active_relations.len(), 1);

        let after_effective_before_knowledge =
            project_temporal_relationships(&[original.clone()], &[revocation.clone()], &cut(T13, 1))
                .unwrap();
        assert_eq!(after_effective_before_knowledge.active_relations.len(), 1);

        let after_effective_and_knowledge =
            project_temporal_relationships(&[original], &[revocation], &cut(T13, 2)).unwrap();
        assert!(after_effective_and_knowledge.active_relations.is_empty());
        assert_eq!(
            after_effective_and_knowledge.revoked_relation_ids,
            vec!["rel-permit-crew"]
        );
    }

    #[test]
    fn event_array_permutation_preserves_snapshot_digest() {
        let first = claim("rel-1", "asset-1", "in_zone", "zone-1", T08, 1);
        let second = claim("rel-2", "crew-1", "assigned_to", "work-order-1", T08, 2);
        let original = vec![first.clone(), second.clone()];
        let mut reordered = vec![second, first];
        reordered.reverse();

        let a = project_temporal_relationships(&original, &[], &cut(T11, 2)).unwrap();
        let b = project_temporal_relationships(&reordered, &[], &cut(T11, 2)).unwrap();
        assert_eq!(a, b);
    }

    #[test]
    fn target_claim_digest_prevents_relation_id_rebinding() {
        let original = claim("rel-1", "asset-1", "in_zone", "zone-1", T08, 1);
        let mut altered = original.clone();
        altered.object_id = "zone-2".to_owned();
        let mut attempted = mutation(
            "event-rel-revoke",
            &original,
            TemporalRelationMutationKindV1::Revoke,
            T10,
            2,
            None,
        );
        attempted.target_claim_digest = temporal_relation_claim_digest(&altered);

        assert_eq!(
            project_temporal_relationships(&[original], &[attempted], &cut(T11, 2)),
            Err(TemporalRelationshipErrorV1::TargetClaimDigestMismatch)
        );
    }

    #[test]
    fn authority_scope_mismatch_fails_closed() {
        let original = claim("rel-1", "asset-1", "in_zone", "zone-1", T08, 1);
        let mut attempted = mutation(
            "event-rel-revoke",
            &original,
            TemporalRelationMutationKindV1::Revoke,
            T10,
            2,
            None,
        );
        attempted.authority_scope = "scope:other-zone".to_owned();

        assert_eq!(
            project_temporal_relationships(&[original], &[attempted], &cut(T11, 2)),
            Err(TemporalRelationshipErrorV1::MutationScopeMismatch)
        );
    }

    #[test]
    fn duplicate_accepted_sequence_fails_closed_across_claims_and_events() {
        let first = claim("rel-1", "asset-1", "in_zone", "zone-1", T08, 1);
        let mut second = claim("rel-2", "crew-1", "assigned_to", "work-order-1", T08, 1);
        second.accepted_sequence = 1;

        assert_eq!(
            project_temporal_relationships(&[first, second], &[], &cut(T11, 1)),
            Err(TemporalRelationshipErrorV1::DuplicateAcceptedSequence)
        );
    }

    #[test]
    fn half_open_validity_interval_excludes_edge_at_expiry() {
        let mut expiring = claim("rel-1", "asset-1", "in_zone", "zone-1", T08, 1);
        expiring.valid_to = Some(T10.to_owned());

        let before = project_temporal_relationships(&[expiring.clone()], &[], &cut(T08, 1)).unwrap();
        let at_expiry = project_temporal_relationships(&[expiring], &[], &cut(T10, 1)).unwrap();
        assert_eq!(before.active_relations.len(), 1);
        assert!(at_expiry.active_relations.is_empty());
    }
}
