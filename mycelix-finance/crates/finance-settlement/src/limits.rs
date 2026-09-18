use std::collections::{BTreeMap, BTreeSet};

use mycelix_business_core::ReferenceId;

use crate::{
    FinalityEvidence, FinalityProfile, SettlementEvaluationContext, SettlementObservation,
    SettlementSubject,
};

const U16_BYTES: u64 = 2;
const U32_BYTES: u64 = 4;
const U64_BYTES: u64 = 8;
const DIGEST_BYTES: u64 = 32;

// Mirrored from canonical.rs. Tests compare analytic lengths to the registered
// production encoders so canonical-layout drift fails loudly.
const PROFILE_DOMAIN_BYTES: u64 =
    b"MYCELIX_FINANCE_SETTLEMENT_FINALITY_PROFILE_V1\0".len() as u64;
const EVIDENCE_DOMAIN_BYTES: u64 = b"MYCELIX_FINANCE_SETTLEMENT_EVIDENCE_V1\0".len() as u64;
const OBSERVATION_DOMAIN_BYTES: u64 =
    b"MYCELIX_FINANCE_SETTLEMENT_OBSERVATION_V1\0".len() as u64;
const EVALUATION_CONTEXT_DOMAIN_BYTES: u64 =
    b"MYCELIX_FINANCE_SETTLEMENT_EVALUATION_CONTEXT_V1\0".len() as u64;
const FRONTIER_DOMAIN_BYTES: u64 = b"MYCELIX_FINANCE_SETTLEMENT_FRONTIER_V1\0".len() as u64;

/// Operational admission budget for an already-typed FIN-ECO-002 v1 request.
///
/// C1 does not change `qualify_settlement` and does not commit this budget into
/// settlement proof bytes. Choosing an authoritative budget profile belongs to a
/// later policy/integration tranche.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SettlementVerificationBudgetV1 {
    pub max_observations: u64,
    pub max_operations: u64,
    pub max_same_revision_observations_per_operation: u64,
    pub max_evidence_per_observation: u64,
    pub max_total_evidence_items: u64,
    pub max_distinct_evidence_ids: u64,
    pub max_distinct_sources: u64,
    pub max_distinct_evidence_kinds: u64,
    pub max_required_evidence_kinds: u64,
    pub max_charged_canonical_bytes_per_observation: u64,
    pub max_total_charged_canonical_bytes: u64,
    pub max_total_hash_input_bytes: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SettlementVerificationUsage {
    pub observations: u64,
    pub operations: u64,
    pub max_same_revision_observations_per_operation: u64,
    pub max_evidence_per_observation: u64,
    pub total_evidence_items: u64,
    pub distinct_evidence_ids: u64,
    pub distinct_sources: u64,
    pub distinct_evidence_kinds: u64,
    pub required_evidence_kinds: u64,
    /// Physical evidence bytes plus a conservative observation envelope.
    pub max_charged_canonical_bytes_per_observation: u64,
    /// One charge for each physical canonical object represented by the request.
    pub total_charged_canonical_bytes: u64,
    /// Conservative upper bound on bytes fed into SHA-256 by the current v1 qualifier.
    pub total_hash_input_bytes_upper_bound: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SettlementVerificationResource {
    Observations,
    Operations,
    SameRevisionObservationsPerOperation,
    EvidencePerObservation,
    TotalEvidenceItems,
    DistinctEvidenceIds,
    DistinctSources,
    DistinctEvidenceKinds,
    RequiredEvidenceKinds,
    ChargedCanonicalBytesPerObservation,
    TotalChargedCanonicalBytes,
    TotalHashInputBytes,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SettlementVerificationBudgetError {
    LimitExceeded {
        resource: SettlementVerificationResource,
        limit: u64,
        actual: u64,
    },
    AccountingOverflow,
    CanonicalCountOverflow,
}

/// Account verifier work without hashing or allocating canonical byte buffers.
///
/// Exact duplicate physical records consume budget. A successful result means only
/// that this already-typed request fits `budget`; it does not imply semantic
/// settlement qualification or authority of the chosen budget.
pub fn assess_settlement_verification_budget_v1(
    subject: &SettlementSubject,
    profile: &FinalityProfile,
    observations: &[SettlementObservation],
    context: &SettlementEvaluationContext,
    budget: SettlementVerificationBudgetV1,
) -> Result<SettlementVerificationUsage, SettlementVerificationBudgetError> {
    let observation_count = count(observations.len())?;
    enforce(
        SettlementVerificationResource::Observations,
        observation_count,
        budget.max_observations,
    )?;

    let required_evidence_kinds = count(profile.required_evidence_kinds.len())?;
    enforce(
        SettlementVerificationResource::RequiredEvidenceKinds,
        required_evidence_kinds,
        budget.max_required_evidence_kinds,
    )?;

    let profile_bytes = profile_canonical_len(profile)?;
    let context_bytes = evaluation_context_canonical_len(context)?;
    let mut total_charged = checked_add(profile_bytes, context_bytes)?;
    let mut total_hash_upper_bound = total_charged;
    enforce(
        SettlementVerificationResource::TotalChargedCanonicalBytes,
        total_charged,
        budget.max_total_charged_canonical_bytes,
    )?;
    enforce(
        SettlementVerificationResource::TotalHashInputBytes,
        total_hash_upper_bound,
        budget.max_total_hash_input_bytes,
    )?;

    let mut operations: BTreeSet<&ReferenceId> = BTreeSet::new();
    let mut same_revision: BTreeMap<(&ReferenceId, u64), u64> = BTreeMap::new();
    let mut evidence_ids: BTreeSet<&ReferenceId> = BTreeSet::new();
    let mut sources: BTreeSet<&ReferenceId> = BTreeSet::new();
    let mut kinds: BTreeSet<&ReferenceId> = BTreeSet::new();

    let mut max_same_revision = 0_u64;
    let mut max_evidence_per_observation = 0_u64;
    let mut total_evidence_items = 0_u64;
    let mut max_charged_per_observation = 0_u64;

    for observation in observations {
        let evidence_count = count(observation.evidence.len())?;
        enforce(
            SettlementVerificationResource::EvidencePerObservation,
            evidence_count,
            budget.max_evidence_per_observation,
        )?;
        max_evidence_per_observation = max_evidence_per_observation.max(evidence_count);

        operations.insert(&observation.operation_id);
        enforce(
            SettlementVerificationResource::Operations,
            count(operations.len())?,
            budget.max_operations,
        )?;

        let revision_count = same_revision
            .entry((&observation.operation_id, observation.revision))
            .or_default();
        *revision_count = checked_add(*revision_count, 1)?;
        enforce(
            SettlementVerificationResource::SameRevisionObservationsPerOperation,
            *revision_count,
            budget.max_same_revision_observations_per_operation,
        )?;
        max_same_revision = max_same_revision.max(*revision_count);

        let mut physical_evidence_bytes = 0_u64;
        for evidence in &observation.evidence {
            total_evidence_items = checked_add(total_evidence_items, 1)?;
            enforce(
                SettlementVerificationResource::TotalEvidenceItems,
                total_evidence_items,
                budget.max_total_evidence_items,
            )?;

            evidence_ids.insert(&evidence.evidence_id);
            enforce(
                SettlementVerificationResource::DistinctEvidenceIds,
                count(evidence_ids.len())?,
                budget.max_distinct_evidence_ids,
            )?;
            sources.insert(&evidence.source);
            enforce(
                SettlementVerificationResource::DistinctSources,
                count(sources.len())?,
                budget.max_distinct_sources,
            )?;
            kinds.insert(&evidence.kind);
            enforce(
                SettlementVerificationResource::DistinctEvidenceKinds,
                count(kinds.len())?,
                budget.max_distinct_evidence_kinds,
            )?;

            let evidence_bytes = evidence_canonical_len(evidence)?;
            physical_evidence_bytes = checked_add(physical_evidence_bytes, evidence_bytes)?;
            total_charged = checked_add(total_charged, evidence_bytes)?;
            total_hash_upper_bound = checked_add(
                total_hash_upper_bound,
                checked_mul(evidence_bytes, 3)?,
            )?;
            enforce(
                SettlementVerificationResource::TotalChargedCanonicalBytes,
                total_charged,
                budget.max_total_charged_canonical_bytes,
            )?;
            enforce(
                SettlementVerificationResource::TotalHashInputBytes,
                total_hash_upper_bound,
                budget.max_total_hash_input_bytes,
            )?;
        }

        // Production observation bytes deduplicate evidence commitments. C1 instead
        // charges one 32-byte slot for every physical evidence record, preventing
        // duplicate multiplicity from becoming a free DoS amplifier.
        let observation_bytes_upper_bound = observation_canonical_len_upper_bound(observation)?;
        let charged_observation = checked_add(physical_evidence_bytes, observation_bytes_upper_bound)?;
        enforce(
            SettlementVerificationResource::ChargedCanonicalBytesPerObservation,
            charged_observation,
            budget.max_charged_canonical_bytes_per_observation,
        )?;
        max_charged_per_observation = max_charged_per_observation.max(charged_observation);

        total_charged = checked_add(total_charged, observation_bytes_upper_bound)?;
        total_hash_upper_bound = checked_add(
            total_hash_upper_bound,
            checked_mul(observation_bytes_upper_bound, 2)?,
        )?;
        enforce(
            SettlementVerificationResource::TotalChargedCanonicalBytes,
            total_charged,
            budget.max_total_charged_canonical_bytes,
        )?;
        enforce(
            SettlementVerificationResource::TotalHashInputBytes,
            total_hash_upper_bound,
            budget.max_total_hash_input_bytes,
        )?;
    }

    let frontier_bytes_upper_bound = frontier_canonical_len_upper_bound(subject, observation_count)?;
    total_charged = checked_add(total_charged, frontier_bytes_upper_bound)?;
    total_hash_upper_bound = checked_add(total_hash_upper_bound, frontier_bytes_upper_bound)?;
    enforce(
        SettlementVerificationResource::TotalChargedCanonicalBytes,
        total_charged,
        budget.max_total_charged_canonical_bytes,
    )?;
    enforce(
        SettlementVerificationResource::TotalHashInputBytes,
        total_hash_upper_bound,
        budget.max_total_hash_input_bytes,
    )?;

    Ok(SettlementVerificationUsage {
        observations: observation_count,
        operations: count(operations.len())?,
        max_same_revision_observations_per_operation: max_same_revision,
        max_evidence_per_observation,
        total_evidence_items,
        distinct_evidence_ids: count(evidence_ids.len())?,
        distinct_sources: count(sources.len())?,
        distinct_evidence_kinds: count(kinds.len())?,
        required_evidence_kinds,
        max_charged_canonical_bytes_per_observation: max_charged_per_observation,
        total_charged_canonical_bytes: total_charged,
        total_hash_input_bytes_upper_bound: total_hash_upper_bound,
    })
}

fn enforce(
    resource: SettlementVerificationResource,
    actual: u64,
    limit: u64,
) -> Result<(), SettlementVerificationBudgetError> {
    if actual > limit {
        Err(SettlementVerificationBudgetError::LimitExceeded {
            resource,
            limit,
            actual,
        })
    } else {
        Ok(())
    }
}

fn count(value: usize) -> Result<u64, SettlementVerificationBudgetError> {
    u64::try_from(value).map_err(|_| SettlementVerificationBudgetError::AccountingOverflow)
}

fn ensure_u32_count(value: usize) -> Result<u64, SettlementVerificationBudgetError> {
    u32::try_from(value).map_err(|_| SettlementVerificationBudgetError::CanonicalCountOverflow)?;
    count(value)
}

fn string_len(value: &str) -> Result<u64, SettlementVerificationBudgetError> {
    u32::try_from(value.len())
        .map_err(|_| SettlementVerificationBudgetError::CanonicalCountOverflow)?;
    checked_add(U32_BYTES, count(value.len())?)
}

fn reference_len(value: &ReferenceId) -> Result<u64, SettlementVerificationBudgetError> {
    string_len(value.as_str())
}

fn profile_canonical_len(
    profile: &FinalityProfile,
) -> Result<u64, SettlementVerificationBudgetError> {
    ensure_u32_count(profile.required_evidence_kinds.len())?;
    let mut total = checked_add(PROFILE_DOMAIN_BYTES, U16_BYTES)?;
    total = checked_add(total, reference_len(&profile.profile_ref.id)?)?;
    total = checked_add(total, U64_BYTES)?;
    total = checked_add(total, reference_len(&profile.rail)?)?;
    total = checked_add(total, reference_len(&profile.network)?)?;
    total = checked_add(total, U32_BYTES)?;
    for kind in &profile.required_evidence_kinds {
        total = checked_add(total, reference_len(kind)?)?;
    }
    total = checked_add(total, U16_BYTES)?;
    total = checked_add(total, U64_BYTES)?;
    checked_add(total, 1)
}

fn evaluation_context_canonical_len(
    context: &SettlementEvaluationContext,
) -> Result<u64, SettlementVerificationBudgetError> {
    let mut total = checked_add(EVALUATION_CONTEXT_DOMAIN_BYTES, U16_BYTES)?;
    total = checked_add(total, 1)?;
    total = checked_add(total, U64_BYTES)?;
    total = checked_add(total, reference_len(&context.temporal_profile_id)?)?;
    total = checked_add(total, U64_BYTES)?;
    checked_add(total, DIGEST_BYTES)
}

fn evidence_canonical_len(
    evidence: &FinalityEvidence,
) -> Result<u64, SettlementVerificationBudgetError> {
    let mut total = checked_add(EVIDENCE_DOMAIN_BYTES, U16_BYTES)?;
    total = checked_add(total, reference_len(&evidence.evidence_id)?)?;
    total = checked_add(total, reference_len(&evidence.subject)?)?;
    total = checked_add(total, reference_len(&evidence.operation_id)?)?;
    total = checked_add(total, U64_BYTES)?;
    total = checked_add(total, reference_len(&evidence.observation_id)?)?;
    total = checked_add(total, reference_len(&evidence.kind)?)?;
    total = checked_add(total, reference_len(&evidence.source)?)?;
    checked_add(total, DIGEST_BYTES)
}

fn observation_canonical_len_upper_bound(
    observation: &SettlementObservation,
) -> Result<u64, SettlementVerificationBudgetError> {
    let evidence_count = ensure_u32_count(observation.evidence.len())?;
    let mut total = checked_add(OBSERVATION_DOMAIN_BYTES, U16_BYTES)?;
    total = checked_add(total, reference_len(&observation.observation_id)?)?;
    total = checked_add(total, reference_len(&observation.subject)?)?;
    total = checked_add(total, DIGEST_BYTES)?;
    total = checked_add(total, reference_len(observation.attempt.as_ref_id())?)?;
    total = checked_add(total, reference_len(&observation.rail)?)?;
    total = checked_add(total, reference_len(&observation.network)?)?;
    total = checked_add(total, reference_len(&observation.operation_id)?)?;
    total = checked_add(total, U64_BYTES)?;
    total = checked_add(total, string_len(observation.amount.asset().as_str())?)?;
    total = checked_add(total, U64_BYTES)?;
    total = checked_add(total, 1)?;
    total = checked_add(total, U64_BYTES)?;
    total = checked_add(total, U32_BYTES)?;
    checked_add(total, checked_mul(evidence_count, DIGEST_BYTES)?)
}

fn frontier_canonical_len_upper_bound(
    subject: &SettlementSubject,
    observation_count: u64,
) -> Result<u64, SettlementVerificationBudgetError> {
    if observation_count > u64::from(u32::MAX) {
        return Err(SettlementVerificationBudgetError::CanonicalCountOverflow);
    }
    let mut total = checked_add(FRONTIER_DOMAIN_BYTES, U16_BYTES)?;
    total = checked_add(total, reference_len(&subject.id)?)?;
    total = checked_add(total, DIGEST_BYTES)?;
    total = checked_add(total, reference_len(subject.attempt.as_ref_id())?)?;
    total = checked_add(total, reference_len(&subject.rail)?)?;
    total = checked_add(total, reference_len(&subject.network)?)?;
    total = checked_add(total, DIGEST_BYTES)?;
    total = checked_add(total, string_len(subject.amount.asset().as_str())?)?;
    total = checked_add(total, U64_BYTES)?;
    total = checked_add(total, DIGEST_BYTES)?;
    total = checked_add(total, U32_BYTES)?;
    checked_add(total, checked_mul(observation_count, DIGEST_BYTES)?)
}

fn checked_add(left: u64, right: u64) -> Result<u64, SettlementVerificationBudgetError> {
    left.checked_add(right)
        .ok_or(SettlementVerificationBudgetError::AccountingOverflow)
}

fn checked_mul(left: u64, right: u64) -> Result<u64, SettlementVerificationBudgetError> {
    left.checked_mul(right)
        .ok_or(SettlementVerificationBudgetError::AccountingOverflow)
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeSet;

    use mycelix_business_core::{Digest32, ExecutionAttemptRef, ReferenceId};
    use mycelix_finance_exact::{AssetAmount, AssetId};

    use crate::{
        FinalityEvidence, FinalityProfile, ObservedSettlementState, ReversalModel,
        SettlementEvaluationContext, SettlementObservation, SettlementSubject,
        canonical_evaluation_context_bytes, canonical_evidence_bytes,
        canonical_finality_profile_bytes, canonical_observation_bytes,
        canonical_selected_evidence_frontier_bytes, evaluation_context_commitment,
        observation_commitment,
    };

    use super::*;

    fn reference(value: &str) -> ReferenceId {
        ReferenceId::new(value).expect("static reference")
    }

    fn attempt(value: &str) -> ExecutionAttemptRef {
        ExecutionAttemptRef::new(value).expect("static attempt")
    }

    fn amount(units: u64) -> AssetAmount {
        AssetAmount::new(
            units,
            AssetId::new("USD.micro").expect("static asset identifier"),
        )
    }

    fn profile() -> FinalityProfile {
        FinalityProfile::new(
            reference("finality:bank:v1"),
            1,
            reference("rail:bank"),
            reference("network:test-bank"),
            BTreeSet::from([reference("evidence:provider-settlement")]),
            1,
            60_000,
            ReversalModel::MayReverse,
        )
        .expect("valid profile")
    }

    fn context() -> SettlementEvaluationContext {
        SettlementEvaluationContext::deterministic_supplied(
            1_100,
            reference("clock:deterministic:test"),
            1,
            Digest32::repeat(4),
        )
        .expect("valid context")
    }

    fn subject(profile: &FinalityProfile) -> SettlementSubject {
        SettlementSubject {
            id: reference("settlement:subject:1"),
            financial_effect_commitment: Digest32::repeat(5),
            attempt: attempt("attempt:1"),
            rail: profile.rail().clone(),
            network: profile.network().clone(),
            amount: amount(100),
            required_profile: profile.profile_ref().clone(),
        }
    }

    fn observation(
        profile: &FinalityProfile,
        operation: &str,
        source: &str,
        revision: u64,
    ) -> SettlementObservation {
        let observation_id = format!("observation:{operation}:{revision}:{source}");
        let subject = subject(profile);
        SettlementObservation {
            observation_id: reference(&observation_id),
            subject: subject.id,
            financial_effect_commitment: subject.financial_effect_commitment,
            attempt: subject.attempt,
            rail: subject.rail,
            network: subject.network,
            operation_id: reference(operation),
            revision,
            amount: amount(100),
            state: ObservedSettlementState::Applied,
            observed_at_unix_ms: 1_000,
            evidence: vec![FinalityEvidence {
                evidence_id: reference(&format!("evidence:{operation}:{revision}:{source}")),
                subject: reference("settlement:subject:1"),
                operation_id: reference(operation),
                operation_revision: revision,
                observation_id: reference(&observation_id),
                kind: reference("evidence:provider-settlement"),
                source: reference(source),
                digest: Digest32::repeat(7),
            }],
        }
    }

    fn generous_budget() -> SettlementVerificationBudgetV1 {
        SettlementVerificationBudgetV1 {
            max_observations: 16,
            max_operations: 16,
            max_same_revision_observations_per_operation: 8,
            max_evidence_per_observation: 8,
            max_total_evidence_items: 64,
            max_distinct_evidence_ids: 64,
            max_distinct_sources: 64,
            max_distinct_evidence_kinds: 16,
            max_required_evidence_kinds: 8,
            max_charged_canonical_bytes_per_observation: 8 * 1_024,
            max_total_charged_canonical_bytes: 64 * 1_024,
            max_total_hash_input_bytes: 128 * 1_024,
        }
    }

    #[test]
    fn analytic_lengths_match_registered_encoders_for_single_evidence() {
        let profile = profile();
        let context = context();
        let subject = subject(&profile);
        let observation = observation(&profile, "op:1", "source:a", 1);

        assert_eq!(
            profile_canonical_len(&profile).expect("profile length"),
            canonical_finality_profile_bytes(&profile)
                .expect("profile bytes")
                .len() as u64
        );
        assert_eq!(
            evaluation_context_canonical_len(&context).expect("context length"),
            canonical_evaluation_context_bytes(&context)
                .expect("context bytes")
                .len() as u64
        );
        assert_eq!(
            evidence_canonical_len(&observation.evidence[0]).expect("evidence length"),
            canonical_evidence_bytes(&observation.evidence[0])
                .expect("evidence bytes")
                .len() as u64
        );
        assert_eq!(
            observation_canonical_len_upper_bound(&observation).expect("observation length"),
            canonical_observation_bytes(&observation)
                .expect("observation bytes")
                .len() as u64
        );

        let selected = BTreeSet::from([
            observation_commitment(&observation).expect("observation commitment")
        ]);
        assert_eq!(
            frontier_canonical_len_upper_bound(&subject, 1).expect("frontier length"),
            canonical_selected_evidence_frontier_bytes(
                &subject,
                &profile,
                evaluation_context_commitment(&context).expect("context commitment"),
                &selected,
            )
            .expect("frontier bytes")
            .len() as u64
        );
    }

    #[test]
    fn duplicate_evidence_pays_physical_cost_despite_canonical_deduplication() {
        let profile = profile();
        let context = context();
        let subject = subject(&profile);
        let mut observation = observation(&profile, "op:1", "source:a", 1);
        observation.evidence.push(observation.evidence[0].clone());

        let usage = assess_settlement_verification_budget_v1(
            &subject,
            &profile,
            &[observation.clone()],
            &context,
            generous_budget(),
        )
        .expect("admitted");
        assert_eq!(usage.total_evidence_items, 2);

        let evidence_bytes = evidence_canonical_len(&observation.evidence[0]).expect("length");
        let actual_observation = canonical_observation_bytes(&observation)
            .expect("observation bytes")
            .len() as u64;
        assert!(
            usage.max_charged_canonical_bytes_per_observation
                > checked_add(evidence_bytes, actual_observation).expect("sum")
        );
    }

    #[test]
    fn same_revision_limit_is_not_history_depth_limit() {
        let profile = profile();
        let context = context();
        let subject = subject(&profile);
        let mut budget = generous_budget();
        budget.max_same_revision_observations_per_operation = 1;

        let history = [
            observation(&profile, "op:1", "source:a", 1),
            observation(&profile, "op:1", "source:a", 2),
        ];
        assert!(assess_settlement_verification_budget_v1(
            &subject, &profile, &history, &context, budget
        )
        .is_ok());

        let same_revision = [
            observation(&profile, "op:1", "source:a", 2),
            observation(&profile, "op:1", "source:b", 2),
        ];
        assert_eq!(
            assess_settlement_verification_budget_v1(
                &subject,
                &profile,
                &same_revision,
                &context,
                budget,
            ),
            Err(SettlementVerificationBudgetError::LimitExceeded {
                resource: SettlementVerificationResource::SameRevisionObservationsPerOperation,
                limit: 1,
                actual: 2,
            })
        );
    }

    #[test]
    fn distinct_sources_and_kinds_are_separately_bounded() {
        let profile = profile();
        let context = context();
        let subject = subject(&profile);
        let first = observation(&profile, "op:1", "source:a", 1);
        let mut second = observation(&profile, "op:2", "source:b", 1);
        second.evidence[0].kind = reference("evidence:bank-ledger");
        let observations = [first, second];

        let mut source_budget = generous_budget();
        source_budget.max_distinct_sources = 1;
        assert!(matches!(
            assess_settlement_verification_budget_v1(
                &subject,
                &profile,
                &observations,
                &context,
                source_budget,
            ),
            Err(SettlementVerificationBudgetError::LimitExceeded {
                resource: SettlementVerificationResource::DistinctSources,
                ..
            })
        ));

        let mut kind_budget = generous_budget();
        kind_budget.max_distinct_evidence_kinds = 1;
        assert!(matches!(
            assess_settlement_verification_budget_v1(
                &subject,
                &profile,
                &observations,
                &context,
                kind_budget,
            ),
            Err(SettlementVerificationBudgetError::LimitExceeded {
                resource: SettlementVerificationResource::DistinctEvidenceKinds,
                ..
            })
        ));
    }

    #[test]
    fn accounting_is_permutation_invariant_and_rejected_states_still_cost_budget() {
        let profile = profile();
        let context = context();
        let subject = subject(&profile);
        let first = observation(&profile, "op:1", "source:a", 1);
        let mut second = observation(&profile, "op:2", "source:b", 1);
        second.state = ObservedSettlementState::Rejected;

        let left = assess_settlement_verification_budget_v1(
            &subject,
            &profile,
            &[first.clone(), second.clone()],
            &context,
            generous_budget(),
        )
        .expect("left admitted");
        let right = assess_settlement_verification_budget_v1(
            &subject,
            &profile,
            &[second, first],
            &context,
            generous_budget(),
        )
        .expect("right admitted");
        assert_eq!(left, right);
        assert_eq!(left.observations, 2);
        assert_eq!(left.total_evidence_items, 2);
    }

    #[test]
    fn exact_total_boundaries_and_arithmetic_overflow_fail_closed() {
        let profile = profile();
        let context = context();
        let subject = subject(&profile);
        let observations = [observation(&profile, "op:1", "source:a", 1)];
        let usage = assess_settlement_verification_budget_v1(
            &subject,
            &profile,
            &observations,
            &context,
            generous_budget(),
        )
        .expect("baseline");

        let mut exact = generous_budget();
        exact.max_total_charged_canonical_bytes = usage.total_charged_canonical_bytes;
        exact.max_total_hash_input_bytes = usage.total_hash_input_bytes_upper_bound;
        assert!(assess_settlement_verification_budget_v1(
            &subject,
            &profile,
            &observations,
            &context,
            exact,
        )
        .is_ok());

        let mut too_small = exact;
        too_small.max_total_charged_canonical_bytes -= 1;
        assert!(matches!(
            assess_settlement_verification_budget_v1(
                &subject,
                &profile,
                &observations,
                &context,
                too_small,
            ),
            Err(SettlementVerificationBudgetError::LimitExceeded {
                resource: SettlementVerificationResource::TotalChargedCanonicalBytes,
                ..
            })
        ));

        assert_eq!(
            checked_add(u64::MAX, 1),
            Err(SettlementVerificationBudgetError::AccountingOverflow)
        );
        assert_eq!(
            checked_mul(u64::MAX, 2),
            Err(SettlementVerificationBudgetError::AccountingOverflow)
        );
    }
}
