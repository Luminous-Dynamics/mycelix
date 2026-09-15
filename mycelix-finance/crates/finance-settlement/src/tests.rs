use super::*;
use mycelix_business_core::{Digest32, ExecutionAttemptRef, ReferenceId};
use mycelix_finance_exact::{AssetAmount, AssetId};

fn reference(value: &str) -> ReferenceId {
    ReferenceId::new(value).expect("static reference")
}

fn attempt(value: &str) -> ExecutionAttemptRef {
    ExecutionAttemptRef::new(value).expect("static attempt")
}

fn amount(units: u64) -> AssetAmount {
    AssetAmount::new(units, AssetId::new("USD.micro").expect("static asset"))
}

fn profile() -> FinalityProfile {
    FinalityProfile {
        profile_ref: FinalityProfileRef {
            id: reference("finality:bank:v1"),
            revision: 1,
            digest: Digest32::repeat(9),
        },
        rail: reference("rail:bank"),
        network: reference("network:test-bank"),
        required_evidence_kinds: [reference("evidence:provider-settlement")].into_iter().collect(),
        min_distinct_sources: 1,
        max_observation_age_ms: 60_000,
        reversal_model: ReversalModel::MayReverse,
    }
}

fn subject() -> SettlementSubject {
    let profile = profile();
    SettlementSubject {
        id: reference("settlement:subject:1"),
        attempt: attempt("attempt:1"),
        rail: profile.rail.clone(),
        network: profile.network.clone(),
        amount: amount(100),
        required_profile: profile.profile_ref,
    }
}

fn evidence(operation: &str, suffix: &str) -> FinalityEvidence {
    FinalityEvidence {
        evidence_id: reference(&format!("evidence:{operation}:{suffix}")),
        subject: reference("settlement:subject:1"),
        operation_id: reference(operation),
        kind: reference("evidence:provider-settlement"),
        source: reference(&format!("source:{suffix}")),
        digest: Digest32::repeat(7),
    }
}

fn observation(
    operation: &str,
    revision: u64,
    units: u64,
    state: ObservedSettlementState,
    observed_at: u64,
) -> SettlementObservation {
    let subject = subject();
    SettlementObservation {
        observation_id: reference(&format!("observation:{operation}:{revision}")),
        subject: subject.id,
        attempt: subject.attempt,
        rail: subject.rail,
        network: subject.network,
        operation_id: reference(operation),
        revision,
        amount: amount(units),
        state,
        observed_at_unix_ms: observed_at,
        evidence: vec![evidence(operation, "provider-a")],
    }
}

include!("tests_a.inc");
include!("tests_b.inc");
