use std::collections::BTreeSet;

use mycelix_business_core::{Digest32, ExecutionAttemptRef, ReferenceId};
use mycelix_finance_exact::{AssetAmount, AssetId};
use serde_json::Value;

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

fn context(time: u64) -> SettlementEvaluationContext {
    SettlementEvaluationContext::deterministic_supplied(
        time,
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
    revision: u64,
    state: ObservedSettlementState,
    observed_at_unix_ms: u64,
) -> SettlementObservation {
    let profile = profile();
    let subject = subject(&profile);
    let observation_id = reference(format!("observation:op:1:{revision}").as_str());
    SettlementObservation {
        observation_id: observation_id.clone(),
        subject: subject.id,
        financial_effect_commitment: subject.financial_effect_commitment,
        attempt: subject.attempt,
        rail: subject.rail,
        network: subject.network,
        operation_id: reference("op:1"),
        revision,
        amount: amount(100),
        state,
        observed_at_unix_ms,
        evidence: vec![FinalityEvidence {
            evidence_id: reference(
                format!("evidence:op:1:{revision}:provider-a").as_str(),
            ),
            subject: reference("settlement:subject:1"),
            operation_id: reference("op:1"),
            operation_revision: revision,
            observation_id,
            kind: reference("evidence:provider-settlement"),
            source: reference("source:provider-a"),
            digest: Digest32::repeat(7),
        }],
    }
}

fn fixture() -> Value {
    serde_json::from_str(include_str!(
        "../test-vectors/invalidation-receipt-v1.json"
    ))
    .expect("checked-in invalidation receipt vector must parse")
}

fn bytes_hex(bytes: &[u8]) -> String {
    let mut out = String::with_capacity(bytes.len() * 2);
    for byte in bytes {
        use std::fmt::Write as _;
        write!(&mut out, "{byte:02x}").expect("writing to string cannot fail");
    }
    out
}

fn digest_hex(digest: Digest32) -> String {
    bytes_hex(&digest.0)
}

#[test]
fn invalidation_receipt_golden_vector_reproduces_exact_bytes_and_hashes() {
    let vector = fixture();
    let profile = profile();
    let subject = subject(&profile);
    let prior = qualify_settlement(
        &subject,
        &profile,
        &[observation(1, ObservedSettlementState::Applied, 1_000)],
        &context(1_100),
    )
    .expect("prior qualification");
    let reversal = observation(2, ObservedSettlementState::Reversed, 1_200);
    let invalidation_context = context(1_250);

    let evidence = &reversal.evidence[0];
    let evidence_bytes = canonical_evidence_bytes(evidence).expect("canonical evidence");
    assert_eq!(
        evidence_bytes.len() as u64,
        vector["invalidating_evidence"]["canonical_length"]
            .as_u64()
            .expect("vector integer")
    );
    assert_eq!(
        bytes_hex(&evidence_bytes),
        vector["invalidating_evidence"]["canonical_hex"]
            .as_str()
            .expect("vector string")
    );
    assert_eq!(
        digest_hex(evidence_commitment(evidence).expect("evidence commitment")),
        vector["invalidating_evidence"]["commitment_hex"]
            .as_str()
            .expect("vector string")
    );

    let observation_bytes =
        canonical_observation_bytes(&reversal).expect("canonical invalidating observation");
    assert_eq!(
        observation_bytes.len() as u64,
        vector["invalidating_observation"]["canonical_length"]
            .as_u64()
            .expect("vector integer")
    );
    assert_eq!(
        bytes_hex(&observation_bytes),
        vector["invalidating_observation"]["canonical_hex"]
            .as_str()
            .expect("vector string")
    );
    assert_eq!(
        digest_hex(observation_commitment(&reversal).expect("observation commitment")),
        vector["invalidating_observation"]["commitment_hex"]
            .as_str()
            .expect("vector string")
    );

    let context_bytes = canonical_evaluation_context_bytes(&invalidation_context)
        .expect("canonical evaluation context");
    assert_eq!(
        context_bytes.len() as u64,
        vector["evaluation_context"]["canonical_length"]
            .as_u64()
            .expect("vector integer")
    );
    assert_eq!(
        bytes_hex(&context_bytes),
        vector["evaluation_context"]["canonical_hex"]
            .as_str()
            .expect("vector string")
    );
    assert_eq!(
        digest_hex(invalidation_context.commitment()),
        vector["evaluation_context"]["commitment_hex"]
            .as_str()
            .expect("vector string")
    );

    let receipt =
        derive_invalidation_with_receipt(&prior, &profile, &reversal, &invalidation_context)
            .expect("invalidation receipt");
    let receipt_bytes = canonical_settlement_invalidation_receipt_bytes(
        receipt.invalidation(),
        receipt.evaluation_context_commitment(),
        receipt.evaluation_context_class(),
    )
    .expect("canonical invalidation receipt");
    assert_eq!(
        receipt_bytes.len() as u64,
        vector["receipt"]["canonical_length"]
            .as_u64()
            .expect("vector integer")
    );
    assert_eq!(
        bytes_hex(&receipt_bytes),
        vector["receipt"]["canonical_hex"]
            .as_str()
            .expect("vector string")
    );
    assert_eq!(
        digest_hex(receipt.receipt_commitment()),
        vector["receipt"]["commitment_hex"]
            .as_str()
            .expect("vector string")
    );
}
