use std::collections::BTreeSet;

use mycelix_business_coordination::{
    ActionContract, AggregateCheckRef, AggregatePolicyKey, CoordinationEnvelope,
    ReversibilityClass, UnknownOutcomePolicy,
};
use mycelix_business_core::{
    ActionContractRef, AuthorityLeaseRef, AuthorityScope, AuthorizedIntentRef, BudgetLimit,
    DecisionFrontiers, Digest32, ExecutionAttemptRef, FrontierRef, ObservationRef, PreparedAction,
    ProposalRef, ReferenceId, ReservationId, ReservationRef, RiskClass, ScopeRef, SubjectRef,
};
use mycelix_business_decision::{
    AuthorizationBinding, DecisionCapsule, DecisionCapsuleRef, ExecutionAttemptRecord,
};
use mycelix_business_revalidation::{
    ExecutionRevalidationContract, FrontierClass, RevalidationEvidence, RevalidationRule,
};
use mycelix_finance_exact::{AssetAmount, AssetId};
use mycelix_finance_settlement::{
    qualify_settlement, FinalityEvidence, FinalityProfile, FinalityProfileRef,
    ObservedSettlementState, ReversalModel, SettlementObservation, SettlementSubject,
};

use super::*;

const AUTHORITY_EPOCH: u64 = 7;
const ATTEMPT_AT: u64 = 400;

fn reference(value: &str) -> ReferenceId {
    ReferenceId::new(value).expect("static canonical reference")
}

fn subject(value: &str) -> SubjectRef {
    SubjectRef(reference(value))
}

fn action_contract_ref() -> ActionContractRef {
    ActionContractRef {
        semantic_id: reference("mycelix.finance.pay.v1"),
        digest: Digest32::repeat(1),
    }
}

fn decision_ref() -> DecisionCapsuleRef {
    DecisionCapsuleRef {
        id: reference("decision:1"),
        digest: Digest32::repeat(2),
    }
}

fn intent_ref() -> AuthorizedIntentRef {
    AuthorizedIntentRef(reference("intent:1"))
}

fn attempt_ref(value: &str) -> ExecutionAttemptRef {
    ExecutionAttemptRef(reference(value))
}

fn profile_ref(value: &str) -> FinalityProfileRef {
    FinalityProfileRef {
        id: reference(value),
        revision: 1,
        digest: Digest32::repeat(30),
    }
}

fn usd(amount: u64) -> AssetAmount {
    AssetAmount::new(
        amount,
        AssetId::new("USD.micro").expect("static asset identifier"),
    )
}

fn policy_frontier() -> FrontierRef {
    FrontierRef {
        domain: reference("policy:finance-spend"),
        sequence: 1,
        digest: Digest32::repeat(12),
    }
}

#[derive(Clone)]
struct Fixture {
    decision: DecisionCapsule,
    coordination: CoordinationEnvelope,
    authorization: AuthorizationBinding,
    attempt: ExecutionAttemptRecord,
    request: FinancialReservationRequest,
    reservation: FinancialReservation,
    revalidation_contract: ExecutionRevalidationContract,
    revalidation_evidence: Vec<RevalidationEvidence>,
}

fn fixture() -> Fixture {
    let finance_domain = reference("finance");
    let treasury_subject = subject("treasury:operating");
    let reservation_id = ReservationId(reference("reservation:finance:1"));
    let reservation_digest = Digest32::repeat(31);
    let finality_profile = profile_ref("finality:bank:v1");
    let aggregate_key = reference("aggregate:treasury:24h");
    let frontier = policy_frontier();

    let authority_lease = AuthorityLeaseRef {
        lease_id: reference("authority:lease:1"),
        authority_epoch: AUTHORITY_EPOCH,
        sequence: 1,
        fencing_token: 11,
        scope: ScopeRef(reference("scope:treasury")),
        issued_at_unix_ms: 100,
        expires_at_unix_ms: 1_000,
    };

    let prepared = PreparedAction {
        action_contract: action_contract_ref(),
        decision_capsule: decision_ref().id.clone(),
        intent_digest: Digest32::repeat(4),
        frontiers: DecisionFrontiers {
            observation: vec![],
            policy: vec![frontier.clone()],
            authority: vec![],
        },
        authority_lease: authority_lease.clone(),
        reservations: vec![ReservationRef {
            domain: finance_domain.clone(),
            reservation_id: reservation_id.clone(),
            subject: treasury_subject.clone(),
            digest: reservation_digest,
            expires_at_unix_ms: 800,
        }],
        idempotency_key: reference("payment:idem:1"),
        prepared_at_unix_ms: 300,
        expires_at_unix_ms: 700,
    };

    let contract = ActionContract {
        contract_ref: action_contract_ref(),
        required_reservation_domains: BTreeSet::from([finance_domain.clone()]),
        required_capabilities: BTreeSet::new(),
        freshness: vec![],
        aggregate_policy_keys: vec![AggregatePolicyKey {
            key: aggregate_key.clone(),
            window_ms: 86_400_000,
        }],
        reversibility: ReversibilityClass::Compensatable,
        maximum_prepared_lifetime_ms: 500,
        unknown_outcome_policy: UnknownOutcomePolicy::ReconcileBeforeRetry,
    };

    let coordination = CoordinationEnvelope {
        contract,
        prepared,
        authority_scope: AuthorityScope {
            capabilities: BTreeSet::new(),
            subjects: BTreeSet::from([treasury_subject.clone()]),
            risk_ceiling: RiskClass::Moderate,
            budget: BudgetLimit::Limited(1_000),
            expires_at_unix_ms: 1_000,
        },
        observation_ages: vec![],
        aggregate_checks: vec![AggregateCheckRef {
            key: aggregate_key.clone(),
            digest: Digest32::repeat(13),
        }],
        coordination_digest: Digest32::repeat(14),
    };

    let recommendation = ProposalRef(reference("proposal:pay"));
    let decision = DecisionCapsule {
        capsule_ref: decision_ref(),
        subject: subject("business:purchase:1"),
        action_contract: action_contract_ref(),
        frontiers: coordination.prepared.frontiers.clone(),
        observations: BTreeSet::from([ObservationRef(reference("observation:need-payment"))]),
        estimates: BTreeSet::new(),
        forecasts: BTreeSet::new(),
        assumptions: BTreeSet::new(),
        conflicts: BTreeSet::new(),
        alternatives: BTreeSet::from([recommendation.clone()]),
        recommendation,
        required_capabilities: BTreeSet::new(),
        model_lineage: reference("model:finance-policy:v1"),
        objective_contract_digest: Digest32::repeat(15),
        human_explanation: None,
        created_at_unix_ms: 100,
        valid_until_unix_ms: 900,
    };

    let authorization = AuthorizationBinding {
        decision: decision_ref(),
        authorized_intent: intent_ref(),
        action_contract: action_contract_ref(),
        intent_digest: Digest32::repeat(4),
        authority_lease_id: authority_lease.lease_id.clone(),
        authority_epoch: AUTHORITY_EPOCH,
        fencing_token: 11,
        approved_at_unix_ms: 150,
        approval_evidence: BTreeSet::from([reference("evidence:approval:1")]),
    };

    let attempt = ExecutionAttemptRecord {
        attempt: attempt_ref("attempt:1"),
        decision: decision_ref(),
        authorized_intent: intent_ref(),
        coordination_digest: coordination.coordination_digest,
        idempotency_key: coordination.prepared.idempotency_key.clone(),
        provider: reference("provider:bank-adapter"),
        started_at_unix_ms: ATTEMPT_AT,
    };

    let request = FinancialReservationRequest {
        request_id: reference("finance-request:1"),
        finance_domain: finance_domain.clone(),
        action_contract: action_contract_ref(),
        decision: decision_ref(),
        authorized_intent: intent_ref(),
        intent_digest: Digest32::repeat(4),
        authority_lease_id: authority_lease.lease_id.clone(),
        authority_epoch: AUTHORITY_EPOCH,
        fencing_token: 11,
        subject: treasury_subject.clone(),
        amount: usd(100),
        effect_class: reference("finance-effect:payment"),
        aggregate_policy_keys: BTreeSet::from([aggregate_key]),
        idempotency_key: coordination.prepared.idempotency_key.clone(),
        required_finality_profile: finality_profile.clone(),
        requested_at_unix_ms: 200,
        expires_at_unix_ms: 900,
    };

    let reservation = FinancialReservation {
        reservation_id,
        request_id: request.request_id.clone(),
        finance_domain,
        subject: treasury_subject,
        amount: request.amount.clone(),
        effect_class: request.effect_class.clone(),
        action_contract: request.action_contract.clone(),
        decision: request.decision.clone(),
        authorized_intent: request.authorized_intent.clone(),
        intent_digest: request.intent_digest,
        authority_lease_id: request.authority_lease_id.clone(),
        authority_epoch: request.authority_epoch,
        fencing_token: request.fencing_token,
        aggregate_policy_keys: request.aggregate_policy_keys.clone(),
        idempotency_key: request.idempotency_key.clone(),
        required_finality_profile: finality_profile,
        reservation_digest,
        finance_frontier: Digest32::repeat(32),
        issued_at_unix_ms: 250,
        expires_at_unix_ms: 800,
    };

    let revalidation_contract = ExecutionRevalidationContract {
        action_contract: action_contract_ref(),
        rules: vec![RevalidationRule::ExactFrontier {
            class: FrontierClass::Policy,
            expected: frontier.clone(),
        }],
        maximum_check_age_ms: 50,
        contract_digest: Digest32::repeat(33),
    };

    let revalidation_evidence = vec![RevalidationEvidence::ExactFrontier {
        class: FrontierClass::Policy,
        current: frontier,
        checked_at_unix_ms: 390,
    }];

    Fixture {
        decision,
        coordination,
        authorization,
        attempt,
        request,
        reservation,
        revalidation_contract,
        revalidation_evidence,
    }
}

fn bind(f: &Fixture) -> Result<FinanceBoundForAttempt, FinanceBindingError> {
    bind_finance_for_attempt(
        &f.decision,
        &f.coordination,
        &f.authorization,
        &f.attempt,
        &f.request,
        &f.reservation,
        &f.revalidation_contract,
        &f.revalidation_evidence,
        AUTHORITY_EPOCH,
        ATTEMPT_AT,
    )
}

include!("tests_binding.inc");
include!("tests_projection.inc");
