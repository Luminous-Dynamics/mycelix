#![forbid(unsafe_code)]

//! Deterministic assurance fixtures for the LISE Institutional Performance
//! Benchmark. This first slice tests conflicts of interest and scoped
//! expenditure authority without changing production governance behavior.

pub mod capture;

use std::collections::BTreeSet;

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

pub const RECEIPT_SCHEMA: &str = "lise.ipb.receipt.v0";
pub const SUITE_ID: &str = "IPB-S2-COI-v0";

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Decision {
    Permit,
    Deny,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ReasonCode {
    Authorized,
    InvalidAmount,
    ProposalMismatch,
    ConflictScreenIncomplete,
    ConflictOfInterest,
    MissingAuthority,
    AuthorityHolderMismatch,
    WrongAuthorityScope,
    WrongResource,
    InvalidAuthorityWindow,
    AuthorityNotYetValid,
    AuthorityExpired,
    AmountExceedsAuthority,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum AuthorityScope {
    ExpenditureApproval,
    Other,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct Policy {
    pub policy_id: String,
    pub version: u32,
    pub require_conflict_screen: bool,
    pub deny_proposer_and_beneficiary_approval: bool,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct AuthorityGrant {
    pub grant_id: String,
    pub subject_id: String,
    pub scope: AuthorityScope,
    pub resource_id: String,
    /// Inclusive lower bound in deterministic fixture time (milliseconds).
    pub valid_from_ms: i64,
    /// Exclusive upper bound in deterministic fixture time (milliseconds).
    pub valid_until_ms: i64,
    pub max_amount_minor: u64,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct Proposal {
    pub proposal_id: String,
    pub proposer_id: String,
    pub beneficiary_id: String,
    pub resource_id: String,
    pub amount_minor: u64,
    pub conflict_screen_complete: bool,
    /// Conflicts established by independent fixture input. This is test
    /// evidence, not an automated inference about real-world interests.
    pub conflicted_actors: BTreeSet<String>,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct ApprovalAttempt {
    pub attempt_id: String,
    pub actor_id: String,
    pub proposal_id: String,
    pub grant_id: String,
    pub at_ms: i64,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct EvaluationInput {
    pub policy: Policy,
    pub proposal: Proposal,
    pub attempt: ApprovalAttempt,
    pub grant: Option<AuthorityGrant>,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct EvidenceReceipt {
    pub schema: String,
    pub suite_id: String,
    pub scenario_id: String,
    pub policy_id: String,
    pub policy_version: u32,
    pub proposal_id: String,
    pub attempt_id: String,
    pub actor_id: String,
    pub grant_id: String,
    pub decision: Decision,
    pub reason: ReasonCode,
    pub policy_sha256: String,
    pub input_sha256: String,
    pub receipt_sha256: String,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct BenchmarkCaseResult {
    pub scenario_id: String,
    pub expected_decision: Decision,
    pub passed: bool,
    pub receipt: EvidenceReceipt,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct BenchmarkSuiteReport {
    pub schema: String,
    pub suite_id: String,
    pub total_cases: usize,
    pub passed_cases: usize,
    pub failed_cases: usize,
    pub cases: Vec<BenchmarkCaseResult>,
}

#[derive(Clone, Debug)]
struct FixtureCase {
    scenario_id: &'static str,
    input: EvaluationInput,
    expected: Decision,
}

fn digest<T: Serialize>(value: &T) -> String {
    let bytes =
        serde_json::to_vec(value).expect("LISE fixture schema contains only serializable values");
    let hash = Sha256::digest(bytes);
    let mut encoded = String::with_capacity(7 + hash.len() * 2);
    encoded.push_str("sha256:");
    for byte in hash {
        use std::fmt::Write;
        write!(&mut encoded, "{byte:02x}").expect("writing to String is infallible");
    }
    encoded
}

fn decide(input: &EvaluationInput) -> (Decision, ReasonCode) {
    let proposal = &input.proposal;
    let attempt = &input.attempt;

    if proposal.amount_minor == 0 {
        return (Decision::Deny, ReasonCode::InvalidAmount);
    }
    if proposal.proposal_id != attempt.proposal_id {
        return (Decision::Deny, ReasonCode::ProposalMismatch);
    }
    if input.policy.require_conflict_screen && !proposal.conflict_screen_complete {
        return (Decision::Deny, ReasonCode::ConflictScreenIncomplete);
    }

    let directly_interested = proposal.conflicted_actors.contains(&attempt.actor_id);
    let proposer_or_beneficiary = input.policy.deny_proposer_and_beneficiary_approval
        && (attempt.actor_id == proposal.proposer_id
            || attempt.actor_id == proposal.beneficiary_id);
    if directly_interested || proposer_or_beneficiary {
        return (Decision::Deny, ReasonCode::ConflictOfInterest);
    }

    let Some(grant) = input.grant.as_ref() else {
        return (Decision::Deny, ReasonCode::MissingAuthority);
    };
    if grant.grant_id != attempt.grant_id {
        return (Decision::Deny, ReasonCode::MissingAuthority);
    }
    if grant.subject_id != attempt.actor_id {
        return (Decision::Deny, ReasonCode::AuthorityHolderMismatch);
    }
    if grant.scope != AuthorityScope::ExpenditureApproval {
        return (Decision::Deny, ReasonCode::WrongAuthorityScope);
    }
    if grant.resource_id != proposal.resource_id {
        return (Decision::Deny, ReasonCode::WrongResource);
    }
    if grant.valid_until_ms <= grant.valid_from_ms {
        return (Decision::Deny, ReasonCode::InvalidAuthorityWindow);
    }
    if attempt.at_ms < grant.valid_from_ms {
        return (Decision::Deny, ReasonCode::AuthorityNotYetValid);
    }
    if attempt.at_ms >= grant.valid_until_ms {
        return (Decision::Deny, ReasonCode::AuthorityExpired);
    }
    if proposal.amount_minor > grant.max_amount_minor {
        return (Decision::Deny, ReasonCode::AmountExceedsAuthority);
    }

    (Decision::Permit, ReasonCode::Authorized)
}

/// Evaluate one approval attempt and return a deterministic evidence receipt.
/// This research fixture is not a production authorizer.
pub fn evaluate_approval(scenario_id: &str, input: &EvaluationInput) -> EvidenceReceipt {
    let (decision, reason) = decide(input);
    let policy_sha256 = digest(&input.policy);
    let input_sha256 = digest(&(&input.proposal, &input.attempt, &input.grant));

    #[derive(Serialize)]
    struct ReceiptMaterial<'a> {
        schema: &'static str,
        suite_id: &'static str,
        scenario_id: &'a str,
        policy_id: &'a str,
        policy_version: u32,
        proposal_id: &'a str,
        attempt_id: &'a str,
        actor_id: &'a str,
        grant_id: &'a str,
        decision: Decision,
        reason: ReasonCode,
        policy_sha256: &'a str,
        input_sha256: &'a str,
    }

    let material = ReceiptMaterial {
        schema: RECEIPT_SCHEMA,
        suite_id: SUITE_ID,
        scenario_id,
        policy_id: &input.policy.policy_id,
        policy_version: input.policy.version,
        proposal_id: &input.proposal.proposal_id,
        attempt_id: &input.attempt.attempt_id,
        actor_id: &input.attempt.actor_id,
        grant_id: &input.attempt.grant_id,
        decision,
        reason,
        policy_sha256: &policy_sha256,
        input_sha256: &input_sha256,
    };
    let receipt_sha256 = digest(&material);

    EvidenceReceipt {
        schema: RECEIPT_SCHEMA.to_owned(),
        suite_id: SUITE_ID.to_owned(),
        scenario_id: scenario_id.to_owned(),
        policy_id: input.policy.policy_id.clone(),
        policy_version: input.policy.version,
        proposal_id: input.proposal.proposal_id.clone(),
        attempt_id: input.attempt.attempt_id.clone(),
        actor_id: input.attempt.actor_id.clone(),
        grant_id: input.attempt.grant_id.clone(),
        decision,
        reason,
        policy_sha256,
        input_sha256,
        receipt_sha256,
    }
}

fn baseline_policy() -> Policy {
    Policy {
        policy_id: "lise-coi-policy".to_owned(),
        version: 1,
        require_conflict_screen: true,
        deny_proposer_and_beneficiary_approval: true,
    }
}

fn proposal(conflicted_actors: &[&str], screen_complete: bool) -> Proposal {
    Proposal {
        proposal_id: "expense-001".to_owned(),
        proposer_id: "member-initiator".to_owned(),
        beneficiary_id: "vendor-42".to_owned(),
        resource_id: "community-fund".to_owned(),
        amount_minor: 5_000,
        conflict_screen_complete: screen_complete,
        conflicted_actors: conflicted_actors.iter().map(|s| (*s).to_owned()).collect(),
    }
}

fn grant(
    grant_id: &str,
    subject_id: &str,
    scope: AuthorityScope,
    resource_id: &str,
    valid_from_ms: i64,
    valid_until_ms: i64,
    max_amount_minor: u64,
) -> AuthorityGrant {
    AuthorityGrant {
        grant_id: grant_id.to_owned(),
        subject_id: subject_id.to_owned(),
        scope,
        resource_id: resource_id.to_owned(),
        valid_from_ms,
        valid_until_ms,
        max_amount_minor,
    }
}

fn attempt(actor_id: &str, grant_id: &str, at_ms: i64) -> ApprovalAttempt {
    ApprovalAttempt {
        attempt_id: format!("attempt-{actor_id}"),
        actor_id: actor_id.to_owned(),
        proposal_id: "expense-001".to_owned(),
        grant_id: grant_id.to_owned(),
        at_ms,
    }
}

fn cases() -> Vec<FixtureCase> {
    let policy = baseline_policy();
    let normal_grant = grant(
        "grant-independent-reviewer",
        "independent-reviewer",
        AuthorityScope::ExpenditureApproval,
        "community-fund",
        0,
        10_000,
        10_000,
    );

    vec![
        FixtureCase {
            scenario_id: "s2-positive-independent-authorized-approver",
            input: EvaluationInput {
                policy: policy.clone(),
                proposal: proposal(&[], true),
                attempt: attempt("independent-reviewer", &normal_grant.grant_id, 1_000),
                grant: Some(normal_grant.clone()),
            },
            expected: Decision::Permit,
        },
        FixtureCase {
            scenario_id: "s2-positive-exact-authority-limit",
            input: EvaluationInput {
                policy: policy.clone(),
                proposal: Proposal {
                    amount_minor: 10_000,
                    ..proposal(&[], true)
                },
                attempt: attempt("independent-reviewer", &normal_grant.grant_id, 1_000),
                grant: Some(normal_grant.clone()),
            },
            expected: Decision::Permit,
        },
        FixtureCase {
            scenario_id: "s2-positive-at-inclusive-grant-start",
            input: EvaluationInput {
                policy: policy.clone(),
                proposal: proposal(&[], true),
                attempt: attempt("independent-reviewer", &normal_grant.grant_id, 0),
                grant: Some(normal_grant.clone()),
            },
            expected: Decision::Permit,
        },
        FixtureCase {
            scenario_id: "s2-positive-before-grant-expiry",
            input: EvaluationInput {
                policy: policy.clone(),
                proposal: proposal(&[], true),
                attempt: attempt("independent-reviewer", &normal_grant.grant_id, 9_999),
                grant: Some(normal_grant.clone()),
            },
            expected: Decision::Permit,
        },
        FixtureCase {
            scenario_id: "s2-positive-substitute-approver-after-proposer-recusal",
            input: EvaluationInput {
                policy: policy.clone(),
                proposal: Proposal {
                    conflicted_actors: BTreeSet::from(["member-initiator".to_owned()]),
                    ..proposal(&[], true)
                },
                attempt: attempt("independent-reviewer", &normal_grant.grant_id, 1_000),
                grant: Some(normal_grant.clone()),
            },
            expected: Decision::Permit,
        },
        FixtureCase {
            scenario_id: "s2-positive-matched-resource-scope",
            input: EvaluationInput {
                policy: policy.clone(),
                proposal: Proposal {
                    resource_id: "community-reserve".to_owned(),
                    ..proposal(&[], true)
                },
                attempt: attempt("independent-reviewer", "grant-community-reserve", 1_000),
                grant: Some(grant(
                    "grant-community-reserve",
                    "independent-reviewer",
                    AuthorityScope::ExpenditureApproval,
                    "community-reserve",
                    0,
                    10_000,
                    10_000,
                )),
            },
            expected: Decision::Permit,
        },
        FixtureCase {
            scenario_id: "s2-negative-conflicted-beneficiary",
            input: EvaluationInput {
                policy: policy.clone(),
                proposal: proposal(&[], true),
                attempt: attempt("vendor-42", "grant-beneficiary", 1_000),
                grant: Some(grant(
                    "grant-beneficiary",
                    "vendor-42",
                    AuthorityScope::ExpenditureApproval,
                    "community-fund",
                    0,
                    10_000,
                    10_000,
                )),
            },
            expected: Decision::Deny,
        },
        FixtureCase {
            scenario_id: "s2-negative-proposer-self-approval",
            input: EvaluationInput {
                policy: policy.clone(),
                proposal: proposal(&[], true),
                attempt: attempt("member-initiator", "grant-initiator", 1_000),
                grant: Some(grant(
                    "grant-initiator",
                    "member-initiator",
                    AuthorityScope::ExpenditureApproval,
                    "community-fund",
                    0,
                    10_000,
                    10_000,
                )),
            },
            expected: Decision::Deny,
        },
        FixtureCase {
            scenario_id: "s2-negative-screened-conflict",
            input: EvaluationInput {
                policy: policy.clone(),
                proposal: proposal(&["reviewer-with-interest"], true),
                attempt: attempt("reviewer-with-interest", "grant-interested", 1_000),
                grant: Some(grant(
                    "grant-interested",
                    "reviewer-with-interest",
                    AuthorityScope::ExpenditureApproval,
                    "community-fund",
                    0,
                    10_000,
                    10_000,
                )),
            },
            expected: Decision::Deny,
        },
        FixtureCase {
            scenario_id: "s2-negative-incomplete-conflict-screen",
            input: EvaluationInput {
                policy: policy.clone(),
                proposal: proposal(&[], false),
                attempt: attempt("independent-reviewer", &normal_grant.grant_id, 1_000),
                grant: Some(normal_grant.clone()),
            },
            expected: Decision::Deny,
        },
        FixtureCase {
            scenario_id: "s2-negative-missing-authority",
            input: EvaluationInput {
                policy: policy.clone(),
                proposal: proposal(&[], true),
                attempt: attempt("independent-reviewer", "grant-missing", 1_000),
                grant: None,
            },
            expected: Decision::Deny,
        },
        FixtureCase {
            scenario_id: "s2-negative-expired-authority",
            input: EvaluationInput {
                policy: policy.clone(),
                proposal: proposal(&[], true),
                attempt: attempt("independent-reviewer", "grant-expired", 10_000),
                grant: Some(grant(
                    "grant-expired",
                    "independent-reviewer",
                    AuthorityScope::ExpenditureApproval,
                    "community-fund",
                    0,
                    10_000,
                    10_000,
                )),
            },
            expected: Decision::Deny,
        },
        FixtureCase {
            scenario_id: "s2-negative-wrong-authority-scope",
            input: EvaluationInput {
                policy: policy.clone(),
                proposal: proposal(&[], true),
                attempt: attempt("independent-reviewer", "grant-delegation-only", 1_000),
                grant: Some(grant(
                    "grant-delegation-only",
                    "independent-reviewer",
                    AuthorityScope::Other,
                    "community-fund",
                    0,
                    10_000,
                    10_000,
                )),
            },
            expected: Decision::Deny,
        },
        FixtureCase {
            scenario_id: "s2-negative-resource-scope",
            input: EvaluationInput {
                policy: policy.clone(),
                proposal: proposal(&[], true),
                attempt: attempt("independent-reviewer", "grant-other-resource", 1_000),
                grant: Some(grant(
                    "grant-other-resource",
                    "independent-reviewer",
                    AuthorityScope::ExpenditureApproval,
                    "different-fund",
                    0,
                    10_000,
                    10_000,
                )),
            },
            expected: Decision::Deny,
        },
        FixtureCase {
            scenario_id: "s2-negative-amount-exceeds-grant",
            input: EvaluationInput {
                policy: policy.clone(),
                proposal: proposal(&[], true),
                attempt: attempt("independent-reviewer", "grant-low-limit", 1_000),
                grant: Some(grant(
                    "grant-low-limit",
                    "independent-reviewer",
                    AuthorityScope::ExpenditureApproval,
                    "community-fund",
                    0,
                    10_000,
                    4_999,
                )),
            },
            expected: Decision::Deny,
        },
        FixtureCase {
            scenario_id: "s2-negative-proposal-substitution",
            input: EvaluationInput {
                policy: policy.clone(),
                proposal: Proposal {
                    proposal_id: "different-proposal".to_owned(),
                    ..proposal(&[], true)
                },
                attempt: attempt("independent-reviewer", "grant-independent-reviewer", 1_000),
                grant: Some(normal_grant.clone()),
            },
            expected: Decision::Deny,
        },
        FixtureCase {
            scenario_id: "s2-negative-authority-holder-mismatch",
            input: EvaluationInput {
                policy: policy.clone(),
                proposal: proposal(&[], true),
                attempt: attempt("independent-reviewer", "grant-other-holder", 1_000),
                grant: Some(grant(
                    "grant-other-holder",
                    "different-reviewer",
                    AuthorityScope::ExpenditureApproval,
                    "community-fund",
                    0,
                    10_000,
                    10_000,
                )),
            },
            expected: Decision::Deny,
        },
        FixtureCase {
            scenario_id: "s2-negative-authority-not-yet-valid",
            input: EvaluationInput {
                policy: policy.clone(),
                proposal: proposal(&[], true),
                attempt: attempt("independent-reviewer", "grant-future", 1_000),
                grant: Some(grant(
                    "grant-future",
                    "independent-reviewer",
                    AuthorityScope::ExpenditureApproval,
                    "community-fund",
                    1_500,
                    10_000,
                    10_000,
                )),
            },
            expected: Decision::Deny,
        },
        FixtureCase {
            scenario_id: "s2-negative-invalid-authority-window",
            input: EvaluationInput {
                policy: policy.clone(),
                proposal: proposal(&[], true),
                attempt: attempt("independent-reviewer", "grant-invalid-window", 2_000),
                grant: Some(grant(
                    "grant-invalid-window",
                    "independent-reviewer",
                    AuthorityScope::ExpenditureApproval,
                    "community-fund",
                    2_000,
                    2_000,
                    10_000,
                )),
            },
            expected: Decision::Deny,
        },
        FixtureCase {
            scenario_id: "s2-negative-zero-amount",
            input: EvaluationInput {
                policy,
                proposal: Proposal {
                    amount_minor: 0,
                    ..proposal(&[], true)
                },
                attempt: attempt("independent-reviewer", "grant-independent-reviewer", 1_000),
                grant: Some(normal_grant),
            },
            expected: Decision::Deny,
        },
    ]
}

/// Run the fixed S2 conflict-of-interest fixture suite. It uses no RNG,
/// wall-clock dependency, network or external service.
pub fn conflict_of_interest_suite() -> BenchmarkSuiteReport {
    let case_results: Vec<BenchmarkCaseResult> = cases()
        .into_iter()
        .map(|case| {
            let receipt = evaluate_approval(case.scenario_id, &case.input);
            BenchmarkCaseResult {
                scenario_id: case.scenario_id.to_owned(),
                expected_decision: case.expected,
                passed: receipt.decision == case.expected,
                receipt,
            }
        })
        .collect();

    let total_cases = case_results.len();
    let passed_cases = case_results.iter().filter(|case| case.passed).count();
    BenchmarkSuiteReport {
        schema: RECEIPT_SCHEMA.to_owned(),
        suite_id: SUITE_ID.to_owned(),
        total_cases,
        passed_cases,
        failed_cases: total_cases - passed_cases,
        cases: case_results,
    }
}
