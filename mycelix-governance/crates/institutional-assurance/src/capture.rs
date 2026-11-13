#![forbid(unsafe_code)]

//! Deterministic IPB-S3 fixtures for bounded adversarial institutional capture.
//!
//! This module evaluates a declared coalition against a scenario's configured
//! control quorums. It is not a general-purpose power-index implementation,
//! identity oracle, production authorizer, or judgment about real institutions.

use std::collections::{BTreeMap, BTreeSet};

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

pub const CAPTURE_RECEIPT_SCHEMA: &str = "lise.ipb.capture-receipt.v0";
pub const CAPTURE_SUITE_ID: &str = "IPB-S3-ICAP-v0";

#[derive(Clone, Copy, Debug, Deserialize, Eq, Ord, PartialEq, PartialOrd, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ControlDomain {
    Agenda,
    Appointments,
    Authorization,
    TreasuryCustody,
    Audit,
    Appeal,
    ConstitutionalAmendment,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum CaptureDisposition {
    /// The declared coalition can execute every configured critical function
    /// and the configured number of oversight functions are compromised.
    CaptureDetected,
    /// The configured capture predicate did not hold in this fixture.
    /// This is not proof that the institution is generally capture-resistant.
    NoCaptureObserved,
    /// At least one critical execution domain lacks its required active quorum.
    ExecutionUnavailable,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct DomainControl {
    pub domain: ControlDomain,
    /// Actors with authority in this domain at the scenario cutoff.
    pub authorized_controllers: BTreeSet<String>,
    /// Authorized controllers available to exercise authority in this scenario.
    pub available_controllers: BTreeSet<String>,
    /// Constitution/scenario-defined number of distinct active controllers
    /// needed to exercise this domain.
    pub required_signers: usize,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct CapturePolicy {
    pub policy_id: String,
    pub version: u32,
    /// Every listed function is part of the critical execution path.
    pub execution_domains: BTreeSet<ControlDomain>,
    /// Independent audit/challenge functions that must remain usable.
    pub oversight_domains: BTreeSet<ControlDomain>,
    /// At least this many oversight domains must be compromised for the
    /// configured capture predicate to fire.
    pub minimum_compromised_oversight_domains: usize,
    /// Each oversight domain must retain at least this many available
    /// controllers outside the declared coalition, in addition to meeting its
    /// own signer quorum.
    pub minimum_independent_oversight_controllers_per_domain: usize,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct CaptureScenario {
    pub scenario_id: String,
    /// Synthetic scenario label. Membership is declared ground truth, not
    /// inferred from identity, affiliation, reputation, or behavior.
    pub coalition_id: String,
    pub coalition_members: BTreeSet<String>,
    pub policy: CapturePolicy,
    pub domains: Vec<DomainControl>,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct DomainAssessment {
    pub domain: ControlDomain,
    /// Stable scenario-local identifiers; real evaluations must use approved
    /// pseudonyms and never publish personal identity data.
    pub authorized_controllers: Vec<String>,
    pub available_controllers: Vec<String>,
    pub coalition_available_controllers: Vec<String>,
    pub independent_available_controllers: Vec<String>,
    pub available_controller_count: usize,
    pub required_signers: usize,
    pub coalition_available_count: usize,
    pub independent_available_count: usize,
    pub coalition_can_exercise: bool,
    pub independent_oversight_ready: bool,
    pub oversight_compromised: bool,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct CaptureReceipt {
    pub schema: String,
    pub suite_id: String,
    pub scenario_id: String,
    pub coalition_id: String,
    pub coalition_members: Vec<String>,
    pub policy_id: String,
    pub policy_version: u32,
    pub disposition: CaptureDisposition,
    pub coalition_controlled_execution_domains: Vec<ControlDomain>,
    pub compromised_oversight_domains: Vec<ControlDomain>,
    pub domain_assessments: Vec<DomainAssessment>,
    pub policy_sha256: String,
    pub input_sha256: String,
    pub receipt_sha256: String,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct CaptureCaseResult {
    pub scenario_id: String,
    pub expected_disposition: CaptureDisposition,
    pub passed: bool,
    pub receipt: CaptureReceipt,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct CaptureSuiteReport {
    pub schema: String,
    pub suite_id: String,
    pub total_cases: usize,
    pub passed_cases: usize,
    pub failed_cases: usize,
    pub cases: Vec<CaptureCaseResult>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CaptureInputError {
    MissingScenarioId,
    MissingCoalitionId,
    EmptyCoalition,
    EmptyExecutionDomains,
    EmptyOversightDomains,
    InvalidOversightThreshold,
    DuplicateDomain,
    EmptyAuthorizedControllerSet,
    InvalidSignerThreshold,
    AvailableControllerNotAuthorized,
    MissingExecutionDomain,
    MissingOversightDomain,
}

fn digest<T: Serialize>(value: &T) -> String {
    let bytes = serde_json::to_vec(value)
        .expect("capture fixture schema contains only serializable values");
    let hash = Sha256::digest(bytes);
    let mut encoded = String::with_capacity(7 + hash.len() * 2);
    encoded.push_str("sha256:");
    for byte in hash {
        use std::fmt::Write;
        write!(&mut encoded, "{byte:02x}").expect("writing to String is infallible");
    }
    encoded
}

fn validate(
    scenario: &CaptureScenario,
) -> Result<BTreeMap<ControlDomain, &DomainControl>, CaptureInputError> {
    if scenario.scenario_id.trim().is_empty() {
        return Err(CaptureInputError::MissingScenarioId);
    }
    if scenario.coalition_id.trim().is_empty() {
        return Err(CaptureInputError::MissingCoalitionId);
    }
    if scenario.coalition_members.is_empty() {
        return Err(CaptureInputError::EmptyCoalition);
    }
    if scenario.policy.execution_domains.is_empty() {
        return Err(CaptureInputError::EmptyExecutionDomains);
    }
    if scenario.policy.oversight_domains.is_empty() {
        return Err(CaptureInputError::EmptyOversightDomains);
    }
    if scenario
        .policy
        .minimum_independent_oversight_controllers_per_domain
        == 0
        || scenario.policy.minimum_compromised_oversight_domains == 0
        || scenario.policy.minimum_compromised_oversight_domains
            > scenario.policy.oversight_domains.len()
    {
        return Err(CaptureInputError::InvalidOversightThreshold);
    }

    let mut by_domain = BTreeMap::new();
    for control in &scenario.domains {
        if by_domain.insert(control.domain, control).is_some() {
            return Err(CaptureInputError::DuplicateDomain);
        }
        if control.authorized_controllers.is_empty() {
            return Err(CaptureInputError::EmptyAuthorizedControllerSet);
        }
        if control.required_signers == 0
            || control.required_signers > control.authorized_controllers.len()
        {
            return Err(CaptureInputError::InvalidSignerThreshold);
        }
        if !control
            .available_controllers
            .is_subset(&control.authorized_controllers)
        {
            return Err(CaptureInputError::AvailableControllerNotAuthorized);
        }
    }

    for domain in &scenario.policy.execution_domains {
        if !by_domain.contains_key(domain) {
            return Err(CaptureInputError::MissingExecutionDomain);
        }
    }
    for domain in &scenario.policy.oversight_domains {
        if !by_domain.contains_key(domain) {
            return Err(CaptureInputError::MissingOversightDomain);
        }
    }

    Ok(by_domain)
}

#[derive(Serialize)]
struct CaptureReceiptMaterial<'a> {
    schema: &'static str,
    suite_id: &'static str,
    scenario_id: &'a str,
    coalition_id: &'a str,
    coalition_members: &'a [String],
    policy_id: &'a str,
    policy_version: u32,
    disposition: CaptureDisposition,
    coalition_controlled_execution_domains: &'a [ControlDomain],
    compromised_oversight_domains: &'a [ControlDomain],
    domain_assessments: &'a [DomainAssessment],
    policy_sha256: &'a str,
    input_sha256: &'a str,
}

/// Evaluate a declared coalition against the thresholds supplied by a
/// synthetic constitution/scenario. No threshold is universal or inferred.
pub fn evaluate_capture(scenario: &CaptureScenario) -> Result<CaptureReceipt, CaptureInputError> {
    let by_domain = validate(scenario)?;
    let mut domain_assessments = Vec::with_capacity(by_domain.len());

    for (domain, control) in &by_domain {
        let coalition_available_count = control
            .available_controllers
            .intersection(&scenario.coalition_members)
            .count();
        let independent_available_count = control
            .available_controllers
            .difference(&scenario.coalition_members)
            .count();
        let coalition_can_exercise = coalition_available_count >= control.required_signers;
        let minimum_independent = scenario
            .policy
            .minimum_independent_oversight_controllers_per_domain
            .max(control.required_signers);
        let independent_oversight_ready = independent_available_count >= minimum_independent;
        let is_oversight_domain = scenario.policy.oversight_domains.contains(domain);
        let oversight_compromised = is_oversight_domain && !independent_oversight_ready;

        domain_assessments.push(DomainAssessment {
            domain: *domain,
            authorized_controllers: control.authorized_controllers.iter().cloned().collect(),
            available_controllers: control.available_controllers.iter().cloned().collect(),
            coalition_available_controllers: control
                .available_controllers
                .intersection(&scenario.coalition_members)
                .cloned()
                .collect(),
            independent_available_controllers: control
                .available_controllers
                .difference(&scenario.coalition_members)
                .cloned()
                .collect(),
            available_controller_count: control.available_controllers.len(),
            required_signers: control.required_signers,
            coalition_available_count,
            independent_available_count,
            coalition_can_exercise,
            independent_oversight_ready,
            oversight_compromised,
        });
    }

    let coalition_controlled_execution_domains: Vec<_> = scenario
        .policy
        .execution_domains
        .iter()
        .copied()
        .filter(|domain| {
            domain_assessments
                .iter()
                .any(|assessment| assessment.domain == *domain && assessment.coalition_can_exercise)
        })
        .collect();

    let compromised_oversight_domains: Vec<_> = scenario
        .policy
        .oversight_domains
        .iter()
        .copied()
        .filter(|domain| {
            domain_assessments
                .iter()
                .any(|assessment| assessment.domain == *domain && assessment.oversight_compromised)
        })
        .collect();

    let execution_unavailable = scenario.policy.execution_domains.iter().any(|domain| {
        let control = by_domain
            .get(domain)
            .expect("validation guarantees every execution domain exists");
        control.available_controllers.len() < control.required_signers
    });
    let execution_captured =
        coalition_controlled_execution_domains.len() == scenario.policy.execution_domains.len();
    let oversight_captured = compromised_oversight_domains.len()
        >= scenario.policy.minimum_compromised_oversight_domains;

    let disposition = if execution_unavailable {
        CaptureDisposition::ExecutionUnavailable
    } else if execution_captured && oversight_captured {
        CaptureDisposition::CaptureDetected
    } else {
        CaptureDisposition::NoCaptureObserved
    };

    let policy_sha256 = digest(&scenario.policy);
    let input_sha256 = digest(scenario);
    let coalition_members: Vec<String> = scenario.coalition_members.iter().cloned().collect();
    let material = CaptureReceiptMaterial {
        schema: CAPTURE_RECEIPT_SCHEMA,
        suite_id: CAPTURE_SUITE_ID,
        scenario_id: &scenario.scenario_id,
        coalition_id: &scenario.coalition_id,
        coalition_members: &coalition_members,
        policy_id: &scenario.policy.policy_id,
        policy_version: scenario.policy.version,
        disposition,
        coalition_controlled_execution_domains: &coalition_controlled_execution_domains,
        compromised_oversight_domains: &compromised_oversight_domains,
        domain_assessments: &domain_assessments,
        policy_sha256: &policy_sha256,
        input_sha256: &input_sha256,
    };
    let receipt_sha256 = digest(&material);

    Ok(CaptureReceipt {
        schema: CAPTURE_RECEIPT_SCHEMA.to_owned(),
        suite_id: CAPTURE_SUITE_ID.to_owned(),
        scenario_id: scenario.scenario_id.clone(),
        coalition_id: scenario.coalition_id.clone(),
        coalition_members,
        policy_id: scenario.policy.policy_id.clone(),
        policy_version: scenario.policy.version,
        disposition,
        coalition_controlled_execution_domains,
        compromised_oversight_domains,
        domain_assessments,
        policy_sha256,
        input_sha256,
        receipt_sha256,
    })
}

fn set(actors: &[&str]) -> BTreeSet<String> {
    actors.iter().map(|actor| (*actor).to_owned()).collect()
}

fn policy() -> CapturePolicy {
    CapturePolicy {
        policy_id: "ipb-s3-fixture-policy".to_owned(),
        version: 1,
        execution_domains: BTreeSet::from([
            ControlDomain::Agenda,
            ControlDomain::Appointments,
            ControlDomain::Authorization,
            ControlDomain::TreasuryCustody,
        ]),
        oversight_domains: BTreeSet::from([ControlDomain::Audit, ControlDomain::Appeal]),
        minimum_compromised_oversight_domains: 2,
        minimum_independent_oversight_controllers_per_domain: 1,
    }
}

fn control(
    domain: ControlDomain,
    authorized: &[&str],
    available: &[&str],
    required_signers: usize,
) -> DomainControl {
    DomainControl {
        domain,
        authorized_controllers: set(authorized),
        available_controllers: set(available),
        required_signers,
    }
}

fn distributed_execution_domains(
    authorized: &[&str],
    available: &[&str],
    required_signers: usize,
) -> Vec<DomainControl> {
    vec![
        control(
            ControlDomain::Agenda,
            authorized,
            available,
            required_signers,
        ),
        control(
            ControlDomain::Appointments,
            authorized,
            available,
            required_signers,
        ),
        control(
            ControlDomain::Authorization,
            authorized,
            available,
            required_signers,
        ),
        control(
            ControlDomain::TreasuryCustody,
            authorized,
            available,
            required_signers,
        ),
    ]
}

fn captured_scenario(scenario_id: &str) -> CaptureScenario {
    let mut domains =
        distributed_execution_domains(&["cartel-a", "cartel-b"], &["cartel-a", "cartel-b"], 2);
    domains.push(control(
        ControlDomain::Audit,
        &["cartel-a", "cartel-b"],
        &["cartel-a", "cartel-b"],
        1,
    ));
    domains.push(control(
        ControlDomain::Appeal,
        &["cartel-a", "cartel-b"],
        &["cartel-a", "cartel-b"],
        1,
    ));
    domains.push(control(
        ControlDomain::ConstitutionalAmendment,
        &["independent-c", "independent-d", "independent-e"],
        &["independent-c", "independent-d", "independent-e"],
        3,
    ));
    CaptureScenario {
        scenario_id: scenario_id.to_owned(),
        coalition_id: "declared-cartel".to_owned(),
        coalition_members: set(&["cartel-a", "cartel-b"]),
        policy: policy(),
        domains,
    }
}

/// Nine known-answer fixtures; these are purposive synthetic cases, not a
/// statistical sample of real institutions or real-world attack prevalence.
pub fn adversarial_capture_suite() -> CaptureSuiteReport {
    let mut cases = Vec::new();

    let attack = captured_scenario("s3-attack-cross-domain-coalition");
    cases.push(attack);

    let mut single_actor = captured_scenario("s3-attack-single-actor-role-concentration");
    single_actor.coalition_id = "single-controller".to_owned();
    single_actor.coalition_members = set(&["single-controller"]);
    for domain in &mut single_actor.domains {
        domain.authorized_controllers = set(&["single-controller"]);
        domain.available_controllers = set(&["single-controller"]);
        domain.required_signers = 1;
    }
    cases.push(single_actor);

    let mut quorum_suppression = captured_scenario("s3-attack-quorum-suppression");
    for domain in &mut quorum_suppression.domains {
        domain
            .authorized_controllers
            .extend(set(&["independent-c", "independent-d"]));
        // Independents retain nominal authority but are unavailable at the
        // declared cutoff; this models absence/quorum suppression, not coercion.
    }
    cases.push(quorum_suppression);

    let mut independent_oversight = captured_scenario("s3-control-independent-audit-and-appeal");
    for domain in &mut independent_oversight.domains {
        if matches!(domain.domain, ControlDomain::Audit | ControlDomain::Appeal) {
            domain.authorized_controllers = set(&["independent-c", "independent-d"]);
            domain.available_controllers = set(&["independent-c", "independent-d"]);
            domain.required_signers = 2;
        }
    }
    cases.push(independent_oversight);

    let mut dual_quorum = captured_scenario("s3-control-dual-independent-oversight-quorums");
    for domain in &mut dual_quorum.domains {
        if dual_quorum
            .policy
            .oversight_domains
            .contains(&domain.domain)
        {
            domain.authorized_controllers =
                set(&["cartel-a", "cartel-b", "independent-c", "independent-d"]);
            domain.available_controllers =
                set(&["cartel-a", "cartel-b", "independent-c", "independent-d"]);
            domain.required_signers = 2;
        }
    }
    cases.push(dual_quorum);

    let mut fragmented = captured_scenario("s3-control-fragmented-execution-control");
    fragmented.coalition_members = set(&["cartel-a"]);
    for domain in &mut fragmented.domains {
        if fragmented.policy.execution_domains.contains(&domain.domain) {
            domain.authorized_controllers = set(&["cartel-a", "independent-c", "independent-d"]);
            domain.available_controllers = set(&["cartel-a", "independent-c", "independent-d"]);
            domain.required_signers = 2;
        } else if fragmented.policy.oversight_domains.contains(&domain.domain) {
            domain.authorized_controllers = set(&["independent-c", "independent-d"]);
            domain.available_controllers = set(&["independent-c", "independent-d"]);
        }
    }
    cases.push(fragmented);

    let mut below_threshold = captured_scenario("s3-control-coalition-below-execution-quorum");
    below_threshold.coalition_members = set(&["cartel-a"]);
    for domain in &mut below_threshold.domains {
        if below_threshold
            .policy
            .execution_domains
            .contains(&domain.domain)
        {
            domain.authorized_controllers = set(&["cartel-a", "independent-c", "independent-d"]);
            domain.available_controllers =
                set(&["cartel-a", "independent-c", "independent-d"]);
            domain.required_signers = 2;
        }
        if below_threshold
            .policy
            .oversight_domains
            .contains(&domain.domain)
        {
            domain.authorized_controllers = set(&["independent-c", "independent-d"]);
            domain.available_controllers = set(&["independent-c", "independent-d"]);
            domain.required_signers = 2;
        }
    }
    cases.push(below_threshold);

    let mut oversight_only = captured_scenario("s3-control-oversight-captured-without-execution");
    for domain in &mut oversight_only.domains {
        if oversight_only
            .policy
            .execution_domains
            .contains(&domain.domain)
        {
            domain.authorized_controllers = set(&["independent-c", "independent-d"]);
            domain.available_controllers = set(&["independent-c", "independent-d"]);
            domain.required_signers = 2;
        }
    }
    cases.push(oversight_only);

    let mut execution_unavailable =
        captured_scenario("s3-stress-independent-execution-quorum-unavailable");
    let agenda = execution_unavailable
        .domains
        .iter_mut()
        .find(|domain| domain.domain == ControlDomain::Agenda)
        .expect("agenda fixture exists");
    agenda
        .authorized_controllers
        .extend(set(&["independent-c", "independent-d"]));
    agenda.available_controllers = set(&["cartel-a"]);
    cases.push(execution_unavailable);

    let mut results = Vec::with_capacity(cases.len());
    for scenario in cases {
        let expected_disposition = match scenario.scenario_id.as_str() {
            "s3-attack-cross-domain-coalition"
            | "s3-attack-single-actor-role-concentration"
            | "s3-attack-quorum-suppression" => CaptureDisposition::CaptureDetected,
            "s3-stress-independent-execution-quorum-unavailable" => {
                CaptureDisposition::ExecutionUnavailable
            }
            _ => CaptureDisposition::NoCaptureObserved,
        };
        let receipt = evaluate_capture(&scenario)
            .expect("static S3 fixture must satisfy its declared input contract");
        results.push(CaptureCaseResult {
            scenario_id: scenario.scenario_id,
            expected_disposition,
            passed: receipt.disposition == expected_disposition,
            receipt,
        });
    }

    let passed_cases = results.iter().filter(|case| case.passed).count();
    CaptureSuiteReport {
        schema: "lise.ipb.suite-report.v0".to_owned(),
        suite_id: CAPTURE_SUITE_ID.to_owned(),
        total_cases: results.len(),
        passed_cases,
        failed_cases: results.len() - passed_cases,
        cases: results,
    }
}

#[cfg(test)]
mod unit_tests {
    use super::*;

    #[test]
    fn missing_oversight_domain_is_not_treated_as_safe() {
        let mut scenario = captured_scenario("s3-invalid-missing-appeal");
        scenario
            .domains
            .retain(|domain| domain.domain != ControlDomain::Appeal);
        assert_eq!(
            evaluate_capture(&scenario),
            Err(CaptureInputError::MissingOversightDomain)
        );
    }

    #[test]
    fn available_controller_must_have_authority() {
        let mut scenario = captured_scenario("s3-invalid-unauthorized-controller");
        let agenda = scenario
            .domains
            .iter_mut()
            .find(|domain| domain.domain == ControlDomain::Agenda)
            .expect("agenda fixture exists");
        agenda.available_controllers = set(&["not-authorized"]);
        assert_eq!(
            evaluate_capture(&scenario),
            Err(CaptureInputError::AvailableControllerNotAuthorized)
        );
    }

    #[test]
    fn duplicate_domain_is_rejected() {
        let mut scenario = captured_scenario("s3-invalid-duplicate-domain");
        scenario.domains.push(scenario.domains[0].clone());
        assert_eq!(
            evaluate_capture(&scenario),
            Err(CaptureInputError::DuplicateDomain)
        );
    }

    #[test]
    fn renaming_actor_labels_preserves_capture_disposition() {
        let scenario = captured_scenario("s3-permutation-invariance");
        let original = evaluate_capture(&scenario).expect("original scenario is valid");

        let mut renamed = scenario.clone();
        renamed.coalition_id = "renamed-coalition".to_owned();
        renamed.coalition_members = scenario
            .coalition_members
            .iter()
            .map(|actor| format!("renamed-{actor}"))
            .collect();
        for domain in &mut renamed.domains {
            domain.authorized_controllers = domain
                .authorized_controllers
                .iter()
                .map(|actor| format!("renamed-{actor}"))
                .collect();
            domain.available_controllers = domain
                .available_controllers
                .iter()
                .map(|actor| format!("renamed-{actor}"))
                .collect();
        }

        let permuted = evaluate_capture(&renamed).expect("renamed scenario is valid");
        assert_eq!(original.disposition, permuted.disposition);
        assert_eq!(
            original.coalition_controlled_execution_domains,
            permuted.coalition_controlled_execution_domains
        );
        assert_eq!(
            original.compromised_oversight_domains,
            permuted.compromised_oversight_domains
        );
    }

    #[test]
    fn insufficient_signer_threshold_is_rejected() {
        let mut scenario = captured_scenario("s3-invalid-zero-signers");
        scenario.domains[0].required_signers = 0;
        assert_eq!(
            evaluate_capture(&scenario),
            Err(CaptureInputError::InvalidSignerThreshold)
        );
    }
}
