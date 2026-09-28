//! Executable COS semantic conformance model.
//!
//! This crate deliberately models evidence transitions rather than manufacturing
//! execution. It is dependency-light so semantic boundaries remain testable without
//! Holochain runtime state.

use serde::{Deserialize, Serialize};

pub const CORPUS_ID: &str = "COS-CONF-001";
pub const FORMAL_OBLIGATIONS: [&str; 10] = [
    "COS-FV-001", "COS-FV-002", "COS-FV-003", "COS-FV-004", "COS-FV-005",
    "COS-FV-006", "COS-FV-007", "COS-FV-008", "COS-FV-009", "COS-FV-010",
];

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Decision { Accepted, Rejected, Unknown, Conflicting, Stale, Superseded, Unauthorized, Unbound }

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum Origin { Local, Foreign(String) }

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Validity {
    pub valid_from: u64,
    pub valid_until: Option<u64>,
}
impl Validity {
    pub fn active_at(&self, at: u64) -> bool {
        self.valid_from <= at && self.valid_until.map_or(true, |end| at <= end)
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Evidence {
    pub id: &'static str,
    pub origin: Origin,
    pub validity: Validity,
    pub superseded: bool,
    pub conflicting: bool,
}
impl Evidence {
    pub fn current_local(id: &'static str, at: u64) -> Self {
        Self { id, origin: Origin::Local, validity: Validity { valid_from: at, valid_until: None }, superseded: false, conflicting: false }
    }
    pub fn current_foreign(id: &'static str, node: &str, at: u64) -> Self {
        Self { id, origin: Origin::Foreign(node.to_owned()), validity: Validity { valid_from: at, valid_until: None }, superseded: false, conflicting: false }
    }
    pub fn stale(id: &'static str, at: u64) -> Self {
        Self { id, origin: Origin::Local, validity: Validity { valid_from: 0, valid_until: Some(at.saturating_sub(1)) }, superseded: false, conflicting: false }
    }
}

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Bindings {
    pub plan_execution: bool,
    pub requirement_availability: bool,
    pub assignment_observed_work: bool,
    pub plan_consumption: bool,
    pub output_qualification: bool,
    pub foreign_recognition: bool,
    pub itc_projection: bool,
    pub frs_projection: bool,
    pub recommendation_authorized: bool,
    pub effect_recorded: bool,
    pub quality_current: bool,
    pub denominator_explicit: bool,
}

pub fn bind_plan_to_execution(b: &mut Bindings) { b.plan_execution = true; }
pub fn bind_requirement_to_availability(b: &mut Bindings) { b.requirement_availability = true; }
pub fn bind_assignment_to_observed_work(b: &mut Bindings) { b.assignment_observed_work = true; }
pub fn bind_plan_to_consumption(b: &mut Bindings) { b.plan_consumption = true; }
pub fn qualify_output(b: &mut Bindings) { b.output_qualification = true; }
pub fn recognize_foreign_evidence(b: &mut Bindings) { b.foreign_recognition = true; }
pub fn project_to_itc(b: &mut Bindings) { b.itc_projection = true; }
pub fn project_to_frs(b: &mut Bindings) { b.frs_projection = true; }
pub fn authorize_recommendation(b: &mut Bindings) { b.recommendation_authorized = true; }
pub fn record_effect(b: &mut Bindings) { b.effect_recorded = true; }

pub fn stale_evidence_is_rejected(e: &Evidence, now: u64) -> Decision {
    if e.superseded { Decision::Superseded }
    else if e.conflicting { Decision::Conflicting }
    else if !e.validity.active_at(now) { Decision::Stale }
    else { Decision::Accepted }
}

pub fn evaluate_negative(id: &str, b: &Bindings, evidence: Option<&Evidence>, now: u64) -> Decision {
    match id {
        "COS-N-001" => if b.plan_execution { Decision::Accepted } else { Decision::Rejected },
        "COS-N-002" => if b.plan_consumption { Decision::Accepted } else { Decision::Rejected },
        "COS-N-003" | "COS-N-004" => if b.requirement_availability { Decision::Accepted } else { Decision::Rejected },
        "COS-N-005" => evidence.map_or(Decision::Unknown, |e| stale_evidence_is_rejected(e, now)),
        "COS-N-006" => if b.output_qualification { Decision::Accepted } else { Decision::Rejected },
        "COS-N-007" => if b.plan_execution { Decision::Accepted } else { Decision::Rejected },
        "COS-N-008" => if b.effect_recorded { Decision::Accepted } else { Decision::Rejected },
        "COS-N-009" | "COS-N-013" => if b.foreign_recognition { Decision::Accepted } else { Decision::Rejected },
        "COS-N-010" => if b.frs_projection { Decision::Accepted } else { Decision::Rejected },
        "COS-N-011" => if b.itc_projection { Decision::Accepted } else { Decision::Rejected },
        "COS-N-012" => if b.itc_projection { Decision::Accepted } else { Decision::Rejected },
        "COS-N-014" => if b.recommendation_authorized && b.effect_recorded { Decision::Accepted } else { Decision::Unauthorized },
        "COS-N-015" => if b.output_qualification { Decision::Accepted } else { Decision::Rejected },
        "COS-N-016" => if b.denominator_explicit { Decision::Accepted } else { Decision::Rejected },
        _ => Decision::Unknown,
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Case {
    pub test_id: &'static str,
    pub formal_obligations: &'static [&'static str],
    pub claim_ceiling: &'static str,
}

pub const CASES: [Case; 16] = [
    Case { test_id:"COS-N-001", formal_obligations:&["COS-FV-002","COS-FV-007"], claim_ceiling:"Plan/execution semantic conformance only." },
    Case { test_id:"COS-N-002", formal_obligations:&["COS-FV-002"], claim_ceiling:"Planned-vs-consumed evidence separation only." },
    Case { test_id:"COS-N-003", formal_obligations:&["COS-FV-003"], claim_ceiling:"Temporal availability binding only." },
    Case { test_id:"COS-N-004", formal_obligations:&["COS-FV-003"], claim_ceiling:"Machine capability/availability separation only." },
    Case { test_id:"COS-N-005", formal_obligations:&["COS-FV-005"], claim_ceiling:"Freshness semantics only." },
    Case { test_id:"COS-N-006", formal_obligations:&["COS-FV-004"], claim_ceiling:"Output qualification boundary only." },
    Case { test_id:"COS-N-007", formal_obligations:&["COS-FV-002"], claim_ceiling:"Single-run/general-capability separation only." },
    Case { test_id:"COS-N-008", formal_obligations:&["COS-FV-006"], claim_ceiling:"Historical failure immutability only." },
    Case { test_id:"COS-N-009", formal_obligations:&["COS-FV-009"], claim_ceiling:"External dependency provenance only." },
    Case { test_id:"COS-N-010", formal_obligations:&["COS-FV-008"], claim_ceiling:"FRS projection/source-observation separation only." },
    Case { test_id:"COS-N-011", formal_obligations:&["COS-FV-001"], claim_ceiling:"Production/ITC projection separation only." },
    Case { test_id:"COS-N-012", formal_obligations:&["COS-FV-001"], claim_ceiling:"ITC/physical-work separation only." },
    Case { test_id:"COS-N-013", formal_obligations:&["COS-FV-010"], claim_ceiling:"Foreign/local provenance separation only." },
    Case { test_id:"COS-N-014", formal_obligations:&["COS-FV-008"], claim_ceiling:"Authorization/effect separation only." },
    Case { test_id:"COS-N-015", formal_obligations:&["COS-FV-004"], claim_ceiling:"Distribution/quality separation only." },
    Case { test_id:"COS-N-016", formal_obligations:&["COS-FV-001"], claim_ceiling:"Explicit-denominator arithmetic claim only." },
];

pub fn conformance_report_json() -> String {
    #[derive(Serialize)]
    struct Report<'a> { corpus_id: &'a str, negative_cases: &'a [Case;16], obligations: &'a [&'a str;10], claim_ceiling: &'a str }
    serde_json::to_string_pretty(&Report {
        corpus_id: CORPUS_ID,
        negative_cases: &CASES,
        obligations: &FORMAL_OBLIGATIONS,
        claim_ceiling: "Semantic conformance of this reference harness only; no physical, safety, economic, ecological, or Integral-validation claim.",
    }).expect("report serialization is infallible for these static values")
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fresh() -> Evidence { Evidence::current_local("evidence-1", 100) }

    #[test]
    fn negative_corpus_001_through_016_fails_closed() {
        let now = 100;
        let stale = Evidence::stale("qa-1", now);
        let current = fresh();
        let b = Bindings::default();
        for case in CASES.iter() {
            let evidence = if case.test_id == "COS-N-005" { Some(&stale) } else { Some(&current) };
            let decision = evaluate_negative(case.test_id, &b, evidence, now);
            assert!(matches!(decision, Decision::Rejected | Decision::Stale | Decision::Unauthorized),
                "{} unexpectedly accepted as {:?}", case.test_id, decision);
        }
    }

    #[test]
    fn positive_counterparts_001_through_016_require_explicit_bindings() {
        let now = 100;
        for case in CASES.iter() {
            let mut b = Bindings::default();
            match case.test_id {
                "COS-N-001" => bind_plan_to_execution(&mut b),
                "COS-N-002" => bind_plan_to_consumption(&mut b),
                "COS-N-003" | "COS-N-004" => bind_requirement_to_availability(&mut b),
                "COS-N-005" => b.quality_current = true,
                "COS-N-006" | "COS-N-015" => qualify_output(&mut b),
                "COS-N-007" => bind_plan_to_execution(&mut b),
                "COS-N-008" => record_effect(&mut b),
                "COS-N-009" | "COS-N-013" => recognize_foreign_evidence(&mut b),
                "COS-N-010" => project_to_frs(&mut b),
                "COS-N-011" | "COS-N-012" => project_to_itc(&mut b),
                "COS-N-014" => { authorize_recommendation(&mut b); record_effect(&mut b); },
                "COS-N-016" => b.denominator_explicit = true,
                _ => unreachable!(),
            }
            let current = fresh();
            let evidence = Some(&current);
            let decision = if case.test_id == "COS-N-005" {
                if b.quality_current && stale_evidence_is_rejected(evidence.unwrap(), now) == Decision::Accepted { Decision::Accepted } else { Decision::Rejected }
            } else {
                evaluate_negative(case.test_id, &b, evidence, now)
            };
            assert_eq!(decision, Decision::Accepted, "{} positive counterpart did not bind explicitly", case.test_id);
        }
    }

    #[test]
    fn every_case_maps_to_at_least_one_formal_obligation() {
        assert_eq!(CASES.len(), 16);
        for case in CASES.iter() {
            assert!(!case.formal_obligations.is_empty(), "{} has no formal mapping", case.test_id);
            for obligation in *case.formal_obligations {
                assert!(FORMAL_OBLIGATIONS.contains(obligation), "{} maps to unknown {}", case.test_id, obligation);
            }
        }
    }

    #[test]
    fn every_formal_obligation_has_negative_and_positive_coverage() {
        let covered: std::collections::BTreeSet<&str> = CASES.iter()
            .flat_map(|c| c.formal_obligations.iter().copied())
            .collect();
        for obligation in FORMAL_OBLIGATIONS {
            assert!(covered.contains(obligation), "untested formal obligation: {obligation}");
        }
    }

    #[test]
    fn report_is_machine_readable_and_claim_bounded() {
        let report = conformance_report_json();
        assert!(report.contains("\"COS-CONF-001\""));
        assert!(report.contains("no physical, safety, economic, ecological"));
        assert!(!report.contains("verified_score"));
    }

    #[test]
    fn provenance_cannot_be_rewritten_by_recognition() {
        let e = Evidence::current_foreign("foreign-1", "node-b", 100);
        let mut b = Bindings::default();
        recognize_foreign_evidence(&mut b);
        assert_eq!(e.origin, Origin::Foreign("node-b".to_owned()));
        assert!(b.foreign_recognition);
    }
}
