#![forbid(unsafe_code)]

use std::collections::{BTreeMap, BTreeSet};

type EvidenceKey = (String, String);

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct InputError(pub String);

impl std::fmt::Display for InputError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}
impl std::error::Error for InputError {}

#[derive(Clone, Debug)]
struct EvidenceLink {
    claim_id: String,
    admissibility: String,
}

#[derive(Clone, Debug)]
struct PortfolioReference {
    record_id: String,
    owner: String,
    reference: String,
}

fn report_value(report: &str, key: &str) -> Result<String, InputError> {
    let prefix = format!("{key}=");
    report
        .lines()
        .find_map(|line| line.strip_prefix(&prefix).map(str::to_owned))
        .ok_or_else(|| InputError(format!("component report missing {key}")))
}

fn optional_report_value(report: &str, key: &str) -> Option<String> {
    let prefix = format!("{key}=");
    report.lines().find_map(|line| line.strip_prefix(&prefix).map(str::to_owned))
}

fn parse_asof_claim_states(report: &str) -> Result<BTreeMap<String, String>, InputError> {
    let mut claims = BTreeMap::new();
    for line in report.lines() {
        let Some(value) = line.strip_prefix("claim=") else {
            continue;
        };
        let fields: Vec<&str> = value.split('|').collect();
        if fields.len() != 10 {
            return Err(InputError("malformed as-of claim report row".to_owned()));
        }
        if claims.insert(fields[0].to_owned(), fields[1].to_owned()).is_some() {
            return Err(InputError(format!("duplicate as-of claim row {}", fields[0])));
        }
    }
    if claims.is_empty() {
        return Err(InputError("as-of report contains no claim dispositions".to_owned()));
    }
    Ok(claims)
}

fn parse_asof_evidence(report: &str) -> Result<BTreeMap<EvidenceKey, EvidenceLink>, InputError> {
    let claims = parse_asof_claim_states(report)?;
    let mut evidence = BTreeMap::new();

    for line in report.lines() {
        let Some(value) = line.strip_prefix("evidence=") else {
            continue;
        };
        let fields: Vec<&str> = value.split('|').collect();
        if fields.len() != 6 {
            return Err(InputError("malformed as-of evidence report row".to_owned()));
        }
        let claim_id = fields[1];
        if !claims.contains_key(claim_id) {
            return Err(InputError(format!(
                "as-of evidence {} references undeclared claim {claim_id}",
                fields[0]
            )));
        }
        let owner = fields[4]
            .strip_prefix("owner=")
            .ok_or_else(|| InputError("as-of evidence row has no owner field".to_owned()))?;
        let reference = fields[5]
            .strip_prefix("ref=")
            .ok_or_else(|| InputError("as-of evidence row has no reference field".to_owned()))?;
        if owner.is_empty() || reference.is_empty() {
            return Err(InputError("as-of evidence row has empty owner/reference".to_owned()));
        }
        let key = (owner.to_owned(), reference.to_owned());
        let link = EvidenceLink {
            claim_id: claim_id.to_owned(),
            admissibility: if fields[3] == "admissible=YES"
                && claims.get(claim_id).is_some_and(|state| state == "SUPPORTED_AS_KNOWN_AT_CUTOFF")
            {
                "VALID".to_owned()
            } else {
                format!("{};{}", fields[2], claims[claim_id])
            },
        };
        if evidence.insert(key.clone(), link).is_some() {
            return Err(InputError(format!(
                "ambiguous as-of owner/reference binding: {} / {}",
                key.0, key.1
            )));
        }
    }
    Ok(evidence)
}

fn parse_portfolio_references(source: &str) -> Result<Vec<PortfolioReference>, InputError> {
    const HEADER: &str = "record_type|record_id|option_id|service_id|cohort_id|scenario_id|interval_id|subject_id|quantity|unit|status|source_owner|evidence_ref";
    let mut header_seen = false;
    let mut record_ids = BTreeSet::new();
    let mut references = Vec::new();

    for (offset, raw) in source.lines().enumerate() {
        let line_number = offset + 1;
        let line = raw.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        if !header_seen {
            if line != HEADER {
                return Err(InputError(format!(
                    "line {line_number}: unexpected portfolio evidence header"
                )));
            }
            header_seen = true;
            continue;
        }
        let fields: Vec<&str> = line.split('|').map(str::trim).collect();
        if fields.len() != 13 {
            return Err(InputError(format!(
                "line {line_number}: expected 13 portfolio fields"
            )));
        }
        if fields[1].is_empty() || fields[11].is_empty() || fields[12].is_empty() {
            return Err(InputError(format!(
                "line {line_number}: portfolio evidence lineage is incomplete"
            )));
        }
        if !record_ids.insert(fields[1].to_owned()) {
            return Err(InputError(format!(
                "line {line_number}: duplicate portfolio record ID {}",
                fields[1]
            )));
        }
        references.push(PortfolioReference {
            record_id: fields[1].to_owned(),
            owner: fields[11].to_owned(),
            reference: fields[12].to_owned(),
        });
    }
    if !header_seen || references.is_empty() {
        return Err(InputError("portfolio contains no evidence references".to_owned()));
    }
    Ok(references)
}

pub fn compose(asof_evidence_source: &str, portfolio_plan_source: &str) -> Result<String, InputError> {
    let asof_report = fsre_asof_evidence::validate_and_report(asof_evidence_source)
        .map_err(|error| InputError(format!("as-of evidence replay failed: {error}")))?;
    let portfolio_report = fsre_portfolio_reference::validate_and_report(portfolio_plan_source)
        .map_err(|error| InputError(format!("portfolio evaluation failed: {error}")))?;

    let asof_scenario = report_value(&asof_report, "scenario_id")?;
    let portfolio_scenario = report_value(&portfolio_report, "scenario_id")?;
    let asof_snapshot = report_value(&asof_report, "evidence_snapshot_id")?;
    let portfolio_snapshot = report_value(&portfolio_report, "evidence_snapshot_id")?;
    let asof_decision_at = report_value(&asof_report, "decision_at")?;
    let asof_clock = report_value(&asof_report, "clock_basis_id")?;
    let portfolio_decision_at = optional_report_value(&portfolio_report, "decision_at");
    let portfolio_clock = optional_report_value(&portfolio_report, "clock_basis_id");
    let portfolio_disposition = report_value(&portfolio_report, "portfolio_disposition")?;

    let scenario_match = asof_scenario == portfolio_scenario;
    let snapshot_match = asof_snapshot == portfolio_snapshot;
    let decision_cutoff_match = portfolio_decision_at.as_deref() == Some(asof_decision_at.as_str());
    let clock_basis_match = portfolio_clock.as_deref() == Some(asof_clock.as_str());
    let scope_match = scenario_match && snapshot_match && decision_cutoff_match && clock_basis_match;

    let asof_index = parse_asof_evidence(&asof_report)?;
    let portfolio_references = parse_portfolio_references(portfolio_plan_source)?;
    let mut matched = 0_usize;
    let mut unmatched = 0_usize;
    let mut nonvalid = 0_usize;
    let mut supported_claims = BTreeSet::new();
    let mut unmatched_record_ids = Vec::new();
    let mut nonvalid_record_ids = Vec::new();

    for reference in &portfolio_references {
        match asof_index.get(&(reference.owner.clone(), reference.reference.clone())) {
            None => {
                unmatched += 1;
                unmatched_record_ids.push(reference.record_id.clone());
            }
            Some(link) if link.admissibility != "VALID" => {
                nonvalid += 1;
                nonvalid_record_ids.push(reference.record_id.clone());
            }
            Some(link) => {
                matched += 1;
                supported_claims.insert(link.claim_id.clone());
            }
        }
    }

    let disposition = if !scope_match {
        "SCOPE_MISMATCH"
    } else if unmatched > 0 || nonvalid > 0 {
        "EVIDENCE_NOT_ESTIMABLE"
    } else if portfolio_disposition == "NOT_ESTIMABLE" {
        "PORTFOLIO_NOT_ESTIMABLE"
    } else if portfolio_disposition != "PORTFOLIO_FEASIBLE_FOR_DECLARED_SCENARIO" {
        "PORTFOLIO_NOT_FEASIBLE"
    } else {
        "ASOF_EVIDENCE_LINKED_AND_PORTFOLIO_FEASIBLE_FOR_DECLARED_DECISION"
    };

    let mut lines = vec![
        "report_schema=fsre-asof-portfolio-composition-report.v0.1".to_owned(),
        format!("scenario_id={asof_scenario}"),
        format!("evidence_snapshot_id={asof_snapshot}"),
        format!("decision_at={asof_decision_at}"),
        format!("clock_basis_id={asof_clock}"),
        format!("portfolio_disposition={portfolio_disposition}"),
        format!("portfolio_evidence_records={}", portfolio_references.len()),
        format!("portfolio_evidence_matched={matched}"),
        format!("portfolio_evidence_unmatched={unmatched}"),
        format!("portfolio_evidence_nonvalid={nonvalid}"),
        format!("portfolio_claims_supported={}", supported_claims.len()),
        format!(
            "unmatched_record_ids={}",
            if unmatched_record_ids.is_empty() { "NONE".to_owned() } else { unmatched_record_ids.join(",") }
        ),
        format!(
            "nonvalid_record_ids={}",
            if nonvalid_record_ids.is_empty() { "NONE".to_owned() } else { nonvalid_record_ids.join(",") }
        ),
        format!("scenario_match={scenario_match}"),
        format!("snapshot_match={snapshot_match}"),
        format!("decision_cutoff_match={decision_cutoff_match}"),
        format!("clock_basis_match={clock_basis_match}"),
        format!("assurance_disposition={disposition}"),
        "assurance_scope=AS_KNOWN_AT_DECISION_CUTOFF_AND_DECLARED_PORTFOLIO_ONLY".to_owned(),
        "claim_completeness=ONLY_PORTFOLIO_REFERENCED_CLAIMS_CHECKED".to_owned(),
        "source_authenticity=NOT_AUTHENTICATED_BY_THIS_TOOL".to_owned(),
        "execution_authority=NONE".to_owned(),
        "operational_actuation=NOT_AUTHORISED".to_owned(),
    ];
    lines.shrink_to_fit();
    Ok(lines.join("\n") + "\n")
}

#[cfg(test)]
mod tests {
    use super::*;

    const ASOF_FIXTURE: &str = include_str!("../fixtures/synthetic-asof.psv");
    const PORTFOLIO_FIXTURE: &str = include_str!("../fixtures/synthetic-portfolio.psv");

    #[test]
    fn exact_snapshot_cutoff_clock_and_references_compose() {
        let report = compose(ASOF_FIXTURE, PORTFOLIO_FIXTURE).expect("composition");
        assert!(report.contains(
            "assurance_disposition=ASOF_EVIDENCE_LINKED_AND_PORTFOLIO_FEASIBLE_FOR_DECLARED_DECISION"
        ));
        assert!(report.contains("portfolio_evidence_records=28"));
        assert!(report.contains("portfolio_evidence_matched=28"));
        assert!(report.contains("portfolio_evidence_unmatched=0"));
        assert!(report.contains("portfolio_evidence_nonvalid=0"));
        assert!(report.contains("decision_cutoff_match=true"));
        assert!(report.contains("clock_basis_match=true"));
        assert!(report.contains("operational_actuation=NOT_AUTHORISED"));
    }

    #[test]
    fn snapshot_mismatch_blocks_composition() {
        let changed = PORTFOLIO_FIXTURE.replace(
            "# snapshot_id=synthetic-decision-snapshot-01",
            "# snapshot_id=other-snapshot",
        );
        let report = compose(ASOF_FIXTURE, &changed).expect("scope mismatch is reportable");
        assert!(report.contains("assurance_disposition=SCOPE_MISMATCH"));
        assert!(report.contains("snapshot_match=false"));
    }

    #[test]
    fn decision_cutoff_mismatch_blocks_composition() {
        let changed = PORTFOLIO_FIXTURE.replace(
            "# decision_at=7200",
            "# decision_at=7201",
        );
        let report = compose(ASOF_FIXTURE, &changed).expect("cutoff mismatch is reportable");
        assert!(report.contains("decision_cutoff_match=false"));
        assert!(report.contains("assurance_disposition=SCOPE_MISMATCH"));
    }

    #[test]
    fn clock_basis_mismatch_blocks_composition() {
        let changed = PORTFOLIO_FIXTURE.replace(
            "# clock_basis_id=UTC_UNIX_SECONDS_V1",
            "# clock_basis_id=OTHER_CLOCK",
        );
        let report = compose(ASOF_FIXTURE, &changed).expect("clock mismatch is reportable");
        assert!(report.contains("clock_basis_match=false"));
        assert!(report.contains("assurance_disposition=SCOPE_MISMATCH"));
    }

    #[test]
    fn stale_evidence_does_not_bind_to_the_portfolio() {
        let changed = ASOF_FIXTURE.replace(
            "EVIDENCE|observation-qual-pump|claim-qual-pump|synthetic-water-shock|6800|6900",
            "EVIDENCE|observation-qual-pump|claim-qual-pump|synthetic-water-shock|100|6900",
        );
        let report = compose(&changed, PORTFOLIO_FIXTURE).expect("stale evidence is reportable");
        assert!(report.contains("portfolio_evidence_nonvalid=1"));
        assert!(report.contains("assurance_disposition=EVIDENCE_NOT_ESTIMABLE"));
    }

    #[test]
    fn missing_evidence_reference_abstains() {
        let changed = ASOF_FIXTURE.replace(
            "synthetic:qualification:pump",
            "synthetic:missing:qualification:pump",
        );
        let report = compose(&changed, PORTFOLIO_FIXTURE).expect("unmatched reference is reportable");
        assert!(report.contains("portfolio_evidence_unmatched=1"));
        assert!(report.contains("assurance_disposition=EVIDENCE_NOT_ESTIMABLE"));
    }

    #[test]
    fn nonfeasible_portfolio_is_not_promoted_by_asof_valid_evidence() {
        let changed = PORTFOLIO_FIXTURE.replace(
            "RESOURCE_CAPACITY|resource-cap-diesel||||synthetic-water-shock|hour-01|diesel|60|L|QUALIFIED|operator|synthetic:stock:diesel",
            "RESOURCE_CAPACITY|resource-cap-diesel||||synthetic-water-shock|hour-01|diesel|40|L|QUALIFIED|operator|synthetic:stock:diesel",
        );
        let report = compose(ASOF_FIXTURE, &changed).expect("infeasible portfolio is reportable");
        assert!(report.contains("portfolio_disposition=NOT_FEASIBLE"));
        assert!(report.contains("assurance_disposition=PORTFOLIO_NOT_FEASIBLE"));
    }

    #[test]
    fn row_order_does_not_change_composition() {
        fn reverse_rows(source: &str) -> String {
            let mut metadata = Vec::new();
            let mut rows = Vec::new();
            for line in source.lines() {
                if line.starts_with('#') { metadata.push(line); }
                else if !line.is_empty() { rows.push(line); }
            }
            let header = rows.remove(0);
            rows.reverse();
            format!("{}\n{}\n{}\n", metadata.join("\n"), header, rows.join("\n"))
        }
        let original = compose(ASOF_FIXTURE, PORTFOLIO_FIXTURE).expect("original");
        let reversed = compose(
            &reverse_rows(ASOF_FIXTURE),
            &reverse_rows(PORTFOLIO_FIXTURE),
        ).expect("reversed");
        assert_eq!(original, reversed);
    }

    #[test]
    fn ambiguous_owner_reference_binding_is_rejected() {
        let duplicate = "EVIDENCE|observation-duplicate|claim-duplicate|synthetic-water-shock|6800|6900|6700|9000|||ASSERTED|UTC_UNIX_SECONDS_V1|operator|synthetic:qualification:pump||\n";
        let claim = "CLAIM|requirement-duplicate|claim-duplicate|synthetic-water-shock|||||||REQUIRED|UTC_UNIX_SECONDS_V1|operator|synthetic:requirement:duplicate|1|\n";
        let asof = format!("{ASOF_FIXTURE}{claim}{duplicate}");
        assert!(compose(&asof, PORTFOLIO_FIXTURE)
            .unwrap_err()
            .0
            .contains("ambiguous as-of owner/reference binding"));
    }
}
