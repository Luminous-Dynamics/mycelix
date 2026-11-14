#![forbid(unsafe_code)]

use std::collections::{BTreeMap, BTreeSet};

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct InputError(pub String);

impl std::fmt::Display for InputError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}
impl std::error::Error for InputError {}

#[derive(Clone, Debug)]
struct PortfolioEvidence {
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

fn extract_decision_evidence(report: &str) -> Result<BTreeMap<(String, String), String>, InputError> {
    let mut evidence = BTreeMap::new();
    let mut ambiguous = BTreeSet::new();
    for line in report.lines() {
        let Some(value) = line.strip_prefix("evidence=") else {
            continue;
        };
        let fields: Vec<&str> = value.split('|').collect();
        if fields.len() != 9 {
            return Err(InputError("malformed decision-time evidence report row".to_owned()));
        }
        let key = (fields[6].to_owned(), fields[7].to_owned());
        if evidence.insert(key.clone(), fields[5].to_owned()).is_some() {
            ambiguous.insert(key);
        }
    }
    if !ambiguous.is_empty() {
        return Err(InputError(format!(
            "ambiguous decision evidence owner/reference pairs: {}",
            ambiguous.len()
        )));
    }
    Ok(evidence)
}

fn extract_portfolio_evidence(source: &str) -> Result<Vec<PortfolioEvidence>, InputError> {
    const HEADER: &str = "record_type|record_id|option_id|service_id|cohort_id|scenario_id|interval_id|subject_id|quantity|unit|status|source_owner|evidence_ref";
    let mut header_seen = false;
    let mut rows = Vec::new();
    let mut ids = BTreeSet::new();
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
                "line {line_number}: portfolio source lineage is incomplete"
            )));
        }
        if !ids.insert(fields[1].to_owned()) {
            return Err(InputError(format!(
                "line {line_number}: duplicate portfolio record ID {}",
                fields[1]
            )));
        }
        rows.push(PortfolioEvidence {
            owner: fields[11].to_owned(),
            reference: fields[12].to_owned(),
        });
    }
    if !header_seen || rows.is_empty() {
        return Err(InputError("missing portfolio evidence rows".to_owned()));
    }
    Ok(rows)
}

pub fn compose(
    decision_evidence_source: &str,
    portfolio_plan_source: &str,
) -> Result<String, InputError> {
    let decision_report =
        fsre_decision_time_replay::validate_and_report(decision_evidence_source)
            .map_err(|error| InputError(format!("decision-time replay failed: {error}")))?;
    let portfolio_report =
        fsre_portfolio_reference::validate_and_report(portfolio_plan_source)
            .map_err(|error| InputError(format!("portfolio evaluation failed: {error}")))?;

    let decision_scenario = report_value(&decision_report, "scenario_id")?;
    let portfolio_scenario = report_value(&portfolio_report, "scenario_id")?;
    let decision_id = report_value(&decision_report, "decision_id")?;
    let decision_time = report_value(&decision_report, "decision_time")?;
    let decision_snapshot = report_value(&decision_report, "evidence_snapshot_id")?;
    let portfolio_snapshot = report_value(&portfolio_report, "evidence_snapshot_id")?;
    let decision_disposition = report_value(&decision_report, "decision_evidence_disposition")?;
    let portfolio_disposition = report_value(&portfolio_report, "portfolio_disposition")?;
    let portfolio_interval = report_value(&portfolio_report, "interval_id")?;

    let portfolio_evidence = extract_portfolio_evidence(portfolio_plan_source)?;
    let decision_index = extract_decision_evidence(&decision_report)?;

    let mut matched = 0_usize;
    let mut unmatched = 0_usize;
    let mut nonvalid = 0_usize;
    for item in &portfolio_evidence {
        match decision_index.get(&(item.owner.clone(), item.reference.clone())) {
            None => unmatched += 1,
            Some(status) if status != "KNOWN_VALID" => nonvalid += 1,
            Some(_) => matched += 1,
        }
    }

    let scope_matches =
        decision_scenario == portfolio_scenario && decision_snapshot == portfolio_snapshot;
    let disposition = if !scope_matches {
        "SCOPE_MISMATCH"
    } else if decision_disposition == "EVIDENCE_NOT_VALID_AT_DECISION" {
        "EVIDENCE_NOT_VALID_AT_DECISION"
    } else if decision_disposition != "ALL_SUBMITTED_EVIDENCE_VALID_AT_DECISION"
        || unmatched > 0
        || nonvalid > 0
    {
        "EVIDENCE_NOT_ESTIMABLE"
    } else if portfolio_disposition == "NOT_ESTIMABLE" {
        "PORTFOLIO_NOT_ESTIMABLE"
    } else if portfolio_disposition != "PORTFOLIO_FEASIBLE_FOR_DECLARED_SCENARIO" {
        "PORTFOLIO_NOT_FEASIBLE"
    } else {
        "SUBMITTED_EVIDENCE_LINKED_AND_PORTFOLIO_FEASIBLE_FOR_DECLARED_DECISION"
    };

    let mut lines = vec![
        "report_schema=fsre-assurance-composition-report.v0.1".to_owned(),
        format!("scenario_id={decision_scenario}"),
        format!("decision_id={decision_id}"),
        format!("decision_time={decision_time}"),
        format!("evidence_snapshot_id={decision_snapshot}"),
        format!("portfolio_interval_id={portfolio_interval}"),
        format!("decision_evidence_disposition={decision_disposition}"),
        format!("portfolio_disposition={portfolio_disposition}"),
        format!("portfolio_evidence_records={}", portfolio_evidence.len()),
        format!("portfolio_evidence_matched={matched}"),
        format!("portfolio_evidence_unmatched={unmatched}"),
        format!("portfolio_evidence_nonvalid={nonvalid}"),
        format!("scenario_and_snapshot_match={}", scope_matches),
        format!("assurance_disposition={disposition}"),
        "assurance_scope=SUBMITTED_EVIDENCE_AND_DECLARED_PORTFOLIO_ONLY".to_owned(),
        "claim_completeness=NOT_ESTABLISHED_BY_THIS_TOOL".to_owned(),
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

    const DECISION_FIXTURE: &str = include_str!("../fixtures/synthetic-decision.psv");
    const PORTFOLIO_FIXTURE: &str = include_str!("../fixtures/synthetic-portfolio.psv");

    #[test]
    fn matched_decision_snapshot_and_all_portfolio_evidence_compose() {
        let report = compose(DECISION_FIXTURE, PORTFOLIO_FIXTURE).expect("composition");
        assert!(report.contains(
            "assurance_disposition=SUBMITTED_EVIDENCE_LINKED_AND_PORTFOLIO_FEASIBLE_FOR_DECLARED_DECISION"
        ));
        assert!(report.contains("portfolio_evidence_records=28"));
        assert!(report.contains("portfolio_evidence_matched=28"));
        assert!(report.contains("portfolio_evidence_unmatched=0"));
        assert!(report.contains("portfolio_evidence_nonvalid=0"));
        assert!(report.contains("operational_actuation=NOT_AUTHORISED"));
    }

    #[test]
    fn snapshot_mismatch_blocks_composition() {
        let changed = PORTFOLIO_FIXTURE.replace(
            "# snapshot_id=synthetic-decision-snapshot-01",
            "# snapshot_id=different-evidence-snapshot",
        );
        let report = compose(DECISION_FIXTURE, &changed).expect("mismatch is reportable");
        assert!(report.contains("assurance_disposition=SCOPE_MISMATCH"));
    }

    #[test]
    fn scenario_mismatch_blocks_composition() {
        let changed = DECISION_FIXTURE.replace(
            "# scenario_id=synthetic-water-shock",
            "# scenario_id=other-water-shock",
        );
        let report = compose(&changed, PORTFOLIO_FIXTURE).expect("scenario mismatch is reportable");
        assert!(report.contains("assurance_disposition=SCOPE_MISMATCH"));
    }

    #[test]
    fn missing_portfolio_source_reference_abstains() {
        let changed = PORTFOLIO_FIXTURE.replace(
            "synthetic:qualification:pump",
            "synthetic:missing:pump",
        );
        let report = compose(DECISION_FIXTURE, &changed).expect("missing lineage is reportable");
        assert!(report.contains("assurance_disposition=EVIDENCE_NOT_ESTIMABLE"));
        assert!(report.contains("portfolio_evidence_unmatched=1"));
    }

    #[test]
    fn mismatched_owner_does_not_bind_by_reference_alone() {
        let changed = PORTFOLIO_FIXTURE.replace(
            "QUALIFIED|operator|synthetic:qualification:pump",
            "QUALIFIED|different-owner|synthetic:qualification:pump",
        );
        let report = compose(DECISION_FIXTURE, &changed).expect("owner mismatch is reportable");
        assert!(report.contains("EVIDENCE_NOT_ESTIMABLE"));
        assert!(report.contains("portfolio_evidence_unmatched=1"));
    }

    #[test]
    fn expired_decision_evidence_blocks_the_portfolio() {
        let changed = DECISION_FIXTURE.replace(
            "|800|1100|820|830||ASSERTED|operator|synthetic:qualification:pump",
            "|800|900|820|830||ASSERTED|operator|synthetic:qualification:pump",
        );
        let report = compose(&changed, PORTFOLIO_FIXTURE).expect("expired evidence is reportable");
        assert!(report.contains("decision_evidence_disposition=EVIDENCE_NOT_VALID_AT_DECISION"));
        assert!(report.contains("assurance_disposition=EVIDENCE_NOT_VALID_AT_DECISION"));
    }

    #[test]
    fn portfolio_and_decision_input_row_order_does_not_change_composition() {
        fn reverse_rows(source: &str) -> String {
            let mut metadata = Vec::new();
            let mut rows = Vec::new();
            for line in source.lines() {
                if line.starts_with('#') {
                    metadata.push(line);
                } else if !line.is_empty() {
                    rows.push(line);
                }
            }
            let header = rows.remove(0);
            rows.reverse();
            format!("{}\n{}\n{}\n", metadata.join("\n"), header, rows.join("\n"))
        }
        let original = compose(DECISION_FIXTURE, PORTFOLIO_FIXTURE).expect("original");
        let permuted = compose(
            &reverse_rows(DECISION_FIXTURE),
            &reverse_rows(PORTFOLIO_FIXTURE),
        )
        .expect("permuted");
        assert_eq!(original, permuted);
    }

    #[test]
    fn nonfeasible_portfolio_is_not_promoted_by_valid_evidence() {
        let changed = PORTFOLIO_FIXTURE.replace(
            "RESOURCE_CAPACITY|resource-cap-diesel||||synthetic-water-shock|hour-01|diesel|60|L|QUALIFIED|operator|synthetic:stock:diesel",
            "RESOURCE_CAPACITY|resource-cap-diesel||||synthetic-water-shock|hour-01|diesel|40|L|QUALIFIED|operator|synthetic:stock:diesel",
        );
        let report = compose(DECISION_FIXTURE, &changed).expect("infeasible plan is reportable");
        assert!(report.contains("assurance_disposition=PORTFOLIO_NOT_FEASIBLE"));
    }
}
