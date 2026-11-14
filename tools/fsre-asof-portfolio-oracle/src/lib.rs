#![forbid(unsafe_code)]

use std::collections::{BTreeMap, BTreeSet};

type OwnerReference = (String, String);

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ReferenceError(pub String);

impl std::fmt::Display for ReferenceError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}
impl std::error::Error for ReferenceError {}

#[derive(Clone, Debug)]
struct ClaimState {
    disposition: String,
}

#[derive(Clone, Debug)]
struct SourceEvidence {
    claim: String,
    qualifies: bool,
}

#[derive(Clone, Debug)]
struct PortfolioSource {
    record: String,
    owner: String,
    reference: String,
}

fn field(report: &str, name: &str) -> Result<String, ReferenceError> {
    let key = format!("{name}=");
    report
        .lines()
        .find_map(|line| line.strip_prefix(&key).map(str::to_owned))
        .ok_or_else(|| ReferenceError(format!("required report field {name} is absent")))
}

fn maybe_field(report: &str, name: &str) -> Option<String> {
    let key = format!("{name}=");
    report.lines().find_map(|line| line.strip_prefix(&key).map(str::to_owned))
}

fn decode_claims(report: &str) -> Result<BTreeMap<String, ClaimState>, ReferenceError> {
    let mut claims = BTreeMap::new();
    for row in report.lines().filter_map(|line| line.strip_prefix("claim=")) {
        let columns: Vec<&str> = row.split('|').collect();
        if columns.len() != 10 {
            return Err(ReferenceError("as-of claim report has invalid column count".to_owned()));
        }
        let state = ClaimState {
            disposition: columns[1].to_owned(),
        };
        let claim_id = columns[0].to_owned();
        if claims.insert(claim_id.clone(), state).is_some() {
            return Err(ReferenceError(format!("as-of report repeats claim {claim_id}")));
        }
    }
    if claims.is_empty() {
        return Err(ReferenceError("as-of report has no claim rows".to_owned()));
    }
    Ok(claims)
}

fn decode_sources(
    report: &str,
) -> Result<BTreeMap<OwnerReference, SourceEvidence>, ReferenceError> {
    let claims = decode_claims(report)?;
    let mut sources = BTreeMap::new();

    for row in report.lines().filter_map(|line| line.strip_prefix("evidence=")) {
        let columns: Vec<&str> = row.split('|').collect();
        if columns.len() != 6 {
            return Err(ReferenceError("as-of evidence report has invalid column count".to_owned()));
        }
        let claim_id = columns[1];
        let claim = claims.get(claim_id).ok_or_else(|| {
            ReferenceError(format!(
                "source record {} refers to undeclared claim {claim_id}",
                columns[0]
            ))
        })?;
        let owner = columns[4]
            .strip_prefix("owner=")
            .ok_or_else(|| ReferenceError("evidence report lacks owner binding".to_owned()))?;
        let reference = columns[5]
            .strip_prefix("ref=")
            .ok_or_else(|| ReferenceError("evidence report lacks source reference".to_owned()))?;
        if owner.is_empty() || reference.is_empty() {
            return Err(ReferenceError("empty evidence owner or source reference".to_owned()));
        }
        let key = (owner.to_owned(), reference.to_owned());
        let value = SourceEvidence {
            claim: claim_id.to_owned(),
            qualifies: columns[3] == "admissible=YES"
                && claim.disposition == "SUPPORTED_AS_KNOWN_AT_CUTOFF",
        };
        if sources.insert(key.clone(), value).is_some() {
            return Err(ReferenceError(format!(
                "owner/reference binding is not unique: {} / {}",
                key.0, key.1
            )));
        }
    }
    Ok(sources)
}

fn decode_portfolio(source: &str) -> Result<Vec<PortfolioSource>, ReferenceError> {
    const HEADER: &str = "record_type|record_id|option_id|service_id|cohort_id|scenario_id|interval_id|subject_id|quantity|unit|status|source_owner|evidence_ref";
    let mut seen_header = false;
    let mut identifiers = BTreeSet::new();
    let mut items = Vec::new();

    for (position, raw) in source.lines().enumerate() {
        let number = position + 1;
        let line = raw.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        if !seen_header {
            if line != HEADER {
                return Err(ReferenceError(format!("line {number}: unexpected portfolio header")));
            }
            seen_header = true;
            continue;
        }
        let columns: Vec<&str> = line.split('|').map(str::trim).collect();
        if columns.len() != 13 {
            return Err(ReferenceError(format!("line {number}: expected 13 fields")));
        }
        let id = columns[1];
        let owner = columns[11];
        let reference = columns[12];
        if id.is_empty() || owner.is_empty() || reference.is_empty() {
            return Err(ReferenceError(format!("line {number}: incomplete owner/source binding")));
        }
        if !identifiers.insert(id.to_owned()) {
            return Err(ReferenceError(format!("line {number}: duplicate record ID {id}")));
        }
        items.push(PortfolioSource {
            record: id.to_owned(),
            owner: owner.to_owned(),
            reference: reference.to_owned(),
        });
    }
    if !seen_header || items.is_empty() {
        return Err(ReferenceError("portfolio has no evidence records".to_owned()));
    }
    Ok(items)
}

pub fn reproduce(asof_source: &str, portfolio_source: &str) -> Result<String, ReferenceError> {
    let asof_report = fsre_asof_evidence_oracle::validate_and_report(asof_source)
        .map_err(|error| ReferenceError(format!("independent as-of reference failed: {error}")))?;
    let portfolio_report = fsre_portfolio_oracle::validate_and_report(portfolio_source)
        .map_err(|error| ReferenceError(format!("independent portfolio reference failed: {error}")))?;

    let scenario_asof = field(&asof_report, "scenario_id")?;
    let scenario_portfolio = field(&portfolio_report, "scenario_id")?;
    let snapshot_asof = field(&asof_report, "evidence_snapshot_id")?;
    let snapshot_portfolio = field(&portfolio_report, "evidence_snapshot_id")?;
    let cutoff_asof = field(&asof_report, "decision_at")?;
    let basis_asof = field(&asof_report, "clock_basis_id")?;
    let cutoff_portfolio = maybe_field(&portfolio_report, "decision_at");
    let basis_portfolio = maybe_field(&portfolio_report, "clock_basis_id");
    let portfolio_state = field(&portfolio_report, "portfolio_disposition")?;

    let scenario_match = scenario_asof == scenario_portfolio;
    let snapshot_match = snapshot_asof == snapshot_portfolio;
    let cutoff_match = cutoff_portfolio.as_deref() == Some(cutoff_asof.as_str());
    let clock_match = basis_portfolio.as_deref() == Some(basis_asof.as_str());
    let scope_match = scenario_match && snapshot_match && cutoff_match && clock_match;

    let source_index = decode_sources(&asof_report)?;
    let portfolio_items = decode_portfolio(portfolio_source)?;
    let mut matches = 0_usize;
    let mut missing = Vec::new();
    let mut rejected = Vec::new();
    let mut supported_claims = BTreeSet::new();

    for item in &portfolio_items {
        match source_index.get(&(item.owner.clone(), item.reference.clone())) {
            None => missing.push(item.record.clone()),
            Some(source) if !source.qualifies => rejected.push(item.record.clone()),
            Some(source) => {
                matches += 1;
                supported_claims.insert(source.claim.clone());
            }
        }
    }

    let disposition = if !scope_match {
        "SCOPE_MISMATCH"
    } else if !missing.is_empty() || !rejected.is_empty() {
        "EVIDENCE_NOT_ESTIMABLE"
    } else if portfolio_state == "NOT_ESTIMABLE" {
        "PORTFOLIO_NOT_ESTIMABLE"
    } else if portfolio_state != "PORTFOLIO_FEASIBLE_FOR_DECLARED_SCENARIO" {
        "PORTFOLIO_NOT_FEASIBLE"
    } else {
        "ASOF_EVIDENCE_LINKED_AND_PORTFOLIO_FEASIBLE_FOR_DECLARED_DECISION"
    };

    let mut report = vec![
        "report_schema=fsre-asof-portfolio-composition-report.v0.1".to_owned(),
        format!("scenario_id={scenario_asof}"),
        format!("evidence_snapshot_id={snapshot_asof}"),
        format!("decision_at={cutoff_asof}"),
        format!("clock_basis_id={basis_asof}"),
        format!("portfolio_disposition={portfolio_state}"),
        format!("portfolio_evidence_records={}", portfolio_items.len()),
        format!("portfolio_evidence_matched={matches}"),
        format!("portfolio_evidence_unmatched={}", missing.len()),
        format!("portfolio_evidence_nonvalid={}", rejected.len()),
        format!("portfolio_claims_supported={}", supported_claims.len()),
        format!(
            "unmatched_record_ids={}",
            if missing.is_empty() { "NONE".to_owned() } else { missing.join(",") }
        ),
        format!(
            "nonvalid_record_ids={}",
            if rejected.is_empty() { "NONE".to_owned() } else { rejected.join(",") }
        ),
        format!("scenario_match={scenario_match}"),
        format!("snapshot_match={snapshot_match}"),
        format!("decision_cutoff_match={cutoff_match}"),
        format!("clock_basis_match={clock_match}"),
        format!("assurance_disposition={disposition}"),
        "assurance_scope=AS_KNOWN_AT_DECISION_CUTOFF_AND_DECLARED_PORTFOLIO_ONLY".to_owned(),
        "claim_completeness=ONLY_PORTFOLIO_REFERENCED_CLAIMS_CHECKED".to_owned(),
        "source_authenticity=NOT_AUTHENTICATED_BY_THIS_TOOL".to_owned(),
        "execution_authority=NONE".to_owned(),
        "operational_actuation=NOT_AUTHORISED".to_owned(),
    ];
    report.shrink_to_fit();
    Ok(report.join("\n") + "\n")
}

#[cfg(test)]
mod tests {
    use super::*;

    const ASOF: &str = include_str!("../../fsre-asof-portfolio-composer/fixtures/synthetic-asof.psv");
    const PORTFOLIO: &str =
        include_str!("../../fsre-asof-portfolio-composer/fixtures/synthetic-portfolio.psv");

    #[test]
    fn independent_composition_reference_matches_positive_snapshot() {
        let report = reproduce(ASOF, PORTFOLIO).expect("positive composition");
        assert!(report.contains(
            "assurance_disposition=ASOF_EVIDENCE_LINKED_AND_PORTFOLIO_FEASIBLE_FOR_DECLARED_DECISION"
        ));
        assert!(report.contains("portfolio_evidence_matched=28"));
        assert!(report.contains("portfolio_claims_supported=28"));
    }

    #[test]
    fn a_different_snapshot_is_never_composed() {
        let changed = PORTFOLIO.replace(
            "# snapshot_id=synthetic-decision-snapshot-01",
            "# snapshot_id=unrelated-snapshot",
        );
        let report = reproduce(ASOF, &changed).expect("snapshot mismatch is reportable");
        assert!(report.contains("snapshot_match=false"));
        assert!(report.contains("assurance_disposition=SCOPE_MISMATCH"));
    }

    #[test]
    fn a_different_decision_cutoff_is_never_composed() {
        let changed = PORTFOLIO.replace("# decision_at=7200", "# decision_at=7201");
        let report = reproduce(ASOF, &changed).expect("cutoff mismatch is reportable");
        assert!(report.contains("decision_cutoff_match=false"));
        assert!(report.contains("assurance_disposition=SCOPE_MISMATCH"));
    }

    #[test]
    fn invalid_source_record_is_not_linked_by_its_owner_reference() {
        let changed = ASOF.replace(
            "EVIDENCE|observation-qual-pump|claim-qual-pump|synthetic-water-shock|6800|6900",
            "EVIDENCE|observation-qual-pump|claim-qual-pump|synthetic-water-shock|100|6900",
        );
        let report = reproduce(&changed, PORTFOLIO).expect("stale record is reportable");
        assert!(report.contains("portfolio_evidence_nonvalid=1"));
        assert!(report.contains("nonvalid_record_ids=cap-pump,qual-pump"));
    }

    #[test]
    fn a_nonfeasible_portfolio_is_not_promoted_by_valid_evidence() {
        let changed = PORTFOLIO.replace(
            "RESOURCE_CAPACITY|resource-cap-diesel||||synthetic-water-shock|hour-01|diesel|60|L|QUALIFIED|operator|synthetic:stock:diesel",
            "RESOURCE_CAPACITY|resource-cap-diesel||||synthetic-water-shock|hour-01|diesel|40|L|QUALIFIED|operator|synthetic:stock:diesel",
        );
        let report = reproduce(ASOF, &changed).expect("infeasible portfolio is reportable");
        assert!(report.contains("portfolio_disposition=NOT_FEASIBLE"));
        assert!(report.contains("assurance_disposition=PORTFOLIO_NOT_FEASIBLE"));
    }

    #[test]
    fn source_binding_ambiguity_is_rejected() {
        let extra_claim = "CLAIM|requirement-extra|claim-extra|synthetic-water-shock|||||||REQUIRED|UTC_UNIX_SECONDS_V1|operator|synthetic:extra-claim|1|\n";
        let extra_row = "EVIDENCE|observation-extra|claim-extra|synthetic-water-shock|6800|6900|6700|9000|||ASSERTED|UTC_UNIX_SECONDS_V1|operator|synthetic:qualification:pump||\n";
        let changed = format!("{ASOF}{extra_claim}{extra_row}");
        assert!(reproduce(&changed, PORTFOLIO)
            .unwrap_err()
            .0
            .contains("owner/reference binding is not unique"));
    }

    #[test]
    fn row_order_does_not_change_independent_composition() {
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
        let original = reproduce(ASOF, PORTFOLIO).expect("original");
        let reversed = reproduce(&reverse_rows(ASOF), &reverse_rows(PORTFOLIO)).expect("reversed");
        assert_eq!(original, reversed);
    }
}

