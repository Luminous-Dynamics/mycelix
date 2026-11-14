use super::*;
use std::collections::{BTreeMap, BTreeSet};

/// Audit an observed frame against one declared reference frame.
///
/// Reconciliation is exact only relative to the supplied register and crosswalk.
/// Real-world population completeness is never inferred by this function.
pub fn audit(source: &str) -> Result<AuditReport, AuditError> {
    let (metadata, references, observed, links) = parse_source(source)?;
    let mut links_by_observed = BTreeMap::<String, Vec<&Link>>::new();
    let mut matched_by_reference = BTreeMap::<String, usize>::new();
    let mut unresolved_by_reference = BTreeMap::<String, usize>::new();
    let mut match_edges = 0usize;
    let mut unresolved_edges = 0usize;
    let mut unique_edges = BTreeSet::new();

    for link in &links {
        let Some(observed_record) = observed.get(&link.observed_id) else {
            return Err(AuditError(format!(
                "crosswalk references missing observed record {}",
                link.observed_id
            )));
        };
        if observed_record.scope != link.scope {
            return Err(AuditError(format!(
                "crosswalk scope status disagrees for observed record {}",
                link.observed_id
            )));
        }
        if let Some(reference_id) = &link.reference_id {
            if !references.contains_key(reference_id) {
                return Err(AuditError(format!(
                    "crosswalk references missing reference record {reference_id}"
                )));
            }
        }
        let edge_key = (
            link.observed_id.clone(),
            link.reference_id.clone(),
            link.decision,
        );
        if !unique_edges.insert(edge_key) {
            return Err(AuditError(format!(
                "duplicate crosswalk assertion for observed record {}",
                link.observed_id
            )));
        }

        match link.decision {
            Decision::Match => {
                let reference_id = link.reference_id.as_ref().expect("validated match target");
                *matched_by_reference.entry(reference_id.clone()).or_default() += 1;
                match_edges += 1;
            }
            Decision::Unresolved => {
                unresolved_edges += 1;
                if let Some(reference_id) = &link.reference_id {
                    *unresolved_by_reference.entry(reference_id.clone()).or_default() += 1;
                }
            }
            Decision::NoMatchInScope | Decision::OutOfScope => {}
        }
        links_by_observed
            .entry(link.observed_id.clone())
            .or_default()
            .push(link);
    }

    for observed_id in observed.keys() {
        let rows = links_by_observed.get(observed_id).ok_or_else(|| {
            AuditError(format!("observed record {observed_id} has no crosswalk disposition"))
        })?;
        let all_unresolved = rows.iter().all(|row| row.decision == Decision::Unresolved);
        if !all_unresolved && rows.len() != 1 {
            return Err(AuditError(format!(
                "observed record {observed_id} has conflicting crosswalk dispositions"
            )));
        }
        if rows.len() > 1 && !all_unresolved {
            return Err(AuditError(format!(
                "observed record {observed_id} has mixed resolved and unresolved links"
            )));
        }
    }

    let mut matched_reference_units = 0usize;
    let mut omitted_reference_units = 0usize;
    let mut duplicate_reference_units = 0usize;
    let mut unresolved_reference_units = 0usize;

    for reference_id in references.keys() {
        let matched = matched_by_reference.get(reference_id).copied().unwrap_or(0);
        let unresolved = unresolved_by_reference.get(reference_id).copied().unwrap_or(0);
        if matched > 1 {
            duplicate_reference_units += 1;
        }
        if unresolved > 0 {
            unresolved_reference_units += 1;
        }
        if matched == 0 && unresolved == 0 {
            omitted_reference_units += 1;
        }
        if matched == 1 && unresolved == 0 {
            matched_reference_units += 1;
        }
    }

    let mut unrepresented_in_scope_observed = 0usize;
    let mut out_of_scope_observed = 0usize;
    let mut unknown_scope_observed = 0usize;
    let mut unresolved_observed = 0usize;

    for rows in links_by_observed.values() {
        match rows[0].decision {
            Decision::Match => {}
            Decision::Unresolved => unresolved_observed += 1,
            Decision::NoMatchInScope => unrepresented_in_scope_observed += 1,
            Decision::OutOfScope => out_of_scope_observed += 1,
        }
        if rows[0].scope == ScopeStatus::Unknown {
            unknown_scope_observed += 1;
        }
    }

    let unsupported_basis = metadata.reference_coverage_basis != "DECLARED_ENUMERATED_REGISTER";
    let has_blockers = omitted_reference_units > 0
        || duplicate_reference_units > 0
        || unresolved_reference_units > 0
        || unrepresented_in_scope_observed > 0
        || unknown_scope_observed > 0
        || unresolved_observed > 0;

    let disposition = if unsupported_basis {
        "NOT_ESTIMABLE_REFERENCE_BASIS_UNSUPPORTED"
    } else if has_blockers {
        "INCOMPLETE_OR_CONFLICTED"
    } else {
        "RECONCILED_TO_DECLARED_REFERENCE_FRAME"
    };

    Ok(AuditReport {
        audit_id: metadata.audit_id,
        disposition: disposition.to_owned(),
        reference_units: references.len(),
        observed_units: observed.len(),
        matched_reference_units,
        omitted_reference_units,
        duplicate_reference_units,
        unresolved_reference_units,
        unrepresented_in_scope_observed,
        out_of_scope_observed,
        unknown_scope_observed,
        unresolved_observed,
        match_edges,
        unresolved_edges,
        reference_coverage_basis: metadata.reference_coverage_basis,
        crosswalk_independence_assessment: metadata.crosswalk_independence_assessment,
    })
}
