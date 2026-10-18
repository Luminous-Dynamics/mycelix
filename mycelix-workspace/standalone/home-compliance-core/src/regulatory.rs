//! Time-aware regulatory source metadata.
//!
//! This module separates a source's legal/publishing status from package
//! lifecycle. It prevents proposals, guidance, stale source checks, and
//! unknown legal status from being silently treated as current binding rules.
//! It does not interpret statutes or establish the law for any jurisdiction.

use std::collections::HashMap;

fn nonblank(value: &str) -> bool {
    !value.trim().is_empty()
}

fn is_sha256_hex(value: &str) -> bool {
    value.len() == 64 && value.bytes().all(|byte| byte.is_ascii_hexdigit())
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LegalSourceStatus {
    InForce,
    AdoptedNotYetEffective,
    Proposed,
    Guidance,
    Withdrawn,
    Unknown,
}

/// Explicitly reviewed geographic applicability for a legal source.
///
/// Geographic ancestry is not enough on its own to establish legal effect.
/// The profile must select the mode after checking the instrument and the
/// jurisdiction's actual delegation/applicability rules. The evaluator never
/// infers descendant reach just from the shape of a jurisdiction ID.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum JurisdictionApplicabilityScope {
    /// Source contributes only when its jurisdiction ID exactly equals the
    /// requested project jurisdiction.
    ExactJurisdiction,
    /// Reviewed profile says this source reaches its own jurisdiction and
    /// descendant jurisdictions in the slash-separated path hierarchy.
    JurisdictionAndDescendants,
    /// Reviewed source/profile enumerates exact project jurisdiction IDs.
    ExplicitJurisdictions(Vec<String>),
}

fn valid_jurisdiction_path(path: &str) -> bool {
    !path.trim().is_empty()
        && path.split('/').all(|segment| {
            !segment.is_empty()
                && segment
                    .bytes()
                    .all(|byte| byte.is_ascii_alphanumeric() || byte == b'-' || byte == b'_')
        })
}

fn path_is_ancestor_or_equal(ancestor: &str, child: &str) -> bool {
    if !valid_jurisdiction_path(ancestor) || !valid_jurisdiction_path(child) {
        return false;
    }
    let ancestor_parts: Vec<_> = ancestor.split('/').collect();
    let child_parts: Vec<_> = child.split('/').collect();
    ancestor_parts.len() <= child_parts.len()
        && ancestor_parts
            .iter()
            .zip(child_parts.iter())
            .all(|(parent, child)| parent == child)
}

fn source_covers_jurisdiction(
    source: &RegulatorySource,
    requested_jurisdiction: &str,
) -> Result<bool, IndeterminateReason> {
    if !valid_jurisdiction_path(requested_jurisdiction)
        || !valid_jurisdiction_path(&source.jurisdiction)
    {
        return Err(IndeterminateReason::InvalidJurisdictionScope);
    }
    match &source.applicability_scope {
        JurisdictionApplicabilityScope::ExactJurisdiction => {
            Ok(source.jurisdiction == requested_jurisdiction)
        }
        JurisdictionApplicabilityScope::JurisdictionAndDescendants => Ok(
            path_is_ancestor_or_equal(&source.jurisdiction, requested_jurisdiction),
        ),
        JurisdictionApplicabilityScope::ExplicitJurisdictions(targets) => {
            let mut seen = std::collections::HashSet::new();
            for target in targets {
                if !valid_jurisdiction_path(target) || !seen.insert(target.as_str()) {
                    return Err(IndeterminateReason::InvalidJurisdictionScope);
                }
            }
            Ok(targets.iter().any(|target| target == requested_jurisdiction))
        }
    }
}

/// Metadata for a jurisdiction-specific source snapshot.
///
/// source_sha256 should identify the exact locally preserved source artifact or
/// a well-defined normalized metadata package. The evaluator checks syntax
/// only; the archival adapter must hash the actual bytes and verify provenance.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RegulatorySource {
    pub source_id: String,
    pub jurisdiction: String,
    pub title: String,
    pub edition: String,
    pub source_uri: String,
    pub source_sha256: String,
    /// Jurisdictional reach reviewed for this instrument; never inferred merely
    /// from slash-separated IDs.
    pub applicability_scope: JurisdictionApplicabilityScope,
    pub legal_status: LegalSourceStatus,
    pub published_on_day: u32,
    pub effective_from_day: Option<u32>,
    /// Exclusive end date: the source is not applicable on or after this day.
    pub effective_until_day_exclusive: Option<u32>,
    /// Day an authorized maintainer last checked source/currentness metadata.
    pub checked_on_day: u32,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum NonBindingReason {
    Proposal,
    Guidance,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum NotInEffectReason {
    NotYetEffective,
    Expired,
    Withdrawn,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ScopeExclusionReason {
    JurisdictionMismatch,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum IndeterminateReason {
    InvalidMetadata(&'static str),
    InvalidSourceDigest,
    FuturePublicationDate,
    FutureVerificationDate,
    InvalidValidityWindow,
    MissingEffectiveDate,
    StaleSourceVerification,
    ContradictoryStatusAndDates,
    UnknownLegalStatus,
    InvalidJurisdictionScope,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum SourceApplicability {
    Applicable,
    NonBinding(NonBindingReason),
    NotInEffect(NotInEffectReason),
    OutOfScope(ScopeExclusionReason),
    Indeterminate(IndeterminateReason),
}

/// Evaluate whether a source is eligible to contribute binding requirements.
///
/// law_as_of_day is the date being assessed under the law. registry_as_of_day
/// is when the caller wants to know what the system currently knows. These are
/// distinct clocks: a current re-assessment may use today's verified registry,
/// while a historical assessment should load the registry snapshot preserved
/// for that historic assessment. max_verification_age_days is a policy chosen
/// by the jurisdiction profile, not a universal legal rule.
pub fn evaluate_source_applicability(
    source: &RegulatorySource,
    requested_jurisdiction: &str,
    law_as_of_day: u32,
    registry_as_of_day: u32,
    max_verification_age_days: u32,
) -> SourceApplicability {
    for (field, value) in [
        ("source_id", source.source_id.as_str()),
        ("jurisdiction", source.jurisdiction.as_str()),
        ("title", source.title.as_str()),
        ("edition", source.edition.as_str()),
        ("source_uri", source.source_uri.as_str()),
    ] {
        if !nonblank(value) {
            return SourceApplicability::Indeterminate(IndeterminateReason::InvalidMetadata(field));
        }
    }
    if !is_sha256_hex(&source.source_sha256) {
        return SourceApplicability::Indeterminate(IndeterminateReason::InvalidSourceDigest);
    }
    if source.published_on_day > registry_as_of_day {
        return SourceApplicability::Indeterminate(IndeterminateReason::FuturePublicationDate);
    }
    if source.checked_on_day > registry_as_of_day {
        return SourceApplicability::Indeterminate(IndeterminateReason::FutureVerificationDate);
    }
    if registry_as_of_day.saturating_sub(source.checked_on_day) > max_verification_age_days {
        return SourceApplicability::Indeterminate(IndeterminateReason::StaleSourceVerification);
    }
    if let (Some(start), Some(end)) = (
        source.effective_from_day,
        source.effective_until_day_exclusive,
    ) {
        if end <= start {
            return SourceApplicability::Indeterminate(IndeterminateReason::InvalidValidityWindow);
        }
    }

    let covers = match source_covers_jurisdiction(source, requested_jurisdiction) {
        Ok(covers) => covers,
        Err(reason) => return SourceApplicability::Indeterminate(reason),
    };
    if !covers {
        return SourceApplicability::OutOfScope(ScopeExclusionReason::JurisdictionMismatch);
    }

    match source.legal_status {
        LegalSourceStatus::Proposed => SourceApplicability::NonBinding(NonBindingReason::Proposal),
        LegalSourceStatus::Guidance => SourceApplicability::NonBinding(NonBindingReason::Guidance),
        LegalSourceStatus::Withdrawn => {
            SourceApplicability::NotInEffect(NotInEffectReason::Withdrawn)
        }
        LegalSourceStatus::Unknown => {
            SourceApplicability::Indeterminate(IndeterminateReason::UnknownLegalStatus)
        }
        LegalSourceStatus::AdoptedNotYetEffective => {
            let Some(effective_from) = source.effective_from_day else {
                return SourceApplicability::Indeterminate(
                    IndeterminateReason::MissingEffectiveDate,
                );
            };
            if law_as_of_day < effective_from {
                SourceApplicability::NotInEffect(NotInEffectReason::NotYetEffective)
            } else {
                // The source registry claims the instrument is not yet effective
                // after its asserted effective date: a maintainer must reconcile it.
                SourceApplicability::Indeterminate(IndeterminateReason::ContradictoryStatusAndDates)
            }
        }
        LegalSourceStatus::InForce => {
            let Some(effective_from) = source.effective_from_day else {
                return SourceApplicability::Indeterminate(
                    IndeterminateReason::MissingEffectiveDate,
                );
            };
            if law_as_of_day < effective_from {
                return SourceApplicability::NotInEffect(NotInEffectReason::NotYetEffective);
            }
            if source
                .effective_until_day_exclusive
                .is_some_and(|end| law_as_of_day >= end)
            {
                return SourceApplicability::NotInEffect(NotInEffectReason::Expired);
            }
            SourceApplicability::Applicable
        }
    }
}

/// A candidate assertion for a single normalized requirement. The source's
/// legal applicability must come from evaluate_source_applicability; this
/// resolver never invents legal precedence or interprets clause text.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RequirementAssertion {
    pub requirement_id: String,
    pub source_id: String,
    pub clause_ref: String,
    /// Canonical, typed encoding such as a dimensioned bound or enum value.
    /// This core compares values exactly; it does not infer which bound is safer.
    pub normalized_value: String,
    /// Jurisdiction-specific, reviewed precedence rank. Higher means higher
    /// precedence in this configured profile; no universal hierarchy is assumed.
    pub precedence_rank: Option<i32>,
    pub applicability: SourceApplicability,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RequirementResolutionPolicy {
    RequireAgreement,
    UseExplicitPrecedence,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SourceClauseRef {
    pub source_id: String,
    pub clause_ref: String,
    pub precedence_rank: Option<i32>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ResolutionBasis {
    Agreement,
    ExplicitJurisdictionPrecedence,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum RequirementResolutionIssue {
    NoApplicableAssertions,
    InvalidAssertionMetadata,
    MixedRequirementIds,
    DuplicateSourceClause {
        source_id: String,
        clause_ref: String,
    },
    SourceStatusIndeterminate {
        source_id: String,
        reason: IndeterminateReason,
    },
    MissingExplicitPrecedence {
        source_id: String,
    },
    NoPrecedenceRanks,
    ConflictingHighestPrecedenceValues,
    ConflictingValues,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum RequirementResolution {
    Resolved {
        requirement_id: String,
        normalized_value: String,
        basis: ResolutionBasis,
        contributing_sources: Vec<SourceClauseRef>,
        superseded_sources: Vec<SourceClauseRef>,
    },
    Conflict {
        requirement_id: String,
        candidates: Vec<(String, Vec<SourceClauseRef>)>,
    },
    Indeterminate(RequirementResolutionIssue),
}

fn clause_ref(assertion: &RequirementAssertion) -> SourceClauseRef {
    SourceClauseRef {
        source_id: assertion.source_id.clone(),
        clause_ref: assertion.clause_ref.clone(),
        precedence_rank: assertion.precedence_rank,
    }
}

/// Resolve assertions for one requirement without silently hiding legal
/// conflicts. Non-binding and out-of-scope assertions do not determine a
/// binding requirement. Any applicable-source status that is indeterminate
/// blocks resolution because the authoritative source set may be incomplete.
///
/// RequireAgreement is the default, conservative policy. Explicit precedence
/// is usable only when the jurisdiction profile supplies a rank for every
/// applicable source and the highest-ranked sources agree. Rank assignment is
/// itself a legal/profile governance responsibility, not inferred here.
pub fn resolve_requirement(
    assertions: &[RequirementAssertion],
    policy: RequirementResolutionPolicy,
) -> RequirementResolution {
    let mut applicable = Vec::new();
    for assertion in assertions {
        if let SourceApplicability::Indeterminate(reason) = &assertion.applicability {
            return RequirementResolution::Indeterminate(
                RequirementResolutionIssue::SourceStatusIndeterminate {
                    source_id: assertion.source_id.clone(),
                    reason: reason.clone(),
                },
            );
        }
        if assertion.applicability == SourceApplicability::Applicable {
            applicable.push(assertion);
        }
    }

    if applicable.is_empty() {
        return RequirementResolution::Indeterminate(
            RequirementResolutionIssue::NoApplicableAssertions,
        );
    }

    if applicable.iter().any(|assertion| {
        !nonblank(&assertion.requirement_id)
            || !nonblank(&assertion.source_id)
            || !nonblank(&assertion.clause_ref)
            || !nonblank(&assertion.normalized_value)
    }) {
        return RequirementResolution::Indeterminate(
            RequirementResolutionIssue::InvalidAssertionMetadata,
        );
    }

    let requirement_id = applicable[0].requirement_id.as_str();
    if applicable
        .iter()
        .any(|assertion| assertion.requirement_id != requirement_id)
    {
        return RequirementResolution::Indeterminate(
            RequirementResolutionIssue::MixedRequirementIds,
        );
    }

    let mut seen_clauses = std::collections::HashSet::new();
    for assertion in &applicable {
        let key = (assertion.source_id.as_str(), assertion.clause_ref.as_str());
        if !seen_clauses.insert(key) {
            return RequirementResolution::Indeterminate(
                RequirementResolutionIssue::DuplicateSourceClause {
                    source_id: assertion.source_id.clone(),
                    clause_ref: assertion.clause_ref.clone(),
                },
            );
        }
    }

    match policy {
        RequirementResolutionPolicy::RequireAgreement => {
            let first_value = applicable[0].normalized_value.as_str();
            if applicable
                .iter()
                .any(|assertion| assertion.normalized_value != first_value)
            {
                let mut grouped: HashMap<String, Vec<SourceClauseRef>> = HashMap::new();
                for assertion in &applicable {
                    grouped
                        .entry(assertion.normalized_value.clone())
                        .or_default()
                        .push(clause_ref(assertion));
                }
                let mut candidates: Vec<_> = grouped.into_iter().collect();
                candidates.sort_by(|left, right| left.0.cmp(&right.0));
                for (_, sources) in &mut candidates {
                    sources.sort_by(|left, right| {
                        (&left.source_id, &left.clause_ref)
                            .cmp(&(&right.source_id, &right.clause_ref))
                    });
                }
                return RequirementResolution::Conflict {
                    requirement_id: requirement_id.to_owned(),
                    candidates,
                };
            }

            let mut sources: Vec<_> = applicable.iter().map(|item| clause_ref(item)).collect();
            sources.sort_by(|left, right| {
                (&left.source_id, &left.clause_ref).cmp(&(&right.source_id, &right.clause_ref))
            });
            RequirementResolution::Resolved {
                requirement_id: requirement_id.to_owned(),
                normalized_value: first_value.to_owned(),
                basis: ResolutionBasis::Agreement,
                contributing_sources: sources,
                superseded_sources: Vec::new(),
            }
        }
        RequirementResolutionPolicy::UseExplicitPrecedence => {
            if let Some(assertion) = applicable
                .iter()
                .find(|assertion| assertion.precedence_rank.is_none())
            {
                return RequirementResolution::Indeterminate(
                    RequirementResolutionIssue::MissingExplicitPrecedence {
                        source_id: assertion.source_id.clone(),
                    },
                );
            }

            let Some(highest) = applicable
                .iter()
                .filter_map(|assertion| assertion.precedence_rank)
                .max()
            else {
                return RequirementResolution::Indeterminate(
                    RequirementResolutionIssue::NoPrecedenceRanks,
                );
            };
            let top: Vec<_> = applicable
                .iter()
                .copied()
                .filter(|assertion| assertion.precedence_rank == Some(highest))
                .collect();
            let first_value = top[0].normalized_value.as_str();
            if top
                .iter()
                .any(|assertion| assertion.normalized_value != first_value)
            {
                let mut grouped: HashMap<String, Vec<SourceClauseRef>> = HashMap::new();
                for assertion in top {
                    grouped
                        .entry(assertion.normalized_value.clone())
                        .or_default()
                        .push(clause_ref(assertion));
                }
                let mut candidates: Vec<_> = grouped.into_iter().collect();
                candidates.sort_by(|left, right| left.0.cmp(&right.0));
                return RequirementResolution::Conflict {
                    requirement_id: requirement_id.to_owned(),
                    candidates,
                };
            }

            let top_ids: std::collections::HashSet<(&str, &str)> = top
                .iter()
                .map(|assertion| (assertion.source_id.as_str(), assertion.clause_ref.as_str()))
                .collect();
            let mut contributing_sources: Vec<_> =
                top.iter().map(|item| clause_ref(item)).collect();
            let mut superseded_sources: Vec<_> = applicable
                .iter()
                .filter(|item| {
                    !top_ids.contains(&(item.source_id.as_str(), item.clause_ref.as_str()))
                })
                .map(|item| clause_ref(item))
                .collect();
            contributing_sources.sort_by(|left, right| {
                (&left.source_id, &left.clause_ref).cmp(&(&right.source_id, &right.clause_ref))
            });
            superseded_sources.sort_by(|left, right| {
                (&left.source_id, &left.clause_ref).cmp(&(&right.source_id, &right.clause_ref))
            });
            RequirementResolution::Resolved {
                requirement_id: requirement_id.to_owned(),
                normalized_value: first_value.to_owned(),
                basis: ResolutionBasis::ExplicitJurisdictionPrecedence,
                contributing_sources,
                superseded_sources,
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn source(status: LegalSourceStatus) -> RegulatorySource {
        RegulatorySource {
            source_id: "za-example-instrument".into(),
            jurisdiction: "ZA/demo-municipality".into(),
            title: "Synthetic residential instrument".into(),
            edition: "fixture-1".into(),
            source_uri: "https://example.invalid/instrument".into(),
            source_sha256: "c".repeat(64),
            applicability_scope: JurisdictionApplicabilityScope::ExactJurisdiction,
            legal_status: status,
            published_on_day: 10,
            effective_from_day: Some(20),
            effective_until_day_exclusive: Some(100),
            checked_on_day: 50,
        }
    }

    fn evaluate(source: &RegulatorySource, law_day: u32, registry_day: u32) -> SourceApplicability {
        evaluate_source_applicability(source, "ZA/demo-municipality", law_day, registry_day, 30)
    }

    fn assertion(
        source_id: &str,
        clause_ref: &str,
        normalized_value: &str,
        precedence_rank: Option<i32>,
        applicability: SourceApplicability,
    ) -> RequirementAssertion {
        RequirementAssertion {
            requirement_id: "egress:min-width".into(),
            source_id: source_id.into(),
            clause_ref: clause_ref.into(),
            normalized_value: normalized_value.into(),
            precedence_rank,
            applicability,
        }
    }

    fn applicable_assertion(
        source_id: &str,
        clause_ref: &str,
        normalized_value: &str,
        precedence_rank: Option<i32>,
    ) -> RequirementAssertion {
        assertion(
            source_id,
            clause_ref,
            normalized_value,
            precedence_rank,
            SourceApplicability::Applicable,
        )
    }

    #[test]
    fn active_in_force_source_is_applicable_in_its_window() {
        assert_eq!(
            evaluate(&source(LegalSourceStatus::InForce), 60, 60),
            SourceApplicability::Applicable
        );
    }

    #[test]
    fn proposal_is_never_silently_promoted_to_binding_rule() {
        assert_eq!(
            evaluate(&source(LegalSourceStatus::Proposed), 60, 60),
            SourceApplicability::NonBinding(NonBindingReason::Proposal)
        );
    }

    #[test]
    fn guidance_is_distinguished_from_binding_law() {
        assert_eq!(
            evaluate(&source(LegalSourceStatus::Guidance), 60, 60),
            SourceApplicability::NonBinding(NonBindingReason::Guidance)
        );
    }

    #[test]
    fn future_effective_date_does_not_apply_early() {
        assert_eq!(
            evaluate(&source(LegalSourceStatus::AdoptedNotYetEffective), 19, 60),
            SourceApplicability::NotInEffect(NotInEffectReason::NotYetEffective)
        );
    }

    #[test]
    fn contradictory_future_status_is_indeterminate_after_effective_date() {
        assert_eq!(
            evaluate(&source(LegalSourceStatus::AdoptedNotYetEffective), 20, 60),
            SourceApplicability::Indeterminate(IndeterminateReason::ContradictoryStatusAndDates)
        );
    }

    #[test]
    fn stale_source_check_is_indeterminate_not_current() {
        assert_eq!(
            evaluate(&source(LegalSourceStatus::InForce), 60, 100),
            SourceApplicability::Indeterminate(IndeterminateReason::StaleSourceVerification)
        );
    }

    #[test]
    fn mismatched_jurisdiction_does_not_apply() {
        let value = source(LegalSourceStatus::InForce);
        assert_eq!(
            evaluate_source_applicability(&value, "ZA/other-municipality", 60, 60, 30),
            SourceApplicability::OutOfScope(ScopeExclusionReason::JurisdictionMismatch)
        );
    }

    #[test]
    fn descendant_applicability_requires_explicit_scope_policy() {
        let mut value = source(LegalSourceStatus::InForce);
        value.jurisdiction = "ZA".into();
        assert_eq!(
            evaluate_source_applicability(&value, "ZA/GP/Johannesburg", 60, 60, 30),
            SourceApplicability::OutOfScope(ScopeExclusionReason::JurisdictionMismatch)
        );

        value.applicability_scope =
            JurisdictionApplicabilityScope::JurisdictionAndDescendants;
        assert_eq!(
            evaluate_source_applicability(&value, "ZA/GP/Johannesburg", 60, 60, 30),
            SourceApplicability::Applicable
        );
    }

    #[test]
    fn descendant_scope_does_not_leak_to_sibling_jurisdictions() {
        let mut value = source(LegalSourceStatus::InForce);
        value.jurisdiction = "ZA/GP".into();
        value.applicability_scope =
            JurisdictionApplicabilityScope::JurisdictionAndDescendants;
        assert_eq!(
            evaluate_source_applicability(&value, "ZA/EC/municipality-a", 60, 60, 30),
            SourceApplicability::OutOfScope(ScopeExclusionReason::JurisdictionMismatch)
        );
        assert_eq!(
            evaluate_source_applicability(&value, "ZA/GP/Johannesburg", 60, 60, 30),
            SourceApplicability::Applicable
        );
    }

    #[test]
    fn explicit_jurisdiction_set_is_exact_and_does_not_expand_automatically() {
        let mut value = source(LegalSourceStatus::InForce);
        value.jurisdiction = "ZA".into();
        value.applicability_scope =
            JurisdictionApplicabilityScope::ExplicitJurisdictions(vec![
                "ZA/GP/Johannesburg".into(),
                "ZA/GP/Tshwane".into(),
            ]);
        assert_eq!(
            evaluate_source_applicability(&value, "ZA/GP/Johannesburg", 60, 60, 30),
            SourceApplicability::Applicable
        );
        assert_eq!(
            evaluate_source_applicability(&value, "ZA/GP/Ekurhuleni", 60, 60, 30),
            SourceApplicability::OutOfScope(ScopeExclusionReason::JurisdictionMismatch)
        );
    }

    #[test]
    fn malformed_or_duplicate_explicit_scope_fails_closed() {
        let mut value = source(LegalSourceStatus::InForce);
        value.applicability_scope =
            JurisdictionApplicabilityScope::ExplicitJurisdictions(vec![
                "ZA/GP/Johannesburg".into(),
                "ZA/GP/Johannesburg".into(),
            ]);
        assert_eq!(
            evaluate_source_applicability(&value, "ZA/GP/Johannesburg", 60, 60, 30),
            SourceApplicability::Indeterminate(IndeterminateReason::InvalidJurisdictionScope)
        );

        value.applicability_scope =
            JurisdictionApplicabilityScope::ExplicitJurisdictions(vec!["ZA//broken".into()]);
        assert_eq!(
            evaluate_source_applicability(&value, "ZA/GP/Johannesburg", 60, 60, 30),
            SourceApplicability::Indeterminate(IndeterminateReason::InvalidJurisdictionScope)
        );
    }

    #[test]
    fn expired_source_does_not_apply() {
        assert_eq!(
            evaluate(&source(LegalSourceStatus::InForce), 100, 100),
            SourceApplicability::Indeterminate(IndeterminateReason::StaleSourceVerification)
        );
        // Fresh source check isolates the legal effective-window result.
        let mut value = source(LegalSourceStatus::InForce);
        value.checked_on_day = 100;
        assert_eq!(
            evaluate(&value, 100, 100),
            SourceApplicability::NotInEffect(NotInEffectReason::Expired)
        );
    }

    #[test]
    fn malformed_digest_and_unknown_status_are_indeterminate() {
        let mut value = source(LegalSourceStatus::InForce);
        value.source_sha256 = "bad".into();
        assert_eq!(
            evaluate(&value, 60, 60),
            SourceApplicability::Indeterminate(IndeterminateReason::InvalidSourceDigest)
        );

        assert_eq!(
            evaluate(&source(LegalSourceStatus::Unknown), 60, 60),
            SourceApplicability::Indeterminate(IndeterminateReason::UnknownLegalStatus)
        );
    }

    #[test]
    fn withdrawn_source_is_not_applicable() {
        let mut value = source(LegalSourceStatus::Withdrawn);
        value.checked_on_day = 60;
        assert_eq!(
            evaluate(&value, 60, 60),
            SourceApplicability::NotInEffect(NotInEffectReason::Withdrawn)
        );
    }

    #[test]
    fn invalid_effective_window_is_indeterminate() {
        let mut value = source(LegalSourceStatus::InForce);
        value.effective_from_day = Some(80);
        value.effective_until_day_exclusive = Some(70);
        assert_eq!(
            evaluate(&value, 60, 60),
            SourceApplicability::Indeterminate(IndeterminateReason::InvalidValidityWindow)
        );
    }

    #[test]
    fn future_publication_or_verification_date_is_indeterminate() {
        let mut future_publication = source(LegalSourceStatus::InForce);
        future_publication.published_on_day = 61;
        assert_eq!(
            evaluate(&future_publication, 60, 60),
            SourceApplicability::Indeterminate(IndeterminateReason::FuturePublicationDate)
        );

        let mut future_check = source(LegalSourceStatus::InForce);
        future_check.checked_on_day = 61;
        assert_eq!(
            evaluate(&future_check, 60, 60),
            SourceApplicability::Indeterminate(IndeterminateReason::FutureVerificationDate)
        );
    }

    #[test]
    fn missing_source_identity_is_indeterminate() {
        let mut value = source(LegalSourceStatus::InForce);
        value.source_id.clear();
        assert_eq!(
            evaluate(&value, 60, 60),
            SourceApplicability::Indeterminate(IndeterminateReason::InvalidMetadata("source_id"))
        );
    }

    #[test]
    fn agreeing_requirements_resolve_with_all_provenance() {
        let assertions = vec![
            applicable_assertion("national-code", "clause-1", "min-width:900mm", Some(10)),
            applicable_assertion("municipal-scheme", "clause-8", "min-width:900mm", Some(20)),
        ];
        let result =
            resolve_requirement(&assertions, RequirementResolutionPolicy::RequireAgreement);
        match result {
            RequirementResolution::Resolved {
                normalized_value,
                basis,
                contributing_sources,
                superseded_sources,
                ..
            } => {
                assert_eq!(normalized_value, "min-width:900mm");
                assert_eq!(basis, ResolutionBasis::Agreement);
                assert_eq!(contributing_sources.len(), 2);
                assert!(superseded_sources.is_empty());
            }
            other => panic!("expected an agreed resolution, got {other:?}"),
        }
    }

    #[test]
    fn conflicting_requirements_are_not_guessed_away() {
        let assertions = vec![
            applicable_assertion("source-a", "clause-1", "min-width:850mm", Some(10)),
            applicable_assertion("source-b", "clause-2", "min-width:900mm", Some(20)),
        ];
        assert!(matches!(
            resolve_requirement(&assertions, RequirementResolutionPolicy::RequireAgreement),
            RequirementResolution::Conflict { .. }
        ));
    }

    #[test]
    fn explicit_precedence_resolves_only_when_top_rank_agrees() {
        let assertions = vec![
            applicable_assertion("source-low", "clause-a", "min-width:850mm", Some(10)),
            applicable_assertion("source-high", "clause-b", "min-width:900mm", Some(20)),
        ];
        match resolve_requirement(
            &assertions,
            RequirementResolutionPolicy::UseExplicitPrecedence,
        ) {
            RequirementResolution::Resolved {
                normalized_value,
                basis,
                contributing_sources,
                superseded_sources,
                ..
            } => {
                assert_eq!(normalized_value, "min-width:900mm");
                assert_eq!(basis, ResolutionBasis::ExplicitJurisdictionPrecedence);
                assert_eq!(contributing_sources.len(), 1);
                assert_eq!(contributing_sources[0].source_id, "source-high");
                assert_eq!(superseded_sources.len(), 1);
                assert_eq!(superseded_sources[0].source_id, "source-low");
            }
            other => panic!("expected precedence resolution, got {other:?}"),
        }
    }

    #[test]
    fn equal_highest_precedence_conflict_remains_blocked() {
        let assertions = vec![
            applicable_assertion("source-a", "clause-1", "value-a", Some(20)),
            applicable_assertion("source-b", "clause-2", "value-b", Some(20)),
            applicable_assertion("source-low", "clause-3", "value-low", Some(10)),
        ];
        assert!(matches!(
            resolve_requirement(
                &assertions,
                RequirementResolutionPolicy::UseExplicitPrecedence
            ),
            RequirementResolution::Conflict { .. }
        ));
    }

    #[test]
    fn missing_precedence_cannot_be_invented_by_the_resolver() {
        let assertions = vec![applicable_assertion(
            "source-a", "clause-1", "value-a", None,
        )];
        assert_eq!(
            resolve_requirement(
                &assertions,
                RequirementResolutionPolicy::UseExplicitPrecedence
            ),
            RequirementResolution::Indeterminate(
                RequirementResolutionIssue::MissingExplicitPrecedence {
                    source_id: "source-a".into()
                }
            )
        );
    }

    #[test]
    fn indeterminate_source_status_blocks_a_partial_resolution() {
        let assertions = vec![
            applicable_assertion("source-current", "clause-1", "value-a", Some(10)),
            assertion(
                "source-uncertain",
                "clause-2",
                "value-a",
                Some(20),
                SourceApplicability::Indeterminate(IndeterminateReason::StaleSourceVerification),
            ),
        ];
        assert!(matches!(
            resolve_requirement(&assertions, RequirementResolutionPolicy::RequireAgreement),
            RequirementResolution::Indeterminate(
                RequirementResolutionIssue::SourceStatusIndeterminate { .. }
            )
        ));
    }

    #[test]
    fn proposal_does_not_count_as_an_applicable_binding_requirement() {
        let assertions = vec![assertion(
            "proposed-change",
            "draft-clause",
            "min-width:1000mm",
            Some(100),
            SourceApplicability::NonBinding(NonBindingReason::Proposal),
        )];
        assert_eq!(
            resolve_requirement(&assertions, RequirementResolutionPolicy::RequireAgreement),
            RequirementResolution::Indeterminate(
                RequirementResolutionIssue::NoApplicableAssertions
            )
        );
    }

    #[test]
    fn duplicate_source_clauses_are_rejected() {
        let value = applicable_assertion("source-a", "clause-1", "value-a", Some(10));
        assert_eq!(
            resolve_requirement(
                &[value.clone(), value],
                RequirementResolutionPolicy::RequireAgreement
            ),
            RequirementResolution::Indeterminate(
                RequirementResolutionIssue::DuplicateSourceClause {
                    source_id: "source-a".into(),
                    clause_ref: "clause-1".into(),
                }
            )
        );
    }

    #[test]
    fn mixed_requirement_ids_are_rejected() {
        let first = applicable_assertion("source-a", "clause-1", "value-a", Some(10));
        let mut second = applicable_assertion("source-b", "clause-2", "value-a", Some(10));
        second.requirement_id = "fire:rating".into();
        assert_eq!(
            resolve_requirement(
                &[first, second],
                RequirementResolutionPolicy::RequireAgreement
            ),
            RequirementResolution::Indeterminate(RequirementResolutionIssue::MixedRequirementIds)
        );
    }

    #[test]
    fn missing_effective_date_blocks_in_force_source() {
        let mut value = source(LegalSourceStatus::InForce);
        value.effective_from_day = None;
        assert_eq!(
            evaluate(&value, 60, 60),
            SourceApplicability::Indeterminate(IndeterminateReason::MissingEffectiveDate)
        );
    }
}
