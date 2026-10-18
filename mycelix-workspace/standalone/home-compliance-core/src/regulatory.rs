//! Time-aware regulatory source metadata.
//!
//! This module separates a source's legal/publishing status from package
//! lifecycle. It prevents proposals, guidance, stale source checks, and
//! unknown legal status from being silently treated as current binding rules.
//! It does not interpret statutes or establish the law for any jurisdiction.

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
            return SourceApplicability::Indeterminate(
                IndeterminateReason::InvalidMetadata(field),
            );
        }
    }
    if !is_sha256_hex(&source.source_sha256) {
        return SourceApplicability::Indeterminate(
            IndeterminateReason::InvalidSourceDigest,
        );
    }
    if source.published_on_day > registry_as_of_day {
        return SourceApplicability::Indeterminate(
            IndeterminateReason::FuturePublicationDate,
        );
    }
    if source.checked_on_day > registry_as_of_day {
        return SourceApplicability::Indeterminate(
            IndeterminateReason::FutureVerificationDate,
        );
    }
    if registry_as_of_day.saturating_sub(source.checked_on_day) > max_verification_age_days {
        return SourceApplicability::Indeterminate(
            IndeterminateReason::StaleSourceVerification,
        );
    }
    if let (Some(start), Some(end)) = (
        source.effective_from_day,
        source.effective_until_day_exclusive,
    ) {
        if end <= start {
            return SourceApplicability::Indeterminate(
                IndeterminateReason::InvalidValidityWindow,
            );
        }
    }

    if source.jurisdiction != requested_jurisdiction {
        return SourceApplicability::OutOfScope(
            ScopeExclusionReason::JurisdictionMismatch,
        );
    }

    match source.legal_status {
        LegalSourceStatus::Proposed => {
            SourceApplicability::NonBinding(NonBindingReason::Proposal)
        }
        LegalSourceStatus::Guidance => {
            SourceApplicability::NonBinding(NonBindingReason::Guidance)
        }
        LegalSourceStatus::Withdrawn => {
            SourceApplicability::NotInEffect(NotInEffectReason::Withdrawn)
        }
        LegalSourceStatus::Unknown => {
            SourceApplicability::Indeterminate(
                IndeterminateReason::UnknownLegalStatus,
            )
        }
        LegalSourceStatus::AdoptedNotYetEffective => {
            let Some(effective_from) = source.effective_from_day else {
                return SourceApplicability::Indeterminate(
                    IndeterminateReason::MissingEffectiveDate,
                );
            };
            if law_as_of_day < effective_from {
                SourceApplicability::NotInEffect(
                    NotInEffectReason::NotYetEffective,
                )
            } else {
                // The source registry claims the instrument is not yet effective
                // after its asserted effective date: a maintainer must reconcile it.
                SourceApplicability::Indeterminate(
                    IndeterminateReason::ContradictoryStatusAndDates,
                )
            }
        }
        LegalSourceStatus::InForce => {
            let Some(effective_from) = source.effective_from_day else {
                return SourceApplicability::Indeterminate(
                    IndeterminateReason::MissingEffectiveDate,
                );
            };
            if law_as_of_day < effective_from {
                return SourceApplicability::NotInEffect(
                    NotInEffectReason::NotYetEffective,
                );
            }
            if source
                .effective_until_day_exclusive
                .is_some_and(|end| law_as_of_day >= end)
            {
                return SourceApplicability::NotInEffect(
                    NotInEffectReason::Expired,
                );
            }
            SourceApplicability::Applicable
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
            legal_status: status,
            published_on_day: 10,
            effective_from_day: Some(20),
            effective_until_day_exclusive: Some(100),
            checked_on_day: 50,
        }
    }

    fn evaluate(source: &RegulatorySource, law_day: u32, registry_day: u32) -> SourceApplicability {
        evaluate_source_applicability(
            source,
            "ZA/demo-municipality",
            law_day,
            registry_day,
            30,
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
            SourceApplicability::Indeterminate(
                IndeterminateReason::ContradictoryStatusAndDates
            )
        );
    }

    #[test]
    fn stale_source_check_is_indeterminate_not_current() {
        assert_eq!(
            evaluate(&source(LegalSourceStatus::InForce), 60, 100),
            SourceApplicability::Indeterminate(
                IndeterminateReason::StaleSourceVerification
            )
        );
    }

    #[test]
    fn mismatched_jurisdiction_does_not_apply() {
        let value = source(LegalSourceStatus::InForce);
        assert_eq!(
            evaluate_source_applicability(&value, "ZA/other-municipality", 60, 60, 30),
            SourceApplicability::OutOfScope(
                ScopeExclusionReason::JurisdictionMismatch
            )
        );
    }

    #[test]
    fn expired_source_does_not_apply() {
        assert_eq!(
            evaluate(&source(LegalSourceStatus::InForce), 100, 100),
            SourceApplicability::Indeterminate(
                IndeterminateReason::StaleSourceVerification
            )
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
            SourceApplicability::Indeterminate(
                IndeterminateReason::InvalidSourceDigest
            )
        );

        assert_eq!(
            evaluate(&source(LegalSourceStatus::Unknown), 60, 60),
            SourceApplicability::Indeterminate(
                IndeterminateReason::UnknownLegalStatus
            )
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
            SourceApplicability::Indeterminate(
                IndeterminateReason::InvalidValidityWindow
            )
        );
    }

    #[test]
    fn future_publication_or_verification_date_is_indeterminate() {
        let mut future_publication = source(LegalSourceStatus::InForce);
        future_publication.published_on_day = 61;
        assert_eq!(
            evaluate(&future_publication, 60, 60),
            SourceApplicability::Indeterminate(
                IndeterminateReason::FuturePublicationDate
            )
        );

        let mut future_check = source(LegalSourceStatus::InForce);
        future_check.checked_on_day = 61;
        assert_eq!(
            evaluate(&future_check, 60, 60),
            SourceApplicability::Indeterminate(
                IndeterminateReason::FutureVerificationDate
            )
        );
    }

    #[test]
    fn missing_source_identity_is_indeterminate() {
        let mut value = source(LegalSourceStatus::InForce);
        value.source_id.clear();
        assert_eq!(
            evaluate(&value, 60, 60),
            SourceApplicability::Indeterminate(
                IndeterminateReason::InvalidMetadata("source_id")
            )
        );
    }

    #[test]
    fn missing_effective_date_blocks_in_force_source() {
        let mut value = source(LegalSourceStatus::InForce);
        value.effective_from_day = None;
        assert_eq!(
            evaluate(&value, 60, 60),
            SourceApplicability::Indeterminate(
                IndeterminateReason::MissingEffectiveDate
            )
        );
    }
}
