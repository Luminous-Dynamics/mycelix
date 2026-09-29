//! Pure qualification rules for provider observations.
//!
//! This layer intentionally performs no I/O. Connectors/adapters can map
//! provider records into MarketObservation and then apply these rules.
//! A rejected qualification does not erase the raw observation.

use super::{EvidenceStatus, MarketObservation};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum QualificationDecision {
    Qualified,
    Stale,
    Conflicting,
    FutureInaccessible,
    Unavailable,
    Protected,
}

/// Validate an observation against an information frontier.
///
/// The raw record remains intact regardless of the result. A future
/// observation is not made visible merely because it arrived early.
pub fn qualify_observation(
    observation: &MarketObservation,
    frontier_micros: i64,
    stale_after_micros: Option<i64>,
) -> QualificationDecision {
    if observation.status == EvidenceStatus::Unavailable {
        return QualificationDecision::Unavailable;
    }
    if observation.status == EvidenceStatus::Protected {
        return QualificationDecision::Protected;
    }

    let available_at = observation
        .available_at_micros
        .unwrap_or(observation.ingested_at_micros);

    if available_at > frontier_micros
        || observation.information_frontier.as_of_micros > frontier_micros
    {
        return QualificationDecision::FutureInaccessible;
    }

    if observation.status == EvidenceStatus::Conflicting {
        return QualificationDecision::Conflicting;
    }

    if let Some(max_age) = stale_after_micros {
        if frontier_micros.saturating_sub(observation.observed_at_micros) > max_age {
            return QualificationDecision::Stale;
        }
    }

    QualificationDecision::Qualified
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        DecimalValue, FinancialSubjectKind, FinancialSubjectRef, IdentifierAlias,
        InformationFrontier, MarketUnit, SourceRef, ValidityInterval,
    };

    fn observation(status: EvidenceStatus, observed_at: i64, available_at: i64) -> MarketObservation {
        MarketObservation {
            observation_id: "obs-1".into(),
            subject: FinancialSubjectRef {
                subject_id: "instrument:example".into(),
                subject_kind: FinancialSubjectKind::Instrument,
                validity_interval: ValidityInterval {
                    valid_from_micros: Some(0),
                    valid_to_micros: None,
                },
                identifier_aliases: vec![IdentifierAlias {
                    namespace: "ticker".into(),
                    value: "EXM".into(),
                    valid_from_micros: Some(0),
                    valid_to_micros: None,
                }],
                lineage_relations: vec![],
                information_frontier: InformationFrontier {
                    as_of_micros: available_at,
                    frontier_id: "source-frontier".into(),
                },
            },
            observed_at_micros: observed_at,
            published_at_micros: None,
            available_at_micros: Some(available_at),
            ingested_at_micros: available_at,
            unit: MarketUnit::Price { currency: "USD".into() },
            value: DecimalValue { value: "123.45".into(), scale: 2 },
            status,
            source: SourceRef {
                source_id: "source-1".into(),
                provider_id: "provider-a".into(),
                common_ancestry_id: Some("upstream-a".into()),
                retrieved_at_micros: available_at,
            },
            evidence_refs: vec![],
            information_frontier: InformationFrontier {
                as_of_micros: available_at,
                frontier_id: "source-frontier".into(),
            },
        }
    }

    #[test]
    fn qualified_observation_is_visible_at_frontier() {
        let o = observation(EvidenceStatus::ObservedUnqualified, 90, 95);
        assert_eq!(qualify_observation(&o, 100, Some(20)), QualificationDecision::Qualified);
    }

    #[test]
    fn stale_is_explicit_not_deleted() {
        let o = observation(EvidenceStatus::ObservedUnqualified, 10, 20);
        assert_eq!(qualify_observation(&o, 100, Some(20)), QualificationDecision::Stale);
        assert_eq!(o.status, EvidenceStatus::ObservedUnqualified);
    }

    #[test]
    fn future_availability_cannot_cross_frontier() {
        let o = observation(EvidenceStatus::ObservedUnqualified, 90, 101);
        assert_eq!(qualify_observation(&o, 100, Some(20)), QualificationDecision::FutureInaccessible);
    }

    #[test]
    fn conflict_is_preserved() {
        let o = observation(EvidenceStatus::Conflicting, 90, 95);
        assert_eq!(qualify_observation(&o, 100, Some(20)), QualificationDecision::Conflicting);
    }

    #[test]
    fn protected_and_unavailable_fail_closed() {
        let protected = observation(EvidenceStatus::Protected, 90, 95);
        let unavailable = observation(EvidenceStatus::Unavailable, 90, 95);
        assert_eq!(qualify_observation(&protected, 100, Some(20)), QualificationDecision::Protected);
        assert_eq!(qualify_observation(&unavailable, 100, Some(20)), QualificationDecision::Unavailable);
    }
}
