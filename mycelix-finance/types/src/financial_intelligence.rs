//! Evidence-native financial intelligence contracts.
//!
//! These types describe observations and evidence-relative projections rather
//! than a universal or mutable FinancialTruth.
//!
//! The semantic invariants are frozen by the finance constitution and known-
//! answer vectors in docs/finance. This module has no execution, authorization,
//! or connector dependencies.

use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum EvidenceStatus {
    Known,
    ObservedUnqualified,
    Unknown,
    Unavailable,
    Stale,
    Conflicting,
    Protected,
    FutureInaccessible,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct InformationFrontier {
    pub as_of_micros: i64,
    pub frontier_id: String,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct FinancialSubjectRef {
    pub subject_id: String,
    pub subject_kind: FinancialSubjectKind,
    pub validity_interval: ValidityInterval,
    pub identifier_aliases: Vec<IdentifierAlias>,
    pub lineage_relations: Vec<LineageRelation>,
    pub information_frontier: InformationFrontier,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum FinancialSubjectKind {
    Issuer,
    Instrument,
    Listing,
    Venue,
    ShareClass,
    Contract,
    Fund,
    Index,
    Other(String),
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct IdentifierAlias {
    pub namespace: String,
    pub value: String,
    pub valid_from_micros: Option<i64>,
    pub valid_to_micros: Option<i64>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct LineageRelation {
    pub relation: SubjectLineageKind,
    pub related_subject_id: String,
    pub valid_from_micros: Option<i64>,
    pub valid_to_micros: Option<i64>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SubjectLineageKind {
    RenamedFrom,
    ReplacedBy,
    MergedInto,
    SpunOffFrom,
    ListedAs,
    DelistedFrom,
    CorporateActionRelated,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ValidityInterval {
    pub valid_from_micros: Option<i64>,
    pub valid_to_micros: Option<i64>,
}

/// Immutable raw observation envelope. Normalization remains derived.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct MarketObservation {
    pub observation_id: String,
    pub subject: FinancialSubjectRef,
    pub observed_at_micros: i64,
    pub published_at_micros: Option<i64>,
    pub available_at_micros: Option<i64>,
    pub ingested_at_micros: i64,
    pub unit: MarketUnit,
    pub value: DecimalValue,
    pub status: EvidenceStatus,
    pub source: SourceRef,
    pub evidence_refs: Vec<EvidenceRef>,
    pub information_frontier: InformationFrontier,
}

/// Decimal-as-text avoids floating-point ambiguity in wire contracts.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct DecimalValue {
    pub value: String,
    pub scale: u32,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum MarketUnit {
    Price { currency: String },
    Quantity,
    Volume { currency: Option<String> },
    Yield { basis_points: bool },
    MarketCapitalization { currency: String },
    Other(String),
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SourceRef {
    pub source_id: String,
    pub provider_id: String,
    /// Shared ancestry lets consumers distinguish provider count from
    /// evidence independence.
    pub common_ancestry_id: Option<String>,
    pub retrieved_at_micros: i64,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct EvidenceRef {
    pub evidence_id: String,
    pub artifact_id: Option<String>,
    pub source: SourceRef,
    pub status: EvidenceStatus,
    pub observed_at_micros: Option<i64>,
    pub available_at_micros: Option<i64>,
    pub information_frontier: InformationFrontier,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct FinancialArtifact {
    pub artifact_id: String,
    pub subject_refs: Vec<FinancialSubjectRef>,
    pub source: SourceRef,
    pub published_at_micros: Option<i64>,
    pub available_at_micros: Option<i64>,
    pub ingested_at_micros: i64,
    pub content_digest: String,
    pub supersedes: Option<String>,
    pub status: EvidenceStatus,
    pub information_frontier: InformationFrontier,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct FinancialClaim {
    pub claim_id: String,
    pub subject_refs: Vec<FinancialSubjectRef>,
    pub claim_kind: FinancialClaimKind,
    pub statement: String,
    pub status: EvidenceStatus,
    pub evidence_refs: Vec<EvidenceRef>,
    pub derived_from_claims: Vec<String>,
    pub transformation_id: Option<String>,
    pub valid_interval: ValidityInterval,
    pub information_frontier: InformationFrontier,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum FinancialClaimKind {
    Observed,
    Extracted,
    Derived,
    Hypothesis,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct FinancialRelation {
    pub relation_id: String,
    pub subject: FinancialSubjectRef,
    pub object: FinancialSubjectRef,
    pub relation_kind: FinancialRelationKind,
    pub status: EvidenceStatus,
    pub evidence_refs: Vec<EvidenceRef>,
    pub valid_interval: ValidityInterval,
    pub information_frontier: InformationFrontier,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum FinancialRelationKind {
    Ownership,
    Supply,
    Customer,
    Exposure,
    Dependency,
    Correlation,
    CausalHypothesis,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct FinancialEvidenceSet {
    pub evidence_set_id: String,
    pub evidence_refs: Vec<EvidenceRef>,
    pub information_frontier: InformationFrontier,
    pub status: EvidenceStatus,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frontier() -> InformationFrontier {
        InformationFrontier {
            as_of_micros: 1_700_000_000_000_000,
            frontier_id: "frontier-001".into(),
        }
    }

    fn subject() -> FinancialSubjectRef {
        FinancialSubjectRef {
            subject_id: "issuer:example".into(),
            subject_kind: FinancialSubjectKind::Issuer,
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
            information_frontier: frontier(),
        }
    }

    #[test]
    fn ticker_is_alias_not_subject_identity() {
        let s = subject();
        assert_eq!(s.subject_id, "issuer:example");
        assert_eq!(s.identifier_aliases[0].namespace, "ticker");
    }

    #[test]
    fn observation_preserves_temporal_frontier_and_source() {
        let o = MarketObservation {
            observation_id: "obs-001".into(),
            subject: subject(),
            observed_at_micros: 1_700_000_000_000_100,
            published_at_micros: Some(1_700_000_000_000_200),
            available_at_micros: Some(1_700_000_000_000_300),
            ingested_at_micros: 1_700_000_000_000_400,
            unit: MarketUnit::Price { currency: "USD".into() },
            value: DecimalValue { value: "123.45".into(), scale: 2 },
            status: EvidenceStatus::ObservedUnqualified,
            source: SourceRef {
                source_id: "provider-observation-1".into(),
                provider_id: "provider-a".into(),
                common_ancestry_id: Some("upstream-1".into()),
                retrieved_at_micros: 1_700_000_000_000_400,
            },
            evidence_refs: vec![],
            information_frontier: frontier(),
        };

        assert_eq!(o.status, EvidenceStatus::ObservedUnqualified);
        assert_eq!(o.value.value, "123.45");
        assert_eq!(o.source.common_ancestry_id.as_deref(), Some("upstream-1"));
    }

    #[test]
    fn extracted_claim_is_not_an_observed_claim() {
        let c = FinancialClaim {
            claim_id: "claim-001".into(),
            subject_refs: vec![subject()],
            claim_kind: FinancialClaimKind::Extracted,
            statement: "Revenue was reported as 10m USD".into(),
            status: EvidenceStatus::ObservedUnqualified,
            evidence_refs: vec![],
            derived_from_claims: vec![],
            transformation_id: Some("extractor-v1".into()),
            valid_interval: ValidityInterval {
                valid_from_micros: Some(1_700_000_000_000_000),
                valid_to_micros: None,
            },
            information_frontier: frontier(),
        };

        assert_ne!(c.claim_kind, FinancialClaimKind::Observed);
        assert_eq!(c.transformation_id.as_deref(), Some("extractor-v1"));
    }

    #[test]
    fn causal_hypothesis_is_not_correlation() {
        assert_ne!(
            FinancialRelationKind::CausalHypothesis,
            FinancialRelationKind::Correlation
        );
    }

    #[test]
    fn protected_evidence_has_explicit_status() {
        let e = FinancialEvidenceSet {
            evidence_set_id: "set-001".into(),
            evidence_refs: vec![],
            information_frontier: frontier(),
            status: EvidenceStatus::Protected,
        };
        assert_eq!(e.status, EvidenceStatus::Protected);
    }
}
