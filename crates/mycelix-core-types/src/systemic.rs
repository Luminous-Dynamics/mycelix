// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Provider-neutral statement and provenance primitives for systemic evidence.
//!
//! These types deliberately stop below storage, causal inference, or policy
//! authority. They encode what a source asserted, when the assertion applies,
//! when it became known, and how statements relate to one another.
//!
//! Core invariants:
//!
//! ```text
//! statement exists != statement is true
//! source assertion != derived finding
//! possible_same_entity != same_entity
//! valid time != knowledge time
//! contradiction != overwrite
//! display label != entity identity
//! unqualified term != cross-source semantic identity
//! historical time != post-1970-only time
//! ```

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

use std::hash::{Hash, Hasher};

use crate::epistemic::EpistemicClassification;

/// Signed milliseconds relative to the Unix epoch.
///
/// Negative values intentionally represent instants before 1970-01-01T00:00:00Z;
/// systemic evidence frequently describes historical institutions and relations.
/// The core-types crate keeps the representation dependency-free. Adapters may
/// convert richer source-native timestamps into this boundary while preserving
/// the original timestamp text in source metadata when necessary.
pub type UnixMillis = i64;

/// A half-open real-world validity interval `[start, end)`.
///
/// `None` means the bound is unknown/open, not that it is infinite with certainty.
/// When both bounds are known, `end` must be strictly later than `start`; point
/// observations should use `observed_at` rather than an empty `[t, t)` interval.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ValidityInterval {
    pub start: Option<UnixMillis>,
    pub end: Option<UnixMillis>,
}

impl ValidityInterval {
    pub fn new(start: Option<UnixMillis>, end: Option<UnixMillis>) -> Result<Self, &'static str> {
        if let (Some(start), Some(end)) = (start, end) {
            if end <= start {
                return Err("validity interval end must be after start");
            }
        }
        Ok(Self { start, end })
    }

    /// Whether a timestamp is inside the known interval.
    pub fn contains(&self, timestamp: UnixMillis) -> bool {
        let after_start = self.start.is_none_or(|start| timestamp >= start);
        let before_end = self.end.is_none_or(|end| timestamp < end);
        after_start && before_end
    }
}

/// Bitemporal scope for a statement.
///
/// `valid_time` answers "when does the source claim this held in the world?"
/// while `known_at` answers "when did this statement become available to this
/// evidence system?". They MUST NOT be silently collapsed.
#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct BitemporalScope {
    pub valid_time: ValidityInterval,
    pub known_at: UnixMillis,
    /// Optional source-native observation/publication time when distinct from
    /// the evidence-system knowledge time.
    pub observed_at: Option<UnixMillis>,
}

impl BitemporalScope {
    pub fn new(valid_time: ValidityInterval, known_at: UnixMillis) -> Self {
        Self {
            valid_time,
            known_at,
            observed_at: None,
        }
    }

    pub fn with_observed_at(mut self, observed_at: UnixMillis) -> Self {
        self.observed_at = Some(observed_at);
        self
    }
}

/// Broad source category without giving any category privileged truth authority.
#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum SourceKind {
    Registry,
    ContractingRelease,
    Sensor,
    Document,
    Dataset,
    Testimony,
    DerivedAnalysis,
    Other(String),
}

/// Provenance reference to the source that emitted or supplied a statement.
#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct SourceRef {
    /// Stable identifier within the importing system or source namespace.
    pub source_id: String,
    pub kind: SourceKind,
    /// URI, dataset key, file identifier, registry record, etc.
    pub locator: String,
    /// When this representation was retrieved by the evidence system.
    pub retrieved_at: UnixMillis,
    /// Optional externally computed digest of the exact source artifact.
    ///
    /// The digest algorithm/encoding SHOULD be included in the string, e.g.
    /// `sha256:<hex>`. This tranche does not invent its own hashing authority.
    pub content_digest: Option<String>,
}

/// Reference to a specific evidence artifact supporting a statement.
#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct EvidenceRef {
    pub evidence_id: String,
    pub locator: Option<String>,
    pub content_digest: Option<String>,
    pub media_type: Option<String>,
}

/// A namespace-qualified vocabulary term.
///
/// Cross-source normalization must never infer semantic equivalence from the
/// local text of a predicate, metric, entity type, or unit alone. Examples of
/// namespaces include `bods:0.4`, `ocds:1.1`, `ucum`, or a versioned Mycelix
/// vocabulary. Mapping one namespace to another is an explicit adapter/ontology
/// operation outside this primitive.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct VocabularyTerm {
    pub namespace: String,
    pub term: String,
}

impl VocabularyTerm {
    pub fn new(namespace: impl Into<String>, term: impl Into<String>) -> Self {
        Self {
            namespace: namespace.into(),
            term: term.into(),
        }
    }
}

/// A provider-neutral entity identifier.
///
/// Examples of namespaces include `lei`, `did`, a jurisdictional company
/// registry, or a dataset-native namespace. A display label is never identity.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct EntityRef {
    pub namespace: String,
    pub identifier: String,
    pub label: Option<String>,
}

impl EntityRef {
    pub fn new(namespace: impl Into<String>, identifier: impl Into<String>) -> Self {
        Self {
            namespace: namespace.into(),
            identifier: identifier.into(),
            label: None,
        }
    }

    pub fn with_label(mut self, label: impl Into<String>) -> Self {
        self.label = Some(label.into());
        self
    }
}

impl PartialEq for EntityRef {
    fn eq(&self, other: &Self) -> bool {
        self.namespace == other.namespace && self.identifier == other.identifier
    }
}

impl Eq for EntityRef {}

impl Hash for EntityRef {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.namespace.hash(state);
        self.identifier.hash(state);
    }
}

/// Lineage shared by all systemic statements.
///
/// Conflicting statements are expected to retain independent lineage rather
/// than overwriting each other.
#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct StatementLineage {
    pub source: SourceRef,
    pub evidence: Vec<EvidenceRef>,
    /// Statement IDs from which this statement was explicitly derived.
    pub derived_from: Vec<String>,
}

impl StatementLineage {
    pub fn from_source(source: SourceRef) -> Self {
        Self {
            source,
            evidence: Vec::new(),
            derived_from: Vec::new(),
        }
    }
}

/// Statement about an entity's existence or source-described attributes.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct EntityStatement {
    pub statement_id: String,
    pub entity: EntityRef,
    pub entity_type: Option<VocabularyTerm>,
    pub classification: EpistemicClassification,
    pub time: BitemporalScope,
    pub lineage: StatementLineage,
    pub jurisdiction: Option<String>,
    pub scope: Option<String>,
}

/// Statement asserting a typed relationship between two entities.
///
/// The predicate remains source/domain extensible, but it is namespace-qualified
/// so adapters cannot accidentally collapse equal-looking terms from different
/// vocabularies. Adapters SHOULD preserve source-native meaning rather than
/// strengthening it during normalization.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct RelationshipStatement {
    pub statement_id: String,
    pub subject: EntityRef,
    pub predicate: VocabularyTerm,
    pub object: EntityRef,
    pub classification: EpistemicClassification,
    pub time: BitemporalScope,
    pub lineage: StatementLineage,
    pub jurisdiction: Option<String>,
    pub scope: Option<String>,
}

/// Provider-neutral value carried by an observation statement.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum ObservationValue {
    Number {
        value: f64,
        /// Optional namespace-qualified unit, preferably from a stable unit
        /// vocabulary such as UCUM when the source supports that mapping.
        unit: Option<VocabularyTerm>,
    },
    Text(String),
    Boolean(bool),
    Identifier(EntityRef),
}

/// Statement recording an observation/measurement/asserted attribute.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ObservationStatement {
    pub statement_id: String,
    pub subject: EntityRef,
    pub metric: VocabularyTerm,
    pub value: ObservationValue,
    pub classification: EpistemicClassification,
    pub time: BitemporalScope,
    pub lineage: StatementLineage,
    pub jurisdiction: Option<String>,
    pub scope: Option<String>,
}

/// The source-level statement envelope accepted by systemic evidence adapters.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum SystemicStatement {
    Entity(EntityStatement),
    Relationship(RelationshipStatement),
    Observation(ObservationStatement),
}

impl SystemicStatement {
    pub fn statement_id(&self) -> &str {
        match self {
            Self::Entity(statement) => &statement.statement_id,
            Self::Relationship(statement) => &statement.statement_id,
            Self::Observation(statement) => &statement.statement_id,
        }
    }

    pub fn lineage(&self) -> &StatementLineage {
        match self {
            Self::Entity(statement) => &statement.lineage,
            Self::Relationship(statement) => &statement.lineage,
            Self::Observation(statement) => &statement.lineage,
        }
    }
}

/// Explicit semantic relationship between two statement records.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum StatementRelationKind {
    Supports,
    Contradicts,
    Supersedes,
    DerivedFrom,
}

#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct StatementRelation {
    pub relation_id: String,
    pub from_statement: String,
    pub relation: StatementRelationKind,
    pub to_statement: String,
    pub classification: EpistemicClassification,
    pub lineage: StatementLineage,
}

/// Explicit entity-resolution status.
///
/// A candidate match stays a candidate match until stronger evidence admits a
/// `SameEntity` assertion; importers MUST NOT collapse identities implicitly.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum IdentityRelation {
    PossibleSameEntity,
    SameEntity,
    DistinctEntity,
}

#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct IdentityAssertion {
    pub assertion_id: String,
    pub left: EntityRef,
    pub relation: IdentityRelation,
    pub right: EntityRef,
    pub classification: EpistemicClassification,
    pub time: BitemporalScope,
    pub lineage: StatementLineage,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::epistemic::{EmpiricalLevel, MaterialityLevel, NormativeLevel};
    use std::collections::HashSet;

    fn classification() -> EpistemicClassification {
        EpistemicClassification::new(
            EmpiricalLevel::Measurable,
            NormativeLevel::Network,
            MaterialityLevel::Persistent,
        )
    }

    fn source() -> SourceRef {
        SourceRef {
            source_id: "source:registry:test".into(),
            kind: SourceKind::Registry,
            locator: "registry:test/record/42".into(),
            retrieved_at: 2_000,
            content_digest: Some("sha256:deadbeef".into()),
        }
    }

    #[test]
    fn rejects_reversed_or_empty_validity_interval() {
        assert!(ValidityInterval::new(Some(20), Some(10)).is_err());
        assert!(ValidityInterval::new(Some(10), Some(10)).is_err());
        assert!(ValidityInterval::new(Some(10), Some(20)).is_ok());
    }

    #[test]
    fn supports_pre_epoch_valid_time() {
        let historical = ValidityInterval::new(Some(-86_400_000), Some(0)).unwrap();
        assert!(historical.contains(-1));
        assert!(!historical.contains(0));
    }

    #[test]
    fn valid_time_and_knowledge_time_are_distinct() {
        let time = BitemporalScope::new(
            ValidityInterval::new(Some(1_000), Some(1_500)).unwrap(),
            2_000,
        )
        .with_observed_at(1_600);

        assert!(time.valid_time.contains(1_200));
        assert!(!time.valid_time.contains(1_700));
        assert_eq!(time.observed_at, Some(1_600));
        assert_eq!(time.known_at, 2_000);
    }

    #[test]
    fn display_label_is_not_part_of_entity_identity() {
        let first = EntityRef::new("lei", "ABC123").with_label("Acme Corp");
        let second = EntityRef::new("lei", "ABC123").with_label("ACME CORPORATION");

        assert_eq!(first, second);

        let mut entities = HashSet::new();
        entities.insert(first);
        entities.insert(second);
        assert_eq!(entities.len(), 1);
    }

    #[test]
    fn equal_local_terms_in_different_vocabularies_are_distinct() {
        let bods = VocabularyTerm::new("bods:0.4", "ownership-or-control");
        let custom = VocabularyTerm::new("example:custom:v1", "ownership-or-control");

        assert_ne!(bods, custom);

        let mut terms = HashSet::new();
        terms.insert(bods);
        terms.insert(custom);
        assert_eq!(terms.len(), 2);
    }

    #[test]
    fn contradictory_statements_remain_distinct_records() {
        let lineage = StatementLineage::from_source(source());
        let subject = EntityRef::new("lei", "A");
        let object = EntityRef::new("lei", "B");
        let time = BitemporalScope::new(ValidityInterval::default(), 2_000);

        let first = SystemicStatement::Relationship(RelationshipStatement {
            statement_id: "stmt:1".into(),
            subject: subject.clone(),
            predicate: VocabularyTerm::new("example:test:v1", "accounting-parent"),
            object: object.clone(),
            classification: classification(),
            time: time.clone(),
            lineage: lineage.clone(),
            jurisdiction: None,
            scope: None,
        });
        let second = SystemicStatement::Relationship(RelationshipStatement {
            statement_id: "stmt:2".into(),
            subject,
            predicate: VocabularyTerm::new("example:test:v1", "not-accounting-parent"),
            object,
            classification: classification(),
            time,
            lineage: lineage.clone(),
            jurisdiction: None,
            scope: None,
        });
        let contradiction = StatementRelation {
            relation_id: "rel:1".into(),
            from_statement: first.statement_id().into(),
            relation: StatementRelationKind::Contradicts,
            to_statement: second.statement_id().into(),
            classification: classification(),
            lineage,
        };

        assert_ne!(first.statement_id(), second.statement_id());
        assert_eq!(contradiction.relation, StatementRelationKind::Contradicts);
    }

    #[test]
    fn possible_identity_is_not_same_identity() {
        assert_ne!(
            IdentityRelation::PossibleSameEntity,
            IdentityRelation::SameEntity
        );
    }
}
