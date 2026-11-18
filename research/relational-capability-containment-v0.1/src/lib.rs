//! Finite reference model for relational capability-set containment.
//!
//! This crate models complete effect tuples rather than independent allow-lists
//! for each tuple column. It is a research oracle for Mycelix #4852 / #6302 and
//! a cross-language comparison candidate for SOV-AI-AUTH-023 / PR #4867. It is
//! not an authority service or runtime policy engine.

use std::collections::BTreeSet;

/// The only schema version implemented by this reference model.
pub const SCHEMA_VERSION_V1: u16 = 1;

/// Domain separator for the canonical version-1 encoding.
pub const CANONICAL_DOMAIN_V1: &[u8] = b"mycelix.relational-effect-capability.v1\0";

/// Maximum UTF-8 byte length of one exact scope atom.
pub const MAX_SCOPE_ATOM_BYTES: usize = 256;

/// A categorical scope value.
///
/// Any is a parent-side wildcard. A child wildcard is contained only by a
/// parent wildcard; an exact child value may be contained by an exact equal
/// parent or a wildcard parent.
#[derive(Clone, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub enum ScopeValue {
    /// One exact, versioned identifier.
    Exact(String),
    /// All values in this schema-defined field domain.
    Any,
}

impl ScopeValue {
    /// Construct and validate an exact scope value.
    pub fn exact(value: impl Into<String>, label: &'static str) -> Result<Self, ScopeError> {
        Ok(Self::Exact(validate_atom(value.into(), label)?))
    }

    fn validate(self, label: &'static str) -> Result<Self, ScopeError> {
        match self {
            Self::Exact(value) => Self::exact(value, label),
            Self::Any => Ok(Self::Any),
        }
    }

    /// Construct the wildcard scope value.
    pub const fn any() -> Self {
        Self::Any
    }

    fn is_subsumed_by(&self, parent: &Self) -> bool {
        match (self, parent) {
            (_, Self::Any) => true,
            (Self::Exact(child), Self::Exact(parent)) => child == parent,
            (Self::Any, Self::Exact(_)) => false,
        }
    }

    fn intersection(&self, other: &Self) -> Option<Self> {
        match (self, other) {
            (Self::Any, Self::Any) => Some(Self::Any),
            (Self::Any, Self::Exact(value)) | (Self::Exact(value), Self::Any) => {
                Some(Self::Exact(value.clone()))
            }
            (Self::Exact(left), Self::Exact(right)) if left == right => {
                Some(Self::Exact(left.clone()))
            }
            (Self::Exact(_), Self::Exact(_)) => None,
        }
    }

    fn append_canonical(&self, out: &mut Vec<u8>) {
        match self {
            Self::Exact(value) => {
                out.push(0);
                push_atom(out, value);
            }
            Self::Any => out.push(1),
        }
    }
}

/// Direction in which information or control crosses an enforcement boundary.
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub enum Direction {
    /// Protected host information is disclosed to the delegate/audience.
    HostToDelegate,
    /// Delegate-originated data or control is applied to the host.
    DelegateToHost,
    /// The operation is local to the protected host.
    HostLocal,
}

impl Direction {
    const fn tag(self) -> u8 {
        match self {
            Self::HostToDelegate => 0,
            Self::DelegateToHost => 1,
            Self::HostLocal => 2,
        }
    }
}

/// Whether a tuple authorizes a proposal or an executable effect.
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub enum EffectMode {
    /// May prepare or propose the operation, but may not execute it.
    Propose,
    /// May execute the exact effect represented by the tuple.
    Execute,
}

impl EffectMode {
    const fn tag(self) -> u8 {
        match self {
            Self::Propose => 0,
            Self::Execute => 1,
        }
    }

    fn is_subsumed_by(self, parent: Self) -> bool {
        matches!(
            (self, parent),
            (Self::Propose, Self::Propose | Self::Execute)
                | (Self::Execute, Self::Execute)
        )
    }

    fn intersection(self, other: Self) -> Self {
        match (self, other) {
            (Self::Propose, _) | (_, Self::Propose) => Self::Propose,
            (Self::Execute, Self::Execute) => Self::Execute,
        }
    }
}

/// A quantitative upper bound with an explicit unit/dimension.
#[derive(Clone, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct QuantitativeLimit {
    /// A versioned unit such as bytes, operations, targets, or milliseconds.
    pub dimension: String,
    /// Maximum permitted quantity in the declared dimension.
    pub maximum: u64,
}

impl QuantitativeLimit {
    /// Construct a bounded quantity with a validated dimension label.
    pub fn new(dimension: impl Into<String>, maximum: u64) -> Result<Self, ScopeError> {
        Ok(Self {
            dimension: validate_atom(dimension.into(), "limit_dimension")?,
            maximum,
        })
    }

    fn is_subsumed_by(&self, parent: &Self) -> bool {
        self.dimension == parent.dimension && self.maximum <= parent.maximum
    }

    fn intersection(&self, other: &Self) -> Option<Self> {
        (self.dimension == other.dimension).then(|| Self {
            dimension: self.dimension.clone(),
            maximum: self.maximum.min(other.maximum),
        })
    }

    fn append_canonical(&self, out: &mut Vec<u8>) {
        push_atom(out, &self.dimension);
        out.extend_from_slice(&self.maximum.to_be_bytes());
    }
}

/// Input fields for one complete capability tuple.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct EffectTupleSpec {
    /// Stable action identifier.
    pub action: ScopeValue,
    /// Exact resource identifier or parent-side wildcard.
    pub resource: ScopeValue,
    /// Information/control direction.
    pub direction: Direction,
    /// Relying-party or audience identifier.
    pub audience: ScopeValue,
    /// Purpose identifier or owner-qualified purpose reference.
    pub purpose: ScopeValue,
    /// Data-class identifier.
    pub data_class: ScopeValue,
    /// Proposal or execution mode.
    pub mode: EffectMode,
    /// Immutable operation class, schema, or one-shot digest binding.
    pub operation_binding: ScopeValue,
    /// Quantitative ceiling with an explicit unit/dimension.
    pub limit: QuantitativeLimit,
}

/// A complete, indivisible capability tuple.
///
/// A permission is the tuple as a whole. Its columns must never be projected
/// into independent sets and recombined to manufacture new tuples.
#[derive(Clone, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct EffectTuple {
    action: ScopeValue,
    resource: ScopeValue,
    direction: Direction,
    audience: ScopeValue,
    purpose: ScopeValue,
    data_class: ScopeValue,
    mode: EffectMode,
    operation_binding: ScopeValue,
    limit: QuantitativeLimit,
}

impl EffectTuple {
    /// Construct one complete effect tuple, validating every exact scope atom.
    pub fn new(spec: EffectTupleSpec) -> Result<Self, ScopeError> {
        let limit = QuantitativeLimit::new(spec.limit.dimension, spec.limit.maximum)?;

        Ok(Self {
            action: spec.action.validate("action")?,
            resource: spec.resource.validate("resource")?,
            direction: spec.direction,
            audience: spec.audience.validate("audience")?,
            purpose: spec.purpose.validate("purpose")?,
            data_class: spec.data_class.validate("data_class")?,
            mode: spec.mode,
            operation_binding: spec.operation_binding.validate("operation_binding")?,
            limit,
        })
    }

    /// Whether this entire effect tuple is covered by one parent tuple.
    ///
    /// This is a relational witness check: all dimensions must be subsumed by
    /// the same parent tuple. Fields from separate parent tuples cannot be
    /// combined to create a new authorization.
    pub fn is_subsumed_by(&self, parent: &Self) -> bool {
        self.action.is_subsumed_by(&parent.action)
            && self.resource.is_subsumed_by(&parent.resource)
            && self.direction == parent.direction
            && self.audience.is_subsumed_by(&parent.audience)
            && self.purpose.is_subsumed_by(&parent.purpose)
            && self.data_class.is_subsumed_by(&parent.data_class)
            && self.mode.is_subsumed_by(parent.mode)
            && self
                .operation_binding
                .is_subsumed_by(&parent.operation_binding)
            && self.limit.is_subsumed_by(&parent.limit)
    }

    fn intersection(&self, other: &Self) -> Option<Self> {
        Some(Self {
            action: self.action.intersection(&other.action)?,
            resource: self.resource.intersection(&other.resource)?,
            direction: (self.direction == other.direction).then_some(self.direction)?,
            audience: self.audience.intersection(&other.audience)?,
            purpose: self.purpose.intersection(&other.purpose)?,
            data_class: self.data_class.intersection(&other.data_class)?,
            mode: self.mode.intersection(other.mode),
            operation_binding: self
                .operation_binding
                .intersection(&other.operation_binding)?,
            limit: self.limit.intersection(&other.limit)?,
        })
    }

    fn append_canonical(&self, out: &mut Vec<u8>) {
        self.action.append_canonical(out);
        self.resource.append_canonical(out);
        out.push(self.direction.tag());
        self.audience.append_canonical(out);
        self.purpose.append_canonical(out);
        self.data_class.append_canonical(out);
        out.push(self.mode.tag());
        self.operation_binding.append_canonical(out);
        self.limit.append_canonical(out);
    }
}

/// A versioned finite relation of complete effect tuples.
///
/// This is a set of complete tuples. It is not equivalent to a product of
/// action/resource/audience/purpose projections.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CapabilitySetV1 {
    tuples: BTreeSet<EffectTuple>,
}

impl CapabilitySetV1 {
    /// Build a versioned relation.
    ///
    /// Duplicate syntactic tuples are rejected instead of being silently
    /// normalized. An empty relation is valid and represents a deny-all scope.
    pub fn new(schema_version: u16, tuples: Vec<EffectTuple>) -> Result<Self, ScopeError> {
        if schema_version != SCHEMA_VERSION_V1 {
            return Err(ScopeError::UnsupportedSchemaVersion(schema_version));
        }

        let mut relation = BTreeSet::new();
        for tuple in tuples {
            if !relation.insert(tuple) {
                return Err(ScopeError::DuplicateTuple);
            }
        }
        Ok(Self { tuples: relation })
    }

    /// Construct a version-1 relation without repeating the schema number.
    pub fn v1(tuples: Vec<EffectTuple>) -> Result<Self, ScopeError> {
        Self::new(SCHEMA_VERSION_V1, tuples)
    }

    /// Number of complete effect tuples in this relation.
    pub fn len(&self) -> usize {
        self.tuples.len()
    }

    /// Whether this relation contains no authorized effect tuples.
    pub fn is_empty(&self) -> bool {
        self.tuples.is_empty()
    }

    /// Whether the exact complete effect tuple is a member of this relation.
    pub fn allows(&self, effect: &EffectTuple) -> bool {
        self.tuples.iter().any(|grant| effect.is_subsumed_by(grant))
    }

    /// Whether every tuple in self has a complete parent witness in parent.
    ///
    /// This is exact relational inclusion with the declared wildcard and
    /// numeric-bound subsumption semantics.
    pub fn is_subset_of(&self, parent: &Self) -> bool {
        self.tuples
            .iter()
            .all(|child| parent.tuples.iter().any(|grant| child.is_subsumed_by(grant)))
    }

    /// Whether this relation is a proper semantic subset of parent.
    ///
    /// Strict attenuation requires inclusion in one direction and failure of
    /// inclusion in the reverse direction. Syntactic inequality alone is not
    /// sufficient when wildcards or different tuple encodings are present.
    pub fn is_strict_subset_of(&self, parent: &Self) -> bool {
        self.is_subset_of(parent) && !parent.is_subset_of(self)
    }

    /// Intersection for two authority scopes that are both required.
    ///
    /// Intersections are computed tuple-by-tuple. Keep source grants and their
    /// lineage separately; this derived relation is not a revocation index.
    pub fn required_intersection(&self, other: &Self) -> Self {
        let tuples = self
            .tuples
            .iter()
            .flat_map(|left| {
                other
                    .tuples
                    .iter()
                    .filter_map(|right| left.intersection(right))
            })
            .collect();
        Self { tuples }
    }

    /// Union for independent alternative grants.
    ///
    /// This is not valid for parent/child attenuation or conjunctive bundles.
    /// Keep each source grant and lineage separately; the union alone cannot
    /// explain which grant supports a particular tuple.
    pub fn independent_union(&self, other: &Self) -> Self {
        Self {
            tuples: self.tuples.union(&other.tuples).cloned().collect(),
        }
    }

    /// Return canonical bytes for the exact relation under schema version 1.
    ///
    /// Tuples are encoded in syntactic sorted order. This defines a deterministic
    /// encoding, not a unique semantic normal form: wildcard and alias
    /// normalization equivalence are deliberately outside this model.
    pub fn canonical_bytes(&self) -> Vec<u8> {
        let mut out = Vec::new();
        out.extend_from_slice(CANONICAL_DOMAIN_V1);
        out.extend_from_slice(&SCHEMA_VERSION_V1.to_be_bytes());
        out.extend_from_slice(&(self.tuples.len() as u64).to_be_bytes());
        for tuple in &self.tuples {
            tuple.append_canonical(&mut out);
        }
        out
    }
}

/// Errors returned by the finite relational-scope model.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ScopeError {
    /// A schema version is not implemented by this oracle.
    UnsupportedSchemaVersion(u16),
    /// A required scope atom is empty or contains only whitespace.
    EmptyAtom(&'static str),
    /// A scope atom exceeds the byte bound.
    AtomTooLong(&'static str),
    /// A scope atom contains a control character.
    ControlCharacter(&'static str),
    /// A relation contains the same complete syntactic tuple more than once.
    DuplicateTuple,
}

fn validate_atom(value: String, label: &'static str) -> Result<String, ScopeError> {
    if value.trim().is_empty() {
        return Err(ScopeError::EmptyAtom(label));
    }
    if value.len() > MAX_SCOPE_ATOM_BYTES {
        return Err(ScopeError::AtomTooLong(label));
    }
    if value.chars().any(char::is_control) {
        return Err(ScopeError::ControlCharacter(label));
    }
    Ok(value)
}

fn push_atom(out: &mut Vec<u8>, value: &str) {
    // Atoms are capped at MAX_SCOPE_ATOM_BYTES, so the u32 conversion is safe.
    out.extend_from_slice(&(value.len() as u32).to_be_bytes());
    out.extend_from_slice(value.as_bytes());
}

#[cfg(test)]
mod tests {
    use super::*;

    fn exact(value: &str) -> ScopeValue {
        ScopeValue::exact(value, "test").unwrap()
    }

    fn effect(
        action: ScopeValue,
        resource: ScopeValue,
        direction: Direction,
        audience: ScopeValue,
        purpose: ScopeValue,
        data_class: ScopeValue,
        mode: EffectMode,
        operation_binding: ScopeValue,
        limit: QuantitativeLimit,
    ) -> EffectTuple {
        EffectTuple::new(EffectTupleSpec {
            action,
            resource,
            direction,
            audience,
            purpose,
            data_class,
            mode,
            operation_binding,
            limit,
        })
        .unwrap()
    }

    fn screen_a_limit(maximum: u64) -> EffectTuple {
        effect(
            exact("xenia.screen.view"),
            exact("host-A"),
            Direction::HostToDelegate,
            exact("viewer-A"),
            exact("remote-support"),
            exact("screen-frame"),
            EffectMode::Execute,
            exact("screen-frame/v1"),
            QuantitativeLimit::new("frames", maximum).unwrap(),
        )
    }

    fn file_b_limit(maximum: u64) -> EffectTuple {
        effect(
            exact("xenia.file.receive"),
            exact("host-B"),
            Direction::DelegateToHost,
            exact("viewer-B"),
            exact("file-delivery"),
            exact("file-bytes"),
            EffectMode::Execute,
            exact("file-transfer/v1"),
            QuantitativeLimit::new("bytes", maximum).unwrap(),
        )
    }

    #[test]
    fn exact_complete_tuple_is_required() {
        let allowed = CapabilitySetV1::v1(vec![screen_a_limit(10)]).unwrap();
        assert!(allowed.allows(&screen_a_limit(10)));

        let changed_direction = effect(
            exact("xenia.screen.view"),
            exact("host-A"),
            Direction::DelegateToHost,
            exact("viewer-A"),
            exact("remote-support"),
            exact("screen-frame"),
            EffectMode::Execute,
            exact("screen-frame/v1"),
            QuantitativeLimit::new("frames", 10).unwrap(),
        );
        let changed_resource = effect(
            exact("xenia.screen.view"),
            exact("host-B"),
            Direction::HostToDelegate,
            exact("viewer-A"),
            exact("remote-support"),
            exact("screen-frame"),
            EffectMode::Execute,
            exact("screen-frame/v1"),
            QuantitativeLimit::new("frames", 10).unwrap(),
        );
        let changed_audience = effect(
            exact("xenia.screen.view"),
            exact("host-A"),
            Direction::HostToDelegate,
            exact("viewer-B"),
            exact("remote-support"),
            exact("screen-frame"),
            EffectMode::Execute,
            exact("screen-frame/v1"),
            QuantitativeLimit::new("frames", 10).unwrap(),
        );
        let changed_purpose = effect(
            exact("xenia.screen.view"),
            exact("host-A"),
            Direction::HostToDelegate,
            exact("viewer-A"),
            exact("file-delivery"),
            exact("screen-frame"),
            EffectMode::Execute,
            exact("screen-frame/v1"),
            QuantitativeLimit::new("frames", 10).unwrap(),
        );
        let changed_class = effect(
            exact("xenia.screen.view"),
            exact("host-A"),
            Direction::HostToDelegate,
            exact("viewer-A"),
            exact("remote-support"),
            exact("file-bytes"),
            EffectMode::Execute,
            exact("screen-frame/v1"),
            QuantitativeLimit::new("frames", 10).unwrap(),
        );
        let changed_operation = effect(
            exact("xenia.screen.view"),
            exact("host-A"),
            Direction::HostToDelegate,
            exact("viewer-A"),
            exact("remote-support"),
            exact("screen-frame"),
            EffectMode::Execute,
            exact("different-operation/v1"),
            QuantitativeLimit::new("frames", 10).unwrap(),
        );

        for mutation in [
            changed_direction,
            changed_resource,
            changed_audience,
            changed_purpose,
            changed_class,
            changed_operation,
        ] {
            assert!(
                !allowed.allows(&mutation),
                "one-column mutation must not match a complete tuple"
            );
        }
    }

    #[test]
    fn marginal_projection_cartesian_product_is_a_false_positive() {
        let parent = CapabilitySetV1::v1(vec![screen_a_limit(100), file_b_limit(200)]).unwrap();

        // Every field in this Frankenstein tuple appears somewhere in the
        // parent relation, but the complete tuple never appeared in it.
        let unauthorized_recombination = effect(
            exact("xenia.screen.view"),
            exact("host-B"),
            Direction::HostToDelegate,
            exact("viewer-B"),
            exact("file-delivery"),
            exact("file-bytes"),
            EffectMode::Execute,
            exact("screen-frame/v1"),
            QuantitativeLimit::new("frames", 50).unwrap(),
        );

        let marginal_match = parent
            .tuples
            .iter()
            .any(|item| item.action == unauthorized_recombination.action)
            && parent
                .tuples
                .iter()
                .any(|item| item.resource == unauthorized_recombination.resource)
            && parent
                .tuples
                .iter()
                .any(|item| item.direction == unauthorized_recombination.direction)
            && parent
                .tuples
                .iter()
                .any(|item| item.audience == unauthorized_recombination.audience)
            && parent
                .tuples
                .iter()
                .any(|item| item.purpose == unauthorized_recombination.purpose)
            && parent
                .tuples
                .iter()
                .any(|item| item.data_class == unauthorized_recombination.data_class)
            && parent
                .tuples
                .iter()
                .any(|item| item.mode == unauthorized_recombination.mode)
            && parent
                .tuples
                .iter()
                .any(|item| item.operation_binding == unauthorized_recombination.operation_binding)
            && parent
                .tuples
                .iter()
                .any(|item| item.limit.dimension == unauthorized_recombination.limit.dimension);

        assert!(
            marginal_match,
            "each projection independently appears in the parent"
        );
        assert!(
            !parent.allows(&unauthorized_recombination),
            "relational membership must reject the unauthorized cross-product"
        );
    }

    #[test]
    fn wildcard_is_parent_subsumption_not_child_expansion() {
        let broad_parent = effect(
            exact("xenia.screen.view"),
            ScopeValue::Any,
            Direction::HostToDelegate,
            ScopeValue::Any,
            exact("remote-support"),
            exact("screen-frame"),
            EffectMode::Execute,
            exact("screen-frame/v1"),
            QuantitativeLimit::new("frames", 100).unwrap(),
        );
        let narrow_child = screen_a_limit(20);
        let broad_child = effect(
            exact("xenia.screen.view"),
            ScopeValue::Any,
            Direction::HostToDelegate,
            ScopeValue::Any,
            exact("remote-support"),
            exact("screen-frame"),
            EffectMode::Execute,
            exact("screen-frame/v1"),
            QuantitativeLimit::new("frames", 20).unwrap(),
        );

        assert!(narrow_child.is_subsumed_by(&broad_parent));
        assert!(!broad_child.is_subsumed_by(&narrow_child));

        let broad_parent_set = CapabilitySetV1::v1(vec![broad_parent]).unwrap();
        let narrow_child_set = CapabilitySetV1::v1(vec![narrow_child]).unwrap();
        let broad_child_set = CapabilitySetV1::v1(vec![broad_child]).unwrap();
        assert!(narrow_child_set.is_subset_of(&broad_parent_set));
        assert!(narrow_child_set.is_strict_subset_of(&broad_parent_set));
        assert!(!broad_child_set.is_subset_of(&narrow_child_set));
    }

    #[test]
    fn quantitative_bounds_attenuate_only_with_matching_dimension() {
        let parent = CapabilitySetV1::v1(vec![screen_a_limit(100)]).unwrap();
        let child = CapabilitySetV1::v1(vec![screen_a_limit(50)]).unwrap();
        let wider = CapabilitySetV1::v1(vec![screen_a_limit(101)]).unwrap();

        assert!(child.is_subset_of(&parent));
        assert!(!wider.is_subset_of(&parent));

        let wrong_unit = effect(
            exact("xenia.screen.view"),
            exact("host-A"),
            Direction::HostToDelegate,
            exact("viewer-A"),
            exact("remote-support"),
            exact("screen-frame"),
            EffectMode::Execute,
            exact("screen-frame/v1"),
            QuantitativeLimit::new("bytes", 50).unwrap(),
        );
        assert!(!CapabilitySetV1::v1(vec![wrong_unit]).unwrap().is_subset_of(&parent));
    }

    #[test]
    fn proposal_is_attenuated_by_execute_but_execute_not_by_propose() {
        let execute = effect(
            exact("xenia.screen.view"),
            exact("host-A"),
            Direction::HostToDelegate,
            exact("viewer-A"),
            exact("remote-support"),
            exact("screen-frame"),
            EffectMode::Execute,
            exact("screen-frame/v1"),
            QuantitativeLimit::new("frames", 100).unwrap(),
        );
        let propose = effect(
            exact("xenia.screen.view"),
            exact("host-A"),
            Direction::HostToDelegate,
            exact("viewer-A"),
            exact("remote-support"),
            exact("screen-frame"),
            EffectMode::Propose,
            exact("screen-frame/v1"),
            QuantitativeLimit::new("frames", 100).unwrap(),
        );

        assert!(propose.is_subsumed_by(&execute));
        assert!(!execute.is_subsumed_by(&propose));
    }

    #[test]
    fn strict_attenuation_is_semantic_not_syntactic() {
        let broad = CapabilitySetV1::v1(vec![screen_a_limit(100)]).unwrap();
        let narrow = CapabilitySetV1::v1(vec![screen_a_limit(50)]).unwrap();
        let semantically_equal = CapabilitySetV1::v1(vec![screen_a_limit(100)]).unwrap();

        assert!(narrow.is_subset_of(&broad));
        assert!(narrow.is_strict_subset_of(&broad));
        assert!(!semantically_equal.is_strict_subset_of(&broad));
    }

    #[test]
    fn required_bundle_intersects_but_independent_grants_union() {
        let left = CapabilitySetV1::v1(vec![screen_a_limit(100), file_b_limit(200)]).unwrap();
        let right = CapabilitySetV1::v1(vec![screen_a_limit(50), file_b_limit(300)]).unwrap();

        let required = left.required_intersection(&right);
        assert_eq!(required.len(), 2);
        assert!(required.allows(&screen_a_limit(50)));
        assert!(required.allows(&file_b_limit(200)));

        let alternatives = left.independent_union(&right);
        assert!(alternatives.allows(&screen_a_limit(100)));
        assert!(alternatives.allows(&file_b_limit(300)));
    }

    #[test]
    fn canonical_encoding_is_order_independent_and_domain_separated() {
        let forward = CapabilitySetV1::v1(vec![screen_a_limit(100), file_b_limit(200)]).unwrap();
        let reverse = CapabilitySetV1::v1(vec![file_b_limit(200), screen_a_limit(100)]).unwrap();

        assert_eq!(forward.canonical_bytes(), reverse.canonical_bytes());
        assert!(forward.canonical_bytes().starts_with(CANONICAL_DOMAIN_V1));
        assert_eq!(
            &forward.canonical_bytes()[CANONICAL_DOMAIN_V1.len()..CANONICAL_DOMAIN_V1.len() + 2],
            &SCHEMA_VERSION_V1.to_be_bytes()
        );
    }

    #[test]
    fn duplicate_tuples_and_unknown_schema_versions_fail_closed() {
        assert_eq!(
            CapabilitySetV1::v1(vec![screen_a_limit(100), screen_a_limit(100)]),
            Err(ScopeError::DuplicateTuple)
        );
        assert_eq!(
            CapabilitySetV1::new(99, vec![screen_a_limit(100)]),
            Err(ScopeError::UnsupportedSchemaVersion(99))
        );
    }

    #[test]
    fn malformed_scope_atoms_fail_closed() {
        assert_eq!(
            ScopeValue::exact(" ", "action"),
            Err(ScopeError::EmptyAtom("action"))
        );
        assert_eq!(
            ScopeValue::exact("bad\nresource", "resource"),
            Err(ScopeError::ControlCharacter("resource"))
        );
        assert_eq!(
            ScopeValue::exact("x".repeat(MAX_SCOPE_ATOM_BYTES + 1), "action"),
            Err(ScopeError::AtomTooLong("action"))
        );
        assert_eq!(
            QuantitativeLimit::new("", 0),
            Err(ScopeError::EmptyAtom("limit_dimension"))
        );
        assert_eq!(
            EffectTuple::new(EffectTupleSpec {
                action: exact("xenia.screen.view"),
                resource: exact("host-A"),
                direction: Direction::HostToDelegate,
                audience: exact("viewer-A"),
                purpose: exact("remote-support"),
                data_class: exact("screen-frame"),
                mode: EffectMode::Execute,
                operation_binding: exact("screen-frame/v1"),
                limit: QuantitativeLimit {
                    dimension: String::new(),
                    maximum: 100,
                },
            }),
            Err(ScopeError::EmptyAtom("limit_dimension"))
        );
    }

    #[test]
    fn empty_relation_is_deny_all() {
        let deny_all = CapabilitySetV1::v1(Vec::new()).unwrap();
        assert!(deny_all.is_empty());
        assert!(!deny_all.allows(&screen_a_limit(1)));
        assert!(deny_all.is_subset_of(&CapabilitySetV1::v1(vec![screen_a_limit(1)]).unwrap()));
    }

    #[test]
    fn all_256_by_256_eight_tuple_relations_obey_subset_and_composition_laws() {
        let universe = (0..8)
            .map(|index| {
                effect(
                    exact(&format!("action-{index}")),
                    exact(&format!("resource-{index}")),
                    if index % 2 == 0 {
                        Direction::HostToDelegate
                    } else {
                        Direction::DelegateToHost
                    },
                    exact(&format!("audience-{index}")),
                    exact(&format!("purpose-{index}")),
                    exact(&format!("class-{index}")),
                    EffectMode::Execute,
                    exact(&format!("operation-{index}/v1")),
                    QuantitativeLimit::new("operations", 100).unwrap(),
                )
            })
            .collect::<Vec<_>>();

        let from_mask = |mask: u8| {
            CapabilitySetV1::v1(
                universe
                    .iter()
                    .enumerate()
                    .filter(|(index, _)| mask & (1 << index) != 0)
                    .map(|(_, tuple)| tuple.clone())
                    .collect(),
            )
            .unwrap()
        };

        for child_mask in 0u16..=u8::MAX as u16 {
            for parent_mask in 0u16..=u8::MAX as u16 {
                let child = from_mask(child_mask as u8);
                let parent = from_mask(parent_mask as u8);
                let subset = child_mask & !parent_mask == 0;

                assert_eq!(
                    child.is_subset_of(&parent),
                    subset,
                    "child={child_mask:08b}, parent={parent_mask:08b}"
                );
                assert_eq!(
                    child.is_strict_subset_of(&parent),
                    subset && child_mask != parent_mask,
                    "strict child={child_mask:08b}, parent={parent_mask:08b}"
                );

                let intersection = child.required_intersection(&parent);
                assert!(intersection.is_subset_of(&child));
                assert!(intersection.is_subset_of(&parent));

                let union = child.independent_union(&parent);
                assert!(child.is_subset_of(&union));
                assert!(parent.is_subset_of(&union));
            }
        }
    }
}
