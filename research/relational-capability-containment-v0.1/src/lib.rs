//! Finite reference model for relational capability-set containment.
//!
//! This crate deliberately models complete effect tuples, not independent
//! allow-lists for each tuple column. It is a research oracle for Mycelix
//! #4852 / #6302 and is not an authority service or runtime policy engine.

use std::collections::BTreeSet;

/// The only schema version implemented by this reference model.
pub const SCHEMA_VERSION_V1: u16 = 1;

/// Domain separator for the canonical version-1 encoding.
pub const CANONICAL_DOMAIN_V1: &[u8] = b"mycelix.relational-effect-capability.v1\0";

/// Maximum UTF-8 byte length of one scope atom.
pub const MAX_SCOPE_ATOM_BYTES: usize = 256;

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
}

/// Input fields for one complete capability tuple.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct EffectTupleSpec {
    /// Stable action identifier.
    pub action: String,
    /// Exact resource identifier.
    pub resource: String,
    /// Information/control direction.
    pub direction: Direction,
    /// Relying-party or audience identifier.
    pub audience: String,
    /// Purpose identifier or owner-qualified purpose reference.
    pub purpose: String,
    /// Data-class identifier.
    pub data_class: String,
    /// Proposal or execution mode.
    pub mode: EffectMode,
    /// Immutable operation class, schema, or one-shot digest binding.
    pub operation_binding: String,
}

/// A complete, indivisible capability tuple.
///
/// A permission is the tuple as a whole. Its columns must never be projected
/// into independent sets and recombined to manufacture new tuples.
#[derive(Clone, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct EffectTuple {
    action: String,
    resource: String,
    direction: Direction,
    audience: String,
    purpose: String,
    data_class: String,
    mode: EffectMode,
    operation_binding: String,
}

impl EffectTuple {
    /// Construct one complete effect tuple, validating every string atom.
    pub fn new(spec: EffectTupleSpec) -> Result<Self, ScopeError> {
        let action = validate_atom(spec.action, "action")?;
        let resource = validate_atom(spec.resource, "resource")?;
        let audience = validate_atom(spec.audience, "audience")?;
        let purpose = validate_atom(spec.purpose, "purpose")?;
        let data_class = validate_atom(spec.data_class, "data_class")?;
        let operation_binding = validate_atom(spec.operation_binding, "operation_binding")?;

        Ok(Self {
            action,
            resource,
            direction: spec.direction,
            audience,
            purpose,
            data_class,
            mode: spec.mode,
            operation_binding,
        })
    }

    /// Return the stable action identifier.
    pub fn action(&self) -> &str {
        &self.action
    }

    /// Return the exact resource identifier.
    pub fn resource(&self) -> &str {
        &self.resource
    }

    /// Return the information/control direction.
    pub const fn direction(&self) -> Direction {
        self.direction
    }

    /// Return the relying-party or audience identifier.
    pub fn audience(&self) -> &str {
        &self.audience
    }

    /// Return the purpose identifier or owner-qualified purpose reference.
    pub fn purpose(&self) -> &str {
        &self.purpose
    }

    /// Return the data-class identifier.
    pub fn data_class(&self) -> &str {
        &self.data_class
    }

    /// Return the proposal or execution mode.
    pub const fn mode(&self) -> EffectMode {
        self.mode
    }

    /// Return the immutable operation class, schema, or one-shot digest binding.
    pub fn operation_binding(&self) -> &str {
        &self.operation_binding
    }

    fn append_canonical(&self, out: &mut Vec<u8>) {
        push_atom(out, &self.action);
        push_atom(out, &self.resource);
        out.push(self.direction.tag());
        push_atom(out, &self.audience);
        push_atom(out, &self.purpose);
        push_atom(out, &self.data_class);
        out.push(self.mode.tag());
        push_atom(out, &self.operation_binding);
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
    /// Duplicate tuples are rejected instead of being silently normalized.
    /// An empty relation is valid and represents a deny-all scope.
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
        self.tuples.contains(effect)
    }

    /// Whether every tuple in self is present in parent.
    ///
    /// This is exact relational inclusion. It does not compare the marginal
    /// projections of individual tuple columns.
    pub fn is_subset_of(&self, parent: &Self) -> bool {
        self.tuples.is_subset(&parent.tuples)
    }

    /// Whether this relation is a proper subset of parent.
    ///
    /// Strict attenuation requires non-widening plus at least one removed
    /// complete tuple. Equality is not attenuation.
    pub fn is_strict_subset_of(&self, parent: &Self) -> bool {
        self.is_subset_of(parent) && self.len() < parent.len()
    }

    /// Intersection for two authority scopes that are both required.
    ///
    /// Keep the source grants and their lineage separately for revocation and
    /// provenance accounting; this derived relation is not a lineage index.
    pub fn required_intersection(&self, other: &Self) -> Self {
        Self {
            tuples: self.tuples.intersection(&other.tuples).cloned().collect(),
        }
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
    /// Tuples are encoded in sorted order. This defines a deterministic byte
    /// representation; it does not sign, hash, authenticate, or qualify the
    /// resulting bytes.
    pub fn canonical_bytes(&self) -> Vec<u8> {
        let mut out = Vec::new();
        out.extend_from_slice(CANONICAL_DOMAIN_V1);
        out.extend_from_slice(&SCHEMA_VERSION_V1.to_be_bytes());
        out.extend_from_slice(&(self.tuples.len() as u32).to_be_bytes());
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
    /// A relation contains the same complete tuple more than once.
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

    fn effect(spec: EffectTupleSpec) -> EffectTuple {
        EffectTuple::new(spec).unwrap()
    }

    fn screen_a() -> EffectTuple {
        effect(EffectTupleSpec {
            action: "xenia.screen.view".to_string(),
            resource: "host-A".to_string(),
            direction: Direction::HostToDelegate,
            audience: "viewer-A".to_string(),
            purpose: "remote-support".to_string(),
            data_class: "screen-frame".to_string(),
            mode: EffectMode::Execute,
            operation_binding: "screen-frame/v1".to_string(),
        })
    }

    fn file_b() -> EffectTuple {
        effect(EffectTupleSpec {
            action: "xenia.file.receive".to_string(),
            resource: "host-B".to_string(),
            direction: Direction::DelegateToHost,
            audience: "viewer-B".to_string(),
            purpose: "file-delivery".to_string(),
            data_class: "file-bytes".to_string(),
            mode: EffectMode::Execute,
            operation_binding: "file-transfer/v1".to_string(),
        })
    }

    #[test]
    fn exact_complete_tuple_is_required() {
        let allowed = CapabilitySetV1::v1(vec![screen_a()]).unwrap();
        assert!(allowed.allows(&screen_a()));

        let mutations = [
            effect(EffectTupleSpec {
                action: "xenia.screen.view".to_string(),
                resource: "host-A".to_string(),
                direction: Direction::DelegateToHost,
                audience: "viewer-A".to_string(),
                purpose: "remote-support".to_string(),
                data_class: "screen-frame".to_string(),
                mode: EffectMode::Execute,
                operation_binding: "screen-frame/v1".to_string(),
            }),
            effect(EffectTupleSpec {
                action: "xenia.screen.view".to_string(),
                resource: "host-B".to_string(),
                direction: Direction::HostToDelegate,
                audience: "viewer-A".to_string(),
                purpose: "remote-support".to_string(),
                data_class: "screen-frame".to_string(),
                mode: EffectMode::Execute,
                operation_binding: "screen-frame/v1".to_string(),
            }),
            effect(EffectTupleSpec {
                action: "xenia.screen.view".to_string(),
                resource: "host-A".to_string(),
                direction: Direction::HostToDelegate,
                audience: "viewer-B".to_string(),
                purpose: "remote-support".to_string(),
                data_class: "screen-frame".to_string(),
                mode: EffectMode::Execute,
                operation_binding: "screen-frame/v1".to_string(),
            }),
            effect(EffectTupleSpec {
                action: "xenia.screen.view".to_string(),
                resource: "host-A".to_string(),
                direction: Direction::HostToDelegate,
                audience: "viewer-A".to_string(),
                purpose: "file-delivery".to_string(),
                data_class: "screen-frame".to_string(),
                mode: EffectMode::Execute,
                operation_binding: "screen-frame/v1".to_string(),
            }),
            effect(EffectTupleSpec {
                action: "xenia.screen.view".to_string(),
                resource: "host-A".to_string(),
                direction: Direction::HostToDelegate,
                audience: "viewer-A".to_string(),
                purpose: "remote-support".to_string(),
                data_class: "file-bytes".to_string(),
                mode: EffectMode::Execute,
                operation_binding: "screen-frame/v1".to_string(),
            }),
            effect(EffectTupleSpec {
                action: "xenia.screen.view".to_string(),
                resource: "host-A".to_string(),
                direction: Direction::HostToDelegate,
                audience: "viewer-A".to_string(),
                purpose: "remote-support".to_string(),
                data_class: "screen-frame".to_string(),
                mode: EffectMode::Execute,
                operation_binding: "different-operation/v1".to_string(),
            }),
        ];

        for mutation in mutations {
            assert!(
                !allowed.allows(&mutation),
                "one-column mutation must not match a complete tuple"
            );
        }
    }

    #[test]
    fn marginal_projection_cartesian_product_is_a_false_positive() {
        let parent = CapabilitySetV1::v1(vec![screen_a(), file_b()]).unwrap();

        // Every field in this Frankenstein tuple appears somewhere in the
        // parent relation, but the complete tuple never appeared in it.
        let unauthorized_recombination = effect(EffectTupleSpec {
            action: "xenia.screen.view".to_string(),
            resource: "host-B".to_string(),
            direction: Direction::HostToDelegate,
            audience: "viewer-B".to_string(),
            purpose: "file-delivery".to_string(),
            data_class: "file-bytes".to_string(),
            mode: EffectMode::Execute,
            operation_binding: "screen-frame/v1".to_string(),
        });

        let marginal_match = parent.tuples.iter().any(|item| item.action == unauthorized_recombination.action)
            && parent.tuples.iter().any(|item| item.resource == unauthorized_recombination.resource)
            && parent.tuples.iter().any(|item| item.direction == unauthorized_recombination.direction)
            && parent.tuples.iter().any(|item| item.audience == unauthorized_recombination.audience)
            && parent.tuples.iter().any(|item| item.purpose == unauthorized_recombination.purpose)
            && parent.tuples.iter().any(|item| item.data_class == unauthorized_recombination.data_class)
            && parent.tuples.iter().any(|item| item.mode == unauthorized_recombination.mode)
            && parent.tuples.iter().any(|item| item.operation_binding == unauthorized_recombination.operation_binding);

        assert!(marginal_match, "each projection independently appears in the parent");
        assert!(
            !parent.allows(&unauthorized_recombination),
            "relational membership must reject the unauthorized cross-product"
        );
    }

    #[test]
    fn strict_attenuation_is_complete_tuple_set_inclusion() {
        let parent = CapabilitySetV1::v1(vec![screen_a(), file_b()]).unwrap();
        let attenuated = CapabilitySetV1::v1(vec![screen_a()]).unwrap();
        let equal = CapabilitySetV1::v1(vec![screen_a(), file_b()]).unwrap();

        assert!(attenuated.is_subset_of(&parent));
        assert!(attenuated.is_strict_subset_of(&parent));
        assert!(equal.is_subset_of(&parent));
        assert!(!equal.is_strict_subset_of(&parent));

        let widened = CapabilitySetV1::v1(vec![
            screen_a(),
            file_b(),
            effect(EffectTupleSpec {
                action: "xenia.clipboard.read".to_string(),
                resource: "host-A".to_string(),
                direction: Direction::HostToDelegate,
                audience: "viewer-A".to_string(),
                purpose: "remote-support".to_string(),
                data_class: "clipboard".to_string(),
                mode: EffectMode::Execute,
                operation_binding: "clipboard-read/v1".to_string(),
            }),
        ])
        .unwrap();
        assert!(!widened.is_subset_of(&parent));
        assert!(!widened.is_strict_subset_of(&parent));
    }

    #[test]
    fn required_bundle_intersects_but_independent_grants_union() {
        let left = CapabilitySetV1::v1(vec![screen_a(), file_b()]).unwrap();
        let right = CapabilitySetV1::v1(vec![file_b()]).unwrap();

        let required = left.required_intersection(&right);
        assert_eq!(required.len(), 1);
        assert!(required.allows(&file_b()));
        assert!(!required.allows(&screen_a()));

        let alternatives = left.independent_union(&right);
        assert_eq!(alternatives.len(), 2);
        assert!(alternatives.allows(&screen_a()));
        assert!(alternatives.allows(&file_b()));
    }

    #[test]
    fn canonical_encoding_is_order_independent_and_domain_separated() {
        let forward = CapabilitySetV1::v1(vec![screen_a(), file_b()]).unwrap();
        let reverse = CapabilitySetV1::v1(vec![file_b(), screen_a()]).unwrap();

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
            CapabilitySetV1::v1(vec![screen_a(), screen_a()]),
            Err(ScopeError::DuplicateTuple)
        );
        assert_eq!(
            CapabilitySetV1::new(99, vec![screen_a()]),
            Err(ScopeError::UnsupportedSchemaVersion(99))
        );
    }

    #[test]
    fn malformed_scope_atoms_fail_closed() {
        assert_eq!(
            EffectTuple::new(EffectTupleSpec {
                action: " ".to_string(),
                resource: "host-A".to_string(),
                direction: Direction::HostToDelegate,
                audience: "viewer-A".to_string(),
                purpose: "remote-support".to_string(),
                data_class: "screen-frame".to_string(),
                mode: EffectMode::Execute,
                operation_binding: "screen-frame/v1".to_string(),
            }),
            Err(ScopeError::EmptyAtom("action"))
        );
        assert_eq!(
            EffectTuple::new(EffectTupleSpec {
                action: "xenia.screen.view".to_string(),
                resource: "host-A".to_string(),
                direction: Direction::HostToDelegate,
                audience: "viewer-A".to_string(),
                purpose: "remote-support".to_string(),
                data_class: "screen-frame".to_string(),
                mode: EffectMode::Execute,
                operation_binding: "bad\noperation".to_string(),
            }),
            Err(ScopeError::ControlCharacter("operation_binding"))
        );
        assert_eq!(
            EffectTuple::new(EffectTupleSpec {
                action: "x".repeat(MAX_SCOPE_ATOM_BYTES + 1),
                resource: "host-A".to_string(),
                direction: Direction::HostToDelegate,
                audience: "viewer-A".to_string(),
                purpose: "remote-support".to_string(),
                data_class: "screen-frame".to_string(),
                mode: EffectMode::Execute,
                operation_binding: "screen-frame/v1".to_string(),
            }),
            Err(ScopeError::AtomTooLong("action"))
        );
    }

    #[test]
    fn empty_relation_is_deny_all() {
        let deny_all = CapabilitySetV1::v1(Vec::new()).unwrap();
        assert!(deny_all.is_empty());
        assert!(!deny_all.allows(&screen_a()));
        assert!(deny_all.is_subset_of(&CapabilitySetV1::v1(vec![screen_a()]).unwrap()));
    }

    #[test]
    fn all_256_by_256_eight_tuple_relations_obey_subset_and_composition_laws() {
        let universe = (0..8)
            .map(|index| {
                effect(EffectTupleSpec {
                    action: format!("action-{index}"),
                    resource: format!("resource-{index}"),
                    direction: if index % 2 == 0 {
                        Direction::HostToDelegate
                    } else {
                        Direction::DelegateToHost
                    },
                    audience: format!("audience-{index}"),
                    purpose: format!("purpose-{index}"),
                    data_class: format!("class-{index}"),
                    mode: EffectMode::Execute,
                    operation_binding: format!("operation-{index}/v1"),
                })
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
