//! Deterministic semantic identity/equivalence reference model.
//!
//! Claim ceiling: ReferenceModelOnly.
//! Identity resolution is scoped, evidence-bound, append-only, and never
//! allowed to turn a projection or similarity score into authority.

use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

pub const IDENTITY_PROFILE_ID: &str = "INTEGRAL-IDENTITY-REF-001";

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum IdentityKind {
    Entity,
    Principal,
    Credential,
    Account,
    Device,
    Resource,
    Locator,
    Pseudonym,
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct IdentityReference {
    pub reference_id: String,
    pub namespace: String,
    pub kind: IdentityKind,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum EquivalenceClass {
    SameRepresentation,
    SameScopedEntity,
    PotentiallySame,
    Distinct,
    Unknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum IdentityEvidence {
    ExactQualifiedBinding,
    ExplicitDistinctEvidence,
    SimilarityOnly,
    NoEvidence,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SemanticIdentityClaim {
    pub claim_id: String,
    pub left: IdentityReference,
    pub right: IdentityReference,
    pub scope_root: String,
    pub equivalence_profile_root: String,
    pub evidence_root: String,
    pub equivalence: EquivalenceClass,
    pub provenance_root: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum BindingStatus {
    Active,
    Superseded,
    Revoked,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum CredentialValidity {
    Active,
    Stale,
    Revoked,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct IdentityBinding {
    pub binding_id: String,
    pub reference: IdentityReference,
    pub scope_root: String,
    pub equivalence_profile_root: String,
    pub entity_reference: IdentityReference,
    pub provenance_root: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum IdentityLifecycleKind {
    Merge,
    Split,
    Supersede,
    Revoke,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct IdentityLifecycleEvent {
    pub event_id: String,
    pub kind: IdentityLifecycleKind,
    pub predecessors: BTreeSet<String>,
    pub successors: BTreeSet<String>,
    pub provenance_root: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ResourceCapacityBinding {
    pub resource_reference: IdentityReference,
    pub capacity_claims: BTreeSet<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum IdentityRecordOutcome {
    Recorded,
    Duplicate,
    Conflict,
    UnknownBinding,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct IdentityGraph {
    pub references: BTreeMap<(String, String, IdentityKind), IdentityReference>,
    pub claims: BTreeMap<String, SemanticIdentityClaim>,
    pub bindings: BTreeMap<String, IdentityBinding>,
    pub lifecycle_events: BTreeMap<String, IdentityLifecycleEvent>,
    pub resource_capacity: BTreeMap<(String, String, IdentityKind), ResourceCapacityBinding>,
}

impl IdentityGraph {
    pub fn new() -> Self {
        Self {
            references: BTreeMap::new(),
            claims: BTreeMap::new(),
            bindings: BTreeMap::new(),
            lifecycle_events: BTreeMap::new(),
            resource_capacity: BTreeMap::new(),
        }
    }

    pub fn register_reference(&mut self, reference: IdentityReference) -> IdentityRecordOutcome {
        let key = (
            reference.namespace.clone(),
            reference.reference_id.clone(),
            reference.kind,
        );
        if let Some(existing) = self.references.get(&key) {
            return if existing == &reference {
                IdentityRecordOutcome::Duplicate
            } else {
                IdentityRecordOutcome::Conflict
            };
        }
        self.references.insert(key, reference);
        IdentityRecordOutcome::Recorded
    }

    pub fn record_claim(&mut self, claim: SemanticIdentityClaim) -> IdentityRecordOutcome {
        if let Some(existing) = self.claims.get(&claim.claim_id) {
            return if existing == &claim {
                IdentityRecordOutcome::Duplicate
            } else {
                IdentityRecordOutcome::Conflict
            };
        }
        self.claims.insert(claim.claim_id.clone(), claim);
        IdentityRecordOutcome::Recorded
    }

    pub fn record_binding(&mut self, binding: IdentityBinding) -> IdentityRecordOutcome {
        if !self.references.contains_key(&(
            binding.reference.namespace.clone(),
            binding.reference.reference_id.clone(),
            binding.reference.kind,
        )) || !self.references.contains_key(&(
            binding.entity_reference.namespace.clone(),
            binding.entity_reference.reference_id.clone(),
            binding.entity_reference.kind,
        )) {
            return IdentityRecordOutcome::UnknownBinding;
        }
        if let Some(existing) = self.bindings.get(&binding.binding_id) {
            return if existing == &binding {
                IdentityRecordOutcome::Duplicate
            } else {
                IdentityRecordOutcome::Conflict
            };
        }
        self.bindings.insert(binding.binding_id.clone(), binding);
        IdentityRecordOutcome::Recorded
    }

    pub fn record_lifecycle(&mut self, event: IdentityLifecycleEvent) -> IdentityRecordOutcome {
        if event.predecessors.is_empty() || event.successors.is_empty() && event.kind != IdentityLifecycleKind::Revoke {
            return IdentityRecordOutcome::Conflict;
        }
        if !event.predecessors.iter().all(|id| self.bindings.contains_key(id))
            || !event.successors.iter().all(|id| self.bindings.contains_key(id))
        {
            return IdentityRecordOutcome::UnknownBinding;
        }
        if let Some(existing) = self.lifecycle_events.get(&event.event_id) {
            return if existing == &event {
                IdentityRecordOutcome::Duplicate
            } else {
                IdentityRecordOutcome::Conflict
            };
        }
        self.lifecycle_events.insert(event.event_id.clone(), event);
        IdentityRecordOutcome::Recorded
    }

    pub fn record_resource_capacity(
        &mut self,
        binding: ResourceCapacityBinding,
    ) -> IdentityRecordOutcome {
        if binding.resource_reference.kind != IdentityKind::Resource {
            return IdentityRecordOutcome::Conflict;
        }
        let key = (
            binding.resource_reference.namespace.clone(),
            binding.resource_reference.reference_id.clone(),
            binding.resource_reference.kind,
        );
        if let Some(existing) = self.resource_capacity.get(&key) {
            return if existing == &binding {
                IdentityRecordOutcome::Duplicate
            } else {
                IdentityRecordOutcome::Conflict
            };
        }
        self.resource_capacity.insert(key, binding);
        IdentityRecordOutcome::Recorded
    }
}

pub fn classify_identity(
    left: &IdentityReference,
    right: &IdentityReference,
    scope_root: &str,
    evidence: IdentityEvidence,
) -> EquivalenceClass {
    if left == right {
        return EquivalenceClass::SameRepresentation;
    }
    match evidence {
        IdentityEvidence::ExactQualifiedBinding if !scope_root.is_empty() => {
            EquivalenceClass::SameScopedEntity
        }
        IdentityEvidence::ExplicitDistinctEvidence => EquivalenceClass::Distinct,
        IdentityEvidence::SimilarityOnly => EquivalenceClass::PotentiallySame,
        IdentityEvidence::ExactQualifiedBinding | IdentityEvidence::NoEvidence => {
            EquivalenceClass::Unknown
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EquivalenceProfile {
    pub profile_id: String,
    pub scope_root: String,
    pub allowed_left_kind: IdentityKind,
    pub allowed_right_kind: IdentityKind,
    pub required_equivalence: EquivalenceClass,
    pub pairwise: bool,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct IdentityBridge {
    pub bridge_id: String,
    pub source_namespace: String,
    pub target_namespace: String,
    pub scope_root: String,
    pub equivalence_profile_root: String,
    pub pairwise: bool,
}

pub fn bridge_allows(
    bridge: &IdentityBridge,
    claim: &SemanticIdentityClaim,
) -> bool {
    claim.left.namespace == bridge.source_namespace
        && claim.right.namespace == bridge.target_namespace
        && claim.scope_root == bridge.scope_root
        && claim.equivalence_profile_root == bridge.equivalence_profile_root
        && matches!(claim.equivalence, EquivalenceClass::SameScopedEntity)
        && (!bridge.pairwise || claim.left.namespace != claim.right.namespace)
}

pub fn substitution_allowed(
    source: &IdentityReference,
    target: &IdentityReference,
    profile: &EquivalenceProfile,
    claim: &SemanticIdentityClaim,
    source_credential_validity: Option<CredentialValidity>,
) -> bool {
    let source_status_ok = match source.kind {
        IdentityKind::Credential => {
            matches!(source_credential_validity, Some(CredentialValidity::Active))
        }
        _ => source_credential_validity.is_none(),
    };

    source_status_ok
        && source == &claim.left
        && target == &claim.right
        && source.kind == profile.allowed_left_kind
        && target.kind == profile.allowed_right_kind
        && claim.scope_root == profile.scope_root
        && claim.equivalence_profile_root == profile.profile_id
        && claim.equivalence == profile.required_equivalence
        && matches!(claim.equivalence, EquivalenceClass::SameScopedEntity)
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct IdentityLifecycleConflict {
    pub binding_id: String,
    pub event_ids: Vec<String>,
}

pub fn lifecycle_conflicts(graph: &IdentityGraph) -> Vec<IdentityLifecycleConflict> {
    let mut by_predecessor: BTreeMap<String, Vec<String>> = BTreeMap::new();
    for event in graph.lifecycle_events.values() {
        for predecessor in &event.predecessors {
            by_predecessor
                .entry(predecessor.clone())
                .or_default()
                .push(event.event_id.clone());
        }
    }
    let mut conflicts = Vec::new();
    for (binding_id, mut event_ids) in by_predecessor {
        event_ids.sort();
        event_ids.dedup();
        if event_ids.len() > 1 {
            conflicts.push(IdentityLifecycleConflict {
                binding_id,
                event_ids,
            });
        }
    }
    conflicts
}

pub fn current_bindings(graph: &IdentityGraph) -> BTreeSet<String> {
    let mut retired = BTreeSet::new();
    for event in graph.lifecycle_events.values() {
        retired.extend(event.predecessors.iter().cloned());
    }
    graph
        .bindings
        .keys()
        .filter(|id| !retired.contains(*id))
        .cloned()
        .collect()
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct IdentityProjection {
    pub reference_id: String,
    pub namespace: String,
    pub kind: IdentityKind,
    pub equivalence: EquivalenceClass,
    pub may_globalize: bool,
    pub may_authorize: bool,
}

pub fn privacy_projection(
    reference: &IdentityReference,
    equivalence: EquivalenceClass,
    pairwise: bool,
) -> IdentityProjection {
    IdentityProjection {
        reference_id: reference.reference_id.clone(),
        namespace: reference.namespace.clone(),
        kind: reference.kind,
        equivalence,
        may_globalize: !pairwise,
        may_authorize: false,
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ResourceAliasAssessment {
    pub same_scoped_entity: bool,
    pub shared_capacity_claims: BTreeSet<String>,
    pub may_union_without_reallocation: bool,
}

pub fn assess_resource_alias(
    left: &ResourceCapacityBinding,
    right: &ResourceCapacityBinding,
    equivalence: EquivalenceClass,
) -> ResourceAliasAssessment {
    let shared_capacity_claims = left
        .capacity_claims
        .intersection(&right.capacity_claims)
        .cloned()
        .collect::<BTreeSet<_>>();
    let same_scoped_entity = matches!(equivalence, EquivalenceClass::SameScopedEntity);
    ResourceAliasAssessment {
        same_scoped_entity,
        may_union_without_reallocation: same_scoped_entity && shared_capacity_claims.is_empty(),
        shared_capacity_claims,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn reference(id: &str, kind: IdentityKind) -> IdentityReference {
        IdentityReference {
            reference_id: id.into(),
            namespace: "ns-1".into(),
            kind,
        }
    }

    fn binding(id: &str, reference: IdentityReference, entity: IdentityReference) -> IdentityBinding {
        IdentityBinding {
            binding_id: id.into(),
            reference,
            scope_root: "scope-1".into(),
            equivalence_profile_root: "profile-1".into(),
            entity_reference: entity,
            provenance_root: format!("prov-{id}"),
        }
    }

    #[test]
    fn same_string_in_different_namespaces_is_not_same_representation() {
        let left = IdentityReference {
            reference_id: "42".into(),
            namespace: "a".into(),
            kind: IdentityKind::Account,
        };
        let right = IdentityReference {
            reference_id: "42".into(),
            namespace: "b".into(),
            kind: IdentityKind::Account,
        };
        assert_eq!(classify_identity(&left, &right, "scope-1", IdentityEvidence::NoEvidence), EquivalenceClass::Unknown);
    }

    #[test]
    fn evidence_classes_do_not_collapse_into_identity() {
        let left = reference("left", IdentityKind::Account);
        let right = reference("right", IdentityKind::Account);
        assert_eq!(classify_identity(&left, &right, "scope-1", IdentityEvidence::SimilarityOnly), EquivalenceClass::PotentiallySame);
        assert_eq!(classify_identity(&left, &right, "scope-1", IdentityEvidence::ExplicitDistinctEvidence), EquivalenceClass::Distinct);
        assert_eq!(classify_identity(&left, &right, "scope-1", IdentityEvidence::NoEvidence), EquivalenceClass::Unknown);
    }

    #[test]
    fn exact_binding_is_scoped_and_typed() {
        let left = reference("cred-1", IdentityKind::Credential);
        let right = reference("principal-1", IdentityKind::Principal);
        let claim = SemanticIdentityClaim {
            claim_id: "claim-1".into(),
            left: left.clone(),
            right: right.clone(),
            scope_root: "scope-1".into(),
            equivalence_profile_root: "profile-1".into(),
            evidence_root: "evidence-1".into(),
            equivalence: EquivalenceClass::SameScopedEntity,
            provenance_root: "prov-1".into(),
        };
        let profile = EquivalenceProfile {
            profile_id: "profile-1".into(),
            scope_root: "scope-1".into(),
            allowed_left_kind: IdentityKind::Credential,
            allowed_right_kind: IdentityKind::Principal,
            required_equivalence: EquivalenceClass::SameScopedEntity,
            pairwise: true,
        };
        assert!(substitution_allowed(
            &left,
            &right,
            &profile,
            &claim,
            Some(CredentialValidity::Active),
        ));
        assert!(!substitution_allowed(
            &left,
            &right,
            &profile,
            &claim,
            Some(CredentialValidity::Stale),
        ));
        assert!(!substitution_allowed(
            &left,
            &right,
            &profile,
            &claim,
            Some(CredentialValidity::Revoked),
        ));

        let bridge = IdentityBridge {
            bridge_id: "bridge-1".into(),
            source_namespace: "ns-1".into(),
            target_namespace: "ns-1".into(),
            scope_root: "scope-1".into(),
            equivalence_profile_root: "profile-1".into(),
            pairwise: true,
        };
        assert!(!bridge_allows(&bridge, &claim));

        let wrong_target = reference("account-1", IdentityKind::Account);
        assert!(!substitution_allowed(&left, &wrong_target, &profile, &claim, Some(CredentialValidity::Active)));
    }

    #[test]
    fn lifecycle_events_are_append_only_and_conflicts_remain_visible() {
        let mut graph = IdentityGraph::new();
        let entity = reference("entity-1", IdentityKind::Entity);
        let account = reference("account-1", IdentityKind::Account);
        let successor_a = reference("entity-2", IdentityKind::Entity);
        let successor_b = reference("entity-3", IdentityKind::Entity);
        for reference in [entity.clone(), account.clone(), successor_a.clone(), successor_b.clone()] {
            assert_eq!(graph.register_reference(reference), IdentityRecordOutcome::Recorded);
        }
        assert_eq!(graph.record_binding(binding("b-1", account, entity)), IdentityRecordOutcome::Recorded);
        assert_eq!(graph.record_binding(binding("b-2", successor_a, successor_b.clone())), IdentityRecordOutcome::Recorded);
        assert_eq!(graph.record_binding(binding("b-3", successor_b.clone(), successor_b)), IdentityRecordOutcome::Recorded);

        let first = IdentityLifecycleEvent {
            event_id: "event-1".into(),
            kind: IdentityLifecycleKind::Supersede,
            predecessors: BTreeSet::from(["b-1".into()]),
            successors: BTreeSet::from(["b-2".into()]),
            provenance_root: "prov-event-1".into(),
        };
        let second = IdentityLifecycleEvent {
            event_id: "event-2".into(),
            kind: IdentityLifecycleKind::Revoke,
            predecessors: BTreeSet::from(["b-1".into()]),
            successors: BTreeSet::new(),
            provenance_root: "prov-event-2".into(),
        };
        assert_eq!(graph.record_lifecycle(first), IdentityRecordOutcome::Recorded);
        assert_eq!(graph.record_lifecycle(second), IdentityRecordOutcome::Recorded);
        assert_eq!(lifecycle_conflicts(&graph).len(), 1);
        assert!(current_bindings(&graph).contains("b-2"));
        assert!(!current_bindings(&graph).contains("b-1"));
        assert!(graph.lifecycle_events.contains_key("event-1"));
        assert!(graph.lifecycle_events.contains_key("event-2"));
    }

    #[test]
    fn resource_aliasing_cannot_duplicate_capacity() {
        let left = ResourceCapacityBinding {
            resource_reference: reference("resource-a", IdentityKind::Resource),
            capacity_claims: BTreeSet::from(["capacity-1".into(), "capacity-2".into()]),
        };
        let right = ResourceCapacityBinding {
            resource_reference: reference("resource-b", IdentityKind::Resource),
            capacity_claims: BTreeSet::from(["capacity-2".into(), "capacity-3".into()]),
        };
        let assessment = assess_resource_alias(&left, &right, EquivalenceClass::SameScopedEntity);
        assert_eq!(assessment.shared_capacity_claims, BTreeSet::from(["capacity-2".into()]));
        assert!(!assessment.may_union_without_reallocation);
    }

    #[test]
    fn resource_capacity_registration_preserves_type_and_identity() {
        let mut graph = IdentityGraph::new();
        let resource = reference("resource-1", IdentityKind::Resource);
        let binding = ResourceCapacityBinding {
            resource_reference: resource.clone(),
            capacity_claims: BTreeSet::from(["capacity-1".into()]),
        };
        assert_eq!(
            graph.record_resource_capacity(binding.clone()),
            IdentityRecordOutcome::Recorded
        );
        assert_eq!(
            graph.record_resource_capacity(binding),
            IdentityRecordOutcome::Duplicate
        );

        let wrong_type = ResourceCapacityBinding {
            resource_reference: reference("account-1", IdentityKind::Account),
            capacity_claims: BTreeSet::from(["capacity-2".into()]),
        };
        assert_eq!(
            graph.record_resource_capacity(wrong_type),
            IdentityRecordOutcome::Conflict
        );
    }

    #[test]
    fn privacy_projection_never_authorizes_and_pairwise_does_not_globalize() {
        let pseudonym = reference("pairwise-7", IdentityKind::Pseudonym);
        let projection = privacy_projection(&pseudonym, EquivalenceClass::PotentiallySame, true);
        assert!(!projection.may_globalize);
        assert!(!projection.may_authorize);
    }

    #[test]
    fn identity_graph_rejects_unknown_binding_references() {
        let mut graph = IdentityGraph::new();
        let result = graph.record_binding(binding("b-unknown", reference("a", IdentityKind::Account), reference("e", IdentityKind::Entity)));
        assert_eq!(result, IdentityRecordOutcome::UnknownBinding);
    }
}