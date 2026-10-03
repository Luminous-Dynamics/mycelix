use crate::identity_lineage::IdentityRef;
use std::collections::BTreeMap;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum QualificationStatus<T> {
    Complete(T),
    Unresolved {
        missing: Vec<IdentityRef>,
        partial: T,
    },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum QualificationValidationError {
    Structural { reason: String },
}

pub type QualificationOutcome<T> =
    Result<QualificationStatus<T>, QualificationValidationError>;

/// Runtime-neutral three-outcome boundary for protocol adapters.
///
/// This mirrors the semantic distinction needed by distributed validation:
/// valid completion, definitive structural invalidity, and unresolved
/// addressable dependencies. It contains no Holochain-specific types so the
/// pure qualification crate remains portable.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum QualificationDecision<T> {
    Valid(T),
    Invalid { reason: String },
    Unresolved {
        missing: Vec<IdentityRef>,
        partial: T,
    },
}

/// Explicit runtime-owned binding from a logical qualification dependency
/// to one opaque protocol address.
///
/// The address type is intentionally generic: the pure layer never learns
/// whether the runtime uses a Holochain hash, a database key, or another
/// addressable representation. Binding is explicit and deterministic.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct QualificationDependencyBindingSet<A> {
    bindings: BTreeMap<IdentityRef, A>,
}

impl<A> Default for QualificationDependencyBindingSet<A> {
    fn default() -> Self {
        Self {
            bindings: BTreeMap::new(),
        }
    }
}

impl<A> QualificationDependencyBindingSet<A> {
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a logical identity binding exactly once.
    ///
    /// Rebinding an existing logical identity is rejected instead of silently
    /// replacing the protocol address. A retry or address migration must build
    /// a new binding set rather than mutate the meaning of an existing one.
    pub fn insert(
        &mut self,
        identity: IdentityRef,
        address: A,
    ) -> Result<(), QualificationValidationError> {
        identity
            .validate()
            .map_err(QualificationValidationError::structural)?;
        if self.bindings.contains_key(&identity) {
            return Err(QualificationValidationError::structural(
                "logical qualification dependency cannot be bound to multiple protocol addresses",
            ));
        }
        self.bindings.insert(identity, address);
        Ok(())
    }

    pub fn get(&self, identity: &IdentityRef) -> Option<&A> {
        self.bindings.get(identity)
    }

    pub fn contains(&self, identity: &IdentityRef) -> bool {
        self.bindings.contains_key(identity)
    }

    pub fn len(&self) -> usize {
        self.bindings.len()
    }

    pub fn is_empty(&self) -> bool {
        self.bindings.is_empty()
    }

    /// Resolve a required logical-dependency set in canonical identity order.
    ///
    /// A malformed logical identity is a definitive structural invalidity.
    /// Otherwise, success returns every bound logical identity alongside its
    /// opaque runtime address. Missing bindings yield an unresolved result with
    /// the canonical missing identities and any available partial bindings.
    pub fn resolve_required<I>(
        &self,
        identities: I,
    ) -> QualificationDecision<Vec<(&IdentityRef, &A)>>
    where
        I: IntoIterator<Item = IdentityRef>,
    {
        let requested: std::collections::BTreeSet<_> = identities.into_iter().collect();

        for identity in &requested {
            if let Err(reason) = identity.validate() {
                return QualificationDecision::Invalid { reason };
            }
        }

        let missing: Vec<_> = requested
            .iter()
            .filter(|identity| !self.bindings.contains_key(*identity))
            .cloned()
            .collect();

        let bound: Vec<_> = self
            .bindings
            .iter()
            .filter(|(identity, _)| requested.contains(*identity))
            .collect();

        if missing.is_empty() {
            QualificationDecision::Valid(bound)
        } else {
            QualificationDecision::Unresolved {
                missing,
                partial: bound,
            }
        }
    }

    /// Iterate in canonical logical-identity order, independent of insertion order.
    pub fn iter(&self) -> impl Iterator<Item = (&IdentityRef, &A)> {
        self.bindings.iter()
    }
}

impl<T> From<QualificationOutcome<T>> for QualificationDecision<T> {
    fn from(outcome: QualificationOutcome<T>) -> Self {
        match outcome {
            Ok(QualificationStatus::Complete(value)) => Self::Valid(value),
            Ok(QualificationStatus::Unresolved { missing, partial }) => Self::Unresolved {
                missing: canonicalize_missing(missing),
                partial,
            },
            Err(QualificationValidationError::Structural { reason }) => {
                Self::Invalid { reason }
            }
        }
    }
}

impl<T> QualificationDecision<T> {
    pub fn is_valid(&self) -> bool {
        matches!(self, Self::Valid(_))
    }

    pub fn is_unresolved(&self) -> bool {
        matches!(self, Self::Unresolved { .. })
    }

    /// Whether this decision is final for the current dependency set.
    ///
    /// Runtime adapters may map this to a protocol's definitive-validation
    /// predicate. Unresolved decisions are deliberately non-definitive.
    pub fn is_definitive(&self) -> bool {
        !self.is_unresolved()
    }

    /// Returns logical engineering identities that remain unavailable.
    ///
    /// These are not protocol/DHT hashes. A runtime adapter must bind each
    /// identity to its own addressable dependency representation before
    /// constructing a protocol-specific unresolved result.
    pub fn missing_identities(&self) -> &[IdentityRef] {
        self.missing()
    }

    pub fn missing(&self) -> &[IdentityRef] {
        match self {
            Self::Unresolved { missing, .. } => missing,
            Self::Valid(_) | Self::Invalid { .. } => &[],
        }
    }

    pub fn invalid_reason(&self) -> Option<&str> {
        match self {
            Self::Invalid { reason } => Some(reason),
            Self::Valid(_) | Self::Unresolved { .. } => None,
        }
    }
}

fn canonicalize_missing(mut missing: Vec<IdentityRef>) -> Vec<IdentityRef> {
    missing.sort_by(|left, right| {
        (&left.namespace, &left.kind, &left.id)
            .cmp(&(&right.namespace, &right.kind, &right.id))
    });
    missing.dedup();
    missing
}

impl<T> QualificationStatus<T> {
    pub fn unresolved(missing: Vec<IdentityRef>, partial: T) -> Self {
        Self::Unresolved {
            missing: canonicalize_missing(missing),
            partial,
        }
    }

    /// Adds unavailable dependencies without discarding the partial
    /// qualification payload. This is the compositional boundary for
    /// independently qualified dependency paths.
    ///
    /// A complete status becomes unresolved when any additional dependency
    /// is unavailable. Existing unresolved dependencies and newly added
    /// dependencies are canonicalized into one deterministic set.
    pub fn require_missing(
        self,
        additional: impl IntoIterator<Item = IdentityRef>,
    ) -> Self {
        let additional: Vec<_> = additional.into_iter().collect();

        match self {
            Self::Complete(partial) => {
                if additional.is_empty() {
                    Self::Complete(partial)
                } else {
                    Self::unresolved(additional, partial)
                }
            }
            Self::Unresolved {
                mut missing,
                partial,
            } => {
                missing.extend(additional);
                Self::unresolved(missing, partial)
            }
        }
    }

    /// Combines two independent qualification paths without collapsing an
    /// unresolved dependency state.
    ///
    /// Both partial qualification values are retained even when either path
    /// is unresolved. Missing dependency identities are unioned and
    /// canonicalized, making this a deterministic product operation for
    /// independently qualified evidence paths.
    pub fn zip<U>(self, other: QualificationStatus<U>) -> QualificationStatus<(T, U)> {
        match (self, other) {
            (Self::Complete(left), Self::Complete(right)) => {
                QualificationStatus::Complete((left, right))
            }
            (
                Self::Unresolved {
                    missing,
                    partial: left,
                },
                Self::Complete(right),
            )
            | (
                Self::Complete(left),
                Self::Unresolved {
                    missing,
                    partial: right,
                },
            ) => QualificationStatus::Unresolved {
                missing: canonicalize_missing(missing),
                partial: (left, right),
            },
            (
                Self::Unresolved {
                    mut missing,
                    partial: left,
                },
                Self::Unresolved {
                    missing: right_missing,
                    partial: right,
                },
            ) => {
                missing.extend(right_missing);
                QualificationStatus::Unresolved {
                    missing: canonicalize_missing(missing),
                    partial: (left, right),
                }
            }
        }
    }

    pub fn normalize_missing(&mut self) {
        if let Self::Unresolved { missing, .. } = self {
            *missing = canonicalize_missing(std::mem::take(missing));
        }
    }

    pub fn map<U>(self, map: impl FnOnce(T) -> U) -> QualificationStatus<U> {
        match self {
            Self::Complete(value) => QualificationStatus::Complete(map(value)),
            Self::Unresolved { missing, partial } => {
                QualificationStatus::unresolved(missing, map(partial))
            }
        }
    }

    pub fn is_complete(&self) -> bool {
        matches!(self, Self::Complete(_))
    }

    pub fn missing(&self) -> &[IdentityRef] {
        match self {
            Self::Complete(_) => &[],
            Self::Unresolved { missing, .. } => missing,
        }
    }
}

impl<T> QualificationOutcome<T> {
    /// Adds unavailable dependencies to a successful qualification result.
    ///
    /// Structural validation errors are preserved unchanged; only the
    /// Complete/Unresolved status is enriched with additional missing
    /// addressable dependencies.
    pub fn require_missing(
        self,
        additional: impl IntoIterator<Item = IdentityRef>,
    ) -> Self {
        self.map(|status| status.require_missing(additional))
    }
}

impl QualificationValidationError {
    pub fn structural(reason: impl Into<String>) -> Self {
        Self::Structural {
            reason: reason.into(),
        }
    }
}

impl std::fmt::Display for QualificationValidationError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Structural { reason } => formatter.write_str(reason),
        }
    }
}

impl std::error::Error for QualificationValidationError {}

impl From<String> for QualificationValidationError {
    fn from(reason: String) -> Self {
        Self::structural(reason)
    }
}

impl From<&str> for QualificationValidationError {
    fn from(reason: &str) -> Self {
        Self::structural(reason)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::identity_lineage::IdentityKind;

    fn id(value: &str) -> IdentityRef {
        IdentityRef {
            kind: IdentityKind::ReconciliationWitness,
            namespace: "mobility".into(),
            id: value.into(),
        }
    }

    #[test]
    fn dependency_binding_set_rejects_logical_identity_rebinding() {
        let mut bindings = QualificationDependencyBindingSet::new();
        let identity = id("logical-1");

        assert!(bindings.insert(identity.clone(), "address-a").is_ok());
        let error = bindings
            .insert(identity, "address-b")
            .expect_err("a logical identity may not be rebound silently");

        assert_eq!(
            error.to_string(),
            "logical qualification dependency cannot be bound to multiple protocol addresses"
        );
        assert_eq!(bindings.len(), 1);
        assert_eq!(bindings.get(&id("logical-1")), Some(&"address-a"));
    }

    #[test]
    fn dependency_binding_set_rejects_invalid_logical_identity() {
        let mut bindings = QualificationDependencyBindingSet::new();
        let invalid = IdentityRef {
            kind: IdentityKind::EvidenceRecord,
            namespace: "holochain".into(),
            id: "not-a-native-engineering-id".into(),
        };

        assert!(bindings.insert(invalid, "address").is_err());
        assert!(bindings.is_empty());
    }

    #[test]
    fn dependency_binding_iteration_is_invariant_to_insertion_order() {
        let a = id("a");
        let b = id("b");

        let mut first = QualificationDependencyBindingSet::new();
        first.insert(b.clone(), "address-b").unwrap();
        first.insert(a.clone(), "address-a").unwrap();

        let mut second = QualificationDependencyBindingSet::new();
        second.insert(a.clone(), "address-a").unwrap();
        second.insert(b.clone(), "address-b").unwrap();

        assert_eq!(
            first.iter().collect::<Vec<_>>(),
            second.iter().collect::<Vec<_>>()
        );
    }

    #[test]
    fn dependency_binding_resolution_returns_canonical_bound_pairs() {
        let mut bindings = QualificationDependencyBindingSet::new();
        let z = id("z");
        let a = id("a");
        bindings.insert(z.clone(), "address-z").unwrap();
        bindings.insert(a.clone(), "address-a").unwrap();

        let resolved = bindings.resolve_required(vec![z.clone(), a.clone(), a]);
        let QualificationDecision::Valid(resolved) = resolved else {
            panic!("all logical dependencies are bound");
        };

        assert_eq!(resolved[0].0, bindings.iter().next().expect("first binding exists").0);
        assert_eq!(resolved[0].1, &"address-a");
        assert_eq!(resolved[1].0, &z);
        assert_eq!(resolved[1].1, &"address-z");
    }

    #[test]
    fn dependency_binding_resolution_reports_only_unbound_logical_identities() {
        let mut bindings = QualificationDependencyBindingSet::new();
        bindings.insert(id("z"), "address-z").unwrap();

        let result = bindings.resolve_required(vec![id("z"), id("m"), id("m"), id("a")]);
        let QualificationDecision::Unresolved { missing, partial } = result else {
            panic!("unbound logical dependencies must remain unresolved");
        };

        assert_eq!(missing, vec![id("a"), id("m")]);
        assert_eq!(partial.len(), 1);
        assert_eq!(partial[0].0, &id("z"));
        assert_eq!(partial[0].1, &"address-z");
    }

    #[test]
    fn dependency_binding_resolution_rejects_malformed_requested_identity() {
        let bindings = QualificationDependencyBindingSet::new();
        let invalid = IdentityRef {
            kind: IdentityKind::EvidenceRecord,
            namespace: "holochain".into(),
            id: "not-a-native-engineering-id".into(),
        };

        let decision = bindings.resolve_required(vec![invalid]);
        assert_eq!(
            decision,
            QualificationDecision::Invalid {
                reason: "Holochain protocol identifiers require explicit binding and cannot be native engineering identities".into(),
            }
        );
    }

    #[test]
    fn dependency_binding_resolution_is_independent_of_request_order() {
        let mut bindings = QualificationDependencyBindingSet::new();
        bindings.insert(id("a"), "address-a").unwrap();
        bindings.insert(id("b"), "address-b").unwrap();
        bindings.insert(id("c"), "address-c").unwrap();

        let forward = bindings
            .resolve_required(vec![id("c"), id("a"), id("b")])
            .unwrap();
        let reverse = bindings
            .resolve_required(vec![id("b"), id("c"), id("a")])
            .unwrap();

        assert_eq!(forward, reverse);
    }

    #[test]
    fn dependency_binding_set_never_derives_an_address_from_identity_text() {
        let mut bindings = QualificationDependencyBindingSet::new();
        let identity = id("logical-with-hash-shaped-text");

        bindings.insert(identity.clone(), 42u64).unwrap();
        assert_eq!(bindings.get(&identity), Some(&42u64));
    }

    #[test]
    fn adapter_boundary_contract_is_machine_readable_and_three_valued() {
        let contract_path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../../docs/mobility/MOBILITY_QUALIFICATION_ADAPTER_BOUNDARY_V1.json");
        let contract = std::fs::read_to_string(&contract_path)
            .unwrap_or_else(|error| panic!("cannot read adapter boundary contract: {error}"));
        let document: serde_json::Value =
            serde_json::from_str(&contract).expect("adapter boundary contract must be valid JSON");

        assert_eq!(
            document
                .get("schema")
                .and_then(serde_json::Value::as_str),
            Some("mycelix.mobility.qualification_adapter_boundary.v1")
        );
        assert_eq!(
            document
                .get("semantic_outcomes")
                .and_then(serde_json::Value::as_array)
                .map(std::vec::Vec::len),
            Some(3)
        );

        let outcomes = document
            .get("semantic_outcomes")
            .and_then(serde_json::Value::as_array)
            .expect("adapter contract must declare semantic outcomes");
        let pure_names: Vec<_> = outcomes
            .iter()
            .filter_map(|outcome| outcome.get("pure"))
            .filter_map(serde_json::Value::as_str)
            .collect();
        assert_eq!(pure_names, vec!["valid", "invalid", "unresolved"]);

        assert_eq!(
            outcomes[0]
                .get("holochain")
                .and_then(serde_json::Value::as_str),
            Some("ValidateCallbackResult::Valid")
        );
        assert_eq!(
            outcomes[1]
                .get("holochain")
                .and_then(serde_json::Value::as_str),
            Some("ValidateCallbackResult::Invalid(String)")
        );
        assert_eq!(
            outcomes[2]
                .get("holochain")
                .and_then(serde_json::Value::as_str),
            Some("ValidateCallbackResult::UnresolvedDependencies(UnresolvedDependencies)")
        );

        let errors = document
            .get("error_boundary")
            .expect("adapter contract must declare error boundary");
        assert_eq!(
            errors
                .get("semantic_invalidity")
                .and_then(serde_json::Value::as_str),
            Some("must be returned as a validation result, not ExternResult::Err")
        );
        assert_eq!(
            errors
                .get("runtime_failure")
                .and_then(serde_json::Value::as_str),
            Some("may remain an ExternResult::Err")
        );
        assert_eq!(
            errors
                .get("dependency_absence")
                .and_then(serde_json::Value::as_str),
            Some("must remain unresolved until the required addressable dependency is available")
        );

        let dependencies = document
            .get("dependency_retrieval")
            .expect("adapter contract must declare dependency retrieval");
        assert_eq!(
            dependencies
                .get("deterministic_host_function_family")
                .and_then(serde_json::Value::as_str),
            Some("must_get_*")
        );
        assert_eq!(
            dependencies
                .get("mutable_link_collections_as_validation_dependencies")
                .and_then(serde_json::Value::as_bool),
            Some(false)
        );
        assert_eq!(
            dependencies
                .get("missing_addressable_dependency_is_semantic_invalidity")
                .and_then(serde_json::Value::as_bool),
            Some(false)
        );

        assert_eq!(
            dependencies
                .get("binding_preserves_address_kind")
                .and_then(serde_json::Value::as_bool),
            Some(true)
        );
        assert_eq!(
            dependencies
                .get("valid_record_requires_action_hash")
                .and_then(serde_json::Value::as_bool),
            Some(true)
        );
        assert_eq!(
            dependencies
                .get("wrong_address_kind_is_adapter_boundary_error")
                .and_then(serde_json::Value::as_bool),
            Some(true)
        );

        let determinism = document
            .get("determinism")
            .expect("adapter contract must declare determinism");
        for key in [
            "retrieval_order_independent",
            "serialization_order_independent",
            "batching_partition_independent",
            "wall_clock_independent",
            "peer_identity_independent",
        ] {
            assert_eq!(
                determinism.get(key).and_then(serde_json::Value::as_bool),
                Some(true),
                "determinism invariant must be enabled: {key}"
            );
        }

        let identity = document
            .get("identity_boundary")
            .expect("adapter contract must declare identity boundary");
        assert_eq!(
            identity
                .get("logical_identity_type")
                .and_then(serde_json::Value::as_str),
            Some("IdentityRef")
        );
        assert_eq!(
            identity
                .get("implicit_string_to_hash_conversion")
                .and_then(serde_json::Value::as_bool),
            Some(false)
        );
        assert_eq!(
            identity
                .get("adapter_binding_required")
                .and_then(serde_json::Value::as_bool),
            Some(true)
        );

        assert_eq!(
            identity
                .get("protocol_address_type")
                .and_then(serde_json::Value::as_str),
            Some("Holochain hash or other runtime-specific addressable dependency")
        );
        assert_eq!(
            identity
                .get("exactly_one_address_per_logical_identity")
                .and_then(serde_json::Value::as_bool),
            Some(true)
        );
        assert_eq!(
            identity
                .get("duplicate_logical_identity_binding")
                .and_then(serde_json::Value::as_str),
            Some("rejected")
        );
        assert_eq!(
            identity
                .get("unbound_logical_identity_is_semantic_invalidity")
                .and_then(serde_json::Value::as_bool),
            Some(false)
        );
        assert_eq!(
            identity
                .get("binding_order_independent")
                .and_then(serde_json::Value::as_bool),
            Some(true)
        );

        assert_eq!(
            identity
                .get("resolution_operation")
                .and_then(serde_json::Value::as_str),
            Some("QualificationDependencyBindingSet::resolve_required")
        );
        assert_eq!(
            identity
                .get("resolution_success")
                .and_then(serde_json::Value::as_str),
            Some("all requested logical identities bound; return canonical identity/address pairs")
        );
        assert_eq!(
            identity
                .get("resolution_invalid")
                .and_then(serde_json::Value::as_str),
            Some("malformed logical identity is definitive invalidity")
        );
        assert_eq!(
            identity
                .get("resolution_missing")
                .and_then(serde_json::Value::as_str),
            Some("return canonical missing logical identities and preserve unresolved semantic state")
        );
        assert_eq!(
            identity
                .get("duplicate_requests_deduplicated")
                .and_then(serde_json::Value::as_bool),
            Some(true)
        );
        assert_eq!(
            identity
                .get("address_derivation")
                .and_then(serde_json::Value::as_bool),
            Some(false)
        );
    }

    #[test]
    fn outcome_maps_to_valid_decision() {
        let decision = QualificationDecision::from(
            Ok::<_, QualificationValidationError>(QualificationStatus::Complete(7u8)),
        );

        assert_eq!(decision, QualificationDecision::Valid(7u8));
        assert!(decision.is_valid());
        assert!(!decision.is_unresolved());
        assert_eq!(decision.missing(), &[]);
    }

    #[test]
    fn outcome_maps_to_unresolved_decision_and_canonicalizes_dependencies() {
        let decision = QualificationDecision::from(Ok::<_, QualificationValidationError>(
            QualificationStatus::Unresolved {
                missing: vec![id("z"), id("a"), id("z")],
                partial: 7u8,
            },
        ));

        assert_eq!(
            decision,
            QualificationDecision::Unresolved {
                missing: vec![id("a"), id("z")],
                partial: 7u8,
            }
        );
        assert!(decision.is_unresolved());
        assert_eq!(decision.missing(), &[id("a"), id("z")]);
    }

    #[test]
    fn outcome_maps_structural_error_to_invalid_decision() {
        let decision = QualificationDecision::from(
            Err::<QualificationStatus<u8>, _>(
                QualificationValidationError::structural("contradiction"),
            ),
        );

        assert_eq!(
            decision,
            QualificationDecision::Invalid {
                reason: "contradiction".into(),
            }
        );
        assert_eq!(decision.invalid_reason(), Some("contradiction"));
        assert!(!decision.is_valid());
        assert!(!decision.is_unresolved());
    }

    #[test]
    fn unresolved_decision_exposes_logical_identities_without_protocol_conversion() {
        let decision = QualificationDecision::Unresolved {
            missing: vec![id("z"), id("a")],
            partial: 7u8,
        };

        assert!(decision.is_unresolved());
        assert!(!decision.is_definitive());
        assert_eq!(decision.missing_identities(), &[id("a"), id("z")]);
    }

    #[test]
    fn valid_and_invalid_decisions_are_definitive() {
        let valid = QualificationDecision::Valid(1u8);
        let invalid = QualificationDecision::Invalid {
            reason: "contradiction".into(),
        };

        assert!(valid.is_definitive());
        assert!(invalid.is_definitive());
    }

    #[test]
    fn decision_projection_has_mutually_exclusive_terminal_states() {
        let valid = QualificationDecision::Valid(1u8);
        let invalid = QualificationDecision::Invalid {
            reason: "x".into(),
        };
        let unresolved = QualificationDecision::Unresolved {
            missing: vec![id("m")],
            partial: 1u8,
        };

        assert!(valid.is_valid());
        assert!(!valid.is_unresolved());
        assert_eq!(valid.invalid_reason(), None);
        assert!(!invalid.is_valid());
        assert!(!invalid.is_unresolved());
        assert_eq!(invalid.missing(), &[]);
        assert!(!unresolved.is_valid());
        assert!(unresolved.is_unresolved());
        assert_eq!(unresolved.invalid_reason(), None);
    }

    #[test]
    fn complete_has_no_missing_dependencies() {
        let status = QualificationStatus::Complete(7u8);
        assert!(status.is_complete());
        assert!(status.missing().is_empty());
    }

    #[test]
    fn unresolved_retains_missing_dependencies_and_partial_state() {
        let missing = id("missing");
        let status = QualificationStatus::Unresolved {
            missing: vec![missing.clone()],
            partial: 7u8,
        };

        assert!(!status.is_complete());
        assert_eq!(status.missing(), &[missing]);
    }

    #[test]
    fn unresolved_normalizes_missing_dependency_set() {
        let first = id("z");
        let second = id("a");
        let status = QualificationStatus::unresolved(
            vec![first.clone(), second.clone(), first.clone()],
            7u8,
        );

        assert_eq!(
            status,
            QualificationStatus::Unresolved {
                missing: vec![second, first],
                partial: 7u8,
            }
        );
    }

    #[test]
    fn unresolved_orders_missing_by_namespace_kind_and_id() {
        let mut first = id("z");
        first.namespace = "zeta".into();
        first.kind = IdentityKind::EvidenceRecord;

        let mut second = id("a");
        second.namespace = "alpha".into();
        second.kind = IdentityKind::ReconciliationWitness;

        let mut third = id("b");
        third.namespace = "alpha".into();
        third.kind = IdentityKind::EvidenceRecord;

        let status = QualificationStatus::unresolved(
            vec![first.clone(), second.clone(), third.clone()],
            7u8,
        );

        assert_eq!(
            status,
            QualificationStatus::Unresolved {
                missing: vec![third, second, first],
                partial: 7u8,
            }
        );
    }

    #[test]
    fn require_missing_promotes_complete_and_preserves_partial() {
        let status = QualificationStatus::Complete(7u8)
            .require_missing(vec![id("b"), id("a"), id("b")]);

        assert_eq!(
            status,
            QualificationStatus::Unresolved {
                missing: vec![id("a"), id("b")],
                partial: 7u8,
            }
        );
    }

    #[test]
    fn require_missing_unions_existing_and_new_dependencies() {
        let status = QualificationStatus::Unresolved {
            missing: vec![id("z")],
            partial: 9u8,
        }
        .require_missing(vec![id("a"), id("z"), id("m")]);

        assert_eq!(
            status,
            QualificationStatus::Unresolved {
                missing: vec![id("a"), id("m"), id("z")],
                partial: 9u8,
            }
        );
    }

    #[test]
    fn require_missing_empty_is_noop() {
        let status = QualificationStatus::Complete(7u8).require_missing(Vec::new());
        assert_eq!(status, QualificationStatus::Complete(7u8));
    }

    #[test]
    fn require_missing_recanonicalizes_existing_unresolved_dependencies() {
        let status = QualificationStatus::Unresolved {
            missing: vec![id("z"), id("a"), id("z")],
            partial: 7u8,
        }
        .require_missing(Vec::new());

        assert_eq!(
            status,
            QualificationStatus::Unresolved {
                missing: vec![id("a"), id("z")],
                partial: 7u8,
            }
        );
    }

    #[test]
    fn require_missing_is_order_independent_and_idempotent() {
        let first = id("a");
        let second = id("b");

        let left = QualificationStatus::Complete(7u8)
            .require_missing(vec![first.clone(), second.clone()])
            .require_missing(vec![first.clone()]);
        let right = QualificationStatus::Complete(7u8)
            .require_missing(vec![second, first]);

        assert_eq!(left, right);
    }

    #[test]
    fn zip_preserves_both_complete_partials() {
        let status = QualificationStatus::Complete(1u8).zip(QualificationStatus::Complete(2u8));
        assert_eq!(status, QualificationStatus::Complete((1u8, 2u8)));
    }

    #[test]
    fn zip_accumulates_one_unresolved_dependency_set() {
        let status = QualificationStatus::Complete(1u8).zip(QualificationStatus::Unresolved {
            missing: vec![id("b"), id("a"), id("b")],
            partial: 2u8,
        });

        assert_eq!(
            status,
            QualificationStatus::Unresolved {
                missing: vec![id("a"), id("b")],
                partial: (1u8, 2u8),
            }
        );
    }

    #[test]
    fn zip_accumulates_unresolved_dependency_when_it_is_the_left_path() {
        let status = QualificationStatus::Unresolved {
            missing: vec![id("left")],
            partial: 1u8,
        }
        .zip(QualificationStatus::Complete(2u8));

        assert_eq!(
            status,
            QualificationStatus::Unresolved {
                missing: vec![id("left")],
                partial: (1u8, 2u8),
            }
        );
    }

    #[test]
    fn zip_unions_two_unresolved_dependency_sets() {
        let status = QualificationStatus::Unresolved {
            missing: vec![id("z"), id("a")],
            partial: 1u8,
        }
        .zip(QualificationStatus::Unresolved {
            missing: vec![id("m"), id("a")],
            partial: 2u8,
        });

        assert_eq!(
            status,
            QualificationStatus::Unresolved {
                missing: vec![id("a"), id("m"), id("z")],
                partial: (1u8, 2u8),
            }
        );
    }

    #[test]
    fn zip_is_order_independent_for_missing_dependencies() {
        let left = QualificationStatus::Unresolved {
            missing: vec![id("b"), id("a")],
            partial: 1u8,
        }
        .zip(QualificationStatus::Unresolved {
            missing: vec![id("c")],
            partial: 2u8,
        });

        let right = QualificationStatus::Unresolved {
            missing: vec![id("c")],
            partial: 2u8,
        }
        .zip(QualificationStatus::Unresolved {
            missing: vec![id("a"), id("b")],
            partial: 1u8,
        });

        assert_eq!(left.missing(), right.missing());
    }

    #[test]
    fn zip_preserves_partial_payloads_even_when_unresolved() {
        let status = QualificationStatus::Unresolved {
            missing: vec![id("left-missing")],
            partial: 11u8,
        }
        .zip(QualificationStatus::Unresolved {
            missing: vec![id("right-missing")],
            partial: 22u8,
        });

        assert_eq!(status.missing().len(), 2);
        assert_eq!(
            status,
            QualificationStatus::Unresolved {
                missing: vec![id("left-missing"), id("right-missing")],
                partial: (11u8, 22u8),
            }
        );
    }

    #[test]
    fn require_missing_is_associative_over_dependency_sets() {
        let a = id("a");
        let b = id("b");
        let c = id("c");

        let grouped = QualificationStatus::Complete(())
            .require_missing(vec![a.clone()])
            .require_missing(vec![b.clone()])
            .require_missing(vec![c.clone()]);
        let batched = QualificationStatus::Complete(())
            .require_missing(vec![c, a, b]);

        assert_eq!(grouped.missing(), batched.missing());
    }

    #[test]
    fn zip_has_grouping_invariant_missing_dependencies() {
        let first = QualificationStatus::Unresolved {
            missing: vec![id("a")],
            partial: 1u8,
        };
        let second = QualificationStatus::Unresolved {
            missing: vec![id("b")],
            partial: 2u8,
        };
        let third = QualificationStatus::Unresolved {
            missing: vec![id("c")],
            partial: 3u8,
        };

        let left_grouped = first.clone().zip(second.clone()).zip(third.clone());
        let right_grouped = first.zip(second.zip(third));

        assert_eq!(left_grouped.missing(), right_grouped.missing());
    }

    #[test]
    fn map_preserves_resolution_state() {
        let complete = QualificationStatus::Complete(2u8).map(|value| value * 2);
        assert_eq!(complete, QualificationStatus::Complete(4u8));

        let unresolved = QualificationStatus::Unresolved {
            missing: vec![id("missing")],
            partial: 3u8,
        }
        .map(|value| value * 2);
        assert_eq!(
            unresolved,
            QualificationStatus::Unresolved {
                missing: vec![id("missing")],
                partial: 6u8,
            }
        );
    }

    #[test]
    fn outcome_require_missing_preserves_structural_errors() {
        let error = QualificationValidationError::structural("contradiction");
        let result: QualificationOutcome<u8> = Err(error.clone())
            .require_missing(vec![id("ignored")]);

        assert_eq!(result, Err(error));
    }

    #[test]
    fn outcome_require_missing_promotes_success_to_unresolved() {
        let result: QualificationOutcome<u8> = Ok(QualificationStatus::Complete(7))
            .require_missing(vec![id("missing")]);

        assert_eq!(
            result,
            Ok(QualificationStatus::Unresolved {
                missing: vec![id("missing")],
                partial: 7,
            })
        );
    }

    #[test]
    fn map_recanonicalizes_unresolved_dependencies() {
        let status = QualificationStatus::Unresolved {
            missing: vec![id("z"), id("a"), id("z")],
            partial: 3u8,
        }
        .map(|value| value * 2);

        assert_eq!(
            status,
            QualificationStatus::Unresolved {
                missing: vec![id("a"), id("z")],
                partial: 6u8,
            }
        );
    }

    #[test]
    fn structural_error_is_machine_checkable() {
        let error = QualificationValidationError::structural("contradiction");
        assert_eq!(
            error,
            QualificationValidationError::Structural {
                reason: "contradiction".into(),
            }
        );
        assert_eq!(error.to_string(), "contradiction");
    }
}
