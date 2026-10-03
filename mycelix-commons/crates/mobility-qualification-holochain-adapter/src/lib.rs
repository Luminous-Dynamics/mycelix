//! Concrete Holochain 0.7 adapter for the mobility qualification boundary.
//!
//! The pure qualification crate remains the sole owner of semantic meaning.
//! This crate owns only:
//! - the concrete ActionHash / EntryHash address representation;
//! - the binding-time address-kind check;
//! - dispatch to the matching deterministic must_get_* host function.
//!
//! An IdentityRef is never parsed as or converted into a Holochain hash.
//! A logical identity must already have an explicit runtime binding before
//! this adapter can enter the Holochain retrieval phase.

use hdi::prelude::*;
use mobility_configuration_qualification::{
    IdentityRef, QualificationDecision, QualificationDependencyBindingProvenance,
    QualificationDependencyBindingSet, QualificationDependencyRetrievalKind,
    QualificationValidationError,
};

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum HolochainDependencyAddress {
    Action(ActionHash),
    Entry(EntryHash),
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum HolochainAdapterBoundaryError {
    SemanticInvalid { reason: String },
    BindingRejected { reason: String },
    AddressKindMismatch {
        retrieval: QualificationDependencyRetrievalKind,
        expected: &'static str,
        actual: &'static str,
    },
    LogicalDependencyNotBound { missing: Vec<IdentityRef> },
}

impl std::fmt::Display for HolochainAdapterBoundaryError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::SemanticInvalid { reason } => write!(f, "semantic invalidity: {reason}"),
            Self::BindingRejected { reason } => write!(f, "binding rejected: {reason}"),
            Self::AddressKindMismatch {
                retrieval,
                expected,
                actual,
            } => write!(
                f,
                "retrieval kind {retrieval:?} requires {expected}, but binding supplied {actual}"
            ),
            Self::LogicalDependencyNotBound { missing } => write!(
                f,
                "logical dependencies are not runtime-bound: {missing:?}; do not enter Holochain validation until the address bindings exist"
            ),
        }
    }
}

impl std::error::Error for HolochainAdapterBoundaryError {}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ResolvedHolochainDependency {
    identity: IdentityRef,
    address: HolochainDependencyAddress,
    retrieval: QualificationDependencyRetrievalKind,
    provenance: QualificationDependencyBindingProvenance,
}

impl ResolvedHolochainDependency {
    pub fn identity(&self) -> &IdentityRef {
        &self.identity
    }

    pub fn address(&self) -> &HolochainDependencyAddress {
        &self.address
    }

    pub fn retrieval(&self) -> QualificationDependencyRetrievalKind {
        self.retrieval
    }

    pub fn provenance(&self) -> &QualificationDependencyBindingProvenance {
        &self.provenance
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct HolochainDependencyBindingSet {
    inner: QualificationDependencyBindingSet<HolochainDependencyAddress>,
    provenance: std::collections::BTreeMap<IdentityRef, QualificationDependencyBindingProvenance>,
}

impl HolochainDependencyBindingSet {
    pub fn new() -> Self {
        Self::default()
    }

    /// Bind exactly one logical identity to one Holochain address and require
    /// an explicit provenance witness for why that logical dependency may use
    /// this runtime address.
    ///
    /// The witness is validated entirely in the pure qualification layer and
    /// must name this exact logical identity. The protocol address remains
    /// runtime-specific and is never part of the pure witness.
    pub fn bind(
        &mut self,
        identity: IdentityRef,
        address: HolochainDependencyAddress,
        retrieval: QualificationDependencyRetrievalKind,
        provenance: QualificationDependencyBindingProvenance,
    ) -> Result<(), HolochainAdapterBoundaryError> {
        provenance
            .validate()
            .map_err(HolochainAdapterBoundaryError::SemanticInvalid)?;

        if !provenance.matches_logical_identity(&identity) {
            return Err(HolochainAdapterBoundaryError::SemanticInvalid {
                reason: "binding provenance witness does not name the exact bound logical identity"
                    .into(),
            });
        }

        identity
            .validate()
            .map_err(|reason| HolochainAdapterBoundaryError::SemanticInvalid { reason })?;

        validate_address_kind(&address, retrieval)?;

        self.inner
            .insert(identity.clone(), address, retrieval)
            .map_err(|error| match error {
                QualificationValidationError::Structural { reason } => {
                    HolochainAdapterBoundaryError::BindingRejected { reason }
                }
            })?;

        self.provenance.insert(identity, provenance);
        Ok(())
    }

    /// Resolve logical identities using the pure three-outcome algebra.
    ///
    /// No Holochain retrieval occurs here. An Unresolved result means that a
    /// valid logical identity still has no explicit runtime address binding.
    /// That state must remain unresolved at the pure layer.
    pub fn resolve_required<I>(
        &self,
        identities: I,
    ) -> QualificationDecision<Vec<ResolvedHolochainDependency>>
    where
        I: IntoIterator<Item = IdentityRef>,
    {
        match self.inner.resolve_required(identities) {
            QualificationDecision::Valid(bound) => QualificationDecision::Valid(
                bound
                    .into_iter()
                    .map(|(identity, binding)| ResolvedHolochainDependency {
                        identity: identity.clone(),
                        address: binding.address.clone(),
                        retrieval: binding.retrieval,
                        provenance: self
                            .provenance
                            .get(identity)
                            .expect("every binding carries explicit provenance")
                            .clone(),
                    })
                    .collect(),
            ),
            QualificationDecision::Invalid { reason } => QualificationDecision::Invalid { reason },
            QualificationDecision::Unresolved { missing, partial } => {
                QualificationDecision::Unresolved {
                    missing,
                    partial: partial
                        .into_iter()
                        .map(|(identity, binding)| ResolvedHolochainDependency {
                            identity: identity.clone(),
                            address: binding.address.clone(),
                            retrieval: binding.retrieval,
                            provenance: self
                                .provenance
                                .get(identity)
                                .expect("every binding carries explicit provenance")
                                .clone(),
                        })
                        .collect(),
                }
            }
        }
    }

    pub fn len(&self) -> usize {
        self.inner.len()
    }

    pub fn is_empty(&self) -> bool {
        self.inner.is_empty()
    }

    pub fn provenance_for(
        &self,
        identity: &IdentityRef,
    ) -> Option<&QualificationDependencyBindingProvenance> {
        self.provenance.get(identity)
    }
}

/// Retrieved value returned by the concrete Holochain host primitive.
#[derive(Debug)]
pub enum HolochainRetrievedDependency {
    ValidRecord(Record),
    Action(SignedActionHashed),
    Entry(EntryHashed),
}

/// Retrieve one already-resolved dependency using the exact host primitive
/// selected by its immutable binding.
///
/// Missing addressable data is deliberately propagated as the raw Holochain
/// host outcome. In a validate callback Holochain turns that missing dependency
/// into UnresolvedDependencies rather than semantic Invalid.
pub fn retrieve_one(
    dependency: &ResolvedHolochainDependency,
) -> ExternResult<HolochainRetrievedDependency> {
    match (&dependency.address, dependency.retrieval) {
        (
            HolochainDependencyAddress::Action(action_hash),
            QualificationDependencyRetrievalKind::ValidRecord,
        ) => Ok(HolochainRetrievedDependency::ValidRecord(
            must_get_valid_record(action_hash.clone())?,
        )),
        (
            HolochainDependencyAddress::Action(action_hash),
            QualificationDependencyRetrievalKind::Action,
        ) => Ok(HolochainRetrievedDependency::Action(
            must_get_action(action_hash.clone())?,
        )),
        (
            HolochainDependencyAddress::Entry(entry_hash),
            QualificationDependencyRetrievalKind::Entry,
        ) => Ok(HolochainRetrievedDependency::Entry(
            must_get_entry(entry_hash.clone())?,
        )),
        _ => unreachable!("HolochainDependencyBindingSet prevents address-kind mismatch"),
    }
}

/// Retrieve a fully resolved dependency set in canonical logical-identity order.
///
/// Call this only after resolve_required returned Valid. An unbound logical
/// identity has no protocol address and therefore cannot be represented as a
/// Holochain unresolved-hash dependency yet.
pub fn retrieve_resolved(
    dependencies: &[ResolvedHolochainDependency],
) -> ExternResult<Vec<HolochainRetrievedDependency>> {
    dependencies.iter().map(retrieve_one).collect()
}

/// Map the pure decision or an adapter-side semantic-invalidity finding into
/// one Holochain callback result.
///
/// There is intentionally one callback-facing semantic seam:
/// - pure Valid -> Holochain Valid;
/// - pure Invalid -> Holochain Invalid;
/// - an adapter-discovered malformed logical identity -> Holochain Invalid;
/// - pure Unresolved -> preflight boundary error until every logical dependency
///   has a concrete protocol binding.
/// True adapter contract defects remain errors instead of becoming semantic
/// findings.
pub fn finalize_callback<T>(
    result: Result<QualificationDecision<T>, HolochainAdapterBoundaryError>,
) -> Result<ValidateCallbackResult, HolochainAdapterBoundaryError> {
    match result {
        Ok(QualificationDecision::Valid(_)) => Ok(ValidateCallbackResult::Valid),
        Ok(QualificationDecision::Invalid { reason })
        | Err(HolochainAdapterBoundaryError::SemanticInvalid { reason }) => {
            Ok(ValidateCallbackResult::Invalid(reason))
        }
        Ok(QualificationDecision::Unresolved { missing, .. }) => {
            Err(HolochainAdapterBoundaryError::LogicalDependencyNotBound { missing })
        }
        Err(error) => Err(error),
    }
}

/// Require the binding preflight to be complete without introducing another
/// semantic algebra. This is a preparation boundary, not a Holochain callback
/// result conversion.
pub fn require_runtime_bindings<T>(
    decision: QualificationDecision<T>,
) -> Result<T, HolochainAdapterBoundaryError> {
    match decision {
        QualificationDecision::Valid(value) => Ok(value),
        QualificationDecision::Invalid { reason } => {
            Err(HolochainAdapterBoundaryError::SemanticInvalid { reason })
        }
        QualificationDecision::Unresolved { missing, .. } => {
            Err(HolochainAdapterBoundaryError::LogicalDependencyNotBound { missing })
        }
    }
}

fn validate_address_kind(
    address: &HolochainDependencyAddress,
    retrieval: QualificationDependencyRetrievalKind,
) -> Result<(), HolochainAdapterBoundaryError> {
    let compatible = matches!(
        (address, retrieval),
        (
            HolochainDependencyAddress::Action(_),
            QualificationDependencyRetrievalKind::ValidRecord
        ) | (
            HolochainDependencyAddress::Action(_),
            QualificationDependencyRetrievalKind::Action
        ) | (
            HolochainDependencyAddress::Entry(_),
            QualificationDependencyRetrievalKind::Entry
        )
    );

    if compatible {
        return Ok(());
    }

    let expected = match retrieval {
        QualificationDependencyRetrievalKind::ValidRecord
        | QualificationDependencyRetrievalKind::Action => "ActionHash",
        QualificationDependencyRetrievalKind::Entry => "EntryHash",
    };
    let actual = match address {
        HolochainDependencyAddress::Action(_) => "ActionHash",
        HolochainDependencyAddress::Entry(_) => "EntryHash",
    };

    Err(HolochainAdapterBoundaryError::AddressKindMismatch {
        retrieval,
        expected,
        actual,
    })
}

#[cfg(test)]
impl HolochainDependencyBindingSet {
    fn bind_for_test(
        &mut self,
        identity: IdentityRef,
        address: HolochainDependencyAddress,
        retrieval: QualificationDependencyRetrievalKind,
    ) -> Result<(), HolochainAdapterBoundaryError> {
        let authority = IdentityRef {
            kind: mobility_configuration_qualification::identity_lineage::IdentityKind::EvidenceRecord,
            namespace: "mobility".into(),
            id: format!("authority-{}", identity.id),
        };
        let provenance = QualificationDependencyBindingProvenance {
            witness_identity: IdentityRef {
                kind: mobility_configuration_qualification::identity_lineage::IdentityKind::ReconciliationWitness,
                namespace: "mobility".into(),
                id: format!("binding-witness-{}", identity.id),
            },
            logical_identity: identity.clone(),
            authority: authority.clone(),
            basis: vec![
                authority,
                IdentityRef {
                    kind: mobility_configuration_qualification::identity_lineage::IdentityKind::EvidenceRecord,
                    namespace: "mobility".into(),
                    id: format!("basis-{}", identity.id),
                },
            ],
        };
        self.bind(identity, address, retrieval, provenance)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use hdi::hdi::{set_hdi, HdiT};
    use mobility_configuration_qualification::identity_lineage::IdentityKind;
    use std::panic::{catch_unwind, AssertUnwindSafe};
    use std::sync::{Arc, Mutex, OnceLock};

    fn identity(value: &str) -> IdentityRef {
        IdentityRef {
            kind: IdentityKind::EvidenceRecord,
            namespace: "mobility".into(),
            id: value.into(),
        }
    }

    fn malformed_identity() -> IdentityRef {
        IdentityRef {
            kind: IdentityKind::EvidenceRecord,
            namespace: "holochain".into(),
            id: "not-a-native-engineering-id".into(),
        }
    }

    fn action_hash(byte: u8) -> ActionHash {
        ActionHash::from_raw_36(vec![byte; 36])
    }

    fn entry_hash(byte: u8) -> EntryHash {
        EntryHash::from_raw_36(vec![byte; 36])
    }

    #[test]
    fn valid_record_accepts_only_action_hash() {
        let mut bindings = HolochainDependencyBindingSet::new();
        assert!(bindings
            .bind_for_test(
                identity("record"),
                HolochainDependencyAddress::Action(action_hash(1)),
                QualificationDependencyRetrievalKind::ValidRecord,
            )
            .is_ok());
        assert_eq!(bindings.len(), 1);
    }

    #[test]
    fn action_accepts_only_action_hash() {
        let mut bindings = HolochainDependencyBindingSet::new();
        assert!(bindings
            .bind_for_test(
                identity("action"),
                HolochainDependencyAddress::Action(action_hash(2)),
                QualificationDependencyRetrievalKind::Action,
            )
            .is_ok());
    }

    #[test]
    fn entry_accepts_only_entry_hash() {
        let mut bindings = HolochainDependencyBindingSet::new();
        assert!(bindings
            .bind_for_test(
                identity("entry"),
                HolochainDependencyAddress::Entry(entry_hash(3)),
                QualificationDependencyRetrievalKind::Entry,
            )
            .is_ok());
    }

    #[test]
    fn wrong_address_kind_is_rejected_before_host_call() {
        let mut bindings = HolochainDependencyBindingSet::new();
        let error = bindings
            .bind_for_test(
                identity("entry"),
                HolochainDependencyAddress::Action(action_hash(4)),
                QualificationDependencyRetrievalKind::Entry,
            )
            .expect_err("Entry retrieval must reject ActionHash");

        assert!(matches!(
            error,
            HolochainAdapterBoundaryError::AddressKindMismatch {
                retrieval: QualificationDependencyRetrievalKind::Entry,
                expected: "EntryHash",
                actual: "ActionHash",
            }
        ));
        assert!(bindings.is_empty());
    }

    #[test]
    fn wrong_address_kind_for_valid_record_is_rejected_before_host_call() {
        let mut bindings = HolochainDependencyBindingSet::new();
        let error = bindings
            .bind_for_test(
                identity("record"),
                HolochainDependencyAddress::Entry(entry_hash(5)),
                QualificationDependencyRetrievalKind::ValidRecord,
            )
            .expect_err("ValidRecord must reject EntryHash");

        assert!(matches!(
            error,
            HolochainAdapterBoundaryError::AddressKindMismatch {
                retrieval: QualificationDependencyRetrievalKind::ValidRecord,
                expected: "ActionHash",
                actual: "EntryHash",
            }
        ));
        assert!(bindings.is_empty());
    }

    #[test]
    fn rebinding_is_rejected_by_the_pure_binding_set() {
        let mut bindings = HolochainDependencyBindingSet::new();
        let id = identity("same");

        bindings
            .bind_for_test(
                id.clone(),
                HolochainDependencyAddress::Action(action_hash(6)),
                QualificationDependencyRetrievalKind::Action,
            )
            .unwrap();

        let error = bindings
            .bind_for_test(
                id,
                HolochainDependencyAddress::Action(action_hash(7)),
                QualificationDependencyRetrievalKind::Action,
            )
            .expect_err("logical identity rebinding must remain forbidden");

        assert!(matches!(
            error,
            HolochainAdapterBoundaryError::BindingRejected { .. }
        ));
        assert_eq!(bindings.len(), 1);
    }

    #[test]
    fn duplicate_binding_remains_an_adapter_contract_failure() {
        let mut bindings = HolochainDependencyBindingSet::new();
        let identity = identity("duplicate");

        bindings
            .bind_for_test(
                identity.clone(),
                HolochainDependencyAddress::Action(action_hash(30)),
                QualificationDependencyRetrievalKind::Action,
            )
            .unwrap();

        let error = bindings
            .bind_for_test(
                identity,
                HolochainDependencyAddress::Action(action_hash(31)),
                QualificationDependencyRetrievalKind::Action,
            )
            .expect_err("rebinding must be rejected");

        assert!(matches!(
            error,
            HolochainAdapterBoundaryError::BindingRejected { .. }
        ));
    }


    #[test]
    fn malformed_identity_precedes_address_kind_mismatch() {
        let mut bindings = HolochainDependencyBindingSet::new();

        let error = bindings
            .bind_for_test(
                malformed_identity(),
                HolochainDependencyAddress::Entry(entry_hash(8)),
                QualificationDependencyRetrievalKind::ValidRecord,
            )
            .expect_err("malformed logical identity must win over adapter kind mismatch");

        assert!(matches!(
            error,
            HolochainAdapterBoundaryError::SemanticInvalid { .. }
        ));
        assert!(bindings.is_empty());
    }

    #[test]
    fn malformed_identity_remains_a_structural_binding_failure() {
        let mut bindings = HolochainDependencyBindingSet::new();
        let error = bindings
            .bind_for_test(
                malformed_identity(),
                HolochainDependencyAddress::Action(action_hash(8)),
                QualificationDependencyRetrievalKind::Action,
            )
            .expect_err("malformed logical identity must never become a binding");

        assert!(matches!(
            error,
            HolochainAdapterBoundaryError::SemanticInvalid { .. }
        ));
    }

    #[test]
    fn one_callback_seam_preserves_semantic_and_adapter_error_classes() {
        assert_eq!(
            finalize_callback(Ok(QualificationDecision::<()>::Valid(())))
                .expect("valid maps directly"),
            ValidateCallbackResult::Valid
        );

        assert_eq!(
            finalize_callback(Ok(QualificationDecision::<()>::Invalid {
                reason: "structural contradiction".into(),
            }))
            .expect("invalid maps directly"),
            ValidateCallbackResult::Invalid("structural contradiction".into())
        );

        let missing = identity("missing");
        assert!(matches!(
            finalize_callback(Ok(QualificationDecision::<()>::Unresolved {
                missing: vec![missing.clone()],
                partial: (),
            })),
            Err(HolochainAdapterBoundaryError::LogicalDependencyNotBound { missing: found })
                if found == vec![missing]
        ));

        assert_eq!(
            finalize_callback::<()>(Err(HolochainAdapterBoundaryError::SemanticInvalid {
                reason: "malformed identity".into(),
            }))
            .expect("semantic invalidity discovered during binding maps to Invalid"),
            ValidateCallbackResult::Invalid("malformed identity".into())
        );

        let boundary = HolochainAdapterBoundaryError::BindingRejected {
            reason: "duplicate binding".into(),
        };
        assert_eq!(
            finalize_callback::<()>(Err(boundary.clone())),
            Err(boundary),
            "adapter contract defects remain errors"
        );
    }


    #[test]
    fn resolve_preserves_the_pure_three_outcome_algebra() {
        let mut bindings = HolochainDependencyBindingSet::new();
        let bound = identity("bound");
        bindings
            .bind_for_test(
                bound.clone(),
                HolochainDependencyAddress::Action(action_hash(9)),
                QualificationDependencyRetrievalKind::Action,
            )
            .unwrap();

        assert!(matches!(
            bindings.resolve_required(vec![bound]),
            QualificationDecision::Valid(_)
        ));
        assert!(matches!(
            bindings.resolve_required(vec![identity("missing")]),
            QualificationDecision::Unresolved { .. }
        ));
        assert!(matches!(
            bindings.resolve_required(vec![malformed_identity()]),
            QualificationDecision::Invalid { .. }
        ));
    }

    #[test]
    fn unresolved_partial_bindings_preserve_provenance() {
        let mut bindings = HolochainDependencyBindingSet::new();
        let bound = identity("bound-partial");
        bindings
            .bind_for_test(
                bound.clone(),
                HolochainDependencyAddress::Action(action_hash(12)),
                QualificationDependencyRetrievalKind::Action,
            )
            .unwrap();

        let QualificationDecision::Unresolved { partial, .. } =
            bindings.resolve_required(vec![bound.clone(), identity("missing-partial")])
        else {
            panic!("one missing dependency should preserve the bound partial");
        };

        assert_eq!(partial.len(), 1);
        assert_eq!(partial[0].identity(), &bound);
        assert_eq!(partial[0].provenance().logical_identity, bound);
    }

    #[test]
    fn resolution_is_invariant_to_request_order() {
        let mut bindings = HolochainDependencyBindingSet::new();
        for (index, value) in ["b", "a", "c"].into_iter().enumerate() {
            bindings
                .bind_for_test(
                    identity(value),
                    HolochainDependencyAddress::Action(action_hash(index as u8 + 10)),
                    QualificationDependencyRetrievalKind::Action,
                )
                .unwrap();
        }

        let left = bindings.resolve_required(vec![identity("c"), identity("a"), identity("b")]);
        let right = bindings.resolve_required(vec![identity("b"), identity("c"), identity("a")]);
        assert_eq!(left, right);
    }

    #[test]
    fn valid_record_dispatches_to_must_get_valid_record() {
        assert_host_dispatch(
            QualificationDependencyRetrievalKind::ValidRecord,
            HolochainDependencyAddress::Action(action_hash(20)),
            "must_get_valid_record",
        );
    }

    #[test]
    fn action_dispatches_to_must_get_action() {
        assert_host_dispatch(
            QualificationDependencyRetrievalKind::Action,
            HolochainDependencyAddress::Action(action_hash(21)),
            "must_get_action",
        );
    }

    #[test]
    fn entry_dispatches_to_must_get_entry() {
        assert_host_dispatch(
            QualificationDependencyRetrievalKind::Entry,
            HolochainDependencyAddress::Entry(entry_hash(22)),
            "must_get_entry",
        );
    }

    fn assert_host_dispatch(
        retrieval: QualificationDependencyRetrievalKind,
        address: HolochainDependencyAddress,
        expected_call: &'static str,
    ) {
        static HOST_TEST_LOCK: OnceLock<Mutex<()>> = OnceLock::new();
        let _guard = HOST_TEST_LOCK
            .get_or_init(|| Mutex::new(()))
            .lock()
            .expect("HDI test lock is not poisoned");

        let mut bindings = HolochainDependencyBindingSet::new();
        bindings
            .bind_for_test(identity("dispatch"), address, retrieval)
            .unwrap();

        let QualificationDecision::Valid(mut resolved) =
            bindings.resolve_required(vec![identity("dispatch")])
        else {
            panic!("binding should resolve");
        };

        let calls = Arc::new(Mutex::new(Vec::new()));
        let _ = set_hdi(RecordingHdi {
            calls: Arc::clone(&calls),
        });

        let result =
            catch_unwind(AssertUnwindSafe(|| retrieve_one(&resolved.remove(0))));

        let _ = set_hdi(ErrHdi);

        assert!(result.is_err(), "recording HDI intentionally stops the call");
        assert_eq!(calls.lock().unwrap().as_slice(), [expected_call]);
    }

    struct RecordingHdi {
        calls: Arc<Mutex<Vec<&'static str>>>,
    }

    impl RecordingHdi {
        fn record(&self, call: &'static str) {
            self.calls.lock().unwrap().push(call);
            panic!("test stop after recording {call}");
        }
    }

    impl HdiT for RecordingHdi {
        fn verify_signature(&self, _: VerifySignature) -> ExternResult<bool> { unimplemented!() }
        fn must_get_entry(&self, _: MustGetEntryInput) -> ExternResult<EntryHashed> {
            self.record("must_get_entry");
        }
        fn must_get_action(&self, _: MustGetActionInput) -> ExternResult<SignedActionHashed> {
            self.record("must_get_action");
        }
        fn must_get_valid_record(&self, _: MustGetValidRecordInput) -> ExternResult<Record> {
            self.record("must_get_valid_record");
        }
        fn must_get_agent_activity(
            &self,
            _: MustGetAgentActivityInput,
        ) -> ExternResult<Vec<AgentActivity>> { unimplemented!() }
        fn dna_info(&self, _: ()) -> ExternResult<DnaInfo> { unimplemented!() }
        fn zome_info(&self, _: ()) -> ExternResult<ZomeInfo> { unimplemented!() }
        fn trace(&self, _: TraceMsg) -> ExternResult<()> { unimplemented!() }
        fn x_salsa20_poly1305_decrypt(
            &self,
            _: XSalsa20Poly1305Decrypt,
        ) -> ExternResult<Option<XSalsa20Poly1305Data>> { unimplemented!() }
        fn x_25519_x_salsa20_poly1305_decrypt(
            &self,
            _: X25519XSalsa20Poly1305Decrypt,
        ) -> ExternResult<Option<XSalsa20Poly1305Data>> { unimplemented!() }
        fn ed_25519_x_salsa20_poly1305_decrypt(
            &self,
            _: Ed25519XSalsa20Poly1305Decrypt,
        ) -> ExternResult<XSalsa20Poly1305Data> { unimplemented!() }
    }
}
