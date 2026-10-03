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

#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum HolochainDependencyAddress {
    Action(ActionHash),
    Entry(EntryHash),
}

#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize)]
pub struct HolochainAuthorityAgentBindingPayload {
    pub schema: &'static str,
    pub authority: IdentityRef,
    pub provenance: QualificationDependencyBindingProvenance,
    pub agent: AgentPubKey,
}

pub const HOLOCHAIN_AUTHORITY_AGENT_BINDING_SCHEMA: &str =
    "mycelix.mobility.holochain_authority_agent_binding.v1";

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SignedHolochainAuthorityAgentBinding {
    pub issuer: AgentPubKey,
    pub signature: Signature,
    pub payload: HolochainAuthorityAgentBindingPayload,
}

#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct HolochainAuthorityAgentBindingSet {
    bindings: std::collections::BTreeMap<IdentityRef, AgentPubKey>,
}

impl HolochainAuthorityAgentBindingSet {
    pub fn new() -> Self {
        Self::default()
    }

    /// Accept exactly one verified protocol agent for each domain authority.
    ///
    /// The credential is validated and signature-checked before the immutable
    /// authority→AgentPubKey relation is stored.
    pub fn bind_attested(
        &mut self,
        credential: SignedHolochainAuthorityAgentBinding,
    ) -> ExternResult<Result<(), HolochainAdapterBoundaryError>> {
        match credential.verify()? {
            HolochainAuthorityAgentBindingVerification::Invalid { reason } => {
                return Ok(Err(HolochainAdapterBoundaryError::SemanticInvalid { reason }));
            }
            HolochainAuthorityAgentBindingVerification::Valid => {}
        }

        let authority = credential.payload.authority.clone();
        let agent = credential.payload.agent.clone();

        if self.bindings.contains_key(&authority) {
            return Ok(Err(HolochainAdapterBoundaryError::BindingRejected {
                reason: "an authority identity may be bound to only one AgentPubKey in an immutable binding set"
                    .into(),
            }));
        }

        self.bindings.insert(authority, agent);
        Ok(Ok(()))
    }

    pub fn agent_for(&self, authority: &IdentityRef) -> Option<&AgentPubKey> {
        self.bindings.get(authority)
    }

    pub fn len(&self) -> usize {
        self.bindings.len()
    }

    pub fn is_empty(&self) -> bool {
        self.bindings.is_empty()
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum HolochainAuthorityAgentBindingVerification {
    Valid,
    Invalid { reason: String },
}

impl HolochainAuthorityAgentBindingPayload {
    pub fn validate(&self) -> Result<(), HolochainAdapterBoundaryError> {
        if self.schema != HOLOCHAIN_AUTHORITY_AGENT_BINDING_SCHEMA {
            return Err(HolochainAdapterBoundaryError::SemanticInvalid {
                reason: "authority-agent binding uses an unexpected schema".into(),
            });
        }

        self.authority
            .validate()
            .map_err(|reason| HolochainAdapterBoundaryError::SemanticInvalid { reason })?;

        if self.provenance.logical_identity != self.authority {
            return Err(HolochainAdapterBoundaryError::SemanticInvalid {
                reason: "authority-agent provenance must name the exact authority identity".into(),
            });
        }

        self.provenance
            .validate()
            .map_err(HolochainAdapterBoundaryError::SemanticInvalid)?;

        Ok(())
    }
}

impl SignedHolochainAuthorityAgentBinding {
    pub fn verify(&self) -> ExternResult<HolochainAuthorityAgentBindingVerification> {
        if let Err(error) = self.payload.validate() {
            return Ok(HolochainAuthorityAgentBindingVerification::Invalid {
                reason: error.to_string(),
            });
        }

        if self.issuer != self.payload.agent {
            return Ok(HolochainAuthorityAgentBindingVerification::Invalid {
                reason: "authority-agent binding issuer must equal the bound agent key".into(),
            });
        }

        let verified = hdi::ed25519::verify_signature(
            self.issuer.clone(),
            self.signature.clone(),
            &self.payload,
        )?;

        if verified {
            Ok(HolochainAuthorityAgentBindingVerification::Valid)
        } else {
            Ok(HolochainAuthorityAgentBindingVerification::Invalid {
                reason: "authority-agent binding signature did not verify".into(),
            })
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize)]
pub struct HolochainBindingAttestationPayload {
    pub schema: &'static str,
    pub provenance: QualificationDependencyBindingProvenance,
    pub address: HolochainDependencyAddress,
    pub retrieval: QualificationDependencyRetrievalKind,
}

pub const HOLOCHAIN_BINDING_ATTESTATION_SCHEMA: &str =
    "mycelix.mobility.holochain_binding_attestation.v1";

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SignedHolochainBindingAttestation {
    pub signer: AgentPubKey,
    pub signature: Signature,
    pub payload: HolochainBindingAttestationPayload,
}

impl HolochainBindingAttestationPayload {
    pub fn validate(&self) -> Result<(), HolochainAdapterBoundaryError> {
        if self.schema != HOLOCHAIN_BINDING_ATTESTATION_SCHEMA {
            return Err(HolochainAdapterBoundaryError::SemanticInvalid {
                reason: "binding attestation uses an unexpected schema".into(),
            });
        }

        self.provenance
            .validate()
            .map_err(HolochainAdapterBoundaryError::SemanticInvalid)?;

        validate_address_kind(&self.address, self.retrieval)?;
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum HolochainBindingAttestationVerification {
    Valid,
    Invalid { reason: String },
}

impl SignedHolochainBindingAttestation {
    pub fn verify(&self) -> ExternResult<HolochainBindingAttestationVerification> {
        if let Err(error) = self.payload.validate() {
            return Ok(HolochainBindingAttestationVerification::Invalid {
                reason: error.to_string(),
            });
        }

        let verified = hdi::ed25519::verify_signature(
            self.signer.clone(),
            self.signature.clone(),
            &self.payload,
        )?;

        if verified {
            Ok(HolochainBindingAttestationVerification::Valid)
        } else {
            Ok(HolochainBindingAttestationVerification::Invalid {
                reason: "binding attestation signature did not verify".into(),
            })
        }
    }
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

        if self
            .provenance
            .values()
            .any(|existing| existing.witness_identity == provenance.witness_identity)
        {
            return Err(HolochainAdapterBoundaryError::BindingRejected {
                reason: "a provenance witness identity may justify only one runtime binding"
                    .into(),
            });
        }

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

    /// Bind an attested runtime dependency only when the authority identity
    /// has an explicit protocol-agent binding credential.
    ///
    /// The credential is deliberately self-authored by the bound agent key:
    /// this proves control of the key for the exact authority statement, while
    /// the pure provenance witness remains responsible for domain semantics.
    pub fn bind_attested_with_authority(
        &mut self,
        binding: SignedHolochainBindingAttestation,
        authority_bindings: &HolochainAuthorityAgentBindingSet,
    ) -> ExternResult<Result<(), HolochainAdapterBoundaryError>> {
        let authority = binding.payload.provenance.authority.clone();
        let Some(authorized_agent) = authority_bindings.agent_for(&authority) else {
            return Ok(Err(HolochainAdapterBoundaryError::LogicalDependencyNotBound {
                missing: vec![authority],
            }));
        };

        if binding.signer != *authorized_agent {
            return Ok(Err(HolochainAdapterBoundaryError::SemanticInvalid {
                reason: "runtime binding signer does not match the registered authority agent key"
                    .into(),
            }));
        }

        match binding.verify()? {
            HolochainBindingAttestationVerification::Invalid { reason } => {
                return Ok(Err(HolochainAdapterBoundaryError::SemanticInvalid { reason }));
            }
            HolochainBindingAttestationVerification::Valid => {}
        }

        Ok(self.bind(
            binding.payload.provenance.logical_identity.clone(),
            binding.payload.address,
            binding.payload.retrieval,
            binding.payload.provenance,
        ))
    }


    /// Bind from a cryptographically attested payload.
    ///
    /// The signature is verified over the canonical payload before the binding
    /// enters the immutable set. A valid signature proves that the payload was
    /// signed by the supplied Holochain agent key; it does not, by itself,
    /// prove that the signer is the real-world authority named in the provenance.
    pub fn bind_attested(
        &mut self,
        attestation: SignedHolochainBindingAttestation,
    ) -> ExternResult<Result<(), HolochainAdapterBoundaryError>> {
        if let Err(reason) = attestation.payload.validate() {
            return Ok(Err(HolochainAdapterBoundaryError::SemanticInvalid { reason: reason.to_string() }));
        }

        match attestation.verify()? {
            HolochainBindingAttestationVerification::Invalid { reason } => {
                return Ok(Err(HolochainAdapterBoundaryError::SemanticInvalid { reason }));
            }
            HolochainBindingAttestationVerification::Valid => {}
        }

        Ok(self.bind(
            attestation.payload.provenance.logical_identity.clone(),
            attestation.payload.address,
            attestation.payload.retrieval,
            attestation.payload.provenance,
        ))
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

/// Verify the optional cryptographic attestation for one logical-to-protocol
/// binding. Verification is deterministic and signs the canonical serialized
/// payload, including the schema, provenance witness, protocol address, and
/// retrieval intent.
pub fn verify_binding_attestation(
    attestation: &SignedHolochainBindingAttestation,
) -> ExternResult<HolochainBindingAttestationVerification> {
    attestation.verify()
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
        let authority_scope = IdentityRef {
            kind: mobility_configuration_qualification::identity_lineage::IdentityKind::ReconciliationWitness,
            namespace: "mobility".into(),
            id: format!("authority-scope-{}", identity.id),
        };
        let authority_delegation = IdentityRef {
            kind: mobility_configuration_qualification::identity_lineage::IdentityKind::ReconciliationWitness,
            namespace: "mobility".into(),
            id: format!("authority-delegation-{}", identity.id),
        };
        let provenance = QualificationDependencyBindingProvenance {
            witness_identity: IdentityRef {
                kind: mobility_configuration_qualification::identity_lineage::IdentityKind::ReconciliationWitness,
                namespace: "mobility".into(),
                id: format!("binding-witness-{}", identity.id),
            },
            logical_identity: identity.clone(),
            authority: authority.clone(),
            authority_scope: authority_scope.clone(),
            authority_delegation: authority_delegation.clone(),
            basis: vec![
                authority,
                authority_scope,
                authority_delegation,
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

    fn host_test_lock() -> &'static Mutex<()> {
        static LOCK: OnceLock<Mutex<()>> = OnceLock::new();
        LOCK.get_or_init(|| Mutex::new(()))
    }

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
    fn duplicate_provenance_witness_is_an_adapter_contract_failure() {
        let mut bindings = HolochainDependencyBindingSet::new();
        let first = identity("first");
        let second = identity("second");
        let authority = identity("shared-authority");
        let authority_scope = IdentityRef {
            kind: IdentityKind::ReconciliationWitness,
            namespace: "mobility".into(),
            id: "shared-authority-scope".into(),
        };
        let authority_delegation = IdentityRef {
            kind: IdentityKind::ReconciliationWitness,
            namespace: "mobility".into(),
            id: "shared-authority-delegation".into(),
        };
        let provenance = QualificationDependencyBindingProvenance {
            witness_identity: IdentityRef {
                kind: IdentityKind::ReconciliationWitness,
                namespace: "mobility".into(),
                id: "shared-binding-witness".into(),
            },
            logical_identity: first.clone(),
            authority: authority.clone(),
            authority_scope: authority_scope.clone(),
            authority_delegation: authority_delegation.clone(),
            basis: vec![
                authority.clone(),
                authority_scope,
                authority_delegation,
                identity("basis-first"),
            ],
        };

        bindings
            .bind(
                first,
                HolochainDependencyAddress::Action(action_hash(13)),
                QualificationDependencyRetrievalKind::Action,
                provenance.clone(),
            )
            .unwrap();

        let second_provenance = QualificationDependencyBindingProvenance {
            logical_identity: second.clone(),
            authority,
            basis: vec![identity("shared-authority"), identity("basis-second")],
            ..provenance
        };

        let error = bindings
            .bind(
                second,
                HolochainDependencyAddress::Action(action_hash(14)),
                QualificationDependencyRetrievalKind::Action,
                second_provenance,
            )
            .expect_err("one provenance witness must not justify two bindings");

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
    fn signed_binding_attestation_dispatches_to_verify_signature() {
        let provenance = QualificationDependencyBindingProvenance {
            witness_identity: IdentityRef {
                kind: IdentityKind::ReconciliationWitness,
                namespace: "mobility".into(),
                id: "signed-binding-witness".into(),
            },
            logical_identity: identity("signed"),
            authority: identity("signed-authority"),
            authority_scope: IdentityRef {
                kind: IdentityKind::ReconciliationWitness,
                namespace: "mobility".into(),
                id: "signed-authority-scope".into(),
            },
            authority_delegation: IdentityRef {
                kind: IdentityKind::ReconciliationWitness,
                namespace: "mobility".into(),
                id: "signed-authority-delegation".into(),
            },
            basis: vec![
                identity("signed-authority"),
                IdentityRef {
                    kind: IdentityKind::ReconciliationWitness,
                    namespace: "mobility".into(),
                    id: "signed-authority-scope".into(),
                },
                IdentityRef {
                    kind: IdentityKind::ReconciliationWitness,
                    namespace: "mobility".into(),
                    id: "signed-authority-delegation".into(),
                },
                identity("signed-basis"),
            ],
        };

        let attestation = SignedHolochainBindingAttestation {
            signer: AgentPubKey::from_raw_36(vec![23u8; 36]),
            signature: Signature([0u8; 64]),
            payload: HolochainBindingAttestationPayload {
                schema: HOLOCHAIN_BINDING_ATTESTATION_SCHEMA,
                provenance,
                address: HolochainDependencyAddress::Action(action_hash(23)),
                retrieval: QualificationDependencyRetrievalKind::Action,
            },
        };

        let _guard = host_test_lock().lock().expect("HDI test lock is not poisoned");
        let calls = Arc::new(Mutex::new(Vec::new()));
        let _previous = set_hdi(RecordingHdi {
            calls: Arc::clone(&calls),
            verify_result: true,
        });
        let result = verify_binding_attestation(&attestation);
        let _ = set_hdi(ErrHdi);

        assert_eq!(
            result.expect("mock signature verification should succeed"),
            HolochainBindingAttestationVerification::Valid
        );
        assert_eq!(calls.lock().unwrap().as_slice(), ["verify_signature"]);
    }

    #[test]
    fn bind_attested_rejects_an_unverified_signature() {
        let provenance = QualificationDependencyBindingProvenance {
            witness_identity: IdentityRef {
                kind: IdentityKind::ReconciliationWitness,
                namespace: "mobility".into(),
                id: "unverified-binding-witness".into(),
            },
            logical_identity: identity("unverified"),
            authority: identity("unverified-authority"),
            authority_scope: IdentityRef {
                kind: IdentityKind::ReconciliationWitness,
                namespace: "mobility".into(),
                id: "unverified-authority-scope".into(),
            },
            authority_delegation: IdentityRef {
                kind: IdentityKind::ReconciliationWitness,
                namespace: "mobility".into(),
                id: "unverified-authority-delegation".into(),
            },
            basis: vec![
                identity("unverified-authority"),
                IdentityRef {
                    kind: IdentityKind::ReconciliationWitness,
                    namespace: "mobility".into(),
                    id: "unverified-authority-scope".into(),
                },
                IdentityRef {
                    kind: IdentityKind::ReconciliationWitness,
                    namespace: "mobility".into(),
                    id: "unverified-authority-delegation".into(),
                },
                identity("unverified-basis"),
            ],
        };
        let attestation = SignedHolochainBindingAttestation {
            signer: AgentPubKey::from_raw_36(vec![25u8; 36]),
            signature: Signature([0u8; 64]),
            payload: HolochainBindingAttestationPayload {
                schema: HOLOCHAIN_BINDING_ATTESTATION_SCHEMA,
                provenance,
                address: HolochainDependencyAddress::Action(action_hash(25)),
                retrieval: QualificationDependencyRetrievalKind::Action,
            },
        };

        let _guard = host_test_lock().lock().expect("HDI test lock is not poisoned");
        let _previous = set_hdi(RecordingHdi {
            calls: Arc::new(Mutex::new(Vec::new())),
            verify_result: false,
        });

        let mut bindings = HolochainDependencyBindingSet::new();
        let result = bindings.bind_attested(attestation);
        let _ = set_hdi(ErrHdi);

        assert!(matches!(
            result,
            Ok(Err(HolochainAdapterBoundaryError::SemanticInvalid { reason }))
                if reason == "binding attestation signature did not verify"
        ));
        assert!(bindings.is_empty());
    }

    #[test]
    fn signature_host_failure_remains_an_extern_error() {
        let provenance = QualificationDependencyBindingProvenance {
            witness_identity: IdentityRef {
                kind: IdentityKind::ReconciliationWitness,
                namespace: "mobility".into(),
                id: "host-error-binding-witness".into(),
            },
            logical_identity: identity("host-error"),
            authority: identity("host-error-authority"),
            authority_scope: IdentityRef {
                kind: IdentityKind::ReconciliationWitness,
                namespace: "mobility".into(),
                id: "host-error-authority-scope".into(),
            },
            authority_delegation: IdentityRef {
                kind: IdentityKind::ReconciliationWitness,
                namespace: "mobility".into(),
                id: "host-error-authority-delegation".into(),
            },
            basis: vec![
                identity("host-error-authority"),
                IdentityRef {
                    kind: IdentityKind::ReconciliationWitness,
                    namespace: "mobility".into(),
                    id: "host-error-authority-scope".into(),
                },
                IdentityRef {
                    kind: IdentityKind::ReconciliationWitness,
                    namespace: "mobility".into(),
                    id: "host-error-authority-delegation".into(),
                },
            ],
        };

        let attestation = SignedHolochainBindingAttestation {
            signer: AgentPubKey::from_raw_36(vec![26u8; 36]),
            signature: Signature([0u8; 64]),
            payload: HolochainBindingAttestationPayload {
                schema: HOLOCHAIN_BINDING_ATTESTATION_SCHEMA,
                provenance,
                address: HolochainDependencyAddress::Action(action_hash(26)),
                retrieval: QualificationDependencyRetrievalKind::Action,
            },
        };

        let _guard = host_test_lock().lock().expect("HDI test lock is not poisoned");
        let _previous = set_hdi(ErrHdi);
        let result = verify_binding_attestation(&attestation);
        let _ = set_hdi(ErrHdi);

        assert!(result.is_err(), "host signature verification failure must remain an ExternResult error");
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
        let _guard = host_test_lock()
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
            verify_result: true,
        });

        let result =
            catch_unwind(AssertUnwindSafe(|| retrieve_one(&resolved.remove(0))));

        let _ = set_hdi(ErrHdi);

        assert!(result.is_err(), "recording HDI intentionally stops the call");
        assert_eq!(calls.lock().unwrap().as_slice(), [expected_call]);
    }

    struct RecordingHdi {
        calls: Arc<Mutex<Vec<&'static str>>>,
        verify_result: bool,
    }

    impl RecordingHdi {
        fn record(&self, call: &'static str) {
            self.calls.lock().unwrap().push(call);
            panic!("test stop after recording {call}");
        }
    }

    impl HdiT for RecordingHdi {
        fn verify_signature(&self, _: VerifySignature) -> ExternResult<bool> {
            self.calls.lock().unwrap().push("verify_signature");
            Ok(self.verify_result)
        }
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
