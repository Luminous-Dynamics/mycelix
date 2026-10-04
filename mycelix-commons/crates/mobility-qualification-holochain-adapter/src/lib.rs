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
use holochain_serialized_bytes::prelude::SerializedBytes;
use mobility_configuration_qualification::{
    IdentityRef, QualificationAuthorityAgentBindingProvenance, QualificationDecision,
    QualificationDependencyBindingProvenance, QualificationDependencyBindingSet,
    QualificationDependencyRetrievalKind, QualificationValidationError,
};

#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum HolochainDependencyAddress {
    Action(ActionHash),
    Entry(EntryHash),
}

#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize, serde::Deserialize, SerializedBytes)]
#[serde(deny_unknown_fields)]
pub struct HolochainAuthorityAgentBindingPayload {
    pub schema: String,
    pub provenance: QualificationAuthorityAgentBindingProvenance,
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
    bindings: std::collections::BTreeMap<IdentityRef, SignedHolochainAuthorityAgentBinding>,
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
                return Ok(Err(HolochainAdapterBoundaryError::SemanticInvalid {
                    reason,
                }));
            }
            HolochainAuthorityAgentBindingVerification::Valid => {}
        }

        let authority = credential.payload.provenance.authority.clone();

        if self.bindings.contains_key(&authority) {
            return Ok(Err(HolochainAdapterBoundaryError::BindingRejected {
                reason:
                    "an authority identity may be bound to only one AgentPubKey in an immutable binding set"
                        .into(),
            }));
        }

        if self.bindings.values().any(|existing| {
            existing.payload.provenance.witness_identity
                == credential.payload.provenance.witness_identity
        }) {
            return Ok(Err(HolochainAdapterBoundaryError::BindingRejected {
                reason:
                    "an authority-agent provenance witness may justify only one registry binding"
                        .into(),
            }));
        }

        self.bindings.insert(authority, credential);
        Ok(Ok(()))
    }

    pub fn agent_for(&self, authority: &IdentityRef) -> Option<&AgentPubKey> {
        self.bindings
            .get(authority)
            .map(|credential| &credential.payload.agent)
    }

    /// Return the fully verified authority-agent credential retained for auditability.
    ///
    /// The registry stores the credential that was admitted, not only the resulting
    /// AgentPubKey, so downstream consumers can inspect the exact signed provenance
    /// statement that established the protocol identity binding.
    pub fn credential_for(
        &self,
        authority: &IdentityRef,
    ) -> Option<&SignedHolochainAuthorityAgentBinding> {
        self.bindings.get(authority)
    }

    /// Return whether a provenance witness has already been admitted by the
    /// authority-agent registry.
    pub fn contains_provenance_witness(&self, witness_identity: &IdentityRef) -> bool {
        self.bindings
            .values()
            .any(|credential| &credential.payload.provenance.witness_identity == witness_identity)
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

#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize, serde::Deserialize, SerializedBytes)]
#[serde(deny_unknown_fields)]
pub struct HolochainBindingAttestationPayload {
    pub schema: String,
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
    SemanticInvalid {
        reason: String,
    },
    BindingRejected {
        reason: String,
    },
    AddressKindMismatch {
        retrieval: QualificationDependencyRetrievalKind,
        expected: &'static str,
        actual: &'static str,
    },
    LogicalDependencyNotBound {
        missing: Vec<IdentityRef>,
    },
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

        // Validate candidate-local protocol typing before consulting immutable
        // binding state, so an invalid address/type pairing cannot be masked by
        // a duplicate witness or logical-identity conflict.
        validate_address_kind(&address, retrieval)?;

        if self
            .provenance
            .values()
            .any(|existing| existing.witness_identity == provenance.witness_identity)
        {
            return Err(HolochainAdapterBoundaryError::BindingRejected {
                reason: "a provenance witness identity may justify only one runtime binding".into(),
            });
        }

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
        if let Err(error) = binding.payload.validate() {
            return Ok(Err(error));
        }

        // Signature validity is a definitive local protocol check. It must
        // precede authority-agent dependency preflight so a bad attestation
        // cannot be masked as an unresolved domain-authority dependency.
        match binding.verify()? {
            HolochainBindingAttestationVerification::Invalid { reason } => {
                return Ok(Err(HolochainAdapterBoundaryError::SemanticInvalid {
                    reason,
                }));
            }
            HolochainBindingAttestationVerification::Valid => {}
        }

        let authority = binding.payload.provenance.authority.clone();
        let Some(authorized_credential) = authority_bindings.credential_for(&authority) else {
            return Ok(Err(
                HolochainAdapterBoundaryError::LogicalDependencyNotBound {
                    missing: vec![authority],
                },
            ));
        };

        if binding.signer != authorized_credential.payload.agent {
            return Ok(Err(HolochainAdapterBoundaryError::SemanticInvalid {
                reason: "runtime binding signer does not match the registered authority agent key"
                    .into(),
            }));
        }

        if binding.payload.provenance.witness_identity
            == authorized_credential.payload.provenance.witness_identity
        {
            return Ok(Err(HolochainAdapterBoundaryError::SemanticInvalid {
                reason: "runtime binding witness identity must differ from the registered authority credential witness"
                    .into(),
            }));
        }

        if authority_bindings
            .contains_provenance_witness(&binding.payload.provenance.witness_identity)
        {
            return Ok(Err(HolochainAdapterBoundaryError::BindingRejected {
                reason:
                    "a provenance witness admitted by the authority-agent registry may not be reused for a runtime binding"
                        .into(),
            }));
        }

        if let Some(missing_basis) = authorized_credential
            .payload
            .provenance
            .basis
            .iter()
            .find(|basis| !binding.payload.provenance.basis.contains(basis))
        {
            return Ok(Err(HolochainAdapterBoundaryError::SemanticInvalid {
                reason: format!(
                    "runtime binding provenance dropped authority credential basis witness: {missing_basis:?}"
                ),
            }));
        }

        if binding.payload.provenance.authority_scope
            != authorized_credential.payload.provenance.authority_scope
        {
            return Ok(Err(HolochainAdapterBoundaryError::SemanticInvalid {
                reason: "runtime binding authority scope does not match the registered authority credential scope"
                    .into(),
            }));
        }

        if binding.payload.provenance.authority_delegation
            != authorized_credential
                .payload
                .provenance
                .authority_delegation
        {
            return Ok(Err(HolochainAdapterBoundaryError::SemanticInvalid {
                reason: "runtime binding authority delegation does not match the registered authority credential delegation"
                    .into(),
            }));
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
        match attestation.verify()? {
            HolochainBindingAttestationVerification::Invalid { reason } => {
                return Ok(Err(HolochainAdapterBoundaryError::SemanticInvalid {
                    reason,
                }));
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
/// binding. Verification is deterministic and signs the payload through
/// Holochain's canonical serialization path. The payload type also derives
/// SerializedBytes, making its byte-level round-trip an explicit interoperability
/// contract. It includes the schema, provenance witness, protocol address, and
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
/// Unavailable addressable data is deliberately propagated as the raw Holochain
/// host outcome. In a validate callback Holochain maps an unavailable dependency
/// to UnresolvedDependencies rather than semantic Invalid.
///
/// For ValidRecord retrieval, the host primitive additionally asks for a record
/// that is reported valid by the visible validation authorities. This is an
/// inductive-validity dependency, not a claim that every later operation related
/// to that record is valid.
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
        ) => Ok(HolochainRetrievedDependency::Action(must_get_action(
            action_hash.clone(),
        )?)),
        (
            HolochainDependencyAddress::Entry(entry_hash),
            QualificationDependencyRetrievalKind::Entry,
        ) => Ok(HolochainRetrievedDependency::Entry(must_get_entry(
            entry_hash.clone(),
        )?)),
        _ => unreachable!("HolochainDependencyBindingSet prevents address-kind mismatch"),
    }
}

/// Retrieve a fully resolved dependency set in canonical logical-identity order.
///
/// Call this only after resolve_required returned Valid. An unbound logical
/// identity has no protocol address and therefore cannot be represented as a
/// Holochain unresolved-hash dependency yet.
fn canonicalize_retrieval_order(
    dependencies: &[ResolvedHolochainDependency],
) -> Vec<ResolvedHolochainDependency> {
    let mut canonical = dependencies.to_vec();
    canonical.sort_by(|left, right| left.identity.cmp(&right.identity));
    canonical
}

pub fn retrieve_resolved(
    dependencies: &[ResolvedHolochainDependency],
) -> ExternResult<Vec<HolochainRetrievedDependency>> {
    let canonical = canonicalize_retrieval_order(dependencies);
    canonical.iter().map(retrieve_one).collect()
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
            kind:
                mobility_configuration_qualification::identity_lineage::IdentityKind::EvidenceRecord,
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
#[rustfmt::skip]
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

    fn action_agent_key(byte: u8) -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![byte; 36])
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
    fn address_kind_mismatch_precedes_duplicate_provenance_binding_conflict() {
        let mut bindings = HolochainDependencyBindingSet::new();
        let first = identity("address-precedence-first");
        let authority = identity("address-precedence-authority");
        let first_provenance = binding_provenance_for_test(first.clone(), authority);
        bindings
            .bind(
                first,
                HolochainDependencyAddress::Action(action_hash(15)),
                QualificationDependencyRetrievalKind::Action,
                first_provenance.clone(),
            )
            .unwrap();

        let second = identity("address-precedence-second");
        let second_provenance = QualificationDependencyBindingProvenance {
            logical_identity: second.clone(),
            ..first_provenance
        };
        let error = bindings
            .bind(
                second,
                HolochainDependencyAddress::Action(action_hash(16)),
                QualificationDependencyRetrievalKind::Entry,
                second_provenance,
            )
            .expect_err("candidate address typing must precede registry witness conflicts");

        assert!(matches!(
            error,
            HolochainAdapterBoundaryError::AddressKindMismatch {
                retrieval: QualificationDependencyRetrievalKind::Entry,
                expected: "EntryHash",
                actual: "ActionHash",
            }
        ));
        assert_eq!(bindings.len(), 1);
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

    fn binding_provenance_for_test(
        logical_identity: IdentityRef,
        authority: IdentityRef,
    ) -> QualificationDependencyBindingProvenance {
        let suffix = logical_identity.id.clone();
        let authority_scope = IdentityRef {
            kind: IdentityKind::ReconciliationWitness,
            namespace: "mobility".into(),
            id: format!("binding-scope-{suffix}"),
        };
        let authority_delegation = IdentityRef {
            kind: IdentityKind::ReconciliationWitness,
            namespace: "mobility".into(),
            id: format!("binding-delegation-{suffix}"),
        };
        QualificationDependencyBindingProvenance {
            witness_identity: IdentityRef {
                kind: IdentityKind::ReconciliationWitness,
                namespace: "mobility".into(),
                id: format!("binding-witness-{suffix}"),
            },
            logical_identity,
            authority: authority.clone(),
            authority_scope: authority_scope.clone(),
            authority_delegation: authority_delegation.clone(),
            basis: vec![authority, authority_scope, authority_delegation],
        }
    }

    fn authority_credential(
        authority: IdentityRef,
        agent: AgentPubKey,
        id_suffix: &str,
    ) -> SignedHolochainAuthorityAgentBinding {
        let scope = IdentityRef {
            kind: IdentityKind::ReconciliationWitness,
            namespace: "mobility".into(),
            id: format!("credential-scope-{id_suffix}"),
        };
        let delegation = IdentityRef {
            kind: IdentityKind::ReconciliationWitness,
            namespace: "mobility".into(),
            id: format!("credential-delegation-{id_suffix}"),
        };
        let witness = IdentityRef {
            kind: IdentityKind::ReconciliationWitness,
            namespace: "mobility".into(),
            id: format!("credential-witness-{id_suffix}"),
        };

        SignedHolochainAuthorityAgentBinding {
            issuer: agent.clone(),
            signature: Signature([0u8; 64]),
            payload: HolochainAuthorityAgentBindingPayload {
                schema: HOLOCHAIN_AUTHORITY_AGENT_BINDING_SCHEMA.into(),
                provenance: QualificationAuthorityAgentBindingProvenance {
                    witness_identity: witness,
                    authority,
                    authority_scope: scope,
                    authority_delegation: delegation,
                    basis: vec![
                        IdentityRef {
                            kind: IdentityKind::EvidenceRecord,
                            namespace: "mobility".into(),
                            id: format!("credential-basis-authority-{id_suffix}"),
                        },
                        IdentityRef {
                            kind: IdentityKind::ReconciliationWitness,
                            namespace: "mobility".into(),
                            id: format!("credential-scope-{id_suffix}"),
                        },
                        IdentityRef {
                            kind: IdentityKind::ReconciliationWitness,
                            namespace: "mobility".into(),
                            id: format!("credential-delegation-{id_suffix}"),
                        },
                    ],
                },
                agent,
            },
        }
    }

    #[test]
    fn authority_agent_payload_has_explicit_canonical_serialized_bytes_roundtrip() {
        let authority = identity("canonical-authority");
        let credential = authority_credential(authority, action_agent_key(45), "canonical");
        let payload = credential.payload.clone();

        let encoded =
            SerializedBytes::try_from(payload.clone()).expect("payload must have canonical bytes");
        let decoded = HolochainAuthorityAgentBindingPayload::try_from(encoded)
            .expect("canonical bytes must round-trip through the declared payload type");

        assert_eq!(decoded, payload);
    }

    #[test]
    fn binding_attestation_payload_has_explicit_canonical_serialized_bytes_roundtrip() {
        let provenance = binding_provenance_for_test(
            identity("canonical-binding"),
            identity("canonical-binding-authority"),
        );
        let payload = HolochainBindingAttestationPayload {
            schema: HOLOCHAIN_BINDING_ATTESTATION_SCHEMA.into(),
            provenance,
            address: HolochainDependencyAddress::Action(action_hash(46)),
            retrieval: QualificationDependencyRetrievalKind::Action,
        };

        let encoded =
            SerializedBytes::try_from(payload.clone()).expect("payload must have canonical bytes");
        let decoded = HolochainBindingAttestationPayload::try_from(encoded)
            .expect("canonical bytes must round-trip through the declared payload type");

        assert_eq!(decoded, payload);
    }

    #[test]
    fn authority_agent_payload_rejects_schema_identifier_change() {
        let authority = identity("schema-version-authority");
        let mut credential = authority_credential(
            authority,
            action_agent_key(49),
            "schema-version",
        );
        credential.payload.schema = "mycelix.mobility.holochain_authority_agent_binding.v2".into();

        let verified = credential
            .verify()
            .expect("schema rejection must not require host I/O");

        assert!(
            matches!(
                verified,
                HolochainAuthorityAgentBindingVerification::Invalid { .. }
            ),
            "a v1 authority credential must reject a changed schema identifier"
        );
    }

    #[test]
    fn malformed_attestation_payload_precedes_missing_authority_preflight() {
        let authority = identity("malformed-precedence-authority");
        let binding = SignedHolochainBindingAttestation {
            signer: action_agent_key(51),
            signature: Signature([0u8; 64]),
            payload: HolochainBindingAttestationPayload {
                schema: "mycelix.mobility.holochain_binding_attestation.v2".into(),
                provenance: binding_provenance_for_test(
                    identity("malformed-precedence-binding"),
                    authority,
                ),
                address: HolochainDependencyAddress::Action(action_hash(52)),
                retrieval: QualificationDependencyRetrievalKind::Action,
            },
        };
        let mut bindings = HolochainDependencyBindingSet::new();
        let authority_bindings = HolochainAuthorityAgentBindingSet::new();

        let result = bindings
            .bind_attested_with_authority(binding, &authority_bindings)
            .expect("payload structural validation should complete without host failure");

        assert!(
            matches!(
                result,
                Err(HolochainAdapterBoundaryError::SemanticInvalid { .. })
            ),
            "malformed attestation payload must remain structural invalidity even when authority preflight is unresolved"
        );
        assert!(bindings.is_empty());
    }

    #[test]
    fn binding_attestation_payload_rejects_schema_identifier_change() {
        let mut payload = HolochainBindingAttestationPayload {
            schema: "mycelix.mobility.holochain_binding_attestation.v2".into(),
            provenance: binding_provenance_for_test(
                identity("schema-version-binding"),
                identity("schema-version-binding-authority"),
            ),
            address: HolochainDependencyAddress::Action(action_hash(50)),
            retrieval: QualificationDependencyRetrievalKind::Action,
        };

        let verified = payload.validate();

        assert!(
            matches!(
                verified,
                Err(HolochainAdapterBoundaryError::SemanticInvalid { .. })
            ),
            "a v1 binding attestation must reject a changed schema identifier"
        );

        payload.schema = HOLOCHAIN_BINDING_ATTESTATION_SCHEMA.into();
        assert!(payload.validate().is_ok());
    }

    #[test]
    fn authority_agent_payload_rejects_unknown_wire_fields() {
        #[derive(Debug, Clone, PartialEq, Eq, serde::Serialize, serde::Deserialize, SerializedBytes)]
        struct AuthorityPayloadWithExtraField {
            schema: String,
            provenance: QualificationAuthorityAgentBindingProvenance,
            agent: AgentPubKey,
            extra: String,
        }

        let authority = identity("unknown-field-authority");
        let credential = authority_credential(authority, action_agent_key(47), "unknown-field");
        let payload = AuthorityPayloadWithExtraField {
            schema: credential.payload.schema.clone(),
            provenance: credential.payload.provenance.clone(),
            agent: credential.payload.agent.clone(),
            extra: "must-not-be-accepted".into(),
        };

        let encoded =
            SerializedBytes::try_from(payload).expect("shadow payload must encode canonically");
        let decoded = HolochainAuthorityAgentBindingPayload::try_from(encoded);

        assert!(
            decoded.is_err(),
            "v1 authority payloads must reject unknown wire fields"
        );
    }

    #[test]
    fn binding_attestation_payload_rejects_unknown_wire_fields() {
        #[derive(Debug, Clone, PartialEq, Eq, serde::Serialize, serde::Deserialize, SerializedBytes)]
        struct BindingPayloadWithExtraField {
            schema: String,
            provenance: QualificationDependencyBindingProvenance,
            address: HolochainDependencyAddress,
            retrieval: QualificationDependencyRetrievalKind,
            extra: String,
        }

        let provenance = binding_provenance_for_test(
            identity("unknown-field-binding"),
            identity("unknown-field-binding-authority"),
        );
        let payload = BindingPayloadWithExtraField {
            schema: HOLOCHAIN_BINDING_ATTESTATION_SCHEMA.into(),
            provenance,
            address: HolochainDependencyAddress::Action(action_hash(48)),
            retrieval: QualificationDependencyRetrievalKind::Action,
            extra: "must-not-be-accepted".into(),
        };

        let encoded =
            SerializedBytes::try_from(payload).expect("shadow payload must encode canonically");
        let decoded = HolochainBindingAttestationPayload::try_from(encoded);

        assert!(
            decoded.is_err(),
            "v1 binding attestation payloads must reject unknown wire fields"
        );
    }

    #[test]
    fn authority_agent_credential_rejects_issuer_not_equal_to_agent() {
        let _guard = host_test_lock().lock().expect("HDI test lock is not poisoned");
        let _previous = set_hdi(RecordingHdi {
            calls: Arc::new(Mutex::new(Vec::new())),
            verify_result: true,
        });

        let authority = identity("issuer-mismatch-authority");
        let agent = action_agent_key(37);
        let mut credential = authority_credential(authority, agent.clone(), "issuer-mismatch");
        credential.issuer = action_agent_key(38);

        let result = credential.verify();
        let _ = set_hdi(ErrHdi);

        assert!(matches!(
            result,
            Ok(HolochainAuthorityAgentBindingVerification::Invalid { reason })
                if reason == "authority-agent binding issuer must equal the bound agent key"
        ));
        assert!(
            calls.lock().unwrap().is_empty(),
            "issuer mismatch must be classified before signature verification"
        );
    }

    #[test]
    fn authority_agent_payload_validation_precedes_signature_verification() {
        let _guard = host_test_lock().lock().expect("HDI test lock is not poisoned");
        let calls = Arc::new(Mutex::new(Vec::new()));
        let _previous = set_hdi(RecordingHdi {
            calls: Arc::clone(&calls),
            verify_result: true,
        });

        let authority = identity("malformed-authority-credential");
        let mut credential =
            authority_credential(authority, action_agent_key(57), "malformed-credential");
        credential.payload.schema =
            "mycelix.mobility.holochain_authority_agent_binding.v2".into();

        let result = credential.verify();

        let _ = set_hdi(ErrHdi);

        assert!(matches!(
            result,
            Ok(HolochainAuthorityAgentBindingVerification::Invalid { reason })
                if reason == "authority-agent binding uses an unexpected schema"
        ));
        assert!(
            calls.lock().unwrap().is_empty(),
            "definitive payload invalidity must prevent signature host I/O"
        );
    }

    #[test]
    fn authority_agent_issuer_mismatch_precedes_signature_verification() {
        let _guard = host_test_lock().lock().expect("HDI test lock is not poisoned");
        let calls = Arc::new(Mutex::new(Vec::new()));
        let _previous = set_hdi(RecordingHdi {
            calls: Arc::clone(&calls),
            verify_result: true,
        });

        let authority = identity("issuer-precedence-authority");
        let agent = action_agent_key(58);
        let mut credential = authority_credential(authority, agent, "issuer-precedence");
        credential.issuer = action_agent_key(59);

        let result = credential.verify();

        let _ = set_hdi(ErrHdi);

        assert!(matches!(
            result,
            Ok(HolochainAuthorityAgentBindingVerification::Invalid { reason })
                if reason == "authority-agent binding issuer must equal the bound agent key"
        ));
        assert!(
            calls.lock().unwrap().is_empty(),
            "issuer/agent mismatch must prevent signature host I/O"
        );
    }

    #[test]
    fn dependency_binding_without_authority_agent_mapping_is_preflight_unresolved() {
        let _guard = host_test_lock().lock().expect("HDI test lock is not poisoned");
        let calls = Arc::new(Mutex::new(Vec::new()));
        let _previous = set_hdi(RecordingHdi {
            calls: Arc::clone(&calls),
            verify_result: true,
        });

        let authority = identity("unmapped-authority");
        let logical = identity("unmapped-logical");
        let binding = SignedHolochainBindingAttestation {
            signer: action_agent_key(39),
            signature: Signature([0u8; 64]),
            payload: HolochainBindingAttestationPayload {
                schema: HOLOCHAIN_BINDING_ATTESTATION_SCHEMA.into(),
                provenance: binding_provenance_for_test(logical, authority.clone()),
                address: HolochainDependencyAddress::Action(action_hash(39)),
                retrieval: QualificationDependencyRetrievalKind::Action,
            },
        };

        let registry = HolochainAuthorityAgentBindingSet::new();
        let mut bindings = HolochainDependencyBindingSet::new();
        let result = bindings.bind_attested_with_authority(binding, &registry);

        let _ = set_hdi(ErrHdi);

        assert!(matches!(
            result,
            Ok(Err(HolochainAdapterBoundaryError::LogicalDependencyNotBound { missing }))
                if missing == vec![authority]
        ));
        assert!(bindings.is_empty());
        assert_eq!(
            calls.lock().unwrap().as_slice(),
            ["verify_signature"],
            "missing authority-agent registration must stop before DHT retrieval but may not mask signature validation"
        );
    }


    #[test]
    fn invalid_attestation_signature_precedes_missing_authority_preflight() {
        let _guard = host_test_lock().lock().expect("HDI test lock is not poisoned");
        let calls = Arc::new(Mutex::new(Vec::new()));
        let _previous = set_hdi(RecordingHdi {
            calls: Arc::clone(&calls),
            verify_result: false,
        });

        let authority = identity("invalid-signature-precedence-authority");
        let logical = identity("invalid-signature-precedence-logical");
        let binding = SignedHolochainBindingAttestation {
            signer: action_agent_key(55),
            signature: Signature([0u8; 64]),
            payload: HolochainBindingAttestationPayload {
                schema: HOLOCHAIN_BINDING_ATTESTATION_SCHEMA.into(),
                provenance: binding_provenance_for_test(logical, authority),
                address: HolochainDependencyAddress::Action(action_hash(55)),
                retrieval: QualificationDependencyRetrievalKind::Action,
            },
        };

        let registry = HolochainAuthorityAgentBindingSet::new();
        let mut bindings = HolochainDependencyBindingSet::new();
        let result = bindings.bind_attested_with_authority(binding, &registry);

        let _ = set_hdi(ErrHdi);

        assert!(
            matches!(
                result,
                Ok(Err(HolochainAdapterBoundaryError::SemanticInvalid { reason }))
                    if reason == "binding attestation signature did not verify"
            ),
            "invalid signature must remain definitive even when authority registration is absent"
        );
        assert_eq!(calls.lock().unwrap().as_slice(), ["verify_signature"]);
        assert!(bindings.is_empty());
    }

    #[test]
    fn signature_verification_host_failure_precedes_missing_authority_preflight() {
        let _guard = host_test_lock().lock().expect("HDI test lock is not poisoned");
        let _previous = set_hdi(ErrHdi);

        let authority = identity("signature-host-error-precedence-authority");
        let binding = SignedHolochainBindingAttestation {
            signer: action_agent_key(56),
            signature: Signature([0u8; 64]),
            payload: HolochainBindingAttestationPayload {
                schema: HOLOCHAIN_BINDING_ATTESTATION_SCHEMA.into(),
                provenance: binding_provenance_for_test(
                    identity("signature-host-error-precedence-logical"),
                    authority,
                ),
                address: HolochainDependencyAddress::Action(action_hash(56)),
                retrieval: QualificationDependencyRetrievalKind::Action,
            },
        };

        let registry = HolochainAuthorityAgentBindingSet::new();
        let mut bindings = HolochainDependencyBindingSet::new();
        let result = bindings.bind_attested_with_authority(binding, &registry);

        let _ = set_hdi(ErrHdi);

        assert!(
            result.is_err(),
            "a signature-verification host failure must not be collapsed into missing authority"
        );
        assert!(bindings.is_empty());
    }

    #[test]
    fn authority_agent_registry_retains_verified_credential_for_auditability() {
        let _guard = host_test_lock().lock().expect("HDI test lock is not poisoned");
        let _previous = set_hdi(RecordingHdi {
            calls: Arc::new(Mutex::new(Vec::new())),
            verify_result: true,
        });

        let authority = identity("registry-audit-authority");
        let agent = action_agent_key(30);
        let credential = authority_credential(authority.clone(), agent, "audit");
        let expected = credential.clone();

        let mut registry = HolochainAuthorityAgentBindingSet::new();
        assert!(matches!(registry.bind_attested(credential), Ok(Ok(()))));
        assert_eq!(registry.credential_for(&authority), Some(&expected));

        let _ = set_hdi(ErrHdi);
    }

    #[test]
    fn authority_agent_registry_accepts_verified_credential() {
        let _guard = host_test_lock().lock().expect("HDI test lock is not poisoned");
        let _previous = set_hdi(RecordingHdi {
            calls: Arc::new(Mutex::new(Vec::new())),
            verify_result: true,
        });

        let authority = identity("registry-authority");
        let agent = action_agent_key(31);
        let credential = authority_credential(authority.clone(), agent.clone(), "accept");

        let mut registry = HolochainAuthorityAgentBindingSet::new();
        let result = registry.bind_attested(credential);

        let _ = set_hdi(ErrHdi);

        assert!(matches!(result, Ok(Ok(()))));
        assert_eq!(registry.agent_for(&authority), Some(&agent));
    }

    #[test]
    fn authority_agent_registry_rejects_duplicate_provenance_witness() {
        let _guard = host_test_lock()
            .lock()
            .expect("HDI test lock is not poisoned");
        let _previous = set_hdi(RecordingHdi {
            calls: Arc::new(Mutex::new(Vec::new())),
            verify_result: true,
        });

        let first_authority = identity("witness-unique-first");
        let second_authority = identity("witness-unique-second");
        let first = authority_credential(first_authority, action_agent_key(51), "witness-shared");
        let mut second =
            authority_credential(second_authority, action_agent_key(52), "witness-second");
        second.payload.provenance.witness_identity =
            first.payload.provenance.witness_identity.clone();

        let mut registry = HolochainAuthorityAgentBindingSet::new();
        assert!(matches!(registry.bind_attested(first), Ok(Ok(()))));
        assert!(matches!(
            registry.bind_attested(second),
            Ok(Err(HolochainAdapterBoundaryError::BindingRejected { .. }))
        ));
        assert_eq!(registry.len(), 1);
    }

    #[test]
    fn malformed_authority_credential_precedes_duplicate_authority_admission() {
        let _guard = host_test_lock().lock().expect("HDI test lock is not poisoned");
        let calls = Arc::new(Mutex::new(Vec::new()));
        let _previous = set_hdi(RecordingHdi { calls: Arc::clone(&calls), verify_result: true });
        let authority = identity("invalid-credential-duplicate-authority");
        let first = authority_credential(authority.clone(), action_agent_key(72), "admission-first");
        let expected = first.clone();
        let mut registry = HolochainAuthorityAgentBindingSet::new();
        assert!(matches!(registry.bind_attested(first), Ok(Ok(()))));
        let mut malformed = authority_credential(authority.clone(), action_agent_key(73), "admission-second");
        malformed.payload.schema = "mycelix.mobility.holochain_authority_agent_binding.v2".into();
        let result = registry.bind_attested(malformed);
        let _ = set_hdi(ErrHdi);
        assert!(matches!(result, Ok(Err(HolochainAdapterBoundaryError::SemanticInvalid { reason }))
            if reason == "authority-agent binding uses an unexpected schema"));
        assert_eq!(registry.len(), 1);
        assert_eq!(registry.credential_for(&authority), Some(&expected));
        assert_eq!(calls.lock().unwrap().as_slice(), ["verify_signature"]);
    }
    #[test]
    fn issuer_mismatch_precedes_duplicate_authority_admission() {
        let _guard = host_test_lock().lock().expect("HDI test lock is not poisoned");
        let calls = Arc::new(Mutex::new(Vec::new()));
        let _previous = set_hdi(RecordingHdi { calls: Arc::clone(&calls), verify_result: true });
        let authority = identity("issuer-duplicate-authority");
        let first = authority_credential(authority.clone(), action_agent_key(74), "issuer-admission-first");
        let expected = first.clone();
        let mut registry = HolochainAuthorityAgentBindingSet::new();
        assert!(matches!(registry.bind_attested(first), Ok(Ok(()))));
        let mut conflicting = authority_credential(authority.clone(), action_agent_key(75), "issuer-admission-second");
        conflicting.issuer = action_agent_key(76);
        let result = registry.bind_attested(conflicting);
        let _ = set_hdi(ErrHdi);
        assert!(matches!(result, Ok(Err(HolochainAdapterBoundaryError::SemanticInvalid { reason }))
            if reason == "authority-agent binding issuer must equal the bound agent key"));
        assert_eq!(registry.len(), 1);
        assert_eq!(registry.credential_for(&authority), Some(&expected));
        assert_eq!(calls.lock().unwrap().as_slice(), ["verify_signature"]);
    }
    #[test]
    fn invalid_authority_credential_signature_precedes_duplicate_authority_admission() {
        let _guard = host_test_lock().lock().expect("HDI test lock is not poisoned");
        let calls = Arc::new(Mutex::new(Vec::new()));
        let _previous = set_hdi(RecordingHdi { calls: Arc::clone(&calls), verify_result: true });
        let authority = identity("signature-duplicate-authority");
        let first = authority_credential(authority.clone(), action_agent_key(77), "signature-admission-first");
        let expected = first.clone();
        let mut registry = HolochainAuthorityAgentBindingSet::new();
        assert!(matches!(registry.bind_attested(first), Ok(Ok(()))));
        let _previous = set_hdi(RecordingHdi { calls: Arc::clone(&calls), verify_result: false });
        let conflicting = authority_credential(authority.clone(), action_agent_key(78), "signature-admission-second");
        let result = registry.bind_attested(conflicting);
        let _ = set_hdi(ErrHdi);
        assert!(matches!(result, Ok(Err(HolochainAdapterBoundaryError::SemanticInvalid { reason }))
            if reason == "authority-agent binding signature did not verify"));
        assert_eq!(registry.len(), 1);
        assert_eq!(registry.credential_for(&authority), Some(&expected));
        assert_eq!(calls.lock().unwrap().as_slice(), ["verify_signature", "verify_signature"]);
    }
    #[test]
    fn valid_duplicate_authority_is_adapter_rejection_and_state_preserving() {
        let _guard = host_test_lock().lock().expect("HDI test lock is not poisoned");
        let calls = Arc::new(Mutex::new(Vec::new()));
        let _previous = set_hdi(RecordingHdi { calls: Arc::clone(&calls), verify_result: true });
        let authority = identity("valid-duplicate-authority");
        let first = authority_credential(authority.clone(), action_agent_key(79), "valid-admission-first");
        let expected = first.clone();
        let second = authority_credential(authority.clone(), action_agent_key(80), "valid-admission-second");
        let mut registry = HolochainAuthorityAgentBindingSet::new();
        assert!(matches!(registry.bind_attested(first), Ok(Ok(()))));
        let result = registry.bind_attested(second);
        let _ = set_hdi(ErrHdi);
        assert!(matches!(result, Ok(Err(HolochainAdapterBoundaryError::BindingRejected { reason }))
            if reason == "an authority identity may be bound to only one AgentPubKey in an immutable binding set"));
        assert_eq!(registry.len(), 1);
        assert_eq!(registry.credential_for(&authority), Some(&expected));
        assert_eq!(calls.lock().unwrap().as_slice(), ["verify_signature", "verify_signature"]);
    }
    #[test]
    fn valid_duplicate_provenance_witness_is_adapter_rejection_and_state_preserving() {
        let _guard = host_test_lock().lock().expect("HDI test lock is not poisoned");
        let calls = Arc::new(Mutex::new(Vec::new()));
        let _previous = set_hdi(RecordingHdi { calls: Arc::clone(&calls), verify_result: true });
        let first = authority_credential(identity("valid-witness-admission-first"), action_agent_key(81), "valid-witness-admission-first");
        let mut second = authority_credential(identity("valid-witness-admission-second"), action_agent_key(82), "valid-witness-admission-second");
        second.payload.provenance.witness_identity = first.payload.provenance.witness_identity.clone();
        let mut registry = HolochainAuthorityAgentBindingSet::new();
        assert!(matches!(registry.bind_attested(first.clone()), Ok(Ok(()))));
        let result = registry.bind_attested(second);
        let _ = set_hdi(ErrHdi);
        assert!(matches!(result, Ok(Err(HolochainAdapterBoundaryError::BindingRejected { reason }))
            if reason == "an authority-agent provenance witness may justify only one registry binding"));
        assert_eq!(registry.len(), 1);
        assert_eq!(registry.credential_for(&first.payload.provenance.authority), Some(&first));
        assert_eq!(calls.lock().unwrap().as_slice(), ["verify_signature", "verify_signature"]);
    }
    #[test]
    fn credential_signature_host_failure_precedes_duplicate_authority_admission() {
        let _guard = host_test_lock().lock().expect("HDI test lock is not poisoned");
        let calls = Arc::new(Mutex::new(Vec::new()));
        let _previous = set_hdi(RecordingHdi { calls: Arc::clone(&calls), verify_result: true });
        let authority = identity("signature-host-duplicate-authority");
        let first = authority_credential(authority.clone(), action_agent_key(83), "signature-host-admission-first");
        let expected = first.clone();
        let mut registry = HolochainAuthorityAgentBindingSet::new();
        assert!(matches!(registry.bind_attested(first), Ok(Ok(()))));
        let _previous = set_hdi(ErrHdi);
        let conflicting = authority_credential(authority.clone(), action_agent_key(84), "signature-host-admission-second");
        let result = registry.bind_attested(conflicting);
        assert!(result.is_err(), "signature host failure must remain an ExternResult error");
        assert_eq!(registry.len(), 1);
        assert_eq!(registry.credential_for(&authority), Some(&expected));
        assert_eq!(calls.lock().unwrap().as_slice(), ["verify_signature"]);
    }
    #[test]
    fn authority_agent_registry_rejects_duplicate_authority() {
        let _guard = host_test_lock().lock().expect("HDI test lock is not poisoned");
        let calls = Arc::new(Mutex::new(Vec::new()));
        let _previous = set_hdi(RecordingHdi {
            calls: Arc::clone(&calls),
            verify_result: true,
        });

        let authority = identity("duplicate-authority");
        let first = authority_credential(authority.clone(), action_agent_key(32), "first");
        let expected = first.clone();
        let second = authority_credential(authority.clone(), action_agent_key(33), "second");

        let mut registry = HolochainAuthorityAgentBindingSet::new();
        assert!(matches!(registry.bind_attested(first), Ok(Ok(()))));
        assert!(matches!(
            registry.bind_attested(second),
            Ok(Err(HolochainAdapterBoundaryError::BindingRejected { .. }))
        ));

        let _ = set_hdi(ErrHdi);
        assert_eq!(registry.len(), 1);
        assert_eq!(registry.credential_for(&authority), Some(&expected));
        assert_eq!(
            calls.lock().unwrap().as_slice(),
            ["verify_signature", "verify_signature"]
        );
    }

    #[test]
    fn dependency_binding_accepts_signer_registered_for_authority() {
        let _guard = host_test_lock().lock().expect("HDI test lock is not poisoned");
        let _previous = set_hdi(RecordingHdi {
            calls: Arc::new(Mutex::new(Vec::new())),
            verify_result: true,
        });

        let authority = identity("accepted-authority");
        let agent = action_agent_key(36);

        let credential = authority_credential(
            authority.clone(),
            agent.clone(),
            "runtime-accepted"
        );
        let runtime_scope = credential.payload.provenance.authority_scope.clone();
        let runtime_delegation = credential.payload.provenance.authority_delegation.clone();

        let mut registry = HolochainAuthorityAgentBindingSet::new();
        assert!(matches!(
            registry.bind_attested(credential),
            Ok(Ok(()))
        ));

        let logical = identity("accepted-logical");
        let binding = SignedHolochainBindingAttestation {
            signer: agent,
            signature: Signature([0u8; 64]),
            payload: HolochainBindingAttestationPayload {
                schema: HOLOCHAIN_BINDING_ATTESTATION_SCHEMA.into(),
                provenance: {
                    let mut provenance =
                        binding_provenance_for_test(logical.clone(), authority);
                    provenance.authority_scope = runtime_scope;
                    provenance.authority_delegation = runtime_delegation;
                    provenance.basis = credential_provenance.basis.clone();
                    provenance.basis.push(provenance.authority.clone());
                    provenance
                },
                address: HolochainDependencyAddress::Action(action_hash(36)),
                retrieval: QualificationDependencyRetrievalKind::Action,
            },
        };

        let mut bindings = HolochainDependencyBindingSet::new();
        assert!(matches!(
            bindings.bind_attested_with_authority(binding, &registry),
            Ok(Ok(()))
        ));

        let decision = bindings.resolve_required(vec![logical]);
        assert!(matches!(decision, QualificationDecision::Valid(_)));

        let _ = set_hdi(ErrHdi);
    }

    #[test]
    fn dependency_binding_rejects_runtime_binding_witness_reuse() {
        let _guard = host_test_lock().lock().expect("HDI test lock is not poisoned");
        let _previous = set_hdi(RecordingHdi {
            calls: Arc::new(Mutex::new(Vec::new())),
            verify_result: true,
        });

        let authority = identity("witness-reuse-authority");
        let agent = action_agent_key(42);
        let credential =
            authority_credential(authority.clone(), agent.clone(), "witness-reuse");
        let mut runtime_provenance =
            binding_provenance_for_test(identity("witness-reuse-logical"), authority);
        runtime_provenance.witness_identity =
            credential.payload.provenance.witness_identity.clone();
        runtime_provenance.authority_scope =
            credential.payload.provenance.authority_scope.clone();
        runtime_provenance.authority_delegation =
            credential.payload.provenance.authority_delegation.clone();
        runtime_provenance.basis = credential.payload.provenance.basis.clone();

        let mut registry = HolochainAuthorityAgentBindingSet::new();
        assert!(matches!(registry.bind_attested(credential), Ok(Ok(()))));

        let binding = SignedHolochainBindingAttestation {
            signer: agent,
            signature: Signature([0u8; 64]),
            payload: HolochainBindingAttestationPayload {
                schema: HOLOCHAIN_BINDING_ATTESTATION_SCHEMA.into(),
                provenance: runtime_provenance,
                address: HolochainDependencyAddress::Action(action_hash(42)),
                retrieval: QualificationDependencyRetrievalKind::Action,
            },
        };

        let mut bindings = HolochainDependencyBindingSet::new();
        let result = bindings.bind_attested_with_authority(binding, &registry);
        let _ = set_hdi(ErrHdi);

        assert!(matches!(
            result,
            Ok(Err(HolochainAdapterBoundaryError::SemanticInvalid { reason }))
                if reason == "runtime binding witness identity must differ from the registered authority credential witness"
        ));
        assert!(bindings.is_empty());
    }

    #[test]
    fn dependency_binding_rejects_provenance_witness_from_other_authority_registry_binding() {
        let _guard = host_test_lock().lock().expect("HDI test lock is not poisoned");
        let _previous = set_hdi(RecordingHdi {
            calls: Arc::new(Mutex::new(Vec::new())),
            verify_result: true,
        });

        let first_authority = identity("cross-registry-first-authority");
        let second_authority = identity("cross-registry-second-authority");
        let first_agent = action_agent_key(53);
        let second_agent = action_agent_key(54);
        let first_credential =
            authority_credential(first_authority.clone(), first_agent.clone(), "cross-registry-first");
        let second_credential =
            authority_credential(second_authority, second_agent, "cross-registry-second");

        let reused_witness = second_credential
            .payload
            .provenance
            .witness_identity
            .clone();
        let registered_provenance = first_credential.payload.provenance.clone();

        let calls = Arc::new(Mutex::new(Vec::new()));
        let _previous = set_hdi(RecordingHdi {
            calls: Arc::clone(&calls),
            verify_result: true,
        });

        let mut registry = HolochainAuthorityAgentBindingSet::new();
        assert!(matches!(
            registry.bind_attested(first_credential),
            Ok(Ok(()))
        ));
        assert!(matches!(
            registry.bind_attested(second_credential),
            Ok(Ok(()))
        ));

        let logical = identity("cross-registry-runtime-logical");
        let mut provenance =
            binding_provenance_for_test(logical, first_authority.clone());
        provenance.witness_identity = reused_witness;
        provenance.authority_scope = registered_provenance.authority_scope;
        provenance.authority_delegation = registered_provenance.authority_delegation;
        provenance.basis = registered_provenance.basis;

        let binding = SignedHolochainBindingAttestation {
            signer: first_agent,
            signature: Signature([0u8; 64]),
            payload: HolochainBindingAttestationPayload {
                schema: HOLOCHAIN_BINDING_ATTESTATION_SCHEMA.into(),
                provenance,
                address: HolochainDependencyAddress::Action(action_hash(54)),
                retrieval: QualificationDependencyRetrievalKind::Action,
            },
        };

        let mut bindings = HolochainDependencyBindingSet::new();
        let result = bindings.bind_attested_with_authority(binding, &registry);
        let _ = set_hdi(ErrHdi);

        assert_eq!(
            calls.lock().unwrap().as_slice(),
            ["verify_signature", "verify_signature"],
            "cross-registry witness reuse must be rejected before runtime signature verification"
        );
        assert!(matches!(
            result,
            Ok(Err(HolochainAdapterBoundaryError::BindingRejected { reason }))
                if reason == "a provenance witness admitted by the authority-agent registry may not be reused for a runtime binding"
        ));
        assert!(bindings.is_empty());
    }

    #[test]
    fn dependency_binding_rejects_dropped_authority_credential_basis() {
        let _guard = host_test_lock().lock().expect("HDI test lock is not poisoned");
        let _previous = set_hdi(RecordingHdi {
            calls: Arc::new(Mutex::new(Vec::new())),
            verify_result: true,
        });

        let authority = identity("basis-drop-authority");
        let agent = action_agent_key(43);
        let credential =
            authority_credential(authority.clone(), agent.clone(), "basis-drop");
        let registered_scope = credential.payload.provenance.authority_scope.clone();
        let registered_delegation =
            credential.payload.provenance.authority_delegation.clone();

        let mut registry = HolochainAuthorityAgentBindingSet::new();
        assert!(matches!(registry.bind_attested(credential), Ok(Ok(()))));

        let logical = identity("basis-drop-logical");
        let mut provenance =
            binding_provenance_for_test(logical, authority);
        provenance.authority_scope = registered_scope;
        provenance.authority_delegation = registered_delegation;
        provenance.basis = vec![
            provenance.authority.clone(),
            provenance.authority_scope.clone(),
            provenance.authority_delegation.clone(),
        ];

        let binding = SignedHolochainBindingAttestation {
            signer: agent,
            signature: Signature([0u8; 64]),
            payload: HolochainBindingAttestationPayload {
                schema: HOLOCHAIN_BINDING_ATTESTATION_SCHEMA.into(),
                provenance,
                address: HolochainDependencyAddress::Action(action_hash(43)),
                retrieval: QualificationDependencyRetrievalKind::Action,
            },
        };

        let mut bindings = HolochainDependencyBindingSet::new();
        let result = bindings.bind_attested_with_authority(binding, &registry);
        let _ = set_hdi(ErrHdi);

        assert!(matches!(
            result,
            Ok(Err(HolochainAdapterBoundaryError::SemanticInvalid { reason }))
                if reason.contains("runtime binding provenance dropped authority credential basis witness")
        ));
        assert!(bindings.is_empty());
    }

    #[test]
    fn dependency_binding_accepts_complete_authority_credential_basis() {
        let _guard = host_test_lock().lock().expect("HDI test lock is not poisoned");
        let _previous = set_hdi(RecordingHdi {
            calls: Arc::new(Mutex::new(Vec::new())),
            verify_result: true,
        });

        let authority = identity("basis-complete-authority");
        let agent = action_agent_key(44);
        let credential =
            authority_credential(authority.clone(), agent.clone(), "basis-complete");
        let credential_provenance = credential.payload.provenance.clone();

        let mut registry = HolochainAuthorityAgentBindingSet::new();
        assert!(matches!(registry.bind_attested(credential), Ok(Ok(()))));

        let logical = identity("basis-complete-logical");
        let mut provenance =
            binding_provenance_for_test(logical, authority);
        provenance.authority_scope = credential_provenance.authority_scope;
        provenance.authority_delegation = credential_provenance.authority_delegation;
        provenance.basis = credential_provenance.basis;
        provenance.basis.push(provenance.authority.clone());
        provenance.basis.reverse();

        let binding = SignedHolochainBindingAttestation {
            signer: agent,
            signature: Signature([0u8; 64]),
            payload: HolochainBindingAttestationPayload {
                schema: HOLOCHAIN_BINDING_ATTESTATION_SCHEMA.into(),
                provenance,
                address: HolochainDependencyAddress::Action(action_hash(44)),
                retrieval: QualificationDependencyRetrievalKind::Action,
            },
        };

        let mut bindings = HolochainDependencyBindingSet::new();
        assert!(matches!(
            bindings.bind_attested_with_authority(binding, &registry),
            Ok(Ok(()))
        ));
        assert_eq!(bindings.len(), 1);

        let _ = set_hdi(ErrHdi);
    }

    #[test]
    fn dependency_binding_rejects_runtime_scope_not_registered_for_authority() {
        let _guard = host_test_lock().lock().expect("HDI test lock is not poisoned");
        let calls = Arc::new(Mutex::new(Vec::new()));
        let _previous = set_hdi(RecordingHdi {
            calls: Arc::clone(&calls),
            verify_result: true,
        });

        let authority = identity("scope-mismatch-authority");
        let agent = action_agent_key(40);
        let credential = authority_credential(
            authority.clone(),
            agent.clone(),
            "scope-mismatch",
        );
        let registered_delegation =
            credential.payload.provenance.authority_delegation.clone();
        let registered_basis = credential.payload.provenance.basis.clone();
        let mut registry = HolochainAuthorityAgentBindingSet::new();
        assert!(matches!(registry.bind_attested(credential), Ok(Ok(()))));

        let logical = identity("scope-mismatch-logical");
        let mut provenance =
            binding_provenance_for_test(logical, authority);
        provenance.authority_delegation = registered_delegation;
        provenance.basis = registered_basis;
        provenance.basis.push(provenance.authority.clone());
        provenance.basis.push(provenance.authority_scope.clone());
        let binding = SignedHolochainBindingAttestation {
            signer: agent,
            signature: Signature([0u8; 64]),
            payload: HolochainBindingAttestationPayload {
                schema: HOLOCHAIN_BINDING_ATTESTATION_SCHEMA.into(),
                provenance,
                address: HolochainDependencyAddress::Action(action_hash(40)),
                retrieval: QualificationDependencyRetrievalKind::Action,
            },
        };

        let mut bindings = HolochainDependencyBindingSet::new();
        let result = bindings.bind_attested_with_authority(binding, &registry);
        let _ = set_hdi(ErrHdi);

        assert!(matches!(
            result,
            Ok(Err(HolochainAdapterBoundaryError::SemanticInvalid { reason }))
                if reason == "runtime binding authority scope does not match the registered authority credential scope"
        ));
        assert_eq!(calls.lock().unwrap().as_slice(), ["verify_signature"]);
        assert!(bindings.is_empty());
    }

    #[test]
    fn dependency_binding_rejects_runtime_delegation_not_registered_for_authority() {
        let _guard = host_test_lock().lock().expect("HDI test lock is not poisoned");
        let calls = Arc::new(Mutex::new(Vec::new()));
        let _previous = set_hdi(RecordingHdi {
            calls: Arc::clone(&calls),
            verify_result: true,
        });

        let authority = identity("delegation-mismatch-authority");
        let agent = action_agent_key(41);
        let credential = authority_credential(
            authority.clone(),
            agent.clone(),
            "delegation-mismatch",
        );
        let registered_scope = credential.payload.provenance.authority_scope.clone();
        let registered_basis = credential.payload.provenance.basis.clone();
        let mut registry = HolochainAuthorityAgentBindingSet::new();
        assert!(matches!(registry.bind_attested(credential), Ok(Ok(()))));

        let logical = identity("delegation-mismatch-logical");
        let mut provenance =
            binding_provenance_for_test(logical.clone(), authority.clone());
        provenance.authority_scope = registered_scope;
        provenance.basis = registered_basis;
        provenance.basis.push(provenance.authority.clone());
        provenance.basis.push(provenance.authority_delegation.clone());
        let binding = SignedHolochainBindingAttestation {
            signer: agent,
            signature: Signature([0u8; 64]),
            payload: HolochainBindingAttestationPayload {
                schema: HOLOCHAIN_BINDING_ATTESTATION_SCHEMA.into(),
                provenance,
                address: HolochainDependencyAddress::Action(action_hash(41)),
                retrieval: QualificationDependencyRetrievalKind::Action,
            },
        };

        let mut bindings = HolochainDependencyBindingSet::new();
        let result = bindings.bind_attested_with_authority(binding, &registry);
        let _ = set_hdi(ErrHdi);

        assert!(matches!(
            result,
            Ok(Err(HolochainAdapterBoundaryError::SemanticInvalid { reason }))
                if reason == "runtime binding authority delegation does not match the registered authority credential delegation"
        ));
        assert_eq!(calls.lock().unwrap().as_slice(), ["verify_signature"]);
        assert!(bindings.is_empty());
    }

    #[test]
    fn dependency_binding_rejects_runtime_signer_not_registered_for_authority() {
        let _guard = host_test_lock().lock().expect("HDI test lock is not poisoned");
        let _previous = set_hdi(RecordingHdi {
            calls: Arc::new(Mutex::new(Vec::new())),
            verify_result: true,
        });

        let authority = identity("bound-authority");
        let registered_agent = action_agent_key(34);
        let wrong_agent = action_agent_key(35);

        let mut registry = HolochainAuthorityAgentBindingSet::new();
        assert!(matches!(
            registry.bind_attested(authority_credential(
                authority.clone(),
                registered_agent,
                "runtime-mismatch"
            )),
            Ok(Ok(()))
        ));

        let logical = identity("runtime-signer-mismatch");
        let binding = SignedHolochainBindingAttestation {
            signer: wrong_agent,
            signature: Signature([0u8; 64]),
            payload: HolochainBindingAttestationPayload {
                schema: HOLOCHAIN_BINDING_ATTESTATION_SCHEMA.into(),
                provenance: QualificationDependencyBindingProvenance {
                    witness_identity: IdentityRef {
                        kind: IdentityKind::ReconciliationWitness,
                        namespace: "mobility".into(),
                        id: "runtime-mismatch-witness".into(),
                    },
                    logical_identity: logical,
                    authority: authority.clone(),
                    authority_scope: IdentityRef {
                        kind: IdentityKind::ReconciliationWitness,
                        namespace: "mobility".into(),
                        id: "runtime-mismatch-scope".into(),
                    },
                    authority_delegation: IdentityRef {
                        kind: IdentityKind::ReconciliationWitness,
                        namespace: "mobility".into(),
                        id: "runtime-mismatch-delegation".into(),
                    },
                    basis: vec![
                        authority,
                        IdentityRef {
                            kind: IdentityKind::ReconciliationWitness,
                            namespace: "mobility".into(),
                            id: "runtime-mismatch-scope".into(),
                        },
                        IdentityRef {
                            kind: IdentityKind::ReconciliationWitness,
                            namespace: "mobility".into(),
                            id: "runtime-mismatch-delegation".into(),
                        },
                    ],
                },
                address: HolochainDependencyAddress::Action(action_hash(35)),
                retrieval: QualificationDependencyRetrievalKind::Action,
            },
        };

        let mut bindings = HolochainDependencyBindingSet::new();
        let result = bindings.bind_attested_with_authority(binding, &registry);
        let _ = set_hdi(ErrHdi);

        assert!(matches!(
            result,
            Ok(Err(HolochainAdapterBoundaryError::SemanticInvalid { reason }))
                if reason == "runtime binding signer does not match the registered authority agent key"
        ));
        assert!(bindings.is_empty());
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
                schema: HOLOCHAIN_BINDING_ATTESTATION_SCHEMA.into(),
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
                schema: HOLOCHAIN_BINDING_ATTESTATION_SCHEMA.into(),
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
                schema: HOLOCHAIN_BINDING_ATTESTATION_SCHEMA.into(),
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
    fn retrieve_resolved_canonicalizes_caller_supplied_order() {
        let mut bindings = HolochainDependencyBindingSet::new();
        for (value, byte) in [("b", 62u8), ("a", 61u8), ("c", 63u8)] {
            bindings
                .bind_for_test(
                    identity(value),
                    HolochainDependencyAddress::Action(action_hash(byte)),
                    QualificationDependencyRetrievalKind::Action,
                )
                .unwrap();
        }

        let QualificationDecision::Valid(resolved) =
            bindings.resolve_required(vec![identity("c"), identity("b"), identity("a")])
        else {
            panic!("bindings should resolve");
        };

        let reversed: Vec<_> = resolved.into_iter().rev().collect();
        let canonical = canonicalize_retrieval_order(&reversed);

        assert_eq!(
            canonical
                .iter()
                .map(|dependency| dependency.identity.id.as_str())
                .collect::<Vec<_>>(),
            vec!["a", "b", "c"]
        );
        assert_eq!(
            reversed
                .iter()
                .map(|dependency| dependency.identity.id.as_str())
                .collect::<Vec<_>>(),
            vec!["c", "b", "a"]
        );
        assert_ne!(canonical, reversed);
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
            .bind_for_test(identity("dispatch"), address.clone(), retrieval)
            .unwrap();

        let QualificationDecision::Valid(mut resolved) =
            bindings.resolve_required(vec![identity("dispatch")])
        else {
            panic!("binding should resolve");
        };

        let calls = Arc::new(Mutex::new(Vec::new()));
        let _ = set_hdi(DispatchRecordingHdi {
            calls: Arc::clone(&calls),
        });

        let result =
            catch_unwind(AssertUnwindSafe(|| retrieve_one(&resolved.remove(0))));

        let _ = set_hdi(ErrHdi);

        assert!(result.is_err(), "recording HDI intentionally stops the call");
        let recorded = calls.lock().unwrap();
        assert_eq!(recorded.len(), 1);
        assert_eq!(recorded[0].function, expected_call);
        assert_eq!(recorded[0].address, address);
    }

    #[derive(Debug, Clone, PartialEq, Eq)]
    struct RecordedDispatch {
        function: &'static str,
        address: HolochainDependencyAddress,
    }

    #[derive(Debug)]
    struct DispatchRecordingHdi {
        calls: Arc<Mutex<Vec<RecordedDispatch>>>,
    }

    #[test]
    fn retrieve_resolved_dispatches_first_canonical_dependency_to_host() {
        let _guard = host_test_lock().lock().expect("HDI test lock is not poisoned");

        let mut bindings = HolochainDependencyBindingSet::new();
        bindings
            .bind_for_test(
                identity("a-dispatch"),
                HolochainDependencyAddress::Action(action_hash(60)),
                QualificationDependencyRetrievalKind::Action,
            )
            .unwrap();
        bindings
            .bind_for_test(
                identity("b-dispatch"),
                HolochainDependencyAddress::Action(action_hash(61)),
                QualificationDependencyRetrievalKind::Action,
            )
            .unwrap();

        let QualificationDecision::Valid(resolved) = bindings.resolve_required(vec![
            identity("a-dispatch"),
            identity("b-dispatch"),
        ]) else {
            panic!("bindings should resolve");
        };
        let caller_order = resolved.into_iter().rev().collect::<Vec<_>>();

        let calls = Arc::new(Mutex::new(Vec::new()));
        let _ = set_hdi(DispatchRecordingHdi {
            calls: Arc::clone(&calls),
        });
        let result =
            catch_unwind(AssertUnwindSafe(|| retrieve_resolved(&caller_order)));
        let _ = set_hdi(ErrHdi);

        assert!(result.is_err(), "recording HDI intentionally stops the first host call");
        let recorded = calls.lock().unwrap();
        assert_eq!(recorded.len(), 1);
        assert_eq!(recorded[0].function, "must_get_action");
        assert_eq!(
            recorded[0].address,
            HolochainDependencyAddress::Action(action_hash(60))
        );
    }

    struct RecordingHdi {
        calls: Arc<Mutex<Vec<&'static str>>>,
        verify_result: bool,
    }

    impl HdiT for DispatchRecordingHdi {
        fn verify_signature(&self, _: VerifySignature) -> ExternResult<bool> {
            Ok(true)
        }

        fn must_get_entry(&self, input: MustGetEntryInput) -> ExternResult<EntryHashed> {
            self.calls.lock().unwrap().push(RecordedDispatch {
                function: "must_get_entry",
                address: HolochainDependencyAddress::Entry(input.into_inner()),
            });
            panic!("test stop after recording must_get_entry");
        }

        fn must_get_action(
            &self,
            input: MustGetActionInput,
        ) -> ExternResult<SignedActionHashed> {
            self.calls.lock().unwrap().push(RecordedDispatch {
                function: "must_get_action",
                address: HolochainDependencyAddress::Action(input.into_inner()),
            });
            panic!("test stop after recording must_get_action");
        }

        fn must_get_valid_record(
            &self,
            input: MustGetValidRecordInput,
        ) -> ExternResult<Record> {
            self.calls.lock().unwrap().push(RecordedDispatch {
                function: "must_get_valid_record",
                address: HolochainDependencyAddress::Action(input.into_inner()),
            });
            panic!("test stop after recording must_get_valid_record");
        }

        fn must_get_agent_activity(
            &self,
            _: MustGetAgentActivityInput,
        ) -> ExternResult<Vec<AgentActivity>> {
            unimplemented!()
        }

        fn dna_info(&self, _: ()) -> ExternResult<DnaInfo> {
            unimplemented!()
        }

        fn zome_info(&self, _: ()) -> ExternResult<ZomeInfo> {
            unimplemented!()
        }

        fn trace(&self, _: TraceMsg) -> ExternResult<()> {
            unimplemented!()
        }

        fn x_salsa20_poly1305_decrypt(
            &self,
            _: XSalsa20Poly1305Decrypt,
        ) -> ExternResult<Option<XSalsa20Poly1305Data>> {
            unimplemented!()
        }

        fn x_25519_x_salsa20_poly1305_decrypt(
            &self,
            _: X25519XSalsa20Poly1305Decrypt,
        ) -> ExternResult<Option<XSalsa20Poly1305Data>> {
            unimplemented!()
        }

        fn ed_25519_x_salsa20_poly1305_decrypt(
            &self,
            _: Ed25519XSalsa20Poly1305Decrypt,
        ) -> ExternResult<XSalsa20Poly1305Data> {
            unimplemented!()
        }
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
