#![forbid(unsafe_code)]
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! FIN-EFFECT-001 pure economic operation identity kernel.
//!
//! This crate deliberately does not talk to Holochain, a payment provider, or a
//! database. It computes stable identities and classifies caller-supplied
//! observations. It cannot authenticate an author, prove that a DHT view is
//! complete, establish global uniqueness, or authorize a value mutation.
//!
//! In particular, BindingProjectionV1::NoBindingObserved is an observation
//! result, not permission to create an effect. An adapter must separately prove
//! that it has the required admission/serialization authority.

use std::collections::{BTreeMap, BTreeSet};

pub const OPERATION_ID_SCHEMA_V1: u16 = 1;
pub const INTENT_FINGERPRINT_SCHEMA_V1: u16 = 1;
pub const EFFECT_ID_SCHEMA_V1: u16 = 1;
pub const CLAIM_PROFILE_DIGEST_SCHEMA_V1: u16 = 1;
pub const MAX_COMPONENT_BYTES: usize = 1_024;

const OPERATION_ID_DOMAIN: &[u8] = b"mycelix:economic-operation:v1\0";
const INTENT_FINGERPRINT_DOMAIN: &[u8] = b"mycelix:economic-intent:v1\0";
const EFFECT_ID_DOMAIN: &[u8] = b"mycelix:economic-effect:v1\0";
const CLAIM_PROFILE_DIGEST_DOMAIN: &[u8] = b"mycelix:economic-claim-profile:v1\0";

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct DigestV1(pub [u8; 32]);

impl DigestV1 {
    pub fn as_bytes(&self) -> &[u8; 32] {
        &self.0
    }

    pub fn to_hex(self) -> String {
        let mut out = String::with_capacity(64);
        for byte in self.0 {
            use std::fmt::Write as _;
            let _ = write!(&mut out, "{byte:02x}");
        }
        out
    }
}

/// Stable key material supplied by the authority that originates an obligation.
///
/// The stable obligation reference MUST be created before the first value-moving
/// attempt and MUST NOT be regenerated from the mutable request payload.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct OperationKeyV1 {
    pub namespace: String,
    pub originating_authority: String,
    pub stable_obligation_reference: String,
}

impl OperationKeyV1 {
    pub fn new(
        namespace: impl Into<String>,
        originating_authority: impl Into<String>,
        stable_obligation_reference: impl Into<String>,
    ) -> Self {
        Self {
            namespace: namespace.into(),
            originating_authority: originating_authority.into(),
            stable_obligation_reference: stable_obligation_reference.into(),
        }
    }

    pub fn derive_identity(&self) -> Result<OperationIdentityV1, KernelErrorV1> {
        validate_component(&self.namespace)?;
        validate_component(&self.originating_authority)?;
        validate_component(&self.stable_obligation_reference)?;

        let mut hasher = blake3::Hasher::new();
        hasher.update(OPERATION_ID_DOMAIN);
        hasher.update(&OPERATION_ID_SCHEMA_V1.to_be_bytes());
        update_field(&mut hasher, self.namespace.as_bytes())?;
        update_field(&mut hasher, self.originating_authority.as_bytes())?;
        update_field(&mut hasher, self.stable_obligation_reference.as_bytes())?;
        Ok(OperationIdentityV1(DigestV1(*hasher.finalize().as_bytes())))
    }

    /// Bind the stable obligation reference to its immutable economic terms.
    /// This prevents a caller from accidentally pairing an operation key for one
    /// obligation with a fingerprint that describes another obligation.
    pub fn bind_intent(
        &self,
        terms: &ImmutableIntentTermsV1,
    ) -> Result<BoundOperationIntentV1, KernelErrorV1> {
        if self.stable_obligation_reference.as_str() != terms.source_obligation_reference.as_str() {
            return Err(KernelErrorV1::ObligationReferenceMismatch);
        }
        Ok(BoundOperationIntentV1 {
            operation_id: self.derive_identity()?,
            intent_fingerprint: terms.fingerprint()?,
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BoundOperationIntentV1 {
    pub operation_id: OperationIdentityV1,
    pub intent_fingerprint: DigestV1,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct OperationIdentityV1(pub DigestV1);

impl OperationIdentityV1 {
    pub fn digest(&self) -> DigestV1 {
        self.0
    }
}

/// Immutable economic terms included in the intent fingerprint.
///
/// Amounts are integer minor units. Currency/instrument, unit, settlement profile
/// and policy revision are explicit fields; display formatting and floating point
/// are intentionally absent.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ImmutableIntentTermsV1 {
    pub settlement_profile: String,
    pub payer_owner: String,
    pub beneficiary_owner: String,
    pub instrument: String,
    pub unit: String,
    pub amount_minor_units: u64,
    pub purpose_code: String,
    pub source_obligation_reference: String,
    pub policy_revision: String,
    pub terms_digest: DigestV1,
}

impl ImmutableIntentTermsV1 {
    pub fn fingerprint(&self) -> Result<DigestV1, KernelErrorV1> {
        if self.amount_minor_units == 0 {
            return Err(KernelErrorV1::ZeroAmount);
        }

        for value in [
            &self.settlement_profile,
            &self.payer_owner,
            &self.beneficiary_owner,
            &self.instrument,
            &self.unit,
            &self.purpose_code,
            &self.source_obligation_reference,
            &self.policy_revision,
        ] {
            validate_component(value)?;
        }

        let mut hasher = blake3::Hasher::new();
        hasher.update(INTENT_FINGERPRINT_DOMAIN);
        hasher.update(&INTENT_FINGERPRINT_SCHEMA_V1.to_be_bytes());
        update_field(&mut hasher, self.settlement_profile.as_bytes())?;
        update_field(&mut hasher, self.payer_owner.as_bytes())?;
        update_field(&mut hasher, self.beneficiary_owner.as_bytes())?;
        update_field(&mut hasher, self.instrument.as_bytes())?;
        update_field(&mut hasher, self.unit.as_bytes())?;
        hasher.update(&self.amount_minor_units.to_be_bytes());
        update_field(&mut hasher, self.purpose_code.as_bytes())?;
        update_field(&mut hasher, self.source_obligation_reference.as_bytes())?;
        update_field(&mut hasher, self.policy_revision.as_bytes())?;
        hasher.update(self.terms_digest.as_bytes());

        Ok(DigestV1(*hasher.finalize().as_bytes()))
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum EffectRoleV1 {
    PayerDebit,
    BeneficiaryOutput,
    BeneficiaryClaim,
    TreasuryFee,
    PoolContribution,
    PoolDisbursement,
    ExternalSettlementObservation,
}

impl EffectRoleV1 {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::PayerDebit => "payer_debit",
            Self::BeneficiaryOutput => "beneficiary_output",
            Self::BeneficiaryClaim => "beneficiary_claim",
            Self::TreasuryFee => "treasury_fee",
            Self::PoolContribution => "pool_contribution",
            Self::PoolDisbursement => "pool_disbursement",
            Self::ExternalSettlementObservation => "external_settlement_observation",
        }
    }
}

/// An effect is scoped by the original obligation, one semantic role and its owner.
/// Different owners/roles intentionally produce different effect IDs.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct EffectIdentityV1(pub DigestV1);

impl EffectIdentityV1 {
    pub fn derive(
        operation: OperationIdentityV1,
        role: EffectRoleV1,
        effect_owner: &str,
    ) -> Result<Self, KernelErrorV1> {
        validate_component(effect_owner)?;

        let mut hasher = blake3::Hasher::new();
        hasher.update(EFFECT_ID_DOMAIN);
        hasher.update(&EFFECT_ID_SCHEMA_V1.to_be_bytes());
        hasher.update(operation.0.as_bytes());
        update_field(&mut hasher, role.as_str().as_bytes())?;
        update_field(&mut hasher, effect_owner.as_bytes())?;
        Ok(Self(DigestV1(*hasher.finalize().as_bytes())))
    }

    pub fn digest(&self) -> DigestV1 {
        self.0
    }
}

/// The adapter's declared completeness of the evidence view.
///
/// CompleteForDeclaredAuthority is an assertion made by the adapter. This pure
/// crate cannot verify DHT authority coverage, source-chain completeness, remote
/// service health or fork discovery. Incorrectly asserting completeness invalidates
/// any projection based on that assertion.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EvidenceCompletenessV1 {
    CompleteForDeclaredAuthority,
    Incomplete,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct OperationBindingObservationV1 {
    pub operation_id: OperationIdentityV1,
    pub intent_fingerprint: DigestV1,
    pub evidence_reference: String,
    pub evidence_digest: DigestV1,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum BindingConflictReasonV1 {
    IntentFingerprintMismatch,
    EvidenceReferenceMutation,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum BindingProjectionV1 {
    /// No matching binding was observed in a view the adapter declared complete.
    /// This is NOT an admission capability and is NOT proof of global absence.
    NoBindingObserved,
    /// Existing evidence binds this operation to the requested fingerprint.
    ExactReplay { evidence_references: Vec<String> },
    /// A known observation contradicts the requested intent/evidence.
    Conflict {
        reason: BindingConflictReasonV1,
        evidence_references: Vec<String>,
    },
    /// The adapter cannot establish a complete view; absence and replay cannot be
    /// promoted to authoritative outcomes.
    Indeterminate { evidence_references: Vec<String> },
}

/// Classify observations for one operation identity.
///
/// The caller must authenticate each observation and justify the completeness
/// declaration. The function only checks internal consistency and never authorizes
/// a new effect.
pub fn classify_operation_binding(
    operation_id: OperationIdentityV1,
    expected_fingerprint: DigestV1,
    observations: &[OperationBindingObservationV1],
    completeness: EvidenceCompletenessV1,
) -> Result<BindingProjectionV1, KernelErrorV1> {
    let mut by_reference: BTreeMap<String, OperationBindingObservationV1> = BTreeMap::new();

    for observation in observations {
        validate_component(&observation.evidence_reference)?;
        if observation.operation_id != operation_id {
            return Err(KernelErrorV1::ObservationOperationMismatch);
        }

        match by_reference.get(&observation.evidence_reference) {
            Some(existing) if existing == observation => {}
            Some(_) => {
                let refs = sorted_references(
                    by_reference
                        .keys()
                        .cloned()
                        .chain(std::iter::once(observation.evidence_reference.clone())),
                );
                return Ok(BindingProjectionV1::Conflict {
                    reason: BindingConflictReasonV1::EvidenceReferenceMutation,
                    evidence_references: refs,
                });
            }
            None => {
                by_reference.insert(observation.evidence_reference.clone(), observation.clone());
            }
        }
    }

    let mut fingerprints = BTreeSet::new();
    for observation in by_reference.values() {
        fingerprints.insert(observation.intent_fingerprint);
    }

    if fingerprints
        .iter()
        .any(|fingerprint| *fingerprint != expected_fingerprint)
    {
        return Ok(BindingProjectionV1::Conflict {
            reason: BindingConflictReasonV1::IntentFingerprintMismatch,
            evidence_references: by_reference.keys().cloned().collect(),
        });
    }

    let references: Vec<String> = by_reference.keys().cloned().collect();
    if completeness == EvidenceCompletenessV1::Incomplete {
        return Ok(BindingProjectionV1::Indeterminate {
            evidence_references: references,
        });
    }

    if references.is_empty() {
        Ok(BindingProjectionV1::NoBindingObserved)
    } else {
        Ok(BindingProjectionV1::ExactReplay {
            evidence_references: references,
        })
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct EffectObservationV1 {
    pub effect_id: EffectIdentityV1,
    pub intent_fingerprint: DigestV1,
    /// Digest of the immutable effect facts, not an API response body.
    pub effect_digest: DigestV1,
    pub receipt_reference: String,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum EffectConflictReasonV1 {
    IntentFingerprintMismatch,
    EffectDigestMismatch,
    ReceiptReferenceMutation,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum EffectProjectionV1 {
    /// No receipt was observed in a view declared complete by the adapter. This
    /// does not independently prove that no remote effect committed.
    NoEffectObserved,
    /// One effect commitment was observed. Multiple receipts may be duplicate
    /// physical evidence for that same immutable effect.
    EffectObserved {
        effect_digest: DigestV1,
        receipt_references: Vec<String>,
    },
    Conflict {
        reason: EffectConflictReasonV1,
        receipt_references: Vec<String>,
    },
    Indeterminate {
        receipt_references: Vec<String>,
    },
}

/// Collapse duplicate physical receipts for one stable effect identity while
/// retaining contradictory facts as an explicit conflict.
pub fn classify_effect_observations(
    effect_id: EffectIdentityV1,
    expected_fingerprint: DigestV1,
    observations: &[EffectObservationV1],
    completeness: EvidenceCompletenessV1,
) -> Result<EffectProjectionV1, KernelErrorV1> {
    let mut by_reference: BTreeMap<String, EffectObservationV1> = BTreeMap::new();

    for observation in observations {
        validate_component(&observation.receipt_reference)?;
        if observation.effect_id != effect_id {
            return Err(KernelErrorV1::ObservationEffectMismatch);
        }
        match by_reference.get(&observation.receipt_reference) {
            Some(existing) if existing == observation => {}
            Some(_) => {
                return Ok(EffectProjectionV1::Conflict {
                    reason: EffectConflictReasonV1::ReceiptReferenceMutation,
                    receipt_references: sorted_references(
                        by_reference
                            .keys()
                            .cloned()
                            .chain(std::iter::once(observation.receipt_reference.clone())),
                    ),
                });
            }
            None => {
                by_reference.insert(observation.receipt_reference.clone(), observation.clone());
            }
        }
    }

    let refs: Vec<String> = by_reference.keys().cloned().collect();
    if by_reference
        .values()
        .any(|observation| observation.intent_fingerprint != expected_fingerprint)
    {
        return Ok(EffectProjectionV1::Conflict {
            reason: EffectConflictReasonV1::IntentFingerprintMismatch,
            receipt_references: refs,
        });
    }

    let digests: BTreeSet<DigestV1> = by_reference
        .values()
        .map(|observation| observation.effect_digest)
        .collect();
    if digests.len() > 1 {
        return Ok(EffectProjectionV1::Conflict {
            reason: EffectConflictReasonV1::EffectDigestMismatch,
            receipt_references: refs,
        });
    }

    if completeness == EvidenceCompletenessV1::Incomplete {
        return Ok(EffectProjectionV1::Indeterminate {
            receipt_references: refs,
        });
    }

    match digests.iter().next() {
        None => Ok(EffectProjectionV1::NoEffectObserved),
        Some(effect_digest) => Ok(EffectProjectionV1::EffectObserved {
            effect_digest: *effect_digest,
            receipt_references: refs,
        }),
    }
}

fn validate_component(value: &str) -> Result<(), KernelErrorV1> {
    if value.is_empty() {
        return Err(KernelErrorV1::EmptyComponent);
    }
    if value.len() > MAX_COMPONENT_BYTES {
        return Err(KernelErrorV1::ComponentTooLong);
    }
    Ok(())
}

fn update_field(hasher: &mut blake3::Hasher, value: &[u8]) -> Result<(), KernelErrorV1> {
    if value.len() > MAX_COMPONENT_BYTES || value.len() > u32::MAX as usize {
        return Err(KernelErrorV1::ComponentTooLong);
    }
    hasher.update(&(value.len() as u32).to_be_bytes());
    hasher.update(value);
    Ok(())
}

fn sorted_references<I>(references: I) -> Vec<String>
where
    I: IntoIterator<Item = String>,
{
    references
        .into_iter()
        .collect::<BTreeSet<_>>()
        .into_iter()
        .collect()
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum KernelErrorV1 {
    EmptyComponent,
    ComponentTooLong,
    ZeroAmount,
    ObservationOperationMismatch,
    ObservationEffectMismatch,
    ObligationReferenceMismatch,
    EmptyRequiredEffects,
    DuplicateRequiredEffect,
    DuplicateEffectEvidenceBatch,
    UnexpectedEffectEvidenceBatch,
    ProfileDigestMismatch,
    TooManyRequiredEffects,
}

/// One owner-specific effect required by an explicit composite claim profile.
/// The effect identity is derived from the existing FIN-EFFECT-001 algorithm;
/// this type introduces no new identity namespace or hash domain.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RequiredEffectV1 {
    pub role: EffectRoleV1,
    pub effect_owner: String,
}

/// A caller-reviewed, versioned statement of which effects support one claim.
///
/// The kernel does not verify that this profile is authorized or complete. Its
/// digest is carried into the output so downstream systems can bind the exact
/// profile used. A successful result means only "supported within this declared
/// scope", never "globally complete", "settled", "delivered" or "spendable".
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CompositeClaimProfileV1 {
    pub profile_id: String,
    pub profile_digest: DigestV1,
    pub claim_name: String,
    pub operation_id: OperationIdentityV1,
    pub expected_intent_fingerprint: DigestV1,
    pub required_effects: Vec<RequiredEffectV1>,
}

impl CompositeClaimProfileV1 {
    /// Compute a canonical digest over the complete claim profile.
    ///
    /// Required effects are ordered by their existing FIN-EFFECT-001 effect IDs,
    /// so input order cannot alter the profile digest. This digest binds only the
    /// profile's declared scope; it does not prove that the profile is approved,
    /// complete, or legally sufficient.
    pub fn derive_profile_digest(&self) -> Result<DigestV1, KernelErrorV1> {
        validate_component(&self.profile_id)?;
        validate_component(&self.claim_name)?;
        if self.required_effects.is_empty() {
            return Err(KernelErrorV1::EmptyRequiredEffects);
        }

        let mut required = Vec::with_capacity(self.required_effects.len());
        let mut seen = BTreeSet::new();
        for requirement in &self.required_effects {
            validate_component(&requirement.effect_owner)?;
            let effect_id = EffectIdentityV1::derive(
                self.operation_id,
                requirement.role,
                &requirement.effect_owner,
            )?;
            if !seen.insert(effect_id) {
                return Err(KernelErrorV1::DuplicateRequiredEffect);
            }
            required.push((
                effect_id,
                requirement.role,
                requirement.effect_owner.as_str(),
            ));
        }
        required.sort_by_key(|(effect_id, _, _)| *effect_id);

        let count =
            u32::try_from(required.len()).map_err(|_| KernelErrorV1::TooManyRequiredEffects)?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(CLAIM_PROFILE_DIGEST_DOMAIN);
        hasher.update(&CLAIM_PROFILE_DIGEST_SCHEMA_V1.to_be_bytes());
        update_field(&mut hasher, self.profile_id.as_bytes())?;
        update_field(&mut hasher, self.claim_name.as_bytes())?;
        hasher.update(self.operation_id.digest().as_bytes());
        hasher.update(self.expected_intent_fingerprint.as_bytes());
        hasher.update(&count.to_be_bytes());

        for (effect_id, role, owner) in required {
            hasher.update(effect_id.digest().as_bytes());
            update_field(&mut hasher, role.as_str().as_bytes())?;
            update_field(&mut hasher, owner.as_bytes())?;
        }

        Ok(DigestV1(*hasher.finalize().as_bytes()))
    }
}

/// Evidence supplied for one effect slot. Completeness is an adapter assertion
/// and remains outside the authority of this pure kernel.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct EffectEvidenceBatchV1 {
    pub effect_id: EffectIdentityV1,
    pub completeness: EvidenceCompletenessV1,
    pub observations: Vec<EffectObservationV1>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct EffectVectorEntryV1 {
    pub effect_id: EffectIdentityV1,
    pub role: EffectRoleV1,
    pub effect_owner: String,
    pub projection: EffectProjectionV1,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CompositeClaimDispositionV1 {
    /// Every effect required by this exact profile was observed consistently.
    /// This is not authorization, legal finality, delivery, or spendability.
    SupportedWithinDeclaredScope { effect_ids: Vec<EffectIdentityV1> },
    /// A declared-complete evidence view did not contain one or more required
    /// effects. This is a pending projection, not permission to create an effect.
    Pending { effect_ids: Vec<EffectIdentityV1> },
    /// One or more required evidence views are absent or incomplete.
    Indeterminate { effect_ids: Vec<EffectIdentityV1> },
    /// At least one required effect has contradictory evidence.
    /// Conflict dominates every less severe aggregate disposition.
    Conflict { effect_ids: Vec<EffectIdentityV1> },
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct EffectVectorProjectionV1 {
    pub profile_id: String,
    pub profile_digest: DigestV1,
    pub claim_name: String,
    pub operation_id: OperationIdentityV1,
    pub intent_fingerprint: DigestV1,
    pub disposition: CompositeClaimDispositionV1,
    /// Entries are sorted by stable effect identity for deterministic replay.
    pub entries: Vec<EffectVectorEntryV1>,
}

/// Project a profile-bound set of independently owned effects.
///
/// Aggregation precedence is Conflict > Indeterminate > Pending >
/// SupportedWithinDeclaredScope. The function is deterministic over the supplied
/// observations. It neither authorizes a first effect nor authenticates sources,
/// verifies that the profile is complete, or proves the adapter's view is complete.
pub fn classify_effect_vector(
    profile: &CompositeClaimProfileV1,
    batches: &[EffectEvidenceBatchV1],
) -> Result<EffectVectorProjectionV1, KernelErrorV1> {
    let derived_profile_digest = profile.derive_profile_digest()?;
    if derived_profile_digest != profile.profile_digest {
        return Err(KernelErrorV1::ProfileDigestMismatch);
    }

    let mut required = Vec::with_capacity(profile.required_effects.len());
    let mut required_ids = BTreeSet::new();
    for requirement in &profile.required_effects {
        validate_component(&requirement.effect_owner)?;
        let effect_id = EffectIdentityV1::derive(
            profile.operation_id,
            requirement.role,
            &requirement.effect_owner,
        )?;
        if !required_ids.insert(effect_id) {
            return Err(KernelErrorV1::DuplicateRequiredEffect);
        }
        required.push((
            effect_id,
            requirement.role,
            requirement.effect_owner.clone(),
        ));
    }
    required.sort_by_key(|(effect_id, _, _)| *effect_id);

    let mut batch_by_id = BTreeMap::new();
    for batch in batches {
        if !required_ids.contains(&batch.effect_id) {
            return Err(KernelErrorV1::UnexpectedEffectEvidenceBatch);
        }
        if batch_by_id.insert(batch.effect_id, batch).is_some() {
            return Err(KernelErrorV1::DuplicateEffectEvidenceBatch);
        }
    }

    let mut entries = Vec::with_capacity(required.len());
    let mut conflicts = Vec::new();
    let mut indeterminate = Vec::new();
    let mut pending = Vec::new();

    for (effect_id, role, effect_owner) in required {
        let projection = match batch_by_id.get(&effect_id) {
            Some(batch) => classify_effect_observations(
                effect_id,
                profile.expected_intent_fingerprint,
                &batch.observations,
                batch.completeness,
            )?,
            None => EffectProjectionV1::Indeterminate {
                receipt_references: Vec::new(),
            },
        };

        match &projection {
            EffectProjectionV1::Conflict { .. } => conflicts.push(effect_id),
            EffectProjectionV1::Indeterminate { .. } => indeterminate.push(effect_id),
            EffectProjectionV1::NoEffectObserved => pending.push(effect_id),
            EffectProjectionV1::EffectObserved { .. } => {}
        }

        entries.push(EffectVectorEntryV1 {
            effect_id,
            role,
            effect_owner,
            projection,
        });
    }

    let all_effect_ids: Vec<EffectIdentityV1> =
        entries.iter().map(|entry| entry.effect_id).collect();

    let disposition = if !conflicts.is_empty() {
        CompositeClaimDispositionV1::Conflict {
            effect_ids: conflicts,
        }
    } else if !indeterminate.is_empty() {
        CompositeClaimDispositionV1::Indeterminate {
            effect_ids: indeterminate,
        }
    } else if !pending.is_empty() {
        CompositeClaimDispositionV1::Pending {
            effect_ids: pending,
        }
    } else {
        CompositeClaimDispositionV1::SupportedWithinDeclaredScope {
            effect_ids: all_effect_ids,
        }
    };

    Ok(EffectVectorProjectionV1 {
        profile_id: profile.profile_id.clone(),
        profile_digest: profile.profile_digest,
        claim_name: profile.claim_name.clone(),
        operation_id: profile.operation_id,
        intent_fingerprint: profile.expected_intent_fingerprint,
        disposition,
        entries,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn terms() -> ImmutableIntentTermsV1 {
        ImmutableIntentTermsV1 {
            settlement_profile: "BilateralOwnerClaimV1".into(),
            payer_owner: "did:mycelix:alice".into(),
            beneficiary_owner: "did:mycelix:bob".into(),
            instrument: "SAP".into(),
            unit: "micro-SAP".into(),
            amount_minor_units: 2_500,
            purpose_code: "hearth_contribution".into(),
            source_obligation_reference: "trade:42:attempt:1".into(),
            policy_revision: "policy:hearth:7".into(),
            terms_digest: DigestV1([7; 32]),
        }
    }

    fn operation() -> OperationIdentityV1 {
        OperationKeyV1::new("finance", "did:mycelix:origin", "trade:42:attempt:1")
            .derive_identity()
            .unwrap()
    }

    #[test]
    fn operation_identity_is_independent_of_mutable_intent_terms() {
        let first = operation();
        let mut changed = terms();
        changed.amount_minor_units += 1;
        let second = OperationKeyV1::new("finance", "did:mycelix:origin", "trade:42:attempt:1")
            .derive_identity()
            .unwrap();
        assert_eq!(first, second);
        assert_ne!(
            terms().fingerprint().unwrap(),
            changed.fingerprint().unwrap()
        );
    }

    #[test]
    fn binding_rejects_a_fingerprint_for_a_different_obligation() {
        let key = OperationKeyV1::new("finance", "did:mycelix:origin", "trade:42:settlement:1");
        let mut mismatched = terms();
        mismatched.source_obligation_reference = "trade:43:settlement:1".into();
        assert_eq!(
            key.bind_intent(&mismatched),
            Err(KernelErrorV1::ObligationReferenceMismatch)
        );
    }

    #[test]
    fn binding_returns_operation_identity_and_exact_fingerprint_together() {
        let key = OperationKeyV1::new("finance", "did:mycelix:origin", "trade:42:attempt:1");
        let bound = key.bind_intent(&terms()).unwrap();
        assert_eq!(bound.operation_id, operation());
        assert_eq!(bound.intent_fingerprint, terms().fingerprint().unwrap());
    }

    #[test]
    fn fingerprint_changes_when_amount_beneficiary_or_profile_changes() {
        let original = terms().fingerprint().unwrap();
        let mut amount = terms();
        amount.amount_minor_units += 1;
        let mut beneficiary = terms();
        beneficiary.beneficiary_owner = "did:mycelix:carol".into();
        let mut profile = terms();
        profile.settlement_profile = "FinanceLedgerAtomicV1".into();

        assert_ne!(original, amount.fingerprint().unwrap());
        assert_ne!(original, beneficiary.fingerprint().unwrap());
        assert_ne!(original, profile.fingerprint().unwrap());
    }

    #[test]
    fn operation_identity_changes_when_any_key_component_changes() {
        let original = OperationKeyV1::new("finance", "did:mycelix:origin", "trade:42:attempt:1")
            .derive_identity()
            .unwrap();
        let variants = [
            OperationKeyV1::new("treasury", "did:mycelix:origin", "trade:42:attempt:1"),
            OperationKeyV1::new("finance", "did:mycelix:other-origin", "trade:42:attempt:1"),
            OperationKeyV1::new("finance", "did:mycelix:origin", "trade:43:attempt:1"),
        ];
        for variant in variants {
            assert_ne!(original, variant.derive_identity().unwrap());
        }
    }

    #[test]
    fn fingerprint_changes_when_every_immutable_field_changes() {
        let original = terms().fingerprint().unwrap();

        let mut settlement_profile = terms();
        settlement_profile.settlement_profile.push_str("_changed");
        let mut payer = terms();
        payer.payer_owner.push_str(":changed");
        let mut beneficiary = terms();
        beneficiary.beneficiary_owner.push_str(":changed");
        let mut instrument = terms();
        instrument.instrument.push_str("_changed");
        let mut unit = terms();
        unit.unit.push_str("_changed");
        let mut amount = terms();
        amount.amount_minor_units += 1;
        let mut purpose = terms();
        purpose.purpose_code.push_str("_changed");
        let mut source_obligation = terms();
        source_obligation
            .source_obligation_reference
            .push_str("_changed");
        let mut policy_revision = terms();
        policy_revision.policy_revision.push_str("_changed");
        let mut terms_digest = terms();
        terms_digest.terms_digest = DigestV1([8; 32]);

        let variants = [
            settlement_profile,
            payer,
            beneficiary,
            instrument,
            unit,
            amount,
            purpose,
            source_obligation,
            policy_revision,
            terms_digest,
        ];
        for variant in variants {
            assert_ne!(original, variant.fingerprint().unwrap());
        }
    }

    #[test]
    fn invalid_or_zero_value_terms_are_rejected() {
        let mut zero = terms();
        zero.amount_minor_units = 0;
        assert_eq!(zero.fingerprint(), Err(KernelErrorV1::ZeroAmount));

        let mut empty = terms();
        empty.payer_owner.clear();
        assert_eq!(empty.fingerprint(), Err(KernelErrorV1::EmptyComponent));
    }

    #[test]
    fn effect_identity_is_scoped_by_role_and_owner() {
        let op = operation();
        let payer =
            EffectIdentityV1::derive(op, EffectRoleV1::PayerDebit, "did:mycelix:alice").unwrap();
        let beneficiary =
            EffectIdentityV1::derive(op, EffectRoleV1::BeneficiaryClaim, "did:mycelix:bob")
                .unwrap();
        let other_owner =
            EffectIdentityV1::derive(op, EffectRoleV1::PayerDebit, "did:mycelix:carol").unwrap();

        assert_ne!(payer, beneficiary);
        assert_ne!(payer, other_owner);
    }

    #[test]
    fn exact_same_binding_is_a_replay() {
        let op = operation();
        let fingerprint = terms().fingerprint().unwrap();
        let observation = OperationBindingObservationV1 {
            operation_id: op,
            intent_fingerprint: fingerprint,
            evidence_reference: "action:binding:1".into(),
            evidence_digest: DigestV1([9; 32]),
        };
        let result = classify_operation_binding(
            op,
            fingerprint,
            &[observation.clone(), observation],
            EvidenceCompletenessV1::CompleteForDeclaredAuthority,
        )
        .unwrap();

        assert_eq!(
            result,
            BindingProjectionV1::ExactReplay {
                evidence_references: vec!["action:binding:1".into()]
            }
        );
    }

    #[test]
    fn same_operation_identity_with_changed_fingerprint_is_a_conflict() {
        let op = operation();
        let expected = terms().fingerprint().unwrap();
        let mut changed = terms();
        changed.amount_minor_units += 1;
        let observed = OperationBindingObservationV1 {
            operation_id: op,
            intent_fingerprint: changed.fingerprint().unwrap(),
            evidence_reference: "action:binding:changed".into(),
            evidence_digest: DigestV1([8; 32]),
        };
        assert!(matches!(
            classify_operation_binding(
                op,
                expected,
                &[observed],
                EvidenceCompletenessV1::CompleteForDeclaredAuthority
            )
            .unwrap(),
            BindingProjectionV1::Conflict {
                reason: BindingConflictReasonV1::IntentFingerprintMismatch,
                ..
            }
        ));
    }

    #[test]
    fn incomplete_empty_view_is_indeterminate_not_admission() {
        let result = classify_operation_binding(
            operation(),
            terms().fingerprint().unwrap(),
            &[],
            EvidenceCompletenessV1::Incomplete,
        )
        .unwrap();
        assert_eq!(
            result,
            BindingProjectionV1::Indeterminate {
                evidence_references: vec![]
            }
        );
    }

    #[test]
    fn complete_empty_view_is_only_no_binding_observed() {
        assert_eq!(
            classify_operation_binding(
                operation(),
                terms().fingerprint().unwrap(),
                &[],
                EvidenceCompletenessV1::CompleteForDeclaredAuthority
            )
            .unwrap(),
            BindingProjectionV1::NoBindingObserved
        );
    }

    #[test]
    fn one_evidence_reference_cannot_change_payload() {
        let op = operation();
        let fingerprint = terms().fingerprint().unwrap();
        let first = OperationBindingObservationV1 {
            operation_id: op,
            intent_fingerprint: fingerprint,
            evidence_reference: "action:binding:1".into(),
            evidence_digest: DigestV1([1; 32]),
        };
        let second = OperationBindingObservationV1 {
            evidence_digest: DigestV1([2; 32]),
            ..first.clone()
        };
        assert!(matches!(
            classify_operation_binding(
                op,
                fingerprint,
                &[first, second],
                EvidenceCompletenessV1::CompleteForDeclaredAuthority
            )
            .unwrap(),
            BindingProjectionV1::Conflict {
                reason: BindingConflictReasonV1::EvidenceReferenceMutation,
                ..
            }
        ));
    }

    #[test]
    fn duplicate_effect_receipts_collapse_by_reference_and_digest() {
        let effect = EffectIdentityV1::derive(
            operation(),
            EffectRoleV1::TreasuryFee,
            "did:mycelix:treasury",
        )
        .unwrap();
        let fingerprint = terms().fingerprint().unwrap();
        let observation = EffectObservationV1 {
            effect_id: effect,
            intent_fingerprint: fingerprint,
            effect_digest: DigestV1([4; 32]),
            receipt_reference: "receipt:treasury:1".into(),
        };
        assert_eq!(
            classify_effect_observations(
                effect,
                fingerprint,
                &[observation.clone(), observation],
                EvidenceCompletenessV1::CompleteForDeclaredAuthority
            )
            .unwrap(),
            EffectProjectionV1::EffectObserved {
                effect_digest: DigestV1([4; 32]),
                receipt_references: vec!["receipt:treasury:1".into()]
            }
        );
    }

    #[test]
    fn conflicting_effect_digests_freeze_the_effect_identity() {
        let effect = EffectIdentityV1::derive(
            operation(),
            EffectRoleV1::PoolDisbursement,
            "did:mycelix:hearth",
        )
        .unwrap();
        let fingerprint = terms().fingerprint().unwrap();
        let observations = [
            EffectObservationV1 {
                effect_id: effect,
                intent_fingerprint: fingerprint,
                effect_digest: DigestV1([1; 32]),
                receipt_reference: "receipt:pool:a".into(),
            },
            EffectObservationV1 {
                effect_id: effect,
                intent_fingerprint: fingerprint,
                effect_digest: DigestV1([2; 32]),
                receipt_reference: "receipt:pool:b".into(),
            },
        ];
        assert!(matches!(
            classify_effect_observations(
                effect,
                fingerprint,
                &observations,
                EvidenceCompletenessV1::CompleteForDeclaredAuthority
            )
            .unwrap(),
            EffectProjectionV1::Conflict {
                reason: EffectConflictReasonV1::EffectDigestMismatch,
                ..
            }
        ));
    }

    #[test]
    fn incomplete_effect_view_never_promotes_to_committed_or_absent() {
        let effect = EffectIdentityV1::derive(
            operation(),
            EffectRoleV1::BeneficiaryClaim,
            "did:mycelix:bob",
        )
        .unwrap();
        assert_eq!(
            classify_effect_observations(
                effect,
                terms().fingerprint().unwrap(),
                &[],
                EvidenceCompletenessV1::Incomplete
            )
            .unwrap(),
            EffectProjectionV1::Indeterminate {
                receipt_references: vec![]
            }
        );
    }

    #[test]
    fn observation_for_another_operation_is_rejected() {
        let op = operation();
        let wrong = OperationKeyV1::new("finance", "did:mycelix:origin", "trade:43")
            .derive_identity()
            .unwrap();
        let fp = terms().fingerprint().unwrap();
        let binding = OperationBindingObservationV1 {
            operation_id: wrong,
            intent_fingerprint: fp,
            evidence_reference: "action:other".into(),
            evidence_digest: DigestV1([3; 32]),
        };
        assert_eq!(
            classify_operation_binding(
                op,
                fp,
                &[binding],
                EvidenceCompletenessV1::CompleteForDeclaredAuthority
            ),
            Err(KernelErrorV1::ObservationOperationMismatch)
        );
    }

    #[test]
    fn component_length_is_bounded() {
        let key = OperationKeyV1::new("finance", "did:mycelix:origin", "x".repeat(1025));
        assert_eq!(key.derive_identity(), Err(KernelErrorV1::ComponentTooLong));
    }

    #[test]
    fn canonical_v1_known_answer_vectors_are_stable() {
        let operation = operation();
        let fingerprint = terms().fingerprint().unwrap();
        let payer_effect =
            EffectIdentityV1::derive(operation, EffectRoleV1::PayerDebit, "did:mycelix:alice")
                .unwrap();
        let beneficiary_effect =
            EffectIdentityV1::derive(operation, EffectRoleV1::BeneficiaryClaim, "did:mycelix:bob")
                .unwrap();

        assert_eq!(
            operation.digest().to_hex(),
            "15a08e9c3bf2599c15b37cb700698aec1898cd4c66b15ebf5208568e4953f204"
        );
        assert_eq!(
            fingerprint.to_hex(),
            "6a52f1479b7a407d6de4f2650b61bf3203741c80e28f836db523f58625e62235"
        );
        assert_eq!(
            payer_effect.digest().to_hex(),
            "984fcc30e27b1ccbb5b266f6b0960c5791be3a68ada8512aa0fb39dcf575e56a"
        );
        assert_eq!(
            beneficiary_effect.digest().to_hex(),
            "35ddfe65d8d3beb9ce06fcb1245802d0cc7799d8366bef9087ee5f6d63e6f24b"
        );
    }

    #[test]
    fn blake3_reference_empty_input_vector_is_stable() {
        assert_eq!(
            blake3::hash(b"").to_hex().as_str(),
            "af1349b9f5f9a1a6a0404dea36dcc9499bcb25c9adc112b7cc9a93cae41f3262"
        );
    }
}

#[cfg(test)]
mod effect_vector_tests {
    use super::*;

    fn required(role: EffectRoleV1, owner: &str) -> RequiredEffectV1 {
        RequiredEffectV1 {
            role,
            effect_owner: owner.into(),
        }
    }

    fn profile(required_effects: Vec<RequiredEffectV1>) -> CompositeClaimProfileV1 {
        let mut profile = CompositeClaimProfileV1 {
            profile_id: "claim:bilateral-payment:v1".into(),
            profile_digest: DigestV1([0; 32]),
            claim_name: "bilateral_payment_reconciled".into(),
            operation_id: OperationKeyV1::new(
                "finance",
                "did:mycelix:origin",
                "trade:42:attempt:1",
            )
            .derive_identity()
            .unwrap(),
            expected_intent_fingerprint: ImmutableIntentTermsV1 {
                settlement_profile: "BilateralOwnerClaimV1".into(),
                payer_owner: "did:mycelix:alice".into(),
                beneficiary_owner: "did:mycelix:bob".into(),
                instrument: "SAP".into(),
                unit: "micro-SAP".into(),
                amount_minor_units: 2_500,
                purpose_code: "hearth_contribution".into(),
                source_obligation_reference: "trade:42:attempt:1".into(),
                policy_revision: "policy:hearth:7".into(),
                terms_digest: DigestV1([7; 32]),
            }
            .fingerprint()
            .unwrap(),
            required_effects,
        };
        profile.profile_digest = profile.derive_profile_digest().unwrap();
        profile
    }

    fn batch(
        profile: &CompositeClaimProfileV1,
        role: EffectRoleV1,
        owner: &str,
        effect_digest: DigestV1,
        reference: &str,
    ) -> EffectEvidenceBatchV1 {
        let effect_id = EffectIdentityV1::derive(profile.operation_id, role, owner).unwrap();
        EffectEvidenceBatchV1 {
            effect_id,
            completeness: EvidenceCompletenessV1::CompleteForDeclaredAuthority,
            observations: vec![EffectObservationV1 {
                effect_id,
                intent_fingerprint: profile.expected_intent_fingerprint,
                effect_digest,
                receipt_reference: reference.into(),
            }],
        }
    }

    fn empty_batch(
        profile: &CompositeClaimProfileV1,
        role: EffectRoleV1,
        owner: &str,
        completeness: EvidenceCompletenessV1,
    ) -> EffectEvidenceBatchV1 {
        let effect_id = EffectIdentityV1::derive(profile.operation_id, role, owner).unwrap();
        EffectEvidenceBatchV1 {
            effect_id,
            completeness,
            observations: vec![],
        }
    }

    #[test]
    fn vector_is_supported_only_when_every_profile_required_effect_is_observed() {
        let p = profile(vec![
            required(EffectRoleV1::PayerDebit, "did:mycelix:alice"),
            required(EffectRoleV1::BeneficiaryClaim, "did:mycelix:bob"),
        ]);
        let batches = vec![
            batch(
                &p,
                EffectRoleV1::PayerDebit,
                "did:mycelix:alice",
                DigestV1([1; 32]),
                "receipt:payer:1",
            ),
            batch(
                &p,
                EffectRoleV1::BeneficiaryClaim,
                "did:mycelix:bob",
                DigestV1([2; 32]),
                "receipt:beneficiary:1",
            ),
        ];
        let result = classify_effect_vector(&p, &batches).unwrap();
        assert!(matches!(
            result.disposition,
            CompositeClaimDispositionV1::SupportedWithinDeclaredScope { .. }
        ));
        assert_eq!(result.entries.len(), 2);
        assert_eq!(result.profile_digest, p.profile_digest);
        assert_eq!(result.operation_id, p.operation_id);
    }

    #[test]
    fn payer_effect_without_beneficiary_claim_is_pending_not_completed() {
        let p = profile(vec![
            required(EffectRoleV1::PayerDebit, "did:mycelix:alice"),
            required(EffectRoleV1::BeneficiaryClaim, "did:mycelix:bob"),
        ]);
        let payer = batch(
            &p,
            EffectRoleV1::PayerDebit,
            "did:mycelix:alice",
            DigestV1([1; 32]),
            "receipt:payer:1",
        );
        let beneficiary = empty_batch(
            &p,
            EffectRoleV1::BeneficiaryClaim,
            "did:mycelix:bob",
            EvidenceCompletenessV1::CompleteForDeclaredAuthority,
        );
        let result = classify_effect_vector(&p, &[payer, beneficiary]).unwrap();
        assert!(matches!(
            result.disposition,
            CompositeClaimDispositionV1::Pending { .. }
        ));
    }

    #[test]
    fn complete_empty_effect_view_is_pending_and_never_admission_permission() {
        let p = profile(vec![required(
            EffectRoleV1::PayerDebit,
            "did:mycelix:alice",
        )]);
        let effect_id = EffectIdentityV1::derive(
            p.operation_id,
            EffectRoleV1::PayerDebit,
            "did:mycelix:alice",
        )
        .unwrap();
        let result = classify_effect_vector(
            &p,
            &[EffectEvidenceBatchV1 {
                effect_id,
                completeness: EvidenceCompletenessV1::CompleteForDeclaredAuthority,
                observations: vec![],
            }],
        )
        .unwrap();
        assert!(matches!(
            result.disposition,
            CompositeClaimDispositionV1::Pending { .. }
        ));
    }

    #[test]
    fn incomplete_effect_view_is_indeterminate() {
        let p = profile(vec![required(
            EffectRoleV1::BeneficiaryClaim,
            "did:mycelix:bob",
        )]);
        let effect_id = EffectIdentityV1::derive(
            p.operation_id,
            EffectRoleV1::BeneficiaryClaim,
            "did:mycelix:bob",
        )
        .unwrap();
        let result = classify_effect_vector(
            &p,
            &[EffectEvidenceBatchV1 {
                effect_id,
                completeness: EvidenceCompletenessV1::Incomplete,
                observations: vec![],
            }],
        )
        .unwrap();
        assert!(matches!(
            result.disposition,
            CompositeClaimDispositionV1::Indeterminate { .. }
        ));
    }

    #[test]
    fn missing_required_batch_is_indeterminate_not_absent() {
        let p = profile(vec![required(
            EffectRoleV1::ExternalSettlementObservation,
            "bank:example",
        )]);
        let result = classify_effect_vector(&p, &[]).unwrap();
        assert!(matches!(
            result.disposition,
            CompositeClaimDispositionV1::Indeterminate { .. }
        ));
    }

    #[test]
    fn conflict_dominates_indeterminate_across_effect_slots() {
        let p = profile(vec![
            required(EffectRoleV1::PayerDebit, "did:mycelix:alice"),
            required(EffectRoleV1::BeneficiaryClaim, "did:mycelix:bob"),
        ]);
        let mut payer = batch(
            &p,
            EffectRoleV1::PayerDebit,
            "did:mycelix:alice",
            DigestV1([1; 32]),
            "receipt:payer:a",
        );
        let mut conflicting = payer.observations[0].clone();
        conflicting.effect_digest = DigestV1([9; 32]);
        conflicting.receipt_reference = "receipt:payer:b".into();
        payer.observations.push(conflicting);
        let beneficiary_id = EffectIdentityV1::derive(
            p.operation_id,
            EffectRoleV1::BeneficiaryClaim,
            "did:mycelix:bob",
        )
        .unwrap();
        let beneficiary = EffectEvidenceBatchV1 {
            effect_id: beneficiary_id,
            completeness: EvidenceCompletenessV1::Incomplete,
            observations: vec![],
        };
        let result = classify_effect_vector(&p, &[payer, beneficiary]).unwrap();
        assert!(matches!(
            result.disposition,
            CompositeClaimDispositionV1::Conflict { .. }
        ));
    }

    #[test]
    fn changed_intent_fingerprint_cannot_be_promoted_by_vector_layer() {
        let p = profile(vec![required(
            EffectRoleV1::PayerDebit,
            "did:mycelix:alice",
        )]);
        let mut b = batch(
            &p,
            EffectRoleV1::PayerDebit,
            "did:mycelix:alice",
            DigestV1([1; 32]),
            "receipt:payer:1",
        );
        b.observations[0].intent_fingerprint = DigestV1([0xEE; 32]);
        let result = classify_effect_vector(&p, &[b]).unwrap();
        assert!(matches!(
            result.disposition,
            CompositeClaimDispositionV1::Conflict { .. }
        ));
    }

    #[test]
    fn duplicate_required_owner_role_is_rejected() {
        let mut p = profile(vec![required(
            EffectRoleV1::PayerDebit,
            "did:mycelix:alice",
        )]);
        p.required_effects.push(p.required_effects[0].clone());
        assert_eq!(
            classify_effect_vector(&p, &[]),
            Err(KernelErrorV1::DuplicateRequiredEffect)
        );
    }

    #[test]
    fn duplicate_evidence_batches_for_one_effect_are_rejected() {
        let p = profile(vec![required(
            EffectRoleV1::PayerDebit,
            "did:mycelix:alice",
        )]);
        let b = batch(
            &p,
            EffectRoleV1::PayerDebit,
            "did:mycelix:alice",
            DigestV1([1; 32]),
            "receipt:payer:1",
        );
        assert_eq!(
            classify_effect_vector(&p, &[b.clone(), b]),
            Err(KernelErrorV1::DuplicateEffectEvidenceBatch)
        );
    }

    #[test]
    fn evidence_for_an_unrequested_effect_is_rejected() {
        let p = profile(vec![required(
            EffectRoleV1::PayerDebit,
            "did:mycelix:alice",
        )]);
        let unexpected = batch(
            &p,
            EffectRoleV1::TreasuryFee,
            "did:mycelix:treasury",
            DigestV1([1; 32]),
            "receipt:treasury:1",
        );
        assert_eq!(
            classify_effect_vector(&p, &[unexpected]),
            Err(KernelErrorV1::UnexpectedEffectEvidenceBatch)
        );
    }

    #[test]
    fn independent_claim_profiles_can_project_different_supported_scopes() {
        let payer_only = profile(vec![required(
            EffectRoleV1::PayerDebit,
            "did:mycelix:alice",
        )]);
        let both_legs = profile(vec![
            required(EffectRoleV1::PayerDebit, "did:mycelix:alice"),
            required(EffectRoleV1::BeneficiaryClaim, "did:mycelix:bob"),
        ]);
        let payer = batch(
            &payer_only,
            EffectRoleV1::PayerDebit,
            "did:mycelix:alice",
            DigestV1([1; 32]),
            "receipt:payer:1",
        );
        let first = classify_effect_vector(&payer_only, std::slice::from_ref(&payer)).unwrap();
        let beneficiary_not_observed = empty_batch(
            &both_legs,
            EffectRoleV1::BeneficiaryClaim,
            "did:mycelix:bob",
            EvidenceCompletenessV1::CompleteForDeclaredAuthority,
        );
        let second =
            classify_effect_vector(&both_legs, &[payer, beneficiary_not_observed]).unwrap();
        assert!(matches!(
            first.disposition,
            CompositeClaimDispositionV1::SupportedWithinDeclaredScope { .. }
        ));
        assert!(matches!(
            second.disposition,
            CompositeClaimDispositionV1::Pending { .. }
        ));
    }

    #[test]
    fn input_order_does_not_change_canonical_projection() {
        let p = profile(vec![
            required(EffectRoleV1::PayerDebit, "did:mycelix:alice"),
            required(EffectRoleV1::BeneficiaryClaim, "did:mycelix:bob"),
        ]);
        let payer = batch(
            &p,
            EffectRoleV1::PayerDebit,
            "did:mycelix:alice",
            DigestV1([1; 32]),
            "receipt:payer:1",
        );
        let beneficiary = batch(
            &p,
            EffectRoleV1::BeneficiaryClaim,
            "did:mycelix:bob",
            DigestV1([2; 32]),
            "receipt:beneficiary:1",
        );
        assert_eq!(
            classify_effect_vector(&p, &[payer.clone(), beneficiary.clone()]).unwrap(),
            classify_effect_vector(&p, &[beneficiary, payer]).unwrap()
        );
    }

    #[test]
    fn empty_profile_is_rejected() {
        let mut p = profile(vec![required(
            EffectRoleV1::PayerDebit,
            "did:mycelix:alice",
        )]);
        p.required_effects.clear();
        assert_eq!(
            classify_effect_vector(&p, &[]),
            Err(KernelErrorV1::EmptyRequiredEffects)
        );
    }

    #[test]
    fn profile_digest_is_invariant_to_required_effect_order() {
        let payer = required(EffectRoleV1::PayerDebit, "did:mycelix:alice");
        let beneficiary = required(EffectRoleV1::BeneficiaryClaim, "did:mycelix:bob");
        let first = profile(vec![payer.clone(), beneficiary.clone()]);
        let second = profile(vec![beneficiary, payer]);
        assert_eq!(first.profile_digest, second.profile_digest);
    }

    #[test]
    fn changed_profile_scope_is_rejected_when_digest_is_not_recomputed() {
        let mut p = profile(vec![required(
            EffectRoleV1::PayerDebit,
            "did:mycelix:alice",
        )]);
        p.claim_name = "external_bank_settlement_final".into();
        assert_eq!(
            classify_effect_vector(&p, &[]),
            Err(KernelErrorV1::ProfileDigestMismatch)
        );
    }

    #[test]
    fn profile_digest_changes_when_required_effect_set_changes() {
        let payer_only = profile(vec![required(
            EffectRoleV1::PayerDebit,
            "did:mycelix:alice",
        )]);
        let payer_and_beneficiary = profile(vec![
            required(EffectRoleV1::PayerDebit, "did:mycelix:alice"),
            required(EffectRoleV1::BeneficiaryClaim, "did:mycelix:bob"),
        ]);
        assert_ne!(
            payer_only.profile_digest,
            payer_and_beneficiary.profile_digest
        );
    }

    #[test]
    fn profile_identifiers_are_validated() {
        let mut p = profile(vec![required(
            EffectRoleV1::PayerDebit,
            "did:mycelix:alice",
        )]);
        p.profile_id.clear();
        assert_eq!(
            classify_effect_vector(&p, &[]),
            Err(KernelErrorV1::EmptyComponent)
        );
    }
}
