// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Dependency-light biomass planning invariants for Mycelix regenerative systems.
//!
//! This crate deliberately stops before physical process execution. It models exact
//! biomass accounting, evidence bindings, fail-closed planning assessments, and
//! reservation arithmetic. It does not establish ecological truth, legal rights,
//! contamination safety, process execution, output quality, or physical authority.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

use mycelix_core_types::{EnvironmentalObservation, EvidenceClass};
use mycelix_regenerative_admission::{
    EvidenceAdmissionError, EvidenceCandidate, EvidenceExpectation, admit_pef_evidence,
};
use mycelix_regenerative_core::{BiomassLotId, RegenerativeSiteId};
use std::{collections::BTreeSet, fmt};

/// Maximum UTF-8 byte length of a bounded external or policy reference.
pub const MAX_REFERENCE_BYTES: usize = 256;
/// Maximum UTF-8 byte length of a stable reason code.
pub const MAX_REASON_CODE_BYTES: usize = 128;

/// Exact non-negative biomass mass in milligrams.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct MassMg(u64);

impl MassMg {
    /// Construct an exact mass value.
    pub const fn new(value: u64) -> Self {
        Self(value)
    }

    /// Return the exact milligram value.
    pub const fn get(self) -> u64 {
        self.0
    }

    /// Checked exact addition.
    pub fn checked_add(self, other: Self) -> Result<Self, BiomassError> {
        self.0
            .checked_add(other.0)
            .map(Self)
            .ok_or(BiomassError::MassOverflow)
    }

    /// Checked exact subtraction.
    pub fn checked_sub(self, other: Self) -> Result<Self, BiomassError> {
        self.0
            .checked_sub(other.0)
            .map(Self)
            .ok_or(BiomassError::MassUnderflow)
    }
}

/// Accounting basis for an exact biomass mass.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum MassBasis {
    /// Physical mass as received, including its then-current water content.
    AsReceived,
    /// Dry-matter-equivalent mass established by a separately identified derivation.
    DryMatterEquivalent,
}

/// Exact biomass quantity with an explicit accounting basis.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct BiomassQuantity {
    mass: MassMg,
    basis: MassBasis,
}

impl BiomassQuantity {
    /// Construct an exact basis-labelled quantity.
    pub const fn new(mass: MassMg, basis: MassBasis) -> Self {
        Self { mass, basis }
    }

    /// Exact mass.
    pub const fn mass(self) -> MassMg {
        self.mass
    }

    /// Accounting basis.
    pub const fn basis(self) -> MassBasis {
        self.basis
    }

    /// Checked addition that refuses mixed accounting bases.
    pub fn checked_add(self, other: Self) -> Result<Self, BiomassError> {
        require_same_basis(self.basis, other.basis)?;
        Ok(Self::new(self.mass.checked_add(other.mass)?, self.basis))
    }

    /// Checked subtraction that refuses mixed accounting bases.
    pub fn checked_sub(self, other: Self) -> Result<Self, BiomassError> {
        require_same_basis(self.basis, other.basis)?;
        Ok(Self::new(self.mass.checked_sub(other.mass)?, self.basis))
    }
}

/// Bounded exact reference to an external record, profile, assessment, or snapshot.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ExactRef(String);

impl ExactRef {
    /// Construct and validate a bounded exact reference.
    pub fn new(value: impl Into<String>) -> Result<Self, BiomassError> {
        let value = value.into();
        require_reference_text("reference", &value)?;
        Ok(Self(value))
    }

    /// Borrow the exact reference bytes.
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

/// Stable bounded machine-readable reason code.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ReasonCode(String);

impl ReasonCode {
    /// Construct a lowercase ASCII reason code.
    pub fn new(value: impl Into<String>) -> Result<Self, BiomassError> {
        let value = value.into();
        if value.is_empty() {
            return Err(BiomassError::EmptyReasonCode);
        }
        if value.len() > MAX_REASON_CODE_BYTES {
            return Err(BiomassError::ReasonCodeTooLong {
                actual: value.len(),
                max: MAX_REASON_CODE_BYTES,
            });
        }
        if !value.bytes().all(|byte| {
            byte.is_ascii_lowercase() || byte.is_ascii_digit() || matches!(byte, b'-' | b'_' | b'.')
        }) {
            return Err(BiomassError::InvalidReasonCode(value));
        }
        Ok(Self(value))
    }

    /// Borrow the exact reason code.
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

/// Biomass-specific use role for one admitted PEF observation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum BiomassEvidenceRole {
    /// Evidence that biomass occurs or was observed at a source context.
    ResourceOccurrence,
    /// Evidence supporting an as-received mass assertion.
    AsReceivedMass,
    /// Evidence supporting a dry-matter-equivalent mass assertion.
    DryMatterMass,
    /// Moisture/water-content evidence used by a separately identified derivation.
    Moisture,
    /// Composition evidence.
    Composition,
    /// Contamination or source-hazard evidence.
    Contamination,
}

/// Exact PEF expectation bound to one biomass evidence role.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BiomassEvidenceBinding {
    role: BiomassEvidenceRole,
    observation_id: String,
    phenomenon: String,
    evidence_class: Option<EvidenceClass>,
}

impl BiomassEvidenceBinding {
    /// Construct a biomass evidence binding while reusing the shared expectation validator.
    pub fn new(
        role: BiomassEvidenceRole,
        observation_id: impl Into<String>,
        phenomenon: impl Into<String>,
        evidence_class: Option<EvidenceClass>,
    ) -> Result<Self, BiomassError> {
        let observation_id = observation_id.into();
        let phenomenon = phenomenon.into();
        EvidenceExpectation::new(&observation_id, &phenomenon, evidence_class)
            .map_err(BiomassError::EvidenceAdmission)?;
        Ok(Self {
            role,
            observation_id,
            phenomenon,
            evidence_class,
        })
    }

    /// Domain role.
    pub const fn role(&self) -> BiomassEvidenceRole {
        self.role
    }

    /// Exact PEF observation identifier.
    pub fn observation_id(&self) -> &str {
        &self.observation_id
    }

    /// Exact expected PEF phenomenon.
    pub fn phenomenon(&self) -> &str {
        &self.phenomenon
    }

    /// Optional exact expected evidence class.
    pub const fn evidence_class(&self) -> Option<EvidenceClass> {
        self.evidence_class
    }

    /// Run the qualified shared PEF-admission theorem for this exact binding.
    pub fn admit<'a>(
        &self,
        candidate: EvidenceCandidate<'a>,
    ) -> Result<&'a EnvironmentalObservation, BiomassError> {
        let expectation =
            EvidenceExpectation::new(&self.observation_id, &self.phenomenon, self.evidence_class)
                .map_err(BiomassError::EvidenceAdmission)?;
        admit_pef_evidence(&expectation, candidate).map_err(BiomassError::EvidenceAdmission)
    }

    fn exact_duplicate_of(&self, other: &Self) -> bool {
        self.role == other.role
            && self.observation_id == other.observation_id
            && self.phenomenon == other.phenomenon
            && self.evidence_class == other.evidence_class
    }
}

/// Exact accounting mass asserted by an identified derivation and evidence set.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BiomassMassAssertion {
    quantity: BiomassQuantity,
    derivation_ref: ExactRef,
    evidence: Vec<BiomassEvidenceBinding>,
}

impl BiomassMassAssertion {
    /// Construct a structurally bound exact mass assertion.
    ///
    /// This does not claim that an arbitrary PEF floating measurement was losslessly
    /// converted into integer milligrams. `derivation_ref` identifies the separate
    /// normalization/accounting proposition that produced the exact ledger value.
    pub fn new(
        quantity: BiomassQuantity,
        derivation_ref: ExactRef,
        evidence: Vec<BiomassEvidenceBinding>,
    ) -> Result<Self, BiomassError> {
        if evidence.is_empty() {
            return Err(BiomassError::MissingMassEvidence);
        }
        for (index, binding) in evidence.iter().enumerate() {
            if evidence[..index]
                .iter()
                .any(|existing| binding.exact_duplicate_of(existing))
            {
                return Err(BiomassError::DuplicateEvidenceBinding);
            }
        }
        let required = match quantity.basis {
            MassBasis::AsReceived => BiomassEvidenceRole::AsReceivedMass,
            MassBasis::DryMatterEquivalent => BiomassEvidenceRole::DryMatterMass,
        };
        if !evidence.iter().any(|binding| binding.role == required) {
            return Err(BiomassError::MissingBasisMassEvidence(quantity.basis));
        }
        Ok(Self {
            quantity,
            derivation_ref,
            evidence,
        })
    }

    /// Exact asserted quantity.
    pub const fn quantity(&self) -> BiomassQuantity {
        self.quantity
    }

    /// Exact derivation/accounting reference.
    pub fn derivation_ref(&self) -> &ExactRef {
        &self.derivation_ref
    }

    /// Evidence bindings supporting the assertion.
    pub fn evidence(&self) -> &[BiomassEvidenceBinding] {
        &self.evidence
    }
}

/// Stable semantic profile of one biomass lot.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BiomassLotProfile {
    lot_id: BiomassLotId,
    source_site_id: RegenerativeSiteId,
    formation_ref: ExactRef,
}

impl BiomassLotProfile {
    /// Construct immutable lot identity metadata.
    pub const fn new(
        lot_id: BiomassLotId,
        source_site_id: RegenerativeSiteId,
        formation_ref: ExactRef,
    ) -> Self {
        Self {
            lot_id,
            source_site_id,
            formation_ref,
        }
    }

    /// Canonical lot identity.
    pub fn lot_id(&self) -> &BiomassLotId {
        &self.lot_id
    }

    /// Source-site semantic identity.
    pub fn source_site_id(&self) -> &RegenerativeSiteId {
        &self.source_site_id
    }

    /// Exact lot-formation reference.
    pub fn formation_ref(&self) -> &ExactRef {
        &self.formation_ref
    }
}

/// Immutable state snapshot for one biomass lot.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BiomassStateSnapshot {
    lot_id: BiomassLotId,
    snapshot_ref: ExactRef,
    mass: BiomassMassAssertion,
}

impl BiomassStateSnapshot {
    /// Construct an immutable lot-state snapshot.
    pub const fn new(
        lot_id: BiomassLotId,
        snapshot_ref: ExactRef,
        mass: BiomassMassAssertion,
    ) -> Self {
        Self {
            lot_id,
            snapshot_ref,
            mass,
        }
    }

    /// Canonical lot identity.
    pub fn lot_id(&self) -> &BiomassLotId {
        &self.lot_id
    }

    /// Exact snapshot reference.
    pub fn snapshot_ref(&self) -> &ExactRef {
        &self.snapshot_ref
    }

    /// Exact mass assertion bound to this snapshot.
    pub fn mass(&self) -> &BiomassMassAssertion {
        &self.mass
    }
}

/// Exact material mass partition before ecological-function eligibility is considered.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BiomassMaterialPartition {
    accessible: BiomassQuantity,
    retained: BiomassQuantity,
    allocable: BiomassQuantity,
    unresolved_residual: BiomassQuantity,
}

impl BiomassMaterialPartition {
    /// Construct a resolved material partition with exact same-basis arithmetic.
    pub fn new(
        accessible: BiomassQuantity,
        retained: BiomassQuantity,
        allocable: BiomassQuantity,
        unresolved_residual: BiomassQuantity,
    ) -> Result<Self, BiomassError> {
        require_same_basis(accessible.basis, retained.basis)?;
        require_same_basis(accessible.basis, allocable.basis)?;
        require_same_basis(accessible.basis, unresolved_residual.basis)?;
        let accounted = retained
            .checked_add(allocable)?
            .checked_add(unresolved_residual)?;
        if accounted != accessible {
            return Err(BiomassError::MaterialPartitionDoesNotClose {
                accessible,
                accounted,
            });
        }
        Ok(Self {
            accessible,
            retained,
            allocable,
            unresolved_residual,
        })
    }

    /// Exact physically accessible material quantity represented by this partition.
    pub const fn accessible(self) -> BiomassQuantity {
        self.accessible
    }

    /// Exact quantity retained from material allocation.
    pub const fn retained(self) -> BiomassQuantity {
        self.retained
    }

    /// Exact quantity materially allocable before ecological-function eligibility.
    pub const fn allocable(self) -> BiomassQuantity {
        self.allocable
    }

    /// Explicit unresolved residual quantity.
    pub const fn unresolved_residual(self) -> BiomassQuantity {
        self.unresolved_residual
    }
}

/// Material-allocation state. The unresolved variant carries no positive allocable claim.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum MaterialAllocationOutcome {
    /// Exact resolved material partition.
    Resolved(BiomassMaterialPartition),
    /// Material allocation remains unresolved.
    Unresolved {
        /// Non-empty stable reason codes.
        reasons: Vec<ReasonCode>,
    },
}

impl MaterialAllocationOutcome {
    /// Construct an unresolved material state with explicit reasons.
    pub fn unresolved(reasons: Vec<ReasonCode>) -> Result<Self, BiomassError> {
        validate_reason_set(&reasons)?;
        Ok(Self::Unresolved { reasons })
    }
}

/// Positive ecological-function eligibility scoped to one material partition and evidence snapshot.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EcologicalEligibility {
    material_partition_ref: ExactRef,
    ecological_profile_ref: ExactRef,
    evidence_snapshot_ref: ExactRef,
    allocable_cap: BiomassQuantity,
}

impl EcologicalEligibility {
    /// Construct a positive ecology eligibility statement.
    pub const fn new(
        material_partition_ref: ExactRef,
        ecological_profile_ref: ExactRef,
        evidence_snapshot_ref: ExactRef,
        allocable_cap: BiomassQuantity,
    ) -> Self {
        Self {
            material_partition_ref,
            ecological_profile_ref,
            evidence_snapshot_ref,
            allocable_cap,
        }
    }

    /// Exact material-partition reference evaluated by the ecology profile.
    pub fn material_partition_ref(&self) -> &ExactRef {
        &self.material_partition_ref
    }

    /// Exact ecology profile reference.
    pub fn ecological_profile_ref(&self) -> &ExactRef {
        &self.ecological_profile_ref
    }

    /// Exact evidence snapshot used for ecological eligibility.
    pub fn evidence_snapshot_ref(&self) -> &ExactRef {
        &self.evidence_snapshot_ref
    }

    /// Maximum quantity that the ecology proposition permits to proceed.
    pub const fn allocable_cap(&self) -> BiomassQuantity {
        self.allocable_cap
    }
}

/// Ecological-function constraint outcome, separate from material mass accounting.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum EcologicalConstraintOutcome {
    /// All required hard ecological obligations were satisfied for the exact scope.
    Eligible(EcologicalEligibility),
    /// At least one hard ecological obligation was violated.
    Ineligible {
        /// Non-empty stable reason codes.
        reasons: Vec<ReasonCode>,
    },
    /// At least one required ecological obligation remains unresolved.
    Unresolved {
        /// Non-empty stable reason codes.
        reasons: Vec<ReasonCode>,
    },
}

impl EcologicalConstraintOutcome {
    /// Construct an ineligible ecology state.
    pub fn ineligible(reasons: Vec<ReasonCode>) -> Result<Self, BiomassError> {
        validate_reason_set(&reasons)?;
        Ok(Self::Ineligible { reasons })
    }

    /// Construct an unresolved ecology state.
    pub fn unresolved(reasons: Vec<ReasonCode>) -> Result<Self, BiomassError> {
        validate_reason_set(&reasons)?;
        Ok(Self::Unresolved { reasons })
    }
}

/// Plural rights and custody references required for a positive feedstock assessment.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RightsAndCustodyRefs {
    custody_refs: Vec<ExactRef>,
    removal_refs: Vec<ExactRef>,
    processing_refs: Vec<ExactRef>,
    access_refs: Vec<ExactRef>,
    transfer_refs: Vec<ExactRef>,
}

impl RightsAndCustodyRefs {
    /// Construct a resolved rights/custody reference set.
    ///
    /// Ownership is intentionally not required as a universal primitive; customary,
    /// communal, delegated, leasehold, stewardship, or other legitimate arrangements
    /// may establish the relevant permissions through authoritative external systems.
    pub fn new(
        custody_refs: Vec<ExactRef>,
        removal_refs: Vec<ExactRef>,
        processing_refs: Vec<ExactRef>,
        access_refs: Vec<ExactRef>,
        transfer_refs: Vec<ExactRef>,
    ) -> Result<Self, BiomassError> {
        require_nonempty_refs("custody_refs", &custody_refs)?;
        require_nonempty_refs("removal_refs", &removal_refs)?;
        require_nonempty_refs("processing_refs", &processing_refs)?;
        validate_unique_refs("custody_refs", &custody_refs)?;
        validate_unique_refs("removal_refs", &removal_refs)?;
        validate_unique_refs("processing_refs", &processing_refs)?;
        validate_unique_refs("access_refs", &access_refs)?;
        validate_unique_refs("transfer_refs", &transfer_refs)?;
        Ok(Self {
            custody_refs,
            removal_refs,
            processing_refs,
            access_refs,
            transfer_refs,
        })
    }

    /// Custody references.
    pub fn custody_refs(&self) -> &[ExactRef] {
        &self.custody_refs
    }

    /// Removal/harvest-right references.
    pub fn removal_refs(&self) -> &[ExactRef] {
        &self.removal_refs
    }

    /// Processing-authority references.
    pub fn processing_refs(&self) -> &[ExactRef] {
        &self.processing_refs
    }

    /// Optional access references.
    pub fn access_refs(&self) -> &[ExactRef] {
        &self.access_refs
    }

    /// Optional transfer references.
    pub fn transfer_refs(&self) -> &[ExactRef] {
        &self.transfer_refs
    }
}

/// Positive process-scoped feedstock assessment.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EligibleFeedstock {
    assessment_ref: ExactRef,
    lot_id: BiomassLotId,
    state_snapshot_ref: ExactRef,
    process_profile_ref: ExactRef,
    evidence_snapshot_ref: ExactRef,
    material_partition_ref: ExactRef,
    material_partition: BiomassMaterialPartition,
    ecology: EcologicalEligibility,
    rights: RightsAndCustodyRefs,
    assessed: BiomassQuantity,
    prerequisite_refs: Vec<ExactRef>,
}

impl EligibleFeedstock {
    /// Construct positive eligibility only when material, ecology, rights, and scope align.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        assessment_ref: ExactRef,
        lot_id: BiomassLotId,
        state_snapshot_ref: ExactRef,
        process_profile_ref: ExactRef,
        evidence_snapshot_ref: ExactRef,
        material_partition_ref: ExactRef,
        material_partition: BiomassMaterialPartition,
        ecology: EcologicalConstraintOutcome,
        rights: RightsAndCustodyRefs,
        assessed: BiomassQuantity,
        prerequisite_refs: Vec<ExactRef>,
    ) -> Result<Self, BiomassError> {
        if assessed.mass.get() == 0 {
            return Err(BiomassError::ZeroEligibleMass);
        }
        require_same_basis(assessed.basis, material_partition.allocable.basis)?;
        if assessed.mass > material_partition.allocable.mass {
            return Err(BiomassError::EligibleMassExceedsMaterialAllocation);
        }
        let EcologicalConstraintOutcome::Eligible(ecology) = ecology else {
            return Err(BiomassError::EcologyNotEligible);
        };
        if ecology.material_partition_ref != material_partition_ref {
            return Err(BiomassError::EcologyPartitionReferenceMismatch);
        }
        if ecology.evidence_snapshot_ref != evidence_snapshot_ref {
            return Err(BiomassError::EcologyEvidenceSnapshotMismatch);
        }
        require_same_basis(assessed.basis, ecology.allocable_cap.basis)?;
        if ecology.allocable_cap.mass > material_partition.allocable.mass {
            return Err(BiomassError::EcologyCapExceedsMaterialAllocation);
        }
        if assessed.mass > ecology.allocable_cap.mass {
            return Err(BiomassError::EligibleMassExceedsEcologyCap);
        }
        validate_unique_refs("prerequisite_refs", &prerequisite_refs)?;
        Ok(Self {
            assessment_ref,
            lot_id,
            state_snapshot_ref,
            process_profile_ref,
            evidence_snapshot_ref,
            material_partition_ref,
            material_partition,
            ecology,
            rights,
            assessed,
            prerequisite_refs,
        })
    }

    /// Exact assessment reference.
    pub fn assessment_ref(&self) -> &ExactRef {
        &self.assessment_ref
    }

    /// Canonical biomass lot.
    pub fn lot_id(&self) -> &BiomassLotId {
        &self.lot_id
    }

    /// Exact lot-state snapshot.
    pub fn state_snapshot_ref(&self) -> &ExactRef {
        &self.state_snapshot_ref
    }

    /// Exact target process profile.
    pub fn process_profile_ref(&self) -> &ExactRef {
        &self.process_profile_ref
    }

    /// Exact assessment evidence snapshot.
    pub fn evidence_snapshot_ref(&self) -> &ExactRef {
        &self.evidence_snapshot_ref
    }

    /// Exact material-partition reference.
    pub fn material_partition_ref(&self) -> &ExactRef {
        &self.material_partition_ref
    }

    /// Resolved material partition.
    pub const fn material_partition(&self) -> BiomassMaterialPartition {
        self.material_partition
    }

    /// Positive ecological eligibility.
    pub fn ecology(&self) -> &EcologicalEligibility {
        &self.ecology
    }

    /// Rights/custody references used by the assessment.
    pub fn rights(&self) -> &RightsAndCustodyRefs {
        &self.rights
    }

    /// Exact process-scoped eligible quantity.
    pub const fn assessed(&self) -> BiomassQuantity {
        self.assessed
    }

    /// Additional exact prerequisite references.
    pub fn prerequisite_refs(&self) -> &[ExactRef] {
        &self.prerequisite_refs
    }
}

/// Negative or unresolved assessment payload.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FeedstockFailure {
    assessment_ref: ExactRef,
    lot_id: BiomassLotId,
    state_snapshot_ref: ExactRef,
    process_profile_ref: ExactRef,
    reasons: Vec<ReasonCode>,
}

impl FeedstockFailure {
    /// Construct a fail-closed assessment payload.
    pub fn new(
        assessment_ref: ExactRef,
        lot_id: BiomassLotId,
        state_snapshot_ref: ExactRef,
        process_profile_ref: ExactRef,
        reasons: Vec<ReasonCode>,
    ) -> Result<Self, BiomassError> {
        validate_reason_set(&reasons)?;
        Ok(Self {
            assessment_ref,
            lot_id,
            state_snapshot_ref,
            process_profile_ref,
            reasons,
        })
    }

    /// Stable reasons.
    pub fn reasons(&self) -> &[ReasonCode] {
        &self.reasons
    }
}

/// Process-scoped feedstock assessment state.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FeedstockAssessment {
    /// Positive eligibility for one exact bounded scope.
    Eligible(Box<EligibleFeedstock>),
    /// Hard ineligibility.
    Ineligible(FeedstockFailure),
    /// Required facts remain unresolved.
    Unresolved(FeedstockFailure),
}

impl FeedstockAssessment {
    /// Derive a software-only reservation capacity witness from positive eligibility.
    pub fn reservation_capacity(&self) -> Result<ReservationCapacity, BiomassError> {
        match self {
            Self::Eligible(eligible) => Ok(ReservationCapacity::from_eligible(eligible)),
            Self::Ineligible(_) | Self::Unresolved(_) => Err(BiomassError::FeedstockNotEligible),
        }
    }
}

/// Software-only capacity witness derived from one exact eligible assessment.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReservationCapacity {
    lot_id: BiomassLotId,
    assessment_ref: ExactRef,
    state_snapshot_ref: ExactRef,
    process_profile_ref: ExactRef,
    maximum: BiomassQuantity,
}

impl ReservationCapacity {
    fn from_eligible(eligible: &EligibleFeedstock) -> Self {
        Self {
            lot_id: eligible.lot_id.clone(),
            assessment_ref: eligible.assessment_ref.clone(),
            state_snapshot_ref: eligible.state_snapshot_ref.clone(),
            process_profile_ref: eligible.process_profile_ref.clone(),
            maximum: eligible.assessed,
        }
    }

    /// Exact lot scope.
    pub fn lot_id(&self) -> &BiomassLotId {
        &self.lot_id
    }

    /// Exact assessment scope.
    pub fn assessment_ref(&self) -> &ExactRef {
        &self.assessment_ref
    }

    /// Exact state snapshot scope.
    pub fn state_snapshot_ref(&self) -> &ExactRef {
        &self.state_snapshot_ref
    }

    /// Exact process-profile scope.
    pub fn process_profile_ref(&self) -> &ExactRef {
        &self.process_profile_ref
    }

    /// Maximum reservable quantity.
    pub const fn maximum(&self) -> BiomassQuantity {
        self.maximum
    }
}

/// Unvalidated request to reserve process-input capacity.
///
/// A request is not an accepted reservation. Exact scope, basis, duplicate-reference,
/// and capacity checks are performed by [`evaluate_reservation_requests`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReservationRequest {
    reservation_ref: ExactRef,
    lot_id: BiomassLotId,
    feedstock_assessment_ref: ExactRef,
    state_snapshot_ref: ExactRef,
    process_profile_ref: ExactRef,
    quantity: BiomassQuantity,
}

impl ReservationRequest {
    /// Construct an unvalidated positive reservation request.
    pub fn new(
        reservation_ref: ExactRef,
        lot_id: BiomassLotId,
        feedstock_assessment_ref: ExactRef,
        state_snapshot_ref: ExactRef,
        process_profile_ref: ExactRef,
        quantity: BiomassQuantity,
    ) -> Result<Self, BiomassError> {
        if quantity.mass.get() == 0 {
            return Err(BiomassError::ZeroReservation);
        }
        Ok(Self {
            reservation_ref,
            lot_id,
            feedstock_assessment_ref,
            state_snapshot_ref,
            process_profile_ref,
            quantity,
        })
    }

    /// Exact request reference.
    pub fn reservation_ref(&self) -> &ExactRef {
        &self.reservation_ref
    }

    /// Requested quantity.
    pub const fn quantity(&self) -> BiomassQuantity {
        self.quantity
    }
}

/// Evaluator-minted accepted reservation.
///
/// This type has no public constructor. It proves only that one request survived the
/// exact software reservation checks for the evaluated batch. It is not consumption,
/// process execution, or physical authority.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AcceptedReservation {
    reservation_ref: ExactRef,
    lot_id: BiomassLotId,
    feedstock_assessment_ref: ExactRef,
    state_snapshot_ref: ExactRef,
    process_profile_ref: ExactRef,
    quantity: BiomassQuantity,
}

impl AcceptedReservation {
    fn from_request(request: &ReservationRequest) -> Self {
        Self {
            reservation_ref: request.reservation_ref.clone(),
            lot_id: request.lot_id.clone(),
            feedstock_assessment_ref: request.feedstock_assessment_ref.clone(),
            state_snapshot_ref: request.state_snapshot_ref.clone(),
            process_profile_ref: request.process_profile_ref.clone(),
            quantity: request.quantity,
        }
    }

    /// Exact accepted reservation reference.
    pub fn reservation_ref(&self) -> &ExactRef {
        &self.reservation_ref
    }

    /// Exact biomass lot scope.
    pub fn lot_id(&self) -> &BiomassLotId {
        &self.lot_id
    }

    /// Exact eligible feedstock assessment scope.
    pub fn feedstock_assessment_ref(&self) -> &ExactRef {
        &self.feedstock_assessment_ref
    }

    /// Exact biomass state snapshot scope.
    pub fn state_snapshot_ref(&self) -> &ExactRef {
        &self.state_snapshot_ref
    }

    /// Exact target process profile scope.
    pub fn process_profile_ref(&self) -> &ExactRef {
        &self.process_profile_ref
    }

    /// Accepted reservation quantity.
    pub const fn quantity(&self) -> BiomassQuantity {
        self.quantity
    }
}

/// Atomic result of validating one reservation-request batch against one capacity witness.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReservationAcceptance {
    accepted: Vec<AcceptedReservation>,
    reserved: BiomassQuantity,
    remaining: BiomassQuantity,
}

impl ReservationAcceptance {
    /// Evaluator-minted accepted reservations. Empty is valid for an empty request batch.
    pub fn accepted(&self) -> &[AcceptedReservation] {
        &self.accepted
    }

    /// Total accepted quantity.
    pub const fn reserved(&self) -> BiomassQuantity {
        self.reserved
    }

    /// Remaining unreserved capacity.
    pub const fn remaining(&self) -> BiomassQuantity {
        self.remaining
    }
}

/// Validate a complete request batch and atomically mint accepted reservations.
///
/// The function validates the entire batch before constructing any
/// [`AcceptedReservation`] visible to the caller. A failed batch therefore returns no
/// partially accepted reservation state.
pub fn evaluate_reservation_requests(
    capacity: &ReservationCapacity,
    requests: &[ReservationRequest],
) -> Result<ReservationAcceptance, BiomassError> {
    let mut seen = BTreeSet::new();
    let mut reserved = BiomassQuantity::new(MassMg::new(0), capacity.maximum.basis);
    for request in requests {
        if !seen.insert(request.reservation_ref.as_str()) {
            return Err(BiomassError::DuplicateReservationReference(
                request.reservation_ref.as_str().to_owned(),
            ));
        }
        if request.lot_id != capacity.lot_id {
            return Err(BiomassError::ReservationScopeMismatch("lot_id"));
        }
        if request.feedstock_assessment_ref != capacity.assessment_ref {
            return Err(BiomassError::ReservationScopeMismatch("assessment_ref"));
        }
        if request.state_snapshot_ref != capacity.state_snapshot_ref {
            return Err(BiomassError::ReservationScopeMismatch("state_snapshot_ref"));
        }
        if request.process_profile_ref != capacity.process_profile_ref {
            return Err(BiomassError::ReservationScopeMismatch(
                "process_profile_ref",
            ));
        }
        require_same_basis(capacity.maximum.basis, request.quantity.basis)?;
        reserved = reserved.checked_add(request.quantity)?;
        if reserved.mass > capacity.maximum.mass {
            return Err(BiomassError::ReservationOverbooked);
        }
    }
    let remaining = capacity.maximum.checked_sub(reserved)?;
    let accepted = requests
        .iter()
        .map(AcceptedReservation::from_request)
        .collect();
    Ok(ReservationAcceptance {
        accepted,
        reserved,
        remaining,
    })
}

/// Biomass core validation failures.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum BiomassError {
    /// Exact mass addition overflowed `u64`.
    MassOverflow,
    /// Exact mass subtraction would become negative.
    MassUnderflow,
    /// Two quantities used incompatible accounting bases.
    MassBasisMismatch {
        /// Expected basis.
        expected: MassBasis,
        /// Actual basis.
        actual: MassBasis,
    },
    /// A bounded reference was empty.
    EmptyReference(&'static str),
    /// A bounded reference exceeded the v1 limit.
    ReferenceTooLong {
        /// Field name.
        field: &'static str,
        /// Actual UTF-8 bytes.
        actual: usize,
        /// Maximum UTF-8 bytes.
        max: usize,
    },
    /// A reason code was empty.
    EmptyReasonCode,
    /// A reason code exceeded the v1 limit.
    ReasonCodeTooLong {
        /// Actual UTF-8 bytes.
        actual: usize,
        /// Maximum UTF-8 bytes.
        max: usize,
    },
    /// A reason code was not canonical lowercase ASCII.
    InvalidReasonCode(String),
    /// Shared PEF admission failed.
    EvidenceAdmission(EvidenceAdmissionError),
    /// An exact evidence binding was duplicated.
    DuplicateEvidenceBinding,
    /// An exact accounting assertion had no evidence bindings.
    MissingMassEvidence,
    /// An exact accounting assertion lacked a same-basis mass evidence role.
    MissingBasisMassEvidence(MassBasis),
    /// A resolved material partition did not exactly close.
    MaterialPartitionDoesNotClose {
        /// Accessible quantity.
        accessible: BiomassQuantity,
        /// Sum of retained, allocable, and explicit unresolved residual.
        accounted: BiomassQuantity,
    },
    /// A fail-closed state supplied no reason codes.
    EmptyReasonSet,
    /// A fail-closed state duplicated a reason code.
    DuplicateReasonCode(String),
    /// A required reference set was empty.
    EmptyReferenceSet(&'static str),
    /// A reference set contained a duplicate exact reference.
    DuplicateReference {
        /// Reference-set field name.
        field: &'static str,
        /// Duplicate value.
        value: String,
    },
    /// Positive feedstock eligibility used zero mass.
    ZeroEligibleMass,
    /// Positive feedstock quantity exceeded the material allocable quantity.
    EligibleMassExceedsMaterialAllocation,
    /// Ecology state was not positive eligibility.
    EcologyNotEligible,
    /// Ecology eligibility referenced a different material partition.
    EcologyPartitionReferenceMismatch,
    /// Ecology eligibility was evaluated against a different evidence snapshot.
    EcologyEvidenceSnapshotMismatch,
    /// Ecology cap exceeded the material allocable quantity.
    EcologyCapExceedsMaterialAllocation,
    /// Positive feedstock quantity exceeded the ecology cap.
    EligibleMassExceedsEcologyCap,
    /// Reservation capacity was requested from a non-eligible assessment.
    FeedstockNotEligible,
    /// A reservation requested zero mass.
    ZeroReservation,
    /// Reservation scope did not match the capacity witness.
    ReservationScopeMismatch(&'static str),
    /// Reservation reference was duplicated.
    DuplicateReservationReference(String),
    /// Reservations exceeded the scoped positive capacity.
    ReservationOverbooked,
}

impl fmt::Display for BiomassError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::MassOverflow => f.write_str("biomass mass arithmetic overflow"),
            Self::MassUnderflow => f.write_str("biomass mass arithmetic underflow"),
            Self::MassBasisMismatch { expected, actual } => {
                write!(
                    f,
                    "mass basis mismatch: expected {expected:?}, got {actual:?}"
                )
            }
            Self::EmptyReference(field) => write!(f, "{field} cannot be empty"),
            Self::ReferenceTooLong { field, actual, max } => {
                write!(f, "{field} is {actual} bytes; maximum is {max}")
            }
            Self::EmptyReasonCode => f.write_str("reason code cannot be empty"),
            Self::ReasonCodeTooLong { actual, max } => {
                write!(f, "reason code is {actual} bytes; maximum is {max}")
            }
            Self::InvalidReasonCode(code) => write!(f, "invalid reason code {code:?}"),
            Self::EvidenceAdmission(error) => {
                write!(f, "shared evidence admission failed: {error}")
            }
            Self::DuplicateEvidenceBinding => f.write_str("duplicate exact evidence binding"),
            Self::MissingMassEvidence => f.write_str("mass assertion requires evidence bindings"),
            Self::MissingBasisMassEvidence(basis) => {
                write!(
                    f,
                    "mass assertion lacks required evidence for basis {basis:?}"
                )
            }
            Self::MaterialPartitionDoesNotClose { .. } => {
                f.write_str("material partition does not exactly close")
            }
            Self::EmptyReasonSet => f.write_str("fail-closed state requires at least one reason"),
            Self::DuplicateReasonCode(code) => write!(f, "duplicate reason code {code}"),
            Self::EmptyReferenceSet(field) => write!(f, "{field} cannot be empty"),
            Self::DuplicateReference { field, value } => {
                write!(f, "duplicate reference {value:?} in {field}")
            }
            Self::ZeroEligibleMass => f.write_str("eligible feedstock mass must be positive"),
            Self::EligibleMassExceedsMaterialAllocation => {
                f.write_str("eligible mass exceeds material allocable quantity")
            }
            Self::EcologyNotEligible => f.write_str("ecological constraints are not eligible"),
            Self::EcologyPartitionReferenceMismatch => {
                f.write_str("ecology eligibility references a different material partition")
            }
            Self::EcologyEvidenceSnapshotMismatch => {
                f.write_str("ecology eligibility references a different evidence snapshot")
            }
            Self::EcologyCapExceedsMaterialAllocation => {
                f.write_str("ecology cap exceeds material allocable quantity")
            }
            Self::EligibleMassExceedsEcologyCap => {
                f.write_str("eligible mass exceeds ecological allocable cap")
            }
            Self::FeedstockNotEligible => {
                f.write_str("reservation capacity requires eligible feedstock")
            }
            Self::ZeroReservation => f.write_str("reservation mass must be positive"),
            Self::ReservationScopeMismatch(field) => {
                write!(f, "reservation scope mismatch in {field}")
            }
            Self::DuplicateReservationReference(reference) => {
                write!(f, "duplicate reservation reference {reference}")
            }
            Self::ReservationOverbooked => f.write_str("reservations exceed scoped capacity"),
        }
    }
}

impl std::error::Error for BiomassError {}

fn require_same_basis(expected: MassBasis, actual: MassBasis) -> Result<(), BiomassError> {
    if expected == actual {
        Ok(())
    } else {
        Err(BiomassError::MassBasisMismatch { expected, actual })
    }
}

fn require_reference_text(field: &'static str, value: &str) -> Result<(), BiomassError> {
    if value.trim().is_empty() {
        return Err(BiomassError::EmptyReference(field));
    }
    if value.len() > MAX_REFERENCE_BYTES {
        return Err(BiomassError::ReferenceTooLong {
            field,
            actual: value.len(),
            max: MAX_REFERENCE_BYTES,
        });
    }
    Ok(())
}

fn validate_reason_set(reasons: &[ReasonCode]) -> Result<(), BiomassError> {
    if reasons.is_empty() {
        return Err(BiomassError::EmptyReasonSet);
    }
    let mut seen = BTreeSet::new();
    for reason in reasons {
        if !seen.insert(reason.as_str()) {
            return Err(BiomassError::DuplicateReasonCode(
                reason.as_str().to_owned(),
            ));
        }
    }
    Ok(())
}

fn require_nonempty_refs(field: &'static str, refs: &[ExactRef]) -> Result<(), BiomassError> {
    if refs.is_empty() {
        Err(BiomassError::EmptyReferenceSet(field))
    } else {
        Ok(())
    }
}

fn validate_unique_refs(field: &'static str, refs: &[ExactRef]) -> Result<(), BiomassError> {
    let mut seen = BTreeSet::new();
    for reference in refs {
        if !seen.insert(reference.as_str()) {
            return Err(BiomassError::DuplicateReference {
                field,
                value: reference.as_str().to_owned(),
            });
        }
    }
    Ok(())
}
