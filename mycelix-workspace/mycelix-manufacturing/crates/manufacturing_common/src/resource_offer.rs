// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Evidence-bounded distributed manufacturing resource-offer contracts.
//!
//! An offer is a provider-scoped declaration that links an exact resource and
//! site to a canonical process/capability profile. It is not an availability
//! calendar, reservation, execution record, capability qualification or
//! product acceptance.

use crate::{ProcessCapabilityProfileRefV1, ProcessDefinitionRefV1};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

pub const RESOURCE_OFFER_SCHEMA_VERSION_V1: &str = "0.1.0";

/// A pointer to an owner-native record. The referenced payload is not fetched,
/// authenticated or interpreted by the manufacturing-offer validator.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub struct OwnerNativeSubjectRefV1 {
    pub owner_repository: String,
    pub schema_id: String,
    pub schema_version: String,
    pub source_revision: String,
    pub subject_id: String,
    pub payload_sha256: String,
}

impl OwnerNativeSubjectRefV1 {
    pub fn validate(&self) -> Result<(), String> {
        let mut repository_parts = self.owner_repository.split('/');
        let owner = repository_parts.next().unwrap_or_default();
        let repository = repository_parts.next().unwrap_or_default();
        if self.owner_repository.len() > 128
            || owner.is_empty()
            || repository.is_empty()
            || repository_parts.next().is_some()
            || self
                .owner_repository
                .bytes()
                .any(|b| !(b.is_ascii_alphanumeric() || b"-._/".contains(&b)))
        {
            return Err(
                "owner_repository must be a bounded non-empty owner/repository coordinate".into(),
            );
        }
        validate_bounded_token("schema_id", &self.schema_id, 256)?;
        validate_bounded_token("schema_version", &self.schema_version, 64)?;
        validate_bounded_token("source_revision", &self.source_revision, 128)?;
        validate_bounded_token("subject_id", &self.subject_id, 256)?;
        if !is_lower_hex_digest(&self.payload_sha256, 64) {
            return Err("payload_sha256 must be 64 lowercase hexadecimal characters".into());
        }
        Ok(())
    }
}

fn validate_bounded_token(field: &str, value: &str, max_bytes: usize) -> Result<(), String> {
    if value.is_empty()
        || value.len() > max_bytes
        || value.trim() != value
        || value.chars().any(char::is_control)
    {
        return Err(format!(
            "{field} must be a canonical 1-{max_bytes} byte token"
        ));
    }
    Ok(())
}

fn is_lower_hex_digest(value: &str, expected_len: usize) -> bool {
    value.len() == expected_len
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

/// Where the capacity statement came from. This describes provenance only;
/// it is not a ranked quality or qualification state.
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub enum CapacityClaimSourceV1 {
    ProviderDeclared,
    ManufacturerDocumentReferenced,
    ThirdPartyEvidenceReferenced,
    InstrumentObservationReferenced,
    QualificationEvidenceReferenced,
}

/// Lifecycle of the offer statement itself. Expiry is evaluated from the
/// validity interval; it is not inferred from a stale machine status.
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResourceOfferPublicationStateV1 {
    Published,
    Withdrawn,
}

/// Exact process/resource declaration. The schema deliberately references
/// owner-native capability, configuration, prerequisite, capacity and
/// calendar contracts rather than copying their engineering quantities.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct ManufacturingResourceOfferV1 {
    pub schema_version: String,
    /// Stable provider-issued identifier; offer_revision distinguishes immutable versions.
    pub offer_id: String,
    pub offer_revision: String,

    pub provider_subject_ref: OwnerNativeSubjectRefV1,
    /// Evidence that the provider is authorized to speak for this resource.
    /// Presence does not prove authority; the issuer/resource relationship is
    /// resolved by its owning identity/resource authority.
    pub provider_authority_ref: Option<OwnerNativeSubjectRefV1>,
    pub resource_subject_ref: OwnerNativeSubjectRefV1,
    pub site_subject_ref: OwnerNativeSubjectRefV1,

    pub process_definition_ref: ProcessDefinitionRefV1,
    pub capability_profile_ref: ProcessCapabilityProfileRefV1,
    pub configuration_ref: OwnerNativeSubjectRefV1,

    #[serde(default)]
    pub tooling_refs: Vec<OwnerNativeSubjectRefV1>,
    #[serde(default)]
    pub fixture_workholding_refs: Vec<OwnerNativeSubjectRefV1>,
    #[serde(default)]
    pub material_applicability_refs: Vec<OwnerNativeSubjectRefV1>,
    #[serde(default)]
    pub environment_requirement_refs: Vec<OwnerNativeSubjectRefV1>,
    #[serde(default)]
    pub utility_requirement_refs: Vec<OwnerNativeSubjectRefV1>,
    #[serde(default)]
    pub operator_competence_refs: Vec<OwnerNativeSubjectRefV1>,
    #[serde(default)]
    pub safety_profile_refs: Vec<OwnerNativeSubjectRefV1>,
    #[serde(default)]
    pub inspection_profile_refs: Vec<OwnerNativeSubjectRefV1>,
    #[serde(default)]
    pub calibration_profile_refs: Vec<OwnerNativeSubjectRefV1>,

    /// Refers to an owner-native capacity profile. The units/quantity model
    /// remain with the owner schema; this field does not create a scheduler.
    pub declared_capacity_profile_ref: OwnerNativeSubjectRefV1,
    /// Optional evidence source for time-bounded availability. Its presence
    /// does not mean a slot is open, bookable or reserved.
    pub availability_calendar_ref: Option<OwnerNativeSubjectRefV1>,
    /// Commercial terms are kept separate from process feasibility.
    pub commercial_terms_ref: Option<OwnerNativeSubjectRefV1>,

    pub capacity_claim_source: CapacityClaimSourceV1,
    #[serde(default)]
    pub capacity_evidence_refs: Vec<OwnerNativeSubjectRefV1>,
    /// References to external lifecycle events. Their authority/state is not
    /// interpreted by this schema-only contract.
    #[serde(default)]
    pub lifecycle_event_refs: Vec<OwnerNativeSubjectRefV1>,

    pub publication_state: ResourceOfferPublicationStateV1,
    pub issued_at_unix_s: u64,
    pub valid_from_unix_s: u64,
    pub valid_until_unix_s: u64,
    /// Pointer to a preceding offer revision only. Graph-wide cycle and
    /// authority checks require a future owner-resolution adapter.
    pub supersedes_offer_ref: Option<OwnerNativeSubjectRefV1>,
}

impl ManufacturingResourceOfferV1 {
    pub fn validate(&self) -> Result<(), String> {
        if self.schema_version != RESOURCE_OFFER_SCHEMA_VERSION_V1 {
            return Err("unsupported resource-offer schema_version".into());
        }
        validate_bounded_token("offer_id", &self.offer_id, 128)?;
        validate_bounded_token("offer_revision", &self.offer_revision, 64)?;
        if self.valid_from_unix_s >= self.valid_until_unix_s {
            return Err("offer validity window must be ordered".into());
        }
        if self.issued_at_unix_s > self.valid_until_unix_s {
            return Err("offer issue time must not be after valid_until".into());
        }

        for reference in [
            &self.provider_subject_ref,
            &self.resource_subject_ref,
            &self.site_subject_ref,
            &self.configuration_ref,
            &self.declared_capacity_profile_ref,
        ] {
            reference.validate()?;
        }
        if let Some(reference) = &self.provider_authority_ref {
            reference.validate()?;
        }
        if let Some(reference) = &self.availability_calendar_ref {
            reference.validate()?;
        }
        if let Some(reference) = &self.commercial_terms_ref {
            reference.validate()?;
        }
        if let Some(reference) = &self.supersedes_offer_ref {
            reference.validate()?;
            if reference.subject_id == self.offer_id
                && reference.source_revision == self.offer_revision
            {
                return Err("resource offer cannot supersede itself".into());
            }
        }
        validate_unique_refs("lifecycle_event_refs", &self.lifecycle_event_refs)?;

        self.process_definition_ref.validate()?;
        self.capability_profile_ref.validate()?;
        if self.process_definition_ref.0.object_id_blake3
            != self.capability_profile_ref.0.process_id_blake3
        {
            return Err(
                "resource offer capability profile must match the process definition".into(),
            );
        }
        if self.process_definition_ref.0.owner_commit != self.capability_profile_ref.0.owner_commit
        {
            return Err("resource offer mixes Symthaea contract source commits".into());
        }

        if self.capacity_claim_source != CapacityClaimSourceV1::ProviderDeclared
            && self.capacity_evidence_refs.is_empty()
        {
            return Err(
                "non-provider capacity claim sources require at least one evidence reference"
                    .into(),
            );
        }

        for (field, references) in [
            ("tooling_refs", &self.tooling_refs),
            ("fixture_workholding_refs", &self.fixture_workholding_refs),
            (
                "material_applicability_refs",
                &self.material_applicability_refs,
            ),
            (
                "environment_requirement_refs",
                &self.environment_requirement_refs,
            ),
            ("utility_requirement_refs", &self.utility_requirement_refs),
            ("operator_competence_refs", &self.operator_competence_refs),
            ("safety_profile_refs", &self.safety_profile_refs),
            ("inspection_profile_refs", &self.inspection_profile_refs),
            ("calibration_profile_refs", &self.calibration_profile_refs),
            ("capacity_evidence_refs", &self.capacity_evidence_refs),
        ] {
            validate_unique_refs(field, references)?;
        }
        Ok(())
    }

    /// Publication validity only. ActiveOfferWindow is not calendar
    /// availability, bookable capacity, reservation or execution.
    pub fn publication_disposition_at(
        &self,
        evaluation_time_unix_s: u64,
    ) -> Result<ResourceOfferPublicationDispositionV1, String> {
        self.validate()?;
        let disposition = match self.publication_state {
            ResourceOfferPublicationStateV1::Withdrawn => {
                ResourceOfferPublicationDispositionV1::Withdrawn
            }
            ResourceOfferPublicationStateV1::Published
                if evaluation_time_unix_s < self.valid_from_unix_s =>
            {
                ResourceOfferPublicationDispositionV1::NotYetValid
            }
            ResourceOfferPublicationStateV1::Published
                if evaluation_time_unix_s >= self.valid_until_unix_s =>
            {
                ResourceOfferPublicationDispositionV1::Expired
            }
            ResourceOfferPublicationStateV1::Published => {
                ResourceOfferPublicationDispositionV1::ActiveOfferWindow
            }
        };
        Ok(disposition)
    }

    /// Local syntax classification only. A valid pointer is not owner-object
    /// resolution, provider/resource authority or qualified capability.
    pub fn resolution_state(&self) -> ResourceOfferReferenceStateV1 {
        if self.validate().is_ok() {
            ResourceOfferReferenceStateV1::TypedOfferPresent
        } else {
            ResourceOfferReferenceStateV1::TypedOfferMalformed
        }
    }
}

/// Offer publication/validity disposition only. Disputed and Superseded
/// require separately resolved lifecycle events/newer-offer history and are
/// not emitted by this local time-window evaluator.
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResourceOfferPublicationDispositionV1 {
    NotYetValid,
    ActiveOfferWindow,
    Expired,
    Withdrawn,
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResourceOfferReferenceStateV1 {
    TypedOfferPresent,
    TypedOfferMalformed,
}

fn validate_unique_refs(field: &str, references: &[OwnerNativeSubjectRefV1]) -> Result<(), String> {
    if references.len() > 64 {
        return Err(format!("{field} exceeds the 64-reference limit"));
    }
    let mut unique = BTreeSet::new();
    for reference in references {
        reference.validate()?;
        if !unique.insert(reference) {
            return Err(format!(
                "{field} contains a duplicate owner-native reference"
            ));
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        ProcessCapabilityProfileRefV1, ProcessDefinitionRefV1, SYMTHAEA_CONTRACTS_CRATE,
        SYMTHAEA_MANUFACTURING_CONTRACT_COMMIT_V1, SYMTHAEA_MANUFACTURING_CONTRACT_REPOSITORY,
        SYMTHAEA_PROCESS_CRATE, SymthaeaManufacturingObjectRefV1,
    };

    fn owner_ref(schema: &str, subject: &str, digest_char: char) -> OwnerNativeSubjectRefV1 {
        OwnerNativeSubjectRefV1 {
            owner_repository: "example/provider".into(),
            schema_id: schema.into(),
            schema_version: "1.0".into(),
            source_revision: "rev-2026-11-23".into(),
            subject_id: subject.into(),
            payload_sha256: digest_char.to_string().repeat(64),
        }
    }

    fn symthaea_ref(
        crate_name: &str,
        contract_type: &str,
        object_id: char,
    ) -> SymthaeaManufacturingObjectRefV1 {
        SymthaeaManufacturingObjectRefV1 {
            owner_repository: SYMTHAEA_MANUFACTURING_CONTRACT_REPOSITORY.into(),
            owner_commit: SYMTHAEA_MANUFACTURING_CONTRACT_COMMIT_V1.into(),
            contract_crate: crate_name.into(),
            contract_type: contract_type.into(),
            object_id_blake3: object_id.to_string().repeat(64),
            process_id_blake3: "a".repeat(64),
            semantic_version: "1.0.0".into(),
        }
    }

    fn valid_offer() -> ManufacturingResourceOfferV1 {
        let mut process = symthaea_ref(SYMTHAEA_PROCESS_CRATE, "ProcessDefinitionV1", 'a');
        process.object_id_blake3 = process.process_id_blake3.clone();
        ManufacturingResourceOfferV1 {
            schema_version: RESOURCE_OFFER_SCHEMA_VERSION_V1.into(),
            offer_id: "offer:machine-7:polymer-extrusion".into(),
            offer_revision: "A".into(),
            provider_subject_ref: owner_ref("provider.identity/1", "provider-1", '1'),
            provider_authority_ref: None,
            resource_subject_ref: owner_ref("equipment.catalog/1", "machine-7", '2'),
            site_subject_ref: owner_ref("facility.site/1", "site-2", '3'),
            process_definition_ref: ProcessDefinitionRefV1(process),
            capability_profile_ref: ProcessCapabilityProfileRefV1(symthaea_ref(
                SYMTHAEA_CONTRACTS_CRATE,
                "ProcessCapabilityProfileV1",
                'e',
            )),
            configuration_ref: owner_ref("equipment.config/1", "machine-7-config-a", '4'),
            tooling_refs: vec![],
            fixture_workholding_refs: vec![],
            material_applicability_refs: vec![],
            environment_requirement_refs: vec![],
            utility_requirement_refs: vec![],
            operator_competence_refs: vec![],
            safety_profile_refs: vec![],
            inspection_profile_refs: vec![],
            calibration_profile_refs: vec![],
            declared_capacity_profile_ref: owner_ref("capacity.profile/1", "capacity-a", '5'),
            availability_calendar_ref: None,
            commercial_terms_ref: None,
            capacity_claim_source: CapacityClaimSourceV1::ProviderDeclared,
            capacity_evidence_refs: vec![],
            lifecycle_event_refs: vec![],
            publication_state: ResourceOfferPublicationStateV1::Published,
            issued_at_unix_s: 1_000,
            valid_from_unix_s: 1_001,
            valid_until_unix_s: 2_000,
            supersedes_offer_ref: None,
        }
    }

    #[test]
    fn valid_offer_is_only_a_typed_declaration() {
        let offer = valid_offer();
        assert_eq!(offer.validate(), Ok(()));
        assert_eq!(
            offer.resolution_state(),
            ResourceOfferReferenceStateV1::TypedOfferPresent
        );
    }

    #[test]
    fn capability_profile_for_a_different_process_is_rejected() {
        let mut offer = valid_offer();
        offer.capability_profile_ref.0.process_id_blake3 = "f".repeat(64);
        assert!(offer.validate().is_err());
    }

    #[test]
    fn wrong_owner_schema_reference_digest_is_rejected() {
        let mut offer = valid_offer();
        offer.configuration_ref.payload_sha256 = "not-a-digest".into();
        assert!(offer.validate().is_err());
    }

    #[test]
    fn malformed_provider_repository_coordinate_is_rejected() {
        let mut offer = valid_offer();
        offer.provider_subject_ref.owner_repository = "provider".into();
        assert!(offer.validate().is_err());
    }

    #[test]
    fn owner_repository_requires_two_nonempty_segments() {
        for invalid in ["/provider", "owner/", "owner/repository/extra"] {
            let mut offer = valid_offer();
            offer.provider_subject_ref.owner_repository = invalid.into();
            assert!(
                offer.validate().is_err(),
                "accepted invalid repo coordinate: {invalid}"
            );
        }
    }

    #[test]
    fn reversed_offer_validity_window_is_rejected() {
        let mut offer = valid_offer();
        offer.valid_from_unix_s = 2_000;
        assert!(offer.validate().is_err());
    }

    #[test]
    fn duplicate_material_applicability_reference_is_rejected() {
        let mut offer = valid_offer();
        let reference = owner_ref("material.profile/1", "polymer-grade-a", '6');
        offer.material_applicability_refs = vec![reference.clone(), reference];
        assert!(offer.validate().is_err());
    }

    #[test]
    fn self_supersession_is_rejected() {
        let mut offer = valid_offer();
        let mut previous = owner_ref("resource.offer/1", &offer.offer_id, '9');
        previous.source_revision = offer.offer_revision.clone();
        offer.supersedes_offer_ref = Some(previous);
        assert!(offer.validate().is_err());
    }

    #[test]
    fn superseding_another_immutable_offer_is_only_a_reference() {
        let mut offer = valid_offer();
        let mut previous = owner_ref("resource.offer/1", &offer.offer_id, '9');
        previous.source_revision = "A".into();
        offer.offer_revision = "B".into();
        offer.supersedes_offer_ref = Some(previous);
        assert_eq!(offer.validate(), Ok(()));
        // A resolver must fetch the exact prior offer and validate its ancestry.
    }

    #[test]
    fn dispute_event_reference_does_not_self_promote_publication_state() {
        let mut offer = valid_offer();
        offer.lifecycle_event_refs = vec![owner_ref("resource.dispute/1", "dispute-event-001", '8')];
        assert_eq!(offer.validate(), Ok(()));
        assert_eq!(
            offer.publication_disposition_at(1_500).unwrap(),
            ResourceOfferPublicationDispositionV1::ActiveOfferWindow
        );
        // The event remains unresolved until its owner-native resolver runs.
    }

    #[test]
    fn capacity_profile_does_not_create_calendar_or_reservation_evidence() {
        let offer = valid_offer();
        assert!(offer.availability_calendar_ref.is_none());
        assert!(offer.supersedes_offer_ref.is_none());
        assert_eq!(
            offer.publication_state,
            ResourceOfferPublicationStateV1::Published
        );
        // The offer schema has no available-now, bookable, reserved or executed boolean.
    }

    #[test]
    fn active_publication_window_does_not_imply_capacity_availability() {
        let offer = valid_offer();
        assert_eq!(
            offer.publication_disposition_at(1_500).unwrap(),
            ResourceOfferPublicationDispositionV1::ActiveOfferWindow
        );
        assert!(offer.availability_calendar_ref.is_none());
    }

    #[test]
    fn expiry_is_derived_from_the_validity_interval() {
        let offer = valid_offer();
        assert_eq!(
            offer.publication_disposition_at(2_000).unwrap(),
            ResourceOfferPublicationDispositionV1::Expired
        );
    }

    #[test]
    fn non_provider_capacity_claim_requires_a_supporting_reference() {
        let mut offer = valid_offer();
        offer.capacity_claim_source = CapacityClaimSourceV1::ThirdPartyEvidenceReferenced;
        assert!(offer.validate().is_err());

        offer.capacity_evidence_refs = vec![owner_ref("inspection.evidence/1", "evidence-1", '8')];
        assert_eq!(offer.validate(), Ok(()));
        // The referenced evidence is not authenticated or independently qualified here.
    }

    #[test]
    fn offer_schema_does_not_serialize_available_bookable_reserved_or_executed_flags() {
        let value = serde_json::to_value(valid_offer()).unwrap();
        let object = value.as_object().unwrap();
        for forbidden in [
            "available_now",
            "bookable_capacity",
            "reserved_capacity",
            "executed_output",
            "qualified_for_production",
        ] {
            assert!(!object.contains_key(forbidden));
        }
    }

    #[test]
    fn frozen_resource_offer_corpus_has_seventeen_fail_closed_cases() {
        let fixture: serde_json::Value = serde_json::from_str(include_str!(
            "../../../tests/fixtures/mfg_proc_001b_resource_offer_v0_1.json"
        ))
        .expect("resource-offer fixture JSON must parse");
        assert_eq!(
            fixture["schema_id"],
            "mycelix.manufacturing.mfg-proc-001b.resource-offer.cases.v0_1"
        );
        let cases = fixture["cases"]
            .as_array()
            .expect("resource-offer fixture cases must be an array");
        assert_eq!(fixture["case_count"].as_u64(), Some(cases.len() as u64));
        assert_eq!(cases.len(), 17);
        for case in cases {
            assert!(
                case["id"]
                    .as_str()
                    .is_some_and(|id| id.starts_with("MFG-PROC-001B-"))
            );
            for boundary in [
                "may_promote_to_qualified_capability",
                "may_claim_calendar_available",
                "may_claim_bookable_capacity",
                "may_claim_reserved_capacity",
                "may_claim_executed_output",
            ] {
                assert_eq!(case[boundary], false, "{boundary} must remain false");
            }
        }
    }

    #[test]
    fn provider_cannot_self_assert_disputed_or_superseded_publication_state() {
        let offer = valid_offer();
        let mut value = serde_json::to_value(offer).unwrap();
        value["publication_state"] = serde_json::Value::String("Disputed".into());
        assert!(serde_json::from_value::<ManufacturingResourceOfferV1>(value).is_err());

        let mut value = serde_json::to_value(valid_offer()).unwrap();
        value["publication_state"] = serde_json::Value::String("Superseded".into());
        assert!(serde_json::from_value::<ManufacturingResourceOfferV1>(value).is_err());
    }

    #[test]
    fn provider_authority_reference_is_optional_and_never_self_authenticating() {
        let mut offer = valid_offer();
        offer.provider_authority_ref = Some(owner_ref("provider.authority/1", "delegation-1", '7'));
        assert_eq!(offer.validate(), Ok(()));
        // The reference is structurally present only; resolving its signer/resource relationship
        // belongs to the identity/resource owner and is intentionally outside this validator.
    }
}
