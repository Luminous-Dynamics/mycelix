// Copyright (C) 2024-2026 Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Exact membership evidence for normalized observations used by Business shadow qualification.
//!
//! This crate replays the exact extraction campaign through the same read-only adapter and proves
//! that each claimed forecast actual is one of the deterministic normalized observations produced
//! from those source files. Raw rows are never copied into qualification evidence.

use std::collections::{BTreeMap, BTreeSet};

use csv::{ReaderBuilder, StringRecord, WriterBuilder};
use mycelix_business_adapter_delimited::DelimitedIngressAdapter;
use mycelix_business_core::{Digest32, ObservationRef, ReferenceId};
use mycelix_business_field_qualification::DataQualityEvidence;
use mycelix_business_hospitality_derived_evidence::{
    DerivedEvidenceError, DerivedHospitalityReport, FixedOffsetSlicePlan,
    forecast_actual_campaign_membership_unverified_ref,
    derive_and_evaluate_hospitality_pilot,
};
use mycelix_business_import_diagnostics::{
    CampaignError, DiagnosticError, ExtractionCampaignManifest,
    diagnose_delimited_import,
};
use mycelix_business_ingress::IngressQualificationBinding;
use mycelix_business_pilot_hospitality::HospitalityForecastPilotRegistration;
use mycelix_business_shadow::{ForecastCase, MetricObservation};
use sha2::{Digest, Sha256};

pub const OBSERVATION_MEMBERSHIP_IS_READ_ONLY: bool = true;

fn zero_digest(value: &Digest32) -> bool {
    value == &Digest32([0; 32])
}

fn hash_str(hasher: &mut Sha256, value: &str) {
    hasher.update((value.len() as u64).to_be_bytes());
    hasher.update(value.as_bytes());
}

fn finish_digest(hasher: Sha256) -> Digest32 {
    Digest32(hasher.finalize().into())
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CampaignSourceFile {
    pub bytes: Vec<u8>,
    /// Must match the ingestion time used to produce the registered diagnostic manifest.
    pub ingested_at_unix_ms: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ObservationClaimBinding {
    pub observation: ObservationRef,
    pub claim_digest: Digest32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ObservationMembershipEvidence {
    pub campaign_digest: Digest32,
    pub connector: IngressQualificationBinding,
    pub source_file_digests: Vec<Digest32>,
    pub claims: Vec<ObservationClaimBinding>,
    pub evidence_digest: Digest32,
}

#[derive(Debug)]
pub enum MembershipError {
    Campaign(CampaignError),
    AdapterConfiguration,
    ConnectorMismatch,
    EmptyClaims,
    InvalidClaim { index: usize },
    DuplicateClaim(ObservationRef),
    SourceFileCountMismatch { expected: usize, actual: usize },
    Diagnostic { index: usize, error: DiagnosticError },
    UnexpectedSourceFile { index: usize },
    ManifestMismatch { index: usize, source_file_digest: Digest32 },
    MissingCampaignFile,
    Csv { index: usize },
    Serialization { index: usize },
    ObservationIdentityCollision { observation: ObservationRef },
    MissingObservation { observation: ObservationRef },
    DuplicateObservationMatch { observation: ObservationRef },
    EvidenceDigestMismatch,
    ClaimSetMismatch,
}

impl ObservationMembershipEvidence {
    pub fn validate_against(
        &self,
        campaign: &ExtractionCampaignManifest,
        claims: &[MetricObservation],
    ) -> Result<(), MembershipError> {
        campaign.validate().map_err(MembershipError::Campaign)?;
        if zero_digest(&self.evidence_digest) || self.campaign_digest != campaign.campaign_digest {
            return Err(MembershipError::EvidenceDigestMismatch);
        }
        if self.connector != campaign.connector {
            return Err(MembershipError::ConnectorMismatch);
        }
        let expected_claims = canonical_claim_bindings(claims)?;
        if self.claims != expected_claims {
            return Err(MembershipError::ClaimSetMismatch);
        }
        let expected_files = campaign
            .files
            .iter()
            .map(|file| file.source_file_digest)
            .collect::<Vec<_>>();
        if self.source_file_digests != expected_files {
            return Err(MembershipError::MissingCampaignFile);
        }
        if self.evidence_digest != membership_evidence_digest(self) {
            return Err(MembershipError::EvidenceDigestMismatch);
        }
        Ok(())
    }
}

/// Reconstruct the exact registered extraction campaign and prove exact normalized-observation
/// membership for every claim.
pub fn verify_observation_membership(
    adapter: &DelimitedIngressAdapter,
    campaign: &ExtractionCampaignManifest,
    files: &[CampaignSourceFile],
    claims: &[MetricObservation],
) -> Result<ObservationMembershipEvidence, MembershipError> {
    campaign.validate().map_err(MembershipError::Campaign)?;
    if claims.is_empty() {
        return Err(MembershipError::EmptyClaims);
    }
    let descriptor = adapter
        .descriptor()
        .map_err(|_| MembershipError::AdapterConfiguration)?;
    let connector = IngressQualificationBinding::from(&descriptor);
    if connector != campaign.connector {
        return Err(MembershipError::ConnectorMismatch);
    }
    let canonical_claims = canonical_claim_bindings(claims)?;
    if files.len() != campaign.files.len() {
        return Err(MembershipError::SourceFileCountMismatch {
            expected: campaign.files.len(),
            actual: files.len(),
        });
    }

    let mut expected_manifests = campaign
        .files
        .iter()
        .map(|manifest| (manifest.source_file_digest, manifest))
        .collect::<BTreeMap<_, _>>();
    let claim_map = claims
        .iter()
        .map(|claim| (claim.observation.clone(), claim))
        .collect::<BTreeMap<_, _>>();
    let mut match_counts = claim_map
        .keys()
        .cloned()
        .map(|observation| (observation, 0_u32))
        .collect::<BTreeMap<_, _>>();
    let mut scanned_file_digests = Vec::with_capacity(files.len());

    for (file_index, file) in files.iter().enumerate() {
        let manifest = diagnose_delimited_import(
            adapter,
            file.bytes.as_slice(),
            file.ingested_at_unix_ms,
        )
        .map_err(|error| MembershipError::Diagnostic {
            index: file_index,
            error,
        })?;
        let Some(expected) = expected_manifests.remove(&manifest.source_file_digest) else {
            return Err(MembershipError::UnexpectedSourceFile { index: file_index });
        };
        if &manifest != expected {
            return Err(MembershipError::ManifestMismatch {
                index: file_index,
                source_file_digest: manifest.source_file_digest,
            });
        }
        scanned_file_digests.push(manifest.source_file_digest);
        scan_file_for_claims(
            adapter,
            file,
            &claim_map,
            &mut match_counts,
            file_index,
        )?;
    }
    if !expected_manifests.is_empty() {
        return Err(MembershipError::MissingCampaignFile);
    }

    for (observation, count) in &match_counts {
        match count {
            0 => {
                return Err(MembershipError::MissingObservation {
                    observation: observation.clone(),
                })
            }
            1 => {}
            _ => {
                return Err(MembershipError::DuplicateObservationMatch {
                    observation: observation.clone(),
                })
            }
        }
    }

    scanned_file_digests.sort();
    let mut evidence = ObservationMembershipEvidence {
        campaign_digest: campaign.campaign_digest,
        connector,
        source_file_digests: scanned_file_digests,
        claims: canonical_claims,
        evidence_digest: Digest32([0; 32]),
    };
    evidence.evidence_digest = membership_evidence_digest(&evidence);
    evidence.validate_against(campaign, claims)?;
    Ok(evidence)
}

fn canonical_claim_bindings(
    claims: &[MetricObservation],
) -> Result<Vec<ObservationClaimBinding>, MembershipError> {
    if claims.is_empty() {
        return Err(MembershipError::EmptyClaims);
    }
    let mut seen = BTreeSet::new();
    let mut bindings = Vec::with_capacity(claims.len());
    for (index, claim) in claims.iter().enumerate() {
        claim
            .validate()
            .map_err(|_| MembershipError::InvalidClaim { index })?;
        if !seen.insert(claim.observation.clone()) {
            return Err(MembershipError::DuplicateClaim(claim.observation.clone()));
        }
        bindings.push(ObservationClaimBinding {
            observation: claim.observation.clone(),
            claim_digest: metric_observation_digest(claim),
        });
    }
    bindings.sort_by(|left, right| left.observation.cmp(&right.observation));
    Ok(bindings)
}

fn scan_file_for_claims(
    adapter: &DelimitedIngressAdapter,
    file: &CampaignSourceFile,
    claim_map: &BTreeMap<ObservationRef, &MetricObservation>,
    match_counts: &mut BTreeMap<ObservationRef, u32>,
    file_index: usize,
) -> Result<(), MembershipError> {
    let mut csv = ReaderBuilder::new()
        .delimiter(adapter.config.delimiter)
        .has_headers(true)
        .flexible(false)
        .trim(csv::Trim::None)
        .from_reader(file.bytes.as_slice());
    let headers = csv
        .headers()
        .map_err(|_| MembershipError::Csv { index: file_index })?
        .clone();
    let expected = StringRecord::from(adapter.config.expected_headers.clone());
    if headers != expected {
        return Err(MembershipError::Csv { index: file_index });
    }

    // Mirror the diagnostic scanner: only the first accepted occurrence of a source event is
    // eligible. Later duplicates are rejected by denominator diagnostics and cannot satisfy claims.
    let mut accepted_source_events = BTreeSet::new();
    for row in csv.records() {
        let Ok(record) = row else {
            continue;
        };
        let one_row = serialize_single_record(adapter.config.delimiter, &headers, &record, file_index)?;
        let Ok(batch) = adapter.parse_batch(one_row.as_slice(), file.ingested_at_unix_ms) else {
            continue;
        };
        let Some(parsed) = batch.records.first() else {
            continue;
        };
        if !accepted_source_events.insert(parsed.event.witness.source_event_id.clone()) {
            continue;
        }
        for normalized in &parsed.normalized {
            let observation_id = &normalized.observation.observation;
            let Some(expected_claim) = claim_map.get(observation_id) else {
                continue;
            };
            if &normalized.observation != *expected_claim {
                return Err(MembershipError::ObservationIdentityCollision {
                    observation: observation_id.clone(),
                });
            }
            let count = match_counts
                .get_mut(observation_id)
                .expect("claim map and match counters have identical keys");
            *count = count.saturating_add(1);
        }
    }
    Ok(())
}

fn serialize_single_record(
    delimiter: u8,
    headers: &StringRecord,
    record: &StringRecord,
    file_index: usize,
) -> Result<Vec<u8>, MembershipError> {
    let mut bytes = Vec::new();
    {
        let mut writer = WriterBuilder::new().delimiter(delimiter).from_writer(&mut bytes);
        writer
            .write_record(headers)
            .map_err(|_| MembershipError::Serialization { index: file_index })?;
        writer
            .write_record(record)
            .map_err(|_| MembershipError::Serialization { index: file_index })?;
        writer
            .flush()
            .map_err(|_| MembershipError::Serialization { index: file_index })?;
    }
    Ok(bytes)
}

fn metric_observation_digest(observation: &MetricObservation) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:normalized-observation-claim:v1");
    hash_str(&mut hasher, observation.observation.as_ref_id().as_str());
    hash_str(&mut hasher, observation.source_system.as_str());
    hash_str(&mut hasher, observation.source_event_id.as_str());
    hasher.update(observation.source_payload_digest.0);
    hasher.update(observation.mapping_digest.0);
    hash_str(&mut hasher, observation.metric.as_str());
    hash_str(&mut hasher, observation.scope.as_ref_id().as_str());
    hasher.update(observation.value.mantissa.to_be_bytes());
    hasher.update(observation.value.scale.to_be_bytes());
    hash_str(&mut hasher, observation.value.unit.as_str());
    hasher.update(observation.observed_at_unix_ms.to_be_bytes());
    finish_digest(hasher)
}

fn membership_evidence_digest(evidence: &ObservationMembershipEvidence) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:observation-membership-evidence:v1");
    hasher.update(evidence.campaign_digest.0);
    hash_str(&mut hasher, evidence.connector.source_system.as_str());
    hash_str(&mut hasher, evidence.connector.adapter_semantic_id.as_str());
    hasher.update(evidence.connector.adapter_digest.0);
    hasher.update(evidence.connector.mapping_digest.0);
    hasher.update(evidence.connector.source_schema_digest.0);
    for digest in &evidence.source_file_digests {
        hasher.update(digest.0);
    }
    for claim in &evidence.claims {
        hash_str(&mut hasher, claim.observation.as_ref_id().as_str());
        hasher.update(claim.claim_digest.0);
    }
    finish_digest(hasher)
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LimitationDischarge {
    pub limitation: ReferenceId,
    pub campaign_digest: Digest32,
    pub supporting_evidence_digest: Digest32,
    pub discharge_digest: Digest32,
}

impl LimitationDischarge {
    fn actual_membership(
        membership: &ObservationMembershipEvidence,
    ) -> Self {
        let limitation = forecast_actual_campaign_membership_unverified_ref();
        let mut discharge = Self {
            limitation,
            campaign_digest: membership.campaign_digest,
            supporting_evidence_digest: membership.evidence_digest,
            discharge_digest: Digest32([0; 32]),
        };
        discharge.discharge_digest = limitation_discharge_digest(&discharge);
        discharge
    }

    pub fn validate(&self) -> bool {
        !zero_digest(&self.supporting_evidence_digest)
            && !zero_digest(&self.campaign_digest)
            && self.discharge_digest == limitation_discharge_digest(self)
    }
}

fn limitation_discharge_digest(discharge: &LimitationDischarge) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:limitation-discharge:v1");
    hash_str(&mut hasher, discharge.limitation.as_str());
    hasher.update(discharge.campaign_digest.0);
    hasher.update(discharge.supporting_evidence_digest.0);
    finish_digest(hasher)
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct VerifiedHospitalityReport {
    pub derived: DerivedHospitalityReport,
    pub membership: ObservationMembershipEvidence,
    /// The inner immutable report retains the original limitation. This outer evidence explicitly
    /// discharges it rather than rewriting prior evidence.
    pub membership_limitation_discharge: LimitationDischarge,
}

#[derive(Debug)]
pub enum VerifiedHospitalityError {
    Derived(DerivedEvidenceError),
    Membership(MembershipError),
}

/// Strong qualification path: validate the ordinary derived report first, then prove every actual
/// observation against the exact source campaign and emit a separate limitation-discharge record.
pub fn derive_evaluate_and_verify_actual_membership(
    registration: &HospitalityForecastPilotRegistration,
    clock: &FixedOffsetSlicePlan,
    adapter: &DelimitedIngressAdapter,
    campaign: ExtractionCampaignManifest,
    source_files: &[CampaignSourceFile],
    cases: Vec<ForecastCase>,
    data_quality: Vec<DataQualityEvidence>,
    known_limitations: BTreeSet<ReferenceId>,
) -> Result<VerifiedHospitalityReport, VerifiedHospitalityError> {
    let actuals = cases
        .iter()
        .map(|case| case.actual.clone())
        .collect::<Vec<_>>();

    // Run field/extraction qualification first so a missing membership proof cannot mask malformed
    // denominator or connector evidence.
    let derived = derive_and_evaluate_hospitality_pilot(
        registration,
        clock,
        campaign.clone(),
        cases,
        data_quality,
        known_limitations,
    )
    .map_err(VerifiedHospitalityError::Derived)?;

    let membership = verify_observation_membership(
        adapter,
        &campaign,
        source_files,
        &actuals,
    )
    .map_err(VerifiedHospitalityError::Membership)?;
    let discharge = LimitationDischarge::actual_membership(&membership);
    debug_assert!(discharge.validate());

    Ok(VerifiedHospitalityReport {
        derived,
        membership,
        membership_limitation_discharge: discharge,
    })
}

#[cfg(test)]
mod tests {
    use mycelix_business_adapter_delimited::{
        DecimalPolicy, DelimitedAdapterConfig, OutputMapping, ScopeMapping, TimestampEncoding,
        ValueMapping,
    };
    use mycelix_business_core::{Digest32, ForecastRef, ObservationRef, ScopeRef};
    use mycelix_business_import_diagnostics::{
        diagnose_delimited_import, upstream_export_completeness_unverified_ref,
    };
    use mycelix_business_pilot_hospitality::{
        DAY_MS, HOUR_MS, HospitalityForecastPilotConfig, HospitalityPilotPolicy, sales_input_ref,
        standard_daypart_slices_v1,
    };
    use mycelix_business_shadow::{
        ForecastDisposition, ForecastTarget, ForecastValue, ScaledValue, ShadowForecast,
    };

    use super::*;

    const INGESTED_AT: u64 = 1_790_000_000_000;

    fn id(value: &str) -> ReferenceId {
        ReferenceId::new(value).unwrap()
    }

    fn adapter() -> DelimitedIngressAdapter {
        DelimitedIngressAdapter::new(DelimitedAdapterConfig {
            adapter_semantic_id: id("adapter:test-membership:v1"),
            source_system: id("source:test-pos"),
            adapter_digest: Digest32::repeat(1),
            source_schema: id("schema:test-membership:v1"),
            delimiter: b',',
            expected_headers: vec![
                "event_id".into(),
                "occurred_at".into(),
                "location".into(),
                "quantity".into(),
            ],
            source_event_id_column: "event_id".into(),
            observed_at_column: "occurred_at".into(),
            timestamp_encoding: TimestampEncoding::UnixMilliseconds,
            scope: ScopeMapping::Column {
                column: "location".into(),
                prefix: "scope:location:".into(),
            },
            outputs: vec![OutputMapping {
                input: sales_input_ref(),
                metric: id("metric:hospitality:item-demand"),
                value: ValueMapping::Column {
                    column: "quantity".into(),
                    decimal: DecimalPolicy::default(),
                },
                unit: id("unit:count"),
                scale: 0,
            }],
            maximum_batch_records: 100,
        })
        .unwrap()
    }

    fn csv() -> Vec<u8> {
        format!(
            "event_id,occurred_at,location,quantity\n\
event:breakfast,{},a,10\n\
event:lunch,{},a,20\n\
event:evening,{},a,30\n\
event:weekend,{},a,40\n",
            1_788_760_800_000_u64 + HOUR_MS,
            1_788_775_200_000_u64 + HOUR_MS,
            1_788_796_800_000_u64 + HOUR_MS,
            1_789_192_800_000_u64 + HOUR_MS,
        )
        .into_bytes()
    }

    fn campaign(
        adapter: &DelimitedIngressAdapter,
        bytes: &[u8],
    ) -> ExtractionCampaignManifest {
        let manifest =
            diagnose_delimited_import(adapter, bytes, INGESTED_AT).unwrap();
        ExtractionCampaignManifest::from_files(vec![manifest]).unwrap()
    }

    fn registration(
        connector: IngressQualificationBinding,
    ) -> HospitalityForecastPilotRegistration {
        HospitalityForecastPilotRegistration::build(HospitalityForecastPilotConfig {
            plan_id: id("pilot:membership-test:v1"),
            scope: ScopeRef(id("scope:location:a")),
            timezone: id("timezone:iana:Africa/Johannesburg"),
            candidate_model_lineage: id("model:candidate:v1"),
            connector,
            registered_at_unix_ms: 1_788_566_400_000,
            evaluation_start_unix_ms: 1_788_739_200_000,
            evaluation_end_unix_ms: 1_789_344_000_000,
            policy: HospitalityPilotPolicy {
                minimum_preregistration_lead_ms: HOUR_MS,
                minimum_evaluation_duration_ms: DAY_MS,
                minimum_total_forecast_cases: 1,
                minimum_cases_per_slice: 1,
                maximum_abstention_bps: 10_000,
                maximum_missing_bps: 10_000,
                maximum_conflicting_bps: 10_000,
                maximum_stale_bps: 10_000,
                maximum_ingest_delay_ms: u64::MAX,
            },
            slices: Some(standard_daypart_slices_v1(
                id("timezone:iana:Africa/Johannesburg"),
                1,
            )),
        })
        .unwrap()
    }

    fn actuals_from_export(
        adapter: &DelimitedIngressAdapter,
        bytes: &[u8],
    ) -> Vec<MetricObservation> {
        let batch = adapter.parse_batch(bytes, INGESTED_AT).unwrap();
        batch
            .records
            .into_iter()
            .flat_map(|record| record.normalized.into_iter())
            .map(|normalized| normalized.observation)
            .collect()
    }

    fn cases(
        registration: &HospitalityForecastPilotRegistration,
        actuals: &[MetricObservation],
    ) -> Vec<ForecastCase> {
        actuals
            .iter()
            .enumerate()
            .map(|(index, actual)| {
                let target_start = actual.observed_at_unix_ms - HOUR_MS;
                let target = ForecastTarget {
                    metric: actual.metric.clone(),
                    scope: actual.scope.clone(),
                    unit: actual.value.unit.clone(),
                    scale: actual.value.scale,
                    window_start_unix_ms: target_start,
                    window_end_unix_ms: actual.observed_at_unix_ms,
                };
                let forecast_value = |point: i128| ForecastValue {
                    point: ScaledValue {
                        mantissa: point,
                        scale: actual.value.scale,
                        unit: actual.value.unit.clone(),
                    },
                    lower: ScaledValue {
                        mantissa: point.saturating_sub(1),
                        scale: actual.value.scale,
                        unit: actual.value.unit.clone(),
                    },
                    upper: ScaledValue {
                        mantissa: point.saturating_add(1),
                        scale: actual.value.scale,
                        unit: actual.value.unit.clone(),
                    },
                };
                ForecastCase {
                    candidate: ShadowForecast {
                        forecast: ForecastRef(id(&format!("forecast:candidate:{index}"))),
                        model_lineage: registration.protocol.candidate_model_lineage.clone(),
                        target: target.clone(),
                        issued_at_unix_ms: target_start - HOUR_MS,
                        disposition: ForecastDisposition::Predicted(forecast_value(
                            actual.value.mantissa,
                        )),
                    },
                    baseline: ShadowForecast {
                        forecast: ForecastRef(id(&format!("forecast:baseline:{index}"))),
                        model_lineage: registration.protocol.baseline_model_lineage.clone(),
                        target,
                        issued_at_unix_ms: target_start - HOUR_MS,
                        disposition: ForecastDisposition::Predicted(forecast_value(
                            actual.value.mantissa.saturating_add(5),
                        )),
                    },
                    actual: actual.clone(),
                }
            })
            .collect()
    }

    fn quality(campaign: &ExtractionCampaignManifest) -> Vec<DataQualityEvidence> {
        vec![DataQualityEvidence {
            input: sales_input_ref(),
            expected_records: campaign.discovered_rows,
            missing_records: campaign.rejected_rows,
            conflicting_records: 0,
            stale_records: 0,
            maximum_observed_ingest_delay_ms: campaign.maximum_observed_ingest_delay_ms,
            evidence_digest: Digest32::repeat(9),
        }]
    }

    #[test]
    fn exact_normalized_actuals_are_proven_members_of_campaign() {
        let adapter = adapter();
        let bytes = csv();
        let campaign = campaign(&adapter, &bytes);
        let actuals = actuals_from_export(&adapter, &bytes);
        let evidence = verify_observation_membership(
            &adapter,
            &campaign,
            &[CampaignSourceFile {
                bytes,
                ingested_at_unix_ms: INGESTED_AT,
            }],
            &actuals,
        )
        .unwrap();
        assert!(OBSERVATION_MEMBERSHIP_IS_READ_ONLY);
        assert_eq!(evidence.claims.len(), actuals.len());
        assert!(evidence.validate_against(&campaign, &actuals).is_ok());
    }

    #[test]
    fn substituted_actual_cannot_reuse_observation_identity() {
        let adapter = adapter();
        let bytes = csv();
        let campaign = campaign(&adapter, &bytes);
        let mut actuals = actuals_from_export(&adapter, &bytes);
        actuals[0].value.mantissa = actuals[0].value.mantissa.saturating_add(1);
        assert!(matches!(
            verify_observation_membership(
                &adapter,
                &campaign,
                &[CampaignSourceFile {
                    bytes,
                    ingested_at_unix_ms: INGESTED_AT,
                }],
                &actuals,
            ),
            Err(MembershipError::ObservationIdentityCollision { .. })
        ));
    }

    #[test]
    fn wrong_source_file_set_fails_manifest_reconstruction() {
        let adapter = adapter();
        let bytes = csv();
        let campaign = campaign(&adapter, &bytes);
        let actuals = actuals_from_export(&adapter, &bytes);
        let wrong = b"event_id,occurred_at,location,quantity\nevent:x,1000,a,1\n".to_vec();
        assert!(matches!(
            verify_observation_membership(
                &adapter,
                &campaign,
                &[CampaignSourceFile {
                    bytes: wrong,
                    ingested_at_unix_ms: INGESTED_AT,
                }],
                &actuals,
            ),
            Err(MembershipError::UnexpectedSourceFile { .. })
                | Err(MembershipError::ManifestMismatch { .. })
        ));
    }

    #[test]
    fn strong_wrapper_emits_limitation_discharge_without_rewriting_inner_report() {
        let adapter = adapter();
        let bytes = csv();
        let campaign = campaign(&adapter, &bytes);
        let registration = registration(campaign.connector.clone());
        let actuals = actuals_from_export(&adapter, &bytes);
        let cases = cases(&registration, &actuals);
        let clock = FixedOffsetSlicePlan::build(
            &registration,
            registration.protocol.registered_at_unix_ms,
            120,
        )
        .unwrap();
        let mut limitations = BTreeSet::new();
        limitations.insert(upstream_export_completeness_unverified_ref());
        let report = derive_evaluate_and_verify_actual_membership(
            &registration,
            &clock,
            &adapter,
            campaign.clone(),
            &[CampaignSourceFile {
                bytes,
                ingested_at_unix_ms: INGESTED_AT,
            }],
            cases,
            quality(&campaign),
            limitations,
        )
        .unwrap();
        assert_eq!(
            report.membership_limitation_discharge.limitation,
            forecast_actual_campaign_membership_unverified_ref()
        );
        assert!(report.membership_limitation_discharge.validate());
    }
}
