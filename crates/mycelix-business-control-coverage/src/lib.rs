// Copyright (C) 2024-2026 Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Preregistered full-interval control coverage for read-only Business qualification.
//!
//! One matching control statement proves only one scope/window. This crate proves that an exact set
//! of preregistered matching control windows covers a larger qualification interval contiguously,
//! under one pinned connector/metric/aggregation/control-source contract family.

use std::collections::{BTreeMap, BTreeSet};

use mycelix_business_campaign_replay::CampaignReplay;
use mycelix_business_control_reconciliation::{
    ControlAggregationKind, ControlReconciliationContract, ControlReconciliationDecision,
    ControlReconciliationEvidence, ControlSourceClass, ControlTotalStatement, ReconciliationError,
    ScopedLimitationTransition, aggregation_semantic_authority_unverified_ref,
    control_metric_semantic_authority_unverified_ref,
    control_source_external_reality_unverified_ref, upstream_export_completeness_unverified_ref,
};
use mycelix_business_core::{Digest32, ReferenceId, ScopeRef};
use mycelix_business_import_diagnostics::ExtractionCampaignManifest;
use mycelix_business_ingress::IngressQualificationBinding;
use mycelix_business_shadow::ScaledValue;
use sha2::{Digest, Sha256};

pub const CONTROL_COVERAGE_IS_READ_ONLY: bool = true;

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
pub struct ControlCoverageWindow {
    pub window_id: ReferenceId,
    pub start_unix_ms: u64,
    pub end_unix_ms: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ControlCoveragePlan {
    pub plan_id: ReferenceId,
    pub connector: IngressQualificationBinding,
    pub source_input: ReferenceId,
    pub metric: ReferenceId,
    pub scope: ScopeRef,
    pub unit: ReferenceId,
    pub scale: u32,
    pub aggregation: ControlAggregationKind,
    pub control_source: ReferenceId,
    pub control_source_class: ControlSourceClass,
    pub registered_at_unix_ms: u64,
    pub qualification_start_unix_ms: u64,
    pub qualification_end_unix_ms: u64,
    pub windows: Vec<ControlCoverageWindow>,
    pub plan_digest: Digest32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CoveragePlanError {
    ZeroConnectorDigest,
    InvalidQualificationWindow,
    NotPreregistered,
    ExcessiveScale,
    NoWindows,
    InvalidWindow { index: usize },
    DuplicateWindowId { window_id: ReferenceId },
    FirstWindowDoesNotStartAtQualification,
    LastWindowDoesNotEndAtQualification,
    GapOrOverlap { previous: usize, next: usize },
    ZeroDigest,
    DigestMismatch,
}

impl ControlCoveragePlan {
    #[allow(clippy::too_many_arguments)]
    pub fn build(
        plan_id: ReferenceId,
        connector: IngressQualificationBinding,
        source_input: ReferenceId,
        metric: ReferenceId,
        scope: ScopeRef,
        unit: ReferenceId,
        scale: u32,
        aggregation: ControlAggregationKind,
        control_source: ReferenceId,
        control_source_class: ControlSourceClass,
        registered_at_unix_ms: u64,
        qualification_start_unix_ms: u64,
        qualification_end_unix_ms: u64,
        mut windows: Vec<ControlCoverageWindow>,
    ) -> Result<Self, CoveragePlanError> {
        windows.sort_by(|left, right| {
            left.start_unix_ms
                .cmp(&right.start_unix_ms)
                .then_with(|| left.end_unix_ms.cmp(&right.end_unix_ms))
                .then_with(|| left.window_id.cmp(&right.window_id))
        });
        let mut value = Self {
            plan_id,
            connector,
            source_input,
            metric,
            scope,
            unit,
            scale,
            aggregation,
            control_source,
            control_source_class,
            registered_at_unix_ms,
            qualification_start_unix_ms,
            qualification_end_unix_ms,
            windows,
            plan_digest: Digest32([0; 32]),
        };
        value.plan_digest = plan_digest(&value);
        value.validate()?;
        Ok(value)
    }

    pub fn validate(&self) -> Result<(), CoveragePlanError> {
        if zero_digest(&self.connector.adapter_digest)
            || zero_digest(&self.connector.mapping_digest)
            || zero_digest(&self.connector.source_schema_digest)
        {
            return Err(CoveragePlanError::ZeroConnectorDigest);
        }
        if self.qualification_start_unix_ms == 0
            || self.qualification_start_unix_ms >= self.qualification_end_unix_ms
        {
            return Err(CoveragePlanError::InvalidQualificationWindow);
        }
        if self.registered_at_unix_ms == 0
            || self.registered_at_unix_ms >= self.qualification_start_unix_ms
        {
            return Err(CoveragePlanError::NotPreregistered);
        }
        if self.scale > ScaledValue::MAX_SCALE {
            return Err(CoveragePlanError::ExcessiveScale);
        }
        if self.windows.is_empty() {
            return Err(CoveragePlanError::NoWindows);
        }
        let mut ids = BTreeSet::new();
        for (index, window) in self.windows.iter().enumerate() {
            if window.start_unix_ms >= window.end_unix_ms
                || window.start_unix_ms < self.qualification_start_unix_ms
                || window.end_unix_ms > self.qualification_end_unix_ms
            {
                return Err(CoveragePlanError::InvalidWindow { index });
            }
            if !ids.insert(window.window_id.clone()) {
                return Err(CoveragePlanError::DuplicateWindowId {
                    window_id: window.window_id.clone(),
                });
            }
        }
        if self.windows[0].start_unix_ms != self.qualification_start_unix_ms {
            return Err(CoveragePlanError::FirstWindowDoesNotStartAtQualification);
        }
        if self.windows[self.windows.len() - 1].end_unix_ms != self.qualification_end_unix_ms {
            return Err(CoveragePlanError::LastWindowDoesNotEndAtQualification);
        }
        for (index, pair) in self.windows.windows(2).enumerate() {
            if pair[0].end_unix_ms != pair[1].start_unix_ms {
                return Err(CoveragePlanError::GapOrOverlap {
                    previous: index,
                    next: index + 1,
                });
            }
        }
        if zero_digest(&self.plan_digest) {
            return Err(CoveragePlanError::ZeroDigest);
        }
        if self.plan_digest != plan_digest(self) {
            return Err(CoveragePlanError::DigestMismatch);
        }
        Ok(())
    }
}

fn plan_digest(value: &ControlCoveragePlan) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:control-coverage-plan:v1");
    hash_str(&mut hasher, value.plan_id.as_str());
    hash_connector(&mut hasher, &value.connector);
    hash_str(&mut hasher, value.source_input.as_str());
    hash_str(&mut hasher, value.metric.as_str());
    hash_str(&mut hasher, value.scope.as_ref_id().as_str());
    hash_str(&mut hasher, value.unit.as_str());
    hasher.update(value.scale.to_be_bytes());
    hasher.update([match value.aggregation {
        ControlAggregationKind::Sum => 1,
    }]);
    hash_str(&mut hasher, value.control_source.as_str());
    hasher.update([match value.control_source_class {
        ControlSourceClass::ProviderAuthoritativeReport => 1,
        ControlSourceClass::IndependentAudit => 2,
    }]);
    hasher.update(value.registered_at_unix_ms.to_be_bytes());
    hasher.update(value.qualification_start_unix_ms.to_be_bytes());
    hasher.update(value.qualification_end_unix_ms.to_be_bytes());
    hasher.update((value.windows.len() as u64).to_be_bytes());
    for window in &value.windows {
        hash_str(&mut hasher, window.window_id.as_str());
        hasher.update(window.start_unix_ms.to_be_bytes());
        hasher.update(window.end_unix_ms.to_be_bytes());
    }
    finish_digest(hasher)
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ControlCoverageEntry {
    pub window_id: ReferenceId,
    pub contract: ControlReconciliationContract,
    pub statement: ControlTotalStatement,
    pub reconciliation: ControlReconciliationEvidence,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct WindowCoverageEvidence {
    pub window_id: ReferenceId,
    pub contract_digest: Digest32,
    pub statement_digest: Digest32,
    pub reconciliation_digest: Digest32,
    pub transition_digests: Vec<Digest32>,
    pub window_digest: Digest32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ControlCoverageEvidence {
    pub plan_digest: Digest32,
    pub campaign_digest: Digest32,
    pub replay_digest: Digest32,
    pub windows: Vec<WindowCoverageEvidence>,
    pub coverage_digest: Digest32,
}

#[derive(Debug)]
pub enum CoverageError {
    Plan(CoveragePlanError),
    EntryCountMismatch { expected: usize, actual: usize },
    DuplicateEntry { window_id: ReferenceId },
    UnknownWindow { window_id: ReferenceId },
    MissingWindow { window_id: ReferenceId },
    ContractInvalid { window_id: ReferenceId },
    ContractRegisteredAfterPlan { window_id: ReferenceId },
    ContractSemanticsMismatch { window_id: ReferenceId },
    ContractWindowMismatch { window_id: ReferenceId },
    StatementOrEvidence { window_id: ReferenceId, error: ReconciliationError },
    ReconciliationDidNotMatch { window_id: ReferenceId },
    TransitionMismatch { window_id: ReferenceId },
    ZeroDigest,
    DigestMismatch,
}

pub fn verify_control_coverage(
    plan: &ControlCoveragePlan,
    campaign: &ExtractionCampaignManifest,
    replay: &CampaignReplay,
    entries: &[ControlCoverageEntry],
) -> Result<ControlCoverageEvidence, CoverageError> {
    plan.validate().map_err(CoverageError::Plan)?;
    if entries.len() != plan.windows.len() {
        return Err(CoverageError::EntryCountMismatch {
            expected: plan.windows.len(),
            actual: entries.len(),
        });
    }
    let planned = plan
        .windows
        .iter()
        .map(|window| (window.window_id.clone(), window))
        .collect::<BTreeMap<_, _>>();
    let mut seen = BTreeSet::new();
    let mut evidence_by_window = BTreeMap::new();

    for entry in entries {
        if !seen.insert(entry.window_id.clone()) {
            return Err(CoverageError::DuplicateEntry {
                window_id: entry.window_id.clone(),
            });
        }
        let Some(window) = planned.get(&entry.window_id) else {
            return Err(CoverageError::UnknownWindow {
                window_id: entry.window_id.clone(),
            });
        };
        entry
            .contract
            .validate()
            .map_err(|_| CoverageError::ContractInvalid {
                window_id: entry.window_id.clone(),
            })?;
        if entry.contract.registered_at_unix_ms > plan.registered_at_unix_ms {
            return Err(CoverageError::ContractRegisteredAfterPlan {
                window_id: entry.window_id.clone(),
            });
        }
        validate_contract_semantics(plan, &entry.contract, &entry.window_id)?;
        if entry.contract.window_start_unix_ms != window.start_unix_ms
            || entry.contract.window_end_unix_ms != window.end_unix_ms
        {
            return Err(CoverageError::ContractWindowMismatch {
                window_id: entry.window_id.clone(),
            });
        }
        entry
            .reconciliation
            .validate_against(campaign, replay, &entry.contract, &entry.statement)
            .map_err(|error| CoverageError::StatementOrEvidence {
                window_id: entry.window_id.clone(),
                error,
            })?;
        if entry.reconciliation.decision != ControlReconciliationDecision::Match {
            return Err(CoverageError::ReconciliationDidNotMatch {
                window_id: entry.window_id.clone(),
            });
        }
        let transitions = entry
            .reconciliation
            .scoped_limitation_transitions(&entry.contract)
            .map_err(|error| CoverageError::StatementOrEvidence {
                window_id: entry.window_id.clone(),
                error,
            })?;
        if transitions.len() != 2
            || transitions
                .iter()
                .any(|transition| !transition.validate_against(&entry.contract, &entry.reconciliation))
        {
            return Err(CoverageError::TransitionMismatch {
                window_id: entry.window_id.clone(),
            });
        }
        let transition_digests = transitions
            .iter()
            .map(|transition| transition.transition_digest)
            .collect::<Vec<_>>();
        let window_digest = window_evidence_digest(
            &entry.window_id,
            entry.contract.contract_digest,
            entry.statement.statement_digest,
            entry.reconciliation.evidence_digest,
            &transition_digests,
        );
        evidence_by_window.insert(
            entry.window_id.clone(),
            WindowCoverageEvidence {
                window_id: entry.window_id.clone(),
                contract_digest: entry.contract.contract_digest,
                statement_digest: entry.statement.statement_digest,
                reconciliation_digest: entry.reconciliation.evidence_digest,
                transition_digests,
                window_digest,
            },
        );
    }

    for window in &plan.windows {
        if !seen.contains(&window.window_id) {
            return Err(CoverageError::MissingWindow {
                window_id: window.window_id.clone(),
            });
        }
    }
    let windows = plan
        .windows
        .iter()
        .map(|window| {
            evidence_by_window
                .remove(&window.window_id)
                .expect("all planned windows were verified")
        })
        .collect::<Vec<_>>();
    let mut evidence = ControlCoverageEvidence {
        plan_digest: plan.plan_digest,
        campaign_digest: campaign.campaign_digest,
        replay_digest: replay.evidence.evidence_digest,
        windows,
        coverage_digest: Digest32([0; 32]),
    };
    evidence.coverage_digest = coverage_evidence_digest(&evidence);
    evidence.validate_against(plan)?;
    Ok(evidence)
}

fn validate_contract_semantics(
    plan: &ControlCoveragePlan,
    contract: &ControlReconciliationContract,
    window_id: &ReferenceId,
) -> Result<(), CoverageError> {
    if contract.connector != plan.connector
        || contract.source_input != plan.source_input
        || contract.metric != plan.metric
        || contract.scope != plan.scope
        || contract.unit != plan.unit
        || contract.scale != plan.scale
        || contract.aggregation != plan.aggregation
        || contract.control_source != plan.control_source
        || contract.control_source_class != plan.control_source_class
    {
        return Err(CoverageError::ContractSemanticsMismatch {
            window_id: window_id.clone(),
        });
    }
    Ok(())
}

impl ControlCoverageEvidence {
    pub fn validate_against(&self, plan: &ControlCoveragePlan) -> Result<(), CoverageError> {
        plan.validate().map_err(CoverageError::Plan)?;
        if zero_digest(&self.coverage_digest) || self.plan_digest != plan.plan_digest {
            return Err(CoverageError::ZeroDigest);
        }
        if self.windows.len() != plan.windows.len() {
            return Err(CoverageError::EntryCountMismatch {
                expected: plan.windows.len(),
                actual: self.windows.len(),
            });
        }
        for (planned, actual) in plan.windows.iter().zip(&self.windows) {
            if planned.window_id != actual.window_id || zero_digest(&actual.window_digest) {
                return Err(CoverageError::DigestMismatch);
            }
        }
        if self.coverage_digest != coverage_evidence_digest(self) {
            return Err(CoverageError::DigestMismatch);
        }
        Ok(())
    }

    /// Full coverage can support qualification-interval transitions because the preregistered
    /// windows collectively cover the entire interval without gaps or overlaps.
    pub fn qualification_limitation_transitions(
        &self,
        plan: &ControlCoveragePlan,
    ) -> Result<Vec<QualificationLimitationTransition>, CoverageError> {
        self.validate_against(plan)?;
        Ok(vec![
            QualificationLimitationTransition::new(
                upstream_export_completeness_unverified_ref(),
                control_source_external_reality_unverified_ref(),
                plan,
                self.coverage_digest,
            ),
            QualificationLimitationTransition::new(
                aggregation_semantic_authority_unverified_ref(),
                control_metric_semantic_authority_unverified_ref(),
                plan,
                self.coverage_digest,
            ),
        ])
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct QualificationLimitationTransition {
    pub from: ReferenceId,
    pub to: ReferenceId,
    pub coverage_plan_digest: Digest32,
    pub scope: ScopeRef,
    pub qualification_start_unix_ms: u64,
    pub qualification_end_unix_ms: u64,
    pub supporting_coverage_digest: Digest32,
    pub transition_digest: Digest32,
}

impl QualificationLimitationTransition {
    fn new(
        from: ReferenceId,
        to: ReferenceId,
        plan: &ControlCoveragePlan,
        supporting_coverage_digest: Digest32,
    ) -> Self {
        let mut value = Self {
            from,
            to,
            coverage_plan_digest: plan.plan_digest,
            scope: plan.scope.clone(),
            qualification_start_unix_ms: plan.qualification_start_unix_ms,
            qualification_end_unix_ms: plan.qualification_end_unix_ms,
            supporting_coverage_digest,
            transition_digest: Digest32([0; 32]),
        };
        value.transition_digest = qualification_transition_digest(&value);
        value
    }

    pub fn validate_against(
        &self,
        plan: &ControlCoveragePlan,
        evidence: &ControlCoverageEvidence,
    ) -> bool {
        self.from != self.to
            && self.coverage_plan_digest == plan.plan_digest
            && self.scope == plan.scope
            && self.qualification_start_unix_ms == plan.qualification_start_unix_ms
            && self.qualification_end_unix_ms == plan.qualification_end_unix_ms
            && self.supporting_coverage_digest == evidence.coverage_digest
            && !zero_digest(&self.supporting_coverage_digest)
            && !zero_digest(&self.transition_digest)
            && self.transition_digest == qualification_transition_digest(self)
    }
}

fn qualification_transition_digest(value: &QualificationLimitationTransition) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:qualification-limitation-transition:v1");
    hash_str(&mut hasher, value.from.as_str());
    hash_str(&mut hasher, value.to.as_str());
    hasher.update(value.coverage_plan_digest.0);
    hash_str(&mut hasher, value.scope.as_ref_id().as_str());
    hasher.update(value.qualification_start_unix_ms.to_be_bytes());
    hasher.update(value.qualification_end_unix_ms.to_be_bytes());
    hasher.update(value.supporting_coverage_digest.0);
    finish_digest(hasher)
}

fn window_evidence_digest(
    window_id: &ReferenceId,
    contract_digest: Digest32,
    statement_digest: Digest32,
    reconciliation_digest: Digest32,
    transition_digests: &[Digest32],
) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:control-window-coverage-evidence:v1");
    hash_str(&mut hasher, window_id.as_str());
    hasher.update(contract_digest.0);
    hasher.update(statement_digest.0);
    hasher.update(reconciliation_digest.0);
    for digest in transition_digests {
        hasher.update(digest.0);
    }
    finish_digest(hasher)
}

fn coverage_evidence_digest(value: &ControlCoverageEvidence) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:control-coverage-evidence:v1");
    hasher.update(value.plan_digest.0);
    hasher.update(value.campaign_digest.0);
    hasher.update(value.replay_digest.0);
    hasher.update((value.windows.len() as u64).to_be_bytes());
    for window in &value.windows {
        hasher.update(window.window_digest.0);
    }
    finish_digest(hasher)
}

fn hash_connector(hasher: &mut Sha256, value: &IngressQualificationBinding) {
    hash_str(hasher, value.source_system.as_str());
    hash_str(hasher, value.adapter_semantic_id.as_str());
    hasher.update(value.adapter_digest.0);
    hasher.update(value.mapping_digest.0);
    hasher.update(value.source_schema_digest.0);
}

#[cfg(test)]
mod tests {
    use mycelix_business_adapter_delimited::{
        DecimalPolicy, DelimitedAdapterConfig, DelimitedIngressAdapter, OutputMapping, ScopeMapping,
        TimestampEncoding, ValueMapping,
    };
    use mycelix_business_campaign_replay::replay_campaign;
    use mycelix_business_control_reconciliation::reconcile_control_total;
    use mycelix_business_import_diagnostics::diagnose_delimited_import;
    use mycelix_business_import_membership::CampaignSourceFile;

    use super::*;

    fn id(value: &str) -> ReferenceId {
        ReferenceId::new(value).unwrap()
    }

    fn adapter() -> DelimitedIngressAdapter {
        DelimitedIngressAdapter::new(DelimitedAdapterConfig {
            adapter_semantic_id: id("adapter:coverage-test:v1"),
            source_system: id("source:test-pos"),
            adapter_digest: Digest32::repeat(1),
            source_schema: id("schema:coverage-test:v1"),
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
                input: id("input:hospitality:sales-transactions:v1"),
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

    fn source_file() -> CampaignSourceFile {
        CampaignSourceFile {
            bytes: b"event_id,occurred_at,location,quantity\nevent:1,1100,a,2\nevent:2,2100,a,3\n".to_vec(),
            ingested_at_unix_ms: 3_100,
        }
    }

    fn campaign(
        adapter: &DelimitedIngressAdapter,
        file: &CampaignSourceFile,
    ) -> ExtractionCampaignManifest {
        let manifest = diagnose_delimited_import(
            adapter,
            file.bytes.as_slice(),
            file.ingested_at_unix_ms,
        )
        .unwrap();
        ExtractionCampaignManifest::from_files(vec![manifest]).unwrap()
    }

    fn plan(connector: IngressQualificationBinding) -> ControlCoveragePlan {
        ControlCoveragePlan::build(
            id("control-coverage:test:v1"),
            connector,
            id("input:hospitality:sales-transactions:v1"),
            id("metric:hospitality:item-demand"),
            ScopeRef(id("scope:location:a")),
            id("unit:count"),
            0,
            ControlAggregationKind::Sum,
            id("control-source:provider-eod-report"),
            ControlSourceClass::ProviderAuthoritativeReport,
            300,
            1_000,
            3_000,
            vec![
                ControlCoverageWindow {
                    window_id: id("control-window:1"),
                    start_unix_ms: 1_000,
                    end_unix_ms: 2_000,
                },
                ControlCoverageWindow {
                    window_id: id("control-window:2"),
                    start_unix_ms: 2_000,
                    end_unix_ms: 3_000,
                },
            ],
        )
        .unwrap()
    }

    fn entry(
        plan: &ControlCoveragePlan,
        campaign: &ExtractionCampaignManifest,
        replay: &CampaignReplay,
        window: &ControlCoverageWindow,
        expected_count: u64,
        expected_value: i128,
    ) -> ControlCoverageEntry {
        let contract = ControlReconciliationContract::build(
            id(&format!("control-contract:{}", window.window_id.as_str())),
            plan.connector.clone(),
            plan.source_input.clone(),
            plan.metric.clone(),
            plan.scope.clone(),
            plan.unit.clone(),
            plan.scale,
            plan.aggregation,
            window.start_unix_ms,
            window.end_unix_ms,
            plan.control_source.clone(),
            plan.control_source_class,
            200,
        )
        .unwrap();
        let statement = ControlTotalStatement::build(
            &contract,
            id(&format!("control-statement:{}", window.window_id.as_str())),
            Digest32::repeat(if expected_value == 2 { 8 } else { 9 }),
            window.end_unix_ms + 10,
            expected_count,
            ScaledValue {
                mantissa: expected_value,
                scale: plan.scale,
                unit: plan.unit.clone(),
            },
        )
        .unwrap();
        let reconciliation = reconcile_control_total(campaign, replay, &contract, &statement).unwrap();
        ControlCoverageEntry {
            window_id: window.window_id.clone(),
            contract,
            statement,
            reconciliation,
        }
    }

    fn fixture() -> (
        ControlCoveragePlan,
        ExtractionCampaignManifest,
        CampaignReplay,
        Vec<ControlCoverageEntry>,
    ) {
        let adapter = adapter();
        let file = source_file();
        let campaign = campaign(&adapter, &file);
        let replay = replay_campaign(&adapter, &campaign, std::slice::from_ref(&file)).unwrap();
        let plan = plan(campaign.connector.clone());
        let entries = vec![
            entry(&plan, &campaign, &replay, &plan.windows[0], 1, 2),
            entry(&plan, &campaign, &replay, &plan.windows[1], 1, 3),
        ];
        (plan, campaign, replay, entries)
    }

    #[test]
    fn contiguous_matched_windows_produce_full_coverage_evidence() {
        let (plan, campaign, replay, entries) = fixture();
        let evidence = verify_control_coverage(&plan, &campaign, &replay, &entries).unwrap();
        assert!(CONTROL_COVERAGE_IS_READ_ONLY);
        assert_eq!(evidence.windows.len(), 2);
        assert!(evidence.validate_against(&plan).is_ok());
        let transitions = evidence.qualification_limitation_transitions(&plan).unwrap();
        assert_eq!(transitions.len(), 2);
        assert!(transitions
            .iter()
            .all(|transition| transition.validate_against(&plan, &evidence)));
        assert!(transitions.iter().all(|transition| {
            transition.qualification_start_unix_ms == 1_000
                && transition.qualification_end_unix_ms == 3_000
        }));
    }

    #[test]
    fn plan_with_gap_fails_before_evidence_collection() {
        let adapter = adapter();
        let file = source_file();
        let campaign = campaign(&adapter, &file);
        let result = ControlCoveragePlan::build(
            id("control-coverage:gap"),
            campaign.connector,
            id("input:hospitality:sales-transactions:v1"),
            id("metric:hospitality:item-demand"),
            ScopeRef(id("scope:location:a")),
            id("unit:count"),
            0,
            ControlAggregationKind::Sum,
            id("control-source:provider-eod-report"),
            ControlSourceClass::ProviderAuthoritativeReport,
            300,
            1_000,
            3_000,
            vec![
                ControlCoverageWindow {
                    window_id: id("w:1"),
                    start_unix_ms: 1_000,
                    end_unix_ms: 1_900,
                },
                ControlCoverageWindow {
                    window_id: id("w:2"),
                    start_unix_ms: 2_000,
                    end_unix_ms: 3_000,
                },
            ],
        );
        assert!(matches!(result, Err(CoveragePlanError::GapOrOverlap { .. })));
    }

    #[test]
    fn missing_window_cannot_be_ignored() {
        let (plan, campaign, replay, mut entries) = fixture();
        entries.pop();
        assert!(matches!(
            verify_control_coverage(&plan, &campaign, &replay, &entries),
            Err(CoverageError::EntryCountMismatch { .. })
        ));
    }

    #[test]
    fn one_mismatched_control_window_blocks_full_coverage() {
        let (plan, campaign, replay, mut entries) = fixture();
        let window = &plan.windows[1];
        entries[1] = entry(&plan, &campaign, &replay, window, 2, 3);
        assert!(matches!(
            verify_control_coverage(&plan, &campaign, &replay, &entries),
            Err(CoverageError::ReconciliationDidNotMatch { .. })
        ));
    }

    #[test]
    fn contract_semantics_cannot_drift_between_windows() {
        let (plan, campaign, replay, mut entries) = fixture();
        entries[1].contract.metric = id("metric:other");
        assert!(matches!(
            verify_control_coverage(&plan, &campaign, &replay, &entries),
            Err(CoverageError::ContractInvalid { .. })
                | Err(CoverageError::ContractSemanticsMismatch { .. })
                | Err(CoverageError::StatementOrEvidence { .. })
        ));
    }

    #[test]
    fn contracts_must_be_frozen_no_later_than_coverage_plan() {
        let (plan, campaign, replay, mut entries) = fixture();
        let window = &plan.windows[1];
        let late_contract = ControlReconciliationContract::build(
            id("control-contract:late"),
            plan.connector.clone(),
            plan.source_input.clone(),
            plan.metric.clone(),
            plan.scope.clone(),
            plan.unit.clone(),
            plan.scale,
            plan.aggregation,
            window.start_unix_ms,
            window.end_unix_ms,
            plan.control_source.clone(),
            plan.control_source_class,
            plan.registered_at_unix_ms + 1,
        )
        .unwrap();
        let late_statement = ControlTotalStatement::build(
            &late_contract,
            id("control-statement:late"),
            Digest32::repeat(9),
            window.end_unix_ms + 10,
            1,
            ScaledValue {
                mantissa: 3,
                scale: plan.scale,
                unit: plan.unit.clone(),
            },
        )
        .unwrap();
        let late_reconciliation =
            mycelix_business_control_reconciliation::reconcile_control_total(
                &campaign,
                &replay,
                &late_contract,
                &late_statement,
            )
            .unwrap();
        entries[1] = ControlCoverageEntry {
            window_id: window.window_id.clone(),
            contract: late_contract,
            statement: late_statement,
            reconciliation: late_reconciliation,
        };
        assert!(matches!(
            verify_control_coverage(&plan, &campaign, &replay, &entries),
            Err(CoverageError::ContractRegisteredAfterPlan { .. })
        ));
    }
}
