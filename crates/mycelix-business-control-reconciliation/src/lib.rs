// Copyright (C) 2024-2026 Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Read-only reconciliation between canonical Business replay and an independently issued control
//! total statement.
//!
//! A match proves consistency with the supplied control statement for one exact scope and time
//! window. It does not prove that the statement is authentic, legally authoritative, semantically
//! correct, or identical to physical reality. A single scoped match must never be interpreted as
//! proof for a larger qualification interval.

use std::collections::BTreeSet;

use mycelix_business_campaign_replay::{CampaignReplay, ReplayedObservation, ReplayError};
use mycelix_business_core::{Digest32, ReferenceId, ScopeRef};
use mycelix_business_import_diagnostics::ExtractionCampaignManifest;
use mycelix_business_ingress::IngressQualificationBinding;
use mycelix_business_shadow::{MetricObservation, ScaledValue};
use sha2::{Digest, Sha256};

pub const CONTROL_RECONCILIATION_IS_READ_ONLY: bool = true;

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

pub fn upstream_export_completeness_unverified_ref() -> ReferenceId {
    ReferenceId::new("limitation:upstream-export-completeness-unverified:v1")
        .expect("static limitation id is canonical")
}

pub fn aggregation_semantic_authority_unverified_ref() -> ReferenceId {
    ReferenceId::new("limitation:aggregation-semantic-authority-unverified:v1")
        .expect("static limitation id is canonical")
}

pub fn control_source_authenticity_unverified_ref() -> ReferenceId {
    ReferenceId::new("limitation:control-source-authenticity-unverified:v1")
        .expect("static limitation id is canonical")
}

pub fn control_source_external_reality_unverified_ref() -> ReferenceId {
    ReferenceId::new("limitation:control-source-external-reality-unverified:v1")
        .expect("static limitation id is canonical")
}

pub fn control_metric_semantic_authority_unverified_ref() -> ReferenceId {
    ReferenceId::new("limitation:control-metric-semantic-authority-unverified:v1")
        .expect("static limitation id is canonical")
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ControlSourceClass {
    ProviderAuthoritativeReport,
    IndependentAudit,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ControlAggregationKind {
    Sum,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ControlReconciliationContract {
    pub contract_id: ReferenceId,
    pub connector: IngressQualificationBinding,
    pub source_input: ReferenceId,
    pub metric: ReferenceId,
    pub scope: ScopeRef,
    pub unit: ReferenceId,
    pub scale: u32,
    pub aggregation: ControlAggregationKind,
    pub window_start_unix_ms: u64,
    pub window_end_unix_ms: u64,
    pub control_source: ReferenceId,
    pub control_source_class: ControlSourceClass,
    pub registered_at_unix_ms: u64,
    pub contract_digest: Digest32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ContractError {
    ZeroConnectorDigest,
    InvalidWindow,
    NotPreregistered,
    ExcessiveScale,
    ZeroDigest,
    DigestMismatch,
}

impl ControlReconciliationContract {
    #[allow(clippy::too_many_arguments)]
    pub fn build(
        contract_id: ReferenceId,
        connector: IngressQualificationBinding,
        source_input: ReferenceId,
        metric: ReferenceId,
        scope: ScopeRef,
        unit: ReferenceId,
        scale: u32,
        aggregation: ControlAggregationKind,
        window_start_unix_ms: u64,
        window_end_unix_ms: u64,
        control_source: ReferenceId,
        control_source_class: ControlSourceClass,
        registered_at_unix_ms: u64,
    ) -> Result<Self, ContractError> {
        let mut value = Self {
            contract_id,
            connector,
            source_input,
            metric,
            scope,
            unit,
            scale,
            aggregation,
            window_start_unix_ms,
            window_end_unix_ms,
            control_source,
            control_source_class,
            registered_at_unix_ms,
            contract_digest: Digest32([0; 32]),
        };
        value.contract_digest = contract_digest(&value);
        value.validate()?;
        Ok(value)
    }

    pub fn validate(&self) -> Result<(), ContractError> {
        if zero_digest(&self.connector.adapter_digest)
            || zero_digest(&self.connector.mapping_digest)
            || zero_digest(&self.connector.source_schema_digest)
        {
            return Err(ContractError::ZeroConnectorDigest);
        }
        if self.window_start_unix_ms == 0 || self.window_start_unix_ms >= self.window_end_unix_ms {
            return Err(ContractError::InvalidWindow);
        }
        if self.registered_at_unix_ms == 0 || self.registered_at_unix_ms >= self.window_start_unix_ms {
            return Err(ContractError::NotPreregistered);
        }
        if self.scale > ScaledValue::MAX_SCALE {
            return Err(ContractError::ExcessiveScale);
        }
        if zero_digest(&self.contract_digest) {
            return Err(ContractError::ZeroDigest);
        }
        if self.contract_digest != contract_digest(self) {
            return Err(ContractError::DigestMismatch);
        }
        Ok(())
    }
}

fn contract_digest(value: &ControlReconciliationContract) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:control-reconciliation-contract:v1");
    hash_str(&mut hasher, value.contract_id.as_str());
    hash_connector(&mut hasher, &value.connector);
    hash_str(&mut hasher, value.source_input.as_str());
    hash_str(&mut hasher, value.metric.as_str());
    hash_str(&mut hasher, value.scope.as_ref_id().as_str());
    hash_str(&mut hasher, value.unit.as_str());
    hasher.update(value.scale.to_be_bytes());
    hasher.update([match value.aggregation {
        ControlAggregationKind::Sum => 1,
    }]);
    hasher.update(value.window_start_unix_ms.to_be_bytes());
    hasher.update(value.window_end_unix_ms.to_be_bytes());
    hash_str(&mut hasher, value.control_source.as_str());
    hasher.update([match value.control_source_class {
        ControlSourceClass::ProviderAuthoritativeReport => 1,
        ControlSourceClass::IndependentAudit => 2,
    }]);
    hasher.update(value.registered_at_unix_ms.to_be_bytes());
    finish_digest(hasher)
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ControlTotalStatement {
    pub statement_id: ReferenceId,
    pub contract_digest: Digest32,
    pub control_source: ReferenceId,
    pub source_document_digest: Digest32,
    pub issued_at_unix_ms: u64,
    pub expected_source_events: u64,
    pub expected_value: ScaledValue,
    pub statement_digest: Digest32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum StatementError {
    InvalidContract(ContractError),
    ContractMismatch,
    ControlSourceMismatch,
    ZeroDocumentDigest,
    IssuedBeforeWindowClosed,
    InvalidValue,
    ZeroDigest,
    DigestMismatch,
}

impl ControlTotalStatement {
    pub fn build(
        contract: &ControlReconciliationContract,
        statement_id: ReferenceId,
        source_document_digest: Digest32,
        issued_at_unix_ms: u64,
        expected_source_events: u64,
        expected_value: ScaledValue,
    ) -> Result<Self, StatementError> {
        contract.validate().map_err(StatementError::InvalidContract)?;
        let mut value = Self {
            statement_id,
            contract_digest: contract.contract_digest,
            control_source: contract.control_source.clone(),
            source_document_digest,
            issued_at_unix_ms,
            expected_source_events,
            expected_value,
            statement_digest: Digest32([0; 32]),
        };
        value.statement_digest = statement_digest(&value);
        value.validate_against(contract)?;
        Ok(value)
    }

    pub fn validate_against(
        &self,
        contract: &ControlReconciliationContract,
    ) -> Result<(), StatementError> {
        contract.validate().map_err(StatementError::InvalidContract)?;
        if self.contract_digest != contract.contract_digest {
            return Err(StatementError::ContractMismatch);
        }
        if self.control_source != contract.control_source {
            return Err(StatementError::ControlSourceMismatch);
        }
        if zero_digest(&self.source_document_digest) {
            return Err(StatementError::ZeroDocumentDigest);
        }
        if self.issued_at_unix_ms < contract.window_end_unix_ms {
            return Err(StatementError::IssuedBeforeWindowClosed);
        }
        self.expected_value
            .validate()
            .map_err(|_| StatementError::InvalidValue)?;
        if self.expected_value.unit != contract.unit || self.expected_value.scale != contract.scale {
            return Err(StatementError::InvalidValue);
        }
        if zero_digest(&self.statement_digest) {
            return Err(StatementError::ZeroDigest);
        }
        if self.statement_digest != statement_digest(self) {
            return Err(StatementError::DigestMismatch);
        }
        Ok(())
    }
}

fn statement_digest(value: &ControlTotalStatement) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:control-total-statement:v1");
    hash_str(&mut hasher, value.statement_id.as_str());
    hasher.update(value.contract_digest.0);
    hash_str(&mut hasher, value.control_source.as_str());
    hasher.update(value.source_document_digest.0);
    hasher.update(value.issued_at_unix_ms.to_be_bytes());
    hasher.update(value.expected_source_events.to_be_bytes());
    hash_scaled(&mut hasher, &value.expected_value);
    finish_digest(hasher)
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ControlReconciliationDecision {
    Match,
    CountMismatch { expected: u64, actual: u64 },
    ValueMismatch { expected: ScaledValue, actual: ScaledValue },
    CountAndValueMismatch {
        expected_count: u64,
        actual_count: u64,
        expected_value: ScaledValue,
        actual_value: ScaledValue,
    },
}

impl ControlReconciliationDecision {
    pub fn is_match(&self) -> bool {
        matches!(self, Self::Match)
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ControlReconciliationEvidence {
    pub contract_digest: Digest32,
    pub statement_digest: Digest32,
    pub campaign_digest: Digest32,
    pub replay_digest: Digest32,
    pub source_observation_count: u64,
    pub source_event_count: u64,
    pub source_observation_set_digest: Digest32,
    pub actual_value: ScaledValue,
    pub decision: ControlReconciliationDecision,
    pub evidence_digest: Digest32,
}

#[derive(Debug)]
pub enum ReconciliationError {
    Contract(ContractError),
    Statement(StatementError),
    ReplayEvidence(ReplayError),
    ConnectorMismatch,
    SourceInputNotDeclared { input: ReferenceId },
    ReplayObservationCountMismatch,
    ReplayObservationSetMismatch,
    InvalidObservation,
    DuplicateObservation,
    IncompatibleObservation,
    ArithmeticOverflow,
    CountOverflow,
    DigestMismatch,
}

pub fn reconcile_control_total(
    campaign: &ExtractionCampaignManifest,
    replay: &CampaignReplay,
    contract: &ControlReconciliationContract,
    statement: &ControlTotalStatement,
) -> Result<ControlReconciliationEvidence, ReconciliationError> {
    contract.validate().map_err(ReconciliationError::Contract)?;
    statement
        .validate_against(contract)
        .map_err(ReconciliationError::Statement)?;
    validate_replay(campaign, replay)?;
    if replay.evidence.connector != contract.connector || campaign.connector != contract.connector {
        return Err(ReconciliationError::ConnectorMismatch);
    }
    if !campaign.supported_inputs.contains(&contract.source_input) {
        return Err(ReconciliationError::SourceInputNotDeclared {
            input: contract.source_input.clone(),
        });
    }

    let mut observation_ids = BTreeSet::new();
    let mut event_ids = BTreeSet::new();
    let mut source_digests = Vec::new();
    let mut sum = 0_i128;

    for replayed in &replay.observations {
        replayed
            .observation
            .validate()
            .map_err(|_| ReconciliationError::InvalidObservation)?;
        if replayed.input != contract.source_input
            || replayed.observation.metric != contract.metric
            || replayed.observation.scope != contract.scope
            || replayed.observation.observed_at_unix_ms < contract.window_start_unix_ms
            || replayed.observation.observed_at_unix_ms >= contract.window_end_unix_ms
        {
            continue;
        }
        if replayed.observation.value.unit != contract.unit
            || replayed.observation.value.scale != contract.scale
        {
            return Err(ReconciliationError::IncompatibleObservation);
        }
        if !observation_ids.insert(replayed.observation.observation.clone()) {
            return Err(ReconciliationError::DuplicateObservation);
        }
        event_ids.insert(replayed.observation.source_event_id.clone());
        match contract.aggregation {
            ControlAggregationKind::Sum => {
                sum = sum
                    .checked_add(replayed.observation.value.mantissa)
                    .ok_or(ReconciliationError::ArithmeticOverflow)?;
            }
        }
        source_digests.push(replayed_observation_digest(replayed));
    }

    source_digests.sort_unstable();
    let source_observation_count =
        u64::try_from(source_digests.len()).map_err(|_| ReconciliationError::CountOverflow)?;
    let source_event_count =
        u64::try_from(event_ids.len()).map_err(|_| ReconciliationError::CountOverflow)?;
    let source_observation_set_digest = source_set_digest(&source_digests);
    let actual_value = ScaledValue {
        mantissa: sum,
        scale: contract.scale,
        unit: contract.unit.clone(),
    };
    let count_matches = source_event_count == statement.expected_source_events;
    let value_matches = actual_value == statement.expected_value;
    let decision = match (count_matches, value_matches) {
        (true, true) => ControlReconciliationDecision::Match,
        (false, true) => ControlReconciliationDecision::CountMismatch {
            expected: statement.expected_source_events,
            actual: source_event_count,
        },
        (true, false) => ControlReconciliationDecision::ValueMismatch {
            expected: statement.expected_value.clone(),
            actual: actual_value.clone(),
        },
        (false, false) => ControlReconciliationDecision::CountAndValueMismatch {
            expected_count: statement.expected_source_events,
            actual_count: source_event_count,
            expected_value: statement.expected_value.clone(),
            actual_value: actual_value.clone(),
        },
    };

    let mut evidence = ControlReconciliationEvidence {
        contract_digest: contract.contract_digest,
        statement_digest: statement.statement_digest,
        campaign_digest: campaign.campaign_digest,
        replay_digest: replay.evidence.evidence_digest,
        source_observation_count,
        source_event_count,
        source_observation_set_digest,
        actual_value,
        decision,
        evidence_digest: Digest32([0; 32]),
    };
    evidence.evidence_digest = reconciliation_evidence_digest(&evidence);
    evidence.validate_against(campaign, replay, contract, statement)?;
    Ok(evidence)
}

impl ControlReconciliationEvidence {
    pub fn validate_against(
        &self,
        campaign: &ExtractionCampaignManifest,
        replay: &CampaignReplay,
        contract: &ControlReconciliationContract,
        statement: &ControlTotalStatement,
    ) -> Result<(), ReconciliationError> {
        contract.validate().map_err(ReconciliationError::Contract)?;
        statement
            .validate_against(contract)
            .map_err(ReconciliationError::Statement)?;
        validate_replay(campaign, replay)?;
        if !campaign.supported_inputs.contains(&contract.source_input) {
            return Err(ReconciliationError::SourceInputNotDeclared {
                input: contract.source_input.clone(),
            });
        }
        if self.contract_digest != contract.contract_digest
            || self.statement_digest != statement.statement_digest
            || self.campaign_digest != campaign.campaign_digest
            || self.replay_digest != replay.evidence.evidence_digest
            || zero_digest(&self.source_observation_set_digest)
            || zero_digest(&self.evidence_digest)
            || self.evidence_digest != reconciliation_evidence_digest(self)
        {
            return Err(ReconciliationError::DigestMismatch);
        }
        Ok(())
    }

    /// Emit only scope/window-bound limitation transitions. A later coverage theorem is required
    /// before these may narrow a limitation attached to a larger qualification interval.
    pub fn scoped_limitation_transitions(
        &self,
        contract: &ControlReconciliationContract,
    ) -> Result<Vec<ScopedLimitationTransition>, ReconciliationError> {
        contract.validate().map_err(ReconciliationError::Contract)?;
        if self.contract_digest != contract.contract_digest {
            return Err(ReconciliationError::DigestMismatch);
        }
        if !self.decision.is_match() {
            return Ok(Vec::new());
        }
        Ok(vec![
            ScopedLimitationTransition::new(
                upstream_export_completeness_unverified_ref(),
                control_source_external_reality_unverified_ref(),
                contract,
                self.evidence_digest,
            ),
            ScopedLimitationTransition::new(
                aggregation_semantic_authority_unverified_ref(),
                control_metric_semantic_authority_unverified_ref(),
                contract,
                self.evidence_digest,
            ),
        ])
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ScopedLimitationTransition {
    pub from: ReferenceId,
    pub to: ReferenceId,
    pub contract_digest: Digest32,
    pub scope: ScopeRef,
    pub window_start_unix_ms: u64,
    pub window_end_unix_ms: u64,
    pub supporting_evidence_digest: Digest32,
    pub transition_digest: Digest32,
}

impl ScopedLimitationTransition {
    fn new(
        from: ReferenceId,
        to: ReferenceId,
        contract: &ControlReconciliationContract,
        supporting_evidence_digest: Digest32,
    ) -> Self {
        let mut value = Self {
            from,
            to,
            contract_digest: contract.contract_digest,
            scope: contract.scope.clone(),
            window_start_unix_ms: contract.window_start_unix_ms,
            window_end_unix_ms: contract.window_end_unix_ms,
            supporting_evidence_digest,
            transition_digest: Digest32([0; 32]),
        };
        value.transition_digest = scoped_transition_digest(&value);
        value
    }

    pub fn validate_against(
        &self,
        contract: &ControlReconciliationContract,
        evidence: &ControlReconciliationEvidence,
    ) -> bool {
        self.from != self.to
            && self.contract_digest == contract.contract_digest
            && self.scope == contract.scope
            && self.window_start_unix_ms == contract.window_start_unix_ms
            && self.window_end_unix_ms == contract.window_end_unix_ms
            && self.window_start_unix_ms < self.window_end_unix_ms
            && self.supporting_evidence_digest == evidence.evidence_digest
            && !zero_digest(&self.supporting_evidence_digest)
            && !zero_digest(&self.transition_digest)
            && self.transition_digest == scoped_transition_digest(self)
    }
}

fn scoped_transition_digest(value: &ScopedLimitationTransition) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:scoped-limitation-transition:v1");
    hash_str(&mut hasher, value.from.as_str());
    hash_str(&mut hasher, value.to.as_str());
    hasher.update(value.contract_digest.0);
    hash_str(&mut hasher, value.scope.as_ref_id().as_str());
    hasher.update(value.window_start_unix_ms.to_be_bytes());
    hasher.update(value.window_end_unix_ms.to_be_bytes());
    hasher.update(value.supporting_evidence_digest.0);
    finish_digest(hasher)
}

fn validate_replay(
    campaign: &ExtractionCampaignManifest,
    replay: &CampaignReplay,
) -> Result<(), ReconciliationError> {
    replay
        .evidence
        .validate_against(campaign)
        .map_err(ReconciliationError::ReplayEvidence)?;
    let normalized = u64::try_from(replay.observations.len())
        .map_err(|_| ReconciliationError::CountOverflow)?;
    if normalized != replay.evidence.normalized_observations {
        return Err(ReconciliationError::ReplayObservationCountMismatch);
    }
    if replay_observation_set_digest(&replay.observations) != replay.evidence.observation_set_digest {
        return Err(ReconciliationError::ReplayObservationSetMismatch);
    }
    Ok(())
}

fn replay_observation_set_digest(observations: &[ReplayedObservation]) -> Digest32 {
    let mut digests = observations
        .iter()
        .map(replayed_observation_digest)
        .collect::<Vec<_>>();
    digests.sort_unstable();
    digest_set_with_label("mycelix:campaign-replay-observation-set:v1", &digests)
}

fn replayed_observation_digest(value: &ReplayedObservation) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:campaign-replayed-observation:v1");
    hash_str(&mut hasher, value.input.as_str());
    hash_metric_observation(&mut hasher, &value.observation);
    finish_digest(hasher)
}

fn hash_metric_observation(hasher: &mut Sha256, observation: &MetricObservation) {
    hash_str(hasher, observation.observation.as_ref_id().as_str());
    hash_str(hasher, observation.source_system.as_str());
    hash_str(hasher, observation.source_event_id.as_str());
    hasher.update(observation.source_payload_digest.0);
    hasher.update(observation.mapping_digest.0);
    hash_str(hasher, observation.metric.as_str());
    hash_str(hasher, observation.scope.as_ref_id().as_str());
    hash_scaled(hasher, &observation.value);
    hasher.update(observation.observed_at_unix_ms.to_be_bytes());
}

fn source_set_digest(digests: &[Digest32]) -> Digest32 {
    digest_set_with_label("mycelix:control-reconciliation-source-set:v1", digests)
}

fn digest_set_with_label(label: &str, digests: &[Digest32]) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, label);
    hasher.update((digests.len() as u64).to_be_bytes());
    for digest in digests {
        hasher.update(digest.0);
    }
    finish_digest(hasher)
}

fn reconciliation_evidence_digest(value: &ControlReconciliationEvidence) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:control-reconciliation-evidence:v1");
    hasher.update(value.contract_digest.0);
    hasher.update(value.statement_digest.0);
    hasher.update(value.campaign_digest.0);
    hasher.update(value.replay_digest.0);
    hasher.update(value.source_observation_count.to_be_bytes());
    hasher.update(value.source_event_count.to_be_bytes());
    hasher.update(value.source_observation_set_digest.0);
    hash_scaled(&mut hasher, &value.actual_value);
    hash_decision(&mut hasher, &value.decision);
    finish_digest(hasher)
}

fn hash_decision(hasher: &mut Sha256, decision: &ControlReconciliationDecision) {
    match decision {
        ControlReconciliationDecision::Match => hasher.update([1]),
        ControlReconciliationDecision::CountMismatch { expected, actual } => {
            hasher.update([2]);
            hasher.update(expected.to_be_bytes());
            hasher.update(actual.to_be_bytes());
        }
        ControlReconciliationDecision::ValueMismatch { expected, actual } => {
            hasher.update([3]);
            hash_scaled(hasher, expected);
            hash_scaled(hasher, actual);
        }
        ControlReconciliationDecision::CountAndValueMismatch {
            expected_count,
            actual_count,
            expected_value,
            actual_value,
        } => {
            hasher.update([4]);
            hasher.update(expected_count.to_be_bytes());
            hasher.update(actual_count.to_be_bytes());
            hash_scaled(hasher, expected_value);
            hash_scaled(hasher, actual_value);
        }
    }
}

fn hash_connector(hasher: &mut Sha256, value: &IngressQualificationBinding) {
    hash_str(hasher, value.source_system.as_str());
    hash_str(hasher, value.adapter_semantic_id.as_str());
    hasher.update(value.adapter_digest.0);
    hasher.update(value.mapping_digest.0);
    hasher.update(value.source_schema_digest.0);
}

fn hash_scaled(hasher: &mut Sha256, value: &ScaledValue) {
    hasher.update(value.mantissa.to_be_bytes());
    hasher.update(value.scale.to_be_bytes());
    hash_str(hasher, value.unit.as_str());
}

#[cfg(test)]
mod tests {
    use mycelix_business_adapter_delimited::{
        DecimalPolicy, DelimitedAdapterConfig, DelimitedIngressAdapter, OutputMapping, ScopeMapping,
        TimestampEncoding, ValueMapping,
    };
    use mycelix_business_campaign_replay::replay_campaign;
    use mycelix_business_import_diagnostics::diagnose_delimited_import;
    use mycelix_business_import_membership::CampaignSourceFile;

    use super::*;

    fn id(value: &str) -> ReferenceId {
        ReferenceId::new(value).unwrap()
    }

    fn adapter() -> DelimitedIngressAdapter {
        DelimitedIngressAdapter::new(DelimitedAdapterConfig {
            adapter_semantic_id: id("adapter:control-test:v1"),
            source_system: id("source:test-pos"),
            adapter_digest: Digest32::repeat(1),
            source_schema: id("schema:control-test:v1"),
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
            bytes: b"event_id,occurred_at,location,quantity\nevent:1,1100,a,2\nevent:2,1200,a,3\nevent:3,2500,a,99\n".to_vec(),
            ingested_at_unix_ms: 3_000,
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

    fn contract(connector: IngressQualificationBinding) -> ControlReconciliationContract {
        ControlReconciliationContract::build(
            id("control-contract:test:v1"),
            connector,
            id("input:hospitality:sales-transactions:v1"),
            id("metric:hospitality:item-demand"),
            ScopeRef(id("scope:location:a")),
            id("unit:count"),
            0,
            ControlAggregationKind::Sum,
            1_000,
            2_000,
            id("control-source:provider-eod-report"),
            ControlSourceClass::ProviderAuthoritativeReport,
            500,
        )
        .unwrap()
    }

    fn statement(
        contract: &ControlReconciliationContract,
        expected_count: u64,
        expected_value: i128,
    ) -> ControlTotalStatement {
        ControlTotalStatement::build(
            contract,
            id("control-statement:test:1"),
            Digest32::repeat(8),
            2_100,
            expected_count,
            ScaledValue {
                mantissa: expected_value,
                scale: 0,
                unit: id("unit:count"),
            },
        )
        .unwrap()
    }

    fn matched_fixture() -> (
        ExtractionCampaignManifest,
        CampaignReplay,
        ControlReconciliationContract,
        ControlTotalStatement,
        ControlReconciliationEvidence,
    ) {
        let adapter = adapter();
        let file = source_file();
        let campaign = campaign(&adapter, &file);
        let replay = replay_campaign(&adapter, &campaign, std::slice::from_ref(&file)).unwrap();
        let contract = contract(campaign.connector.clone());
        let statement = statement(&contract, 2, 5);
        let evidence = reconcile_control_total(&campaign, &replay, &contract, &statement).unwrap();
        (campaign, replay, contract, statement, evidence)
    }

    #[test]
    fn exact_match_emits_only_scoped_transitions() {
        let (_campaign, _replay, contract, _statement, evidence) = matched_fixture();
        assert!(CONTROL_RECONCILIATION_IS_READ_ONLY);
        assert_eq!(evidence.decision, ControlReconciliationDecision::Match);
        assert_eq!(evidence.source_event_count, 2);
        assert_eq!(evidence.actual_value.mantissa, 5);
        let transitions = evidence.scoped_limitation_transitions(&contract).unwrap();
        assert_eq!(transitions.len(), 2);
        assert!(transitions
            .iter()
            .all(|transition| transition.validate_against(&contract, &evidence)));
        assert!(transitions.iter().all(|transition| {
            transition.scope == contract.scope
                && transition.window_start_unix_ms == 1_000
                && transition.window_end_unix_ms == 2_000
        }));
    }

    #[test]
    fn count_mismatch_is_preserved_and_emits_no_transition() {
        let adapter = adapter();
        let file = source_file();
        let campaign = campaign(&adapter, &file);
        let replay = replay_campaign(&adapter, &campaign, std::slice::from_ref(&file)).unwrap();
        let contract = contract(campaign.connector.clone());
        let statement = statement(&contract, 3, 5);
        let evidence = reconcile_control_total(&campaign, &replay, &contract, &statement).unwrap();
        assert!(matches!(
            evidence.decision,
            ControlReconciliationDecision::CountMismatch { expected: 3, actual: 2 }
        ));
        assert!(evidence
            .scoped_limitation_transitions(&contract)
            .unwrap()
            .is_empty());
    }

    #[test]
    fn count_and_value_mismatch_is_preserved() {
        let adapter = adapter();
        let file = source_file();
        let campaign = campaign(&adapter, &file);
        let replay = replay_campaign(&adapter, &campaign, std::slice::from_ref(&file)).unwrap();
        let contract = contract(campaign.connector.clone());
        let statement = statement(&contract, 3, 6);
        let evidence = reconcile_control_total(&campaign, &replay, &contract, &statement).unwrap();
        assert!(matches!(
            evidence.decision,
            ControlReconciliationDecision::CountAndValueMismatch {
                expected_count: 3,
                actual_count: 2,
                ..
            }
        ));
    }

    #[test]
    fn undeclared_control_input_cannot_match_zero() {
        let adapter = adapter();
        let file = source_file();
        let campaign = campaign(&adapter, &file);
        let replay = replay_campaign(&adapter, &campaign, std::slice::from_ref(&file)).unwrap();
        let mut contract = contract(campaign.connector.clone());
        contract.source_input = id("input:undeclared:v1");
        contract.contract_digest = contract_digest(&contract);
        let statement = statement(&contract, 0, 0);
        assert!(matches!(
            reconcile_control_total(&campaign, &replay, &contract, &statement),
            Err(ReconciliationError::SourceInputNotDeclared { .. })
        ));
    }

    #[test]
    fn mutated_replay_observation_fails_self_validation() {
        let adapter = adapter();
        let file = source_file();
        let campaign = campaign(&adapter, &file);
        let mut replay = replay_campaign(&adapter, &campaign, std::slice::from_ref(&file)).unwrap();
        replay.observations[0].observation.value.mantissa += 1;
        let contract = contract(campaign.connector.clone());
        let statement = statement(&contract, 2, 5);
        assert!(matches!(
            reconcile_control_total(&campaign, &replay, &contract, &statement),
            Err(ReconciliationError::ReplayObservationSetMismatch)
        ));
    }

    #[test]
    fn statement_cannot_be_issued_before_control_window_closes() {
        let adapter = adapter();
        let file = source_file();
        let campaign = campaign(&adapter, &file);
        let contract = contract(campaign.connector.clone());
        let result = ControlTotalStatement::build(
            &contract,
            id("control-statement:early"),
            Digest32::repeat(8),
            1_999,
            2,
            ScaledValue {
                mantissa: 5,
                scale: 0,
                unit: id("unit:count"),
            },
        );
        assert!(matches!(result, Err(StatementError::IssuedBeforeWindowClosed)));
    }

    #[test]
    fn transition_scope_or_window_mutation_invalidates_evidence() {
        let (_campaign, _replay, contract, _statement, evidence) = matched_fixture();
        let mut transition = evidence.scoped_limitation_transitions(&contract).unwrap()[0].clone();
        transition.window_end_unix_ms += 1;
        assert!(!transition.validate_against(&contract, &evidence));
    }
}
