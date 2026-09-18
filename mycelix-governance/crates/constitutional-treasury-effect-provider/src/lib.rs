use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use thiserror::Error;

pub const TREASURY_EFFECT_SCHEMA_VERSION: u16 = 1;
pub const MAX_ID_LEN: usize = 256;
pub const MAX_REF_LEN: usize = 512;
pub const REQUEST_COMMITMENT_PREFIX: &str = "treasury-effect-request-v1:";
pub const EXECUTION_ID_PREFIX: &str = "treasury-effect-execution-v1:";
pub const INTENT_COMMITMENT_PREFIX: &str = "treasury-effect-intent-v1:";

const REQUEST_DOMAIN: &[u8] = b"MYCELIX-CONSTITUTIONAL-TREASURY-EFFECT-REQUEST\0V1\0";
const EXECUTION_DOMAIN: &[u8] = b"MYCELIX-CONSTITUTIONAL-TREASURY-EXECUTION-ID\0V1\0";
const INTENT_DOMAIN: &[u8] = b"MYCELIX-CONSTITUTIONAL-TREASURY-EFFECT-INTENT\0V1\0";

#[derive(Debug, Error, Clone, PartialEq, Eq)]
pub enum TreasuryEffectError {
    #[error("{0}")]
    Violation(String),
    #[error("treasury effect integrity halted: {0}")]
    IntegrityHalted(String),
}

pub type TreasuryEffectResult<T> = Result<T, TreasuryEffectError>;

fn violation(message: impl Into<String>) -> TreasuryEffectError {
    TreasuryEffectError::Violation(message.into())
}

fn require_opaque(label: &str, value: &str, max_len: usize) -> TreasuryEffectResult<()> {
    if value.trim().is_empty() || value.len() > max_len {
        return Err(violation(format!(
            "{label} must be non-empty and <= {max_len} bytes"
        )));
    }
    Ok(())
}

fn require_did(label: &str, value: &str) -> TreasuryEffectResult<()> {
    require_opaque(label, value, MAX_ID_LEN)?;
    if !value.starts_with("did:") {
        return Err(violation(format!("{label} must be a DID")));
    }
    Ok(())
}

fn require_tagged_hash(label: &str, value: &str, prefix: &str) -> TreasuryEffectResult<()> {
    let digest = value
        .strip_prefix(prefix)
        .ok_or_else(|| violation(format!("{label} must use {prefix}<hex>")))?;
    if digest.len() != 64
        || !digest
            .bytes()
            .all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b))
    {
        return Err(violation(format!(
            "{label} must contain exactly 64 lowercase hexadecimal digits"
        )));
    }
    Ok(())
}

fn push_str(hasher: &mut blake3::Hasher, value: &str) {
    let bytes = value.as_bytes();
    hasher.update(&(bytes.len() as u64).to_be_bytes());
    hasher.update(bytes);
}

fn push_optional_str(hasher: &mut blake3::Hasher, value: Option<&str>) {
    match value {
        Some(value) => {
            hasher.update(&[1]);
            push_str(hasher, value);
        }
        None => {
            hasher.update(&[0]);
        }
    }
}

fn tagged_hash(prefix: &str, hash: blake3::Hash) -> String {
    format!("{prefix}{}", hash.to_hex())
}

/// Exact subject references consumed by the provider-effect layer.
///
/// F0 deliberately treats upstream authority/value/capacity objects as opaque
/// exact commitments. It does not reimplement or positively qualify them.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct TreasuryEffectSubject {
    pub operation_id: String,
    pub action_id: String,
    pub proposal_id: String,
    pub claim_binding_commitment: String,
    pub action_commitment: String,
    pub authorization_id: String,
    pub authorization_subject_commitment: String,
    pub treasury_descriptor_commitment: String,
    pub allocation_subject_commitment: String,
    pub approval_projection_commitment: String,
    pub capacity_allocation_commitment: Option<String>,
    pub recipient_did: String,
    pub recipient_commitment: String,
    pub value_profile_id: String,
    pub value_authority_commitment: String,
    pub policy_revision_commitment: String,
    pub adapter_profile_id: String,
    pub effect_target_commitment: String,
}

impl TreasuryEffectSubject {
    pub fn validate(&self) -> TreasuryEffectResult<()> {
        require_opaque("operation_id", &self.operation_id, MAX_ID_LEN)?;
        require_opaque("action_id", &self.action_id, MAX_ID_LEN)?;
        require_opaque("proposal_id", &self.proposal_id, MAX_ID_LEN)?;
        require_opaque(
            "claim_binding_commitment",
            &self.claim_binding_commitment,
            MAX_REF_LEN,
        )?;
        require_opaque("action_commitment", &self.action_commitment, MAX_REF_LEN)?;
        require_opaque("authorization_id", &self.authorization_id, MAX_ID_LEN)?;
        require_opaque(
            "authorization_subject_commitment",
            &self.authorization_subject_commitment,
            MAX_REF_LEN,
        )?;
        require_opaque(
            "treasury_descriptor_commitment",
            &self.treasury_descriptor_commitment,
            MAX_REF_LEN,
        )?;
        require_opaque(
            "allocation_subject_commitment",
            &self.allocation_subject_commitment,
            MAX_REF_LEN,
        )?;
        require_opaque(
            "approval_projection_commitment",
            &self.approval_projection_commitment,
            MAX_REF_LEN,
        )?;
        if let Some(capacity) = &self.capacity_allocation_commitment {
            require_opaque("capacity_allocation_commitment", capacity, MAX_REF_LEN)?;
        }
        require_did("recipient_did", &self.recipient_did)?;
        require_opaque("recipient_commitment", &self.recipient_commitment, MAX_REF_LEN)?;
        require_opaque("value_profile_id", &self.value_profile_id, MAX_ID_LEN)?;
        require_opaque(
            "value_authority_commitment",
            &self.value_authority_commitment,
            MAX_REF_LEN,
        )?;
        require_opaque(
            "policy_revision_commitment",
            &self.policy_revision_commitment,
            MAX_REF_LEN,
        )?;
        require_opaque("adapter_profile_id", &self.adapter_profile_id, MAX_ID_LEN)?;
        require_opaque(
            "effect_target_commitment",
            &self.effect_target_commitment,
            MAX_REF_LEN,
        )?;
        Ok(())
    }
}

/// Canonical provider request identity.
///
/// No wall-clock time, attempt number, worker identity, random retry nonce, or
/// external receipt identifier participates in this commitment.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct TreasuryEffectRequest {
    pub schema_version: u16,
    pub subject: TreasuryEffectSubject,
    pub request_commitment: String,
}

impl TreasuryEffectRequest {
    pub fn new(subject: TreasuryEffectSubject) -> TreasuryEffectResult<Self> {
        subject.validate()?;
        let mut request = Self {
            schema_version: TREASURY_EFFECT_SCHEMA_VERSION,
            subject,
            request_commitment: String::new(),
        };
        request.request_commitment = request.compute_request_commitment()?;
        Ok(request)
    }

    pub fn validate(&self) -> TreasuryEffectResult<()> {
        if self.schema_version != TREASURY_EFFECT_SCHEMA_VERSION {
            return Err(violation(format!(
                "unsupported treasury effect schema version {}",
                self.schema_version
            )));
        }
        self.subject.validate()?;
        require_tagged_hash(
            "request_commitment",
            &self.request_commitment,
            REQUEST_COMMITMENT_PREFIX,
        )?;
        let expected = self.compute_request_commitment()?;
        if expected != self.request_commitment {
            return Err(violation(
                "request_commitment does not match treasury effect subject",
            ));
        }
        Ok(())
    }

    fn compute_request_commitment(&self) -> TreasuryEffectResult<String> {
        self.subject.validate()?;
        let s = &self.subject;
        let mut hasher = blake3::Hasher::new();
        hasher.update(REQUEST_DOMAIN);
        hasher.update(&self.schema_version.to_be_bytes());
        push_str(&mut hasher, &s.operation_id);
        push_str(&mut hasher, &s.action_id);
        push_str(&mut hasher, &s.proposal_id);
        push_str(&mut hasher, &s.claim_binding_commitment);
        push_str(&mut hasher, &s.action_commitment);
        push_str(&mut hasher, &s.authorization_id);
        push_str(&mut hasher, &s.authorization_subject_commitment);
        push_str(&mut hasher, &s.treasury_descriptor_commitment);
        push_str(&mut hasher, &s.allocation_subject_commitment);
        push_str(&mut hasher, &s.approval_projection_commitment);
        push_optional_str(
            &mut hasher,
            s.capacity_allocation_commitment.as_deref(),
        );
        push_str(&mut hasher, &s.recipient_did);
        push_str(&mut hasher, &s.recipient_commitment);
        push_str(&mut hasher, &s.value_profile_id);
        push_str(&mut hasher, &s.value_authority_commitment);
        push_str(&mut hasher, &s.policy_revision_commitment);
        push_str(&mut hasher, &s.adapter_profile_id);
        push_str(&mut hasher, &s.effect_target_commitment);
        Ok(tagged_hash(REQUEST_COMMITMENT_PREFIX, hasher.finalize()))
    }

    pub fn execution_id(&self) -> TreasuryEffectResult<String> {
        self.validate()?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(EXECUTION_DOMAIN);
        hasher.update(&self.schema_version.to_be_bytes());
        push_str(&mut hasher, &self.subject.action_id);
        push_str(&mut hasher, &self.subject.authorization_id);
        push_optional_str(
            &mut hasher,
            self.subject.capacity_allocation_commitment.as_deref(),
        );
        push_str(&mut hasher, &self.subject.adapter_profile_id);
        push_str(&mut hasher, &self.request_commitment);
        Ok(tagged_hash(EXECUTION_ID_PREFIX, hasher.finalize()))
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EffectIntent {
    pub schema_version: u16,
    pub execution_id: String,
    pub request: TreasuryEffectRequest,
    pub intent_commitment: String,
    /// Diagnostic/audit time only. It is deliberately excluded from semantic identity.
    pub committed_at_unix_ms: u64,
}

impl EffectIntent {
    pub fn new(
        request: TreasuryEffectRequest,
        committed_at_unix_ms: u64,
    ) -> TreasuryEffectResult<Self> {
        request.validate()?;
        let execution_id = request.execution_id()?;
        let mut intent = Self {
            schema_version: TREASURY_EFFECT_SCHEMA_VERSION,
            execution_id,
            request,
            intent_commitment: String::new(),
            committed_at_unix_ms,
        };
        intent.intent_commitment = intent.compute_intent_commitment()?;
        Ok(intent)
    }

    pub fn validate(&self) -> TreasuryEffectResult<()> {
        if self.schema_version != TREASURY_EFFECT_SCHEMA_VERSION {
            return Err(violation("effect intent schema version drift"));
        }
        self.request.validate()?;
        require_tagged_hash("execution_id", &self.execution_id, EXECUTION_ID_PREFIX)?;
        require_tagged_hash(
            "intent_commitment",
            &self.intent_commitment,
            INTENT_COMMITMENT_PREFIX,
        )?;
        if self.execution_id != self.request.execution_id()? {
            return Err(violation(
                "execution_id does not match exact treasury effect request",
            ));
        }
        if self.intent_commitment != self.compute_intent_commitment()? {
            return Err(violation(
                "intent_commitment does not match execution/request identity",
            ));
        }
        Ok(())
    }

    fn compute_intent_commitment(&self) -> TreasuryEffectResult<String> {
        self.request.validate()?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(INTENT_DOMAIN);
        hasher.update(&self.schema_version.to_be_bytes());
        push_str(&mut hasher, &self.execution_id);
        push_str(&mut hasher, &self.request.request_commitment);
        Ok(tagged_hash(INTENT_COMMITMENT_PREFIX, hasher.finalize()))
    }
}

/// Reference-model basis for a retry decision.
///
/// These variants are decision inputs, not portable proof that the named provider
/// capability/evidence has itself been authenticated or qualified. F1 must obtain
/// such evidence from a separately qualified provider boundary.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RetryBasis {
    Initial,
    ProviderReplayQualified {
        capability_evidence_id: String,
    },
    ReconciledNoEffect {
        observation_id: String,
    },
}

impl RetryBasis {
    fn validate(&self) -> TreasuryEffectResult<()> {
        match self {
            Self::Initial => Ok(()),
            Self::ProviderReplayQualified {
                capability_evidence_id,
            } => require_opaque(
                "capability_evidence_id",
                capability_evidence_id,
                MAX_REF_LEN,
            ),
            Self::ReconciledNoEffect { observation_id } => {
                require_opaque("observation_id", observation_id, MAX_ID_LEN)
            }
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EffectAttempt {
    pub attempt_id: String,
    pub attempt_ordinal: u32,
    pub execution_id: String,
    pub started_at_unix_ms: u64,
    pub retry_basis: RetryBasis,
}

impl EffectAttempt {
    fn validate_against(&self, intent: &EffectIntent) -> TreasuryEffectResult<()> {
        require_opaque("attempt_id", &self.attempt_id, MAX_ID_LEN)?;
        if self.attempt_ordinal == 0 {
            return Err(violation("attempt_ordinal must be >= 1"));
        }
        if self.execution_id != intent.execution_id {
            return Err(violation("attempt execution_id mismatch"));
        }
        self.retry_basis.validate()?;
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum ProviderObservationKind {
    KnownSuccess {
        effect_commitment: String,
        external_receipt_id: String,
        receipt_commitment: String,
    },
    KnownNoEffect {
        no_effect_evidence_id: String,
    },
    UnknownOutcome {
        provider_evidence_id: String,
    },
}

impl ProviderObservationKind {
    fn validate(&self) -> TreasuryEffectResult<()> {
        match self {
            Self::KnownSuccess {
                effect_commitment,
                external_receipt_id,
                receipt_commitment,
            } => {
                require_opaque("effect_commitment", effect_commitment, MAX_REF_LEN)?;
                require_opaque("external_receipt_id", external_receipt_id, MAX_ID_LEN)?;
                require_opaque("receipt_commitment", receipt_commitment, MAX_REF_LEN)
            }
            Self::KnownNoEffect {
                no_effect_evidence_id,
            } => require_opaque(
                "no_effect_evidence_id",
                no_effect_evidence_id,
                MAX_REF_LEN,
            ),
            Self::UnknownOutcome {
                provider_evidence_id,
            } => require_opaque(
                "provider_evidence_id",
                provider_evidence_id,
                MAX_REF_LEN,
            ),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ProviderObservation {
    pub observation_id: String,
    pub execution_id: String,
    pub request_commitment: String,
    /// Provider evidence claims to cover every dispatch attempt through this ordinal.
    pub covers_through_attempt_ordinal: u32,
    pub kind: ProviderObservationKind,
    /// Evidence time only; it is not part of execution identity.
    pub observed_at_unix_ms: u64,
}

impl ProviderObservation {
    pub fn validate(&self) -> TreasuryEffectResult<()> {
        require_opaque("observation_id", &self.observation_id, MAX_ID_LEN)?;
        require_tagged_hash("execution_id", &self.execution_id, EXECUTION_ID_PREFIX)?;
        require_tagged_hash(
            "request_commitment",
            &self.request_commitment,
            REQUEST_COMMITMENT_PREFIX,
        )?;
        if self.covers_through_attempt_ordinal == 0 {
            return Err(violation(
                "provider observation must cover at least one dispatch attempt",
            ));
        }
        self.kind.validate()
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum EffectIntegrityFault {
    ObservationIdCollision {
        observation_id: String,
    },
    SuccessWithoutDispatch {
        observation_id: String,
    },
    ConflictingSuccess {
        prior_observation_id: String,
        candidate_observation_id: String,
    },
    SuccessContradictsNoEffect {
        no_effect_observation_id: String,
        success_observation_id: String,
    },
    NoEffectContradictsSuccess {
        success_observation_id: String,
        no_effect_observation_id: String,
    },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum EffectState {
    Pending,
    InFlight {
        attempt_id: String,
        attempt_ordinal: u32,
    },
    UnknownOutcome {
        through_attempt_ordinal: u32,
        evidence_ref: String,
    },
    KnownNoEffect {
        through_attempt_ordinal: u32,
        observation_id: String,
    },
    KnownSuccess {
        through_attempt_ordinal: u32,
        observation_id: String,
        effect_commitment: String,
    },
    IntegrityHalted {
        fault: EffectIntegrityFault,
    },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ObservationDecision {
    Accepted,
    ExistingSame,
    HistoricalOnly,
    IntegrityConflict {
        fault: EffectIntegrityFault,
    },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EffectRecord {
    pub intent: EffectIntent,
    pub state: EffectState,
    pub attempts: Vec<EffectAttempt>,
    pub observations: BTreeMap<String, ProviderObservation>,
}

impl EffectRecord {
    pub fn new(intent: EffectIntent) -> TreasuryEffectResult<Self> {
        intent.validate()?;
        Ok(Self {
            intent,
            state: EffectState::Pending,
            attempts: Vec::new(),
            observations: BTreeMap::new(),
        })
    }

    pub fn begin_attempt(
        &mut self,
        attempt_id: impl Into<String>,
        started_at_unix_ms: u64,
        retry_basis: RetryBasis,
    ) -> TreasuryEffectResult<EffectAttempt> {
        if let EffectState::IntegrityHalted { fault } = &self.state {
            return Err(TreasuryEffectError::IntegrityHalted(format!(
                "{:?}",
                fault
            )));
        }
        retry_basis.validate()?;
        let attempt_id = attempt_id.into();
        require_opaque("attempt_id", &attempt_id, MAX_ID_LEN)?;
        if self
            .attempts
            .iter()
            .any(|attempt| attempt.attempt_id == attempt_id)
        {
            return Err(violation("attempt_id has already been used"));
        }

        match (&self.state, &retry_basis) {
            (EffectState::Pending, RetryBasis::Initial) if self.attempts.is_empty() => {}
            (
                EffectState::UnknownOutcome { .. },
                RetryBasis::ProviderReplayQualified { .. },
            ) => {}
            (
                EffectState::KnownNoEffect { observation_id, .. },
                RetryBasis::ReconciledNoEffect {
                    observation_id: supplied,
                },
            ) if observation_id == supplied => {}
            (EffectState::UnknownOutcome { .. }, _) => {
                return Err(violation(
                    "UnknownOutcome cannot be retried without qualified replay evidence",
                ));
            }
            (EffectState::KnownNoEffect { .. }, _) => {
                return Err(violation(
                    "KnownNoEffect retry must cite the exact reconciliation observation",
                ));
            }
            (EffectState::KnownSuccess { .. }, _) => {
                return Err(violation("KnownSuccess is terminal for this execution_id"));
            }
            (EffectState::InFlight { .. }, _) => {
                return Err(violation("an effect attempt is already in flight"));
            }
            (EffectState::Pending, _) => {
                return Err(violation("the first attempt must use RetryBasis::Initial"));
            }
            (EffectState::IntegrityHalted { .. }, _) => unreachable!(),
        }

        let ordinal = u32::try_from(self.attempts.len() + 1)
            .map_err(|_| violation("attempt ordinal overflow"))?;
        let attempt = EffectAttempt {
            attempt_id: attempt_id.clone(),
            attempt_ordinal: ordinal,
            execution_id: self.intent.execution_id.clone(),
            started_at_unix_ms,
            retry_basis,
        };
        attempt.validate_against(&self.intent)?;
        self.attempts.push(attempt.clone());
        self.state = EffectState::InFlight {
            attempt_id,
            attempt_ordinal: ordinal,
        };
        Ok(attempt)
    }

    /// A transport timeout/error establishes uncertainty, never authoritative no-effect.
    pub fn record_transport_failure(
        &mut self,
        attempt_id: &str,
        transport_error_ref: impl Into<String>,
    ) -> TreasuryEffectResult<()> {
        let transport_error_ref = transport_error_ref.into();
        require_opaque(
            "transport_error_ref",
            &transport_error_ref,
            MAX_REF_LEN,
        )?;
        match &self.state {
            EffectState::InFlight {
                attempt_id: active,
                attempt_ordinal,
            } if active == attempt_id => {
                self.state = EffectState::UnknownOutcome {
                    through_attempt_ordinal: *attempt_ordinal,
                    evidence_ref: transport_error_ref,
                };
                Ok(())
            }
            EffectState::IntegrityHalted { fault } => {
                Err(TreasuryEffectError::IntegrityHalted(format!("{:?}", fault)))
            }
            _ => Err(violation(
                "transport failure must refer to the currently in-flight attempt",
            )),
        }
    }

    pub fn apply_observation(
        &mut self,
        observation: ProviderObservation,
    ) -> TreasuryEffectResult<ObservationDecision> {
        observation.validate()?;
        if observation.execution_id != self.intent.execution_id {
            return Err(violation("provider observation execution_id mismatch"));
        }
        if observation.request_commitment != self.intent.request.request_commitment {
            return Err(violation(
                "provider observation request_commitment mismatch",
            ));
        }
        let attempt_count = u32::try_from(self.attempts.len())
            .map_err(|_| violation("attempt count overflow"))?;
        if observation.covers_through_attempt_ordinal > attempt_count {
            return Err(violation(
                "provider observation claims to cover an attempt that does not exist",
            ));
        }

        if let Some(existing) = self.observations.get(&observation.observation_id) {
            if existing == &observation {
                return Ok(ObservationDecision::ExistingSame);
            }
            let fault = EffectIntegrityFault::ObservationIdCollision {
                observation_id: observation.observation_id,
            };
            self.state = EffectState::IntegrityHalted {
                fault: fault.clone(),
            };
            return Ok(ObservationDecision::IntegrityConflict { fault });
        }

        let decision = match &observation.kind {
            ProviderObservationKind::KnownSuccess {
                effect_commitment, ..
            } => self.apply_success_observation(&observation, effect_commitment)?,
            ProviderObservationKind::KnownNoEffect { .. } => {
                self.apply_no_effect_observation(&observation)?
            }
            ProviderObservationKind::UnknownOutcome { .. } => {
                self.apply_unknown_observation(&observation)?
            }
        };

        if !matches!(decision, ObservationDecision::IntegrityConflict { .. }) {
            self.observations
                .insert(observation.observation_id.clone(), observation);
        }
        Ok(decision)
    }

    fn apply_success_observation(
        &mut self,
        observation: &ProviderObservation,
        effect_commitment: &str,
    ) -> TreasuryEffectResult<ObservationDecision> {
        let current = self.state.clone();
        match current {
            EffectState::Pending => {
                let fault = EffectIntegrityFault::SuccessWithoutDispatch {
                    observation_id: observation.observation_id.clone(),
                };
                self.state = EffectState::IntegrityHalted {
                    fault: fault.clone(),
                };
                Ok(ObservationDecision::IntegrityConflict { fault })
            }
            EffectState::KnownSuccess {
                observation_id,
                effect_commitment: existing,
                ..
            } => {
                if existing == effect_commitment {
                    Ok(ObservationDecision::Accepted)
                } else {
                    let fault = EffectIntegrityFault::ConflictingSuccess {
                        prior_observation_id: observation_id,
                        candidate_observation_id: observation.observation_id.clone(),
                    };
                    self.state = EffectState::IntegrityHalted {
                        fault: fault.clone(),
                    };
                    Ok(ObservationDecision::IntegrityConflict { fault })
                }
            }
            EffectState::KnownNoEffect { observation_id, .. } => {
                let fault = EffectIntegrityFault::SuccessContradictsNoEffect {
                    no_effect_observation_id: observation_id,
                    success_observation_id: observation.observation_id.clone(),
                };
                self.state = EffectState::IntegrityHalted {
                    fault: fault.clone(),
                };
                Ok(ObservationDecision::IntegrityConflict { fault })
            }
            EffectState::IntegrityHalted { fault } => {
                Err(TreasuryEffectError::IntegrityHalted(format!("{:?}", fault)))
            }
            EffectState::InFlight { .. } | EffectState::UnknownOutcome { .. } => {
                self.state = EffectState::KnownSuccess {
                    through_attempt_ordinal: observation.covers_through_attempt_ordinal,
                    observation_id: observation.observation_id.clone(),
                    effect_commitment: effect_commitment.to_string(),
                };
                Ok(ObservationDecision::Accepted)
            }
        }
    }

    fn apply_no_effect_observation(
        &mut self,
        observation: &ProviderObservation,
    ) -> TreasuryEffectResult<ObservationDecision> {
        let current = self.state.clone();
        match current {
            EffectState::Pending => Err(violation(
                "KnownNoEffect cannot cover a semantic effect before any dispatch attempt",
            )),
            EffectState::InFlight {
                attempt_ordinal, ..
            } => {
                if observation.covers_through_attempt_ordinal < attempt_ordinal {
                    Ok(ObservationDecision::HistoricalOnly)
                } else {
                    self.state = EffectState::KnownNoEffect {
                        through_attempt_ordinal: observation.covers_through_attempt_ordinal,
                        observation_id: observation.observation_id.clone(),
                    };
                    Ok(ObservationDecision::Accepted)
                }
            }
            EffectState::UnknownOutcome {
                through_attempt_ordinal,
                ..
            } => {
                if observation.covers_through_attempt_ordinal < through_attempt_ordinal {
                    Ok(ObservationDecision::HistoricalOnly)
                } else {
                    self.state = EffectState::KnownNoEffect {
                        through_attempt_ordinal: observation.covers_through_attempt_ordinal,
                        observation_id: observation.observation_id.clone(),
                    };
                    Ok(ObservationDecision::Accepted)
                }
            }
            EffectState::KnownNoEffect {
                through_attempt_ordinal,
                ..
            } => {
                if observation.covers_through_attempt_ordinal < through_attempt_ordinal {
                    Ok(ObservationDecision::HistoricalOnly)
                } else {
                    self.state = EffectState::KnownNoEffect {
                        through_attempt_ordinal: observation.covers_through_attempt_ordinal,
                        observation_id: observation.observation_id.clone(),
                    };
                    Ok(ObservationDecision::Accepted)
                }
            }
            EffectState::KnownSuccess {
                through_attempt_ordinal,
                observation_id,
                ..
            } => {
                if observation.covers_through_attempt_ordinal < through_attempt_ordinal {
                    Ok(ObservationDecision::HistoricalOnly)
                } else {
                    let fault = EffectIntegrityFault::NoEffectContradictsSuccess {
                        success_observation_id: observation_id,
                        no_effect_observation_id: observation.observation_id.clone(),
                    };
                    self.state = EffectState::IntegrityHalted {
                        fault: fault.clone(),
                    };
                    Ok(ObservationDecision::IntegrityConflict { fault })
                }
            }
            EffectState::IntegrityHalted { fault } => {
                Err(TreasuryEffectError::IntegrityHalted(format!("{:?}", fault)))
            }
        }
    }

    fn apply_unknown_observation(
        &mut self,
        observation: &ProviderObservation,
    ) -> TreasuryEffectResult<ObservationDecision> {
        let current = self.state.clone();
        match current {
            EffectState::Pending => Err(violation(
                "UnknownOutcome cannot cover a semantic effect before any dispatch attempt",
            )),
            EffectState::InFlight {
                attempt_ordinal, ..
            } => {
                if observation.covers_through_attempt_ordinal < attempt_ordinal {
                    Ok(ObservationDecision::HistoricalOnly)
                } else {
                    self.state = EffectState::UnknownOutcome {
                        through_attempt_ordinal: observation.covers_through_attempt_ordinal,
                        evidence_ref: observation.observation_id.clone(),
                    };
                    Ok(ObservationDecision::Accepted)
                }
            }
            EffectState::UnknownOutcome {
                through_attempt_ordinal,
                ..
            } => {
                if observation.covers_through_attempt_ordinal < through_attempt_ordinal {
                    Ok(ObservationDecision::HistoricalOnly)
                } else {
                    self.state = EffectState::UnknownOutcome {
                        through_attempt_ordinal: observation.covers_through_attempt_ordinal,
                        evidence_ref: observation.observation_id.clone(),
                    };
                    Ok(ObservationDecision::Accepted)
                }
            }
            EffectState::KnownNoEffect { .. } | EffectState::KnownSuccess { .. } => {
                Ok(ObservationDecision::HistoricalOnly)
            }
            EffectState::IntegrityHalted { fault } => {
                Err(TreasuryEffectError::IntegrityHalted(format!("{:?}", fault)))
            }
        }
    }

    pub fn validate_invariants(&self) -> TreasuryEffectResult<()> {
        self.intent.validate()?;
        for (index, attempt) in self.attempts.iter().enumerate() {
            attempt.validate_against(&self.intent)?;
            let expected = u32::try_from(index + 1)
                .map_err(|_| violation("attempt ordinal overflow"))?;
            if attempt.attempt_ordinal != expected {
                return Err(violation("attempt ordinals must be contiguous"));
            }
        }
        for (id, observation) in &self.observations {
            if id != &observation.observation_id {
                return Err(violation("observation index key mismatch"));
            }
            observation.validate()?;
            if observation.execution_id != self.intent.execution_id
                || observation.request_commitment != self.intent.request.request_commitment
            {
                return Err(violation(
                    "stored observation does not bind this exact effect intent",
                ));
            }
            if observation.covers_through_attempt_ordinal
                > u32::try_from(self.attempts.len())
                    .map_err(|_| violation("attempt count overflow"))?
            {
                return Err(violation("stored observation covers nonexistent attempt"));
            }
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RegistryIntegrityFault {
    ExecutionIdentityCollision {
        execution_id: String,
    },
    ActionIdentityCollision {
        action_id: String,
        existing_execution_id: String,
        candidate_execution_id: String,
    },
    AuthorizationReuseConflict {
        authorization_id: String,
        existing_execution_id: String,
        candidate_execution_id: String,
    },
    CapacityReuseConflict {
        capacity_allocation_commitment: String,
        existing_execution_id: String,
        candidate_execution_id: String,
    },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RegistryDecision {
    Created {
        execution_id: String,
    },
    ExistingSame {
        execution_id: String,
    },
    IntegrityConflict {
        fault: RegistryIntegrityFault,
    },
}

/// Pure registry enforcing semantic uniqueness before any future provider dispatch.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TreasuryEffectRegistry {
    pub records: BTreeMap<String, EffectRecord>,
    pub action_index: BTreeMap<String, String>,
    pub authorization_index: BTreeMap<String, String>,
    pub capacity_index: BTreeMap<String, String>,
    pub integrity_fault: Option<RegistryIntegrityFault>,
}

impl Default for TreasuryEffectRegistry {
    fn default() -> Self {
        Self::new()
    }
}

impl TreasuryEffectRegistry {
    pub fn new() -> Self {
        Self {
            records: BTreeMap::new(),
            action_index: BTreeMap::new(),
            authorization_index: BTreeMap::new(),
            capacity_index: BTreeMap::new(),
            integrity_fault: None,
        }
    }

    pub fn submit(
        &mut self,
        request: TreasuryEffectRequest,
        committed_at_unix_ms: u64,
    ) -> TreasuryEffectResult<RegistryDecision> {
        if let Some(fault) = &self.integrity_fault {
            return Err(TreasuryEffectError::IntegrityHalted(format!(
                "{:?}",
                fault
            )));
        }
        request.validate()?;
        let intent = EffectIntent::new(request, committed_at_unix_ms)?;
        let execution_id = intent.execution_id.clone();
        let request_commitment = intent.request.request_commitment.clone();
        let action_id = intent.request.subject.action_id.clone();
        let authorization_id = intent.request.subject.authorization_id.clone();
        let capacity = intent
            .request
            .subject
            .capacity_allocation_commitment
            .clone();

        if let Some(existing) = self.records.get(&execution_id) {
            if existing.intent.request.request_commitment == request_commitment {
                return Ok(RegistryDecision::ExistingSame { execution_id });
            }
            return Ok(self.halt(RegistryIntegrityFault::ExecutionIdentityCollision {
                execution_id,
            }));
        }

        if let Some(existing_execution_id) = self.action_index.get(&action_id).cloned() {
            let existing = self.records.get(&existing_execution_id).ok_or_else(|| {
                violation("action index references missing treasury effect record")
            })?;
            if existing.intent.request.request_commitment == request_commitment {
                return Ok(RegistryDecision::ExistingSame {
                    execution_id: existing_execution_id,
                });
            }
            return Ok(self.halt(RegistryIntegrityFault::ActionIdentityCollision {
                action_id,
                existing_execution_id,
                candidate_execution_id: execution_id,
            }));
        }

        if let Some(existing_execution_id) =
            self.authorization_index.get(&authorization_id).cloned()
        {
            return Ok(self.halt(RegistryIntegrityFault::AuthorizationReuseConflict {
                authorization_id,
                existing_execution_id,
                candidate_execution_id: execution_id,
            }));
        }

        if let Some(capacity_commitment) = capacity.as_ref() {
            if let Some(existing_execution_id) =
                self.capacity_index.get(capacity_commitment).cloned()
            {
                return Ok(self.halt(RegistryIntegrityFault::CapacityReuseConflict {
                    capacity_allocation_commitment: capacity_commitment.clone(),
                    existing_execution_id,
                    candidate_execution_id: execution_id,
                }));
            }
        }

        let record = EffectRecord::new(intent)?;
        self.action_index.insert(action_id, execution_id.clone());
        self.authorization_index
            .insert(authorization_id, execution_id.clone());
        if let Some(capacity_commitment) = capacity {
            self.capacity_index
                .insert(capacity_commitment, execution_id.clone());
        }
        self.records.insert(execution_id.clone(), record);
        self.validate_invariants()?;
        Ok(RegistryDecision::Created { execution_id })
    }

    fn halt(&mut self, fault: RegistryIntegrityFault) -> RegistryDecision {
        self.integrity_fault = Some(fault.clone());
        RegistryDecision::IntegrityConflict { fault }
    }

    pub fn record(&self, execution_id: &str) -> Option<&EffectRecord> {
        self.records.get(execution_id)
    }

    pub fn record_mut(&mut self, execution_id: &str) -> TreasuryEffectResult<&mut EffectRecord> {
        if let Some(fault) = &self.integrity_fault {
            return Err(TreasuryEffectError::IntegrityHalted(format!(
                "{:?}",
                fault
            )));
        }
        self.records
            .get_mut(execution_id)
            .ok_or_else(|| violation("treasury effect execution_id not found"))
    }

    pub fn by_action(&self, action_id: &str) -> Option<&EffectRecord> {
        self.action_index
            .get(action_id)
            .and_then(|execution_id| self.records.get(execution_id))
    }

    pub fn by_authorization(&self, authorization_id: &str) -> Option<&EffectRecord> {
        self.authorization_index
            .get(authorization_id)
            .and_then(|execution_id| self.records.get(execution_id))
    }

    pub fn validate_invariants(&self) -> TreasuryEffectResult<()> {
        if self.records.len() != self.action_index.len()
            || self.records.len() != self.authorization_index.len()
        {
            return Err(violation(
                "every treasury effect must have exactly one action and authorization index",
            ));
        }
        for (execution_id, record) in &self.records {
            record.validate_invariants()?;
            if execution_id != &record.intent.execution_id {
                return Err(violation("execution registry key mismatch"));
            }
            let subject = &record.intent.request.subject;
            if self.action_index.get(&subject.action_id) != Some(execution_id) {
                return Err(violation("action index does not bind exact execution_id"));
            }
            if self.authorization_index.get(&subject.authorization_id) != Some(execution_id) {
                return Err(violation(
                    "authorization index does not bind exact execution_id",
                ));
            }
            if let Some(capacity) = &subject.capacity_allocation_commitment {
                if self.capacity_index.get(capacity) != Some(execution_id) {
                    return Err(violation(
                        "capacity index does not bind exact execution_id",
                    ));
                }
            }
        }
        for execution_id in self.capacity_index.values() {
            if !self.records.contains_key(execution_id) {
                return Err(violation("capacity index references missing effect record"));
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn subject() -> TreasuryEffectSubject {
        TreasuryEffectSubject {
            operation_id: "op-1".into(),
            action_id: "action-1".into(),
            proposal_id: "proposal-1".into(),
            claim_binding_commitment: "claim-binding-ref-1".into(),
            action_commitment: "action-commitment-1".into(),
            authorization_id: "authorization-1".into(),
            authorization_subject_commitment: "authorization-subject-1".into(),
            treasury_descriptor_commitment: "treasury-descriptor-1".into(),
            allocation_subject_commitment: "allocation-subject-1".into(),
            approval_projection_commitment: "approval-projection-1".into(),
            capacity_allocation_commitment: Some("capacity-allocation-1".into()),
            recipient_did: "did:mycelix:recipient".into(),
            recipient_commitment: "recipient-commitment-1".into(),
            value_profile_id: "qualified-sap-amount-v1".into(),
            value_authority_commitment: "sap-authority-1".into(),
            policy_revision_commitment: "policy-revision-1".into(),
            adapter_profile_id: "treasury-adapter-v1".into(),
            effect_target_commitment: "provider-target-1".into(),
        }
    }

    fn request() -> TreasuryEffectRequest {
        TreasuryEffectRequest::new(subject()).unwrap()
    }

    fn success_observation(
        record: &EffectRecord,
        observation_id: &str,
        through: u32,
        effect_commitment: &str,
    ) -> ProviderObservation {
        ProviderObservation {
            observation_id: observation_id.into(),
            execution_id: record.intent.execution_id.clone(),
            request_commitment: record.intent.request.request_commitment.clone(),
            covers_through_attempt_ordinal: through,
            kind: ProviderObservationKind::KnownSuccess {
                effect_commitment: effect_commitment.into(),
                external_receipt_id: format!("receipt-{observation_id}"),
                receipt_commitment: format!("receipt-commitment-{observation_id}"),
            },
            observed_at_unix_ms: 100,
        }
    }

    fn no_effect_observation(
        record: &EffectRecord,
        observation_id: &str,
        through: u32,
    ) -> ProviderObservation {
        ProviderObservation {
            observation_id: observation_id.into(),
            execution_id: record.intent.execution_id.clone(),
            request_commitment: record.intent.request.request_commitment.clone(),
            covers_through_attempt_ordinal: through,
            kind: ProviderObservationKind::KnownNoEffect {
                no_effect_evidence_id: format!("no-effect-{observation_id}"),
            },
            observed_at_unix_ms: 100,
        }
    }

    #[test]
    fn wall_clock_does_not_change_execution_or_intent_identity() {
        let req = request();
        let a = EffectIntent::new(req.clone(), 1).unwrap();
        let b = EffectIntent::new(req, 999_999).unwrap();
        assert_eq!(a.execution_id, b.execution_id);
        assert_eq!(a.intent_commitment, b.intent_commitment);
    }

    #[test]
    fn semantic_subject_change_changes_request_and_execution_identity() {
        let base = request();
        for mutate in [0_u8, 1, 2] {
            let mut changed = subject();
            match mutate {
                0 => changed.recipient_commitment.push_str("-changed"),
                1 => changed.value_authority_commitment.push_str("-changed"),
                2 => changed.adapter_profile_id.push_str("-changed"),
                _ => unreachable!(),
            }
            let changed = TreasuryEffectRequest::new(changed).unwrap();
            assert_ne!(base.request_commitment, changed.request_commitment);
            assert_ne!(base.execution_id().unwrap(), changed.execution_id().unwrap());
        }
    }

    #[test]
    fn exact_duplicate_submission_returns_existing_effect() {
        let mut registry = TreasuryEffectRegistry::new();
        let req = request();
        let first = registry.submit(req.clone(), 1).unwrap();
        let second = registry.submit(req, 2).unwrap();
        let first_id = match first {
            RegistryDecision::Created { execution_id } => execution_id,
            other => panic!("unexpected decision: {other:?}"),
        };
        assert_eq!(
            second,
            RegistryDecision::ExistingSame {
                execution_id: first_id
            }
        );
    }

    #[test]
    fn same_action_with_changed_request_halts_registry() {
        let mut registry = TreasuryEffectRegistry::new();
        registry.submit(request(), 1).unwrap();
        let mut changed = subject();
        changed.recipient_commitment = "recipient-commitment-2".into();
        let changed = TreasuryEffectRequest::new(changed).unwrap();
        let decision = registry.submit(changed, 2).unwrap();
        assert!(matches!(
            decision,
            RegistryDecision::IntegrityConflict {
                fault: RegistryIntegrityFault::ActionIdentityCollision { .. }
            }
        ));
        assert!(registry.integrity_fault.is_some());
    }

    #[test]
    fn one_shot_authorization_cannot_mint_second_execution() {
        let mut registry = TreasuryEffectRegistry::new();
        registry.submit(request(), 1).unwrap();
        let mut changed = subject();
        changed.action_id = "action-2".into();
        changed.action_commitment = "action-commitment-2".into();
        changed.recipient_commitment = "recipient-commitment-2".into();
        let decision = registry
            .submit(TreasuryEffectRequest::new(changed).unwrap(), 2)
            .unwrap();
        assert!(matches!(
            decision,
            RegistryDecision::IntegrityConflict {
                fault: RegistryIntegrityFault::AuthorizationReuseConflict { .. }
            }
        ));
    }

    #[test]
    fn exact_capacity_allocation_cannot_back_two_effects() {
        let mut registry = TreasuryEffectRegistry::new();
        registry.submit(request(), 1).unwrap();
        let mut changed = subject();
        changed.action_id = "action-2".into();
        changed.action_commitment = "action-commitment-2".into();
        changed.authorization_id = "authorization-2".into();
        changed.authorization_subject_commitment = "authorization-subject-2".into();
        let decision = registry
            .submit(TreasuryEffectRequest::new(changed).unwrap(), 2)
            .unwrap();
        assert!(matches!(
            decision,
            RegistryDecision::IntegrityConflict {
                fault: RegistryIntegrityFault::CapacityReuseConflict { .. }
            }
        ));
    }

    #[test]
    fn transport_failure_is_unknown_not_no_effect() {
        let intent = EffectIntent::new(request(), 1).unwrap();
        let mut record = EffectRecord::new(intent).unwrap();
        record
            .begin_attempt("attempt-1", 10, RetryBasis::Initial)
            .unwrap();
        record
            .record_transport_failure("attempt-1", "transport-timeout")
            .unwrap();
        assert!(matches!(record.state, EffectState::UnknownOutcome { .. }));
    }

    #[test]
    fn unknown_outcome_cannot_blind_retry() {
        let intent = EffectIntent::new(request(), 1).unwrap();
        let mut record = EffectRecord::new(intent).unwrap();
        record
            .begin_attempt("attempt-1", 10, RetryBasis::Initial)
            .unwrap();
        record
            .record_transport_failure("attempt-1", "transport-timeout")
            .unwrap();
        let err = record
            .begin_attempt("attempt-2", 20, RetryBasis::Initial)
            .unwrap_err();
        assert!(err.to_string().contains("UnknownOutcome"));
    }

    #[test]
    fn qualified_replay_reuses_same_execution_identity() {
        let intent = EffectIntent::new(request(), 1).unwrap();
        let execution_id = intent.execution_id.clone();
        let mut record = EffectRecord::new(intent).unwrap();
        record
            .begin_attempt("attempt-1", 10, RetryBasis::Initial)
            .unwrap();
        record
            .record_transport_failure("attempt-1", "transport-timeout")
            .unwrap();
        let second = record
            .begin_attempt(
                "attempt-2",
                20,
                RetryBasis::ProviderReplayQualified {
                    capability_evidence_id: "qualified-provider-replay-evidence".into(),
                },
            )
            .unwrap();
        assert_eq!(second.execution_id, execution_id);
        assert_eq!(second.attempt_ordinal, 2);
    }

    #[test]
    fn authoritative_no_effect_reconciliation_allows_same_execution_retry() {
        let intent = EffectIntent::new(request(), 1).unwrap();
        let execution_id = intent.execution_id.clone();
        let mut record = EffectRecord::new(intent).unwrap();
        record
            .begin_attempt("attempt-1", 10, RetryBasis::Initial)
            .unwrap();
        record
            .record_transport_failure("attempt-1", "transport-timeout")
            .unwrap();
        let no_effect = no_effect_observation(&record, "obs-no-effect-1", 1);
        assert_eq!(
            record.apply_observation(no_effect).unwrap(),
            ObservationDecision::Accepted
        );
        let retry = record
            .begin_attempt(
                "attempt-2",
                20,
                RetryBasis::ReconciledNoEffect {
                    observation_id: "obs-no-effect-1".into(),
                },
            )
            .unwrap();
        assert_eq!(retry.execution_id, execution_id);
        assert_eq!(retry.attempt_ordinal, 2);
    }

    #[test]
    fn success_is_terminal_for_dispatch_retry() {
        let intent = EffectIntent::new(request(), 1).unwrap();
        let mut record = EffectRecord::new(intent).unwrap();
        record
            .begin_attempt("attempt-1", 10, RetryBasis::Initial)
            .unwrap();
        let success = success_observation(&record, "obs-success-1", 1, "effect-1");
        record.apply_observation(success).unwrap();
        assert!(matches!(record.state, EffectState::KnownSuccess { .. }));
        assert!(record
            .begin_attempt(
                "attempt-2",
                20,
                RetryBasis::ProviderReplayQualified {
                    capability_evidence_id: "replay".into(),
                },
            )
            .is_err());
    }

    #[test]
    fn conflicting_success_evidence_halts_effect() {
        let intent = EffectIntent::new(request(), 1).unwrap();
        let mut record = EffectRecord::new(intent).unwrap();
        record
            .begin_attempt("attempt-1", 10, RetryBasis::Initial)
            .unwrap();
        let first = success_observation(&record, "obs-success-1", 1, "effect-1");
        record.apply_observation(first).unwrap();
        let second = success_observation(&record, "obs-success-2", 1, "effect-2");
        let decision = record.apply_observation(second).unwrap();
        assert!(matches!(
            decision,
            ObservationDecision::IntegrityConflict {
                fault: EffectIntegrityFault::ConflictingSuccess { .. }
            }
        ));
        assert!(matches!(record.state, EffectState::IntegrityHalted { .. }));
    }

    #[test]
    fn no_effect_covering_success_horizon_is_conflict() {
        let intent = EffectIntent::new(request(), 1).unwrap();
        let mut record = EffectRecord::new(intent).unwrap();
        record
            .begin_attempt("attempt-1", 10, RetryBasis::Initial)
            .unwrap();
        let success = success_observation(&record, "obs-success-1", 1, "effect-1");
        record.apply_observation(success).unwrap();
        let no_effect = no_effect_observation(&record, "obs-no-effect-1", 1);
        assert!(matches!(
            record.apply_observation(no_effect).unwrap(),
            ObservationDecision::IntegrityConflict {
                fault: EffectIntegrityFault::NoEffectContradictsSuccess { .. }
            }
        ));
    }

    #[test]
    fn stale_provider_observation_cannot_overwrite_later_inflight_attempt() {
        let intent = EffectIntent::new(request(), 1).unwrap();
        let mut record = EffectRecord::new(intent).unwrap();
        record
            .begin_attempt("attempt-1", 10, RetryBasis::Initial)
            .unwrap();
        record
            .record_transport_failure("attempt-1", "timeout")
            .unwrap();
        record
            .begin_attempt(
                "attempt-2",
                20,
                RetryBasis::ProviderReplayQualified {
                    capability_evidence_id: "replay".into(),
                },
            )
            .unwrap();
        let stale = no_effect_observation(&record, "obs-old", 1);
        assert_eq!(
            record.apply_observation(stale).unwrap(),
            ObservationDecision::HistoricalOnly
        );
        assert!(matches!(
            record.state,
            EffectState::InFlight {
                attempt_ordinal: 2,
                ..
            }
        ));
    }

    #[test]
    fn observation_must_bind_exact_request() {
        let intent = EffectIntent::new(request(), 1).unwrap();
        let mut record = EffectRecord::new(intent).unwrap();
        record
            .begin_attempt("attempt-1", 10, RetryBasis::Initial)
            .unwrap();
        let mut observation = success_observation(&record, "obs-success-1", 1, "effect-1");
        observation.request_commitment = format!(
            "{}{}",
            REQUEST_COMMITMENT_PREFIX,
            "0".repeat(64)
        );
        assert!(record.apply_observation(observation).is_err());
    }
}
