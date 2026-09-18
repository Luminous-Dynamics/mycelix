use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
use thiserror::Error;

pub const PARAMETER_PROVIDER_SCHEMA_VERSION: u16 = 1;
pub const COMMITMENT_PREFIX: &str = "blake3-256:";
pub const MAX_ID_LEN: usize = 256;
pub const MAX_BINDING_LEN: usize = 512;
pub const MAX_PARAMETER_NAME_LEN: usize = 256;
pub const MAX_STRING_VALUE_LEN: usize = 4096;
pub const MAX_DECIMAL_LEN: usize = 128;

const VALUE_DOMAIN: &[u8] = b"MYCELIX-CONSTITUTIONAL-PARAMETER-VALUE\0V1\0";
const REQUEST_DOMAIN: &[u8] = b"MYCELIX-CONSTITUTIONAL-PARAMETER-REQUEST\0V1\0";
const REVISION_DOMAIN: &[u8] = b"MYCELIX-CONSTITUTIONAL-PARAMETER-REVISION\0V1\0";

#[derive(Debug, Error, Clone, PartialEq, Eq)]
pub enum ParameterProviderError {
    #[error("{0}")]
    Violation(String),
    #[error("json canonicalization failed: {0}")]
    Json(String),
    #[error("parameter history is integrity halted: {0}")]
    IntegrityHalted(String),
}

type ParameterResult<T> = Result<T, ParameterProviderError>;

fn violation(message: impl Into<String>) -> ParameterProviderError {
    ParameterProviderError::Violation(message.into())
}

fn require_opaque(label: &str, value: &str, max_len: usize) -> ParameterResult<()> {
    if value.trim().is_empty() || value.len() > max_len {
        return Err(violation(format!(
            "{label} must be non-empty and <= {max_len} bytes"
        )));
    }
    Ok(())
}

fn require_did(label: &str, value: &str) -> ParameterResult<()> {
    require_opaque(label, value, MAX_ID_LEN)?;
    if !value.starts_with("did:") {
        return Err(violation(format!("{label} must be a DID")));
    }
    Ok(())
}

fn require_commitment(label: &str, value: &str) -> ParameterResult<()> {
    let digest = value
        .strip_prefix(COMMITMENT_PREFIX)
        .ok_or_else(|| violation(format!("{label} must use {COMMITMENT_PREFIX}<hex>")))?;
    if digest.len() != 64
        || !digest
            .bytes()
            .all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b))
    {
        return Err(violation(format!(
            "{label} must contain exactly 64 lowercase hexadecimal BLAKE3-256 digits"
        )));
    }
    Ok(())
}

/// Authority material a future P1 DHT provider must verify before accepting a
/// constitutional parameter revision. P0 deliberately carries ClaimBinding as
/// opaque material and makes no positive runtime-authority claim.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ParameterAuthorityBinding {
    pub operation_id: String,
    pub action_id: String,
    pub proposal_id: String,
    pub claim_binding_commitment: String,
    pub action_commitment: String,
    pub publisher_did: String,
}

impl ParameterAuthorityBinding {
    pub fn validate(&self) -> ParameterResult<()> {
        require_opaque("operation_id", &self.operation_id, MAX_ID_LEN)?;
        require_opaque("action_id", &self.action_id, MAX_ID_LEN)?;
        require_opaque("proposal_id", &self.proposal_id, MAX_ID_LEN)?;
        require_opaque(
            "claim_binding_commitment",
            &self.claim_binding_commitment,
            MAX_BINDING_LEN,
        )?;
        require_opaque(
            "action_commitment",
            &self.action_commitment,
            MAX_BINDING_LEN,
        )?;
        require_did("publisher_did", &self.publisher_did)?;
        Ok(())
    }
}

/// P0's exact versioned value profile.
///
/// Decimal and Percentage are normalized base-10 decimal strings rather than
/// IEEE-754 values. Duration is explicit milliseconds. This prevents NaN,
/// infinity, exponent spelling, or platform floating-point behavior from
/// becoming part of constitutional identity.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ParameterValueType {
    Integer,
    Decimal,
    Percentage,
    DurationMillis,
    Boolean,
    String,
}

impl ParameterValueType {
    fn tag(self) -> &'static str {
        match self {
            Self::Integer => "integer",
            Self::Decimal => "decimal-string-v1",
            Self::Percentage => "percentage-decimal-string-v1",
            Self::DurationMillis => "duration-millis",
            Self::Boolean => "boolean",
            Self::String => "utf8-string",
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum CanonicalParameterValue {
    Integer(i64),
    Decimal(String),
    Percentage(String),
    DurationMillis(u64),
    Boolean(bool),
    String(String),
}

impl CanonicalParameterValue {
    /// Canonicalize a logical typed value. This intentionally does not accept an
    /// arbitrary JSON document: the provider contract owns the exact value type.
    pub fn from_typed_input(
        value_type: ParameterValueType,
        input: &str,
    ) -> ParameterResult<Self> {
        match value_type {
            ParameterValueType::Integer => {
                let normalized = normalize_integer(input)?;
                let value = normalized.parse::<i64>().map_err(|_| {
                    violation("integer parameter is outside the signed 64-bit range")
                })?;
                Ok(Self::Integer(value))
            }
            ParameterValueType::Decimal => Ok(Self::Decimal(normalize_decimal(input)?)),
            ParameterValueType::Percentage => {
                Ok(Self::Percentage(normalize_decimal(input)?))
            }
            ParameterValueType::DurationMillis => {
                let trimmed = input.trim();
                if trimmed.is_empty()
                    || !trimmed.bytes().all(|b| b.is_ascii_digit())
                    || (trimmed.len() > 1 && trimmed.starts_with('0'))
                {
                    return Err(violation(
                        "duration-millis must be canonical unsigned base-10 digits",
                    ));
                }
                let value = trimmed
                    .parse::<u64>()
                    .map_err(|_| violation("duration-millis is outside the u64 range"))?;
                Ok(Self::DurationMillis(value))
            }
            ParameterValueType::Boolean => match input.trim() {
                "true" => Ok(Self::Boolean(true)),
                "false" => Ok(Self::Boolean(false)),
                _ => Err(violation("boolean parameter must be exactly true or false")),
            },
            ParameterValueType::String => {
                if input.len() > MAX_STRING_VALUE_LEN {
                    return Err(violation(format!(
                        "string parameter must be <= {MAX_STRING_VALUE_LEN} bytes"
                    )));
                }
                Ok(Self::String(input.to_owned()))
            }
        }
    }

    pub fn value_type(&self) -> ParameterValueType {
        match self {
            Self::Integer(_) => ParameterValueType::Integer,
            Self::Decimal(_) => ParameterValueType::Decimal,
            Self::Percentage(_) => ParameterValueType::Percentage,
            Self::DurationMillis(_) => ParameterValueType::DurationMillis,
            Self::Boolean(_) => ParameterValueType::Boolean,
            Self::String(_) => ParameterValueType::String,
        }
    }

    pub fn validate(&self) -> ParameterResult<()> {
        match self {
            Self::Integer(_) | Self::DurationMillis(_) | Self::Boolean(_) => Ok(()),
            Self::Decimal(value) | Self::Percentage(value) => {
                let normalized = normalize_decimal(value)?;
                if &normalized != value {
                    return Err(violation(
                        "stored decimal parameter is not canonical decimal-string-v1",
                    ));
                }
                Ok(())
            }
            Self::String(value) => {
                if value.len() > MAX_STRING_VALUE_LEN {
                    return Err(violation(format!(
                        "string parameter must be <= {MAX_STRING_VALUE_LEN} bytes"
                    )));
                }
                Ok(())
            }
        }
    }

    /// Canonical interoperable JSON. Decimal/Percentage are JSON strings by
    /// design; their independent type tags prevent confusion with String.
    pub fn canonical_json(&self) -> ParameterResult<String> {
        self.validate()?;
        match self {
            Self::Integer(value) => Ok(value.to_string()),
            Self::Decimal(value) | Self::Percentage(value) | Self::String(value) => {
                serde_json::to_string(value)
                    .map_err(|e| ParameterProviderError::Json(e.to_string()))
            }
            Self::DurationMillis(value) => Ok(value.to_string()),
            Self::Boolean(value) => Ok(value.to_string()),
        }
    }

    pub fn commitment(&self) -> ParameterResult<String> {
        let canonical_json = self.canonical_json()?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(VALUE_DOMAIN);
        push_str(&mut hasher, self.value_type().tag());
        push_str(&mut hasher, &canonical_json);
        Ok(tagged_hash(hasher.finalize()))
    }
}

fn normalize_integer(input: &str) -> ParameterResult<String> {
    let trimmed = input.trim();
    if trimmed.is_empty() || trimmed.starts_with('+') {
        return Err(violation(
            "integer parameter must be signed base-10 digits without '+'",
        ));
    }
    let (negative, digits) = if let Some(rest) = trimmed.strip_prefix('-') {
        (true, rest)
    } else {
        (false, trimmed)
    };
    if digits.is_empty() || !digits.bytes().all(|b| b.is_ascii_digit()) {
        return Err(violation("integer parameter must contain only base-10 digits"));
    }
    let stripped = digits.trim_start_matches('0');
    let body = if stripped.is_empty() { "0" } else { stripped };
    if body == "0" {
        Ok("0".into())
    } else if negative {
        Ok(format!("-{body}"))
    } else {
        Ok(body.into())
    }
}

fn normalize_decimal(input: &str) -> ParameterResult<String> {
    let trimmed = input.trim();
    if trimmed.is_empty() || trimmed.len() > MAX_DECIMAL_LEN || trimmed.starts_with('+') {
        return Err(violation(
            "decimal parameter must be a bounded base-10 decimal without '+'",
        ));
    }
    let (negative, unsigned) = if let Some(rest) = trimmed.strip_prefix('-') {
        (true, rest)
    } else {
        (false, trimmed)
    };
    if unsigned.is_empty() || unsigned.contains('e') || unsigned.contains('E') {
        return Err(violation(
            "decimal parameter must not use exponent notation",
        ));
    }
    let mut parts = unsigned.split('.');
    let integer = parts.next().unwrap_or_default();
    let fraction = parts.next();
    if parts.next().is_some()
        || integer.is_empty()
        || !integer.bytes().all(|b| b.is_ascii_digit())
        || fraction
            .map(|f| f.is_empty() || !f.bytes().all(|b| b.is_ascii_digit()))
            .unwrap_or(false)
    {
        return Err(violation("invalid decimal-string-v1 parameter"));
    }

    let stripped_integer = integer.trim_start_matches('0');
    let integer = if stripped_integer.is_empty() {
        "0"
    } else {
        stripped_integer
    };
    let fraction = fraction.map(|f| f.trim_end_matches('0')).unwrap_or("");
    let zero = integer == "0" && fraction.is_empty();

    let mut normalized = String::new();
    if negative && !zero {
        normalized.push('-');
    }
    normalized.push_str(integer);
    if !fraction.is_empty() {
        normalized.push('.');
        normalized.push_str(fraction);
    }
    Ok(normalized)
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum PriorRevisionExpectation {
    CreateOnly,
    Exact {
        revision: u64,
        revision_commitment: String,
    },
}

impl PriorRevisionExpectation {
    pub fn validate(&self) -> ParameterResult<()> {
        match self {
            Self::CreateOnly => Ok(()),
            Self::Exact {
                revision,
                revision_commitment,
            } => {
                if *revision == 0 {
                    return Err(violation("expected prior revision must be >= 1"));
                }
                require_commitment("expected prior revision commitment", revision_commitment)
            }
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ParameterMutationRequest {
    pub schema_version: u16,
    pub authority: ParameterAuthorityBinding,
    pub parameter_name: String,
    pub value: CanonicalParameterValue,
    pub expected_prior: PriorRevisionExpectation,
    pub request_commitment: String,
}

impl ParameterMutationRequest {
    pub fn new(
        authority: ParameterAuthorityBinding,
        parameter_name: impl Into<String>,
        value: CanonicalParameterValue,
        expected_prior: PriorRevisionExpectation,
    ) -> ParameterResult<Self> {
        let mut request = Self {
            schema_version: PARAMETER_PROVIDER_SCHEMA_VERSION,
            authority,
            parameter_name: parameter_name.into(),
            value,
            expected_prior,
            request_commitment: String::new(),
        };
        request.validate_without_commitment()?;
        request.request_commitment = request.compute_commitment()?;
        Ok(request)
    }

    fn validate_without_commitment(&self) -> ParameterResult<()> {
        if self.schema_version != PARAMETER_PROVIDER_SCHEMA_VERSION {
            return Err(violation(format!(
                "unsupported parameter provider schema version {}",
                self.schema_version
            )));
        }
        self.authority.validate()?;
        require_opaque(
            "parameter_name",
            &self.parameter_name,
            MAX_PARAMETER_NAME_LEN,
        )?;
        if self.parameter_name.trim() != self.parameter_name {
            return Err(violation("parameter_name may not have surrounding whitespace"));
        }
        self.value.validate()?;
        self.expected_prior.validate()?;
        Ok(())
    }

    pub fn validate(&self) -> ParameterResult<()> {
        self.validate_without_commitment()?;
        require_commitment("request_commitment", &self.request_commitment)?;
        let expected = self.compute_commitment()?;
        if expected != self.request_commitment {
            return Err(violation(
                "request_commitment does not match constitutional parameter request semantics",
            ));
        }
        Ok(())
    }

    fn compute_commitment(&self) -> ParameterResult<String> {
        self.validate_without_commitment()?;
        Ok(compute_request_commitment(
            self.schema_version,
            &self.authority,
            &self.parameter_name,
            &self.value,
            &self.expected_prior,
        )?)
    }

    pub fn provider_key(&self) -> &str {
        &self.authority.action_id
    }
}

fn compute_request_commitment(
    schema_version: u16,
    authority: &ParameterAuthorityBinding,
    parameter_name: &str,
    value: &CanonicalParameterValue,
    expected_prior: &PriorRevisionExpectation,
) -> ParameterResult<String> {
    authority.validate()?;
    value.validate()?;
    expected_prior.validate()?;
    let mut hasher = blake3::Hasher::new();
    hasher.update(REQUEST_DOMAIN);
    hasher.update(&schema_version.to_be_bytes());
    push_str(&mut hasher, &authority.operation_id);
    push_str(&mut hasher, &authority.action_id);
    push_str(&mut hasher, &authority.proposal_id);
    push_str(&mut hasher, &authority.claim_binding_commitment);
    push_str(&mut hasher, &authority.action_commitment);
    push_str(&mut hasher, &authority.publisher_did);
    push_str(&mut hasher, parameter_name);
    push_str(&mut hasher, value.value_type().tag());
    push_str(&mut hasher, &value.canonical_json()?);
    match expected_prior {
        PriorRevisionExpectation::CreateOnly => push_str(&mut hasher, "create-only"),
        PriorRevisionExpectation::Exact {
            revision,
            revision_commitment,
        } => {
            push_str(&mut hasher, "exact-prior");
            hasher.update(&revision.to_be_bytes());
            push_str(&mut hasher, revision_commitment);
        }
    }
    Ok(tagged_hash(hasher.finalize()))
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ParameterRevision {
    pub schema_version: u16,
    pub authority: ParameterAuthorityBinding,
    pub parameter_name: String,
    pub value: CanonicalParameterValue,
    pub revision: u64,
    pub prior_revision_commitment: Option<String>,
    pub request_commitment: String,
    pub revision_commitment: String,
    /// Diagnostic/audit time only. It is deliberately excluded from identity.
    pub committed_at_unix_ms: u64,
}

impl ParameterRevision {
    fn from_request(
        request: &ParameterMutationRequest,
        revision: u64,
        prior_revision_commitment: Option<String>,
        committed_at_unix_ms: u64,
    ) -> ParameterResult<Self> {
        request.validate()?;
        if revision == 0 {
            return Err(violation("parameter revision must be >= 1"));
        }
        if revision == 1 && prior_revision_commitment.is_some() {
            return Err(violation("first parameter revision cannot have a predecessor"));
        }
        if revision > 1 && prior_revision_commitment.is_none() {
            return Err(violation(
                "non-first parameter revision must bind its predecessor",
            ));
        }
        let mut result = Self {
            schema_version: PARAMETER_PROVIDER_SCHEMA_VERSION,
            authority: request.authority.clone(),
            parameter_name: request.parameter_name.clone(),
            value: request.value.clone(),
            revision,
            prior_revision_commitment,
            request_commitment: request.request_commitment.clone(),
            revision_commitment: String::new(),
            committed_at_unix_ms,
        };
        result.revision_commitment = result.compute_commitment()?;
        result.validate()?;
        Ok(result)
    }

    fn implied_request_expectation(&self) -> ParameterResult<PriorRevisionExpectation> {
        match (&self.prior_revision_commitment, self.revision) {
            (None, 1) => Ok(PriorRevisionExpectation::CreateOnly),
            (Some(commitment), revision) if revision > 1 => {
                require_commitment("prior_revision_commitment", commitment)?;
                Ok(PriorRevisionExpectation::Exact {
                    revision: revision - 1,
                    revision_commitment: commitment.clone(),
                })
            }
            _ => Err(violation(
                "parameter predecessor shape does not match revision number",
            )),
        }
    }

    pub fn validate(&self) -> ParameterResult<()> {
        if self.schema_version != PARAMETER_PROVIDER_SCHEMA_VERSION {
            return Err(violation(format!(
                "unsupported parameter revision schema version {}",
                self.schema_version
            )));
        }
        self.authority.validate()?;
        require_opaque(
            "parameter revision name",
            &self.parameter_name,
            MAX_PARAMETER_NAME_LEN,
        )?;
        self.value.validate()?;
        if self.revision == 0 {
            return Err(violation("parameter revision must be >= 1"));
        }
        let implied_expectation = self.implied_request_expectation()?;
        require_commitment("request_commitment", &self.request_commitment)?;
        let reconstructed_request = compute_request_commitment(
            self.schema_version,
            &self.authority,
            &self.parameter_name,
            &self.value,
            &implied_expectation,
        )?;
        if reconstructed_request != self.request_commitment {
            return Err(violation(
                "stored request_commitment does not reconstruct from revision semantics",
            ));
        }
        require_commitment("revision_commitment", &self.revision_commitment)?;
        let expected = self.compute_commitment()?;
        if expected != self.revision_commitment {
            return Err(violation(
                "revision_commitment does not match constitutional parameter revision semantics",
            ));
        }
        Ok(())
    }

    fn compute_commitment(&self) -> ParameterResult<String> {
        self.authority.validate()?;
        self.value.validate()?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(REVISION_DOMAIN);
        hasher.update(&self.schema_version.to_be_bytes());
        push_str(&mut hasher, &self.parameter_name);
        hasher.update(&self.revision.to_be_bytes());
        match &self.prior_revision_commitment {
            Some(commitment) => push_str(&mut hasher, commitment),
            None => push_str(&mut hasher, "<genesis>"),
        }
        push_str(&mut hasher, &self.request_commitment);
        push_str(&mut hasher, &self.authority.operation_id);
        push_str(&mut hasher, &self.authority.action_id);
        push_str(&mut hasher, &self.authority.proposal_id);
        push_str(&mut hasher, &self.authority.claim_binding_commitment);
        push_str(&mut hasher, &self.authority.action_commitment);
        push_str(&mut hasher, &self.authority.publisher_did);
        push_str(&mut hasher, self.value.value_type().tag());
        push_str(&mut hasher, &self.value.canonical_json()?);
        Ok(tagged_hash(hasher.finalize()))
    }

    pub fn provider_key(&self) -> &str {
        &self.authority.action_id
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum ParameterIntegrityFault {
    ActionIdentityCollision {
        action_id: String,
        existing_revision: u64,
        existing_request_commitment: String,
        candidate_request_commitment: String,
    },
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum ParameterApplyDecision {
    Committed {
        revision: ParameterRevision,
    },
    ExistingSame {
        action_id: String,
        revision: u64,
        revision_commitment: String,
    },
    StaleRevision {
        current_revision: Option<u64>,
        current_revision_commitment: Option<String>,
    },
    IntegrityConflict {
        action_id: String,
        existing_revision: u64,
        existing_request_commitment: String,
        candidate_request_commitment: String,
    },
}

/// Pure append-only parameter history for one parameter name.
///
/// `action_index` is historical, not merely current-state metadata. A delayed
/// retry of an old action therefore resolves to the exact revision originally
/// created by that action even after later valid revisions exist.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ParameterHistory {
    pub parameter_name: String,
    pub revisions: BTreeMap<u64, ParameterRevision>,
    pub action_index: BTreeMap<String, u64>,
    pub integrity_fault: Option<ParameterIntegrityFault>,
}

impl ParameterHistory {
    pub fn new(parameter_name: impl Into<String>) -> ParameterResult<Self> {
        let parameter_name = parameter_name.into();
        require_opaque(
            "parameter history name",
            &parameter_name,
            MAX_PARAMETER_NAME_LEN,
        )?;
        if parameter_name.trim() != parameter_name {
            return Err(violation(
                "parameter history name may not have surrounding whitespace",
            ));
        }
        Ok(Self {
            parameter_name,
            revisions: BTreeMap::new(),
            action_index: BTreeMap::new(),
            integrity_fault: None,
        })
    }

    pub fn current(&self) -> Option<&ParameterRevision> {
        self.revisions.last_key_value().map(|(_, revision)| revision)
    }

    pub fn revision(&self, revision: u64) -> Option<&ParameterRevision> {
        self.revisions.get(&revision)
    }

    pub fn by_action(&self, action_id: &str) -> Option<&ParameterRevision> {
        self.action_index
            .get(action_id)
            .and_then(|revision| self.revisions.get(revision))
    }

    pub fn apply(
        &mut self,
        request: ParameterMutationRequest,
        committed_at_unix_ms: u64,
    ) -> ParameterResult<ParameterApplyDecision> {
        request.validate()?;
        if request.parameter_name != self.parameter_name {
            return Err(violation(
                "parameter request name does not match this parameter history",
            ));
        }
        if let Some(fault) = &self.integrity_fault {
            return Err(ParameterProviderError::IntegrityHalted(format!(
                "{:?}",
                fault
            )));
        }

        if let Some(existing_revision_number) = self.action_index.get(request.provider_key()) {
            let existing = self
                .revisions
                .get(existing_revision_number)
                .ok_or_else(|| violation("action index references missing parameter revision"))?;
            let existing_revision = existing.revision;
            let existing_request_commitment = existing.request_commitment.clone();
            let existing_revision_commitment = existing.revision_commitment.clone();

            if existing_request_commitment == request.request_commitment {
                return Ok(ParameterApplyDecision::ExistingSame {
                    action_id: request.authority.action_id,
                    revision: existing_revision,
                    revision_commitment: existing_revision_commitment,
                });
            }

            let candidate_request_commitment = request.request_commitment.clone();
            self.integrity_fault = Some(ParameterIntegrityFault::ActionIdentityCollision {
                action_id: request.authority.action_id.clone(),
                existing_revision,
                existing_request_commitment: existing_request_commitment.clone(),
                candidate_request_commitment: candidate_request_commitment.clone(),
            });
            return Ok(ParameterApplyDecision::IntegrityConflict {
                action_id: request.authority.action_id,
                existing_revision,
                existing_request_commitment,
                candidate_request_commitment,
            });
        }

        let current = self.current().cloned();
        let predecessor_matches = match (&current, &request.expected_prior) {
            (None, PriorRevisionExpectation::CreateOnly) => true,
            (
                Some(current),
                PriorRevisionExpectation::Exact {
                    revision,
                    revision_commitment,
                },
            ) => {
                *revision == current.revision
                    && revision_commitment == &current.revision_commitment
            }
            _ => false,
        };

        if !predecessor_matches {
            return Ok(ParameterApplyDecision::StaleRevision {
                current_revision: current.as_ref().map(|revision| revision.revision),
                current_revision_commitment: current
                    .as_ref()
                    .map(|revision| revision.revision_commitment.clone()),
            });
        }

        let next_revision = match current.as_ref() {
            Some(revision) => revision
                .revision
                .checked_add(1)
                .ok_or_else(|| violation("parameter revision overflow"))?,
            None => 1,
        };
        let prior_commitment = current
            .as_ref()
            .map(|revision| revision.revision_commitment.clone());
        let revision = ParameterRevision::from_request(
            &request,
            next_revision,
            prior_commitment,
            committed_at_unix_ms,
        )?;

        self.action_index
            .insert(request.authority.action_id.clone(), next_revision);
        self.revisions.insert(next_revision, revision.clone());
        self.validate_invariants()?;

        Ok(ParameterApplyDecision::Committed { revision })
    }

    pub fn validate_invariants(&self) -> ParameterResult<()> {
        require_opaque(
            "parameter history name",
            &self.parameter_name,
            MAX_PARAMETER_NAME_LEN,
        )?;
        if self.revisions.len() != self.action_index.len() {
            return Err(violation(
                "every parameter revision must have exactly one historical action index",
            ));
        }

        let mut seen_actions = BTreeSet::new();
        let mut previous: Option<&ParameterRevision> = None;
        let mut expected_revision = 1u64;

        for (key, revision) in &self.revisions {
            revision.validate()?;
            if *key != expected_revision || revision.revision != expected_revision {
                return Err(violation(
                    "parameter revisions must be contiguous from revision 1",
                ));
            }
            if revision.parameter_name != self.parameter_name {
                return Err(violation(
                    "parameter revision belongs to a different parameter history",
                ));
            }
            match previous {
                None => {
                    if revision.prior_revision_commitment.is_some() {
                        return Err(violation("revision 1 must not bind a predecessor"));
                    }
                }
                Some(previous) => {
                    if revision.prior_revision_commitment.as_deref()
                        != Some(previous.revision_commitment.as_str())
                    {
                        return Err(violation(
                            "parameter revision predecessor commitment does not chain",
                        ));
                    }
                }
            }
            if !seen_actions.insert(revision.authority.action_id.as_str()) {
                return Err(violation(
                    "one constitutional action identity cannot own multiple parameter revisions",
                ));
            }
            if self.action_index.get(&revision.authority.action_id) != Some(key) {
                return Err(violation(
                    "historical action index does not point to its exact parameter revision",
                ));
            }
            previous = Some(revision);
            expected_revision = expected_revision
                .checked_add(1)
                .ok_or_else(|| violation("parameter revision invariant counter overflow"))?;
        }

        for (action_id, revision_number) in &self.action_index {
            let revision = self
                .revisions
                .get(revision_number)
                .ok_or_else(|| violation("action index points to a missing parameter revision"))?;
            if &revision.authority.action_id != action_id {
                return Err(violation(
                    "action index key does not match parameter revision action identity",
                ));
            }
        }
        Ok(())
    }
}

fn tagged_hash(hash: blake3::Hash) -> String {
    format!("{COMMITMENT_PREFIX}{}", hash.to_hex())
}

fn push_str(hasher: &mut blake3::Hasher, value: &str) {
    push_bytes(hasher, value.as_bytes());
}

fn push_bytes(hasher: &mut blake3::Hasher, value: &[u8]) {
    hasher.update(&(value.len() as u64).to_be_bytes());
    hasher.update(value);
}

#[cfg(test)]
mod tests {
    use super::*;

    fn authority(action_id: &str) -> ParameterAuthorityBinding {
        ParameterAuthorityBinding {
            operation_id: "operation-001".into(),
            action_id: action_id.into(),
            proposal_id: "proposal-003".into(),
            claim_binding_commitment: "claim-binding:opaque-b4-pending".into(),
            action_commitment: format!("action-commitment:{action_id}"),
            publisher_did: "did:mycelix:test-publisher".into(),
        }
    }

    fn create_request(action_id: &str, value: CanonicalParameterValue) -> ParameterMutationRequest {
        ParameterMutationRequest::new(
            authority(action_id),
            "quorum_threshold",
            value,
            PriorRevisionExpectation::CreateOnly,
        )
        .unwrap()
    }

    fn advance_request(
        action_id: &str,
        value: CanonicalParameterValue,
        current: &ParameterRevision,
    ) -> ParameterMutationRequest {
        ParameterMutationRequest::new(
            authority(action_id),
            "quorum_threshold",
            value,
            PriorRevisionExpectation::Exact {
                revision: current.revision,
                revision_commitment: current.revision_commitment.clone(),
            },
        )
        .unwrap()
    }

    #[test]
    fn integer_input_is_canonicalized() {
        assert_eq!(
            CanonicalParameterValue::from_typed_input(ParameterValueType::Integer, "  -00042 ")
                .unwrap(),
            CanonicalParameterValue::Integer(-42)
        );
        assert_eq!(
            CanonicalParameterValue::from_typed_input(ParameterValueType::Integer, "-000")
                .unwrap(),
            CanonicalParameterValue::Integer(0)
        );
    }

    #[test]
    fn decimal_input_has_one_canonical_spelling() {
        assert_eq!(normalize_decimal("001.2300").unwrap(), "1.23");
        assert_eq!(normalize_decimal("-000.000").unwrap(), "0");
        assert_eq!(normalize_decimal("0005").unwrap(), "5");
        assert!(normalize_decimal("1e3").is_err());
        assert!(normalize_decimal("+1.2").is_err());
    }

    #[test]
    fn decimal_and_string_do_not_share_value_identity() {
        let decimal = CanonicalParameterValue::Decimal("1.25".into());
        let string = CanonicalParameterValue::String("1.25".into());
        assert_ne!(decimal.commitment().unwrap(), string.commitment().unwrap());
    }

    #[test]
    fn strings_are_json_escaped_deterministically() {
        let value = CanonicalParameterValue::String("line\n\"quoted\"".into());
        assert_eq!(value.canonical_json().unwrap(), "\"line\\n\\\"quoted\\\"\"");
    }

    #[test]
    fn first_revision_requires_create_only() {
        let mut history = ParameterHistory::new("quorum_threshold").unwrap();
        let bogus = ParameterMutationRequest::new(
            authority("action-1"),
            "quorum_threshold",
            CanonicalParameterValue::Percentage("0.67".into()),
            PriorRevisionExpectation::Exact {
                revision: 1,
                revision_commitment: format!("{COMMITMENT_PREFIX}{}", "0".repeat(64)),
            },
        )
        .unwrap();
        assert!(matches!(
            history.apply(bogus, 10).unwrap(),
            ParameterApplyDecision::StaleRevision {
                current_revision: None,
                ..
            }
        ));
    }

    #[test]
    fn first_valid_action_commits_revision_one() {
        let mut history = ParameterHistory::new("quorum_threshold").unwrap();
        let request = create_request(
            "action-1",
            CanonicalParameterValue::Percentage("0.67".into()),
        );
        let revision = match history.apply(request, 100).unwrap() {
            ParameterApplyDecision::Committed { revision } => revision,
            other => panic!("unexpected {other:?}"),
        };
        assert_eq!(revision.revision, 1);
        assert!(revision.prior_revision_commitment.is_none());
        assert_eq!(history.by_action("action-1"), Some(&revision));
    }

    #[test]
    fn same_action_same_request_is_historically_idempotent() {
        let mut history = ParameterHistory::new("quorum_threshold").unwrap();
        let first_request = create_request(
            "action-1",
            CanonicalParameterValue::Percentage("0.67".into()),
        );
        let first = match history.apply(first_request.clone(), 100).unwrap() {
            ParameterApplyDecision::Committed { revision } => revision,
            other => panic!("unexpected {other:?}"),
        };
        let retry = history.apply(first_request, 999).unwrap();
        assert!(matches!(
            retry,
            ParameterApplyDecision::ExistingSame {
                revision: 1,
                revision_commitment,
                ..
            } if revision_commitment == first.revision_commitment
        ));
        assert_eq!(history.revisions.len(), 1);
    }

    #[test]
    fn delayed_retry_still_resolves_after_later_revisions() {
        let mut history = ParameterHistory::new("quorum_threshold").unwrap();
        let action_a = create_request(
            "action-a",
            CanonicalParameterValue::Percentage("0.60".into()),
        );
        let r1 = match history.apply(action_a.clone(), 100).unwrap() {
            ParameterApplyDecision::Committed { revision } => revision,
            other => panic!("unexpected {other:?}"),
        };
        let action_b = advance_request(
            "action-b",
            CanonicalParameterValue::Percentage("0.67".into()),
            &r1,
        );
        let r2 = match history.apply(action_b, 200).unwrap() {
            ParameterApplyDecision::Committed { revision } => revision,
            other => panic!("unexpected {other:?}"),
        };
        assert_eq!(r2.revision, 2);
        assert!(matches!(
            history.apply(action_a, 999).unwrap(),
            ParameterApplyDecision::ExistingSame { revision: 1, .. }
        ));
        assert_eq!(history.current().unwrap().revision, 2);
    }

    #[test]
    fn same_action_different_payload_is_integrity_conflict_and_halts() {
        let mut history = ParameterHistory::new("quorum_threshold").unwrap();
        history
            .apply(
                create_request(
                    "action-1",
                    CanonicalParameterValue::Percentage("0.67".into()),
                ),
                100,
            )
            .unwrap();
        assert!(matches!(
            history
                .apply(
                    create_request(
                        "action-1",
                        CanonicalParameterValue::Percentage("0.75".into()),
                    ),
                    200,
                )
                .unwrap(),
            ParameterApplyDecision::IntegrityConflict { .. }
        ));
        assert!(history.integrity_fault.is_some());

        let later = ParameterMutationRequest::new(
            authority("action-2"),
            "quorum_threshold",
            CanonicalParameterValue::Percentage("0.70".into()),
            PriorRevisionExpectation::Exact {
                revision: 1,
                revision_commitment: history.current().unwrap().revision_commitment.clone(),
            },
        )
        .unwrap();
        assert!(matches!(
            history.apply(later, 300),
            Err(ParameterProviderError::IntegrityHalted(_))
        ));
    }

    #[test]
    fn stale_compare_and_set_does_not_mutate_history() {
        let mut history = ParameterHistory::new("quorum_threshold").unwrap();
        let r1 = match history
            .apply(
                create_request(
                    "action-1",
                    CanonicalParameterValue::Percentage("0.60".into()),
                ),
                100,
            )
            .unwrap()
        {
            ParameterApplyDecision::Committed { revision } => revision,
            other => panic!("unexpected {other:?}"),
        };
        let r2 = match history
            .apply(
                advance_request(
                    "action-2",
                    CanonicalParameterValue::Percentage("0.67".into()),
                    &r1,
                ),
                200,
            )
            .unwrap()
        {
            ParameterApplyDecision::Committed { revision } => revision,
            other => panic!("unexpected {other:?}"),
        };
        let stale = advance_request(
            "action-3",
            CanonicalParameterValue::Percentage("0.75".into()),
            &r1,
        );
        assert!(matches!(
            history.apply(stale, 300).unwrap(),
            ParameterApplyDecision::StaleRevision {
                current_revision: Some(2),
                ..
            }
        ));
        assert_eq!(
            history.current().unwrap().revision_commitment,
            r2.revision_commitment
        );
        assert!(history.by_action("action-3").is_none());
    }

    #[test]
    fn correct_compare_and_set_builds_hash_chain() {
        let mut history = ParameterHistory::new("quorum_threshold").unwrap();
        let r1 = match history
            .apply(
                create_request(
                    "action-1",
                    CanonicalParameterValue::Percentage("0.60".into()),
                ),
                100,
            )
            .unwrap()
        {
            ParameterApplyDecision::Committed { revision } => revision,
            other => panic!("unexpected {other:?}"),
        };
        let r2 = match history
            .apply(
                advance_request(
                    "action-2",
                    CanonicalParameterValue::Percentage("0.67".into()),
                    &r1,
                ),
                200,
            )
            .unwrap()
        {
            ParameterApplyDecision::Committed { revision } => revision,
            other => panic!("unexpected {other:?}"),
        };
        assert_eq!(
            r2.prior_revision_commitment.as_deref(),
            Some(r1.revision_commitment.as_str())
        );
        history.validate_invariants().unwrap();
    }

    #[test]
    fn revision_reconstructs_request_commitment() {
        let mut history = ParameterHistory::new("quorum_threshold").unwrap();
        let revision = match history
            .apply(
                create_request(
                    "action-1",
                    CanonicalParameterValue::Percentage("0.67".into()),
                ),
                100,
            )
            .unwrap()
        {
            ParameterApplyDecision::Committed { revision } => revision,
            other => panic!("unexpected {other:?}"),
        };
        let mut tampered = revision;
        tampered.request_commitment = format!("{COMMITMENT_PREFIX}{}", "a".repeat(64));
        tampered.revision_commitment = tampered.compute_commitment().unwrap();
        assert!(tampered.validate().is_err());
    }

    #[test]
    fn wall_clock_commit_time_is_not_revision_identity() {
        let request = create_request(
            "action-1",
            CanonicalParameterValue::Percentage("0.67".into()),
        );
        let a = ParameterRevision::from_request(&request, 1, None, 100).unwrap();
        let b = ParameterRevision::from_request(&request, 1, None, 999_999).unwrap();
        assert_eq!(a.revision_commitment, b.revision_commitment);
    }

    #[test]
    fn proposal_and_claim_binding_participate_in_request_identity() {
        let baseline = create_request(
            "action-1",
            CanonicalParameterValue::Percentage("0.67".into()),
        );
        let mut proposal = authority("action-1");
        proposal.proposal_id = "proposal-other".into();
        let proposal = ParameterMutationRequest::new(
            proposal,
            "quorum_threshold",
            CanonicalParameterValue::Percentage("0.67".into()),
            PriorRevisionExpectation::CreateOnly,
        )
        .unwrap();
        let mut claim = authority("action-1");
        claim.claim_binding_commitment = "claim-binding:different".into();
        let claim = ParameterMutationRequest::new(
            claim,
            "quorum_threshold",
            CanonicalParameterValue::Percentage("0.67".into()),
            PriorRevisionExpectation::CreateOnly,
        )
        .unwrap();
        assert_ne!(baseline.request_commitment, proposal.request_commitment);
        assert_ne!(baseline.request_commitment, claim.request_commitment);
    }

    #[test]
    fn prior_revision_precondition_participates_in_request_identity() {
        let value = CanonicalParameterValue::Integer(42);
        let create = ParameterMutationRequest::new(
            authority("action-1"),
            "limit",
            value.clone(),
            PriorRevisionExpectation::CreateOnly,
        )
        .unwrap();
        let exact = ParameterMutationRequest::new(
            authority("action-1"),
            "limit",
            value,
            PriorRevisionExpectation::Exact {
                revision: 1,
                revision_commitment: format!("{COMMITMENT_PREFIX}{}", "1".repeat(64)),
            },
        )
        .unwrap();
        assert_ne!(create.request_commitment, exact.request_commitment);
    }

    #[test]
    fn malformed_or_uppercase_commitments_are_rejected() {
        let mut request = create_request("action-1", CanonicalParameterValue::Integer(1));
        request.request_commitment = format!("{COMMITMENT_PREFIX}{}", "A".repeat(64));
        assert!(request.validate().is_err());
    }

    #[test]
    fn action_index_drift_is_detected() {
        let mut history = ParameterHistory::new("quorum_threshold").unwrap();
        history
            .apply(
                create_request(
                    "action-1",
                    CanonicalParameterValue::Percentage("0.67".into()),
                ),
                100,
            )
            .unwrap();
        history.action_index.insert("action-1".into(), 99);
        assert!(history.validate_invariants().is_err());
    }

    #[test]
    fn broken_predecessor_chain_is_detected() {
        let mut history = ParameterHistory::new("quorum_threshold").unwrap();
        let r1 = match history
            .apply(
                create_request(
                    "action-1",
                    CanonicalParameterValue::Percentage("0.60".into()),
                ),
                100,
            )
            .unwrap()
        {
            ParameterApplyDecision::Committed { revision } => revision,
            other => panic!("unexpected {other:?}"),
        };
        history
            .apply(
                advance_request(
                    "action-2",
                    CanonicalParameterValue::Percentage("0.67".into()),
                    &r1,
                ),
                200,
            )
            .unwrap();
        history.revisions.get_mut(&2).unwrap().prior_revision_commitment = Some(format!(
            "{COMMITMENT_PREFIX}{}",
            "f".repeat(64)
        ));
        assert!(history.validate_invariants().is_err());
    }
}
