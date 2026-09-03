use std::collections::BTreeSet;

use mycelix_infrastructure_types::{PartyIdV1, UnixMillisV1};
use serde::{Deserialize, Serialize};

use crate::{
    canonical::{append_field, record_id},
    ContentErrorV1, EncryptionRequirementV1, FailureDomainPolicyV1, ObjectIdV1,
    CONTENT_SCHEMA_V1,
};

const MAX_JURISDICTIONS_V1: usize = 128;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum StorageClassV1 {
    EdgeCache,
    ReplicatedHot,
    Durable,
    Archive,
    PrivateBackup,
}

impl StorageClassV1 {
    fn tag(self) -> u8 {
        match self {
            Self::EdgeCache => 0,
            Self::ReplicatedHot => 1,
            Self::Durable => 2,
            Self::Archive => 3,
            Self::PrivateBackup => 4,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct JurisdictionV1(String);

impl JurisdictionV1 {
    pub fn new(value: impl Into<String>) -> Result<Self, ContentErrorV1> {
        let value = value.into();
        let candidate = Self(value);
        candidate.validate()?;
        Ok(candidate)
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }

    pub fn validate(&self) -> Result<(), ContentErrorV1> {
        let bytes = self.0.as_bytes();
        if bytes.len() < 2
            || bytes.len() > 32
            || !bytes[0].is_ascii_uppercase()
            || bytes
                .iter()
                .any(|b| !(b.is_ascii_uppercase() || b.is_ascii_digit() || *b == b'-'))
        {
            return Err(ContentErrorV1::InvalidJurisdiction);
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum RetentionRequirementV1 {
    BestEffort,
    MinimumSeconds(u64),
    Until(UnixMillisV1),
    Indefinite,
}

impl RetentionRequirementV1 {
    pub fn validate(self) -> Result<(), ContentErrorV1> {
        if matches!(self, Self::MinimumSeconds(0)) {
            return Err(ContentErrorV1::ZeroRetentionDuration);
        }
        Ok(())
    }

    fn canonical_bytes(self) -> Vec<u8> {
        match self {
            Self::BestEffort => vec![0],
            Self::MinimumSeconds(seconds) => {
                let mut out = vec![1];
                out.extend_from_slice(&seconds.to_be_bytes());
                out
            }
            Self::Until(until) => {
                let mut out = vec![2];
                out.extend_from_slice(&until.0.to_be_bytes());
                out
            }
            Self::Indefinite => vec![3],
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct PlacementRequirementsV1 {
    pub minimum_replicas: u16,
    pub failure_domains: FailureDomainPolicyV1,
    pub allowed_jurisdictions: BTreeSet<JurisdictionV1>,
    pub forbidden_jurisdictions: BTreeSet<JurisdictionV1>,
    pub encryption: EncryptionRequirementV1,
    pub retention: RetentionRequirementV1,
}

impl PlacementRequirementsV1 {
    pub fn new(
        minimum_replicas: u16,
        failure_domains: FailureDomainPolicyV1,
        allowed_jurisdictions: BTreeSet<JurisdictionV1>,
        forbidden_jurisdictions: BTreeSet<JurisdictionV1>,
        encryption: EncryptionRequirementV1,
        retention: RetentionRequirementV1,
    ) -> Result<Self, ContentErrorV1> {
        let value = Self {
            minimum_replicas,
            failure_domains,
            allowed_jurisdictions,
            forbidden_jurisdictions,
            encryption,
            retention,
        };
        value.validate()?;
        Ok(value)
    }

    pub fn validate(&self) -> Result<(), ContentErrorV1> {
        if self.minimum_replicas == 0 {
            return Err(ContentErrorV1::ZeroReplicas);
        }
        self.failure_domains
            .validate_against_replicas(self.minimum_replicas)?;
        if self.allowed_jurisdictions.len() > MAX_JURISDICTIONS_V1
            || self.forbidden_jurisdictions.len() > MAX_JURISDICTIONS_V1
        {
            return Err(ContentErrorV1::TooManyJurisdictions);
        }
        for jurisdiction in self
            .allowed_jurisdictions
            .iter()
            .chain(self.forbidden_jurisdictions.iter())
        {
            jurisdiction.validate()?;
        }
        if self
            .allowed_jurisdictions
            .iter()
            .any(|j| self.forbidden_jurisdictions.contains(j))
        {
            return Err(ContentErrorV1::JurisdictionConflict);
        }
        self.retention.validate()?;
        Ok(())
    }

    /// Evaluate jurisdiction as a hard eligibility constraint before optimization.
    pub fn jurisdiction_allowed(&self, jurisdiction: &JurisdictionV1) -> bool {
        if self.forbidden_jurisdictions.contains(jurisdiction) {
            return false;
        }
        self.allowed_jurisdictions.is_empty() || self.allowed_jurisdictions.contains(jurisdiction)
    }

    fn canonical_bytes(&self) -> Vec<u8> {
        let mut out = Vec::new();
        out.extend_from_slice(&self.minimum_replicas.to_be_bytes());
        append_field(&mut out, &self.failure_domains.canonical_bytes());
        append_jurisdictions(&mut out, &self.allowed_jurisdictions);
        append_jurisdictions(&mut out, &self.forbidden_jurisdictions);
        out.push(self.encryption.tag());
        append_field(&mut out, &self.retention.canonical_bytes());
        out
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct PlacementPreferencesV1 {
    pub target_latency_ms: Option<u32>,
    pub cost_weight: u16,
    pub latency_weight: u16,
    pub energy_weight: u16,
    pub locality_weight: u16,
}

impl PlacementPreferencesV1 {
    pub fn new(
        target_latency_ms: Option<u32>,
        cost_weight: u16,
        latency_weight: u16,
        energy_weight: u16,
        locality_weight: u16,
    ) -> Result<Self, ContentErrorV1> {
        let value = Self {
            target_latency_ms,
            cost_weight,
            latency_weight,
            energy_weight,
            locality_weight,
        };
        value.validate()?;
        Ok(value)
    }

    pub fn validate(&self) -> Result<(), ContentErrorV1> {
        if self.target_latency_ms == Some(0) {
            return Err(ContentErrorV1::InvalidLatencyTarget);
        }
        Ok(())
    }

    fn canonical_bytes(self) -> Vec<u8> {
        let mut out = Vec::new();
        match self.target_latency_ms {
            Some(value) => {
                out.push(1);
                out.extend_from_slice(&value.to_be_bytes());
            }
            None => out.push(0),
        }
        out.extend_from_slice(&self.cost_weight.to_be_bytes());
        out.extend_from_slice(&self.latency_weight.to_be_bytes());
        out.extend_from_slice(&self.energy_weight.to_be_bytes());
        out.extend_from_slice(&self.locality_weight.to_be_bytes());
        out
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct StorageIntentIdV1(pub [u8; 32]);

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct StorageIntentV1 {
    pub schema_version: u16,
    pub id: StorageIntentIdV1,
    pub object_id: ObjectIdV1,
    pub owner: PartyIdV1,
    pub class: StorageClassV1,
    pub requirements: PlacementRequirementsV1,
    pub preferences: PlacementPreferencesV1,
    pub created_at: UnixMillisV1,
}

impl StorageIntentV1 {
    pub fn new(
        object_id: ObjectIdV1,
        owner: PartyIdV1,
        class: StorageClassV1,
        requirements: PlacementRequirementsV1,
        preferences: PlacementPreferencesV1,
        created_at: UnixMillisV1,
    ) -> Result<Self, ContentErrorV1> {
        validate_intent_fields(class, &requirements, &preferences)?;
        let id = storage_intent_id(
            object_id,
            owner,
            class,
            &requirements,
            preferences,
            created_at,
        );
        Ok(Self {
            schema_version: CONTENT_SCHEMA_V1,
            id,
            object_id,
            owner,
            class,
            requirements,
            preferences,
            created_at,
        })
    }

    pub fn validate(&self) -> Result<(), ContentErrorV1> {
        if self.schema_version != CONTENT_SCHEMA_V1 {
            return Err(ContentErrorV1::UnsupportedSchemaVersion);
        }
        validate_intent_fields(self.class, &self.requirements, &self.preferences)?;
        if self.id != self.recompute_id()? {
            return Err(ContentErrorV1::IdMismatch);
        }
        Ok(())
    }

    pub fn recompute_id(&self) -> Result<StorageIntentIdV1, ContentErrorV1> {
        validate_intent_fields(self.class, &self.requirements, &self.preferences)?;
        Ok(storage_intent_id(
            self.object_id,
            self.owner,
            self.class,
            &self.requirements,
            self.preferences,
            self.created_at,
        ))
    }
}

fn validate_intent_fields(
    class: StorageClassV1,
    requirements: &PlacementRequirementsV1,
    preferences: &PlacementPreferencesV1,
) -> Result<(), ContentErrorV1> {
    requirements.validate()?;
    preferences.validate()?;
    if class == StorageClassV1::PrivateBackup && !requirements.encryption.requires_client_side() {
        return Err(ContentErrorV1::PrivateBackupRequiresClientEncryption);
    }
    Ok(())
}

fn append_jurisdictions(out: &mut Vec<u8>, values: &BTreeSet<JurisdictionV1>) {
    out.extend_from_slice(&(values.len() as u32).to_be_bytes());
    for value in values {
        append_field(out, value.as_str().as_bytes());
    }
}

fn storage_intent_id(
    object_id: ObjectIdV1,
    owner: PartyIdV1,
    class: StorageClassV1,
    requirements: &PlacementRequirementsV1,
    preferences: PlacementPreferencesV1,
    created_at: UnixMillisV1,
) -> StorageIntentIdV1 {
    let mut body = Vec::new();
    body.extend_from_slice(&object_id.0);
    body.extend_from_slice(&owner.0);
    body.push(class.tag());
    append_field(&mut body, &requirements.canonical_bytes());
    append_field(&mut body, &preferences.canonical_bytes());
    body.extend_from_slice(&created_at.0.to_be_bytes());
    StorageIntentIdV1(record_id("storage-intent", CONTENT_SCHEMA_V1, &body))
}
