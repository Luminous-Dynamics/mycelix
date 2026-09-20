use std::fmt;

use mycelix_finance_exact::AssetAmount;
use serde::{Deserialize, Deserializer, Serialize, Serializer};

pub const MAX_TEXT_BYTES: usize = 256;
pub const MIN_LEGS: usize = 2;
pub const MAX_LEGS: usize = 64;
pub const MAX_DEPENDENCIES: usize = 256;
pub const MAX_GROUPS: usize = 32;
pub const MAX_GROUP_MEMBERS: usize = 64;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum GraphError {
    InvalidText,
    InvalidCommitmentHex,
    TooFewLegs,
    TooManyLegs,
    TooManyDependencies,
    TooManyGroups,
    ZeroAmount,
    InvalidLegRoleSemantics,
    InvalidCoordinationRoleComposition,
    DuplicateLegAlias,
    DuplicateSemanticLeg,
    DuplicateSemanticIdempotencyRef,
    UnknownLegAlias,
    SelfDependency,
    DuplicateDependency,
    DuplicateGroupAlias,
    InvalidGroupSize,
    DuplicateGroupMember,
    LegInMultipleGroups,
    DuplicateSemanticGroup,
    DependencyInsideStrongGroup,
    DependencyCycle,
    DisconnectedGraph,
    CanonicalLengthOverflow,
    InternalInvariant,
}

impl fmt::Display for GraphError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::InvalidText => "text value is empty, too long, or contains control characters",
            Self::InvalidCommitmentHex => "commitment must be exactly 64 hexadecimal characters",
            Self::TooFewLegs => "settlement graph requires at least two legs",
            Self::TooManyLegs => "settlement graph exceeds the leg bound",
            Self::TooManyDependencies => "settlement graph exceeds the dependency bound",
            Self::TooManyGroups => "settlement graph exceeds the coordination-group bound",
            Self::ZeroAmount => "settlement leg amount must be non-zero",
            Self::InvalidLegRoleSemantics => {
                "settlement leg role is inconsistent with its delivery/purpose semantics"
            }
            Self::InvalidCoordinationRoleComposition => {
                "coordination-group class is inconsistent with member leg roles"
            }
            Self::DuplicateLegAlias => "construction leg aliases must be unique",
            Self::DuplicateSemanticLeg => "two construction legs resolve to the same semantic leg",
            Self::DuplicateSemanticIdempotencyRef => {
                "two graph legs reuse the same semantic idempotency reference"
            }
            Self::UnknownLegAlias => "dependency or group references an unknown leg alias",
            Self::SelfDependency => "ordinary dependency cannot reference the same semantic leg twice",
            Self::DuplicateDependency => "duplicate dependency is not admitted in v1",
            Self::DuplicateGroupAlias => "construction group aliases must be unique",
            Self::InvalidGroupSize => "coordination-group size is invalid for its v1 class",
            Self::DuplicateGroupMember => "coordination group repeats the same semantic leg",
            Self::LegInMultipleGroups => "v1 does not admit one leg in multiple coordination groups",
            Self::DuplicateSemanticGroup => "duplicate semantic coordination group",
            Self::DependencyInsideStrongGroup => {
                "ordinary dependency inside a strongly-coupled group is unsupported"
            }
            Self::DependencyCycle => {
                "ordinary dependency graph contains a residual cycle after group collapse"
            }
            Self::DisconnectedGraph => "settlement graph contains disconnected leg components",
            Self::CanonicalLengthOverflow => "canonical encoding length exceeds u32",
            Self::InternalInvariant => "internal graph invariant could not be reconstructed",
        };
        f.write_str(message)
    }
}

impl std::error::Error for GraphError {}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize)]
#[serde(transparent)]
pub struct BoundedText(String);

impl BoundedText {
    pub fn new(value: impl Into<String>) -> Result<Self, GraphError> {
        let value = value.into();
        if value.is_empty()
            || value.len() > MAX_TEXT_BYTES
            || value.chars().any(char::is_control)
        {
            return Err(GraphError::InvalidText);
        }
        Ok(Self(value))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl fmt::Display for BoundedText {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl<'de> Deserialize<'de> for BoundedText {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let value = String::deserialize(deserializer)?;
        Self::new(value).map_err(serde::de::Error::custom)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Commitment32([u8; 32]);

impl Commitment32 {
    pub const fn from_bytes(bytes: [u8; 32]) -> Self {
        Self(bytes)
    }

    pub const fn as_bytes(&self) -> &[u8; 32] {
        &self.0
    }

    pub fn from_hex(value: &str) -> Result<Self, GraphError> {
        let bytes = value.as_bytes();
        if bytes.len() != 64 {
            return Err(GraphError::InvalidCommitmentHex);
        }

        let mut out = [0_u8; 32];
        for (index, pair) in bytes.chunks_exact(2).enumerate() {
            let high = decode_hex_nibble(pair[0]).ok_or(GraphError::InvalidCommitmentHex)?;
            let low = decode_hex_nibble(pair[1]).ok_or(GraphError::InvalidCommitmentHex)?;
            out[index] = (high << 4) | low;
        }
        Ok(Self(out))
    }

    pub fn to_hex(self) -> String {
        const HEX: &[u8; 16] = b"0123456789abcdef";
        let mut out = String::with_capacity(64);
        for byte in self.0 {
            out.push(char::from(HEX[usize::from(byte >> 4)]));
            out.push(char::from(HEX[usize::from(byte & 0x0f)]));
        }
        out
    }
}

impl fmt::Display for Commitment32 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.to_hex())
    }
}

impl Serialize for Commitment32 {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        serializer.serialize_str(&self.to_hex())
    }
}

impl<'de> Deserialize<'de> for Commitment32 {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let value = String::deserialize(deserializer)?;
        Self::from_hex(&value).map_err(serde::de::Error::custom)
    }
}

fn decode_hex_nibble(value: u8) -> Option<u8> {
    match value {
        b'0'..=b'9' => Some(value - b'0'),
        b'a'..=b'f' => Some(value - b'a' + 10),
        b'A'..=b'F' => Some(value - b'A' + 10),
        _ => None,
    }
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SemanticProfileRefV1 {
    id: BoundedText,
    revision: u64,
    digest: Commitment32,
}

impl SemanticProfileRefV1 {
    pub fn new(
        id: impl Into<String>,
        revision: u64,
        digest: Commitment32,
    ) -> Result<Self, GraphError> {
        Ok(Self {
            id: BoundedText::new(id)?,
            revision,
            digest,
        })
    }

    pub fn id(&self) -> &BoundedText {
        &self.id
    }

    pub fn revision(&self) -> u64 {
        self.revision
    }

    pub fn digest(&self) -> Commitment32 {
        self.digest
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SettlementLegRoleV1 {
    /// A payment/value leg participating in PvP, DvP, or another profiled flow.
    Payment,
    /// A delivery leg for an explicitly identified non-payment asset subject.
    Delivery,
    /// A fee/tax/reserve/other profiled supporting leg.
    Auxiliary,
}

impl SettlementLegRoleV1 {
    pub(crate) const fn canonical_tag(self) -> u8 {
        match self {
            Self::Payment => 1,
            Self::Delivery => 2,
            Self::Auxiliary => 3,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SettlementLegSpecV1 {
    /// Construction-only local alias. It is intentionally excluded from authority bytes.
    pub alias: BoundedText,
    pub adapter_profile: SemanticProfileRefV1,
    pub rail: BoundedText,
    pub network: BoundedText,
    pub role: SettlementLegRoleV1,
    /// Source -> destination ordering is the authoritative direction in v1.
    pub source_subject: BoundedText,
    pub destination_subject: BoundedText,
    pub amount: AssetAmount,
    pub asset_unit_profile: SemanticProfileRefV1,
    pub required_finality_profile: SemanticProfileRefV1,
    /// Mycelix semantic replay identity, never a provider bearer secret.
    pub semantic_idempotency_ref: BoundedText,
    pub purpose_profile: Option<SemanticProfileRefV1>,
    pub delivery_asset_subject: Option<BoundedText>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields, tag = "kind", rename_all = "snake_case")]
pub enum DependencySpecV1 {
    Requires {
        leg_alias: BoundedText,
        prerequisite_alias: BoundedText,
    },
    Before {
        before_alias: BoundedText,
        after_alias: BoundedText,
    },
    ConditionalOnEvidence {
        leg_alias: BoundedText,
        predicate_profile: SemanticProfileRefV1,
    },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CoordinationGroupClassV1 {
    Pvp,
    Dvp,
    AllOrNone,
    Saga,
}

impl CoordinationGroupClassV1 {
    pub(crate) const fn canonical_tag(self) -> u8 {
        match self {
            Self::Pvp => 1,
            Self::Dvp => 2,
            Self::AllOrNone => 3,
            Self::Saga => 4,
        }
    }

    pub(crate) const fn is_strongly_coupled(self) -> bool {
        !matches!(self, Self::Saga)
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CoordinationGroupSpecV1 {
    /// Construction-only local alias. It is intentionally excluded from authority bytes.
    pub alias: BoundedText,
    pub class: CoordinationGroupClassV1,
    pub coordination_profile: SemanticProfileRefV1,
    pub member_aliases: Vec<BoundedText>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SettlementGraphInputV1 {
    pub economic_effect_commitment: Commitment32,
    pub graph_profile: SemanticProfileRefV1,
    pub temporal_profile: Option<SemanticProfileRefV1>,
    pub legs: Vec<SettlementLegSpecV1>,
    pub dependencies: Vec<DependencySpecV1>,
    pub coordination_groups: Vec<CoordinationGroupSpecV1>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct SettlementLegV1 {
    pub(crate) leg_id: Commitment32,
    pub(crate) adapter_profile: SemanticProfileRefV1,
    pub(crate) rail: BoundedText,
    pub(crate) network: BoundedText,
    pub(crate) role: SettlementLegRoleV1,
    pub(crate) source_subject: BoundedText,
    pub(crate) destination_subject: BoundedText,
    pub(crate) amount: AssetAmount,
    pub(crate) asset_unit_profile: SemanticProfileRefV1,
    pub(crate) required_finality_profile: SemanticProfileRefV1,
    pub(crate) semantic_idempotency_ref: BoundedText,
    pub(crate) purpose_profile: Option<SemanticProfileRefV1>,
    pub(crate) delivery_asset_subject: Option<BoundedText>,
}

impl SettlementLegV1 {
    pub fn leg_id(&self) -> Commitment32 {
        self.leg_id
    }

    pub fn adapter_profile(&self) -> &SemanticProfileRefV1 {
        &self.adapter_profile
    }

    pub fn rail(&self) -> &BoundedText {
        &self.rail
    }

    pub fn network(&self) -> &BoundedText {
        &self.network
    }

    pub fn role(&self) -> SettlementLegRoleV1 {
        self.role
    }

    pub fn source_subject(&self) -> &BoundedText {
        &self.source_subject
    }

    pub fn destination_subject(&self) -> &BoundedText {
        &self.destination_subject
    }

    pub fn amount(&self) -> &AssetAmount {
        &self.amount
    }

    pub fn asset_unit_profile(&self) -> &SemanticProfileRefV1 {
        &self.asset_unit_profile
    }

    pub fn required_finality_profile(&self) -> &SemanticProfileRefV1 {
        &self.required_finality_profile
    }

    pub fn semantic_idempotency_ref(&self) -> &BoundedText {
        &self.semantic_idempotency_ref
    }

    pub fn purpose_profile(&self) -> Option<&SemanticProfileRefV1> {
        self.purpose_profile.as_ref()
    }

    pub fn delivery_asset_subject(&self) -> Option<&BoundedText> {
        self.delivery_asset_subject.as_ref()
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum SettlementDependencyV1 {
    Requires {
        leg_id: Commitment32,
        prerequisite_id: Commitment32,
    },
    Before {
        before_id: Commitment32,
        after_id: Commitment32,
    },
    ConditionalOnEvidence {
        leg_id: Commitment32,
        predicate_profile: SemanticProfileRefV1,
    },
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct CoordinationGroupV1 {
    pub(crate) group_id: Commitment32,
    pub(crate) class: CoordinationGroupClassV1,
    pub(crate) coordination_profile: SemanticProfileRefV1,
    pub(crate) member_leg_ids: Vec<Commitment32>,
}

impl CoordinationGroupV1 {
    pub fn group_id(&self) -> Commitment32 {
        self.group_id
    }

    pub fn class(&self) -> CoordinationGroupClassV1 {
        self.class
    }

    pub fn coordination_profile(&self) -> &SemanticProfileRefV1 {
        &self.coordination_profile
    }

    pub fn member_leg_ids(&self) -> &[Commitment32] {
        &self.member_leg_ids
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct SettlementGraphV1 {
    pub(crate) economic_effect_commitment: Commitment32,
    pub(crate) graph_profile: SemanticProfileRefV1,
    pub(crate) temporal_profile: Option<SemanticProfileRefV1>,
    pub(crate) legs: Vec<SettlementLegV1>,
    pub(crate) dependencies: Vec<SettlementDependencyV1>,
    pub(crate) coordination_groups: Vec<CoordinationGroupV1>,
    pub(crate) graph_commitment: Commitment32,
}

impl SettlementGraphV1 {
    /// V1 intentionally uses the full semantic graph commitment as graph identity.
    pub fn graph_id(&self) -> Commitment32 {
        self.graph_commitment
    }

    pub fn graph_commitment(&self) -> Commitment32 {
        self.graph_commitment
    }

    pub fn economic_effect_commitment(&self) -> Commitment32 {
        self.economic_effect_commitment
    }

    pub fn graph_profile(&self) -> &SemanticProfileRefV1 {
        &self.graph_profile
    }

    pub fn temporal_profile(&self) -> Option<&SemanticProfileRefV1> {
        self.temporal_profile.as_ref()
    }

    pub fn legs(&self) -> &[SettlementLegV1] {
        &self.legs
    }

    pub fn dependencies(&self) -> &[SettlementDependencyV1] {
        &self.dependencies
    }

    pub fn coordination_groups(&self) -> &[CoordinationGroupV1] {
        &self.coordination_groups
    }
}
