// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! WASM-safe mirror types for the Mycelix Personal cluster.
//!
//! These types mirror `personal-types` but use only `serde` — no HDI/HDK
//! dependencies. Holochain `Timestamp` is replaced with `i64` (microseconds).

use serde::{Deserialize, Serialize};

// ============================================================================
// Vault Types
// ============================================================================

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum VaultVisibility {
    Private,
    Public,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum DisclosureScope {
    ExistenceOnly,
    SelectedFields(Vec<String>),
    Full,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum CredentialType {
    Identity,
    Health,
    FederatedLearning,
    Governance,
    Domain(String),
}

impl CredentialType {
    pub fn label(&self) -> &str {
        match self {
            Self::Identity => "Identity",
            Self::Health => "Health",
            Self::FederatedLearning => "Federated Learning",
            Self::Governance => "Governance",
            Self::Domain(s) => s.as_str(),
        }
    }
}

// ============================================================================
// K-Vector Trust Types
// ============================================================================

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum TrustTier {
    Observer,
    Basic,
    Standard,
    Elevated,
    Guardian,
}

impl TrustTier {
    pub fn min_score(&self) -> f64 {
        match self {
            Self::Observer => 0.0,
            Self::Basic => 0.3,
            Self::Standard => 0.4,
            Self::Elevated => 0.6,
            Self::Guardian => 0.8,
        }
    }

    pub fn from_score(score: f64) -> Self {
        if score >= 0.8 {
            Self::Guardian
        } else if score >= 0.6 {
            Self::Elevated
        } else if score >= 0.4 {
            Self::Standard
        } else if score >= 0.3 {
            Self::Basic
        } else {
            Self::Observer
        }
    }

    pub fn label(&self) -> &'static str {
        match self {
            Self::Observer => "Observer",
            Self::Basic => "Basic",
            Self::Standard => "Standard",
            Self::Elevated => "Elevated",
            Self::Guardian => "Guardian",
        }
    }

    pub fn css_class(&self) -> &'static str {
        match self {
            Self::Observer => "tier-observer",
            Self::Basic => "tier-basic",
            Self::Standard => "tier-standard",
            Self::Elevated => "tier-elevated",
            Self::Guardian => "tier-guardian",
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TrustScoreRange {
    pub lower: f32,
    pub upper: f32,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum KVectorComponent {
    Reputation,
    Activity,
    Integrity,
    Performance,
    Membership,
    Stake,
    History,
    Topology,
}

impl KVectorComponent {
    pub fn label(&self) -> &'static str {
        match self {
            Self::Reputation => "Reputation",
            Self::Activity => "Activity",
            Self::Integrity => "Integrity",
            Self::Performance => "Performance",
            Self::Membership => "Membership",
            Self::Stake => "Stake",
            Self::History => "History",
            Self::Topology => "Topology",
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum AttestationStatus {
    Pending,
    Fulfilled,
    Declined,
    Expired,
    Cancelled,
}

// ============================================================================
// Presentation Types
// ============================================================================

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CredentialPresentation {
    pub credential_type: CredentialType,
    pub disclosed_data: String,
    pub scope: DisclosureScope,
    pub presented_at: i64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PresentationRequest {
    pub credential_type: CredentialType,
    pub scope: DisclosureScope,
    pub context: Option<String>,
}

// ============================================================================
// View Types (UI-facing structs with String IDs)
// ============================================================================

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProfileView {
    pub display_name: String,
    pub avatar: Option<String>,
    pub bio: Option<String>,
    pub metadata: std::collections::HashMap<String, String>,
    pub updated_at: i64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MasterKeyView {
    pub label: String,
    pub purpose: String,
    pub public_key_hex: String,
    pub active: bool,
    pub created_at: i64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HealthRecordView {
    pub hash: String,
    pub record_type: String,
    pub data: String,
    pub source: String,
    pub event_date: i64,
    pub updated_at: i64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BiometricView {
    pub hash: String,
    pub metric_type: String,
    pub value: f64,
    pub unit: String,
    pub measured_at: i64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConsentGrantView {
    pub hash: String,
    pub grantee: String,
    pub record_types: Vec<String>,
    pub expires_at: Option<i64>,
    pub active: bool,
    pub created_at: i64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConsentGrantInputView {
    pub grantee: String,
    pub record_types: Vec<String>,
    pub expires_at: Option<i64>,
    pub active: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DataSharingPreferenceView {
    pub source_cluster: String,
    pub target_cluster: String,
    pub allowed: bool,
    pub blocked_zomes: Vec<String>,
    pub reason: String,
    pub updated_at: i64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PreferenceChangeLogView {
    pub source_cluster: String,
    pub target_cluster: String,
    pub was_allowed: bool,
    pub now_allowed: bool,
    pub changed_at: i64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BridgeQueryView {
    pub hash: String,
    pub domain: String,
    pub query_type: String,
    pub requester: String,
    pub params: String,
    pub result: Option<String>,
    pub created_at: i64,
    pub resolved_at: Option<i64>,
    pub success: Option<bool>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BridgeEventView {
    pub hash: String,
    pub domain: String,
    pub event_type: String,
    pub source_agent: String,
    pub payload: String,
    pub created_at: i64,
    pub related_hashes: Vec<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ActivityItemView {
    pub id: String,
    pub kind: String,
    pub domain: String,
    pub title: String,
    pub detail: String,
    pub created_at: i64,
    pub success: Option<bool>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StoredCredentialView {
    pub hash: String,
    pub credential_type: CredentialType,
    pub issuer: String,
    pub issued_at: i64,
    pub expires_at: Option<i64>,
    pub revoked: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrustCredentialView {
    pub id: String,
    pub subject_did: String,
    pub issuer_did: String,
    pub trust_tier: TrustTier,
    pub trust_score_range: TrustScoreRange,
    pub issued_at: i64,
    pub expires_at: Option<i64>,
    pub revoked: bool,
}

// ============================================================================
// Source-aware Life Continuity / Asset Evidence Types
// ============================================================================

/// State of a read from one source adapter.
///
/// These variants are intentionally not interchangeable:
/// - `LiveConfirmedEmpty` means the source read succeeded and returned no
///   records at the stated time.
/// - `Unavailable` means the source could not be read; absence is unknown.
/// - `Stale` means a previous successful result exists but needs revalidation.
/// - `Demo` marks synthetic fixture data.
/// - `Conflict` means evidence disagrees and must not be silently resolved.
/// - `Unsupported` means no source adapter exists for this domain.
///
/// Adapters should emit this state alongside records, rather than manufacture
/// empty lists when requests fail.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "state", content = "details", rename_all = "snake_case")]
pub enum SourceReadState {
    NotChecked,
    Loading,
    LiveWithRecords { checked_at: i64 },
    LiveConfirmedEmpty { checked_at: i64 },
    Unavailable { reason: SourceUnavailableReason },
    Stale { last_successful_read_at: Option<i64> },
    Demo,
    Conflict,
    Unsupported,
}

/// Sanitized reason code for a failed source read.
///
/// Keep raw error bodies, provider responses and credentials out of this type:
/// they can contain identifiers or sensitive implementation details.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SourceUnavailableReason {
    Network,
    Timeout,
    Unauthorized,
    MalformedResponse,
    Storage,
    Unknown,
}

/// How an evidence item entered the system. Origin does not, by itself,
/// establish truth or legal authority.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EvidenceOrigin {
    HolderReported,
    IssuerAttested,
    ProviderReported,
    InstrumentObserved,
    Derived,
    Unknown,
}

/// The kind of authority represented by a source.
///
/// This is descriptive metadata, not an authorization decision. For example,
/// a government registry may be authoritative for registration status but not
/// necessarily for insurance, maintenance quality or every ownership dispute.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EvidenceAuthorityClass {
    HolderAssertion,
    CredentialIssuer,
    ServiceProvider,
    Instrument,
    GovernmentRegistry,
    CourtOrAdjudicator,
    LicensedProfessional,
    DerivedSystem,
    Unknown,
}

/// Cryptographic integrity is kept separate from source trust and claim status.
///
/// `Valid` means only that the checked cryptographic proof passed. It does
/// not mean that the issuer is trusted or the claim is legally authoritative.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CryptographicIntegrity {
    NotApplicable,
    NotChecked,
    Valid,
    Invalid,
    Unknown,
}

/// Current lifecycle assessment of a particular claim, independent of whether
/// a signature was valid.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ClaimLifecycleStatus {
    Unassessed,
    Active,
    Expired,
    Revoked,
    Superseded,
    Disputed,
    Unknown,
}

/// Freshness assessment under the source adapter's declared policy.
/// Timestamps should be preserved so consumers can inspect the basis.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EvidenceFreshness {
    FreshAtRead,
    Stale,
    Unknown,
    NotApplicable,
}

/// Evidence metadata for one claim in a private Personal view.
///
/// `source_ref` may itself be sensitive or correlatable. It must not be
/// copied into public DHT entries or external disclosure payloads by default.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ClaimEvidenceView {
    pub origin: EvidenceOrigin,
    pub authority: EvidenceAuthorityClass,
    pub cryptographic_integrity: CryptographicIntegrity,
    pub lifecycle_status: ClaimLifecycleStatus,
    pub freshness: EvidenceFreshness,
    pub source_label: String,
    pub source_ref: Option<String>,
    /// Time the underlying event/assertion occurred, in microseconds when known.
    pub observed_at: Option<i64>,
    /// Time this source was last checked successfully, in microseconds.
    pub checked_at: Option<i64>,
    /// Source-claimed expiration time, in microseconds.
    pub valid_until: Option<i64>,
}

/// Broad category of real-world asset represented in a personal workspace.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LifeAssetKind {
    Vehicle,
    Property,
    Equipment,
    Appliance,
    Electronics,
    EnergySystem,
    Other(String),
}

/// A claim category is deliberately more granular than a generic
/// "verified/unverified" asset status.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LifeAssetClaimCategory {
    Identity,
    Provenance,
    Ownership,
    Registration,
    Custody,
    LienOrEncumbrance,
    Insurance,
    Condition,
    Maintenance,
    Recall,
    Inspection,
    Warranty,
    Other(String),
}

/// One display-safe claim projection with its evidence metadata.
///
/// The summary belongs to the private UI model; it is not a portable credential
/// or a statement that Mycelix has independently established legal truth.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct LifeAssetClaimView {
    pub category: LifeAssetClaimCategory,
    pub summary: String,
    pub evidence: ClaimEvidenceView,
}

/// Read-model for an asset passport. It composes sourced claims; it is not a
/// replacement for a title registry, registration certificate, insurer record,
/// inspection authority or manufacturer record.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct LifeAssetPassportSummaryView {
    /// Opaque reference scoped to the local/private Personal app.
    pub local_asset_ref: String,
    pub kind: LifeAssetKind,
    pub display_name: String,
    pub claims: Vec<LifeAssetClaimView>,
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn vault_visibility_serde_roundtrip() {
        let vis = VaultVisibility::Private;
        let json = serde_json::to_string(&vis).unwrap();
        let back: VaultVisibility = serde_json::from_str(&json).unwrap();
        assert_eq!(back, VaultVisibility::Private);
    }

    #[test]
    fn disclosure_scope_selected_fields() {
        let scope = DisclosureScope::SelectedFields(vec!["name".into(), "email".into()]);
        let json = serde_json::to_string(&scope).unwrap();
        let back: DisclosureScope = serde_json::from_str(&json).unwrap();
        assert_eq!(back, scope);
    }

    #[test]
    fn credential_type_domain() {
        let ct = CredentialType::Domain("water_steward".into());
        let json = serde_json::to_string(&ct).unwrap();
        let back: CredentialType = serde_json::from_str(&json).unwrap();
        assert_eq!(back, ct);
    }

    #[test]
    fn trust_tier_from_score_boundaries() {
        assert_eq!(TrustTier::from_score(0.0), TrustTier::Observer);
        assert_eq!(TrustTier::from_score(0.29), TrustTier::Observer);
        assert_eq!(TrustTier::from_score(0.3), TrustTier::Basic);
        assert_eq!(TrustTier::from_score(0.4), TrustTier::Standard);
        assert_eq!(TrustTier::from_score(0.6), TrustTier::Elevated);
        assert_eq!(TrustTier::from_score(0.8), TrustTier::Guardian);
        assert_eq!(TrustTier::from_score(1.0), TrustTier::Guardian);
    }

    #[test]
    fn trust_tier_min_score_consistent() {
        for tier in &[
            TrustTier::Observer,
            TrustTier::Basic,
            TrustTier::Standard,
            TrustTier::Elevated,
            TrustTier::Guardian,
        ] {
            assert_eq!(TrustTier::from_score(tier.min_score()), *tier);
        }
    }

    #[test]
    fn profile_view_serde_roundtrip() {
        let view = ProfileView {
            display_name: "Alice".into(),
            avatar: Some("https://example.com/avatar.png".into()),
            bio: Some("Hello world".into()),
            metadata: [("role".into(), "researcher".into())].into(),
            updated_at: 1_700_000_000,
        };
        let json = serde_json::to_string(&view).unwrap();
        let back: ProfileView = serde_json::from_str(&json).unwrap();
        assert_eq!(back.display_name, "Alice");
    }

    #[test]
    fn kvector_labels() {
        assert_eq!(KVectorComponent::Reputation.label(), "Reputation");
        assert_eq!(KVectorComponent::Topology.label(), "Topology");
    }

    #[test]
    fn trust_tier_css_classes() {
        assert_eq!(TrustTier::Observer.css_class(), "tier-observer");
        assert_eq!(TrustTier::Guardian.css_class(), "tier-guardian");
    }
}
