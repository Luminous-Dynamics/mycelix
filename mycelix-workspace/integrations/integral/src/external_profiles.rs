use serde::{Deserialize, Serialize};
use serde_json::Value;

use crate::{ExternalStatus, IntegralSystem, SourceFamily};

/// Current public Technical Specifications web observation.
///
/// This identifier is deliberately not presented as a content-addressed capture:
/// the exact external capture remains owned by the EPI/WEB-CAPTURE line.
pub const TECHNICAL_SPECIFICATIONS_OBSERVATION: &str =
    "integralcollective.io/documents/specifications.html@observed-2026-09-27";

/// Exact GitHub blob observed for the Development Guide source used by this generation.
pub const DEVELOPMENT_GUIDE_BLOB: &str =
    "a99f095eae0790014163fd932488baf566ce4e6b";

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AuthoritySemantics {
    None,
}

/// Serializable source/schema identity carried with an external compatibility object.
///
/// `content_commitment` remains optional because the current Technical Specifications
/// web observation is not yet an EPI-qualified byte capture. Absence of a commitment
/// must never be interpreted as equivalent to a verified capture.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ExternalSchemaBinding {
    pub source_family: SourceFamily,
    pub source_status: ExternalStatus,
    pub source_revision: String,
    pub observed_at: String,
    pub schema_id: String,
    pub schema_generation: String,
    pub content_commitment: Option<String>,
    pub authority_semantics: AuthoritySemantics,
}

impl ExternalSchemaBinding {
    pub fn technical_specification(schema_id: &str) -> Self {
        Self {
            source_family: SourceFamily::TechnicalSpecifications,
            source_status: ExternalStatus::Draft,
            source_revision: TECHNICAL_SPECIFICATIONS_OBSERVATION.to_string(),
            observed_at: "2026-09-27".to_string(),
            schema_id: schema_id.to_string(),
            schema_generation: "public-draft-observed-2026-09-27".to_string(),
            content_commitment: None,
            authority_semantics: AuthoritySemantics::None,
        }
    }

    pub fn development_guide(object_name: &str) -> Self {
        Self {
            source_family: SourceFamily::DevelopmentGuide,
            source_status: ExternalStatus::DevelopmentGuideProposal,
            source_revision: format!("github-blob:{DEVELOPMENT_GUIDE_BLOB}"),
            observed_at: "2026-09-27".to_string(),
            schema_id: format!("DEVGUIDE:{object_name}"),
            schema_generation: "devguide-v0.1-source-observed-2026-09-27".to_string(),
            content_commitment: Some(DEVELOPMENT_GUIDE_BLOB.to_string()),
            authority_semantics: AuthoritySemantics::None,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ProfiledExternalObject<T> {
    pub source: ExternalSchemaBinding,
    pub payload: T,
}

macro_rules! optional_value_struct {
    ($name:ident { $($field:ident),+ $(,)? }) => {
        #[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
        #[serde(deny_unknown_fields)]
        pub struct $name {
            $(
                #[serde(default, skip_serializing_if = "Option::is_none")]
                pub $field: Option<Value>,
            )+
        }
    };
}

// -----------------------------------------------------------------------------
// Current Technical Specifications DRAFT objects.
//
// All fields are Option<Value> intentionally. The current public specifications
// name fields and describe semantics but do not yet constitute a ratified JSON/Rust
// type system or a stable required/optional contract. This generation freezes the
// known top-level field vocabulary without pretending Mycelix's inferred types are
// Integral's canonical types.
// -----------------------------------------------------------------------------

optional_value_struct!(CertifiedDesignDraft20260927 {
    design_id,
    version,
    bill_of_materials,
    production_steps,
    ecological_flag,
    itc_access_cost,
    design_lineage,
});

optional_value_struct!(LaborEventDraft20260927 {
    participant_id,
    task_ref,
    hours_verified,
    skill_tier,
    itc_credits_issued,
    ecological_flag,
});

optional_value_struct!(MaterialConsumptionEventDraft20260927 {
    material_id,
    quantity_consumed,
    ecological_flag,
    source,
    production_ref,
});

optional_value_struct!(ItcLedgerEntryDraft20260927 {
    entry_id,
    timestamp,
    participant_id,
    entry_type,
    amount,
    balance_after,
    source_ref,
});

optional_value_struct!(FrsSignalPacketDraft20260927 {
    labor_summary,
    materials_summary,
    itc_summary,
    qa_summary,
    ecological_summary,
    findings,
    recommendations,
});

optional_value_struct!(DecisionPacketDraft20260927 {
    decision_id,
    issue_ref,
    outcome,
    rationale,
    dissenting_positions,
    dispatch_targets,
    implementation_constraints,
    review_trigger,
});

// -----------------------------------------------------------------------------
// Development Guide proposal objects.
// These are kept in a distinct source/status namespace from Technical Specs.
// -----------------------------------------------------------------------------

optional_value_struct!(ItcAccountDevGuideV01 {
    account_id,
    participant_id,
    node_id,
    balance,
    total_earned,
    total_redeemed,
    total_decayed,
    active_decay_rule_id,
    last_decay_applied_at,
    last_contribution_at,
    last_access_at,
    decay_half_life_days,
    decay_inactivity_grace_days,
    decay_min_balance_protected,
    decay_max_annual_fraction,
    need_adjustment_active,
    need_adjustment_ref,
    transferable,
    federation_scope,
    last_updated_at,
});

optional_value_struct!(DiagnosticFindingDevGuideV01 {
    finding_id,
    packet_ref,
    node_id,
    detected_at,
    finding_type,
    severity,
    scope,
    persistence,
    confidence,
    summary,
    rationale,
    evidence_refs,
    indicators,
    target_system,
    requires_cds,
});

optional_value_struct!(RecommendationDevGuideV01 {
    recommendation_id,
    finding_ref,
    constraint_model_ref,
    node_id,
    created_at,
    target_system,
    recommendation_type,
    severity,
    scope,
    confidence,
    summary,
    rationale,
    payload,
    status,
    response_ref,
});

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FieldOriginKind {
    SourceOwned,
    DerivedSnapshot,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExternalFieldOrigin {
    pub field: &'static str,
    pub owner: IntegralSystem,
    pub kind: FieldOriginKind,
}

/// The public LaborEvent convenience shape crosses system ownership boundaries.
/// In particular, `itc_credits_issued` is an ITC-derived result even when carried
/// inside the external LaborEvent projection.
pub const LABOR_EVENT_FIELD_ORIGINS: [ExternalFieldOrigin; 6] = [
    ExternalFieldOrigin {
        field: "participant_id",
        owner: IntegralSystem::Cos,
        kind: FieldOriginKind::SourceOwned,
    },
    ExternalFieldOrigin {
        field: "task_ref",
        owner: IntegralSystem::Cos,
        kind: FieldOriginKind::SourceOwned,
    },
    ExternalFieldOrigin {
        field: "hours_verified",
        owner: IntegralSystem::Cos,
        kind: FieldOriginKind::SourceOwned,
    },
    ExternalFieldOrigin {
        field: "skill_tier",
        owner: IntegralSystem::Cos,
        kind: FieldOriginKind::SourceOwned,
    },
    ExternalFieldOrigin {
        field: "itc_credits_issued",
        owner: IntegralSystem::Itc,
        kind: FieldOriginKind::DerivedSnapshot,
    },
    ExternalFieldOrigin {
        field: "ecological_flag",
        owner: IntegralSystem::Oad,
        kind: FieldOriginKind::DerivedSnapshot,
    },
];

/// `itc_access_cost` is carried by the external CertifiedDesign package but is
/// an ITC-derived snapshot, not evidence that OAD owns value/accounting policy.
pub const CERTIFIED_DESIGN_FIELD_ORIGINS: [ExternalFieldOrigin; 7] = [
    ExternalFieldOrigin {
        field: "design_id",
        owner: IntegralSystem::Oad,
        kind: FieldOriginKind::SourceOwned,
    },
    ExternalFieldOrigin {
        field: "version",
        owner: IntegralSystem::Oad,
        kind: FieldOriginKind::SourceOwned,
    },
    ExternalFieldOrigin {
        field: "bill_of_materials",
        owner: IntegralSystem::Oad,
        kind: FieldOriginKind::SourceOwned,
    },
    ExternalFieldOrigin {
        field: "production_steps",
        owner: IntegralSystem::Oad,
        kind: FieldOriginKind::SourceOwned,
    },
    ExternalFieldOrigin {
        field: "ecological_flag",
        owner: IntegralSystem::Oad,
        kind: FieldOriginKind::SourceOwned,
    },
    ExternalFieldOrigin {
        field: "itc_access_cost",
        owner: IntegralSystem::Itc,
        kind: FieldOriginKind::DerivedSnapshot,
    },
    ExternalFieldOrigin {
        field: "design_lineage",
        owner: IntegralSystem::Oad,
        kind: FieldOriginKind::SourceOwned,
    },
];
