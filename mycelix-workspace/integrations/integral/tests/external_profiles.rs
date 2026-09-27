use mycelix_integral_shell::{
    AuthoritySemantics, CertifiedDesignDraft20260927, DecisionPacketDraft20260927,
    DiagnosticFindingDevGuideV01, ExternalSchemaBinding, ExternalStatus, FieldOriginKind,
    FrsSignalPacketDraft20260927, IntegralSystem, ItcAccountDevGuideV01,
    ItcLedgerEntryDraft20260927, LaborEventDraft20260927,
    MaterialConsumptionEventDraft20260927, RecommendationDevGuideV01, SourceFamily,
    CERTIFIED_DESIGN_FIELD_ORIGINS, DEVELOPMENT_GUIDE_BLOB, LABOR_EVENT_FIELD_ORIGINS,
};
use serde_json::{json, Value};

#[test]
fn technical_spec_binding_is_draft_non_authoritative_and_source_identified() {
    let binding = ExternalSchemaBinding::technical_specification("SPEC-DS-01");
    assert_eq!(binding.source_family, SourceFamily::TechnicalSpecifications);
    assert_eq!(binding.source_status, ExternalStatus::Draft);
    assert_eq!(binding.schema_id, "SPEC-DS-01");
    assert_eq!(binding.authority_semantics, AuthoritySemantics::None);
    assert!(binding.content_commitment.is_none());
    assert!(binding.source_revision.contains("observed-2026-09-27"));
}

#[test]
fn development_guide_objects_cannot_masquerade_as_technical_spec_drafts() {
    let binding = ExternalSchemaBinding::development_guide("ITCAccount");
    assert_eq!(binding.source_family, SourceFamily::DevelopmentGuide);
    assert_eq!(binding.source_status, ExternalStatus::DevelopmentGuideProposal);
    assert_eq!(binding.schema_id, "DEVGUIDE:ITCAccount");
    assert_eq!(binding.content_commitment.as_deref(), Some(DEVELOPMENT_GUIDE_BLOB));
    assert_eq!(binding.authority_semantics, AuthoritySemantics::None);
}

#[test]
fn draft_shapes_reject_unknown_top_level_fields() {
    let bad = json!({
        "design_id": "D-1",
        "future_unratified_field": true
    });
    assert!(serde_json::from_value::<CertifiedDesignDraft20260927>(bad).is_err());

    let bad = json!({
        "decision_id": "DR-1",
        "authorization": {"execute": true}
    });
    assert!(serde_json::from_value::<DecisionPacketDraft20260927>(bad).is_err());
}

#[test]
fn unsettled_external_field_types_are_not_overclaimed() {
    let value = json!({
        "design_id": ["opaque", "external", "shape"],
        "version": {"not": "a ratified type"},
        "itc_access_cost": null
    });
    let parsed: CertifiedDesignDraft20260927 = serde_json::from_value(value).unwrap();
    assert!(matches!(parsed.design_id, Some(Value::Array(_))));
    assert!(matches!(parsed.version, Some(Value::Object(_))));
    assert_eq!(parsed.itc_access_cost, Some(Value::Null));
}

#[test]
fn all_six_current_technical_spec_shapes_parse_known_fields_only() {
    let _: CertifiedDesignDraft20260927 = serde_json::from_value(json!({
        "design_id": "D",
        "version": 1,
        "bill_of_materials": [],
        "production_steps": [],
        "ecological_flag": "low",
        "itc_access_cost": 10,
        "design_lineage": null
    })).unwrap();

    let _: LaborEventDraft20260927 = serde_json::from_value(json!({
        "participant_id": "P",
        "task_ref": "T",
        "hours_verified": 2,
        "skill_tier": "medium",
        "itc_credits_issued": 2,
        "ecological_flag": "low"
    })).unwrap();

    let _: MaterialConsumptionEventDraft20260927 = serde_json::from_value(json!({
        "material_id": "M",
        "quantity_consumed": 4,
        "ecological_flag": "low",
        "source": "internal",
        "production_ref": "P"
    })).unwrap();

    let _: ItcLedgerEntryDraft20260927 = serde_json::from_value(json!({
        "entry_id": "E",
        "timestamp": "opaque",
        "participant_id": "P",
        "entry_type": "LABOR_EVENT",
        "amount": 2,
        "balance_after": 10,
        "source_ref": "S"
    })).unwrap();

    let _: FrsSignalPacketDraft20260927 = serde_json::from_value(json!({
        "labor_summary": {},
        "materials_summary": {},
        "itc_summary": {},
        "qa_summary": {},
        "ecological_summary": {},
        "findings": [],
        "recommendations": []
    })).unwrap();

    let _: DecisionPacketDraft20260927 = serde_json::from_value(json!({
        "decision_id": "D",
        "issue_ref": "I",
        "outcome": "opaque",
        "rationale": "R",
        "dissenting_positions": [],
        "dispatch_targets": [],
        "implementation_constraints": {},
        "review_trigger": {}
    })).unwrap();
}

#[test]
fn development_guide_support_shapes_are_distinct_and_strict() {
    let _: ItcAccountDevGuideV01 = serde_json::from_value(json!({
        "account_id": "A",
        "participant_id": "P",
        "transferable": false
    })).unwrap();

    let _: DiagnosticFindingDevGuideV01 = serde_json::from_value(json!({
        "finding_id": "F",
        "requires_cds": true
    })).unwrap();

    let _: RecommendationDevGuideV01 = serde_json::from_value(json!({
        "recommendation_id": "R",
        "status": "pending"
    })).unwrap();

    assert!(serde_json::from_value::<RecommendationDevGuideV01>(json!({
        "recommendation_id": "R",
        "decision_id": "must-not-collapse"
    })).is_err());
}

#[test]
fn labor_event_field_origin_preserves_cos_vs_itc_split() {
    let hours = LABOR_EVENT_FIELD_ORIGINS
        .iter()
        .find(|entry| entry.field == "hours_verified")
        .unwrap();
    assert_eq!(hours.owner, IntegralSystem::Cos);
    assert_eq!(hours.kind, FieldOriginKind::SourceOwned);

    let credits = LABOR_EVENT_FIELD_ORIGINS
        .iter()
        .find(|entry| entry.field == "itc_credits_issued")
        .unwrap();
    assert_eq!(credits.owner, IntegralSystem::Itc);
    assert_eq!(credits.kind, FieldOriginKind::DerivedSnapshot);
}

#[test]
fn certified_design_keeps_itc_access_cost_as_itc_derived_snapshot() {
    let cost = CERTIFIED_DESIGN_FIELD_ORIGINS
        .iter()
        .find(|entry| entry.field == "itc_access_cost")
        .unwrap();
    assert_eq!(cost.owner, IntegralSystem::Itc);
    assert_eq!(cost.kind, FieldOriginKind::DerivedSnapshot);

    assert!(CERTIFIED_DESIGN_FIELD_ORIGINS
        .iter()
        .filter(|entry| entry.field != "itc_access_cost")
        .all(|entry| entry.owner == IntegralSystem::Oad));
}

#[test]
fn source_binding_round_trip_preserves_generation_and_non_authority() {
    let binding = ExternalSchemaBinding::development_guide("DiagnosticFinding");
    let encoded = serde_json::to_string(&binding).unwrap();
    let decoded: ExternalSchemaBinding = serde_json::from_str(&encoded).unwrap();
    assert_eq!(decoded, binding);
    assert_eq!(decoded.authority_semantics, AuthoritySemantics::None);
}
