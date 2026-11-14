use fsre_frame_dependency_audit::audit;

const COMMON_ROOT: &str = r#"
#audit_id=frame-dependency-demo
#target_scope_id=municipal-wastewater
#study_start_seconds=0
#study_end_seconds=100
#source_use_status=AUTHORIZED
record_type|frame_id|upstream_frame_id|frame_role|owner_id|collection_process_id|snapshot_id|valid_from_seconds|valid_until_seconds|evidence_owner|evidence_ref|assessment
FRAME|admin-register|-|UPSTREAM|department|admin-extract|snapshot-a|0|100|department|register-doc|UNKNOWN
FRAME|operator-frame|-|CAPTURE|operator|operator-survey|snapshot-b|0|100|reviewer|operator-doc|SEPARATE_DESIGN_DECLARED
FRAME|field-frame|-|CAPTURE|survey-team|field-enumeration|snapshot-c|0|100|reviewer|field-doc|SEPARATE_DESIGN_DECLARED
DEPENDENCY|operator-frame|admin-register|-|-|-|-|0|100|reviewer|inherits-admin|-
"#;

#[test]
fn one_frames_upstream_link_does_not_prove_pairwise_dependence() {
    let report = audit(COMMON_ROOT).unwrap().unwrap();
    assert_eq!(report.frame_count, 3);
    assert_eq!(report.capture_frame_count, 2);
    assert_eq!(report.dependency_edge_count, 1);
    assert_eq!(report.pair_count, 1);
    let pair = &report.pairs[0];
    assert_eq!(pair.status, "NO_DECLARED_DEPENDENCE_INDEPENDENCE_UNAUTHENTICATED");
    assert!(!pair.same_owner);
    assert!(!pair.same_collection_process);
}

#[test]
fn shared_ancestor_requires_both_capture_frames_to_depend_on_root() {
    let input = COMMON_ROOT.replace(
        "DEPENDENCY|operator-frame|admin-register|-|-|-|-|0|100|reviewer|inherits-admin|-",
        "DEPENDENCY|operator-frame|admin-register|-|-|-|-|0|100|reviewer|inherits-admin|-\nDEPENDENCY|field-frame|admin-register|-|-|-|-|0|100|reviewer|inherits-admin-too|-",
    );
    let report = audit(&input).unwrap().unwrap();
    assert_eq!(report.pairs[0].status, "DECLARED_SHARED_UPSTREAM_SOURCE");
    assert_eq!(report.pairs[0].shared_ancestors, vec!["admin-register".to_owned()]);
}

#[test]
fn shared_collection_process_is_flagged() {
    let input = COMMON_ROOT.replace("field-enumeration", "operator-survey");
    let report = audit(&input).unwrap().unwrap();
    assert_eq!(report.pairs[0].status, "DECLARED_SHARED_COLLECTION_PROCESS");
    assert!(report.pairs[0].same_collection_process);
}

#[test]
fn separate_design_assertions_do_not_prove_independence() {
    let report = audit(COMMON_ROOT).unwrap().unwrap();
    assert_eq!(report.pairs[0].status, "NO_DECLARED_DEPENDENCE_INDEPENDENCE_UNAUTHENTICATED");
    assert!(report.render().contains("independence_conclusion=NOT_ESTABLISHED_BY_STRUCTURAL_AUDIT"));
    assert!(report.render().contains("dual_system_estimator_admissibility=NOT_ESTABLISHED"));
}

#[test]
fn missing_independence_design_assessment_remains_unknown() {
    let input = COMMON_ROOT.replace("field-doc|SEPARATE_DESIGN_DECLARED", "field-doc|UNKNOWN");
    let report = audit(&input).unwrap().unwrap();
    assert_eq!(report.pairs[0].status, "INDEPENDENCE_NOT_ESTABLISHED");
}

#[test]
fn common_owner_is_a_review_signal_not_automatic_dependence() {
    let input = COMMON_ROOT
        .replace("survey-team|field-enumeration", "operator|field-enumeration")
        .replace("field-doc|SEPARATE_DESIGN_DECLARED", "field-doc|UNKNOWN");
    let report = audit(&input).unwrap().unwrap();
    assert_eq!(report.pairs[0].status, "POTENTIAL_COMMON_OWNER_REQUIRES_REVIEW");
    assert!(report.pairs[0].same_owner);
}

#[test]
fn cycles_are_rejected() {
    let input = COMMON_ROOT.replace(
        "DEPENDENCY|operator-frame|admin-register|-|-|-|-|0|100|reviewer|inherits-admin|-",
        "DEPENDENCY|operator-frame|field-frame|-|-|-|-|0|100|reviewer|edge-a|-\nDEPENDENCY|field-frame|operator-frame|-|-|-|-|0|100|reviewer|edge-b|-",
    );
    assert!(audit(&input).is_err());
}

#[test]
fn source_use_block_prevents_frame_row_parsing() {
    let input = COMMON_ROOT
        .replace("source_use_status=AUTHORIZED", "source_use_status=BLOCKED")
        .replace("DEPENDENCY|operator-frame|admin-register|-|-|-|-|0|100|reviewer|inherits-admin|-", "malformed row");
    let qualification = audit(&input).unwrap().unwrap_err();
    assert_eq!(qualification.status, "BLOCKED_SOURCE_USE");
    assert!(qualification.render().contains("estimators=NOT_ADMISSIBLE"));
}

#[test]
fn row_order_does_not_change_pair_report() {
    let original = audit(COMMON_ROOT).unwrap().unwrap().render();
    let mut metadata = Vec::new();
    let mut header = String::new();
    let mut rows = Vec::new();
    for line in COMMON_ROOT.lines() {
        let line = line.trim();
        if line.starts_with('#') { metadata.push(line); }
        else if line.starts_with("record_type|") { header = line; }
        else if !line.is_empty() { rows.push(line); }
    }
    rows.reverse();
    let reordered = format!("{}\n{}\n{}\n", metadata.join("\n"), header, rows.join("\n"));
    assert_eq!(audit(&reordered).unwrap().unwrap().render(), original);
}


#[test]
fn shared_snapshot_is_detected_even_without_declared_edge() {
    let input = COMMON_ROOT.replace("field-enumeration", "operator-survey");
    let input = input.replace("snapshot-c|0|100", "snapshot-b|0|100");
    let report = audit(&input).unwrap().unwrap();
    assert_eq!(report.pairs[0].status, "DECLARED_SHARED_SOURCE_SNAPSHOT");
}
