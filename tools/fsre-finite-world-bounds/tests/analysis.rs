use fsre_finite_world_bounds::analyze;

const FIXTURE: &str = r#"
#analysis_id=water-restoration-demo
#estimand=safe_service_value
#unit=service_points
#decision_cutoff=40
#decision_direction=MAXIMIZE
#safety_floor=70
#assumption_set_id=assumptions-v1
#input_snapshot_id=snapshot-v1
#source_use_status=AUTHORIZED
#authority_status=ESTABLISHED
#world_set_completeness=DECLARED_COMPLETE
world_id|action_id|decision_value|safety_value
w1|repair|80|90
w1|inspect|70|95
w1|replace|60|60
w2|repair|75|90
w2|inspect|85|92
w2|replace|90|55
w3|repair|90|80
w3|inspect|82|89
w3|replace|100|65
"#;

#[test]
fn exact_bounds_regret_and_safety_are_reported_separately() {
    let report = analyze(FIXTURE).expect("valid input").expect("eligible");
    assert_eq!(report.status, "PARTIALLY_IDENTIFIED_WITHIN_DECLARED_WORLD_SET");
    assert_eq!(report.world_count, 3);
    assert_eq!(report.action_count, 3);
    assert_eq!(report.ranking_status, "RANKING_NOT_INVARIANT");
    let repair = report.actions.iter().find(|a| a.action_id == "repair").unwrap();
    assert_eq!((repair.decision_lower, repair.decision_upper), (75, 90));
    assert_eq!(repair.worst_case_regret, 15);
    assert!(repair.robustly_safe);
    let replace = report.actions.iter().find(|a| a.action_id == "replace").unwrap();
    assert!(!replace.robustly_safe);
    assert!(report.robust_optimal_action.is_none());
    assert_eq!(report.minimax_regret_actions, vec!["repair".to_owned()]);
}

#[test]
fn source_use_block_prevents_outcome_table_evaluation() {
    let blocked = FIXTURE
        .replace("source_use_status=AUTHORIZED", "source_use_status=BLOCKED")
        .replace("w1|repair|80|90", "this-is-not-a-valid-data-row");
    let qualification = analyze(&blocked).expect("blocked").expect_err("must block");
    assert_eq!(qualification.status, "BLOCKED_SOURCE_USE");
    assert!(qualification.render().contains("bounds=NOT_COMPUTED"));
}

#[test]
fn unknown_authority_blocks_numerical_analysis() {
    let input = FIXTURE.replace("authority_status=ESTABLISHED", "authority_status=UNKNOWN");
    let qualification = analyze(&input).unwrap().unwrap_err();
    assert_eq!(qualification.status, "BLOCKED_AUTHORITY_UNESTABLISHED");
}

#[test]
fn incomplete_world_set_is_not_identified_and_has_no_bounds() {
    let input = FIXTURE.replace("world_set_completeness=DECLARED_COMPLETE", "world_set_completeness=UNKNOWN");
    let qualification = analyze(&input).unwrap().unwrap_err();
    assert_eq!(qualification.status, "NOT_IDENTIFIED_WORLD_SET_INCOMPLETE");
    assert!(qualification.render().contains("world_count=NOT_COMPUTED"));
}

#[test]
fn incomplete_world_action_grid_is_rejected() {
    let input = FIXTURE.replace("w3|replace|100|65\n", "");
    assert!(analyze(&input).is_err());
}

#[test]
fn duplicate_world_action_cell_is_rejected() {
    let input = FIXTURE.replace("w3|replace|100|65", "w3|replace|100|65\nw3|replace|100|65");
    assert!(analyze(&input).is_err());
}

#[test]
fn row_order_does_not_change_report() {
    let original = analyze(FIXTURE).unwrap().unwrap().render();
    let mut metadata = Vec::new();
    let mut header = String::new();
    let mut rows = Vec::new();
    for line in FIXTURE.lines() {
        let line = line.trim();
        if line.starts_with('#') { metadata.push(line); }
        else if line.starts_with("world_id|") { header = line; }
        else if !line.is_empty() { rows.push(line); }
    }
    rows.reverse();
    let reordered = format!("{}\n{}\n{}\n", metadata.join("\n"), header, rows.join("\n"));
    assert_eq!(analyze(&reordered).unwrap().unwrap().render(), original);
}

#[test]
fn no_action_meeting_floor_produces_no_minimax_recommendation() {
    let input = FIXTURE
        .replace("|90\n", "|60\n")
        .replace("|95\n", "|60\n")
        .replace("|92\n", "|60\n")
        .replace("|89\n", "|60\n")
        .replace("|80\n", "|60\n");
    let report = analyze(&input).unwrap().unwrap();
    assert!(report.actions.iter().all(|a| !a.robustly_safe));
    assert!(report.minimax_regret_actions.is_empty());
    assert!(report.robust_optimal_action.is_none());
}
