use fsre_temporal_identity_lineage::replay;

const FIXTURE: &str = r#"event_id|event_type|predecessors|successors|effective_at|recorded_at|source_snapshot|evidence_owner|evidence_ref|adjudicator|rule_version
rename-1|RENAME|asset-a|asset-a|10|10|registry-v1|owner-a|evidence-1|reviewer-a|rules-v1
split-1|SPLIT|asset-a|asset-b,asset-c|20|25|registry-v2|owner-a|evidence-2|reviewer-a|rules-v1
late-1|OPERATOR_TRANSFER|asset-b|asset-b|30|50|registry-v3|owner-b|evidence-3|reviewer-b|rules-v1
future-1|RELOCATION|asset-c|asset-c|90|90|registry-v4|owner-b|evidence-4|reviewer-b|rules-v1
"#;

#[test]
fn replay_uses_both_effective_and_recorded_time() {
    let report = replay(FIXTURE, 40, 40).expect("valid history");
    assert_eq!(report.input_events, 4);
    assert_eq!(report.replayed_events, 2);
    assert_eq!(report.excluded_not_yet_recorded, 1);
    assert_eq!(report.excluded_not_yet_effective, 1);
    assert_eq!(report.event_type_counts.get("SPLIT"), Some(&1));
}

#[test]
fn late_arriving_event_does_not_rewrite_prior_knowledge() {
    let before = replay(FIXTURE, 40, 40).expect("historical view");
    let after = replay(FIXTURE, 60, 40).expect("later recorded view");
    assert_eq!(before.replayed_events, 2);
    assert_eq!(after.replayed_events, 3);
    assert_eq!(before.excluded_not_yet_recorded, 1);
    assert_eq!(after.excluded_not_yet_effective, 1);
}

#[test]
fn merge_requires_multiple_predecessors_and_one_successor() {
    let invalid = FIXTURE.replace("split-1|SPLIT|asset-a|asset-b,asset-c", "split-1|MERGE|asset-a|asset-b,asset-c");
    assert!(replay(&invalid, 100, 100).is_err());
}

#[test]
fn identity_preserving_event_cannot_change_entity_id() {
    let invalid = FIXTURE.replace("rename-1|RENAME|asset-a|asset-a", "rename-1|RENAME|asset-a|asset-z");
    assert!(replay(&invalid, 100, 100).is_err());
}

#[test]
fn identifier_reassignment_is_not_a_rename() {
    let invalid = FIXTURE.replace("rename-1|RENAME|asset-a|asset-a", "rename-1|IDENTIFIER_REASSIGNMENT|asset-a|asset-a");
    assert!(replay(&invalid, 100, 100).is_err());
}

#[test]
fn lineage_cycle_is_rejected() {
    let cycle = format!("{FIXTURE}cycle-1|SPLIT|asset-b|asset-a,asset-d|35|35|registry-v5|owner-a|evidence-5|reviewer-a|rules-v1\n");
    assert!(replay(&cycle, 100, 100).is_err());
}

#[test]
fn report_is_deterministic_under_row_reordering() {
    let mut rows: Vec<&str> = FIXTURE.lines().collect();
    let header = rows.remove(0);
    rows.reverse();
    let reordered = format!("{header}\n{}\n", rows.join("\n"));
    assert_eq!(replay(FIXTURE, 40, 40).unwrap().render(), replay(&reordered, 40, 40).unwrap().render());
}

#[test]
fn duplicate_event_ids_are_rejected() {
    let duplicate = format!("{FIXTURE}rename-1|RENAME|asset-a|asset-a|11|11|registry-v1|owner-a|evidence-x|reviewer-a|rules-v1\n");
    assert!(replay(&duplicate, 100, 100).is_err());
}

#[test]
fn every_event_requires_evidence_and_adjudication_metadata() {
    let invalid = FIXTURE.replace("|reviewer-a|rules-v1", "||rules-v1");
    assert!(replay(&invalid, 100, 100).is_err());
}
