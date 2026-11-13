const EXEC_INTEGRITY: &str = include_str!("../../../zomes/execution/integrity/src/lib.rs");
const EXEC_COORDINATOR: &str = include_str!("../../../zomes/execution/coordinator/src/lib.rs");
const CONSTITUTION_V025: &str = include_str!(
    "../../../../mycelix-core/docs/architecture/THE MYCELIX SPORE CONSTITUTION (v0.24).md"
);
const AMENDMENT_V026: &str =
    include_str!("../../../../mycelix-core/docs/architecture/CONSTITUTION_V026_AMENDMENTS.md");
const CONSTITUTIONAL_ALIGNMENT: &str = include_str!("../../../CONSTITUTIONAL_ALIGNMENT.md");
const ANTI_TYRANNY_DESIGN: &str = include_str!("../../../ANTI_TYRANNY_DESIGN.md");

#[test]
fn current_runtime_threshold_is_the_fixed_two_thirds_value() {
    assert!(EXEC_INTEGRITY.contains("pub const VETO_OVERRIDE_THRESHOLD: f64 = 0.67;"));
    assert!(
        EXEC_COORDINATOR.contains("override_ratio >= execution_integrity::VETO_OVERRIDE_THRESHOLD")
    );
}

#[test]
fn adaptive_threshold_helper_is_not_mistaken_for_runtime_behavior() {
    assert!(EXEC_INTEGRITY.contains("pub fn adaptive_override_threshold"));
    assert!(!EXEC_COORDINATOR.contains("adaptive_override_threshold"));
    assert!(ANTI_TYRANNY_DESIGN.contains("DRAFT POLICY; NOT ACTIVE IN THE RESOLVER"));
    assert!(CONSTITUTIONAL_ALIGNMENT.contains("must remain draft-only"));
}

#[test]
fn current_runtime_comments_do_not_claim_an_eighty_percent_threshold() {
    assert!(!EXEC_INTEGRITY.contains("always 0.80"));
    assert!(!EXEC_INTEGRITY.contains("exactly 0.80"));
    assert!(!CONSTITUTIONAL_ALIGNMENT.contains("**Code**: `VETO_OVERRIDE_THRESHOLD = 0.80` (80%)"));
}

#[test]
fn v025_two_house_rule_is_distinguished_from_v026_draft_insurance() {
    assert!(CONSTITUTION_V025.contains("two-thirds (⅔) majority in both houses"));
    assert!(AMENDMENT_V026.contains("Attempt 2: 62%"));
    assert!(AMENDMENT_V026.contains("60% majority (absolute floor)"));
    assert!(
        CONSTITUTIONAL_ALIGNMENT.contains("current resolver computes one aggregate weighted ratio")
    );
}

#[test]
fn eligible_voter_denominator_mismatch_remains_explicitly_disclosed() {
    assert!(EXEC_COORDINATOR.contains("total_eligible_voters: voter_count"));
    assert!(
        CONSTITUTIONAL_ALIGNMENT.contains("not an independently established eligible-voter count")
    );
}
