use mycelix_core_types::systemic::ValidityInterval;

#[test]
fn completely_unknown_interval_does_not_establish_definite_containment() {
    let interval = ValidityInterval::default();

    assert!(
        !interval.contains(0),
        "unknown start + unknown end must not establish definite containment"
    );
}

#[test]
fn unknown_start_does_not_establish_definite_containment_before_known_end() {
    let interval = ValidityInterval::new(None, Some(100)).expect("valid interval");

    assert!(
        !interval.contains(50),
        "missing start is epistemic uncertainty, not known -infinity"
    );
    assert!(
        !interval.contains(100),
        "the known half-open end must still exclude its boundary"
    );
}

#[test]
fn unknown_end_does_not_establish_definite_containment_after_known_start() {
    let interval = ValidityInterval::new(Some(-100), None).expect("valid interval");

    assert!(
        !interval.contains(50),
        "missing end is epistemic uncertainty, not known +infinity"
    );
    assert!(
        !interval.contains(-101),
        "the known start must still exclude earlier timestamps"
    );
}

#[test]
fn fully_known_interval_can_establish_containment() {
    let interval = ValidityInterval::new(Some(-100), Some(100)).expect("valid interval");

    assert!(interval.contains(0));
    assert!(!interval.contains(-101));
    assert!(!interval.contains(100));
}