include!("regenerative_successor_depth_fixture.rs");

use commons_types::{
    verify_regenerative_multigeneration_depth_evidence,
    RegenerativeMultigenerationDepthEvidenceV1,
    REGENERATIVE_MULTIGENERATION_DEPTH_SCHEMA_V1,
};

fn multigeneration_evidence() -> RegenerativeMultigenerationDepthEvidenceV1 {
    let parent = depth_evidence();
    RegenerativeMultigenerationDepthEvidenceV1 {
        schema_version: REGENERATIVE_MULTIGENERATION_DEPTH_SCHEMA_V1,
        multigeneration_id: "manta-forge-multigeneration-depth-v1".into(),
        parent_successor_depth_content_digest: parent.content_digest().unwrap(),
        symtropy_multigeneration_binding:
            "symtropy-pr:787:d2453d2818b87cbfb3a22e979c9d1b77391fc686".into(),
        symthaea_multigeneration_binding:
            "symthaea-pr:1917:a2831ba8cc4ce910a56a9087fc4b517083ecd512".into(),
        reproduction_policy_binding:
            "reproduction-policy:one-complete-period-bootstrap-v1".into(),
        reproductive_case_count: 18,
        initial_handoff_count: 18,
        descendant_handoff_count: 22,
        total_successful_handoff_count: 40,
        generation_depth_buckets: parent.depth_buckets.clone(),
        temporal_depth_matches_generation_depth: true,
    }
}

#[test]
fn multigeneration_record_composes_exactly_with_successor_depth() {
    let parent = depth_evidence();
    let record = multigeneration_evidence();
    verify_regenerative_multigeneration_depth_evidence(&parent, &record).unwrap();

    assert_eq!(record.parent_successor_depth_content_digest, parent.content_digest().unwrap());
    assert_eq!(record.reproductive_case_count, 18);
    assert_eq!(record.initial_handoff_count, 18);
    assert_eq!(record.descendant_handoff_count, 22);
    assert_eq!(record.total_successful_handoff_count, 40);
    assert_eq!(record.max_generation_depth(), 4);
    assert!(record.temporal_depth_matches_generation_depth);

    let weighted: u64 = record
        .generation_depth_buckets
        .iter()
        .map(|bucket| bucket.complete_periods * u64::from(bucket.case_count))
        .sum();
    assert_eq!(weighted, 40);
}

#[test]
fn exact_multigeneration_subjects_and_policy_are_bound() {
    let record = multigeneration_evidence();
    assert_eq!(
        record.symtropy_multigeneration_binding,
        "symtropy-pr:787:d2453d2818b87cbfb3a22e979c9d1b77391fc686"
    );
    assert_eq!(
        record.symthaea_multigeneration_binding,
        "symthaea-pr:1917:a2831ba8cc4ce910a56a9087fc4b517083ecd512"
    );
    assert_eq!(
        record.reproduction_policy_binding,
        "reproduction-policy:one-complete-period-bootstrap-v1"
    );
}

#[test]
fn handoff_arithmetic_and_equivalence_claim_fail_closed() {
    let parent = depth_evidence();

    let mut bad_total = multigeneration_evidence();
    bad_total.total_successful_handoff_count = 39;
    assert!(bad_total.validate().is_err());

    let mut divergent = multigeneration_evidence();
    divergent.generation_depth_buckets[0].case_count = 5;
    divergent.generation_depth_buckets[1].case_count = 6;
    assert!(
        verify_regenerative_multigeneration_depth_evidence(&parent, &divergent).is_err()
    );
}

#[test]
fn multigeneration_provenance_fits_existing_maritime_transport() {
    let record = multigeneration_evidence();
    let envelope = record
        .to_maritime_envelope(
            "manta-lineage-frontier",
            3,
            0,
            1_789_230_000_000_000,
            "evidence:multigeneration-lineage-depth-event-v1",
        )
        .unwrap();
    let decoded: RegenerativeMultigenerationDepthEvidenceV1 =
        serde_json::from_str(&envelope.payload_json).unwrap();
    assert_eq!(decoded, record);
}
