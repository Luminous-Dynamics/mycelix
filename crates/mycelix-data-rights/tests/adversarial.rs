use mycelix_data_rights::{
    AssessmentError, AssessmentState, ConflictEvidenceError, EvidenceDisposition, EvidenceRef,
    ExternalPrerequisite, IntendedUseProfileV1, Obligation, ReviewedTermsEvidenceV1, SourceTermsEvidenceV1,
    SplitRole, SubjectId, TermsAssessmentV1, TermsProfileCurrentness, UseDimension,
    assess_terms_compatibility,
};

fn subject(value: &str) -> SubjectId {
    SubjectId::new(value).expect("test subject")
}

fn evidence_ref(value: &str) -> EvidenceRef {
    EvidenceRef::new(value).expect("test evidence ref")
}

fn request(
    subjects: &[&str],
    split: SplitRole,
    dimensions: &[UseDimension],
    prerequisites: &[ExternalPrerequisite],
) -> IntendedUseProfileV1 {
    IntendedUseProfileV1::new(
        subjects.iter().map(|value| subject(value)).collect(),
        split,
        dimensions.iter().copied(),
        prerequisites.iter().copied(),
    )
    .expect("valid test request")
}

fn row(
    subject_id: &str,
    reference: Option<&str>,
    dimensions: &[(UseDimension, EvidenceDisposition)],
    obligations: &[Obligation],
) -> SourceTermsEvidenceV1 {
    SourceTermsEvidenceV1::new(
        subject(subject_id),
        reference.map(evidence_ref),
        dimensions.iter().copied(),
        obligations.iter().copied(),
    )
    .expect("valid test evidence")
}

fn evidence(
    currentness: TermsProfileCurrentness,
    rows: Vec<SourceTermsEvidenceV1>,
) -> ReviewedTermsEvidenceV1 {
    ReviewedTermsEvidenceV1::new(currentness, rows).expect("valid test evidence set")
}

#[test]
fn positive_assessment_preserves_obligations_and_privacy_prerequisite() {
    let intended = request(
        &["synthetic:human:p:v1"],
        SplitRole::Training,
        &[UseDimension::Training],
        &[ExternalPrerequisite::PrivacyConsentAssessment],
    );
    let reviewed = evidence(
        TermsProfileCurrentness::Current,
        vec![row(
            "synthetic:human:p:v1",
            None,
            &[(UseDimension::Training, EvidenceDisposition::Compatible)],
            &[Obligation::SourceLineageRetention],
        )],
    );

    let result = assess_terms_compatibility(&intended, &reviewed).expect("assessment");
    assert_eq!(
        result.state(),
        AssessmentState::CompatibleWithReviewedTermsProfile
    );
    assert!(
        result
            .context()
            .obligations()
            .contains(&Obligation::SourceLineageRetention)
    );
    assert!(
        result
            .context()
            .external_prerequisites()
            .contains(&ExternalPrerequisite::PrivacyConsentAssessment)
    );
}

#[test]
fn evaluation_only_and_benchmark_holdout_block_training() {
    for split in [SplitRole::EvaluationOnly, SplitRole::BenchmarkHoldout] {
        let intended = request(
            &["synthetic:dataset:q:v1"],
            split,
            &[UseDimension::Training],
            &[],
        );
        let reviewed = evidence(
            TermsProfileCurrentness::Current,
            vec![row(
                "synthetic:dataset:q:v1",
                None,
                &[(UseDimension::Training, EvidenceDisposition::Compatible)],
                &[],
            )],
        );
        assert_eq!(
            assess_terms_compatibility(&intended, &reviewed)
                .expect("assessment")
                .state(),
            AssessmentState::IncompatibleWithReviewedTermsProfile
        );
    }
}

#[test]
fn unknown_expired_and_subject_mismatch_fail_closed() {
    let intended = request(
        &["synthetic:dataset:g:v2"],
        SplitRole::Training,
        &[UseDimension::Training],
        &[],
    );
    let base_row = row(
        "synthetic:dataset:g:v1",
        None,
        &[(UseDimension::Training, EvidenceDisposition::Compatible)],
        &[Obligation::ReviewOnTermsChange],
    );

    assert_eq!(
        assess_terms_compatibility(
            &intended,
            &evidence(TermsProfileCurrentness::Unknown, vec![base_row.clone()]),
        )
        .expect("assessment")
        .state(),
        AssessmentState::HumanReviewRequired
    );
    assert_eq!(
        assess_terms_compatibility(
            &intended,
            &evidence(TermsProfileCurrentness::Expired, vec![base_row.clone()]),
        )
        .expect("assessment")
        .state(),
        AssessmentState::TermsProfileExpiredOrSuperseded
    );
    assert_eq!(
        assess_terms_compatibility(
            &intended,
            &evidence(TermsProfileCurrentness::Current, vec![base_row]),
        )
        .expect("assessment")
        .state(),
        AssessmentState::IntendedUseOutOfProfile
    );
}

#[test]
fn obligations_union_across_sources_and_order_is_irrelevant() {
    let intended = request(
        &["synthetic:i:v1", "synthetic:j:v1"],
        SplitRole::Training,
        &[UseDimension::Training],
        &[],
    );
    let left = row(
        "synthetic:i:v1",
        None,
        &[(UseDimension::Training, EvidenceDisposition::Compatible)],
        &[Obligation::Attribution],
    );
    let right = row(
        "synthetic:j:v1",
        None,
        &[(UseDimension::Training, EvidenceDisposition::Compatible)],
        &[
            Obligation::NoRawRedistribution,
            Obligation::SourceLineageRetention,
        ],
    );

    let first = assess_terms_compatibility(
        &intended,
        &evidence(
            TermsProfileCurrentness::Current,
            vec![left.clone(), right.clone()],
        ),
    )
    .expect("assessment");
    let reversed = assess_terms_compatibility(
        &intended,
        &evidence(TermsProfileCurrentness::Current, vec![right, left]),
    )
    .expect("assessment");

    assert_eq!(first, reversed);
    assert_eq!(
        first.state(),
        AssessmentState::CompatibleWithReviewedTermsProfile
    );
    assert_eq!(first.context().obligations().len(), 3);
}

#[test]
fn restrictive_source_cannot_be_washed_by_permissive_source() {
    let intended = request(
        &["synthetic:k:v1", "synthetic:l:v1"],
        SplitRole::Training,
        &[UseDimension::CommercialProductDevelopment],
        &[],
    );
    let reviewed = evidence(
        TermsProfileCurrentness::Current,
        vec![
            row(
                "synthetic:k:v1",
                None,
                &[(
                    UseDimension::CommercialProductDevelopment,
                    EvidenceDisposition::Incompatible,
                )],
                &[Obligation::NonCommercialUseOnly],
            ),
            row(
                "synthetic:l:v1",
                None,
                &[(
                    UseDimension::CommercialProductDevelopment,
                    EvidenceDisposition::Compatible,
                )],
                &[],
            ),
        ],
    );

    let result = assess_terms_compatibility(&intended, &reviewed).expect("assessment");
    assert_eq!(
        result.state(),
        AssessmentState::IncompatibleWithReviewedTermsProfile
    );
    assert!(
        result
            .context()
            .obligations()
            .contains(&Obligation::NonCommercialUseOnly)
    );
}

#[test]
fn same_subject_same_dimension_contradiction_is_conflict() {
    let intended = request(
        &["synthetic:t:v1"],
        SplitRole::Training,
        &[UseDimension::Training],
        &[],
    );
    let reviewed = evidence(
        TermsProfileCurrentness::Current,
        vec![
            row(
                "synthetic:t:v1",
                Some("synthetic:review:t:a"),
                &[(UseDimension::Training, EvidenceDisposition::Compatible)],
                &[Obligation::SourceLineageRetention],
            ),
            row(
                "synthetic:t:v1",
                Some("synthetic:review:t:b"),
                &[(UseDimension::Training, EvidenceDisposition::Incompatible)],
                &[Obligation::ReviewOnTermsChange],
            ),
        ],
    );

    let result = assess_terms_compatibility(&intended, &reviewed).expect("assessment");
    assert_eq!(result.state(), AssessmentState::TermsEvidenceConflict);
    let TermsAssessmentV1::TermsEvidenceConflict(conflict) = result else {
        panic!("expected conflict")
    };
    assert_eq!(conflict.conflicts().len(), 1);
    assert_eq!(conflict.conflicts()[0].evidence_refs().len(), 2);
}

#[test]
fn contradictory_records_require_distinct_provenance() {
    let intended = request(
        &["synthetic:t:v1"],
        SplitRole::Training,
        &[UseDimension::Training],
        &[],
    );

    let missing = evidence(
        TermsProfileCurrentness::Current,
        vec![
            row(
                "synthetic:t:v1",
                None,
                &[(UseDimension::Training, EvidenceDisposition::Compatible)],
                &[],
            ),
            row(
                "synthetic:t:v1",
                Some("synthetic:review:t:b"),
                &[(UseDimension::Training, EvidenceDisposition::Incompatible)],
                &[],
            ),
        ],
    );
    assert_eq!(
        assess_terms_compatibility(&intended, &missing),
        Err(AssessmentError::MalformedConflictEvidence {
            subject: subject("synthetic:t:v1"),
            dimension: UseDimension::Training,
            reason: ConflictEvidenceError::MissingEvidenceRef,
        })
    );

    let duplicate = evidence(
        TermsProfileCurrentness::Current,
        vec![
            row(
                "synthetic:t:v1",
                Some("synthetic:review:t:same"),
                &[(UseDimension::Training, EvidenceDisposition::Compatible)],
                &[],
            ),
            row(
                "synthetic:t:v1",
                Some("synthetic:review:t:same"),
                &[(UseDimension::Training, EvidenceDisposition::Incompatible)],
                &[],
            ),
        ],
    );
    assert_eq!(
        assess_terms_compatibility(&intended, &duplicate),
        Err(AssessmentError::MalformedConflictEvidence {
            subject: subject("synthetic:t:v1"),
            dimension: UseDimension::Training,
            reason: ConflictEvidenceError::DuplicateEvidenceRef,
        })
    );
}

#[test]
fn different_subject_restrictions_are_composition_not_conflict() {
    let intended = request(
        &["synthetic:t:v1", "synthetic:u:v1"],
        SplitRole::Training,
        &[UseDimension::Training],
        &[],
    );
    let reviewed = evidence(
        TermsProfileCurrentness::Current,
        vec![
            row(
                "synthetic:t:v1",
                Some("synthetic:review:t"),
                &[(UseDimension::Training, EvidenceDisposition::Compatible)],
                &[],
            ),
            row(
                "synthetic:u:v1",
                Some("synthetic:review:u"),
                &[(UseDimension::Training, EvidenceDisposition::Incompatible)],
                &[],
            ),
        ],
    );

    assert_eq!(
        assess_terms_compatibility(&intended, &reviewed)
            .expect("assessment")
            .state(),
        AssessmentState::IncompatibleWithReviewedTermsProfile
    );
}

#[test]
fn missing_requested_dimension_is_out_of_profile_not_compatible() {
    let intended = request(
        &["synthetic:r:v1"],
        SplitRole::Training,
        &[UseDimension::Training, UseDimension::DerivedModelDistribution],
        &[],
    );
    let reviewed = evidence(
        TermsProfileCurrentness::Current,
        vec![row(
            "synthetic:r:v1",
            None,
            &[(UseDimension::Training, EvidenceDisposition::Compatible)],
            &[Obligation::Attribution],
        )],
    );

    assert_eq!(
        assess_terms_compatibility(&intended, &reviewed)
            .expect("assessment")
            .state(),
        AssessmentState::IntendedUseOutOfProfile
    );
}
