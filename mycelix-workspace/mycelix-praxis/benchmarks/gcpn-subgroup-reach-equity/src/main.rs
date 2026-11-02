use gcpn_civil_evidence_contracts::{
    AuthorityAssessment, BaseAuditAssessment, CaseFrameAssessment, CivilEvidenceReport,
    DeliveryAssessment, DeliveryCounts, RegistryAuthorityStatus, SignatureStatus,
};
use gcpn_compositional_assurance::ClaimDisposition;
use gcpn_subgroup_reach_equity::{
    audit_subgroup_reach, DeduplicationAttestation, DeduplicationVerification,
    DenominatorState, SubgroupDimension, SubgroupReachEvidence, SubgroupReachPolicy,
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let policy = SubgroupReachPolicy {
        as_of_cutoff: 100,
        expected_denominator_source_id: "municipal-register".into(),
        expected_denominator_source_version: "snapshot-2026-10".into(),
        expected_dedup_source_id: "privacy-union-service".into(),
        expected_dedup_source_version: "release-3".into(),
        expected_dedup_method_id: "aggregate-union".into(),
        expected_dedup_method_version: "v1".into(),
        minimum_group_coverage: 0.95,
        maximum_coverage_gap: 0.05,
    };

    let report = synthetic_delivery_report();
    let groups = vec![
        subgroup(
            "rural",
            "cohort-rural",
            &["cell-broadcast", "sms"],
            100,
            105,
            180,
            70,
        ),
        subgroup(
            "urban",
            "cohort-urban",
            &["cell-broadcast", "sms"],
            150,
            160,
            180,
            155,
        ),
    ];

    let result = audit_subgroup_reach(&report, &groups, &policy)?;

    println!("# Synthetic subgroup reach and equity assessment.");
    println!("# Aggregate-only inputs; no personal recipient identifiers or persistent tokens.");
    println!("# Multi-channel unique reach uses an external aggregate union attestation.");
    println!("# Coverage bounds use unique_reached / [denominator_upper, denominator_lower].");
    print!("{}", result.to_csv());
    println!(
        "# groups={} supports={} refutes={} indeterminate={} unavailable={} equity={:?}",
        result.assessed_group_count,
        result.supports_group_count,
        result.refutes_group_count,
        result.indeterminate_group_count,
        result.unavailable_group_count,
        result.equity_disposition
    );

    Ok(())
}

fn subgroup(
    category: &str,
    cohort: &str,
    channels: &[&str],
    denominator_lower: u64,
    denominator_upper: u64,
    channel_sum: u64,
    unique_reached: u64,
) -> SubgroupReachEvidence {
    SubgroupReachEvidence {
        group_id: category.into(),
        cohort_id: cohort.into(),
        dimensions: vec![
            SubgroupDimension {
                dimension_id: "rurality".into(),
                category_id: category.into(),
            },
            SubgroupDimension {
                dimension_id: "language".into(),
                category_id: "local-language".into(),
            },
        ],
        channel_ids: channels.iter().map(|channel| (*channel).into()).collect(),
        denominator: DenominatorState::KnownInterval {
            lower: denominator_lower,
            upper: denominator_upper,
            source_id: "municipal-register".into(),
            source_version: "snapshot-2026-10".into(),
            recorded_at: 80,
        },
        deduplication: Some(DeduplicationAttestation {
            attestation_id: format!("union-{category}"),
            source_id: "privacy-union-service".into(),
            source_version: "release-3".into(),
            method_id: "aggregate-union".into(),
            method_version: "v1".into(),
            input_channel_ids: channels.iter().map(|channel| (*channel).into()).collect(),
            channel_count_sum: channel_sum,
            unique_union_count: unique_reached,
            overlap_removed: channel_sum - unique_reached,
            attestation_digest: format!("opaque-digest-{category}"),
            verification: DeduplicationVerification::Verified,
            event_time: 90,
            recorded_at: 95,
        }),
    }
}

fn synthetic_delivery_report() -> CivilEvidenceReport {
    let deliveries = vec![
        delivery("cohort-rural", "cell-broadcast", 100),
        delivery("cohort-rural", "sms", 80),
        delivery("cohort-urban", "cell-broadcast", 100),
        delivery("cohort-urban", "sms", 80),
    ];

    CivilEvidenceReport {
        scenario_id: "synthetic-subgroup-reach".into(),
        as_of_cutoff: 100,
        base_audit: BaseAuditAssessment {
            disposition: ClaimDisposition::Supports,
            required_claims: 1,
            supporting_claims: 1,
            violations: Vec::new(),
        },
        authority: AuthorityAssessment {
            disposition: ClaimDisposition::Supports,
            assertion_id: Some("authority-1".into()),
            registry_status: Some(RegistryAuthorityStatus::Recognized),
            signature_status: Some(SignatureStatus::Verified),
            violations: Vec::new(),
        },
        delivery: deliveries,
        accessibility: Vec::new(),
        comprehension: Vec::new(),
        actions: Vec::new(),
        case_frame: CaseFrameAssessment {
            frame_id: "synthetic-frame".into(),
            disposition: ClaimDisposition::Supports,
            eligible_cases: 1,
            observed_cases: 1,
            missing_cases: 0,
            disputed_cases: 0,
            excluded_cases: 0,
            violations: Vec::new(),
        },
    }
}

fn delivery(cohort_id: &str, channel_id: &str, delivered: u64) -> DeliveryAssessment {
    DeliveryAssessment {
        cohort_id: cohort_id.into(),
        channel_id: channel_id.into(),
        disposition: if delivered == 100 {
            ClaimDisposition::Supports
        } else {
            ClaimDisposition::Indeterminate
        },
        counts: DeliveryCounts {
            intended_receivers: 100,
            provider_accepted: 100,
            device_delivered: delivered,
            failed: 0,
            delayed: 0,
            unknown: 0,
            duplicates: 0,
            not_confirmed_device_delivered: 100 - delivered,
        },
        violations: Vec::new(),
    }
}
