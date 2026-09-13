use election_adversarial_corpus::{
    CorpusLayer, PROTOCOL_DEPENDENT_GAPS, REQUIRED_EXECUTABLE_CASES,
};
use election_integrity_types::PUBLIC_ELECTION_THEOREMS;

const REQUIRED_EXECUTABLE_LAYERS: [CorpusLayer; 13] = [
    CorpusLayer::AnonymousAuthority,
    CorpusLayer::TransparencyLineage,
    CorpusLayer::WitnessQuorum,
    CorpusLayer::EvidencePackage,
    CorpusLayer::OfflineExecution,
    CorpusLayer::VerifierRun,
    CorpusLayer::VerifierAgreement,
    CorpusLayer::PhysicalAccounting,
    CorpusLayer::PhysicalCustody,
    CorpusLayer::CaptureReconciliation,
    CorpusLayer::AuditPolicy,
    CorpusLayer::AuditEvidence,
    CorpusLayer::CrossLayerComposition,
];

#[test]
fn every_assurance_layer_retains_executable_attack_coverage() {
    for required_layer in REQUIRED_EXECUTABLE_LAYERS {
        assert!(
            REQUIRED_EXECUTABLE_CASES
                .iter()
                .any(|case| case.layer == required_layer),
            "missing executable adversarial coverage for layer {required_layer:?}"
        );
    }
}

#[test]
fn every_certification_theorem_is_executable_or_explicitly_blocked() {
    for theorem in PUBLIC_ELECTION_THEOREMS {
        assert!(theorem.required_for_certification);

        let executable = REQUIRED_EXECUTABLE_CASES
            .iter()
            .any(|case| case.primary_theorem == theorem.id);
        let explicitly_blocked = PROTOCOL_DEPENDENT_GAPS
            .iter()
            .any(|gap| gap.primary_theorem == theorem.id && gap.blocks_protocol_security_claim);

        assert!(
            executable || explicitly_blocked,
            "certification theorem {:?} has neither an executable attack nor an explicit protocol blocker",
            theorem.id
        );
    }
}

#[test]
fn corpus_contains_single_fault_and_composed_failure_cases() {
    assert!(
        REQUIRED_EXECUTABLE_CASES
            .iter()
            .any(|case| case.single_fault)
    );
    assert!(
        REQUIRED_EXECUTABLE_CASES
            .iter()
            .any(|case| !case.single_fault)
    );
}

#[test]
fn protocol_gaps_never_count_as_executable_attack_coverage() {
    for gap in PROTOCOL_DEPENDENT_GAPS {
        assert!(gap.blocks_protocol_security_claim);
    }

    let executable_case_count = REQUIRED_EXECUTABLE_CASES.len();
    let protocol_gap_count = PROTOCOL_DEPENDENT_GAPS.len();
    assert_eq!(executable_case_count, 26);
    assert_eq!(protocol_gap_count, 7);
}
