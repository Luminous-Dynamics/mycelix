use constitutional_qualification_receipt::QualificationReceiptV1;
use std::{env, fs};

fn required(name: &str) -> String {
    env::var(name).unwrap_or_else(|_| panic!("{name} must be set"))
}

#[test]
#[ignore = "trusted-builder external fixture only"]
fn external_receipt_matches_trusted_builder_context() {
    let receipt_path = required("QREC_RECEIPT_PATH");
    let bytes = fs::read(&receipt_path)
        .unwrap_or_else(|err| panic!("could not read QREC_RECEIPT_PATH {receipt_path}: {err}"));
    let receipt: QualificationReceiptV1 =
        serde_json::from_slice(&bytes).expect("receipt must deserialize as QualificationReceiptV1");

    receipt
        .validate()
        .expect("receipt must satisfy QREC-001 structural and commitment semantics");

    assert_eq!(receipt.dependency_id, required("QREC_EXPECTED_DEPENDENCY_ID"));
    assert_eq!(receipt.issuer_id, required("QREC_EXPECTED_ISSUER_ID"));
    assert_eq!(receipt.semantic_head, required("QREC_EXPECTED_SEMANTIC_HEAD"));
    assert_eq!(receipt.verifier_head, required("QREC_EXPECTED_VERIFIER_HEAD"));
    assert_eq!(
        receipt.run_id,
        required("QREC_EXPECTED_RUN_ID")
            .parse::<u64>()
            .expect("QREC_EXPECTED_RUN_ID must be u64"),
    );
    assert_eq!(
        receipt.run_attempt,
        required("QREC_EXPECTED_RUN_ATTEMPT")
            .parse::<u32>()
            .expect("QREC_EXPECTED_RUN_ATTEMPT must be u32"),
    );
    assert_eq!(
        receipt.artifact_digest,
        required("QREC_EXPECTED_ARTIFACT_DIGEST")
    );

    println!("QREC_VALIDATED_RECEIPT_COMMITMENT={}", receipt.receipt_commitment);
}
