use serde::{Deserialize, Serialize};

use crate::{QualificationReceiptDigestV1, Sha256DigestV1};

pub const VERIFIER_EXECUTION_EVIDENCE_VERSION_V1: u32 = 1;
pub const MAX_VERIFIER_EXECUTION_IDENTITY_BYTES_V1: usize = 1024;
pub const MAX_VERIFIED_RESULTS_V1: u32 = 8;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum VerifierExecutionAuthorityV1 {
    ExecutionEvidenceOnly,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NixVerifierClosureIdentityV1 {
    pub store_path: String,
    pub closure_digest: Sha256DigestV1,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct VerifierExecutableIdentityV1 {
    pub profile_id: String,
    pub backend_family: String,
    pub semantic_version: String,
    pub executable_sha256: Sha256DigestV1,
    pub platform_profile: String,
    pub command_profile_id: String,
    pub nix_closure: Option<NixVerifierClosureIdentityV1>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum VerifierTrustRootModeV1 {
    OnlineFetchThenRetainedVerifyV1,
    OfflinePinnedRootV1,
}

/// Exact process termination observed for the verifier invocation.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum VerifierProcessOutcomeV1 {
    ExitCode(i32),
    TerminatedBySignal(i32),
    TimedOut,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum VerifierExecutionEvidenceErrorV1 {
    UnsupportedEvidenceVersion { actual: u32 },
    EmptyField { field: &'static str },
    FieldTooLong {
        field: &'static str,
        maximum: usize,
        actual: usize,
    },
    InvalidNixStorePath,
    TrustRootAcquiredBeforeExecutionWindow,
    TrustRootAcquiredAfterExecution,
    InvalidExecutionWindow,
    TooManyVerifiedResults { maximum: u32, actual: u32 },
}

/// Non-authoritative structural evidence for an exact verifier process sequence
/// before strict verifier-output parsing establishes a result count.
///
/// This record is intentionally serializable provenance. It is not proof that the
/// process actually ran; only the separately sealed executor capability can establish
/// process-local execution origin.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct VerifierProcessExecutionReceiptV1 {
    pub evidence_version: u32,
    pub verifier: VerifierExecutableIdentityV1,
    pub canonical_receipt_digest: QualificationReceiptDigestV1,
    pub attestation_bundle_digest: Sha256DigestV1,
    pub trust_root_mode: VerifierTrustRootModeV1,
    pub trusted_root_material_digest: Sha256DigestV1,
    pub trusted_root_acquired_at_unix_seconds: u64,
    pub command_arguments_digest: Sha256DigestV1,
    pub environment_profile_digest: Sha256DigestV1,
    pub verifier_stdout_digest: Sha256DigestV1,
    pub verifier_stderr_digest: Sha256DigestV1,
    pub execution_started_at_unix_seconds: u64,
    pub execution_completed_at_unix_seconds: u64,
    pub process_outcome: VerifierProcessOutcomeV1,
}

/// Non-authoritative evidence describing one exact verifier execution after strict
/// output parsing has also established the exact candidate count.
///
/// This record can be serialized/deserialized for provenance. It is not a
/// cryptographic capability and cannot establish receipt authentication by itself.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct VerifierExecutionReceiptV1 {
    pub evidence_version: u32,
    pub verifier: VerifierExecutableIdentityV1,
    pub canonical_receipt_digest: QualificationReceiptDigestV1,
    pub attestation_bundle_digest: Sha256DigestV1,
    pub trust_root_mode: VerifierTrustRootModeV1,
    pub trusted_root_material_digest: Sha256DigestV1,
    pub trusted_root_acquired_at_unix_seconds: u64,
    pub command_arguments_digest: Sha256DigestV1,
    pub environment_profile_digest: Sha256DigestV1,
    pub verifier_stdout_digest: Sha256DigestV1,
    pub verifier_stderr_digest: Sha256DigestV1,
    pub execution_started_at_unix_seconds: u64,
    pub execution_completed_at_unix_seconds: u64,
    pub process_outcome: VerifierProcessOutcomeV1,
    pub parsed_result_count: u32,
}

impl VerifierExecutableIdentityV1 {
    pub fn validate(&self) -> Result<(), VerifierExecutionEvidenceErrorV1> {
        check_text("verifier.profile_id", &self.profile_id)?;
        check_text("verifier.backend_family", &self.backend_family)?;
        check_text("verifier.semantic_version", &self.semantic_version)?;
        check_text("verifier.platform_profile", &self.platform_profile)?;
        check_text("verifier.command_profile_id", &self.command_profile_id)?;
        if let Some(nix) = &self.nix_closure {
            check_text("verifier.nix_closure.store_path", &nix.store_path)?;
            if !nix.store_path.starts_with("/nix/store/") {
                return Err(VerifierExecutionEvidenceErrorV1::InvalidNixStorePath);
            }
        }
        Ok(())
    }
}

impl VerifierProcessExecutionReceiptV1 {
    pub fn validate(&self) -> Result<(), VerifierExecutionEvidenceErrorV1> {
        self.clone().into_parsed_receipt(0)?.validate()
    }

    pub const fn authority_scope(&self) -> VerifierExecutionAuthorityV1 {
        VerifierExecutionAuthorityV1::ExecutionEvidenceOnly
    }

    pub const fn process_exited_successfully(&self) -> bool {
        matches!(self.process_outcome, VerifierProcessOutcomeV1::ExitCode(0))
    }

    pub const fn establishes_parsed_result_count(&self) -> bool {
        false
    }

    pub const fn establishes_receipt_authentication(&self) -> bool {
        false
    }

    pub const fn grants_production_authority(&self) -> bool {
        false
    }

    pub const fn grants_application_authority(&self) -> bool {
        false
    }

    pub(crate) fn into_parsed_receipt(
        self,
        parsed_result_count: u32,
    ) -> Result<VerifierExecutionReceiptV1, VerifierExecutionEvidenceErrorV1> {
        let receipt = VerifierExecutionReceiptV1 {
            evidence_version: self.evidence_version,
            verifier: self.verifier,
            canonical_receipt_digest: self.canonical_receipt_digest,
            attestation_bundle_digest: self.attestation_bundle_digest,
            trust_root_mode: self.trust_root_mode,
            trusted_root_material_digest: self.trusted_root_material_digest,
            trusted_root_acquired_at_unix_seconds: self.trusted_root_acquired_at_unix_seconds,
            command_arguments_digest: self.command_arguments_digest,
            environment_profile_digest: self.environment_profile_digest,
            verifier_stdout_digest: self.verifier_stdout_digest,
            verifier_stderr_digest: self.verifier_stderr_digest,
            execution_started_at_unix_seconds: self.execution_started_at_unix_seconds,
            execution_completed_at_unix_seconds: self.execution_completed_at_unix_seconds,
            process_outcome: self.process_outcome,
            parsed_result_count,
        };
        receipt.validate()?;
        Ok(receipt)
    }
}

impl VerifierExecutionReceiptV1 {
    pub fn validate(&self) -> Result<(), VerifierExecutionEvidenceErrorV1> {
        if self.evidence_version != VERIFIER_EXECUTION_EVIDENCE_VERSION_V1 {
            return Err(VerifierExecutionEvidenceErrorV1::UnsupportedEvidenceVersion {
                actual: self.evidence_version,
            });
        }

        self.verifier.validate()?;

        if self.execution_completed_at_unix_seconds < self.execution_started_at_unix_seconds {
            return Err(VerifierExecutionEvidenceErrorV1::InvalidExecutionWindow);
        }

        match self.trust_root_mode {
            VerifierTrustRootModeV1::OnlineFetchThenRetainedVerifyV1 => {
                if self.trusted_root_acquired_at_unix_seconds
                    < self.execution_started_at_unix_seconds
                {
                    return Err(
                        VerifierExecutionEvidenceErrorV1::TrustRootAcquiredBeforeExecutionWindow,
                    );
                }
            }
            VerifierTrustRootModeV1::OfflinePinnedRootV1 => {
                if self.trusted_root_acquired_at_unix_seconds
                    > self.execution_started_at_unix_seconds
                {
                    return Err(
                        VerifierExecutionEvidenceErrorV1::TrustRootAcquiredAfterExecution,
                    );
                }
            }
        }

        if self.trusted_root_acquired_at_unix_seconds
            > self.execution_completed_at_unix_seconds
        {
            return Err(VerifierExecutionEvidenceErrorV1::TrustRootAcquiredAfterExecution);
        }

        if self.parsed_result_count > MAX_VERIFIED_RESULTS_V1 {
            return Err(VerifierExecutionEvidenceErrorV1::TooManyVerifiedResults {
                maximum: MAX_VERIFIED_RESULTS_V1,
                actual: self.parsed_result_count,
            });
        }

        Ok(())
    }

    pub const fn authority_scope(&self) -> VerifierExecutionAuthorityV1 {
        VerifierExecutionAuthorityV1::ExecutionEvidenceOnly
    }

    pub const fn process_exited_successfully(&self) -> bool {
        matches!(self.process_outcome, VerifierProcessOutcomeV1::ExitCode(0))
    }

    pub const fn establishes_receipt_authentication(&self) -> bool {
        false
    }

    pub const fn grants_production_authority(&self) -> bool {
        false
    }

    pub const fn grants_application_authority(&self) -> bool {
        false
    }
}

fn check_text(
    field: &'static str,
    value: &str,
) -> Result<(), VerifierExecutionEvidenceErrorV1> {
    if value.trim().is_empty() {
        return Err(VerifierExecutionEvidenceErrorV1::EmptyField { field });
    }
    let actual = value.len();
    if actual > MAX_VERIFIER_EXECUTION_IDENTITY_BYTES_V1 {
        return Err(VerifierExecutionEvidenceErrorV1::FieldTooLong {
            field,
            maximum: MAX_VERIFIER_EXECUTION_IDENTITY_BYTES_V1,
            actual,
        });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::QualificationReceiptCanonicalizationV1;

    fn digest(byte: u8) -> Sha256DigestV1 {
        Sha256DigestV1::from_bytes([byte; 32])
    }

    fn verifier() -> VerifierExecutableIdentityV1 {
        VerifierExecutableIdentityV1 {
            profile_id: "github-cli-verified-v1".into(),
            backend_family: "github-cli-attestation".into(),
            semantic_version: "2.101.0".into(),
            executable_sha256: digest(1),
            platform_profile: "linux-x86_64".into(),
            command_profile_id: "github-public-qualification-v1".into(),
            nix_closure: Some(NixVerifierClosureIdentityV1 {
                store_path: "/nix/store/example-gh/bin/gh".into(),
                closure_digest: digest(2),
            }),
        }
    }

    fn process_receipt(mode: VerifierTrustRootModeV1) -> VerifierProcessExecutionReceiptV1 {
        let started = 1_000;
        let acquired = match mode {
            VerifierTrustRootModeV1::OnlineFetchThenRetainedVerifyV1 => 1_010,
            VerifierTrustRootModeV1::OfflinePinnedRootV1 => 900,
        };
        VerifierProcessExecutionReceiptV1 {
            evidence_version: VERIFIER_EXECUTION_EVIDENCE_VERSION_V1,
            verifier: verifier(),
            canonical_receipt_digest: QualificationReceiptDigestV1 {
                canonicalization: QualificationReceiptCanonicalizationV1::BinaryV1,
                sha256: digest(3),
            },
            attestation_bundle_digest: digest(4),
            trust_root_mode: mode,
            trusted_root_material_digest: digest(5),
            trusted_root_acquired_at_unix_seconds: acquired,
            command_arguments_digest: digest(6),
            environment_profile_digest: digest(7),
            verifier_stdout_digest: digest(8),
            verifier_stderr_digest: digest(9),
            execution_started_at_unix_seconds: started,
            execution_completed_at_unix_seconds: 1_020,
            process_outcome: VerifierProcessOutcomeV1::ExitCode(0),
        }
    }

    fn receipt(mode: VerifierTrustRootModeV1) -> VerifierExecutionReceiptV1 {
        process_receipt(mode).into_parsed_receipt(1).unwrap()
    }

    #[test]
    fn preparse_process_receipt_preserves_execution_invariants_without_count_claim() {
        let evidence = process_receipt(VerifierTrustRootModeV1::OnlineFetchThenRetainedVerifyV1);
        assert_eq!(evidence.validate(), Ok(()));
        assert!(evidence.process_exited_successfully());
        assert!(!evidence.establishes_parsed_result_count());
        assert!(!evidence.establishes_receipt_authentication());
    }

    #[test]
    fn parser_owned_promotion_adds_only_the_exact_result_count() {
        let process = process_receipt(VerifierTrustRootModeV1::OnlineFetchThenRetainedVerifyV1);
        let full = process.clone().into_parsed_receipt(2).unwrap();
        assert_eq!(full.parsed_result_count, 2);
        assert_eq!(full.verifier, process.verifier);
        assert_eq!(full.verifier_stdout_digest, process.verifier_stdout_digest);
        assert_eq!(full.execution_completed_at_unix_seconds, process.execution_completed_at_unix_seconds);
        assert_eq!(
            process.into_parsed_receipt(MAX_VERIFIED_RESULTS_V1 + 1),
            Err(VerifierExecutionEvidenceErrorV1::TooManyVerifiedResults {
                maximum: MAX_VERIFIED_RESULTS_V1,
                actual: MAX_VERIFIED_RESULTS_V1 + 1,
            })
        );
    }

    #[test]
    fn online_and_offline_modes_encode_distinct_time_theorems() {
        assert_eq!(
            receipt(VerifierTrustRootModeV1::OnlineFetchThenRetainedVerifyV1).validate(),
            Ok(())
        );
        assert_eq!(
            receipt(VerifierTrustRootModeV1::OfflinePinnedRootV1).validate(),
            Ok(())
        );

        let mut invalid = receipt(VerifierTrustRootModeV1::OnlineFetchThenRetainedVerifyV1);
        invalid.trusted_root_acquired_at_unix_seconds = 999;
        assert_eq!(
            invalid.validate(),
            Err(VerifierExecutionEvidenceErrorV1::TrustRootAcquiredBeforeExecutionWindow)
        );

        let mut invalid = receipt(VerifierTrustRootModeV1::OfflinePinnedRootV1);
        invalid.trusted_root_acquired_at_unix_seconds = 1_001;
        assert_eq!(
            invalid.validate(),
            Err(VerifierExecutionEvidenceErrorV1::TrustRootAcquiredAfterExecution)
        );
    }

    #[test]
    fn process_outcome_does_not_collapse_timeout_or_signal_into_exit_code() {
        let mut evidence = receipt(VerifierTrustRootModeV1::OnlineFetchThenRetainedVerifyV1);
        assert!(evidence.process_exited_successfully());

        evidence.process_outcome = VerifierProcessOutcomeV1::TimedOut;
        assert!(!evidence.process_exited_successfully());

        evidence.process_outcome = VerifierProcessOutcomeV1::TerminatedBySignal(9);
        assert!(!evidence.process_exited_successfully());
    }

    #[test]
    fn execution_evidence_never_upgrades_authority() {
        let evidence = receipt(VerifierTrustRootModeV1::OnlineFetchThenRetainedVerifyV1);
        assert_eq!(
            evidence.authority_scope(),
            VerifierExecutionAuthorityV1::ExecutionEvidenceOnly
        );
        assert!(evidence.process_exited_successfully());
        assert!(!evidence.establishes_receipt_authentication());
        assert!(!evidence.grants_production_authority());
        assert!(!evidence.grants_application_authority());
    }

    #[test]
    fn result_count_and_identity_fail_closed() {
        let mut evidence = receipt(VerifierTrustRootModeV1::OnlineFetchThenRetainedVerifyV1);
        evidence.parsed_result_count = MAX_VERIFIED_RESULTS_V1 + 1;
        assert!(matches!(
            evidence.validate(),
            Err(VerifierExecutionEvidenceErrorV1::TooManyVerifiedResults { .. })
        ));

        let mut evidence = receipt(VerifierTrustRootModeV1::OnlineFetchThenRetainedVerifyV1);
        evidence.verifier.command_profile_id.clear();
        assert!(matches!(
            evidence.validate(),
            Err(VerifierExecutionEvidenceErrorV1::EmptyField {
                field: "verifier.command_profile_id"
            })
        ));

        let mut evidence = receipt(VerifierTrustRootModeV1::OnlineFetchThenRetainedVerifyV1);
        evidence.verifier.nix_closure.as_mut().unwrap().store_path = "/tmp/gh".into();
        assert_eq!(
            evidence.validate(),
            Err(VerifierExecutionEvidenceErrorV1::InvalidNixStorePath)
        );
    }
}
