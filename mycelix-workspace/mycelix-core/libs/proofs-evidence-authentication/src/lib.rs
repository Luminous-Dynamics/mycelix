// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Canonical qualification receipts and the receipt-authentication capability boundary.
//!
//! This crate intentionally contains no cryptographic verifier backend yet. It freezes
//! the canonical receipt identity, exact authentication policy, opaque capability,
//! strict untrusted-wire parsing boundary, non-authoritative verifier-execution
//! evidence, deterministic GitHub verifier command/I/O planning, and strict parsed
//! verifier-output evidence that future separately-qualified backends may use after
//! full verification.

mod canonical;
mod capability;
mod github_cli_execution_policy;
mod github_cli_plan;
mod github_cli_steps;
mod github_verification_result;
mod policy;
mod predicate;
mod verifier_execution;
mod wire;

pub use canonical::{
    GitObjectIdParseErrorV1, GitObjectIdV1, MAX_RECEIPT_IDENTIFIER_BYTES_V1,
    MAX_RECEIPT_NONCLAIMS_V1, MAX_RECEIPT_NONCLAIM_BYTES_V1,
    QUALIFICATION_RECEIPT_CANONICALIZATION_PROFILE_V1,
    QualificationReceiptCanonicalizationV1, QualificationReceiptDigestV1,
    QualificationReceiptV1, QualificationResultV1, ReceiptCanonicalizationErrorV1,
    Sha256DigestParseErrorV1, Sha256DigestV1,
};
pub use capability::{
    AuthenticatedCapabilityConstructionErrorV1, AuthenticatedQualificationReceiptV1,
    AuthenticatedReceiptAuthorityV1, AuthenticationEvidenceSummaryV1,
    VerifiedAuthenticationContextV1, VerifiedFreshnessV1, VerifiedQualificationPredicateV1,
    VerifiedSignerIdentityV1, VerifiedTransparencyV1,
};
pub use github_cli_execution_policy::{
    GitHubPublicVerifierExecutionPolicyV1, GitHubVerifierExecutionPolicyAuthorityV1,
    GitHubVerifierExecutionPolicyErrorV1, MAX_GITHUB_EXECUTION_POLICY_TEXT_BYTES_V1,
    build_github_public_command_plan_with_execution_policy_v1,
};
pub use github_cli_plan::{
    FixedEnvironmentVariableV1, GITHUB_ACTIONS_OIDC_ISSUER_V1,
    GITHUB_PUBLIC_BACKEND_FAMILY_V1, GITHUB_PUBLIC_COMMAND_PROFILE_ID_V1,
    GITHUB_PUBLIC_HOSTNAME_V1, GITHUB_PUBLIC_MAX_ATTESTATIONS_V1,
    GITHUB_PUBLIC_MAX_STDERR_BYTES_V1, GITHUB_PUBLIC_MAX_STDOUT_BYTES_V1,
    GITHUB_PUBLIC_PLATFORM_PROFILE_V1, GITHUB_PUBLIC_PROCESS_TIMEOUT_SECONDS_V1,
    GitHubCommandPlanAuthorityV1, GitHubCommandPlanErrorV1, GitHubPublicCommandPlanV1,
    MYCELIX_QUALIFICATION_PREDICATE_TYPE_V1, MYCELIX_QUALIFICATION_SUBJECT_NAME_V1,
    MYCELIX_REPOSITORY_OWNER_V1, MYCELIX_REPOSITORY_V1,
};
pub use github_cli_steps::{
    GitHubExecutionStepAuthorityV1, GitHubVerifierExecutionStepV1,
    GitHubVerifierStdoutDispositionV1, GitHubVerifierStepPurposeV1,
    github_public_verifier_execution_steps_v1,
};
pub use github_verification_result::{
    GITHUB_VERIFICATION_RESULT_PARSER_PROFILE_V1, MAX_GITHUB_CERTIFICATE_JSON_BYTES_V1,
    MAX_GITHUB_PREDICATE_JSON_BYTES_V1, MAX_GITHUB_RESULT_TEXT_BYTES_V1,
    MAX_GITHUB_STATEMENT_SUBJECTS_V1, MAX_GITHUB_VERIFICATION_RESULTS_V1,
    MAX_GITHUB_VERIFICATION_STDOUT_BYTES_V1, MAX_GITHUB_VERIFIED_TIMESTAMPS_V1,
    SIGSTORE_VERIFICATION_RESULT_MEDIA_TYPE_V1, VERIFIER_STDOUT_DIGEST_PROFILE_V1,
    GitHubAttestationFactProvenanceV1, GitHubCertificateIdentityFactsV1,
    GitHubParsedVerificationCandidateV1, GitHubParsedVerifierOutputAuthorityV1,
    GitHubSignedPredicateClaimV1, GitHubSignedStatementSubjectV1,
    GitHubTrustInstanceStatusV1, GitHubVerificationResultParseErrorV1,
    GitHubVerifiedTimestampEvidenceV1, GitHubVerifiedTimestampWitnessKindV1,
    ParsedGitHubAttestationVerifierOutputV1, github_verifier_stdout_digest_v1,
    parse_github_attestation_verifier_output_v1,
};
pub use policy::{
    AuthenticationFreshnessPolicyV1, AuthenticationPolicyErrorV1,
    ReceiptAuthenticationPolicyV1, SourceRevisionPolicyV1, TransparencyPolicyV1,
    VerifierProfileV1, WorkflowRevisionPolicyV1,
};
pub use predicate::{
    QUALIFICATION_ATTESTATION_PREDICATE_SCHEMA_V1,
    QualificationAttestationPredicateErrorV1, QualificationAttestationPredicateV1,
};
pub use verifier_execution::{
    MAX_VERIFIED_RESULTS_V1, MAX_VERIFIER_EXECUTION_IDENTITY_BYTES_V1,
    VERIFIER_EXECUTION_EVIDENCE_VERSION_V1, NixVerifierClosureIdentityV1,
    VerifierExecutableIdentityV1, VerifierExecutionAuthorityV1,
    VerifierExecutionEvidenceErrorV1, VerifierExecutionReceiptV1,
    VerifierProcessOutcomeV1, VerifierTrustRootModeV1,
};
pub use wire::{
    MAX_UNTRUSTED_WIRE_JSON_BYTES_V1, MAX_UNTRUSTED_WIRE_STRING_BYTES_V1,
    MAX_UNTRUSTED_WORKFLOW_REVISIONS_V1, UntrustedWireParseErrorV1,
    parse_untrusted_authentication_policy_json_v1,
    parse_untrusted_qualification_predicate_json_v1,
};
