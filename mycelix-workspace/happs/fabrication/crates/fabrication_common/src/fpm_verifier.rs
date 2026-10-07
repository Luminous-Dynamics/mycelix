//! Authority-approved expected verifier implementation identity for FPM.
//!
//! This is an expected-artifact/build-provenance binding, not proof that a
//! particular binary actually executed. Runtime/coordinator attestation remains
//! a separate boundary.

use hdi::prelude::{ActionHash, AgentPubKey};
use serde::{Deserialize, Serialize};

pub const FPM_VERIFIER_IMPLEMENTATION_TRUST_ANCHOR_SCHEMA_VERSION: &str =
    "fpm.attestation.verifier-implementation-trust-anchor.v1";
pub const FPM_VERIFIER_BUILDER_ID_MAX_BYTES: usize = 512;

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct FpmVerifierImplementationTrustAnchor {
    pub schema_version: String,
    pub verifier_agent: AgentPubKey,
    pub verification_key_trust_anchor_action: ActionHash,
    /// Exact SHA-256 commitment to the deployable verifier artifact expected
    /// by this trust root. This does not prove that artifact executed.
    pub implementation_digest: String,
    /// SHA-256 commitment to the build-provenance statement associated with
    /// the expected verifier artifact. The statement itself is not interpreted
    /// or verified by this layer.
    pub build_provenance_digest: String,
    /// Identity of the builder asserted by the associated provenance statement.
    /// It is an approved expectation, not runtime evidence.
    pub builder_id: String,
    /// Carried from the trusted verification-key anchor so the implementation
    /// identity and cryptographic key cannot silently diverge.
    pub verifier_profile_digest: String,
}

pub fn is_valid_fpm_verifier_builder_id(value: &str) -> bool {
    !value.trim().is_empty()
        && value == value.trim()
        && value.len() <= FPM_VERIFIER_BUILDER_ID_MAX_BYTES
        && !value.chars().any(char::is_control)
}

pub fn is_canonical_fpm_verifier_digest(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| matches!(byte, b'0'..=b'9' | b'a'..=b'f'))
}

pub fn validate_fpm_verifier_implementation_identity_fields(
    anchor: &FpmVerifierImplementationTrustAnchor,
) -> bool {
    anchor.schema_version == FPM_VERIFIER_IMPLEMENTATION_TRUST_ANCHOR_SCHEMA_VERSION
        && is_canonical_fpm_verifier_digest(&anchor.implementation_digest)
        && is_canonical_fpm_verifier_digest(&anchor.build_provenance_digest)
        && is_valid_fpm_verifier_builder_id(&anchor.builder_id)
        && is_canonical_fpm_verifier_digest(&anchor.verifier_profile_digest)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(byte: u8) -> String {
        format!("{:064x}", byte)
    }

    fn valid_anchor() -> FpmVerifierImplementationTrustAnchor {
        FpmVerifierImplementationTrustAnchor {
            schema_version: FPM_VERIFIER_IMPLEMENTATION_TRUST_ANCHOR_SCHEMA_VERSION.into(),
            verifier_agent: AgentPubKey::from_raw_36(vec![1; 36]),
            verification_key_trust_anchor_action: ActionHash::from_raw_36(vec![2; 36]),
            implementation_digest: digest(3),
            build_provenance_digest: digest(4),
            builder_id: "https://builder.example.invalid/fpm".into(),
            verifier_profile_digest: digest(5),
        }
    }

    #[test]
    fn exact_identity_fields_are_accepted() {
        assert!(validate_fpm_verifier_implementation_identity_fields(&valid_anchor()));
    }

    #[test]
    fn malformed_or_mixed_case_digests_are_rejected() {
        let mut anchor = valid_anchor();
        anchor.implementation_digest = "not-a-digest".into();
        assert!(!validate_fpm_verifier_implementation_identity_fields(&anchor));

        anchor = valid_anchor();
        anchor.build_provenance_digest = "abcdef".repeat(10) + "abcd";
        anchor.build_provenance_digest.make_ascii_uppercase();
        assert!(!validate_fpm_verifier_implementation_identity_fields(&anchor));
    }

    #[test]
    fn builder_identifier_is_closed_world() {
        let mut anchor = valid_anchor();
        anchor.builder_id = "  builder ".into();
        assert!(!validate_fpm_verifier_implementation_identity_fields(&anchor));

        anchor = valid_anchor();
        anchor.builder_id = "builder\u{0000}".into();
        assert!(!validate_fpm_verifier_implementation_identity_fields(&anchor));

        anchor = valid_anchor();
        anchor.builder_id = "a".repeat(FPM_VERIFIER_BUILDER_ID_MAX_BYTES + 1);
        assert!(!validate_fpm_verifier_implementation_identity_fields(&anchor));
    }

    #[test]
    fn implementation_identity_is_distinct_from_runtime_execution() {
        let anchor = valid_anchor();
        assert!(
            anchor.implementation_digest != anchor.build_provenance_digest,
            "artifact and provenance commitments are distinct evidence domains"
        );
    }
}
