//! Signed, replay-resistant binding of generation input to evidence exposure.
//!
//! This module reuses the crate's signature-verifier and replay-guard interfaces.
//! It does not implement cryptography, establish the trustworthiness of a witness,
//! prove that a model semantically attended to an item, or replace the evidence ledger.
//! A PASS means that an independently trusted verifier accepted a signed claim whose
//! ordered evidence inventory exactly matches the inventory supplied to the diagnostic.

use std::collections::BTreeSet;

use sha2::{Digest, Sha256};

use crate::{
    diagnose_evidence_use_bottlenecks, Applicability, AvailableEvidenceConstraint,
    EvidenceUseDiagnostic, EvidenceUseDiagnosticStatus, ReceiptSignatureVerifier,
    RelationalEvidenceReport, RelationalEvidenceScope, ReplayGuard, ResponseGrounding,
};

const EXPOSURE_DOMAIN: &[u8] = b"MYCELIX-GENERATION-INPUT-EXPOSURE-V0.1\0";
const EXPOSURE_SCHEMA: &str = "generation-input-exposure-v0.1";

fn nonempty(value: &str) -> bool {
    !value.trim().is_empty()
}

fn valid_digest(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn digest_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    let mut output = String::with_capacity(64);
    for byte in digest {
        use std::fmt::Write;
        let _ = write!(&mut output, "{byte:02x}");
    }
    output
}

fn push_len(out: &mut Vec<u8>, value: usize) {
    out.extend_from_slice(&(value as u64).to_be_bytes());
}

fn push_str(out: &mut Vec<u8>, value: &str) {
    push_len(out, value.len());
    out.extend_from_slice(value.as_bytes());
}

fn status_tag(value: crate::EvidenceStatus) -> u8 {
    match value {
        crate::EvidenceStatus::Current => 1,
        crate::EvidenceStatus::Superseded => 2,
        crate::EvidenceStatus::Disputed => 3,
        crate::EvidenceStatus::Expired => 4,
        crate::EvidenceStatus::Unknown => 5,
    }
}

fn applicability_tag(value: Applicability) -> u8 {
    match value {
        Applicability::Applicable => 1,
        Applicability::NotApplicable => 2,
        Applicability::Conditional => 3,
        Applicability::Unknown => 4,
    }
}

/// One evidence item as it appeared in the actual generation input inventory.
///
/// The order of these entries is significant: the signed receipt binds the
/// ordered inventory, not merely an unordered set of identifiers.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct GenerationEvidenceEntry {
    pub request_id: String,
    pub evidence_id: String,
    pub source_digest: String,
    pub subject_id: String,
    pub scope_id: String,
    pub authorized_responder_id: String,
    pub status: crate::EvidenceStatus,
    pub applicability: Applicability,
    pub verified: bool,
    pub required: bool,
}

impl From<&AvailableEvidenceConstraint> for GenerationEvidenceEntry {
    fn from(item: &AvailableEvidenceConstraint) -> Self {
        Self {
            request_id: item.request_id.clone(),
            evidence_id: item.evidence_id.clone(),
            source_digest: item.source_digest.clone(),
            subject_id: item.subject_id.clone(),
            scope_id: item.scope_id.clone(),
            authorized_responder_id: item.authorized_responder_id.clone(),
            status: item.status,
            applicability: item.applicability,
            verified: item.verified,
            required: item.required,
        }
    }
}

/// Claims captured by the generation-input exposure witness.
///
/// input_payload_digest is the SHA-256 digest of the exact serialized input
/// bytes observed at the model/runtime boundary. The ordered exposed_evidence
/// inventory ties evidence identifiers and source digests to that input receipt.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct GenerationInputExposureClaims {
    pub schema_version: String,
    pub run_id: String,
    pub generation_id: String,
    pub request_id: String,
    pub responder_id: String,
    pub purpose_id: String,
    pub challenge_nonce: String,
    pub signer_key_id: String,
    pub model_snapshot_digest: String,
    pub prompt_template_digest: String,
    pub input_payload_digest: String,
    pub generated_at_ms: u64,
    pub exposed_evidence: Vec<GenerationEvidenceEntry>,
}

impl GenerationInputExposureClaims {
    /// Deterministic, domain-separated bytes that an external witness signs.
    ///
    /// This is a versioned crate-specific encoding, not canonical JSON and not
    /// a general-purpose wire format. A production witness must implement this
    /// exact version or negotiate a separately versioned encoding.
    pub fn signing_bytes(&self) -> Vec<u8> {
        let mut out = Vec::new();
        out.extend_from_slice(EXPOSURE_DOMAIN);
        for value in [
            self.schema_version.as_str(),
            self.run_id.as_str(),
            self.generation_id.as_str(),
            self.request_id.as_str(),
            self.responder_id.as_str(),
            self.purpose_id.as_str(),
            self.challenge_nonce.as_str(),
            self.signer_key_id.as_str(),
            self.model_snapshot_digest.as_str(),
            self.prompt_template_digest.as_str(),
            self.input_payload_digest.as_str(),
        ] {
            push_str(&mut out, value);
        }
        out.extend_from_slice(&self.generated_at_ms.to_be_bytes());
        push_len(&mut out, self.exposed_evidence.len());
        for item in &self.exposed_evidence {
            push_str(&mut out, &item.request_id);
            push_str(&mut out, &item.evidence_id);
            push_str(&mut out, &item.source_digest);
            push_str(&mut out, &item.subject_id);
            push_str(&mut out, &item.scope_id);
            push_str(&mut out, &item.authorized_responder_id);
            out.push(status_tag(item.status));
            out.push(applicability_tag(item.applicability));
            out.push(u8::from(item.verified));
            out.push(u8::from(item.required));
        }
        out
    }

    /// Digest of the exact claims encoding; this is not a signature.
    pub fn receipt_digest(&self) -> String {
        digest_hex(&self.signing_bytes())
    }
}

/// Independently preregistered identities and admissible freshness window.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ExpectedGenerationExposure {
    pub run_id: String,
    pub generation_id: String,
    pub request_id: String,
    pub responder_id: String,
    pub purpose_id: String,
    pub challenge_nonce: String,
    pub trusted_signer_key_id: String,
    pub model_snapshot_digest: String,
    pub prompt_template_digest: String,
    pub earliest_generated_at_ms: u64,
    pub latest_generated_at_ms: u64,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExposureQualificationStatus {
    Pass,
    Fail,
    Indeterminate,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ExposureQualificationReport {
    pub status: ExposureQualificationStatus,
    pub receipt_digest: Option<String>,
    pub exposed_evidence_count: usize,
    pub reasons: Vec<String>,
}

fn exposure_report(
    status: ExposureQualificationStatus,
    receipt_digest: Option<String>,
    exposed_evidence_count: usize,
    reasons: Vec<String>,
) -> ExposureQualificationReport {
    ExposureQualificationReport {
        status,
        receipt_digest,
        exposed_evidence_count,
        reasons,
    }
}

/// Validates identity, scope, freshness, evidence inventory, signature and nonce.
///
/// The signature verifier and replay guard are supplied by an external trust
/// adapter. A verifier outage or replay-store outage is INDETERMINATE, never PASS.
pub fn qualify_generation_input_exposure<V: ReceiptSignatureVerifier, R: ReplayGuard>(
    claims: &GenerationInputExposureClaims,
    signature: &[u8],
    expected: &ExpectedGenerationExposure,
    verifier: &V,
    replay: &mut R,
) -> ExposureQualificationReport {
    let mut errors = Vec::new();

    for (label, value) in [
        ("expected run id", expected.run_id.as_str()),
        ("expected generation id", expected.generation_id.as_str()),
        ("expected request id", expected.request_id.as_str()),
        ("expected responder id", expected.responder_id.as_str()),
        ("expected purpose id", expected.purpose_id.as_str()),
        ("expected signer key id", expected.trusted_signer_key_id.as_str()),
    ] {
        if !nonempty(value) {
            errors.push(format!("{label} must be non-empty"));
        }
    }
    if expected.challenge_nonce.len() < 32 {
        errors.push("expected challenge nonce must contain at least 32 characters".into());
    }
    if expected.latest_generated_at_ms < expected.earliest_generated_at_ms {
        errors.push("expected generation freshness window is inverted".into());
    }
    for (label, value) in [
        ("expected model snapshot", expected.model_snapshot_digest.as_str()),
        ("expected prompt template", expected.prompt_template_digest.as_str()),
    ] {
        if !valid_digest(value) {
            errors.push(format!("{label} digest is malformed"));
        }
    }

    if claims.schema_version != EXPOSURE_SCHEMA {
        errors.push("unsupported generation exposure schema version".into());
    }
    for (label, actual, expected_value) in [
        ("run id", claims.run_id.as_str(), expected.run_id.as_str()),
        (
            "generation id",
            claims.generation_id.as_str(),
            expected.generation_id.as_str(),
        ),
        ("request id", claims.request_id.as_str(), expected.request_id.as_str()),
        (
            "responder id",
            claims.responder_id.as_str(),
            expected.responder_id.as_str(),
        ),
        ("purpose id", claims.purpose_id.as_str(), expected.purpose_id.as_str()),
        (
            "challenge nonce",
            claims.challenge_nonce.as_str(),
            expected.challenge_nonce.as_str(),
        ),
        (
            "signer key id",
            claims.signer_key_id.as_str(),
            expected.trusted_signer_key_id.as_str(),
        ),
        (
            "model snapshot digest",
            claims.model_snapshot_digest.as_str(),
            expected.model_snapshot_digest.as_str(),
        ),
        (
            "prompt template digest",
            claims.prompt_template_digest.as_str(),
            expected.prompt_template_digest.as_str(),
        ),
    ] {
        if actual != expected_value {
            errors.push(format!("{label} differs from the preregistered generation"));
        }
    }
    if !valid_digest(&claims.model_snapshot_digest)
        || !valid_digest(&claims.prompt_template_digest)
        || !valid_digest(&claims.input_payload_digest)
    {
        errors.push("model, prompt or exact input payload digest is malformed".into());
    }
    if claims.generated_at_ms < expected.earliest_generated_at_ms
        || claims.generated_at_ms > expected.latest_generated_at_ms
    {
        errors.push("generation receipt timestamp falls outside the preregistered window".into());
    }
    if claims.exposed_evidence.is_empty() {
        errors.push("generation exposure must bind at least one evidence item".into());
    }

    let mut evidence_ids = BTreeSet::new();
    for item in &claims.exposed_evidence {
        if !nonempty(&item.request_id)
            || item.request_id != claims.request_id
            || !nonempty(&item.evidence_id)
            || !evidence_ids.insert(item.evidence_id.as_str())
            || !valid_digest(&item.source_digest)
            || !nonempty(&item.subject_id)
            || !nonempty(&item.scope_id)
            || item.authorized_responder_id != claims.responder_id
            || !item.verified
            || !item.required
        {
            errors.push(format!(
                "exposed evidence {} has invalid identity, scope, digest or required/verified state",
                item.evidence_id
            ));
        }
    }

    if !errors.is_empty() {
        return exposure_report(
            ExposureQualificationStatus::Fail,
            None,
            claims.exposed_evidence.len(),
            errors,
        );
    }
    if signature.is_empty() {
        return exposure_report(
            ExposureQualificationStatus::Fail,
            None,
            claims.exposed_evidence.len(),
            vec!["generation exposure signature is empty".into()],
        );
    }

    let signing_bytes = claims.signing_bytes();
    match verifier.verify(&claims.signer_key_id, &signing_bytes, signature) {
        Ok(true) => {}
        Ok(false) => {
            return exposure_report(
                ExposureQualificationStatus::Fail,
                None,
                claims.exposed_evidence.len(),
                vec!["generation exposure signature is invalid".into()],
            );
        }
        Err(error) => {
            return exposure_report(
                ExposureQualificationStatus::Indeterminate,
                None,
                claims.exposed_evidence.len(),
                vec![format!("generation exposure signature verifier unavailable: {error}")],
            );
        }
    }

    match replay.consume_once(
        &claims.run_id,
        "generation-input-exposure",
        &claims.challenge_nonce,
    ) {
        Ok(true) => {}
        Ok(false) => {
            return exposure_report(
                ExposureQualificationStatus::Fail,
                None,
                claims.exposed_evidence.len(),
                vec!["generation exposure challenge nonce was already consumed".into()],
            );
        }
        Err(error) => {
            return exposure_report(
                ExposureQualificationStatus::Indeterminate,
                None,
                claims.exposed_evidence.len(),
                vec![format!("generation exposure replay guard unavailable: {error}")],
            );
        }
    }

    exposure_report(
        ExposureQualificationStatus::Pass,
        Some(digest_hex(&signing_bytes)),
        claims.exposed_evidence.len(),
        vec![
            "signed claims bind the exact input payload digest and ordered evidence inventory".into(),
            "signature and consume-once nonce checks passed through the supplied external trust adapters".into(),
            "this establishes receipt integrity, not semantic attention or factual correctness".into(),
        ],
    )
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BoundEvidenceUseStatus {
    Complete,
    ExposureNotQualified,
    GeneratorInventoryMismatch,
    DownstreamNotEvaluable,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BoundEvidenceUseDiagnostic {
    pub status: BoundEvidenceUseStatus,
    pub exposure: ExposureQualificationReport,
    pub diagnostic: Option<EvidenceUseDiagnostic>,
    pub reasons: Vec<String>,
}

/// Verifies a signed generation exposure receipt, compares its ordered evidence
/// inventory with the exact inventory supplied to the response-use diagnostic,
/// and only then runs the existing RER/ACB bottleneck classifier.
///
/// The expected context must be pinned independently of the candidate's own
/// receipt. This function does not authenticate the underlying candidate pool,
/// source registry, relation index or natural-language auditor.
#[allow(clippy::too_many_arguments)]
pub fn diagnose_evidence_use_with_verified_exposure<
    V: ReceiptSignatureVerifier,
    R: ReplayGuard,
>(
    scope: &RelationalEvidenceScope,
    refinement: &RelationalEvidenceReport,
    generator_visible: &[AvailableEvidenceConstraint],
    response: &ResponseGrounding,
    claims: &GenerationInputExposureClaims,
    signature: &[u8],
    expected: &ExpectedGenerationExposure,
    verifier: &V,
    replay: &mut R,
) -> BoundEvidenceUseDiagnostic {
    let not_qualified = |reason: String, exposure: ExposureQualificationReport| {
        BoundEvidenceUseDiagnostic {
            status: BoundEvidenceUseStatus::ExposureNotQualified,
            exposure,
            diagnostic: None,
            reasons: vec![reason],
        }
    };

    if expected.request_id != scope.request_id
        || expected.responder_id != scope.recipient_id
        || expected.purpose_id != scope.purpose_id
        || response.request_id != expected.request_id
        || response.responder_id != expected.responder_id
        || response.response_id != expected.generation_id
    {
        return not_qualified(
            "scope or response identity differs from the preregistered generation context".into(),
            exposure_report(
                ExposureQualificationStatus::Fail,
                None,
                claims.exposed_evidence.len(),
                vec!["generation context mismatch".into()],
            ),
        );
    }

    let exposure =
        qualify_generation_input_exposure(claims, signature, expected, verifier, replay);
    if exposure.status != ExposureQualificationStatus::Pass {
        return not_qualified(
            "generation input exposure receipt did not qualify".into(),
            exposure,
        );
    }

    let actual_inventory: Vec<GenerationEvidenceEntry> =
        generator_visible.iter().map(GenerationEvidenceEntry::from).collect();
    if actual_inventory != claims.exposed_evidence {
        return BoundEvidenceUseDiagnostic {
            status: BoundEvidenceUseStatus::GeneratorInventoryMismatch,
            exposure,
            diagnostic: None,
            reasons: vec![
                "signed exposure inventory differs from the exact evidence inventory supplied to the response-use diagnostic".into(),
                "no retrieval, handoff or constraint-application classification is emitted for a mismatched inventory".into(),
            ],
        };
    }

    let Some(receipt_digest) = exposure.receipt_digest.as_deref() else {
        return not_qualified(
            "qualified exposure report omitted its receipt digest".into(),
            exposure,
        );
    };
    let diagnostic = diagnose_evidence_use_bottlenecks(
        scope,
        refinement,
        generator_visible,
        response,
        receipt_digest,
    );
    let status = if diagnostic.status == EvidenceUseDiagnosticStatus::Complete {
        BoundEvidenceUseStatus::Complete
    } else {
        BoundEvidenceUseStatus::DownstreamNotEvaluable
    };
    BoundEvidenceUseDiagnostic {
        status,
        exposure,
        diagnostic: Some(diagnostic),
        reasons: vec![
            "the bottleneck classifier was run only after signature, replay and exact-inventory checks".into(),
            "complete means structurally classifiable, not that the response is correct".into(),
        ],
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        refine_relational_evidence, ConstraintHandling, EvidenceStatus,
        RelationalDisclosurePermission, RelationalEvidenceRecord, RelationalEvidenceStatus,
        RelationalVerification, ResponseConstraintUse,
    };
    use std::collections::BTreeSet;

    struct EchoVerifier;

    impl ReceiptSignatureVerifier for EchoVerifier {
        fn verify(&self, _key_id: &str, message: &[u8], signature: &[u8]) -> Result<bool, String> {
            Ok(message == signature)
        }
    }

    #[derive(Default)]
    struct MemoryReplay {
        consumed: BTreeSet<(String, String, String)>,
    }

    impl ReplayGuard for MemoryReplay {
        fn consume_once(&mut self, run: &str, phase: &str, nonce: &str) -> Result<bool, String> {
            Ok(self.consumed.insert((
                run.to_owned(),
                phase.to_owned(),
                nonce.to_owned(),
            )))
        }
    }

    fn digest(ch: char) -> String {
        std::iter::repeat_n(ch, 64).collect()
    }

    fn scope() -> RelationalEvidenceScope {
        RelationalEvidenceScope {
            request_id: "request-1".into(),
            recipient_id: "assistant-1".into(),
            purpose_id: "trip-planning".into(),
            history_cutoff_ms: 200,
            seed_evidence_ids: vec!["e1".into()],
            max_relation_hops: 2,
            max_selected_evidence: 8,
            candidate_pool_complete: true,
            candidate_pool_receipt_digest: Some(digest('d')),
            relation_index_complete: true,
            relation_index_receipt_digest: Some(digest('e')),
        }
    }

    fn candidate() -> RelationalEvidenceRecord {
        RelationalEvidenceRecord {
            evidence_id: "e1".into(),
            request_id: "request-1".into(),
            source_origin_id: "origin-1".into(),
            source_holder_id: "holder-1".into(),
            source_message_id: "message-1".into(),
            response_subject_id: "travel-date".into(),
            response_scope_id: "trip-2026".into(),
            payload_digest: digest('a'),
            source_registry_receipt_digest: digest('b'),
            disclosure_receipt_digest: digest('c'),
            source_timestamp_ms: 100,
            recorded_at_ms: 110,
            subject_ids: vec!["travel-date".into()],
            event_ids: vec!["trip-2026".into()],
            decision_ids: vec!["booking-1".into()],
            permission: RelationalDisclosurePermission::Scoped,
            permitted_recipient_ids: vec!["assistant-1".into()],
            permitted_purpose_ids: vec!["trip-planning".into()],
            verification: RelationalVerification::Verified,
            status: EvidenceStatus::Current,
            applicability: Applicability::Applicable,
        }
    }

    fn fixture() -> (
        RelationalEvidenceScope,
        RelationalEvidenceReport,
        Vec<AvailableEvidenceConstraint>,
        ResponseGrounding,
        GenerationInputExposureClaims,
        ExpectedGenerationExposure,
    ) {
        let scope = scope();
        let candidate = candidate();
        let refinement = refine_relational_evidence(&scope, std::slice::from_ref(&candidate), &[]);
        assert_eq!(refinement.status, RelationalEvidenceStatus::Pass);

        let visible = crate::bind_refined_evidence_constraints(
            &scope,
            std::slice::from_ref(&candidate),
            &refinement,
        )
        .unwrap();
        let response = ResponseGrounding {
            request_id: scope.request_id.clone(),
            response_id: "generation-1".into(),
            responder_id: scope.recipient_id.clone(),
            constraints: visible
                .iter()
                .map(|item| ResponseConstraintUse {
                    evidence_id: item.evidence_id.clone(),
                    source_digest: item.source_digest.clone(),
                    subject_id: item.subject_id.clone(),
                    scope_id: item.scope_id.clone(),
                    status: item.status,
                    applicability: item.applicability,
                    handling: ConstraintHandling::Applied,
                    rationale: None,
                })
                .collect(),
        };
        let claims = GenerationInputExposureClaims {
            schema_version: EXPOSURE_SCHEMA.into(),
            run_id: "run-1".into(),
            generation_id: "generation-1".into(),
            request_id: scope.request_id.clone(),
            responder_id: scope.recipient_id.clone(),
            purpose_id: scope.purpose_id.clone(),
            challenge_nonce: "n".repeat(32),
            signer_key_id: "external-witness-key-1".into(),
            model_snapshot_digest: digest('1'),
            prompt_template_digest: digest('2'),
            input_payload_digest: digest('3'),
            generated_at_ms: 150,
            exposed_evidence: visible.iter().map(GenerationEvidenceEntry::from).collect(),
        };
        let expected = ExpectedGenerationExposure {
            run_id: "run-1".into(),
            generation_id: "generation-1".into(),
            request_id: scope.request_id.clone(),
            responder_id: scope.recipient_id.clone(),
            purpose_id: scope.purpose_id.clone(),
            challenge_nonce: "n".repeat(32),
            trusted_signer_key_id: "external-witness-key-1".into(),
            model_snapshot_digest: digest('1'),
            prompt_template_digest: digest('2'),
            earliest_generated_at_ms: 100,
            latest_generated_at_ms: 200,
        };
        (scope, refinement, visible, response, claims, expected)
    }

    #[test]
    fn signed_exact_inventory_and_generation_input_qualify() {
        let (scope, refinement, visible, response, claims, expected) = fixture();
        let signature = claims.signing_bytes();
        let report = diagnose_evidence_use_with_verified_exposure(
            &scope,
            &refinement,
            &visible,
            &response,
            &claims,
            &signature,
            &expected,
            &EchoVerifier,
            &mut MemoryReplay::default(),
        );
        assert_eq!(report.status, BoundEvidenceUseStatus::Complete);
        assert_eq!(report.exposure.status, ExposureQualificationStatus::Pass);
        assert_eq!(
            report.diagnostic.as_ref().unwrap().status,
            EvidenceUseDiagnosticStatus::Complete
        );
    }

    #[test]
    fn tampered_input_payload_digest_invalidates_signed_receipt() {
        let (_, _, _, _, mut claims, expected) = fixture();
        let signature = claims.signing_bytes();
        claims.input_payload_digest = digest('9');
        let report = qualify_generation_input_exposure(
            &claims,
            &signature,
            &expected,
            &EchoVerifier,
            &mut MemoryReplay::default(),
        );
        assert_eq!(report.status, ExposureQualificationStatus::Fail);
        assert!(report.receipt_digest.is_none());
    }

    #[test]
    fn visible_inventory_substitution_is_not_classified() {
        let (scope, refinement, mut visible, response, claims, expected) = fixture();
        visible[0].source_digest = digest('9');
        let signature = claims.signing_bytes();
        let report = diagnose_evidence_use_with_verified_exposure(
            &scope,
            &refinement,
            &visible,
            &response,
            &claims,
            &signature,
            &expected,
            &EchoVerifier,
            &mut MemoryReplay::default(),
        );
        assert_eq!(report.status, BoundEvidenceUseStatus::GeneratorInventoryMismatch);
        assert!(report.diagnostic.is_none());
    }

    #[test]
    fn wrong_generation_or_response_identity_fails_closed() {
        let (scope, refinement, visible, mut response, claims, expected) = fixture();
        response.response_id = "other-generation".into();
        let signature = claims.signing_bytes();
        let report = diagnose_evidence_use_with_verified_exposure(
            &scope,
            &refinement,
            &visible,
            &response,
            &claims,
            &signature,
            &expected,
            &EchoVerifier,
            &mut MemoryReplay::default(),
        );
        assert_eq!(report.status, BoundEvidenceUseStatus::ExposureNotQualified);
    }

    #[test]
    fn replayed_nonce_fails_closed() {
        let (_, _, _, _, claims, expected) = fixture();
        let signature = claims.signing_bytes();
        let mut replay = MemoryReplay::default();
        let first = qualify_generation_input_exposure(
            &claims,
            &signature,
            &expected,
            &EchoVerifier,
            &mut replay,
        );
        let second = qualify_generation_input_exposure(
            &claims,
            &signature,
            &expected,
            &EchoVerifier,
            &mut replay,
        );
        assert_eq!(first.status, ExposureQualificationStatus::Pass);
        assert_eq!(second.status, ExposureQualificationStatus::Fail);
    }

    #[test]
    fn out_of_window_exposure_is_rejected() {
        let (_, _, _, _, claims, mut expected) = fixture();
        expected.latest_generated_at_ms = 149;
        let signature = claims.signing_bytes();
        let report = qualify_generation_input_exposure(
            &claims,
            &signature,
            &expected,
            &EchoVerifier,
            &mut MemoryReplay::default(),
        );
        assert_eq!(report.status, ExposureQualificationStatus::Fail);
    }

    #[test]
    fn duplicate_exposed_evidence_ids_are_rejected() {
        let (_, _, _, _, mut claims, expected) = fixture();
        claims.exposed_evidence.push(claims.exposed_evidence[0].clone());
        let signature = claims.signing_bytes();
        let report = qualify_generation_input_exposure(
            &claims,
            &signature,
            &expected,
            &EchoVerifier,
            &mut MemoryReplay::default(),
        );
        assert_eq!(report.status, ExposureQualificationStatus::Fail);
    }

    #[test]
    fn signature_verifier_outage_is_indeterminate() {
        struct Unavailable;
        impl ReceiptSignatureVerifier for Unavailable {
            fn verify(&self, _: &str, _: &[u8], _: &[u8]) -> Result<bool, String> {
                Err("witness service offline".into())
            }
        }
        let (_, _, _, _, claims, expected) = fixture();
        let signature = claims.signing_bytes();
        let report = qualify_generation_input_exposure(
            &claims,
            &signature,
            &expected,
            &Unavailable,
            &mut MemoryReplay::default(),
        );
        assert_eq!(report.status, ExposureQualificationStatus::Indeterminate);
    }
}
