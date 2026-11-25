//! Independent external-witness receipt qualification.
//!
//! This module validates a separately signed statement about whether unique
//! per-run canary tokens reached witness-controlled endpoints. It does not
//! create network isolation or prove that the witness itself is uncompromised.

use std::collections::{BTreeMap, BTreeSet};

use super::{
    is_sha256, push_len, push_str, AiOffClaims, ExpectedAiOffPhase, QualificationReport,
    QualificationStatus, ReceiptSignatureVerifier, ReplayGuard,
};

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum WitnessChannel {
    InferenceEndpoint,
    AgentControlPlane,
    Ipv4Egress,
    Ipv6Egress,
    DnsEgress,
    ProxyEgress,
    UdpEgress,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum WitnessOutcome {
    NoArrival,
    Arrived,
    Unknown,
    NotObserved,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RequiredWitnessProbe {
    pub channel: WitnessChannel,
    pub challenge_token_digest: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct WitnessProbeObservation {
    pub channel: WitnessChannel,
    pub challenge_token_digest: String,
    pub outcome: WitnessOutcome,
    pub window_start_ms: u64,
    pub window_end_ms: u64,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ExternalWitnessReceipt {
    pub schema_version: String,
    pub run_id: String,
    pub phase_id: String,
    pub challenge_nonce: String,
    pub witness_id: String,
    pub witness_key_id: String,
    pub endpoint_set_digest: String,
    pub observation_start_ms: u64,
    pub observation_end_ms: u64,
    pub event_sequence_start: u64,
    pub event_sequence_end: u64,
    pub probes: Vec<WitnessProbeObservation>,
}

impl ExternalWitnessReceipt {
    /// Crate-local deterministic encoding for the reference validator only.
    /// Production must use a separately versioned standard canonical encoding.
    pub fn signing_bytes(&self) -> Vec<u8> {
        let mut out = Vec::new();
        for value in [
            self.schema_version.as_str(),
            self.run_id.as_str(),
            self.phase_id.as_str(),
            self.challenge_nonce.as_str(),
            self.witness_id.as_str(),
            self.witness_key_id.as_str(),
            self.endpoint_set_digest.as_str(),
        ] {
            push_str(&mut out, value);
        }
        out.extend_from_slice(&self.observation_start_ms.to_be_bytes());
        out.extend_from_slice(&self.observation_end_ms.to_be_bytes());
        out.extend_from_slice(&self.event_sequence_start.to_be_bytes());
        out.extend_from_slice(&self.event_sequence_end.to_be_bytes());

        let mut probes = self.probes.clone();
        probes.sort_by_key(|probe| probe.channel);
        push_len(&mut out, probes.len());
        for probe in probes {
            out.push(channel_tag(probe.channel));
            push_str(&mut out, &probe.challenge_token_digest);
            out.push(outcome_tag(probe.outcome));
            out.extend_from_slice(&probe.window_start_ms.to_be_bytes());
            out.extend_from_slice(&probe.window_end_ms.to_be_bytes());
        }
        out
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ExpectedExternalWitness {
    pub run_id: String,
    pub phase_id: String,
    pub challenge_nonce: String,
    pub trusted_witness_id: String,
    pub trusted_witness_key_id: String,
    pub endpoint_set_digest: String,
    pub phase_start_ms: u64,
    pub phase_end_ms: u64,
    pub required_probes: Vec<RequiredWitnessProbe>,
}

/// Validates an independently signed canary-arrival receipt against a frozen phase.
///
/// This is intentionally separate from `qualify_ai_off`: a local host/controller
/// report and an external witness report are different evidence sources. Production
/// callers should use `qualify_ai_off_with_witness` so both reports are required.
pub fn qualify_external_witness<V: ReceiptSignatureVerifier>(
    receipt: &ExternalWitnessReceipt,
    signature: &[u8],
    expected: &ExpectedExternalWitness,
    verifier: &V,
) -> QualificationReport {
    let mut report = QualificationReport::new();

    if expected.run_id.trim().is_empty()
        || expected.phase_id.trim().is_empty()
        || expected.trusted_witness_id.trim().is_empty()
        || expected.trusted_witness_key_id.trim().is_empty()
    {
        report.fail("expected witness run, phase, identity, and key identifiers must be non-empty");
    }
    if expected.challenge_nonce.len() < 32 {
        report.fail("expected witness challenge nonce must contain at least 32 characters");
    }
    if !is_sha256(&expected.endpoint_set_digest) {
        report.fail("expected witness endpoint-set digest is malformed");
    }
    if expected.phase_start_ms >= expected.phase_end_ms {
        report.fail("expected phase observation window is empty or reversed");
    }
    if expected.required_probes.is_empty() {
        report.fail("expected witness probe set is empty");
    }

    if receipt.schema_version != "external-witness-v0.1" {
        report.fail("external witness schema version is unsupported");
    }
    for (label, actual, wanted) in [
        ("run id", receipt.run_id.as_str(), expected.run_id.as_str()),
        ("phase id", receipt.phase_id.as_str(), expected.phase_id.as_str()),
        (
            "challenge nonce",
            receipt.challenge_nonce.as_str(),
            expected.challenge_nonce.as_str(),
        ),
        (
            "witness identity",
            receipt.witness_id.as_str(),
            expected.trusted_witness_id.as_str(),
        ),
        (
            "witness key id",
            receipt.witness_key_id.as_str(),
            expected.trusted_witness_key_id.as_str(),
        ),
        (
            "endpoint-set digest",
            receipt.endpoint_set_digest.as_str(),
            expected.endpoint_set_digest.as_str(),
        ),
    ] {
        if actual != wanted {
            report.fail(format!("external witness {label} differs from frozen phase"));
        }
    }
    if !is_sha256(&receipt.endpoint_set_digest) {
        report.fail("external witness endpoint-set digest is malformed");
    }
    if receipt.observation_start_ms >= receipt.observation_end_ms {
        report.fail("external witness observation window is empty or reversed");
    }
    if receipt.event_sequence_start > receipt.event_sequence_end {
        report.fail("external witness event sequence is reversed");
    }

    if signature.is_empty() {
        report.fail("external witness signature is empty");
    } else {
        match verifier.verify(
            &receipt.witness_key_id,
            &receipt.signing_bytes(),
            signature,
        ) {
            Ok(true) => {}
            Ok(false) => report.fail("external witness signature is invalid"),
            Err(error) => {
                report.indeterminate(format!("external witness verifier unavailable: {error}"))
            }
        }
    }

    if report.status == QualificationStatus::Fail {
        return report;
    }
    if receipt.observation_start_ms > expected.phase_start_ms
        || receipt.observation_end_ms < expected.phase_end_ms
    {
        report.indeterminate("external witness observation does not cover the complete phase");
    }

    let mut required_by_channel = BTreeMap::new();
    let mut expected_tokens = BTreeSet::new();
    for required in &expected.required_probes {
        if !is_sha256(&required.challenge_token_digest) {
            report.fail("expected witness canary-token digest is malformed");
        }
        if !expected_tokens.insert(required.challenge_token_digest.as_str()) {
            report.fail("expected witness canary-token digests must be unique");
        }
        if required_by_channel
            .insert(required.channel, required.challenge_token_digest.as_str())
            .is_some()
        {
            report.fail("expected witness probe channels must be unique");
        }
    }

    let mut observed_by_channel = BTreeMap::new();
    for probe in &receipt.probes {
        if !is_sha256(&probe.challenge_token_digest) {
            report.fail("observed witness canary-token digest is malformed");
        }
        if !required_by_channel.contains_key(&probe.channel) {
            report.fail("external witness receipt contains an unexpected probe channel");
        }
        if observed_by_channel.insert(probe.channel, probe).is_some() {
            report.fail("external witness receipt contains a duplicate probe channel");
        }
        if probe.window_start_ms >= probe.window_end_ms {
            report.fail("external witness probe window is empty or reversed");
        }
    }

    for (channel, expected_token) in &required_by_channel {
        let Some(observed) = observed_by_channel.get(channel) else {
            report.indeterminate(format!("external witness probe {channel:?} is missing"));
            continue;
        };
        if observed.challenge_token_digest.as_str() != *expected_token {
            report.fail(format!("external witness probe {channel:?} token digest differs"));
        }
        if observed.window_start_ms > expected.phase_start_ms
            || observed.window_end_ms < expected.phase_end_ms
        {
            report.indeterminate(format!(
                "external witness probe {channel:?} does not cover the complete phase"
            ));
        }
        match observed.outcome {
            WitnessOutcome::NoArrival => {}
            WitnessOutcome::Arrived => {
                report.fail(format!("prohibited canary arrived at witness channel {channel:?}"))
            }
            WitnessOutcome::Unknown | WitnessOutcome::NotObserved => {
                report.indeterminate(format!(
                    "external witness channel {channel:?} has no complete observation"
                ))
            }
        }
    }

    report
}

/// Combined entry point: a local AI-off validator PASS is insufficient without
/// a separately authenticated external-witness PASS for the same frozen phase.
pub struct AiOffWitnessBundle<'a> {
    pub claims: &'a AiOffClaims,
    pub host_signature: &'a [u8],
    pub expected_phase: &'a ExpectedAiOffPhase,
    pub witness_receipt: &'a ExternalWitnessReceipt,
    pub witness_signature: &'a [u8],
    pub expected_witness: &'a ExpectedExternalWitness,
}

pub fn qualify_ai_off_with_witness<V: ReceiptSignatureVerifier, R: ReplayGuard>(
    bundle: AiOffWitnessBundle<'_>,
    verifier: &V,
    replay: &mut R,
) -> QualificationReport {
    let host_report = super::qualify_ai_off(
        bundle.claims,
        bundle.host_signature,
        bundle.expected_phase,
        verifier,
        replay,
    );
    let witness_report = qualify_external_witness(
        bundle.witness_receipt,
        bundle.witness_signature,
        bundle.expected_witness,
        verifier,
    );

    let mut combined = QualificationReport::new();
    if host_report.status == QualificationStatus::Fail
        || witness_report.status == QualificationStatus::Fail
    {
        combined.status = QualificationStatus::Fail;
    } else if host_report.status == QualificationStatus::Indeterminate
        || witness_report.status == QualificationStatus::Indeterminate
    {
        combined.status = QualificationStatus::Indeterminate;
    }
    combined.reasons.extend(
        host_report
            .reasons
            .into_iter()
            .map(|reason| format!("host: {reason}")),
    );
    combined.reasons.extend(
        witness_report
            .reasons
            .into_iter()
            .map(|reason| format!("external witness: {reason}")),
    );
    combined
}

fn channel_tag(channel: WitnessChannel) -> u8 {
    match channel {
        WitnessChannel::InferenceEndpoint => 1,
        WitnessChannel::AgentControlPlane => 2,
        WitnessChannel::Ipv4Egress => 3,
        WitnessChannel::Ipv6Egress => 4,
        WitnessChannel::DnsEgress => 5,
        WitnessChannel::ProxyEgress => 6,
        WitnessChannel::UdpEgress => 7,
    }
}

fn outcome_tag(outcome: WitnessOutcome) -> u8 {
    match outcome {
        WitnessOutcome::NoArrival => 1,
        WitnessOutcome::Arrived => 2,
        WitnessOutcome::Unknown => 3,
        WitnessOutcome::NotObserved => 4,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct TestVerifier;

    impl ReceiptSignatureVerifier for TestVerifier {
        fn verify(
            &self,
            key_id: &str,
            message: &[u8],
            signature: &[u8],
        ) -> Result<bool, String> {
            Ok(key_id == "witness-key" && !message.is_empty() && signature == b"valid")
        }
    }

    fn digest(character: char) -> String {
        std::iter::repeat_n(character, 64).collect()
    }

    fn expected() -> ExpectedExternalWitness {
        ExpectedExternalWitness {
            run_id: "run-1".into(),
            phase_id: "ai-off-1".into(),
            challenge_nonce: "n".repeat(32),
            trusted_witness_id: "witness-1".into(),
            trusted_witness_key_id: "witness-key".into(),
            endpoint_set_digest: digest('a'),
            phase_start_ms: 100,
            phase_end_ms: 200,
            required_probes: vec![
                RequiredWitnessProbe {
                    channel: WitnessChannel::InferenceEndpoint,
                    challenge_token_digest: digest('b'),
                },
                RequiredWitnessProbe {
                    channel: WitnessChannel::Ipv4Egress,
                    challenge_token_digest: digest('c'),
                },
            ],
        }
    }

    fn receipt(expected: &ExpectedExternalWitness) -> ExternalWitnessReceipt {
        ExternalWitnessReceipt {
            schema_version: "external-witness-v0.1".into(),
            run_id: expected.run_id.clone(),
            phase_id: expected.phase_id.clone(),
            challenge_nonce: expected.challenge_nonce.clone(),
            witness_id: expected.trusted_witness_id.clone(),
            witness_key_id: expected.trusted_witness_key_id.clone(),
            endpoint_set_digest: expected.endpoint_set_digest.clone(),
            observation_start_ms: 90,
            observation_end_ms: 210,
            event_sequence_start: 1,
            event_sequence_end: 4,
            probes: expected
                .required_probes
                .iter()
                .map(|required| WitnessProbeObservation {
                    channel: required.channel,
                    challenge_token_digest: required.challenge_token_digest.clone(),
                    outcome: WitnessOutcome::NoArrival,
                    window_start_ms: 90,
                    window_end_ms: 210,
                })
                .collect(),
        }
    }

    fn qualify(
        receipt: &ExternalWitnessReceipt,
        signature: &[u8],
        expected: &ExpectedExternalWitness,
    ) -> QualificationReport {
        qualify_external_witness(receipt, signature, expected, &TestVerifier)
    }

    #[test]
    fn complete_signed_witness_receipt_passes() {
        let expected = expected();
        let receipt = receipt(&expected);
        let report = qualify(&receipt, b"valid", &expected);
        assert_eq!(report.status, QualificationStatus::Pass);
        assert!(report.reasons.is_empty());
    }

    #[test]
    fn missing_required_probe_is_indeterminate() {
        let expected = expected();
        let mut receipt = receipt(&expected);
        receipt.probes.pop();
        let report = qualify(&receipt, b"valid", &expected);
        assert_eq!(report.status, QualificationStatus::Indeterminate);
    }

    #[test]
    fn successful_canary_arrival_fails() {
        let expected = expected();
        let mut receipt = receipt(&expected);
        receipt.probes[0].outcome = WitnessOutcome::Arrived;
        let report = qualify(&receipt, b"valid", &expected);
        assert_eq!(report.status, QualificationStatus::Fail);
    }

    #[test]
    fn mismatched_nonce_fails() {
        let expected = expected();
        let mut receipt = receipt(&expected);
        receipt.challenge_nonce = "x".repeat(32);
        let report = qualify(&receipt, b"valid", &expected);
        assert_eq!(report.status, QualificationStatus::Fail);
    }

    #[test]
    fn invalid_signature_fails() {
        let expected = expected();
        let receipt = receipt(&expected);
        let report = qualify(&receipt, b"invalid", &expected);
        assert_eq!(report.status, QualificationStatus::Fail);
    }

    #[test]
    fn partial_witness_window_is_indeterminate() {
        let expected = expected();
        let mut receipt = receipt(&expected);
        receipt.observation_end_ms = expected.phase_end_ms - 1;
        let report = qualify(&receipt, b"valid", &expected);
        assert_eq!(report.status, QualificationStatus::Indeterminate);
    }

    #[test]
    fn duplicate_probe_channel_fails() {
        let expected = expected();
        let mut receipt = receipt(&expected);
        receipt.probes.push(receipt.probes[0].clone());
        let report = qualify(&receipt, b"valid", &expected);
        assert_eq!(report.status, QualificationStatus::Fail);
    }

    #[test]
    fn mismatched_canary_token_fails() {
        let expected = expected();
        let mut receipt = receipt(&expected);
        receipt.probes[0].challenge_token_digest = digest('f');
        let report = qualify(&receipt, b"valid", &expected);
        assert_eq!(report.status, QualificationStatus::Fail);
    }

    #[test]
    fn unknown_probe_outcome_is_indeterminate() {
        let expected = expected();
        let mut receipt = receipt(&expected);
        receipt.probes[0].outcome = WitnessOutcome::Unknown;
        let report = qualify(&receipt, b"valid", &expected);
        assert_eq!(report.status, QualificationStatus::Indeterminate);
    }

    #[test]
    fn probe_window_must_cover_full_phase() {
        let expected = expected();
        let mut receipt = receipt(&expected);
        receipt.probes[0].window_start_ms = expected.phase_start_ms + 1;
        let report = qualify(&receipt, b"valid", &expected);
        assert_eq!(report.status, QualificationStatus::Indeterminate);
    }
}
