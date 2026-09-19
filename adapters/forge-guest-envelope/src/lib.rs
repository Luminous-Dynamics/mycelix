// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! FORGE-004D3B2C2B: re-qualifiable guest evidence crossing the sandbox boundary.
//!
//! A child process must not send a serialized positive authority object and ask
//! the parent to trust it. This envelope carries only raw/portable evidence
//! objects. The parent re-derives local-key trust, gittuf request binding, and
//! guest-transcript qualification from the exact plan and tool-map subjects.

use mycelix_forge_core::{Digest, DigestAlgorithm};
use mycelix_forge_gittuf_adapter::{AdapterError, GittufLocalReceipt};
use mycelix_forge_gittuf_trust_profile::{
    qualify_local_key_profile, PolicyTrustInventory, QualifiedLocalKeyTrustProfile,
    TrustProfileError,
};
use mycelix_forge_guest_plan::{GuestPlanError, GuestVerificationPlanV1};
use mycelix_forge_guest_tool_map::{GuestToolMapError, GuestToolMapV1};
use mycelix_forge_guest_transcript::{
    qualify_guest_transcript, GuestTranscriptBindings, GuestTranscriptError, GuestTranscriptV1,
    QualifiedGuestTranscript,
};
use mycelix_forge_linux_isolation_evidence::{
    InsideIsolationEvidence, IsolationEvidenceError,
};
use serde::{Deserialize, Serialize};
use thiserror::Error;

const ENVELOPE_DOMAIN_V1: &[u8] = b"mycelix-forge/guest-evidence-envelope/v1\0";

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct GuestEvidenceEnvelopeV1 {
    plan_digest: Digest,
    tool_map_digest: Digest,
    inside_isolation: InsideIsolationEvidence,
    policy_inventory: PolicyTrustInventory,
    gittuf_receipt: GittufLocalReceipt,
    transcript: GuestTranscriptV1,
}

impl GuestEvidenceEnvelopeV1 {
    pub fn new(
        plan_digest: Digest,
        tool_map_digest: Digest,
        inside_isolation: InsideIsolationEvidence,
        policy_inventory: PolicyTrustInventory,
        gittuf_receipt: GittufLocalReceipt,
        transcript: GuestTranscriptV1,
    ) -> Self {
        Self {
            plan_digest,
            tool_map_digest,
            inside_isolation,
            policy_inventory,
            gittuf_receipt,
            transcript,
        }
    }

    pub fn plan_digest(&self) -> &Digest {
        &self.plan_digest
    }

    pub fn tool_map_digest(&self) -> &Digest {
        &self.tool_map_digest
    }

    pub fn inside_isolation(&self) -> &InsideIsolationEvidence {
        &self.inside_isolation
    }

    pub fn policy_inventory(&self) -> &PolicyTrustInventory {
        &self.policy_inventory
    }

    pub fn gittuf_receipt(&self) -> &GittufLocalReceipt {
        &self.gittuf_receipt
    }

    pub fn transcript(&self) -> &GuestTranscriptV1 {
        &self.transcript
    }

    pub fn canonical_bytes(&self) -> Result<Vec<u8>, GuestEnvelopeError> {
        let inside = self.inside_isolation.digest(DigestAlgorithm::Sha256)?;
        let inventory = self.policy_inventory.digest(DigestAlgorithm::Sha256)?;
        let receipt = self.gittuf_receipt.commitment(DigestAlgorithm::Sha256)?;
        let transcript = self.transcript.digest(DigestAlgorithm::Sha256)?;

        let mut out = Vec::new();
        out.extend_from_slice(ENVELOPE_DOMAIN_V1);
        for digest in [
            &self.plan_digest,
            &self.tool_map_digest,
            &inside,
            &inventory,
            &receipt,
            &transcript,
        ] {
            push_digest(&mut out, digest)?;
        }
        Ok(out)
    }

    pub fn digest(&self, algorithm: DigestAlgorithm) -> Result<Digest, GuestEnvelopeError> {
        Ok(Digest::of_bytes(algorithm, &self.canonical_bytes()?))
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct QualifiedGuestEvidenceEnvelope {
    envelope_digest: Digest,
    plan_digest: Digest,
    tool_map_digest: Digest,
    inside_digest: Digest,
    inventory_digest: Digest,
    trust: QualifiedLocalKeyTrustProfile,
    gittuf_receipt: Digest,
    transcript: QualifiedGuestTranscript,
}

impl QualifiedGuestEvidenceEnvelope {
    pub fn envelope_digest(&self) -> &Digest {
        &self.envelope_digest
    }

    pub fn plan_digest(&self) -> &Digest {
        &self.plan_digest
    }

    pub fn tool_map_digest(&self) -> &Digest {
        &self.tool_map_digest
    }

    pub fn inside_digest(&self) -> &Digest {
        &self.inside_digest
    }

    pub fn inventory_digest(&self) -> &Digest {
        &self.inventory_digest
    }

    pub fn trust(&self) -> &QualifiedLocalKeyTrustProfile {
        &self.trust
    }

    pub fn gittuf_receipt(&self) -> &Digest {
        &self.gittuf_receipt
    }

    pub fn transcript(&self) -> &QualifiedGuestTranscript {
        &self.transcript
    }
}

/// Re-qualify all portable child evidence in the parent process.
///
/// This function intentionally receives the exact guest plan and raw tool map
/// again. Tool-map qualification against the enclosing ExecutionSpec/Nix
/// closure is a separate parent-side theorem; here we ensure the child envelope
/// cannot switch to a different plan/map subject while crossing the boundary.
pub fn qualify_guest_evidence_envelope(
    envelope: &GuestEvidenceEnvelopeV1,
    plan: &GuestVerificationPlanV1,
    tool_map: &GuestToolMapV1,
) -> Result<QualifiedGuestEvidenceEnvelope, GuestEnvelopeError> {
    let plan_digest = plan.digest(DigestAlgorithm::Sha256)?;
    if envelope.plan_digest != plan_digest || tool_map.plan_digest() != &plan_digest {
        return Err(GuestEnvelopeError::PlanDigestMismatch);
    }
    if tool_map.execution_subject() != plan.execution_subject() {
        return Err(GuestEnvelopeError::ExecutionSubjectMismatch);
    }

    let tool_map_digest = tool_map.digest(DigestAlgorithm::Sha256)?;
    if envelope.tool_map_digest != tool_map_digest {
        return Err(GuestEnvelopeError::ToolMapDigestMismatch);
    }

    if envelope.policy_inventory.policy_state() != plan.policy_state() {
        return Err(GuestEnvelopeError::PolicyStateMismatch);
    }
    let inventory_digest = envelope.policy_inventory.digest(DigestAlgorithm::Sha256)?;
    let trust = qualify_local_key_profile(&envelope.policy_inventory)?;
    if trust.profile_digest() != plan.trust_profile() {
        return Err(GuestEnvelopeError::TrustProfileMismatch);
    }

    // This validates the receipt's internal commitments and binds it to the
    // exact typed repository request carried by the guest plan.
    envelope
        .gittuf_receipt
        .clone()
        .into_observation_for(plan.request())?;
    let receipt = envelope
        .gittuf_receipt
        .commitment(DigestAlgorithm::Sha256)?;
    if &receipt != plan.expected_replay_receipt() {
        return Err(GuestEnvelopeError::GittufReceiptMismatch);
    }

    let inside_digest = envelope
        .inside_isolation
        .digest(DigestAlgorithm::Sha256)?;
    let bindings = GuestTranscriptBindings::new(
        inside_digest.clone(),
        inventory_digest.clone(),
        trust.evidence_digest().clone(),
        receipt.clone(),
    );
    let transcript = qualify_guest_transcript(plan, &envelope.transcript, &bindings)?;

    Ok(QualifiedGuestEvidenceEnvelope {
        envelope_digest: envelope.digest(DigestAlgorithm::Sha256)?,
        plan_digest,
        tool_map_digest,
        inside_digest,
        inventory_digest,
        trust,
        gittuf_receipt: receipt,
        transcript,
    })
}

fn push_digest(out: &mut Vec<u8>, digest: &Digest) -> Result<(), GuestEnvelopeError> {
    let algorithm = digest.algorithm().id().as_bytes();
    let algorithm_len = u16::try_from(algorithm.len())
        .map_err(|_| GuestEnvelopeError::CanonicalLengthOverflow("digest algorithm"))?;
    let digest_len = u16::try_from(digest.as_bytes().len())
        .map_err(|_| GuestEnvelopeError::CanonicalLengthOverflow("digest"))?;
    out.extend_from_slice(&algorithm_len.to_be_bytes());
    out.extend_from_slice(algorithm);
    out.extend_from_slice(&digest_len.to_be_bytes());
    out.extend_from_slice(digest.as_bytes());
    Ok(())
}

#[derive(Debug, Error)]
pub enum GuestEnvelopeError {
    #[error(transparent)]
    Adapter(#[from] AdapterError),
    #[error(transparent)]
    GuestPlan(#[from] GuestPlanError),
    #[error(transparent)]
    GuestToolMap(#[from] GuestToolMapError),
    #[error(transparent)]
    GuestTranscript(#[from] GuestTranscriptError),
    #[error(transparent)]
    Isolation(#[from] IsolationEvidenceError),
    #[error(transparent)]
    Trust(#[from] TrustProfileError),
    #[error("guest envelope names a different guest plan")]
    PlanDigestMismatch,
    #[error("guest tool map names a different execution subject")]
    ExecutionSubjectMismatch,
    #[error("guest envelope names a different guest tool map")]
    ToolMapDigestMismatch,
    #[error("guest trust inventory names a different repository policy state")]
    PolicyStateMismatch,
    #[error("guest trust qualification names a different local-key profile")]
    TrustProfileMismatch,
    #[error("guest gittuf receipt differs from the plan's expected replay receipt")]
    GittufReceiptMismatch,
    #[error("canonical length overflow: {0}")]
    CanonicalLengthOverflow(&'static str),
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(byte: u8) -> Digest {
        Digest::new(DigestAlgorithm::Sha256, vec![byte; 32]).unwrap()
    }

    #[test]
    fn envelope_domain_separates_component_order() {
        let mut a = Vec::new();
        a.extend_from_slice(ENVELOPE_DOMAIN_V1);
        push_digest(&mut a, &digest(1)).unwrap();
        push_digest(&mut a, &digest(2)).unwrap();

        let mut b = Vec::new();
        b.extend_from_slice(ENVELOPE_DOMAIN_V1);
        push_digest(&mut b, &digest(2)).unwrap();
        push_digest(&mut b, &digest(1)).unwrap();

        assert_ne!(
            Digest::of_bytes(DigestAlgorithm::Sha256, &a),
            Digest::of_bytes(DigestAlgorithm::Sha256, &b)
        );
    }
}
