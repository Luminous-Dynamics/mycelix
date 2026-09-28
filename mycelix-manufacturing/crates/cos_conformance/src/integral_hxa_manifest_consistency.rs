//! HXA refinement-manifest consistency witness.
//!
//! This is a traceability gate, not a proof of semantic equivalence or production refinement.

use serde::Deserialize;
use super::integral_hxa_formal::OBLIGATIONS;

#[derive(Debug, Deserialize)]
struct Manifest {
    manifest_id: String,
    status: String,
    source_status: String,
    interface_profile: String,
    closure_rule: String,
    claim_ceiling: Vec<String>,
    artifacts: Vec<Artifact>,
    obligations: Vec<ManifestObligation>,
    composition_obligations: Vec<ManifestObligation>,
    participant_evidence_required: Vec<String>,
    non_claims: Vec<String>,
}
#[derive(Debug, Deserialize)]
struct Artifact { path: String, sha: String, role: String }
#[derive(Debug, Deserialize)]
struct ManifestObligation { id: String, witness: String, claim_ceiling: String }

const MANIFEST: &str = include_str!("../../../../docs/integral-hxa-refinement-manifest-v1.json");

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn manifest_matches_executable_hxa_registry() {
        let manifest: Manifest = serde_json::from_str(MANIFEST).expect("manifest JSON must parse");
        assert_eq!(manifest.manifest_id, "INTEGRAL-HXA-REFINEMENT-MANIFEST-001");
        assert_eq!(manifest.status, "BoundedExecutableWitness");
        assert_eq!(manifest.source_status, "CandidateInterface");
        assert_eq!(manifest.interface_profile, "INTEGRAL-HXA-IF01-COMPOSITION-CANDIDATE");
        assert!(!manifest.closure_rule.is_empty());
        assert_eq!(manifest.claim_ceiling.len(), 2);
        assert_eq!(manifest.artifacts.len(), 3);
        for artifact in &manifest.artifacts {
            assert!(!artifact.path.is_empty());
            assert!(!artifact.sha.is_empty());
            assert!(!artifact.role.is_empty());
        }
        assert_eq!(manifest.obligations.len(), OBLIGATIONS.len());
        for (entry, executable) in manifest.obligations.iter().zip(OBLIGATIONS.iter()) {
            assert_eq!(entry.id, executable.id);
            assert_eq!(entry.witness, executable.witness);
            assert_eq!(entry.claim_ceiling, executable.claim_ceiling);
        }
        assert_eq!(manifest.composition_obligations.len(), 2);
        assert_eq!(manifest.composition_obligations[0].id, "HXA-IF01-C01");
        assert_eq!(manifest.composition_obligations[0].witness, "evaluate_cross_layer");
        assert_eq!(manifest.composition_obligations[1].id, "HXA-IF01-C02");
        assert_eq!(manifest.composition_obligations[1].witness, "if01_and_hxa_are_both_bounded_reference_models");
        assert!(!manifest.participant_evidence_required.is_empty());
        assert!(manifest.non_claims.iter().any(|c| c.contains("Traceability")));
        assert!(manifest.non_claims.iter().any(|c| c.contains("scalar")));
    }
}
