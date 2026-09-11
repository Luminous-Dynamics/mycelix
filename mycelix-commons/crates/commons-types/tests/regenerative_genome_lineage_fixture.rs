// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use commons_types::{MaritimeEvidenceEnvelope, RegenerativeGenomeLineageEvidenceV1};

const FIXTURE: &str = include_str!(
    "../fixtures/maritime-evidence-v1-regenerative-genome-lineage.json"
);

#[test]
fn canonical_genome_lineage_fixture_pins_inner_and_outer_identities() {
    let envelope: MaritimeEvidenceEnvelope = serde_json::from_str(FIXTURE.trim()).unwrap();
    envelope.validate().unwrap();
    assert_eq!(
        envelope.content_digest().unwrap(),
        "ca5ac3b0c79012f852646e1bf6a61c2f07a8caf41abc69e676ff5bf949bae285"
    );

    let lineage: RegenerativeGenomeLineageEvidenceV1 =
        serde_json::from_str(&envelope.payload_json).unwrap();
    lineage.validate().unwrap();
    assert_eq!(
        lineage.content_digest().unwrap(),
        "a319bd837e9035f576ac5854d194dd3dbd351a5eae2511407c45135517775ca6"
    );
    assert_eq!(lineage.genome_id, "manta-genome-v2");
    assert_eq!(
        lineage.parent_genome_binding.as_deref(),
        Some("genome:manta-v1:sha256:example")
    );
    assert_eq!(
        lineage.substitution_evidence_refs,
        vec!["substitution:electronics-local-v1"]
    );
}
