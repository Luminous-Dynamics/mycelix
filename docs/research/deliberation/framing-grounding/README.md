# DELIB-FRAME-GROUND-001 v0.1

**Owner:** Mycelix #6084  
**Parent:** DELIB-BENCH-001 #5999  
**Status:** annotation contract + external-source metadata audit; no human data collected

## What is included

- `FRAMING_ANNOTATION_CODEBOOK_V0_1.md`: semantic relations, framing functions, cue tags, disagreement handling and language protocol.
- `FRAMING_ANNOTATION_RECORD_V0_1.schema.json`: strict contract separating human annotations, model annotations, rights status and adjudication.
- `FRAMING_EXTERNAL_INTAKE_V0_1.json` and schema: metadata-only rights/access ledger for WildFrame, framing-function annotation work, PNAS choice-architecture study and African/South African language resources.
- `FRAMING_CONTRACT_FIXTURES_V0_1.json`: 12 synthetic contract controls; no real participant data and no human annotation labels.
- `qualifier/`: small Rust validator for source custody and fixture status.
- `.github/workflows/delib-frame-ground-qualifier.yml`: path-filtered exact-head workflow.

## Critical data-custody finding

The WildFrame ACL Anthology page establishes the publication metadata and the paper's ACL publication rights, but does not provide a dataset-specific license or consent statement for the underlying real-world texts and human response annotations. This repository therefore records WildFrame as `metadata_only`: no raw text, human responses, dataset payload or derived data are committed, and evaluation/training/redistribution permissions are not assumed.

The same separation applies to source social-media content, human annotations and supplementary materials in other research. A public paper, an open dataset card, and a permission to train or redistribute participant-level content are not interchangeable.

## Run locally

```bash
cargo test --manifest-path docs/research/deliberation/framing-grounding/qualifier/Cargo.toml

cargo run --manifest-path docs/research/deliberation/framing-grounding/qualifier/Cargo.toml -- \
  docs/research/deliberation/framing-grounding/FRAMING_EXTERNAL_INTAKE_V0_1.json \
  docs/research/deliberation/framing-grounding/FRAMING_CONTRACT_FIXTURES_V0_1.json
```

The Rust program validates a bounded JSON contract. It is not a full JSON Schema implementation or a rights adjudicator.

## Language readiness

The required SA language target set is encoded, but the v0.1 package deliberately contains no generated translations or synthetic native-speaker claims. New non-English items require a native-authorship or native-adjudication receipt, and should not enter the benchmark merely because an automatic translation exists.

## Claim ceiling

A passing qualifier proves only that the committed metadata and synthetic contract fixtures satisfy the declared rules. It does not demonstrate human annotation reliability, semantic equivalence in natural language, valid consent, dataset rights, language parity, response effects or model superiority.
