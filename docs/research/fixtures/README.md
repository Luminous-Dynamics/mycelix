# Cross-Scale Wisdom Corpus v1

This directory contains synthetic, versioned cases for checking epistemic distinctions, personal agency, consent boundaries, pluralism, dissent, and evidence lineage from individual reflection to group decisions.

## Files

- `cross-scale-wisdom-v1.jsonl`: one scenario per line.
- `cross-scale-wisdom-v1.manifest.json`: corpus identity, record count, severity breakdown, required invariants, and SHA-256 of the exact JSONL bytes.

## Record contract

Each record contains:
- `schema_version`: `cross-scale-wisdom-case/1.0`
- `case_id`: stable unique identifier
- `family`, `severity`, and `context`
- `evidence`: typed claims/records with status, description, and synthetic provenance
- `expected_invariants`: required safeguards
- `forbidden_behaviors`: failure patterns
- `expected_response_contract`: observable response obligation, not a single gold answer
- `rationale`: what the case tests

This is not a set of canonical answers to theological questions. It defines evaluation obligations while preserving different legitimate worldviews. `review` cases require qualified human review; `hard_fail` cases represent declared protocol violations when a prohibited behavior is observed.

## Reproducibility

The manifest hash is SHA-256 over the exact JSONL bytes, including the final newline. Verify it before evaluation. Any content change requires a new corpus version and updated manifest; do not silently mutate a frozen corpus used by a reported result.

## Limits

All records are synthetic. They contain no participant reflections and are not a validated psychological, clinical, or spiritual-development measure. Passing the corpus shows only that a tested implementation met specified fixture obligations. It does not establish human efficacy, ethical adequacy, wisdom, enlightenment, consciousness, or field safety.

This corpus complements rather than replaces Symthaea WCARE-09 adversarial/longitudinal evaluation and WCARE-10 fail-closed qualification receipts. Version 1 is a research seed requiring independent review and adversarial expansion.


## Integral-lens supplement v0.1

- Cases: 12
- Manifest: [cross-scale-wisdom-integral-lens-v0.1.manifest.json](cross-scale-wisdom-integral-lens-v0.1.manifest.json)
- Supplement: [cross-scale-wisdom-integral-lens-v0.1.jsonl](cross-scale-wisdom-integral-lens-v0.1.jsonl)
- Rationale and limits: [cross-scale-wisdom-integral-lens-v0.1.md](cross-scale-wisdom-integral-lens-v0.1.md)

This supplement does not modify corpus v1.0.0. It tests optional use of Integral Theory/AQAL, including no developmental-stage ranking, state-versus-trait distinction, no forced quadrant classification, respect for lens opt-out, preserving disagreement between lenses, not psychologizing material barriers, consent despite community norms, acknowledging diversity within traditions, and avoiding overgeneralization from narrow empirical evidence. It is synthetic and has not yet undergone independent cultural or semantic review.

## Spiritual-practice boundary supplement v0.1

- Cases: 12 (10 hard-fail candidates, 2 human-review cases)
- Manifest: [cross-scale-wisdom-spiritual-practice-boundaries-v0.1.manifest.json](cross-scale-wisdom-spiritual-practice-boundaries-v0.1.manifest.json)
- Supplement: [cross-scale-wisdom-spiritual-practice-boundaries-v0.1.jsonl](cross-scale-wisdom-spiritual-practice-boundaries-v0.1.jsonl)
- Rationale and limits: [cross-scale-wisdom-spiritual-practice-boundaries-v0.1.md](cross-scale-wisdom-spiritual-practice-boundaries-v0.1.md)

This separate, unscored supplement covers non-pathologizing reflection on spiritual experience, immediate-risk support boundaries, coercive authority, unsupported clinical-cure claims, stage-ranking misuse, secular opt-out, material barriers, commercial guarantees, cross-tradition disagreement, high-intensity practice under uncertainty, sensitive-intake minimization, and translation uncertainty. It does not modify the immutable v1 corpus or the existing Integral-lens supplement. Structural validity does not establish safety or efficacy.

## Standalone Rust verifier

The implementation and adversarial tests are in [tools/cross-scale-wisdom-verifier](../../../tools/cross-scale-wisdom-verifier/README.md). To run both corpus fixtures, follow its two CLI examples. The report status is deliberately STRUCTURALLY_VALID_UNSCORED; a successful structural check is not model-response scoring or human-efficacy evidence.

## Spiritual-practice boundary supplement v0.1

- Cases: 12 (10 hard-fail candidates, 2 human-review cases)
- Manifest: [cross-scale-wisdom-spiritual-practice-boundaries-v0.1.manifest.json](cross-scale-wisdom-spiritual-practice-boundaries-v0.1.manifest.json)
- Supplement: [cross-scale-wisdom-spiritual-practice-boundaries-v0.1.jsonl](cross-scale-wisdom-spiritual-practice-boundaries-v0.1.jsonl)
- Rationale and limits: [cross-scale-wisdom-spiritual-practice-boundaries-v0.1.md](cross-scale-wisdom-spiritual-practice-boundaries-v0.1.md)

This separate, unscored supplement covers non-pathologizing reflection on spiritual experience, immediate-risk support boundaries, coercive authority, unsupported clinical-cure claims, stage-ranking misuse, secular opt-out, material barriers, commercial guarantees, cross-tradition disagreement, high-intensity practice under uncertainty, sensitive-intake minimization, and translation uncertainty. It does not modify the immutable v1 corpus or the existing Integral-lens supplement. Structural validity does not establish safety or efficacy.
