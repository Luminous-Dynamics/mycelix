# DELIB-AGENDA-001 v0.1

**Status:** synthetic fixture design / structural qualification only  
**Owner:** Mycelix #6082  
**Parent:** Mycelix #5999

## Research question

Can a collective decision be materially steered before discussion begins by changing the agenda, option set, defaults, framing, order, bundles or timing—even when participants, evidence, preferences and decision rules are held fixed?

## Files

- DELIB_AGENDA_PROTOCOL_V0_1.json and schema: protocol, transformation families, metrics and claim ceiling.
- DELIB_AGENDA_FIXTURES_V0_1.json and schema: 18 fictional known-answer fixtures across 14 transformation families.
- qualifier/: standalone Rust structural and semantic qualifier.
- .github/workflows/delib-agenda-qualifier.yml: path-filtered exact-revision workflow.

## Local qualification

```bash
cargo test --manifest-path docs/research/deliberation/agenda-integrity/qualifier/Cargo.toml

cargo run --manifest-path docs/research/deliberation/agenda-integrity/qualifier/Cargo.toml -- \
  docs/research/deliberation/agenda-integrity/DELIB_AGENDA_PROTOCOL_V0_1.json \
  docs/research/deliberation/agenda-integrity/DELIB_AGENDA_PROTOCOL_V0_1.schema.json \
  docs/research/deliberation/agenda-integrity/DELIB_AGENDA_FIXTURES_V0_1.json \
  docs/research/deliberation/agenda-integrity/DELIB_AGENDA_FIXTURES_V0_1.schema.json
```

The validator checks a bounded structural contract, not full JSON Schema conformance or natural-language semantic equivalence. It computes source-option and presented-agenda SHA-256 commitments at qualification time and reports required-option coverage, constraint coverage and terminal disposition.

## Measurement semantics

- The source-contributed option registry is independent of the candidate agenda.
- Required options must remain represented; source origins and constraint IDs must resolve.
- A presented bundle may represent multiple source options only if it preserves all declared constraints.
- A source option split into multiple presented items requires a shared split group and explicit disclosure.
- Defaults require a declaration and an authority reference.
- A presented item introduced after the frozen cutoff is invalid unless its late mutation is explicitly authorized.
- Normative-pluralism tasks have no gold policy option.
- A disclosed trade-off may legitimately change a decision; the validator does not demand invariance in that case.
- Incomplete source registries produce INDETERMINATE rather than PASS.

## What this does not establish

No LLM agents or human participants were run. The corpus does not certify wording equivalence, agenda neutrality, fairness, legitimacy or decision quality. Those require blinded adjudication, matched randomized model experiments and separately approved human studies.
