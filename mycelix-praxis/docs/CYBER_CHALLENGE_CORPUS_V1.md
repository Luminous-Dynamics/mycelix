# Praxis Cyber Challenge Corpus v1 (research/conformance draft)

## Purpose

This is a small, entirely synthetic corpus for teaching and testing evidence-bound defensive cyber-engineering decisions. It is the first structural slice of [issue #5124](https://github.com/Luminous-Dynamics/mycelix/issues/5124), not a production cyber-range, incident-response engine, or qualification claim.

Files under `examples/cyber-challenges/v1/` are deliberately split:

- `solver-visible.json`: case prompts, synthetic observations, freshness labels, bounded objectives, permitted next-step examples, and authority boundaries.
- `evaluator-only.json`: expected dispositions, evidence references, acceptable inferences, prohibited claims, rubric weights, and authority gates.
- `solver-visible.schema.json` and `evaluator-only.schema.json`: JSON Schema 2020-12 wire contracts.
- `corpus-manifest.json`: corpus identity/version and paths for maintainers.

The production Rust module `crates/praxis-core/src/cyber_challenges.rs` parses only the solver-visible contract. It does **not** include or load the evaluator fixture. The integration-test binary separately reads both files to check conformance. All evaluator cases in this public corpus are visible in the repository; they are public format/evaluator fixtures, **not held-out qualification data**. A learner-facing build must not load the evaluator file.

## Paradigm shift

The assessment target is not simply “can recall a security fact?” It is:

> Can the learner make a defensible security decision from incomplete, stale, or conflicting evidence, explain what is observed versus inferred, request the next useful evidence, and respect action authority?

The five synthetic cases cover:
1. identity sign-in signals with shared-egress ambiguity and stale endpoint context;
2. a network-isolation configuration risk without proof of exploitation;
3. a dependency-digest conflict with stale registry metadata;
4. an untrusted prompt-injection instruction without evidence that a tool action occurred;
5. a quarantine request without a receipt proving the effect happened.

Every case includes evidence IDs, source classes, freshness, conflict status, a time offset, a concrete prompt, allowed next-step examples, and an explicit authority boundary. Freshness and conflict are independent: current evidence can conflict with another source, while stale evidence may not have been reconciled. The evaluator separately encodes the expected disposition, supporting evidence references, acceptable alternatives, claim limits, safe follow-up, and a 1,000-permille rubric.

## Structural gates implemented

- Solver corpus schema/corpus ID/version must be recognized and non-empty.
- Scenario IDs must be well-formed and unique.
- Every case must explicitly be synthetic.
- Required learning prompt fields, evidence observations, permitted next steps, and authority boundaries cannot be empty.
- Evidence IDs must be unique inside their scenario.
- Freshness (`current`, `stale`, `unknown`) is represented separately from conflict status (`no_known_conflict`, `conflicts_with_other_evidence`, `not_assessed`).
- Rust's `deny_unknown_fields` rejects evaluator-only fields inserted into a solver scenario.
- Test tooling requires an exact match between solver and evaluator scenario IDs, verifies that evaluator evidence references exist in the solver view, and checks that each rubric sums to 1,000.
- Schema and corpus versioning, separation paths, and selected coverage properties are checked in integration tests.
- The new exact-head workflow checks Rust formatting and runs the focused `cyber_challenges` test target on Rust 1.96.0.

These are structural conformance checks, not proof that the scenarios are psychometrically valid, representative, fair, secure against all oracle leakage, or effective at teaching learners.

## Benchmark plan

### Gate 0 — contract correctness

- 100% of public positive fixtures parse and validate.
- 100% of malformed/mutated fixtures fail with a specific validation finding.
- Zero evaluator-only fields are accepted in the solver-visible projection.
- 100% of evaluator-required evidence references resolve to solver-visible evidence for the same scenario.
- Every scoring rubric sums to 1,000; no missing or unscored case is silently dropped.

### Gate 1 — evidence reasoning

On separately authored, reviewed scenarios, measure independently:
- decision/disposition correctness;
- correct evidence citation and provenance;
- ability to separate observation, inference, hypothesis, and unknown;
- freshness/conflict handling;
- appropriate requests for additional evidence;
- uncertainty calibration and justified abstention;
- unsafe-action and unauthorized-action rate;
- evidence-gathering efficiency / information gain;
- explanation quality and corrective learning after feedback.

Do not combine these into one compensatory score. Unsupported claims, unauthorized actions, fabricated evidence, or evaluator leakage should be reported as non-compensable failures.

### Gate 2 — transfer and reliability

Before a serious qualification study, create a predeclared corpus of at least 50 synthetic cases with domain balance, paired variants, benign alternatives, stale/conflicting evidence, and holdout variants. Use independent raters and publish rubric agreement, task/form versions, missingness, exclusions, confidence intervals, and negative/null results. The 5-case public seed is not sufficient evidence of cyber competence.

The public evaluator fixtures are useful for contract conformance only. Real capability measurement requires separately versioned held-out scenarios and evaluator material with controlled exposure. Record the exact commit/corpus version, scenario and rubric IDs, solver/tool assistance, evaluator version, source snapshot, per-case disposition, and every failure. Do not claim general cybersecurity competence from a small synthetic fixture.

## External references

These frameworks help map the instructional domain; they do not establish local applicability or incident truth:

- [NIST NICE Framework, current versions](https://www.nist.gov/itl/applied-cybersecurity/nice/nice-framework-resource-center/nice-framework-current-versions) — role/task/knowledge/skill vocabulary.
- [MITRE ATT&CK tactics](https://attack.mitre.org/tactics/) — behavioral context; a matching tactic label is not proof a behavior occurred.
- [CISA Known Exploited Vulnerabilities Catalog](https://www.cisa.gov/known-exploited-vulnerabilities-catalog) — prioritization context; catalog presence/absence does not alone prove applicability to a particular asset.
- [OWASP Top 10 for LLM Applications](https://genai.owasp.org/llm-top-10/) — context for AI/tool-boundary and prompt-injection cases; a suspicious instruction is not evidence that an action executed.

## Nonclaims and next steps

This seed does not establish:
- that a live device/account was compromised;
- that a vulnerable configuration was exploited;
- that a package is malicious because a digest conflicts;
- that a prompt-injection string caused data exfiltration;
- that a requested quarantine succeeded without an effect receipt;
- production readiness, universal cybersecurity competence, certification eligibility, or operational authority.

Next steps are to add evidence-strength and source-snapshot binding, frozen negative controls, a separately held-out corpus, rubric calibration, accessibility review, and transfer/delayed-retention evaluation. Compose with existing Symthaea cyber reasoning/range work rather than duplicating its ontology or operational authority. Link every future benchmark output to its exact source, corpus, evaluator, rubric, and environment versions.
