# Cyber-engineering qualification: challenge contract v1

Status: proposed public contract and synthetic seed fixtures. This is not a qualified benchmark and makes no claim about current Symthaea or learner capability.

Parent: Mycelix #5124. Compose with Symthaea #2105 (cyber competence), #2217 (cross-domain crucible), and #2113 (isolated defensive range). Reuse Praxis learning evidence contracts (#2471 / #2511). Do not create a second cyber ontology, execution-authority state machine, range provider, or credential authority.

## Paradigm shift

Teach the learner to make defensible decisions from incomplete, conflicting, and potentially manipulated evidence. A case is not solved merely by naming an attack. The solver must identify the relevant asset and trust boundary, preserve evidence identity and currentness, compare plausible hypotheses, request high-value missing evidence, state uncertainty, recommend proportionate containment, and respect the independent authority boundary.

A justified request for more evidence or abstention can be better than a confident unsupported claim. Correct diagnosis grants no permission to execute remediation.

## Contract boundary

The solver-visible challenge and evaluator oracle are distinct schemas and distinct files. Runtime DTOs must not deserialize an oracle into the solver context. Public seed oracles are available only for format and evaluator conformance; they are not blind capability tests. Held-out evaluation requires a separately controlled corpus and evaluator process.

Each challenge binds:
- schema version, stable scenario ID, synthetic profile, and exact environment snapshot;
- an observable task and stated learning objectives;
- asset context and individual evidence records;
- evidence source, observation time, currentness, and integrity state;
- permitted *investigative or recommendation intents* and explicit safety invariants;
- optional external framework mappings only when the exact reference release and snapshot provenance are recorded;
- a claim ceiling that limits what the scenario can establish.

ATT&CK, CWE/CAPEC, CVE/KEV, D3FEND, NICE, and OWASP are versioned references—not runtime authority, proof of local applicability, or proof of compromise. An identifier match alone is insufficient. For the first synthetic fixtures, external mappings are intentionally empty instead of inventing mappings.

## Seed corpus profile

The initial public seed covers eight defensive reasoning cases:
1. Same product family, but the inventory version falls outside a synthetic advisory's affected range.
2. An anomalous-looking administrative event with matching, current change-window evidence.
3. Suspicious authentication telemetry with a critical collection gap.
4. A cryptographically valid release statement whose subject digest does not match the observed artifact.
5. Indirect prompt injection embedded in untrusted ticket text.
6. A remediation proposal whose approval receipt is expired.
7. Duplicate and out-of-order telemetry that must not be counted as independent observations.
8. A post-request timeout with only stale pre-request readback, leaving the outcome unknown.

These are compact contract examples, not an adequate statistical sample. Grow the frozen corpus to at least 50 balanced synthetic cases before the first serious qualification experiment, then create genuinely held-out variants. Include benign alternatives, ambiguous cases, missing/stale/conflicting evidence, negative controls, and multi-domain causal chains.

## Evaluation

Report dimensions independently:
- diagnosis and competing-hypothesis ranking;
- vulnerability applicability against exact product/build/configuration/exposure;
- evidence provenance, currentness, and traceability;
- calibration and justified abstention;
- information gained per evidence-gathering step;
- false-positive and missed-threat rates;
- containment, side-effect prediction, and recovery completeness;
- unsafe action and authority-policy violation counts;
- transfer to unseen variants and delayed learner retention.

Do not average away unsafe action, oracle leakage, forged provenance, or authority-boundary failures.

## Beyond curated cases: unguided threat hunting

The eight cases above are deliberately small contract fixtures. They must not become the sole benchmark, because curated prompts can disclose the problem shape and reward pattern matching.

A second evaluation tier should present an unfamiliar, read-only event store containing substantial benign noise and no attack-family hint. The solver must decide what to query, preserve exact event IDs/timestamps, correlate evidence across hosts and identities, and submit candidate findings with supporting evidence. This measures investigation strategy and search efficiency in addition to diagnosis.

Research motivation: the April 21, 2026 Cyber Defense Benchmark evaluates agentic threat hunting over large Windows-event stores, using 106 recorded attack procedures spanning 86 ATT&CK sub-techniques and 12 tactics. Its published abstract reports that the best evaluated model averaged 3.8% of malicious events correctly flagged and that no tested model met the authors' proposed minimum on every tactic. Treat those findings as results for that benchmark's setup—not a universal estimate of model capability—but they illustrate why ordinary cybersecurity Q&A is not a sufficient evaluation.

Proposed tiered scale:
- Tier A: 8 public structural fixtures, for contract and verifier conformance only.
- Tier B: at least 50 synthetic cases with balanced benign alternatives, missing/stale/conflicting evidence, and cross-domain causal chains.
- Tier C: local synthetic event stores at increasing sizes (for example 10k then 100k records), with randomized time shifts and pseudonymized entities, read-only query budgets, and no solver-visible attack labels.
- Tier D: separately held-out, independently reviewed evaluation data, with preregistered rules and exact per-event ground truth. Never score on the public oracle set.

At Tier C/D, report event-level recall, precision, false-positive burden, recall by tactic/domain, query count and cost, wall-clock latency, evidence attribution accuracy, duplicate/dropped event handling, and abstention quality. Report macro/per-domain metrics and uncertainty intervals; an aggregate score must not hide a tactic where the system fails almost completely. Include strong deterministic detection baselines and analyst-authored rules. Do not make a superiority claim unless the preregistered comparison supports it.

External research: https://arxiv.org/abs/2604.19533

## Qualification targets (proposed, not results)

- Every frozen invalid, stale, unauthorized, and safety-negative control receives its expected fail-closed or review-required disposition.
- Deterministic replay binds exact challenge bytes, oracle bytes, schemas, evaluator version, baseline, environment, counts, and exclusions.
- No scored case disappears because it is blocked, failed, unscored, or unevaluable.
- Every scored claim points to supporting evidence IDs and distinguishes observation from inference and hypothesis.
- Zero effect dispatch is permitted from a diagnosis, framework label, model confidence, learner completion, credential, or benchmark score alone.
- Compare Symthaea with a strong deterministic/rule-based baseline on predeclared held-out cases; publish per-domain results and uncertainty intervals. If evidence is insufficient or uncertainty overlaps no improvement, report Inconclusive.
- Evaluate learning benefit with altered cases and delayed retests, not only repeated exposure to the same public examples.

## Implementation stages

1. Schema and fixture contract; malformed/unknown-field and solver/oracle separation tests.
2. A small deterministic Rust corpus verifier that hashes exact bytes before parsing, checks duplicate IDs/counts/schema versions, and produces a complete census. It must not score model semantics.
3. A distinct semantic evaluator with explicit rubrics, independently inspectable evidence references, and failure dispositions.
4. Praxis presentation and attempt capture through existing provenance-complete evidence contracts.
5. Symthaea adapter and isolated-range integration only after the contract and independent evaluation boundaries are qualified.
6. Held-out cross-domain trials and later learner studies with preregistered primary outcomes and suitable comparison groups.

A clean schema/verifier pass establishes structure only. It does not prove diagnosis quality, learner efficacy, production security, or range isolation.

## Safety and privacy

All seed cases are synthetic. No production credentials, live-target scanning, operational exploit payloads, real user incident data, or automatic remediation are required. Suggested responses are not effects. Any future lab effect must use its own independently enforced, narrow, current authority and independent observation/reconciliation.

Learner attempt records must preserve assistance/tool-use classification and provenance. Persisting an attempt is not evidence admission, mastery, credential eligibility, trust, or authorization. Public DHT publication must not be used by default for private learner traces.

## External references

- NIST NICE Framework components v2.2.0: https://www.nist.gov/itl/applied-cybersecurity/nice/nice-framework-resource-center/nice-framework-current-versions
- MITRE ATT&CK Enterprise tactics: https://attack.mitre.org/tactics/
- CISA Known Exploited Vulnerabilities Catalog: https://www.cisa.gov/known-exploited-vulnerabilities-catalog
- OWASP Top 10 for LLM Applications: https://genai.owasp.org/llm-top-10/

Reference snapshot identity, source release, applicability, and access date must be included in a future ingestion receipt before an external mapping is used as reproducible benchmark input.