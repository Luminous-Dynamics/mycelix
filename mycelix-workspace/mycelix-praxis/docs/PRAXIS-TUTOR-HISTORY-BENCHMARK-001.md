# PRAXIS-TUTOR-HISTORY-001 — History-grounded tutoring benchmark v1

Status: research/implementation prototype; all examples are synthetic. This contract is not a learner record format and does not establish mastery.

## Thesis: score trajectories, not isolated turns

A tutor can answer an individual prompt correctly and still make a poor teaching decision when it misreads the history. Praxis should test whether a system distinguishes recorded attempts from estimates, independent correctness from scaffolded correctness, old from recent evidence, and true progress from context-dependent performance. When evidence cannot distinguish plausible explanations, the right action is a targeted evidence request or abstention—not invented certainty.

This complements (rather than reproduces) LongTutor's three task families—history evidence acquisition, knowledge-state diagnosis and adaptive teaching action—and MMTutorBench's key-step/rubric approach. It also complements Praxis's support-mode decision benchmark and Symthaea adviser contract. No tutor is compared with either paper from this synthetic corpus.

## Artifacts

- `examples/tutor-history/v1/observations.json`: solver-visible, fixed-ID synthetic trajectories only.
- `examples/tutor-history/v1/oracle.json`: evaluation labels and required evidence IDs; keep this input isolated from the system under test.
- Both have strict JSON Schema 2020-12 files.
- `crates/praxis-core/src/tutor_history_benchmark.rs`: strict Rust data types, cross-file invariants, exact-byte BLAKE3 binding, and an outcome scorer.
- The oracle is not an input feature. A test harness must send only observations to the candidate, then score candidate predictions against the separate oracle.

The example set intentionally includes short histories and ambiguous histories, not just clean positive examples. It distinguishes raw observations and imported summaries from derived estimates. No direct learner identifiers, free-text answers, or private learner logs are present.

## Output contract

Each candidate emits one result per case with one of three dispositions:

- `claim`: a bounded diagnosis and low-risk next action, with supporting and contradicting observation IDs;
- `request_more_evidence`: one specific evidence class needed to distinguish plausible explanations;
- `abstain`: no diagnosis/action asserted.

The v1 taxonomy is finite and deliberately not a universal theory of learning. It does not authorize mastery updates, credentials, placement, sanctions, or access decisions. It has no unqualified probability/confidence field.

## Metrics

Report each separately. Do not collapse them into a weighted score that can compensate for safety/evidence failures.

1. **Decision-fit rate**: disposition and, when relevant, diagnosis/action/request match the oracle's accepted labels, before evidence completeness is factored in. **Acceptable-decision rate** requires both decision fit and a grounded claim; a semantically correct diagnosis with missing required citations does not receive full credit.
2. **Supporting-evidence coverage** and **contradicting-evidence coverage**: fraction of oracle-required IDs returned in the correct lists for claims; safe abstentions do not incur citation-omission penalties.
3. **Reference validity**: fraction of citations that point to an observation in that case, separately for support and contradiction. This measures ID integrity, not semantic correctness; required-evidence coverage is reported separately.
4. **Evidence-grounded claim rate**: claims that cite every required supporting and contradicting observation and contain no unsupported references. Safe abstentions and targeted evidence requests are not penalized for omission of claim citations.
5. **Critical violations**: unsupported evidence references, omitted required contradictions in claims, and claims made when the oracle does not permit a claim. Target is zero. **Appropriate uncertainty** separately measures correct requests for missing evidence and abstentions on histories that do not justify a claim; also report unnecessary abstention.
6. **Metamorphic suite (next stage)**: row-order invariance; invariance to irrelevant evidence; sensitivity to changed assistance labels, recency and missing metadata; rejection of corrupted provenance; whole-trajectory train/dev/test splits.

The Rust scorer enforces exact case cardinality and binds predictions to the exact observation JSON bytes. It reports semantic decision fit separately from evidence-grounded acceptable decisions, so the right label with missing required citations does not receive full credit. The digest proves byte identity only; the caller still owns the custody of the bytes and the isolation of the oracle.

## Qualification ladder

- **Level 0 — artifact conformance:** strict parsing, schema/Rust parity, unique cases/events, exact evidence references and digest negative controls.
- **Level 1 — synthetic evaluator conformance:** deterministic oracle replay, metamorphic negative controls, row-order permutations whose semantic scores remain invariant despite different byte digests, and full report reproducibility.
- **Level 2 — expert adjudication:** at least 120 trajectories from at least 8 diagnostic families, double-reviewed independently with disagreement resolution; hold out complete trajectory families, not turns. Report inter-rater agreement and confidence intervals.
- **Level 3 — external benchmark comparison:** map taxonomy carefully to LongTutor/MMTutorBench/TutorBench where constructs overlap; run the actual released benchmark under its rules and pinned model/configuration. A synthetic pass is not comparable to their leaderboard scores.
- **Level 4 — efficacy and non-harm:** preregistered, ethically reviewed study with independent assessment, delayed retention, transfer, assistance exposure, engagement/displacement, and subgroup analysis. Assisted correctness is not an outcome proxy.

Provisional engineering targets (not measured results): zero critical violations; at least 95% acceptable-decision rate on independently adjudicated held-out synthetic/adversarial cases; at least 95% required-evidence coverage; 100% correct disposition on hard abstain/request controls; 100% row-order invariance. These are qualification goals, not a claim that these thresholds alone establish educational efficacy or human parity.

## How to use Symthaea safely

First compare B0 (existing deterministic Praxis evidence/BKT/SRS route) with B1 (same route plus pinned, optional Symthaea advice) on exactly the same trajectory corpus and policy gates. The first useful role is proposing evidence conflicts, plausible hypotheses and the next discriminating observation. Do not map learner proficiency to Phi or interpret generic planner confidence as pedagogical confidence. The independent Praxis evaluator, not Symthaea, scores the candidate; only Praxis's pre-existing authority boundary can admit a support action.

## Research anchors

- LongTutor, ACL 2026: https://aclanthology.org/2026.acl-long.1371/
- MMTutorBench, ACL 2026: https://aclanthology.org/2026.acl-long.1068/
- 2026 field-experiment evidence is heterogeneous: a middle-school mastery-based math study reports improved recovery after mistakes but slower progress/fewer attempts and mixed delayed outcomes: https://www.nber.org/papers/w35621
- A two-year middle-school study of Khanmigo reported modest intent-to-treat math effects and identified engagement as a binding constraint: https://www.nber.org/papers/w35620
- An October 2026 university RCT reported negative outcomes in its studied course-integrated tutor condition; this motivates displacement/non-harm measurement, not a universal claim about AI tutors: https://www.iza.org/publications/dp/18989/the-effects-of-course-integrated-ai-tutoring-on-student-performance-and-engagement-a-randomized-university-trial

## Limits

A passing Rust unit test or synthetic benchmark score validates only its stated proposition. This v1 corpus has no real learner outcomes, external baseline run, expert-reviewed labels, calibration proof, or causal evidence. Do not publish performance or safety claims until the exact artifact digests, evaluator, candidate commit/configuration, and full output report are retained.
