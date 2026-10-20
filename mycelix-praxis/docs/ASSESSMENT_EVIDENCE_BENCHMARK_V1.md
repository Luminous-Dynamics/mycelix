# Praxis Assessment Evidence Benchmark v1 (research draft)

## Purpose

This contribution adds a typed Rust contract for checking whether a learning-study dataset is structurally eligible for analysis. It is intentionally narrower than a mastery algorithm. A valid record sequence does not prove competence, validate an evaluator's judgment, establish a causal effect, issue a credential, or show that Praxis beats another product.

The contract, language-neutral schema, and synthetic comparison fixture are:

- `crates/praxis-core/src/assessment_evidence.rs` — executable semantic validator.
- `examples/assessment-benchmarks/v1/assessment_benchmark.schema.json` — JSON Schema 2020-12 wire shape.
- `examples/assessment-benchmarks/v1/ai_scaffold_fading_synthetic.json` — synthetic comparison fixture.

The checked-in fixture contains three **synthetic examples**, one per arm, with 21 total assessment events. These records are only test data for schema and negative-control behavior. They are far too small to estimate learning effects and must never be reported as user results.

## Paradigm shift: evidence about capability, not engagement

The proposed study loop is:

`Baseline → Practice (record help) → Immediate check → Novel transfer → 14-day retention → 30-day retention`

The main outcomes remain separate:
- correct performance while instructional assistance is available;
- immediate unaided performance;
- independent performance on a different task form;
- delayed unaided retention;
- time/attempts to a predeclared retained-competency criterion;
- assistance used and missingness;
- rubric and task form/version.

This addresses a meaningful failure mode: a tool can increase task throughput without increasing durable capability. A 2025 *PNAS* field experiment with nearly 1,000 high-school mathematics students reported that GPT-4 help improved practice performance, yet students using a less learning-protective tutor performed worse than the no-AI comparison when the tool was removed; a hint-oriented design mitigated much of that effect. A separate 2025 randomized study of 120 undergraduates reported lower scores on a surprise 45-day retention test in the ChatGPT-supported group than in the traditional-study group. Both results are bounded by their populations and learning domains; they do not show that all AI assistance is harmful.

Sources:
- Bastani et al., *Generative AI can harm learning*, PNAS (2025): https://doi.org/10.1073/pnas.2422633122
- Barcaui, randomized study of ChatGPT-assisted AI-concept learning and delayed retention (2025): https://doi.org/10.1016/j.ssaho.2025.102287

## What the Rust validator enforces

The validator checks:

1. A recognized schema version, named protocol/rubric, at least two declared study arms, unique arm IDs, and episode-to-arm binding.
2. Unique participant references per episode and globally unique attempt IDs.
3. Non-empty task family, task form, and task-form version; each attempt must match the episode's task family.
4. Integer rubric scores in the range 0-1000, non-negative timestamps, and a consistent rubric version.
5. Explicit assistance classification. Unknown assistance makes the controlled record ineligible; it is never silently interpreted as no help.
6. Unaided baseline, immediate post-test, transfer, and retention phases. Practice can include hints, scaffolding, tools, or direct-answer assistance.
7. Required baseline, practice, transfer, and 14-day retention, with optional protocol-required 30-day retention.
8. Distinct task forms for assessment anchors and no reuse of a practice form as an anchor.
9. Phase ordering and minimum elapsed time from final practice to delayed retention.
10. Presence of at least one episode in each declared arm.
11. JSON Schema parity with the serialized root/arm/episode/attempt field sets and Rust phase/assistance enums.

The output is a structural summary and stable validation findings. It contains no global learner score and makes no mastery or causal claim.

### Deliberate limits

- A form ID being different does not prove two forms are psychometrically equivalent. Equivalent difficulty and content coverage require empirical item/form evaluation.
- A rubric version string does not prove the rubric is valid or raters apply it consistently.
- A pseudonymous participant ID can still be linkable. This synthetic fixture contains no real learner records; production identity mapping requires separate controls and consent.
- The current contract does not prescribe randomization, sample size, statistical analysis, accessibility evaluation, or differential privacy. Those must be specified in a study protocol before recruiting participants.
- The schema treats unaided as absence of instructional help. Any ordinary instruments allowed during a task must be standardized and documented in the protocol; the word does not mean every real-world tool must be unavailable.
- Passing validation only says the data satisfy this contract.

## Comparative benchmark design

For AI-assisted instruction, compare at least three policies on equivalent skills and reasonably comparable task difficulty/time:

- **Active retrieval baseline:** deliberate practice, retrieval, and feedback without AI answer generation.
- **Hint-first AI with scaffold fading:** learner attempts first, gets bounded hints and feedback, and gradually receives less support.
- **Direct-answer AI:** an informative comparator for understanding the cost/benefit of answer disclosure.

Use independent assessors and novel task forms. Predeclare exclusions, missing-data handling, analysis choices, and primary outcomes. Randomize assignment or counterbalance task order when feasible and ethically appropriate. Do not reuse the exact practice items for transfer or delayed checks.

## Proposed target ladder (hypotheses, not results)

| Gate | Measure | Proposed acceptance target |
|---|---|---|
| Contract correctness | Frozen positive and adversarial fixtures | 100% of included deterministic fixtures produce their expected structural disposition |
| Independence integrity | Tool/help leakage into unaided assessments | Zero accepted transfer/retention records with instructional assistance recorded |
| Task novelty | Practice form reused as transfer/retention anchor | Zero |
| Retention integrity | Check submitted before configured delay | Zero accepted early retention records |
| Assessment quality | Rubric clarity and independent scoring | Report weighted agreement/ICC appropriate to the rubric; set a justified threshold before consequential use |
| Learning efficacy | Novel transfer and delayed retention vs. active baseline | Initial hypothesis: ≥0.20 SD transfer improvement, or ≥15% less time to retained criterion without material retention/access regression |
| Replication | Independent cohort/site and assessor | Reproduce direction and magnitude with confidence intervals; report null and negative results |

The numerical efficacy targets are aspirational hypotheses. They are not validated universal thresholds, achievements, or grounds for claiming state of the art. A sample-size/power analysis must precede a real study. Time-to-criterion requires a predeclared criterion and treatment of non-completion; do not compare only learners who finish.

## Industry comparison and standards fit

Praxis should complement, not replace, existing standards and learning platforms:

- **1EdTech Open Badges 3.0** describes portable verifiable achievement claims with issuer, criteria, and possible supporting evidence, and has a formal conformance/certification process. Praxis should retain interoperable credential exports rather than inventing a closed badge.
  - Overview: https://www.1edtech.org/standards/open-badges
  - Conformance guide: https://standards.1edtech.org/open-badges/specifications/standards/v3p0/cert
- **W3C Verifiable Credentials Data Model 2.0** is a W3C Recommendation published 15 May 2025. It supplies a cryptographically verifiable, privacy-aware data model for issuer/holder/verifier exchanges; it does not by itself prove that the asserted skill was assessed well.
  - Specification: https://www.w3.org/TR/vc-data-model/
- **Khan Academy's mastery system** is an established example of skill-level practice and mastery tracking. Praxis should not claim differentiation merely for offering exercises or progress states; it needs comparative evidence for cross-domain transfer, delayed retention, evidence export, and learner agency.
  - Overview: https://support.khanacademy.org/hc/en-us/articles/360007253831-What-is-self-paced-Mastery
- **Universal Design for Learning 3.0** emphasizes access, choice, varied means of engagement/representation/action and learner agency. Accessibility should be built into the task protocol and analyzed as part of outcomes, not measured only as an afterthought.
  - Guidelines: https://udlguidelines.cast.org/

A defensible product distinction would be **portable, inspectable evidence about durable capability**, while remaining compatible with the established credential ecosystem. It must be demonstrated by conformance results, user research, and independent comparative studies rather than feature claims.

## Running the contract

From the Praxis workspace root:

```sh
cargo test --manifest-path mycelix-praxis/Cargo.toml -p praxis-core assessment_evidence::tests
```

The tests use the checked-in fixture, verify JSON Schema parity, and include negative controls for:
- assisted transfer;
- practice/transfer form reuse;
- retention before 14 days, even with a perfect score;
- unknown assistance;
- mismatched rubric versions;
- scores above 1000;
- duplicate attempt IDs;
- undeclared study arms.

This command has **not** been executed in the current authoring environment. A passing remote check on the exact final commit is still required.

## Next steps

1. Run the focused Rust tests and workspace checks on the exact PR head.
2. Add schema-versioned fixtures for missing phases, malformed timestamps, duplicate anchors, unregistered arms, and unknown fields.
3. Define form-equivalence evidence and rater calibration before field studies.
4. Preregister an appropriately powered comparison and independently review task materials.
5. Add Open Badges / VC export only after agreement on the actual claim and evidence semantics; do not convert this validator's summary into a credential automatically.
