# Wish-to-Outcome Protocol v0.1 (Draft)

**Status:** research draft. Not an execution authorization, production interface, safety certification, or claim that any wish has been fulfilled.

## 1. Purpose

Wish-to-Outcome (W2O) is a person-owned workflow for translating an expressed goal into feasible, consent-bounded work with explicit resource commitments and verifiable outcome states. “Wish-granting” is shorthand for the ambition; a wish is not a promise that the system can deliver it.

The protocol should improve a person's agency by reducing planning and coordination friction without allowing an AI to decide what a person ought to want, invent available resources, or silently cross an authority boundary.

## 2. System boundaries

- **Symthaea** proposes candidate interpretations and plans, surfaces uncertainty and requests only permitted evidence within declared budgets. It does not decide consent or declare real-world fulfillment.
- **Symtropy** tests candidate task graphs, dependencies, capacity and schedules in deterministic, effects-disabled simulations. A feasible simulation is not an execution receipt.
- **Mycelix** records consent references, voluntary resource offers and commitments, coordination/operation receipts, disputes, and outcome state transitions. A signed attestation is evidence of who asserted something, not automatically proof that the assertion is true.
- **The requester** retains control of the goal, acceptable trade-offs, visibility, consent and cancellation. Other participants control whether to offer resources or participate.
- **An authorized execution boundary** separately approves any real external action or expenditure. A Wish Request object never grants this authority.

These are proposed ownership boundaries for integration, not claims that the full integrated behavior exists today.

## 3. Core model: separate intent, plan, commitment, action and outcome

W2O treats at least five things as different artifacts:

1. **Wish Request:** what the requester wants, the success criteria they choose, constraints, visibility and consent scope.
2. **Candidate Plan:** proposed tasks, assumptions, estimated cost/time, unknowns and required resources. This is a proposal, not a commitment.
3. **Resource Offer / Commitment:** a provider's voluntary, scoped, time-bounded statement of availability. A suggestion or search result is not a commitment.
4. **Action / Delivery Receipt:** evidence that an authorized operation was attempted and what it returned. Timeout or lost acknowledgement may mean *outcome unknown*, not success or failure.
5. **Outcome Assessment:** recipient feedback and, where appropriate, independent evidence of delivery against the original success criteria. A plan, attestation or action receipt alone does not prove that the desired outcome was achieved.

The initial JSON Schema in `schemas/wish-request-v0.1.schema.json` validates only the Wish Request envelope. Separate versioned contracts should be defined for plans, resource commitments, action receipts and outcome assessments rather than overloading this request schema.

## 4. Lifecycle and honest dispositions

A reference lifecycle is:

`draft → consented → planning → awaiting_resources → ready_for_authorization → authorized → in_progress → outcome_pending_verification`

Branches and terminal outcomes include `blocked`, `declined`, `withdrawn`, `cancelled`, `expired`, `partially_fulfilled`, `fulfilled`, `outcome_unknown` and `disputed`. Implementations may add states, but must define valid transitions and preserve old evidence. They must not skip authorization checks or rewrite past failed attempts as successes.

The outcome dimension is separate from workflow progress. A request can be operationally complete but not verified against the requester's success criteria. `fulfilled` should only be used with the declared evidence threshold met; a claim without adequate evidence remains `outcome_pending_verification` or `outcome_unknown`. The requester must be able to contest an outcome status.

## 5. Consent, privacy and authority

- Default visibility is private. Public discovery should expose only a requester-approved summary.
- Collect the minimum data required for a specific plan. A person's need is not public reputation data and must not automatically affect eligibility for unrelated services.
- Consent records must be scoped, time-bounded, attributable to an authorized actor, and revocable. Consent to plan is not consent to publish, contact third parties, spend money, execute an operation, or retain personal data indefinitely.
- A consent revocation prevents new actions in the revoked scope. In-flight operations must enter a truthful reconciliation/cancellation path; revocation does not make an already-sent operation disappear.
- The declared budget is a cap requested by the person, not spending authority. Execution requires a separate authoritative authorization check at the action boundary.
- High-impact domains (including medical, legal, emergency, credit, housing-eligibility and comparable consequential decisions) are out of scope for the first pilot and require their own governance, professional review and legal analysis before any proposed use.
- Provide accessible cancellation, status correction, complaint and escalation paths.

## 6. Resource and evidence semantics

Represent proposed, offered, confirmed, reserved, delivered, expired, withdrawn and unavailable resources distinctly. Confirmed status must refer to a provider-controlled commitment or authoritative inventory receipt. Never promote model-generated text or synthetic fixture inventory into real-world availability.

Every transition should emit an append-only, versioned event with timestamp, actor/producer, input/output references, policy/build version where applicable, decision reason, and integrity digest. Keep sensitive evidence access-controlled; auditability does not imply public disclosure. W3C PROV is a useful provenance reference, but a compatible representation may be chosen without requiring a particular serialization.

For consequential statuses, distinguish:
- requester feedback (did this help, as defined by the requester?);
- operational evidence (was the deliverable/service actually supplied?);
- independent assessment (is independent verification appropriate and available?).

These are complementary measures, not interchangeable substitutes.

## 7. Benchmark design

The Mycelix WISH-001 issue defines the full evaluation work. The first harness should be deterministic and Rust-first, with candidate policy input isolated from evaluator-only ground truth. At least 100 fixed synthetic episodes should cover at least 10 stress/failure strata, with corpus and build digests, a complete run census and explicit dispositions for every failed or missing run.

Fair comparator arms should include a manual/rule baseline, recommendation-only AI, deterministic resource matching, relevant component ablations and the composed system. Apply equal initial observations, permitted tools, action authority, budgets and horizons. Include negative controls for fabricated resources, unauthorized actions, forced action, unpriced abstention, replayed operations, silent request dropping and false fulfillment.

Score separately:
- verified outcome / partial / no outcome rate, with unserved and declined requests retained in the denominator;
- requester-defined usefulness and agency;
- total cost per verified outcome, including human review, coordination, integration, recovery and failed work;
- time to outcome, delay and unmet demand;
- resource commitment-to-realization rate;
- false-fulfillment rate, unauthorized action/spend, budget violations and consent-scope violations;
- consistency across repeated trials, fault recovery, accessibility, privacy exposure and distributional differences.

Do not optimize only the count of “wishes granted” or acceptance rate; either can reward cherry-picking easy requests or suppressing refusals. Report a dashboard of outcomes and trade-offs instead of a single welfare score.

## 8. Proposed v0.1 contract targets (not measured results)

- Invalid schemas, duplicate identifiers and missing required provenance/authority fields fail closed.
- Zero unauthorized actions or expenditures in the qualification fixtures.
- Zero unconfirmed/synthetic-only resources reported as real commitments.
- Zero blocked, unknown, partial or unverified outcomes promoted to fulfilled.
- 100% of requests, including unserved, declined, withdrawn and failed cases, are represented in the final census.
- 100% of executed operations reference current authorization and a canonical operation/outcome receipt.
- Identical canonical fixture bytes, binary, configuration and seed produce identical reports.
- At least 95% exact expected-state agreement on preregistered deterministic contract fixtures; publish every disagreement and each stratum's result.

These are proposed engineering gates. They are not measurements, guarantees about real people, or evidence of real-world effectiveness. The initial schema alone cannot satisfy them.

For real-world comparative claims, first establish baselines with an opt-in pilot, recipient-controlled success criteria, independent review where appropriate, participant complaint/withdrawal mechanisms, uncertainty intervals and explicit accounting for every enrolled request. Do not claim impact from synthetic episodes alone.

## 9. Research references

- OpenAI, GDPval: real-world economically valuable work tasks — https://openai.com/index/gdpval/
- Yao et al., τ-bench: tool-agent-user interaction and repeated-trial reliability — https://arxiv.org/abs/2406.12045
- W3C PROV overview: provenance, reproducibility and versioning — https://www.w3.org/TR/prov-overview/
- NIST AI Risk Management Framework — https://www.nist.gov/itl/ai-risk-management-framework
- UNICEF HOPE Annual Report 2025: a relevant precedent for accountable assistance delivery and recipient-data controls — https://www.unicef.org/hope-hct/reports/hope-annual-report-2025

These references inform design; they do not endorse, certify or validate this implementation.

## 10. Current evidence boundary

This document and its schema define a research contract only. No runtime implementation, production integration, real-user pilot, benchmark execution, or CI result is implied. Synthetic evidence must be labelled synthetic; queued or skipped checks are not passes.


## 11. Benchmark research update (2026-10-16)

W2O should compose with existing agent benchmarks instead of relabeling them as proof of wish fulfillment:

- **AgencyBench (ACL 2026)** covers 138 long-horizon tasks across 32 scenarios, using deliverables and rubric-based sandbox checks. It is a useful source of ideas for multi-step task breakdown, feedback loops and functional evidence, but its software/research/game tasks do not establish recipient-defined real-world benefit. https://aclanthology.org/2026.acl-long.337/
- **τ³-bench / current τ-bench release line** covers tool-agent-user interaction and has continued to revise task sets and grading; the published July 2026 release notes say a banking-knowledge grading update changes affected scores and older/newer scores are not directly comparable. W2O should pin dataset, task version, evaluator and tool semantics in every result artifact. https://github.com/sierra-research/tau2-bench
- **JSON Schema Draft 2020-12** is the selected request-contract dialect. The Rust harness should explicitly validate the schema itself, enable date/time format checks, and test acceptance and rejection cases. https://json-schema.org/specification
- **RFC 8785 / JCS** defines deterministic JSON canonicalization suitable for hashing structured receipts; canonical bytes establish integrity of a representation, not truth of the claims inside it. https://www.rfc-editor.org/rfc/rfc8785.html
- **Agentic reward-hacking research** warns that agents may game objectives, evaluation or environment. Keep candidate input separate from evaluator-only truth, preserve negative controls, test invalid/replayed receipts, and do not let the system under test rewrite the scoring inputs. https://link.springer.com/article/10.1007/s44163-026-01980-z

These benchmarks are complementary reference points, not interchangeable baselines. W2O adds a missing distinction: a completed digital artifact, a user's reported benefit and a verified physical/service outcome are separate evidence levels.

### Evaluation ladder

1. **Contract conformance:** schema-meta-validation, format validation, lifecycle constraints, canonical digest checks, duplicate-ID checks, adversarial fixtures and deterministic reports.
2. **Synthetic closed world:** sealed truth, equal budgets, seeded scenarios, baseline/ablation/negative-control arms and a complete census including unserved requests.
3. **Shadow mode:** evaluate proposed plans against human adjudication without external side effects; record disagreements and decision latency.
4. **Opt-in, bounded pilot:** low-risk tasks, explicit consent, human steward, clear cancellation/complaint paths, recipient-defined success criteria and independent checks where feasible.
5. **Comparative impact claim:** only after preregistered baselines, suitable sample-size reasoning, uncertainty intervals, distributional analysis and full failure/cost accounting. The 100-episode/10-stratum contract corpus is a smoke/contract target, not by itself a statistically powered impact study.

Do not optimize a single “wish fulfillment rate” metric. Publish outcome quality, delay, lifecycle cost per verified outcome, resource realization, false-fulfillment, access, agency and all unserved/failed dispositions as a dashboard. A synthetic fixture passing is not evidence that a real person was helped.


### Exact-value rule for budgets

The request schema represents `requested_budget.max_spend` as a non-negative decimal **string**, not a JSON floating-point number (for example `"12.50"`). This avoids silently rounding a money cap through binary floating-point parsing. Before any real commitment or payment, the execution boundary must validate that value against a pinned currency metadata version, reject excess fractional digits, use exact decimal/minor-unit arithmetic, and independently enforce the authorized amount. The schema's string pattern does not by itself establish the currency's permitted precision and does not authorize spending.


### Snapshot and receipt integrity boundary

A versioned Wish Request snapshot is not a mutable global truth record. Any change to goal, consent, visibility, state or evidence references creates a new version and digest; implementations should retain prior snapshots and append a state-transition event rather than silently overwriting history. The request schema validates reference shape and cross-field consistency only. It cannot prove that a plan, authorization, commitment, delivery receipt or outcome assessment exists, is authentic, or supports the stated conclusion; those artifacts require separate fetch, authorization, signature/provenance and policy checks. A production verifier must reject missing, revoked, expired, wrong-subject or wrong-scope references at the point of use.

The fixture corpus includes both a valid evidence-backed fulfilled state and a “verified” state lacking its required assessment/delivery references. This guards against a benchmark that tests refusals without proving that legitimate success paths remain representable.
