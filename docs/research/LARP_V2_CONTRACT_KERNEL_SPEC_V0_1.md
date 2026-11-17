# LARP v2 — Institutional Contract Kernel
## Research and engineering specification v0.1

**Status:** proposed research contract; documentation and synthetic fixtures only  
**Date:** 2026-11-14  
**Branch:** `research/larp-v2-contract-kernel-v0.1`  
**Scope:** a versioned, typed contract model that composes with LISE and owner-native systems  
**Non-claim:** this document does not define a legal instrument, production runtime, certification, or completed implementation.

## 1. Executive decision

Revive the useful ideas in the historical `.larp` pacts, but do not revive their original execution assumptions.

LARP v2 is a **declarative institutional contract layer**. It describes what an institution has committed to, what rights and obligations attach to that commitment, which evidence can establish performance, how changes and challenges are handled, and what remedies are available. It does not confer authority on its author, decide moral questions, execute payments, or certify its own outcomes.

LISE supplies the independent assurance of authority, effective control, dependencies, execution and institutional continuity. Existing owner systems remain authoritative for credentials, voting, funds, evidence custody, provider execution and local law.

The historical repository records the names `EpistemicBounty` and `ProofOfRestoration`, but the inspected sources do not establish a canonical expansion of the acronym LARP. This specification retains LARP as the project identifier and intentionally does not invent an expansion.

## 2. The paradigm shift

Move from **governance rules that say what should happen** to **institutional commitments whose authority, execution, effects and remedies can be independently reconstructed**.

The core chain is:

```text
Contract intent
  → local constitutional authority
  → effective-control and conflict analysis
  → typed policy evaluation
  → separately authorized execution
  → owner-observed effect
  → independent evidence assessment
  → contest, remedy, amendment or exit
```

Each arrow is a boundary with its own evidence. A successful policy evaluation is not an execution receipt. An execution receipt is not an observed outcome. An observed outcome is not proof that the institution acted legitimately.

## 3. Non-negotiable invariants

1. **Policy is not authority.** A contract cannot grant its author, compiler, evaluator, AI agent, provider or counterparty new constitutional power.
2. **Authority is local and non-transitive.** A peer credential, external approval or federation membership is evidence to evaluate, never universal authorization.
3. **No self-ratification.** Symthaea may draft, compare, simulate and explain amendments; only the locally authorized process may ratify them.
4. **No self-executing sanctions.** A score, semantic similarity, anomaly, reputation value or model confidence cannot alone justify slashing, expulsion, loss of rights or permanent blacklist.
5. **Unknown is not false and not safe.** Missing route, dependency, revocation, execution or outcome evidence remains indeterminate or held, not silently treated as compliant.
6. **Effects are distinct from intentions.** Proposed, authorized, attempted, dispatched, acknowledged, applied, observed and reconciled are distinct states.
7. **Historical truth is not erased by freshness decay.** Evidence can become stale for a current decision without becoming invalid as a historical record.
8. **Rights are not compensable by efficiency.** A weighted score cannot trade away consent, due process, privacy, exit, non-discrimination or other locally protected rights.
9. **Delegation attenuates.** A child grant cannot broaden its parent along any supported dimension.
10. **Recovery does not create a sovereign administrator.** Service restoration and authority restoration are separate transitions.
11. **Remedy is part of the contract.** Every high-impact obligation identifies how a participant can challenge a decision and obtain an accountable response.
12. **Local institutions retain their constitutions.** Interoperability transports evidence and bounded requests; it does not merge jurisdictions.

## 4. Contract object

The v2 contract is a versioned, immutable declaration. Amendments create a new version and an explicit lifecycle event; they do not rewrite the prior contract.

A contract must represent at least:

| Domain | Required content |
|---|---|
| Identity | Stable contract ID, version, canonical digest, predecessor/supersedes links |
| Jurisdiction | Local institution, jurisdiction, scope and explicit exclusions |
| Constitution | Authority reference, exact epoch and constitutional digest |
| Parties | Principals, affected participants, counterparties and responsible stewards |
| Purpose | Specific permitted purpose and prohibited secondary uses |
| Rights | Rights floor, consent basis, privacy/disclosure limits, exit and appeal |
| Obligations | Actor, action, recipient, timing, conditions and measurable completion evidence |
| Authority | Who may propose, ratify, delegate, execute, suspend, amend and remedy |
| Constraints | Resource, amount, frequency, operation count, duration and scope bounds |
| Evidence | Admissible evidence types, source owners, provenance, freshness, uncertainty and independence requirements |
| Outcomes | Intended outcome, counterfactual or baseline, measurement window, externalities and adverse outcomes |
| Finance | Funding source, ceiling, escrow/streaming terms, release conditions and reconciliation owner |
| Disputes | Notice, disclosure, challenge window, independent review, appeal and conflict handling |
| Remedies | Pause, correction, restitution, compensation, capability restriction, reauthorization or termination |
| Lifecycle | Activation, renewal, suspension, expiry, amendment, revocation, exit and dissolution |
| Dependencies | Known providers, common-cause dimensions, evidence owners and inventory-completeness state |

Missing required fields do not acquire permissive defaults. The compiler reports an explicit validation error or unsupported/indeterminate condition.

## 5. Typed decision result

The evaluator is a pure decision function over a pinned contract version and explicit owner-resolved inputs. It returns one of:

- `permit`: every required local-authority, rights, scope, evidence and precondition check is satisfied;
- `deny`: a known constraint or right prohibits the requested transition;
- `hold`: a required fact is stale, not yet observed, or awaiting reconciliation;
- `indeterminate`: the inventory, semantics or source evidence is incomplete or unsupported;
- `challenge_required`: the contract's dispute or independent-review process must run before a high-impact transition.

Every result includes stable reason codes, the contract digest, constitutional epoch/digest, policy/adapter versions, input evidence digests, dependency report, effective/knowledge time, and the exact action scope evaluated.

The evaluator does not call a provider, write a journal, transfer value, penalize a person or mutate a constitution. The owner-native execution system must independently re-check authority and preconditions at the protected effect boundary.

## 6. Contract lifecycle and effect lifecycle

### 6.1 Contract lifecycle

```text
Draft
  → StructurallyValidated
  → Proposed
  → LocallyRatified
  → Active
  → (Challenged | Suspended | Amended | Expired | Terminated)
  → ArchivedWithHistory
```

Each transition is an append-only event bound to the exact predecessor digest, local authority epoch, actor, action, purpose and receipt. A proposal or simulation is never an activation.

### 6.2 Execution lifecycle

```text
Planned
  → Authorized
  → Dispatched
  → ProviderAcknowledged
  → Applied
  → IndependentlyObserved
  → Reconciled
```

Additional terminal/intermediate conditions include `PartiallyApplied`, `UnknownOutcome`, `Rejected`, `CancelledBeforeDispatch` and `CompensatedByNewAction`.

No timeout is interpreted as proof of failure or success. Unknown outcomes require reconciliation before retry unless the exact owner/provider contract establishes idempotent replay safety. Compensation is a new governed action, not deletion of an already-observed effect.

## 7. Evidence contract

Evidence is typed and owner-qualified. A minimum evidence record binds:

- artifact/claim identity and content digest;
- issuer, collector, observer and source owner;
- measurement or review method and version;
- signature/attestation and verification result;
- effective time, observation time, receipt time and knowledge time;
- uncertainty, calibration and known limitations;
- purpose, consent, disclosure and retention constraints;
- revocation/compromise state and its effective/observed times;
- lineage to upstream data, tools, models, operators and providers;
- declared independence relationships and inventory completeness.

The system preserves both **as-known-at-the-time** and **current-as-of-now** views. A later revocation or correction changes current applicability but must not rewrite the earlier evidence of what was known and acted upon.

Freshness is domain-specific. Flood observations, land cover, soil moisture, reviewer expertise, credentials and constitutional approvals have different validity conditions. Avoid one universal exponential decay formula and never silently lower an ordinal truth tier merely because time passed.

## 8. Rights, challenge and remedies

Every high-impact contract must define:

1. who receives notice and in what accessible form;
2. what evidence and reasons are disclosed, subject to legitimate privacy limits;
3. how a participant can contest a claim, measurement, authority chain or remedy;
4. an independent or conflict-screened review route;
5. appeal or escalation where the first reviewer is implicated;
6. response and remedy deadlines;
7. how compensation, correction, restoration or reinstatement is recorded;
8. how the record preserves the original decision and later challenge history.

An anomaly or collusion signal opens an inquiry; it is not a verdict. Severe remedies require evidence, notice, an authorized decision-maker, reasons, proportionality and a meaningful challenge route. Emergency temporary holds may protect funds or safety, but must be narrow, time-bounded, reviewable and unable to become permanent punishment by inaction.

## 9. Federation and local sovereignty

A federated contract carries a verifiable claim and a bounded request, not foreign authority.

The receiving institution independently resolves:
- issuer and credential status;
- local membership and role;
- purpose compatibility;
- rights and disclosure constraints;
- resource/action limits;
- constitutional and policy compatibility;
- revocation freshness;
- remedy, dispute and exit routes.

A valid partner credential is not a local authorization. A partner cannot amend another institution's constitution, appoint its stewards, direct its treasury or waive its participants' rights. Refusal and exit must be meaningful and must not silently destroy historical evidence or accrued remedies.

## 10. Dependency and independence semantics

Contract evidence must distinguish:
- different people from independent organizations;
- independent organizations from independent control;
- different credentials from independent issuers;
- different auditors from independent data, tools or infrastructure;
- multiple observations from genuinely independent measurements.

Known shared dependencies remain represented even when the rest of the dependency graph is incomplete. An incomplete inventory is not evidence of independence. The contract may require an independence threshold, but LISE's dependency assurance remains the owner for common-cause and cut-set semantics.

## 11. Recovery and institutional continuity

Continuity contracts separately define:

- essential services allowed to operate in degraded mode;
- high-impact capabilities that must remain paused;
- who can attest to loss, compromise or unavailability;
- recovery quorum and organizational diversity;
- successor authority and exact new epoch;
- stale credential, key and session rejection;
- evidence reconstruction and reconciliation;
- emergency authority scope, expiry and post-event review;
- exit, dissolution and distribution of remaining obligations.

Restoring a database, key, service or operator identity never automatically restores their former authority. A recovered service is not proof of recovered legitimacy.

## 12. Compiler and runtime boundary

The first implementation should be a small Rust-first, deterministic contract kernel. Do not build a general-purpose scripting language or replace mature authorization engines.

Proposed stages:

1. Parse a constrained, versioned contract representation (initially JSON; a readable DSL can follow).
2. Validate schema, required fields, identifiers, references, lifecycle transitions and canonicalization.
3. Type-check actors, purposes, resources, bounds, rights, evidence and time semantics.
4. Perform static checks for non-attenuating delegation, authority self-grant, missing remedy, stale constitution, implicit defaults and unsupported effects.
5. Lower the authorization subset to owner-native adapters (Cedar, OPA/Rego or existing service policy).
6. Evaluate against owner-resolved inputs and frozen evidence snapshots.
7. Emit deterministic decision evidence; do not perform the protected action.
8. Pass an authorized, exact-operation request to the owner execution service, which performs its own currentness and effect-boundary checks.
9. Record receipts, independent observations, reconciliation and challenge events through the owning systems.

The contract kernel must expose stable reason codes and cross-language golden vectors. It should be deterministic under a pinned contract, compiler, policy adapter, clock input and evidence snapshot. Time is an explicit input, not an implicit call to wall-clock time.

## 13. Composition with industry standards and systems

LARP should compose with established components rather than compete with each at its own specialty.

| Existing system or reference | Established strength | LARP/LISE relationship |
|---|---|---|
| Cedar | Typed authorization policies, schema validation and analyzable permit/forbid rules | Delegate the authorization subset; add contract obligations, lifecycle, evidence and remedy |
| OPA/Rego | General policy-as-code, structured-data evaluation, tests and decision logs | Adapter for existing environments; do not treat logs as complete effect evidence |
| Zanzibar-style systems | Scalable relationship-based authorization and consistency semantics | Use as an authorization data source; add exact provider-effect and institutional legitimacy evidence |
| OMG DMN / BPMN | Decision models, decision tables and process coordination | Import/export or map decision logic; do not invent a competing process notation |
| Decidim | Participatory processes, assemblies, initiatives and community discussion | Integrate participation workflows; LISE tests power/authority and remedy boundaries |
| OpenFisca | Executable policy and microsimulation of tax/benefit rules | Potential adapter for distributional and counterfactual policy analysis |
| W3C Verifiable Credentials 2.0 | Interoperable, tamper-evident issuer-holder-verifier credential model | Carry scoped evidence; credential validity never implies universal authority |
| ISO 37000 / ISO 37301 | Organization governance guidance and compliance management systems | Map institutional commitments and audits to contract/evidence controls; do not claim certification |
| NIST NICE Framework | Common language for cybersecurity tasks, knowledge, skills, work roles and competency areas | Map human responsibility and capability requirements, not institutional authority |
| Google HEART | User-experience measurement categories | Adapt as a participant-outcome lens, not a governance or security control |

Reference links:
- Cedar: https://docs.cedarpolicy.com/policies/validation.html
- OPA policy language: https://www.openpolicyagent.org/docs/policy-language
- OPA decision logs: https://www.openpolicyagent.org/docs/management-decision-logs
- OMG DMN: https://www.omg.org/spec/DMN
- Decidim: https://decidim.org/features
- OpenFisca: https://openfisca.org/doc/
- W3C VC Data Model 2.0: https://www.w3.org/TR/vc-data-model/
- ISO 37000: https://www.iso.org/standard/65036.html
- ISO 37301: https://www.iso.org/standard/75080.html
- NIST NICE: https://www.nist.gov/itl/applied-cybersecurity/nice/nice-framework-resource-center/about
- Google HEART paper: https://research.google/pubs/measuring-the-user-experience-on-a-large-scale-user-centered-metrics-for-web-applications/

### 13.1 HEART: useful, but not an authority mechanism

The Google HEART framework means **Happiness, Engagement, Adoption, Retention and Task success**. Use it only as a measurement framework for participant experience and service quality.

For civic or institutional contexts:
- Happiness: perceived dignity, procedural fairness, trust and felt safety;
- Engagement: meaningful and voluntary participation, not raw clicks;
- Adoption: who can access and use the service, including underserved groups;
- Retention: continuity where beneficial, paired with frictionless exit and freedom not to participate;
- Task success: whether participants can complete the service they came to use.

Add counter-metrics for burden, coercion, exclusion, complaint suppression, exit friction, unequal access and unwanted persistence. High retention can mean lock-in; high engagement can mean conflict or compulsory use. HEART must never become a single institutional-quality score or outweigh rights.

### 13.2 NICE: useful for role and capability mapping, not authorization

NIST's NICE Framework describes cybersecurity work through Task, Knowledge and Skill statements, competency areas and work roles. It can help define what a security reviewer, authority operator, recovery steward or independent auditor must be able to do.

Use it to:
- specify capability prerequisites for human duties;
- identify unstaffed or single-person-dependent critical roles;
- separate proposing, approving, executing and auditing duties;
- create transparent training and evidence pathways;
- measure whether institutions can maintain required skills over time.

Do **not** turn a NICE work-role label, certificate or skills profile into a blanket grant of institutional power. Competence evidence is not local constitutional authority. The local institution still appoints, scopes, supervises and can revoke the relevant capability.

### 13.3 HEARTH is distinct from HEART

The Mycelix repository also has a **Hearth** domain cluster and related living/community functions. That is a separate application/domain concept; do not silently rename it HEART or treat it as the Google HEART metric framework. Hearth may provide a participant-facing integration surface for care, kinship and community processes where its own scope and readiness permit.

## 14. Benchmark programme: LARP-CONTRACTS-001

The first conformance seed is in:
- `LARP_V2_CONFORMANCE_SCENARIOS_V0_1.json` — input scenarios only;
- `LARP_V2_CONFORMANCE_ORACLE_V0_1.json` — separately maintained expected dispositions and reason codes.

Keep fixtures and oracle separate so that the evaluator cannot generate its own expected answers. Freeze a holdout subset before tuning. Every report must identify suite revision, scenario digest, oracle digest, adapter versions, compiler version, hardware and repeat count.

### 14.1 Required comparator arms

1. LISE/LARP Rust reference evaluator.
2. Cedar for the authorization subset, with schema/policy validation enabled.
3. OPA/Rego for the equivalent authorization subset, with tests and pinned policy bundles.
4. A Zanzibar-style or OpenFGA relationship adapter where the scenario semantics fit.
5. Existing owner-native Mycelix approval/authority/execute flows.
6. Manual reconstruction by at least two independent reviewers.

No arm is expected to implement all others' responsibilities. Unsupported semantics are reported, not scored as a win or loss. Compare equivalent decision obligations and preserve each product's documented guarantees.

### 14.2 Non-compensatory metrics

- unauthorized protected effects;
- unauthorized permits and false denials;
- rights-floor and consent violations;
- self-grant and authority-transitivity acceptance;
- delegation amplification by dimension;
- stale-epoch / stale-evidence acceptance;
- unknown/partial outcome classification accuracy;
- blind retry and duplicate-effect count;
- dependency-independence false claims;
- challenge access, remedy timeliness and successful correction;
- completeness of evidence and contract reconstruction;
- participant burden, exclusion and exit friction.

Runtime, throughput, memory, compile time, adapter calls and operator effort are reported separately. Fast incorrect authorization is a failure.

### 14.3 Prospective targets, not achieved results

- 100% correct disposition on the frozen finite known-answer panel;
- zero unauthorized effects and zero false permits in high-severity fixtures;
- 100% rejection of peer-only authority and non-attenuating delegation fixtures;
- 100% hold/indeterminate classification for unknown or incomplete protected facts;
- zero automatic severe sanctions from similarity, anomaly or model scores alone;
- 100% preservation of historical evidence while current applicability is recomputed;
- 100% exact constitutional-digest binding for ratification and effectuation;
- 100% explicit partial/unknown states, with no blind retry in unsupported provider cases;
- deterministic replay of every fixture under pinned inputs;
- no weighted overall score that can hide a rights or authority failure.

These are finite-suite targets, not universal guarantees. Superiority claims require at least three pinned comparator arms, independent adjudication, repeated trials, confidence intervals, full counterexamples and a declared equivalence boundary.

## 15. Benchmark construction and research discipline

The initial scenario panel is synthetic and deliberately small. Expand only when:
- a new attack or legitimate positive case is distinct;
- expected outcomes are independently adjudicated;
- preconditions and effect semantics are explicit;
- the oracle can be tested independently;
- the case is not duplicated under a different label.

Required methods for the executable phase:
- strict JSON schema and canonical digest validation;
- parser/compiler golden vectors;
- property-based tests for scope attenuation and lifecycle ordering;
- mutation tests for dropped authority, rights and remedy checks;
- differential tests against an independently implemented simple oracle;
- model-based tests for amendments, revocation, recovery and disputes;
- crash/replay tests for dispatch, partial completion and unknown outcome;
- matched Cedar/OPA authorization comparisons;
- two independent adjudicators for rights-sensitive outcomes;
- held-out cases and publicly reported denominator/counterexample data.

Do not benchmark only successful cases. Include benign agreement, legitimate refusal, legitimate exit, ambiguous authority, incomplete inventories and adversarial cases.

## 16. HEART and NICE measurement integration

HEART and NICE are supporting frameworks, not core policy engines.

For HEART, report category-level metrics plus counter-metrics; do not maximize retention or engagement unconditionally. Survey and behavioral measures must be voluntary, purpose-limited, privacy-preserving and disaggregated where lawful and safe.

For NICE, keep a versioned crosswalk from contract duties to TKS statements / competencies / work roles. Report:
- duty coverage by capability;
- critical duties with only one qualified person;
- separation-of-duty conflicts;
- time to qualify a successor;
- training and reassessment burden.

A workforce framework must not become a social-credit system or a mechanism for denying basic rights.

## 17. Integration sequence

1. **Specification and fixtures (this increment).** Review the contract vocabulary, the HEART/NICE boundaries, and the independent input/oracle panel.
2. **Typed Rust model.** Add contract IDs, versions, authority/constitution references, rights, obligation, evidence and lifecycle types. Avoid executing embedded code.
3. **Validator and evaluator.** Build deterministic structural/type validation and the five-valued decision result.
4. **LISE adapter.** Consume owner-native constitutional, S2 authorization, S3 capture, dependency, S4 federation and S5 continuity evidence. Do not copy their state into a competing authority store.
5. **Cedar/OPA adapters.** Translate only the authorization subset; preserve policy IDs, diagnostics, error/unknown distinctions and version hashes.
6. **First vertical slice — Epistemic Bounty.** Rebuild as evidence/reward/appeal contract. Similarity and cartel signals trigger review, not automatic punishment. Treasury performs separately authorized transfers.
7. **Second vertical slice — Proof of Restoration.** Use independently sourced ecological observations, calibrated uncertainty, watershed externalities, baseline/counterfactual and staged compensation. The contract pauses releases only under explicit, reviewable conditions.
8. **Participant and workforce pilots.** Add HEART outcome measures and NICE role/capability mapping only after the basic contract and rights paths work.
9. **Independent evaluation.** Freeze holdout cases, run matched adapters, invite two independent reviewers and publish limitations.

## 18. Acceptance boundary for this specification PR

This documentation increment is acceptable only as a research/design proposal when:
- fixture JSON parses;
- all scenario IDs are unique and have exactly one oracle row;
- all oracle rows reference existing scenarios;
- expected outcomes use the declared disposition vocabulary;
- no fixture assumes an implementation that has not been built;
- the scope and non-claims remain explicit.

This PR does **not** claim that LARP has a parser, compiler, runtime, authorization enforcement, payment integration, independent ecological oracle, HEART instrumentation, NICE credential integration, or passed CI. Those are future engineering increments.

## 19. Relationship to existing LISE work

LARP v2 is not a substitute for existing LISE increments. It consumes and composes with:
- IPB-S2 authorization and exact-operation constraints;
- IPB-S3 adversarial capture and effective-control analysis;
- IPB-S4 sovereignty-preserving federation;
- IPB-S5 authority-preserving continuity;
- owner-native dependency assurance and known common-cause overlap;
- Institutional Authorization Assurance Profile (IAAP);
- IPB-FINALITY and owner-native execution/reconciliation semantics.

Where an owner implementation is absent, LARP must report the property as specified, structurally checked, adversarially exercised, independently observed or externally replicated at the appropriate scope. It must never upgrade an evidence claim by inference.
