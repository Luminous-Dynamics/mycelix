# Decision Justification Composition Certificate V0.1

Status: research profile and benchmark protocol. No runtime, cryptographic trust bridge, production authorization, or end-to-end qualification is claimed by this document.

Issue: #6821  
Parent/owners: #6783, #6798, #6810, #6817; composes #6792, #6812, #6816, #6818, #6782, #6786, #6787, #6795, #6797 and #2953.

## 1. Research question

Can an independent verifier reconstruct **why one consequential decision was justified at one exact historical knowledge frontier**, while separately determining whether that decision remains justified now, whether a distinct authority authorized its actions, whether execution occurred, and whether the claimed outcome was independently observed?

The target is a composition certificate that binds existing owner-native results. It is not another event store, temporal database, transparency log, policy engine, permission service, executor, or outcome adjudicator.

## 2. The paradigm shift: prove the chain, not a pile of claims

A conventional architecture can store individually valid artifacts while leaving their relationships ambiguous:

\`frontier receipt + inclusion proof + interval index + query result + policy result + action plan + execution transcript\`

DJCC asks whether those exact artifacts compose for one decision:

\`decision + scope + verifier frontier + authenticated source coverage + semantic projection + query + policy + authority + execution + independent outcome\`

The difference is not the number of hashes. It is the obligation to verify every transition and preserve the claim ceiling of every owner.

A valid inclusion proof relative to a supplied root is not authenticated root identity. A witness quorum is not owner-universe closure. An interval-index root is not proof that the source assertions were mapped correctly. A policy decision is not legal authority. An authorized plan is not an execution receipt. An execution acknowledgement is not an independently observed outcome.

## 3. Existing owners and non-duplication

| DJCC stage | Existing owner / work | Composition responsibility |
|---|---|---|
| Verifier frontier | #6679, #6680, #6684, #6795, #6797 | Bind verifier identity, accepted partial-order cut, coverage, ancestry, gaps and forks |
| Source/log trust | #6530, #5863, #5942, #6727, #6810, #6812, #6818 | Reuse SCITT signed-statement/COSE receipt interoperability; bind issuer identity, TS identity, explicit VDS profile, inclusion/consistency, checkpoint/key epoch, governed witnesses and split-view state |
| Semantic projection | #6782, #6786, #6787 | Bind exact assertion payloads, validity-episode identities, correction resolution and interval projection |
| Query result | #6783, #6784, #6788 | Bind query plan, scope, valid time, historical frontier, result and independent replay |
| Policy decision | #5792, #6783 | Bind immutable policy epoch, evaluator build, canonical input, reason codes and historical output |
| Authority/action plan | #6588/#6593/#6596 and #6792/#6819 | Bind current authority profile/epoch and canonical action-set digest/amendment lineage |
| Execution | #6816, #6817, #2953 | Bind attempt identity, one-shot scope, transcript, unknown-outcome barrier and correction impact |
| Outcome | owner-native observation and adjudication contracts; #2953 | Bind independently sourced observation, coverage, contradiction and outcome assessment |

The composition layer may verify adapter mappings and stage-edge digests. It must not take ownership of the underlying facts, histories, authorities, or semantic evaluators.

## 4. Certificate envelope

The machine-readable profile lives at:

\`schemas/economic/decision-justification-composition-certificate-v0.1.schema.json\`

The certificate includes:

- **Identity:** immutable certificate ID, profile/schema version, decision reference and revision.
- **Scope:** subject/resource, purpose, jurisdiction, valid-time scope and canonical scope digest.
- **Assessment vintage:** historical or current; exact verifier identity, accepted frontier digest and policy epoch. A current reassessment links to a prior certificate but never mutates it.
- **Eight typed stages:** frontier, source trust, semantic projection, query, policy, authorization, execution and outcome.
- **Stage producer:** owner/component, version, source commit and build digest.
- **Stage inputs:** named input bindings with exact digest and schema/profile reference.
- **Stage output:** canonical output digest or explicit null when not evaluated/not applicable.
- **Stage claim ceiling:** authentication, completeness, truth, authority, effect and independence are independent fields.
- **Stage dependencies:** exact upstream stage output digests consumed by the stage.
- **Composition result:** profile-qualified disposition and bounded decision claim.

### 4.1 Stage dispositions

Every stage uses a closed disposition vocabulary:

- \`VerifiedForDeclaredClaim\`: the stage's own declared claim passed under its named profile.
- \`Rejected\`: the stage found a contradiction or failed a mandatory invariant.
- \`Indeterminate\`: the evidence is incomplete, conflicting, unavailable, stale or otherwise insufficient to decide.
- \`NotEvaluated\`: no evaluator result exists; it is not a pass.
- \`UnsupportedByProfile\`: this profile cannot make the requested claim.
- \`NotApplicable\`: the stage is genuinely outside this decision profile, with a recorded reason.

The six per-stage claim dimensions—authentication, completeness, truth, authority, effect and independence—use the same explicit state approach. A stage can verify structural receipt integrity while leaving authentication, source completeness and truth as NotEvaluated.

### 4.2 Composition disposition

The composition engine must not collapse all stage outcomes into one boolean. The root disposition is one of:

- \`DecisionJustifiedAtAcceptedFrontier\`
- \`DecisionNotJustifiedAtAcceptedFrontier\`
- \`Indeterminate\`
- \`HistoricalReplayMismatch\`
- \`UnsupportedByProfile\`

A rejected mandatory prerequisite cannot be averaged away. A missing mandatory stage cannot be silently skipped. A stage marked NotApplicable is accepted only where the named composition profile explicitly permits it.

The composition certificate never means "authorized to execute". Execution permission remains with the current owner authority and one-shot effect gate.

## 5. Composition invariants

### 5.1 Exact edge binding

Every consumer input must equal the named producer output digest under the exact schema/profile. Matching IDs, counts, timestamps or human-readable names are insufficient.

The verifier checks:
- each required stage appears exactly once;
- stage identity and producer version are in profile;
- every dependency points to an existing allowed stage;
- the dependency's consumed digest equals the producer's output digest;
- no cycle or undeclared dependency is present;
- no mandatory stage is absent;
- certificate and nested input canonicalization are deterministic.

JSON Schema checks shape, not these graph semantics. A separate composition validator/oracle is mandatory.

### 5.2 No false absence or false completeness

An inclusion proof proves membership under a root, not completeness of a declared universe. An absence claim requires an authenticated owner universe and covered owner prefixes with no unaccounted gaps, under the relevant verifier frontier.

Missing, hidden, withheld, gapped, unknown, forked or untrusted source state is Indeterminate, never empty. A witness quorum does not establish the owner roster is exhaustive.

### 5.2.1 SCITT and verifiable-data-structure profile

Do not create a new transparency ledger or signed-statement protocol. Compose the existing SCITT adapter owner (#6530) and the signed-evidence / anti-equivocation owners (#5863, #5942).

Standards snapshot as of 28 November 2026:
- RFC 9943 (June 2026, Proposed Standard) defines SCITT signed-statement transparency, issuer and subject claims, Transparency Services, registration policies and COSE receipts.
- RFC 9942 (June 2026, Proposed Standard) defines COSE Receipts for Verifiable Data Structures and proof-type interoperability.
- draft-ietf-scitt-receipts-mmr-profile-00, published 21 November 2026, is an active SCITT Working Group Internet-Draft for COSE receipts over Merkle Mountain Ranges. It is experimental, not a final RFC or production dependency.

Use the SCITT statement/receipt format as a source-trust adapter. The source-trust stage must distinguish:
1. issuer signature and protected-header validation;
2. issuer identity/key epoch and trust anchor;
3. statement subject/profile/payload digest;
4. TS identity and registration policy;
5. COSE receipt signature and VDS proof verification;
6. selected VDS identifier/version/hash algorithm and proof profile;
7. inclusion and consistency from the verifier's pinned state;
8. cross-client non-equivocation/split-view evidence;
9. issuer participation and completeness of the relevant statement universe;
10. semantic truth and independent outcome, which are not established by registration.

RFC 9943 explicitly warns that issuer statements may be false or inaccurate and that issuers can refuse to register, or selectively submit, statements. A registered statement proves that an issuer made it and a trusted TS registered it under its policy; it does not prove real-world truth or complete participation. The owner-universe and source-prefix-closure obligations remain independent DJCC stages.

Treat RFC9162_SHA256 and the SCITT MMR profile as distinct VDS algorithms. Bind every receipt to its declared VDS profile, hash algorithm, TS identity, tree size/accumulator and trusted prior state. Do not reinterpret proof bytes across VDS profiles or migrate an accepted checkpoint without an explicit, separately verified transition.

MMR-specific controls for any experimental adapter:
- Inclusion and consistency receipt payloads are detached; recompute the accumulator/root from proof material and compare it with the signed payload.
- Consistency must start at the verifier's already trusted VDS, tree size and accumulator; trusted state advances only forward.
- An MMR inclusion receipt alone does not pin a historic tree size. An empty inclusion path does not bind the index.
- The MMR draft's node construction does not provide RFC9162-style leaf/internal-node domain separation. The application leaf-commitment profile must prevent an entry preimage from being confused with an internal-node preimage and must bind position where an empty path requires it.
- A valid TS receipt must not satisfy the owner-universe, semantic projection, current authority or factual-outcome stage by itself.

Primary references:
- RFC 9943: https://www.rfc-editor.org/rfc/rfc9943
- RFC 9942: https://www.rfc-editor.org/rfc/rfc9942
- SCITT MMR profile -00: https://datatracker.ietf.org/doc/html/draft-ietf-scitt-receipts-mmr-profile-00
- SCITT WG document status: https://datatracker.ietf.org/wg/scitt/documents/

Compare any witness overlay against RFC 9943 non-equivocation/auditor requirements and conventional CT-style client gossip/monitoring. A witness quorum is defense-in-depth, not a substitute for a conformant TS or independent owner-universe closure.


### 5.2.2 Independence is not authentication or completeness

Independence is a sixth, separate claim dimension. Authentication proves that an evidence item is attributable to a key/identity under the selected trust policy; completeness concerns the declared source universe and covered frontier. Neither implies that two evidence items have independent provenance, operators, custody, sensors, power, network paths, incentives, or upstream data feeds.

A corroboration claim therefore needs an explicit, profile-bound failure-domain model. It should record (at minimum) the organization/operator, administrative control plane, signing-key custody, collection device, calibration chain, network/transport, upstream source, transformation pipeline, and relevant economic/control dependency. Unknown lineage is Indeterminate, not independent. Every shared failure-domain class must also be explicitly classified as material or explicitly non-material by the profile; a shared but unclassified class remains Indeterminate, never implicitly harmless. A different key, vendor logo, API endpoint, or receipt format is not sufficient evidence of independence. A new dependency-free Rust module, `failure_domain`, checks a declared lineage DAG for canonical ordering, missing parents, cycles, unreachable nodes, required failure-domain class coverage and shared material ancestor IDs. It reports `StructurallyDisjointWithinDeclaredGraph`, `CommonCauseDetected` or `LineageCoverageIncomplete`; it deliberately does not turn a structurally disjoint graph into an established independence claim. Provenance authenticity, lineage completeness and threat-model coverage remain external obligations.

For the independent-corroboration profile, independence must be established at every mapped producer stage and carried across each dependency edge that consumes it. A common-cause mutation must keep the root claim Indeterminate even when every receipt is authentic and each source is individually complete. Independence is not a generic multiplier or a numeric score: it is a profile-specific claim with explicit failure-domain assumptions and falsifiable counterexamples.

### 5.2.3 Planned MMR adversarial extension (not yet executable)

The current eight symbolic seed cases do not qualify an MMR implementation. Add these as distinct mutation classes before claiming MMR interoperability:

| Candidate class | Mutation / counterexample | Required conservative result |
|---|---|---|
| DJCC-009 | Validly registered issuer statement contradicted by independent field measurement | Authentication may pass; Truth is Rejected or Indeterminate under the adjudication profile |
| DJCC-010 | Issuer selectively withholds a relevant statement | Completeness is Indeterminate; no absence claim |
| DJCC-011 | Detached MMR receipt payload differs from recomputed accumulator | Source trust Rejected |
| DJCC-012 | Receipt proof bytes are reinterpreted under a different VDS/hash profile | Source trust Rejected |
| DJCC-013 | Empty/short MMR inclusion path asserted for an arbitrary historical tree size or index | Indeterminate or Rejected unless leaf commitment and position are bound by the application profile |
| DJCC-014 | MMR consistency proof starts from a state other than the verifier's pinned VDS/tree-size/accumulator | Rejected; trusted frontier does not advance |
| DJCC-015 | Two clients receive conflicting roots/accumulators for a purportedly common checkpoint | Indeterminate pending non-equivocation resolution |
| DJCC-016 | Multiple receipts share one operator, key custodian, or upstream measurement feed | Independence is Indeterminate |
| DJCC-017 | Application leaf commitment permits leaf/internal-node preimage ambiguity | Rejected or UnsupportedByProfile |

These are test specifications only. They must not be counted toward seed execution, mutation kills, or standards conformance until exact fixtures, independent expected outcomes, canonical byte vectors, and executable adapters exist. The MMR draft's verifier starts consistency checks from an already trusted VDS, size, and accumulator and states that trusted state advances only forward; it also leaves the leaf commitment scheme to the application profile. Bind these requirements to tests rather than inferring them from a valid COSE signature.

### 5.3 No hindsight

Historical replay uses the exact verifier frontier and policy/evaluator versions accepted for the original decision. Later evidence may change the current assessment only when accepted into a later frontier. It cannot rewrite the historical certificate.

### 5.4 No semantic laundering

The semantic projection must be reproduced from the exact committed assertion payloads, schema/profile and mapper build. Validity endpoints require explicit validity-episode identity. Missing VALID_TO is not OPEN unless an explicit profile-qualified open disposition exists.

### 5.5 No authority/effect laundering

Policy output, model recommendation, provider completion, action coverage and execution acknowledgement cannot mint authority, capability, physical-effect evidence or closure. An amended action denominator requires the canonical owner and a distinct authority decision. Outcome claims require independent observations under the relevant measurement profile.

### 5.6 Unknown outcome freezes mutation

An OutcomeUnknown execution result blocks new mutation in the same effect/idempotency scope until exact reconciliation. Correction/recompute must invalidate the exact affected dependency cone; unaffected independent artifacts remain current where the dependency oracle says they are unaffected.

## 6. Initial composition corpus

The initial seed manifest is:

\`tools/economic/evidence-execution-composition/DJCC_V0_1_SEEDS.json\`

It defines eight synthetic scenarios:
1. fully composed positive control, within a declared structural claim ceiling;
2. inclusion valid only relative to an unauthenticated root;
3. hidden owner / uncovered prefix;
4. backdated correction accepted only beyond the historical frontier;
5. policy/evaluator epoch substitution;
6. same-count altered action set;
7. OutcomeUnknown followed by an attempted new mutation;
8. provider completion without independent physical outcome.

This is a seed corpus, not the final 24-class acceptance corpus. The issue's 24 required adversarial classes remain the coverage target. Every case must have an independently specified expected stage disposition, a final disposition, a positive control and at least one negative control.

## 7. Oracle and implementation strategy

Use two independently authored implementations:
- Go composition oracle, independent of candidate Rust implementation and owner logic;
- Rust verifier/reference adapter, consuming exact owner receipts through public APIs.

Do not share the candidate composition evaluator's decision code with its oracle. Shared serialization vectors are permitted; expected-result derivation is not.

Qualification order:
1. validate schema and corpus manifests;
2. freeze canonical bytes and SHA-256 known-answer vectors;
3. test exact edge binding and profile validation;
4. test positive and negative composition cases;
5. run adversarial mutations and independent oracle parity;
6. add bounded state-machine/model-check cases for frontier and OutcomeUnknown barriers;
7. measure performance only after correctness passes.

Cross-language parity is a byte contract: canonical field ordering, UTF-8 rules, integer encoding, optional/null distinction, array ordering, domain separation and digest preimage must be specified explicitly. Generic JSON serialization is not itself a canonical encoding.

### 7.1 V0.1 structural candidate

The current research branch contains:
- the Go seed-registry oracle under `tools/economic/evidence-execution-composition/oracle`, with an independently hard-coded expected-disposition table for DJCC-001..008;
- the Rust candidate crate `mycelix-workspace/crates/mycelix-djcc-verifier`;
- a Draft 2020-12 certificate schema;
- exact-head Go and Rust formatting/test/race/vet/Clippy workflows.

The Rust candidate checks this fixed dependency graph:

```
verifier_frontier
  └─ source_trust
       └─ semantic_projection
            └─ query
                 └─ policy
                      └─ authorization
                           └─ execution

outcome consumes:
  source_trust + semantic_projection + execution
authorization also consumes verifier_frontier directly
```

The graph is encoded as an allowlisted stage-edge table, not supplied by the certificate author. Each edge binds a named consumer input to the exact producer output digest and output-schema reference. Every stage also binds the same canonical scope digest under the selected scope schema. The selected composition profile reference/digest must match the certificate.

The candidate also checks stage output/disposition consistency, canonical dependency/input/reason ordering, profile-limited NotApplicable stages, and claim-ceiling monotonicity.

**Dependency requirements are claim-specific, not stage-wide.** Each edge carries a non-empty, sorted, unique set drawn from `authentication`, `authority`, `completeness`, `effect`, `independence`, and `truth`. A verified consumer must see every named claim EstablishedWithinProfile on that exact producer. It may still consume a narrower established claim from a parent whose overall stage disposition is Indeterminate for a different dimension. For example, semantic projection can use an authenticated issuer statement without implying that all issuer statements were submitted; a query that requires complete coverage must separately require the completeness claim.

The selected profile maps final root claims to their responsible stages. The verifier checks that every root dimension declared EstablishedWithinProfile is established by every stage in that profile-owned claim map. Final composition disposition is derived from the profile's required root claim set, not from an all-or-nothing rule over unrelated stage dispositions. A profile must explicitly allow NotApplicable stages; a NotApplicable stage must mark all six of its claim dimensions NotApplicable and cannot be a producer for a required root claim.

The three V0.1 profiles are in `DJCC_PROFILES_V0_1.json`: full evidence-to-outcome composition, historical decision-only composition, and independent-corroboration decision composition. The corroboration profile requires independence as a root claim and treats execution/outcome as explicitly NotApplicable; it does not imply a physical-outcome claim.

### 7.2 Shared stage-edge byte contract

The first shared Go/Rust known-answer contract is `DJCC_STAGE_EDGE_V1_KAT.tsv`. Its preimage is:

1. UTF-8 string `MYCELIX-DJCC-STAGE-EDGE-V1`;
2. producer stage ID;
3. consumer stage ID;
4. consumer input name;
5. producer output-schema reference;
6. big-endian u32 count of required claim dimensions;
7. each required claim dimension in canonical order;
8. producer output digest as raw 32 bytes.

Every string is prefixed by a big-endian u32 **byte** length. Claim dimension order is authentication, authority, completeness, effect, independence, truth. The corpus includes ASCII and non-ASCII UTF-8 input names, single-claim and multi-claim edges, and an explicit independence-bearing edge. Go and Rust independently encode and compare the same exact preimage bytes.

This locks down only one edge encoding. It is not a complete certificate canonical encoding, JSON canonicalization profile, or signed certificate digest. Full-certificate canonicalization remains a separate deliverable.

This is a *structural composition verifier only*. It does not compute cryptographic hashes over serialized certificates yet; the digest values are inputs from owner receipts. It does not authenticate their producers or verify signatures, root identity, owner-universe completeness, actual source truth, policy semantics, legal authority, IVC proofs or physical outcome. Go seed validation does not evaluate Rust candidate certificates; candidate/oracle parity remains open work.

The eight JSON seeds remain symbolic fixtures until exact positive/negative fixture bytes and corpus hashes are frozen. The MMR adversarial matrix below is a planned extension, not executable coverage. The 24-class composition corpus remains the research target; eight initial cases are not 24/24 coverage.

## 8. Adversarial benchmark

The 24 composition classes are defined in #6821. Keep the following score families separate:

- hard-safety mutation detection;
- false-allow / false-closure / false-absence / false-complete rate;
- claim-ceiling widening;
- historical hindsight leak;
- stage-edge substitution;
- exact replay parity;
- correction-impact recall;
- correction-impact precision / unnecessary revalidation;
- abstention and explicit indeterminate behavior;
- reviewer burden and false holds.

An equivalent mutant is not a killed mutant. Invalid fixture shape is not an evaluator pass. An unexecuted or skipped vector is not a pass. Publish denominators and all per-case outcomes.

## 9. Matched SOTA / industry comparison

| Baseline | Native capability to respect | DJCC comparison boundary |
|---|---|---|
| SQL Server temporal tables / XTDB | Valid/system-time history and retrospective correction | Same temporal query and correction semantics; causal verifier frontier is a separate obligation |
| OpenTelemetry | Source Timestamp and ObservedTimestamp | Interoperability and clock/observation semantics, not causal acceptance/completeness |
| RFC 9162 transparency | Merkle inclusion and consistency | Root identity, witnesses, source-universe closure and semantic projection remain separate |
| VeriTS / VTRQ / authenticated temporal ADS | Authenticated time/range query structures | Proof domain coverage, generation/verification bytes and update cost |
| TruthTable | Verifiable SQL over committed database | Same query semantics, setup, proof generation and verification; source universe and real-world truth are not assumed |
| DBSP / Differential Dataflow / Materialize | Incremental result/view maintenance | Result delta vs proof delta vs independent end-to-end requalification |
| Cedar | Schema-aware policy validation and authorization evaluation | Same principal/action/resource/context/policy result; source completeness and physical outcome are separate |
| Zanzibar | Causal ordering and externally consistent authorization | Do not claim causal authorization as novel; compare wider evidence/query/decision composition separately |
| W3C PROV-O / OCEL 2.0 | Provenance and object-centric event interchange | Mapping/round-trip fidelity, not source trust or authority |
| Human, checklist, CMMS/ITSM, LLM/RAG | Operational workflow and decision-support capability | Blinded identical inputs, native features, time/access budget, reviewer minutes and adjudicated outcomes |

Record product/version/configuration and adapters. A capability that is not exposed by the tested interface is NotTested, not a failure. Publish per-obligation comparison matrices and the cost of stronger guarantees.

Primary reference set for baseline reproduction:

- VeriTS, *Verifiable Model-Enhanced Time-Series Queries on Blockchain Systems*, arXiv:2608.28318 (28 August 2026): https://arxiv.org/abs/2608.28318
- Cedar policy language and validator documentation: https://docs.cedarpolicy.com/
- OpenTelemetry Logs Data Model (Timestamp / ObservedTimestamp): https://opentelemetry.io/docs/specs/otel/logs/data-model/
- RFC 9162, Certificate Transparency Version 2.0: https://www.rfc-editor.org/rfc/rfc9162
- RFC 9942, COSE Receipts: https://www.rfc-editor.org/rfc/rfc9942
- RFC 9943, An Architecture for Trustworthy and Transparent Digital Supply Chains: https://www.rfc-editor.org/rfc/rfc9943
- SCITT MMR COSE Receipt Profile -00 (Internet-Draft, not a final RFC): https://datatracker.ietf.org/doc/html/draft-ietf-scitt-receipts-mmr-profile-00

### 9.1 Matched comparison protocol: compete at the composition boundary

DJCC should not claim to outperform a specialist on a capability the specialist is designed to optimize. In particular, VeriTS (arXiv:2608.28318, 28 August 2026) reports authenticated time-series query improvements—over two orders of magnitude in one windowed-aggregation verification comparison and up to 14.5× smaller range-query proofs than per-record proofs. Those results are a baseline to reproduce, not an invitation to make an unsupported blanket “faster than SOTA” claim.

Use a two-plane evaluation:

1. **Native component plane.** Compare the component with its natural specialist: query proof vs VeriTS/VTRQ/TruthTable; policy decision vs Cedar; temporal correction vs SQL Server temporal/XTDB; incremental view maintenance vs DBSP/Differential Dataflow; transparency receipts vs RFC 9162 and the selected SCITT/RFC 9942 VDS profile. Match input data, query/policy semantics, data freshness, hardware, cryptographic suite, setup costs and verification placement. Report setup, proof generation, proof size, verification and update cost separately.
2. **End-to-end composition plane.** Starting from the exact same synthetic operational decision, compare each native component chain with and without DJCC's independent composition verifier. Require the same accepted frontier, owner universe, source claims, semantic projection, query, policy epoch, authority scope, execution transcript and outcome obligations. Report both the additional assurance obtained and its marginal cost. A specialist's result is NotTested for obligations outside its advertised interface, not an automatic failure.

For query specialists, report two distinct outcomes: (a) same query proof task, where the specialist's native design should be allowed to win; and (b) the cost to carry the authenticated query into a complete decision certificate with the other owners' exact dependency bindings. Do not conflate them.

For Cedar, use the exact same principal, action, resource, context, schema and policy in the native policy evaluator. Compare decision correctness and evaluation latency directly. Separately measure DJCC's ability to bind the policy epoch, evaluator build, canonical input digest, historical replay result and downstream authority edge. That composition layer is additional; it is not a replacement for Cedar's formalized policy semantics or sound schema validation.

For OpenTelemetry, preserve and test Timestamp and ObservedTimestamp semantics in adapters. Treat them as source-event and observation timestamps, respectively; neither alone is an accepted causal frontier, an authenticated owner-universe closure, or proof that no late/withheld event exists.

### 9.2 Assurance frontier and falsifiable novelty claims

The primary novelty hypothesis is not “more hashes” or “one universal proof.” It is that an independent verifier can identify exactly which decision claims are justified, which are not, and which remain indeterminate—while checking the relationships across independently owned systems.

Measure the following separately:

| Metric | Operational definition | Required interpretation |
|---|---|---|
| Assurance-cost frontier | Pareto curve of verification time, proof/certificate bytes, storage/network cost and operator effort against the set of root claims established | Compare only profiles with the same claim set and trust assumptions |
| Claim-boundary precision | Number of root claims established without violating their declared stage/edge obligations | Do not reward the verifier for making broader unsupported claims |
| Common-cause detection | Fraction of common-operator, common-key-custody, shared-sensor, shared-calibration, shared-feed and shared-control-plane mutants that leave Independence non-established | Authentic signatures do not count as independent corroboration |
| Completeness/absence soundness | False-absence and false-completeness rates under hidden, withheld, gapped, stale and forked owner universes | Required high-consequence target: zero false-safe results |
| Correction locality | Impacted decisions requalified / all decisions requalified, with affected-decision recall and unaffected-decision precision reported separately | Never optimize precision by sacrificing recall |
| Historical stability | Byte-identical historical certificate/replay under evidence and policy updates outside its pinned frontier | Current reassessment must be a separate artifact |
| Abstention quality | Coverage vs error/risk curve, with reason codes and time-to-resolution | A safe Indeterminate is preferable to an unsupported pass |
| Review burden | Blinded human minutes, escalations and false holds per accepted decision | Compare under equal evidence, training and time budgets |

The research claim should be rejected or narrowed if a specialist baseline achieves the same claims with lower total cost, if the independent-corroboration profile accepts common-cause evidence, or if a correction can alter a historical certificate. Publish those negative outcomes.


## 10. Performance protocol

Do not benchmark the certificate as one monolithic JSON blob. Measure each stage and the composition critical path.

| Dimension | Workloads |
|---|---|
| owners | 1 / 10 / 100 / 1,000 |
| receipts/events | 1 / 10 / 100 / 1,000 / 10,000 / 100,000 |
| decision dependencies | 1 / 10 / 1,000 / 100,000 / 1,000,000 |
| mutation rate | 0 / 0.1 / 1 / 10 / 50 / 100% |
| correction position | 0 / 1 / 10 / 50 / 90 / 99 / 100 percentile |
| dependency fan-out | 1 / 2 / 4 / 8 / 16 / 64 |
| frontier regime | comparable / concurrent / gapped / forked / stale / withheld |

Report p50/p95/p99, CPU/hash operations, allocations, peak RSS, proof bytes, certificate bytes, network bytes, storage amplification, full replay, incremental requalification, correction recall/precision, common-cause detection, false-absence/false-completeness, historical stability, assurance-cost frontier, human-review burden, abstention coverage/error, false holds and recovery time. Separate proof generation, proof verification, frontier authentication, owner-universe closure, semantic projection, query, policy, authority, execution and outcome costs; include cold-start/setup costs and cache-hit/miss regimes.

Run at least 30 independent timed trials after warm-up. Freeze source SHAs, corpus hashes, toolchains, machine/kernel, benchmark parameters and scripts. Publish raw machine-readable samples, including failures and timeouts.

## 11. Proposed qualification gates

Targets, not current results:
- 100% deterministic canonicalization and Go/Rust known-answer parity;
- 100% exact stage-edge binding;
- 100% exact replay parity on the frozen corpus;
- zero hard-safety false allows, false absence, false completeness, false authentication and false outcome closure;
- zero claim-ceiling widening;
- 100% hard-safety mutation detection;
- 100% correction-impact recall, with precision independently reported;
- zero unauthorized denominator change;
- zero mutation after OutcomeUnknown before reconciliation;
- historical certificates unchanged by later correction/policy revisions.

No aggregate score can mask one high-consequence false-safe result.

## 12. Claim ceiling

A DJCC certificate is a profile-bound composition and replay artifact. It is not source truth, trusted global time, legal authority, a permit, an execution capability, a proof of physical effect, a service-level outcome, or production readiness.

Authentication, owner-universe closure, semantic correctness, policy correctness, current authority, execution and physical outcome remain separate claims made by their owning evaluators. Fixtures are synthetic; no live municipal, resident, worker, financial or operational data is in scope.
