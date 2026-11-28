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
| Source/log trust | #6727, #6810, #6812, #6818 | Bind receipt/log identity, inclusion, consistency, checkpoint, key epoch, governed witnesses and split-view state |
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
- **Stage claim ceiling:** authentication, completeness, truth, authority and effect are independent fields.
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

The five per-stage claim dimensions use the same explicit state approach. A stage can verify structural receipt integrity while leaving authentication, source completeness and truth as NotEvaluated.

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

The candidate also checks stage output/disposition consistency, canonical dependency/input/reason ordering, profile-limited NotApplicable stages, and claim-ceiling monotonicity. A root claim dimension cannot be EstablishedWithinProfile if any required stage has that dimension as Rejected, Indeterminate, NotEvaluated, UnsupportedByProfile or NotApplicable. A child stage also cannot claim VerifiedForDeclaredClaim when any required producer stage is Rejected or Indeterminate; descendants of a blocked stage must abstain or return their own non-success disposition rather than preserving a false green chain.

### 7.2 Shared stage-edge byte contract

The first shared Go/Rust known-answer contract is `DJCC_STAGE_EDGE_V1_KAT.tsv`. Its preimage is:

1. UTF-8 string `MYCELIX-DJCC-STAGE-EDGE-V1`;
2. producer stage ID;
3. consumer stage ID;
4. consumer input name;
5. producer output-schema reference;
6. producer output digest as raw 32 bytes.

Each string is prefixed with a big-endian u32 byte length. The corpus includes ASCII and non-ASCII UTF-8 input names. Go and Rust independently encode and compare the same exact preimage bytes.

This locks down only one edge encoding. It is not a complete certificate canonical encoding, JSON canonicalization profile, or signed certificate digest. Full-certificate canonicalization remains a separate deliverable.

This is a *structural composition verifier only*. It does not compute cryptographic hashes over serialized certificates yet; the digest values are inputs from owner receipts. It does not authenticate their producers or verify signatures, root identity, owner-universe completeness, actual source truth, policy semantics, legal authority, IVC proofs or physical outcome. Go seed validation does not evaluate Rust candidate certificates; candidate/oracle parity remains open work.

The eight JSON seeds remain symbolic fixtures until exact positive/negative fixture bytes and corpus hashes are frozen. The 24-class composition corpus remains the research target; eight initial cases are not 24/24 coverage.

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

Report p50/p95/p99, CPU/hash operations, allocations, peak RSS, proof bytes, certificate bytes, network bytes, storage amplification, full replay, incremental requalification, correction recall/precision, human-review burden, abstention and recovery time.

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
