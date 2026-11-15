# PPAC Benchmark Protocol V1 — matched proof, privacy and institutional assurance

**Status:** Research benchmark protocol. No result in this document is an achieved Mycelix/Symthaea/Xenia result.

**Tracking:** Mycelix #6230 and draft PR #6231.

**Snapshot date:** 2026-11-15.

## 1. Benchmark thesis

Do not attempt to win a single generic ZKP speed leaderboard. Mycelix's opportunity is to optimize the full cost of a privacy-preserving institutional decision while preserving proof soundness, source provenance, consent and change-driven requalification.

The benchmark should answer four different questions:

1. **Is the exact mathematical statement proved?** — statement binding, verifier correctness, soundness and program identity.
2. **What does the verifier learn?** — direct disclosure, low-entropy inference, metadata leakage and cross-context linkability.
3. **Can the proof support a legitimate claim?** — issuer, source, credential status, policy and LISE dependencies.
4. **Can the claim safely support one effect?** — Xenia consent/session, domain authorization, one-time action capability, execution outcome and independent observation.

A fast proof that leaks the witness, proves the wrong statement, accepts an unauthorized issuer, or authorizes a substituted effect is a failed benchmark result.

## 2. Standards and external baseline snapshot

Use standards conformance and workload-matched measurements as separate baselines.

### Credential and presentation standards

| Standard/profile | Status on 2026-11-15 | Benchmark implication |
|---|---|---|
| W3C Verifiable Credentials Data Model 2.0 | Recommendation, 15 May 2025 | Stable interoperability baseline for issuer/holder/verifier credential semantics. |
| W3C Verifiable Credential Data Integrity 1.0 | Recommendation, 15 May 2025 | Stable baseline for securing credential data with cryptographic signatures/proofs. |
| W3C Data Integrity BBS Cryptosuites 1.0 | Candidate Recommendation Draft, 10 September 2026 | Selective-disclosure/unlinkable-derived-proof candidate; evaluate against its exact conformance suite and metadata caveats, not as a completed Recommendation. The working group calls for two independent implementations per feature to exit CR. |
| W3C VC Data Model 2.1 / Data Integrity 1.1 | Working Drafts, latest 30 September 2026 | Track as forward-compatibility work; don't silently describe V2.0 implementations as V2.1 conformant. |
| W3C Bitstring Status List 1.0 | Recommendation, 15 May 2025 | Stable baseline for credential status publication; benchmark list fetch privacy, shared-list/group privacy and cache freshness. |
| W3C Bitstring Status List 1.1 | Working Draft, 11 November 2026 | Track as a draft; specifically test URL/slot correlation, list size, group size, status freshness and holder/verifier linkability. |

### Post-quantum status is a separate cryptographic profile

| Standard | Status | What it covers | What it does not establish |
|---|---|---|---|
| NIST FIPS 203 / ML-KEM | Final, 13 August 2024 | Post-quantum key encapsulation / shared-secret establishment | Digital signatures, ZK proofs or credential privacy |
| NIST FIPS 204 / ML-DSA | Final, 13 August 2024 | Post-quantum digital signatures | Privacy of a credential or proof witness |
| NIST FIPS 205 / SLH-DSA | Final, 13 August 2024 | Stateless hash-based post-quantum signatures | ZK soundness or institutional authority |
| W3C Quantum-Resistant Cryptosuites v1.0 | First Public Working Draft, 16 June 2026 | Draft PQ Data Integrity cryptosuite work for VCs | A final interoperable PQ credential profile |

The BBS CRD uses BLS12-381 pairing-based key material. Treat BBS as a classical selective-disclosure/interoperability track, not a post-quantum credential suite. Xenia's ML-DSA/ML-KEM support does not make the Mycelix credential signature, BBS presentation, RISC Zero receipt, Winterfell proof, Binius/Iron Spartan proof, verifier software supply chain or stored evidence post-quantum by inheritance.

Every proof/credential profile must publish its own crypto inventory: issuer signature, holder/presentation proof, commitment, hash, proof-system/FRI or polynomial-commitment layer, recursion/compression, verifier trust root, session KEM/signature and long-term evidence timestamping. Classify each component as PQ, classical, hybrid, unknown or not applicable, with exact algorithm/version and migration rule. A high-level label such as "PQC-enabled" is insufficient if one binding layer remains classical.

For a policy requiring PQ security, reject downgrade to classical-only keys, mixed or unenrolled hybrid pairs, stale algorithm profiles and receipts signed under an unapproved suite. Preserve algorithm agility and historical verification separately: a new algorithm policy must not rewrite the historical fact that an older signature verified under the then-applicable profile.

Primary references:
- NIST FIPS 203: https://csrc.nist.gov/pubs/fips/203/final
- NIST FIPS 204: https://csrc.nist.gov/pubs/fips/204/final
- NIST FIPS 205: https://csrc.nist.gov/pubs/fips/205/final
- W3C Quantum-Resistant Cryptosuites: https://www.w3.org/TR/vc-di-quantum-resistant/
- W3C Bitstring Status List 1.1: https://www.w3.org/TR/vc-bitstring-status-list-1.1/
- W3C BBS: https://www.w3.org/TR/2026/CRD-vc-di-bbs-20260910/

Primary sources:
- https://www.w3.org/TR/vc-data-model/
- https://www.w3.org/TR/2025/REC-vc-data-integrity-20250515/
- https://www.w3.org/TR/2026/CRD-vc-di-bbs-20260910/
- https://www.w3.org/groups/wg/vc/activity/2026/

### zkVM performance references (context only, not a PPAC ranking)

The following figures come from published, dated workloads. They are not directly comparable to one another or to an eligibility range proof. Keep the source workload, host, proof mode and measurement boundary visible.

| System / source | Workload and date | Verification | Proof size | Qualification |
|---|---|---:|---:|---|
| RISC Zero 3.0.4, independent eth-act zkEVM benchmark | Workload df07f52, measured 21 Sep 2026, 50 samples on Intel Xeon Gold 6548N | median 14 ms; AMD EPYC Milan median 16 ms | 218.06 KiB | Ethereum/zkEVM workload; benchmark lists security bits N/A. Do not transfer this number to a balance predicate or HDC trace. |
| OpenVM 1.4.3, same benchmark | Same workload/date; 50 samples | median 69 ms; AMD EPYC Milan median 83 ms | 773.67–808.93 KiB | Security bits N/A in the reported table. |
| SP1 5.2.4, same benchmark | Same workload/date; 50 samples | median 95 ms; AMD EPYC Milan median 112 ms | 1.41 MiB | Benchmark reports 100 bits UDR. |
| Zisk 0.15.0, same benchmark | Same workload/date; 50 samples | median 17 ms; AMD EPYC Milan median 36 ms | 237.91 KiB | Benchmark reports 128 bits JBR. |
| ZKM compressed proof, vendor performance page | Fibonacci n=500, 28 Oct 2026, 1 RTX 5090 prover and AMD EPYC 9355 host | median 25.5 ms over five warm native verifies | 290,530 bytes (284 KiB) | Vendor-reported; excludes setup, deserialization and one warm-up verification. Separately, ZKM reports Ethereum block proving throughput on 8 RTX 5090 GPUs; not the same workload. |

Sources:
- Independent zkEVM runs: https://eth-act.github.io/zkevm-benchmark-runs/verification/
- ZKM performance methodology and measurements: https://docs.zkm.io/introduction/performance.html

Do not rank the systems above as a universal fastest-to-slowest list. The benchmark panel is evidence that modern zkVM verification can be in the tens of milliseconds for specific workloads, with compressed proof sizes commonly in the hundreds of KiB to MiB range. It is not a PPAC target without a matched guest and security policy.

### Proof-system capability boundaries

| System/profile | Appropriate comparison | Privacy status to preserve |
|---|---|---|
| RISC Zero zkVM | General program-execution relation; bind receipt to independently expected ImageID and inspect public journal | ZKVM execution proof. The application still defines what enters the public journal; hiding an input does not authenticate its source. |
| Winterfell | Small, explicit AIR theorem; exact proof options and measured security profile | Do not claim zero-knowledge for the current Mycelix Winterfell profile. Soundness, privacy and measured security are separate. |
| Binius64 base prover/verifier | Binary-field computational-integrity workload | Not zero-knowledge in the vendored architecture description. |
| Iron Spartan wrapper | Binary-field multiplication relation with an explicitly qualified ZK wrapper | Privacy depends on the exact wrapper and transcript; upstream relation-target disclosure history remains a qualification concern. |
| Miden | VM-based verifiable computation | Experimental Mycelix adapter until exact verifier and runtime evidence exists. |
| BBS credentials | Issuer-signed messages with selectively disclosed derived presentations | Not the same workload as arbitrary arithmetic or general program execution. Metadata and revealed structure still require privacy tests. |

RISC Zero's verifier must use the expected ImageID for the intended ELF/program, and the journal is the declared public output. The application must bind that ImageID and journal schema to its own policy/program registry:
- https://docs.rs/risc0-zkvm/latest/risc0_zkvm/struct.Receipt.html
- https://dev.risczero.com/api/zkvm

NIST describes ZKP, MPC, FHE, PSI and PIR as distinct privacy-enhancing tools. Benchmark the primitive that matches the information flow rather than forcing every use case into a ZKP:
- https://csrc.nist.gov/projects/pec

## 3. Mycelix workload families

Publish separate benchmark suites. Never average their results into one leaderboard score.

### P1 — Credential disclosure

Question: can a holder prove the necessary issuer-signed claims while withholding irrelevant attributes?

Workloads:
- disclose one signed attribute;
- prove a range/threshold over a signed attribute;
- prove membership in an issuer-defined set;
- generate two presentations for distinct verifiers;
- status/revocation checks at different privacy profiles.

Compare:
- W3C VC DM 2.0 + Data Integrity 1.0 baseline;
- BBS derived proof when using the exact CRD profile;
- existing Mycelix selective-disclosure profile only after #5751 has actual proof verification.

Measure proof generation, verification, proof/presentation size, canonicalization cost, status lookup cost, attribute disclosure, cross-verifier linkability and metadata leakage.

### P2 — Private eligibility predicate

Question: can a verifier establish that an issuer-attested value satisfies a policy predicate without learning the value?

Workloads:
- age/income/balance threshold;
- interval/range;
- combined predicates with overflow/units/boundary tests;
- stale or revoked source credentials;
- wrong issuer and wrong subject;
- same value represented under different units/encodings.

Compare only exact matching statements and public inputs. Candidate backends include a qualified RISC Zero guest, a repaired/qualified Winterfell AIR for integrity-only profiles, and any later qualified privacy backend. A current underconstrained HealthRangeAir or structural-only adapter is not a valid performance baseline.

### P3 — General verifiable computation

Question: what is the cost of proving a deterministic program rather than a hand-written circuit?

Use the same source-level kernel and journal relation for each admitted VM. Compare:
- guest runtime;
- proving time and acceleration;
- receipt size;
- native verification;
- verification in browser WASM and Holochain/Wasmer;
- program/image identity and build reproducibility;
- cold start and memory footprint.

RISC Zero, Miden and future candidates belong here only after a real verifier exists. Don't compare unrelated vendor demos.

### P4 — HDC computation

Question: what does privacy cost for actual HDC operations?

Each HDC operation gets its own relation:
- exact Hamming distance/popcount;
- binding/XOR;
- bundled strict-majority threshold;
- CfC recurrence versus numerical sigmoid/state transition;
- triple-stack and federated composition.

Compare:
1. integrity-only Binius64 baseline;
2. Iron Spartan before the witness-hiding repair as a negative/control profile only;
3. Iron Spartan after exact source repair and independent privacy qualification;
4. one independent ZK backend implementing the same relation, if practical.

Never compare an integrity-only proof against a ZK proof as though they offer the same security property. Do not count analytic Winterfell estimates as measured performance.

### P5 — Cross-institution computation

Question: which party should learn which part of a joint computation?

Use NIST PEC distinctions:
- ZKP when a party proves a predicate over its own witness;
- PSI when parties need only identify intersection elements or cardinality;
- MPC when parties jointly compute on distributed private inputs;
- FHE when computation should occur over ciphertext;
- PIR when query privacy from the data holder is required.

Measure communication rounds/bytes, compute, output leakage and collusion assumptions. A ZKP-only baseline is not mandatory where another primitive is the right tool.

## 4. Matched-comparison protocol

Every comparative result must publish:

- exact repository, commit, dirty-tree state and dependency graph;
- exact source/program/AIR/circuit digest and compiler/toolchain;
- exact proof-system version and cryptographic configuration;
- exact mathematical statement, public-input schema and input/output vectors;
- witness distribution and sizes (including low-entropy, sparse and worst-case witnesses);
- privacy mode and declared leakage function;
- trusted setup assumptions, if any;
- hardware model, CPU/GPU, RAM, storage, OS, kernel, drivers and power profile;
- prover/verifier concurrency, thread count, batch size and cache policy;
- cold/warm status, setup inclusion/exclusion, serialization and network boundaries;
- raw timing samples, median, p95, p99, dispersion and confidence interval;
- reproducibility script and content-addressed corpus.

Keep independent measurements for:
1. setup/compilation;
2. source statement preparation;
3. proof generation;
4. proof serialization and transfer;
5. proof verification;
6. issuer/credential status checks;
7. LISE qualification/requalification;
8. Xenia consent and capability issuance/consumption;
9. effect execution/reconciliation;
10. independent observation.

Do not hide slow or expensive steps outside the timing boundary. Report both proof-only and end-to-end time-to-trustworthy-decision.

Recommended minimum:
- 30 repeated runs for routine latency distributions after separately reporting cold starts;
- at least three independent clean reproductions for a headline result;
- raw samples and scripts retained;
- bootstrap confidence intervals or another predeclared uncertainty method;
- one independent verifier for interoperability claims;
- no aggregate "score" that can average away a security failure.

## 5. Security, privacy and institutional metrics

### Soundness and implementation refinement

Hard gate:
- zero false accepts in the frozen adversarial corpus;
- every positive control reaches the intended path;
- mutation of public input, issuer, subject, commitment, program/image, verifier, policy or statement must fail at the intended gate;
- formally specified kernels must be linked to the exact production guest, not a verification-only duplicate;
- malformed proof inputs return typed errors, not panics.

Finite tests are not a proof of cryptographic soundness. Keep formal relation refinement, proof-system soundness and verifier implementation correctness as separate evidence classes.

### Privacy and linkability

Measure:
- unnecessary attributes revealed;
- low-entropy value inference from hashes/commitments;
- same-subject presentation linkability across verifiers;
- stable issuer/credential/subject identifiers;
- timing, proof-size, message-position and status-lookup metadata;
- transcript/query/opening leakage;
- verifier collusion and repeated-query inference.
- adaptive chosen-predicate sequences (for example, repeated threshold queries that narrow a hidden income/balance value), including query-order changes, cross-session continuity and policy/consent changes between requests.

Prospective target for a defined balanced unlinkability experiment: no unauthorized seeded-attribute disclosure, and distinguisher advantage upper confidence bound <= 0.01 for the declared test adversary. This is a finite experiment target, not a universal anonymity theorem.

For every privacy result, state the information intentionally revealed by the public statement and metadata. A proof that reveals the threshold result necessarily reveals that result.

A separate query-governance experiment must test that the domain's allowed predicate family, resolution, purpose, audience and repeated-query policy constrain adaptive disclosure. Do not call this differential privacy unless a formally defined DP mechanism and accounting model are actually used. Query accounting should reuse Mycelix PEC/domain policy owners and pairwise scoped subject references; it must not create a universal global privacy-budget service. Missing query history or incomplete policy coverage should block or return indeterminate, never imply a fresh allowance.


### LISE change-driven assurance

Reuse Mycelix #6219 and existing temporal/evidence owners. PPAC adds predicate-specific dependency mutations; it does not implement a second invalidation engine.

Measure:
- material changes detected;
- stale false-positive currentness accepts;
- affected claims invalidated;
- unrelated claims incorrectly invalidated;
- historical evidence preservation;
- time and compute to targeted requalification;
- missing change-feed/common-dependency behavior.

Prospective gates: no known material mutation missed in the frozen panel; no stale claim treated as current; unknown dependency coverage becomes indeterminate; historical evidence remains intact; no unrelated claim is invalidated on minimal-pair controls.

### Xenia consent and effect boundary

Measure:
- exact action binding coverage;
- denied/revoked consent rejection;
- session, lane and host substitution rejection;
- action substitution after consent;
- durable one-time capability consumption;
- restart and DeliveryUnknown behavior;
- revocation propagation and decision-to-effect race behavior;
- independent observation coverage.

The capability must not survive a change to the exact action, target, recipient, route, authority generation or consent scope.

### End-to-end institutional metric

Introduce **time-to-trustworthy-decision (TTD)**:

time from canonical request acceptance to a qualified claim plus domain authorization, including:
- proof generation/verification;
- issuer and credential validation;
- policy and LISE qualification;
- Xenia authentication and consent;
- one-time action capability issue/consume.

Report TTD both with warm caches and with a cold-start/revocation path. Do not omit human consent or policy checks to improve the number.

Also measure **requalification amplification**:

number of claims rechecked / number of claims actually affected by a material dependency change.

This is a potential Mycelix differentiator: a precise change should cause precise, explainable requalification rather than global cache invalidation or silent stale acceptance.

## 6. Proposed targets

These are research targets, not achieved results.

| Dimension | Initial gate | Stretch target |
|---|---|---|
| Invalid proof/statement acceptance | 0 in frozen adversarial set | Independent proof-system mutation suite and formal relation review |
| Wrong issuer/subject/program/challenge/action | 0 accepts | Differential verifier agreement across implementations |
| Unnecessary disclosure | 0 seeded protected-attribute leaks | Low correlation/metadata inference under multi-verifier collusion models |
| Historical/currentness separation | 100% correct in frozen temporal panel | Minimal requalification amplification at scale |
| Cross-verifier consistency | 100% frozen reference agreement | Two independent verifier implementations |
| Performance | Establish matched baseline | >=2x improvement in one named resource dimension at unchanged statement, proof mode, security assumptions and leakage model |
| End-to-end assurance | Full stage-by-stage measurement | Reduce TTD and requalification amplification without bypassing owners |
| Interoperability | VC DM 2.0 / Data Integrity 1.0 bounded profile conformance | Two independent BBS implementations for each selected feature, as required by the CR exit criteria |

Do not set numerical latency targets for Mycelix before baseline measurements of the same claim and hardware. zkEVM figures in Section 2 are context, not targets.

## 7. Industry position and differentiation strategy

Industry leaders have specialized strengths:
- zkVM vendors optimize general computation and recursive proof throughput;
- credential vendors optimize standards-compatible presentation and selective disclosure;
- proof-system projects optimize arithmetic, binary-field or domain-specific circuits;
- MPC/PSI vendors solve distributed multi-party privacy problems.

Trying to beat all of them on every primitive would waste effort and create invalid comparisons.

Mycelix can differentiate at the composition layer:
1. issuer-authenticated source values bound to the same hidden witness that satisfies the predicate;
2. privacy profiles selected by actual information-flow requirements, with NoQualifiedPlan when none qualifies;
3. exact proof-to-request-to-consent-to-effect binding;
4. LISE change-driven invalidation and historical reconstruction;
5. Xenia-authenticated operator consent and durable, single-use authority;
6. independent observation of the resulting effect.

The competitive claim to test is not "fastest ZKP". It is:

**lowest measured end-to-end cost of a privacy-preserving, independently reconstructable institutional decision under a matched assurance and disclosure policy.**

That claim should not be made until the complete path is implemented and independently reproduced.

## 8. Current blockers and nonclaims

- Mycelix #227 / #1873: quarantine and audit of underconstrained Winterfell AIRs.
- Mycelix #1899: Winterfell AIR soundness does not imply witness privacy.
- Mycelix #1905: candidate proof options are not a measured production security level.
- Mycelix #1919: malformed dimensions must not panic the verifier.
- Mycelix #5751 / #5724: selective disclosure and cross-DID verification/replay must be real and fail closed.
- Symthaea #5842 / #12269: guest relation refinement and issuer/request binding are separate.
- Symthaea PR #7403 / #12274: HDC semantic corrections, exact Binius source identity and witness-hiding qualification are separate.
- Xenia #359 / #364 / #369 / #435 / #449 / #450: receipt attestation, live trust, portable replay, session capability and session evidence remain distinct owners.
- Mycelix #6219 / #6140 / #6069: reuse existing LISE/effect-observation machinery.

This protocol creates no result by itself. All figures are external references with their stated workload limits. No proof-system, privacy property, standards conformance, institutional authority or deployment claim is established by writing this plan.
