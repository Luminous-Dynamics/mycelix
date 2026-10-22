# Crypto-Economic Assurance Benchmark (CEAB-001)

**Status:** Proposed benchmark contract; not yet implemented or independently qualified  
**Revision:** 0.1.0  
**Prepared:** 2026-10-22  
**Tracking issue:** [#5407](https://github.com/Luminous-Dynamics/mycelix/issues/5407)

## 1. Purpose

Measure whether Mycelix preserves the binding between cryptographic identity, current authority, and the exact economic effect under adversarial conditions. This is not a signature-speed contest and does not introduce a new cryptographic algorithm.

Anthropic's 2026 ECDSA curve-origin result reconstructed the historical origin of public parameters; it did **not** break ECDSA. Anthropic's separate HAWK research demonstrates a different lesson: security estimates must be tied to the exact construction and parameter set. The engineering response is reproducible adversarial evidence and crypto agility, not an automatic switch away from ECDSA.

This benchmark composes work already tracked by:
- [#252](https://github.com/Luminous-Dynamics/mycelix/issues/252) and [#261](https://github.com/Luminous-Dynamics/mycelix/issues/261): exact post-quantum scheme/backend/wire semantics;
- [#510](https://github.com/Luminous-Dynamics/mycelix/issues/510): dependency-light algorithm registry;
- [#1354](https://github.com/Luminous-Dynamics/mycelix/issues/1354): historical validation and cryptographic renewal;
- [#667](https://github.com/Luminous-Dynamics/mycelix/issues/667), [#674](https://github.com/Luminous-Dynamics/mycelix/issues/674), [#679](https://github.com/Luminous-Dynamics/mycelix/issues/679), [#685](https://github.com/Luminous-Dynamics/mycelix/issues/685), and [#691](https://github.com/Luminous-Dynamics/mycelix/issues/691): SAP value-note spending, output spendability, witness policy, and conflict classification;
- [#1265](https://github.com/Luminous-Dynamics/mycelix/issues/1265): exact settlement-to-reservation/effect binding.

The benchmark must compose those boundaries without duplicating their implementation or promoting an unqualified prerequisite.

## 2. Core thesis: assurance is a chain, not a boolean

The protocol must not collapse these independent questions into `signature_valid: bool`:

```text
exact signed subject
  -> exact algorithm and wire profile
  -> cryptographic authenticity
  -> current signer/key authority
  -> authorization under a named policy generation
  -> exact economic-effect binding
  -> spend/conflict assurance
  -> settlement evidence
  -> historical revalidation, if needed
```

Every arrow is an explicit checked boundary. Evidence at one stage is not proof of a later stage.

Examples of forbidden inference:

```text
well-formed signature bytes       != verified signature
valid signature                   != currently authorized signer
authorized request                != exact intended economic effect
DHT publication                   != globally unique spend
countersigned action              != universal double-spend prevention
provider receipt                  != qualified settlement finality
historically valid signature      != permission for a new action today
algorithm deprecated today        != historical proof automatically erased
```

The economic effect commitment must bind all fields that can change what value moves or what obligation is discharged: protocol version, currency/instrument identity, exact denomination/base unit, amount, sender/owner, beneficiary, consumed input-note identities, fee, reservation/effect identity, idempotency identity, and required assurance-policy/profile revision. Field selection remains domain-specific; no field may be inferred from display text.

## 3. Threat model and deterministic adversarial corpus

The fixture format should be implementation-independent first. Each fixture records a case ID, exact subject commitment, profile/policy generation, setup, mutation, expected classification, and reason code. Rust implementations and multi-agent Holochain scenarios can consume the same semantic cases later.

| ID | Adversarial mutation | Required outcome |
|---|---|---|
| ALG-001 | A legacy or incompatible backend is labelled with a standardized algorithm ID (for example, pre-standard Dilithium bytes under an ML-DSA identity, or Kyber bytes under an ML-KEM identity) | Reject/quarantine until exact standard and wire semantics are qualified; byte length alone cannot promote it |
| ALG-002 | Unknown algorithm/profile ID, truncated key/signature, oversized material, or absent profile | Explicit unsupported/invalid result; never guess from size or fall back |
| HYB-001 | A profile requires both classical and PQ components but only one verifies | Reject the hybrid profile; no partial promotion |
| HYB-002 | Negotiation strips the PQ component, lowers the required profile, or reuses a valid signature under a weaker context | Reject downgrade unless a separately authorized migration policy explicitly allows that exact transition |
| SUBJ-001 | A validly signed request is replayed with a different recipient, asset, amount, input note, reservation, or effect identifier | Reject because the exact signed subject/commitment no longer matches |
| CAN-001 | Equivalent-looking but non-canonical encodings, reordered inputs, duplicate input IDs, integer overflow, or ambiguous unit/currency encodings | Reject or canonicalize to one unique subject before signing; independent verifiers must derive the same commitment |
| RPL-001 | Exact same intent is delivered repeatedly after timeout/retry | Idempotent duplicate observation; exactly one economic effect |
| RPL-002 | Same request/idempotency identity appears with a different subject digest | Conflict; neither version silently overwrites the other |
| KEY-001 | A key has been rotated, revoked, or disabled; a new operation attempts to use the old generation | Reject new authorization unless the selected historical/current policy explicitly permits it |
| KEY-002 | Old evidence was valid under a historical policy but is reviewed after the algorithm is deprecated | Preserve original evidence and report historical validity separately from present-day reliance policy |
| KEY-003 | A claimed signing time, local device time, trusted timestamp, compromise-effective time, and validation time are conflated | Reject a false time inference; return indeterminate when required evidence is absent |
| ECO-001 | Two distinct spends consume the same value-note input under parallel publication or a network partition | Classify as conflict under the value-note model; do not choose a winner by arrival time, timestamp, or action hash |
| ECO-002 | A child output is derived from a conflicted spend | Child output is frozen/non-spendable until an explicit, independently qualified resolution rule applies |
| WIT-001 | Witness quorum is missing, stale, policy-mismatched, or drawn from an unauthorized key generation | No high-assurance spend promotion; return rejected or indeterminate according to the missing-evidence semantics |
| WIT-002 | A witness signs two incompatible allocations of the same scarce input | Detect equivocation and classify the affected allocation set as conflict; do not conceal it by taking the first observation |
| REC-001 | Process restart, lost acknowledgement, timeout, or out-of-order delivery occurs between reservation, effect creation, and settlement | Recovery is deterministic and idempotent; ambiguous settlement is not upgraded to success |
| REN-001 | A successor cryptographic/archive renewal omits or changes its predecessor commitment | Reject renewal; preserve the original bytes and record |
| REN-002 | An algorithm/profile migration rewrites old algorithm identity or old signature bytes in place | Reject migration; add a predecessor-bound successor record instead |

A test returning `indeterminate` is not a pass for an operation that requires positive authorization. It is an acceptable *safe outcome* only when the caller cannot promote, spend, settle, or grant authority on that basis.

### Positive controls

Negative tests alone can reward an implementation that rejects everything. Include positive controls for:
1. a supported, correctly labelled classical profile;
2. each PQ profile only after its exact backend/wire qualification;
3. a hybrid profile where all required components verify over the same canonical subject;
4. a correctly authorized current key/policy generation;
5. a single valid spend with checked conservation and a qualified assurance mode;
6. historical validation with complete required validation material and a separately reported current-use policy result;
7. legitimate key rotation and predecessor-bound evidence renewal.

Unsupported profiles must remain unsupported; the test suite must not silently manufacture positive controls for them.

## 4. Benchmark metrics and target gates

### Security gates — hard requirements

- **False promotion:** zero curated cases may promote invalid, unsupported, downgraded, unauthorized, conflicted, or insufficiently evidenced state to an authoritative success state.
- **Negative-control coverage:** 100% of the named corpus cases must execute in the qualification run and reach the specified result classification.
- **Algorithm identity:** every production-authority-eligible algorithm ID must map to one exact standard revision, backend, wire encoding, and operation contract. Unqualified IDs remain blocked from authority-bearing paths.
- **Hybrid integrity:** 100% of single-component failure/downgrade controls fail closed for profiles requiring both components.
- **Economic conservation:** every accepted transfer satisfies checked input/output conservation; every distinct spend conflict is represented explicitly; conflicting descendants are not spendable.
- **Idempotency:** repeated delivery of one exact economic intent produces one effect; reusing its identity with different subject terms cannot silently overwrite or duplicate the effect.
- **Lifecycle separation:** historical verification, current key authority, current algorithm policy, and current permission to act are separately observable results.
- **Reproducibility:** a second verifier over the same pinned fixture and evidence set derives the same subject/profile commitments and semantic classification.

Any false promotion in a critical negative control blocks qualification regardless of throughput or aggregate score. Do not average away a security failure.

### Adversarial execution targets

These are proposed engineering targets, not demonstrated results:
- Run at least **10,000 deterministically seeded state-machine schedules** per supported spend-assurance mode for concurrent duplicate/replay/conflicting-input scenarios before claiming the tested schedule space is covered.
- Run crash/restart and message reordering at each durable state transition; preserve the seed and event trace for every failure.
- Require **100% rejection/classification of curated critical mutations**. Report mutation score for additional generated mutations separately; do not use a high aggregate mutation score to excuse one surviving critical mutation.
- Use multi-agent integration tests for DHT and witness behavior. A pure Rust state-machine test is not a substitute for Holochain conductor/integrity validation, and one successful multi-agent scenario is not a proof of all network behavior.

### Performance — measure, do not make up a winner

Report, for each exact profile and fixed environment:
- key-generation, signing, verification, and full transaction-validation latency (p50/p95/p99);
- throughput under single-threaded and concurrent load;
- public-key, signature, signed-subject, and complete-record byte sizes;
- peak memory and persistent-storage/write amplification;
- DHT publication-to-observation delay and recovery time under loss/reordering;
- witness count, policy/quorum configuration, and incremental network messages;
- end-to-end user-visible transfer latency, separated from primitive-crypto timing.

Compare identical standards/parameter sets and the same hardware/toolchain. Compare classical, PQ, and hybrid profiles as distinct security/size/performance choices, not as interchangeable units. Record the current best qualified backend as the per-algorithm baseline; any optimization must preserve the exact wire contract and all hard security gates. Initial results establish a baseline—do not assert “better than SOTA” until externally comparable, reproducible results exist.

### Evidence manifest

Every report should include:
- repository, branch, exact commit SHA, and source/dependency lockfile digest;
- test-harness version and fixture-corpus digest;
- algorithm ID, standard revision, concrete backend/version, feature flags, and policy generation;
- CPU/OS/compiler/profile/build settings;
- deterministic seeds and full minimized traces for failures;
- expected versus observed classification and explicit unsupported/indeterminate cases;
- raw timing samples, percentile calculation method, and benchmark commands.

Hosted CI states must be reported truthfully: queued, not run, cancelled, or missing evidence is **not PASS**.

## 5. Industry baseline and differentiation hypothesis

This proposal composes rather than replaces established work:
- NIST CSWP 39/39upd1 frames crypto agility as the ability to replace algorithms while preserving secure operations.
- NIST FIPS 203, 204, and 205 standardize ML-KEM, ML-DSA, and SLH-DSA, respectively; exact implementation conformance remains necessary.
- NIST SP 800-57 supplies key-management and lifecycle guidance.
- Holochain's own validation documentation assigns application-integrity rules to integrity zomes. Its countersigning documentation describes witnessed “lightweight consensus” for scarce-resource allocation and warns that trust assumptions matter.

The differentiation hypothesis is **cross-layer correctness**: the same reproducible corpus tests cryptographic profile identity, key/policy currentness, exact transaction meaning, spend conflicts, settlement claims, and later historical review. Generic primitive benchmarks rarely cover all of those economic semantics in one qualification story. This is a testable research hypothesis, not a present claim of superiority over industry systems.

Primary references:
- Anthropic, [Claude reveals ECDSA curve origins](https://www.anthropic.com/research/ecdsa-curve-origins) (2026-10-16).
- Anthropic, [Discovering cryptographic weaknesses with Claude](https://www.anthropic.com/research/discovering-cryptographic-weaknesses) (2026-07-28).
- NIST, [Considerations for Achieving Crypto Agility, CSWP 39upd1](https://csrc.nist.gov/pubs/cswp/39/upd1/considerations-for-achieving-crypto-agility/final).
- NIST, [Post-Quantum Cryptography standards](https://csrc.nist.gov/projects/post-quantum-cryptography/post-quantum-cryptography-standardization).
- Holochain, [Validation](https://developer.holochain.org/concepts/7_validation/) and [Countersigning](https://developer.holochain.org/concepts/10_countersigning/).

## 6. Implementation sequence

**Stage A — freeze the semantic corpus.** Review and version the case IDs above; bind each to an existing issue and exact expected outcome. No runtime algorithm changes.

**Stage B — pure Rust kernel.** Implement storage-independent canonical subject/effect fixtures, typed result classifications, deterministic identity derivation, idempotency/conflict model, and property/mutation tests. Reuse established types where possible; do not create a parallel algorithm registry.

**Stage C — cryptographic provider qualification.** Compose only already-qualified provider implementations. Keep the standardized algorithm ID blocked while its wire/backend contract remains unqualified. An algorithm registry test does not prove cryptographic correctness.

**Stage D — Holochain adapter and multi-agent tests.** Exercise separate agents, DHT integration, competing spends, witness quorum, disconnection, restart, and recovery under the exact pinned runtime. Do not claim globally atomic settlement if the selected mechanism only detects conflicts after publication.

**Stage E — lifecycle and performance.** Add key/policy rotation, revocation-time distinctions, predecessor-bound renewals, and apples-to-apples performance reports.

Each stage is a separate reviewable tranche. A documentation merge is not a security qualification; a local pass is not hosted exact-head evidence; a queued workflow is not a pass.

## 7. Explicit non-claims

This contract does not prove any current Mycelix implementation safe, establish real-world currency legitimacy or redemption, prove a production deployment, qualify any cryptographic backend, establish global consensus, or assert an industry performance lead. Those claims require their own source-bound implementation and independent evidence.
