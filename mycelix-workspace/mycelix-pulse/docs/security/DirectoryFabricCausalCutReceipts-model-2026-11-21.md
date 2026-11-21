# Directory Fabric causal-cut and effect-receipt model — 21 November 2026

**Model:** `DirectoryFabricCausalCutReceipts.tla`  
**Safe configuration:** `DirectoryFabricCausalCutReceipts.cfg`  
**Negative configurations:** omitted dependency, scalar frontier, retry while unknown, duplicate commit.  
**Status:** model and configs are source-present; TLC has not been executed in this environment.  
**Claim ceiling:** finite abstraction only; no signature, dependency resolver, Holochain conductor, external effect or production claim.

## State and intended theorem

The finite universe contains four mandatory dependency roles:
- membership;
- issuer admission;
- revocation;
- explicit deny.

The model separates:
1. the complete dependency universe (the hidden oracle's ground truth);
2. the supplied request cut;
3. the dependency positions observed at the PEP;
4. the resolver's closure-completeness status;
5. the independent policy evaluation result;
6. the resulting decision;
7. the effect lifecycle and logical attempt/commit counts.

The safety invariant is:

> An Allow requires a complete verified dependency cut, current evidence for every dependency in that cut, and a Permit policy result.

The dependency-omission negative control deliberately preserves the supplied cut's own freshness while omitting a mandatory revocation dependency. The PEP checks the dependencies it received; the hidden invariant checks that the received set equals the full required set. This models the actual trust boundary: a PEP cannot discover an omitted dependency merely by checking the dependencies it was told to check.

The scalar-frontier mutant uses a high sequence-like value from one authority to mask a missing required authority component. The safe model uses set/component coverage, not a sum of heights.

Effect lifecycle invariants:
- a committed effect requires an Allow decision;
- no more than one effect commits per logical operation;
- no new attempt starts while the previous outcome is Unknown;
- a no-effect reconciliation can permit a retry; a renewed Unknown blocks another retry.

## Safe model configuration

`DirectoryFabricCausalCutReceipts.cfg` sets all four mutant switches to FALSE and checks:
- `TypeOK`
- `ClosureClaimRequiresCompleteCut`
- `AllowRequiresCompleteCut`
- `EffectRequiresAllow`
- `AtMostOneCommit`
- `NoRetryWhileUnknown`
- `NoCommitWithoutEffectAttempt`

## Negative-control configurations

| Configuration | Single semantic mutation | Expected violated invariant |
|---|---|---|
| `...-omit-dependency-mutant.cfg` | Closure verifier labels an omitted-revocation cut complete | `ClosureClaimRequiresCompleteCut` and `AllowRequiresCompleteCut` |
| `...-scalar-frontier-mutant.cfg` | Sum-like frontier progress replaces per-dependency coverage | `AllowRequiresCompleteCut` |
| `...-retry-unknown-mutant.cfg` | Starts a new attempt while previous effect remains Unknown | `NoRetryWhileUnknown` |
| `...-duplicate-commit-mutant.cfg` | Allows a second effect commit for one logical operation | `AtMostOneCommit` |

These are expected counterexamples, not reported model-checking outcomes. The configurations must be run with a pinned TLC release; exact model/config/JAR digests and state counts must accompany the eventual results.

## Abstraction limits

- The model treats the dependency universe as a finite constant. It does not derive a dependency cut from a real policy/entity graph.
- The closure status is an abstract input. No verifier signature, proof format, trust-root or key-rotation protocol is implemented.
- Scalar height is a deliberately simplified mutant; it is not an exact model of every possible vector-clock or ZedToken design.
- Explicit deny is one dependency role; the model does not encode the full #4854 allow/deny denotation algebra.
- Policy evaluation is an abstract Permit/Forbid/DefaultDeny value; the model does not reproduce Cedar's full policy semantics.
- Effect ownership, idempotency and reconciliation are abstract state variables, not observations from real providers or resource owners.
- The model does not establish Holochain host admission or DHT currentness.

## Required next formal work

1. Run TLC on safe and each mutant configuration; bind JAR hash, Java version, exact source/config digests and state counts.
2. Add a model for policy/entity graph mutation between resolver snapshot, closure verification and decision evaluation.
3. Add cross-realm federation and explicit-deny inheritance to the dependency universe.
4. Add clock/lease expiry and decision nonce consumption, consuming the existing reference model semantics.
5. Compare each counterexample class with the #4752 full-context model and Pulse #5771 lifecycle models; do not create duplicate models for their existing claims.
