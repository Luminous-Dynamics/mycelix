# Claim-Ceiling Policy Gate: Adversarial Qualification V1

**Status:** research and test plan; proposed thresholds below are not measured results or a certification claim.  
**Scope:** the standalone claim-ceiling policy-profile fit checker introduced by [Mycelix PR #5264](https://github.com/Luminous-Dynamics/mycelix/pull/5264).  
**Primary invariant:** no candidate can become admissible by widening policy scope, dropping required controls, substituting a different policy artifact, or exploiting ambiguity at the CLI/parser boundary.

## 1. What the existing evidence establishes

At exact head `ac48ea53b2f434292017bca5419cf48dcac70906`, the dedicated Claim Ceiling Policy Profile V1 workflow passed formatting, Rust tests, strict Clippy, byte-identical repeated CLI receipts, and independent jq verification of positive, negative, and altered-policy-digest cases. Its tests exhaust all 256 combinations of eight binary mutations. The separate D6S Canonical Qualification workflow failed in its integration suite; its verifier steps were skipped. These results are distinct and must not be combined into a claim that canonical admission is qualified.

The existing profile-fit result does not prove truth, publisher identity, source independence, trusted time, canonical evidence admission, user consent, or action authority. A SHA-256 pin establishes byte identity relative to a trusted expected digest; it does not authenticate who selected that digest.

## 2. Paradigm shift: qualify the policy lifecycle, not just the predicate

The current gate checks that a candidate is no broader than a separately pinned finite profile. The next assurance boundary is the complete lifecycle:

1. **Semantic scope:** all coupled claim atoms and subject/context/time/action dimensions remain inside the allowed ceiling.
2. **Artifact identity:** request, profile, schema, verifier, and receipt are bound to explicit identities.
3. **Authority:** a verifiable policy publisher and trusted policy root are established outside the candidate request.
4. **Freshness and revocation:** stale, revoked, superseded, or rolled-back policy versions cannot become valid merely because an old digest still matches.
5. **Consumer enforcement:** a passing fit receipt is not confused with evidence admission or an operational grant.
6. **Resource safety:** malformed or oversized input cannot turn the checker into an unbounded parser or ambiguous CLI invocation.
7. **Independent agreement:** separately implemented verifiers agree on disposition and material diagnostics.

This should evolve toward a *policy capability with provenance and lifecycle*, rather than a boolean allow/deny function. The authority, evidence-admission, and action-authorization decisions must remain distinct, separately governed checks.

## 3. Threat model

| Threat | Required behavior | Existing status |
|---|---|---|
| Forbidden full claim atom survives a mathematically valid meet | Reject against the explicit profile | Covered by current tests |
| Scope widened or mandatory requirement dropped | Reject with stable dimension code | Covered for eight dimensions |
| Profile bytes differ from separately pinned digest | Emit digest mismatch and reject | Covered by current fixtures/verifier |
| Repeated CLI option or unknown argument changes interpretation | Reject invocation before reading inputs | Follow-on hardening added in this branch; requires hosted tests |
| Request/profile file is excessively large | Reject before unbounded allocation/parsing | 1 MiB per-file cap added in this branch; requires hosted tests |
| Valid old profile is replayed after revocation or supersession | Reject against authoritative freshness/revocation state | Not implemented |
| Policy signer/key is unknown, rotated, revoked, or outside the expected domain | Reject or require explicit key transition | Not implemented |
| Candidate mutates schema version, uses unknown fields, or encodes invalid intervals | Reject structurally before semantic evaluation | Existing schema/serde checks; expand adversarial corpus |
| Rust and independent verifier disagree | Qualification fails; never select whichever result is permissive | Current workflow checks representative cases; broader differential testing proposed |
| Downstream caller interprets profile fit as action authorization | Consumer integration must require an independent grant | Explicit nonclaim now; end-to-end consumer enforcement not yet qualified |

Assume the candidate request may be malicious. Also assume that a caller who can replace both the profile and the expected digest controls the pin channel; the current SHA-256 mechanism cannot defend against compromise of that channel. That is a boundary to document, not a cryptographic weakness in SHA-256.

## 4. Benchmark suite and proposed gates

All thresholds below are *proposed acceptance criteria*. First record a reproducible baseline and environment; do not report a target as a result.

### A. Semantic mutation adequacy

- Keep all current 256 combinations of the eight binary dimension mutations.
- Add boundary mutations for empty sets, exact interval endpoints, one-second widening, duplicate identifiers, Unicode/escaped JSON strings, null/type substitutions, unknown enum variants, missing fields, and maximum-length identifiers.
- Maintain a hand-curated high-severity mutation set. Target **100% rejection** of each known forbidden semantic mutation.
- For a broader generated corpus, target **zero observed false accepts in at least 100,000 reproducibly seeded cases**, comparing the Rust evaluator with a separately implemented oracle. Publish the seed, generator version, corpus digest, and minimized counterexamples.
- Interpret zero failures only as corpus evidence, not proof of zero real-world risk.

### B. Differential and metamorphic testing

Check the following properties over generated valid structures:

- **No amplification:** any admitted candidate is a subset of the profile in every positive-scope dimension and preserves every profile-required control.
- **Mutation monotonicity:** adding an unapproved atom/scope or removing a mandatory control cannot change rejection to admission.
- **Meet separation:** mathematical meet success cannot override policy rejection.
- **Pin sensitivity:** changing any byte in the pinned profile without changing the expected digest must reject; changing only whitespace must also reject because this gate pins raw bytes.
- **Receipt determinism:** the same exact source, executable, request bytes, profile bytes, schema bytes, and arguments produce byte-identical receipts.
- **Oracle independence:** the second implementation must not import or call the Rust evaluator. Compare normalized violation codes and relevant values as well as the final disposition.

Target 100% agreement over the frozen conformance corpus. Any disagreement is a release blocker, even when both implementations reject.

### C. Parser and resource robustness

- Test duplicated and unknown options, absent values, option-looking values, missing required flags, malformed UTF-8/JSON, deeply nested inputs, and size boundaries at limit−1, limit, and limit+1.
- Fuzz the CLI parser, JSON schema boundary, and evaluator with a recorded seed/corpus. Required result: no panic, no silent option override, no acceptance following a parse error, and no unbounded read beyond configured limits.
- Run a sustained fuzz campaign (initial target: 24 CPU-hours per release candidate) and retain crashes, minimized inputs, and corpus digests. A fuzz-duration target is operational evidence, not a proof of absence of bugs.

### D. Scale and latency

Benchmark policy/profile sizes of 10, 100, 1,000, 10,000, and 100,000 atoms/scopes where the schema and test environment permit, plus request/profile byte sizes of 1 KiB, 64 KiB, 256 KiB, and the configured maximum.

Report p50, p95, p99, maximum latency, throughput, peak resident memory, build/toolchain, CPU, OS, dataset seed and digest. Separate cold-start/schema-compilation cost from steady-state evaluation. The initial CLI currently compiles/validates schemas per invocation, so a later long-running library/service benchmark should measure amortized and cold-start paths separately.

Proposed engineering guardrails to test against—not existing measurements:

- 1,000 atoms/scopes: p95 below 10 ms in a documented reference environment.
- 10,000 atoms/scopes: p95 below 100 ms in that same environment.
- Inputs at or below 1 MiB per document remain below 64 MiB peak resident memory for a single CLI invocation.
- No benchmark regression over 20% without an explained, reviewed trade-off.

If a baseline shows these are unrealistic, publish the measured distribution and revise the target explicitly rather than weakening correctness gates.

### E. Policy authority and lifecycle (next major slice)

Before using the profile gate as a production trust boundary, specify and test:

- a signed policy envelope with key identifier, issuer, intended domain/audience, profile digest, monotonic version/epoch, issuance and expiry, and schema version;
- a trust-root distribution mechanism independent of the candidate request;
- explicit key rotation and compromise response;
- authoritative revocation/supersession status and anti-rollback checks;
- fail-closed behavior during stale registry state or loss of required freshness evidence;
- a canonical receipt that binds the policy envelope, trust-root/registry snapshot, request, verifier/source identity, decision, and freshness status.

Do not use local wall-clock time as the sole freshness authority without a stated trust model. Offline verification should be tied to an explicit trusted registry snapshot/epoch or should return an indeterminate/rejected result when freshness cannot be established.

## 5. Comparison to established policy-engine practice

This gate is not a drop-in replacement for an authorization engine. The useful comparison is in assurance techniques:

- **NIST ABAC (SP 800-162)** describes decisions based on subject, object, operation, and environmental attributes evaluated against policy. Mycelix's claim-specific coupled atom and evidence/provenance dimensions extend the input model for epistemic claims, but should borrow ABAC's explicit subject/action/resource/context framing. Reference: https://csrc.nist.gov/pubs/sp/800/162/upd2/final
- **Cedar** separates authorization policy from application logic and supports schema-based policy validation before runtime use. Mycelix should retain explicit schema validation and add policy lifecycle and consumer-boundary tests; Cedar validation itself does not determine whether a policy is normatively correct. References: https://docs.cedarpolicy.com/policies/validation.html and https://docs.cedarpolicy.com/auth/authorization.html
- **Open Policy Agent (OPA)** has policy unit testing/coverage, performance benchmarking, and policy-bundle signature verification against out-of-band configured public keys. Mycelix should match this rigor in reproducible tests and performance reports, and should consider authenticated, freshness-aware policy envelopes rather than treating a digest as publisher authentication. References: https://www.openpolicyagent.org/docs/policy-testing, https://www.openpolicyagent.org/docs/policy-performance, and https://www.openpolicyagent.org/docs/management-bundles

These sources provide practices to benchmark against, not evidence that Mycelix currently outperforms those systems. Mycelix's potentially distinctive contribution is the explicit separation among claim-ceiling algebra, policy fit, evidence admission, and action authority—backed by deterministic receipts and independent verification.

## 6. Qualification record format

Every qualification run should publish:

- exact repository, commit SHA, workflow/run identity, toolchain and host details;
- source, request, policy, schema, oracle, corpus, seed and receipt digests;
- positive, negative, boundary, malformed, tampered-policy, stale-policy, revoked-policy and rollback case counts;
- expected vs observed dispositions and violation codes/values;
- false accepts, false rejects, oracle disagreements, crashes, timeout/OOM events;
- latency and memory distributions for every configured size;
- explicit skipped checks and the reason each was skipped.

A queued workflow is not a pass. A failed integration suite is not rescued by a successful isolated gate. A passing profile fit is not an authorization grant.
