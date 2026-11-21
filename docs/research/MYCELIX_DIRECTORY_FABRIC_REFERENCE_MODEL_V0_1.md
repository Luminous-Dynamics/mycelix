# Directory Fabric reference model v0.1 — semantics and qualification plan

**Status:** executable reference semantics authored; not yet compiler- or runtime-qualified.  
**Parent design:** PR #6430, architecture and 30-case synthetic corpus.  
**Implementation:** `tools/myc-dir-fabric-reference/` (dependency-free Rust library).  
**Claim ceiling:** finite model semantics only; no production enforcement, cryptographic verification, Holochain integration, protocol conformance or measured performance.

## 1. The next paradigm shift: prove the causal cut, not global freshness

A decentralized directory has no single global revision that safely orders every realm administrator, issuer, federation partner and revocation authority. A wall-clock timestamp, DHT arrival order, peer count, or the largest sequence number cannot establish a coherent authority history.

The new reference model makes a narrower, falsifiable claim:

> A PEP may pass a minimum-freshness precondition only when its local snapshot covers every required authority position, and every older position is tied to the current head by exact ancestry evidence checked by an independent verifier.

This matters because a higher sequence number is not itself an ancestry proof. The new tests explicitly reject a newer sequence with no matching older-position witness, reject same-sequence/different-digest forks, and return Incomparable when some required authority is behind while another independent authority has advanced.

The design direction after v0.1 is a **request-scoped causal cut**: carry only the authority dependencies required by the exact evaluation (realm policy, subject membership, issuer admission, relevant group/delegation edges, revocation state and federation contract), rather than requiring every peer or unrelated realm authority to be globally current. The dependency-set construction must itself be policy-bound and complete; it must not omit a deny-relevant or revocation-relevant source. This is a research direction, not an implemented guarantee.

This is complementary to rather than a replacement for established consistency systems. SpiceDB already offers per-request consistency including `at_least_as_fresh` using ZedTokens; fully consistent reads bypass caches and trade performance for stronger datastore freshness. OpenFGA also exposes a per-query consistency choice, with a higher-consistency path that reads the datastore rather than a cache. Mycelix's research question is whether independently governed authority histories can provide a verifiable causal lower bound without pretending there is one datastore revision or one global root. It is not yet evidence of superiority.

## 2. Why the first executable slice is deliberately small

The crate models three deterministic boundaries:

| Boundary | Reference behaviour | What remains outside |
|---|---|---|
| Causal frontier | Canonical vector of authority positions; exact ancestry witness required to cover an older position; forks and incomplete comparisons are incomparable | Signature verification, inclusion proofs, authority admission, merge protocol and Holochain replication |
| Snapshot lifecycle | Realm binding and persisted policy high-water rollback check | Durable storage, crash consistency, policy signature and recovery ceremony |
| Offline lease | Exact scope + policy epoch/digest + minimum authority frontier + clock interval + low-risk read-only restriction | Trusted clock attestation, risk classification provenance, session/token revocation and actual resource enforcement |
| Group projection | Bounded child-to-parent closure; cycles or budget overrun reject all output | Full policy evaluation, role composition and deny/allow subsumption |

The model intentionally does **not** implement complete authorization. Mycelix #4743 owns the one-coherent-authority-witness rule over the entire evaluation context. #4854 owns decidable constraint subsumption, normalization semantics and deny preservation. Directory Fabric should project realm membership and graph inputs into those authorities, not fork their semantics.

## 3. Critical implementation rules

### 3.1 Frontier semantics

Each component is an authority identifier, sequence and head digest. Components are canonicalized by authority identifier and duplicate authority entries are rejected.

For a required authority position:

- Same sequence and same digest: exact position matched.
- Older local sequence: Behind.
- Newer local sequence with exact older-position coverage evidence: covered.
- Newer local sequence without exact ancestry evidence: Incomparable.
- Same sequence with a different digest: Incomparable (possible fork).

Extra independent authority progress does not repair a missing or stale required component. Mixed progress is Incomparable. No LWW tie-break, timestamp ordering, or majority-of-peers shortcut exists in this model.

The constructor checks shape and exact binding only. The caller must validate the cryptographic ancestry proof and source authorization before supplying a covered position. This crate does not claim to validate that proof.

### 3.2 Snapshot and lease composition

A freshness result is a precondition, never an authorization Allow. A production PEP must require:

1. expected realm matches;
2. policy epoch is not below the persisted local high-water mark;
3. local frontier covers the caller's minimum frontier;
4. any offline lease matches the complete request scope, exact policy epoch and policy digest;
5. the snapshot covers the lease's authenticated minimum authority frontier;
6. the trusted-time interval is wholly within the lease interval and within its clock uncertainty budget;
7. the operation is independently classified as a low-risk read;
8. the full authorization evaluator returns a positive result for one coherent witness under #4743/#4854.

Missing or incomparable evidence is non-allow. High-risk and write operations are not eligible for this offline lease path.

The implementation exposes versioned, domain-separated canonical byte encodings for the frontier, evidence set, policy snapshot and offline lease. The evidence encoding includes exact covered authority positions; the lease encoding includes its exact policy epoch, policy digest and minimum frontier. These bytes make those fields available for a downstream signature/digest envelope. The crate itself does not hash, sign, authenticate the issuer, or verify the ancestry proof. A remote snapshot's canonical bytes intentionally exclude the local persisted high-water mark, because that value is local anti-rollback state rather than an issuer assertion.

### 3.3 Group graph semantics

Group expansion is a projection, not authorization. It has explicit maximum depth, node and edge budgets. Cycles and resource-budget exhaustion reject the entire operation, rather than silently truncating and returning a potentially over-permissive partial closure.

## 4. Industry comparison and where Mycelix could differentiate

| System | Existing strength | Correct comparison for Mycelix |
|---|---|---|
| Microsoft AD DS | Mature domains/forests, LDAP, Kerberos, replication, delegated administration and Windows logon | Compare Windows logon, group/ACL semantics, replication recovery, machine-account lifecycle and administrator workflow end-to-end. A Rust graph evaluator is not an AD DS replacement. |
| Microsoft Entra ID | Conditional Access combines identity, device, location, application and risk signals; session and token behaviour are integrated | Compare signal provenance, freshness, policy explainability, revocation-to-effect exposure and migration operations. Mycelix must not claim equivalent device/risk coverage from a claims mapper. |
| FreeIPA | Integrated Linux identity with LDAP, Kerberos, DNS and certificate/CA facilities | Compare full Linux domain operations, multi-master replication, renewal/recovery and client integration. |
| SpiceDB | Zanzibar-style relationship authorization; ZedTokens support `at_least_as_fresh` and exact-snapshot semantics | Compare the same graph and request corpus at matched consistency requirements. The research delta is multi-authority causal coverage plus local bounded leases, not basic relationship traversal. |
| OpenFGA | Relationship-based authorization with per-request consistency preferences | Compare equivalent authorization models, caches, check APIs, consistency settings and offered load. |
| Cedar / OPA | Mature local policy evaluation models and policy-tooling ecosystems | Compare local evaluator latency, policy validation, explainability, policy distribution/revision and adversarial semantic correctness. Reuse these strengths; do not invent a new DSL without a proven need. |
| SPIFFE/SPIRE | Workload identity, SVID issuance and trust-domain federation | Consume interoperable SVIDs and bundles; bind them to audience/trust domain. Do not replace workload identity with a Mycelix-specific token. |

The practical adoption path remains: read-only import → shadow decisions → SCIM provisioning/lifecycle → OIDC application bridge → SPIFFE workload bridge → one local Linux PEP → narrow production pilot. Kerberos, Windows logon, DNS/PKI and endpoint desired-state management remain separate workstreams.

## 5. Standards-first update

The SCIM adapter perimeter should pin the actual supported profiles, not merely say “SCIM 2.0”:

- RFC 7642/7643/7644 for the base provisioning model and protocol.
- RFC 9865 for cursor-based pagination.
- RFC 9944 for Device and EndpointApp resource extensions.
- RFC 9967 for asynchronous SCIM Security Event Token exchange.

These standards strengthen the path for device provisioning and change delivery. They do not turn a SCIM event receipt into proof that every session, refresh token, application worker and enforcement point has stopped accepting old authority. Measure the event-to-effective-enforcement path, not just HTTP acknowledgement.

For workload identity, use SPIFFE ID/SVID and trust-domain semantics. For application access, use OIDC/OAuth with current security guidance and audience binding. For fine-grained authorization, retain complete request-context checks; a signed identity or an access token does not become unlimited authority.

## 6. Qualification protocol: hard correctness gates before speed

### 6.1 Non-compensable correctness gates

A benchmark profile fails regardless of throughput if any of these occur:

- Any false Allow on frozen or held-out adversarial cases.
- A newer sequence accepted without exact verified ancestry coverage.
- A same-sequence fork or incomparable frontier treated as current.
- A missing or stale required authority component treated as absent revocation.
- Policy rollback below a persisted high-water mark accepted.
- An offline lease used for a different realm, principal, resource, action, audience, purpose or policy digest.
- An uncertain time interval that crosses expiry treated as valid.
- A high-risk/write operation accepted through the low-risk offline path.
- A cyclic or over-budget group graph returns a partial successful closure.
- Private DHT payloads, links, logs, telemetry or receipts disclose prohibited attributes.

Zero false acceptance is a release gate, not an average score.

### 6.2 Workload matrix

| Profile | Principals | Relationship edges | Real conductors | Purpose |
|---|---:|---:|---:|---|
| Smoke | 1,000 | 10,000 | 3–5 | Basic multi-agent correctness |
| Development | 10,000 | 100,000 | 20 | Churn, indexing and cache behaviour |
| Qualification | 100,000 | 1,000,000 | 100 | Multi-agent scale and fault injection |
| Stress | 1,000,000 | 10,000,000 | 500+ | Capacity and failure-domain study |

Simulated peers must be reported separately from real conductors. No capacity claim follows from a simulated topology.

### 6.3 Metrics

Record at least:

- Local policy check p50/p95/p99/max at fixed offered load, including cold and warm caches.
- Frontier comparison latency by number of authorities and number of ancestry witnesses.
- Group expansion latency, allocations, memory and rejected graph-budget pressure by nodes/edges/depth.
- Decision throughput and saturation behaviour; use open-loop offered load to avoid coordinated omission.
- Revocation-to-last-accepted-effect time for every PEP, session, token, worker, connector and offline device.
- Clock uncertainty and lease-expiry boundary outcomes.
- DHT gossip bytes, validation CPU, peer churn, replication lag and convergence under partitions.
- SCIM cursor recovery, duplicate/out-of-order event handling, deprovision-to-effective-denial latency and residual active sessions.
- Restart/recovery, key rotation and admin recovery time.
- Operator labour, enrolment time, helpdesk recovery and lockout rates.

Use at least 10 independent repetitions for headline numbers; publish raw samples and distributions. Bind results to exact source/tree SHAs, dependency lock, compiler, runtime, DNA hash, verifier identity, trust roots, topology, hardware, clock profile, dataset generator and fault schedule. Do not average p99 values across runs; report per-run tails and aggregate distribution.

### 6.4 Provisional targets (not measured)

| Metric | Research target |
|---|---:|
| Warm local PEP decision, excluding external IdP/MFA | p95 ≤10 ms; p99 ≤25 ms |
| Pure frontier comparison, up to 64 authority components | p95 ≤250 μs |
| Pure lease check | p99 ≤50 μs |
| False authorization on frozen and held-out controls | 0 |
| Known-answer decision agreement with independent oracle | 100% |
| Sensitive disclosure through public DHT/logs/telemetry | 0 |
| High-risk action after required freshness/lease expiry | 0 accepted |

The micro-operation targets are hypotheses to test on declared reference hardware, not current results or promises. First publish unoptimized baseline measurements. Change architecture, not invariants, if the targets are missed.

### 6.5 Matched baselines

- **Local policy:** Cedar and OPA, plus a no-cache baseline, using the same request/context corpus and policy semantics.
- **Relationship authorization:** SpiceDB with `at_least_as_fresh` ZedTokens and OpenFGA with its higher-consistency path, using equivalent tuples, group depths and freshness requirements.
- **Enterprise integration:** AD DS, Entra and FreeIPA on their real supported surfaces. Measure end-to-end lifecycle, user/admin experience, session/token revocation and recovery; do not compare them to a microbenchmark of this library.
- **Workload identity:** SPIRE/SPIFFE for SVID issuance, rotation, trust-domain federation and workload authentication; compare only the shared workload-identity responsibilities.
- **Distributed substrate:** exact Holochain conductor/HDK/Lair tuple, real conductors, fault schedule and DHT observation points.

For all comparisons, hold constant request semantics, principal/group corpus, freshness requirements, hardware class, offered load, cache policy and fault schedule. Report correctness, latency, resource cost, availability and operational complexity as separate dimensions; do not collapse sovereignty and security into one weighted score.

## 8. Observed upstream Holochain runtime blockers (checked 21 November 2026)

Before selecting the Directory Fabric runtime tuple, the following open upstream reports must be reproduced against the exact chosen release. They are reporter issues, not Mycelix reproductions and not proof that every supported release is affected:

- [holochain#6071](https://github.com/holochain/holochain/issues/6071): a multi-function zome-call grant reportedly admitted calls for some seconds after revocation in 4/6 trials on 0.7.0/0.7.1. The reporter points to #6063 merged to `develop` and requests a 0.7-line backport. The issue reports single-function grants refused promptly; per-function grants are a workaround, not a verified release fix.
- [holochain#6062](https://github.com/holochain/holochain/issues/6062): gossip exchanges with about 10,900+ offered op IDs reportedly hit SQLite's bind-variable limit and can fail to converge under large two-way divergence. A 1 MiB gossip budget was reported as a workaround in one controlled test (convergence in roughly eight minutes); do not present that result as a general convergence guarantee.
- [holochain#6065](https://github.com/holochain/holochain/issues/6065): installation using an existing agent key reportedly fails with roughly 40+ Lair key entries because the key-list response exceeds an 8 KiB frame. Key inventory failure must remain unavailable/Indeterminate; it must never silently mint or substitute an agent identity.
- [holochain#6066](https://github.com/holochain/holochain/issues/6066): 0.7.1 time-slice reads reportedly scan the Action table and can keep conductors busy at roughly 100k ops, with long restart/startup and first-call delays. The reporter's indexed-query patch showed large improvements in its test environment, but it is not treated as an upstream release fix until incorporated and validated on the selected binary.

A dedicated qualification gate is [MYC-DIR-HC-QUAL-001 / issue #6435](https://github.com/Luminous-Dynamics/mycelix/issues/6435). It requires a component-level version matrix (conductor, `hc`, HDK, HDI, Holochain type crates, JS/Rust clients, Lair, Kitsune2, Rust/WASM build target and mirrored manifests/lockfiles), reproducible tests for grant revocation, partition convergence, large Lair inventories and high-cardinality restart/idle behaviour, and exact binary/source receipts. Directory Fabric must not integrate host-grant or DHT freshness assumptions until this gate is satisfied.

The Mycelix identity manifests need a careful component-level reconciliation: the top-level README's version label and the standalone/workspace Identity manifest pins do not identify one coherent tested runtime tuple. No single "Holochain version" label is sufficient for qualification.

## 9. Next gates

1. Run rustfmt, Cargo test, Clippy and mutation tests at the exact new head.
2. Independently review the ancestry-proof format and the completeness of the request-scoped dependency cut.
3. Add an independent oracle for the snapshot/lease/group semantics and a held-out corpus.
4. Add storage/restart tests for the persisted policy high-water mark.
5. Qualify trusted-time uncertainty and lease invalidation across sessions, tokens, workers and actual PEPs.
6. Select and publish a component-level Holochain compatibility matrix before writing zomes or conductor integration.
7. Connect SCIM/OIDC/SPIFFE only after read-only import and shadow comparison are stable.
8. Run matched baselines before making any SOTA or product superiority claim.

Queued, skipped or absent hosted checks are not passes. Source present, structurally reviewed, compiled, test-executed, runtime-qualified and physically measured remain different evidence states.
