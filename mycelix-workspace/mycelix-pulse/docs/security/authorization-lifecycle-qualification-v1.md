# Pulse Authorization Lifecycle Qualification v1

**Profile:** `PULSE-AUTH-LIFECYCLE-QUAL-001`  
**Status:** Research corpus; not runtime-qualified  
**Dataset:** Synthetic only; no production effects  
**Created:** 4 November 2026

## Purpose

Qualify authorization as an end-to-end distributed lifecycle, not as a collection of isolated allow/deny unit tests. The corpus forces each observation into six separate planes:

1. **Host decision** — what the Holochain conductor actually authorized or rejected for the exact incoming call.
2. **Domain result** — whether coordinator/application policy executed, rejected or did not run.
3. **Source-chain result** — whether an action was actually authored.
4. **Integrity validation** — whether an independent validator accepted or rejected the authored operation.
5. **Projection state** — whether DHT/UI-derived state is current, stale, conflicting or unknown.
6. **Observer result** — what the caller actually learned, including no acknowledgement and unknown outcome.

These planes are not interchangeable. In particular:

- a DHT record saying `active` is not a host authorization decision;
- a coordinator result is not proof that an operation reached a source chain;
- local source-chain authoring is not proof of remote DHT acceptance;
- a lost response is not proof of denial or non-application;
- a DirectSignal with no acknowledgement is not proof that the recipient rejected it;
- a content digest is not issuer authentication or proof that its contents are true.

## Paradigm shift: authority as a typed, evidence-carrying lifecycle

The proposed lifecycle is:

```text
authority intent
  -> exact grant identity and scope
  -> conductor authorization
  -> domain transition
  -> source-chain action
  -> independent validation
  -> DHT integration / projection
  -> observed outcome
  -> revocation / expiry / migration
  -> recovery and independently verified history
```

The system must preserve an evidence record for each transition. An application capability record is a projection and audit reference; it is not a substitute for a real host CapGrant. A domain policy may independently reject an operation even after the conductor authorizes the zome call.

### Conductor authority is necessary, but not a universal object ACL

The Holochain conductor enforces whether an incoming call may invoke a granted zome function for the target cell. That is a host boundary, not a generic mailbox-row or document-object policy engine.

Therefore Pulse must preserve two separate checks:

- **Function invocation authority:** the conductor checks the incoming call against the exact cell's host grant, caller key/secret constraint and function set.
- **Domain-resource authority:** the invoked coordinator checks whether that authenticated principal may access the exact mailbox/message/resource and whether the requested state transition is permitted.

A narrow host function allowlist reduces reachable attack surface. It does not make a broad function safe if that function omits record-level participant, owner, membership, generation or purpose checks. Conversely, a DHT membership record cannot manufacture a host grant that the conductor does not recognize.

The same separation applies to integrity validation: validators decide whether authored data transitions obey the DNA's rules; they do not retroactively prove that an incoming zome call was authorized at the host boundary.

### Exact identities

Future runtime traces should bind (without exposing secrets):

- DNA and cell identity;
- exact grant Create/Update ActionHash;
- assigned grantee AgentPubKey;
- grant tag and secret fingerprint reference;
- zome and exact function allowlist;
- authority generation / revocation generation;
- runtime and protocol version;
- logical operation ID and separate attempt ID;
- source-chain action hash, validation disposition and projection version where observed.

No raw CapSecret, message plaintext, private payload or unnecessary stable identifier belongs in routine traces.

### Concurrency rule

A call that overlaps revocation must not be assigned an order using wall-clock timestamps alone. Capture the actual conductor decision, durable revoke event and causality evidence. Until the runtime's linearization point is established, classify the ambiguous outcome as `UNKNOWN`; do not retrofit a favorable sequential explanation.

### Retry and recovery rule

A logical operation ID remains stable across retries; attempt IDs change. State-changing effects need durable idempotency/deduplication. The permitted claims are at-most-once logical effect and conditional progress under stated liveness assumptions—not exactly-once network delivery.

### Independent validation rule

For adversarial mailbox transitions, the test coordinator should be deliberately permissive while the production integrity WASM remains unchanged. The test must separately prove:

1. the malicious coordinator authored an action;
2. the receiving validator evaluated that action;
3. the production validator rejected it;
4. the remote observer's canonical mailbox projection did not change.

A coordinator-side error without these observations is insufficient.

## Corpus

The companion JSON contains 28 synthetic scenarios across:

- conductor capability and exact scope;
- revocation and overlapping calls;
- grant-creation retry idempotency, revocation retries, crashes and recovery;
- 0.7.0-to-0.7.1 migration and DirectSignal grants;
- independent validation, roots and fork ambiguity;
- policy mapping and observability.

Each scenario provides an event sequence, explicit happens-before edges where known, expected observations by plane, and assertions. The schema is closed-world: unknown fields fail validation. The corpus includes no raw secrets or personal data.

## Finite-state model artifact

The branch now includes a small TLA+ contract model:

- `docs/security/PulseAuthorityLifecycle.tla`
- `docs/security/PulseAuthorityLifecycle.cfg`
- `docs/security/PulseAuthorityLifecycle-mutant.cfg` — deliberate projection-as-authority mutant
- `docs/security/PulseAuthorityLifecycle-grant-idempotency-mutant.cfg` — deliberate duplicate-grant-on-retry mutant

It models two grant identities, two logical issuance requests, grantor/grantee/outsider actors, host grant state, recipient claims, application revocation state, stale/current projections, grant retries, acknowledgement loss, host revocation, projection convergence, caller outcomes and one logical domain effect per operation.

The duplicate-grant mutant enables `DuplicateGrantForRetry`: after a grant's acknowledgement is lost, the retry mints a second CapGrant for the same logical request, overwrites the operation-to-grant binding and leaves the original live. `GrantIdentityInjective` should catch the orphaned first grant. The safe config disables this transition; the grant-idempotency mutant config enables it and checks the identity invariant.

The normal configuration deliberately gives one logical operation domain permission and leaves the other domain-rejected. This encodes the distinction between host function authorization and resource/domain policy: a valid host grant is necessary, but not sufficient, for a domain effect.

The unsafe mutant switches host decisions to the application projection. Its expected counterexample schedule is:

```text
IssueGrant -> DeliverClaim -> RevokeGrant
           -> (projection remains active)
           -> Call(grantee, same grant)
```

The safe model must deny that post-revoke call based on host grant state. The projection-authority mutant config should violate `StaleProjectionDoesNotAllow` and `NoProjectionAuthority`. The grant-idempotency mutant config should independently violate `GrantIdentityInjective` after creating two host grants for one logical request. The timeout action changes only caller-observed outcome to `unknown`; it does not create or undo host grants or domain effects.

**Executed bounded result:** TLC 1.8.0 build `2026.08.11.125311` (JAR SHA-256 `ab323b79802aedc3203b3f9af37c6aca3ed43f4e0225b36f2aa77b26de46c05f`) completed the safe configuration with 83,941 generated states, 10,249 distinct states and an empty state queue. The projection-authority mutant produced a counterexample violating `NoProjectionAuthority`; the duplicate-grant retry mutant produced a counterexample violating `GrantIdentityInjective`. Exact workflow run: https://github.com/Luminous-Dynamics/mycelix/actions/runs/41834410638. The uploaded artifact contains the logs and counterexample traces.

This is a finite abstract model only: two grant identities, two logical requests, three actors and two attempts. It is not a conductor test or a general proof of distributed-system correctness. The constant-level configuration checks emit TLC informational warnings; they evaluated true. The exact human-readable receipt is `docs/security/pulse-authority-model-check-2026-11-04.md`.

Next expand the model with two-conductor validator acceptance, concurrent call/revoke orderings, crash points, conflicting roots and partition schedules, while retaining negative controls.

## Malicious mailbox coordinator model

The branch now also includes `PulseMailboxValidation.tla` plus three TLC configurations:

- `PulseMailboxValidation.cfg` — safe original-state validation;
- `PulseMailboxValidation-proposed-state-mutant.cfg` — proposed-membership self-authorization mutant;
- `PulseMailboxValidation-field-mutation-mutant.cfg` — protected-field smuggling mutant.

The abstract test coordinator can author any proposal. The receiving validator is separate and checks original committed state. It models the two relevant source-level threats:

1. **Proposed-state self-promotion:** an attacker appends itself as Admin and the mutant validator derives authority from the proposed member list rather than the original owner.
2. **Field smuggling:** an otherwise legitimate owner member append also changes an immutable mailbox field such as name/settings; the mutant ignores that mutation.

The safe model requires the original owner for the direct transition, one append only, the correct author provenance, and no protected-field mutation. A remote observer's projection is a subset of validator-integrated state; authoring an action does not make it accepted or visible. A valid delegated Admin transition is intentionally not modeled as available until the controller-signed lease protocol is implemented and qualified.

A third small model now makes the AuthorityCheckpoint proposal executable:
- `PulseAuthorityCheckpoint.tla`
- `PulseAuthorityCheckpoint.cfg` — intended freshness-constrained reader
- `PulseAuthorityCheckpoint-bypass-mutant.cfg` — reader that ignores the required checkpoint and labels its stale projection trusted

It separates host CapGrant state, canonical authority epoch, local projection epoch, a caller's minimum required epoch, the actual conductor probe result and the application read classification. After host revocation commits, the observer may still hold an old active projection; a constrained read must return `UNKNOWN` until the projection reaches the checkpoint. The mutant's false-fresh response must violate `NoFalseFreshAuthority`. The checkpoint never creates or changes a host grant.

The same pinned TLC workflow ran both checkpoint configurations in run #56: safe model PASS (27 generated / 17 distinct states, queue exhausted); checkpoint-bypass mutant caught by `NoFalseFreshAuthority` (10 generated / 9 distinct states). Exact run: https://github.com/Luminous-Dynamics/mycelix/actions/runs/41856892659. Receipt: `docs/security/PulseAuthorityCheckpoint-model-check-2026-11-04.md`.

This validates only the bounded abstract contract; it does not establish that Holochain already supports a ZedToken-equivalent API or that the production runtime implements the checkpoint.

The TLC workflow now runs all three mailbox configurations in addition to the authorization lifecycle configurations. **The first exact run (#32) passed the safe mailbox model** with 37 generated / 20 distinct states and no invariant error. The proposed-state mutant was caught by `OnlyOriginalOwnerMayUpdate`; the protected-field mutant was caught by `ProtectedFieldsRemainImmutable`. Exact run: https://github.com/Luminous-Dynamics/mycelix/actions/runs/41852865450. Full tool/source/state receipt: `docs/security/PulseMailboxValidation-model-check-2026-11-04.md`. The corresponding real two-conductor experiment remains open in [#5781](https://github.com/Luminous-Dynamics/mycelix/issues/5781).

## Remote signal trust-boundary model

A fourth finite model now captures notification authenticity and replay safety:

- `PulseSignalTrust.tla`
- `PulseSignalTrust.cfg` — safe evidence-backed consumer;
- `PulseSignalTrust-forged-payload-mutant.cfg` — sender-controlled payload directly applied as trusted UI state;
- `PulseSignalTrust-replay-mutant.cfg` — duplicate logical event applied more than once.

The model separates transport caller, payload-claimed source, claimed ActionHash, exact canonical record author/value, record availability/validity, evidence verification, event ID, deduplication state and trusted UI effect. A remote signal may wake a client, but only a matching transport identity and exact backing record can update trusted state. A replay of the same logical event must not produce a second effect.

The safe configuration must exhaust its bounded state space. The forged-payload mutant must violate `NoUnverifiedSignalMutatesTrustedState`; the replay mutant must violate `AtMostOneEffectPerLogicalEvent`. The same pinned TLC workflow now includes these three configurations. No model-check result is claimed until an exact run completes.

## Signal protocol source auditor

The new standalone Rust source tool at `tools/pulse-signal-contract-audit/` uses `syn` to parse the four active Rust enums and extracts Serde representation + variant field sets. It compares those results to the matching TypeScript unions and both sets of category filters in `SignalHub.ts`. Its JSON report lists missing/stale variants, field drift, filter drift and codec incompatibility. Default execution is fail-closed; CI initially uses explicit report-only mode to preserve the baseline report without treating current source drift as a pass.

This is source inventory, not a notification-authentication fix. The expected initial findings are:
- Mail and Trust are adjacent-tagged as `{type, data}`, not flat payloads;
- Sync and Federation use externally tagged enum encoding;
- the current SignalHub decoder expects a flat `payload.type` object;
- TypeScript unions and SignalHub filters are incomplete/stale relative to Rust variants;
- individual event fields differ between Rust and TypeScript.

The next repair phase should establish one versioned event envelope, generate Rust/TypeScript fixtures from a single schema, and classify remote payloads as unverified hints until exact record retrieval and validation succeed. Keep that client-protocol work separate from authorization enforcement.

## Proposed remote signal envelope v1

A strict draft schema has been added at `docs/security/remote-signal-envelope-v1.proposed.schema.json`. It is explicitly a proposal, not a production wire-format change.

The key protocol distinction is:

- **Transport identity:** obtained from Holochain's authenticated callback metadata, never from a sender-controlled `source_agent` payload field.
- **Wire hint:** event ID, namespaced event type, schema version, optional exact source ActionHash, optional logical operation/attempt IDs, informational timestamp and the minimal event payload.
- **Local classification:** `VERIFIED_RECORD`, `AUTHENTICATED_PEER_HINT`, `UNVERIFIED_HINT`, `LOCAL_ONLY`, `UNKNOWN` or `REJECTED` is computed by the receiving code. It must never be asserted by the sender.
- **Evidence reference:** created locally after the receiver fetches and verifies the exact record; not accepted as sender proof.
- **Authority:** still enforced by host CapGrant plus integrity/domain validation. Neither the envelope nor an AuthorityCheckpoint is a permission token.

The schema forbids sender-supplied trust/verification fields, source-agent claims and secret-like keys at the envelope level. It is not yet an event-complete schema: event-specific payload schemas, size bounds, serialization fixtures, compatibility policy and two-conductor tests are prerequisites for implementation. The schema was parsed successfully as JSON after creation.

## Qualification stages

| Stage | Required evidence | Claim ceiling |
|---|---|---|
| 0. Corpus/schema | Schema validation, unique IDs, exact event references, canonical bytes | Corpus integrity only |
| 1. Pure contract | Rust tests for plane separation, identities, Unknown semantics, idempotency key rules | Contract model only |
| 2. Abstract model | Bounded exhaustive interleaving/model check; negative mutants for each invariant | Encoded finite model only |
| 3. Conductor runtime | Real signed AppRequest::CallZome, host grant/revoke, exact function scope, restart tests | Exact runtime/commit and tested bounds |
| 4. Multi-conductor | Independent validators, malicious coordinator, forks, partitions, DHT convergence | Exact topology, workload and faults |
| 5. Independent replay | Second verifier consumes redacted traces and reproduces dispositions/counterexamples | Exact trace and verifier version |

A stage cannot inherit a PASS from a later or unrelated stage. Every receipt binds the exact source SHA and artifact/runtime identities.

## Hard safety gates

These are qualification requirements, not measured results:

- zero unauthorized host accepts across all negative controls;
- zero protected domain effects after confirmed host revocation;
- zero invalid remote updates accepted by the production validator;
- zero duplicate logical state changes for replayed operation IDs;
- zero authority inferred from stale, missing or conflicting projections;
- zero timeout-to-denial or timeout-to-success fabrication;
- zero raw capability secrets in logs, signals, public records or evidence artifacts;
- 100% of test events classified as observed, explicitly unknown, or not applicable; no silent omission;
- all unsupported access semantics reject before any source-chain action.

Finite test counts cannot prove a universal zero-failure rate. Report denominators, confidence bounds, schedule coverage and mutation sensitivity.

## Benchmark protocol

### Workload matrix

Start with:

- agents: 2, 10, 50, 100; 500 stretch;
- active grants: 1, 100, 1,000, 10,000; 100,000 stretch;
- concurrent grant/revoke/call streams: 1, 10, 100;
- source-chain/DHT history: 1,000 to 1,000,000 relevant actions;
- network loss: 0%, 1%, 5%;
- partitions: 0, 30, 120 seconds;
- restarts: before and after each durable lifecycle boundary;
- payloads: empty diagnostic, 1 KiB, 64 KiB and 1 MiB where the operation legitimately carries data.

Run at least 30 independent process-level repetitions per configuration. Publish raw per-run observations and at least 100 observations before reporting within-run percentiles. Separate warm/cold cache, local and remote call paths, and healthy versus faulted topology.

### Statistical and reproducibility rules

A finite test count must never be translated into a universal zero-failure claim.

- For a **zero-accept** result over (n) independent Bernoulli trials, report the one-sided 95% upper confidence bound (1-0.05^{1/n}) (approximately (3/n) for large (n)). At 10,000 independent trials this is about 0.03%, not zero.
- Distributed schedules are often correlated. Do not count 10,000 retries from one process as 10,000 independent samples. Report unique scenario seeds, independent process restarts, schedule classes, fault strata and repeat counts separately.
- Keep the primary safety result deterministic: every explicitly enumerated negative control and mutation must be rejected. Statistical sampling supplements this; it does not replace adversarial coverage.
- Report latency percentiles with sample counts and uncertainty. Bootstrap over independent process-level runs (not individual requests alone) so one unusually healthy process does not dominate the interval.
- Preserve raw per-run traces, environment manifests, source/WASM/DNA hashes, workload seed, fault schedule and verifier build. Publish failed runs and censored observations; do not discard timeouts as outliers.

### Freshness-carrying authorization context

A useful next paradigm shift is to make *causal freshness* explicit without making a DHT token or application record authoritative.

SpiceDB provides a useful design reference: a write returns an opaque ZedToken, and a subsequent read can require `at_least_as_fresh` while still using caches. A fully-consistent query bypasses those caches and has a latency/resource cost. OpenFGA likewise exposes per-request higher-consistency versus minimize-latency modes. These systems demonstrate that freshness is an explicit API contract, not a property that can be inferred from a recent timestamp. Pulse should borrow the *causal watermark idea*, not copy their server-side authorization model.

For Pulse, prototype a redacted **AuthorityCheckpoint** containing:
- exact DNA/cell identity and protocol generation;
- exact host CapGrant ActionHash (or an explicit no-grant/revoked disposition);
- relevant source-chain transition ActionHash and action sequence;
- logical operation ID and separate attempt ID;
- a causal watermark / parent checkpoint reference;
- projection observer and verifier identity, plus VERIFIED / STALE / CONFLICTING / UNKNOWN disposition.

The checkpoint is evidence about a particular observed state, not a bearer token and not a grant. A protected call remains authorized by the conductor's current CapGrant and the function's domain-resource checks. A checkpoint can let a caller request a read at least as fresh as a known transition; if the peer cannot establish that freshness, it returns UNKNOWN rather than silently using an older active membership projection.

**Research experiment:** after revoke / membership removal / policy downgrade, record the mutation checkpoint; issue reads through stale, lagging and converged observers; compare (a) unconstrained projection reads, (b) checkpoint-constrained reads and (c) direct conductor probes. Measure false-fresh responses, unknown rate, added network/CPU cost and convergence. A good design drives false-fresh responses to zero while preserving the option of fast explicitly-stale UI reads.

### Metrics

| Metric | Definition |
|---|---|
| Host revocation enforcement latency | Durable conductor revoke completion to first verified host denial of the same grant |
| Projection convergence latency | Host revoke completion to DHT/UI/remote projection showing the matching revocation |
| Authorization latency | Call issue to conductor's observed allow/deny disposition |
| Domain completion latency | Call issue to durable domain outcome, not merely host authorization |
| Retry amplification | Physical attempts / logical operation IDs |
| Duplicate effect rate | Repeated durable effects per logical operation ID |
| Recovery time | Restart to a verified, consistent host/domain disposition |
| Partition safety | Unauthorized accepts during partition; must be zero |
| Unknown accuracy | Unknown outcomes correctly retained rather than fabricated |
| Resource cost | CPU, peak RSS, disk writes/bytes, network bytes and energy if instrumented |

**Do not combine host enforcement freshness and projection knowledge freshness into one “revocation latency.”** The first is a safety property; the second is a convergence property.

### Provisional performance objectives

The following are targets to test, not claims:

- local host revocation denial p95 ≤ 50 ms on a declared LAN;
- healthy-LAN projection convergence p95 ≤ 1 s;
- ≥20% p95 lifecycle improvement over a matched, semantically equivalent baseline;
- zero safety regressions at every tested load and fault level.

Freeze targets after the first exact matched baseline. If the host's documented execution boundary cannot support a meaningful 50 ms measurement, revise the target transparently rather than moving the measurement start point.

## Industry and research comparison

| Comparator | Published/observable behavior | Pulse's fair comparison |
|---|---|---|
| Google Zanzibar | 2019 paper reports trillions of ACLs, millions of authorization requests/sec, p95 below 10 ms and >99.999% availability over three years | Do not claim raw scale superiority. Compare semantic freshness, local execution, offline/partition behavior, privacy and independent evidence |
| SpiceDB | Per-request `minimize_latency`, `at_least_as_fresh` via ZedTokens, `at_exact_snapshot`, and `fully_consistent`; freshness has explicit cache/latency cost | Prototype causal AuthorityCheckpoint reads while retaining conductor grants as enforcement. Compare freshness guarantees, cache hit ratio, p95/p99 and datastore/network cost |
| OpenFGA | Per-request `MINIMIZE_LATENCY` versus `HIGHER_CONSISTENCY`; higher consistency can bypass cache | Compare request-scoped freshness modes and truthful UNKNOWN states, not just average latency |
| UCAN | Portable, cryptographically verifiable attenuated delegation with hierarchical authority and an invocation/policy model | Compare delegation depth, proof bytes, verification cost, key rotation, offline verification and revocation behavior |
| Biscuit | Attenuated bearer tokens with revocation identifiers and verifier-side revocation lists; derived-token revocation can follow parent identifiers | Compare exact per-grant revocation, list distribution delay, offline behavior and cost of revocation freshness |
| Holochain native capability grants | Conductor-enforced incoming zome-call authorization with exact caller/secret/function scope; 0.7.0→0.7.1 has a grant-format break | Qualify exact tuple, real assigned grant/claim/revoke and DirectSignal grants on fresh conductors; do not infer grant compatibility across releases |

No cross-system latency claim is valid without matching semantics, hardware, topology, dataset cardinality, cache state, consistency mode, fault assumptions and measurement boundaries.

## Highest-value research questions

1. **Revocation linearization:** What exact event is the enforcement boundary for a concurrent host call and grant deletion? Determine by instrumented runtime traces, not timestamp inference.
2. **Authority epochs:** Can an explicit monotonically increasing authority generation prevent stale claim/projection resurrection across grant updates and 0.7.0→0.7.1 re-issuance?
3. **Fork semantics:** What deterministic, separately authorized protocol resolves competing roots without privileging DHT arrival order?
4. **Evidence minimization:** What is the minimal redacted event record that lets an independent verifier reproduce authorization histories without exposing capability secrets, message metadata or unnecessary identities?
5. **Compositional verification:** Can the Rust history checker, bounded model and Sweettest trace exporter share a frozen event vocabulary while keeping the reference checker independent?
6. **Failure-domain calibration:** When does a remote peer's non-observation indicate delay, partition, validator rejection or application-level denial? Preserve these as distinct outcomes.

## Required next implementation increments

1. Export the six-plane event trace from the real 0.7 canary without recording secrets.
2. Build an independent trace/schema verifier and deterministic canonical receipt.
3. Add the malicious-coordinator/remote-validator Sweettest using production integrity WASM.
4. Instrument host grant revocation and protected call overlap to establish the actual ordering boundary.
5. Port the exact same scenarios to the accepted 0.7.1 tuple only after the release profile and canonical-source parity gate are accepted.
6. Extend #5091's bounded model with authorization grant/revoke/claim states, crash points and unknown outcomes; do not merge delivery and authorization into one state machine.

## Sources and provenance

- Pulse 0.7.0 migration canary: https://github.com/Luminous-Dynamics/mycelix/pull/4891
- Holochain 0.7.1 migration profile: https://github.com/Luminous-Dynamics/mycelix/pull/5718
- Holochain 0.7.1 qualification contract: https://github.com/Luminous-Dynamics/mycelix/issues/5719
- Holochain workspace closure auditor: https://github.com/Luminous-Dynamics/mycelix/issues/5720
- Pulse bounded delivery model-checking work: https://github.com/Luminous-Dynamics/mycelix/issues/5091

### External references for the comparison matrix

- Holochain capability model and host enforcement: https://developer.holochain.org/build/capabilities/ and https://developer.holochain.org/concepts/8_calls_capabilities/
- Holochain 0.7.0 → 0.7.1 breaking changes, including grant shape, re-issuance and DirectSignal grants: https://developer.holochain.org/resources/upgrade/upgrade-holochain-0.7/
- Holochain 0.7 compatibility table: https://developer.holochain.org/resources/compatibility/holochain-0.7/
- Holochain roadmap snapshot (3 November 2026): https://www.holochain.org/roadmap/
- Zanzibar paper (USENIX ATC 2019): https://research.google/pubs/zanzibar-googles-consistent-global-authorization-system/
- SpiceDB consistency modes and read-after-write ZedTokens: https://authzed.com/docs/spicedb/concepts/consistency and https://authzed.com/docs/spicedb/concepts/read-after-write
- OpenFGA per-request consistency modes: https://openfga.dev/blog/query-consistency-options-announcement
- UCAN v1.0.0 and delegation: https://github.com/ucan-wg/spec and https://github.com/ucan-wg/delegation
- Biscuit revocation identifiers and verifier-side revocation lists: https://www.biscuitsec.org/docs/guides/revocation/
- Jepsen fault injection and concurrent-history checking: https://github.com/jepsen-io/jepsen and https://jepsen.io/analyses
- Porcupine linearizability checker: https://github.com/anishathalye/porcupine


- Holochain capability model and host enforcement: https://developer.holochain.org/build/capabilities/ and https://developer.holochain.org/concepts/8_calls_capabilities/
- Holochain 0.7.0 → 0.7.1 breaking changes, including grant shape, re-issuance and DirectSignal grants: https://developer.holochain.org/resources/upgrade/upgrade-holochain-0.7/
- Holochain roadmap snapshot (3 November 2026): https://www.holochain.org/roadmap/
- Zanzibar paper (USENIX ATC 2019): https://research.google/pubs/zanzibar-googles-consistent-global-authorization-system/
- SpiceDB consistency modes and ZedTokens: https://authzed.com/docs/spicedb/concepts/consistency and https://authzed.com/docs/spicedb/concepts/read-after-write
- OpenFGA query consistency modes: https://openfga.dev/blog/query-consistency-options-announcement
- UCAN v1.0.0 and delegation: https://github.com/ucan-wg/spec and https://github.com/ucan-wg/delegation
- Biscuit revocation identifiers and verifier-side revocation lists: https://www.biscuitsec.org/docs/guides/revocation/

The Zanzibar paper reports trillions of ACLs, millions of authorization requests per second, p95 latency below 10 ms and availability above 99.999% over three years of production use. These are Google's published 2019 production figures, not a directly comparable Pulse baseline. SpiceDB and OpenFGA expose explicit consistency preferences, illustrating the latency/freshness trade-off that the Pulse benchmark must report rather than hide. UCAN and Biscuit provide useful portable delegation, attenuation and revocation comparators, but use different verifier and revocation-distribution architectures.

The Holochain roadmap snapshot dated 3 November 2026 reports the 0.7 Wind Tunnel work at 77%, while Wind Tunnel Analysis and Wind Tunnel Core Scenarios are both shown at 0%. Treat the upstream Wind Tunnel work as an evolving substrate reference, not a complete matched Pulse 0.7.1 analytical benchmark.

This profile is a research specification and synthetic corpus. It does not claim conductor runtime correctness, malicious-validator resistance, cryptographic truth, linearizability, crash consistency, liveness, privacy, production readiness or benchmark superiority. Those claims require the corresponding later qualification stages.
