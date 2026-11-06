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
- grant-creation retry idempotency scoped to the authenticated grantor, revocation retries, crashes and recovery;
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

The scalar freshness model was checked in run #56: safe model PASS (27 generated / 17 distinct states); scalar checkpoint-bypass mutant caught by `NoFalseFreshAuthority`. Exact run: https://github.com/Luminous-Dynamics/mycelix/actions/runs/41856892659. Receipt: `docs/security/PulseAuthorityCheckpoint-model-check-2026-11-04.md`.

**Important limitation:** the scalar model is a preliminary contract only. Equal epoch numbers do not prove two views descend from the same source-chain transition. A foreign or conflicting history can share the same integer epoch.

### Lineage-bound AuthorityCheckpoint refinement

The stronger model is now in:
- `PulseAuthorityCheckpointLineage.tla`
- `PulseAuthorityCheckpointLineage.cfg` — safe lineage check
- `PulseAuthorityCheckpointLineage-bypass-mutant.cfg` — deliberately unsafe epoch-only reader

The checkpoint binds canonical DNA/cell identity, author/source-chain identity, exact transition ActionHash and epoch. A read is trusted only when the observer proves its projection descends from that checkpoint. An equal-height foreign or conflicting history remains UNKNOWN.

TLC run #134 checked the original lineage model:
- Safe: PASS, 31 generated / 14 distinct states, queue exhausted.
- Epoch-only bypass mutant: expected counterexample, `NoForeignLineageCanBeTrusted` violated (11 generated / 8 distinct states).

Exact run: https://github.com/Luminous-Dynamics/mycelix/actions/runs/41945984183. Receipt: `docs/security/PulseAuthorityCheckpointLineage-model-check-2026-11-05.md`. Artifact ID `13379245045`, ZIP SHA-256 `c56658f77793f63003611c01da60af059fe43d9a8103273093d083aa874d95`.

The exact same six-domain model suite was repeated in TLC run #144: its lineage-safe configuration again exhausted its 31-state / 14-distinct-state queue, and the epoch-only mutant again violated `NoForeignLineageCanBeTrusted` (11 / 8 states). Run #144 is the broader current suite at https://github.com/Luminous-Dynamics/mycelix/actions/runs/41992920357; its artifact records every safe and mutant log.

This is still a bounded abstract contract, not proof that Holochain exposes a global snapshot token or that its actual DHT can provide this ancestry proof. The production design must build the checkpoint from actual DNA/cell and source-chain evidence.

The TLC workflow now runs all three mailbox configurations in addition to the authorization lifecycle configurations. **The first exact run (#32) passed the safe mailbox model** with 37 generated / 20 distinct states and no invariant error. The proposed-state mutant was caught by `OnlyOriginalOwnerMayUpdate`; the protected-field mutant was caught by `ProtectedFieldsRemainImmutable`. Exact run: https://github.com/Luminous-Dynamics/mycelix/actions/runs/41852865450. Full tool/source/state receipt: `docs/security/PulseMailboxValidation-model-check-2026-11-04.md`. The corresponding real two-conductor experiment remains open in [#5781](https://github.com/Luminous-Dynamics/mycelix/issues/5781).

### Multi-chain causal-frontier refinement

A further model now explores two independent source-chain histories rather than collapsing them into one epoch:

- `PulseAuthorityCausalFrontier.tla`
- `PulseAuthorityCausalFrontier.cfg` — component-wise checkpoint validation
- `PulseAuthorityCausalFrontier-scalar-mutant.cfg` — unsafe aggregate-epoch comparison

The requested checkpoint is a vector frontier `(A:1, B:1)`. The deliberately misleading projection `(A:2, B:0)` has the same scalar sum but lacks B's required action and does not descend from the required A head. The safe reader checks each source-chain sequence, exact action identity and ancestry independently; otherwise it returns UNKNOWN. The scalar mutant compares only aggregate height and should be caught by `NoFalseFreshCausalFrontier`.

The model and workflow step are committed, but the first TLC run is pending. No result is claimed until the safe queue is exhausted and the scalar mutant is caught. This model does not assert that a Holochain conductor exports a global total order; it explicitly avoids inventing one across independent chains.

## Remote signal trust-boundary model

A fourth finite model now captures notification authenticity and replay safety:

- `PulseSignalTrust.tla`
- `PulseSignalTrust.cfg` — safe evidence-backed consumer;
- `PulseSignalTrust-forged-payload-mutant.cfg` — sender-controlled payload directly applied as trusted UI state;
- `PulseSignalTrust-replay-mutant.cfg` — duplicate logical event applied more than once.

The model separates transport caller, payload-claimed source, claimed ActionHash, exact canonical record author/value, record availability/validity, evidence verification, event ID, deduplication state and trusted UI effect. A remote signal may wake a client, but only a matching transport identity and exact backing record can update trusted state. A replay of the same logical event must not produce a second effect.

TLC run #144 executed all three configurations with the pinned 1.8.0 build:
- Safe model: PASS, 21 generated / 12 distinct states, queue exhausted.
- Forged-payload mutant: expected counterexample, `NoUnverifiedSignalMutatesTrustedState` violated (7 / 7 states).
- Replay mutant: expected counterexample, `AtMostOneEffectPerLogicalEvent` violated (23 / 13 states).

Exact run: https://github.com/Luminous-Dynamics/mycelix/actions/runs/41992920357. Artifact ID `13399540056`, ZIP SHA-256 `980b8db7abf53bfbae94c9bd9d69d6483ccdd73626b9371e45df3b8b0783c5a2`. This is a finite abstract model; it does not prove a real remote callback or UI consumer authenticates a DHT record.

## Signal protocol source auditor

The standalone Rust source tool at `tools/pulse-signal-contract-audit/` uses `syn` to parse the four active Rust enums, their Serde representation and named field sets. It compares the producers with TypeScript unions, both category-filter passes in `SignalHub.ts`, and `pulse-zome-signal-wire-v1.schema.json`.

The initial protocol drift has been repaired: all four producers now use Serde adjacent tagging `{type, data}`; SignalHub unwraps that envelope and only dispatches known variants; all 28 event variants and field sets match Rust, TypeScript and the schema. Signal Contract Audit run #86 passed seven parser tests and reported 28 variants with zero source-contract findings: https://github.com/Luminous-Dynamics/mycelix/actions/runs/41992920393. This is a source-contract result, not sender authentication or event truth.

A second hardening pass is now in the research branch:
- runtime checks validate 39-byte ActionHash/AgentPubKey Uint8Array values, byte-vector fields, safe integer timestamps/counters, exact nullable fields, bounded strings/arrays, vector clocks, unit/custom enum shapes and score ranges;
- the JSON Schema now defines concrete field-level and nested types rather than leaving most properties unconstrained; the portable JSON projection documents hashes as base64url and binary vectors as base64;
- SyncOpType is aligned to every Rust integrity enum variant, including previously missing folder-move, draft-delete, filter, trust-attestation and contact operations;
- the Rust audit tool now fails on unconstrained properties or missing runtime field validation, and has mutation tests for those conditions.

**Qualification status:** the field-type/schema audit changes are committed but their new exact-head CI run is still required. Do not cite the earlier run #86 as qualification of these latest validators until the new run completes. This remains source-level validation only; callback transport identity, exact backing-record validation and two-conductor behavior still require runtime tests.

## Proposed remote signal envelope v1

A strict draft schema has been added at `docs/security/remote-signal-envelope-v1.proposed.schema.json`. It is explicitly a proposal, not a production wire-format change.

The key protocol distinction is:

- **Transport identity:** obtained from Holochain's authenticated callback metadata, never from a sender-controlled `source_agent` payload field.
- **Wire hint:** event ID, namespaced event type, schema version, optional exact source ActionHash, optional logical operation/attempt IDs, informational timestamp and the minimal event payload.
- **Local classification:** `VERIFIED_RECORD`, `AUTHENTICATED_PEER_HINT`, `UNVERIFIED_HINT`, `LOCAL_ONLY`, `UNKNOWN` or `REJECTED` is computed by the receiving code. It must never be asserted by the sender.
- **Evidence reference:** created locally after the receiver fetches and verifies the exact record; not accepted as sender proof.
- **Authority:** still enforced by host CapGrant plus integrity/domain validation. Neither the envelope nor an AuthorityCheckpoint is a permission token.

The schema forbids sender-supplied trust/verification fields, source-agent claims and secret-like keys at the envelope level. It is not yet an event-complete schema: event-specific payload schemas, size bounds, serialization fixtures, compatibility policy and two-conductor tests are prerequisites for implementation. The schema was parsed successfully as JSON after creation.

## Multi-chain causal-frontier semantics

The research branch now includes `PulseAuthorityCausalFrontier.tla` and two configurations:
- `PulseAuthorityCausalFrontier.cfg` — componentwise per-chain freshness with ancestry.
- `PulseAuthorityCausalFrontier-scalar-mutant.cfg` — unsafe aggregate-height comparison.

The design treats a checkpoint across independent Holochain source chains as a **vector frontier**, not a scalar clock. For each participating chain, a read must prove:
1. observed sequence is at least the required sequence;
2. observed history descends from the exact required checkpoint action;
3. the observed head belongs to the same source-chain identity.

Exact equality of the current head hash is not required: a later action on the same chain can satisfy an older checkpoint when ancestry is proven. A higher sequence on one chain cannot compensate for a lagging or unrelated history on another. The model therefore includes post-checkpoint descendant actions on both chains, as well as a foreign same-height projection used by the scalar mutant.

The checkpoint remains a freshness constraint for application reads only. It does not authorize a host call, establish DHT consensus or impose a global total order across agents. Host CapGrant enforcement and resource/domain authorization remain independent.

The current model expansion is being checked with the pinned TLC workflow. Do not claim the expanded descendant semantics are model-checked until the exact run shows the safe frontier configuration exhausts its state queue and the scalar mutant violates its intended invariant. The runtime implementation still needs actual source-chain ActionHash ancestry and two-conductor evidence; the TLA+ boolean ancestry proof is an abstraction, not a Holochain API.
 
## Trust-root and policy-lifecycle model

The branch now also includes a finite TLA+ model for the trust-policy boundary in the DSSE verifier:

- `PulseTrustRootLifecycle.tla`
- `PulseTrustRootLifecycle.cfg` — pinned-root child policy, monotonic epoch and key revocation
- `PulseTrustRootLifecycle-self-root-mutant.cfg` — self-asserted root
- `PulseTrustRootLifecycle-rollback-mutant.cfg` — replay of an older policy
- `PulseTrustRootLifecycle-revocation-mutant.cfg` — acceptance of a currently revoked key

The safe model requires a policy update to chain from the current digest and be authorized by the already pinned root. It advances policy epoch 1→2, revokes key A and activates key B. The three mutants target:
1. accepting a policy because its declared root matches itself;
2. reinstalling older policy epoch 1 after epoch 2;
3. accepting key A for a current verification after the active policy revoked it.

This model is intentionally separate from historical signature verification. A policy may allow a historically valid statement to be reported as historically authenticated, while current authorization under that policy must still reject a now-revoked issuer. The model and CI configs have been authored; their TLC result is pending until the exact workflow log shows the safe queue exhausted and each mutant failing on its intended invariant. No trust-root model-check result is claimed yet.

## Exact-head assurance and verifier results — 6 November 2026

Research PR #5771 head `8b9a2de05c1c562e6f6135fc2645679e756a6cff` completed the following separate hosted checks:

| Lane | Result | Exact evidence | Claim ceiling |
|---|---|---|---|
| Six-model TLC suite | PASS | Run #283, artifact 13501137151 | Bounded abstract models only |
| Lifecycle Rust auditor | 19/19 tests | Run #184, artifact 13502115166 | Structural synthetic trace contract |
| Signal contract Rust auditor | 9/9 tests; 28 schema variants; source report clean | Run #222, artifact 13500599118 | Source/schema drift audit, not runtime signal delivery |
| Signed evidence verifier | 26/26 tests; format and Clippy pass | Run #90, artifact 13501032365 | DSSE role authentication relative to supplied trust policy |
| Multi-conductor Rust auditor | 37/37 tests | Run #166, artifact 13500828166 | Synthetic partial-order structural audit |
| Synthetic audit benchmark | 30 processes at 1k/10k/100k; one 1M probe | Run #166, artifact 13501052421 | Synthetic workload only |
| Owned-vs-borrowed parse benchmark | 30 processes at 1k/10k/100k; one 1M probe | Run #166, artifact 13501252101 | Parse + event-shape validation only |

All six safe TLA+ models exhausted their bounded state queues with no invariant failures, and all eight intentional mutants were caught by their named invariants. The exact state counts and claim limitations are in `pulse-authority-model-check-2026-11-06.md`.

The current multi-conductor auditor still uses a fully materialized owned history for semantic checks. The isolated borrowed parser is 3.2–3.4× faster and requests 15–23× fewer allocation bytes in the measured fixture, but the production path still performs the borrowed preflight and then materializes `Vec<Event>` / `serde_json::Value`. No whole-auditor optimization is claimed. See `pulse-borrowed-parser-microbenchmark-2026-11-06.md` and issue #5872.

The signed evidence verifier is not a global trust-root implementation. Its 26 tests qualify Ed25519 DSSE / in-toto role signatures relative to a supplied policy; the root keyset label is not itself cryptographically authenticated. COSE/SCITT receipt signatures and VDS inclusion/consistency proofs remain unimplemented and therefore INCOMPLETE. See #5863 and `pulse-signed-evidence-profile-v1.md`.

The generic Mycelix CI workflow was skipped in this research PR run. These independent assurance lanes do not compile, package or runtime-qualify the Holochain Pulse hApp.

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

## Independent Rust lifecycle trace verifier

A standalone structural verifier now exists under `tools/pulse-lifecycle-audit/`:

- `src/main.rs` — strict typed event decoder, six-plane invariant checker and JSON report;
- `examples/safe-trace.json` — synthetic positive fixture covering issue, claim, host allow, domain effect, revoke, post-revoke denial, stale checkpoint read, unverified signal and unknown caller acknowledgement;
- `README.md` — event and evidence contract;
- `docs/security/pulse-lifecycle-trace-v1.schema.json` — strict JSON Schema.

The verifier checks:
- `(grantor, idempotency_key)` → single grant identity; independent grantors may reuse the same logical operation ID.
- exact grant ActionHash binding across issuance, claim, revocation and use;
- original grantor is the revocation actor and revoke is monotonic;
- claim recipient and lifecycle position;
- monotonic conductor authority-order index (not wall-clock timestamp order);
- host allow cannot occur after revocation, outside assigned scope, before required claim, or from a projection;
- domain allow and durable effect require their separate host/domain dispositions;
- at most one durable effect per logical operation ID across retry attempts;
- checkpoint-constrained stale projection returns `UNKNOWN`;
- remote signals cannot authorize operations and unverified signals cannot mutate trusted state;
- an unknown acknowledgement is not treated as evidence of no effect.

The CLI emits PASS / FAIL / INCOMPLETE, an input SHA-256, counts, findings and a fixed `STRUCTURAL_ONLY` claim ceiling. Coordinator assertions or missing host/validator receipts become INCOMPLETE rather than being promoted to proof. The verifier does not cryptographically authenticate receipts or prove conductor honesty; a future runtime exporter and receipt-signature verifier remain required.

CI: [Pulse Lifecycle Evidence Audit](https://github.com/Luminous-Dynamics/mycelix/actions/workflows/pulse-lifecycle-audit.yml). Its synthetic fixture is only a structural self-test; it is not runtime evidence. The real exporter must bind exact source commit, conductor build, DNA hash, host authority-order index, source-chain actions, validator decisions and observer projections.

## Required next implementation increments

1. Export the six-plane event trace from the real 0.7 canary without recording secrets.
2. Connect the existing Rust lifecycle verifier to a conductor trace exporter and add cryptographic receipt authentication plus deterministic canonical receipts.
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
- Pulse lifecycle trace schema and Rust verifier: `tools/pulse-lifecycle-audit/` and `docs/security/pulse-lifecycle-trace-v1.schema.json`
- Pulse lifecycle evidence audit workflow: https://github.com/Luminous-Dynamics/mycelix/actions/workflows/pulse-lifecycle-audit.yml

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


The Zanzibar paper reports trillions of ACLs, millions of authorization requests per second, p95 latency below 10 ms and availability above 99.999% over three years of production use. These are Google's published 2019 production figures, not a directly comparable Pulse baseline. SpiceDB and OpenFGA expose explicit consistency preferences, illustrating the latency/freshness trade-off that the Pulse benchmark must report rather than hide. UCAN and Biscuit provide useful portable delegation, attenuation and revocation comparators, but use different verifier and revocation-distribution architectures.

The Holochain roadmap snapshot dated 3 November 2026 reports the 0.7 Wind Tunnel work at 77%, while Wind Tunnel Analysis and Wind Tunnel Core Scenarios are both shown at 0%. Treat the upstream Wind Tunnel work as an evolving substrate reference, not a complete matched Pulse 0.7.1 analytical benchmark.

This profile is a research specification and synthetic corpus. It does not claim conductor runtime correctness, malicious-validator resistance, cryptographic truth, linearizability, crash consistency, liveness, privacy, production readiness or benchmark superiority. Those claims require the corresponding later qualification stages.
