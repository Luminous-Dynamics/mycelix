# Pulse Routing Research Agenda: From Path Selection to Delivery Economics

- **Status:** Research/qualification plan; targets below are proposed, not measured
- **Reviewed:** 2026-10-16
- **Scope:** Public Mycelix Pulse transport-neutral routing; no dependency on the private monorepo
- **Parent design:** [Multi-Transport Interoperability RFC](MULTI_TRANSPORT_INTEROPERABILITY_RFC.md)
- **Benchmark gates:** [Transport Benchmarks](TRANSPORT_BENCHMARKS.md)

## Executive thesis

The field's useful unit of optimization is not a protocol, socket, hop, or packet. It is a **logical message that reaches the intended recipient, with verifiable evidence, before its declared deadline, while respecting consent, privacy, law, energy, bandwidth, storage, and fair use of community relays**.

Pulse should therefore evolve from a best-path selector into a constrained, evidence-carrying delivery scheduler. This is a systems architecture proposal, not a claim that Pulse currently delivers across independent transports. The isolated transport core remains an experimental contract and is not connected to the alpha send path.

## 1. Five paradigm shifts

### 1.1 Count independent failure domains, not adapters

Three adapters can still be one failure path if they depend on the same carrier core, power supply, operating system permission, gateway, DNS authority, or regional control plane. Conversely, a low-bandwidth community relay may offer more resilience than a second high-speed interface that fails for the same reason.

- Keep adapter identity, transport profile, and failure-domain identity separate.
- Model *shared causes* explicitly: carrier, physical device, local power, gateway/provider, geography, discovery/control plane, and software dependency.
- Treat missing correlation evidence as unknown; never equate different names with independence.
- Optimize marginal resilience of a candidate route set, not raw path count.

**Research question:** how much deadline-delivery gain does each additional path contribute after conditioning on already-selected paths and plausible shared causes?

### 1.2 Optimize recipient evidence, not intermediate acknowledgements

The delivery lifecycle is an evidence state machine, not a Boolean flag. Local queue acceptance, adapter send completion, link acknowledgement, gateway acceptance, remote decryption, and a cryptographically validated recipient receipt are different events.

- Only a trusted verifier may authorize the user-visible Delivered state.
- Every receipt must bind the exact logical message, recipient, relevant envelope/context, and replay policy.
- Duplicates from retries and parallel paths should be deduplicated logically without deleting their audit evidence.
- Expiry, process restart, clock uncertainty, and revoked consent must remain visible states, not be converted into success.

**Research question:** can every reported outcome be traced to immutable event evidence, while unknowns remain unknown?

### 1.3 Replace a single weighted score with hard constraints plus a Pareto frontier

A weighted sum can hide a safety violation or trade an unacceptable privacy loss for a small latency improvement. Use two stages instead:

1. **Hard feasibility filter:** consent, destination authorization, privacy class, applicable regulatory profile, supported envelope/framing profiles, expiry, capability evidence, payload bounds, device permissions, retry/storage budgets, and cryptographic preconditions. Unknown or denied inputs fail closed.
2. **Constrained optimization:** among feasible actions, compare deadline delivery probability, tail latency, energy, user cost, bytes, storage occupancy, privacy metadata exposure, and relay burden. Preserve the non-dominated Pareto frontier; apply an explicit, versioned user/service policy to select from it.

This makes trade-offs inspectable and prevents a metric weight from overriding a hard prohibition.

### 1.4 Make uncertainty and the value of measurement first-class

A route observation is a time-bounded claim with provenance, not a fact forever. Discovery can be incomplete; a link can flap; measured success can be biased toward previously reachable peers.

- Store observation time in a monotonic clock domain and message expiry in a wall-clock domain, keeping their types distinct as the prototype does.
- Attach source, age, sample count, confidence/calibration state, and known failure-domain relationships to route estimates.
- Where evidence is thin, use conservative bounds or abstain. Never invent a precise probability from an available Boolean.
- Probe only when the expected value of information justifies its energy, bytes, privacy exposure, and rate-limit budget. Bound probe/retry frequency to prevent storms.
- Any learned/Symthaea advisor starts in offline replay and shadow mode; deterministic policy remains authoritative.

**Research question:** when does a bounded probe or a second path change a deadline outcome enough to justify its cost?

### 1.5 Treat community capacity and fairness as correctness properties

A design that achieves high aggregate delivery by draining one peer's battery, exhausting one relay's storage, or starving a class of users is not resilient.

- Track per-peer deadline success, tail wait, bytes forwarded, energy, stored bytes, retransmissions, and expirations.
- Compare only similarly eligible workload groups; state eligibility and workload assumptions.
- Enforce per-peer quotas, bounded custody/storage, backpressure, expiry, and revocation.
- Report aggregate, per-scenario, and per-peer results. A good aggregate must not hide a failing population.

**Research question:** can delivery improve without concentrating cost or starving low-bandwidth/offline participants?

## 2. Scheduling model to investigate

Let each candidate action be a permitted route, route set, delayed store-carry-forward opportunity, or bounded probe. The selector first applies hard policy gates, then estimates the marginal benefit of each feasible action.

The main objective is:

P(recipient receipt verified by deadline | selected routes, observations, policy)

Then measure the incremental benefit of adding an action to the current set, alongside incremental joules, bytes, monetary cost, metadata exposure, storage use, and peer burden. Do **not** assume route independence: condition on common-cause domains or use conservative lower bounds until calibration evidence supports a richer model.

The scheduler should support at least these modes:

- **Minimum-cost feasible:** choose the least-cost feasible action expected to meet the deadline.
- **Sequential failover:** preserve energy by waiting for evidence that the preferred route failed, subject to the deadline.
- **Budgeted redundancy:** send on one or more materially independent routes when the expected delivery gain justifies added cost.
- **Store-carry-forward:** retain opaque, expiring data and schedule transfer at an observed or opportunistic contact window; no requirement that sender and recipient be online together.
- **Abstain / unavailable:** report a truthful failure or unknown state when no action is safely feasible.

This is a hypothesis to benchmark, not a commitment to deploy a learned optimizer. Start with deterministic baselines and interpretable policies.

## 3. Proposed qualification targets

All performance values are **starter engineering targets**. Freeze the workload profile, policy, toolchain, hardware, and comparison baseline before running qualification. Do not silently change the corpus to meet a gate.

| Dimension | Initial target | Qualification rule |
|---|---|---|
| Safety/privacy | Zero unauthorized sends, plaintext fallback, crypto downgrade, policy bypass, or fabricated recipient receipts | One violation blocks release; never average it away |
| Delivery truth | 100% of Delivered states have an independently validated receipt bound to the expected message and recipient | Negative tests include wrong recipient/message, replay, malformed signature, and missing receipt |
| Idempotency | At most one user-visible logical message per message/recipient; all duplicate evidence remains inspectable | Retry, multipath, receive reorder, crash/restart corpus |
| Resilience | Candidate policy point estimate improves deadline success by at least 20% relative to a predeclared preferred-path baseline on the mixed-outage holdout | Also report absolute percentage-point delta; paired 95% confidence interval for the improvement must exclude zero; publish every scenario result |
| Non-regression | Stable, well-connected baseline loses no more than 2 percentage points of deadline success and stays within a predeclared energy/byte budget | Separate no-outage workload; evaluate policy revisions on a holdout, not only tuning traces |
| Correlated outages | 100% of explicit common-cause negative controls cause every dependent path to be treated consistently | Include carrier, shared power, gateway, DNS/control plane, and device-permission domains; don't infer independence from different adapter names |
| Route selection latency | P95 ≤10 ms for ranking 64 eligible candidates; proposed stretch target ≤40 ms at 256 candidates | Published host, compiler/toolchain, warm/cold distinction, allocation/runtime distribution |
| Scale | No superlinear regression over 1, 4, 16, 64, 256 candidates without a documented reason | Report absolute latency and memory; distinguish discovery cost from ranking cost |
| Outbox durability | Zero silent loss of accepted unexpired ciphertext across crash points | Every item ends delivered, expired, or explicitly failed/unknown |
| Offline recovery | 100% of permitted unexpired messages are reconsidered after connectivity returns; expired or revoked messages are not transmitted | Virtual-clock replay plus restart/revocation interleavings |
| Fairness | No starvation among equally eligible peers in the qualification workload; report max/min and distribution of success, delay, bytes, and relay burden | Any proposed numeric fairness bound must be fixed with the workload definition before testing |
| Resource limits | Every queue, peer quota, fragment/reassembly budget, retry budget, and payload cap has boundary and boundary+1 tests | Malformed input cannot cause unbounded allocation or work |
| Interoperability | 100% pass for declared-compatible golden envelope vectors across at least two independent implementations before claiming that profile works | Capture implementation/version/profile identifiers and failed cases; never infer conformance from a source citation |
| Energy/cost | Report joules and currency per verified receipt, bytes per receipt, and deadline success under the same declared budget | Use actual device measurements before making hardware/energy claims |

The 20% resilience gate is a target, not a prior belief. If a baseline is near 100%, relative gain becomes misleading; publish baseline/candidate rates, absolute delta, tail latency, cost/energy, uncertainty, and regressions. A candidate does not qualify by winning only one scenario.

## 4. Statistical protocol: prevent synthetic certainty

1. Keep the checked-in trace corpus explicitly synthetic; add measured traces only after consent, minimization, and removal of identifying metadata.
2. First validate each strategy on a fixed, quick smoke seed. Then run at least 30 predeclared seeds and 10,000 logical messages per scenario for model-level qualification, unless a documented safety or runtime constraint justifies a different sample.
3. Use paired comparisons: baseline and candidate see the same scenario, seed, logical-message inputs, eligibility, and policy. Record code, corpus, configuration and toolchain digests.
4. Treat independent trace windows or independent failure epochs—not every packet or correlated message—as the statistical unit when calculating uncertainty. Use a paired bootstrap or another justified interval method and document its unit.
5. Publish per-scenario results, macro-average across scenarios, and a workload-weighted result only when its weights were declared beforehand. Do not allow the largest scenario to erase a severe low-connectivity failure.
6. Tune on development traces, keep holdout scenarios/seeds out of tuning, and version every new policy/corpus. Preserve failed runs and negative controls.
7. A simulator's “path success” is never an authenticated receipt. Upgrade claim strength only as evidence moves from model → software integration → virtual interface → hardware-in-the-loop → independent endpoints.

## 5. Research and implementation sequence

### P0 — Formalize contracts and evidence (current tranche)

- Keep envelope profile, transport profile, and failure domain separate.
- Keep message-expiry time distinct from monotonic link freshness.
- Bind trusted recipient receipt to the exact message and recipient.
- Preserve unknown and deny states; reject unsupported profiles and over-budget frames before large allocations.
- Add model-based state-transition cases for expiry, retry, duplicates, restart, revocation, and receipt arrival reordering.

**Exit:** isolated contract tests compile and pass at the exact PR head; no claim until that evidence exists.

### P1 — Better independent evaluation (implementation added; execution pending)

- Added `standards-registry/validator/src/evaluation.rs`, a paired multi-seed evaluator comparing sequential failover or parallel redundancy to preferred-single-path on the declared holdout scenarios.
- Added an exhaustive, disjoint scenario split: five development scenarios and six holdout scenarios, with three separately declared resilience scenarios, including stable-network non-regression, privacy exclusion, flapping links, energy trade-offs, and shared-carrier failure.
- Added per-holdout scenario rates, absolute percentage-point change, relative point-estimate improvement, a paired seed-level interval, and a model-only screen against the proposed 20% resilience / -2 percentage-point stable-network gates.
- Added manifest-bound CLI flags for the checked-out subject SHA and SHA-256 of a generated input manifest. CI is configured for a 30-seed × 1,000-message smoke evaluation; `just evaluate-transport` defines the heavier 30-seed × 10,000-message run.
- The interval explicitly measures variation in this synthetic model only. It is not empirical uncertainty about real network performance, and even a model-screen pass is not release qualification.
- This tranche does **not** yet implement calibrated weak/unknown failure-correlation models, a true held-out test process history, hardware traces, or real device metrics.

**Exit:** exact-head Rust formatting, tests, and paired evaluation execute successfully; artifact subject/input hashes match the tested commit; negative controls pass; no performance result is claimed before that evidence appears.

### P2 — Durable messaging semantics

- Integrate the outbox/event state machine with the supported Pulse alpha path without exposing plaintext to adapters.
- Exercise crash/restart, expiration, revocation, deduplication, partial receive, and bounded reassembly.
- Validate the existing Pulse trusted receipt verifier rather than treating a trait boundary as cryptographic proof.

**Exit:** exact-head tests over the real application integration, plus explicit failure evidence. No radio qualification implied.

### P3 — First real adapters

Choose the first adapter based on measurable user utility, implementation risk, available hardware, and genuinely independent failure domains—not novelty. A nearby Bluetooth LE prototype is useful for local opportunistic transfer; a BPv7 convergence-layer experiment is useful for persistent store-carry-forward. These solve different topologies and should be compared against the same logical-envelope and receipt contracts.

- Start with loopback/virtual adapters and deterministic golden vectors.
- Then test two independent endpoints for each declared transport profile.
- Measure packet/framing overhead, loss, retries, queue behavior, energy, and expiry under repeatable conditions.
- Keep hardware-specific permissions, channel/region constraints, and device absence as explicit fail-closed states.

**Exit:** reproducible endpoint matrix and raw traces for the listed hardware/firmware/OS and environmental conditions. External certification claims require actual certification evidence.

### P4 — Symthaea advisory, never policy authority

- Replay the advisor against identical baselines, then shadow-route with minimized observations.
- Evaluate calibration, abstention, decision latency, per-peer fairness, energy, privacy exposure, and distribution shift.
- Permit it to rank only options that deterministic policy has already declared feasible.
- Require clear regression thresholds and an immediate deterministic fallback.

**Exit:** statistically justified benefit on held-out workload, zero hard-gate violations, and no unexplained population-level regression.

## 6. Research basis and boundaries

- [RFC 4838 — Delay-Tolerant Networking Architecture](https://datatracker.ietf.org/doc/rfc4838/) describes store-and-forward operation over long timescales, including cases with no contemporaneous end-to-end path. Pulse can borrow the architectural principle without claiming BP interoperability.
- [RFC 9171 — Bundle Protocol Version 7](https://www.rfc-editor.org/rfc/rfc9171.html) standardizes bundles and convergence-layer services; route computation and forwarding-table population are outside that specification. This leaves an application/policy scheduling problem for Pulse, not a reason to invent or conflate protocol semantics.
- [RFC 8684 — Multipath TCP](https://www.rfc-editor.org/rfc/rfc8684.html) defines multipath operation for a TCP session. Multiple subflows within one connection are not automatically independent failure domains.
- [Multipath QUIC status page](https://datatracker.ietf.org/doc/draft-ietf-quic-multipath/) describes a multipath extension and explicitly leaves application address discovery/management and scheduling outside its scope. The Datatracker status page was checked on 2026-10-16 and showed the work in the RFC Editor queue; re-check status before implementation rather than treating the draft text as a final published RFC.
- [RFC 9221 — QUIC DATAGRAM](https://www.rfc-editor.org/rfc/rfc9221.html) defines an unreliable datagram extension. A transport's packet-level acknowledgement semantics must not be promoted into application-recipient delivery.

These references are architecture inputs, not implementation/conformance evidence. Standards-registry support status stays unreviewed until the applicable text/revision, implementation behavior, and test evidence are reviewed separately.

## 7. Explicit non-goals for this tranche

- No automatic activation, installation, or radio transmission from registry or model output.
- No learned policy bypassing consent, security, or legal constraints.
- No claim that simulated paths imply actual availability or authenticated delivery.
- No claim of hardware, interoperability, certification, or achieved resilience targets from this research plan.
