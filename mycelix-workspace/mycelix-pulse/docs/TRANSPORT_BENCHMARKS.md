# Pulse Multi-Transport Benchmarks and Acceptance Gates

- **Status:** Proposed targets; not measured performance claims
- **Applies to:** Pulse transport-neutral contracts, Mycelix relay coordination, future Symthaea route advice
- **Parent RFC:** [Multi-Transport Communications and Standards Registry RFC](MULTI_TRANSPORT_INTEROPERABILITY_RFC.md)
- **Rule:** Local deterministic work proceeds independently of hosted CI queue state. A queued, cancelled, missing, or wrong-head workflow is never a pass.

## 1. Paradigm shift: measure useful delivery, not radio count

A count of supported protocols is a poor measure of resilience. Pulse should optimize **authenticated delivery before a user-defined deadline under explicit cost, privacy, power, and legal constraints**.

The north-star metric is:

`deadline_delivery_rate = authenticated recipient receipts before expiry / eligible logical messages`

Report this alongside:
- delivery probability by deadline (P50/P90/P95 time-to-authenticated-receipt);
- joules and monetary cost per authenticated delivery, measured on real hardware where possible;
- bytes sent per successful delivery, including retries, headers, fragmentation, and duplicate paths;
- outage survival / time to resume delivery after connectivity returns;
- privacy exposure by path (metadata disclosed, relay/gateway visibility, identifier lifetime);
- policy violations and false delivery claims (hard safety metrics, not averaged away).

A higher delivery rate is not a win if it leaks plaintext, violates a transmit constraint, defeats consent, or falsely marks a message delivered.

## 2. Acceptance gates

The numerical values below are engineering targets for the first qualification programme, not universal guarantees or claims about any radio. They must be revisited after baseline traces and device measurements are collected. Where a device or network makes a target impossible, report the limit rather than changing the test silently.

| Gate | Starter target | Evidence required |
|---|---|---|
| Security invariants | **Zero** plaintext fallback; zero unauthorized send/receive; zero crypto downgrade; zero fabricated recipient-delivery receipts | Deterministic negative tests plus verifier-produced results. Any violation blocks release. |
| Envelope integrity | 100% of tampered, truncated, reordered-invalid, expired, replayed, or wrongly addressed fixture cases are rejected or explicitly classified; no partial plaintext reaches the UI | Golden vectors, mutation tests, adversarial fragment corpus |
| Transport-neutral envelope portability | **100%** of declared-compatible adapters can carry the same opaque Pulse envelope while selecting distinct framing profiles; no adapter silently changes the envelope profile or sees plaintext | Contract test carries one envelope profile across at least two different transport-profile IDs with exact-head evidence |
| Clock-domain separation | **100%** of fixtures use monotonic time for link freshness and Unix-epoch time for durable envelope expiry; neither clock can be substituted for the other in the API | Typed-time API tests, expired-message boundary tests, stale/future link-observation fixtures |
| Receipt-token binding | **100%** of wrong-message, wrong-recipient, invalid-signature, and missing-receipt fixtures remain non-delivered | Trusted-verifier tests; a verified-receipt token must bind the exact logical message ID and recipient |
| Logical idempotency | 100% of duplicate-path/retry trials produce at most one visible logical message; all duplicate evidence remains inspectable | Multi-adapter deterministic simulation and restart tests |
| Outbox durability | 100% of accepted queued ciphertext survives process restart and is either delivered, expired, or surfaced as failed/unknown; never silently lost | Crash/restart tests at each state transition |
| Truthful receipts | 100% of outcomes distinguish local queue acceptance, link acknowledgement, gateway receipt, recipient cryptographic receipt, and unknown outcome | State-machine tests plus user-visible contract checks |
| Multi-path resilience | **At least 20% relative improvement** in authenticated deliveries before deadline versus a predeclared preferred-single-path baseline on a fixed mixed-outage corpus, with no security/policy regression | Same traces/seeds and eligible routes for both implementations; include confidence intervals and failures |
| Failure-domain correctness | **100%** of common-cause outage fixtures disable every affected adapter; a shared failure domain is not counted as independent redundancy | Domain IDs, correlated outage windows, negative controls with one truly independent route; report shared-domain and independent-domain scenarios separately |
| Route-decision latency | P95 **≤10 ms** for ranking up to 64 eligible adapters on a declared reference host, excluding OS scans, discovery, and radio handshake | Optimized local benchmark, warm/cold-start distinction, exact host/toolchain/commit |
| Scaling | No superlinear blow-up in route selection when candidate count doubles; record runtime/allocation curves at 1, 4, 16, 64, and 256 candidates | Local Criterion-style or equivalent repeatable Rust benchmark; include variance |
| Offline recovery | 100% of unexpired eligible messages resume retry after restored connectivity; expired items are not sent; policy-revoked paths are not reused | Time-controlled trace replay, queue snapshots and event log |
| Resource bounds | Queue bytes, fragment count, concurrent reassemblies, retry count, and per-peer storage remain within configured limits under stress; malformed input never triggers unbounded allocation | Limit tests at boundary and boundary+1; fuzzing; RSS/heap observations |
| Standards registry integrity | 100% schema-valid entries; every entry has a stable ID, authority, source URL, support state, and provenance; unknown metadata stays unknown | Local schema validator and negative fixtures for missing/invalid fields |
| Standards freshness | Security/transport records reviewed within 90 days; other tracked entries within 180 days, or clearly marked stale/unknown | Recorded last-reviewed date, source revision/hash and due-status generation. These are project policy intervals, not standards-body requirements. |
| Real interoperability | No “interoperable” claim until at least two independent endpoints, exact profile/revision, compatibility conditions and failure cases are recorded; certified means external certification actually occurred | Device/firmware/OS matrix, packet/trace evidence, test report, and certification identifier where applicable |

### Interpreting the 20% resilience target

This is a target to test, not a premise. The absolute percentage-point change must also be shown. If the baseline is already near 100%, percentage improvement can be misleading; report both baseline and candidate rates, deadline distribution, confidence interval, and all regressions. A candidate that improves one trace by overfitting is not qualified.

## 3. Fixed deterministic trace corpus

Start with a small checked-in corpus and expand from measured, anonymized observations. Synthetic traces must be explicitly labelled synthetic.

Each scenario should fix:
- seed, time unit, topology, link availability schedule and change points;
- payload sizes and expiry/deadline;
- per-link latency, loss, capacity, power/cost model and acknowledgement semantics;
- allowed adapters, user policy, geographic/regulatory profile where relevant;
- baseline selector and exact scoring configuration;
- expected invariants and output metrics.

Initial scenarios:
1. Stable IP, no loss (sanity baseline).
2. IP fails mid-send; alternate path remains available.
3. All paths unavailable, then recover; durable queue must resume.
4. Rapidly flapping link; prevent retry storms and route oscillation.
5. High-latency, low-bandwidth path; queue a compact envelope without silent truncation.
6. Low-payload path; the current model checks whole-envelope payload eligibility only. Fragmentation/reassembly limits remain a future adapter-level test.
7. Duplicate delivery across two paths and repeated restarts.
8. Malformed, replayed, expired, wrong-recipient and tampered frame attempts.
9. Gateway acknowledges receipt but recipient is offline; UI must not claim recipient delivery.
10. High congestion and adversarial wake/retry traffic; enforce backpressure.
11. Stale or conflicting capability/standards observations; use UNKNOWN and fail closed.
12. Privacy-sensitive message with a policy that excludes a relay/provider; route around or report unavailable.
13. Device permission revoked during queueing; do not reopen or transmit without renewed authorization.
14. Power-limited endpoint; compare delivery/cost trade-offs under an explicit energy budget.
15. Partitioned community/store-carry-forward topology with bounded storage and expiring items.
16. Two nominally separate adapters share a failure domain; compare with an independent relay. Correlate success draws and a declared outage window across the shared domain.

For pull-request-speed local checks, use a fixed seed and a short corpus. For qualification, use at least 30 declared seeds and 10,000 logical messages per scenario, or document why a specific hardware test uses a smaller safe sample. Synthetic runs do not establish RF performance.

## 3A. Executable synthetic route replay

The checked-in starter corpus is `benchmarks/traces.v1.json`; the Rust model is `standards-registry/validator/src/simulation.rs` and the CLI is `standards-registry/validator/src/bin/route-sim.rs`.

The first adapter contract is in `crates/pulse-transport-core/`. Its isolated tests exercise stale observations, unknown/denied policies, payload and profile bounds, expiry, declared failure domains, symmetric opaque send/receive, and the rule that adapter/link/gateway acknowledgements are not recipient delivery. It separates message-envelope version from transport framing and uses distinct types for monotonic freshness versus Unix-epoch expiry. It remains a standalone prototype and is not wired into the live Pulse send path.

Run from the Pulse workspace root:

```sh
cargo test --locked --manifest-path crates/pulse-transport-core/Cargo.toml
just test-standards-registry
just validate-standards-registry
just standards-search-smoke
just simulate-transport-smoke
just simulate-transport
```

The corpus starts with eleven explicit synthetic scenarios and 10,000 logical messages per scenario. Each run uses a fixed seed and compares the same deterministic draws, correlated within declared failure domains, under three policies: preferred single path, sequential failover, and parallel redundancy. It reports modeled path successes before deadline, P50/P95 modeled latency among successful simulations only, attempts, illustrative cost units, and illustrative energy units. Fixed-point aggregate fields are scaled by 1,000 to avoid floating-point output. The model applies payload budget, explicit permission, link availability and message-index outage ranges before any send attempt.

**Important limitation:** this is a route-policy smoke model, not a radio simulator. It uses synthetic success thresholds correlated within declared failure domains and fixed latency/cost inputs; it does not model RF propagation, interference, congestion, fragmentation, hardware, real joules, end-to-end cryptography or authenticated recipient receipts. Its modeled-success metrics mean only a synthetic path success before deadline; report field names deliberately avoid calling this real delivery. The JSON report hard-codes these non-claims.

Any model-level relative improvement is useful only as a harness sanity check. It cannot establish that the real Pulse adapter will produce the same improvement. Qualification must later replay measured traces, exercise the actual transport contract, and verify cryptographic recipient receipts.

- QUIC (RFC 9000): https://www.rfc-editor.org/rfc/rfc9000
- QUIC DATAGRAM Extension (RFC 9221): https://www.rfc-editor.org/rfc/rfc9221
- Bundle Protocol v7 (RFC 9171), including its delivery-status semantics and statement that end-to-end delivery assurance needs application-level mechanisms: https://www.rfc-editor.org/rfc/rfc9171

## 4. Required baselines

Compare every new route policy against:
- **Preferred-single-path:** current deterministic policy, no alternate route.
- **Fixed priority:** static adapter ranking with the same hard constraints.
- **Simple cost-aware baseline:** deterministic score from measured latency, success, monetary cost and energy.
- **Symthaea adviser (later):** evaluated on the exact same traces, first in shadow/advisory mode.

All baselines receive the same traces, key/message corpus, policy and adapter capability inputs. Do not compare the candidate with a weaker baseline, remove failed cases, tune and report on the same holdout trace, or alter weights after seeing results without recording a new candidate version.

## 5. Symthaea qualification sequence

1. **Offline replay:** recommendations cannot affect live delivery. Compare decision and outcome against deterministic baselines.
2. **Shadow mode:** receive live, privacy-minimized link observations; log recommendations and uncertainty, but do not route based on them.
3. **Bounded advisory:** allow recommendations only among adapters already permitted by deterministic policy. Keep a deterministic fallback.
4. **Controlled rollout:** compare against baseline on predeclared scenarios and devices with an immediate opt-out.
5. **Promotion review:** require repeatable improvement in deadline delivery or cost, no security/policy violation, and no material regression in fairness or tail latency.

Report calibration and confidence only if defined and tested. The model must be able to abstain. Unknown hardware, region, standard revision, stale observations, or unavailable route evidence must not become a positive recommendation.

## 6. Fairness, privacy and negative controls

Track, at minimum, per-peer delivery success and wait time, starvation, bytes relayed, storage consumed, and whether one peer/network is disproportionately burdened. Aggregate metrics can conceal systematic exclusion.

Negative controls must include:
- revoked permission / deleted capability;
- disabled or absent radio hardware;
- unsupported profile or unreviewed standard revision;
- wrong region/channel/power profile;
- expired/replayed/tampered fragments;
- duplicate or reordered bundle;
- unavailable gateway or gateway lies about recipient delivery;
- stale standards source and source digest mismatch;
- simulation accidentally presented as hardware qualification;
- Symthaea recommends a forbidden adapter;
- conflicting route observations and empty/partial discovery;
- outbox full, disk failure, clock skew and process crash.

Every negative control must fail closed and remain represented in the evidence record.

## 7. Local execution before hosted runners

Implementation PRs should provide focused local commands for:
- formatting and type checks;
- registry schema validation;
- deterministic transport-contract tests;
- short seeded trace simulation;
- benchmark smoke run with metrics written to a machine-readable file.

A release/qualification process can later run the extended corpus and real hardware tests. Until implemented, these are acceptance requirements, not commands known to exist. Record the exact commit SHA before execution and bind every report to that SHA, registry hash, trace-corpus hash, toolchain, and configuration.

## 8. Release claim ceiling

- Unit tests prove the tested local contract only.
- Simulation proves behaviour under the specified model and traces only.
- A virtual interface/emulator proves the exercised software integration only.
- A hardware-in-the-loop test proves only the listed devices, versions, settings, region, and scenarios.
- Independent interoperability tests show the recorded endpoints interoperate for the recorded profile.
- External certification may be claimed only with verifiable certification evidence.

A green CI badge is not a substitute for these distinctions. A queued runner is not a pass.
