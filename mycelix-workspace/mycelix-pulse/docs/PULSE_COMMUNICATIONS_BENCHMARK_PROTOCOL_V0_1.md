# Pulse Communications Benchmark Protocol v0.1

**Benchmark ID:** `PULSE-FABRIC-BENCH-001`  
**Status:** pre-registered, not executed  
**Last source review:** 21 November 2026

## Purpose

Stop comparing messaging systems with a single throughput number that quietly ignores what each system means by accepted, delivered, committed, verified or recoverable. This protocol defines matched experiments for Pulse and mature protocol references. It is a test design, not a report of achieved performance.

The primary outcome is:

\[
VDR_D = \frac{\text{unique logical operations with independently verified recipient receipt by deadline }D}
{\text{all admitted logical message intents}}
\]

A transport ACK, relay acceptance, HTTP success, source-chain publication or DHT visibility is not a verified recipient effect. Unknown, incomplete and duplicate effects do not count as successes.

## Paradigm shift: compare guarantees, not product labels

Run three separate comparison layers:

1. **Cryptographic protocol:** initial key establishment, per-message cost, ratchet evolution, group epoch operations, device add/remove, forward secrecy and post-compromise recovery.
2. **Delivery semantics:** durable outbox, retries, deduplication, recipient commit, receipt binding and terminal state projection.
3. **End-to-end system:** actual client, local persistence, conductor/relay, network, backup and recovery costs.

A candidate may be strongest in one layer and inapplicable in another. A classical ratchet is not a PQ ratchet; MLS group key management is not a substitute for 1:1 asynchronous session semantics; and faster relay ACKs are not equivalent to faster verified delivery.

## Candidate set and current industry boundary

| Candidate | Current reference role | Limitation to preserve in reporting |
|---|---|---|
| Pulse V2 | Current stateless hybrid envelope baseline | No PQ ratchet, later-static-key-compromise FS or PCS claim |
| vodozemac 0.11.2 | Rust Olm/Megolm ratchet reference | Classical Matrix protocol; not ML-KEM Braid/SPQR |
| Signal libsignal | Official Signal Rust protocol reference | Rust internals exist, but documented platform APIs are Java, Swift and TypeScript; external integration contract needs audit |
| OpenMLS 0.9.2 | Rust MLS RFC 9420 group reference | Current listed suites are classical; WASM is built in CI but not listed among tested targets |
| MLS PQ ciphersuites -08 | PQ group protocol specification target | Active Internet-Draft, not RFC; no runtime score until reviewed implementation and interop evidence exist |

The OpenMLS release documented on 22 October 2026 is 0.9.2. Its supported suites and platform test/build distinction are recorded from the crate documentation. vodozemac 0.11.2 was published on 5 November 2026 and is an Apache-2.0 Rust Olm/Megolm implementation. Signal's libsignal README describes Rust implementations exposed through Java, Swift and TypeScript APIs. The IETF MLS PQ ciphersuite draft -08 was published 18 November 2026 and entered IESG Last Call on 19 November, due 3 December 2026. Treat it as work in progress, not a finalized standard. See the source list in the machine-readable profile and:
- [OpenMLS 0.9.2](https://docs.rs/crate/openmls/0.9.2)
- [vodozemac 0.11.2](https://docs.rs/crate/vodozemac/0.11.2)
- [Signal libsignal](https://github.com/signalapp/libsignal)
- [Signal ML-KEM Braid](https://signal.org/docs/specifications/mlkembraid/)
- [MLS PQ ciphersuites draft](https://datatracker.ietf.org/doc/draft-ietf-mls-pq-ciphersuites/)

### Paradigm shift: compose mature cryptography, innovate in verifiable operations

Signal's ML-KEM Braid is a Sparse Continuous Key Agreement design that emits
ordered key epochs and is intended to feed higher-level protocols with
forward-secrecy and post-compromise-security properties. Its important
benchmark variable is not just KEM cost: under low-bandwidth, intermittent
communication, measure the vulnerable-message-set size and time/messages to
heal after compromise. Its incremental KEM and erasure-coded chunking are
particularly relevant to poor-connectivity and mobile-offline scenarios.
Pulse V2 must not claim these ratchet properties merely because it uses a
hybrid envelope.

The MLS PQ ciphersuite draft -08 is an active Internet-Draft, published
18 November 2026, with IESG Last Call due 3 December 2026. It includes
traditional/hybrid KEM profiles and PQ-signature variants. A PQ KEM does not
automatically imply post-quantum authentication: the draft explicitly
distinguishes suites that retain traditional signatures from suites that
also use ML-DSA. Track the exact suite and threat model, not just the
"post-quantum" label.

The strategic bet for Pulse is therefore not a bespoke ratchet. Use
Signal's mature 1:1 protocol direction and MLS for group key management as
cryptographic comparators/integration candidates; invest differentiation in
host-enforced authority, idempotent offline delivery, recipient-verified
effects, privacy-preserving manifests, portable recovery and independently
replayable evidence. The fair win condition is a better **verified effect per
joule/byte and per failure-domain outage**, with no reduction in the underlying
cryptographic guarantee.


## Frozen workload

Use all listed payload sizes and network/failure dimensions, but pre-register the factorial design or stratified subset before seeing results. Do not silently drop inconvenient combinations.

| Dimension | Values |
|---|---|
| Payload | 1 KiB, 16 KiB, 256 KiB, 1 MiB |
| RTT | 20 ms, 100 ms, 500 ms |
| Packet loss | 0%, 1%, 5%, 20% |
| Peers | 1, 2, 5, 10 |
| Group size | 2, 10, 100, 1,000 where supported |
| Offline window | 0 s, 60 s, 1 h, 24 h |
| Delivery deadline | 1 s, 5 s, 30 s |

Failure cases include single-path drop, process and power loss at persistence boundaries, recipient commit with lost ACK, relay restart, carrier/gateway common-cause failure, DNS/discovery common-cause failure, partition and heal, duplicate/reorder, revoked/stale devices, corrupt backup, key-directory rollback and downgrade attempts.

Cryptographic cases include initial establishment, steady-state messages, skipped/out-of-order messages, device addition/removal, rotation, later static-key compromise of recorded transcripts, compromise of current session state followed by recovery, and restart/backup rollback. Do not claim FS/PCS unless the exact protocol's threat and erasure assumptions are tested and reviewed.

Metadata experiments must have separate attacker vantage points: relay, network observer, conductor operator, DHT observer, sender and recipient. Record the inference task and ground-truth labels. Report AUC and calibration against a declared prior; ciphertext alone does not establish anonymity.

## Experimental protocol

- 30 independent replicates per reported condition; at least 1,000 logical intents per replicate.
- 60-second warm-up, followed by a fresh state for each replicate.
- Use identical deterministic fault schedules across candidates and randomize candidate order.
- Publish failed, excluded, unsupported and incomplete runs with reasons.
- Report p50, p95 and p99, not only averages.
- Estimate paired 95% confidence intervals by resampling independent replicate pairs; publish the resampling seed and method.
- Bind every run to source commit, lock digest, protocol suite/version, toolchain, client/conductor digests, hardware/firmware, OS/power mode, network emulator, topology/failure domains, corpus digest, seed and verifier digest.
- Measure physical energy using a documented rail or external instrument. Do not use CPU time or estimated TDP as a substitute for energy per verified delivery.

The 30-replicate / 1,000-intent minimum is an initial preregistration floor, not a claim of adequate statistical power for every effect size. Conduct a blinded pilot to estimate variance, then increase sample size before opening the comparative results. Do not reduce sample size after looking at the outcome.

## Metrics

Publish latency quantiles for key establishment, per-message crypto, recipient commit, receipt verification and end-to-end verified receipt. Also publish ciphertext expansion, bytes per verified delivery, retry amplification, CPU time, RSS, storage growth/write amplification, physical energy per verified delivery, recovery time/completeness, per-peer relay burden, metadata-inference AUC, false verified rate, duplicate logical-effect rate and acknowledged-outbox loss rate.

For unsupported features, use **unsupported / not implemented / unmeasured** with an explanation. Never encode unsupported as zero latency, zero leakage or zero energy. For unmeasured behavior, do not infer failure or success.

## Non-compensable gates

- Zero false `SUCCESS_VERIFIED` in the declared frozen corpus.
- Zero duplicate recipient logical effects per idempotency key.
- Zero silent loss of acknowledged outbox state within the declared storage durability model.
- Zero host capability widening across cell, grantee, function or capability kind.
- Zero post-revocation admission in non-overlapping host tests.
- Zero silent downgrade to weaker or unqualified crypto.
- Zero plaintext, raw key, private seed or capability secret in telemetry.
- Every unknown outcome remains `UNKNOWN` or `INCOMPLETE`.

These cannot be traded for performance. Do not compute an overall weighted score that allows a faster but unsafe system to rank first.

## Proposed targets (not current measurements)

- Healthy-network deadline-delivery regression no greater than 2 percentage points versus a simple single-path baseline.
- At least 20% relative improvement in verified delivery by deadline during correlated outages versus that baseline, with paired 95% confidence intervals excluding zero.
- Healthy-condition energy overhead no greater than 10% for added assurance/resilience mechanisms.
- 100% detection of critical verifier/protocol mutants.
- 0% false verified claims and 0% duplicate logical inbox effects in the frozen in-domain corpus.

The numerical targets must be revisited after a blinded baseline pilot. They are not evidence of present Pulse performance or a guarantee of statistical power.

## Group fan-out: per-recipient truth before aggregate claims

The target product semantics use a parent fanout identity and independent
recipient-child logical operations. The parent identity and recipient-set
commitment are coordination metadata, not delivery receipts or bearer
authority. Every required recipient gets one stable child logical operation;
retries and alternate routes create new attempt IDs without creating a second
logical recipient effect.

The recipient-set snapshot binds the send to a declared membership/epoch
state. Qualification traces carry only a redacted commitment and exact
policy/runtime evidence references; they do not expose the raw roster. A child
success claim requires recipient admission, one logical inbox commit,
decryption verification and a separately attributed receipt-verification
event. The target aggregate semantics are COMPLETE_VERIFIED only when every
required child is cryptographically verified by the trusted deadline.

The current v0.1 verifier does not yet establish that stronger claim. It checks
exact parent/child byte digests, recipient-set completeness, child causal
consistency and receipt-event bindings. Its honest output is
ALL_REQUIRED_RECEIPT_CLAIMS_CONSISTENT. It does not verify receipt signatures,
manifest authority, membership snapshot authenticity, principal independence
or trusted time. Therefore, the benchmark must not count this structural
verdict as a production COMPLETE_VERIFIED event until a trust-root and
trusted-time layer is separately implemented and qualified.

The default target aggregate policy is all required recipients. PARTIAL,
UNKNOWN, INCOMPLETE, EXPIRED and FAILED remain distinct. A quorum/threshold
override is permitted only as an explicit user-visible policy bound to an
authenticated policy snapshot; it must not be silently substituted for full
completion. Device semantics must be declared: one device receipt does not
imply all-device synchronization. Membership removal, late join and
historical-message decryption are separate cases evaluated against the exact
membership epoch.

Group-specific metrics include per-recipient delivery by deadline,
complete-group delivery by deadline, partial coverage distribution,
worst-required-recipient latency, group epoch commit time/bytes,
post-removal acceptance, late-joiner historical-decrypt acceptance, and
CPU/memory/storage/network/physical-energy cost per recipient-verified
effect and per fully verified group send. Report both recipient-level and
aggregate results; a high average must not hide one unreachable or
unauthorized member. Commitment privacy tests must include dictionary attacks,
correlation across epochs and small-group membership inference from metadata.

The machine-readable profile also pre-registers group cost per successful
effect: CPU time, peak memory, storage growth, network bytes and instrumented
physical energy per recipient-verified effect, plus CPU time, peak memory,
storage writes, network bytes and physical energy per completely verified
group send. It adds recipient-set dictionary-attack success,
cross-epoch recipient-linkability AUC and small-group membership-inference
AUC. Publish costs and failures for incomplete groups separately; do not
exclude expensive failures from the group-send cost picture.

## Execution order

1. Freeze source, lockfiles, runtime tuple, protocol versions, hardware and corpus.
2. Run primitive protocol experiments independently of Holochain.
3. Run delivery/idempotency/receipt experiments against the same deterministic fault injector.
4. Run the end-to-end system on each supported platform; classify unsupported adapters explicitly.
5. Run metadata inference and recovery experiments separately.
6. Publish raw result artifacts, verification receipts, exclusions, analysis scripts, confidence intervals and claim limits.
7. Obtain independent replication before making superiority claims.

The benchmark profile is linked to [PULSE-FABRIC-QUAL-001](PULSE_COMMUNICATIONS_FABRIC_QUALIFICATION_V0_1.md). This protocol does not certify the cryptographic references or qualify Pulse runtime behavior.
