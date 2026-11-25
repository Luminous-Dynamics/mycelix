# AI-off Execution Boundary Protocol v0.1

**Status:** implementation protocol and qualification contract. This document is not a deployed runner, a security certification, or evidence that any host is isolated.

## 1. Objective: qualify a bounded exclusion claim

An AI-off phase cannot prove the universal statement "no AI influence exists." People retain what they learned, and digital systems may have unknown channels outside the declared threat model. The defensible claim is narrower:

> During the declared run and observation window, the candidate workload could not reach the enumerated AI services, model runtimes, credentials, or candidate-accessible AI-produced artifacts through the declared execution boundary; an independent verifier received sufficient evidence to qualify that bounded claim.

The receipt must name the platform profile, threat model, observation interval, covered channels, exclusions, and evidence sources. A missing channel is not silently treated as covered. Unsupported or unmeasured conditions yield `INDETERMINATE`.

The protocol preserves the existing three-valued result:
- `PASS`: all required evidence is complete, fresh, independently authenticated, and consistent with the declared profile; all required denial controls were exercised and no prohibited access succeeded.
- `FAIL`: a prohibited process, artifact, credential path, successful connection, policy drift, invalid signature, replay, or evidence-tampering event is observed.
- `INDETERMINATE`: a required measurement, witness, inventory, heartbeat, trust root, or control is absent, stale, ambiguous, unsupported, or unverifiable.

A blocked canary attempt is not a breach. A successful connection is. Report both attempts and successful connections separately.

## 2. Trust domains

The candidate workload must not be the authority that measures or certifies itself.

| Domain | Responsibility | Candidate access |
|---|---|---|
| Independent verifier | Freezes the phase manifest, issues a fresh challenge, validates evidence, consumes the nonce, and publishes the final decision | None |
| Host isolation controller | Launches the declared boundary, applies policy, measures the host-side process/mount/device/network state, and monitors liveness | No candidate-writable control socket or credentials |
| Candidate execution domain | Runs only the declared task and approved harness | No monitor, signer, verifier, model, or credential-vault access |
| External witness | Operates independently controlled canary endpoints and records whether per-run challenge tokens arrive | No candidate control or shared signing key |
| Evidence recorder | Emits sequenced observations and signs its own receipts | Append-only interface; not an authority to change verifier policy |

The host controller, external witness, and verifier must not collapse into one candidate-controlled process. Where operational constraints force shared administration, record that as a trust limitation and do not claim independent control-plane separation.

## 3. Declared execution profiles

Profiles are distinct security claims, not interchangeable implementation labels.

### `microvm-kvm-v1` — proposed high-assurance profile

Use a separately booted guest kernel under a measured host/KVM stack. The profile requires:
- a pinned runner, guest kernel, root filesystem, microVM configuration, and host policy digest;
- no candidate-visible network device, host gateway, proxy, model endpoint, credential mount, vsock, virtiofs, host Unix socket, or writable host-shared directory;
- an explicit device allowlist and read-only task-input channel;
- ephemeral scratch storage destroyed after the phase;
- host-side monitoring outside the guest for process/cgroup, mount, device, and network-policy state;
- measured-boot evidence and a fresh platform quote when the declared platform supports it.

A KVM microVM is an isolation mechanism, not proof that the host, firmware, hypervisor, or monitor is uncompromised. A profile without a verifiable measurement root cannot emit a hardware-attested `PASS`.

### `gvisor-runsc-v1` — separate, weaker trust profile

gVisor uses a userspace application kernel while sharing the host kernel. It must be reported as a distinct profile with its own threat model and attack results. Do not pool its results with KVM microVM results or describe the two as equivalent.

### Development-only profiles

Ordinary containers, process groups, environment-variable toggles, DNS-only blocks, and application-level feature flags are useful development controls. They do not qualify as the high-assurance profile described above. They may return diagnostic results but must not be relabeled as a production AI-off attestation.

## 4. Run lifecycle and freshness

1. **Freeze the manifest.** Bind run ID, phase ID, task-input digest, profile ID, runner digest, host-policy digest, guest image/kernel digests, device/mount allowlists, expected probes, witness set, maximum observation gap, and expiration.
2. **Issue an unpredictable challenge.** The verifier generates a cryptographically random nonce only after the manifest is frozen. Bind the nonce to the run, phase, profile, manifest digest, and expiry. The candidate cannot choose or reuse it.
3. **Establish the boundary before launch.** The host controller applies network, credential, process, filesystem, IPC, and device restrictions before candidate code starts. Failure to apply any mandatory restriction aborts the run.
4. **Measure before, during, and after.** Start the monitor before candidate execution. Record an initial snapshot, monotonic sequence numbers, periodic authenticated heartbeats, policy-change events, and a final snapshot. A monitor gap beyond the frozen limit is `INDETERMINATE`; evidence of policy bypass is `FAIL`.
5. **Run unpredictable canaries.** Approved harness probes use fresh per-run tokens and exercise the declared endpoint, control-plane, network, credential, and cache-replay paths during the observation window. The host monitor observes blocked attempts; the external witness independently records any successful arrival. Do not count a self-reported "blocked" result as independent evidence.
6. **Close and reconcile.** Stop the candidate, take the final host-side inventory, reconcile event sequence and counters, destroy ephemeral state, and seal the evidence bundle.
7. **Verify out of band.** The verifier validates platform evidence, registered signer identity, policy and image digests, witness receipts, sequence continuity, inventory completeness, all required probes, and nonce freshness. It then atomically consumes the nonce in a durable store outside the runner host.
8. **Publish a scoped receipt.** The receipt includes the exact profile and limitations, observation start/end, evidence digests, verifier result, and reason codes. A receipt is not portable to another run, phase, host profile, or policy digest.

## 5. Required observations

### Host process and artifact inventory

The inventory must be collected outside the candidate's trust domain and bound to the candidate's cgroup / VM identity. It must account for child processes, exec transitions, open file descriptors, mounts, device mappings, shared-memory and IPC channels, and task-visible artifacts. An in-guest `ps` output or a candidate-provided file list is not sufficient.

The monitor must distinguish approved harness processes from candidate agents and model runtimes. Unknown processes, unknown mounts, unresolved file descriptors, or incomplete inventory prevent `PASS`.

### Network and endpoint exclusion

The policy must cover, as applicable, IPv4, IPv6, TCP, UDP/QUIC, DNS, local proxies, host gateways, loopback forwarding, Unix sockets, vsock, and any host-mediated broker. "No successful HTTP request" does not establish network denial.

For each required channel:
- the frozen manifest identifies the control and its expected behavior;
- a fresh canary attempt is attributable to the declared run;
- the host monitor records the enforcement decision and counter;
- the external witness records whether the unique token arrived;
- successful receipt at any prohibited endpoint is `FAIL`;
- missing host or witness evidence is `INDETERMINATE`.

### Credential separation

Model credentials and agent-control credentials must remain in a separate trust domain. The candidate must not inherit them through environment variables, files, inherited descriptors, process inspection, metadata services, local brokers, or host helper APIs. The credential probe must test the real access boundary, not merely search for a known environment-variable name.

### Artifact and cache exclusion

Candidate-reachable model weights, AI-generated answers, prompt/session caches, tool transcripts, and replayable agent outputs are prohibited unless the preregistered protocol explicitly classifies a particular artifact as an allowed experimental input. The inventory must record digest, classification, location, access path, and candidate reachability. Unknown classification is not a pass.

Human learning from a preceding assisted phase is not a machine artifact leak and must not be misrepresented as removable. The study measures retained human capability; it does not claim to erase learning.

## 6. Evidence and cryptographic envelope

The current v0.1 `AiOffClaims::signing_bytes()` is a crate-local length-prefixed encoding. It is useful for reference tests but is not a standard canonical serialization and must not become a production wire format by accident.

A production version must:
- use a versioned, deterministic serialization (for example, a specified deterministic-CBOR profile) with strict rejection of duplicate/unknown fields where relevant;
- bind every signature to the full manifest digest, run, phase, nonce, profile, observation interval, sequence range, and evidence digests;
- use a registered signing identity with revocation and rotation rules;
- verify platform quotes against independently provisioned trust roots and the verifier's fresh nonce;
- keep host-controller, external-witness, and verifier keys separate;
- sign the witness's observation window and endpoint-set digest, not just a boolean "blocked" field;
- retain the evidence bundle and its root outside the runner operator's mutable storage.

A hash chain detects mutation relative to a retained root; it does not authenticate the event source or prove that the event occurred. A signature authenticates a key's statement; it does not make a compromised signer truthful. The trust model must state both limitations.

Replay protection is an atomic consume-once operation outside the runner host. If the verifier or replay store is unavailable, return `INDETERMINATE`; do not fall back to an in-memory map or accept a retry as a new phase.

## 7. Adversarial qualification tournament

Test the actual deployed profile, not only the Rust reference validator. Keep positive controls and negative controls separate. At minimum, inject:

1. direct inference endpoint, agent control-plane, and model API attempts;
2. DNS, DNS-over-HTTPS, DNS-over-TLS, IPv4, IPv6, TCP, UDP/QUIC, and proxy tunnelling;
3. loopback, host-gateway, Unix-socket, vsock, and host-broker paths;
4. model credentials through environment, inherited descriptor, filesystem, metadata service, and helper process;
5. hidden model weights, answer files, prompt caches, tool transcripts, and stale replay artifacts;
6. candidate-agent/model-runtime process launch, child-process escape, ptrace, namespace or cgroup drift;
7. unexpected mounts, devices, shared memory, writable host paths, and artifact aliases;
8. missing, delayed, duplicated, reordered, forged, and replayed observations;
9. monitor crash, heartbeat gap, runner restart, host reboot, clock rollback, and verifier outage;
10. witness key rotation/revocation, wrong endpoint set, late canary arrival, and an observation window that does not cover the candidate run;
11. signer compromise simulation and a candidate attempt to alter, truncate, or replace the evidence bundle;
12. concurrency races around nonce consumption and two simultaneous submissions for the same challenge.

For every attack, assert the exact expected disposition and reason category. A denied attempt should remain distinguishable from a successful breach. Every generated case must be reproducible from a committed seed and attack-manifest digest.

### Metrics

Report, by fault class and profile:
- observed false-`PASS` count (target: zero in the declared qualification corpus);
- detection and containment rate, with successful egress and credential access reported separately;
- time from first prohibited event to containment;
- monitor coverage, missing-evidence rate, and `INDETERMINATE` rate;
- clean-run false failure rate;
- p50/p95 runtime overhead and task completion rate;
- attack-family coverage and untested-channel list.

"Zero false passes in 1,000 tests" is a corpus result, not proof of zero risk. Do not turn a hand-designed or correlated fault set into a binomial confidence claim. If reporting a statistical upper bound, preregister a sampling model and explain how attack samples represent the target threat distribution. Always publish per-class results so a large easy class cannot hide one uncovered attack family.

## 8. Non-compensatory acceptance gates

No task-accuracy, speed, cost, or capability gain compensates for:
- any successful prohibited AI endpoint or credential access;
- candidate reachability of a prohibited AI answer, model weight, or prompt/session cache;
- false `PASS`, invalid or replayed receipt, or untrusted measurement accepted as trusted;
- host policy drift or unexplained monitor gap;
- missing independent witness or incomplete inventory;
- unauthorized disclosure or removal of required accessibility support.

A successful run qualifies only the named profile and channels for the named time window. Do not generalize it to another host, runner version, model service, operating system, or threat model without requalification.

## 9. Staged implementation sequence

1. **Protocol conformance harness:** canonical manifest, reason codes, sequence/freshness validator, duplicate and replay tests, and a simulator that cannot emit production `PASS`.
2. **Unprivileged integration:** exercise the launcher interface against a disposable local VM and test endpoints; label all results synthetic/development.
3. **Host controller:** implement platform-specific cgroup, mount, device, process, and network measurement outside the guest. Unsupported kernel/platform features become `INDETERMINATE`.
4. **Independent witness and verifier:** deploy separate accounts/keys/storage, fresh canaries, signed observation windows, platform quote verification, and durable atomic nonce consumption.
5. **External red-team review:** attack the complete boundary and publish failures and coverage gaps before any human capability study.
6. **Human study gate:** only after separate ethics approval, accessible consent, compensation, preregistration, pilot-informed power analysis, and independent scoring.

The current Mycelix crate remains the reference contract at step 1. It does not implement steps 2–5 and must not claim that it does.

## 10. Relationship to the capability study

The AI-off phase removes experimental AI services and candidate-accessible digital AI artifacts from the declared machine boundary. It does not remove memory, prior learning, human communication, translation, assistive technology, or essential accommodations. A participant may request help, pause, or stop. The study must not convert independence into a compulsory self-sufficiency score.

The research endpoint remains delayed, held-out human/team capability under a precisely specified support condition—not a claim that people are better off without tools.
