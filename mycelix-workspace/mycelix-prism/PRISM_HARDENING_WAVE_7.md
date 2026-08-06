# Prism Hardening Wave 7

Wave 7 closes the principal network and reviewer-lifecycle gaps left by Wave 6.
Compatibility browsing can now run inside a network namespace with loopback as
its only IP interface, while an authenticated Unix-socket tunnel broker performs
public-only host egress. Reviewer trust can rotate through a signed append-only
history without erasing the provenance of earlier decisions, and index/release
evidence commits to that complete history.

This wave still does not add browsing features. It turns previously documented
frontiers into explicit, auditable boundaries and preserves fail-closed behavior
when those boundaries cannot be established.

## Scope

This wave contains thirteen ordered implementation and evidence patch sets
before this campaign document:

1. **Fail-closed raw egress destination policy**
   - Adds an explicit destination policy shared by the egress broker.
   - Requires an admitted port and rejects a hostname when any resolved address
     is loopback, private, link-local, multicast, unspecified, documentation,
     benchmarking, or otherwise non-public.
   - Pins connections to the validated address set instead of resolving again
     during connect.

2. **Authenticated public-only tunnel broker**
   - Adds the standalone `prism-egress` crate.
   - Creates a fresh session identifier and high-entropy key for every broker.
   - Authenticates requests with keyed BLAKE3 over the exact session, sequence,
     connection identifier, host, and port.
   - Verifies Unix peer credentials, rejects replay and reordered requests,
     applies per-session tunnel limits, and uses bounded framing and timeouts.

3. **Kernel-isolated Compatibility launch path**
   - Launches Chromium under Bubblewrap with `--unshare-net`.
   - Exposes only namespace loopback to Chromium.
   - Mounts the host broker socket read-only and starts an admitted namespace
     runner that provides a local CONNECT relay.
   - Supplies session bootstrap material only through inherited standard input.
   - Disables browser DNS prefetch, QUIC, non-proxied WebRTC, and direct proxy
     bypasses.

4. **Append-only reviewer policy history**
   - Anchors a genesis reviewer policy by ID and digest.
   - Requires every transition to name the exact previous policy digest and
     previous transition hash.
   - Requires signatures satisfying the previous policy's retraction quorum.
   - Prevents key removal, historical key rewriting, retroactive activation,
     backdated revocation, policy-ID changes, and silent quorum changes.

5. **Policy-history-bound review and index evidence**
   - Adds secure no-follow loading for policy histories.
   - Carries the current policy digest and complete policy-history root through
     `VerifiedReviewLedger`.
   - Advances index snapshot schema to v6 and publication receipt schema to v2.
   - Rejects an otherwise valid index when opened under a different reviewer
     history, even if both histories end at byte-identical current policies.

6. **Policy-history-bound release attestations**
   - Advances release attestation schema to v2.
   - Binds the current reviewer policy digest, append-only policy-history root,
     and review-ledger root independently.
   - Requires exactly one authoritative reviewer trust path: direct policy or
     verified policy history.

7. **Secret-free egress session evidence**
   - Records session identity, destination policy, tunnel budget, accepted and
     rejected tunnel counts, transferred bytes, and broker lifecycle state.
   - Never serializes the session key or authenticated control frames.

8. **Worker-finalized egress accounting**
   - Stops broker admission, wakes the listener, joins the accept loop, and waits
     for every tunnel worker before evidence finalization.
   - Prevents audit receipts from racing with in-flight tunnels.

9. **Durable Compatibility audit receipts**
   - Atomically publishes mode-0600 start and completion receipts.
   - Binds origin, request/session identity, browser, Bubblewrap, and runner
     digests, admitted ports, tunnel budget, process result, and finalized egress
     counters.
   - Requires an audit root by default; unrecorded kernel egress is an explicit
     degradation.

10. **Cross-crate reviewer-history substitution gate**
    - Constructs two independently valid histories ending at the same policy.
    - Proves an index built under one history is rejected under the other.
    - Exercises the trust root across ingestion, review verification, search
      snapshot publication, and loading.

11. **Reproducible Wave 7 evidence gates**
    - Adds repository-native checks for kernel namespace wiring, broker MAC and
      replay controls, peer-UID enforcement, all-answer-public DNS policy,
      worker-finalized evidence, reviewer history, index schema v6, attestation
      schema v2, and the cross-crate substitution test.
    - Updates supply-chain evidence for the new workspace crate and lockfile.

12. **Operational documentation**
    - Documents Compatibility kernel egress deployment and reviewer policy
      rotation ceremonies.
    - Separates namespace guarantees from browser-level mitigations.

13. **Portable namespace-runner build behavior**
    - Replaces a crate-wide Linux compilation exclusion with an explicit
      non-Linux fail-closed executable stub.
    - Allows cross-platform workspace checks to compile the binary while making
      it impossible to execute the Linux-only isolation path elsewhere.

## Security invariants

### Compatibility egress

A kernel-mediated Compatibility session is admitted only when all of the
following hold:

- Chromium, Bubblewrap, and the namespace runner pass executable admission;
- Bubblewrap creates a fresh network namespace with no external interface;
- Chromium can reach only the loopback CONNECT relay;
- the relay can reach only the session-specific Unix broker socket;
- the broker verifies the connecting UID and every keyed control frame;
- request sequence numbers are strictly monotonic and connection IDs cannot be
  replayed;
- the requested port is admitted;
- every DNS answer is public and the outbound socket is pinned to a validated
  address;
- tunnel and byte accounting is finalized only after all workers terminate;
- the start and completion receipts are published atomically.

The session key is delivered through inherited standard input, removed from the
launcher copy after serialization, absent from debug output, and never persisted
in audit evidence.

### Reviewer policy rotation

A policy-history transition is accepted only when:

- the anchor matches the genesis policy ID and exact digest;
- sequence numbers, timestamps, previous policy digests, and transition hashes
  form one continuous chain;
- the previous policy's authorized retraction reviewers satisfy its quorum;
- existing key identities and immutable metadata are preserved;
- newly introduced keys do not become valid before the transition;
- revocation and validity changes cannot be backdated;
- the final ledger verifies under the current policy derived from that exact
  history.

A current policy alone cannot reconstruct or substitute for the policy-history
root. Indexes and release statements bind both values independently.

## Production configuration

### Kernel-mediated Compatibility mode

The normal Wave 7 path requires:

- `PRISM_COMPAT_KERNEL_EGRESS=true`
- `PRISM_COMPAT_NETNS_RUNNER_PATH=/absolute/prism-compat-netns-runner`
- executable admission for Chromium, Bubblewrap, and the runner through exact
  BLAKE3 pins or immutable Nix-store paths
- `PRISM_COMPAT_AUDIT_ROOT=/absolute/private/audit-directory`

Optional policy controls include:

- `PRISM_COMPAT_EGRESS_ROOT=/absolute/private/session-directory`
- `PRISM_COMPAT_EGRESS_PORTS=443`
- `PRISM_COMPAT_EGRESS_MAX_TUNNELS=<bounded-count>`

The existing shared-host-network proxy and direct-network paths remain available
only behind their explicit degradation acknowledgements. They are not equivalent
to the kernel-mediated path.

### Reviewer policy history

A rotated reviewer deployment supplies:

- the independently provisioned review anchor;
- the append-only policy-history file;
- the review ledger;
- the admitted local hybrid-signature verifier agent.

Direct policy and policy-history inputs are mutually exclusive. A deployment
must not provide both and rely on precedence.

## Evidence and release flow

1. Verify the review anchor and complete policy history.
2. Verify the review ledger under the current policy derived from that history.
3. Build core and full index snapshots and atomically publish receipt schema v2.
4. Capture canonical Rust and Nix evidence from the same clean Git tree and exact
   locks.
5. Prepare release attestation schema v2, binding:
   - source commit and tree;
   - Cargo and Nix lock digests;
   - Rust and Nix evidence;
   - reviewed index publication;
   - current reviewer policy digest;
   - reviewer policy-history root;
   - review-ledger root.
6. Hybrid-sign the canonical payload and independently verify it through the
   admitted verifier agent.

## Static verification

```sh
python3 scripts/verify-wave7-static.py \
  --check security/wave7-static-evidence.json
python3 scripts/verify-supply-chain.py \
  --check security/supply-chain-evidence.json
```

Release-mode supply-chain admission remains expected to fail until a canonical,
reviewed `flake.lock` exists:

```sh
python3 scripts/verify-supply-chain.py --release
```

## Remaining limitations

- The host kernel, Bubblewrap, Unix-socket implementation, DNS resolver, and
  egress broker remain in the trusted computing base.
- The broker enforces destination identity and transport policy; it does not
  decrypt or inspect TLS. Chromium remains responsible for TLS validation.
- Display-server and GPU isolation are not strengthened by the network namespace.
- Reviewer quorum structure and policy identity are intentionally immutable
  within one history. A governance-constitution change requires a new anchor and
  a separately reviewed migration ceremony.
- No live Cargo, Nix, Bubblewrap, Chromium, network-namespace, DNS-rebinding, or
  production hybrid-signature tests were executed in this environment.
- A reviewed `flake.lock` remains required before release admission.

## Canonical target validation

```sh
python3 scripts/verify-wave7-static.py --check security/wave7-static-evidence.json
python3 scripts/verify-supply-chain.py --check security/supply-chain-evidence.json

cargo metadata --frozen --format-version 1
cargo fmt --all -- --check
cargo check --workspace --all-targets --frozen
cargo test --workspace --all-targets --frozen
cargo clippy --workspace --all-targets --frozen -- -D warnings

nix flake lock
# Independently review and commit flake.lock.
nix flake metadata --no-update-lock-file
nix flake check --no-update-lock-file
python3 scripts/verify-supply-chain.py --release
```

The target validation should additionally run a live Compatibility session in a
fresh namespace, prove direct TCP/UDP egress fails from the browser namespace,
exercise accepted and rejected broker destinations, verify audit counters after
concurrent tunnels, rotate reviewer keys through a signed history, rebuild the
indexes, and independently verify a hybrid-signed release statement.
