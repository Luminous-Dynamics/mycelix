# Mycelix Pulse

Mycelix Pulse is a working-alpha, agent-centric encrypted messenger built on
Holochain. The milestone is deliberately narrow: two distinct users should be
able to exchange, verify, recover, and reproduce one encrypted message
lifecycle without mock behavior being mistaken for live behavior.

## Canonical product path

- `apps/leptos/`: supported browser application.
- `holochain/`: Holochain 0.6 DNA and zomes.
- `crates/mail-leptos-types/`: serializer-independent protocol contracts.
- `tests/sweettest_mail_security.rs`: multi-agent lifecycle evidence.
- `mycelix-identity/crates/mycelix-crypto`: shared hybrid-PQC primitives.

The React/Node, legacy `happ`, mobile, extension, parallel backend, and
deployment prototypes remain reference surfaces. They are not supported alpha
paths. The desktop shell and SMTP gateway are experimental and independently
testable, but excluded from the alpha claim. See
[the surface inventory](docs/SURFACE_STATUS.md).

## Security profile

New live sends use the versioned `V2HybridPqc` envelope:

- X25519 and ML-KEM-768 hybrid key establishment.
- Domain-separated HKDF-SHA256 combiner.
- One AES-256-GCM ciphertext for the canonical subject/body payload.
- Holochain agent signature plus application-level ML-DSA-65 signature;
  recipients require both layers.
- Agent-signed, content-addressed V2 key bundles.
- Strict version/suite handling with no automatic classical downgrade.

V1 classical messages remain readable for compatibility and diagnostic
isolation. V2 private seeds are encrypted by a non-exportable Web Crypto
wrapping key and stored in IndexedDB. Recovery is same-browser-profile only;
there is no portable key export in this milestone.

The implementation is still behind its complete release-evidence gate. Pulse
does not claim a post-quantum Holochain substrate, a PQ ratchet, forward secrecy
against later static endpoint compromise, or post-compromise recovery. The
normative byte contract and exact exclusions are in
[the V2 crypto specification](docs/PULSE_V2_CRYPTO_SPEC.md).

## Experimental multi-transport contract

The isolated `crates/pulse-transport-core/` crate defines the first transport-neutral contracts for future adapters: opaque payloads with a 16 MiB contract ceiling, distinct `EnvelopeProfileId` and `TransportProfileId` values, send and non-blocking receive outcomes, separate monotonic link freshness and Unix-epoch expiry types, fail-closed consent/privacy/regulatory checks, failure-domain identity, and user-visible delivery-evidence stages. A `VerifiedRecipientReceipt` token is bound to the exact logical message ID, recipient, and immutable 32-byte envelope commitment. The configured verifier must authenticate that commitment; transport acknowledgements alone can never map to “Delivered.”

The crate is **experimental and not wired into the supported alpha send path**. It does not implement a radio driver or encrypt payloads. `delivery_state.rs` defines bounded in-memory retry/deadline semantics; `delivery_journal.rs` adds a storage-neutral append/CAS/event-ID/fencing contract and an in-memory reference model, including receipt evidence that retains encoded bytes and verifier-version metadata, owned snapshot/history reads, and a 256-event per-message cap. Neither module provides durable storage or production receipt verification. Contract tests are authored; do not infer test success until the exact-head artifact completes.

```sh
cargo test --locked --manifest-path crates/pulse-transport-core/Cargo.toml
```

## Durable delivery protocol design

The next-step design for durable outbox/inbox semantics is recorded in
[the durable delivery protocol](docs/DURABLE_DELIVERY_PROTOCOL.md). It specifies
stable logical identity, envelope commitments, event IDs, worker fencing,
receiver-side deduplication, receipt binding, and a crash/restart fault matrix.
This is a design contract for follow-on work, not evidence that the current
in-memory delivery lifecycle survives process or power loss. Track implementation
in [issue #5062](https://github.com/Luminous-Dynamics/mycelix/issues/5062).

## Synthetic routing evaluation

The synthetic trace harness compares preferred-single-path, sequential failover, and parallel redundancy. Its paired evaluator uses a declared development/holdout split, reports per-scenario attempts/cost/illustrative energy, and can sweep the common-draw mixture weight over 0, 2500, 5000, 7500 and 10000 for explicitly designated resilience scenarios. The weight is a synthetic sensitivity knob, not a calibrated empirical correlation coefficient. Short smoke runs scale declared outage windows across the whole reference trace so they include outage and recovery phases.

From the Pulse workspace root:

```sh
just simulate-transport-smoke
just evaluate-transport-smoke
just evaluate-transport
```

The smoke command uses 30 seeds and 500 messages per scenario; the heavier evaluation uses 30 seeds and 10,000 messages per scenario in release mode. Reports include the verified subject SHA, input-manifest path/digest, and generated validator lockfile. Evidence mode checks the subject against the checked-out Git HEAD, verifies every manifest entry with `sha256sum --check --strict`, and recomputes the manifest digest before running. The evaluator and tests are not considered passing until exact-head CI artifacts confirm execution. All route-success, confidence-interval, cost, and energy numbers are synthetic-model diagnostics—not authenticated delivery, RF behavior, or hardware measurements.

## Runtime truth

The UI exposes three states: `Live`, `Demo`, and `Unavailable`. Connection,
authentication, decoding, or zome-call failure never silently substitutes demo
records. Live inbox queries may truthfully return an empty inbox.

## Development

Prerequisites are Nix with flakes enabled and a browser with Web Crypto and
IndexedDB support.

```bash
cd mycelix-pulse
nix develop
just check
just build-dna
just dev
```

`just dev` serves the Leptos UI on `http://127.0.0.1:8117` and expects an
authenticated Holochain app WebSocket on `ws://localhost:8888` using installed
app ID `mycelix_mail` and role `main`.

Useful gates:

```bash
just test-protocol       # canonical envelope and receipt contracts
just test-crypto         # native + wasm hybrid-PQC implementation
just test-zomes          # integrity/coordinator unit tests
just build-dna           # fresh WASM build and DNA packing
just test-delivery       # bounded separate-conductor evidence
```

The complete working-alpha contracts are recorded in
[ADR-002](docs/adr/ADR-002-working-alpha-contracts.md). Directory presence,
ignored legacy tests, or a successful UI compile is not release evidence by
itself.

## License

AGPL-3.0-or-later. Commercial licensing terms are described by the repository
licensing files.

- Routing research: [ROUTING_RESEARCH_AGENDA.md](docs/ROUTING_RESEARCH_AGENDA.md) proposes constrained, evidence-carrying routing and the qualification sequence.
