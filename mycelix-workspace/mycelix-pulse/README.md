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
