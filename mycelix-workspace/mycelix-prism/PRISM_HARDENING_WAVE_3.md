# Prism Hardening Wave 3

Date: 2026-07-20

Wave 3 continues from `PRISM_HARDENING_WAVE_2.md`. Its purpose is to close the
remaining trust-boundary gaps between Prism's renderer, Spore bridge, imported
corpus, DHT publication path, Compatibility mode, and paid external providers.

## Patch order

Apply these commits in order:

1. `6d1ad5c bridge: authenticate IPC sessions and reject replay`
2. `ead28c0 ingest: add stable review and retraction ledger`
3. `4635ad6 search: bind index snapshots to corpus and review evidence`
4. `3cb934a privacy: bind DHT publication to exact claim evidence`
5. `ffd2192 compatibility: require an external ephemeral browser process`
6. `05d29f7 serve: keep provider secrets out of browser storage`

The commits are intentionally separated by invariant so each can be reviewed,
reverted, or tested independently.

## 1. Authenticated bridge sessions

The renderer↔Spore MessagePack envelope is now protocol version 4 and carries:

- a 128-bit session identifier;
- an independent strictly monotonic sequence for each direction;
- an explicit direction marker;
- a keyed BLAKE3 authenticator covering version, direction, session, sequence,
  length, and payload;
- content-bound encoding authorization revalidation after authentication.

Tampered, replayed, reordered, cross-session, wrong-direction, and wrong-key
frames fail before application payload decoding. `BridgeSessionKey` is
non-serializable and redacts its `Debug` output.

This patch supplies the authenticated codec and tests. Production process
launch/configuration must provision the same high-entropy key and session ID to
both endpoints; the repository still does not contain a complete Spore process
supervisor.

## 2. Review and retraction ledger

Imported claims now have stable canonical `ClaimId` values. A versioned JSON
ledger can:

- confirm a claim with reviewed E/N/M levels;
- reject a candidate before indexing;
- retract a claim that appeared in an earlier index.

Confirmations require a named reviewer, a substantive rationale, and at least
one evidence source. Equal-timestamp conflicts fail closed. The newest valid
record wins and exclusions happen before HDC indexing.

The embedded ledger begins empty at `prism-ingest/review/records.json`; its
README documents the review contract and operational workflow.

## 3. Evidence-bound index snapshots

The serialized search index is now wrapped in an `IndexSnapshot` with an
`IndexManifest` that binds:

- schema version;
- claim count;
- deterministic corpus digest;
- exact review-ledger digest;
- source-kind counts.

Loading validates the manifest before constructing a `SearchEngine`. Legacy
raw precomputed indexes can still be decoded deliberately, but are marked as
legacy/provenance-unknown and should be rebuilt.

## 4. Exact DHT publication artifacts

A publication permit is no longer merely permission to share a zone. It is
bound to a `PublicationArtifact` containing:

- the canonical claim ID;
- the exact byte-level BLAKE3 content hash.

Queued retries recompute and reauthorize both values. Editing a claim after
approval invalidates the permit. DHT records use deterministic claim-derived
IDs and carry provenance, review status, claim ID, and content-hash evidence
rather than timestamp identities.

## 5. Process-separated Compatibility mode

The trusted WASM reader no longer contains an iframe execution path.
Compatibility mode now fails closed unless the desktop host brokers an exact,
session-authorized HTTPS URL into a separate Chromium-family process.

The broker:

- requires an absolute canonical executable path;
- invokes the process directly without a shell;
- creates a fresh mode-0700 profile;
- disables extensions, sync, background services, notifications, component
  updates, translation, and selected networked browser features;
- uses a session-bound launch permit;
- removes stale temporary profiles.

This is meaningful renderer/process separation, not a claim of a complete OS
sandbox. The process still shares the user's network namespace and host kernel.
A future deployment should add namespaces/seccomp or an equivalent platform
sandbox around the brokered process.

## 6. Server-owned provider credentials

Brave Search and Perplexity secrets have been removed from browser storage and
client-controlled headers in both `prism-serve` and the legacy
`prism-proxy`.

The new contract is:

```text
PRISM_ENABLE_PROVIDER_PROXY=true
BRAVE_SEARCH_API_KEY=...
PERPLEXITY_API_KEY=...
```

Without explicit enablement, provider routes return 503. When enabled:

- the browser sends only bounded query text;
- Perplexity model, roles, and provider payload are constructed server-side;
- provider redirects are disabled;
- response bodies are streamed under a 2 MiB cap;
- responses are `Cache-Control: no-store`;
- provider routes have a separate concurrency ceiling;
- old prefixed and unprefixed browser credentials are deleted at UI startup;
- CORS no longer admits arbitrary custom headers in the standalone proxy.

The built-in concurrency bound is not billing-grade abuse prevention. Public
operators should add authenticated sessions and per-account quotas at their
reverse proxy before enabling paid providers.

## Security invariants after Wave 3

1. A bridge frame is not trusted solely because it came from a Unix socket.
2. Review and retraction decisions address stable claim identities, not array
   positions.
3. An index snapshot can identify exactly which corpus and review ledger built
   it.
4. A DHT publication permit cannot authorize edited claim bytes.
5. Active web content cannot execute inside the trusted Prism reader.
6. A browser cannot supply or retain paid-provider credentials.
7. External provider endpoints are disabled unless the operator explicitly
   enables them.

## Recommended merge and deployment sequence

1. Merge the six commits in order.
2. Rebuild the corpus and indexes so manifests and review status are current.
3. Provision bridge secrets through the process supervisor; do not pass them on
   command lines or serialize them into configuration artifacts.
4. Configure an absolute `PRISM_COMPAT_CHROMIUM_PATH` only on desktop hosts that
   intentionally support Compatibility mode.
5. Leave provider proxying disabled until reverse-proxy authentication and
   quota policy are defined.
6. Run the canonical Cargo, Clippy, WASM, Tauri, and integration lanes listed in
   `PRISM_HARDENING_WAVE_3_VALIDATION.md`.

## Highest-leverage next wave

The next work should be narrower rather than broader:

- wire `BridgeCodec` into the real Spore process lifecycle and key provisioning;
- place Compatibility mode inside an OS-level sandbox/network policy;
- add signed reviewer identities and append-only transparency for the review
  ledger;
- make index publication atomic and expose its manifest in the UI;
- add authenticated, per-account provider quotas for public deployments;
- create end-to-end adversarial tests spanning consent → encoding → IPC → index
  → publication rather than testing each crate only in isolation.
