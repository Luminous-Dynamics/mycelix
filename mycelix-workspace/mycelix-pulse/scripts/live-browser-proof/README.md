# Live-browser Live-mode proof

Drives the actual `apps/leptos` browser build against two real, independent
Holochain conductors — the piece of the alpha that has never been exercised
outside a Sweettest: the browser's WASM hybrid-PQC crypto, IndexedDB key
wrapping, and WebSocket conductor handshake.

Confirmed working in this environment (2026-07-15/16): headless Chromium,
`npm install playwright` against the system Chromium, `trunk build --release`
for the UI, and a matching `holochain`/`hc` 0.6.1 pair.

**Blocked as of 2026-07-16, not yet run end-to-end.** `launch-conductors.sh`
itself is unreliable in this environment: `hc sandbox generate`/`run`
sometimes exits immediately with `Error: No such device or address (os
error 6)` on the admin-interface WebSocket bind, and sometimes exits
silently with no error at all a few seconds after printing "Admin port set
to: N" — neither reproduces consistently enough to have been root-caused
yet. This is a **different** class of problem from the Sweettest suite's
earlier host-contention issues (see `ALPHA_EVIDENCE.md`): the in-process
`SweetConductor` conductors those tests use bind reliably every time, and a
real, independent conductor (`mail-conductor.service`, admin port 33800) is
already running successfully on this same box — so it's not a blanket
"conductor binding doesn't work here" problem, but something specific to
how `hc sandbox`'s ephemeral admin-interface setup interacts with this
environment. Worth trying next: `RUST_LOG=info` / `--structured Json` for
more detail from `hc` itself, an explicit (non-zero, non-`-f`-overridden)
admin port from the start, or filing upstream if it reproduces outside this
sandboxed dev environment too. `launch-conductors.sh`'s fixed `-f` position
(must come after `sandbox`, not before — see the script) is a real,
separate bug that was found and fixed along the way; the binding issue
above is unrelated and still open.

## Steps

```bash
cd /path/to/mycelix-pulse
just build-happ                       # fresh restricted alpha hApp
just build-ui                         # fresh apps/leptos/dist/

cd scripts/live-browser-proof
./launch-conductors.sh                # Alice on :4444/:4445, Bob on :4446/:4447
python3 -m http.server 8409 --directory ../../apps/leptos/dist &

npm install playwright                # or reuse an existing scratch install
node live-proof.mjs

./stop-conductors.sh
```

## What it proves

- Real `Live` runtime status (not `Demo`/`Unavailable`) against a real
  conductor, using the app's actual `window.__HC_CONDUCTOR_URL` override —
  no application code changes needed to run two identities.
- Real onboarding: device-local ML-KEM/ML-DSA/X25519 key generation,
  non-exportable Web Crypto wrapping, IndexedDB persistence, and
  `publish_hybrid_key_bundle_v2` — all previously compiled but never
  actually executed in a browser.
- A real compose → hybrid-seal → `send_email_v2` → gossip →
  `get_inbox_v2` → ML-DSA-verify → decrypt → render round trip between two
  distinct agents.

## What it does not prove

- Browser restart recovery (reload the page, confirm the wrapped key survives
  via IndexedDB) — not implemented in `live-proof.mjs` yet; a natural next
  step once the base round trip is confirmed working.
- Any of the negative/hostile-content browser cases listed in
  `ALPHA_EVIDENCE.md` (forged envelopes, revoked keys, malformed ciphertext,
  hostile HTML).

## Known risk

The selectors in `live-proof.mjs` were read directly out of the Leptos
source (`profile_setup.rs`, `compose.rs`, `settings.rs`, `nav.rs`,
`rich_editor.rs`) — not guessed — but the script has not yet been run
against a live app. Treat it as a strong first draft. If a selector doesn't
match, that's real information about the UI, not necessarily a bug in this
script; verify against current source before assuming either side is wrong.
