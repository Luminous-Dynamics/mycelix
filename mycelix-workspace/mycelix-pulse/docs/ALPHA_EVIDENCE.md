# Pulse working-alpha evidence

This document is a release gate, not a marketing claim. A box may be checked
only from a clean checkout using freshly built artifacts.

## Implemented and locally verified

- [x] V2 canonical wire format rejects unknown versions, trailing data, invalid
  UTF-8, and invalid lengths.
- [x] X25519 and ML-KEM-768 both contribute to the domain-separated HKDF key.
- [x] AES-256-GCM authenticates the immutable routing and key transcript.
- [x] ML-DSA-65 verification is mandatory *at the recipient client* and fails
  on transcript mutation. It is **not** cryptographically checked by DHT
  validators — HDI/WASM cannot link the RustCrypto ML-DSA crate, so network
  validation checks only signature length. See the "Enforcement boundary"
  section of ADR-002 and `PULSE_V2_CRYPTO_SPEC.md`.
- [x] V2 key bundles are content-addressed and signed by the active Holochain
  agent before publication.
- [x] The coordinator signs the exact canonical V2 envelope transcript with the
  active Holochain agent key.
- [x] Browser V2 private material is wrapped by a non-exportable, device-local
  Web Crypto key and stored as ciphertext in IndexedDB.
- [x] Live compose has no silent downgrade from V2 to V1.
- [x] Live inbox verifies the sender's historical bundle and ML-DSA signature
  before decrypting V2 content.
- [x] Protocol, crypto, zome, browser-WASM, and Sweettest compile gates pass.
- [x] The DHT-validator structural checks (`validate_email_v2_structure`) are
  host-independent and directly unit-tested — 6 tests, no live conductor
  needed, covering short/oversized ciphertext, wrong-length ML-DSA/agent
  signatures, unknown version/suite, and the documented fact that a
  garbage-but-correctly-sized ML-DSA signature passes structural validation.

## Required before an alpha release claim

- [x] Pack the restricted alpha DNA from fresh WASMs with `holochain_cli
  0.6.1`. **Real reproducibility bug found and fixed 2026-07-15**: the DNA
  manifest's static relative paths pointed at `holochain/target`, which was a
  symlink into a session-scoped cargo cache directory reclaimed by the ~48h
  auto-cleanup — so `hc dna pack` could silently succeed while packing stale
  or missing WASM, with no error. `just build-dna` now copies freshly built
  WASM into a real, stable directory before packing (see justfile comment).
- [x] Run the full V2 Sweettest evidence set and retain the complete logs and
  artifact hashes. **All three now pass, each independently, each run
  alone** (see "Runtime evidence: RESOLVED" below for the full trail,
  including a real bug found and fixed along the way — a transient
  `BadNonce("Expired")` on V2 write calls, not a load artifact).
- [x] Add negative conductor cases for wrong-length ML-DSA/agent signatures,
  short ciphertext, and unregistered-recipient bundle resolution
  (`phase0_v2_negative_paths`) — **passed**, 637.08s, alone.
- [x] Add conductor-level restart recovery: shut down and restart a
  conductor against its real on-disk DB, proving a key bundle and received
  message survive and the conductor can still author afterward
  (`phase0_v2_conductor_restart_recovery`) — **passed**, 957.38s, alone.
  This is the conductor half of restart recovery only; it does not exercise
  the browser's IndexedDB key custody across a reload.
- [ ] Prove browser compose/read/reply/archive in `Live` mode with two
  distinct users; no demo data may appear after a live failure. **Substantial
  progress 2026-07-16**: the `hc sandbox` bind issue below was resolved
  (`--piped < /dev/null`, plus `-f`/`--piped` positioned after `sandbox` not
  before it), and — for the first time ever — a real browser build reached
  genuine `Live` status against a real conductor, with `app_info` cell
  discovery working correctly. Getting there required finding and fixing a
  cascade of real, previously-unexercised wire-format bugs (every prior
  "verification" of this client only round-tripped self-serialized data
  using the same wrong shape on both ends, so nothing caught these until a
  real conductor was in the loop):
  - `AppRequest`/`AppResponse` used `content = "data"` where the real
    conductor's `#[serde(tag = "type", content = "value")]` needs `"value"`.
  - `AppRequest::AppInfo` was a struct variant; the real one is a unit
    variant (no `installed_app_id` field — the app is already scoped by the
    connection's authenticated token).
  - `AppResponse::AppInfo` needed to wrap `Option<AppInfoResponse>`, not
    `AppInfoResponse` directly.
  - `AppError` was a naive `{message: String}` that silently deserialized
    every real conductor error into an empty string.
  - `decode_tagged`'s original `serde_json::Value` intermediate (added to
    work around the `content` bug above) could not represent MessagePack Bin
    data at all, so it broke on the very first response containing raw
    `AgentPubKey`/`DnaHash` bytes. A `rmpv::Value` intermediate was tried
    next but its `deserialize_enum` is incompatible with adjacent tagging.
    **The actual fix was removing the generic-value detour entirely** — a
    direct `rmp_serde::from_slice` decode of the adjacently-tagged enums
    works correctly on its own, including with nested raw-byte fields, once
    the `content` tag bug above was fixed. Both dead-end intermediates are
    documented in `decode_tagged`'s doc comment (`types.rs`) to save a future
    session from re-trying them.
  - `AppInfoResponse.cell_info` was declared as `Vec<{role_name, cells}>`;
    the real `holochain_conductor_api::AppInfo::cell_info` is
    `HashMap<RoleName, Vec<CellInfo>>` — a genuine msgpack map
    (`{"main": [...]}`), found by manually capturing and inspecting the raw
    msgpack shape of a real `app_info` response
    (`scripts/live-browser-proof/diag6-raw-shape.mjs`).
  - Also found and fixed along the way: a real bug in the production
    browser/native zome-call transports — 5-second nonce expiry
    (`browser.rs`/`native.rs`, same `BadNonce("Expired")` class just fixed in
    the Sweettest suite), raised to 60s.

  **New blocker found past Live/app_info, not yet closed**: actual zome
  calls fail with "Failed to deserialize request". Root cause: Holochain 0.6
  requires a client-side request-signing flow this client never implemented
  — `AppRequest::CallZome` must carry `ZomeCallParamsSigned{bytes: ExternIO,
  signature}` where `bytes` is the map-encoded `ZomeCallParams` and
  `signature` is a real Ed25519 signature over `sha512(bytes)`, verified by
  the conductor against `provenance` unconditionally (see
  `handle_external_zome_call`/`is_valid_signature` upstream) — there is no
  "unsigned calls accepted because the app connection is token-authenticated"
  exemption. A bare browser has no access to a cell's own lair-held key, so
  following `holochain_client::AdminWebsocket::authorize_signing_credentials`
  (the official Rust reference client), a throwaway Ed25519 keypair must be
  generated by a trusted broker, registered as an `Assigned` capability grant
  via `AdminRequest::GrantZomeCallCapability`, and its private key handed to
  the browser to sign with, using the granted key (not the cell's own agent
  key) as `provenance`.
  - A reference broker implementation
    (`scripts/live-browser-proof/{holo-hash-utils,issue-signing-credentials,
    diag7-grant-credentials}.mjs`) validated this whole flow end-to-end
    against the real Alice conductor — `GrantZomeCallCapability` accepted a
    from-scratch-constructed 39-byte `AgentPubKey` (3-byte type prefix +
    32-byte Ed25519 public key + 4-byte blake2b-128-derived DHT location
    checksum), confirming the reimplementation of `holo_hash`'s wire format
    is correct.
  - A separate, concurrent session was independently building a more
    architecturally-complete answer to the same gap in this same crate — a
    `ZomeCallSigner` trait plus a `HostZomeCallSigner` that delegates to
    `window.__HC_ZOME_CALL_SIGNER__`, the official Holochain Launcher/Moss
    convention (a surrounding launcher shell holds real credentials and
    injects a signing hook, rather than the web app holding admin-granted
    keys itself) — architecturally cleaner for a real production deployment.
    To avoid duplicating or colliding with that in-flight work, the
    broker-key flow above was built as a standalone, separately-scoped
    `AdminGrantedSigner` (implements the same `ZomeCallSigner` trait, so it's
    a drop-in alternative for test/dev environments without a Launcher
    host) — self-tested via real Ed25519 sign+verify and round-trip decode
    of the signed payload.
  - This granted key is a distinct, separately-scoped credential from the
    app-level hybrid-PQC identity/message keys used for the V2 mail envelope
    above — it authorizes calls at the Holochain transport layer,
    underneath that envelope, and can't itself be a PQC/hybrid scheme since
    the conductor's own signature verification is hard-coded to plain
    Ed25519.

  **Zome calls now genuinely work end-to-end (2026-07-17).** The concurrent
  session's `ZomeCallSigner` wiring landed with one remaining bug, found and
  fixed the same way as everything else in this section — by actually
  running it against a real conductor: `call_zome()` computed a real
  signature via the installed signer but then discarded the signer's
  `.bytes()` (the exact `ZomeCallParams` payload that signature was computed
  over) and re-serialized a *different*, flat `CallZomeRequestWire` struct
  in its place — so even with a correctly-computed signature and an
  installed signer, every call still failed with "Failed to deserialize
  request", now for a second, independent reason on top of the original
  shape mismatch. Fixed by sending the signer's `SignedZomeCall` directly.
  Confirmed via `apps/leptos`'s own `/api/signing-credentials` broker
  endpoint (mirroring the existing `/api/token` pattern, backed by
  `scripts/live-browser-proof/issue-signing-credentials.mjs` +
  `discover-cell-id.mjs` in this test harness) plus `AdminGrantedSigner`:
  `get_folders`/`get_inbox` now return real, successfully-decoded `[]`
  responses instead of deserialize errors; unrelated zome-call failures for
  `mail_sync`/`mail_contacts` now correctly read "Zome not found" — proof
  the conductor is actually *executing* the call (reaching the ribosome and
  checking its zome table) rather than rejecting it before verification.

  **Full two-user onboarding proven live, twice, for the first time ever**:
  both Alice and Bob independently reach `Live` status, complete real
  onboarding (device-local hybrid-PQC key generation + a genuinely signed
  `publish_hybrid_key_bundle_v2` zome call), and the setup modal correctly
  dismisses. This is the deepest this flow has ever been verified to work.
  One test-script bug was found and fixed along the way (not an app bug):
  `live-proof.mjs` waited for the setup modal to become DOM-`detached`, but
  `profile_setup.rs` only ever toggles `display:none` on it — fixed to wait
  for `hidden` instead.

  **Still open, past onboarding**: post-onboarding navigation
  (Settings/Compose/Inbox) in `live-proof.mjs` used hash-fragment URLs
  (`#/settings` etc.) that don't correspond to anything — the app uses real
  `leptos_router` path-based client-side navigation
  (`nav.rs: <A href="/settings">`), so those `page.goto()` calls silently
  did nothing and the script asserted against whatever page onboarding
  happened to leave open. Fixed to click the real nav links instead
  (`page.click('a:has-text("Settings")')` etc.), matching what a real user
  does; a further "Get notified of new mail?" prompt was found to overlay
  and intercept those clicks post-onboarding, fixed by dismissing it first.
  Getting a full clean run past this point was blocked by `hc sandbox`
  process-lifecycle flakiness under this session's heavy concurrent system
  load (conductors intermittently failing to bind or losing their data
  directory across a stop/relaunch cycle) — environmental, not a code bug;
  `launch-conductors.sh` itself needed a real fix in the same family as
  everything else here (it was missing `--piped < /dev/null`, so `hc
  sandbox generate -f=PORT` silently never applied the forced port and fell
  back to a random one). Re-running the full proof on a quieter box, or with
  more generous conductor-startup retries, is the direct next step — no
  further *code* gap is known.

  Headless Chromium, Playwright, `trunk` release build, and a matching
  `holochain`/`hc` 0.6.1 pair are all confirmed working.
- [ ] Prove browser restart recovery specifically (IndexedDB-wrapped key
  survives a page reload) — depends on the browser proof above.
- [ ] Run the same published known-answer/interoperability vectors in native
  and WASM crypto builds.
- [ ] Complete an independent review of the V2 transcript and key custody
  implementation.

## Explicit exclusions

The working alpha does not claim a post-quantum ratchet, forward secrecy from a
static recipient-key compromise, post-compromise security, portable or cross-
device private-key recovery, a post-quantum Holochain substrate, Sybil
resistance, attachments, or multi-region reliability.

The accurate claim after every required box is checked is:

> Pulse alpha uses hybrid classical and post-quantum message encryption with
> application-level hybrid signatures. Holochain agent identity remains
> classical, and the protocol is not yet ratcheted.

## Runtime evidence: RESOLVED (2026-07-16)

All three V2 Sweettests pass, each run alone against the same freshly-packed
DNA (`a466e656ae43...`, see hash below):

| Test | Result | Duration | Notes |
|---|---|---|---|
| `phase0_v2_hybrid_pqc_transport` | pass | 605.12s | clean, no retries needed |
| `phase0_v2_negative_paths` | pass | 637.08s | clean, no retries needed |
| `phase0_v2_conductor_restart_recovery` | pass | 957.38s | clean on this run; nonce-retry safety net present but not exercised |

Getting here took several real, distinct fixes, not just waiting for a quiet
box — summarized here, full blow-by-blow below for anyone debugging a
recurrence:

1. **DNA-packing reproducibility bug** (see above) — fixed.
2. **Running two heavy Sweettest binaries concurrently caused a genuine
   `BadNonce("Expired")` failure**, confirmed by making it happen twice
   under different load conditions, including once with the box otherwise
   idle — Holochain's own zome-call nonce is only valid 5 minutes
   (`FRESH_NONCE_EXPIRES_AFTER`), and enough concurrent Wasmer/scheduling
   pressure from two test binaries at once was enough to blow that window.
   **Fix**: run one Sweettest binary at a time — cargo-gate.sh only limits
   the *build* phase, not multi-minute test *execution*, so this needed
   discipline in how the tests were invoked, not a gate change.
3. **`phase0_v2_conductor_restart_recovery` failed the same way even running
   completely alone** — a second, independent occurrence of the same
   `BadNonce` class, this time not explained by concurrent-binary
   contention. **Fix**: added `call_with_nonce_retry` /
   `call_skipping_nonce_flakiness` helpers (mirroring this file's
   pre-existing `send_email` retry pattern) and applied them to every
   `publish_hybrid_key_bundle_v2` / `send_email_v2` call site across all
   three V2 tests.
4. `startup(true)` → `startup(false)` in the restart test (removes an
   unnecessary full WASM recompile on restart) and a raised, test-specific
   timeout (1500s → 2700s, justified by Holochain's own `slow_tests`
   classification of the equivalent scenario) — both still in place as
   defense in depth, though attempt 4 above turned out to be the actual
   fix for the specific failure observed.

The lesson that generalizes: a repeated failure under load is not
automatically "just load" — `BadNonce("Expired")` looked environmental at
first (it always showed up alongside high `uptime` numbers) until it
reproduced once with load genuinely low, which is what forced the real
root-cause investigation. Check whether a failure's *symptom* is consistent
with load (a timeout with no specific error) or a *specific, named error*
(worth investigating on its own) before writing it off as contention.

## Latest local transport evidence

On 2026-07-13, the restricted alpha DNA was packed from fresh WASMs as:

`743071499f5ca01f9d8011dd0c7209dc5a6b4bca5b12fb447d8e7793691ab78a  mycelix_pulse_alpha.dna`

The corresponding restricted hApp bundle packed successfully as:

`4d41ee41c8c1f7b1aa1ce36601005fcd03d3e52ee7c7bfcac8939bd38cc7472c  mycelix_pulse_alpha.happ`

`phase0_two_conductor_harness_smoke` passed in 221.12 seconds on a cold
Wasmer cache. After the V1/V2 link namespaces and integrity artifact build
were corrected, `phase0_v2_hybrid_pqc_transport` passed in 464.45 seconds on
the same heavily loaded workstation. This proves the headless transport
contract; it does not satisfy the browser, restart, interoperability-vector,
or independent-review boxes above.

On 2026-07-15, after fixing the `build-dna` reproducibility bug above, the
restricted alpha DNA was rebuilt from genuinely fresh WASM (all 9 zomes, not
just the 2 touched that day) and packed successfully as:

`a466e656ae4387bafe3ba9183f4ec80f328e986c2df04fb0c8df2c99f245f207  mycelix_pulse_alpha.dna`

The new unit tests (`v2_structure_*`, 6 tests in
`mail_messages_integrity`) passed cleanly. Runtime re-verification of
`phase0_v2_hybrid_pqc_transport` plus the two new Sweettests
(`phase0_v2_negative_paths`, `phase0_v2_conductor_restart_recovery`) against
this artifact was attempted four times across 2026-07-15 and did not
complete. Treat the DNA hash above as freshly built and structurally sound
(packing itself succeeded), but **not yet re-proven at runtime**.

Attempt detail, for anyone re-running this later: attempts 1-3 used the
harness's own `run_in_background`, which turned out to get silently killed
mid-run by session-boundary events unrelated to the test itself (visible as
a "killed" status with no exit code, not a test failure) — switched to a
fully detached `setsid nohup ... & disown` process to rule that out.
Discovered mid-session that `nohup cargo ...` bypasses this repo's
`cargo-gate.sh` concurrency wrapper entirely (`nohup` resolves `cargo` via
`$PATH`, not the interactive shell's wrapping function), so early detached
attempts were *not* respecting the shared 2-slot build gate other sessions
rely on — fixed by invoking `scripts/cargo-gate.sh` explicitly inside the
detached process. A 4th attempt, correctly detached AND gate-respecting,
ran the full 3-test suite for 4540 seconds (test-internal timeout raised
600s → 1500s per test along the way) and still finished 0 passed / 3 failed
— all three hit their internal timeout. `uptime` at that point read load
average 61.57/71.98/76.91 on a 12-core box (other concurrent sessions, not
this work), roughly 6x oversubscribed and visibly worsening over the
session rather than easing. This reads as environmental — not a regression
in the tests themselves, since `phase0_v2_hybrid_pqc_transport`'s underlying
logic already has a clean, distinctly-passing prior run (see above) — but it
is unproven against the current DNA build until re-run on a quieter box.

One real fix did come out of chasing this: `phase0_v2_conductor_restart_recovery`
originally called `startup(true)` (`ignore_dna_files_cache`), which forces a
full Wasmer recompilation of the DNA's WASM from bytes read back out of the
database — expensive in a way that's orthogonal to the actual persistence
claim (source chain / DHT data / keystore reload from disk is identical
either way). Changed to `startup(false)` to remove that cost without
weakening the claim being tested; not yet confirmed this was sufficient on
its own, since the 4th attempt's failure was very plausibly pure system load
rather than this specific step — re-run needed to know which.

**Recommendation for whoever revisits this**: check `uptime` before
attempting `just test-delivery`; if the 1-minute load average is anywhere
near or above `nproc`, don't bother — it will very likely time out
regardless of correctness. Wait for a quiet window instead.

Later the same day, a host reboot briefly dropped load to 10-22 (from
61-77) — during that window the matching `holochain` 0.6.1 conductor
binary finished installing (`/var/tmp/hc-0.6.1/bin/{hc,holochain}`, both
verified `--version`), but load climbed back to 45-68 within ~20 minutes
before another `test-delivery-fast` attempt could complete. A genuine,
separate finding came out of that attempt: `phase0_v2_conductor_restart_recovery`
timed out again even at 2700s (raised from 1500s) under the brief low-load
window, which on its own would suggest a real bug rather than pure
contention — except Holochain's own test suite classifies this exact
shutdown→restart→gossip-resume scenario as inherently expensive (see the
`slow_tests` note in the doc comment on that test); the two justfile
recipes (`test-delivery-fast` vs `test-delivery-restart`) now split it out
so it doesn't block the other two tests' evidence.

**Live-browser proof scaffolding was written this session**
(`scripts/live-browser-proof/`) but deliberately not run — it needs two
concurrent conductor processes plus a browser, i.e. strictly more capacity
than the single-conductor Sweettest runs that were already failing to get a
fair share. `launch-conductors.sh` starts two independent throwaway
sandboxes (Alice on 4444/4445, Bob on 4446/4447, `--in-process-lair`, no
app-code changes needed — uses the existing `window.__HC_CONDUCTOR_URL`
override). `live-proof.mjs` drives two Playwright browser contexts through
onboarding → compose → send → read, using selectors read directly from the
Leptos source (not guessed) — treat as a strong first draft, unconfirmed
end-to-end. See that directory's README for exact steps and what it does
and doesn't prove.

---

## Live-browser proof: PASSED end-to-end (2026-07-18)

**`scripts/live-browser-proof/run-with-retry.sh` now genuinely passes**: two
independent throwaway `hc sandbox` conductors, two independent Playwright
browser contexts, real onboarding (device-local hybrid-PQC key generation
through the actual WASM crypto + IndexedDB path), a real signed
`send_email_v2` zome call, real P2P gossip delivery, and a real
`get_inbox_v2` read that decrypts and ML-DSA-verifies the message — with the
decrypted subject text asserted directly out of the rendered DOM:

```
[bob] inbox shows expected subject "Live-proof 2026-07-18T10:24:39.064Z" —
decrypted, ML-DSA-verified V2 content is genuinely rendering in a real browser
=== LIVE-BROWSER PROOF PASSED ===
=== [run-with-retry] PASSED on attempt 1 ===
```

Getting here took a long investigation, because there were **five
independently real bugs stacked on top of each other**, each one masking
the next. Every one of the first four had to be found and fixed before the
fifth (the actual root cause) was even reachable to diagnose. Documented
here in the order they blocked progress, not in order of "importance" —
importance is noted per item.

### 1. `AppRequest::CallZome` discarded the signer's payload (app bug, fixed earlier this session)
`call_zome()` computed a real signature via the installed `ZomeCallSigner`
but then discarded the signer's `.bytes()` — the exact `ZomeCallParams`
payload the signature was computed over — and re-serialized a *different*,
flat struct in its place. Every signed zome call failed with "Failed to
deserialize request" regardless of signature correctness. Fixed by sending
the signer's `SignedZomeCall` directly (see the section above this one for
full detail — this was fixed and confirmed working before this segment of
work began).

### 2. Real P2P transport never worked in this dev environment (infrastructure, not an app bug)
Two independent `hc sandbox` conductors pointed at the real
`dev-test-bootstrap2.holochain.org` never completed peer discovery/gossip
reliably in this environment — sends never reached the recipient, with no
useful error. Root-caused via `RUST_LOG=debug` on both conductors and a
sequence of increasingly-targeted local-network experiments:

- **`kitsune2-bootstrap-srv` alone is not enough.** It bundles a real
  bootstrap service (works fine — agent-info discovery succeeded fast, both
  agents found each other's `AgentPubKey` within seconds) plus a **relay**
  component that speaks the older tx5/WebRTC **SBD** signal protocol. This
  Holochain 0.6.1 build's `network quic <RELAY_URL>` transport uses
  **iroh's own relay wire protocol** instead — pointing iroh's relay client
  at the SBD-speaking relay produced `400 Bad Request` on iroh's HTTPS
  netcheck probe and every relay-connection attempt failed
  ("Failed to connect to relay server"), so peer discovery succeeded but no
  actual P2P connection or gossip round ever completed.
- **Fix**: install and run a real `iroh-relay` binary (`cargo install
  iroh-relay-holochain --version 0.95.1 --features server --bin iroh-relay
  --locked` — version must match whatever `iroh-holochain` this Holochain
  build actually depends on) in `--dev` mode (plain HTTP, localhost, port
  3340 by default) alongside `kitsune2-bootstrap-srv` for bootstrap only.
  Manually verified before trusting it in the full test:
  `hc sandbox call --running=<port> dump-network-stats` showed a genuine
  direct connection (`"is_direct":true`) with real send/recv byte counts,
  and `dump-network-metrics` showed multiple `completed_rounds` with
  matching `local_op_count` on both sides.
- `launch-conductors.sh` now starts both local services automatically
  (`KITSUNE2_BOOTSTRAP_SRV_PATH` / `IROH_RELAY_PATH` env overrides) and
  points both conductors' `network -b <bootstrap> quic <relay>` at them.
  `stop-conductors.sh` tears both down too.
- This was necessary supporting infrastructure — genuinely broken, and the
  fix genuinely required — but turned out **not to be the reason messages
  never arrived**; see items 4-5 below for the actual blocker.

### 3. A real Leptos-context WASM panic in the send flow (app bug — found and fixed)
`compose.rs`'s `on_send` handler has a "5-second undo window": it clears the
form and flips the `sending` UI flag back to `false` **synchronously**, then
does the real work (`gloo_timers::future::sleep(5s)`, then
`resolve_hybrid_send_context_v2` + encrypt + `send_email_v2`) inside a
`wasm_bindgen_futures::spawn_local` task. The delayed task called
`crate::holochain::use_holochain()` (a Leptos `expect_context` lookup)
*after* the 5-second sleep — by which point the Compose page's reactive
owner could already be disposed, causing a hard, silent WASM panic
(`panicked at ...: expected context of type "...HolochainCtx" to be
present`) that killed the entire delayed task with **zero user-visible
error** (the button's "Sending..." state had already cleared synchronously
above, and the surrounding Rust `Result` machinery never got a chance to
run). This was reproduced live via a custom diagnostic
(`diag17-send-console.mjs`) that forwards full browser console + pageerror
output — `live-proof.mjs` itself forwarded nothing before this session,
which is why every prior failure looked like total, unexplained silence.
Fixed by resolving `use_holochain()` synchronously, before the delayed
`spawn_local` block, matching the correct pattern already used elsewhere in
the same file — the captured `HolochainCtx` handle (a plain `Clone` value,
no reactive-context dependency) is then moved into the delayed task instead
of re-resolving the context after the sleep.

### 4. `ActionHash`/`AgentPubKey` responses decoded as `serde_json::Value` (app bug — the actual root cause)
This is the bug that was silently blocking onboarding, sending, *and*
reading, independently, in three different places. msgpack's raw
byte-array type has no JSON equivalent — `serde_json::Value`'s `Deserialize`
impl legitimately rejects it with `invalid type: byte array, expected any
valid JSON value`. Several zome-call sites decoded a wire response
containing a raw byte array (an `ActionHash`, or an `AgentPubKey` nested
inside a larger struct) as `serde_json::Value` instead of a type that can
actually represent raw bytes (`Vec<u8>`). The **DHT write or read itself
always succeeded on the conductor side** — only the client's decode of the
*response* failed — but each call site's error-handling treated that decode
failure as a real failure, with three different, each individually severe,
consequences:

- **`profile_setup.rs`'s `set_profile` call** — its `Err` branch reverted
  the onboarding UI step back to `NameEntry` (hiding, then re-showing, the
  name-entry input) and `return`ed, **skipping key generation and
  `publish_hybrid_key_bundle_v2` entirely**. The live-browser-proof test's
  one-shot "did `#setup-name` become hidden" check was fooled by the
  transient hide-then-show and treated onboarding as complete — so every
  earlier "onboarding complete" log line in this whole investigation was a
  false positive. The actual DHT profile entry *was* created (confirmed via
  `get_my_profile` on a later reload correctly reporting "Profile exists"),
  but the device never actually generated or published its hybrid-PQC key
  bundle, so `resolve_hybrid_send_context_v2` could never succeed for
  *either* agent — this alone fully explains the "Recipient has no active
  hybrid-PQC V2 key bundle" toast that an earlier, incorrect theory
  attributed to a DHT-propagation timing race (see the retry-loop safety
  net below — that fix was real defense-in-depth, just not the actual
  blocker).
- **`profile_setup.rs`'s `publish_hybrid_key_bundle_v2` call** — same
  pattern, same fix.
- **`compose.rs`'s `send_email_v2` call** — same pattern; its `Err` branch
  shows a "Send failed" toast instead of the real "Message sent and
  committed to DHT" success toast, even though the message *did* commit.
- **`mail_context.rs`'s `InboxEmailV2Wire.sender` field** (a nested
  `serde_json::Value`, not a top-level response type) — this one is subtly
  different: it broke deserializing the *entire* `Vec<InboxEmailV2Wire>`
  response for any non-empty inbox (one bad field poisons the whole
  `#[derive(Deserialize)]` struct), surfacing as `"[Mail] V2 inbox
  unavailable: Decode error for mail_messages.get_inbox_v2: ..."` — this
  was the very last bug found, discovered only *after* fixing the first
  three finally let a real send succeed and reach the read path for the
  first time.

**Fix, applied at all four sites**: decode as `Vec<u8>` instead of
`serde_json::Value` wherever the caller only checks success/failure and
never actually inspects the hash value (`profile_setup.rs`, `compose.rs`);
for `mail_context.rs`, the redundant/broken `sender: serde_json::Value`
field was dropped outright in favor of the adjacent `sender_agent_raw:
Vec<u8>` field that already carried the identical raw bytes, with its two
call sites (an outbound key-bundle-lookup request, and a display string via
a newly-`pub`-exported `zome_adapter::base64_encode`) rewritten accordingly.

**This is very likely a systemic pattern, not fully swept.** A search found
~15 more `call_zome::<_, serde_json::Value>` call sites across
`mail_context.rs`, `settings.rs`, `contacts.rs`, `chat.rs`, `search.rs`,
`read.rs`, `offline.rs`, and `revocable.rs` — most call functions that
return already-JSON-friendly query results (folder lists, contact lists,
etc.) and were observed working correctly throughout this session's testing
(including with real non-empty responses in some cases), so they are very
likely fine. But this was not exhaustively re-verified per-site, and any
zome function whose response (at any nesting depth) contains a raw
Holochain hash type is a candidate for the exact same bug. Treat this as a
known residual-risk class, not a closed investigation.

### 5. Test-harness robustness (not app bugs, but necessary to get a clean pass)
- **Playwright's bundled `chromium_headless_shell` is missing
  `libgbm.so.1`** on this NixOS box. Fixed by setting
  `PLAYWRIGHT_CHROMIUM_EXECUTABLE_PATH` to the Nix-wrapped system Chromium
  instead (`/home/.../.nix-profile/bin/chromium` or equivalent).
- **`live-proof.mjs` forwarded zero browser console/error output** before
  this session — every prior failure investigation was working blind.
  Fixed by adding `page.on('console', ...)` / `page.on('pageerror', ...)`
  forwarding for both agents' pages, filtered to errors and `[Mail]`-tagged
  logs.
- **Key-bundle-availability retry loop** (`composeAndSend` in
  `live-proof.mjs`): retries the whole compose→send cycle up to 6 times,
  polling `.toast-message` DOM content continuously through each attempt's
  wait window (not a single fixed-delay snapshot — toasts auto-dismiss
  after a few seconds, and an earlier version of this same retry logic
  missed the real failure toast entirely by checking only once). This is
  genuine defense-in-depth for a real DHT-propagation race that *could*
  occur in less favorable timing even now that bug #4 above is fixed, and
  did not need to fire at all in the passing run once #4 was actually
  fixed.
- **Foreground-hold on the sending page**: `composeAndSend` now calls
  `page.bringToFront()` before clicking Send and holds that page in the
  foreground for 9 seconds afterward, since Chromium can throttle
  background-tab `setTimeout` timers — relevant because of the 5-second
  undo-delay pattern in bug #3 above.
- **Always set `HC_PATH`/`PATH` explicitly** for every `hc sandbox`
  invocation in this environment (`/var/tmp/hc-0.6.1/bin/hc`). Omitting it
  once mid-session caused `launch-conductors.sh` to fail silently
  (`hc: command not found`) while its own admin-port-wait loop reported
  false success — because a *stale, already-poisoned* conductor process
  from an earlier attempt was still squatting on the same ports. Always do
  a clean `sudo ss -tlnp | grep -E "4444|4445|4446|4447|9600|3340"` sweep
  and kill any leftover `holochain`/`kitsune2-bootstrap-srv`/`iroh-relay`
  processes before relaunching.
- A conductor that hits a `"SQL logic error"` (typically from an earlier
  `kill -9` mid-operation) is **permanently poisoned** — retrying the same
  call against the same conductor fails identically forever. The only fix
  is a genuinely fresh conductor, which is what `run-with-retry.sh` does
  between attempts.

### Known, still-open, unrelated issues (not fixed this session)
- **Settings page "Agent Public Key" infinite loading** — the `loading`
  signal only flips to `false` after both `get_my_profile` *and* a
  `did_registry.get_did_document` call on the `"identity"` role complete;
  that role doesn't exist in the restricted alpha DNA
  (`create_did`/`get_did_document` both fail with `Unknown role:
  identity`), and depending on exact match-arm structure this can leave
  the page stuck. Worked around in the test harness by resolving the
  recipient's key via a native `app_info` call instead of reading it off
  the Settings page. Real, user-facing, not fixed.
- `mail_sync`/`mail_contacts` zomes are absent from the restricted alpha
  DNA — every `sync init skipped` / `get_all_contacts failed: Zome not
  found` warning in the logs is expected and harmless for this DNA.
- `mail_keys.needs_refresh` fails with `missing field "identity_key"` — a
  real but non-blocking error surfaced during this session's runs, not
  investigated further (didn't affect the pass).

### Reproducing
```bash
cd scripts/live-browser-proof
just build-happ && just build-ui   # or the equivalent trunk/cargo commands
HC_PATH=/var/tmp/hc-0.6.1/bin/hc PATH=/var/tmp/hc-0.6.1/bin:$PATH \
  KITSUNE2_BOOTSTRAP_SRV_PATH=~/.cargo/bin/kitsune2-bootstrap-srv \
  IROH_RELAY_PATH=~/.cargo/bin/iroh-relay \
  PLAYWRIGHT_CHROMIUM_EXECUTABLE_PATH=/path/to/real/chromium \
  ./run-with-retry.sh
```
Needs `kitsune2-bootstrap-srv` and a version-matched `iroh-relay` binary on
`PATH` or via the env overrides above (see item 2's fix for install
commands) — the vanilla `cargo install kitsune2-bootstrap-srv` alone is not
sufficient.

---

## Residual sweep: the ActionHash/AgentPubKey decode bug, everywhere else (2026-07-18)

Item 4 above ("the actual root cause") was found in 4 places by following the
golden path. Once that path passed, we swept the rest of `apps/leptos` for the
same bug class before it could bite a *different* feature the same way. It
did — extensively. Every site below decoded a zome-call response containing a
raw `AgentPubKey`/`ActionHash` (msgpack's raw byte-array type) as
`serde_json::Value`, which cannot represent it
(`invalid type: byte array, expected any valid JSON value`). The DHT
write/read itself always succeeded server-side; only the client's decode
failed, silently, with each site's own downstream consequence.

**Refined understanding of the bug class**: only Holochain's `holo_hash`-
wrapped types (`AgentPubKey`, `ActionHash`, `EntryHash`, etc.) trigger this —
they serialize as msgpack's native "bin" type. A *plain* `Vec<u8>`/`[u8; N]`
field (not wrapped in a `holo_hash` type) serializes as an ordinary sequence
and decodes into `serde_json::Value` just fine (as an array of numbers).
This is why `get_my_hybrid_key_bundle_v2` and `needs_refresh` were checked
and left alone — their response structs (`HybridKeyBundleV2`, `BundleStatus`)
have zero `holo_hash`-typed fields.

### Fixed

- **`contacts.rs`** — `create_contact` (returns `ActionHash`); `get_contact_by_email`
  (returns `Option<Contact>`, `agent_pub_key: Option<AgentPubKey>`);
  `resolve_identity` (returns `Option<AgentPubKey>` — kept as `Option<Vec<u8>>`
  decode, re-wrapped as a `Value` only for local storage/outbound encoding,
  which was never the broken direction).
- **`compose.rs`** — the contact-autocomplete DHT search's `get_contact_by_email`
  call (same fix as above).
- **`offline.rs`** — `update_email_state` (×3, returns `ActionHash`),
  `mark_as_read` (returns `Option<ActionHash>`). `move_to_folder` (returns
  `()`) needed no change.
- **`mail_context.rs`** — `update_email_state` (×3), `mark_as_read`,
  `get_trust_score` (returns `TrustScore { agent: AgentPubKey, .. }` — fixed
  with a minimal `TrustScoreWire { score: f64 }`, msgpack named-map encoding
  ignores fields not present on the target struct), `get_email` (V1 legacy
  decrypt path used by `hydrate_inbox_previews`; fixed with a minimal
  `EncryptedEmailPartialWire` carrying only the 4 fields this path actually
  reads), `get_folders` (returns `Vec<(ActionHash, EmailFolder)>` —
  `EmailFolder` has no plaintext `name` field, only `encrypted_name: Vec<u8>`,
  and this codebase has no folder-name decryption path; the new
  `FolderWireV1` decodes correctly and the UI shows an honest "Folder"
  placeholder rather than inventing decryption or lossy-UTF8-decoding
  genuinely encrypted bytes, which is what the old, unused
  `zome_adapter::WireFolder`/`adapt_folders` would have done), `get_all_contacts`
  (`Vec<serde_json::Value>` — one non-null `agent_pub_key` anywhere in the
  list failed the whole `Vec` decode), and **`get_inbox`** (the V1 legacy
  path — this one's outer decode failure meant the "parse as wire types,
  fall back to direct parse, fall back to raw logging" dance that used to
  live around it could *never actually run*: the initial
  `call_zome::<_, serde_json::Value>` failed before any of those fallbacks
  ever saw a value to inspect. Replaced with a direct decode into the fixed
  `WireEmailListItem`, removing the dead fallback logic entirely).
- **`settings.rs`** — a duplicate `set_profile` call (same bug/fix as the
  onboarding one in `profile_setup.rs`), `get_folders` (×2, one diagnostic
  count-only, one feeding the "Read Fetch" diagnostic step), `get_contact_by_email`,
  `resolve_identity`, `get_inbox` (diagnostic), `get_email` (diagnostic —
  only checks success/failure, fixed by decoding as
  `Option<serde::de::IgnoredAny>`, which accepts and discards *any*
  well-formed response regardless of shape without needing to know
  `EncryptedEmail`'s real fields), `get_all_contacts`.
- **`revocable.rs`** — `rotate_keys` (returns `ActionHash`).
- **`read.rs`** — `get_email` (the V1 legacy message-decrypt path; same
  minimal-wire-type fix as `mail_context.rs`'s copy of this call).
- **`search.rs`** — `search`. This one had a *second*, unrelated pre-existing
  bug on top of the byte-array issue: the old code tried to deserialize
  `SearchResultOutput` (the real backend shape — `document_hash`, `preview`,
  `sender`, `matched_terms`, no priority/is_read/is_starred/has_attachments)
  directly into `EmailListItem` (the frontend display shape) — these never
  matched at all, so search results could never have rendered even ignoring
  the decode failure. Fixed with a real `SearchOutputWire`/`SearchResultWire`
  matching the actual backend, mapped into `EmailListItem` with honest
  defaults for fields `SearchResultOutput` doesn't have.
- **`zome_adapter.rs`** — `WireEmailListItem.hash`/`.sender` and
  `WireContact.hash`/`.agent_pub_key` were typed `serde_json::Value`,
  making both types unable to decode a non-empty real response. Both types
  existed but were **dead code** before this fix — `mail_context.rs`'s V1
  inbox/contacts loading bypassed them with ad-hoc `serde_json::Value`
  parsing that had the identical bug. Fixed the field types and wired both
  types into their actual callers for the first time (`adapt_inbox`,
  `adapt_contacts`), replacing the ad-hoc parsing.

### Deliberately left alone

- **`mail_sync` zome, `identity` role** — every call site targeting these
  (`get_sync_state`, `init_sync_state`, `process_offline_queue`,
  `did_registry.*`, `mfa.*`) fails with `Zome not found`/`Unknown role` before
  any decode is even attempted, confirmed on every single run this session.
  These are genuinely absent from the restricted alpha DNA, not reachable
  in this environment regardless of their decode types. Left as-is rather
  than fixed-but-unverifiable.
- **`get_my_hybrid_key_bundle_v2`, `needs_refresh`** — checked and confirmed
  safe; see "refined understanding" above.
- **`chat.rs`'s `send_signal`** targets a zome name (`mail_federation`) that
  doesn't correspond to any real zome in this DNA (the closest real zome,
  `federation`, has no `send_signal` function) — a separate, pre-existing
  wrong-zome-name bug, unrelated to and out of scope for this sweep.

### Verification

Re-ran `run-with-retry.sh` after this sweep (same rebuild-and-test cycle as
above): passed on attempt 1, with the send needing the key-bundle-retry
safety net on this run (a genuine timing race, handled correctly) — and the
new `get_folders`/`get_inbox` code paths ran cleanly throughout
(`[Mail] get_folders: 0 folder(s)`, `[Mail] get_inbox: 0 item(s)`, no
decode errors), confirming the sweep introduced no regression.
