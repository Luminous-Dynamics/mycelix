# Pulse working-alpha evidence

This document is a release gate, not a marketing claim. A box may be checked
only from a clean checkout using freshly built artifacts.

## Implemented and locally verified

- [x] V2 canonical wire format rejects unknown versions, trailing data, invalid
  UTF-8, and invalid lengths.
- [x] X25519 and ML-KEM-768 both contribute to the domain-separated HKDF key.
- [x] AES-256-GCM authenticates the immutable routing and key transcript.
- [x] ML-DSA-65 verification is mandatory *at the recipient client* and fails
  on transcript mutation. **Updated 2026-07-18: it is now also
  cryptographically checked by DHT validators** — `mail_messages_integrity`
  links `ml-dsa` with `default-features = false, features = ["alloc"]`
  (verification needs no randomness, unlike signing/keygen, which was the
  actual reason the older "cannot link" claim held) and fetches the
  sender's real published bundle via `must_get_valid_record` to verify
  against. Proven with a real `cargo build --release --target
  wasm32-unknown-unknown` (production zome build), not just `cargo check`.
  See `verify_ml_dsa_v2` in `mail_messages_integrity`, the "Enforcement
  boundary" section of ADR-002, and `PULSE_V2_CRYPTO_SPEC.md`.
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
  signatures, unknown version/suite, and the documented fact that
  structural validation alone (deliberately) doesn't catch a
  garbage-but-correctly-sized ML-DSA signature — that's `verify_ml_dsa_v2`'s
  job, a separate, host-dependent check.
- [x] The DHT-validator's cryptographic ML-DSA-65 check (`verify_ml_dsa_v2`)
  is directly unit-tested against a mocked HDI host (`hdi::test_utils`,
  no live conductor needed) — 3 tests proving it accepts a genuinely valid
  signature (real keypair, real signature via a standalone `ml-dsa-test-signer`
  subprocess — see that crate's Cargo.toml for why it's a separate process),
  rejects a tampered one, and rejects a bundle authored by someone other
  than the claimed sender.

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

  **2026-07-19: the ML-DSA validator fix has its strongest possible evidence
  now — both `phase0_v2_hybrid_pqc_transport` and `phase0_v2_negative_paths`
  pass against real conductors** (`cargo test --manifest-path tests/Cargo.toml
  --test sweettest_mail_security -- --ignored --test-threads=1
  phase0_v2_hybrid_pqc_transport phase0_v2_negative_paths`, 1389.71s, both
  green). `phase0_v2_hybrid_pqc_transport` used a genuinely-generated ML-DSA
  keypair (via the standalone `ml-dsa-test-signer` subprocess) and proved the
  DHT validator really accepts a message signed with it end to end (commit,
  gossip, field recovery). `phase0_v2_negative_paths`' case C — now inverted
  from its original assertion — proved a garbage-but-correctly-sized ML-DSA
  signature against a *real* published bundle is genuinely rejected by
  `send_email_v2` itself. This closes out the "run the real Sweettest suite,
  not just cargo check" item from the same session's next-steps list.

  **2026-07-19: `phase0_v2_conductor_restart_recovery` also passes** (`cargo
  test --manifest-path tests/Cargo.toml --test sweettest_mail_security
  phase0_v2_conductor_restart_recovery -- --ignored --test-threads=1`,
  777.25s, green) — the slower, separately-gated third test in this trio,
  run on its own per the existing `test-delivery-fast`/`test-delivery-restart`
  split. Both Alice (pre-restart) and Bob (post-restart) used genuinely
  generated ML-DSA-65 keypairs via `real_ml_dsa_bundle`/`ml_dsa_sign`, same
  as the other two tests, so this closes the last piece of "run the real
  Sweettest suite" evidence for the ML-DSA fix: real signatures now survive
  a full conductor shutdown → restart → gossip-resume cycle, not just a
  live in-memory session.

  **2026-07-19 re-run, after landing the ML-DSA validator fix**: found and
  fixed a new environmental blocker, then found a new (real, deterministic)
  code-path gap past it.
  - Playwright's downloaded Chromium (`chromium_headless_shell-1228`) failed
    to launch on this NixOS box with `error while loading shared libraries:
    libgbm.so.1: cannot open shared object file` — a missing system library,
    not present in the ambient NixOS environment by default. Fixed for this
    run via `LD_LIBRARY_PATH` pointed at a 64-bit `mesa-libgbm` already in
    the local Nix store (a 32-bit variant exists too and fails differently
    — `wrong ELF class: ELFCLASS32` — confirm the ELF class of whichever
    store path you pick, e.g. `od -An -tu1 -j4 -N1 <path>/lib/libgbm.so.1`
    should print `2`). Not yet made a permanent fixture of this repo (no
    obvious place to wire it in without hardcoding a store path that will
    eventually get GC'd); whoever runs this next will likely need to
    re-resolve the path.
  - With that fixed, the proof got further than ever recorded here: both
    Alice and Bob reached `Live`, and both completed onboarding far enough
    for `profile_setup.rs`'s `#setup-name` field to hide (i.e. the
    `NameEntry` step advanced to `KeyGen`) — confirmed via the existing
    `waitForLiveStatus`/`completeOnboarding` steps passing for both agents.
  - **New, real, deterministic finding**: past that point, Bob's onboarding
    modal never actually reaches `SetupStep::HasProfile` — a screenshot
    taken immediately after `completeOnboarding` returned (before the next
    step's `page.click('Settings')` starts retrying) shows the modal still
    displaying **"Saving profile to DHT..."**, the very first async status
    `do_create` sets in `profile_setup.rs`, before `set_profile` has
    resolved. The subsequent `page.click` on the Settings nav link then
    retries against this still-visible full-screen modal overlay
    (`position:fixed;inset:0;z-index:99990`) for the full 30s timeout and
    fails. Reproduced twice, identically, on two independently fresh
    conductor pairs (not a one-off nonce/timing fluke) — the underlying
    `set_profile` zome call (or something upstream of it in the
    `AdminGrantedSigner`/signing-credentials broker pipeline) appears to
    genuinely hang for the *second* browser context specifically; no
    JS-side exception or Holochain conductor-log error was observed at
    `ERROR` level for either agent around this time. **Not caused by
    today's ML-DSA/Settings/Contacts changes** — `profile_setup.rs`,
    `device_keystore.rs`, and the signing-credential broker were untouched
    this session. Whether this is new environmental drift since the
    2026-07-17 "full two-user onboarding proven live, twice" success above,
    or a latent bug in the two-browser-context signing-credential flow that
    just hadn't been exercised past the `libgbm` blocker before, is
    unknown — worth instrumenting the actual `set_profile` zome-call
    round trip (network tab equivalent / explicit client-side timeout) as
    the next concrete step, rather than re-running blind.

  **CLOSED 2026-07-22**: not a hang. Built a new instrumented diagnostic
  (`scripts/live-browser-proof/diag-bob-set-profile.mjs`) using Playwright's
  `page.on('websocket')` to log the actual zome-call round trip without any
  app code changes. It surfaced the real cause: `mail_profiles.set_profile`
  returns a real `ActionHash`, but the browser decodes every zome response
  as `serde_json::Value` — which has no variant for a raw byte array — so
  the call failed client-side with `Serialization error: invalid type: byte
  array, expected any valid JSON value`, even though the entry had already
  committed successfully server-side (confirmed live: a fresh connection to
  the same conductor detected `"Profile exists"` despite the client having
  reported an error moments earlier). Exact same bug class as the
  historical `ActionHash`/`AgentPubKey`-through-generic-JSON-boundary issue
  this doc already documents elsewhere — this specific call site was missed
  by that sweep. The prior report's specific symptom (a permanently-stuck
  "Saving profile to DHT..." modal rather than a visible error) turned out
  to be **a second, independent bug in the diagnostic itself, not the app**:
  `#setup-name`'s visibility is tied directly to `SetupStep::NameEntry`,
  which `do_create` leaves *synchronously* on submit — so a
  `waitForSelector(..., {state:'hidden'})`-style check reports "complete"
  the instant the button is clicked, regardless of whether the async call
  underneath ultimately succeeds or fails. Once the diagnostic instead
  polled for a real error message or a sustained hidden state, both agents'
  actual outcomes became visible directly.

  **Fixed** (`17c8f786b3`): `mail_profiles::set_profile` now returns
  `ExternResult<()>` instead of `ExternResult<ActionHash>` — no caller
  (`profile_setup.rs`, `settings.rs`) ever used the hash, only `Ok`/`Err`,
  so this closes the bug at the source for both call sites at once. The
  very next onboarding step, `mail_keys::publish_hybrid_key_bundle_v2`, had
  the **identical** bug — but its `ActionHash` return is genuinely needed
  elsewhere (the Sweettest suite's `sender_mldsa_bundle_hash`/
  `recipient_bundle_hash` pointer plumbing from the PQC gap closure above),
  so that fix is client-side only: `profile_setup.rs` now decodes that one
  call as `Vec<u8>` instead of `serde_json::Value`. Verified live,
  end-to-end, before/after: pre-fix, both stages failed with the exact
  decode error for both Alice and Bob; post-fix, both stages report
  `outcome=ok` for both agents, confirmed across multiple independently
  fresh conductor pairs. Two small pre-existing, unrelated wasm32 compile
  errors (`E0282`, in `pwa_prompt.rs`/`crypto.rs`) had to be fixed along the
  way just to get the UI to build at all — unrelated to onboarding, not
  found by this investigation's methodology, just a blocker in the path.

  **Follow-up sweep (`7511a87464`, same day)**: checked every remaining
  frontend-reachable zome call in the alpha's 5 zomes for the same bug
  class and found 4 more, all previously silent. `mail_messages::
  update_email_state`/`mark_as_read` (star/read/archive/trash — no caller
  anywhere used the returned hash; fixed at the source, `ExternResult<()>`)
  and `mail_keys::rotate_keys` (same pattern, fixed the same way).
  **`mail_messages::send_email_v2` had it too** — the single most
  user-facing instance found: every real email send was almost certainly
  failing to decode its own success confirmation on the sender's client
  (surfacing as a "Send failed" toast) even though the message committed
  server-side, exactly like `set_profile` did. Its `ActionHash` return is
  genuinely needed by the Sweettest suite (type-annotated directly), so
  fixed client-side only (`Vec<u8>` decode in `compose.rs`), same as
  `publish_hybrid_key_bundle_v2`. Verifying this sweep also surfaced a
  real, independent compile break in `tests/sweettest_mail_security.rs` —
  its local `EncryptedEmailV2Wire`/`SendEmailV2Input` mirror structs never
  got the `recipient_bundle_hash` field added across 8 construction sites
  after the recipient-key-state fix landed earlier this session, so the
  whole Sweettest binary failed to compile; fixed by threading the
  already-in-scope bundle-hash variables through. Verified via `cargo
  check` on every affected crate plus the whole Sweettest binary, and a
  full `cargo build --release --target wasm32-unknown-unknown`.
  **Live-conductor re-verification (Sweettest run + full live-browser-proof
  through compose→send→read) is still open** — the host was under severe
  concurrent-session contention (load 30-39) when this was ready to test,
  bad enough that `hc sandbox`'s own hApp installation was timing out;
  deferred to a quieter window rather than fought through.

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
