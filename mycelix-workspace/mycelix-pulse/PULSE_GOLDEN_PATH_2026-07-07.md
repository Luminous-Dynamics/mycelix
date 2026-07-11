# Pulse Golden Path: make one PQC message provably deliver, end to end

*Authored 2026-07-07 from a 3-probe deep-dive. Goal: Alice → Bob, one
authenticated hybrid-PQC message, delivered and **proven by a test that runs in
CI** — the reference for what "done" means, and the first real home for the
`mycelix-crypto` hybrid-PQC library.*

## The reframe: delivery is far closer than the docs imply

The "never proven delivery / signing impossible / auth hang" framing is **stale**.
Verified 2026-07-07:

- **Delivery is real and built** — `send_email` creates an `EncryptedEmail` entry + an
  `AgentToInbox` link based at the *recipient's* key; Bob pulls via `get_inbox`
  (`get_links` on his key) with a best-effort real-time signal. Inbox-forgery
  prevention (`validate_inbox_link`, `messages/integrity:794-858`) is genuine.
- **The two delivery tests are NOT code-blocked** — `phase0_two_conductor_harness_smoke`
  and `phase0_alice_sends_bob_receives` (`tests/sweettest_mail_security.rs:1864,2154`)
  run **in-process** (`SweetConductorBatch::from_standard_config_rendezvous(2)`), no
  external conductor. Phase 0.8 already made the signed content deterministic.
- **The auth hang is one small shared-client bug** (see Phase 2).

## Two separable tracks, one proof

"Delivery works" (native sweettest, in-process, placeholder keys) and "the payload is
real hybrid-PQC" are **independent**. Green delivery first (cheap, proves the mechanism +
the claim), then make the payload PQC (the library payoff), then the live browser app.

---

## Phase 1 — Green the delivery proof in CI  *(fastest; highest proof-per-effort)*

Turns "never proven" into "proven on every push." No browser, no PQC needed.

**PROGRESS 2026-07-08 — chain 2/3 done and verified; final gate is build-queue-blocked:**
1. ✅ **getrandom-0.2 prereq fixed + VERIFIED** — commit added `getrandom custom` to
   `mail-bridge/coordinator` (zkp-core→rand→getrandom 0.2 source). The full pulse zome
   wasm build now **Finishes clean** (`cargo build --release --target wasm32-unknown-unknown`
   from `holochain/`, 6m09s, all 24 zome wasms produced).
2. ✅ **Fresh DNA packs** at `holochain/dna/mycelix_mail.dna` (the path the test loads).
   TWO turnkey workarounds discovered:
   - The pulse **`nix develop` shell is unusable** — it builds `hc` from source and fails
     on `mold: undefined symbol: __rust_probestack` (system mold vs the CLI build). **Bypass:
     use the prebuilt `hc` binary directly:**
     `/nix/store/g2iq49a41l7x9hma8fiwjaqc9754x2kg-hc-0.6.0/bin/hc dna pack dna/ -o dna/mycelix_mail.dna`.
   - The zome build used the session `CARGO_TARGET_DIR`, but `dna.yaml` references
     `../target/...`; **symlink** `holochain/target` → the CARGO_TARGET_DIR so the wasms resolve.
3. ⏸ **Delivery sweettest — blocked by the cross-session cargo-gate** (2 build slots, 14
   sessions): `cargo test -p mycelix-pulse-sweettests --test sweettest_mail_security
   phase0_alice_sends_bob_receives -- --ignored --test-threads=1` (set `LIBCLANG_PATH` to a
   nix-store `libclang.so` dir; the nix shell can't provide it). The sweettest crate never
   got a build slot (killed while queued). **Run this in a low-session-count window** — the
   compile is the full Holochain 0.6 + sweettest stack; everything it needs upstream is now
   ready. This is the only remaining step to a green delivery proof.

**⚠ PREREQUISITE found 2026-07-07 (blocks the wasm zome build): pulse's `holochain/`
workspace is missing the getrandom-0.2 `custom` force that every sibling cluster has.**
A zome pulls `rand 0.8` → getrandom **0.2** (via `ppv-lite86`); the existing
`holochain/.cargo/config.toml` only sets the backend cfg for getrandom **0.3**
(`__getrandom_v03_custom`), so getrandom 0.2 fails: *"the wasm targets are not supported
by default, enable the js feature."* Fix per `mycelix-commons/Cargo.toml:164-165`:
add to `holochain/Cargo.toml` `[workspace.dependencies]`
`getrandom = { version = "0.2", features = ["custom"] }` + `getrandom_03 = { package =
"getrandom", version = "0.3" }`, and have one built crate reference it (commons routes it
via `mutualaid-common/Cargo.toml:14` `getrandom_03 = { workspace = true }` — replicate that
in a pulse shared crate so feature-unification applies `custom` to the transitive 0.2).
Use `custom`, NOT `js` (workspace CLAUDE.md: js pulls wasm-bindgen, incompatible with the
HC WASM runtime). **Verify with the wasm zome build below — needs a quiet machine.**

1. **Rebuild the stale DNA.** On-disk `holochain/dna/mycelix_mail_dna.dna` is dated
   2026-04-07 — *predates* the Phase 0.3/0.8 validation+timestamp logic (integrity
   2026-05-25, coordinator 2026-06-16), so it proves nothing. Under `nix develop`:
   `cargo build --release --target wasm32-unknown-unknown` then `hc dna pack holochain/dna/`.
   **Fix the name mismatch**: the test helper `mail_dna_path()` expects `mycelix_mail.dna`
   (not `..._dna.dna`) — repack to that name or set `MAIL_DNA_PATH`.
2. **Un-ignore + stabilize** `phase0_two_conductor_harness_smoke` (harness) first, then
   `phase0_alice_sends_bob_receives` (the real proof: byte-exact subject + µs-timestamp
   round-trip). They carry flakiness scars (BadNonce retry, 180s polling) — stabilizing
   under CI load is the real work, not writing.
3. **CI job that runs them.** Add a `checks.sweettest-delivery` nix derivation mirroring
   the existing `checks.gateway-smoke` (`flake.nix:279`, `tests/pulse-gateway-smoke.nix`),
   packing the DNA fresh, running `cargo test -p mycelix-pulse-sweettests --test
   sweettest_mail_security -- --ignored --test-threads=1 phase0_...`. Wire into `ci-pass`.
   (The sweettest crate is a separate package outside `holochain/`, so `--lib` CI never
   compiles it today.)

**Effort ~1-2 days** (mostly DNA rebuild + flakiness). **Proves:** "A 2-agent in-process
conductor test shows Alice's signed email reaches Bob's inbox intact, run in CI."

## Phase 2 — Fix the live web app  *(highest cross-app leverage: one fix helps every app)*

1. **The auth-hang root cause** (`crates/mycelix-leptos-client/src/browser.rs:509` + the
   `mycelix-workspace/crates/` mirror): the client `send_request("authenticate", …)` and
   awaits a response, but Holochain 0.6 `authenticate` is **fire-and-forget — the conductor
   never replies**, so the oneshot dies at the 30s timeout → "mock mode." Praxis never hit
   it because it passes no token and skips the branch; **Pulse is the first app to send a
   real token, so the first to hit it.** Fix: send the `authenticate` frame directly
   (no id-correlated pending request, no `.await`), then let the next `app_info` request
   implicitly confirm auth. *This fixes live mode for every provider-based app at once.*
2. **Server-side send signing.** Browsers can't sign with the lair agent key. `send_email`
   currently stores `input.signature` verbatim; instead compute `email_signing_content`
   in-coordinator and `sign_raw(my_agent, content)` — reusing the exact pattern already
   used for read/delivery receipts (`messages/coordinator:666,749`). Browser then sends
   envelope fields only.

**Effort ~1 day.**

## Phase 3 — Wire hybrid PQC into the payload  *(the library payoff; gate on crypto audit)*

Opaque-blob envelope — **zero `EncryptedEmail` schema change**: `encrypted_subject`/`body`
carry `field_crypto` `mxc1:`+bs58 bytes; `ephemeral_pubkey` stays a 32-byte X25519 value;
the on-chain Ed25519 envelope signature stays (so `messages/integrity:543-552` still
verifies). Steps:

- **A. Lib prep (safe, in `mycelix-crypto`).** (1) ✅ **DONE** (commit `3d6d8576c6`) —
  `to_bytes`/`from_bytes` secret-key persistence on `HybridKemKeyPair` (x25519 secret +
  ML-KEM seed) and `HybridSigner` (ed25519 secret + ML-DSA seed), 2 round-trip tests.
  (2) **getrandom gating — MOVED to Phase 3-B.** Attempted here but reverted: gating the
  `custom` dep behind `wasm` is correct, but it breaks the standalone hybrid-rc wasm32
  build (nothing to supply a browser backend), and — key finding — the browser build needs
  getrandom **0.2 `js` FEATURE + 0.4 `wasm_js` FEATURE** (ml-kem/ml-dsa pull getrandom
  **0.4**, not 0.3), *not just a `--cfg getrandom_backend` rustflag*. Only the consumer
  (Pulse) can supply those features, so the gating is verifiable only alongside Pulse's
  deps — do it in 3-B, not standalone.
- **B. Pulse getrandom backend (the critical build blocker).** Pulse leptos is a detached
  workspace with `getrandom 0.2 features=["js"]` and **no `.cargo/config.toml`**. ml-kem/
  ml-dsa pull **getrandom 0.3**, which won't compile for wasm without a backend. Add
  `apps/leptos/.cargo/config.toml` → `rustflags=['--cfg','getrandom_backend="wasm_js"']`
  and a `getrandom_03 = { package="getrandom", version="0.3", features=["wasm_js"] }` dep.
  (Trunk honors `.cargo/config.toml` rustflags.)
- **C. PreKeyBundle PQC slots** (`keys/integrity:13-28`): add `#[serde(default)]`
  `ml_kem_ek` (1184), `ml_dsa_vk` (1952), `ed25519_vk` (32) with fire-only-when-non-empty
  length checks (create + update arms). Publish in `profile_setup.rs:181`, fetch in
  `compose.rs:326`. Legacy bundles (empty) → fall back to classical path (gradual rollout).
- **D. Client swap** (new `apps/leptos/src/hybrid.rs` + `compose.rs:341`, `read.rs:137`):
  seal via `field_crypto::encrypt_field`; on read, `is_encrypted_field` → `decrypt_field`
  (which **verifies the hybrid signature** — closing the current "read.rs never verifies"
  gap), else legacy AES fallback.
- **E. Optional:** advertise hybrid suite labels in `messages/integrity:494` (on-chain can
  still only verify the Ed25519 half — HDK has no ML-DSA host fn; that's the Holochain
  contribution noted in the PQC roadmap).

**Effort ~3-4 days.** **Do not land before the `mycelix-crypto` crypto audit clears** (all
hybrid modules are marked EXPERIMENTAL).

## Phase 4 — Send-side spam gate  *(before opening delivery to real users)*

✅ **Step 1 DONE** (commit `6fdde6b2e3`) — `send_email` now enforces the sender trust it
already fetched: rejects explicitly-negative trust, allows unknown (0.3). Compiles wasm32;
matches the ignored `test_send_email_blocked_for_negative_trust`. ⏳ Step 2 (per-agent
volume cap via the unused `rate_limiter` crate) + integrity-level enforcement remain.

Send-side abuse resistance was entirely inert: the fetched trust score was discarded
(`messages/coordinator:248-249` `let _trust_score`), `AnchorToPublicEmails` returns `Valid`
unconditionally (`integrity:774`), and a complete 468-LOC `rate_limiter` crate has zero
consumers. Cheapest real gate (one hook at `coordinator:248`):
1. **Enforce the already-fetched trust** — `return Err` below a floor (there's already an
   ignored test: `test_send_email_blocked_for_negative_trust:1633`). ~3 lines.
2. **Per-agent volume cap** — depend on `mycelix_rate_limiter`, call
   `RateLimiter::for_email_send().enforce(&my_agent, Some(trust))?`. Source-chain-backed
   (correct WASM pattern), but *cooperative* (not Sybil-proof) — fine for a closed group.

**Effort ~half day.**

---

## Explicitly deferred (not part of the golden path)

- **External SMTP / Gmail interop** (Phase 5B: Hetzner MX, DKIM signing, port-25, real
  `holochain_client` swap for `StubZomeBridge`). A funded multi-week effort; internal
  mycelix↔mycelix mail needs none of it.
- **Epoch ratchet / forward secrecy** (readiness-plan Phase 3) — separate multi-week effort.
- **On-chain ML-DSA verification** — blocked on an HDK host function (propose upstream).
- **Sybil-proof rate limiting** — needs the cap in integrity validation, not coordinator.

## Acceptance criteria for "Pulse genuinely delivers a PQC message, proven in CI"

1. `phase0_two_conductor_harness_smoke` + `phase0_alice_sends_bob_receives` un-ignored and
   green, run in-process, byte-exact round-trip asserted.
2. A CI job (nix check) packs the DNA fresh and runs them on every push, in `ci-pass`.
3. Payload is hybrid-PQC (`field_crypto`) with the sender signature verified on read.
4. Send gated by an enforced trust check + per-agent rate cap.

**Honest claim after Phases 1-4:** "A CI test proves an authenticated, hybrid-PQC email is
delivered Alice→Bob in-process; delivery is trust- and rate-gated." *Not yet:* multi-node
network delivery, Sybil-proof limiting, on-chain PQC-signature verification, or external
email interop.

## Recommended sequence

**1 → 2 → 4 → 3.** Phase 1 proves the claim cheapest; Phase 2 unblocks the live app for
every cluster; Phase 4 is a half-day safety gate; Phase 3 (the PQC payoff) is last because
it is the largest and is **gated on the crypto audit**. Phase 3-A (lib prep) is the one
piece that is safe to start immediately — it's additive, in the collision-free
`mycelix-crypto` crate, and unblocks the rest of Phase 3.
