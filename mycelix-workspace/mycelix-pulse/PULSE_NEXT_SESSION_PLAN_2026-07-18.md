# Pulse — Next Session Plan

**Authored:** 2026-07-18, immediately after the live-browser proof session that
finally closed out `PULSE_READINESS_PLAN.md`'s "killer gap" — see
`docs/ALPHA_EVIDENCE.md` for the full evidence trail. This doc picks up from
there. Read `docs/ALPHA_EVIDENCE.md` first; it has the ground truth for what
"live-verified" means in this codebase and why several past claims here need
re-checking against it.

**Status update, same day, later session:** items 1, 2, 3, and two of item 4's
three follow-ups are now closed — see the per-item notes below and commits
`68d5409bc6` (items 1+2, plus the `needs_refresh` and `chat.rs` pieces of item
4) and `aca807309a`/`bf716b8fbe`/`0676287a0f` (item 3, the PQC gap). Only
item 4's folder-name-decryption follow-up, and item 5 (Phase 5B, external
blocker), remain open. See "Suggested next session" at the bottom for what's
actually left.

**What changed since `PULSE_READINESS_PLAN.md` (2026-04-18)**: that doc's
opening claim — "Nobody has verified that Alice sending a message results in
Bob receiving it" — is no longer true. `scripts/live-browser-proof/run-with-retry.sh`
now passes end-to-end: real onboarding, a real signed `send_email_v2` call,
real P2P gossip delivery over a real (if locally-hosted) iroh/QUIC transport,
and a real decrypted + ML-DSA-verified inbox read, asserted out of the
rendered DOM in an actual browser. That was blocked for most of one session by
five stacked bugs (P2P relay protocol mismatch, a Leptos-context panic, and —
the actual root cause — an `ActionHash`/`AgentPubKey` response type decoded as
`serde_json::Value`, which can't represent msgpack's raw byte-array type). A
follow-up sweep found and fixed ~15 more instances of that last bug class
across the rest of the app.

---

## 0. Ground truth check before touching anything

The alpha DNA (`holochain/dna-alpha/dna.yaml`) is **deliberately scoped** —
its own `alpha_scope` property says so: `"profile+hybrid-keys+encrypted-messages"`.
It bundles exactly 5 zomes: `mail_profiles`, `mail_trust`, `mail_keys`,
`mail_messages`, `mail_capabilities`. Every other zome referenced by the
frontend — `mail_contacts`, `mail_sync`, `mail_search`, `mail_federation`,
`mail-bridge`, `scheduler`, `audit`, `backup` — is **not installed** and every
call to it fails with `Zome not found`, confirmed live on every single test
run this session. This is not a bug to fix; it's the alpha's actual scope.

**The consequence that IS worth fixing**: the frontend doesn't know this.
Contacts, Search, offline sync, and several Settings diagnostics render as if
those features exist, then fail silently or with a raw zome error in the
console — a user gets a broken feature with no explanation, not a "not in
this build yet" message. See item 2 below.

## 1. The still-open Settings page bug — ✅ CLOSED (`68d5409bc6`)

~~`ProfileSetup`'s `Effect` gates the onboarding-complete transition on *two*
zome calls...~~ **Investigation found the actual bug wasn't where this note
guessed**: `profile_setup.rs` never gated onboarding on `did_registry` at all.
The real bug was in `pages/settings.rs`'s `ProfileSection`/`SecuritySection`,
which awaited an always-failing `identity`-role call (that role isn't bundled
in the alpha) *before* revealing the already-loaded profile — adding latency
and, for `SecuritySection`, gating an entire MFA UI on a call that can never
succeed. Fixed: `ProfileSection` now shows the profile as soon as its own
call resolves and skips the identity-dependent call entirely (see item 2);
`SecuritySection`'s whole MFA card is now gated behind `identity_role_available()`
(see below) instead of always attempting the doomed call.

## 2. Make the alpha's scope honest in the UI — ✅ CLOSED (`68d5409bc6`)

Added `apps/leptos/src/alpha_scope.rs` — a static mirror of `dna-alpha/dna.yaml`'s
real 5-zome/single-role scope (`zome_available()`, `identity_role_available()`,
plus a shared `scope_notice()` view helper). Wired into:
- Settings → Identity section: the DSID field now shows "Not available in
  this alpha build" instead of a misleading "(not created yet)".
- Settings → Security section: the entire Assurance-Level/MFA card is
  replaced with an honest notice instead of a permanently-empty, non-functional UI.
- Contacts → "Discover on Network" card: replaced with an honest notice in
  live (non-mock) mode, since it depends on both the identity role and
  `mail_contacts`, neither of which exist in this build.

Not done (correctly, on investigation): a matching notice for Search — its
zome-backed BM25 path already degrades gracefully to real client-side
semantic search over the loaded inbox when the zome is unavailable, so
nothing was actually broken there, just a "BM25" badge that's a little
optimistic about which path served results. Left as a very-low-priority
polish item, not a functional gap.

## 3. The PQC verification gap — ✅ CLOSED (`aca807309a`, `bf716b8fbe`, `0676287a0f`)

The disclosed trust boundary — "HDI/WASM zomes cannot currently link the
RustCrypto ML-DSA crate, so DHT validators only check signature length, not
cryptographic validity" — turned out to be stale. `ml-dsa`'s *default*
feature set (signing/keygen) needs `getrandom` 0.4's browser-only backend;
**verification needs no randomness at all**, and links cleanly with
`default-features = false, features = ["alloc"]`. Proven with a real
`cargo build --release --target wasm32-unknown-unknown` (the earlier
"cannot link" belief had only ever been checked with `cargo check`, which
skips the link step).

`mail_messages_integrity` now has a real `verify_ml_dsa_v2`: it fetches the
sender's published `HybridKeyBundleV2` via `must_get_valid_record` (using a
new `sender_mldsa_bundle_hash` pointer field, deterministic and
validator-safe), independently re-derives that bundle's content hash and
real Holochain authorship before trusting it (so a forged/wrong pointer just
breaks the lookup, never bypasses verification), checks its lifecycle state
is `Active`, and finally verifies the ML-DSA-65 signature against its real
public key. `phase0_v2_negative_paths`' case C is inverted (a garbage
signature is now rejected by `send_email_v2` itself, not accepted); 3 new
native mocked-HDI-host unit tests prove accept/tamper-reject/wrong-author-reject.

Two real complications surfaced and were resolved along the way, both worth
knowing about if you touch this area again:
- The Sweettest harness's own `holochain`/`iroh` dependency tree pulls in
  pre-release RustCrypto crates that conflict with `ml-dsa`'s stable
  `crypto-common` requirement — a *different*, harness-specific blocker from
  the zome-linking one above. Real ML-DSA signatures for
  `phase0_v2_hybrid_pqc_transport`/`phase0_v2_conductor_restart_recovery`'s
  "happy path" sends are generated by a new standalone
  `tests/ml-dsa-test-signer/` subprocess helper instead.
- Depending on the full `keys_integrity` zome crate directly from
  `mail_messages_integrity` (to reuse `HybridKeyBundleV2`) caused
  duplicate-`#[no_mangle]`-symbol WASM link errors — two zome crates'
  macro-generated `validate`/`entry_defs` exports collide when linked into
  one cdylib. Fixed by splitting a new `keys_types` crate (plain data, no
  `#[hdk_extern]` surface) out of `keys_integrity`.

✅ **CLOSED 2026-07-19**: the DHT validator previously checked only the
*sender's* key-bundle state, not the *recipient's* — a message to a
recipient whose key had since been revoked/expired was still accepted onto
the DHT. Closed by mirroring the sender-side fix: `EncryptedEmailV2` gained
a `recipient_bundle_hash: ActionHash` lookup pointer (same contract as
`sender_mldsa_bundle_hash` — not itself trusted, independently re-verified
via authorship + content-hash match), and a new
`verify_recipient_key_state` in `mail_messages_integrity` fetches it and
rejects the message if the recipient's bundle isn't `Active`. The
common fetch-and-verify logic was factored into a shared
`fetch_and_verify_bundle` helper used by both checks. Plumbed through
`resolve_hybrid_send_context_v2` (keys coordinator), `send_email_v2`'s
input (messages coordinator), and the compose/inbox wire types
(`apps/leptos/src/pages/compose.rs`, `mail_context.rs`). Proven via 3 new
native mocked-HDI-host unit tests (accept-Active / reject-Revoked /
reject-wrong-author — same evidence tier as the sender-side fix) plus a
real `cargo build --release --target wasm32-unknown-unknown` for the whole
`holochain/` workspace. Not yet re-run through the live-conductor
Sweettest trio (would need `tests/sweettest_mail_security.rs`'s mirror
structs updated the same way — a reasonable next step, not done this
round given the Sweettest suite's ~35-minute total runtime).

## 4. Residual-sweep follow-ups

From `docs/ALPHA_EVIDENCE.md`'s "residual sweep" section:
- ✅ **CLOSED (`68d5409bc6`)** — `mail_keys.needs_refresh` failed with
  `missing field "identity_key"`. Root cause: it read the legacy V1
  `PreKeyBundle` shape via an untagged link query that also picked up the
  real V2 `HybridKeyBundleV2` bundles this app actually publishes — trying
  to deserialize a V2 entry as V1 unconditionally failed. Rewritten to read
  the real V2 bundle's `state`/`expires_at` directly.
- ✅ **CLOSED (`68d5409bc6`)** — `chat.rs`'s `send_signal` targeted a
  `mail_federation` zome+function that doesn't exist anywhere in this
  codebase (not even the closest real zome, `federation`, which has no
  `send_signal` extern). Removed rather than "fixed" to a real target, since
  there was nothing valid to rename it to; the Chat page already runs
  entirely on mock data regardless.
- ✅ **DECIDED 2026-07-19: folder creation stays out of scope for this
  alpha.** Folder names are stored encrypted (`EmailFolder.encrypted_name:
  Vec<u8>`) but there is no client-side folder-name decryption path.
  Re-confirmed this round: the coordinator zome's `create_folder` extern
  genuinely exists (`holochain/zomes/messages/coordinator/src/lib.rs`), but
  `init()` is a deliberate no-op ("folder provisioning... [is] outside the
  restricted alpha artifact") and **no frontend code calls `create_folder`
  anywhere** — so no folder, encrypted or otherwise, can exist in this alpha
  through the actual shipped product surface. Since nothing in the UI
  references folders today, there is no dangling/broken feature to fix (unlike
  Contacts/Identity/Security in item 2, which had live UI pointing at
  unavailable backends) — this is a clean, inert gap, not a bug. Consistent
  with the alpha's own declared scope (`dna-alpha/dna.yaml`'s
  `alpha_scope: "profile+hybrid-keys+encrypted-messages"` — folders aren't in
  it) and with Phase 5B's descope reasoning below: no engineering now:
  **if folder creation becomes a real feature ask later**, the device-local
  symmetric-key pattern already established in `device_keystore.rs`
  (`pulseStoreDeviceSecret`/`pulseLoadDeviceSecret`, non-exportable
  per-device AES-GCM wrapping key in IndexedDB) is the natural,
  already-disclosed trust model to extend to a folder-name key — reuse it,
  don't invent a new one.

## 5. Phase 5B — ✅ DECIDED 2026-07-19: deliberately descoped, not blocked

`PULSE_READINESS_PLAN.md`/`PULSE_GOLDEN_PATH_2026-07-07.md`'s SMTP federation
work (own VPS, own MX/DKIM, port-25, swapping `StubZomeBridge` for a real
`holochain_client`) was previously "blocked" on a funded VPS account and the
1-month customer-age rule before port 25 unblocks.

Raised this session: since the VPS requirement is the actual pain point
(cost, port-25 ticket + wait, weeks of Gmail/Outlook IP-reputation warmup,
ongoing server admin), a serverless alternative was proposed — Cloudflare
Email Routing for inbound + a transactional-email API (Postmark/SES/Resend)
for outbound, eliminating the VPS and port-25 wait entirely. **Explicitly
rejected**: it would still put a third-party vendor in the mail-sending
path, which conflicts with this project's own stated Phase 11 rationale
(`PULSE_READINESS_PLAN.md` §Phase 11 framing note: "using SaaS instead would
teach us how to be SaaS customers, not how to run the substrate"). Since
fully self-hosted is the only option that fits that principle, and its
operational cost isn't worth paying without real demand for external
Gmail/Outlook interop, **the decision is to drop external interop from
scope entirely for now** — Pulse stays Holochain-native-only (already
live-verified, per the evidence above) until real user demand justifies
either the self-hosted VPS path or a fresh look at this tradeoff.

Not an action item for a future session unless that demand materializes —
don't silently re-scope this back in without checking whether it has.

## Suggested next session

**Update, same day:** the "run the real evidence" item below is now mostly
closed. `phase0_v2_hybrid_pqc_transport` and `phase0_v2_negative_paths` both
pass against real conductors (1389.71s, see `docs/ALPHA_EVIDENCE.md`) —
the ML-DSA fix now has this repo's strongest evidence tier, including the
new `ml-dsa-test-signer` subprocess helper working correctly inside the
Sweettest's async runtime. The live-browser-proof half is **not** closed —
see below, it surfaced a new real (if unrelated) gap. What's left:

1. ~~Run the real evidence...~~ **Sweettest half now fully closed.**
   `phase0_v2_conductor_restart_recovery` ran and passed (777.25s, real
   ML-DSA keypairs for both agents — see `ALPHA_EVIDENCE.md`'s 2026-07-19
   entry). All three Sweettests in this trio
   (`phase0_v2_hybrid_pqc_transport`, `phase0_v2_negative_paths`,
   `phase0_v2_conductor_restart_recovery`) now pass against real conductors
   with genuinely generated signatures — this is the strongest evidence
   tier this repo has for the ML-DSA fix. Still open:
   - ~~`scripts/live-browser-proof/run-with-retry.sh`... Bob's onboarding
     modal never leaves "Saving profile to DHT..."~~ **CLOSED 2026-07-22**
     (`17c8f786b3`) — not a hang at all. A new instrumented diagnostic
     (`diag-bob-set-profile.mjs`, WebSocket frame logging via Playwright,
     no app code changes) found the real cause: both `set_profile` and the
     next onboarding step, `publish_hybrid_key_bundle_v2`, return a real
     `ActionHash`, but the browser decodes every response as
     `serde_json::Value` — which can't represent a raw byte array — so
     both calls failed client-side with a decode error even though the
     entry had already committed successfully server-side. The original
     "stuck modal" symptom was a *second*, independent bug in the test
     methodology, not the app: `#setup-name`'s visibility leaves
     `NameEntry` synchronously on submit regardless of the async call's
     eventual outcome, so the original script's completion check was a
     false positive. Fixed `set_profile`'s return type to `()` (its value
     was never used) and `publish_hybrid_key_bundle_v2`'s *client-side*
     decode type to `Vec<u8>` (its `ActionHash` return is genuinely needed
     by the Sweettest suite's pointer plumbing, so the zome signature
     stayed). Verified live before/after on real conductors — see
     `ALPHA_EVIDENCE.md`'s 2026-07-22 entry for full detail.
   - **Follow-up sweep, same day (`7511a87464`)**: checked every remaining
     frontend-reachable zome call for the same bug class — found 4 more
     (`update_email_state`/`mark_as_read`/`rotate_keys` fixed at the
     source; **`send_email_v2` fixed client-side** — this one meant real
     email sends were almost certainly failing to decode their own success
     confirmation). Also fixed a real, independent compile break in
     `tests/sweettest_mail_security.rs` found while verifying the sweep
     (never got `recipient_bundle_hash` added to its own mirror structs).
     **Still open**: live-conductor re-verification (re-run the Sweettest
     trio + a full live-browser-proof compose→send→read) — the host was
     too contended (load 30-39, `hc sandbox` itself timing out) when this
     was ready to test; do this first in the next session on a quieter box.
     **Independent source-review substitute done 2026-07-25** (host still
     contended, load 34, so the live run itself remains blocked): every
     frontend call site for a bare-`ActionHash`/`AgentPubKey`-returning
     extern in the 5 alpha zomes was traced by hand and confirmed either
     already fixed or safe. One additional latent (not live) instance of
     the same bug class found: `contacts.rs`'s `create_contact` call
     (`mail_contacts::create_contact` returns a bare `ActionHash`, decoded
     generically) — confirmed inert today, since `discovery_available()`
     is provably false in live mode while `mail_contacts` stays
     uninstalled (item 0). **Flag for whoever eventually expands the
     alpha's zome scope to include `mail_contacts`**: fix this decode
     site at the same time, or it'll silently fail exactly like the ones
     this sweep closed.
2. ~~The folder-creation product decision~~ **Decided 2026-07-19: stays out
   of scope.** See item 4 above — no code change needed, since nothing in
   the shipped UI references folders at all.
3. ~~The recipient-key-state gap~~ **Closed 2026-07-19** — see item 3
   above. Not yet re-verified through the live-conductor Sweettest trio;
   worth doing if another security pass touches this area.
4. ~~Phase 5B — still blocked...~~ **Decided 2026-07-19: descoped, not
   blocked.** See item 5 above — not an open task unless real demand for
   external Gmail/Outlook interop shows up.
