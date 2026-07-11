# Mycelix Post-Quantum Cryptography Roadmap

*Authored 2026-07-07 from a 3-agent evidence review + a verified wasm feasibility build.*
*Status: plan + Phase-0 KEM primitive LANDED (commit `f6565c578a`,
`mycelix-crypto::hybrid_kem`, feature `hybrid-rc`, 5/5 tests, wasm32 verified).
It is EXPERIMENTAL and off by default — the construction MUST get a crypto
review before it is wired into any cluster (see "Review gate").*

## 1. Scope: three layers, two are ours

"PQC for all of Mycelix" splits into three layers; only two are Mycelix's to change.

| Layer | Owner | Today | PQC-able? |
|---|---|---|---|
| App data (mail bodies, health records, vault contents) | **Mycelix** | X25519 / AES-GCM / XChaCha20 | **Yes** — hybrid ML-KEM envelope, opaque bytes on DHT |
| App signatures (credential proofs, message sigs) | **Mycelix** | Ed25519 | **Yes** — hybrid Ed25519+ML-DSA |
| Substrate (agent keys, DHT action signing, source chain, gossip) | **Holochain** | Ed25519 | **No** — conductor/lair-owned; stays Ed25519 until Holochain ships PQC |

Evidence for the boundary: every zome only *consumes* `agent_info().agent_initial_pubkey`
(a conductor-managed Ed25519 `AgentPubKey`); no Mycelix code generates or PQC-signs it.
The honest posture is **hybrid app-layer PQC + a "PQC-ready" substrate**, with the DID
document carrying a *separate* registered PQC key for app use.

## 2. What already exists (this is consolidation, not greenfield)

- **`mycelix-identity/crates/mycelix-crypto/`** — a crypto-agile crate: `AlgorithmId`
  enum (Ed25519, MlDsa65/87, SlhDsa, MlKem768/1024, HybridEd25519MlDsa65,
  XChaCha20Poly1305 — `src/algorithm.rs:18`), `TaggedPublicKey`/`TaggedSignature`,
  `EncryptedEnvelope { kem_algorithm, encapsulated_key, nonce, ciphertext,
  recipient_key_id }` (`src/envelope.rs:265`), a `HybridSigner` (Ed25519||ML-DSA-65,
  `src/pqc/hybrid.rs`), and a `wasm`(types-only) / `native`(pqcrypto) feature split.
  **Gap:** the `native` path uses `pqcrypto-*` C-FFI which does NOT build for wasm, and
  the `wasm` path is types-only — so there is **no path giving real PQC crypto in a
  browser client**, which is what every client needs.
- **DID document already has PQC key slots** — `did_registry` `DidDocument.verification_method`
  (algorithm-tagged) + `key_agreement`, and `add_key_agreement()` already validates
  ML-KEM-768/1024. Nobody populates them yet.
- **Pulse DHT already accepts PQC** — `messages/integrity` allowlists `kyber1024`/`dilithium3`
  and length-checks their keys/sigs. It just never *verifies* the signature.
- **wasm PQC is proven in-repo** — `xenia/xenia-wire/xenia-viewer-web/src/handshake.rs` is a
  complete, working, browser-wasm hybrid X25519/ML-KEM-768 + Ed25519/ML-DSA-65 handshake using
  pure-Rust `ml-kem = 0.3.0-rc.2` + `ml-dsa = 0.1.1`. **Verified 2026-07-07:** `cargo build
  --target wasm32-unknown-unknown -p xenia-viewer-web` → Finished, 0 errors (ml-kem + ml-dsa
  both compiled for wasm32).

## 3. The real gaps

1. Three duplicate, incompatible crypto stacks: `mycelix-crypto` (best), `mycelix-health/crates/health-crypto`, `mycelix-pulse/.../crypto.rs`; plus ed25519/x25519 scattered across ~20 crates.
2. No client (Leptos/web) imports any PQC crate — every client encrypts classically.
3. No PQC keypairs client-side, and no slot to publish a PQC pubkey (Pulse `PreKeyBundle.identity_key` locked to 32 bytes).
4. PQC signatures are never *verified* — DHT + credential verifiers length-check ML-DSA sigs and skip verification ("off-chain" — but nothing off-chain does it).
5. **`mycelix-personal` vaults store PHI in plaintext at rest** (health-vault `HealthRecord.data` is plaintext JSON) — a live exposure independent of quantum threats.
6. `health-crypto` `derive_symmetric_key` claims HKDF-SHA256 but does plain `SHA256(salt‖secret‖ctx)` — a real bug to fix while consolidating.

## 4. Architecture

1. Promote `mycelix-crypto` to a **workspace-root shared crate** (peer of `mycelix-bridge-common`); every cluster depends on it; delete the duplicates.
2. Add a **RustCrypto (`ml-kem`/`ml-dsa`) backend under a new wasm-capable feature** (Phase 0 spec below). Keep the existing `native`(pqcrypto) and `wasm`(types) paths intact.
3. **Hybrid only, never pure PQC** — X25519+ML-KEM for KEM, Ed25519+ML-DSA for signatures. Never weaker than classical; quantum-resistant if either primitive holds.
4. Add `format_version: u8` + a combined `SealedEnvelope { format_version, EncryptedEnvelope, Option<TaggedSignature> }` for crypto-agility.
5. A **pluggable key-storage trait** — localStorage now, IndexedDB / OS keystore (Tauri StrongBox / Secure Enclave) later. Keys are base64-in-localStorage today.

## 5. Sequenced rollout (by value)

- **Phase 0 — Derisk & unblock.** RustCrypto wasm backend for `mycelix-crypto` (spec §7):
  - ✅ **`hybrid_kem`** (commit `f6565c578a`) — hybrid X25519+ML-KEM-768 seal/open.
  - ✅ **`hybrid_sig`** (commit `e6e26969aa`) — hybrid Ed25519+ML-DSA-65 sign/verify (both halves required).
  - ✅ **`SealedEnvelope`** (commit `e6e26969aa`) — versioned (`format_version`) wrapper over `EncryptedEnvelope` + optional `TaggedSignature`.
  - ✅ **`sealed_box`** (commit `d6d2a84205`) — the one-call client API: `seal_signed` / `open_verified`, composing KEM+sig with encrypt-then-sign + verify-before-decrypt.
  - All behind non-default `hybrid-rc`; **31/31 tests**; whole feature builds for wasm32. **The crypto library is functionally complete — clients call `sealed_box`.**
  - ⏳ **Remaining (deferred — structural/schema changes, do deliberately, NOT during heavy concurrent-session activity):**
    - **Promote `mycelix-crypto` to a root shared crate** (peer of `mycelix-bridge-common`) — a cross-workspace crate move that rewrites path deps in the identity zomes and every future consumer. Contained but collision-prone; the crypto *value* already lands regardless of the crate's location.
    - **Make Pulse's `CryptoSuite` a validated enum** — changes a DHT-serialized wire type on Pulse message entries (schema/back-compat implications); tackle as part of Phase 1 (Pulse) where the entry versioning is already in scope.
    - Add on-chain ML-DSA verify → blocked on an HDK host function (see §1 boundary); until then the ML-DSA half is verified client-side/native only.
- **Phase 1 — Pulse mail (~1wk).** PQC pubkey field in `PreKeyBundle`; client keygen/storage; hybrid encrypt/decrypt; **real Dilithium verify on receive**; flip the cosmetic `send_pqc` toggle to real. Retires the false "PQC email" claim.
- **Phase 2 — Health at-rest (~1wk).** Highest intrinsic value (regulated data + harvest-now-decrypt-later). Wire health-crypto's dead ML-KEM to real client-side encryption via the shared crate; fix the HKDF bug.
- **Phase 3 — Identity credentials (~1wk).** Move hybrid Ed25519+ML-DSA sigs from CLI-only into the VC issue/verify path; populate DID-doc ML-DSA verification methods + ML-KEM key-agreement keys.
- **Phase 4 — Personal vaults (~3-5d).** Zero crypto today — encrypt at rest. *Consider doing this BEFORE the PQC niceties: plaintext PHI outranks classical-vs-PQC.*
  - ✅ **Client-side crypto API ready** (`field_crypto`, commit `087fb9b0a1`) — `encrypt_field`/`decrypt_field` store PQC ciphertext in the existing `HealthRecord.data: String` with **zero zome/schema change** (marked `mxc1:` + bs58; unmarked = legacy plaintext). Bounds-checked codec (no panics on corrupt DHT data). 3 tests + `tests/vault_flow.rs` reference.
  - ⏳ **Remaining = client integration** (no shared-zome edit). Verified 2026-07-07 that both remaining blockers are build/app work, NOT crypto:
    1. **No write path exists.** `mycelix-personal/apps/leptos` is a mock display shell (`context.rs:8` "renders stable typed mock data now"); health records are only a *count* from `mock_data`, with **no `create_health_record` call anywhere**. The vault's real write feature (form → `call_zome` create) must be built first — app work, out of PQC scope. Encrypt at that point via `field_crypto::encrypt_field` before the zome call; decrypt via `decrypt_field` on read (fall back to plaintext when `!is_encrypted_field`).
    2. **Browser getrandom backend.** The `mycelix-identity` workspace configures getrandom for the *zome* target (`getrandom 0.2 custom`; "HDK provides `__getrandom_v03_custom`", root `Cargo.toml:76-85`). A browser client has no HDK, so it must register a web-crypto backend for the getrandom versions ml-kem/ml-dsa pull (0.2 **and** 0.3/0.4). The shared client already reaches `crypto.getRandomValues` (`crates/mycelix-leptos-client/src/browser.rs:690`) — reuse it via `register_custom_getrandom!` (0.2) + the `__getrandom_v03_custom` shim (0.3/0.4), or add the `wasm_js`/`js` features on the app side. **Requires iterating the leptos wasm build to verify at runtime** — do this in a dedicated build session, not autonomously.
    3. Agent hybrid-keypair gen + storage (localStorage now; for a personal vault the recipient KEM keypair and sender signer are the same self-agent).
- **Phase 5 — Governance native verify (~3-5d).** feldman-dkg ML-DSA is real but test-only; call it in a native coordinator/sidecar so threshold sigs are actually verified.

Total ~4-6 focused weeks.

## 6. Beyond PQC (make it better regardless of quantum)

- Actually **verify** PQC signatures (length-check-only is theater).
- **Encrypt the plaintext personal vaults.**
- One versioned envelope + one algorithm registry = single crypto-agility control point.
- Key storage off localStorage.
- CI: no-new-bespoke-crypto-crate gate; PQC-labeled entries must carry a verified signature.

---

## 7. Phase-0 implementation spec (turnkey)

**Goal:** give `mycelix-crypto` a real hybrid-KEM seal/open that builds for wasm32.
**Target crate:** `mycelix-workspace/mycelix-identity/crates/mycelix-crypto`.

**Cargo.toml — new opt-in feature (does NOT touch existing `wasm`/`native` consumers):**
```toml
[features]
# RustCrypto hybrid backend — real PQC that builds for wasm32 (browser clients)
hybrid-rc = ["dep:ml-kem", "dep:ml-dsa", "dep:x25519-dalek", "dep:ed25519-dalek",
             "dep:hkdf", "dep:sha2", "dep:chacha20poly1305", "dep:rand_core"]

[dependencies]
ml-kem = { version = "=0.3.0-rc.2", default-features = false, features = ["getrandom"], optional = true }
ml-dsa = { version = "=0.1.1", optional = true }
x25519-dalek = { version = "2", optional = true, features = ["static_secrets"] }
hkdf = { version = "0.12", optional = true }
# ed25519-dalek / chacha20poly1305 already present as optional
```

**New module `src/pqc/hybrid_kem.rs` — construction (matches Pulse's reviewed `encrypt_hybrid`,
X-Wing-style, RNG via the crates' internal getrandom):**
- keygen: `let (dk, ek) = MlKem768::generate_keypair();` + X25519 static keypair.
- seal(recipient_x25519_pk, recipient_mlkem_ek_bytes, plaintext):
  1. ephemeral X25519 keypair; `ss1 = DH(eph_sk, recipient_x25519_pk)`
  2. `let (mlkem_ct, ss2) = ek.encapsulate();` (ek reconstructed from bytes)
  3. `ikm = ss1 || ss2.as_slice()`
  4. `key = HKDF-SHA256(salt = context_transcript, ikm, info = "mycelix-hybrid-kem-v1")[..32]`
     — bind `eph_x25519_pk || mlkem_ct` into the salt/info (transcript binding vs re-encaps).
  5. random 24-byte XNonce; `ct = XChaCha20Poly1305(key, nonce, plaintext)`
  6. return `{ x25519_eph_pk (32), mlkem_ct, nonce (24), ciphertext }`
- open: `ss1 = DH(recipient_x25519_sk, eph_pk)`; `ss2 = dk.decapsulate(&mlkem_ct)`; same HKDF; AEAD decrypt.

**Exact ml-kem 0.3.0-rc.2 API** (verified from registry source):
`use ml_kem::{MlKem768, kem::{Decapsulate, Encapsulate, Kem}};`
`let (dk, ek) = MlKem768::generate_keypair(); let (ct, k) = ek.encapsulate(); let k2 = dk.decapsulate(&ct);`
`SharedKey = Array<u8, U32>`; key/ct byte serialization via `EncodedSizeUser` (`.as_bytes()` /
`from_bytes(&Encoded<_>)`). **ml-dsa 0.1.1**: `MlDsaSigningKey::<MlDsa65>::generate()`, `signature::{Signer,Verifier}`.
Working reference: `xenia/.../src/handshake.rs:44-64,462-498`.

**getrandom note:** ml-kem's `getrandom` feature works on native (OS backend). For a *browser
client* build, the client crate must supply `getrandom` `wasm_js` — NOT mycelix-crypto's
crate-level `features=["custom"]` (that is HDK's zome backend). This is resolved at the
client (Pulse) integration, not in mycelix-crypto. The Pulse Leptos client is conflict-free
(no `aead 0.6-rc`/`iroh` — the clash that disabled ml-kem in the root workspace `Cargo.toml:746`).

**Tests (native, `cargo test -p mycelix-crypto --features hybrid-rc`):**
seal→open round-trip; tamper-a-byte→open fails; wrong-recipient-key→open fails; empty plaintext.

**Review gate (blocking):** before wiring `hybrid_kem` into any cluster, a crypto reviewer
must sign off on: the KEM-combiner (concatenation + HKDF with transcript binding), the AEAD
nonce policy (24-byte random XChaCha is fine; do NOT reuse), and the "encrypt-then-sign vs
sign-then-encrypt" order for `SealedEnvelope`. Ship `hybrid-rc` as a **non-default feature**
until then — it affects nothing while unwired.

## 8. First action

Do **Phase 0 §7** as a focused, reviewed effort (fast leaf-crate builds, ~minutes each — unlike
the zome workspaces). If picking the first *user-facing* target, do **Phase 4 (encrypt the
plaintext vaults)** before the PQC niceties — it is a live exposure today.
