# mycelix-lawful-identity

The **legal-DID** namespace for Mycelix. It explores government-ID-backed credential references, issuer trust tiers, and cross-DID coordination interfaces — strictly isolated from the primary identity namespace. The current cross-DID coordinator issues nonces and records submitted proof strings for replay bookkeeping; it explicitly does not verify those proof strings cryptographically.

Part of the [state-coexistence extensions](../MYCELIX_STATE_COEXISTENCE.md).

## The dual-DID rule

```
did:mycelix:primary  ──  governance, MYCEL, TEND, social graph  (pseudonymous)
did:mycelix:legal    ──  ONLY state-facing interactions         (this cluster)
```

The design goal is to minimize direct on-chain linkability between the two DID namespaces. The current cross-DID zome provides nonce issuance and replay bookkeeping, not a verified zero-knowledge bridge. Do not claim that a submitted proof is valid or that unlinkability has been demonstrated until a real proof verifier, independent tests, and privacy analysis are in place.

## What's here

| Path | What |
|------|------|
| `zomes/legal-did/` | Isolated DID namespace with distinct DHT partition |
| `zomes/issuer-trust-tier/` | Three tiers: Sovereign / RegulatedIntermediary / Peer |
| `zomes/cross-did-zkp/` | Nonce issuance and proof-submission replay bookkeeping; current coordinator stores proof bytes without cryptographically verifying them |
| `cli/` | `lawful-id` CLI for legal-DID lifecycle, credential-hash import, nonce requests, and onboarding disclosures; inspect each command for whether it is live or locally staged |
| `docs/THREAT_MODEL.md` | Four adversary vectors: what's cryptographically neutralized, what isn't |
| `docs/GOV_ID_CLAIM_SHAPES.md` | Claim-key conventions for mDL / passport / SSN-equivalent |

## What's NOT here

- No KYC enforcement on primary-DID functions.
- No governance tier bonus for verified legal identity.
- No sanctions screening (kills mutual credit; delegates sovereignty).
- No "legal name" field on the primary DID profile.
- No court-order key disclosure protocol (separate `mycelix-lawful-response` cluster if ever built).

## Underlying credential and proof components

The `mycelix-identity/crates/eidas-zkp/` crate currently provides experimental VC-shaped structures, caller-supplied proof metadata, and ordinary JSON claim filtering. It does **not** generate or verify DASTARK/ML-DSA proofs, implement an interoperable W3C Data Integrity cryptosuite, or establish eIDAS/EUDI Wallet conformance. Do not treat a `ProvenClaim` description as a verified predicate.

There is a separate, feature-gated Winterfell-backed jurisdiction-containment circuit in [`mycelix-zkp-core::circuits::jurisdiction_proof`](../crates/mycelix-zkp-core/src/circuits/jurisdiction_proof.rs). When built with `backend-winterfell`, it generates and verifies range proofs that a supplied coordinate lies within a selected bounding box. **This proves containment of the submitted coordinate, not that the coordinate is truthful or came from a trusted device.** The associated [location-attestation adapters](../crates/mycelix-zkp-core/src/location_attestation.rs) have separate trust requirements: the current T1 phone-GPS adapter does not perform the downstream device-signature verification itself, and T2–T4 adapters are documented stubs returning `NotImplemented`.

The cross-DID coordinator is also separate from proof verification: it issues nonces and stores caller-submitted proof strings for replay bookkeeping; its own module documentation explicitly says it does not cryptographically verify `proof_value`. The claim-key conventions in `docs/GOV_ID_CLAIM_SHAPES.md` are data-shape guidance, not issuer assurance or proof-generation code. See the [Mycelix Identity Interoperability and Assurance Roadmap](../docs/identity/INTEROPERABILITY_AND_ASSURANCE.md).

## Build

```bash
nix develop ../mycelix-praxis    # provides hc, holochain, lair-keystore
cargo build --workspace --target wasm32-unknown-unknown --release
hc dna pack dna/ -o dna/mycelix_lawful_identity.dna
hc app pack . -o mycelix-lawful-identity.happ
```

See [`QUICKSTART.md`](./QUICKSTART.md) for the full install-on-conductor
recipe and end-to-end CLI walkthrough.

## Install on the shared ecosystem conductor

```bash
# Assumes the Mycelix ecosystem conductor is running on :33800 (admin).
# See ../CLAUDE.md for conductor-startup details.
hc sandbox call --running=33800 install-app \
    /srv/luminous-dynamics/mycelix-lawful-identity/mycelix-lawful-identity.happ \
    --app-id mycelix-lawful-identity
hc sandbox call --running=33800 enable-app mycelix-lawful-identity
```

## Live CLI — `lawful-id`

The `lawful-id` binary in `cli/` has two modes: a lightweight
default build (call-sheet only, ~5 MB binary) and a **live** build
with `--features conductor` that makes real zome calls against a
running Holochain conductor. The full seven-function zome surface
is reachable from the CLI:

| Command | Zome function | Notes |
|---------|---------------|-------|
| `lawful-id init` | — | First-run threat-model disclosure + acknowledge |
| `lawful-id disclose` | — | Reprint disclosure on demand |
| `lawful-id status` | — | Local state summary (staged DIDs/issuers) |
| `lawful-id ping` | `legal_did.ping` | Liveness check |
| `lawful-id new-legal-did --live [--label X]` | `legal_did.create_legal_did` | Creates `did:mycelix:legal:<64-hex>` |
| `lawful-id list-dids --live` | `legal_did.list_my_legal_dids` | Your DIDs on this agent key |
| `lawful-id import-credential --live ...` | `legal_did.import_credential` | Attach passport/mDL/SSN-derived credential |
| `lawful-id list-credentials DID` | `legal_did.get_credentials_for_did` | Credentials attached to a legal DID |
| `lawful-id classify-issuer DID --tier T --live` | `issuer_trust_tier.classify_issuer` | Sovereign / RegulatedIntermediary / Peer |
| `lawful-id lookup-tier DID` | `issuer_trust_tier.lookup_tier` | Latest tier for an issuer |
| `lawful-id request-nonce DID` | `cross_did_zkp.request_nonce` | 32-byte base64 nonce for a verifier |
| `lawful-id new-legal-did [--label X]` | — | Stage intent locally (no `--live`) |
| `lawful-id classify-issuer DID --tier T` | — | Stage intent locally |
| `lawful-id call-sheet` | — | Emit ready-to-paste `hc sandbox call` strings |

**Build with live conductor support** (heavy dep tree, ~200 MB
transitive; off by default):

```bash
cd cli/
cargo build --features conductor
```

## Example: full passport-import flow

```bash
# First-run disclosure + acknowledge.
lawful-id init

# Create a legal DID.
lawful-id new-legal-did --live --label "SA passport"
# → Created legal DID: did:mycelix:legal:34b1fd05…e4bcd13e

# Classify the issuer.
lawful-id classify-issuer did:web:home.affairs.gov.za \
    --tier sovereign --rationale "SA Dept of Home Affairs" --live

# Attach the credential (hash commitment + metadata; the credential
# body itself never reaches the DHT).
lawful-id import-credential --live \
    --legal-did "did:mycelix:legal:34b1fd05…e4bcd13e" \
    --credential-hash "blake3:7a8b9c0d…" \
    --issuer-did "did:web:home.affairs.gov.za" \
    --credential-type "SaIdCredential" \
    --issued-at "2022-05-10" --expires-at "2032-05-10"

# Verify.
lawful-id list-credentials "did:mycelix:legal:34b1fd05…e4bcd13e"
lawful-id lookup-tier did:web:home.affairs.gov.za
```

## Port

Frontend reserved at `8133` (see `.claude/rules/PORTS.md`).
Domain: `lawful.mycelix.net`.

## License

AGPL-3.0-or-later (matching the rest of the Mycelix workspace).
