# Mycelix Pulse — Security-Focused Working Alpha

Mycelix Pulse is an authenticated encrypted-messaging alpha built with Leptos
and Holochain. The canonical release surface is deliberately smaller than the
repository's wider product vision.

**Canonical public origin:** `mail.mycelix.net`

## Implemented alpha claim

The supported browser path provides:

- device-local X25519 and ML-KEM-768 recipient keys;
- AES-256-GCM message encryption;
- Ed25519 agent authentication plus ML-DSA-65 bundle authentication;
- canonical transcript and AAD encoding;
- Holochain profiles, trust, keys, messages, and capability zomes;
- a plaintext-by-construction composer and renderer;
- explicit Live, Demo, and Unavailable runtime states;
- a service worker that never handles or caches `/api/*` credentials;
- same-browser-profile wrapped-key recovery through IndexedDB.

Calendar, contacts, search, chat, meetings, SMTP federation, economic staking,
and other screens remain experimental or frozen unless
`docs/SURFACE_STATUS.md` says otherwise. Their presence is not a release claim.

## Canonical architecture

```text
Leptos WASM
    │
    ├── POST /api/token and /api/signing-credentials
    │       └── authenticated loopback credential broker
    │               └── private Holochain admin interface
    │
    └── authenticated app WebSocket
            └── restricted alpha hApp
                    ├── mail_profiles
                    ├── mail_trust
                    ├── mail_keys
                    ├── mail_messages
                    └── mail_capabilities
```

The installed app id and role remain `mycelix_mail` and `main` for compatibility.

## Build

The Leptos crate has path dependencies on the surrounding Mycelix workspace.
Build from the complete workspace, not from this directory in isolation.

```bash
rustup target add wasm32-unknown-unknown
cargo install trunk
cd apps/leptos
trunk build --release
```

## Local runtime

A local app interface normally listens at `ws://localhost:8888`. Live mode is
published only after transport authentication, signer acquisition, a signed
`mail_profiles.get_my_profile` call, and response decoding all succeed. Failure
becomes **Unavailable**; the app does not silently replace Live data with demo
data. Demo mode requires an explicit path.

## Credential broker

The browser must never reach the Holochain admin interface. Deploy the reference
broker on loopback behind an authenticating reverse proxy:

```bash
npm ci --prefix scripts/live-browser-proof --omit=dev
npm test --prefix services/pulse-credential-broker
npm start --prefix services/pulse-credential-broker
```

See `services/pulse-credential-broker/README.md` and
`deploy/nginx/pulse-security-headers.conf` for the required trust boundary.

## Verification

Fast source and manifest guard:

```bash
python3 scripts/verify-pulse-alpha-boundary.py
node --test services/pulse-credential-broker/test/*.test.mjs
```

The full live-browser proof drives two conductors and two independent browser
contexts through onboarding, wrapped-identity reload recovery, send, gossip,
verification, decryption, and rendering. See `scripts/live-browser-proof/`.

## Repository status

`docs/SURFACE_STATUS.md` is authoritative. Legacy React, Node, desktop, mobile,
SMTP, alternate DNA, and broad deployment trees remain reference or
experimental surfaces until explicitly promoted.

## License

AGPL-3.0-or-later
