# mycelix-leptos-client

**Experimental browser Holochain transport for Leptos frontends.**

A browser-native Rust transport experiment intended for WASM frontends. It is
not yet a drop-in replacement for
[`@holochain/client`](https://www.npmjs.com/package/@holochain/client).

Uses `web-sys::WebSocket` + [`rmp-serde`](https://crates.io/crates/rmp-serde) (MessagePack) to communicate with a Holochain conductor over binary WebSocket frames.

## Why

The official Rust client, [`holochain_client`](https://crates.io/crates/holochain_client), is built for native Rust — it transitively depends on `tokio`, `holochain_websocket`, and other crates that do not compile to `wasm32-unknown-unknown`. For browser WASM frontends, the official path is `@holochain/client` (TypeScript) + some JS bundling layer.

`mycelix-leptos-client` explores a third option: a browser-native Rust client
that avoids a TypeScript application layer. Connection and app-info discovery
are implemented. Zome calls fail closed unless an authorized signer is
installed through Rust or the official launcher host-signer convention.

## Features

- **Rust-first WASM transport.** WebSocket and protocol handling stay in Rust;
  launcher deployments can delegate key custody to the standard host-signer hook.
- **MessagePack wire format.** Matches what Holochain conductors speak natively; no JSON adaptation layer.
- **Trait-based transport** so the same `HolochainClient` type works across:
  - [`BrowserWsTransport`] — browser WebSocket (feature `browser`, default)
  - [`NativeWsTransport`] — native integration tests (feature `native`)
  - `TauriIpcTransport` — planned, invoke a Tauri backend
  - `MockTransport` — unit tests, no network
- **Typed zome calls** via serde-derived input/output structs, gated on an explicit signer.
- **No unsigned fallback.** Missing credentials, zero signatures, and unavailable secure randomness are errors.
- **AGPL-3.0-or-later.** Intended for application-layer code. Pair with your own AGPL app or negotiate a commercial exception.

## Example

```rust
use mycelix_leptos_client::{HolochainClient, BrowserWsTransport};
use serde::{Serialize, Deserialize};

#[derive(Serialize)]
struct CreateProposal { title: String, body: String }

#[derive(Deserialize)]
struct ProposalHash { hash: Vec<u8> }

async fn example() -> Result<(), Box<dyn std::error::Error>> {
    let transport = BrowserWsTransport::new();
    // A launcher/native host must expose window.__HC_ZOME_CALL_SIGNER__, or
    // the application must call transport.set_zome_call_signer(...).
    let client = HolochainClient::new(transport, "mycelix-unified", "governance");

    // Holochain 0.6 app WebSockets require an app authentication token.
    client.connect("ws://localhost:8888", Some(app_token)).await?;

    let result: ProposalHash = client
        .call_zome(
            "agora",
            "create_proposal",
            &CreateProposal {
                title: "Test".into(),
                body:  "Body".into(),
            },
        )
        .await?;

    Ok(())
}
```

## Cargo feature flags

| Feature | Default | What it turns on |
|---|---|---|
| `browser` | yes | `BrowserWsTransport` over `web-sys::WebSocket` |
| `tauri` | no | Tauri IPC transport (planned) |
| `native` | no | `NativeWsTransport` over `tokio-tungstenite` — integration tests only |

The default feature set is correct for a Leptos WASM frontend. For a Tauri desktop app wrapper you would enable `tauri` instead.

## Compatibility

- **Holochain conductor:** targets the 0.6 app wire and official `CallZomeRequestSigned`/host-signer shapes; release still requires a real-conductor conformance lane.
- **Rust:** 1.94+ (edition 2024, as declared by the crate manifest).
- **Browser:** anything with WebSocket + WebCrypto; no Service Worker assumptions.

## Readiness

Treat the browser transport as experimental until tests exercise authentication,
authorized host signing, zome calls, errors, signals, reconnects, and payload
compatibility through this implementation against each supported conductor
line. Tests that use only the official client do not establish compatibility
for this crate.

Run the dependency-free source drift guard before the Rust/WASM and
real-conductor lanes:

```sh
python3 scripts/validate_wire_contract.py
```

The connection and reconnect state machine is documented in
[`docs/transport-lifecycle.md`](docs/transport-lifecycle.md).

## License

AGPL-3.0-or-later. See [`LICENSE`](LICENSE) for the full text.

Commercial licensing: contact the authors for dual-license terms if AGPL does not fit your deployment shape.

## Contributing

This crate is developed in the private Luminous Dynamics monorepo. External PRs on the public mirror are welcome and will be carried forward into the monorepo source of truth by the maintainers. Do not expect a fast turnaround yet — the public repo is a source mirror, not a primary-development surface.
