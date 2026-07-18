# Mycelix Marketplace Leptos migration

This workspace is the staged replacement for the SvelteKit frontend in `../frontend`.

## Invariants

- Production builds fail closed when no live transport is configured.
- Development fixtures require the explicit `dev-fixtures` feature.
- Pages do not construct raw zome or function names.
- Typed request and response structures mirror the backend coordinator contract.
- The Svelte application remains untouched until live migration gates pass.

## Layout

- `crates/marketplace-domain`: wire-compatible domain and identifier types.
- `crates/marketplace-client`: exact typed coordinator calls over a transport trait.
- `apps/marketplace-web`: Leptos CSR shell and initial browse/listing routes.

## Commands

Install the pinned bridge dependencies and Trunk, then run the fail-closed live application:

```sh
cd frontend-leptos
npm --prefix bridge ci
trunk serve apps/marketplace-web/index.html
```

The bridge reads Launcher/Tauri-injected configuration when available. A separated browser deployment can set an app interface URL and token before WASM starts:

```html
<script>
window.__MYCELIX_MARKETPLACE_CONFIG__ = {
  url: "ws://127.0.0.1:8888",
  tokenBase64: "APP_INTERFACE_TOKEN_BASE64URL",
  origin: "mycelix-marketplace",
};
</script>
```

Do not expose the conductor admin interface to the web application. Signing must come from an authorized official-client credential or trusted host signer.

Run the explicitly labeled fixture preview:

```sh
cd frontend-leptos
trunk serve apps/marketplace-web/index.html --no-default-features --features dev-fixtures
```

The fixture build is for visual migration work only. It is not evidence of a live Holochain integration.

## Vertical slice V1

The current migration gate covers:

- live listing discovery;
- typed `ActionHash` listing routes;
- DHT-backed listing detail;
- guarded pending-transaction creation;
- direct transaction recovery;
- current-agent transaction indexing.

Transaction lifecycle controls are deliberately not exposed yet. The backend must first reduce Holochain update relationships to the newest valid revision for both direct and link-based reads. See `../docs/migration/LEPTOS_VERTICAL_SLICE_V1.md` for the two-agent conductor procedure and evidence requirements.

Run every static, bridge, Rust, and WASM gate with:

```sh
cd ..
scripts/validate-leptos-migration.sh
```
