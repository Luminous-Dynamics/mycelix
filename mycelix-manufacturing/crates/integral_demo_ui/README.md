# Integral Reference Node UI

A Leptos CSR cockpit for the bounded Integral reference-node model.

The UI is deliberately downstream of the machine-readable trace:

`reference trace → validation → deterministic cockpit projection → Leptos presentation`

The presentation layer must not create evidence, authority, provenance, or legitimacy. Symthaea may later provide assistive explanations, but the cockpit remains understandable without it.

## Local development

From `mycelix-manufacturing/crates/integral_demo_ui`:

```bash
rustup target add wasm32-unknown-unknown
cargo install --locked trunk
trunk serve
```

The UI uses Leptos 0.8.21 CSR. The current implementation is a reference/demo surface, not a production Integral node.
