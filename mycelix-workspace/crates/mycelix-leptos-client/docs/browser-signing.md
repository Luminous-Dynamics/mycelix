# Browser zome-call signing contract

`BrowserWsTransport` never constructs an unsigned or zero-signed zome call.
After app authentication and role discovery, every call requires one of:

1. a Rust implementation installed with `set_zome_call_signer`, or
2. the official launcher convention at
   `window.__HC_ZOME_CALL_SIGNER__.signZomeCall(request)`.

The host callback receives this shape:

```text
{
  cell_id: [Uint8Array, Uint8Array],
  zome_name: string,
  fn_name: string,
  provenance: Uint8Array,
  payload: unknown
}
```

It must return a promise resolving to:

```text
{
  bytes: Uint8Array,
  signature: Uint8Array(64)
}
```

This is the official unsigned `CallZomeRequest` shape: `payload` is the decoded
JavaScript input value, not pre-encoded ExternIO bytes. The host is responsible
for selecting conductor-authorized signing credentials and capability secret;
encoding the payload and unsigned call parameters; generating the nonce and
expiration; hashing as required by the supported conductor; and producing the
Ed25519 signature. Raw private keys do not cross into the Leptos WASM
application.

The built-in Rust-to-host adapter currently accepts JSON-shaped MessagePack
payloads (maps with string keys, arrays, strings, booleans, null, and JSON
numbers). It fails closed before signing if an input uses MessagePack binary or
extension values that cannot be represented without changing their meaning.

The transport rejects a missing callback, non-`Uint8Array` result, empty call
bytes, wrong signature length, or an all-zero signature before network send.
Nonce generation also fails closed if Web Crypto is unavailable.

Authentication and signing are distinct. A valid app-interface token permits
the WebSocket and app-info request; it does not authorize zome calls by itself.
