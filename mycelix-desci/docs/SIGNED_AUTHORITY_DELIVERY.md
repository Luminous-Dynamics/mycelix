# Signed Authority Delivery Envelopes

Every PostgreSQL outbox row contains a domain-separated Ed25519-signed delivery
envelope. Leasing, retries, and HTTP relays may change operational metadata, but
they cannot change the committed publication payload.

## Bound fields

The signature binds:

- protocol, codec, and schema version;
- delivery UUID;
- topic;
- aggregate identifier and sequence;
- server-controlled creation time; and
- the BLAKE3 payload hash.

The payload is carried inside the envelope and must reproduce the committed
hash. Receivers should verify the signature and `delivery_hash`, then deduplicate
by `delivery_id`. The API publisher also sends `Idempotency-Key`,
`X-Mycelix-Outbox-Id`, and `X-Mycelix-Delivery-Hash` headers.

## Key separation

`DESCI_AUTHORITY_OUTBOX_SIGNING_KEY_FILE` must contain a dedicated mode-0600
Ed25519 seed. PostgreSQL startup rejects reuse of any trusted authority-receipt
key. Operators must also keep this key separate from scientific actor keys.

The current public key is exposed in the health response so subscribers can pin
it through an authenticated deployment channel. Key rotation requires retaining
the former public key at subscribers until every old envelope has been consumed.

## Delivery semantics

Publication is at least once. A successful 2xx response marks the row published;
timeouts and non-2xx responses leave it retryable. A valid signature proves the
envelope was produced by the configured authority-delivery service, not that a
subscriber stored it permanently.
