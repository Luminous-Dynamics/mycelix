# Craft temporal causal-frontier oracle V0.1

Independent Go reference for a **bounded single-epoch-per-stream** causal-frontier profile. This is test-only; it is not production runtime, a transparency log, source authentication, owner-universe completeness or a general distributed database.

## Contract

- Each selected stream cut binds a stream ID, epoch, contiguous offset and exact head digest.
- Included events must have unique IDs and unique stream coordinates, valid SHA-256 digests, canonical parent sets and complete causal-parent closure.
- Each selected stream prefix must be contiguous from offset 1 through the declared cut.
- A cross-stream edge exists only when a selected event names another selected event as a causal parent. Equal timestamps do not create order.
- A fork, head mismatch, cycle or non-causal same-stream parent fails closed.
- Missing prefix offsets or parents yield `Incomplete`.
- Payload fields of events beyond the selected stream cut are deliberately not validated; appending a malformed future payload must not rewrite an earlier receipt.
- `SourceUniverseCompleteness` is always `NotEvaluated`; stream-frontier validity does not prove that every source/owner is represented.
- The receipt commits to every selected entry, not just stream-head digests. Nil and empty parent sets have one canonical representation.

Epoch rollover is unsupported in V0.1 and must be added only with explicit, authenticated epoch-transition and parent-closure semantics. The oracle assumes admission coordinates have already been authenticated by their owner; it does not verify signatures or transparency proofs. It checks digest syntax and commitment consistency only; it does not possess source payload bytes to validate the source event digest.

## Run

```sh
gofmt -w oracle/*.go
go test ./...
go test -race ./...
go vet ./...
```

## Claim ceiling

Tests qualify only this bounded reference algorithm and its frozen unit cases. They do not establish event-source authenticity, source-universe completeness, wall-clock correctness, legal authority, field truth or operational outcomes.

The profile has explicit implementation bounds: at most 4,096 stream cuts, 1,000,000 included entries and offsets no greater than 1,000,000. This is a single-epoch-per-stream research oracle. It is not a general distributed log implementation.
