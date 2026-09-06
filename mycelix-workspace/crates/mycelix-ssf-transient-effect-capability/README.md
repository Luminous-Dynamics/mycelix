# SSF Transient Effect Capability v0.1

This crate constructs the first SSF object that intentionally carries effect authority, but it still contains no actuator implementation and performs no external effect.

The capability is private-field, non-`Clone`, non-`Copy`, and has no serialization dependency. Its only public constructor consumes an exact `FinalEffectCapabilityReadyV1` plus fresh trusted time. There is no constructor from the copyable audit binding.

The capability carries the durable capability record as its single-use identity together with the exact final revalidation receipt, execution generation, provider descriptor, provider-owned opaque operation handle, handle-resolution evidence, operation carrier, payload/encoding commitments, payload length, mint-time evidence commitment, and natural expiry.

No caller-supplied payload bytes or replacement handle enter this boundary. A future actuator must consume the owned capability by value and atomically claim the durable capability record before resolving/invoking the provider-owned handle.
