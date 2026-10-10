# CIV-ECON-001A transition validator

This dependency-free Rust reference model checks invariants defined in
`docs/research/civ/CIV-ECON-001A_REGIME_PROFILE_AND_TRANSITION_PROTOCOL_V1.md`.

It operates on a **normalized typed projection** of a transition manifest.
A JSON-Schema-to-Rust adapter is not implemented in this crate yet, and the
reference model does not parse the schema file. It checks:

- versioned policy-profile, economic-instrument, and unit identities as distinct types;
- source-scope coverage with exactly one typed mapping per item;
- exact identity and atomic-quantity preservation for Preserve and ReReserve;
- explicit conversion profiles, authority decisions, novation acceptance, and discharge receipts;
- source-origin preservation across conversion and novation;
- lifecycle progression and frozen manifest-digest binding;
- simulation run receipt, output digest, scenario-corpus digest, and environment binding;
- evaluator binding to the exact simulation output and scenario corpus, with a separate evaluation report digest and receipt;
- rights-floor, resource-reservation, external-obligation, cutover, and reconciliation gates;
- idempotent effect identity, duplicate replay, and payload-conflict rejection;
- append-only local effect intent/receipt journal with fsync before dispatch, strict replay, and unresolved-effect recovery.
- a conservative external-effect recovery policy that binds retries to the exact request, explicit idempotency-guarantee deadlines, and fresh no-effect evidence.

## Run

From this directory:

    cargo test --offline
    cargo fmt --check
    cargo clippy --offline --all-targets -- -D warnings

The crate has an isolated Cargo workspace and no third-party dependencies.

The `EffectLedger` in `lib.rs` remains an in-memory reference model. The new
`durable_journal::DurableEffectJournal` persists begin, indeterminate, and
acknowledgement records to an append-only local file. It reads legacy J1 records
and writes J2 records, syncing each before returning success. New J2 acknowledgements
bind both the provider receipt digest and source-evidence digest; a legacy J1
acknowledgement replays with no source-evidence binding and reports that absence
explicitly. It refuses concurrent cooperating writers with a PID-marked sidecar
lock and fails closed on truncated/corrupt history. A surviving Pending record
after restart is a reconciliation requirement, never permission to dispatch again.

The integration tests in `tests/crash_recovery.rs` launch child processes to
exercise two distinct crash windows: after a synthetic external effect but before
journal acknowledgement, and after the acknowledgement is synchronized but before
the caller can observe success. The parent explicitly clears each dead child's
stale test lock, reopens the journal, checks the recovered state/receipt, and
asserts no duplicate dispatch. These are process-boundary tests with a synthetic
provider ledger only; they do not contact or prove behavior of a real payment
provider, bank, physical actuator, or settlement authority.

The `external_effect_policy` module is a pure decision policy, not a provider
adapter. Provider observations and idempotency contracts are bound to an explicit
provider-profile digest and the exact request digest; an applied result also
carries a source-evidence digest that must be independently verified. With an
unknown outcome, the policy permits a retry only when the same request and
provider profile are bound to a still-valid idempotency guarantee; at the
guarantee's exact expiry second, it remains indeterminate. A fresh, request-bound,
recent no-effect observation can authorize a new key only when it also asserts
that no prior request remains in flight. Conflicting or misbound evidence is
quarantined. The module does not authenticate these claims, query a provider,
dispatch a request, or persist a retry authorization; those remain responsibilities
of a verified adapter and the future transition driver.

This journal is not tamper-evident: it does not hash-chain or sign the log,
and it stores supplied request, receipt, and source-evidence digests without
authenticating the provider or independently verifying the source evidence. J2
preserves the source-evidence digest alongside the receipt digest; digest presence
alone is not proof that the underlying event occurred. On Unix, new journal and
lock files are created with owner-only mode (0600), and an existing journal with
group/other permission bits is rejected. The parent directory must still be trusted and protected against
unauthorized replacement; non-Unix deployments must verify their native ACL
policy. These controls do not authenticate external evidence. The journal also
cannot atomically commit a local record with an external bank/payment/physical
side effect. Filesystem sync semantics are platform and storage dependent, stale
lock files require deliberate operator recovery, and the caller must validate
source-bound receipts. Do not describe this as exactly-once distributed execution.

## Claim ceiling

This is a reference validator, not a source of authority. It does not hash
canonical manifests, verify signatures, query live state, establish authority
validity/currentness, prove physical capacity, prove counterparty acceptance,
verify conversion arithmetic from source data, establish legal discharge, or
establish external settlement finality. Production work still needs the JSON
adapter, live evidence bindings, and independent qualification.

The unit and integration tests are authored regression tests, not a PASS claim
until the exact-head CI run executes and succeeds. Even then, passing them proves
only the local invariants actually tested, not end-to-end transition correctness
or production readiness.
