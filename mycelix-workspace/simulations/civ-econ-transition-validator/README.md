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

### Source-contract preflight

Run the fast, stdlib-only structural audit before the Rust checks:

    python3 scripts/audit_journal_source_contract.py

It verifies selected source invariants, regression-test annotations, scale-scenario
wiring, and the exact-head qualification workflow; it emits SHA-256 fingerprints
for each reviewed source/artifact. When `QUALIFIED_SHA` is set (as it is in the
pull-request workflow), the audit requires Git metadata to be available and the
checked-out commit to match that exact subject. A missing or mismatched subject
fails the audit. This is an early process check, not a Rust parser, compiler,
unit test, Clippy run, or proof of runtime correctness. In CI, the report is
written to the runner's temporary directory and uploaded as a run-specific
artifact for 14 days, including when the preflight fails after producing the
report. The JSON binds the report to the observed commit and expected
`QUALIFIED_SHA`, includes the run ID when available, and carries hashes of the
reviewed files. Artifact retention is a convenience for review, not immutable
long-term archival or a cryptographic attestation.

### Journal replay scale probe

Build the dependency-free scale probe once, then run each size separately so
`/usr/bin/time -v` captures the probe process rather than Cargo's build process:

    cargo build --offline --release --bin journal-replay-scale
    for scenario in pending indeterminate acknowledged reconciled; do
      for n in 10000 100000 1000000; do
        printf '\n== %s effects / %s ==\n' "$n" "$scenario"
        /usr/bin/time -v target/release/journal-replay-scale "$n" "$scenario" 2>&1
      done
    done

The probe generates a synthetic J3 journal, syncs it, then reports journal bytes,
bytes per effect, generation time, and open-plus-replay time. Scenarios cover
Pending, Indeterminate, acknowledged directly, and Indeterminate-to-acknowledged
reconciliation. It verifies total replayed identity count, unresolved count, and
the expected first/last final states without allocating an ID list. `/usr/bin/time -v` supplies peak
resident-set size (RSS). The probe accepts 1 through 1,000,000 effects and deletes
its private temporary directory after each run.

Record the commit, OS/kernel, filesystem/storage, Rust version, effect count,
journal bytes, generation time, open-plus-replay time, and maximum RSS when
comparing runs. Compare repeated runs on the same machine; do not infer asymptotic
complexity or production capacity from one sample. The current harness does not
measure allocator-level peak memory in isolation and has not itself established
performance numbers in this repository yet.
The exact-head PR workflow also runs all four state scenarios at 10,000 effects
after tests, Clippy, and formatting checks. It records runner image, kernel,
filesystem information, Rust version, wall time, and peak RSS as a smoke-level
measurement. This checks that the harness executes in the pinned CI environment;
it is not a substitute for repeated 100k/1m runs on a documented benchmark host.

Before implementing compaction, use the explicit [checkpoint/compaction design gate](JOURNAL_CHECKPOINT_COMPACTION_DESIGN.md). It records why acknowledged effect IDs must remain as no-redispatch tombstones, which crash boundaries require tests, and why compaction alone cannot reduce the memory needed for the full identity index. The current replay parser also uses borrowed fixed-field splitting rather than allocating a Vec of fields per record.

The `EffectLedger` in `lib.rs` remains an in-memory reference model. The new
`durable_journal::DurableEffectJournal` persists begin, indeterminate, and
acknowledgement records to an append-only local file. It reads legacy J1/J2
records and writes J3 records, syncing each before returning success. New J3
begin/indeterminate/acknowledgement records bind the operation to a provider-profile
digest; acknowledgements also bind the receipt and source-evidence digests. Legacy
J1/J2 entries replay with no provider-profile binding and may not be automatically
resumed or acknowledged under a caller-selected provider: they require explicit
reconciliation. Legacy acknowledgements report absent bindings as `None`, rather
than upgrading their evidence claim. Startup replay is streaming: a bounded reader
processes one newline-terminated record at a time (maximum 4,096 record bytes and
1,024 decoded effect-ID bytes), rejects an unterminated final record, and does not
create a second in-memory copy of the complete journal. The effect index itself
still grows with the number of distinct effect IDs. The journal refuses concurrent
cooperating writers with a PID-marked sidecar lock and fails closed on truncated/corrupt history.
A surviving Pending record after restart is a reconciliation requirement, never
permission to dispatch again.

The integration tests in `tests/crash_recovery.rs` launch child processes to
exercise two distinct crash windows: after a synthetic external effect but before
journal acknowledgement, and after the acknowledgement is synchronized but before
the caller can observe success. The parent explicitly clears each dead child's
stale test lock, reopens the journal, checks the recovered receipt, source-evidence
digest, and provider-profile binding, and asserts no duplicate dispatch. These are
process-boundary tests with a synthetic provider ledger only; they do not contact
or prove behavior of a real payment
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
authenticating the provider or independently verifying the source evidence. J3
preserves the provider-profile and source-evidence digests alongside the receipt digest; digest presence
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
