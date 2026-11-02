# Construction Provenance Core v0.1

A dependency-free Rust reference verifier for one narrow Mycelix responsibility in
the infrastructure lifecycle: checking that construction records remain bound to
the exact design revision, simulation artifact, externally resolved authority,
material evidence, custody chain and inspection record they claim to describe.

This crate is deliberately standalone under `mycelix-workspace/standalone`. It
does not add a Holochain zome, replace Mycelix's identity/evidence/provenance
owners, or create a second project ledger.

## Run

```sh
cargo fmt --manifest-path mycelix-workspace/standalone/construction-provenance-core/Cargo.toml -- --check
cargo test --locked --manifest-path mycelix-workspace/standalone/construction-provenance-core/Cargo.toml
cargo clippy --locked --manifest-path mycelix-workspace/standalone/construction-provenance-core/Cargo.toml --all-targets -- -D warnings
python3 mycelix-workspace/standalone/construction-provenance-core/tests/validate_fixture.py
```

## First executable contract

The trace keeps these subjects distinct:

- design revisions and explicit supersession, each bound to an opaque source-system artifact reference and digest;
- simulation runs bound to an opaque execution-system run reference and result digest;
- an optional Sol-Atlas Foundry alternative reference, without copying its GIS/portfolio state;
- simulation runs bound to exact design revision IDs and design digests;
- approvals bound to exact design, simulation, signer, role, authority reference,
  authority scope reference, event time and supporting evidence references;
- external authority assertions with explicit status, scope and time bounds;
- supplier assertions kept separate from independently resolved certification evidence;
- material batches and a monotonic, predecessor-linked custody chain;
- inspection records bound to the current design, material batch, terminal custody
  receipt, inspector authority and inspection evidence.

Evidence and authority records carry both domain-valid time and
`available_at`. An artifact that became available after a decision cannot be
used to justify that earlier decision. Revoked evidence is rejected, disputed
evidence remains disputed, and missing/unknown/stale evidence remains
indeterminate.

The Rust verifier emits per-subject findings. It deliberately does not collapse
them into a project-wide `ready`, `safe`, `compliant` or `approved` boolean.

## Ownership and trust boundary

Mycelix remains the authority for identity, externally authenticated authority
resolution, evidence/provenance references, participant commitments and
correction/revocation history. This crate consumes resolved assertions and checks
their exact bindings. Cross-system artifact pointers bind owner, role, revision and digest but do not authenticate source bytes; it does not verify signatures, decide legal authority,
adjudicate disputes, or operate a Holochain network.

Symtropy remains the owner of physical/resource scenario execution and engineering
checks. Symthaea may propose design and manufacturing hypotheses but does not
self-certify them. Sol-Atlas remains the owner of spatial evidence and alternative
portfolio comparison. This crate is an opt-in coordination seam between those
owners, not a substitute for them.

## Claim ceiling

A finding of `Accepted` means only that this narrow record and its referenced
bindings pass the v0.1 contract. It does not establish:

- that a supplier assertion is true;
- that a certificate issuer is competent or a certificate technically sound;
- that a digital signature or signer identity has been cryptographically verified;
- that a design, simulation, material, robot, structure or installation is safe;
- that an authority has issued a permit or statutory approval;
- that an inspection is professionally sufficient or that an asset is fit for service.

The external authority/evidence resolver, byte store, engineering specialists,
inspectors and competent public authorities remain independently accountable.

## Wire format

`fixtures/construction-project-trace-v0.1.schema.json` defines the versioned
JSON wire contract. The synthetic pavilion fixture is intentionally aggregate
and contains pseudonymous party references only. The standard-library-only
fixture checker validates the schema's supported keyword subset and the
fixture's cross-record relationships; it is test tooling, not a production
Python dependency or a complete general-purpose JSON Schema implementation.
