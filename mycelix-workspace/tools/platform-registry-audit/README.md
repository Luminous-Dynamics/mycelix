# Mycelix Platform Registry Audit

A Rust-first, manifest-derived verifier for the public Mycelix deployment registry.

## Why

The unified and resilience hApp manifests are the deployment inputs. The JSON registry is a non-authoritative index and must not drift from those manifests. This tool checks the relationship instead of relying on manually maintained role lists.

## Commands

From the repository root:

```bash
cargo run --manifest-path mycelix-workspace/tools/platform-registry-audit/Cargo.toml -- --check
cargo run --manifest-path mycelix-workspace/tools/platform-registry-audit/Cargo.toml -- --write
```

- `--check` is read-only and exits nonzero when role IDs, DNA paths, profile role membership, manifest Git blob hashes or source-commit provenance disagree.
- `--write` refreshes manifest-derived fields and Git provenance. Existing logical-domain annotations, trust classifications, authority classifications, notes and discrepancy records are preserved. New roles default to `unclassified`; the tool never infers trust or authority from names.
- `--root PATH` selects a repository checkout. The tool requires the source manifests to be committed and clean before generating or checking a snapshot.

## Invariants

- Role IDs are unique within a manifest.
- Every profile role exists in the unified registry.
- Registry role IDs and DNA artifact paths equal the unified manifest.
- Profile membership and manifest blob hashes match the exact source YAML.
- The recorded source commit contains the exact declared unified and resilience manifest blobs.
- Trust and authority classifications use explicit closed vocabularies.

The tool does not inspect DNA bundle bytes, infer application semantics, resolve every dynamic `OtherRole` call, or establish runtime qualification. Those are separate gates.

## Dependency note

Uses the maintained YAML organization fork `yaml_serde` rather than the archived `serde_yaml` crate. The tool is standalone and is not a Holochain production dependency.


## Advisory topology inventory

The tool also exposes an AST-based inventory mode:

```bash
cargo run --manifest-path mycelix-workspace/tools/platform-registry-audit/Cargo.toml -- --audit-otherrole > otherrole-topology-report.json
```

This scans Rust source with Syn, inventories `CallTargetCell::OtherRole(...)`, resolves string literals and unique simple string constants, and reports dynamic expressions as unresolved. It discovers hApp manifests and records their exact Git blob SHA when available.

Important interpretation limits:

- A nearest ancestor `happ.yaml` is only a **source context hint**. It does not prove the Rust source is included in one of that manifest's DNA bundles.
- A role name declared in the nearest manifest is reported as `nearest-manifest-declared`, not as a proven runtime-valid bridge.
- A matching role name in another manifest is only a candidate. It does not prove the caller DNA is the same artifact or is deployed in that profile.
- Dynamic role expressions and source roots with no nearest manifest remain unresolved.
- Macro-generated references and parse errors are reported separately; the inventory is advisory and must not be presented as complete source-to-DNA coverage.
- The report does not currently resolve every Rust source module into a compiled WASM zome or verify DNA byte identity.

This is intentionally not a global role-name allowlist. A later, independently qualified source-to-DNA ownership map is required before this inventory can become a hard deployment gate.


### Current advisory inventory status

The AST inventory currently has ten authored unit tests covering literal calls, unique string constants, dynamic expressions, nearest-manifest declarations, cross-profile name overlap, unresolved targets and unrelated functions named `OtherRole`.

The output deliberately uses classifications such as:

- `nearest-manifest-declared` — a context manifest contains the target name; this is **not** proof of runtime bridge validity.
- `profile-dependent-candidate` — caller-context role names and target names overlap in another manifest; this is only a candidate, not proof of DNA identity.
- `cross-profile-or-source-mapping-unresolved` — target exists elsewhere, but no sufficient role-name overlap establishes a candidate profile.
- `dynamic-unresolved`, `missing-role-candidate`, `source-context-unresolved`.

Every call-site record states `source_to_dna_identity: not-proven`. The report's manifest proximity coverage is only a context metric, never source-to-DNA ownership coverage.

CI publishes the JSON report as an artifact for review. The AST inventory is advisory and does not fail the deployment registry gate based on incomplete source ownership.
