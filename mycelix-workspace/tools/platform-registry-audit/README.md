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
