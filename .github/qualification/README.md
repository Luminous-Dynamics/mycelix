# Qualification preflight

This directory contains **non-evidentiary local admission checks** for exact qualification subjects.

The purpose is narrow:

```text
frozen exact subject
+ deterministic local prerequisite
-> eligible to request scarce hosted qualification
```

It does **not** establish qualification PASS.

```text
preflight ELIGIBLE
!= qualification PASS
!= theorem PASS
!= scientific evidence
!= merge/deployment authority
```

A real qualification attempt must rerun every registered theorem predicate from scratch.

## V1 scope

V1 is intentionally limited to one deterministic prerequisite: a pinned Rust `rustfmt --check` gate.

Profiles are **data, not command authority**. They cannot provide arbitrary argv. The engine constructs the only admitted execution sequence:

```text
rustup toolchain list
rustup run <pinned-toolchain> cargo --version
rustup run <pinned-toolchain> rustfmt --version
rustup run <pinned-toolchain> cargo fmt \
  --manifest-path <absolute isolated subject manifest> \
  -- --check
```

The formatting command runs from outside the subject checkout with an isolated `HOME`, `CARGO_HOME`, XDG directories, and temp directory. Cargo is forced offline. Ambient `CARGO_*`, `RUST*`, and Git-config injection variables are removed before execution.

Broader compile/test/Clippy preflight is explicitly deferred to a separately reviewed successor with a stronger execution-containment model.

## Exact-subject binding

Before a gate can execute, V1 requires:

- lowercase 40-hex SHA-1 subject identity;
- exact repository identity from `origin`;
- SHA-1 Git object format;
- exact single parent from the profile;
- exact changed-path set using `--no-ext-diff`, `--no-textconv`, `--no-renames`, `--name-only`, and NUL delimiters;
- canonical, closed JSON profile fields;
- pinned Rust toolchain version;
- canonical repository-relative manifest path.

Branch names and PR heads are not accepted as subject identity.

## Isolated materialization

The subject is materialized into a disposable local clone using:

```text
--local
--no-hardlinks
--no-checkout
```

V1 rejects an unexpected Git object-alternates file.

After checkout, every ordinary tracked file/symlink is checked against its indexed Git blob identity using the repository's SHA-1 object rule:

```text
SHA1("blob " + byte_length + NUL + exact_bytes)
```

Gitlinks may remain uninitialized, matching normal checkout behavior.

The exact materialization receives a domain-separated SHA-256 diagnostic commitment.

## Mutation detection

The isolated subject must be clean before execution and remain unchanged afterward.

V1 rejects:

- tracked file mutation;
- non-ignored untracked output;
- ignored-file output;
- tracked materialization/blob drift.

The caller's HEAD and porcelain status (including ignored matches) are snapshotted and must remain unchanged.

## Environment boundary

The preflight is deliberately stricter than invoking `cargo fmt` in the developer checkout.

It:

- runs the formatter from a directory outside the subject;
- isolates `HOME`, `CARGO_HOME`, XDG config/cache, and `TMPDIR`;
- removes inherited `CARGO_*` and `RUST*` variables before adding the small registered environment;
- removes Git config-injection/environment overrides;
- sets `CARGO_NET_OFFLINE=true`;
- disables interactive Git prompting;
- disables system/global Git config for preflight-owned Git commands;
- records executable/tool probes as diagnostic hashes.

This reduces ambient configuration and subject-local Cargo alias/config influence. It is not a hostile-code sandbox or machine attestation mechanism.

## Result vocabulary

V1 emits one of:

```text
ELIGIBLE
NOT_ELIGIBLE
UNAVAILABLE
INVALID
```

Every output carries:

```text
qualification_result = null
qualification_authority = false
```

Meaning:

- `ELIGIBLE`: the registered deterministic preflight gate passed; a real qualifier may be requested.
- `NOT_ELIGIBLE`: the deterministic gate ran or detected mutation and the subject should be repaired/refrozen before qualification.
- `UNAVAILABLE`: the required local toolchain/executable could not be established; no product conclusion follows.
- `INVALID`: profile/identity/ancestry/scope/materialization invariants were not satisfied.

## Commitments

The engine emits two distinct commitments:

```text
profile_commitment
preflight_implementation_commitment
```

The profile commitment binds canonical profile data. The implementation commitment binds the exact `preflight.py` bytes under a separate domain separator.

Neither commitment creates qualification authority.

## AMSAP-004A regression profile

`profiles/amsap-004a-rustfmt-v1.json` captures the exact parent/review surface and Rust 1.98.1 formatting prerequisite that would have rejected historical formatter-defective AMSAP-004A subject:

```text
8c2527db15aec1506f36aec70cb2802a9fdc0b9e
```

before registering its hosted qualifier.

The formatter-repaired replacement is:

```text
4fa9a8597152f404ec922afdabd113fd6d305e49
```

Those exact subjects must actually be executed with the pinned local toolchain before their preflight classifications are claimed. The existence of this profile alone proves neither result.

## Framework tests

Run:

```text
python3 .github/qualification/test_preflight.py
```

The synthetic suite covers exact-subject stability, caller-checkout preservation, tracked/ignored mutation, missing/wrong toolchain states, closed profile fields, duplicate JSON keys, path/parent mismatch, command-authority rejection, environment scrubbing, and no-object-alternates clone isolation.

Framework test PASS is not product qualification evidence.
