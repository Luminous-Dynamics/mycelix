# REGEN-008A — Cross-Step Toolchain Binding v1

Status: preregistration / assurance refinement. Parent semantics: REGEN-008 / #972.

## Trigger

REGEN-019C preparation exposed a class of evidence error that can survive a green workflow result:

```text
toolchain installed
!= toolchain selected
```

and:

```text
toolchain selected in step A
!= toolchain selected in step B
```

The first REGEN-019C preparation run installed Rust/Cargo 1.96.0 and invoked `rustup override set`, but later workflow steps resolved the runner-default Rust/Cargo 1.98.1. The tests and Clippy passed, yet the generated preparation receipt correctly revealed the mismatch. That artifact is therefore not admissible as a 1.96 preparation lineage.

This document generalizes that finding so later REGEN preparation and qualification workflows do not rely on ambient cross-step toolchain state without re-proving it.

## Core theorem

For every material build/preparation/qualification stage:

```text
intended toolchain
+ installed toolchain
+ earlier-step selection
!= toolchain actually used by this command
```

The toolchain identity attached to evidence must describe the commands that actually produced or tested the subject, not merely a toolchain installed somewhere in the job.

## Material toolchain stages

A workflow must treat at least these as potentially material:

- formatting/source normalization;
- dependency resolution / lock generation;
- compilation;
- tests;
- linting/static analysis;
- code generation;
- documentation generation when bytes are evidence-bearing;
- artifact/package production;
- receipt generation when toolchain identity is included in the receipt.

If different toolchains are intentionally used for different stages, that difference must be explicit rather than collapsed into one `toolchain=<x>` statement.

## Preferred explicit selection

For Rust workflows using rustup proxies, a strong default is explicit per-command selection:

```text
rustc +1.96.0 ...
cargo +1.96.0 ...
rustfmt +1.96.0 ...
cargo +1.96.0 fmt ...
cargo +1.96.0 generate-lockfile ...
cargo +1.96.0 test --locked ...
cargo +1.96.0 clippy --locked ...
```

Equivalent exact selection mechanisms are allowed, but the workflow must prove the selected identity at the stage that matters.

An ambient default, shell profile, PATH order, `rust-toolchain.toml`, `rustup override`, environment variable, container image, Nix shell, or action configuration may be a valid selection mechanism only when its effect on the material commands is explicitly verified.

## Cross-step persistence is not assumed

Workflow engines may execute steps in distinct shells and may alter environment/tool resolution between steps. Therefore:

```text
verified in setup step
!= verified in test step
```

A setup step may install tooling, but later material steps must either:

1. select the exact toolchain directly in the command; or
2. execute an immediately adjacent identity check proving the command-resolution state they depend on.

## Preparation-byte rule

The toolchain used to create evidence-bearing bytes must be bound separately from the toolchain later used to test them.

For example:

```text
rustfmt identity
-> formatted source bytes

Cargo identity
-> Cargo.lock bytes

compiler identity
-> compiled/tested product
```

A later successful test under the intended toolchain does not retroactively prove that an earlier formatter or lock resolver used that same toolchain.

## Receipt rule

A receipt must record the tool identity using the same explicit selection mechanism as the material command whenever practical.

A receipt whose recorded toolchain differs from the intended campaign profile makes the campaign indeterminate or failed for that intended profile, even if all workflow steps otherwise conclude successfully.

```text
workflow conclusion = success
+ toolchain receipt mismatch
!= profile PASS
```

## Promotion rule

Preparation artifacts may be promoted into ProductFrozen subjects only when the preparation evidence proves the exact toolchain identities required by the intended profile.

A wrong-toolchain artifact is not repaired by relabeling it. It requires a new preparation subject/run or an explicitly different profile.

Historical wrong-toolchain runs remain useful evidence and should not be deleted or rewritten into PASSes.

## Dependency interaction

REGEN-008's dependency-state theorem remains unchanged:

```text
ProductFrozen dependency graph
!= hermetic system closure
```

REGEN-008A adds another orthogonal axis:

```text
ProductFrozen dependency graph
!= exact toolchain selection
```

A mature receipt therefore binds at least:

```text
subject identity
dependency identity
toolchain identity per material stage
target identity
feature/profile identity
system-closure classification
result
```

## Nix / container profiles

A Nix-frozen or container-digest-frozen environment may reduce ambiguity, but the claim remains scoped:

- exact environment reference != command used that environment;
- container digest != complete host/kernel/hardware closure;
- Nix derivation identity != external mutable service identity;
- environment closure != scientific validity.

Qualification should record the mechanism actually used and avoid upgrading it into a stronger claim.

## Minimum adversarial controls

Future qualification-framework tests should include at least:

1. intended toolchain installed but default toolchain different;
2. setup-step selection lost in later step;
3. formatter uses wrong toolchain while tests use right toolchain;
4. lock resolver uses wrong Cargo while compiler uses right Rust;
5. receipt detects mismatch and refuses PASS;
6. explicit per-command selection produces intended identity;
7. mixed intentional toolchains remain visibly distinct rather than normalized away.

## Deliberate non-claims

REGEN-008A does not establish toolchain trustworthiness, compiler correctness, reproducible binaries, dependency safety, native/system hermeticity, supply-chain security, scientific correctness, regulatory validity, governance authority, or physical-action authority.

It establishes only the evidence rule that a claimed qualification toolchain must be proven at the commands that materially create or evaluate the qualified subject.
