# Forge Matrix Selection Oracle v1

Issue: CI-GOV-001C3 / #1786.

This directory freezes an **offline proposal oracle** for admitted Mycelix Forge CI.
It does not change `.github/workflows/forge.yml`, suppress any lane, cancel any run,
or establish Forge product qualification.

## Frozen evidence subject

The v1 profile was audited against Forge frontier:

`1af145717945c85bd07fb381c6b2ee4bd45015c8`

and workflow Git blob:

`9885d3a6079195591c3e6e297682f4b9ee9fb4b0`

with 22 named matrix lanes.

The profile also binds the exact Git SHA-1 blob ID of every one of the 22
`Cargo.toml` dependency manifests. Before the bound selector can propose a
subset, it re-hashes the checkout's exact workflow and manifest bytes using the
Git blob preimage format:

`sha1("blob " + decimal_length + NUL + bytes)`

Any mismatch yields `FullMatrix(frozen_source_binding_mismatch)`.

This makes the frozen dependency graph executable rather than merely documented.

## Selection rule

For known source changes, select the transitive **reverse dependency closure** of
the changed lanes using frozen Cargo dependencies plus reviewed semantic edges.

The first explicit semantic edge is:

`hermetic-host -> hermetic-guest`

because the host consumes the guest executable as a runtime artifact even though
Cargo does not link the guest crate into the host crate.

## Fail closed

The oracle returns the full frozen matrix when:

- `forge.yml` changes;
- this selector, profile, or tests change;
- any Forge `Cargo.toml` changes;
- an unknown Forge crate/adapter path appears;
- changed path syntax is non-canonical;
- the checked-out `forge.yml` blob differs from the frozen profile;
- any checked-out Forge `Cargo.toml` blob differs from the frozen profile.

Profile validation rejects unknown dependencies, duplicate lanes/roots,
unsupported Git object suites, malformed Git blob IDs, and dependency cycles.

An integration that cannot load or validate the profile must also fall back to
the full matrix. A selector process failure is never permission to skip CI.

## Trusted-oracle boundary

`SelectedMatrix` is only meaningful when the selector/profile implementation is
itself a previously reviewed/qualified policy subject.

A future PR must not be allowed to modify the oracle that decides which of its
own checks may be suppressed. Workflow integration therefore needs an
independent trusted-oracle theorem (for example, a previously qualified pinned
selector/profile or an equivalent base-branch policy path).

Changing selector/profile bytes must conservatively preserve the full matrix
until the new oracle subject is independently reviewed and qualified.

## Rollout

1. qualify the offline profile/selector/tests;
2. replay historical Forge changed-path sets;
3. integrate in **observation-only** mode while the full admitted matrix still runs;
4. compare proposed vs actual matrix;
5. enable pruning only under a separate qualified scheduling theorem.

`SelectedMatrix` is a proposal, not suppression authority.
