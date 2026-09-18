# Mycelix Forge FORGE-004D2B1 — Two-Channel Isolation Evidence Contract

**Status:** implementation candidate  
**Depends on:** FORGE-004D2A canonical Linux isolation policy  
**Runtime collector:** deferred to FORGE-004D2B2

## Purpose

FORGE-004D2B1 separates the evidence theorem from the Linux mechanism that produces it. The protocol layer must not become a transcript of one bubblewrap launcher implementation.

The tranche therefore defines canonical inside-sandbox and parent-side observations plus a single qualification function. It performs no host I/O.

## Positive theorem

`qualify_linux_isolation(...) -> QualifiedIsolationEvidence` requires:

1. the exact 004D2A policy to re-bind to the supplied Nix closure and verifier invocation;
2. inside evidence to name the exact policy digest;
3. inside namespace identities to equal the independently parent-observed child identities;
4. user, mount, PID, IPC, network, and UTS namespaces to all differ from the parent;
5. inside and parent mount-info commitments to agree;
6. child exit code zero;
7. every Linux capability mask zero;
8. exact strict hostname/environment values;
9. visible Nix store roots exactly equal the committed closure roots;
10. every committed artifact to match role, destination, digest, size, regular-file type, and write denial;
11. `/work` and `/tmp` writable while host home state, `/sys`, and `/run` remain absent;
12. no non-loopback route, no successful outbound connection, and no successful nested user-namespace creation.

The final evidence commitment binds the exact policy, inside observation, and parent observation digests.

## Why artifact type is explicit

A directory opened with `OpenOptions::write(true)` can fail for reasons unrelated to mount read-only state. D2B1 therefore refuses to infer a read-only regular-file claim from write failure alone. The observation must separately establish regular-file type, byte digest, size, and write denial.

## Observation versus authority

A producer can still lie about observations. `QualifiedIsolationEvidence` means the supplied observations satisfy the contract; it does not mean their producer is trusted.

FORGE-004D2B2 must implement the collector using an active in-sandbox probe plus independent parent `/proc/<child-pid>` observations and bubblewrap status output. That collector must itself become an exact qualified tool before this evidence can contribute to `OfflineEvidence`.

## Remaining composition boundary

D2B1 binds the 004D2A isolation policy, while FORGE-004D1 separately binds the complete execution specification. The eventual M0 composition must require the D2A policy digest as an exact D1 input/tool-policy commitment; no implicit association is sufficient.
