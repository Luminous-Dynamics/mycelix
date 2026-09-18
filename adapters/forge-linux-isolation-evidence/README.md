# mycelix-forge-linux-isolation-evidence

FORGE-004D2B1 defines the provider-facing evidence contract for observed Linux isolation.

The crate contains **no process launcher, `/proc` reader, socket probe, or bubblewrap integration**. It specifies what two independent observation channels must report and exactly when those reports are sufficient to qualify one observed isolation run.

## Two channels

The inside channel reports:

- exact 004D2A policy digest;
- user/mount/PID/IPC/network/UTS namespace identities;
- inheritable/permitted/effective/bounding/ambient capability masks;
- sandbox hostname;
- exact environment;
- visible `/nix/store` root names;
- mounted artifact role, destination, digest, byte size, regular-file identity, and write denial;
- writable ephemeral `/work` and `/tmp`;
- visible `/home` entries;
- `/sys` and `/run` visibility;
- mount-info and route-table commitments;
- non-loopback-route observation;
- outbound-connect observation;
- nested-user-namespace creation observation.

The parent channel independently reports host and child namespace identities, the child mount-info commitment, bubblewrap status commitment, and child exit code.

Qualification requires the inside namespace set to equal the parent-observed child set, and every required namespace to differ from the parent namespace set.

## Artifact identity

A failed write-open is not sufficient evidence that an arbitrary path is a read-only file. D2B1 therefore requires each committed artifact observation to match the 004D2A role, destination, digest, and byte size, to be a regular file, and to reject writes.

## Claim boundary

`QualifiedIsolationEvidence` proves that two structurally independent observation channels agree on one strict 004D2A isolation subject and satisfy the v1 evidence predicates. It does not prove that the observation producer is trustworthy or that the observations were collected from a real Linux kernel. FORGE-004D2B2 must implement and qualify that collector/launcher.

It also does not yet mint repository `OfflineEvidence`; the eventual composition must bind this isolation evidence to the exact FORGE-004D1 execution specification and a FORGE-004D2C verifier trust profile.
