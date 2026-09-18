# mycelix-forge-linux-isolation-collector

FORGE-004D2B2 implements concrete Linux observation collection for the pure FORGE-004D2B1 evidence contract.

It deliberately does **not** contain qualification logic. Collection answers “what did this process/parent observe?”; D2B1 answers “does that evidence satisfy the strict policy?”

## Inside channel

`collect_inside_evidence(...)` records:

- current user/mount/PID/IPC/network/UTS namespace identities;
- all Linux capability masks from `/proc/self/status`;
- hostname and complete environment;
- visible `/nix/store` roots;
- every committed artifact's actual digest, byte size, regular-file identity, and writeability;
- `/work` and `/tmp` write probes;
- `/home` entries and `/sys`/`/run` visibility;
- SHA-256 commitments to mountinfo and route table;
- presence of non-loopback routes;
- an outbound TCP connection attempt;
- nested user-namespace creation.

The nested-user-namespace probe runs last because an unexpected success mutates the probe process. The real verifier must run in a separate sandbox process.

## Parent channel

`observe_parent_start(...)` consumes bubblewrap JSON status records while the child is held behind `--block-fd`, obtains `child-pid`, and independently reads the host and child namespace links plus the child's mountinfo through `/proc/<pid>`.

`PendingParentObservation::finish(...)` then binds the final bubblewrap status stream and exit code into `ParentIsolationEvidence`.

## Artifact bytes

Mounted regular files are streamed and hashed using the digest suite committed by the corresponding 004D2A `ArtifactMount` (`sha256` or `blake3-256`). Byte count overflow fails closed.

## Remaining boundary

D2B2 does not launch bubblewrap and does not independently re-derive Nix NAR hashes. A launcher must wire `--json-status-fd` + `--block-fd` around the 004D2A command, and FORGE-004D2B3 will verify that each visible store path's actual NAR content matches the committed closure manifest.

The collector itself must also become a committed Nix tool before its observations may contribute to final `OfflineEvidence`.
