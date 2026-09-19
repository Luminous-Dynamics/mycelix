# QCAP-001A3B — Executed-byte snapshot and Python import closure

QCAP-001A3B is a narrow successor to frozen QCAP-001A3A. It strengthens the reference Python adapter without changing capsule, limits, execution-context, receipt, or interoperability-vector formats.

## Claim

For the reference qcap3 adapter:

- each gate is read once from a non-symlink canonical artifact, verified against its registered SHA-256, copied into a fresh private per-gate snapshot, reverified, and executed from that snapshot rather than the mutable canonical pathname;
- mutation of a later canonical gate is detected before that gate executes;
- the command-line adapter requires Python isolated/no-bytecode mode (`-I -B`) before local qcap3 modules are imported;
- sibling standard-library shadows and local `.pyc` caches are rejected by the reference bootstrap;
- the committed qcap3 source-set commitment is recomputed after an attempt, and no receipt is returned if those runner sources changed during execution.

## Nonclaims

This tranche does not establish OS/container sandbox completeness, authenticated repository origin, hermetic system-library provenance, interpreter-binary provenance, process privilege separation, or QCAP framework qualification. A3A resource/output bounds and all historical A1/A2/A3A schemas/vectors remain unchanged.
