# QCAP-001A2 execution/provenance hardening

Tracked by `#1657`.

This tranche is a direct successor of frozen QCAP-001A `4ab6084cbf6d11bb58ea7917b2006fb180a20908`.

Implemented product properties:

- explicit `GateNotRun` suffix semantics after the first runner failure;
- receipt-verifier enforcement of the closed attempt automaton;
- immediate pre-execution gate-script SHA-256 revalidation;
- resolved runner commitment bound in execution-context v2 and verified against dispatcher bytes;
- fresh exact-subject worktree per gate;
- tracked/non-ignored untracked mutation isolation;
- Linux process-group timeout termination plus subreaper descendant reaping;
- closed inherited gate environment;
- configured GitHub-origin consistency check when a recognizable `origin` is present;
- independent v2 PASS / theorem-FAIL / infrastructure+not-run canonical vectors.

Local product tests are not qualification evidence. A separate qualification theorem is still required before QCAP-001A2 is promoted from staged candidate status.
