# Forge Hermetic Host

FORGE-004D3B2D2C is the concrete host-side orchestrator for one M0 hermetic repository-verification run.

It intentionally does not implement repository policy or evidence semantics again. Instead it composes the existing qualified layers:

1. re-derive the guest tool map against the exact execution subject and Nix closure;
2. audit and qualify the runtime NAR closure before launch;
3. snapshot every policy input through the sealed-memfd transport;
4. construct the canonical bubblewrap command and add `--json-status-fd` + `--block-fd` control channels;
5. launch bubblewrap with an empty host environment and piped stdout/stderr;
6. while the sandbox child is still blocked, collect parent namespace/mount evidence and acquire a pidfd for the exact reported child PID;
7. require that pidfd is not exit-signaled before release;
8. release the block FD and concurrently drain the single framed guest stdout evidence channel;
9. require bubblewrap success and require the same pidfd to become exit-signaled;
10. finish parent isolation evidence and independently re-qualify the raw guest evidence envelope;
11. combine parent + inside observations into qualified Linux isolation evidence;
12. independently re-audit the runtime NAR closure and require exact pre/post equality;
13. construct the seven ordered same-run phases, with `VerifierExecution` bound to the qualified guest transcript evidence rather than directly to the raw gittuf receipt;
14. qualify the same-run evidence and emit a host evidence commitment over the resulting qualified components.

## Trust boundary

The host launch manifest is a retrieval manifest, not authority. Runtime input paths are re-hashed and sealed against the isolation policy; typed plans/specifications are cross-linked again; the bubblewrap and host executable paths must be exact Nix-store executables in the committed closure.

The host requires a bubblewrap tool version of at least 0.10.0 because M0 input transport depends on the FD-based read-only bind primitive.

This tranche still does **not** mint repository `OfflineEvidence`. The final D3A refinement must consume `QualifiedHermeticHostRun`, bind the exact guest-tool-map JSON input/mount, replace the old direct verifier-receipt same-run cross-link with the qualified guest transcript relationship, bind sealed-input evidence, and resolve the host/NAR-auditor tool-role model before the repository capability is promoted.
