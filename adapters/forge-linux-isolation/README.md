# mycelix-forge-linux-isolation

FORGE-004D2A freezes the first concrete Linux isolation policy used by Mycelix Forge verification.

It uses bubblewrap as a low-level namespace/mount constructor and Nix closure metadata as the exact tool/dependency boundary. This tranche defines the policy and deterministic command construction only; FORGE-004D2B must still run an in-sandbox probe and qualify that the host kernel actually enforced the requested isolation.

## v1 policy

The strict policy requires:

- explicit user namespace (failure is fatal, not `--unshare-user-try`);
- IPC, PID, network, and UTS namespaces;
- nested user namespaces disabled;
- new terminal session;
- child killed with parent;
- all Linux capabilities dropped;
- inherited environment cleared;
- private `/proc`;
- minimal `/dev`;
- hidden host home directory;
- ephemeral `/tmp` and `/work`;
- no `/sys`, `/run`, D-Bus socket, SSH agent, or host home mounts;
- only the exact committed Nix store closure mounted read-only;
- evidence/trust artifacts mounted read-only under `/inputs` or `/trust`.

The network namespace contains only its private loopback context; there is no host or external network attachment.

## Exact invocation

`VerifierInvocation` commits to the exact executable Nix-store path and argv. No shell is used to construct or execute the verifier command.

This closes a gap that tool/input digests alone do not close: the same qualified binary invoked with different flags is a different verification subject.

## Nix closure

`NixClosureManifest` commits to the exact set of Nix store paths and each path's NAR hash. The command builder mounts each closure entry individually at its original store path and intentionally never bind-mounts the entire host `/nix/store`.

FORGE-004D2B must independently re-derive and check the closure/NAR hashes before execution.

## Bubblewrap runtime

The command builder emits an argv vector rather than shell text. Its baseline includes explicit `--unshare-*`, `--disable-userns`, `--new-session`, `--die-with-parent`, `--cap-drop ALL`, and `--clearenv` controls, then reconstructs only the required filesystem view.

Environment is rebuilt from a tiny fixed baseline including `GIT_NO_LAZY_FETCH=1`, noninteractive Git, and disabled gittuf developer/debug modes.

## Not yet a proof

Correct command construction is not evidence that the current kernel supports or enforced every isolation feature. Bubblewrap itself documents that the strength of the sandbox depends on the exact invocation and platform behavior.

FORGE-004D2B therefore remains mandatory. It will bind runtime namespace IDs, capability state, mount state, network probes, artifact hashes, and sandbox-runtime identity back to this exact policy commitment.

## Sigstore boundary

This policy does not solve Sigstore trusted-root closure by itself. The first qualified verifier profile is local-key oriented. Sigstore becomes a separate trust profile after gittuf can consume explicitly pinned trusted material for both Git-object and DSSE verification without ambient TUF refreshes.
