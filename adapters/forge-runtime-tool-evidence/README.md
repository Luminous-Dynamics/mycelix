# FORGE-004D3B2D2D — Runtime Tool Evidence

This crate closes the final executable-identity gap in the Forge M0 hermetic execution chain.

`ExecutionSpec::ToolArtifact` commits a role, semantic version, digest, byte size, and optional derivation. The Nix closure proves the filesystem tree containing a tool. Neither fact alone proves that the executable path actually used for a role contains the bytes committed by that `ToolArtifact`.

`audit_runtime_tools` therefore requires an exact role → executable-path map equal to the complete `ExecutionSpec` tool-role set and, for every role:

1. requires the named path to be an absolute `/nix/store/...` executable inside a committed closure root;
2. resolves symlinks and requires the resolved target to remain inside a committed closure root;
3. requires a regular file with at least one executable bit;
4. streams the actual resolved file bytes through the digest algorithm declared by `ToolArtifact`;
5. requires exact digest and byte-size equality with the specification;
6. rejects duplicate named or resolved executable paths;
7. emits a canonical evidence digest bound to the exact execution-spec digest and Nix-closure digest.

The M0 composer builds the path map from the concrete host receipt, the sandbox `VerifierInvocation`, and the qualified guest-tool map. The existing pre/post canonical-NAR theorem independently requires the containing store roots to remain identical across the hermetic run.

This evidence does **not** by itself prove sandbox isolation, repository verification, or tool correctness. It proves that every declared M0 tool artifact is tied to the actual executable bytes selected by the execution stack.
