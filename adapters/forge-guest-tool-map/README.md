# FORGE-004D3B2C2A — Guest Tool Map

This crate freezes the exact in-sandbox executable mapping used by the Forge M0 guest.

The guest must not resolve tools through `$PATH`, scan `/nix/store` by package name, or accept host-generated inner verifier argv. `GuestToolMapV1` instead binds the exact guest plan, execution subject, and Nix closure to exactly three inner executables:

- `forge-isolation-probe`
- `git`
- `gittuf`

Each binding carries the absolute `/nix/store/.../bin/...` executable path and the exact `ToolArtifact` metadata already committed by the enclosing execution specification: semantic version, artifact digest, byte size, and optional derivation commitment.

`qualify_guest_tool_map` re-derives the guest-plan digest and Nix-closure digest, requires the same execution subject as both the plan and execution spec, checks every executable is under an exact committed store root, and requires every artifact field to match the corresponding execution-spec tool entry.

The map is deliberately separate from the guest plan. This keeps the subject graph acyclic: the plan is already an input to the execution spec, while this map may bind both the plan and selected tool artifacts and can itself become another exact read-only execution input in the following composition refinement.

## Non-claims

This tranche does not execute tools, prove the store contents, launch bubblewrap, qualify the guest transcript, or mint `OfflineEvidence`. NAR content qualification remains in FORGE-004D2B3; the concrete guest runner must consume only a qualified tool map.
