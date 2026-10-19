# REA adoption pilot: evidence-first reverse engineering

**Status:** research proposal; no REA execution or zome inspection has yet been claimed  
**Research date:** 2026-10-13  
**Initial target:** Mycelix Energy Holochain zome WASM artifacts  
**Scope:** local/static investigation first; no changes to zome behavior, CI gates, or runtime authority

## Decision

Adopt [REA](https://rea.tools/) as an optional reverse-engineering workbench, not as a Rust dependency and not as an independent security authority. REA is a local CLI/MCP application whose current supported workflows span native binaries, JavaScript/Electron, managed assemblies, browser/network observations, packaged artifacts, and—on current upstream main—offline WASM inspection.

The narrow Rust integration should come later: only after we have one real REA evidence bundle, a known schema/version, and a demonstrably useful consumer. Do not build a second decompiler, introduce REA into every default Rust build, or let agent-generated interpretations pass a verifier without independent checks.

## Repository investigation

The [Mycelix repository](https://github.com/Luminous-Dynamics/mycelix) already has a Holochain workspace and `mycelix-energy` crate. The root README identifies Energy as a backend/research-stage cluster, and the workspace `justfile` includes `build-energy` as a WASM-targeted build path. This gives us an owned, domain-relevant target that can eventually connect evidence to Sol-Atlas and Symthaea's engineering/digital-twin work.

REA's [offline WASM guide](https://github.com/morluto/rea/blob/main/docs/wasm-artifacts.md) documents `inspect-wasm-artifact`, which runs caller-supplied WABT 1.0.42 against a local file and reports validation, module sections, imports/exports, decoded WAT, and tool/artifact identity. It does not execute the WASM or establish runtime behavior.

### Release gate discovered

Do not assume every feature on upstream `main` exists in the published npm package.

- The npm release available during this review was `rea-agents@6.4.0` (published 2026-10-11). Its tagged README, CLI command-name catalog, and WASM source paths were checked; the tag does not expose the documented `inspect-wasm-artifact` feature.
- Current upstream `main` contains the WASM implementation and documentation (including `src/wasm/WabtArtifactProvider.ts` and `docs/wasm-artifacts.md`), observed at commit [`8fc2043eb8bcedbf34dce064f977e455438635d6`](https://github.com/morluto/rea/commit/8fc2043eb8bcedbf34dce064f977e455438635d6).
- The WASM guide identifies WABT 1.0.42 as a caller-supplied requirement. It says the real-tool profile has been exercised on macOS arm64 and other hosts require their own verification.

Therefore, no CI job may invoke `inspect-wasm-artifact` through `@latest` or presume it exists in 6.4.0. First pin an upstream release that advertises the command, or run a separately reviewed, commit-pinned source build and validate its capabilities on our host. A tool merely existing on `main` is not a releasable dependency contract.

## Pilot phases

### P0 — verify availability before analysis

1. In an isolated development environment, install/use a pinned REA version and run its read-only doctor/capability checks.
2. Verify the CLI or MCP tool catalog explicitly advertises the selected operation. Treat a missing tool, unsupported host, missing WABT executable, or version mismatch as **capability unavailable**, not as a clean audit.
3. Record the REA package/version or source commit, host OS/architecture, Node version, WABT version, executable paths/digests where available, and exact command arguments.
4. Keep analysis targets read-only. REA documents that this is not an OS sandbox; use only owned/authorized artifacts and isolate any provider processing of untrusted inputs.

### P1 — inspect one Energy zome artifact

After the release gate passes:

1. Build Mycelix Energy using the repository's established build path and identify one exact `.wasm` emitted by that build. Do not guess an artifact filename; select it from the build output and record its path.
2. Record Git HEAD, dirty-tree status, relevant lockfile digest, target/profile/features, artifact byte length, and an independent SHA-256 of the exact selected bytes.
3. Run REA's offline WASM inspection against that exact file using the documented caller-supplied WABT profile. Preserve the complete JSON result locally as an evidence artifact; retain only a bounded summary in public logs if the full WAT or metadata is unnecessarily large.
4. Independently recheck the input digest and run the repository's existing build/tests. For an ABI contract, compare the reported imports/exports with an explicit, reviewed expectation; do not infer compatibility merely from successful WASM validation.
5. Save the result, limitations, tool identities, and reproduction command together. Every report must state that WAT is a decoded representation, not original source, and static inspection does not prove runtime behavior.

### P2 — evaluate value before building a bridge

The pilot succeeds only if it reveals a concrete discrepancy or yields a reusable contract check—for example, an unexpected host import/export, a change in a reviewed interface, or a reproducible distinction between two selected builds. A passing inspection with no useful downstream check is not sufficient reason to add permanent CI.

If successful, define a small, versioned Rust import boundary for the subset actually needed:
- preserve the exact subject artifact digest and source-tool identity;
- separate observed tool output from derived/inferred relationships;
- retain tool version/profile, incomplete coverage, unknown values, and redaction state;
- fail closed on malformed envelopes, digest mismatch, or missing required provenance;
- never convert "validated WASM" or "imports/exports match" into a claim that the zome is secure, semantically equivalent, or correct.

The shared deterministic provenance graph in Symthaea's engineering layer can be a downstream projection, but the imported evidence record must remain the source record; consumers must not recreate its topology or erase unknowns.

## What this pilot will not do

- It will not execute analyzed WASM.
- It will not grant REA, Symthaea, or a language model authority to approve its own findings.
- It will not block merges or change deployment authority in the first iteration.
- It will not add unpinned npm dependencies, download arbitrary tools during ordinary Rust builds, or run analysis on untrusted pull-request artifacts with credentials available.
- It will not claim completeness from a truncated report, omitted field, or unsupported host profile.

## References

- [REA website](https://rea.tools/)
- [REA repository and published CLI/MCP workflow](https://github.com/morluto/rea)
- [REA CLI/Evidence guide](https://github.com/morluto/rea/blob/main/docs/cli.md)
- [REA offline WASM guide (upstream main)](https://github.com/morluto/rea/blob/main/docs/wasm-artifacts.md)
- [Mycelix build recipes](https://github.com/Luminous-Dynamics/mycelix/blob/main/mycelix-workspace/justfile)
- [Holochain zomes](https://developer.holochain.org/build/zomes/): zomes are WASM modules invoked by the Holochain conductor.

## Current evidence status

| Item | Status |
|---|---|
| REA website/repository/docs reviewed | Yes |
| Published 6.4.0 checked for WASM CLI support | Yes; feature not advertised in the checked tag |
| Upstream-main WASM route identified | Yes; host profile still requires verification |
| Mycelix Energy build path identified | Yes |
| Actual zome artifact selected and inspected | Not yet |
| Rust bridge implemented | No |
| CI or runtime authority changed | No |
