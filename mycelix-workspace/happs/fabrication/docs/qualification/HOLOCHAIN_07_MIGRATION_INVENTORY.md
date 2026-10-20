# Fabrication Holochain 0.6 → 0.7 migration inventory

This inventory is planning evidence for #4246. It does not claim that any migration step is complete.

## Upstream breaking changes

Holochain's official 0.6→0.7 guide identifies the rewritten Action model as the bulk of the upgrade work. The 0.7 release also removes tx5/WebRTC in favor of Iroh/QUIC and has no automatic database migration path from 0.6.

## Repository-local impact

### 1. Workspace manifests

Update the Fabrication workspace dependency family together:

- `hdk 0.6.1 → 0.7.0`
- `hdi 0.7.1 → 0.8.0`
- `holochain_zome_types 0.6.1 → 0.7.0`
- `holochain_integrity_types 0.6.1 → 0.7.0`
- `holo_hash 0.6.1 → 0.7.0`
- `hdk_derive 0.6.1 → 0.7.0`

Sweettest must move with the application workspace rather than remaining on the old host runtime.

### 2. Integrity validation callbacks

Audit every validator for 0.6-era Action access patterns. Existing verifier code relies on action-derived predicates such as action type, entry type, author, signer, timestamp, action sequence, previous action, and entry hash.

Under 0.7 these must be re-bound to the new `header + data` Action model without changing the security predicate.

### 3. Coordinator lifecycle/action handling

Audit every coordinator path that consumes action callbacks or source-chain action metadata. Preserve the distinction between the exact Create/Update/Delete action and the referenced entry content.

### 4. Fabrication verifier trust chain

Re-run the security-critical anchors after the port:

`registration anchor → acquisition root → verification-key trust anchor → verifier-implementation trust anchor → challenge → EAT/COSE verification → source-attestation result`

The Holochain 0.7 port is a substrate migration, not permission to weaken any of these equality, provenance, or fail-closed checks.

### 5. Packaging and runtime

Rebuild all 14 WASM zomes, package the DNA and hApp from the generated artifacts, and execute the previously excluded Sweettest suite against a real 0.7 conductor/test runtime.

Qualification must bind:

- exact source commit;
- exact 0.7 dependency/toolchain versions;
- exact WASM artifact digests;
- exact DNA/hApp package digests;
- exact conductor/runtime version;
- test corpus and result identity.

### 6. Data continuity

Holochain's upgrade guide states there is no automatic 0.6→0.7 database migration path. Existing 0.6 installs therefore cannot simply be treated as upgraded in place. Any migration/deployment plan must explicitly choose a new install/network posture and preserve old-state handling as a separate policy.

## Security invariants during migration

Do not trade security properties for compatibility fixes.

`exact ActionHash ≠ EntryHash`
`DNA hash ≠ coordinator implementation identity`
`approved artifact ≠ executed artifact`
`verified provenance ≠ runtime execution`
`runtime observation ≠ race-safe invocation`
`source-level tests ≠ conductor qualification`
`queued CI ≠ pass`

## Exit gates

Do not close #4246 until the dependency/API port, 14-zome WASM rebuild, packaging, 0.7 conductor execution, and exact-head evidence are all demonstrated.

Related: #4483, #4371, #4248, #4436, #4437.