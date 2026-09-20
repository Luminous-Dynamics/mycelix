# MYC-ZKP-GH-NIX-001R — NixOS Delegated Offline Verifier Profile v1

Status: **design contract / no execution or authentication authority**

Issue: #2319  
Parent containment contract: #2295 / `ZKP_RACE_SAFE_VERIFIER_CONTAINMENT_CONTRACT.md`

## 1. Purpose

This contract defines a NixOS/systemd deployment profile for the race-safe verifier containment theorem.

Its primary design decision is to separate network acquisition from verification:

```text
networked attestation fetcher
        ↓ retained bounded evidence
immutable / digest-bound handoff
        ↓
offline verifier service
        ↓
race-safe cgroup/pidfd containment
        ↓
strict verifier-result processing
```

The offline verifier must not silently regain network access or fetch new evidence.

## 2. Governing distinctions

```text
NixOS declarative service
!= cryptographic trust root

Delegate=yes
!= RaceSafeVerifierContainmentV1

Nix store path
!= qualified Nix closure identity

network fetch success
!= verifier success

offline verifier success
!= receipt authentication
```

## 3. Roles

### 3.1 Networked fetcher

The fetcher may contact GitHub/Sigstore endpoints to obtain:

- the attestation bundle;
- current trusted-root material;
- acquisition metadata required by the authentication policy.

It must not own the sealed verifier containment capability and must not mint authenticated-receipt objects.

### 3.2 Offline verifier

The verifier consumes only retained local inputs:

- canonical qualification receipt bytes and digest;
- attestation bundle bytes and digest;
- trusted-root bytes and digest;
- root-acquisition evidence;
- exact authentication policy;
- exact verifier executable identity;
- exact source/signer revision constraints.

It must run without network access under the delegated containment profile.

## 4. Typed plan split

The existing monolithic GitHub command profile should be replaced for this deployment by two typed plans:

```rust
pub struct GitHubAttestationFetchPlanV1 { /* private fields */ }
pub struct GitHubOfflineVerificationPlanV1 { /* private fields */ }
```

The fetch plan may contain network-capable commands.

The offline verification plan must contain no network acquisition commands and must fail construction if any argument would cause the verifier to fetch bundle/root material remotely.

## 5. Handoff object

The boundary between fetcher and verifier should use a typed retained-evidence object:

```rust
pub struct RetainedGitHubAttestationInputsV1 {
    // serializable evidence only
}
```

It should bind at least:

- canonical receipt digest/profile;
- bundle SHA-256 and length;
- trusted-root SHA-256 and length;
- root-acquisition profile;
- acquisition completion chronology/profile;
- repository/source/signer policy identity;
- expected predicate/profile identifiers;
- exact format/schema versions.

The handoff object remains evidence, not authority.

## 6. Handoff filesystem semantics

The preferred v1 handoff is one-way and immutable by profile:

1. fetcher writes into a private staging directory using create-new semantics;
2. all retained files are bounded before promotion;
3. each file is fsync/read-back checked;
4. a manifest binds the exact digests/lengths/profile IDs;
5. promotion to verifier input occurs atomically where the filesystem/profile permits;
6. verifier input is read-only to the verifier process;
7. verifier refuses pre-existing unmanifested files, symlinks, non-regular files, path traversal and digest mismatch;
8. verifier cannot ask the fetcher to mutate an admitted input in place.

A future content-addressed store may replace this layout only if it preserves the same theorem.

## 7. NixOS systemd unit profile

The offline verifier should be provisioned declaratively as a NixOS systemd service rather than launched through ambient `sudo` or `systemd-run`.

Candidate systemd service properties include:

```text
Delegate=yes
NoNewPrivileges=yes
PrivateNetwork=yes
PrivateTmp=yes
ProtectSystem=strict
ProtectHome=yes
ProtectKernelTunables=yes
ProtectKernelModules=yes
ProtectKernelLogs=yes
RestrictSUIDSGID=yes
LockPersonality=yes
```

`RestrictAddressFamilies=` should permit only the minimum local IPC families actually required by the verifier profile.

Resource-control limits should be explicit where compatible with cgroup delegation.

### Important exception

Do not set `ProtectControlGroups=yes` if it prevents the service from managing its explicitly delegated child cgroups.

The unit profile must record this as an intentional capability, not omit the discussion and claim generic maximum hardening.

## 8. Delegation theorem

systemd `Delegate=` permits an unprivileged service to manage a cgroup subhierarchy beneath its unit. That is only the starting condition.

The NixOS adapter must additionally prove:

```text
exact declarative unit profile
+ dedicated service identity
+ owned delegated subtree
+ no migration authority outside subtree
+ required cgroup-v2 control files available
+ #2295 host capability probe passes
    -> NixOSDelegatedVerifierHostV1
```

`NixOSDelegatedVerifierHostV1` remains non-authenticating.

## 9. Service identity

The implementation must choose and qualify one service-identity model rather than accepting several implicitly.

Candidate profiles:

### Fixed locked-down system user

Advantages:

- stable UID for delegation testing;
- straightforward ownership semantics;
- easier cross-run evidence correlation.

### DynamicUser

Advantages:

- reduced persistent identity/state;
- good default isolation properties.

Risks/requirements:

- delegation ownership and persisted handoff paths must be proven under dynamic UID allocation;
- no evidence identity may depend on a transient UID value unless explicitly profiled.

The selected model must be named in the profile ID.

## 10. Nix executable identity

The offline verifier should execute an exact binary from the Nix store.

Bind:

- store path;
- executable raw SHA-256;
- package/profile identity;
- target platform;
- command profile.

Do not yet bind a generic `nix_closure` claim unless a separate canonical Nix closure digest theorem has been specified and qualified.

```text
/nix/store/... path
!= canonical closure identity
```

## 11. Race-safe containment integration

Inside the delegated service cgroup, the verifier executor may create a fresh child cgroup for each verification step/run using the #2295 profile.

The service-level cgroup is the delegation boundary; per-verification child cgroups are the execution-containment boundaries.

```text
systemd service cgroup (delegated)
    ├── verifier-run-A
    ├── verifier-run-B
    └── verifier-run-C
```

One run must never receive write authority over a sibling run's containment object.

## 12. Network isolation theorem

The offline verifier service must establish:

```text
PrivateNetwork / equivalent admitted isolation
+ no inherited network namespace escape capability
+ offline-only command plan
+ retained local inputs
    -> OfflineVerificationNetworkBoundaryV1
```

This theorem should be tested by demonstrating that the verifier can successfully consume retained local evidence while outbound network access fails.

Network isolation alone does not prove result correctness.

## 13. Fetcher authority ceiling

The fetcher should machine-readably report that it can establish only acquisition evidence.

Suggested authority:

```text
AttestationAcquisitionOnly
```

It must return false for:

```text
establishes_verifier_execution()
establishes_receipt_authentication()
grants_production_authority()
grants_application_authority()
```

## 14. Offline verifier authority ceiling

The offline verifier's strongest pre-authentication result should combine:

- exact retained-input lineage;
- race-safe execution containment;
- actual offline verifier execution;
- strict parsed verifier result;

Suggested authority:

```text
OfflineVerifierExecutionOnly
```

It still must return false for receipt authentication until the existing authentication layers bind trusted builder, Public Good transparency, currentness and the final authenticated-receipt mint.

## 15. Clock/currentness interaction

`PrivateNetwork=yes` means the verifier cannot refresh root material or obtain network time during verification.

Therefore:

- trusted-root acquisition chronology belongs to the fetcher evidence;
- host-observed verifier chronology remains distinct;
- a trusted-clock theorem remains separate;
- offline verification must not reinterpret local wall clock as authenticated time.

## 16. Failure behavior

If the NixOS/systemd containment profile cannot be established, return a typed unsupported-profile error before running the verifier.

Examples:

- delegation absent;
- cgroup v2 unavailable;
- cgroup child creation denied;
- `cgroup.kill` unavailable;
- pidfd/clone3 capability unavailable;
- offline network isolation unavailable;
- exact Nix executable identity mismatch;
- handoff input is mutable/substituted;
- service unit profile differs from the qualified profile.

No failure may silently switch to a weaker process-group-only authority profile.

## 17. Declarative profile digest

A future `NixOSVerifierServiceProfileV1` should have a canonical digest over security-relevant unit semantics, including:

- service identity model;
- delegation setting;
- network isolation setting;
- system/filesystem protection settings;
- allowed address families;
- writable/read-only paths;
- capability bounding set / ambient capability policy;
- no-new-privileges setting;
- resource-limit profile;
- executable/store identity;
- runtime/state directory semantics;
- containment backend/profile ID.

Do not hash arbitrary generated unit text if semantically irrelevant ordering/formatting can change identity. Prefer a canonical typed policy object.

## 18. Qualification gates

### NIX-A — static module/unit profile

Prove the Nix evaluation renders the exact intended unit semantics and exact verifier package identity.

### NIX-B — delegation capability

On a qualified NixOS VM/host:

- service receives only its delegated subtree;
- service can create/remove per-run child cgroups;
- service cannot migrate verifier processes outside its delegated subtree;
- #2295 capability probe succeeds.

### NIX-C — offline isolation

Prove retained evidence verifies successfully while network access is unavailable from the verifier service.

### NIX-D — handoff integrity

Prove create-new/read-only/digest-bound retained evidence semantics and reject symlink/substitution/replay profile mismatches.

### NIX-E — adversarial containment

Prove descendants, timeout, stream overflow and teardown behave according to #2295 and that unrelated services/processes survive cleanup.

### NIX-F — reproducibility

Bind the qualification evidence to:

- NixOS revision / flake lock;
- exact module source;
- exact system closure/profile required for the test;
- kernel/systemd versions;
- exact verifier package and executable digest.

## 19. GitHub-hosted CI rule

GitHub-hosted CI must not be assumed to satisfy this NixOS profile.

If the hosted runner lacks a delegated writable cgroup subtree or exact NixOS/systemd service semantics, qualification must run in a purpose-built NixOS VM or qualified self-hosted environment.

A lower-assurance CI smoke test may still exist, but its evidence must use a different profile/authority name.

## 20. Integration with Spore/Nixward

This adapter is a natural future integration point with Nixward/Spore:

- Nixward can validate the declarative unit/store/profile invariants;
- Spore can provision/recover the exact service profile on supported NixOS hosts;
- neither should weaken the authentication theorem or silently mutate the verifier unit at runtime.

Those integrations are optional adapters, not prerequisites for the generic Mycelix authentication core.

## 21. Final theorem

```text
qualified NixOS service profile
+ delegated cgroup authority
+ immutable retained fetch inputs
+ exact Nix verifier executable
+ offline network boundary
+ RaceSafeVerifierContainmentV1
+ actual bounded verifier execution
    -> NixOSOfflineVerifierExecutionV1

NixOSOfflineVerifierExecutionV1
    != trusted clock
    != receipt authentication
    != production admission
    != application authority
```
