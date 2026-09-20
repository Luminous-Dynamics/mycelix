# MYC-ZKP-GH-CONTAIN-002R — Race-Safe Verifier Containment Contract v1

Status: **design contract / no implementation authority**

Issue: #2295  
Parent executor lineage: #2213  
Predecessor containment repair: #2279

## 1. Purpose

This contract defines the minimum theorem required before Mycelix may describe a GitHub/Sigstore verifier execution as living inside a **race-safe verifier containment domain**.

It exists because a fresh POSIX process group is useful containment, but its numeric PID/PGID is not a persistent kernel object identity after the original process/group disappears. Numeric reuse must therefore never become the basis of a stronger authentication theorem.

The preferred Linux profile combines:

```text
owned delegated cgroup-v2 subtree
+ retained cgroup control handles
+ atomic clone3(CLONE_INTO_CGROUP)
+ CLONE_PIDFD direct-child identity
+ bounded stdout/stderr + deadline
+ cgroup.kill descendant-tree teardown
+ cgroup.events populated=0 completion evidence
    -> RaceSafeVerifierContainmentV1
```

This still does **not** establish trusted time, verifier-result validity, receipt authentication, production admission, or application authority.

## 2. Governing distinctions

The following implications are forbidden:

```text
fresh process group
!= race-safe persistent containment identity

numeric PID/PGID
!= lifetime-stable process identity

pidfd direct-child identity
!= descendant containment

cgroup path string
!= owned cgroup kernel object

cgroup created
!= verifier atomically started inside it

cgroup.kill requested
!= containment empty

process exited
!= all descendants exited

cgroup.events populated=0
!= trusted verifier result

race-safe containment
!= receipt authentication
```

## 3. Threat model

This profile addresses failures or hostile behavior in the verifier process tree including:

- direct-child timeout;
- descendants retaining stdout/stderr after the direct child exits;
- descendants closing inherited pipes but remaining alive;
- concurrent forks during teardown;
- numeric PID/PGID reuse;
- accidental cleanup of unrelated sibling processes;
- verifier attempts to survive timeout by daemonizing inside the owned containment tree;
- stale or replaced cgroup path names;
- unsupported host cgroup capabilities being silently downgraded.

This profile does not by itself claim protection against arbitrary kernel compromise, privileged host administrators, unrestricted access to another writable delegated cgroup ancestor, filesystem/network abuse outside the containment theorem, or a verifier that has been granted cgroup-migration authority.

## 4. Required kernel profile

The first candidate profile is Linux x86_64 with:

1. unified cgroup v2 hierarchy;
2. an explicitly delegated writable parent subtree owned by the executor boundary;
3. `clone3(2)` support for `CLONE_INTO_CGROUP`;
4. `CLONE_PIDFD` support;
5. non-root child cgroups exposing `cgroup.kill`;
6. non-root child cgroups exposing `cgroup.events` with recursive `populated` semantics;
7. permission to create and remove one dedicated verifier containment cgroup;
8. permission to start the child directly inside that cgroup;
9. permission to kill the owned cgroup tree;
10. no verifier-side permission to migrate itself out of the owned containment domain.

A missing capability is an explicit unsupported-profile result. There is no authority-preserving fallback to PGID-only execution.

## 5. Containment identity

The authority-bearing containment object must not be reconstructible from a path string or serializable receipt alone.

Conceptually:

```rust
pub struct RaceSafeVerifierContainmentV1 {
    // private, process-local kernel-backed handles
}
```

The object should retain, directly or through an internal backend object:

- the delegated-parent identity/profile;
- the created child-cgroup identity;
- retained handles/fds for the child cgroup and required control files;
- the direct-child pidfd;
- the exact containment policy/profile version;
- the exact kernel/platform capability profile observed before execution;
- teardown state.

It must not implement `Serialize`, `Deserialize`, `Clone`, `Default`, or a public constructor capable of forging kernel ownership.

## 6. Cgroup path discipline

The cgroup pathname is presentation/debug evidence, not the lifetime identity.

Where practical, child objects should be opened relative to a retained delegated-parent/cgroup directory fd using strict directory-relative resolution. Reopening an arbitrary absolute path later must not silently substitute a newly created cgroup with the same textual name.

The executor must reject:

- symlinked containment paths;
- path traversal;
- unexpected filesystem type;
- unexpected cgroup version/profile;
- reopened path identity that no longer matches the retained containment object.

## 7. Atomic child placement

The direct verifier child must begin life inside the dedicated cgroup using `clone3(CLONE_INTO_CGROUP)` or a successor mechanism with an equivalent atomic theorem.

The following weaker sequence is insufficient for this profile:

```text
spawn child normally
then write child PID to cgroup.procs
```

because verifier code could execute before placement.

The spawn operation should also request `CLONE_PIDFD`, yielding a pidfd for the direct verifier process in the same creation transaction.

## 8. Direct-child identity

The direct child is identified by pidfd, not by a numeric PID alone.

The numeric PID may be retained for observability, but all authority-sensitive direct-child signaling/liveness semantics must use pidfd-capable primitives where supported by the implementation profile.

Required distinction:

```text
pidfd retained
-> direct process identity resistant to PID reuse

pidfd retained
!= descendant-tree containment
```

Descendant containment remains the cgroup theorem.

## 9. Escape prevention

The verifier must not receive write authority that lets it move itself or descendants outside the owned cgroup subtree.

The delegated hierarchy must therefore separate:

```text
executor control authority
from
verifier runtime authority
```

At minimum the verifier must not be handed writable handles/paths for an ancestor `cgroup.procs` or another destination cgroup that would permit escape.

If the host delegation model cannot establish this, the profile is unsupported.

## 10. Clean-success condition

A successful direct child exit is not sufficient.

Before the executor may seal successful containment evidence:

1. direct-child exit/reap must be observed through the admitted direct-process identity path;
2. both bounded stdout/stderr readers must complete within the step deadline;
3. `cgroup.events` for the owned containment tree must report `populated 0`;
4. no teardown failure may remain pending;
5. the exact containment handles must still refer to the owned domain being evaluated.

Therefore a daemonized/background descendant that closes stdout/stderr but remains in the cgroup prevents clean success.

## 11. Failure / timeout teardown

On timeout, bounded-stream overflow, process failure, parser-independent cancellation, or executor abort:

1. mark the step as teardown-required;
2. request direct-child termination through pidfd where appropriate;
3. write `1` to the retained `cgroup.kill` control for the owned containment tree;
4. continue bounded cleanup while observing the same retained cgroup identity;
5. require `cgroup.events` to reach `populated 0`;
6. reap/observe the direct child using the admitted direct-process identity path;
7. join all local pipe-reader tasks/threads;
8. release handles/remove the empty child cgroup only after teardown completion;
9. return the original typed execution failure only if teardown itself completed;
10. if teardown cannot prove an empty owned containment domain, return a stronger containment-teardown failure.

A cleanup failure must never be hidden behind the original verifier error.

## 12. Concurrent forks and migration

The design relies on cgroup-v2 `cgroup.kill` semantics for tree-wide teardown rather than enumerating numeric descendant PIDs.

Qualification must specifically exercise concurrent descendant creation during cleanup.

The theorem also depends on preventing verifier-controlled migration out of the containment subtree. This must be tested as a permission boundary, not assumed from application intent.

## 13. Capability probe

Before verifier execution, perform a fail-closed profile probe.

Conceptually:

```rust
pub enum RaceSafeContainmentCapabilityV1 {
    Supported(RaceSafeContainmentHostProfileV1),
    Unsupported(RaceSafeContainmentUnsupportedReasonV1),
}
```

Possible unsupported reasons should include at least:

- not Linux x86_64 for this profile;
- cgroup v2 unavailable;
- no delegated writable parent;
- child-cgroup creation denied;
- `cgroup.kill` unavailable;
- `cgroup.events` unavailable or malformed;
- `clone3` unavailable;
- `CLONE_INTO_CGROUP` unavailable/denied;
- `CLONE_PIDFD` unavailable/denied;
- unable to retain required control handles;
- verifier escape-prevention policy cannot be established.

No unsupported reason may silently select `ProcessGroupV1` for an authority-bearing caller.

## 14. Evidence model

Serializable evidence may describe what the containment backend observed, but it cannot recreate the process-local containment capability.

A future evidence type should bind at least:

- containment profile ID/version;
- kernel release / architecture profile;
- cgroup v2 mount/profile evidence;
- delegated-parent evidence identity;
- child-cgroup presentation identifier;
- capability-probe result;
- direct-child numeric PID for observability;
- pidfd-backed lifecycle result identifier/profile;
- execution start/finish chronology source;
- whether `cgroup.kill` was used;
- whether teardown reached `populated 0`;
- teardown duration/timeout profile;
- stdout/stderr limit profile;
- exact verifier executable identity;
- exact command/environment policy digests;
- final containment disposition.

The evidence object remains non-authoritative without the sealed process-local capability.

## 15. Suggested dispositions

```rust
pub enum RaceSafeContainmentDispositionV1 {
    CleanExitContainmentEmpty,
    FailedAndContainmentEmptied,
    TimedOutAndContainmentEmptied,
    StreamLimitAndContainmentEmptied,
    UnsupportedHostProfile,
    ContainmentAcquisitionFailed,
    DirectChildIdentityFailed,
    TeardownFailedContainmentNotProvenEmpty,
}
```

Names must describe what was established. Avoid generic `Safe`, `Verified`, or `Contained` booleans.

## 16. Authority ceiling

The strongest positive object from this layer should report an authority such as:

```text
RaceSafeProcessContainmentOnly
```

and machine-readably return false for:

```text
establishes_trusted_clock()
establishes_verifier_result_validity()
establishes_receipt_authentication()
grants_production_authority()
grants_application_authority()
```

## 17. Relationship to current PGID executor

The existing #2213/#2279 process-group implementation remains a lower-assurance profile useful for bounded modeled execution and failure cleanup.

It must not be automatically promoted into this profile.

```text
ProcessGroupContainmentV1
!= RaceSafeVerifierContainmentV1
```

A caller that requires race-safe containment must fail closed when only the PGID profile is available.

## 18. Qualification plan

Qualification should be split into independent gates.

### Gate C0 — host capability / acquisition

Prove:

- exact kernel/platform capsule;
- cgroup-v2 profile detection;
- delegated parent acquisition;
- child-cgroup creation;
- retained control-handle acquisition;
- `clone3(CLONE_INTO_CGROUP | CLONE_PIDFD)` support;
- unsupported hosts fail before verifier execution.

### Gate C1 — clean execution

Prove:

- verifier starts inside the owned cgroup from its first executable instruction;
- pidfd identifies the direct process;
- exact stdout/stderr bounds apply;
- successful direct child plus `populated 0` permits clean completion;
- clean direct-child exit with a surviving background descendant does not permit success.

### Gate C2 — teardown

Prove:

- timeout kills the owned tree;
- stdout overflow kills the owned tree;
- stderr overflow kills the owned tree;
- non-zero verifier failure tears down the owned tree;
- `populated 0` is required before teardown success;
- reader cleanup cannot outlive the bounded teardown window.

### Gate C3 — adversarial descendants

Prove:

- concurrent forks during `cgroup.kill` do not escape;
- descendants retaining pipes cannot disable the deadline;
- descendants closing pipes cannot fool clean-success admission;
- attempted cgroup migration is denied under the admitted delegation profile;
- unrelated sibling processes/cgroups remain alive.

### Gate C4 — identity/reuse

Prove:

- direct-child lifecycle operations use pidfd identity rather than a reused PID;
- retained cgroup handles do not silently retarget when textual paths are removed/recreated;
- cleanup never targets a different same-named cgroup or unrelated process tree.

## 19. Integration rule for authenticated receipts

The future concrete GitHub authentication backend may require this capability as an additional input, but this contract does not itself mint `AuthenticatedQualificationReceiptV1`.

Recommended composition:

```text
ExecutedGitHubVerificationV1
+ RaceSafeVerifierContainmentV1
+ strict verifier-result parser
+ retained-root/Public Good resolution
+ unique trusted-builder binding
+ qualified currentness/clock provenance
    -> eligible input to receipt-authentication mint
```

Every input must bind to the same execution lineage.

## 20. External references

Authoritative Linux behavior should be grounded against current kernel/man-pages documentation at qualification time, including:

- Linux kernel cgroup v2 documentation (`cgroup.kill`, `cgroup.events`, delegation semantics);
- `clone3(2)` (`CLONE_INTO_CGROUP`, `CLONE_PIDFD`);
- pidfd lifecycle/signaling semantics.

The contract intentionally specifies semantic requirements rather than pinning one Rust wrapper crate. Dependency choice is a later implementation decision and must not weaken the theorem.

## 21. Final theorem

```text
race-safe kernel process identity
+ owned descendant containment domain
+ atomic placement
+ bounded execution
+ tree-wide teardown
+ observed empty containment
    -> RaceSafeVerifierContainmentV1

RaceSafeVerifierContainmentV1
    != cryptographic verifier validity
    != qualification receipt authentication
    != production admission
    != application authority
```
