# Shared Process Containment Contract

Status: design contract only  
Issue: #2374  
Consumers: authentication (#2295), QCAP (#1926), NixOS host adapter (#2319)

## 1. Purpose

Mycelix has reached the same Linux process-lifecycle boundary in more than one subsystem. The authentication verifier executor and QCAP both need a way to prove that a process and all descendants inside an admitted execution domain have terminated before execution is treated as cleanly contained.

This contract defines one shared substrate for those mechanics.

It deliberately does **not** define GitHub/Sigstore semantics, QCAP gate semantics, NixOS service policy, trusted time, qualification PASS, receipt authentication, or application authority.

## 2. Governing separations

```text
process started
!= process contained

fresh process group
!= escaped-descendant containment

numeric PID / PGID
!= persistent race-safe kernel identity

process exited
!= containment domain empty

serialized containment evidence
!= live containment capability

containment PASS
!= theorem PASS
!= receipt authentication
```

The strongest v1 Linux profile is based on cgroup v2 plus pidfd identity.

## 3. Profile vocabulary

Consumers must use an explicit closed profile vocabulary. At minimum:

```text
ProcessGroupExperimentalV1
CgroupV2PidfdV1
```

`ProcessGroupExperimentalV1` may describe historical/current lower-assurance execution using process groups and bounded cleanup.

`CgroupV2PidfdV1` is the first profile eligible for the shared race-safe containment theorem defined here.

There is no implicit ordering conversion and no silent fallback.

```text
ProcessGroupExperimentalV1
!= CgroupV2PidfdV1
```

A consumer that requires `CgroupV2PidfdV1` must fail closed if only process-group evidence is available.

## 4. Shared ownership boundary

The shared substrate owns only:

- Linux cgroup-v2 capability discovery;
- explicit delegated-parent admission;
- child containment-domain creation;
- retained cgroup control handles;
- atomic or equivalently qualified process placement into the owned cgroup;
- direct-child pidfd identity;
- lifecycle observation needed to distinguish running/direct-child-exited/domain-empty;
- bounded `cgroup.kill` teardown;
- bounded `cgroup.events` observation;
- `populated=0` success postcondition;
- direct-child wait/reap coordination;
- cleanup/removal of the owned empty containment domain;
- typed errors and structural audit evidence.

It does not own:

- verifier command arguments;
- theorem/gate commands;
- Sigstore parsing;
- authentication policies;
- QCAP attempt verdicts;
- NixOS/systemd unit definitions;
- network, filesystem, seccomp, user-namespace or VM isolation;
- trusted time;
- production authority.

## 5. Host prerequisite

The substrate consumes an already-established delegated cgroup-v2 parent capability/profile.

It must not obtain host authority by invoking `sudo`, `systemd-run`, privileged helper CLIs, or ambient administrative APIs.

A NixOS/systemd adapter such as #2319 may provision the parent delegation declaratively. Other host adapters may exist later under separate qualification profiles.

## 6. Strong v1 theorem

The target theorem is:

```text
admitted cgroup-v2 delegated parent
+ retained parent/control handles
+ owned child containment domain
+ atomic child placement into that domain
+ pidfd direct-child identity
+ descendants inherit containment membership
+ bounded process execution/teardown policy
+ cgroup.kill when termination is required
+ cgroup.events confirms populated=0
+ direct child is observed/reaped
+ domain cleanup succeeds
    -> RaceSafeProcessContainmentV1
```

Still:

```text
RaceSafeProcessContainmentV1
!= executable authenticity
!= command correctness
!= input-closure correctness
!= theorem PASS
!= trusted current time
!= authentication
!= ProductionAdmissionV1
```

## 7. Kernel-object identity

Authority must be tied to retained kernel objects/handles as far as the admitted Linux profile permits.

Do not treat a reopened cgroup path string or a numeric PID/PGID as a permanent lifetime identity.

Preferred implementation direction:

- retain cgroup directory/control file descriptors for the execution lifetime;
- use directory-relative strict path resolution for cgroup members where practical;
- use `CLONE_PIDFD` or an equivalently qualified pidfd acquisition path for the direct child;
- use `CLONE_INTO_CGROUP` or an equivalently qualified atomic placement primitive so the child does not execute outside the containment domain before migration;
- retain path strings only as audit/presentation evidence.

Exact syscall/wrapper choices belong to the implementation tranche and qualification evidence, not this abstract contract.

## 8. Escape boundary

The child must not receive authority to move itself or descendants outside the owned containment domain.

A host-adapter design that gives the verifier writable access to an ancestor `cgroup.procs` or another migration surface weakens this theorem and must be rejected by the stronger profile.

Delegation therefore needs an explicit ownership split:

```text
host adapter / supervisor
    owns delegated parent controls

shared containment substrate
    owns child-domain lifecycle

contained workload
    does not own migration-out authority
```

## 9. Lifecycle

A conceptual sealed state machine is:

```text
CapabilityProbed
-> DomainCreated
-> ChildPlaced
-> Running
-> DirectChildExited | TerminationRequested
-> TreeTerminationRequested      (when required)
-> EmptyObserved
-> DirectChildReaped
-> DomainReleased
```

Success requires `EmptyObserved` under the exact stronger profile.

A direct child may exit successfully while a descendant remains. That is not clean containment completion.

A direct child may close stdout/stderr while a descendant survives. That is not clean containment completion.

## 10. Clean-success rule

For `CgroupV2PidfdV1`, clean direct-child exit is insufficient.

Before sealing success:

1. observe the direct child's terminal state through the admitted identity/wait profile;
2. observe the owned cgroup subtree as `populated=0`;
3. verify no required teardown error occurred;
4. reap/close the direct-child identity according to the admitted lifecycle;
5. release/remove the empty owned domain.

If the direct child exits but the containment domain remains populated beyond the bound, return a typed containment failure.

## 11. Failure/timeout cleanup

On timeout, output overflow, cancellation, process failure, or consumer-requested abort:

1. mark the attempt as termination-required in the process-local state;
2. request direct-child termination through the admitted direct-child identity where appropriate;
3. invoke the retained `cgroup.kill` control for the owned domain;
4. continue bounded observation until `cgroup.events` reports `populated 0`;
5. reap/observe the direct child;
6. release the domain only after the empty postcondition;
7. return the original consumer failure only if containment cleanup itself completed successfully;
8. otherwise surface the containment cleanup failure as a distinct infrastructure/containment result.

A consumer must not convert containment-cleanup failure into theorem/authentication failure.

## 12. Typed failure taxonomy

The shared substrate should preserve distinct failures at least for:

```text
UnsupportedPlatform
CgroupV2Unavailable
DelegatedParentUnavailable
DelegatedParentInvalid
ContainmentDomainCreateFailed
ContainmentControlOpenFailed
AtomicPlacementUnsupported
AtomicPlacementFailed
PidfdUnavailable
PidfdAcquireFailed
ChildSpawnFailed
ChildWaitFailed
CgroupKillFailed
CgroupEventsReadFailed
CgroupEventsMalformed
ContainmentStillPopulated
TeardownDeadlineExceeded
DomainReleaseFailed
ContainmentMigrationViolation
```

Exact enum names may evolve before implementation freeze, but error classes must remain distinguishable.

## 13. Process-local capability vs audit evidence

The substrate should expose two deliberately different classes of output.

### Sealed live capability

Conceptually:

```text
RaceSafeProcessContainmentV1
```

Properties:

- private fields;
- no public unchecked constructor;
- no `Serialize` / `Deserialize` / `Default`;
- retains live kernel handles while the domain exists;
- cannot be recreated from JSON or a receipt;
- read-only accessors may expose structural facts.

### Serializable audit evidence

Conceptually:

```text
ProcessContainmentReceiptV1
```

May bind:

- profile/version;
- platform/kernel facts admitted by profile;
- host-adapter profile identity;
- containment-domain presentation identity;
- lifecycle timestamps/durations where structurally useful;
- whether tree termination was requested;
- whether `populated=0` was observed;
- direct-child terminal outcome;
- teardown outcome;
- explicit nonclaims.

The structural receipt does not recreate the sealed capability.

## 14. Consumer policy wrapper

Consumers should not modify historical execution-policy constructors merely to add containment.

Prefer an additive wrapper such as:

```text
ContainmentRequirementV1 {
    required_profile,
    fallback_policy,
}

BoundExecutionPolicyV1<ConsumerPolicy> {
    consumer_policy,
    containment_requirement,
}
```

or consumer-specific equivalents.

For authentication, a future wrapper may bind:

```text
GitHubPublicVerifierExecutionPolicyV1
+ VerifierContainmentRequirementV1
```

For QCAP, a runner profile may bind the same shared containment profile separately from theorem/capsule identity.

The important invariant is:

```text
consumer semantic policy
!= containment policy
```

while both are exact inputs to the resulting execution evidence.

## 15. Authentication consumer rule

Authentication may consume the shared capability as follows:

```text
RaceSafeProcessContainmentV1
+ exact verifier execution policy
+ exact verifier/input evidence
+ actual bounded verifier execution
    -> authentication-specific execution-origin evidence
```

A future authority-bearing authenticated-receipt mint may require:

```text
containment_profile == CgroupV2PidfdV1
```

and refuse `ProcessGroupExperimentalV1` even if every other authentication field is valid.

The current #2213 process-group executor remains lower-assurance historical/experimental evidence.

## 16. QCAP consumer rule

QCAP may consume the shared capability to establish only the runner-plane postcondition:

```text
QCAP gate execution
+ RaceSafeProcessContainmentV1
    -> containment postcondition established for this gate attempt
```

QCAP still owns:

- capsule/theorem identity;
- worktree isolation;
- gate ordering;
- output/resource ceilings;
- `GatePass` / `GateFail`;
- `RunnerInfrastructureFailure`;
- `GateNotRun`;
- attempt and final receipt semantics.

The shared substrate never emits a QCAP PASS.

## 17. NixOS adapter rule

#2319 may establish an exact NixOS/systemd host adapter such as:

```text
NixOSDelegatedContainmentHostV1
```

That adapter proves the service received the exact delegated cgroup parent and hardening profile expected by this substrate.

Systemd `Delegate=` or NixOS configuration is provisioning evidence, not the containment theorem itself.

## 18. QUAL-001D boundary

Exact executable/dependency/environment input closure remains QUAL-001D / #1335.

```text
uses Nix
!= qualified input closure

has race-safe cgroup containment
!= qualified input closure
```

A stronger production execution may compose both the QUAL-001D closure theorem and this containment theorem without either absorbing the other.

## 19. Qualification gates

The shared substrate should be qualified independently before authentication or QCAP treat it as a positive prerequisite.

### PC-A — profile/types

Prove closed profile vocabulary, no PGID->cgroup conversion, sealed capability non-forgeability and structural-receipt separation.

### PC-B — host capability acquisition

Prove exact cgroup-v2/delegation/control/pidfd/atomic-placement prerequisites and fail-closed behavior on unsupported hosts.

### PC-C — clean process lifecycle

Prove ordinary direct-child completion reaches `populated=0`, reaps correctly and releases only the owned empty domain.

### PC-D — descendant containment

Prove descendants survive process-group/session changes but remain inside the cgroup containment domain.

### PC-E — adversarial teardown

Prove timeout, ignored signals, retained pipes, closed-pipe background descendants and concurrent forks all converge to bounded `populated=0` or typed containment failure.

### PC-F — noninterference

Prove sibling/unrelated processes and cgroups are unaffected by teardown of the owned domain.

### PC-G — identity/reuse

Prove direct-child operations use pidfd semantics such that numeric PID reuse cannot retarget the direct-child capability.

## 20. Minimum adversarial corpus

At minimum include:

1. ordinary child success;
2. direct child non-zero exit;
3. direct child timeout;
4. child forks ordinary descendant;
5. child forks descendant retaining stdout/stderr;
6. child forks descendant closing stdout/stderr;
7. descendant calls `setsid()`;
8. descendant ignores SIGTERM;
9. descendant forks during teardown;
10. unrelated sibling process remains alive;
11. unrelated sibling cgroup remains populated/untouched;
12. unsupported delegation fails before admitted execution;
13. missing `cgroup.kill`/`cgroup.events` fails profile admission;
14. direct-child pidfd identity remains bound despite numeric-PID reuse pressure;
15. structural receipt cannot recreate sealed capability;
16. process-group-only evidence cannot satisfy `CgroupV2PidfdV1` requirement.

## 21. Authority ceiling

This substrate may establish only that one execution domain was managed according to an exact containment profile and reached the registered lifecycle postcondition.

It does not establish:

- executable authenticity;
- cryptographic verification correctness;
- theorem correctness;
- trusted clock/currentness;
- filesystem/network isolation;
- kernel/hypervisor integrity;
- authentication;
- qualification PASS;
- production admission;
- governance, finance, identity or application authority.
