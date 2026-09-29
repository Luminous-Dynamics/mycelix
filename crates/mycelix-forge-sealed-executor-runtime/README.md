# mycelix-forge-sealed-executor-runtime

FORGE-009K is the concrete process-execution boundary for the positive FORGE-009J sealed executor invocation.

The runtime deliberately has one execution input: SealedExecutorInvocationV1.

It does not accept a transaction plan, Git ref, object ID, command string, executable path, repository path, environment map, or caller-supplied argv/stdin at execution time.

A provider-fixed runtime instance owns:

- one fixed absolute Git executable path;
- one fixed absolute repository path;
- one exact executor identity commitment;
- one exact repository identity commitment;
- one fixed execution timeout.

Before a process is spawned, the runtime requires:

1. the invocation executor identity to equal the provider-fixed executor identity;
2. the invocation repository identity to equal the provider-fixed repository identity;
3. an opaque MergeExecutionAuthorizationV1 to bind the exact invocation;
4. an opaque `PreparedExecutionPermitV1` proving that durable preparation already committed the exact invocation, runtime configuration, and authority evidence.

Only after those gates does the runtime construct the process. It:

- invokes the fixed executable directly, never through a shell;
- uses the exact argv from the sealed invocation;
- uses the exact NUL-framed stdin from the sealed invocation;
- clears the inherited environment and supplies only fixed safety variables;
- uses the provider-fixed repository as the process working directory;
- never accepts caller-controlled environment overrides.

The runtime records a terminal or in-doubt outcome after the execution attempt, bound to both the same authority-evidence commitment and the exact preparation-evidence commitment. A spawn failure is terminal because no child process was created. Input-write failure, timeout, and wait uncertainty are explicitly in-doubt because process-side effects cannot be safely excluded. None of these outcomes may be automatically retried; they require external reconciliation.

If terminal persistence fails, the observed outcome is returned inside an in-doubt persistence error. A crash after preparation but before terminal persistence likewise remains in doubt for the external journal/reconciliation system.

## Authority boundary

MergeExecutionAuthorizationV1 is intentionally unconstructible through the public API and is not deserializable or cloneable. A future qualifier in this crate must construct it only after the complete merge-execution theorem succeeds.

`PreparedExecutionPermitV1` is likewise opaque, non-deserializable, non-cloneable, and only issuable through a crate-private constructor. The final coordinator must issue it only after the durable prepared record is committed. The runtime then consumes both authorization and preparation permit by value.

A qualified FORGE-009J binding is not execution authority.

## Claim boundary

FORGE-009K runtime
!= merge authorization
!= M0 qualification
!= proof that the host is uncompromised
!= proof that the configured provider paths are trustworthy

The independent provider/verifier evidence must bind the concrete deployment's executable, repository, authority, journal, and runtime configuration to the qualified FORGE-009J interface.


## FORGE-010 deliberate authority join

The final authority qualifier lives in this crate because MergeExecutionAuthorizationV1 and PreparedExecutionPermitV1 are intentionally opaque to all other crates.

The authority qualifier joins:

- MergeProtectedTrustedReviewBasisCurrentnessV1;
- SourceQualifiedProtectedMergeRequestV1;
- AtomicProtectedRefConsumptionIntentV1;
- QualifiedM0OfflineEvidenceV6;
- GittufMarkerNamespacePolicyEvidenceV1;
- ExecutorConstrainedGitRefTransactionPlanV1;
- SealedExecutorInterfaceBindingV1;
- SealedExecutorInvocationV1;
- the exact SealedExecutorRuntimeConfigV1.

It constructs only MergeExecutionAuthorizationV1. It does not issue PreparedExecutionPermitV1, because that permit may only be issued after the durable prepared record is committed.

The qualifier requires the provider currentness observation time to equal the supplied authority-observation time. This is an exact freshness binding, not a trusted-clock claim.

The resulting authorization does not prove that the host is uncompromised, that the provider configuration is trustworthy, or that a Git mutation has already happened.
