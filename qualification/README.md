# Qualification capsules (QCAP)

QCAP-001A is the runner-neutral substrate for EVIDENCE-CI-004 / #1611.

```text
qualification theorem != GitHub Actions workflow
qualification capsule = immutable theorem definition
runner adapter = execution mechanism
```

`runner/qcap.py` is Python-stdlib-only. It validates a closed v1 manifest, computes a domain-separated capsule commitment, verifies every referenced gate script by SHA-256, materializes a fresh detached worktree at the exact frozen subject for every gate, executes one ordered gate set, and emits one same-attempt conjunctive receipt.

The capsule manifest, execution context, and attempt receipt each have closed v1 schemas under `schema/`.

## Evidence-plane isolation

Every gate runs in a separate temporary Git worktree at the exact `product_subject_sha`.

```text
Gate A -> fresh exact subject worktree -> discard
Gate B -> fresh exact subject worktree -> discard
Gate C -> fresh exact subject worktree -> discard
```

Non-ignored untracked files, tracked modifications, or HEAD movement are gate-input mutations and convert that gate to `GateFail`. Ignored compiler/build outputs are permitted inside the isolated worktree but are discarded before the next gate. This prevents one evidence plane from contaminating another through working-tree residue or build artifacts.

The caller's source checkout is never reset or detached by QCAP; it is used only as the Git object store from which exact frozen worktrees are materialized.

## Execution-context binding

The capsule commits the required `toolchain_profile_ref` and `environment_profile_ref`. The runner must provide a closed `QualificationExecutionContextV1` containing:

```text
runner_profile_ref
toolchain_profile_ref        # must exactly equal capsule requirement
environment_profile_ref      # must exactly equal capsule requirement
resolved_toolchain_commitment
resolved_environment_commitment
```

The receipt binds that whole execution context. A free-form environment hash cannot substitute for a mismatched required profile.

A valid profile/content commitment is still not external proof that a runner honestly resolved that environment; signed/OIDC/Xenia provenance remains a separate future layer.

## Gate contract

Each gate commits a `timeout_seconds` ceiling (1..7200 seconds) so a runner adapter has a deterministic no-hang boundary.

```text
0   executed predicate PASS
10  executed predicate FAIL
20  runner/environment failure
other nonzero -> conservatively RunnerInfrastructureFailure
wall-clock timeout -> RunnerInfrastructureFailure
```

If bounded runtime is itself a theorem predicate, that predicate must be implemented inside a theorem gate and deliberately return `10`; QCAP's outer wall-clock timeout is an execution-plane safety limit only.

The capsule commitment is `SHA-256(domain || u64_be(length) || canonical_json)`, with domain `MYCELIX_QUALIFICATION_CAPSULE_V1\0`. Floats are forbidden; object keys use UTF-8 byte ordering; semantic-set arrays must be sorted/unique. The manifest never contains its own digest.

Verdicts:

```text
all GatePass                    -> CompletedConjunctivePass
any RunnerInfrastructureFailure -> RunnerInfrastructureFailure
otherwise                       -> CompletedConjunctiveFail
```

This preserves #1499 (no cross-run stitching) and #1599 (runner-plane failure is not theorem RED).

## Lineage classification

Runner preflight proves only that the named repository is the expected repository identity and that the frozen subject commit is available in its object store. Predecessor/scope/blob identity is a content-addressed **theorem gate**, `gates/subject_identity.py`.

That gate exits `10` for an executed lineage predicate failure and `20` only when Git machinery itself cannot execute. A missing expected object is therefore theorem RED; a runner unable to materialize the frozen subject is runner-plane failure.

## Receipt verification

`qcap.py verify-receipt` recomputes the domain-separated receipt commitment and checks that the receipt binds the exact capsule, repository, subject, theorem, gate set, claim/nonclaims, execution-context profile refs, and both resolved toolchain/environment commitments. It recomputes the conjunctive verdict and rejects status/exit-code contradictions even if a malformed receipt was re-committed.

**Status:** staged substrate only. It does not qualify or reinterpret any FIN-ECO subject. Existing queued FIN-ECO attempts remain immutable. QCAP-001B should translate FIN-ECO-002F predicate-for-predicate only after this substrate is reviewed/qualified.
