# REGEN-007 — Qualification Evidence Contract v1

Status: normative architecture draft; no scientific, agronomic, climate, market, governance, or physical-action authority.

## 1. Purpose

REGEN qualification must prove not merely that a workflow turned green, but **what exact subject executed, under what environment, with what commands, and what proposition the result actually supports**.

This contract exists because pull-request CI can execute more than one legitimate Git subject. In particular, GitHub may expose a synthetic merge ref that combines a pull-request head with its base. A successful synthetic-merge run is useful integration evidence, but it is not automatically evidence that the immutable product head itself executed.

REGEN therefore freezes the following principle:

```text
workflow success
!= exact-subject execution
!= implementation qualification
!= scientific validity
!= authority
```

The qualification plane must identify and preserve each distinction.

## 2. Subject classes

Every evidence-bearing execution MUST declare one subject class.

### 2.1 ProductHead

An immutable Git commit that is the exact proposed product head.

Required proposition:

```text
observed HEAD == declared product SHA
```

This is the preferred subject for implementation qualification.

### 2.2 IntegrationMerge

A specific integration commit, including a deliberately authored multi-parent integration commit or a synthetic pull-request merge subject.

Integration evidence proves only properties of that exact composition.

```text
IntegrationMerge PASS
!= ProductHead PASS
```

If a synthetic provider-generated merge ref is used, the receipt must say so explicitly.

### 2.3 ReleaseArtifact

An immutable built artifact identified by a cryptographic digest and enough provenance to bind it to its source/build lineage.

A ReleaseArtifact result does not retroactively prove a source commit unless the artifact-to-source binding is independently established.

### 2.4 ExternalFixture

An exact externally supplied corpus, profile, dataset, protocol, or artifact used by an oracle or compatibility campaign.

The fixture identity and the executed implementation identity remain separate fields.

## 3. No implicit subject substitution

A workflow name, job name, branch name, pull-request number, or UI label does not prove execution subject identity.

A qualification job MUST NOT infer exact-head execution from language such as:

```text
Checkout exact subject
```

The job must verify it.

For Git-backed ProductHead qualification, the minimum evidence is:

```text
declared_subject_sha
observed_git_head
assert(declared_subject_sha == observed_git_head)
```

The assertion must occur **before** the evidence-bearing test campaign.

## 4. Qualification receipt

A REGEN qualification lane SHOULD emit or preserve a machine-readable receipt containing at least:

- receipt schema/version;
- repository identity;
- subject class;
- declared subject identity;
- independently observed subject identity;
- base/integration identities where applicable;
- workflow identity;
- workflow/relevant contract revision identity where practical;
- runner operating-system identity;
- language/runtime/toolchain identities;
- qualification command identities or checked-in command-contract identity;
- fixture/data identities;
- start/completion time as observational metadata;
- per-stage result;
- final conclusion;
- produced artifact/receipt digests where applicable.

A timestamp is metadata, not proof of causal order beyond the semantics actually supplied by the execution platform.

## 5. Subject verification occurs first

The intended qualification order is:

```text
checkout / materialize candidate
        -> observe candidate identity
        -> compare with declared identity
        -> fail on mismatch
        -> record environment identity
        -> execute qualification contract
        -> hygiene/postflight
        -> emit result
```

A test suite that ran before the subject mismatch was detected cannot be promoted as exact-subject evidence.

## 6. Product and integration evidence are both useful

REGEN does not treat synthetic-merge evidence as invalid.

It preserves two different propositions:

```text
ProductHead PASS
    = exact product candidate passed its declared campaign

IntegrationMerge PASS
    = exact integration composition passed its declared campaign
```

A mature tranche may require both.

For example:

```text
product-head qualification
+
base-integration qualification
+
independent oracle parity
```

may jointly provide stronger evidence than any single lane.

But the receipts remain distinct and may not be silently collapsed into one PASS.

## 7. Historical evidence is immutable

When a qualification defect is discovered later, earlier run evidence MUST NOT be rewritten into a stronger proposition.

Allowed:

```text
run X succeeded on synthetic merge subject Y;
we previously described it too strongly;
current interpretation = IntegrationMerge PASS.
```

Forbidden:

```text
run X was green;
therefore treat it as ProductHead PASS anyway.
```

The corrected interpretation should be visible in the active PR or evidence record.

## 8. Failed candidates remain evidence

A failed candidate is not noise.

REGEN SHOULD retain enough information to distinguish:

- subject mismatch;
- syntax/format failure;
- compile failure;
- unit/property/oracle failure;
- lint failure;
- hygiene failure;
- infrastructure cancellation/unavailability;
- scientific falsification;
- unsupported/unexecuted lane.

A later corrected candidate does not erase the failed lineage.

## 9. Queued and unexecuted states

The following equivalences are forbidden:

```text
queued == PASS
no steps == PASS
workflow exists == executed
mergeable == qualified
checks pending == latent support
```

Where the platform supplies no executed steps/log surface, the correct state is unexecuted or unresolved, not PASS or FAIL of the implementation theorem.

## 10. Qualification command authority

Where qualification commands are safety/science-significant, REGEN SHOULD prefer a checked-in, hashable command contract over duplicated workflow prose.

Conceptually:

```text
checked-in qualification contract
        -> workflow invokes exact contract
        -> pre/postflight digest identity
        -> receipt binds contract identity
```

This reduces drift between what a PR claims to run and what CI actually runs.

A workflow remains transport/orchestration; the checked-in contract is the preferred long-lived command authority where practical.

## 11. Environment identity

A green result can depend on environment.

Qualification SHOULD preserve, where material:

- OS/runner image;
- architecture;
- language/compiler/runtime version;
- dependency lock identity;
- feature flags;
- environment/configuration digests;
- hardware/backend identity where results can vary materially.

Absence of one field should remain explicit rather than being interpreted as reproducibility.

## 12. Reproducibility boundary

```text
complete environment identity
!= deterministic execution
!= scientific validity
```

Environment capture makes reproduction targetable. It does not prove that the implementation or model is correct.

## 13. Fixture identity

Golden vectors, field protocols, external standards, datasets, and oracle inputs used by qualification MUST have exact identity appropriate to their role.

For immutable file fixtures, a cryptographic digest is preferred.

For example:

```text
fixture path
+ fixture SHA-256
+ schema identity
```

A fixture PASS proves only the implementation/fixture proposition declared by the campaign.

## 14. Independent oracle boundary

An oracle intended to provide implementation independence SHOULD NOT import the production implementation whose behavior it is supposed to check.

Shared fixtures are acceptable where fixture bytes themselves are the frozen interoperability contract.

```text
same fixture
+ independent implementation
= useful cross-implementation evidence
```

not:

```text
same production library imported twice
= independent confirmation
```

## 15. Mutation and adversarial qualification

Later REGEN tranches SHOULD distinguish ordinary regression PASS from adversarial strength.

Possible evidence classes include:

- authored positive/negative cases;
- property testing;
- fuzzing;
- mutation testing;
- malformed-input campaigns;
- independent oracle parity;
- deterministic replay;
- fault injection;
- compound-shock scenarios.

A plain unit-test PASS does not imply these stronger classes have been exercised.

## 16. Scientific execution boundary

For scientific/modeling tranches:

```text
software qualification
!= hypothesis supported
```

The execution stack should preserve at least:

```text
implementation qualified enough to execute experiment
        -> frozen experiment protocol
        -> exact data/fixture identity
        -> observed result
        -> analysis identity
        -> claim/non-claim boundary
```

A compile/test PASS proves software properties only.

## 17. Field evidence boundary

For future soil/agronomic field work, REGEN MUST preserve distinctions among:

- protocol qualification;
- measurement ingestion qualification;
- actual field execution;
- outcome observation;
- analysis execution;
- contextual interpretation;
- replication.

A qualified field-trial schema does not mean a field trial occurred.

A completed trial does not mean the treatment is universally effective.

## 18. Cross-repository qualification

Where Mycelix and Symthaea share a protocol, compatibility SHOULD be tested from both sides when material.

The preferred shape is:

```text
Mycelix production implementation
    -> frozen language-neutral vectors

Symthaea independent implementation/oracle
    -> exact same vector bytes

compare propositions
```

Each repository proves its own exact execution subject.

One repository's PASS cannot qualify the other repository's product head.

## 19. Parent/child stack integrity

A stacked child PR MUST NOT silently carry stale copies of changed parent files when it is restacked.

After parent movement, the child should be reconstructed from:

```text
current exact parent tree
+ child-owned paths only
```

unless a child intentionally modifies a parent-owned path, in which case that modification must be explicit in review.

This prevents a child tree from accidentally reverting a qualified parent correction.

## 20. Exact ancestry for integration

Where a convergence/integration commit is used, qualification SHOULD prove:

- declared parents are the actual commit parents;
- expected parent blobs survive on non-conflicting paths;
- conflict resolutions are explicit;
- integration-specific modifications are separately inspectable.

Ancestry presence alone does not prove blob preservation.

## 21. Hygiene

A focused qualification lane SHOULD fail on unexpected checkout mutation where practical.

Typical postflight checks include:

```text
git diff --check
clean tracked/untracked state according to declared policy
```

Generated caches/build outputs should be intentionally ignored or isolated rather than making cleanliness unknowable.

## 22. Evidence promotion vocabulary

REGEN SHOULD use precise states such as:

- authored;
- queued;
- running;
- infrastructure-blocked;
- ProductHead PASS;
- ProductHead FAIL;
- IntegrationMerge PASS;
- IntegrationMerge FAIL;
- oracle PASS/FAIL;
- scientifically supported / not supported / unresolved;
- field observed / not observed;
- replicated / not replicated.

Avoid generic `green`, `validated`, `proven`, or `safe` when a narrower proposition is available.

## 23. Security boundary

Qualification infrastructure is part of the evidence supply chain.

REGEN SHOULD minimize unnecessary workflow permissions and SHOULD pin consequential third-party actions to immutable revisions where practical.

A compromised CI environment can still produce misleading evidence; CI success is therefore not a root of unlimited trust.

## 24. Authority boundary

No qualification receipt creates governance or execution authority.

```text
qualified implementation
!= adopted policy
!= permission to apply amendment
!= permission to operate equipment
!= carbon-credit authority
```

Qualification informs downstream decisions. It does not manufacture them.

## 25. Immediate application

REGEN-002, REGEN-003, REGEN-004A, and REGEN-004B SHOULD apply this contract immediately:

1. exact PR/product head checkout;
2. explicit observed-vs-declared SHA assertion;
3. historical synthetic-merge results relabeled as integration evidence;
4. atomic child stacks reconstructed from current parent trees;
5. current exact-head execution required before promotion.

## 26. Future reuse

REGEN-010+ should use the same subject semantics for:

- soil evidence contracts;
- biomass provenance;
- batch lineage;
- field-trial protocols;
- Symthaea accounting/model kernels;
- independent mass/nutrient/energy oracles;
- resilience shock campaigns;
- climate/carbon bridges.

The concrete toolchain may differ, but the evidence semantics should not.

## 27. Deliberate non-claims

REGEN-007 does not prove any current implementation correct, establish agricultural efficacy, establish quality conformance, authenticate evidence, prove regulatory compliance, establish carbon removal, qualify physical equipment, or authorize physical action.

It freezes how future REGEN qualification evidence must describe what actually executed and what that result can legitimately claim.
