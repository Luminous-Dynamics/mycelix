# FORGE-009F — Git Protected Ref Transaction Plan

This adapter converts a positive FORGE-009E atomic-consumption intent into the exact bytes for a future Git `update-ref` transaction.

It deliberately does **not** execute Git.

## Exact command surface

The later executor is expected to invoke a configured absolute Git executable with:

```text
update-ref --no-deref --stdin -z
```

`--no-deref` ensures the named refs themselves are updated rather than following symbolic refs. `-z` uses NUL-framed fields and avoids shell/C-quoting ambiguity.

The plan stdin is exactly:

```text
start\0
update <target-ref>\0<proposed-oid>\0<expected-old-oid>\0
create <consumption-marker-ref>\0<proposed-oid>\0
prepare\0
commit\0
```

The actual plan is binary NUL-framed data; the notation above is explanatory.

## Type gate

`GitRefTransactionPlanV1::new(...)` accepts only:

`AtomicProtectedRefConsumptionIntentV1`

from FORGE-009E.

The plan has no `Deserialize` implementation. A caller cannot deserialize arbitrary ref names/OIDs into an executable-looking positive plan.

## Evidence binding

The plan commits:

- exact FORGE-009E atomic intent;
- protected target ref;
- exact old target OID;
- exact new target OID;
- exact consumption-marker ref;
- exact marker value;
- exact NUL-framed stdin bytes.

`stdin_digest` commits the exact bytes independently, while `evidence_commitment` binds those bytes back to the higher-level transaction subject.

## No execution API

This crate intentionally does not expose `Command::new`, process spawning, repository mutation, or receipt generation.

The concrete executor must wait for the future sealed merge-execution authorization type that joins:

```text
protected review authority
+ source qualification
+ qualified M0 OfflineEvidence
+ atomic request-consumption policy
```

Only that type should unlock mutation of a protected ref.

## Git semantics relied upon later

The planned transaction uses Git's `update-ref --stdin` transaction commands:

```text
start
update
create
prepare
commit
```

The target `update` carries an expected-old OID precondition. The marker `create` requires the marker ref not to exist. These commands are intended to be committed in one Git ref transaction.

FORGE-009F proves only exact command construction. It does not prove Git executed the plan.

## Tests

Focused tests cover:

- fixed shell-free argv;
- exact NUL-framed transaction bytes;
- stdin digest binding;
- marker/target alias rejection;
- marker value equality with the proposed revision;
- public constructor requiring positive FORGE-009E input.
