# MYC-EVID-001C — Workflow parent-availability provenance lint v1

Status: candidate executable evidence-plane profile; report-only repository census until existing findings are classified.

## Purpose

Prevent an exact-parent or ancestry qualification gate from asserting more Git provenance than the runner has actually fetched.

Core separation:

```text
exact checked-out HEAD
!= parent traversal available
!= arbitrary object present
!= qualified subject
```

A shallow repository marks boundary commits so Git treats them as roots for traversal. Merely fetching a parent object by object ID does not prove that `HEAD^` or `HEAD~N` is traversable from the checked-out shallow HEAD.

## Frozen v1 scanner

The scanner operates job-locally and step-order-sensitively over GitHub Actions workflow YAML text. It recognizes ancestry-sensitive shell Git commands only inside `run` steps; names, YAML comments, and echoed/printed documentation are not authority-bearing commands.

Recognized v1 operations include:

- `git rev-parse` using `HEAD^` / `HEAD~N`;
- `git merge-base`;
- ancestry range `git diff`;
- exact-object or ancestry-sensitive `git cat-file`, `git show`, and `git log`.

For checked-out-HEAD ancestry traversal, one of these must dominate the operation:

```text
fetch-depth: 0
fetch-depth >= ancestry distance + 1
git fetch --deepen=N sufficient to move the shallow boundary
git fetch --unshallow
```

An exact object fetch may satisfy an exact-object check for that object, but does not satisfy a `HEAD^` traversal while HEAD remains shallow.

## State tracked per job

```text
checkout_seen
ancestry_depth_available
exact_objects_available
```

A checkout in one job provides no evidence to another job. A later fetch does not retroactively satisfy an earlier ancestry command.

Dynamic or unresolved `fetch-depth` fails closed for ancestry traversal in v1.

## Finding vocabulary

```text
OK
PARENT_OBJECT_AVAILABILITY_UNPROVEN
ANCESTRY_DEPTH_INSUFFICIENT
DYNAMIC_FETCH_DEPTH_UNPROVEN
```

The report binds workflow path, job, step index/name, source line, operation, required and observed checkout depth, exact fetched objects, finding code, and PASS/FAIL.

## Permanent controls

`shallow-parent-bad.yml` preserves the original failure class: default checkout depth followed by `git rev-parse HEAD^`.

`shallow-parent-fixed.yml` is the minimum corrected form with `fetch-depth: 2`.

These controls are scanned by CI independently from the unit suite.

## Rollout

The repository-wide scan is initially **report-only**. It runs twice and the canonical JSON reports must be byte-identical. Existing findings should be classified before this becomes a mandatory blocking repository gate.

The scanner itself remains a fail-closed CLI by default; `--report-only` changes only process exit status, never finding semantics.

## Local preflight

The stdlib regression suite passes **18/18** locally, covering omitted/depth-1/depth-2 checkout, `HEAD~2`, full history, `--deepen`, `--unshallow`, exact-object fetches, the shallow-boundary distinction, job isolation, command ordering, dynamic depth, no checkout, harmless workflows, documentation/comment filtering, and deterministic multiline processing.

## Nonclaims

PASS proves only that this frozen static scanner can establish enough local Git ancestry/object availability for the recognized command. It does not prove the asserted parent is the correct policy parent, semantic tests pass, GitHub metadata is authentic, a runner is trustworthy, or the workflow is otherwise secure.
