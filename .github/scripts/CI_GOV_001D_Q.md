# CI-GOV-001D-Q — exact-subject offline qualifier v0.2

This tranche qualifies source identity and deterministic offline behavior for the frozen CI-GOV-001D planner/executor chain. It adds no GitHub Actions workflow and has no GitHub API client.

## Frozen subjects

- qualified CI-GOV-001A: `884a14e14758a91d3c1d370d49648946dc8b89ef`
- 001D-A planner: `4fc54eee4bc8a590d8cbe348bfa81ba649ddd1a0`
- 001D-B executor: `b55758c19bb60e3a9266f2144116dee10c5de274`

The qualifier must remain a direct child of the exact executor subject. It verifies exact commit/tree/parent relationships, exact changed paths, exact Git blob OIDs, the executor lock contract, and the qualified GET-only census client inherited from 001A.

## Local Git object-integrity boundary

V0.2 treats repository rewrite state as part of the qualification threat model.

Every Git subprocess runs with:

`GIT_NO_REPLACE_OBJECTS=1`

Qualification additionally refuses to start when any of these are present:

- any `refs/replace/*` ref;
- a non-empty `.git/info/grafts`;
- `GIT_DIR`;
- `GIT_WORK_TREE`;
- `GIT_INDEX_FILE`;
- `GIT_OBJECT_DIRECTORY`;
- `GIT_ALTERNATE_OBJECT_DIRECTORIES`;
- `GIT_COMMON_DIR`;
- `GIT_REPLACE_REF_BASE`.

This closes a concrete false-positive path: without this rule, local replace refs can make an original Git object name resolve to substitute commit/tree/blob content while commands continue to print the original object identity.

The qualifier also requires its own executed file path to be the canonical repository path and compares all three qualifier working files byte-for-byte with the exact `HEAD` versions before and after qualification.

## Minimal reconstruction surface

Qualification does **not** archive the repository. `git archive` is fixed in source to exactly five executable/imported inputs:

- `.github/scripts/ci_queue_census.py`
- `.github/scripts/ci_superseded_run_plan.py`
- `.github/scripts/test_ci_superseded_run_plan.py`
- `.github/scripts/ci_superseded_run_execute.py`
- `.github/scripts/test_ci_superseded_run_execute.py`

There is no runtime archive-scope override. After archive creation the qualifier verifies that the complete regular-file set is exactly this allowlist; an extra or missing regular file is a qualification failure.

Documentation and the executor lock are not copied into the execution sandbox. They are verified directly by exact Git object identity and lock content.

## Extraction boundary

The temporary archive extractor rejects:

- absolute or parent-traversing paths;
- symlinks;
- hardlinks;
- device nodes;
- unsupported archive member types.

This makes unrelated repository structure, submodules, links, or future application files incapable of entering the qualification sandbox.

## Python execution boundary

Qualifier and subject tests execute with:

`python -E -s -S -B <test>`

The flags ignore `PYTHON*` environment settings, disable the user site and normal `site` startup, and disable bytecode writes while retaining the test script directory as the local import root.

This prevents ambient `PYTHONPATH`, user-site packages, or `sitecustomize` startup from silently changing what the qualification tests mean.

## Executed tests

A qualifying execution runs:

1. the qualifier's own adversarial self-test suite from the clean qualifier checkout;
2. `test_ci_superseded_run_plan.py` from the exact minimal archived subject;
3. `test_ci_superseded_run_execute.py` from the exact minimal archived subject.

## Authority derivation

The qualifier statically re-derives the authority boundary from exact frozen source:

- executor contains exactly one HTTP `POST`;
- that POST is the Actions run-cancel endpoint;
- no `DELETE`, `PATCH`, or `PUT` mutation method exists;
- no `--all`, workflow-path, or time override exists;
- planner imports the qualified `GitHubReadOnlyClient` and contains no cancellation POST;
- inherited census client contains exactly one explicit `GET` and no mutation method.

## Qualifier self-test surface

The committed suite now contains **26 adversarial cases**, including:

- canonical receipt commitment stability;
- frozen 001A/planner/executor lineage;
- exact planner/executor/qualifier path sets;
- exact five-file subject archive allowlist and lack of runtime scope override;
- rejection of extra archive regular files;
- enforced `GIT_NO_REPLACE_OBJECTS=1`;
- a real temporary Git replace-ref attack that changes ordinary `git cat-file` output but is defeated by the qualifier Git environment;
- rejection of Git redirect environment;
- rejection of replace refs and grafts;
- direct-child qualifier checkout enforcement;
- exact qualifier working-file-to-HEAD byte equality;
- dirty-tree refusal;
- mandatory out-of-checkout receipt destination;
- exact executor-lock acceptance and widened-lock rejection;
- happy-path authority re-derivation;
- extra-POST rejection;
- CLI scope-widening rejection;
- parent-traversal archive rejection;
- symlink archive rejection;
- normal archive extraction;
- fail-closed identity mismatch;
- receipt-body commitment sensitivity;
- isolated Python startup flags;
- proof that the qualifier source itself has no GitHub/network/Actions mutation client.

## Receipt

On PASS the qualifier emits canonical JSON binding:

- qualifier commit/tree;
- 001A/planner/executor commit and tree identities;
- observed source blob OIDs;
- executor-lock SHA-256;
- exact five-file subject archive allowlist;
- Git rewrite/redirect rejection policy;
- Python isolation flags;
- authority-surface result;
- qualifier self-test return code and stdout/stderr SHA-256;
- each exact subject-test return code and stdout/stderr SHA-256;
- Python version;
- proposition and explicit nonclaims;
- receipt commitment over the complete receipt body.

A receipt is execution evidence for this narrow theorem only. It is not cancellation authority.

## Required clean boundary

Formal execution must start from a clean checkout of the exact qualifier commit. The qualifier verifies cleanliness and qualifier-file byte identity before execution and again after all tests.

If `--receipt-output` is supplied, its resolved path must be **outside** the repository checkout. In-checkout receipt destinations fail before qualification starts.

## Nonclaims

PASS does not cancel any run, establish that any live run is superseded, qualify product/scientific claims, establish GitHub platform integrity, guarantee runner availability, or authorize a future cancellation pilot. A pilot still requires a fresh unexpired 001D-A plan plus explicit operator-selected run IDs and authority.
