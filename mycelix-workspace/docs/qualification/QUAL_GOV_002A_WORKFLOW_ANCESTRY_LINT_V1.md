# QUAL-GOV-002A — Qualification Workflow Ancestry Lint v1

Status: repository qualification-infrastructure hygiene only.  
Tracking issue: #2532.

## Purpose

Prevent exact-subject qualification workflows from failing before semantic validation merely because `actions/checkout` did not fetch enough Git ancestry for the workflow's own parent/history checks.

The failure mode discovered in the CIV-RES r1 line was:

```text
workflow requires HEAD^
+ checkout implicit/default fetch-depth = 1
-> parent object unavailable
-> qualifier fails before semantic validator
```

This layer exists to catch that class of harness defect before a subject is treated as semantically red.

```text
QualificationHarnessFailure != SemanticContractFailure
```

## V1 classification

The lint recognizes two closed ancestry-requirement classes.

### Immediate-parent sensitive

V1 recognizes:

```text
HEAD^
HEAD~1
```

These require an explicit checkout history of either:

```text
fetch-depth: 0
```

or:

```text
fetch-depth >= 2
```

The implicit checkout default and `fetch-depth: 1` are insufficient.

### Full-history sensitive

V1 recognizes:

```text
git merge-base
```

and conservatively requires:

```text
fetch-depth: 0
```

V1 does not attempt to prove that a particular shallow history would be sufficient for an arbitrary merge-base/history query.

## Closed self-test corpus

The checker must preserve these exact behavioral cases:

```text
HEAD^ + implicit/default depth -> REFUSE
HEAD^ + fetch-depth: 1       -> REFUSE
HEAD^ + fetch-depth: 2       -> ADMIT
HEAD^ + fetch-depth: 0       -> ADMIT
HEAD~1 + fetch-depth: 2      -> ADMIT
merge-base + fetch-depth: 2  -> REFUSE
merge-base + fetch-depth: 0  -> ADMIT
no ancestry-sensitive command -> ADMIT
```

These are workflow-harness judgments only.

## Conservative textual analysis

V1 is intentionally dependency-free and does not claim to implement the complete GitHub Actions YAML language.

It:

1. scans changed `.github/workflows/*.yml` / `*.yaml` files;
2. identifies the closed V1 ancestry-sensitive patterns;
3. finds `actions/checkout` steps;
4. attributes an explicit `fetch-depth` only inside the checkout step's own indentation block;
5. refuses an ancestry-sensitive workflow when no checkout step provides the required history.

The lint does not normalize an unsafe workflow into a safe one. It only admits/refuses the checked text under this V1 rule.

## Why the lint workflow uses full history

The reusable lint workflow itself checks the PR base-to-head workflow diff. Therefore it deliberately uses a pinned checkout action with:

```text
fetch-depth: 0
```

This avoids reproducing the same ancestry insufficiency inside the guard.

## Scope and trigger

The guard runs only when a PR changes:

```text
.github/workflows/**
scripts/qualification/qual_gov_002a_workflow_ancestry_lint.py
mycelix-workspace/docs/qualification/QUAL_GOV_002A_WORKFLOW_ANCESTRY_LINT_V1.md
```

It is not intended to allocate a runner for every product/code PR.

## Required non-equivalences

```text
WorkflowLintPass != QualifierPass
SufficientFetchDepth != CorrectSemanticValidator
QualificationHarnessReady != ProductReady
StaticPatternAbsent != WorkflowCorrect
FetchDepthZero != SemanticCorrectness
```

## Deliberate nonclaims

A PASS does not establish:

- that the workflow's expected parent is correct;
- that the workflow validates the right changed-file set;
- that the semantic validator is correct;
- that the workflow is secure against every Actions/YAML edge case;
- that a subject passed qualification;
- product/runtime correctness;
- security or privacy adequacy;
- deployment readiness;
- legal validity.

## Future extension boundary

Later versions may add additional closed classes such as explicit arbitrary-SHA ancestry, `git log`, `rev-list` traversal, submodule/history requirements, or reusable-workflow composition. Those must be added with their own positive/refusal fixtures rather than silently broadening V1 interpretation.

## Claim ceiling

A PASS may establish only that the exact checker self-tests pass and that the changed workflow texts satisfy the frozen V1 ancestry-fetch hygiene rule. It establishes no semantic or product theorem beyond that.
