# FORGE-009C — pinned Git source verifier

FORGE-009C ports the concrete Git-plumbing verifier from the earlier FORGE-008B work onto the current FORGE-009B protected-merge source interface.

No stale collaboration/review ancestry is imported.

## Exact Git checks

For one exact `ChangeProposal`, the adapter requires:

```text
runtime git --version == configured exact version
repository object format == proposal object format
base object type          == commit
proposed object type      == commit
resulting object type     == tree
proposed^{tree}           == proposal.resulting_tree
merge-base --is-ancestor base proposed == success
```

The process runner requires an absolute Git executable and uses a cleared environment with:

```text
GIT_CONFIG_NOSYSTEM=1
GIT_CONFIG_GLOBAL=/dev/null
GIT_TERMINAL_PROMPT=0
GIT_NO_LAZY_FETCH=1
GIT_NO_REPLACE_OBJECTS=1
```

Commands additionally use `--no-replace-objects` where Git object semantics are inspected.

## Evidence compatibility

The following v1 domains are intentionally unchanged from FORGE-008B:

- `mycelix-forge/git-relevant-source-state/v1`
- `mycelix-forge/git-ancestry-evidence/v1`
- `mycelix-forge/git-tree-evidence/v1`
- `mycelix-forge/git-source-verifier-evidence/v1`

This preserves the meaning of the source evidence while moving it onto the repaired collaboration/authority line.

## Two-pass boundary

`observe(...)` produces a raw `ProposalSourceObservationV1`.

FORGE-009B later invokes the same adapter through `ProposalSourceVerifierV1`, causing the Git inspection to execute again and requiring the observation's exact source-state, base/proposed/tree, ancestry evidence and tree evidence to match the second inspection.

The raw observation is therefore not trusted merely because this adapter constructed it once.

## Non-claims

A positive verifier result does not establish:

- portable/self-contained repository closure;
- M0 OfflineEvidence qualification;
- Git executable-byte authenticity by itself;
- repository-tip currentness at merge time;
- one-time merge-request consumption;
- merge authorization;
- merge execution.

M0 Runtime Tool Evidence/NAR closure remains the stronger executable-byte theorem for the hermetic profile.
