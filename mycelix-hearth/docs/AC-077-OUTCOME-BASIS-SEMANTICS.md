# AC-077: DecisionOutcome Semantic Basis Binding

## Purpose

AC-072 records which exact Decision action/version a new outcome resolved. AC-077 makes that basis authoritative for the outcome's bounded semantic fields.

## Invariants

For a new `DecisionOutcome`:

```text
chosen_option < basis.options.len()
quorum_bp == basis.quorum_bp
```

This prevents an outcome from referencing a nonexistent option or carrying a quorum snapshot that contradicts the Decision version it claims to resolve.

## Why the basis is authoritative

The exact Decision action is the schema snapshot observed by the resolver. Using it as the validation dependency keeps the outcome self-consistent without relying on mutable collection queries or current UI state.

## Boundary

These checks establish structural/semantic consistency only. They do not prove that the option was actually the winning tally, that participation was honestly reported, or that the decision itself was legitimate.

AC-071 handles competing outcome candidates.
AC-072 binds outcomes to exact Decision revisions.
AC-073 resolves current Decision revisions deterministically.
AC-075 gives Finalized precedence over Closed.