# mycelix-business-revalidation

Execution-time revalidation contracts for the Mycelix Business Fabric.

Freshness alone is not enough for consequential actions. This crate distinguishes:

- **Exact frontier** — required for policy and authority continuity; any material version/digest change forces re-preparation.
- **Dependency witness** — an authoritative domain attests that the specific dependency needed by the action still holds, allowing unrelated concurrent state changes.
- **Freshness-only observation** — permitted only for observation-class inputs when the Action Contract explicitly accepts bounded staleness.

Policy and authority can never use the weaker dependency/freshness modes. The crate also re-checks the prepared action's authority epoch and expiry at execution time.

A successful result is only `RevalidatedForAttempt`: it is not a new authority grant and carries no mutation capability.
