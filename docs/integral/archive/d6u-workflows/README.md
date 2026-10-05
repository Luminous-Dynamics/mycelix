# Archived D6U workflow lanes

These workflow definitions are preserved verbatim for historical/audit reference but are intentionally outside `.github/workflows` and therefore are not active GitHub Actions workflows.

The authoritative D6U runtime path is:

`D6S Canonical Qualification`
→ `D6U Exact-Head Runtime Executor` (main-owned, unprivileged)
→ inert evidence artifact
→ `D6U Trusted Evidence Attestation` (trusted privileged root)

Archived sources:
- `d6u-runtime-qualification.yml`
- `d6u-exact-head-runtime.yml`

Do not reactivate either archived workflow as a second qualification lane; doing so would reintroduce duplicate runtime execution and weaken the single-authoritative-path evidence model.
