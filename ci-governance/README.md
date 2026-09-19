# CI-GOV-001E-A — workflow trust-policy checker

This tranche is a read-only, deterministic foundation for CI-GOV-001E. It does **not** mutate repository settings, cancel workflow runs, or change the frozen ASSURE-002B product subject.

## v0.1 policy

The checker requires:

- workflow-level `permissions: contents: read`;
- no effective `*: write` permission in any job;
- `changes` to opt into only `contents: read` plus `pull-requests: read`;
- `test-assurance` and `ci-pass` to remain `contents: read`;
- every remote action in `changes`, `test-assurance`, and `ci-pass` to be pinned to a full 40-hex commit;
- every `actions/checkout` in those critical jobs to set `persist-credentials: false`.

The parser intentionally supports only the small YAML surface needed for these checks. Ambiguous security-relevant constructs such as YAML anchors/aliases or inline permission maps fail closed rather than being guessed.

## Local verification

```bash
python3 -m unittest -v ci-governance/test_check_workflow_trust.py
python3 ci-governance/check_workflow_trust.py .github/workflows/ci.yml \
  --policy ci-governance/trust-policy-v0.1.json
```

The first command verifies the checker against frozen positive and adversarial fixtures. The second audits a real workflow. A failing audit is a policy finding, not product-test failure.

## Deliberate sequencing

001E-A freezes the checker and negative corpus first. A later 001E-B may apply the policy to `.github/workflows/ci.yml` only after the current ASSURE-002B final-tree boundary is resolved, so generic CI hardening does not silently change an evidence subject already waiting for execution.

## Nonclaims

Passing this checker means only that the inspected workflow text conforms to the declared token/action trust policy. It does not establish product correctness, scientific validity, branch-protection enforcement, absence of GitHub platform compromise, or correctness of third-party action code.

## Payload identity

`CI_GOV_001E_A.lock.json` binds the exact checker, policy, test corpus, README, and lock-checker bytes to the scheduler-qualified `main` parent. Verify it with:

```bash
python3 ci-governance/check_payload_lock.py
```

The lock's manifest digest is domain-separated and canonicalized over the sorted path/SHA-256 set. The lock file itself is excluded from its payload to avoid a circular hash dependency.
