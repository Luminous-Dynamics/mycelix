# Risk-Tiered Scientific Credential Governance

**Protocol:** `mycelix-desci-credential-governance` v1, additive action set
**Status:** experimental, append-only authority layer

Mycelix-DeSci no longer treats every credential-governance action as equally risky. Each proposal is classified when it is accepted and snapshots the quorum rule that must remain satisfied at execution.

## Risk tiers

| Tier | Typical actions | Intended control |
|---|---|---|
| `routine` | ordinary actor registration, organization membership grant | baseline threshold and delay |
| `sensitive` | actor-key authorization, ordinary role changes, acceptance-key authorization, witness enrollment | stronger threshold, organization diversity, longer delay |
| `critical` | administrator grants/removals, key compromise or revocation, authority-key revocation, governance-policy changes, risk-policy changes, witness revocation, witness-compromise intervals | strongest threshold, broadest organization diversity, longest delay |

Classification is deterministic code, not caller input. A client cannot label an administrator removal as routine.

## Policy shape

```json
{
  "type": "update_governance_risk_policy",
  "policy": {
    "routine": {
      "approval_threshold": 2,
      "minimum_distinct_organizations": 1,
      "activation_delay_seconds": 3600
    },
    "sensitive": {
      "approval_threshold": 3,
      "minimum_distinct_organizations": 2,
      "activation_delay_seconds": 86400
    },
    "critical": {
      "approval_threshold": 4,
      "minimum_distinct_organizations": 3,
      "activation_delay_seconds": 259200
    }
  }
}
```

Thresholds, organization requirements, and delays must be monotonic from routine through critical. The critical delay must remain shorter than the base proposal lifetime.

The first risk policy is installed through the existing threshold policy. After installation, every later risk-policy change is itself critical and is evaluated under the currently active risk policy.

## Proposal snapshots

An accepted proposal records:

- its deterministic risk tier;
- required unique active administrators;
- required distinct administrator organizations;
- server-derived activation time;
- bound credential-registry head;
- action hash and expiry.

Later policy changes cannot weaken an already open proposal. At execution, approvals are re-evaluated against the historical credential state at execution time. Administrators whose role or active keys were revoked no longer count.

## Organization diversity

Approvals are still unique by actor. Organization diversity is an additional condition:

- one actor contributes at most one organization;
- multiple administrators from the same organization contribute one diversity slot;
- an administrator with no organization can approve but contributes no organization slot;
- the canonical lowest organization identifier is used for multi-affiliated actors, preventing one actor from filling multiple slots.

This is a conservative initial model. A future protocol may introduce a governed primary-governance-organization credential rather than deriving it from memberships.

## Readiness

Readiness fails until a risk policy has been governed into the journal. This is intentional: the base threshold remains a bootstrap mechanism, not a complete production authority policy.

## Remaining limitations

- Risk classification is code-defined rather than separately versioned as a policy artifact.
- Organization identity is asserted by the credential registry; independent organizational credential verification remains future work.
- Emergency compromise response uses the critical proposal path and has no separately constrained break-glass quorum. Compromise intervals must be historical at execution and cannot pre-authorize restoration.
