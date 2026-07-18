# Marketplace deployment profiles

`backend/happ.yaml` is the base Marketplace development profile. It contains an
active `marketplace` role and a deferred optional `identity` role. It is not a
settlement-capable release profile.

A settlement-capable package must be rendered from exact built DNA bundle paths:

```sh
python3 scripts/render-marketplace-deployment.py \
  --marketplace-dna /absolute/path/mycelix_marketplace.dna \
  --finance-dna /absolute/path/mycelix_finance.dna \
  --identity-dna /absolute/path/mycelix_identity.dna \
  --output /tmp/mycelix_marketplace_settlement.happ.yaml
```

The renderer refuses missing or non-`.dna` paths. The resulting manifest makes
both `marketplace` and `finance` active roles. Identity remains optional and is
provisioned deferred when supplied.

Packaging a Finance role is necessary but not sufficient for promotion. Runtime
`AppInfo.cell_info` must report both required roles as active, and the lifecycle,
arbitration, and settlement evidence scenarios must pass against the same build.
