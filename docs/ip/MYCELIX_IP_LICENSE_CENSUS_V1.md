# Mycelix IP / License Reconciliation Census v1

Status: Initial diligence census  
Tracking: #884 (`MYC-IP-001`)

## Purpose

Record license declarations that must agree before investor/customer diligence.

This file is a reconciliation aid, not legal advice or a representation of freedom to operate.

## Initial verified discrepancies

| Path | README declaration | Workspace/manifest declaration | Repository schedule | Current disposition |
|---|---|---|---|---|
| `mycelix-identity/` | Apache 2.0 | AGPL-3.0-or-later | AGPL-3.0-or-later | README stale; reconcile to exact local LICENSE/manifest authority |
| `mycelix-finance/` | Apache-2.0 | AGPL-3.0-or-later | AGPL-3.0-or-later | README stale; reconcile to exact local LICENSE/manifest authority |

## Required full census

For each cluster/shared crate, record:

```text
path
LICENSE file/profile
Cargo/workspace SPDX declaration
README declaration
repository LICENSING.md classification
copyright/contributor provenance status
commercial-relicense authority status
exceptions / third-party code notes
```

## Fail-closed rule

When declarations disagree, do not choose the most permissive interpretation for commercial diligence.

Reconcile against the exact directory license file, manifest metadata, repository policy and actual copyright/provenance evidence.

## Follow-up

The complete MYC-IP-001 tranche should cover all commercial/product-relevant paths, fix stale documentation, reconcile shared-crate wording, address currently unspecified paths, and introduce a drift check where practical.
