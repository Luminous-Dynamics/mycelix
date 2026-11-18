# Mycelix Property

**Land, housing, and asset registries for the Mycelix Civilizational OS**

## Overview

Mycelix Property provides application-level records and coordination for assets, agreements, transfers, disputes and commons management. For land and natural-resource use, it is an internal evidence and workflow system; it is not a national cadastral/deeds registry, adjudicator or source of legal title.

> [!IMPORTANT]
> **Mycelix Property is an application-level ledger, not a national deeds registry or a source of legal title.** Its Property, Transfer and TitleDeed entries describe state recorded inside Mycelix. A completed internal transfer or generated TitleDeed does not, by itself, prove registration, legal effectiveness, transferability of customary/informal rights, valid community consent, or the absence of competing claims. Those questions remain governed by applicable external law, competent institutions and affected communities. See [PROPERTY_LEGAL_EFFECT_BOUNDARY.md](./PROPERTY_LEGAL_EFFECT_BOUNDARY.md) before using the land/resource flows for real-world decisions.


## Zomes

### registry
Internal application records (not proof of external registration):
- Property registration with geospatial anchoring
- Title deed management
- Ownership history chain
- Fractional ownership support
- Property metadata and attachments

### transfer
Internal transfer workflow with escrow (not proof of a legally effective land transfer):
- Transfer initiation and acceptance
- Multi-party escrow
- Condition verification
- Conditional internal completion (external legal effect is not inferred)
- Rollback on failure

### disputes
Competing claim resolution:
- Boundary disputes
- Ownership challenges
- Lien management
- Integration with mycelix-justice

### commons
Shared resource management:
- Common pool resources
- Usage rights and quotas
- Maintenance responsibilities
- Benefit distribution
- Governance rules

## Architecture

```
mycelix-property/
├── dna/
│   └── dna.yaml              # DNA manifest
├── zomes/
│   ├── registry/
│   │   ├── integrity/        # Internal record validation
│   │   └── coordinator/      # Internal registry projection
│   ├── transfer/
│   │   ├── integrity/        # Transfer validation
│   │   └── coordinator/      # Escrow and internal workflow completion
│   ├── disputes/
│   │   ├── integrity/        # Dispute validation
│   │   └── coordinator/      # Claim resolution
│   └── commons/
│       ├── integrity/        # Commons validation
│       └── coordinator/      # Resource management
├── client/                   # TypeScript client
└── tests/                    # Integration tests
```

## Property Types

- **Land**: Parcels with geospatial boundaries
- **Buildings**: Structures with addresses
- **Units**: Apartments, condos, offices
- **Equipment**: Vehicles, machinery
- **Intellectual**: Patents, copyrights, trademarks
- **Digital**: NFTs, domains, credentials

## Integration Points

- **mycelix-identity**: Owner DID verification
- **mycelix-finance**: Mortgages and liens
- **mycelix-justice**: Dispute escalation
- **mycelix-governance**: Zoning and land use
- **mycelix-energy**: Energy asset registration

## Building

```bash
# Build all zomes
cargo build --release --target wasm32-unknown-unknown

# Package the hApp
hc app pack .
```

## License

Apache-2.0
