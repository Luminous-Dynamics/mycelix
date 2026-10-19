# Lifetime Product Record — Existing Owner Crosswalk (Draft)

**Status:** Integration review required  
**Purpose:** Prevent the LKI-01 research record and its validator from becoming a competing product, BOM, qualification, supplier-provenance, or repair ledger.

The draft product record is an **aggregate engineering view**. It must point to existing authoritative records where those owners exist. Its embedded BOM/component/supplier fields are provisional read-only snapshots for evaluating the engineering interface, not a second canonical system of record.

## Owner mapping

| Draft concept | Intended authoritative owner | Rule for this draft |
|---|---|---|
| Product family / design revision | Mycelix physical product/design revision contract, [mycelix#3084](https://github.com/Luminous-Dynamics/mycelix/issues/3084), and Fabrication design records | Treat product_id as a model/family reference; do not use it as a manufactured-unit serial. revision must eventually reference an immutable artifact-set revision. |
| CAD, drawings, parametric source and licence manifest | Existing Mycelix Fabrication design/artifact provenance | This record may link to exact artifacts/digests; it must not become another artifact store or licence registry. |
| BOM and subassemblies | Existing [Mycelix Manufacturing BOM coordinator](https://github.com/Luminous-Dynamics/mycelix/blob/main/mycelix-manufacturing/zomes/bom/coordinator/src/lib.rs) | Embedded bom.items is a derived snapshot for offline audit only. Production integration should resolve the canonical BOM identity/revision and preserve a source digest; BOM creation/explosion remains with Manufacturing. |
| Supplier, batch, origin and chain-of-custody evidence | Existing Mycelix Supply Chain / supplychain provenance | The draft supplier records are candidate-evidence snapshots, not proof of current inventory, price, capability, legal permission, or independent qualification. Do not create a second custody ledger. |
| Manufactured unit / serial / actual configuration | Mycelix generic physical-product identity, [mycelix#3084](https://github.com/Luminous-Dynamics/mycelix/issues/3084), and appliance profile [mycelix#5156](https://github.com/Luminous-Dynamics/mycelix/issues/5156) | A design revision is not a manufactured unit. This draft contains no unit serial or production-yield authority. |
| Functional qualification and conformity | Exact qualified profile and competent external/market-specific authority; Mycelix evidence references | A release gate can refer to evidence but cannot issue regulatory certification. A test of one unit is not batch qualification or independent replication. |
| Repair work order and used parts | Existing Mycelix Manufacturing work-order/material events | Planned BOM items are not evidence that parts were actually consumed/replaced. Do not create a second work-order state machine. |
| Restored capability and recurrence | [RepairProductiveLoopV1 owner, mycelix#3313](https://github.com/Luminous-Dynamics/mycelix/issues/3313), and [validator, mycelix#3318](https://github.com/Luminous-Dynamics/mycelix/issues/3318) | A completed repair job is not restoration evidence; one successful post-check is not recurrence-free operation. This draft has no repair event authority. |
| Repair task graph / service procedure | Existing Symtropy repairability compiler/graph work, [symtropy#1773](https://github.com/Luminous-Dynamics/symtropy/issues/1773) | The product record references a procedure revision; task dependency and safety-isolation semantics remain owned by the qualified repair-task graph. |
| Appliance profile and portfolio qualification | [Symtropy appliance qualification #1805](https://github.com/Luminous-Dynamics/symtropy/issues/1805), [portfolio #1825](https://github.com/Luminous-Dynamics/symtropy/issues/1825), and Mycelix [#5156](https://github.com/Luminous-Dynamics/mycelix/issues/5156) | Use the staged lower-risk appliance pilot; the combined refrigerator/cooktop remains a later, higher-hazard configuration. |
| Candidate selection, uncertainty and next test | Symthaea physical-product pilot portfolio [#5752](https://github.com/Luminous-Dynamics/symthaea/issues/5752) and its existing experiment/decision owners | Symthaea may propose candidates and tests; model output is not a physical measurement or safety approval. |
| Spatial placement / room accessibility | Symtropy spatial design [#1766](https://github.com/Luminous-Dynamics/symtropy/issues/1766) and existing exact-geometry/accessibility owners | This draft describes appliance/module envelopes, not a competing room geometry or accessibility authority. |
| Regional material and repair capability | Sol-Atlas material context [#159](https://github.com/Luminous-Dynamics/sol-atlas-leptos/issues/159) and repair-workcell continuity [#161](https://github.com/Luminous-Dynamics/sol-atlas-leptos/issues/161) | A map point is not proof of qualified capacity, stock, grade, or service performance. Use date-stamped, licensed, uncertainty-aware context. |

## Promotion gate before adopting this schema

Do not promote the draft to canonical or consume it as a production API until a cross-project review has:

1. Mapped every identity and lifecycle field to an existing owner or documented the precise missing schema boundary.
2. Replaced duplicated embedded BOM/supplier data with authoritative references plus digest-bound derived snapshots where needed.
3. Confirmed the exact unit/design/BOM/qualification separation from #3084 and the repair/restoration semantics from #3313/#3318.
4. Added schema-migration, unknown-field, compatibility, revocation/supersession and source-licensing rules.
5. Executed the Rust validator on a pinned toolchain/dependency lock and passed deliberate mutation/negative-control fixtures.
6. Reviewed applicable safety standards and jurisdiction-specific legal requirements with competent professionals.
7. Kept all claims bounded to the exact configuration, protocol, evidence and validity period.

Until then, the only valid status is **draft integration contract**. A passing local audit cannot make the record canonical, qualify the product, or override the current schema owners.
