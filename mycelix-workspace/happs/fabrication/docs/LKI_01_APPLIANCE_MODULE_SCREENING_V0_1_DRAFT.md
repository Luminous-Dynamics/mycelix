# LKI-01 Appliance Module Screening Note v0.1

**Status:** Engineering reference only; no candidate selected or locally qualified  
**Checked:** 2026-10-19  
**Target:** LKI-01 Lifetime Kitchen Island, future physical demonstrator  
**Important:** This document does not assert current South African stock, an active local model listing, supplier pricing, service availability, or appliance safety approval.

## 1. Immediate lesson: design around qualified module envelopes

A product family should not be built around a fictional universal refrigerator size. Real integrated appliances have manufacturer-defined chassis dimensions, cavity openings, ventilation/duct requirements, mounting details, panel weights, electrical access, and removal paths. These dimensions are not interchangeable. A model that fits the visible cabinet opening can still be incompatible with required exhaust ducting, drawer travel, door/panel geometry, electrical access, or safe servicing.

The current LKI-01 JSON example uses a **hypothetical placeholder** for its cold-module envelope and bay. It is not dimensioned to the manufacturer example below and must not be treated as a fit claim.

## 2. Reference A — Fisher & Paykel Series 9 Integrated CoolDrawer, RB90S64MKIW1

Primary source: [manufacturer planning guide / data sheet](https://www.fisherpaykel.com/on/demandware.static/-/Sites-fpa-master-catalog/default/dw52ca4c8d/DesignPlanning-FisherPaykelCHN/FP-PlanningGuide-en-RB90S64MKIW1-IntegratedCooldrawer-0-90005463C-AU-NZ-CN.pdf) and [manufacturer installation instructions](https://www.fisherpaykel.com/on/demandware.static/-/Sites-fpa-master-catalog/default/dwfd9bb7f0/InstallationManuals-FisherPaykelCHN/FP-InstallGuide-RB90S64MKIW1-IntegratedCoolDrawer-QuickRef-CN-840714.pdf).

The published source describes the RB90S chassis at approximately:

| Dimension / requirement | Published value | Engineering interpretation |
|---|---:|---|
| Chassis width | 855 mm | Larger than the illustrative 600 mm module placeholder |
| Chassis height | 640 mm | Does not include every custom panel/installation allowance |
| Chassis depth | 557 mm | The planned cavity is deeper once installation constraints are included |
| Overall cabinetry-frame width | 900 mm | This is a cabinet/frame class, not the actual clear inside width |
| Minimum inside cabinetry width | 864 mm | Use clear internal dimensions after rails, liners and obstructions |
| Internal height to spacer block | 644 mm | Installation geometry depends on the selected outlet duct and cabinetry setup |
| Minimum distance from cabinetry front datum to rear wall | 560 mm | Confirm exact installation drawing and air/duct layout |
| Cabinet support | 100 kg combined product + food load | Check complete load path, shelf/bearer design and anchorage |
| Electrical access | Switched outlet outside the drawer cavity and accessible after installation | Electrical isolation/replacement must not require destructive cabinet work |

The installation instructions explicitly say the product must remain removable from the enclosure for service and must not be sealed into cabinetry with silicone or glue. They also warn that installation requires competent mechanical and electrical skills and that the cabinet must be securely anchored. The manual warns that the model shown may not be available in all markets and instructs readers to verify country-specific availability with the manufacturer/dealer.

**Status:** Useful design reference for dimensional and serviceability constraints only. It is not selected for LKI-01 and has not been verified as currently available or serviceable in South Africa.

## 3. Reference B — Sub-Zero integrated drawer installation guide for South Africa

Primary source: [Sub-Zero/Wolf South Africa integrated refrigeration installation guide](https://subzero-wolf.co.za/wp-content/uploads/2021/07/Integrated-Ref-Install-Guide-2.pdf).

This guide shows integrated drawer opening classes at 610, 762 and 914 mm widths, with a listed drawer opening height of 876 mm and an opening depth of 635 mm for the relevant illustrated family; it gives a 220–240 V AC supply requirement. Exact dimensions and installation constraints depend on the model and its current installation guide. A family-level guide must not be used to approve a specific SKU or to assume that the SKU is locally stocked, current, serviceable, or compatible with the CoolDrawer interface.

**Status:** Secondary dimensional reference pending exact SKU/model identification, current local written quotation, service-parts policy, and qualified installation review.

## 4. What these sources change about LKI-01

1. **Do not freeze one 600 mm generic module bay.** Choose a real candidate based on target-market availability and service support, retrieve its exact current datasheet/installation guide, then create the module interface from those artifacts.
2. **Separate chassis, cavity, face panel and service envelopes.** The manufacturer chassis envelope is not the same thing as the clear internal bay, the finished visible panel, an exhaust-duct envelope, or the technician's access/removal space.
3. **Treat required airflow and ducting as hard constraints.** A module must not be sealed in, have its inlet/outlet blocked, or have its required outlet duct removed to make the concept look cleaner.
4. **Make removal a physical requirement.** Demonstrate the removal path, cord/connector protection, isolation access, tooling and a timed service trial on the actual cabinet fixture. “The manual says it is removable” is not evidence that our integrated assembly preserves that removability.
5. **Design around the actual load path.** Include appliance mass, full food load, drawer-extension forces and foreseeable misuse; verify the support structure and floor anchorage with appropriate engineering review.
6. **Keep target-market qualification separate from web discovery.** Before procurement, obtain a written, current local quote; confirm exact SKU/model, voltage/frequency, installation revision, lead time, warranty terms, local service channel, repair parts and promised availability window.
7. **Do not combine refrigerator and cooktop in the first powered test.** Validate an unpowered structure and one independently installed cold-storage module first. The cooktop needs a separate thermal, electrical, spill and ventilation assessment before any combined energized configuration.

## 5. Required evidence before accepting a candidate

A candidate can move through these states, without skipping:
- `discovered`: model or family found in a source; nothing implied about availability.
- `datasheet_reviewed`: exact model's current technical and installation documents retrieved and revision recorded.
- `local_availability_confirmed`: written local quote or supplier confirmation tied to the exact SKU/date; not yet a safety qualification.
- `sample_inspected`: exact supplied unit/model checked against labels, documents and physical condition.
- `interface_tested`: fit, installation access, removal path and scoped operational tests recorded with methods and digests.
- `qualified_for_declared_scope`: competent review approves the exact model, product revision, installation configuration and requirements in scope.
- `rejected` or `stale`: preserve reason/history; never silently turn stale evidence into a current candidate.

No stage in this list by itself establishes appliance safety certification or 50-year product life.

## 6. Integration with the four-repository workflow

- **Mycelix:** retain the exact product/BOM revision and evidence/source digests; authoritative BOM, unit, supplier custody and repair records remain with their existing owners.
- **Symtropy:** check the declared geometry, module clear envelope, drawer motion, access and removal path against a versioned fixture. This does not replace structural, airflow/thermal, electrical or safety qualification.
- **Symthaea:** compare feasible candidates on serviceability, energy evidence, lifetime cost, local support, uncertainty and resource impact; reject candidates that fail hard requirements rather than compensating via a score.
- **Sol-Atlas:** map sourced, date-stamped regional material/fabrication/service context only when the source and spatial coverage are explicit. A mapped supplier location does not prove current stock, capacity or a parts commitment.

## 7. Research references

- Fisher & Paykel, RB90S64MKIW1 Integrated CoolDrawer planning guide and data sheet: https://www.fisherpaykel.com/on/demandware.static/-/Sites-fpa-master-catalog/default/dw52ca4c8d/DesignPlanning-FisherPaykelCHN/FP-PlanningGuide-en-RB90S64MKIW1-IntegratedCooldrawer-0-90005463C-AU-NZ-CN.pdf
- Fisher & Paykel, RB90S/RB36S integrated CoolDrawer installation instructions: https://www.fisherpaykel.com/on/demandware.static/-/Sites-fpa-master-catalog/default/dwfd9bb7f0/InstallationManuals-FisherPaykelCHN/FP-InstallGuide-RB90S64MKIW1-IntegratedCoolDrawer-QuickRef-CN-840714.pdf
- Sub-Zero/Wolf South Africa, integrated refrigeration installation guide: https://subzero-wolf.co.za/wp-content/uploads/2021/07/Integrated-Ref-Install-Guide-2.pdf

These documents establish manufacturer-published dimensions/instructions for the named reference families. They do not establish present local inventory or qualify a proposed LKI-01 assembly.
