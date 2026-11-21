# Biomass ecological-retention and cascading-use ledger v0.1

**Status:** synthetic source-eligibility and allocation audit. It does not grant harvest rights, define ecological retention percentages, or authorise feedstock extraction.

## Why this is a separate gate

Gross crop or forestry residue is not equivalent to available feedstock. Residues can maintain soil cover, return carbon and nutrients, reduce erosion, moderate water loss, support habitat and already supply feed, bedding, fuel, mulch, fibre or community use. Removing them has an ecological and opportunity cost.

Commission Delegated Regulation (EU) 2026/285 under the EU Carbon Removals Certification Framework states that biochar carbon-removal activity should not create unsustainable biomass demand and should follow cascading use (Recital 8). This is a relevant carbon-removal methodology comparator, not a universal South African harvesting rule: https://eur-lex.europa.eu/eli/reg_del/2026/285/oj/eng

## Source-level balance

For each source lot and season, compute a conservative interval:

`gross dry mass - ecological retention - existing competing uses - prior batch allocations`

Every term has explicit uncertainty. The lower available-surplus bound is formed using the low gross-mass bound and high retention / use / prior-allocation bounds. The upper bound uses the opposite conservative sides.

An external source-specific seasonal harvest cap applies independently. Total new allocations plus prior allocated mass cannot exceed it.

## Eligibility

- verified rights / consent, approved ecological retention assessment and an approved harvest cap are required;
- prohibited rights or ecological removal status blocks the source;
- unknown rights, missing retention evidence or absent cap leave the source unresolved;
- an allocation passes only when the requested total is below both the conservative available-surplus lower bound and remaining approved cap;
- if the request definitely exceeds the available/cap upper side, it is blocked;
- if the uncertainty interval overlaps the request, it is unresolved rather than rounded into a pass;
- existing uses and prior batch allocations are deducted before new allocations are evaluated.

No universal residue-removal percentage is built in. Retention floors and harvest caps are externally set for the local crop/forest, soil, slope, rainfall, land-use and rights context by qualified authorities and rights holders.

## Nutrient export

The ledger calculates the mass of C, N, P and K removed from the source in the requested dry biomass quantity, using the declared source nutrient content per dry tonne and its uncertainty. This is a source-removal ledger, not proof that these nutrients would all have remained plant-available in the soil or that their loss is compensated by another input.

## Anti-double-counting

All batch allocations for a source/season are summed before eligibility is assessed. A single allocation ID cannot repeat; source IDs and batch IDs remain explicit. A source-level seasonal cap is enforced against the total allocation rather than each batch independently.

## Synthetic fixture

The manufactured example has a source inventory, retention floor, competing uses, prior batch allocation, source-level harvest cap and three requested batches. It is designed to show a pass within conservative surplus, a second allocation blocked by the source cap, and an unknown retention status remaining unresolved.

All quantities and approvals in the fixture are fictional; none are South African residue thresholds or harvesting permits.

## Proposed qualification targets (not achieved)

- zero double-allocation across batches;
- zero source eligibility when rights or ecological retention are unknown;
- zero pass when requested allocations exceed conservative available surplus or the source-level cap;
- source C/N/P/K export retained separately from soil nutrient balance;
- full source/season/approval digest continuity into the downstream biochar batch passport;
- uncertainty and unresolved coverage reported, not converted to assumed surplus.

## Boundaries

This is an accounting and evidence contract. It is not a harvest permit, environmental approval, community-consent substitute, feedstock procurement instruction or biochar application recommendation. Real assessments require local rights holders, qualified agronomic/ecological authorities and source-specific seasonal monitoring.