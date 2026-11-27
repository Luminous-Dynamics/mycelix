# Terra Preta measurement observation contract

This dependency-free Rust crate defines the admission contract for a measurement-aware soil hydraulic benchmark. It keeps measurement protocol, physical support, original method labels, units, uncertainty and canonical sample/site/study group identifiers attached to each observation.

It deliberately does not parse or claim to ingest the external datasets. Their exact file dictionaries and row-level canonical group IDs must be inspected before writing an adapter.

- SoilKsatDB v0.3: the current Zenodo record reports 13,267 Ksat measurements at 1,910 sites. The 2021 ESSD article reports 13,258 values at 1,908 locations, or 13,286 when both field and lab results recorded at the same location are included; its method split is 4,131 field and 9,155 laboratory measurements. Treat these as different version/counting statements and reconcile exact downloaded rows, duplicate-location policy and release before creating folds. The method metadata file preserves method and source details. https://zenodo.org/records/4384592 ; https://essd.copernicus.org/articles/13/1593/2021/
- UNSODA 2.0: about 790 soil samples with measured retention, unsaturated conductivity and diffusivity; USDA/NAL archive distributed by ICPSR as V1 on 2026-06-28, DOI 10.3886/E250569V1. https://www.datalumos.org/datalumos/project/250569/version/V1/view
- SoilKsatDB v0.3: 13,258 Ksat values, 1,908 sites; associated publication reports 4,131 field and 9,155 laboratory measurements. Method metadata are available in the release. https://zenodo.org/records/4384592
- FG/KSAT/6/1358: 1,358 fine-grained saturated-conductivity observations compiled from 33 studies; current Mendeley Data v2 release 2026-10-25. https://data.mendeley.com/datasets/mvpctdnmxw/2
- CG/KSAT/7/1278: 1,278 coarse-grained saturated conductivity measurements from 53 studies, with k, Gs, D10-D90, Cu, Cc and void ratio (some covariates missing). https://data.mendeley.com/datasets/fp8n58ny46/1
- FG/KSAT/6/1358: fine-grained companion database described in the November 2026 data note. https://data.mendeley.com/datasets/mvpctdnmxw/2
- EUPTFv2 + LUCAS: 500 m EU/UK derived hydraulic property map with uncertainty outputs; explicitly registered as a derived map, not independent measurement truth. https://esdac.jrc.ec.europa.eu/content/soil-hydraulic-properties-europe-using-euptfv2-and-lucas

## Core rules

1. Preserve original units and raw method strings. Unit conversion is a separate, tested transformation.
2. Retain the physical support: field/in situ, undisturbed core, repacked core, point-retention sample, map cell or numerical domain.
3. Keep protocol class distinct from support domain.
4. Require canonical grouping IDs for sample-, site- or study-level folds. Missing IDs cause strict folds to fail closed; they are never replaced with row IDs or approximate coordinate matching.
5. All repeated observations sharing a canonical group ID must map to one fold.
6. Derived maps and numerical reference data cannot be admitted as observed measurement truth.
7. Folds are balanced deterministically by record count at the selected group level. Larger groups are assigned first to the currently least-loaded fold; FNV-1a and canonical group ID break ties. This makes the result independent of input row order.
8. The contract rejects fewer independent groups than folds and rejects assignments with an empty fold.

FNV-1a is only a deterministic tie-break, not a security primitive. Balancing record counts does not itself stratify target, protocol, soil class, geography or study era. Those strata need preregistered crosswalks and a separate stratified grouping policy; never split a canonical group to force class balance. The contract does not itself define the final train/test proportions. A real benchmark should freeze fold manifests, source-file digests, crosswalk versions, balancing summaries and exclusion reasons before modelling.

## Intended paradigm

Observation values are not raw labels of a universal scalar. A field tension infiltration estimate, a constant-head core measurement and a PTF map cell have different spatial/temporal supports and protocol operators. The intended hierarchical model separates latent soil response, observation operator, study/protocol effects and measurement error. This contract is the provenance and split layer; it is not yet the hierarchical inference model.

The test suite contains 16 dependency-free unit tests, including count/version reconciliation and order-invariant record-balanced group-fold controls.

## Run

    cargo fmt --manifest-path research/terra-preta/tools/observation-contract/Cargo.toml -- --check
    cargo test --manifest-path research/terra-preta/tools/observation-contract/Cargo.toml
    cargo clippy --manifest-path research/terra-preta/tools/observation-contract/Cargo.toml --all-targets -- -D warnings
