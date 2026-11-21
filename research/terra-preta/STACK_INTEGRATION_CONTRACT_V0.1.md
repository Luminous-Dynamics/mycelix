# Terra Preta stack integration contract v0.1

**Status:** research interface proposal; synthetic example only. No cross-repository adapter or validated soil digital twin is claimed.

## Why this integration is worth doing

The programme already has discrete work on field-realized greenhouse gases, seasonal event coverage, South African NIR/MIR metrology, CT-informed matrix–macropore exchange, functional soil-system benchmarking, uncertainty/covariance, and adaptive measurement design. The next architectural step is not to merge every codebase into one model. It is to make observations, model runs, claims, proposals and authority boundaries portable across systems without losing their evidence status.

The 2026 soil–water digital-twin review emphasizes multi-source observation operators, explicit state/parameter/bias uncertainty, physics-based models, data assimilation, tracer constraints, reproducible benchmarking and decision relevance. Its suggested diagnostics include bias/RMSE, ensemble reliability and CRPS, balance closure, ET/recharge constraints, threshold probabilities and decision regret/Value of Information (DOI: [10.3390/w18040440](https://doi.org/10.3390/w18040440)). This contract is a small interoperability layer for those requirements, not a replacement for the scientific methods.

## Responsibility split

| System | Use it for | Do not use it to claim |
|---|---|---|
| **Symthaea** | Generate explicit hypotheses; reason over uncertainty and causal alternatives; propose the next measurement or simulation; compare acquisition policies by information gain, expected regret, cost and risk. | That an internal model score is ground truth; that a proposed measurement or intervention is authorized; that cognitive architecture implies scientific validity. |
| **Symtropy** | Spatial/temporal scene and physical-twin infrastructure; deterministic orchestration/replay where actually supported; coupling/visualization of qualified domain-solver outputs; manufactured mechanics tests. | That a general rigid-body/PBD engine solves Richards flow, dual permeability, reactive transport, soil chemistry or root hydraulics. Those need their own validated solvers and matched benchmarks. |
| **Mycelix** | Identity, custody, source provenance, consent/permissions, access control, signed attestations, governance and auditable approval receipts. | That DHT replication, signatures, consensus or ledger inclusion prove a scientific claim true. Mycelix attests who said/approved what and when, not the truth of the underlying measurement. |
| **Sol-Atlas / Terra Atlas** | Geo-temporal exploration, maps, layered views, scenario comparison and public communication of source-linked observations and model outputs. Reuse its Rust geographic/core and visualization infrastructure where appropriate. | That a rendered layer is a measurement; that interpolation is observation; that simulated values or unverified energy-atlas assumptions are soil evidence. Never invent coordinates or silently upgrade evidence status. |

The soil programme's process models remain domain-specific. HYDRUS and appropriate dual-permeability / reactive-transport references should remain comparators, not be replaced by a generic engine. The 2026 HYDRUS review documents its role and continuing parameterization/process-representation gaps (DOI: [10.1016/j.envsoft.2025.106801](https://doi.org/10.1016/j.envsoft.2025.106801)). The CT-informed exchange study offers a promising mechanism-specific reference term, but its interface area and flux-density provenance must remain separate (DOI: [10.1016/j.jhydrol.2025.133791](https://doi.org/10.1016/j.jhydrol.2025.133791)).

## Package contract

The normative JSON Schema is `schemas/soil-twin-package.schema.json`. The example under `examples/` contains only manufactured values. A package has:

1. **Provenance:** immutable source locator, SHA-256, license/permission basis, source class and method.
2. **Experimental-unit hierarchy:** site → farm/field → block/plot → core/sample/aliquot, with stable IDs, parent IDs, and dependency/grouping keys. A split must operate on connected groups, not random rows.
3. **Observations:** measured or synthetic values with units, timestamp, method, reference method, QC, replicate group, censoring and uncertainty.
4. **Model runs:** system, role, version, code commit, configuration/input/output digests, execution state, scope and qualification state. Build success is not scientific qualification.
5. **Claims:** separate from raw observations, with a stated claim type, estimate, unit, uncertainty, validation design, applicability-domain state, evidence references and limitations.
6. **Experiment proposals:** explicitly proposals, with no field-action authority. The v0.1 contract permits measurement, analysis, simulation and review proposals only.
7. **Attestation:** exactly what Mycelix recorded or verified. Scientific truth is never inferred from an attestation.
8. **Visualization projection:** optional Sol-Atlas layer metadata preserves CRS, spatial and temporal support, interpolation state and uncertainty display. Rendering is visualization-only.

## Fail-closed rules

- Package version, evidence class, source digest, permission basis, experimental-unit links and grouping keys are mandatory.
- Unknown fields are rejected by consumers.
- Observations must resolve to known source and experimental-unit IDs.
- Claims must resolve to known observation/model evidence references.
- Completed model runs require code, configuration, input and output digests. A model is otherwise not reproducible evidence.
- A synthetic package cannot contain field-qualified claims.
- An out-of-domain or unassessed prediction is not silently promoted into a measured fact.
- A Mycelix receipt does not certify truth; an absent or invalid receipt remains visible.
- Symthaea proposals are not actuation requests. `field_action_authorized` is fixed to `false`.
- Sol-Atlas layers do not infer missing coordinates or upgrade the underlying claim status.
- Unknown, incomplete, contradictory or unqualified evidence stays explicitly unknown/incomplete/unqualified.

## How to benchmark the integrated architecture

Do not benchmark the brand names. Benchmark each contribution against a matched reference and ablation:

| Layer | Required comparator / ablation | Primary measurements |
|---|---|---|
| Symthaea experiment design | Random design; space-filling design; greedy one-step design; domain-expert fixed plan; optimal policy on small known-truth cases | Regret, information gain, cost per uncertainty reduction, recovery under model misspecification, calibration, abstention and worst-case performance |
| Soil process model / Symtropy coupling | HYDRUS-1D/2D/3D or appropriate dual-permeability reference; Richards-only ablation; fixed vs dynamic interface; measured-curve interpolation | Water/solute balance residuals, moisture/flux timing, breakthrough, held-out curves, solver stability and runtime |
| Spectral metrology | Published local MIR baseline, global library baseline, PLSR, RF/Cubist and NIR-only/MIR-only/paired fusion | Grouped external MAE/RMSE/bias, interval coverage/width, transfer loss by instrument/site/property and reference-method uncertainty |
| Mycelix evidence layer | Flat files with checksums; signed but unreplicated records; independent verifier; tampering/replay/permission-negative controls | Provenance completeness, verification reproducibility, revocation behavior, custody gaps and unauthorized-mutation rejection |
| Sol-Atlas presentation | Static report/table; raw points; documented interpolation; alternative spatial aggregation | Fidelity to source values, uncertainty visibility, CRS correctness, support-aware aggregation, missingness and provenance click-through |

A model improvement is credible only on frozen, identical, leakage-safe partitions against the strongest valid comparator, with cluster-aware uncertainty and no trade-off hidden in calibration, conservation or a key subgroup. The South African Western Highveld paired NIR/MIR dataset is a particularly valuable local benchmark, but the public article dataset must be acquired and hashed before any full-data result is claimed (DOI: [10.1016/j.dib.2026.113160](https://doi.org/10.1016/j.dib.2026.113160)). The 2026 MIR metrology study also warns that reference-chemistry error can materially reduce apparent model performance and that preprocessing/model choice varies by soil property and library (DOI: [10.1016/j.geoderma.2026.117804](https://doi.org/10.1016/j.geoderma.2026.117804)).

## Implementation sequence

**Now (this contract):** freeze the hand-off vocabulary, schema, synthetic fixture and fail-closed invariants.

**Next, separately reviewable adapters:**
1. Mycelix source/custody receipt adapter and a verifier that does not imply claim truth.
2. Symthaea read-only experiment-design adapter consuming only qualified observations/model summaries and emitting proposal-only outputs.
3. Symtropy adapter for geometry/state/solver metadata and replay receipts; use an external validated Richards/transport solver until an equivalent solver is independently qualified.
4. Sol-Atlas projection adapter that ingests source-linked GeoJSON/rasters and renders uncertainty, missingness, CRS, spatial support and provenance without changing claim status.

**Then:** a prospective, locally governed South African field protocol with independent lab reference methods, event-aware sampling, sensor calibration, control treatments and multi-year follow-up. No software output substitutes for agronomic review, ethics, land access, ecological safeguards, community consent or statutory permissions.

## Evidence boundary

The included example is synthetic and exists only to test the contract. This PR does not integrate any of the other repositories, validate Symthaea or Symtropy for soil science, run a physical model, acquire the Zenodo data, or qualify an intervention.
