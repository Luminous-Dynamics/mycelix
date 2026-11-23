# Dynamic root–soil network topology reference v0.1

**Status:** synthetic known-truth graph evaluator. It is not a hydraulic solver, CT segmenter, empirical root classifier or field-validated network model.

## Motivation

A 2026 synthesis of 330 studies identifies root architecture, root mortality/decomposition, pathway connectivity, soil heterogeneity, antecedent wetness and water input as interacting controls on root-induced preferential flow. It reports four non-exclusive structural expressions and notes that direct evidence connecting root provenance, three-dimensional geometry, connectivity and flux remains limited (Qian et al., 2026, DOI https://doi.org/10.1016/j.earscirev.2026.105734).

A 2026 review also describes root channels as an evolving network and emphasizes that fine-root network topology and interconnection can matter beyond individual channel capacity (Agarwal et al., 2026, DOI https://doi.org/10.1002/vzj2.70148).

The earlier root transport reference #6568 models independent ordered paths. This increment adds shared branching and rejoining topology. The static CT pore/throat graph and threshold-sensitivity scope remains in #5548.

## Input graph

- 3D node coordinates and undirected edges with endpoint identity, segment length and origin class.
- Separate source and sink terminal sets.
- Repeated, strictly ordered snapshots with one state per edge.
- Root origin is distinct from root occupancy; root occupancy is distinct from hydraulic state.
- Each edge is explicitly active, inactive or unassessed.

Every edge length must be at least the Euclidean chord between its declared 3D endpoints. This is a structural consistency check, not evidence that the geometry was reconstructed correctly.

## Two-valued observability envelope

For each snapshot, two graphs are evaluated:

- **Confirmed/lower-bound graph:** only edges marked active.
- **Possible/upper-bound graph:** active plus unassessed edges.

For every source/sink pair:

- confirmed connected means an active-only path exists.
- indeterminate unassessed edge means no active-only path exists, but a path exists when unknown edges are admitted as possible.
- confirmed disconnected means even the possible graph has no path.

This prevents an unknown edge from being silently converted into an inactive edge or a confirmed conduit. The pair statuses are bounds on topological reachability, not probabilities.

## Topology and single-element fragility

For both the confirmed and possible graphs, the report records:

- connected-component count, including isolated declared nodes;
- cycle rank, E - V + C;
- confirmed/possible source-to-sink pair counts.

On the confirmed graph, it also tests every active edge and every nonterminal node under single-element removal. A listed critical edge/node is one whose removal reduces the number of confirmed source/sink pairs. This is a deterministic single-failure sensitivity audit, not a probabilistic reliability estimate or a minimum-cut solver.

The v0.1 graph is undirected. It does not infer flow direction or quantify hydraulic resistance. Edge-disjoint path count, weighted minimum cuts, event-specific activation and pore-throat percolation are future comparisons.

## Known-truth fixture

The fixture contains two routes between shared inlet/outlet junctions:

1. Initially both routes are active; the graph contains one independent cycle, while the inlet and outlet trunks are single-element critical.
2. One branch becomes inactive; the other still preserves source-to-sink reachability.
3. The remaining branch includes an unassessed edge; the source-to-sink result becomes indeterminate rather than confirmed connected or disconnected.
4. Both internal routes become inactive; the source/sink pair is confirmed disconnected.

The lower route retains legacy-root provenance even while unoccupied. The outlet edge is an abiotic crack. This is manufactured truth only.

## Reproducibility

The CLI accepts an input JSON and a new output path. It hashes the exact input bytes with SHA-256, emits deterministic ordered results, and refuses to overwrite an existing report. The focused workflow checks formatting, tests, strict Clippy, Draft 2020-12 schemas, key known-truth expectations and no-overwrite behavior.

## Comparison plan

Integrate the graph after the source-aware CT extraction and pore-network checks in #5548, but before physical solver qualification in #5602/#5650. Compare raw geometric graph and segmentation-threshold sweeps, static geometric connectivity, event-specific confirmed/possible active topology, single-domain Richards and dual-permeability baselines where their assumptions match, and HYDRUS DualPerm, MACRO and pore-network solvers only with matched constitutive inputs, boundary conditions and observation support.

Hold out complete soil profiles, sites, plant/root-functional groups and hydrologic events. Score false source-to-sink certificates, abstention/indeterminate rate, topology metric accuracy, tracer arrival/peak/tail errors, water/solute residuals, interval calibration and full measurement/compute cost. Unsupported mechanisms remain unsupported rather than being scored as if represented.

## Claim ceiling

All v0.1 values are synthetic. The reference tests graph algorithms and evidence-state semantics only. It does not prove that a CT edge is a water-filled path, establish root origin from morphology, estimate conductivity/flux, predict tracer breakthrough, or support agronomic recommendations or field actions.

## Proposed benchmark targets (not achieved)

- 100% agreement with manufactured truth for component counts, cycle rank, critical single edges/nodes and terminal reachability.
- Zero false confirmed connections due to unassessed edges.
- Zero false disconnections where an unassessed edge still permits a topological route.
- Deterministic replay and stable input digest.
- On real CT/tracer data, no physical claim until segmentation, event activation and independent transport observations are jointly qualified.

## 2026 comparator landscape and benchmark discipline

The new graph layer should be evaluated alongside—not conflated with—emerging process-connectivity methods:

- **Temporal soil-moisture connectivity:** a 2026 headwater-catchment framework combines temporal similarity (including Dynamic Time Warping) with spatial distance to estimate hydrological process connectivity and compare it with structural connectivity (DOI [10.1029/2025WR042599](https://doi.org/10.1029/2025WR042599)). This is a relevant event/process-level comparator for sensor networks, not ground truth for pore-level graph topology.
- **Spatial-attention time-series prediction:** a 2026 LSTM+attention study across 19 USCRN stations reports median Nash–Sutcliffe efficiencies of 0.91 at 50 cm and 0.93 at 100 cm, with dynamic attention changes during preferential-flow periods (DOI [10.1029/2025WR043099](https://doi.org/10.1029/2025WR043099)). Reproduce its observation task and grouped split where feasible. Predictive attention is a model diagnostic, not direct proof that a particular imaged pore carries water.
- **Root-origin / 3D pore topology:** the 2026 root-channel review (DOI [10.1002/vzj2.70148](https://doi.org/10.1002/vzj2.70148)) identifies the evolving pathway network as central and emphasizes the limitations of dye/image evidence. The 330-study review (DOI [10.1016/j.earscirev.2026.105734](https://doi.org/10.1016/j.earscirev.2026.105734)) distinguishes multiple non-exclusive root-flow expressions. These justify an origin-aware, volumetric, time-dependent evaluation, not an assumption that our graph implementation already exceeds field SOTA.

### Keep the leaderboard task-specific

| Task | Strongest matched baselines | Primary score |
|---|---|---|
| 3D graph reconstruction | Known graph; PoreSpy/SNOW2 extraction; segmentation-threshold sweep | Edge precision/recall, component/cycle-rank error, percolating-component recovery |
| Event activation | Static geometry; moisture-response SFSI/HCI; spatial-attention sequence model | Activation timing, connectivity calibration, unknown-state interval coverage |
| Flow / tracer response | Richards single-domain; HYDRUS DualPerm; MACRO; qualified pore-network models | Arrival/peak/tail timing, breakthrough distribution, water/solute residual |
| Decision support | Fixed cadence; expert schedule; random/stratified sampling; generic active sensing | Regret / missed-event loss at matched budget and decision-risk constraint |

Only compare models on the same physical question, with matched boundary/initial conditions, constitutive inputs, observation support and evaluation groups. A model without support for a mechanism is **not applicable**, not a poor score and not silently approximated.

### High-value target gates (targets, not results)

- **Synthetic topology:** exact reconstruction of all manufactured components, cycles, terminal reachability and critical single elements; reject every 2D-only or unassessed-to-confirmed promotion.
- **Empirical connectivity:** report a calibrated lower/upper connectivity envelope; target 90% nominal interval coverage in the 85–95% band on independent profile/site/event groups, with interval width and proper score included.
- **Physical transport:** target at least 10% lower primary transport error and 20% lower P90/P95 breakthrough-tail error than the best valid matched baseline, with group-aware uncertainty excluding zero improvement and no material worst-domain degradation. Also require conservative mass closure within combined measurement/model uncertainty.
- **Decision value:** compare expected loss and missed-event risk at the same total cost. Include sampling, travel, preparation, analysis, calibration/QC, repeats, instrument amortization, interpretation and turnaround. Report cost per correctly resolved decision—not cost per scan or R² alone.

Do not pool pixel, edge, timepoint or subsample rows across train/test when they share a specimen, core, profile, site, plant, event or acquisition batch. Freeze a prequential forecast before the target observation is acquired. Reserve a truly independent site/event group for the final test. These controls are necessary for any SOTA claim.

