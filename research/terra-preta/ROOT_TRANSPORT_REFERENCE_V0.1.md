# Known-truth 3D root pathway activation and tracer reference v0.1

**Status:** synthetic reference-law evaluator. It is not a soil-flow PDE solver, CT reconstruction system, empirical root classifier, or field model validation.

## Scientific motivation

Agarwal, Ojha & Guha (2026), *A review of preferential flow and transport through root channels*, Vadose Zone Journal, DOI [10.1002/vzj2.70148](https://doi.org/10.1002/vzj2.70148), review the evolving root–soil interfacial processes and limitations of dye/image evidence. Qian et al. (2026), *Root-induced preferential flow in the critical zone*, Earth-Science Reviews, DOI [10.1016/j.earscirev.2026.105734](https://doi.org/10.1016/j.earscirev.2026.105734), synthesize 330 studies and distinguish non-exclusive root-flow expressions.

The inventory auditor #6437 already separates pathway origin, expression, source support and root occupancy. This reference advances one boundary: **geometric connectivity is not hydraulic activation**. A connected 3D path can be inactive under the declared state; a 2D projection can alias two different 3D paths.

## v0.1 network reference law

For each pathway, segment conductance is:
`g_i = g_{i,baseline} × scale_i`

For ordered segments in series:
`G_path = 1 / Σ(1 / g_i)`

For parallel pathways under a common declared head drop `Δh`:
`Q_total = Δh × Σ G_path`

`Q_path = Q_total × G_path / Σ G_path`

A path containing a known inactive segment has zero path conductance. If any required path activation is unassessed, the complete flow allocation is withheld. Unknown is not converted to inactive.

This is a deliberately simple hydraulic-network abstraction. Conductance units are m³ h⁻¹ m-head⁻¹; flow units are m³ h⁻¹. It does not represent Richards constitutive relations, unsaturated water content, hysteresis, matrix exchange, dual-permeability domains, capillarity, boundary transients, fluid momentum, dispersion, sorption or reactive transport.

## Tracer accounting

Injected tracer mass is allocated by path flow fraction. Each path has a declared recovery fraction. Potential recovered mass is:
`M_recovered,path = M_injected × flow_fraction × recovery_fraction`

The remainder is kept as **unrecovered or retained**, not silently assigned to one mechanism. The evaluator reports a path transit-time sum and a sampled arrival time quantized to the declared sample interval. Below-detection and outside-window arrivals are represented separately. This is an observation-contract test, not a full breakthrough-curve model.

## Adversarial controls

- The root and abiotic crack pathways share identical XZ projections but differ in the suppressed Y coordinate and origin truth.
- A geometrically continuous but hydraulically inactive textural finger contributes no flow/tracer.
- Root occupancy changes from occupied → partially occupied → unoccupied legacy; origin provenance remains root-origin while conductance changes through explicit state inputs.
- Any unassessed edge state withholds complete flow allocation.
- Ordered edge-chain continuity, source/outlet identity, node geometry, dimensions and positive physical quantities are validated.
- Tracer recovered plus unrecovered/retained mass closes to injected mass within the declared tolerance.
- Existing outputs are not overwritten.

## Integration and next comparison

- Parent root pathway ontology: [#6436](https://github.com/Luminous-Dynamics/mycelix/issues/6436) / [PR #6437](https://github.com/Luminous-Dynamics/mycelix/pull/6437).
- CT/tracer observation operators: [#5643](https://github.com/Luminous-Dynamics/mycelix/issues/5643).
- Multiscale CT-to-tensorial permeability: [#5650](https://github.com/Luminous-Dynamics/mycelix/issues/5650).
- Specialist model escalation: Symtropy [#4487](https://github.com/Luminous-Dynamics/symtropy/issues/4487).

The next physical bake-off should compare this known-truth reference family and empirical support against Richards / measured-curve, HYDRUS DualPerm, MACRO, static geometry-only and dynamic root-history models. Pin solver versions, constitutive parameters, boundary/initial conditions and observation operators. Use independent soil profile, root-functional-group, antecedent wetness and storm/event holdouts. Score arrival-time intervals, peak timing/concentration, early bypass, tailing, cumulative mass recovery, water/solute residual, calibration and compute/measurement cost. A solver unsupported for a mechanism is marked unsupported, not scored as if it represented it.

## Claim ceiling

All fixture values are manufactured. The reference law tests evidence separation, arithmetic and replay only. It does not validate flow in real soil, identify root origin from CT, estimate saturated/unsaturated conductivity, or demonstrate a field or crop benefit.
