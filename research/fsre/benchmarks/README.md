# FSRE public infrastructure benchmark programme

This folder holds versioned provenance manifests, benchmark contracts and result schemas. It deliberately distinguishes source benchmark results from FSRE decision-assurance metrics.

## First public water benchmark: Exeter Hanoi network

The first reproducible import target is the Hanoi water distribution network from the University of Exeter Centre for Water Systems. The source collection documents twelve water-distribution design problems and publishes best-known Pareto fronts; its page says five multi-objective evolutionary algorithms were used, including NSGA-II, ε-MOEA, ε-NSGAII, AMALGAM and Borg. The Hanoi instance is classified medium-sized, with 34 pipes, 31 demand nodes, one reservoir, a 30 m minimum pressure head and six commercial pipe sizes. The published search-space count is 2.87 × 10^26.

Source pages:
- Benchmark overview and problem metadata: https://www.exeter.ac.uk/research/centres/cws/resources/benchmarks/
- Published best-known Pareto fronts: https://www.exeter.ac.uk/research/centres/cws/resources/benchmarks/pareto/
- EPANET input URL (Hanoi): https://www.exeter.ac.uk/v8media/universityofexeter/emps/research/cws/downloads/hanoi.inp
- WNTR resilience metric documentation: https://usepa.github.io/WNTR/resilience.html

The current manifest records the official source URL and known problem properties. A direct raw-file download and source SHA-256 are still pending; do not treat the text visible in a web preview as a byte-identical source file. The benchmark dataset is not rehosted in this repository while its redistribution license remains unverified. The source webpage states general University copyright ownership unless otherwise stated; review the actual data terms or obtain permission before bundling source files.

## Matched optimization baselines

Reproduce the benchmark's original two objectives first: minimise capital pipe cost and maximise the published network-resilience objective. Use exactly the same network, decision variables, diameter choices, hydraulic assumptions, constraints and units for every method.

Baselines to implement or reproduce where code/license availability permits:
1. Published best-known Pareto archive (reference, not a proven global Pareto frontier).
2. NSGA-II.
3. ε-MOEA.
4. ε-NSGAII.
5. AMALGAM.
6. Borg.

Report hypervolume with a frozen common reference point, IGD+/epsilon indicators when a reference archive is defined, feasibility rate, constraint violation, nondominated archive size, function evaluations, wall time, CPU allocation, peak memory, and multiple seeds. The known-best archive is not ground truth; report the distance to it and the archive's provenance.

## WNTR resilience scorecard

Keep the hydraulic/service and security/economic dimensions separate. WNTR documents five broad categories: topographic, hydraulic, water-quality, water-security and economic metrics. Candidate measures include:
- pressure-threshold violations and their duration;
- delivered/expected demand and water-service availability;
- Todini and modified resilience indices;
- tank-capacity trajectories;
- population impacted;
- contaminant mass/volume consumed and extent of contamination;
- annual network cost, pump energy/cost and greenhouse-gas emissions.

Version the WNTR and EPANET engine, demand model, hydraulic time step, water-quality time step, simulation duration, pressure/flow units, population conversion assumptions and all scenario files. A system-wide scalar resilience index must never hide local service-floor failure.

## FSRE decision-integrity extension

The published Pareto objective is not a substitute for FSRE's decision-assurance scorecard. Run two evaluations:
- **Matched optimizer evaluation:** reproduce published objectives exactly.
- **Decision-integrity evaluation:** add qualified population/frame, temporal identity, evidence/source dependence, rights/authority, uncertainty-set adequacy, robust service floors, abstention correctness, experiment potency, and independent-verifier acceptance.

The latter is a new benchmark dimension. Do not claim the published algorithms were designed to optimise it, and do not collapse it into the original Pareto objective.

## E0/E1/E2 ladder

- E0: checksummed public source artifacts, exact parser/schema, unit tests, known-answer hydraulic values and deterministic run receipts.
- E1: reproducible EPANET/WNTR simulation with calibrated model parameters and declared model-form uncertainty; outputs remain simulation-derived.
- E2: prospective operator-authorised, independently adjudicated field evidence; report actual service levels, missingness, noncompliance, intervention timing, adverse effects and operational constraints.

No E0 or E1 result alone demonstrates real-world service improvement.
