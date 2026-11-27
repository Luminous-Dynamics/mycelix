# Mixed-dimensional cell-complex linear transport reference v0.1

**Status:** synthetic reference package for incidence identities and steady linear conductance networks. It is not a PDE solver or a soil-physics validation.

## Scientific idea

A registered cell complex provides an oriented topological substrate that can be shared by different transport processes. In v0.1:
- 0-cells are nodes;
- 1-cells are oriented edges;
- 2-cells are oriented faces;
- 3-cells are oriented volumes.

Every face lists its oriented edge boundary. Every volume lists its oriented face boundary. The validator checks both boundary-of-boundary identities:
- edge boundary composed with face boundary is zero;
- face boundary composed with volume boundary is zero.

For each physical process, a separate positive edge-conductance operator is supplied. The scalar potential is a 0-cochain; the integrated edge flux is a 1-cochain. For an edge oriented tail→head, the convention is:
`q_e = G_e (u_tail - u_head)`.
Internal node conservation is the vanishing incidence divergence of `q`. Dirichlet sources/sinks close the boundary budget.

Hydraulic and thermal examples share one unit-cube complex but use separate, explicitly unit-labelled conductance arrays. A shared incidence structure does not make the constitutive physics interchangeable.

## Scope and limitations

This is a linear resistor/conduction network, not a spatially resolved finite-volume, finite-element or mixed-dimensional PDE solver. Edge conductances are supplied as qualified inputs; the reference does not derive them from pore geometry or materials. The v0.1 model has no storage, saturation dependence, inertia, advection-diffusion coupling, latent heat or reaction terms. It does not model root uptake, matrix exchange or biological processes.

The 2026 Forman cochain study (Liu et al., DOI 10.1016/j.ijrmms.2026.106432) is a mathematical comparator. Its sandstone permeability results underpredict by 16–60% in the tested cases, partly because sub-resolution microporosity is omitted; its six XCT specimens have only two bulk permeability references, one per sandstone type. That limitation is a required caution, not evidence that a cochain formulation alone solves soil hydraulics.

## Synthetic cube fixture

The unit cube contains 8 nodes, 12 edges, 6 faces and 1 volume. Four parallel x-directed edges carry the potential drop from the source side to the sink side; the other edges close the cell complex and conserve flow.

The hydraulic operator assigns conductance 1 to each edge and expects source/sink flux 4. The thermal operator assigns conductance 2 to each edge and expects source/sink flux 8. These are manufactured units and values.

## Reproducibility

The CLI hashes exact input bytes, validates source-artifact digest, cell identities, orientations, dimensions, units and conductances, writes deterministic sorted results and refuses to overwrite an existing output. Tests cover:
- exact boundary-of-boundary identities for a cube;
- shared topology with separate hydraulic and thermal operators;
- analytic series and parallel resistor networks;
- edge-orientation invariance;
- local/global conservation and the conductance quadratic form;
- invalid face/volume incidence, missing conductance and unanchored components.

## Qualification targets (not achieved)

- Exact algebraic incidence identities for manufactured complexes.
- Analytic series/parallel solutions to near machine precision.
- Local interior divergence and global boundary-flux residual below a predeclared tolerance.
- Positive dissipation / quadratic form for positive conductances.
- Refinement and anisotropy only after geometrically meaningful cell-complex refinements are implemented.

A complete software pass validates the manufactured network algebra only. It does not prove hydraulic/thermal constitutive adequacy or field performance.