# CT segmentation uncertainty → graph topology audit v0.1

**Status:** deterministic sensitivity evaluator over already-extracted graph variants. It does not segment raw CT images, solve flow or estimate field hydraulic activity.

## Scientific motivation

Jiang, Leung & Liu (2025), *Plant and Soil* 511, 969–995, DOI [10.1007/s11104-024-07030-w](https://doi.org/10.1007/s11104-024-07030-w), quantify segmentation uncertainty in vegetated porous media. Their percentile-segmentation procedure uses phase-probability thresholds of 10%, 30%, 50%, 70% and 90%, measures relative change against the 50% reference, and studies propagation from phase volumes to pore-network morphology and absolute permeability. They report that segmentation uncertainty depends on phase, moisture and root architecture, and that downstream effects can be amplified strongly.

This audit extends that uncertainty chain to graph topology: edge/node persistence, source–sink reachability, connected components, cycle rank and pairwise graph similarity. It is not a reproduction of their image segmentation code, PNM, or permeability solver.

## Input contract

- One raw-image URI/SHA-256, dimensions and anisotropic voxel spacing.
- One declared phase, segmentation method/version, preprocessing identity and network extractor/version.
- A deterministic percentile set, including exactly one 50% reference.
- Mask, extracted-network and extractor-settings SHA-256 digests for every variant.
- A canonical node/edge catalog with stable feature IDs and unchanged endpoints.
- A fixed source/sink boundary set.
- Per-variant present-node and present-edge IDs.

v0.1 assumes a one-to-one canonical feature correspondence. If a pore/throat splits or merges and the feature cannot be matched without ambiguity, the caller must not fabricate an ID mapping: the ensemble is unqualified and must be resolved upstream. Geometry alignment and split/merge correspondence are future extensions.

## Report semantics

- Feature sensitivity frequency = number of supplied variants in which the feature appears / total declared variants.
- Terminal connectivity sensitivity frequency = number of supplied variants in which the source–sink pair is connected / total variants.
- Pairwise edge-set Jaccard measures overlap among the supplied graph variants.
- Per-variant connected components and cycle rank describe topology for that variant.
- Relative change from the 50% reference is reported for graph counts. If a reference metric is zero, the relative change is null rather than infinite or silently replaced.

These are deterministic frequencies over a declared sensitivity grid. They are **not probabilities**, posterior confidence, or hydraulic activation probabilities. The 50% reference is a convention, not ground truth.

## Synthetic fixture

Five manufactured percentile variants describe a network with two redundant routes:
- 10%, 30%, 50%: both routes present; cycle rank 1; source–sink connected.
- 70%: one route remains; cycle rank 0; source–sink connected.
- 90%: both downstream branch throats absent; the terminal pair is disconnected.

Expected result: 4/5 variants connect the source to the sink; the result is segmentation-sensitive. The upper branch's final throat is present in 3/5 variants; the lower branch's final throat is present in 4/5. Cycle rank ranges from 0 to 1. These values are manufactured controls.

## Reproducibility and safety

The Rust CLI validates immutable image/mask/network/settings digests, canonical IDs, unchanged edge endpoints, source/sink inclusion, percentile uniqueness and graph references. It hashes exact input bytes, reports deterministic sorted results and writes with create-new/no-overwrite semantics.

## Interpretation boundary

A feature present in 4/5 deterministic variants is not "80% likely to exist." It is present in 4/5 variants in this declared sensitivity set. Variant selection and weights require an external uncertainty model. Structural persistence is not water-filled connectivity, hydraulic activation, permeability, tracer transport or biological function. Those require their separate evidence and model layers (#5650, #6574).

## Qualification targets (not achieved)

- 100% agreement with manufactured graph truth for feature incidence, terminal connectivity, components, cycle rank and Jaccard.
- Zero stable-connected claims if any supplied variant disconnects the pair.
- No posterior/probability language for deterministic sensitivity frequencies.
- No ambiguous split/merge mappings, unknown IDs or invalid endpoint correspondences accepted.
- Exact deterministic replay, input digests and no-overwrite behavior.
- Empirical evaluation only after raw CT bytes, segmentation probability maps, extractor settings and independent physical reference data are admitted.