# 3D pore-network audit v0.1

This standalone Rust CLI audits a **pore/throat graph that has already been extracted from a segmented 3D CT volume**. It does not segment CT images, infer missing throats, simulate multiphase flow or assert that a connected geometric pathway was wet during a field event.

## Run

    cargo test --manifest-path research/terra-preta/tools/pore-network-audit/Cargo.toml
    cargo run --manifest-path research/terra-preta/tools/pore-network-audit/Cargo.toml -- \
      research/terra-preta/tools/pore-network-audit/fixtures/synthetic-connected-network.json \
      /tmp/terra-preta-pore-network-report.json

The report binds to the SHA-256 of the exact input bytes and refuses to overwrite an existing report.

## Input provenance

The input records source-image, segmentation-output and extracted-network SHA-256 digests; image dimensions and voxel spacing; segmentation method/threshold/software; and network extractor/version/parameters. Pore records carry voxel centroids, volume, inscribed radius and top/bottom boundary-touch labels. Throat records carry endpoint pore IDs, total/direct length and inscribed radius.

The intended export bridge is an already extracted pore/throat network, for example PoreSpy SNOW2/regions-to-network or another declared image-analysis pipeline. PoreSpy documents SNOW2 support for 3D images and exports pore/throat connectivity and geometric fields. The auditor does not assume an undocumented CSV mapping: transform to the explicit JSON contract and record the transformation digest.

## Metrics

- Connected components and largest connected component pore-volume fraction.
- Top-to-bottom percolating component count and percolating pore-volume fraction.
- Widest top-to-bottom path bottleneck (maximum, over all boundary-to-boundary paths, of each path's minimum throat inscribed radius).
- Shortest graph path length, adding voxel-centroid to boundary offsets, and vertical tortuosity relative to sample height.
- Percolation sensitivity across a declared throat-radius cutoff sweep, to expose segmentation/voxel-resolution fragility.

## Interpretation boundaries

These are geometric network descriptors, not hydraulic conductance. A CT-segmented connected path may not be water-filled or hydraulically active during a storm. The widest-path bottleneck is not a capillary-entry pressure calculation. The shortest path is not flow-weighted tortuosity. No dynamic matrix–macropore exchange coefficient is calculated here.

Use matched cores/events to relate 3D graph metrics to tracer breakthrough, transient moisture/pressure, measured flux and CT-derived specific wetted interfacial area. Keep segmentation and extraction uncertainty as explicit factors; do not present one image threshold as immutable soil structure.

Reference methods: PoreSpy SNOW2 and regions-to-network documentation; Chaif et al. (2025), Journal of Hydrology 661, 133791, DOI 10.1016/j.jhydrol.2025.133791.