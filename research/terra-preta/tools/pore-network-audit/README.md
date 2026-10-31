# 3D pore-network audit v0.1

This standalone Rust CLI audits a **pore/throat graph that has already been extracted from a segmented 3D CT volume**. It does not segment CT images, infer missing throats, simulate multiphase flow or assert that a connected geometric pathway was wet during a field event. The v0.1 input contract explicitly requires z to be the vertical axis; reorient/reorder the image, centroids, dimensions and spacing upstream when the physical vertical axis differs.

## Run

    cargo test --manifest-path research/terra-preta/tools/pore-network-audit/Cargo.toml
    cargo run --manifest-path research/terra-preta/tools/pore-network-audit/Cargo.toml -- \
      research/terra-preta/tools/pore-network-audit/fixtures/synthetic-connected-network.json \
      /tmp/terra-preta-pore-network-report.json

The report binds to the SHA-256 of the exact input bytes and refuses to overwrite an existing report.

## Input provenance

The input records source-image, segmentation-output and extracted-network SHA-256 digests; image dimensions and voxel spacing; segmentation method/threshold/software; and network extractor/version/parameters. Pore records carry voxel centroids, volume, inscribed radius and top/bottom boundary-touch labels. Throat records carry endpoint pore IDs, total/direct length and inscribed radius.

The intended export bridge is an already extracted pore/throat network, for example PoreSpy SNOW2/regions-to-network or another declared image-analysis pipeline. PoreSpy documents SNOW2 support for 3D images and exports pore/throat connectivity and geometric fields. The auditor does not assume an undocumented CSV mapping: transform to the explicit JSON contract and record the transformation digest.

## PoreSpy SNOW2 import crosswalk

A declared PoreSpy `regions_to_network` / SNOW2 export can be mapped as follows:

| Auditor field | PoreSpy source | Mapping rule |
|---|---|---|
| `centroid_voxels_xyz` | `pore.geometric_centroid` | Preserve xyz coordinate order after orienting physical vertical to z |
| `volume_um3` | `pore.volume` or `pore.region_volume` | Choose one volume definition and record it; do not mix both in one graph |
| `inscribed_radius_um` | `pore.inscribed_diameter` | Divide the documented inscribed diameter by two |
| `throat.total_length_um` | `throat.total_length` | Use the pore-centre to pore-centre distance via the throat centre |
| `throat.direct_length_um` | `throat.direct_length` | Use the straight-line pore-centre distance |
| `throat.inscribed_radius_um` | `throat.inscribed_diameter` | Divide the documented inscribed diameter by two |
| throat endpoints | `throat.conns` | Map integer indices to the corresponding `pore_id` values without changing connectivity |

PoreSpy describes `throat.total_length` as the length between pore centres via the throat centre, and `throat.direct_length` as the straight-line distance between pore centres. The auditor's shortest-path calculation uses the total throat length plus only the first/last pore-centroid-to-sample-boundary offsets; do not add pore radii or pore-body lengths a second time.

PoreSpy's documented `voxel_size` argument is scalar. When source CT voxels are anisotropic, resample to an isotropic grid with a recorded interpolation operator before standard SNOW extraction, or compute all geometry in a validated anisotropic-aware extractor. The auditor expects already converted physical lengths/areas/volumes and records original xyz spacing; it does not repair anisotropic metric errors.

Boundary labels must be derived from the declared sample ROI faces, not from padded boundary regions automatically added during network extraction. Retain the original segmented volume/ROI digest and the exact rule used to label top and bottom pore regions.

## Metrics

- Connected components and largest connected component pore-volume fraction.
- Top-to-bottom percolating component count and percolating pore-volume fraction.
- Widest top-to-bottom path bottleneck (maximum, over all boundary-to-boundary paths, of each path's minimum throat inscribed radius).
- Shortest graph path length, adding voxel-centroid to boundary offsets, and vertical tortuosity relative to sample height.
- Percolation sensitivity across a declared throat-radius cutoff sweep, to expose segmentation/voxel-resolution fragility.

## Interpretation boundaries

These are geometric network descriptors, not hydraulic conductance. A CT-segmented connected path may not be water-filled or hydraulically active during a storm. The widest-path bottleneck is not a capillary-entry pressure calculation. The shortest path is not flow-weighted tortuosity. No dynamic matrix–macropore exchange coefficient is calculated here.

Use matched cores/events to relate 3D graph metrics to tracer breakthrough, transient moisture/pressure, measured flux and CT-derived specific wetted interfacial area. Keep segmentation and extraction uncertainty as explicit factors; do not present one image threshold as immutable soil structure.

Reference methods: PoreSpy SNOW2 and regions-to-network documentation (https://porespy.org/examples/networks/reference/regions_to_network.html and https://porespy.org/examples/networks/reference/snow2.html); Chaif et al. (2025), Journal of Hydrology 661, 133791, DOI 10.1016/j.jhydrol.2025.133791.