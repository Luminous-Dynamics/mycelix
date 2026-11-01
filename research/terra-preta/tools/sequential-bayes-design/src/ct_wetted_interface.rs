//! CT-support-aware wetted macropore–matrix interface measurement.
//!
//! The operator computes the voxel-face estimate of macropore/matrix-pore
//! interfacial area and weights each interface by the lesser local wet
//! fraction. It then divides by specimen volume to return specific wetted
//! interfacial area [m^2/m^3 = m^-1] for the transient exchange operator.
//!
//! Phase segmentation and wet fractions are explicit inputs. This does not
//! infer wetting from CT intensity, solve multiphase flow, correct partial
//! volume effects, or estimate sub-voxel interface area.

use crate::transient_exchange::ExchangeObservation;

/// Segmented phase label for one voxel.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum VoxelPhase {
    Solid,
    Macropore,
    MatrixPore,
}

/// One voxel's segmented phase and local water-filled fraction.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VoxelCell {
    pub phase: VoxelPhase,
    /// Fraction in [0,1]. Must be zero for solid voxels.
    pub wet_fraction: f64,
}

/// Finite 3D CT support with physical voxel spacing and stable support identity.
#[derive(Clone, Debug, PartialEq)]
pub struct VoxelGrid3D {
    pub support_id: String,
    pub dimensions: [usize; 3],
    /// Voxel pitch in metres along x, y and z.
    pub spacing_m: [f64; 3],
    /// X-fastest storage: index = (z * ny + y) * nx + x.
    pub cells: Vec<VoxelCell>,
}

/// Area summary of the segmented macropore–matrix interface.
#[derive(Clone, Debug, PartialEq)]
pub struct WettedInterfaceMeasurement {
    pub support_id: String,
    pub sample_volume_m3: f64,
    /// Full geometric area of macropore/matrix-pore voxel faces [m^2].
    pub geometric_interface_area_m2: f64,
    /// Wet-fraction-weighted macropore/matrix-pore interface area [m^2].
    pub wetted_interface_area_m2: f64,
    /// Number of unique geometric interface faces.
    pub geometric_interface_face_count: usize,
    /// Number of faces with positive wet weighting.
    pub wetted_interface_face_count: usize,
    /// Wetted interface area divided by total specimen volume [m^-1].
    pub specific_wetted_interface_area_per_m: f64,
}

fn checked_cell_count(dimensions: [usize; 3]) -> Result<usize, String> {
    if dimensions.iter().any(|dimension| *dimension == 0) {
        return Err("voxel grid dimensions must all be positive".into());
    }
    dimensions[0].checked_mul(dimensions[1])
        .and_then(|xy| xy.checked_mul(dimensions[2]))
        .ok_or_else(|| "voxel grid dimensions overflow addressable size".into())
}

impl VoxelGrid3D {
    pub fn validate(&self) -> Result<(), String> {
        if self.support_id.trim().is_empty() {
            return Err("CT support ID must be non-empty".into());
        }
        let expected_cells = checked_cell_count(self.dimensions)?;
        if self.cells.len() != expected_cells {
            return Err(format!(
                "voxel cell count {} does not match grid volume {expected_cells}",
                self.cells.len()
            ));
        }
        if self.spacing_m.iter().any(|spacing| !spacing.is_finite() || *spacing <= 0.0) {
            return Err("voxel spacing must be finite and positive".into());
        }
        for cell in &self.cells {
            if !cell.wet_fraction.is_finite() || !(0.0..=1.0).contains(&cell.wet_fraction) {
                return Err("voxel wet fractions must be finite and lie in [0,1]".into());
            }
            if cell.phase == VoxelPhase::Solid && cell.wet_fraction != 0.0 {
                return Err("solid voxels must have zero pore-water wet fraction".into());
            }
        }
        Ok(())
    }

    fn index(&self, x: usize, y: usize, z: usize) -> usize {
        (z * self.dimensions[1] + y) * self.dimensions[0] + x
    }

    fn is_macro_matrix_interface(left: VoxelPhase, right: VoxelPhase) -> bool {
        matches!(
            (left, right),
            (VoxelPhase::Macropore, VoxelPhase::MatrixPore)
                | (VoxelPhase::MatrixPore, VoxelPhase::Macropore)
        )
    }

    /// Compute unique voxel-face interface area. Only positive-axis neighbors
    /// are visited, so each shared face is counted exactly once.
    pub fn measure_wetted_interface(&self) -> Result<WettedInterfaceMeasurement, String> {
        self.validate()?;
        let [nx, ny, nz] = self.dimensions;
        let [dx, dy, dz] = self.spacing_m;
        let sample_volume_m3 = nx as f64 * dx * ny as f64 * dy * nz as f64 * dz;
        if !sample_volume_m3.is_finite() || sample_volume_m3 <= 0.0 {
            return Err("voxel support volume is invalid".into());
        }

        let mut geometric_interface_area_m2 = 0.0;
        let mut wetted_interface_area_m2 = 0.0;
        let mut geometric_interface_face_count = 0usize;
        let mut wetted_interface_face_count = 0usize;

        for (linear_index, left) in self.cells.iter().copied().enumerate() {
            let x = linear_index % nx;
            let y = (linear_index / nx) % ny;
            let z = linear_index / (nx * ny);
            let neighbors = [
                if x + 1 < nx {
                    Some((self.cells[self.index(x + 1, y, z)], dy * dz))
                } else { None },
                if y + 1 < ny {
                    Some((self.cells[self.index(x, y + 1, z)], dx * dz))
                } else { None },
                if z + 1 < nz {
                    Some((self.cells[self.index(x, y, z + 1)], dx * dy))
                } else { None },
            ];

            for (right, face_area) in neighbors.into_iter().flatten() {
                if Self::is_macro_matrix_interface(left.phase, right.phase) {
                    geometric_interface_area_m2 += face_area;
                    geometric_interface_face_count += 1;
                    let wet_weight = left.wet_fraction.min(right.wet_fraction);
                    if wet_weight > 0.0 {
                        wetted_interface_area_m2 += face_area * wet_weight;
                        wetted_interface_face_count += 1;
                    }
                }
            }
        }

        let specific_wetted_interface_area_per_m =
            wetted_interface_area_m2 / sample_volume_m3;
        if !geometric_interface_area_m2.is_finite()
            || !wetted_interface_area_m2.is_finite()
            || !specific_wetted_interface_area_per_m.is_finite()
        {
            return Err("CT interface measurement is non-finite".into());
        }

        Ok(WettedInterfaceMeasurement {
            support_id: self.support_id.clone(),
            sample_volume_m3,
            geometric_interface_area_m2,
            wetted_interface_area_m2,
            geometric_interface_face_count,
            wetted_interface_face_count,
            specific_wetted_interface_area_per_m,
        })
    }
}

impl WettedInterfaceMeasurement {
    /// Convert the CT-derived specific wetted area into the input expected by
    /// the transient exchange operator. Macropore flux retains its signed
    /// direction and physical units [m/s].
    pub fn to_exchange_observation(
        &self,
        time_seconds: f64,
        macropore_flux_density_m_per_s: f64,
    ) -> Result<ExchangeObservation, String> {
        if !time_seconds.is_finite() || !macropore_flux_density_m_per_s.is_finite() {
            return Err("exchange observation time and flux must be finite".into());
        }
        if !self.specific_wetted_interface_area_per_m.is_finite()
            || self.specific_wetted_interface_area_per_m < 0.0
        {
            return Err("specific wetted interface area must be finite and non-negative".into());
        }
        Ok(ExchangeObservation {
            time_seconds,
            wetted_area_per_m: self.specific_wetted_interface_area_per_m,
            macropore_flux_m_per_s: macropore_flux_density_m_per_s,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cell(phase: VoxelPhase, wet_fraction: f64) -> VoxelCell {
        VoxelCell { phase, wet_fraction }
    }

    fn grid(
        dimensions: [usize; 3],
        spacing_m: [f64; 3],
        cells: Vec<VoxelCell>,
    ) -> VoxelGrid3D {
        VoxelGrid3D {
            support_id: "core-A:ct-volume-1".into(),
            dimensions,
            spacing_m,
            cells,
        }
    }

    #[test]
    fn one_macro_matrix_face_has_exact_voxel_area_and_specific_area() {
        let volume = grid(
            [2, 1, 1],
            [1.0, 1.0, 1.0],
            vec![
                cell(VoxelPhase::Macropore, 0.5),
                cell(VoxelPhase::MatrixPore, 0.8),
            ],
        ).measure_wetted_interface().unwrap();
        assert!((volume.geometric_interface_area_m2 - 1.0).abs() < 1e-12);
        assert!((volume.wetted_interface_area_m2 - 0.5).abs() < 1e-12);
        assert!((volume.sample_volume_m3 - 2.0).abs() < 1e-12);
        assert!((volume.specific_wetted_interface_area_per_m - 0.25).abs() < 1e-12);
        assert_eq!(volume.geometric_interface_face_count, 1);
        assert_eq!(volume.wetted_interface_face_count, 1);
    }

    #[test]
    fn anisotropic_voxel_spacing_uses_correct_face_orientation_area() {
        // Interface is normal to x, so face area is dy*dz = 0.1*0.5 = 0.05 m².
        let volume = grid(
            [2, 1, 1],
            [0.2, 0.1, 0.5],
            vec![
                cell(VoxelPhase::Macropore, 1.0),
                cell(VoxelPhase::MatrixPore, 1.0),
            ],
        ).measure_wetted_interface().unwrap();
        assert!((volume.geometric_interface_area_m2 - 0.05).abs() < 1e-12);
        assert!((volume.sample_volume_m3 - 0.02).abs() < 1e-12);
        assert!((volume.specific_wetted_interface_area_per_m - 2.5).abs() < 1e-12);
    }

    #[test]
    fn every_voxel_face_is_counted_once() {
        // Two x-oriented macro/matrix faces and one y-oriented face.
        let volume = grid(
            [2, 2, 1],
            [1.0, 1.0, 1.0],
            vec![
                cell(VoxelPhase::Macropore, 1.0),
                cell(VoxelPhase::MatrixPore, 1.0),
                cell(VoxelPhase::MatrixPore, 1.0),
                cell(VoxelPhase::Macropore, 1.0),
            ],
        ).measure_wetted_interface().unwrap();
        assert_eq!(volume.geometric_interface_face_count, 3);
        assert!((volume.geometric_interface_area_m2 - 3.0).abs() < 1e-12);
        assert!((volume.wetted_interface_area_m2 - 3.0).abs() < 1e-12);
    }

    #[test]
    fn unwetted_matrix_has_geometric_but_no_wetted_interface_area() {
        let volume = grid(
            [2, 1, 1],
            [1.0, 1.0, 1.0],
            vec![
                cell(VoxelPhase::Macropore, 1.0),
                cell(VoxelPhase::MatrixPore, 0.0),
            ],
        ).measure_wetted_interface().unwrap();
        assert_eq!(volume.geometric_interface_face_count, 1);
        assert_eq!(volume.wetted_interface_face_count, 0);
        assert!((volume.geometric_interface_area_m2 - 1.0).abs() < 1e-12);
        assert_eq!(volume.wetted_interface_area_m2, 0.0);
        assert_eq!(volume.specific_wetted_interface_area_per_m, 0.0);
    }

    #[test]
    fn solid_neighbors_are_not_counted_as_pore_interface() {
        let volume = grid(
            [2, 1, 1],
            [1.0, 1.0, 1.0],
            vec![
                cell(VoxelPhase::Macropore, 1.0),
                cell(VoxelPhase::Solid, 0.0),
            ],
        ).measure_wetted_interface().unwrap();
        assert_eq!(volume.geometric_interface_face_count, 0);
    }

    #[test]
    fn exchange_observation_preserves_ct_support_and_units_boundary() {
        let measurement = grid(
            [2, 1, 1],
            [1.0, 1.0, 1.0],
            vec![
                cell(VoxelPhase::Macropore, 0.6),
                cell(VoxelPhase::MatrixPore, 0.8),
            ],
        ).measure_wetted_interface().unwrap();
        let exchange = measurement.to_exchange_observation(12.0, 0.03).unwrap();
        assert_eq!(exchange.time_seconds, 12.0);
        assert!((exchange.wetted_area_per_m - 0.3).abs() < 1e-12);
        assert_eq!(exchange.macropore_flux_m_per_s, 0.03);
    }

    #[test]
    fn ct_wetting_history_drives_conservative_transient_exchange_end_to_end() {
        use crate::transient_exchange::{
            check_paired_exchange_balance, TransientExchangeSeries,
        };

        let dry_event = grid(
            [2, 1, 1],
            [1.0, 1.0, 1.0],
            vec![
                cell(VoxelPhase::Macropore, 0.2),
                cell(VoxelPhase::MatrixPore, 0.5),
            ],
        ).measure_wetted_interface().unwrap();
        let wet_event = grid(
            [2, 1, 1],
            [1.0, 1.0, 1.0],
            vec![
                cell(VoxelPhase::Macropore, 1.0),
                cell(VoxelPhase::MatrixPore, 0.8),
            ],
        ).measure_wetted_interface().unwrap();

        let series = TransientExchangeSeries::new(&[
            dry_event.to_exchange_observation(0.0, 0.02).unwrap(),
            wet_event.to_exchange_observation(10.0, 0.02).unwrap(),
        ]).unwrap();

        let early = series.sample_at(0.0).unwrap();
        let late = series.sample_at(10.0).unwrap();
        assert!((early.wetted_area_per_m - 0.1).abs() < 1e-12);
        assert!((late.wetted_area_per_m - 0.4).abs() < 1e-12);
        assert!((early.exchange_rate_per_s - 0.002).abs() < 1e-12);
        assert!((late.exchange_rate_per_s - 0.008).abs() < 1e-12);

        let transferred = series.integrate_full().unwrap();
        assert!((transferred - 0.05).abs() < 1e-12);
        let balance = check_paired_exchange_balance(
            -transferred,
            transferred,
            transferred,
            1e-12,
        ).unwrap();
        assert!(balance.balanced);
    }

    #[test]
    fn rejects_bad_grid_dimensions_spacing_phases_and_support_id() {
        assert!(grid([0, 1, 1], [1.0, 1.0, 1.0], vec![]).measure_wetted_interface().is_err());
        assert!(grid([1, 1, 1], [0.0, 1.0, 1.0], vec![
            cell(VoxelPhase::Macropore, 1.0),
        ]).measure_wetted_interface().is_err());
        assert!(grid([1, 1, 1], [1.0, 1.0, 1.0], vec![
            cell(VoxelPhase::Solid, 0.2),
        ]).measure_wetted_interface().is_err());
        let mut invalid = grid([1, 1, 1], [1.0, 1.0, 1.0], vec![
            cell(VoxelPhase::MatrixPore, f64::NAN),
        ]);
        assert!(invalid.measure_wetted_interface().is_err());
        invalid.support_id = " ".into();
        invalid.cells[0].wet_fraction = 1.0;
        assert!(invalid.measure_wetted_interface().is_err());
    }

    #[test]
    fn rejects_exchange_bridge_with_nonfinite_time_or_flux() {
        let measurement = grid(
            [2, 1, 1],
            [1.0, 1.0, 1.0],
            vec![
                cell(VoxelPhase::Macropore, 1.0),
                cell(VoxelPhase::MatrixPore, 1.0),
            ],
        ).measure_wetted_interface().unwrap();
        assert!(measurement.to_exchange_observation(f64::NAN, 0.1).is_err());
        assert!(measurement.to_exchange_observation(0.0, f64::INFINITY).is_err());
    }
}
