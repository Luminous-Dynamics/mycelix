//! Stable observation-coordinate identities and validated measurement history.
//!
//! This module prevents adaptive-design code from treating a numeric matrix
//! index as a durable measurement identity. Coordinate IDs bind modality,
//! physical support and units to covariance coordinates. An acquired action
//! appends a typed block of observations exactly once. It does not estimate
//! dependence or infer correlation from support overlap.

use std::collections::{BTreeMap, BTreeSet};

/// One coordinate in a globally ordered candidate observation vector.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ObservationCoordinate {
    pub coordinate_id: String,
    pub modality: String,
    pub support_id: String,
    pub unit: String,
}

/// Canonical mapping between durable coordinate IDs and covariance indices.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ObservationLayout {
    coordinates: Vec<ObservationCoordinate>,
    index_by_id: BTreeMap<String, usize>,
}

impl ObservationLayout {
    /// Create a layout. Input order is the canonical covariance order.
    pub fn new(coordinates: Vec<ObservationCoordinate>) -> Result<Self, String> {
        if coordinates.is_empty() {
            return Err("observation layout must contain at least one coordinate".into());
        }
        let mut index_by_id = BTreeMap::new();
        for (index, coordinate) in coordinates.iter().enumerate() {
            if coordinate.coordinate_id.trim().is_empty() {
                return Err("coordinate identifiers must be non-empty".into());
            }
            if coordinate.modality.trim().is_empty() {
                return Err(format!("modality is required for {:?}", coordinate.coordinate_id));
            }
            if coordinate.support_id.trim().is_empty() {
                return Err(format!("support ID is required for {:?}", coordinate.coordinate_id));
            }
            if coordinate.unit.trim().is_empty() {
                return Err(format!("unit is required for {:?}", coordinate.coordinate_id));
            }
            if index_by_id.insert(coordinate.coordinate_id.clone(), index).is_some() {
                return Err(format!("duplicate coordinate ID {:?}", coordinate.coordinate_id));
            }
        }
        Ok(Self { coordinates, index_by_id })
    }

    pub fn coordinates(&self) -> &[ObservationCoordinate] {
        &self.coordinates
    }

    pub fn len(&self) -> usize {
        self.coordinates.len()
    }

    pub fn is_empty(&self) -> bool {
        self.coordinates.is_empty()
    }

    pub fn index_of(&self, coordinate_id: &str) -> Result<usize, String> {
        self.index_by_id.get(coordinate_id).copied()
            .ok_or_else(|| format!("unknown observation coordinate ID {coordinate_id:?}"))
    }

    /// Resolve IDs in the caller's order. Duplicate IDs are rejected.
    pub fn indices_for(&self, coordinate_ids: &[String]) -> Result<Vec<usize>, String> {
        let mut seen = BTreeSet::new();
        coordinate_ids.iter().map(|coordinate_id| {
            if !seen.insert(coordinate_id.as_str()) {
                return Err(format!("duplicate coordinate ID in request: {coordinate_id:?}"));
            }
            self.index_of(coordinate_id)
        }).collect()
    }
}

/// One candidate acquisition that produces a declared block of coordinates.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ObservationAction {
    pub action_id: String,
    /// Coordinates produced by the action, in observation-value order.
    pub coordinate_ids: Vec<String>,
}

impl ObservationAction {
    pub fn validate(&self, layout: &ObservationLayout) -> Result<(), String> {
        if self.action_id.trim().is_empty() {
            return Err("observation action ID must be non-empty".into());
        }
        if self.coordinate_ids.is_empty() {
            return Err(format!("action {:?} must produce at least one coordinate", self.action_id));
        }
        layout.indices_for(&self.coordinate_ids)?;
        Ok(())
    }

    pub fn layout_indices(&self, layout: &ObservationLayout) -> Result<Vec<usize>, String> {
        self.validate(layout)?;
        layout.indices_for(&self.coordinate_ids)
    }
}

/// Append-only history of acquired observations, aligned with durable IDs.
#[derive(Clone, Debug, PartialEq)]
pub struct ObservationHistory {
    coordinate_ids: Vec<String>,
    values: Vec<f64>,
}

impl ObservationHistory {
    pub fn empty() -> Self {
        Self { coordinate_ids: Vec::new(), values: Vec::new() }
    }

    pub fn coordinate_ids(&self) -> &[String] {
        &self.coordinate_ids
    }

    pub fn values(&self) -> &[f64] {
        &self.values
    }

    pub fn is_empty(&self) -> bool {
        self.coordinate_ids.is_empty()
    }

    /// Validate that every observation is finite, unique and present in the
    /// exact layout snapshot.
    pub fn validate(&self, layout: &ObservationLayout) -> Result<(), String> {
        if self.coordinate_ids.len() != self.values.len() {
            return Err("history coordinate/value lengths differ".into());
        }
        layout.indices_for(&self.coordinate_ids)?;
        if self.values.iter().any(|value| !value.is_finite()) {
            return Err("history values must all be finite".into());
        }
        Ok(())
    }

    /// Return covariance indices in the same order as the stored values.
    pub fn layout_indices(&self, layout: &ObservationLayout) -> Result<Vec<usize>, String> {
        self.validate(layout)?;
        layout.indices_for(&self.coordinate_ids)
    }

    /// Create a new history with one action's observations appended.
    ///
    /// Validation is completed before the new history is returned; failures
    /// leave the original immutable history unchanged.
    pub fn with_appended(
        &self,
        layout: &ObservationLayout,
        action: &ObservationAction,
        values: &[f64],
    ) -> Result<Self, String> {
        self.validate(layout)?;
        action.validate(layout)?;
        if values.len() != action.coordinate_ids.len() {
            return Err(format!(
                "action {:?} expected {} values but received {}",
                action.action_id,
                action.coordinate_ids.len(),
                values.len()
            ));
        }
        if values.iter().any(|value| !value.is_finite()) {
            return Err("new observation values must all be finite".into());
        }

        let existing: BTreeSet<&str> =
            self.coordinate_ids.iter().map(String::as_str).collect();
        for coordinate_id in &action.coordinate_ids {
            if existing.contains(coordinate_id.as_str()) {
                return Err(format!(
                    "coordinate {:?} was already observed; repeated acquisitions need new coordinate IDs",
                    coordinate_id
                ));
            }
        }

        let mut appended = self.clone();
        appended.coordinate_ids.extend(action.coordinate_ids.iter().cloned());
        appended.values.extend_from_slice(values);
        appended.validate(layout)?;
        Ok(appended)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn coordinate(id: &str, modality: &str, support: &str, unit: &str) -> ObservationCoordinate {
        ObservationCoordinate {
            coordinate_id: id.into(),
            modality: modality.into(),
            support_id: support.into(),
            unit: unit.into(),
        }
    }

    fn layout() -> ObservationLayout {
        ObservationLayout::new(vec![
            coordinate("ct:pore_fraction:core-A", "ct", "core-A:volume-1", "1"),
            coordinate("tracer:arrival:core-A", "tracer", "core-A:depth-10-20", "hour"),
            coordinate("tracer:tail:core-A", "tracer", "core-A:depth-10-20", "mg/L"),
        ]).unwrap()
    }

    #[test]
    fn layout_indices_follow_canonical_coordinate_order() {
        let layout = layout();
        assert_eq!(layout.index_of("ct:pore_fraction:core-A").unwrap(), 0);
        assert_eq!(
            layout.indices_for(&[
                "tracer:tail:core-A".into(),
                "ct:pore_fraction:core-A".into(),
            ]).unwrap(),
            vec![2, 0]
        );
    }

    #[test]
    fn rejects_empty_layout_duplicate_ids_and_missing_metadata() {
        assert!(ObservationLayout::new(vec![]).is_err());
        assert!(ObservationLayout::new(vec![
            coordinate("same", "ct", "core", "1"),
            coordinate("same", "tracer", "core", "mg/L"),
        ]).is_err());
        assert!(ObservationLayout::new(vec![
            coordinate("id", "ct", "", "1"),
        ]).is_err());
        assert!(ObservationLayout::new(vec![
            coordinate("id", "ct", "core", ""),
        ]).is_err());
    }

    #[test]
    fn rejects_unknown_and_duplicate_coordinate_requests() {
        let layout = layout();
        assert!(layout.indices_for(&["unknown".into()]).is_err());
        assert!(layout.indices_for(&[
            "ct:pore_fraction:core-A".into(),
            "ct:pore_fraction:core-A".into(),
        ]).is_err());
    }

    #[test]
    fn action_coordinates_resolve_in_value_order() {
        let layout = layout();
        let action = ObservationAction {
            action_id: "ct-plus-tracer".into(),
            coordinate_ids: vec![
                "ct:pore_fraction:core-A".into(),
                "tracer:arrival:core-A".into(),
            ],
        };
        assert_eq!(action.layout_indices(&layout).unwrap(), vec![0, 1]);
    }

    #[test]
    fn rejects_invalid_actions() {
        let layout = layout();
        assert!(ObservationAction {
            action_id: "".into(),
            coordinate_ids: vec!["ct:pore_fraction:core-A".into()],
        }.validate(&layout).is_err());
        assert!(ObservationAction {
            action_id: "empty".into(),
            coordinate_ids: vec![],
        }.validate(&layout).is_err());
        assert!(ObservationAction {
            action_id: "unknown".into(),
            coordinate_ids: vec!["not-in-layout".into()],
        }.validate(&layout).is_err());
    }

    #[test]
    fn history_appends_values_with_stable_coordinate_identity() {
        let layout = layout();
        let action = ObservationAction {
            action_id: "ct-plus-tracer".into(),
            coordinate_ids: vec![
                "ct:pore_fraction:core-A".into(),
                "tracer:arrival:core-A".into(),
            ],
        };
        let history = ObservationHistory::empty()
            .with_appended(&layout, &action, &[0.37, 2.5]).unwrap();

        assert_eq!(history.coordinate_ids().len(), 2);
        assert_eq!(history.values(), &[0.37, 2.5]);
        assert_eq!(history.layout_indices(&layout).unwrap(), vec![0, 1]);
    }

    #[test]
    fn repeated_acquisition_requires_a_new_coordinate_identity() {
        let layout = layout();
        let action = ObservationAction {
            action_id: "tracer-first".into(),
            coordinate_ids: vec!["tracer:arrival:core-A".into()],
        };
        let history = ObservationHistory::empty()
            .with_appended(&layout, &action, &[2.5]).unwrap();
        let repeated = ObservationAction {
            action_id: "tracer-repeat".into(),
            coordinate_ids: vec!["tracer:arrival:core-A".into()],
        };
        assert!(history.with_appended(&layout, &repeated, &[2.6]).is_err());
        assert_eq!(history.values(), &[2.5]);
    }

    #[test]
    fn append_rejects_dimension_mismatch_and_nonfinite_values_atomically() {
        let layout = layout();
        let action = ObservationAction {
            action_id: "ct".into(),
            coordinate_ids: vec!["ct:pore_fraction:core-A".into()],
        };
        let history = ObservationHistory::empty();
        assert!(history.with_appended(&layout, &action, &[]).is_err());
        assert!(history.with_appended(&layout, &action, &[f64::NAN]).is_err());
        assert!(history.is_empty());
    }

    #[test]
    fn history_rejects_unknown_ids_and_duplicate_coordinates() {
        let layout = layout();
        let unknown = ObservationHistory {
            coordinate_ids: vec!["missing".into()],
            values: vec![1.0],
        };
        assert!(unknown.validate(&layout).is_err());

        let duplicate = ObservationHistory {
            coordinate_ids: vec!["ct:pore_fraction:core-A".into(), "ct:pore_fraction:core-A".into()],
            values: vec![1.0, 2.0],
        };
        assert!(duplicate.validate(&layout).is_err());
    }

    #[test]
    fn action_history_indices_preserve_measurement_value_alignment() {
        let layout = layout();
        let action = ObservationAction {
            action_id: "late-tracer-first".into(),
            coordinate_ids: vec![
                "tracer:tail:core-A".into(),
                "ct:pore_fraction:core-A".into(),
            ],
        };
        let history = ObservationHistory::empty()
            .with_appended(&layout, &action, &[0.04, 0.37]).unwrap();
        assert_eq!(history.layout_indices(&layout).unwrap(), vec![2, 0]);
        assert_eq!(history.values(), &[0.04, 0.37]);
    }
}
