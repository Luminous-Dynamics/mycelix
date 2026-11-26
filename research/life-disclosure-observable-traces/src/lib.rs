use std::collections::BTreeMap;

/// A map of fields visible to one declared observer profile. Evaluator-only
/// labels and protected source state must never be inserted into this view.
pub type ObserverView = BTreeMap<String, String>;

#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct ObserverTraceComparison {
    /// Fields present in both views whose public values differ.
    pub mismatched_fields: Vec<String>,
    /// Required observer fields absent from either view.
    pub missing_fields: Vec<String>,
}

impl ObserverTraceComparison {
    pub fn is_complete(&self) -> bool {
        self.missing_fields.is_empty()
    }

    pub fn is_equivalent(&self) -> bool {
        self.is_complete() && self.mismatched_fields.is_empty()
    }
}

/// Compare only the explicitly declared observer-visible fields.
///
/// Missing fields are not treated as equal: they make the comparison
/// incomplete. Callers decide whether a difference is authorised under the
/// experiment's disclosure profile.
pub fn compare_observer_views(
    left: &ObserverView,
    right: &ObserverView,
    required_fields: &[&str],
) -> ObserverTraceComparison {
    let mut report = ObserverTraceComparison::default();
    for field in required_fields {
        match (left.get(*field), right.get(*field)) {
            (Some(left_value), Some(right_value)) if left_value != right_value => {
                report.mismatched_fields.push((*field).to_owned());
            }
            (Some(_), Some(_)) => {}
            _ => report.missing_fields.push((*field).to_owned()),
        }
    }
    report.mismatched_fields.sort();
    report.missing_fields.sort();
    report
}

pub mod memory_metadata_leakage;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn generic_observer_comparison_is_order_independent() {
        let left = ObserverView::from([
            ("admission".into(), "allow".into()),
            ("response_size".into(), "96".into()),
        ]);
        let right = ObserverView::from([
            ("response_size".into(), "96".into()),
            ("admission".into(), "allow".into()),
        ]);
        let diff = compare_observer_views(&left, &right, &["admission", "response_size"]);
        assert!(diff.is_equivalent());
    }

    #[test]
    fn missing_observer_fields_are_indeterminate_not_equal() {
        let left = ObserverView::from([("admission".into(), "allow".into())]);
        let right = ObserverView::from([("admission".into(), "allow".into())]);
        let diff = compare_observer_views(&left, &right, &["admission", "cache_state"]);
        assert!(!diff.is_complete());
        assert!(!diff.is_equivalent());
        assert_eq!(diff.missing_fields, vec!["cache_state"]);
    }
}
