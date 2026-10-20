use serde::{Deserialize, Serialize};

/// An absolute time represented as seconds since Unix epoch UTC.
///
/// Parsing from wall-clock strings is intentionally outside this semantic
/// layer. Callers must normalize an unambiguous offset-aware value first.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(transparent)]
pub struct TemporalPoint(pub i64);

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TemporalInterval {
    pub start: Option<TemporalPoint>,
    pub end: Option<TemporalPoint>,
}

impl TemporalInterval {
    pub fn validate(&self) -> Result<(), String> {
        if let (Some(start), Some(end)) = (self.start, self.end) {
            if end < start {
                return Err("temporal interval end precedes start".into());
            }
        }
        Ok(())
    }

    /// Returns None when either bound is unknown.
    pub fn overlaps(&self, other: &Self) -> Option<bool> {
        match (self.start, self.end, other.start, other.end) {
            (Some(a_start), Some(a_end), Some(b_start), Some(b_end)) => {
                Some(a_start <= b_end && b_start <= a_end)
            }
            _ => None,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TemporalProvenance {
    EngineeringClaim,
    ExternalAuthority,
    ProtocolPublication,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TemporalAssertion {
    pub event_time: Option<TemporalPoint>,
    pub applicability: TemporalInterval,
    pub publication_time: TemporalPoint,
    pub provenance: TemporalProvenance,
}

impl TemporalAssertion {
    /// Validate temporal structure only. Publication time is deliberately
    /// separate from event/applicability time.
    pub fn validate(&self) -> Result<(), String> {
        self.applicability.validate()?;

        if self.provenance == TemporalProvenance::ProtocolPublication {
            return Err(
                "protocol publication metadata cannot be used as engineering temporal applicability"
                    .into(),
            );
        }

        Ok(())
    }
}

/// Compare applicability only when both intervals are fully bounded.
pub fn applicability_overlaps(
    left: &TemporalInterval,
    right: &TemporalInterval,
) -> Result<Option<bool>, String> {
    left.validate()?;
    right.validate()?;
    Ok(left.overlaps(right))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn point(value: i64) -> TemporalPoint {
        TemporalPoint(value)
    }

    #[test]
    fn publication_time_is_distinct_from_event_time() {
        let assertion = TemporalAssertion {
            event_time: Some(point(100)),
            applicability: TemporalInterval {
                start: Some(point(100)),
                end: None,
            },
            publication_time: point(200),
            provenance: TemporalProvenance::EngineeringClaim,
        };
        assert!(assertion.validate().is_ok());
        assert_ne!(assertion.event_time, Some(assertion.publication_time));
    }

    #[test]
    fn impossible_interval_is_rejected() {
        let interval = TemporalInterval {
            start: Some(point(20)),
            end: Some(point(10)),
        };
        assert!(interval.validate().is_err());
    }

    #[test]
    fn unknown_end_remains_open() {
        let interval = TemporalInterval {
            start: Some(point(10)),
            end: None,
        };
        assert!(interval.validate().is_ok());
    }

    #[test]
    fn unknown_bounds_make_overlap_indeterminate() {
        let open = TemporalInterval {
            start: Some(point(10)),
            end: None,
        };
        let bounded = TemporalInterval {
            start: Some(point(20)),
            end: Some(point(30)),
        };
        assert_eq!(open.overlaps(&bounded), None);
    }

    #[test]
    fn adjacent_intervals_without_shared_point_do_not_overlap() {
        let left = TemporalInterval {
            start: Some(point(10)),
            end: Some(point(20)),
        };
        let right = TemporalInterval {
            start: Some(point(21)),
            end: Some(point(30)),
        };
        assert_eq!(left.overlaps(&right), Some(false));
    }

    #[test]
    fn overlapping_intervals_are_structurally_detectable() {
        let left = TemporalInterval {
            start: Some(point(10)),
            end: Some(point(20)),
        };
        let right = TemporalInterval {
            start: Some(point(20)),
            end: Some(point(30)),
        };
        assert_eq!(left.overlaps(&right), Some(true));
    }

    #[test]
    fn protocol_publication_cannot_become_engineering_applicability() {
        let assertion = TemporalAssertion {
            event_time: Some(point(100)),
            applicability: TemporalInterval {
                start: Some(point(100)),
                end: Some(point(200)),
            },
            publication_time: point(300),
            provenance: TemporalProvenance::ProtocolPublication,
        };
        assert!(assertion.validate().is_err());
    }

    #[test]
    fn later_observation_does_not_delete_earlier_negative_evidence() {
        let failed = TemporalAssertion {
            event_time: Some(point(100)),
            applicability: TemporalInterval {
                start: Some(point(100)),
                end: Some(point(100)),
            },
            publication_time: point(110),
            provenance: TemporalProvenance::EngineeringClaim,
        };
        let passed = TemporalAssertion {
            event_time: Some(point(200)),
            applicability: TemporalInterval {
                start: Some(point(200)),
                end: Some(point(200)),
            },
            publication_time: point(210),
            provenance: TemporalProvenance::EngineeringClaim,
        };
        assert!(failed.validate().is_ok());
        assert!(passed.validate().is_ok());
    }

    #[test]
    fn external_authority_time_remains_attributed() {
        let assertion = TemporalAssertion {
            event_time: Some(point(100)),
            applicability: TemporalInterval {
                start: Some(point(100)),
                end: Some(point(200)),
            },
            publication_time: point(300),
            provenance: TemporalProvenance::ExternalAuthority,
        };
        assert!(assertion.validate().is_ok());
    }
}
