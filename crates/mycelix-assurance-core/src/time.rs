//! Normative time primitives.

/// Errors constructing time intervals.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TimeError {
    /// A finite half-open interval requires `end > start`.
    InvalidFiniteInterval,
}

/// Signed Unix time in microseconds.
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct UnixMicros(i64);

impl UnixMicros {
    /// Construct a Unix-microsecond timestamp.
    pub const fn new(value: i64) -> Self {
        Self(value)
    }

    /// Return the raw Unix-microsecond value.
    pub const fn get(self) -> i64 {
        self.0
    }
}

/// Explicit end semantics for a validity interval.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ValidityEnd {
    /// Finite exclusive end bound.
    At(UnixMicros),
    /// Explicitly unbounded validity. This is never inferred from absence.
    Unbounded,
}

/// Half-open validity interval `[valid_from, valid_until)` when finite.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ValidityInterval {
    valid_from: UnixMicros,
    valid_until: ValidityEnd,
}

impl ValidityInterval {
    /// Construct a validity interval, rejecting empty or reversed finite ranges.
    pub const fn new(
        valid_from: UnixMicros,
        valid_until: ValidityEnd,
    ) -> Result<Self, TimeError> {
        if let ValidityEnd::At(end) = valid_until {
            if end.get() <= valid_from.get() {
                return Err(TimeError::InvalidFiniteInterval);
            }
        }

        Ok(Self {
            valid_from,
            valid_until,
        })
    }

    /// Inclusive start of the interval.
    pub const fn valid_from(self) -> UnixMicros {
        self.valid_from
    }

    /// Exclusive finite end, or explicit unbounded state.
    pub const fn valid_until(self) -> ValidityEnd {
        self.valid_until
    }

    /// Return whether the supplied instant is within the interval.
    pub const fn contains(self, instant: UnixMicros) -> bool {
        if instant.get() < self.valid_from.get() {
            return false;
        }

        match self.valid_until {
            ValidityEnd::At(end) => instant.get() < end.get(),
            ValidityEnd::Unbounded => true,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn finite_intervals_are_half_open() {
        let interval = ValidityInterval::new(
            UnixMicros::new(10),
            ValidityEnd::At(UnixMicros::new(20)),
        )
        .expect("valid interval");

        assert!(interval.contains(UnixMicros::new(10)));
        assert!(interval.contains(UnixMicros::new(19)));
        assert!(!interval.contains(UnixMicros::new(20)));
    }

    #[test]
    fn finite_interval_rejects_nonpositive_width() {
        assert_eq!(
            ValidityInterval::new(
                UnixMicros::new(10),
                ValidityEnd::At(UnixMicros::new(10)),
            ),
            Err(TimeError::InvalidFiniteInterval)
        );
    }

    #[test]
    fn unbounded_is_explicit() {
        let interval = ValidityInterval::new(UnixMicros::new(10), ValidityEnd::Unbounded)
            .expect("explicit unbounded interval");
        assert!(interval.contains(UnixMicros::new(i64::MAX)));
    }
}
