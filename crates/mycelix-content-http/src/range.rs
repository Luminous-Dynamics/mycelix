#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ByteRangeV1 {
    pub start: u64,
    pub end_inclusive: u64,
}

impl ByteRangeV1 {
    pub fn len(self) -> u64 {
        self.end_inclusive - self.start + 1
    }

    /// Parsed v1 byte ranges are always non-empty by construction.
    pub const fn is_empty(self) -> bool {
        false
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RangeErrorV1 {
    UnsupportedUnit,
    MultipleRangesUnsupported,
    InvalidSyntax,
    Unsatisfiable,
}

pub fn parse_single_range_v1(value: &str, size_bytes: u64) -> Result<ByteRangeV1, RangeErrorV1> {
    let Some(spec) = value.strip_prefix("bytes=") else {
        return Err(RangeErrorV1::UnsupportedUnit);
    };
    if spec.contains(',') {
        return Err(RangeErrorV1::MultipleRangesUnsupported);
    }
    let Some((start, end)) = spec.split_once('-') else {
        return Err(RangeErrorV1::InvalidSyntax);
    };
    if size_bytes == 0 {
        return Err(RangeErrorV1::Unsatisfiable);
    }

    if start.is_empty() {
        let suffix = end
            .parse::<u64>()
            .map_err(|_| RangeErrorV1::InvalidSyntax)?;
        if suffix == 0 {
            return Err(RangeErrorV1::Unsatisfiable);
        }
        let length = suffix.min(size_bytes);
        return Ok(ByteRangeV1 {
            start: size_bytes - length,
            end_inclusive: size_bytes - 1,
        });
    }

    let start = start
        .parse::<u64>()
        .map_err(|_| RangeErrorV1::InvalidSyntax)?;
    if start >= size_bytes {
        return Err(RangeErrorV1::Unsatisfiable);
    }

    let end_inclusive = if end.is_empty() {
        size_bytes - 1
    } else {
        let requested_end = end
            .parse::<u64>()
            .map_err(|_| RangeErrorV1::InvalidSyntax)?;
        if requested_end < start {
            return Err(RangeErrorV1::Unsatisfiable);
        }
        requested_end.min(size_bytes - 1)
    };

    Ok(ByteRangeV1 {
        start,
        end_inclusive,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_closed_range() {
        let range = parse_single_range_v1("bytes=2-5", 10).unwrap();
        assert_eq!(
            range,
            ByteRangeV1 {
                start: 2,
                end_inclusive: 5
            }
        );
        assert_eq!(range.len(), 4);
        assert!(!range.is_empty());
    }

    #[test]
    fn clamps_end_to_object_size() {
        assert_eq!(
            parse_single_range_v1("bytes=8-999", 10).unwrap(),
            ByteRangeV1 {
                start: 8,
                end_inclusive: 9
            }
        );
    }

    #[test]
    fn parses_open_ended_range() {
        assert_eq!(
            parse_single_range_v1("bytes=7-", 10).unwrap(),
            ByteRangeV1 {
                start: 7,
                end_inclusive: 9
            }
        );
    }

    #[test]
    fn parses_suffix_range() {
        assert_eq!(
            parse_single_range_v1("bytes=-3", 10).unwrap(),
            ByteRangeV1 {
                start: 7,
                end_inclusive: 9
            }
        );
    }

    #[test]
    fn rejects_multi_range_and_unsatisfiable_range() {
        assert_eq!(
            parse_single_range_v1("bytes=0-1,4-5", 10),
            Err(RangeErrorV1::MultipleRangesUnsupported)
        );
        assert_eq!(
            parse_single_range_v1("bytes=10-", 10),
            Err(RangeErrorV1::Unsatisfiable)
        );
        assert_eq!(
            parse_single_range_v1("bytes=-0", 10),
            Err(RangeErrorV1::Unsatisfiable)
        );
    }
}
