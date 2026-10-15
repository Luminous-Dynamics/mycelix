//! Bounded synthetic planar-polygon research kernel.
//!
//! Coordinates are integer millimetres in one declared synthetic local Cartesian
//! frame. This module does not parse GeoJSON/AIXM, transform geographic CRSs,
//! model uncertainty envelopes, or provide any operational aviation authority.

use std::cmp::Ordering;

use serde::{Deserialize, Serialize};

use super::{FlightIntent, IntentState, PositionMm};

const MAX_LOCAL_COORDINATE_MM: i64 = 1_000_000_000;
const MAX_POLYGON_VERTICES: usize = 512;
const MAX_POLYGON_HOLES: usize = 64;
const MAX_GEOMETRY_HORIZON_S: i64 = 7 * 24 * 60 * 60;

const POLYGON_AUTHORITY_CEILING: &str =
    "synthetic local-planar polygon centerline impact screen only; no geographic transformation, uncertainty reachability, flight authorization, or safety determination";

#[derive(Debug, Clone, Copy, Deserialize, Serialize, PartialEq, Eq)]
pub struct PolygonPointMm {
    pub x: i64,
    pub y: i64,
}

/// Static horizontal polygon plus a closed vertical band and half-open validity.
/// Ring coordinates are local Cartesian millimetres, not longitude/latitude.
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct PolygonExclusionVolume {
    pub constraint_id: String,
    pub authority_id: String,
    /// Opaque fixture/source label only; this crate does not verify a digest.
    pub source_digest: String,
    pub reference_frame: String,
    pub vertical_datum: String,
    pub time_basis: String,
    pub valid_from_s: i64,
    pub valid_until_s: i64,
    pub floor_z_mm: i64,
    pub ceiling_z_mm: i64,
    /// Explicitly closed exterior ring; last point must equal first.
    pub exterior_ring: Vec<PolygonPointMm>,
    /// Explicitly closed interior rings, representing excluded holes.
    pub holes: Vec<Vec<PolygonPointMm>>,
}

#[derive(Debug, Clone, Copy, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum PolygonIntersectionOutcome {
    ImpactCandidate,
    NoIntersectionInProvidedGeometry,
    Reject,
    Unresolved,
}

#[derive(Debug, Clone, Copy, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum PolygonGeometryError {
    EmptyIdentity,
    UnqualifiedReference,
    UnqualifiedFlightTime,
    IncompatibleReferenceFrame,
    IncompatibleVerticalDatum,
    IncompatibleTimeBasis,
    UnsupportedUncertaintyModel,
    InvalidValidityInterval,
    InvalidVerticalBand,
    InvalidFlightWindow,
    InvalidFlightGeometry,
    TooManyVertices,
    TooManyHoles,
    RingTooShort,
    RingNotClosed,
    CoordinateOutOfBounds,
    DuplicateAdjacentVertex,
    DegenerateRing,
    SelfIntersection,
    HoleOutsideExterior,
    HoleIntersection,
    WorkBudgetExceeded,
}

#[derive(Debug, Clone, Serialize)]
pub struct PolygonEvaluationReport {
    pub outcome: PolygonIntersectionOutcome,
    pub error: Option<PolygonGeometryError>,
    pub checked_start_s: Option<i64>,
    pub checked_end_s_exclusive: Option<i64>,
    pub authority_ceiling: &'static str,
}

impl PolygonEvaluationReport {
    fn result(
        outcome: PolygonIntersectionOutcome,
        error: Option<PolygonGeometryError>,
        interval: Option<(i64, i64)>,
    ) -> Self {
        Self {
            outcome,
            error,
            checked_start_s: interval.map(|value| value.0),
            checked_end_s_exclusive: interval.map(|value| value.1),
            authority_ceiling: POLYGON_AUTHORITY_CEILING,
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct Fraction {
    numerator: i128,
    denominator: i128,
}

impl Fraction {
    const ZERO: Self = Self { numerator: 0, denominator: 1 };
    const ONE: Self = Self { numerator: 1, denominator: 1 };

    fn new(numerator: i128, denominator: i128) -> Self {
        debug_assert_ne!(denominator, 0);
        if denominator < 0 {
            Self { numerator: -numerator, denominator: -denominator }
        } else {
            Self { numerator, denominator }
        }
    }

    fn compare(self, other: Self) -> Ordering {
        (self.numerator * other.denominator).cmp(&(other.numerator * self.denominator))
    }

    fn within_unit_interval(self) -> bool {
        self.compare(Self::ZERO) != Ordering::Less
            && self.compare(Self::ONE) != Ordering::Greater
    }

    /// The mediant lies strictly between two distinct non-negative fractions.
    /// Unlike averaging fractions, it adds rather than multiplies denominators.
    fn mediant(self, other: Self) -> Self {
        debug_assert_eq!(self.compare(other), Ordering::Less);
        Self::new(self.numerator + other.numerator, self.denominator + other.denominator)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PointLocation {
    Outside,
    Boundary,
    Inside,
}

fn cross(ax: i128, ay: i128, bx: i128, by: i128) -> i128 {
    ax * by - ay * bx
}

fn orientation(a: PolygonPointMm, b: PolygonPointMm, c: PolygonPointMm) -> i128 {
    cross(
        i128::from(b.x) - i128::from(a.x),
        i128::from(b.y) - i128::from(a.y),
        i128::from(c.x) - i128::from(a.x),
        i128::from(c.y) - i128::from(a.y),
    )
}

fn point_on_segment(
    point: PolygonPointMm,
    a: PolygonPointMm,
    b: PolygonPointMm,
) -> bool {
    orientation(a, b, point) == 0
        && point.x >= a.x.min(b.x)
        && point.x <= a.x.max(b.x)
        && point.y >= a.y.min(b.y)
        && point.y <= a.y.max(b.y)
}

fn segments_intersect(
    a: PolygonPointMm,
    b: PolygonPointMm,
    c: PolygonPointMm,
    d: PolygonPointMm,
) -> bool {
    let o1 = orientation(a, b, c);
    let o2 = orientation(a, b, d);
    let o3 = orientation(c, d, a);
    let o4 = orientation(c, d, b);

    if (o1 > 0 && o2 < 0 || o1 < 0 && o2 > 0)
        && (o3 > 0 && o4 < 0 || o3 < 0 && o4 > 0)
    {
        return true;
    }
    (o1 == 0 && point_on_segment(c, a, b))
        || (o2 == 0 && point_on_segment(d, a, b))
        || (o3 == 0 && point_on_segment(a, c, d))
        || (o4 == 0 && point_on_segment(b, c, d))
}

fn signed_area_twice(ring: &[PolygonPointMm]) -> i128 {
    ring.windows(2).fold(0_i128, |area, pair| {
        area + i128::from(pair[0].x) * i128::from(pair[1].y)
            - i128::from(pair[1].x) * i128::from(pair[0].y)
    })
}

fn validate_ring(ring: &[PolygonPointMm]) -> Result<(), PolygonGeometryError> {
    if ring.len() < 4 {
        return Err(PolygonGeometryError::RingTooShort);
    }
    if ring.first() != ring.last() {
        return Err(PolygonGeometryError::RingNotClosed);
    }
    if ring.len() > MAX_POLYGON_VERTICES {
        return Err(PolygonGeometryError::TooManyVertices);
    }
    if ring.iter().any(|point| {
        point.x.unsigned_abs() > MAX_LOCAL_COORDINATE_MM as u64
            || point.y.unsigned_abs() > MAX_LOCAL_COORDINATE_MM as u64
    }) {
        return Err(PolygonGeometryError::CoordinateOutOfBounds);
    }
    if ring.windows(2).any(|pair| pair[0] == pair[1]) {
        return Err(PolygonGeometryError::DuplicateAdjacentVertex);
    }

    let edge_count = ring.len() - 1;
    // Reject adjacent collinear backtracking, while allowing a straight-through
    // collinear vertex that does not overlap either adjacent edge.
    for index in 0..edge_count {
        let previous = ring[(index + edge_count - 1) % edge_count];
        let current = ring[index];
        let next = ring[(index + 1) % edge_count];
        if orientation(previous, current, next) == 0
            && (point_on_segment(next, previous, current)
                || point_on_segment(previous, current, next))
        {
            return Err(PolygonGeometryError::SelfIntersection);
        }
    }

    // Ring and total-vertex bounds cap this quadratic topology check.
    for i in 0..edge_count {
        for j in (i + 1)..edge_count {
            if j == i + 1 || (i == 0 && j + 1 == edge_count) {
                continue;
            }
            if segments_intersect(ring[i], ring[i + 1], ring[j], ring[j + 1]) {
                return Err(PolygonGeometryError::SelfIntersection);
            }
        }
    }
    if signed_area_twice(ring) == 0 {
        return Err(PolygonGeometryError::DegenerateRing);
    }
    Ok(())
}

fn point_location_at_fraction(
    path_start: PolygonPointMm,
    path_end: PolygonPointMm,
    parameter: Fraction,
    ring: &[PolygonPointMm],
) -> PointLocation {
    let denominator = parameter.denominator;
    let x_numerator = i128::from(path_start.x) * denominator
        + (i128::from(path_end.x) - i128::from(path_start.x)) * parameter.numerator;
    let y_numerator = i128::from(path_start.y) * denominator
        + (i128::from(path_end.y) - i128::from(path_start.y)) * parameter.numerator;
    let mut inside = false;

    for pair in ring.windows(2) {
        let a = pair[0];
        let b = pair[1];
        let ax = i128::from(a.x) * denominator;
        let ay = i128::from(a.y) * denominator;
        let bx = i128::from(b.x) * denominator;
        let by = i128::from(b.y) * denominator;

        // Keep the edge vector unscaled and the query displacement scaled by
        // the positive parameter denominator. Scaling both would square that
        // denominator and could overflow i128 at the declared coordinate limits.
        let edge_dx = i128::from(b.x) - i128::from(a.x);
        let edge_dy = i128::from(b.y) - i128::from(a.y);
        let edge_cross = edge_dx * (y_numerator - ay)
            - edge_dy * (x_numerator - ax);
        if edge_cross == 0
            && x_numerator >= ax.min(bx)
            && x_numerator <= ax.max(bx)
            && y_numerator >= ay.min(by)
            && y_numerator <= ay.max(by)
        {
            return PointLocation::Boundary;
        }

        let straddles = (ay > y_numerator) != (by > y_numerator);
        if straddles {
            if (by > ay && edge_cross > 0) || (by < ay && edge_cross < 0) {
                inside = !inside;
            }
        }
    }

    if inside { PointLocation::Inside } else { PointLocation::Outside }
}

fn validate_polygon_volume(
    volume: &PolygonExclusionVolume,
) -> Result<(), PolygonGeometryError> {
    if volume.constraint_id.trim().is_empty()
        || volume.authority_id.trim().is_empty()
        || volume.source_digest.trim().is_empty()
    {
        return Err(PolygonGeometryError::EmptyIdentity);
    }
    if volume.reference_frame.trim().is_empty()
        || volume.vertical_datum.trim().is_empty()
        || volume.time_basis.trim().is_empty()
    {
        return Err(PolygonGeometryError::UnqualifiedReference);
    }
    let duration = volume.valid_until_s.checked_sub(volume.valid_from_s);
    if duration.is_none()
        || duration.unwrap_or(0) <= 0
        || duration.unwrap_or(i64::MAX) > MAX_GEOMETRY_HORIZON_S
    {
        return Err(PolygonGeometryError::InvalidValidityInterval);
    }
    if volume.floor_z_mm > volume.ceiling_z_mm
        || volume.floor_z_mm.unsigned_abs() > MAX_LOCAL_COORDINATE_MM as u64
        || volume.ceiling_z_mm.unsigned_abs() > MAX_LOCAL_COORDINATE_MM as u64
    {
        return Err(PolygonGeometryError::InvalidVerticalBand);
    }
    if volume.holes.len() > MAX_POLYGON_HOLES {
        return Err(PolygonGeometryError::TooManyHoles);
    }

    let total_vertices = volume.exterior_ring.len()
        + volume.holes.iter().map(Vec::len).sum::<usize>();
    if total_vertices > MAX_POLYGON_VERTICES {
        return Err(PolygonGeometryError::TooManyVertices);
    }

    validate_ring(&volume.exterior_ring)?;
    for hole in &volume.holes {
        validate_ring(hole)?;
    }

    for hole in &volume.holes {
        if point_location_at_fraction(
            hole[0], hole[0], Fraction::ZERO, &volume.exterior_ring,
        ) != PointLocation::Inside
        {
            return Err(PolygonGeometryError::HoleOutsideExterior);
        }
        for outer_edge in volume.exterior_ring.windows(2) {
            for hole_edge in hole.windows(2) {
                if segments_intersect(
                    outer_edge[0], outer_edge[1], hole_edge[0], hole_edge[1],
                ) {
                    return Err(PolygonGeometryError::HoleIntersection);
                }
            }
        }
    }

    for i in 0..volume.holes.len() {
        for j in (i + 1)..volume.holes.len() {
            for edge_a in volume.holes[i].windows(2) {
                for edge_b in volume.holes[j].windows(2) {
                    if segments_intersect(
                        edge_a[0], edge_a[1], edge_b[0], edge_b[1],
                    ) {
                        return Err(PolygonGeometryError::HoleIntersection);
                    }
                }
            }
            let a_in_b = point_location_at_fraction(
                volume.holes[i][0], volume.holes[i][0], Fraction::ZERO, &volume.holes[j],
            ) != PointLocation::Outside;
            let b_in_a = point_location_at_fraction(
                volume.holes[j][0], volume.holes[j][0], Fraction::ZERO, &volume.holes[i],
            ) != PointLocation::Outside;
            if a_in_b || b_in_a {
                return Err(PolygonGeometryError::HoleIntersection);
            }
        }
    }

    Ok(())
}

fn axis_parameter_interval(
    start: i64,
    end: i64,
    minimum: i64,
    maximum: i64,
) -> Option<(Fraction, Fraction)> {
    if start == end {
        return (start >= minimum && start <= maximum)
            .then_some((Fraction::ZERO, Fraction::ONE));
    }
    let delta = i128::from(end) - i128::from(start);
    let first = Fraction::new(i128::from(minimum) - i128::from(start), delta);
    let second = Fraction::new(i128::from(maximum) - i128::from(start), delta);
    let mut enter = if first.compare(second) == Ordering::Greater { second } else { first };
    let mut leave = if first.compare(second) == Ordering::Greater { first } else { second };

    if enter.compare(Fraction::ZERO) == Ordering::Less {
        enter = Fraction::ZERO;
    }
    if leave.compare(Fraction::ONE) == Ordering::Greater {
        leave = Fraction::ONE;
    }
    (enter.compare(leave) != Ordering::Greater).then_some((enter, leave))
}

fn parameter_for_point(
    point: PolygonPointMm,
    start: PolygonPointMm,
    end: PolygonPointMm,
) -> Option<Fraction> {
    if !point_on_segment(point, start, end) {
        return None;
    }
    let dx = i128::from(end.x) - i128::from(start.x);
    let dy = i128::from(end.y) - i128::from(start.y);
    if dx == 0 && dy == 0 {
        return Some(Fraction::ZERO);
    }
    let (numerator, denominator) = if dx.unsigned_abs() >= dy.unsigned_abs() {
        (i128::from(point.x) - i128::from(start.x), dx)
    } else {
        (i128::from(point.y) - i128::from(start.y), dy)
    };
    let parameter = Fraction::new(numerator, denominator);
    parameter.within_unit_interval().then_some(parameter)
}

fn add_segment_intersection_parameters(
    path_start: PolygonPointMm,
    path_end: PolygonPointMm,
    edge_start: PolygonPointMm,
    edge_end: PolygonPointMm,
    output: &mut Vec<Fraction>,
) {
    let rx = i128::from(path_end.x) - i128::from(path_start.x);
    let ry = i128::from(path_end.y) - i128::from(path_start.y);
    let sx = i128::from(edge_end.x) - i128::from(edge_start.x);
    let sy = i128::from(edge_end.y) - i128::from(edge_start.y);
    let qpx = i128::from(edge_start.x) - i128::from(path_start.x);
    let qpy = i128::from(edge_start.y) - i128::from(path_start.y);
    let denominator = cross(rx, ry, sx, sy);

    if denominator != 0 {
        let mut t_numerator = cross(qpx, qpy, sx, sy);
        let mut u_numerator = cross(qpx, qpy, rx, ry);
        let mut positive_denominator = denominator;
        if positive_denominator < 0 {
            positive_denominator = -positive_denominator;
            t_numerator = -t_numerator;
            u_numerator = -u_numerator;
        }
        let t = Fraction::new(t_numerator, positive_denominator);
        let u = Fraction::new(u_numerator, positive_denominator);
        if t.within_unit_interval() && u.within_unit_interval() {
            output.push(t);
        }
        return;
    }

    if cross(qpx, qpy, rx, ry) != 0 {
        return;
    }
    if let Some(parameter) = parameter_for_point(edge_start, path_start, path_end) {
        output.push(parameter);
    }
    if let Some(parameter) = parameter_for_point(edge_end, path_start, path_end) {
        output.push(parameter);
    }
    if path_start == path_end && point_on_segment(path_start, edge_start, edge_end) {
        output.push(Fraction::ZERO);
    }
}

fn polygon_covers_at_fraction(
    path_start: PolygonPointMm,
    path_end: PolygonPointMm,
    parameter: Fraction,
    volume: &PolygonExclusionVolume,
) -> bool {
    let exterior = point_location_at_fraction(
        path_start, path_end, parameter, &volume.exterior_ring,
    );
    if exterior == PointLocation::Outside {
        return false;
    }
    // Spatial boundaries are closed for conservative screening. A hole interior
    // is excluded, but touching a hole boundary is still an impact candidate.
    !volume.holes.iter().any(|hole| {
        point_location_at_fraction(path_start, path_end, parameter, hole)
            == PointLocation::Inside
    })
}

fn position_at_local_time(
    flight: &FlightIntent,
    time_s: i64,
) -> Option<PositionMm> {
    let elapsed = i128::from(time_s) - i128::from(flight.valid_from_s);
    let position = flight.position_at_valid_from_mm;
    let velocity = flight.velocity_mm_s;
    let result = PositionMm {
        x: i64::try_from(i128::from(position.x) + i128::from(velocity.x) * elapsed).ok()?,
        y: i64::try_from(i128::from(position.y) + i128::from(velocity.y) * elapsed).ok()?,
        z: i64::try_from(i128::from(position.z) + i128::from(velocity.z) * elapsed).ok()?,
    };
    (result.x.unsigned_abs() <= MAX_LOCAL_COORDINATE_MM as u64
        && result.y.unsigned_abs() <= MAX_LOCAL_COORDINATE_MM as u64
        && result.z.unsigned_abs() <= MAX_LOCAL_COORDINATE_MM as u64)
        .then_some(result)
}

fn unresolved(error: PolygonGeometryError) -> PolygonEvaluationReport {
    PolygonEvaluationReport::result(
        PolygonIntersectionOutcome::Unresolved, Some(error), None,
    )
}

fn rejected(error: PolygonGeometryError) -> PolygonEvaluationReport {
    PolygonEvaluationReport::result(
        PolygonIntersectionOutcome::Reject, Some(error), None,
    )
}

/// Screen a constant-velocity centerline against a static local-planar polygon
/// and closed vertical band during [start, end). Nonzero uncertainty is unresolved
/// because this kernel intentionally does not claim to bound trajectory tubes.
pub fn evaluate_polygon_intersection(
    flight: &FlightIntent,
    volume: &PolygonExclusionVolume,
    now_s: i64,
) -> PolygonEvaluationReport {
    if let Err(error) = validate_polygon_volume(volume) {
        return match error {
            PolygonGeometryError::UnqualifiedReference => unresolved(error),
            _ => rejected(error),
        };
    }

    if flight.flight_id.trim().is_empty() {
        return rejected(PolygonGeometryError::EmptyIdentity);
    }
    if !flight.time_qualified {
        return unresolved(PolygonGeometryError::UnqualifiedFlightTime);
    }
    if flight.reference_frame.trim().is_empty()
        || flight.vertical_datum.trim().is_empty()
        || flight.time_basis.trim().is_empty()
    {
        return unresolved(PolygonGeometryError::UnqualifiedReference);
    }
    if flight.reference_frame != volume.reference_frame {
        return unresolved(PolygonGeometryError::IncompatibleReferenceFrame);
    }
    if flight.vertical_datum != volume.vertical_datum {
        return unresolved(PolygonGeometryError::IncompatibleVerticalDatum);
    }
    if flight.time_basis != volume.time_basis {
        return unresolved(PolygonGeometryError::IncompatibleTimeBasis);
    }
    if flight.horizontal_uncertainty_mm != 0 || flight.vertical_uncertainty_mm != 0 {
        return unresolved(PolygonGeometryError::UnsupportedUncertaintyModel);
    }

    let flight_duration = flight.valid_until_s.checked_sub(flight.valid_from_s);
    if flight_duration.is_none()
        || flight_duration.unwrap_or(0) <= 0
        || flight_duration.unwrap_or(i64::MAX) > MAX_GEOMETRY_HORIZON_S
    {
        return rejected(PolygonGeometryError::InvalidFlightWindow);
    }
    if flight.position_at_valid_from_mm.x.unsigned_abs() > MAX_LOCAL_COORDINATE_MM as u64
        || flight.position_at_valid_from_mm.y.unsigned_abs() > MAX_LOCAL_COORDINATE_MM as u64
        || flight.position_at_valid_from_mm.z.unsigned_abs() > MAX_LOCAL_COORDINATE_MM as u64
    {
        return rejected(PolygonGeometryError::InvalidFlightGeometry);
    }

    if matches!(flight.state, IntentState::Withdrawn | IntentState::Ended) {
        return PolygonEvaluationReport::result(
            PolygonIntersectionOutcome::NoIntersectionInProvidedGeometry,
            None,
            None,
        );
    }

    let start_s = now_s.max(flight.valid_from_s).max(volume.valid_from_s);
    let end_s = flight.valid_until_s.min(volume.valid_until_s);
    if start_s >= end_s {
        return PolygonEvaluationReport::result(
            PolygonIntersectionOutcome::NoIntersectionInProvidedGeometry,
            None,
            Some((start_s, end_s)),
        );
    }
    if end_s.checked_sub(start_s).map(|duration| duration > MAX_GEOMETRY_HORIZON_S).unwrap_or(true) {
        return rejected(PolygonGeometryError::InvalidFlightWindow);
    }

    let Some(start_position) = position_at_local_time(flight, start_s) else {
        return rejected(PolygonGeometryError::InvalidFlightGeometry);
    };
    // end_s is an excluded temporal endpoint. Calculating the extrapolated point
    // there is necessary to represent the approach to that boundary continuously;
    // a hit only at parameter 1 is filtered below.
    let Some(end_position) = position_at_local_time(flight, end_s) else {
        return rejected(PolygonGeometryError::InvalidFlightGeometry);
    };

    let Some((vertical_enter, vertical_leave)) = axis_parameter_interval(
        start_position.z,
        end_position.z,
        volume.floor_z_mm,
        volume.ceiling_z_mm,
    ) else {
        return PolygonEvaluationReport::result(
            PolygonIntersectionOutcome::NoIntersectionInProvidedGeometry,
            None,
            Some((start_s, end_s)),
        );
    };

    let path_start = PolygonPointMm { x: start_position.x, y: start_position.y };
    let path_end = PolygonPointMm { x: end_position.x, y: end_position.y };
    let mut breaks = vec![Fraction::ZERO, Fraction::ONE, vertical_enter, vertical_leave];
    for ring in std::iter::once(&volume.exterior_ring).chain(volume.holes.iter()) {
        for edge in ring.windows(2) {
            add_segment_intersection_parameters(
                path_start, path_end, edge[0], edge[1], &mut breaks,
            );
        }
    }
    breaks.sort_by(|left, right| left.compare(*right));
    breaks.dedup_by(|left, right| left.compare(*right) == Ordering::Equal);

    // Event points include edge crossings and the exact vertical/time slab bounds.
    // The upper temporal endpoint u=1 is excluded; all other event points are tested
    // with exact rational point-in-ring predicates.
    for parameter in &breaks {
        if parameter.compare(vertical_enter) != Ordering::Less
            && parameter.compare(vertical_leave) != Ordering::Greater
            && parameter.compare(Fraction::ONE) == Ordering::Less
            && polygon_covers_at_fraction(path_start, path_end, *parameter, volume)
        {
            return PolygonEvaluationReport::result(
                PolygonIntersectionOutcome::ImpactCandidate,
                None,
                Some((start_s, end_s)),
            );
        }
    }

    // Between consecutive boundary events topology is constant. Use the fraction
    // mediant as an exact rational interior witness; no floating-point point-in-
    // polygon test or sampling grid is used.
    for pair in breaks.windows(2) {
        if pair[0].compare(pair[1]) != Ordering::Less {
            continue;
        }
        let witness = pair[0].mediant(pair[1]);
        if witness.compare(vertical_enter) != Ordering::Less
            && witness.compare(vertical_leave) != Ordering::Greater
            && witness.compare(Fraction::ONE) == Ordering::Less
            && polygon_covers_at_fraction(path_start, path_end, witness, volume)
        {
            return PolygonEvaluationReport::result(
                PolygonIntersectionOutcome::ImpactCandidate,
                None,
                Some((start_s, end_s)),
            );
        }
    }

    PolygonEvaluationReport::result(
        PolygonIntersectionOutcome::NoIntersectionInProvidedGeometry,
        None,
        Some((start_s, end_s)),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde::Deserialize;

    #[derive(Deserialize)]
    struct PolygonFixtureSet {
        schema: String,
        status: String,
        authority_ceiling: String,
        cases: Vec<PolygonFixtureCase>,
    }

    #[derive(Deserialize)]
    struct PolygonFixtureCase {
        case_id: String,
        now_s: i64,
        flight: FlightIntent,
        volume: PolygonExclusionVolume,
        expected_outcome: PolygonIntersectionOutcome,
    }

    fn fixture_set() -> PolygonFixtureSet {
        serde_json::from_str(include_str!("../fixtures/polygon_cases.json"))
            .expect("the committed polygon fixture set must parse")
    }

    #[test]
    fn all_committed_polygon_golden_vectors_match() {
        let set = fixture_set();
        assert_eq!(set.schema, "airspace-polygon-scenarios-v1");
        assert_eq!(set.status, "synthetic_research_fixture");
        assert_eq!(set.authority_ceiling, "synthetic_local_planar_geometry_only");
        assert!(set.cases.len() >= 18, "polygon golden-vector corpus unexpectedly shrank");

        let mismatches: Vec<_> = set.cases.iter().filter_map(|case| {
            let report = evaluate_polygon_intersection(&case.flight, &case.volume, case.now_s);
            (report.outcome != case.expected_outcome)
                .then_some((case.case_id.as_str(), case.expected_outcome, report))
        }).collect();
        assert!(mismatches.is_empty(), "polygon fixture mismatches: {mismatches:#?}");
    }

    #[test]
    fn polygon_kernel_fails_unresolved_for_nonzero_uncertainty() {
        let set = fixture_set();
        let case = set.cases.iter().find(|case| case.case_id == "uncertainty_model_unresolved")
            .expect("uncertainty negative control exists");
        let report = evaluate_polygon_intersection(&case.flight, &case.volume, case.now_s);
        assert_eq!(report.outcome, PolygonIntersectionOutcome::Unresolved);
        assert_eq!(report.error, Some(PolygonGeometryError::UnsupportedUncertaintyModel));
    }

    #[test]
    fn polygon_kernel_enforces_half_open_validity_at_excluded_end() {
        let set = fixture_set();
        let case = set.cases.iter().find(|case| case.case_id == "constraint_valid_until_boundary_is_exclusive")
            .expect("half-open end negative control exists");
        let report = evaluate_polygon_intersection(&case.flight, &case.volume, case.now_s);
        assert_eq!(report.outcome, PolygonIntersectionOutcome::NoIntersectionInProvidedGeometry);
        assert_eq!(report.checked_end_s_exclusive, Some(1));
    }

    #[test]
    fn polygon_kernel_rejects_vertex_budget_before_topology_work() {
        let mut set = fixture_set();
        let mut volume = set.cases.remove(0).volume;
        volume.exterior_ring = (0..=MAX_POLYGON_VERTICES)
            .map(|index| PolygonPointMm { x: index as i64, y: 0 })
            .collect();
        let error = validate_polygon_volume(&volume).expect_err("oversized ring must be rejected");
        assert_eq!(error, PolygonGeometryError::TooManyVertices);
    }
}
