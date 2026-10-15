# Airspace Geometry Interoperability Profile V1

**Status:** geographic interoperability design contract; geographic adapters/conformance not yet implemented or executed  
**Authority ceiling:** data interpretation and planning/simulation evidence only  
**Research checked:** 2026-10-15  
**Tracking:** [AIRSPACE-COMMONS-001](https://github.com/Luminous-Dynamics/mycelix/issues/5023)  
**Architecture:** [AIRSPACE_COORDINATION_ARCHITECTURE_V1.md](AIRSPACE_COORDINATION_ARCHITECTURE_V1.md)

## 1. Purpose and non-goals

Define a conservative boundary for interoperating with published aeronautical geometry and time-dependent trajectory data without inventing a competing aviation data standard. The existing axis-aligned-box evaluator remains intact. A separate `src/polygon.rs` kernel now tests synthetic local-planar polygon rings, holes and centerline/time/vertical intersections using bounded integer/rational predicates. It is not a geographic adapter. This document does **not** add CRS transformation, parse GeoJSON/AIXM, implement an OGC API, model nonzero uncertainty envelopes, or claim standards conformance.

This is an adapter/profile proposal, not a flight-safety specification. No geometry result, record signature, successful parse, reservation, or simulation may be treated as flight authorization, separation assurance, or proof that the physical airspace is safe.

## 2. Standards boundaries

| Concern | Proposed source/profile role | Explicit non-claim |
|---|---|---|
| Authoritative aeronautical data | Accept AIXM 5.1.1 as an exchange source where the responsible jurisdiction/provider publishes it and the receiving operation is approved to rely on it. Keep the original feature, source identifier, temporality and source message lineage. | AIXM parsing does not establish that the data is current, authoritative for this operation, complete, or legally applicable. |
| Moving trajectory interchange | Consider OGC Moving Features JSON for time-parameterized points or more complex moving geometries when its declared CRS/TRS and interpolation semantics fit the data. Consider OGC API — Moving Features for standard access patterns only after an authenticated endpoint and freshness profile exist. | A trajectory encoding does not define aircraft performance, uncertainty, legal priority, authorization, tactical separation, or a safe maneuver. |
| Geographic geometry syntax | Use GeoJSON/RFC 7946 only where its WGS 84 longitude/latitude convention and geometry types accurately represent the source and operation. Other CRSs require a documented prior arrangement or a separately specified encoding. | Never assume a generic GeoJSON reader will preserve foreign members or aviation-specific vertical/time semantics. |
| Geometric validity | Define and test the accepted polygon/multipolygon topology, ring closure, hole relation, dimensionality, and boundary predicate for this profile. | A parser accepting a geometry is not equivalent to proving topological validity or correct aviation interpretation. |

References: [AIXM 5.1.1 model/schema/temporality](https://aixm.aero/page/aixm-511-specification), [OGC Moving Features JSON 1.0](https://docs.ogc.org/is/19-045r3/19-045r3.html), [OGC API — Moving Features](https://www.ogc.org/standards/ogc-api-moving-features/), [RFC 7946 GeoJSON](https://www.rfc-editor.org/rfc/rfc7946/), and [OGC Simple Feature Access](https://www.ogc.org/standards/sfa/). They describe interoperable data models and formats; no single source replaces jurisdictional rules, source qualification, or an assurance case.

## 3. Canonical coordination envelope

The Mycelix-side envelope should preserve, not overwrite, source semantics. Minimum conceptual fields:

- **Identity and provenance:** profile/schema version; stable record identifier; source organization/authority reference; source feature identifier; raw-source digest computed by a defined implementation; retrieval/receipt time; issuer issue time; supersession/predecessor reference; source status and evidence of authentication.
- **Geometry:** geometry type and coordinates; dimension order; spatial CRS identifier; axis order and units; any vertical coordinate reference/datum; source boundary/interpolation semantics; validity result from a named geometry validator.
- **Time:** temporal CRS/time scale; epoch and normalization rule; source literal timestamps; `valid_from` inclusive and `valid_until` exclusive in the coordination projection; clock uncertainty and transformation provenance where relevant.
- **Transformation:** source and target CRS identifiers; operation/pipeline identifier and version; grids/resources relied upon; area of use; stated accuracy/error bound; coordinate epoch when required; output units and axis mapping.
- **Uncertainty:** model kind (deterministic bounds, covariance/probability model, or unknown); frame, units, coverage semantics, validity horizon, time-growth model and source. Never silently reinterpret a statistical confidence region as a hard bound.
- **Authority scope:** jurisdiction/operation/profile applicability; source freshness profile; explicit authority reference; result authority ceiling. This layer can say “unresolved” or “candidate impact”; it cannot mint an authorization.

Required semantic rule: if an input cannot be mapped without guessing its CRS, vertical datum, temporal basis, interpolation rule, boundary semantics, or uncertainty meaning, preserve the source and return **unresolved**. Do not silently apply a library default.

## 4. Coordinate and time rules

### 4.1 Coordinate qualification

1. A CRS must be identified by a stable, resolvable identifier from a controlled registry or by an explicitly versioned local profile. Free-text labels such as `local-frame` are acceptable only in synthetic test fixtures.
2. Coordinate axis order and units must be bound to that CRS. Longitude/latitude ordering must not be inferred from numeric magnitude.
3. Vertical coordinates must state their reference (for example, ellipsoidal height versus a named orthometric datum) and unit. “Altitude” by itself is underspecified.
4. Transform only with an identified operation whose area of use covers the geometry. Record transformation version, resources, and accuracy/error bound. Missing grids, out-of-area coordinates, unknown epochs, or unavailable transformations fail closed.
5. Propagate the declared transformation-error bound into the uncertainty representation using a specified method. If that propagation is undefined, return unresolved rather than silently buffering or dropping it.
6. Never buffer longitude/latitude values using metre-valued margins. A metric operation requires an appropriate qualified projected/local/geodesic method with domain and error bounds.

### 4.2 Time qualification

1. Keep the input time scale and source timestamp. Normalize to the profile's internal integer time scale only through a specified parser and mapping; record leap-second/offset policy and source-clock uncertainty.
2. Validate monotonic sample times and explicit interpolation semantics before constructing continuous occupancy.
3. The coordination projection uses `[valid_from, valid_until)`: the start is included and the end excluded. Translating source intervals with other boundary rules requires an explicit adapter rule; if no lossless mapping exists, return unresolved.
4. Do not equate a timestamp's numeric representation with synchronized clocks. Cross-provider offset/uncertainty must be within the consuming profile's declared limits.
5. A temporal gap, unknown interpolation, stale input, or incompatible temporal reference is not an empty volume.

## 5. Geometry and intersection semantics

The initial local-frame box evaluator is retained as a small reference oracle. A future geographic adapter must specify these before it is allowed to feed the conflict screen:

- **Boundary predicate:** adopt closed spatial occupancy for conservative screening, including edge, vertex and face tangencies. Preserve source semantics and describe how a source boundary maps into this predicate.
- **Validity/topology:** reject malformed coordinate arrays, non-finite values, invalid ring closure, self-intersections, invalid hole relationships, unsupported geometry types, empty volumes where disallowed, and coordinates outside declared limits. Do not “repair” geometry silently. A separately versioned repair operation may be evaluated later but must produce its own provenance.
- **Antimeridian and poles:** test dateline-crossing rings, multipolygon splitting, winding/edge interpretation, and polar geometries. RFC 7946 recommends cutting geometries that cross the antimeridian for interoperability; a parser must not reinterpret a 170°E-to-170°W edge as a 340° path by accident.
- **Interpolation:** declare whether edges/trajectories are planar, geodesic, rhumb-line, or use a defined time-dependent interpolation. If source semantics are absent or unsupported, do not select the most convenient default.
- **Altitude limits:** preserve floor/ceiling reference, unit, inclusivity and any flight-level/pressure-reference semantics. A 2D polygon plus an “altitude” number is not a qualified 3D volume.
- **Temporal crossing:** intersect the geometry's spatial occupancy with its own effective interval and the trajectory's validity. Use explicit half-open interval handling; do not round samples first and use the rounded values as exact truth.
- **Uncertainty envelope:** specify whether shape expansion is a deterministic worst-case bound or a probability distribution. For deterministic bounds, define frame and growth model. For statistical models, a separate approved risk interpretation is necessary; do not make a probability threshold up in this prototype.
- **Resource limits:** cap feature bytes, vertices, rings, temporal segments, transformation steps, and pairwise checks before expensive work. Exceeding a limit returns reject/degraded according to the declared failure type; it must not return “no intersection.”

## 6. Outcome mapping

| Condition | Required profile outcome | Forbidden inference |
|---|---|---|
| Valid qualified geometry and a conservative intersection is found | `ReassessmentRequired` / impact candidate | The service can command a maneuver or withdraw authorization. |
| No intersection in a complete, fresh and qualified supplied view | `NoImpactInProvidedSnapshot` | The entire airspace is conflict-free or a flight is cleared. |
| Snapshot is stale, incomplete, partitioned, or source freshness is unproved | `Degraded`, unless a known impact takes precedence | Missing records mean no restrictions exist. |
| Unknown CRS/datum/time/interpolation or unsupported geometry semantics | `Unresolved` | Numeric coordinates that look plausible are comparable. |
| Malformed geometry, duplicate identity, unsupported dimensions, invalid interval, or input/work limit violation | `Reject` | Invalid data can be sanitized implicitly and then treated as authoritative. |
| Source disagreement, contradictory version history, or incompatible source authority | `Unresolved` | Last arrival wins or a reputation/quorum score establishes physical truth. |

Known impact takes precedence over stale-view degradation, matching the dynamic-constraint fixture contract. A “no impact” outcome is scoped to the supplied set, selected model, and declared assumptions only.

## 7. Proposed golden/adversarial profile cases

The AIGEO cases remain a *future geographic-adapter conformance backlog*, not executed by the new local-planar kernel. The kernel has a separate committed synthetic fixture corpus at `tools/airspace-sim/fixtures/polygon_cases.json`; its Rust unit tests do not qualify geographic CRS transforms or formats. Implement each AIGEO case only with a selected geographic library/reference algorithm, exact source fixture data, and reviewed expected outcomes.

| Case | Input / mutation | Expected result |
|---|---|---|
| AIGEO-001 | Known CRS, vertical datum, temporal basis and supported volume | Eligible for bounded geometry evaluation; no authority claim |
| AIGEO-002 | Missing/unknown CRS or ambiguous axis order | Unresolved |
| AIGEO-003 | CRS transform requires an unavailable grid or lies outside operation area | Unresolved |
| AIGEO-004 | Missing/ambiguous vertical datum, or ellipsoidal vs orthometric heights mixed | Unresolved |
| AIGEO-005 | Antimeridian-crossing polygon with approved split/encoding | Same topological intent after round-trip |
| AIGEO-006 | Antimeridian edge interpreted using the opposite wrap | Reject/unresolved; never choose a long path silently |
| AIGEO-007 | Polygon edge, vertex, or face is tangent to trajectory/volume | Impact candidate under closed spatial boundaries |
| AIGEO-008 | Self-intersecting ring, invalid holes, open ring where closure is required | Reject; no silent repair |
| AIGEO-009 | Validity interval ends at exactly the tested time | End excluded by coordination projection |
| AIGEO-010 | Interpolation is unknown/unsupported or a temporal segment is missing | Unresolved |
| AIGEO-011 | Uncertainty model lacks units, coverage semantics, or growth horizon | Unresolved |
| AIGEO-012 | Input exceeds a declared byte/vertex/segment/work limit | Reject or declared degraded outcome; never no-impact |
| AIGEO-013 | Two sources disagree on geometry or validity for the same asserted authority/version | Unresolved with both source records retained |
| AIGEO-014 | Same identifier/version carries a different payload | Unresolved |
| AIGEO-015 | Full source and transform lineage round-trip through adapter | Equivalent canonical meaning, within the declared conversion error |
| AIGEO-016 | Stale/partitioned snapshot plus a known geometric hit | Reassessment required, not merely degraded |
| AIGEO-017 | Complete fresh snapshot and no hit under the declared bounded model | No impact in provided snapshot only; never clearance |
| AIGEO-018 | Transform/error-bound propagation would change the intersection classification but cannot be computed | Unresolved, not a crisp result |

Acceptance requires a source fixture and SHA, independent expected result, an intentionally invalid/negative control, round-trip tests where conversion applies, and exact-head evidence. A test count alone is not qualification.

## 8. Current local-planar kernel boundary

The separate Rust module `tools/airspace-sim/src/polygon.rs` implements a deliberately small first geometry kernel for synthetic local Cartesian coordinates in millimetres. It validates explicit ring closure, coordinate limits, degenerate/self-intersecting rings, hole containment/overlap, a 512-total-vertex cap, and a seven-day horizon. For straight constant-velocity centerlines it checks the simultaneous intersection of the polygon's closed spatial footprint, a closed vertical band, and the half-open flight/volume time interval. Rational segment-boundary parameters and rational interior witnesses are used rather than a temporal sampling grid.

The 28-case corpus in `tools/airspace-sim/fixtures/polygon_cases.json` exercises impacts, clear misses, tangencies, holes, simultaneous versus non-simultaneous horizontal/vertical occupancy, temporal boundaries, negative velocity, stationary paths, validity-window disjointness, coordinate/ring defects, source-frame/datum/time mismatches and unsupported uncertainty.

**This is a bounded geometry research kernel, not a geographic engine.** Coordinates are not longitude/latitude and no CRS conversion, antimeridian normalization, ellipsoidal/geodesic edge, altitude transformation, statistical uncertainty, or physical aircraft reachability is implemented. The kernel refuses any nonzero horizontal/vertical uncertainty as `unresolved`; it cannot silently claim the zero-width centerline represents a trajectory tube. Its geometry tests do not establish safety or authorization.

## 9. Qualification sequence

1. **Schema review:** approve field definitions, units, source-authority mapping and versioning. Freeze synthetic fixture schemas separately from operational sources.
2. **Adapter golden vectors:** parse and emit representative AIXM/Moving Features fixtures without dropping identifiers, temporal features, CRS/TRS, or unknown fields required for audit.
3. **Independent geometry oracle:** compare intersection results with a separately implemented/reference implementation over curated boundary and fuzz-generated cases. Preserve disagreements; do not majority-vote geometry truth.
4. **Numerical robustness:** test antimeridian/poles, near-tangent configurations, very small/large coordinates, transform error, narrow time windows, and degenerate inputs with declared finite work bounds.
5. **Security and provenance:** test maliciously huge geometries, entity-expansion/resource issues in XML parsing, bad signatures, replay, changed source payloads, and version forks. Cryptographic verification must be real and versioned before claiming it.
6. **Jurisdiction and operations review:** establish which provider/source is authoritative and applicable, whether the source is current, required operating approvals, vertical limit interpretation, emergency policy and responsibility boundaries.
7. **Only then** consider switching the conflict evaluator from synthetic local boxes to a new geometric adapter. Keep the old reference fixtures permanently as regression controls.

## 10. References checked

1. EUROCONTROL/AIXM, [AIXM 5.1.1 specification](https://aixm.aero/page/aixm-511-specification): UML data model, XML Schema and temporality concept.
2. OGC, [Moving Features Encoding Extension — JSON 1.0](https://docs.ogc.org/is/19-045r3/19-045r3.html): temporal geometry, spatial/temporal reference system members and interpolation concepts.
3. OGC, [OGC API — Moving Features](https://www.ogc.org/standards/ogc-api-moving-features/): temporal movement data query/access interface.
4. IETF, [RFC 7946 — The GeoJSON Format](https://www.rfc-editor.org/rfc/rfc7946/): WGS 84 longitude/latitude semantics and antimeridian guidance.
5. OGC, [Simple Feature Access — Part 1: Common Architecture](https://www.ogc.org/standards/sfa/): geometry model associated with a spatial reference system.

## Change discipline

This profile is not yet an implementation or qualification result. Future changes require versioned semantics, golden fixtures and negative controls. Queued CI is not a pass, and a passing data/geometry test is not an aviation safety case or permission to fly.
