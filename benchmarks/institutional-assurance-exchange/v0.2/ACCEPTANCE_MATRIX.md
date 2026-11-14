# IAE v0.2 acceptance matrix

| ID | Test | Expected |
|---|---|---|
| VS-01 | Exact brief byte spans and modalities | 4 candidates match source byte spans and modality |
| VS-02 | Mandatory title sourced from custody-only record | BLOCK; competence mismatch |
| VS-03 | Preferred inspection absent | Does not block historical lease |
| VS-04 | Title evidence removed | Lease BLOCK |
| VS-05 | Independent condition inspection with title/custody omitted | ALLOW_SCOPED if actor authority is active |
| VS-06 | Authority revoked after earlier historical decision | Earlier view remains active; current view is revoked |
| VS-07 | Late correction effective before earlier decision but learned later | Historical query sees v4; later knowledge query sees v5 |
| VS-08 | Unrelated condition claim added | Historical lease disposition and reason codes invariant |
| VS-09 | External submission timed out after possible mutation | UNKNOWN / HOLD_FOR_RECONCILIATION; retry=0 |
| VS-10 | Receipt/source/requirement/authority trace missing | Structural qualification failure |
| VS-11 | Write trace targets another asset, adds an unauthorized field, or uses an unapproved external effect | Unauthorized side-effect failure |
| VS-11 | Network or mutation enabled in synthetic fixture mode | Reject fixture |
| VS-12 | Exact input replay | Same deterministic selection and disposition for identical frozen inputs |

The evaluator's development oracle covers VS-01–VS-09 and fixture safety. VS-10–VS-12 are additionally checked by structural guards and deterministic implementation tests; none constitutes an external qualification.
