# V6 Blind Review Findings — Disposition

This document records the independent review performed against the UMLCAD V6 repository without telling the reviewer that the implementation represented V6. The review is evidence for adversarial testing, not authority over V6 semantics.

## V6 semantic decisions

The following observations were intentionally retained as V6 design:

- `spatial_analysis` is an AABB broad-phase filtered analysis; the downstream engineering predicate must not pretend that this alone proves spatial correctness.
- Multi-wrap arcs are supported. Arc length retains winding length (`radius * |span|`), `point_at` remains parameterized over the full span, and point/angle containment is geometric image-set membership. For spans of at least one full turn, every angular location is therefore contained as a point-set property.
- Arc `Distance` semantics are defined by V6 and are not inherited from V5.
- `Fixed` has a snapshot-aware solver path because the fixed target is part of the authoritative base snapshot; the public geometry-only residual remains pure.
- Constraint residuals are not pre-scaled in the residual function; V6 performs column normalization in the numerical solver.
- Validation diagnostics currently use `Severity::Error` according to the V6 validation contract.

## Confirmed V6 defects and fixes

### Symmetric relation rank

The previous residual contained four equations, with the final two exactly equal to twice the first two. That representation introduced artificial rank deficiency and polluted DOF/conditioning analysis. V6 now emits only the two independent midpoint equations.

### Analytic arc distance claims

`line_arc`, `arc_circle`, and `arc_arc` were labelled `analytic-2d` while relying on a small sample set. V6 now checks analytic line-circle and circle-circle intersection candidates and filters them through arc membership, with additional exact endpoint/closest-point candidates for non-intersection distances.

### Topology self-loop degree

A vertex incident to an edge whose start and end are the same vertex contributes two incidences. V6 now counts start and end separately instead of treating the self-loop as degree one.

### Engineering spatial validity

The previous `intersects == (distance <= tolerance)` clause repeated the same predicate used to construct `intersects` and therefore added no independent evidence. V6 now treats the spatial validity check as a finiteness/integrity check rather than as a proof of intersection semantics.

### Parameter tolerance units

Spatial arc membership now delegates to the geometry's angular/set-membership contract instead of reusing the length `EPSILON` against a dimensionless normalized parameter. Segment intersection parameters also use a dedicated `PARAM_EPSILON`.

### Solver geometry lookup performance

The solver's finite-difference loop previously performed repeated linear string searches through `SemanticSnapshot.geometry`. V6 now builds one immutable per-solve `HashMap<String, usize>` index and uses it throughout candidate, residual, Jacobian, analysis, and materialization paths. Snapshot serialization and semantic structure are unchanged.

## Review claims independently disproved by V6 implementation

### `rank_condition(max.max(1.0))`

The numerical solver column-normalizes the Jacobian before SVD. Each nonzero normalized column has unit norm, so the largest singular value cannot fall below one. The review's proposed scale failure therefore does not occur in this implementation. The clamp has nevertheless been simplified to the mathematically direct `tol * max` expression, and a scale-invariance regression test now locks the behavior.

### `initial_damping = 0`

The SVD solver computes the damped pseudoinverse term `s / (s^2 + d)`. With `d = 0`, this is the ordinary pseudoinverse for nonzero singular values; zero singular values contribute zero. An underdetermined system therefore has a finite minimum-norm solution. A direct regression test now verifies zero damping on an underdetermined system.

## Disposition rule

A future external review must be replayed against the current V6 source. A finding becomes a V6 defect only after independent reproduction and contract verification. Confirmed defects require a regression test before or with the fix. A disagreement with historical V4/V5 behavior is not itself a V6 defect.
