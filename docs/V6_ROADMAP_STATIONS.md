# UMLCAD V6 — Roadmap Stations

This document is the live station map for the V6 geometry-kernel program. The mathematical/semantic layer is developed ahead of native realization; stations therefore distinguish semantic authority, backend-neutral contracts, and native conformance.

## Completed stations

### S8 — Interchange + visualization boundary hardening — COMPLETE

Analytic primitives, transforms, Booleans, extrusion, revolution, loft, fillet, chamfer, rational NURBS curves/surfaces, circular-arc sweep, mesh validation/tessellation, STEP/IGES exchange, and the opaque OCCT boundary are established. OCCT DataExchange remains serialized because release validation exposed unsafe concurrent lifetime behavior.

### S9 — Native tensor-product NURBS surface backend — COMPLETE

The validated backend-neutral `NurbsSurface3DDefinition` is realized by OCCT `Geom_BSplineSurface`, including complete knot-vector conversion, 2D rational weights, preserved `u * count_v + v` control-net ordering, opaque native results, and deterministic construction tests.

### S10 — NURBS surface differential/backend conformance — COMPLETE

Independent first/second differential mathematics, rational homogeneous-to-Euclidean quotient rules, normals, continuity policy, and deterministic edge cases are validated against native OCCT `Geom_Surface::D2` behavior. Full debug/release/format/Clippy/kernel gates passed.

### S11 — Freeform face construction and trimming — COMPLETE

Backend-neutral trimmed NURBS surface definitions now validate UV-domain containment, loop closure/orientation, simplicity, holes, and loop intersections. OCCT creates native p-curves and 3D boundary curves behind the opaque boundary, with deterministic topology/area/validity tests. S11 passed its full authoritative CI gate.

## Current station

### S12 — Surface-surface / curve-surface operations — COMPLETE

S12 establishes the representative geometric operations needed to derive and modify freeform boundaries without promoting numerical guesses into topology.

Completed capabilities:

1. Backend-neutral line-segment / NURBS-surface intersection with deterministic multi-seed Newton isolation and exact affine-planar classification.
2. Explicit handling for unique roots, no intersection, tangent contact, and coplanar/coincident or underdetermined relations in the supported cases.
3. General NURBS-curve / NURBS-surface isolated-root semantics using an independent rational curve evaluator, explicit parameter domains, deterministic three-variable Newton isolation, root ordering, ambiguity reporting, and conservative tangent classification.
4. Exact degree-1 linear-curve handling through the established line/surface semantic authority, preserving the curve's real parameter interval.
5. Backend-neutral point/surface closest-point and distance semantics for the supported affine-planar NURBS family, including arbitrary UV domains and boundary projection.
6. Exact affine-planar NURBS surface/surface intersection through an independent plane-plane oracle with bounded UV-domain clipping and explicit parallel/coincident behavior.
7. Intersection-driven deterministic line splitting; ambiguous intersection evidence fails closed and cannot become arbitrary split topology.
8. A certified general NURBS surface-pair broad phase using the positive-weight control-net convex-hull property. Strictly separated control-net AABBs certify disjointness; overlapping bounds are only `PotentialContact`.
9. `intersect_nurbs_surfaces` consumes that broad phase first. Certified disjoint pairs produce deterministic `NoIntersection`; potential-contact pairs are delegated only to a proven exact solver family, otherwise returning `UnsupportedSurfaceFamily`.
10. Independent semantic expectations are checked against opaque OCCT realizations using `GeomAPI_IntCS`, `GeomAPI_IntSS`, and `GeomAPI_ProjectPointOnSurf`; OCCT does not define semantic meaning.
11. Deterministic ordering, explicit tolerance/domain policies, validation-before-FFI, and fail-closed unsupported/error paths are covered by the authoritative test matrix.
12. The S12 surface-intersection boundary and continuation requirements are documented in `docs/S12_SURFACE_INTERSECTION_CONTRACT.md` and `docs/S12_CONTINUATION_HANDOFF.md`.

S12 scope boundary:

- The implemented exact surface/surface intersection curve construction is intentionally limited to the affine-planar patch family.
- The general surface-pair dispatcher can certify disjointness for arbitrary valid positive-weight NURBS surfaces, but it does not claim exhaustive arbitrary NURBS/NURBS intersection-curve isolation.
- General potential contact outside the proven solver families remains explicitly unsupported rather than approximated.
- General freeform intersection-curve tracing and generalized trimming remain later kernel work; no renderer approximation is promoted into topology.

**S12 exit gate:** satisfied by the authoritative full CI matrix on PR #44: workspace debug/release tests, format, Clippy, kernel format/tests, and kernel Clippy gates all green, with semantic/native conformance tests passing.

### S13 — Offsets and healing — IN PROGRESS

The first bounded S13 slice establishes exact backend-neutral offsets for oriented planar line segments and planar rectangular surface patches, with OCCT realization and independent conformance tests. General NURBS offsets, self-intersection/trimming, corner joining, shell/thickness, and generic healing remain explicitly unimplemented.

The healing contract requires explicit defect classes, permitted changes, tolerance budget, invariants, repair evidence, and failure/unsupported outcomes. A generic backend `make valid` result is not accepted as UMLCAD healing authority.

See `docs/S13_OFFSET_HEALING_CONTRACT.md`.

## Following stations

### S14 — Freeform feature generation

Extend robust sweep/pipe, variable-radius sweep, blend/fillet extensions, shell/thickness, drafted surfaces, and related freeform construction primitives.

### S15 — Robust B-Rep topology kernel completion

Strengthen sewing, shell construction, orientation propagation, degeneracy handling, repair, imported-pathology validation, and topology evidence until advanced B-Rep cases have explicit pass/fail/unsupported outcomes.

### S16 — Deterministic kernel integration

Complete backend-neutral operation graphs/provenance, stable semantic topology references, immutable snapshots, cancellation/error semantics, and service integration without making OCCT state a semantic dependency.

### S17 — Kernel performance and stress gate

Measure repeated freeform construction, Booleans, tessellation, exchange, clone/drop cycles, numerical edge cases, and FFI overhead; optimize only after correctness remains stable.

### S18 — V6 geometry-kernel completion gate

V6 completion means the applicable geometry/B-Rep/freeform contracts required for a SolidWorks/Inventor-class **part geometry kernel** are implemented and green. Assemblies, kinematics, drawings, FEA, and machine-design application features remain subsequent system layers.

## Scope discipline

The station pipeline is:

```text
mathematical / semantic authority
        ↓
backend-neutral contract
        ↓
native realization (OCCT reference backend)
        ↓
independent conformance against the mathematics
        ↓
B-Rep topology / advanced freeform operations
```

A backend never defines semantics. A station is never complete merely because code exists or a renderer displays a shape; the applicable TDD, adversarial, determinism, integration, release, and CI gates must pass.