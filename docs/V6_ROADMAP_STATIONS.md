# UMLCAD V6 — Roadmap Stations

This document is the live station map for the V6 geometry-kernel program. The mathematical/semantic layer is developed ahead of native realization; stations below therefore distinguish semantic authority from backend implementation and conformance.

## Current station

**S12 — Surface-surface / curve-surface operations — CURRENT**

S8, S9, S10, and S11 are complete. The NURBS surface mathematical authority, native realization, differential conformance, and trimmed-face B-Rep boundary layer are now established.

## Completed stations

### S8 — Interchange + visualization boundary hardening — COMPLETE

- analytic primitives, transforms, Booleans, extrusion, revolution, loft, fillet, and chamfer contracts are established;
- 2D/3D B-spline and rational NURBS curve contracts are established;
- tensor-product rational NURBS surface semantics and exact differential geometry are established;
- circular-arc sweep semantics are established;
- backend-neutral freeform APIs are established;
- deterministic mesh validation and the OCCT tessellation adapter are established;
- STEP/IGES import/export contracts and the OCCT DataExchange adapter are implemented;
- all native OCCT objects remain behind the backend boundary;
- imported exchange geometry remains ordinary UMLCAD geometry subject to validation and semantic classification;
- OCCT DataExchange operations are serialized at the backend boundary because the release gate exposed unsafe concurrent DataExchange lifetime behavior.

### S9 — Native tensor-product NURBS surface backend — COMPLETE

The already-defined backend-neutral `NurbsSurface3DDefinition` contract is now realized by the OCCT reference backend:

1. semantic definition validation occurs before FFI;
2. complete U/V knot vectors are converted to OCCT distinct knots plus multiplicities;
3. `Geom_BSplineSurface` is constructed with the 2D rational weight net;
4. semantic U/V control-net ordering `u * count_v + v` is preserved;
5. the native result crosses the boundary only as an opaque UMLCAD backend shape;
6. bilinear, rational, invalid-definition, and deterministic-construction tests are present;
7. surface construction remains separate from future trimmed-face topology.

**S9 exit condition:** satisfied by the full authoritative OCCT TDD matrix on PR #41 before merge to `main`.

### S10 — NURBS surface differential/backend conformance — COMPLETE

The objective was to prove that the native surface realization remains faithful to the independent semantic mathematics rather than using OCCT as the authority.

Completed work:

1. backend-neutral first/second differential contract;
2. independent tensor-product rational differential evaluator with explicit homogeneous-to-Euclidean quotient rules;
3. normal computation with explicit degenerate-normal handling;
4. second-order continuity policy for interior knot queries;
5. native OCCT `Geom_Surface::D2` measurement behind the opaque backend boundary;
6. numerical differential conformance between native OCCT results and the independent semantic evaluator;
7. non-finite parameter rejection and deterministic native behavior;
8. low-degree surface handling where second derivatives are identically zero;
9. full release/debug, format, Clippy, kernel test, and kernel Clippy gates passed on PR #42.

**S10 exit condition:** satisfied by the fully green authoritative CI matrix on PR #42. Merge commit: `ae5b0236cda3a792f27b63883835190e3c149f90`.

### S11 — Freeform face construction and trimming — COMPLETE

Promote validated NURBS surfaces into B-Rep faces with explicit trimming wires/edges. Establish orientation, parameter-space trimming, closure, and native face validity without exposing OCCT topology through the semantic API.

Completed work:

1. backend-neutral trimmed NURBS surface definition independent of OCCT topology types;
2. explicit outer and inner UV trimming-loop semantics;
3. semantic validation of finite coordinates, domain containment, loop closure, orientation, simplicity, hole containment, and loop intersections;
4. native OCCT construction using NURBS-surface p-curves;
5. native 3D boundary-curve completion using `BRepLib::BuildCurves3d`;
6. opaque native B-Rep face boundary;
7. deterministic tests for valid rectangular trims, holes, topology, planar area, and fail-closed invalid/self-intersecting inputs;
8. correct separation between surface validity and the repository's solid-only manifold indicator;
9. full authoritative debug/release, format, Clippy, kernel tests, and kernel Clippy gates passed on PR #43.

**S11 exit condition:** satisfied by the fully green authoritative CI matrix on PR #43. Merge commit: `1c1d00af46cf5bad8ba4ef71648efe97852585b8`.

## Current station

### S12 — Surface-surface / curve-surface operations

Establish the geometric operations required to derive and modify freeform boundaries from existing curves and surfaces while preserving mathematical authority and deterministic semantics.

Required work:

1. define backend-neutral curve/surface and surface/surface intersection contracts;
2. define projection, closest-point, and distance contracts with explicit convergence/failure semantics;
3. support curve splitting and surface trimming driven by computed intersections, not renderer approximations;
4. distinguish isolated roots, tangent contact, coincident geometry, and overlapping/underdetermined intersections;
5. establish numerical tolerances, parameter-domain behavior, and degeneracy handling before native implementation;
6. provide independent geometric/numerical oracles for representative analytic and NURBS cases;
7. realize supported operations through OCCT only after semantic validation, keeping native intersection algorithms behind opaque boundaries;
8. preserve deterministic ordering/evidence and fail closed on ambiguous or unsupported solutions.

Current S12 implementation checkpoint:

- line-segment / NURBS-surface intersection has a backend-neutral contract, deterministic multi-seed Newton authority, exact affine-planar classification, OCCT `GeomAPI_IntCS` realization, segment-domain filtering, and native-point conformance;
- exact affine-planar line/surface classification distinguishes transverse intersection, parallel-disjoint `NoIntersection`, and coplanar `CoincidentOrUnderdetermined` relations before generic Newton;
- generalized NURBS-curve / NURBS-surface semantics use an independent rational curve evaluator, explicit curve/surface parameter domains, deterministic three-variable Newton isolation, local per-seed failure handling, deterministic root ordering, and ambiguity reporting;
- isolated NURBS curve/surface tangency is conservatively classified from an independent curve-derivative/surface-normal test, with numerically co-located tangent roots clustered rather than misreported as several distinct intersections;
- exact degree-1, two-control-point, unit-weight NURBS curves reuse the established planar line/surface semantic authority, including explicit coplanar/underdetermined handling and mapping back to the curve's actual parameter domain;
- planar affine 2x2 NURBS surface/surface intersection has an independent exact plane-plane oracle, bounded UV-domain clipping, explicit distinction between parallel-disjoint and coincident/underdetermined cases, OCCT `GeomAPI_IntSS` realization, and native intersection-endpoint conformance;
- planar affine 2x2 NURBS point/surface closest-point and distance semantics have an independent bounded-parallelogram oracle and OCCT `GeomAPI_ProjectPointOnSurf` conformance;
- a certified plane-vs-NURBS relation primitive now uses control-net half-space evidence: strict one-sided control points certify disjointness, all control points within tolerance certify coplanarity within tolerance, and mixed-sign control nets remain explicitly `Undetermined` rather than being converted into an unproven intersection claim;
- the certified plane-vs-NURBS relation exposes an explicit typed error contract for non-finite inputs, invalid surfaces, degenerate planes, and numerical failure;
- arbitrary NURBS parameter intervals are handled explicitly by normalized semantic coordinates rather than assuming `[0,1]` domains;
- native OCCT remains opaque to the semantic crates;
- the semantic/root solvers remain the authority: native OCCT results are checked against independently computed expectations rather than defining semantics from the backend;
- the current surface/surface semantic implementation is intentionally restricted to exact affine planar patches and is not yet a general NURBS surface/surface root solver or intersection-curve generator;
- generalized surface trimming from computed intersections remains outstanding, as does full topology evidence for generated intersection boundaries;
- authoritative CI run #562 is fully green across workspace debug/release tests, Clippy, kernel format, kernel debug/release tests, and both kernel Clippy gates.

**S12 exit condition:** representative curve-surface and surface-surface operations are independently defined, numerically validated, natively realized, deterministic, and proven not to silently convert ambiguous geometric relations into arbitrary topology.

## Following stations

### S13 — Offsets and healing

Implement offset surfaces/curves and controlled healing with explicit failure states. Healing may repair geometry only under declared rules; it must never silently alter engineering intent or semantic identity.

### S14 — Freeform feature generation

Add robust sweep/pipe, variable-radius sweep, blend/fillet extensions, shell/thickness, drafted surfaces, and other freeform feature primitives required for SolidWorks/Inventor-class part generation.

### S15 — Robust B-Rep topology kernel completion

Strengthen sewing, shell construction, topology repair, orientation propagation, degeneracy handling, and imported-pathology validation until advanced B-Rep cases have explicit pass/fail/unsupported outcomes.

### S16 — Deterministic kernel integration

Complete the backend-neutral operation graph/provenance boundary, stable semantic topology references, immutable snapshots, cancellation/error semantics, and .NET/service integration without making OCCT state a semantic dependency.

### S17 — Kernel performance and stress gate

Measure repeated freeform construction, Boolean operations, tessellation, exchange, clone/drop cycles, and FFI overhead. Optimize only after correctness is stable.

### S18 — V6 geometry-kernel completion gate

V6 kernel completion means the applicable geometry/B-Rep/freeform contracts required for a SolidWorks/Inventor-class **part geometry kernel** are implemented and green. Assemblies, kinematics, drawings, FEA, and machine-design application features remain subsequent system layers, not hidden dependencies of this gate.

## Scope discipline

The station map distinguishes three layers:

```text
mathematical / semantic authority
        ↓
backend-neutral contract
        ↓
native realization (OCCT reference backend)
        ↓
backend conformance against the mathematics
        ↓
B-Rep topology / advanced freeform operations
```

A backend may implement the mathematics, but it never defines the semantics. A station is never marked complete merely because code exists or a renderer displays a shape. The applicable TDD, adversarial, determinism, integration, release, and CI gates must pass.
