# UMLCAD V6 — Roadmap Stations

This document is the live station map for the V6 geometry-kernel program. The mathematical/semantic layer is developed ahead of native realization; stations below therefore distinguish semantic authority from backend implementation and conformance.

## Current station

**S11 — Freeform face construction and trimming — CURRENT**

S8, S9, and S10 are complete. The NURBS surface mathematical authority, native OCCT realization, and native-versus-mathematical differential conformance gate are now established.

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

## Current station

### S11 — Freeform face construction and trimming

Promote validated NURBS surfaces into B-Rep faces with explicit trimming wires/edges. Establish orientation, parameter-space versus 3D curve consistency, closure, seam handling, and manifold validation. No guessed topology identity.

Required work:

1. define a backend-neutral trimmed-face contract independent of OCCT topology types;
2. define explicit outer and inner trimming-loop semantics in surface parameter space;
3. construct and validate 3D boundary curves from semantic curves and surface restrictions;
4. establish wire closure, edge orientation, face orientation, and seam rules;
5. verify that trimming curves remain geometrically consistent with the underlying surface within declared validation tolerance;
6. construct an opaque native B-Rep face only after semantic validation succeeds;
7. add independent point-on-surface/curve and loop-closure checks plus adversarial self-intersection and degeneracy cases;
8. preserve deterministic references/evidence and fail closed on unsupported topology.

**S11 exit condition:** a semantic trimmed-face definition can be validated independently, realized by the OCCT backend, and proven to preserve boundary geometry, orientation, closure, and manifold validity without exposing OCCT topology through the semantic API.

## Following stations

### S12 — Surface-surface / curve-surface operations

Add intersection, projection, closest-point, split/trim, and continuity-sensitive operations needed by real feature construction. Every operation gets independent numerical or geometric oracles.

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
