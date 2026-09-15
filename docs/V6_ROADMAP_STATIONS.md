# UMLCAD V6 — Roadmap Stations

This document is the live station map for the V6 geometry-kernel program. The mathematical/semantic layer is developed ahead of native realization; stations below therefore distinguish semantic authority from backend implementation and conformance.

## Current station

**S10 — NURBS surface differential/backend conformance — NEXT / CURRENT**

S8 and S9 are complete. The semantic NURBS surface mathematics and exact differential contracts already exist; S9 added the OCCT reference realization without making OCCT the mathematical authority.

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

**S9 exit condition:** satisfied by the full authoritative OCCT TDD matrix: workspace debug/release tests, Clippy, kernel debug/release tests, kernel source/test Clippy, and formatting gates all passed on PR #41 before merge to `main`.

## Immediate next station

### S10 — NURBS surface differential/backend conformance

The goal is **not** to invent the mathematical layer; it already exists. The goal is to prove that the native surface realization is mathematically faithful to it.

Required work:

1. expose or add backend measurements/evaluation needed to compare native NURBS surface points with the semantic evaluator;
2. compare first and second partial derivatives against the independent mathematical implementation/oracle;
3. verify normals and cross-product orientation where regular;
4. explicitly classify singular/degenerate parameter cases rather than returning plausible but invalid values;
5. test rational and non-rational surfaces, interior knots, non-unit parameter domains, and asymmetric U/V degrees;
6. preserve tolerance separation: mathematical comparison tolerance must not become hidden modeling tolerance;
7. establish deterministic conformance evidence suitable for later trimmed B-Rep construction.

**Station exit condition:** native and mathematical results agree within explicitly declared numerical bounds across regular and adversarial cases, with singular cases explicitly diagnosed and no OCCT types exposed through the semantic API.

## Following stations

### S11 — Freeform face construction and trimming

Promote validated NURBS surfaces into B-Rep faces with explicit trimming wires/edges. Establish orientation, parameter-space versus 3D curve consistency, closure, seam handling, and manifold validation. No guessed topology identity.

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
