# UMLCAD V6 — Roadmap Stations

This document is the live station map for the V6 geometry-kernel program.

## Current station

**S8 — Interchange + visualization boundary hardening — COMPLETE**

The project has crossed the basic B-Rep modeling boundary and has validated the two external geometry boundaries without weakening the semantic kernel:

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

**Station exit condition:** satisfied. The clean-mainline OCCT CI candidate must pass test, release, formatting, kernel, and clippy gates with the OCCT 7.6 DataExchange development package explicitly installed and verified.

## Immediate next station

**S9 — Native tensor-product NURBS surface backend**

Implement the OCCT reference realization for the already-defined backend-neutral `NurbsSurface3DDefinition` contract:

1. validate the semantic definition before FFI;
2. convert the full U/V knot vectors into OCCT distinct knots plus multiplicities;
3. construct `Geom_BSplineSurface` with the 2D rational weight net;
4. preserve the semantic U/V control-net ordering exactly;
5. expose the result only as an opaque UMLCAD backend shape;
6. create focused conformance tests for bilinear surfaces, rational surfaces, invalid weights/knots, and deterministic repeated construction;
7. keep surface construction separate from trimmed-face topology.

**Station exit condition:** exact semantic definition validation, successful OCCT construction, deterministic measurements, and no OCCT type leakage across the Rust/API boundary.

## Following stations

### S10 — NURBS surface differential/backend conformance

Connect native NURBS surfaces to analytic first/second differential measurements and independent mathematical oracles. Verify points, partial derivatives, normals, singular/degenerate cases, and tolerance separation.

### S11 — Freeform face construction and trimming

Promote validated NURBS surfaces into B-Rep faces with explicit trimming wires/edges. Establish orientation, parameter-space versus 3D curve consistency, closure, and manifold validation. No guessed topology identity.

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

The station map is intentionally ordered by dependency:

```text
semantic freeform definition
        ↓
native freeform realization
        ↓
differential geometry
        ↓
trimmed B-Rep faces
        ↓
freeform intersections / offsets / healing
        ↓
feature generation
        ↓
robust topology
        ↓
integration + performance
        ↓
V6 kernel completion
```

A station is never marked complete merely because code exists or a renderer displays a shape. The applicable TDD, adversarial, determinism, integration, release, and CI gates must pass.
