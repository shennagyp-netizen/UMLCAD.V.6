# UMLCAD V6 — S12 Continuation Handoff

## Station status

S12 is the active station until its exit gate is satisfied and merged to `main`.

The branch is `v6-s12-curve-surface-operations` and the work is carried by PR #44.

## Completed S12 semantic capabilities

- exact affine-planar line/NURBS-surface intersection with explicit transverse, disjoint, and coplanar/underdetermined outcomes;
- deterministic multi-seed NURBS curve/surface isolated-root solver with explicit ambiguity handling;
- conservative isolated tangent detection and coincident/underdetermined handling for supported curve/surface cases;
- bounded affine-planar surface/surface intersection with exact plane-plane construction and UV clipping;
- bounded planar point/surface closest-point and distance oracle;
- intersection-driven deterministic line splitting that fails closed on ambiguity;
- positive-weight NURBS control-net convex-hull broad phase for certified surface-pair disjointness;
- general surface-pair dispatcher that returns deterministic `NoIntersection` when disjointness is certified and otherwise delegates only to a proven solver family;
- opaque OCCT realizations with semantic-first conformance against independently computed expectations.

## S12 completion gate

S12 may be marked complete only when the following are all true:

1. representative curve/surface and surface/surface operations have independent semantic definitions;
2. invalid, degenerate, tangent, coincident, ambiguous, and unsupported cases have explicit outcomes;
3. numerical tests cover non-unit parameter domains and deterministic ordering;
4. OCCT realization is validated against the semantic authority rather than defining it;
5. intersection-driven operations do not create arbitrary topology from unverified numerical traces;
6. authoritative CI is fully green, including debug/release workspace tests, Clippy, kernel format/tests, and kernel Clippy gates;
7. PR #44 is merged to `main` without bypassing the CI gate.

## Scope boundary

The branch must not claim a generic exhaustive NURBS/NURBS intersection-curve solver merely because a broad-phase or numerical seed solver exists. General potential contact outside the proven solver families remains explicitly unsupported until an independently validated continuation/curve-isolation method is added.

## Post-S12 continuation

After merge, create the next station from `main` rather than continuing to accumulate S12 work on the merged branch:

- **S13:** offsets and healing with explicit repair authority and failure semantics;
- **S14:** freeform feature generation extensions;
- **S15:** robust B-Rep topology completion;
- **S16:** deterministic kernel integration and provenance/reference stability;
- **S17:** performance and stress validation;
- **S18:** V6 part-geometry-kernel completion gate.

The next implementation should preserve the invariant:

```text
semantic mathematics -> backend-neutral contract -> opaque native realization -> independent conformance -> topology
```

No later station may turn an unsupported or ambiguous S12 result into silently accepted geometry.