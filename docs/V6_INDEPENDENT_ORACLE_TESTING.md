# UMLCAD V6 — Independent Oracle Testing

## Purpose

V6 must be tested from more than one epistemic direction. The OCCT backend is the reference geometric realization, but a test that calls OCCT and then checks the same OCCT-derived property is not an independent mathematical proof.

This document defines an additional adversarial layer: **independent-oracle tests**.

The oracle derives the expected result from mathematics, geometry definitions, invariants, or an independently implemented calculation rather than reproducing the implementation under test.

## What this protects against

Independent oracles are intended to detect:

- semantic regressions hidden by implementation-aligned tests;
- algorithms that claim exactness while using approximation;
- incorrect tolerance scaling;
- incorrect domain handling;
- algebraic or topological invariant violations;
- operation-order mistakes;
- accidental dependence on backend traversal details;
- backend-specific behavior escaping into UMLCAD semantics.

## Required separation

The following are different authorities and must not be conflated:

```text
UMLCAD semantic contract
        ↓
mathematical / engineering oracle
        ↓
implementation under test
        ↓
backend realization
```

For OCCT-backed geometry, OCCT is the realization under test. It is not automatically the independent oracle for every asserted property.

## Oracle classes

### 1. Analytic geometry

Use closed-form geometry where available.

Examples:

- line length = Euclidean endpoint distance;
- circle circumference = `2πr`;
- primitive bounds from their defining parameters;
- transformed coordinates from explicit affine equations.

### 2. Topological invariants

Use independently derived invariant relationships where the topology is sufficiently constrained.

For the canonical box:

```text
solid = 1
shell = 1
face = 6
edge = 12
vertex = 8
V - E + F = 2
```

The invariant is a cross-check, not a replacement for explicit topology validation.

### 3. Algebraic transform laws

Examples:

- zero translation is identity;
- composition of translations is vector addition;
- full-turn rotation is identity within declared validation tolerance;
- source geometry remains unchanged after functional transforms.

### 4. Metamorphic relations

When a direct closed-form oracle is unavailable, derive relationships that must remain true after a controlled transformation.

Examples:

- translating a shape must translate every bounding-box coordinate by the same vector;
- applying two translations must equal their vector sum;
- equivalent independent executions must produce equivalent semantic evidence.

## V6 test rule

An independent-oracle test must not calculate its expected value by repeating the same backend operation being tested.

Bad:

```text
expected = OCCT_measure(shape)
actual   = UMLCAD_measure(shape)
```

Good:

```text
expected = analytic_formula(parameters)
actual   = UMLCAD_measure(shape)
```

For backend-specific tolerances, the expected mathematical value remains exact and the acceptance envelope is defined by the V6 validation tolerance and documented backend behavior.

## Blind external review

An independent AI/code review may be used as an adversarial source of candidate defects. Its findings are not automatically accepted.

Each finding must be classified:

```text
candidate finding
      ↓
reproduce against current V6 source
      ↓
derive expected behavior independently
      ↓
check V6 contract
      ↓
┌──────────────┬────────────────┬──────────────────┐
│ confirmed    │ intentional    │ unsupported      │
│ defect       │ semantics      │ claim            │
└──────┬───────┴───────┬────────┴────────┬─────────┘
       ↓               ↓                 ↓
regression test   document contract   discard/review
       ↓
fix implementation
```

A review that accidentally analyzes an older kernel or stale repository artifact is useful only for the properties it actually inspected. It must not be used to redefine V6 semantics merely because the older implementation behaved differently.

## Current V6 oracle coverage

The initial independent-oracle suite covers:

- analytic axis-aligned bounds for box, cylinder, sphere, cone, and ring torus;
- canonical box topology counts and Euler invariant;
- exact translation of bounding-box coordinates;
- translation composition;
- full-turn rotation equivalence to identity within validation tolerance.

These tests intentionally sit beside the existing OCCT conformance tests rather than replacing them.

## Promotion rule

Every independently discovered V6 correctness defect must become a permanent regression contract.

A feature is not complete merely because the OCCT implementation and its direct tests are green. Where a mathematical or metamorphic oracle exists, it should be included in the applicable V6 gate.
