# UMLCAD V6 — Mathematical Authority

## Purpose

This document records the mathematical authority of the V6 kernel so that backend implementations cannot silently become the definition of UMLCAD geometry.

## Authority split

UMLCAD owns the semantic mathematical model:

- point/vector and transform semantics;
- B-spline and NURBS parameterization;
- de Boor evaluation in homogeneous coordinates;
- rational dehomogenization;
- exact analytic derivatives implemented by differentiated control nets and the quotient rule;
- parameter domains and validation contracts;
- control-hull bounds used as backend-independent conservative bounds;
- immutable transformation semantics;
- mathematical regression oracles and adversarial fixtures.

OCCT is a reference realization and conformance oracle. It does not define the UMLCAD semantic equations.

## Independent verification rule

For every mathematical operation that has an analytic UMLCAD implementation, at least one contract test must have an expected result derived independently from the implementation under test.

A backend comparison alone is not a mathematical proof because both implementations could share the same defect or convention.

The preferred oracle hierarchy is:

1. closed-form analytic solution;
2. independently derived polynomial/rational identity;
3. geometric invariant;
4. independent backend conformance;
5. numerical approximation only where no exact practical oracle exists.

Numerical finite differences must not replace an available analytic derivative.

## NURBS representation

For a rational curve, the semantic model is

`C(u) = (Σ N_i,p(u) w_i P_i) / (Σ N_i,p(u) w_i)`.

The homogeneous control point is `H_i = (w_i P_i, w_i)`. UMLCAD evaluates the B-spline combination in homogeneous space and performs Euclidean dehomogenization only after the basis evaluation.

The production evaluator uses the de Boor recurrence. Independent contract coverage also evaluates the normalized rational basis with the Cox–de Boor recursion. Agreement between these two formulations is therefore a mathematical regression signal independent of OCCT.

For first derivatives, the differentiated homogeneous control polygon is evaluated analytically. If `H(u) = (X(u), W(u))`, then the Euclidean derivative is

`C'(u) = (X'(u) W(u) - X(u) W'(u)) / W(u)^2`.

The tensor-product surface implementation follows the same homogeneous rule independently in U and V directions, with normals obtained from the normalized cross product of the analytic partial derivatives.

## Backend independence

An OCCT adapter may reject inputs because of backend-specific realizability limits, native tolerance requirements, or API restrictions. Such limits are backend facts, not changes to UMLCAD's mathematical definition.

A backend failure must therefore remain distinguishable from:

- invalid UMLCAD mathematical input;
- a mathematically degenerate result;
- an unsupported UMLCAD operation;
- an OCCT construction failure.

## Review rule

When a backend and the independent semantic evaluator disagree:

```text
Do not widen tolerance first.
Do not copy the backend result into the semantic layer.
Reproduce → classify → derive an independent oracle → fix the faulty side → add regression coverage.
```
