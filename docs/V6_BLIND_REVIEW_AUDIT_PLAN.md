# V6 Blind Review Audit Plan

The external review is used as a blind adversarial input. The reviewer is not told the branch/version intent. Findings are rechecked against the current V6 source and mathematical semantics.

## Required disposition flow

```text
blind finding
  ↓
reproduce against current source
  ↓
derive independent expected behavior
  ↓
check V6 contract
  ↓
confirmed → regression test → fix → full CI
intentional → document contract
false → document why the implementation already satisfies it
```

## Current verified dispositions

| Finding | V6 disposition |
|---|---|
| AABB pruning | Design; retain, but do not use self-derived `intersects` as independent evidence |
| Symmetric redundant rows | Confirmed defect; fixed |
| Arc/circle and line/arc sampled distance | Confirmed defect; replaced with analytic candidate intersections |
| Multi-wrap arcs | Intentional V6 semantics; preserve winding length and image-set containment |
| Rank threshold clamp | Not an actual failure after column normalization; simplified and scale regression added |
| Zero damping | Not a failure; SVD pseudoinverse gives a finite minimum-norm solution; regression added |
| Self-loop topology degree | Confirmed defect; fixed |
| Spatial validity tautology | Confirmed defect; fixed |
| Parameter/length epsilon mixing | Confirmed defect; fixed in spatial membership/intersection parameter handling |
| Snapshot linear lookup | Confirmed performance issue; solver now uses a per-solve immutable index |
| Arc distance semantics | V6-defined; no V5 parity assumption |
| Fixed residual split | V6-defined snapshot-aware solver behavior |

## Rule for future reviews

Do not convert an external reviewer's recommendation directly into code. Promote only independently confirmed V6 defects, and retain an explicit regression test for every promoted defect.
