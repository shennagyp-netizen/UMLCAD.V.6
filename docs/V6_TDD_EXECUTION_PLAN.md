# UMLCAD V6 — TDD Execution Plan

## 0. Authority and status

This document is the authoritative implementation plan for the V6 geometry-kernel work on branch `v6-occt-tdd`.

V6 is engineering-first. Geometry is a realization of an engineering model, not the engineering model itself.

The implementation rule is strict:

```text
RED → implement the smallest correct change → GREEN → refactor → repeat
```

No feature is considered implemented because code compiles, an API exists, or a sample renders. A feature is implemented only when its contract tests, adversarial tests, integration tests, and required determinism/accuracy checks are green.

The branch is not called complete until the full applicable test gate is green.

## 1. Frozen V6 architecture

```text
Engineering intent / semantic model
            ↓
     UMLCAD Geometry API
            ↓
    backend adapter boundary
            ↓
   native C++ OCCT reference backend
            ↓
       OCCT geometry kernel
            ↓
 B-Rep / topology / surfaces / NURBS
```

The OCCT C++ implementation is the first reference backend because it gives UMLCAD direct access to the native OCCT API without depending on an immature Rust wrapper.

Rust remains an acceptable host/orchestration language where already required by the repository, but the authoritative OCCT implementation is C++.

The backend adapter owns the translation between UMLCAD contracts and OCCT. OCCT types, lifetimes, exception mechanics, tolerance conventions, and backend-specific IDs must not leak through the public UMLCAD geometry contract.

### 1.1 Authority boundaries

**UMLCAD owns:**

- engineering semantics;
- semantic identities;
- immutable inputs/snapshots;
- operation contracts;
- dependency/provenance semantics;
- acceptance criteria;
- deterministic result semantics;
- diagnostics/evidence exposed to clients;
- backend-independent regression fixtures.

**OCCT owns:**

- geometric realization;
- B-Rep topology;
- geometric algorithms;
- geometric validity as assessed by OCCT algorithms;
- native geometric tolerances required by its algorithms.

OCCT validity never replaces UMLCAD semantic validity.

## 2. Non-negotiable TDD rules

1. Every new operation begins with a failing contract test.
2. The first implementation must be the smallest implementation capable of making that contract green.
3. Refactoring occurs only after green tests.
4. Every discovered defect becomes a regression test before the defect is fixed.
5. Negative tests are first-class tests, not optional cleanup.
6. Backend-specific tests verify OCCT behavior; contract tests verify UMLCAD behavior independent of backend.
7. Determinism tests run repeatedly and compare stable semantic results, not pointer addresses.
8. No cache may become a source of semantic truth.
9. No tolerance widening is allowed merely to turn a failure into success.
10. Unsupported behavior must return an explicit stable unsupported result until it is actually implemented.
11. A green unit test suite is necessary but insufficient: integration, adversarial, and release-mode tests must also pass.
12. Do not mark a phase complete with an ignored test that represents required V6 behavior.

## 3. Test pyramid and gates

Each operation moves through these gates:

```text
G0 compile/type gate
G1 unit contract gate
G2 backend conformance gate
G3 red-team/adversarial gate
G4 determinism gate
G5 integration boundary gate
G6 release/performance smoke gate
G7 CI gate
```

A phase is green only when all applicable gates are green.

### 3.1 Failure policy

A failed test is evidence, not a reason to weaken the contract.

For every failure:

```text
reproduce
→ classify contract/backend/test defect
→ add or strengthen regression test
→ fix implementation or test
→ rerun focused tests
→ rerun full gate
```

## 4. Geometry contract to implement first

The first V6 backend contract is deliberately small:

- construct an axis-aligned solid box;
- reject invalid dimensions before entering OCCT;
- validate the resulting solid;
- translate an existing immutable shape into a new shape;
- preserve validation status through translation;
- return explicit backend/tolerance evidence;
- maintain ownership safety across Rust/C++ boundaries;
- never expose raw OCCT pointers through the UMLCAD API.

The first primitive is intentionally simple because the purpose of the phase is to prove the boundary, not to claim a complete CAD kernel.

## 5. Phase A — make the contract genuinely RED

### A1. Replace placeholder OCCT tests

Turn the currently ignored box/translation contract tests into mandatory tests.

Tests must cover:

- positive finite dimensions succeed;
- zero dimensions reject;
- negative dimensions reject;
- NaN dimensions reject;
- positive infinity rejects;
- negative infinity rejects;
- invalid tolerance rejects;
- finite translation succeeds;
- NaN/infinite translation rejects;
- validation of source and translated shape succeeds;
- source remains valid after translation;
- translation creates a distinct immutable result.

Do not use generic assertions that require backend shape types to implement `Debug` or `PartialEq`.

### A2. Add boundary ownership tests

The Rust-side shape handle must prove:

- destruction is exactly-once;
- cloned handles remain valid independently;
- dropping one clone does not invalidate another;
- a failed native call does not leak an allocated shape;
- a null native handle is rejected rather than represented as valid geometry.

These tests may use C++ test instrumentation in debug/test builds where direct leak detection is otherwise impractical.

### A3. Expected state

At the end of Phase A, the branch must be RED for the right reason: the required OCCT operation is absent, not because of an invalid test harness.

## 6. Phase B — native C++ OCCT backend

### B1. C++ project

Create a small CMake target under the V6 Rust geometry workspace, for example:

```text
rust/crates/occt-backend/
├── CMakeLists.txt
├── cpp/
│   ├── bridge.hpp
│   ├── bridge.cpp
│   └── test hooks where required
├── build.rs
└── src/
    └── lib.rs
```

The exact tree may change during implementation, but the boundary must remain explicit.

### B2. Native OCCT operations

Implement the first operations with native OCCT APIs:

- `BRepPrimAPI_MakeBox` for construction;
- `BRepBuilderAPI_Transform` plus `gp_Trsf` for translation;
- `BRepCheck_Analyzer` for geometric validity;
- topology exploration sufficient to prove that a primitive box contains a solid;
- explicit native error/status conversion.

C++ exceptions must not cross an FFI boundary.

### B3. ABI boundary

Expose a narrow C ABI from the C++ library. The public Rust crate should see opaque handles and POD-like status values only.

The C ABI must provide deterministic ownership semantics for:

```text
create box
clone shape
translate shape
validate shape
release shape
```

Native OCCT objects remain entirely behind the C++ boundary.

### B4. Lifetime model

Use RAII inside C++ and an owning Rust handle around the opaque native object.

Do not declare `Send` or `Sync` for the native shape handle until thread-safety has been experimentally and contractually established.

## 7. Phase C — first GREEN

The first real GREEN milestone is:

```text
cargo test --workspace --all-targets
```

with the OCCT tests enabled rather than ignored, using a supported OCCT installation in CI.

The milestone is valid only when:

- the C++ bridge compiles;
- OCCT links correctly;
- box creation is green;
- invalid-input tests are green;
- translation is green;
- validation is green;
- ownership tests are green;
- existing geometry API unit tests remain green.

No CI claim is made until an actual GitHub Actions run succeeds.

## 8. Phase D — red-team geometry suite

After the first GREEN, deliberately attack the backend.

### D1. Numeric boundary attacks

Test geometry dimensions and transforms around:

- `0`;
- machine epsilon-scale values;
- modeling tolerance;
- validation tolerance;
- just-above/just-below tolerance;
- `1e-12`, `1e-9`, `1e-6`;
- `1e3`, `1e6`, and other large engineering scales;
- mixed large coordinates plus small features;
- NaN;
- positive/negative infinity.

The objective is to ensure that numeric scale does not silently alter the UMLCAD semantic decision.

### D2. Immutability attacks

Repeated operations must prove:

- source geometry does not mutate when an operation returns a new shape;
- failure leaves the original shape unchanged;
- repeated evaluation of identical immutable inputs gives equivalent semantic results.

### D3. Topology attacks

For the primitive baseline test:

- zero-volume bodies reject;
- invalid topology is never returned as valid;
- solid/manifold status is explicit;
- validation failures remain failures even when a renderer could display the shape.

### D4. Resource attacks

Repeated create/clone/translate/validate/release cycles must not show handle growth in the test instrumentation.

## 9. Phase E — .NET sample geometry corpus

Before implementing more operations, inspect the actual existing `.NET` sample/test projects in V6 and inventory the geometries that are genuinely present.

Do not invent a geometry corpus from memory.

For every imported fixture record:

- source project/file;
- source construction method;
- intended geometric meaning;
- expected topology;
- exact/analytic versus tolerance-based expectations;
- whether it exercises a new OCCT capability.

Prioritize fixtures that represent real engineering usage rather than visually interesting shapes.

Convert discovered geometry cases into reproducible backend-neutral fixtures where possible.

## 10. Phase F — expand operations one contract at a time

Every operation follows the same cycle:

```text
write contract tests
→ prove RED
→ implement native OCCT path
→ GREEN
→ add adversarial tests
→ GREEN
→ determinism test
→ GREEN
→ integration test
→ GREEN
→ performance smoke
→ GREEN
```

Order of expansion:

1. transforms (translation, rotation, scaling where semantically valid);
2. primitive solids;
3. curves and analytic surfaces;
4. topology inspection and stable geometric references;
5. Boolean union/difference/intersection;
6. extrusions and revolutions;
7. sweeps/lofts;
8. fillets/chamfers;
9. offsets and healing;
10. NURBS/freeform surfaces;
11. tessellation for visualization;
12. STEP/IGES exchange;
13. advanced assembly-related geometry operations.

The list is not permission to implement everything at once. Each line is a separate red-green gate.

## 11. Semantic reference strategy

OCCT topology objects are not UMLCAD semantic identities.

V6 must define a stable mapping layer for semantic references to geometric/topological targets.

Requirements:

- no array-index identity;
- no pointer-address identity;
- no renderer-derived identity;
- no unstable traversal order used as semantic identity;
- references must survive equivalent regeneration when the contract says they should;
- ambiguity must produce an explicit ambiguous/indeterminate result rather than a guessed target.

Topology-reference tests are mandatory before assembly or feature-history work depends on them.

## 12. Accuracy and tolerance discipline

Maintain separate concepts for:

```text
engineering acceptance tolerance
OCCT/modeling tolerance
validation/measurement tolerance
```

The adapter must not silently replace one with another.

Every result that depends materially on tolerance should expose enough evidence to determine:

- which tolerance context was used;
- backend identity/version where relevant;
- whether validity was exact/analytic or tolerance-based;
- whether the result is valid, invalid, ambiguous, indeterminate, or unsupported.

A wider tolerance may never be introduced solely to make a test pass.

## 13. Determinism gate

For deterministic operations, run the same immutable request repeatedly and compare:

- semantic result status;
- topology counts where contractually relevant;
- stable measurements within declared tolerances;
- serialized backend-independent result evidence.

Do not compare native pointer values or process-local addresses.

Where OCCT is nondeterministic internally but UMLCAD semantics are deterministic, normalize only at the UMLCAD boundary and test that normalization explicitly.

## 14. Integration with the existing .NET boundary

Only after the native OCCT geometry contract is green should it become the backend of the existing .NET→Rust integration path.

Required integration sequence:

```text
.NET framework input
→ existing client/protocol boundary
→ Rust host/service
→ UMLCAD geometry contract
→ C++ OCCT backend
→ validated result
→ protocol response
```

Existing Rust kernel client and end-to-end tests remain regression gates.

Invalid geometry, stale reference, timeout/cancellation, malformed response, wrong build identity, and concurrent-request tests must continue to pass.

## 15. Performance gate

Performance work starts only after correctness is green.

Measure at minimum:

- primitive construction latency;
- validation latency;
- transform latency;
- clone/copy cost;
- FFI overhead;
- repeated-operation throughput;
- peak allocation/handle count where instrumentation permits.

Optimize only without changing observable semantics.

The first optimization targets are expected to be boundary crossings, unnecessary copies, and repeated validation—not speculative algorithm replacement.

## 16. CI gate

The CI workflow must eventually execute at least:

```text
cargo fmt --all --check
cargo test --workspace --all-targets
cargo test --workspace --all-targets --release
cargo clippy --workspace --all-targets -- -D warnings
```

CI must install a known supported OCCT development environment and build the C++ backend from source.

The exact supported Linux package/version will be pinned by the workflow once the first green CI environment is established. Local success without CI success is not a completion claim.

## 17. Completion criteria for the first V6 geometry milestone

The milestone is complete only when all are true:

- native C++ OCCT backend is used;
- no required OCCT contract test is ignored;
- box/create/validate/translate contracts are green;
- invalid numeric/tolerance inputs are green;
- ownership and lifetime tests are green;
- red-team numeric/topology/resource tests are green;
- determinism tests are green;
- existing .NET/Rust regression tests remain green where applicable;
- release-mode tests are green;
- CI is green;
- documentation records the exact implemented boundary and known limitations.

## 18. V6 expansion policy

The first green milestone does **not** mean V6 is a feature-complete SolidWorks-class kernel.

It means the architecture and implementation method are proven with a scientifically testable native OCCT reference backend.

Further V6 work adds engineering capabilities through the same contracts until the defined V6 product boundary is complete.

A future native UMLCAD geometry kernel may replace individual OCCT capabilities only when it is demonstrably correct under the same contract and regression suite, with equal-or-better accuracy, determinism, and measured performance.
