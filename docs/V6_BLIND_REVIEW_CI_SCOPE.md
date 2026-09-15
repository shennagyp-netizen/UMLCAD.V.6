# V6 blind-review CI scope

The V6 CI gate now covers both geometry-kernel workspace tests and the semantic `kernel_rust` crate. This prevents blind-review findings in the semantic layer from remaining outside the repository's required green gate.

The semantic kernel gates are:

```text
cargo fmt --all -- --check
cargo test --all-targets
cargo test --all-targets --release
cargo clippy --all-targets -- -D warnings
```

The OCCT geometry gates remain unchanged.

A blind-review finding is promoted only through a regression test and implementation change justified by the V6 contract. Review claims that are disproven by the implementation receive regression tests when useful and remain documented as disproven claims.
