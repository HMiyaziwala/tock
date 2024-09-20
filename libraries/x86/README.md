x86
==============

This library is a trimmed clone of [rust-x86](https://github.com/gz/rust-x86),
pulled in for Tock's x86 arch crate.

For complete documentation please visit
https://docs.rs/x86/latest/x86/

Status / Why a Copy?
--------------------

Due to auditability concerns, Tock currently has a policy against linking in
external code. For a simpler library, the approach taken here was to audit a
snapshot and include only what was needed.
