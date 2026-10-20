# mycelix-stewardship-runtime-store

A deliberately small SQLite transaction substrate for local Mycelix stewardship
authority runtimes.

This crate implements the `STEW-RUNTIME-STORE-001` source candidate with:

- persistent `page_size=4096`, `auto_vacuum=NONE`, and `encoding=UTF-8` selected before schema/WAL;
- SQLite WAL mode;
- `synchronous=FULL`;
- foreign keys enabled;
- `trusted_schema=OFF`;
- an explicit 5 second busy timeout;
- exact live `sqlite_schema` admission;
- Unix main DB and present WAL/SHM files restricted against group/other access;
- `BEGIN IMMEDIATE` for every authority-state mutation;
- one closed single-assignment state-cell operation;
- one closed compare-and-swap plus append-only transition-receipt operation;
- bounded inputs;
- full `PRAGMA integrity_check` for positive storage/backup evidence;
- cheap `quick_check` retained only as a diagnostic;
- SQLite online-backup support with full pre/post integrity checks;
- no public raw-SQL or `Connection` escape hatch.

The underlying reviewed transaction kernel remains byte-identical in `src/lib.rs` and
is private to the `src/profiled.rs` crate root. The profile wrapper adds persistent
file-format, local-file, and full-integrity admission without reimplementing CAS.

File permissions are defense in depth rather than stewardship authority. The v1
source checks file type and group/other permission bits, but does not yet claim
hostile-root resistance, exact owner/eUID binding, full ancestor-path confinement,
or network-filesystem detection in-process; those deployment facts remain part of
the exact qualification profile.

A successful transaction does **not** establish stewardship semantic authority,
capability legitimacy, external-effect success/finality, hostile-administrator
resistance, or rollback resistance against restoration of an older valid database
or VM snapshot.

`STEW-RUNTIME-ROLLBACK-001` and an exact provider profile are required when
snapshot rollback is in scope. `STEW-RUNTIME-CODE-001` separately binds the
runtime interpreter allowed to mutate authoritative state.

The source pins the direct `rusqlite` version but does not claim a qualified native
dependency closure until a Cargo-generated lock, bundled SQLite identity/build
inputs, compiler environment, and execution evidence are independently reviewed.
