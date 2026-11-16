# NeuroTwin BIDS intake audit

**Status:** local metadata audit utility; not the official BIDS Validator, clinical data validator, ethics approval or evidence of consent.  
**Issue:** [NEUROTWIN-003 / #6281](https://github.com/Luminous-Dynamics/mycelix/issues/6281).

This isolated Rust tool inventories BIDS-like subject/session structure, compares participant identifiers with subject directories locally, and emits a versioned aggregate-only receipt.

## What it reads

- Presence and file type of dataset_description.json; its contents are not parsed.
- Presence and first TSV column of participants.tsv; participant IDs are read locally only to check uniqueness and correspondence with subject directories.
- Directory names and file metadata under recognized modality folders: anat, func, dwi, fmap, eeg, meg, ieeg, beh, perf, pet, micr and motion.

It **does not open MRI, EEG, MEG, PET, NIfTI, EDF or other modality payload contents**. It does not hash any file.

## What it emits

Only aggregate counts, stable finding codes and fixed policy flags. It never emits participant IDs, subject/session labels, filenames or filesystem paths. It skips symlinks and never follows them during traversal. Reports are suppressed below 20 subject directories; the CLI does not allow this floor to be lowered.

The report must remain inside the approved local analysis environment. Aggregate counts may still be disclosive for small or distinctive cohorts; a provider's stricter output-suppression policy always takes precedence.

## Run

From this directory:

    cargo fmt -- --check
    cargo test --locked
    cargo clippy --all-targets --locked -- -D warnings
    cargo run --locked -- DATASET_ROOT /approved/local/path/intake-receipt.json

The receipt schema is in schemas/intake-receipt-v0.1.schema.json.

## Required separate validation

This tool is not the BIDS Validator and is not a replacement for it. Every admitted imaging dataset must also be validated with the official [BIDS Validator](https://bids-standard.github.io/bids-validator/), at a version pinned in the study manifest. A BIDS Validator pass checks BIDS structure/metadata; it does not establish participant consent, lawful data use, phenotype validity, endpoint quality, leakage-free modelling or clinical utility.

The tool likewise cannot verify REB/IRB approval, a DUA, source authority, data license, or consent enforcement. Those are separate mandatory admission gates in [NEUROTWIN-001](https://github.com/Luminous-Dynamics/mycelix/issues/6276).

## Synthetic CI fixtures

scripts/create-synthetic-fixture.sh creates a disposable 21-subject or 3-subject directory tree with synthetic text placeholders. It is not a valid BIDS imaging dataset and must never be used to claim BIDS conformance. It exists only to test the aggregate metadata contract and small-cohort suppression.

No clinical datasets or participant records are stored in this repository.
