# Mycelix EK Template Appraisal v0.1

This theorem distinguishes three separate facts: an EK-looking public key, a TCG-profile-compatible EK template, and actual creation under the endorsement hierarchy.

Current tpm2-tools documentation describes `tpm2_createek` as generating a TCG-profile-compliant EK that is the primary object of the endorsement hierarchy, with RSA-2048 selecting the low-range template by default unless a manufacturer-defined template is explicitly requested. citeturn678432search0

The current TCG EK Credential Profile Version 2.7 publishes the RSA-2048 L-1 default template: RSA type, SHA-256 name algorithm, fixedTPM/fixedParent/sensitiveDataOrigin/adminWithPolicy/restricted/decrypt attributes, the standard PolicySecret(TPM_RH_ENDORSEMENT) policy digest, AES-128-CFB symmetric parameters, NULL signing scheme, 2048-bit key, exponent zero, and a 256-byte unique field. citeturn287722search0

v0.1 parses the exact TPMT_PUBLIC wire structure for this RSA-2048 template. It intentionally does not treat template matching as proof of manufacturer certificate authenticity or hierarchy execution.

Offline and live-unintegrated creation provenance remains INDETERMINATE. A future physical capture can supply the actual `tpm2_createek` transcript and ReadPublic material, but the final hierarchy/manufacturer trust claim remains separate.

Claim ceiling: **ReferenceModelOnly**.