# Mycelix ActivateCredential Binding v0.1

This theorem is the live association seam between the EK and AK.

Current tpm2-tools documentation states that ActivateCredential validates the credentialed object in a way that, for attestation, guarantees the attestation key belongs to the TPM with a qualified parent key. The MakeCredential documentation describes the privacy-preserving protocol as using the EK public key and only the credentialed object's Name, so the protocol can establish same-TPM association without disclosing the AK public key. citeturn130096search0turn130096search9

The capture path uses a fresh registrar secret. The final receipt contains only hashes; the raw challenge and recovered secret remain restricted capture artifacts. A PASS therefore requires successful makecredential and activatecredential execution plus exact secret-byte equality. The theorem may return PASS for a genuine LiveVerifierSession receipt; the separate qualification ceiling remains ReferenceModelOnly, so this does not by itself promote the overall platform to hardware-qualified.

This theorem deliberately does not replace AK creation certification, EK certificate trust, Quote verification, or measured-boot reconstruction.

Claim ceiling remains ReferenceModelOnly.
