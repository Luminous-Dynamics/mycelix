# Mycelix TPM Public Area -> Name Coherence v0.1

This theorem closes the object-identity seam beneath AK/EK lineage.

tpm2_readpublic returns the public area, object Name, and Qualified Name from the TPM. Current tpm2-tools documentation exposes all three outputs and describes Qualified Name as parent-binding material. The Name is derived from the object's TPMT_PUBLIC using its nameAlg. citeturn345367search0turn947860search1

The theorem checks:

NAME = nameAlg || Hash(TPMT_PUBLIC)

For TPM2B_PUBLIC the outer size field is excluded from the hash. v0.1 deliberately does not parse every TPMT_PUBLIC field; it establishes exact-byte identity and leaves complete template semantics to later layers.

A Name/hash match alone is not residency proof, manufacturer authenticity, AK/EK lineage, or measured-boot validity. Those remain separate propositions.

The deterministic corpus covers both supported wire formats and adversarial substitutions. The claim ceiling remains ReferenceModelOnly.