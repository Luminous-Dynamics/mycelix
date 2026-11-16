# NeuroTwin controlled-data access readiness checklist v0.1

**Status:** access planning only; no application submitted, institutional sponsor secured, restricted data acquired or analysis run.  
**Tracking:** [NEUROTWIN-005 / #6289](https://github.com/Luminous-Dynamics/mycelix/issues/6289).

This checklist converts the public access instructions for CAN-BIND-1 / Brain-CODE and EMBARC / NIMH Data Archive into a safe sequence. Requirements must be rechecked with the data custodian against the exact dataset release, consent group and current application form before submission.

## 1. CAN-BIND-1 through Brain-CODE

Public Brain-CODE controlled-access guidance currently describes:

- An external researcher must be affiliated with an accredited academic institution, think tank, company or other recognized research organization.
- A local REB/IRB/REC approval or exemption is required for the requested controlled dataset. The approved protocol should explicitly name the dataset; an existing protocol may require an amendment.
- A Data Access Request is submitted after account/Study Dashboard steps. The application package includes a project title and brief description (300 words maximum), a brain-health impact description (300 words maximum), full REB protocol and approval/exemption documentation, dissemination plan, applicant CV, special considerations and a signed Data Use Agreement.
- Access to the controlled release is limited to approved users and the approved project purpose. A Brain-CODE Workspace is normally used for analysis, with MFA.
- The REB FAQ describes DAC / Informatics Steering Committee review and restrictions on re-identification, participant contact and onward disclosure. Data access and workspace terms may require annual reporting.

**Before submitting:** confirm whether the intended organization and named PI satisfy the current external-researcher rules, whether this NeuroTwin protocol is eligible for the selected CAN-BIND release, whether imaging / EEG packages are in scope, and whether aggregate evidence receipts or model artifacts may leave the workspace.

Sources:
- [Brain-CODE Data Access Request Form FAQ](https://indocconsortium.atlassian.net/wiki/spaces/JSDNXT/pages/2001895483/Data+Access+Request+Form+FAQ)
- [Brain-CODE Research Ethics Board FAQ](https://indocconsortium.atlassian.net/wiki/spaces/JSDNXT/pages/2107113500/Research+Ethics+Board+FAQ)
- [Brain-CODE Controlled Data Release FAQ](https://indocconsortium.atlassian.net/wiki/spaces/JSDNXT/pages/2105737361/Controlled+Data+Release+FAQ)

## 2. EMBARC through NIMH Data Archive

The NDA Electronic Data Access Request (eDAR) prerequisites page was last modified 7 November 2025. It states that a Lead Recipient needs:

- an NDA account affiliated with an NIH-recognized research institution;
- an institutional email address;
- an associated eRA Commons account;
- an eRA Commons affiliation with the same institution and a Principal Investigator role;
- an institution registered in eRA Commons with at least one Signing Official role and an active Federalwide Assurance (FWA);
- a research-related need for access.

The eDAR also defines approved data, project purpose, named recipients and potential cloud-computing use. Access is limited to the approved request and dataset-specific terms; new uses, recipients, cloud services or datasets may require amendments or new requests. The current NIH controlled-access DUC also describes institutional and PI responsibilities, security, non-identification, reporting and close-out for datasets within its scope.

**Important scope qualification:** do not assume the NIH Genomic Data Sharing DUC applies identically to every dataset in NDA. Confirm the exact EMBARC study page, consent group, current eDAR request type and dataset-specific access terms before deciding which certification applies.

Sources:
- [NDA eDAR prerequisites](https://nda.nih.gov/nda/tutorials/electronic-data-access-request?chapter=prerequisites-1)
- [NIH Data Use Certification Agreement](https://www.grants.nih.gov/policy-and-compliance/policy-topics/sharing-policies/accessing-data/certification-agreement)
- [NIMH Data Archive](https://nda.nih.gov/)

## 3. Recommended collaboration structure

The cross-trial benchmark depends on both data providers. The practical route is to work with an institutional Principal Investigator or clinical research group that can:
- sponsor and take responsibility for the scientific protocol;
- meet NIMH eRA Commons / Signing Official / FWA prerequisites for EMBARC access, where those apply;
- obtain local ethics approval or exemption and approved data-access terms for both data sources;
- name all approved researchers, analysts and IT / cloud operators;
- provide an approved compute environment and supervise output disclosure.

An institutional partner is not a formality. It is the accountable research sponsor for participant protection, data-use conditions, protocol deviations, security incidents, reporting and close-out. A Mycelix credential, company affiliation, signed on-chain assertion or provenance hash cannot replace these requirements.

## 4. Study package to prepare before the applications

1. **Scientific protocol:** MDD-only, exact reproduction of the JAMA cross-trial protocol, baseline and week-2 tasks separated, endpoint and scale mapping, participant-flow graph, explicit EMBARC stage overlap, missingness rules, all model comparators, split design and planned statistics.
2. **Research use statement:** precise, truthful description of the permitted prognosis, validation and—only if explicitly in scope—randomized treatment-effect/policy analyses.
3. **Data inventory:** exact release and package identifiers; permitted modalities and variables; source-specific consent groups; missing-data and derivation plan.
4. **Ethics materials:** full protocol with version number, participant/data protection plan, privacy/security assessment, and the approval or exemption letter. Confirm the named datasets and cross-border/cloud processing are explicitly covered.
5. **Data governance:** named PI and recipients, role-based access, MFA, retention/deletion, incident response, third-party and subcontractor access, export review and publication-suppression rules.
6. **Compute statement:** provider-hosted workspace versus approved institutional or cloud environment, software installation, storage, backup, model execution, data transfer and export pathways.
7. **Dissemination plan:** peer-reviewed publication, scientific communication, stakeholder and public summaries, disclosure review, dataset acknowledgements and custodian review.
8. **Reproducibility plan:** code commit, preprocessing and split digests, model identity, immutable protocol version, aggregate metrics and a disclosure-reviewed evidence receipt. Do not presume the provider will permit public release of any artifact.

## 5. Non-negotiable execution rules

- No download, copy, transfer or analysis until both institutional and provider authorities approve the exact dataset and purpose.
- Do not move participant rows, MRI/EEG, subject identifiers, individual embeddings, gradients or prediction outputs to personal devices, public GitHub, the Mycelix DHT or IPFS.
- Do not install unapproved dependencies, use an unapproved cloud compute service, add an unapproved collaborator or run a secondary analysis without the required authorization.
- Do not infer that a dataset is de-identified or safe to export merely because direct identifiers have been removed.
- No synthetic fixture, BIDS structural audit, CI run or signed receipt counts as ethics approval, DUA approval, source authorization or clinical validation.
- Maintain study-specific deletion, annual reporting, amendment and close-out obligations; delete approved copies and derivatives as required.

## 6. Access gate status

| Gate | Current state |
|---|---|
| MDD cross-trial research protocol | Draft in main Mycelix; not an institutional protocol |
| Named institutional PI / partner | Not secured |
| Brain-CODE institutional affiliation | Not confirmed |
| Brain-CODE REB approval / exemption | Not obtained |
| Brain-CODE DUA and approved workspace | Not obtained |
| NDA institution / eRA Commons / FWA eligibility | Not confirmed |
| NDA eDAR / dataset terms | Not submitted or approved |
| Controlled cohort access | Not authorized |
| Participant-level analysis | Not performed |

No gate should be inferred as complete from the existence of this checklist.

## 7. Next decision

Before writing an application or requesting any restricted dataset, identify a potential institutional research partner who can take PI responsibility and confirm its eligibility for both access pathways. Once that partnership exists, the protocol and data-use materials can be adapted to the institution's ethics and legal requirements.
