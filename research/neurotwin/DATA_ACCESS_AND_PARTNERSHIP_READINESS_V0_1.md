# NeuroTwin data-access and institutional partnership readiness v0.1

**Status:** access-path research and partnership plan; no access request has been submitted and no controlled cohort data have been obtained.  
**Canonical repository:** [Luminous-Dynamics/mycelix](https://github.com/Luminous-Dynamics/mycelix).  
**Tracking:** [NEUROTWIN-005 / #6284](https://github.com/Luminous-Dynamics/mycelix/issues/6284).

## 1. Why partnership is a scientific dependency

The most relevant treatment-response cohorts are controlled human-subject datasets. Permission to use them depends on the data provider, study consent, institutional ethics review, an eligible Principal Investigator, signed data-use agreements, named user approvals and an approved compute environment.

A public repository, LLC, DID, consent object, dataset hash or successful software CI run does not confer those rights. Mycelix's role is to make research execution and evidence reproducible **inside the provider-approved boundary**; it is not a substitute for the institution, data custodian or clinical investigator.

## 2. Candidate trial and dataset routes

### CAN-BIND-1 — Brain-CODE

The Brain-CODE access workflow is a promising primary route for the published cross-trial reproduction:

- Create an account and request the relevant Controlled Data Release through the Study Dashboard.
- Brain-CODE's current REB FAQ says external researchers must have an affiliation with an accredited academic institution, think tank, company or other research organization; local, national and international researchers may apply. This means an academic affiliation is not the only possible route, but the provider must accept the applicant organization and the named Study Investigator.
- The Study Investigator is asked for project title/description, anticipated brain-health impact, the full REB/IRB proposal and approval/exemption documentation, dissemination plan, CV, special considerations and a signed Data Use Agreement.
- Access is limited to the approved analysis and named users who have signed the required terms. The approved REB protocol must explicitly mention use of the requested Brain-CODE dataset; amendments may be required.
- Analyses use a Brain-CODE Workspace; multifactor authentication is required.
- The request is reviewed by the Brain-CODE Data Access Committee and Informatics Steering Committee. Current timelines and technical/workspace limits must be reconfirmed with the provider at the time of application.

The Brain-CODE workspace guidance recommends RStudio Web for clinical-only analysis and an approved Windows remote desktop where other tools are required; imaging or omics workflows require a separate hardware discussion. Workspace provisioning and dataset access are separate approvals.

**Consequence:** Luminous Dynamics may be able to apply as a company/research organization if Brain-CODE accepts its status and the project has the required local REB/IRB review. This is an eligibility question to confirm directly—not an assumed right of access. Do not plan to download raw CAN-BIND MRI/EEG into a developer laptop, public GitHub Actions runner, public DHT or IPFS. The source institution's current policy and approved workspace determine the permitted execution path.

### EMBARC — NIMH Data Archive

The NIMH Data Archive's current Electronic Data Access Request prerequisites are materially institution-bound:

- A Lead Recipient's NDA account must be affiliated with an NIH-recognized research institution and use an institutional email.
- The account must be linked to an eRA Commons account at the same institution with the Principal Investigator role.
- The institution must be registered in eRA Commons with an Institutional Signing Official and have an active Federalwide Assurance (FWA).
- NIH defines the PI role in its Data Use Certification guidance in terms of a permanent institutional employee with responsibility and proper authority to direct the project.
- A Data Access Request and Data Use Certification bind the approved users, requester and institution to the study's permitted use and protection of human-subject data.

**Consequence:** an independent software project or commercial LLC should not assume it can sponsor an NDA request in its own name. Work with an eligible university, hospital or research institution whose PI can sponsor the application and whose ethics/data-governance offices can approve the project. Do not invent or borrow an institutional affiliation.

### iSPOT-D — Stanford BRAINnet / Center for Precision Mental Health

iSPOT-D is a strong independent validation candidate because it randomized participants to three active antidepressants and collected longitudinal clinical, EEG and imaging data.

- Stanford's dataset catalogue describes 1,008 unmedicated clinical MDD participants and 336 matched healthy controls.
- Participants were randomized to escitalopram, sertraline or extended-release venlafaxine.
- MRI was acquired in approximately 20% of clinical participants and matched controls; the dataset catalogue lists EEG, structural MRI, task fMRI, diffusion MRI, clinical, cognitive and genetic measures.
- The 2020 JAMA Network Open secondary analysis used 518 participants (162 escitalopram, 176 sertraline and 180 venlafaxine) and five-fold cross-validation to predict improvement in individual HDRS-21 symptoms. Its metric was a symptom-specific C-index, not the response AUROC used in the 2025 EMBARC/CAN-BIND paper.
- Stanford's current BRAINnet page says data requests are temporarily limited because of request volume. It invites a short paper proposal by email and describes data access as non-commercial research or special projects for PMH Corporate Members.

**Consequence:** iSPOT-D is a high-value third-trial candidate, but access is not guaranteed. Before using it in a roadmap or grant commitment, obtain written confirmation of eligibility, available release, allowed modality/endpoint use, collaborator requirements and whether a commercial software company can participate directly or only through a qualifying academic collaboration. A Stanford collaboration or corporate membership should not be presumed or claimed until confirmed.

The three-arm design requires a multi-arm estimator. Do not coerce it into the current two-arm AIPW reference by dropping an arm or comparing only a post hoc subset.

### Secondary options — HCP-DES, RAD, ENGAGE and PReDICT

- **HCP-DES** includes 250 clinical participants and 50 controls, with multimodal imaging and repeated symptom measures. It is useful for brain-network representation and distribution-shift research, but it is not an antidepressant randomized treatment-response validation cohort.
- **RAD** includes 388 clinical participants with depression/anxiety spectrum measures and follow-up. It can support transdiagnostic or longitudinal transport questions, but its population and treatment design differ from the primary antidepressant trial.
- **ENGAGE** is a smaller randomized depression/obesity intervention study with behavioural problem-solving versus treatment as usual and longitudinal imaging/wearable data. It is a possible later intervention-domain benchmark, but not a direct antidepressant comparison.
- **PReDICT** randomized treatment-naïve adults to CBT, duloxetine or escitalopram and collected multimodal baseline/early-treatment measures. It is scientifically attractive, but a viable data-access/availability path has not yet been established in this programme.

These options must not be pooled together or used to inflate the main MDD antidepressant benchmark. Each requires a separate estimand, access and validation protocol.

## 3. Scientific role of iSPOT-D

iSPOT-D's 2020 symptom-level EEG model is an architecture/method comparator, not a direct leaderboard result:

- It used 518 people across three active treatment arms and baseline HRSD-21 plus EEG.
- Five-fold cross-validation produced C-index values of at least 0.80 on 12 of 21 clinician-rated symptoms, with particularly high scores for some individual symptoms.
- The target was change in individual symptoms from baseline to week 8, not a binary 50% response label and not a policy-value contrast.
- The authors themselves identified prospective validation as the next step.

The programme should reproduce the published symptom-specific task separately if approved. For the main cross-trial response task, harmonize endpoints and prediction time only where scientifically justified. For treatment-policy value, use all three randomized arms and the source randomization probabilities; compare a frozen candidate policy with a frozen clinical-only policy.

## 4. Minimum institutional partnership

The initial collaboration should include:

1. **Eligible Study Investigator / PI:** an accountable investigator affiliated with an organization accepted by the data provider. For Brain-CODE this may be an accredited academic institution, think tank, company or other research organization; for NIMH NDA it must satisfy the stricter institutional eRA Commons / PI / FWA requirements.
2. **Clinical MDD investigator:** confirms intended use, eligibility, outcomes, safety framing and clinically meaningful evaluation.
3. **Biostatistician / causal-inference lead:** prespecifies estimands, power, randomization analysis, missingness, policy-value evaluation and uncertainty.
4. **Data custodian / research-compliance and information-security contact:** ensures REB/IRB review, DUA, controlled workspace, retention, export, incident and disclosure terms are met.
5. **Patient / lived-experience partner:** shapes relevant outcomes, acceptable burdens, harms and communication.

Luminous Dynamics can contribute reproducible infrastructure, Rust reference tools, protocol and evidence receipts. It must not describe itself as the data custodian, clinical provider, PI institution or sponsor unless that status is formally established.

## 5. Execution plan

### Phase A — no-data preparation

- Freeze the JAMA reproduction checklist and the primary endpoint.
- Prepare a short, non-commercial research concept that states the intended public-interest question and the software contribution separately from any future commercial product.
- Prepare a data-flow and threat-model diagram showing that raw data and sensitive derivatives stay inside provider-approved systems.
- Prepare a model/analysis plan with clinical-only, week-2 clinical and published clinical + dACC baselines.
- Prepare draft output-disclosure rules for cohort counts, uncertainty, small-cell suppression and allowed exports.

### Phase B — provider eligibility confirmation

- Ask Brain-CODE for the current route, required Study Investigator/REB/DUA documents, permitted workspace and allowed output/export terms.
- Ask Stanford PMH whether iSPOT-D data sharing is currently accepting requests; confirm non-commercial eligibility and whether an institutional collaboration or Corporate Member special-project route is required.
- Identify an eligible institutional PI before beginning an NIMH NDA eDAR request; verify eRA Commons, institutional email, FWA and Signing Official requirements with the institution.

### Phase C — controlled cohort admission

- Obtain the institution's ethics approval or exemption and data-provider permission for the exact analysis, modalities, named users, derived data and publication outputs.
- Sign the relevant DUA/DUC and user agreements.
- Provision a provider-approved workspace and verify package/tool availability there.
- Run the official BIDS Validator at a version pinned in the study manifest, in addition to Mycelix's local aggregate intake audit.
- Only after these gates are met, execute the frozen source reproduction. The first output should be an aggregate reproduction receipt, not a new model leaderboard.

### Phase D — external and prospective qualification

- Evaluate an independent third trial without tuning on its outcomes.
- Use an appropriate multi-arm policy-value estimator for iSPOT-D or any other multi-arm randomized trial.
- If and only if independent validation supports the claim, plan a silent prospective evaluation with institutional and clinical oversight.
- Consider a randomized policy-impact study only after separate power, ethics, regulatory and clinical review.

## 6. Decision gates and no-go conditions

| Gate | Required evidence | No-go condition |
|---|---|---|
| Entity eligibility | Written provider confirmation of allowed applicant/partner status | Commercial or non-commercial use is unclear or prohibited |
| Institutional sponsorship | Named eligible PI and institution | No responsible PI / no accountable institution |
| Human-subject authority | REB/IRB approval or exemption and applicable participant-consent scope | No approval, uncertain consent scope or secondary use prohibition |
| Contractual authority | Signed DUA/DUC and named-user approval | Unsigned or mismatched data-use terms |
| Compute boundary | Approved controlled workspace and retention/deletion plan | Raw data must be moved to unapproved systems |
| Technical intake | Official BIDS validation plus aggregate intake evidence | Unknown release identity, structural mismatch or unsafe output rules |
| Scientific reproduction | Exact population, overlap, endpoint, timing and splits verified | Cohort mismatch or leakage not resolved |
| Clinical interpretation | Independent clinical review and prospective evidence | Retrospective model score used as treatment authority |

## 7. Public and private evidence separation

- Public repository: protocols, source code, synthetic fixtures, schema definitions, non-sensitive citations and reviewed aggregate results.
- Controlled institutional workspace: raw records, MRI/EEG, participant identifiers, linkage keys, individual embeddings/predictions, detailed split manifests where these may reveal participants, and all restricted intermediate data.
- Mycelix DHT/IPFS: no raw clinical data, participant IDs, per-person hashes, individual embeddings, gradients or small-cell tables.
- A public receipt may bind only reviewed aggregate outputs and non-identifying study/protocol references. Even aggregate cohort counts may require suppression under provider terms.

A public protocol or successful local test does not mean a dataset was accessed, approved or analyzed.

## 8. Sources and provider links

- Brain-CODE Data Access Request FAQ: https://indocconsortium.atlassian.net/wiki/spaces/JSDNXT/pages/2001895483/Data+Access+Request+Form+FAQ
- Brain-CODE Workspace request guidance: https://indocconsortium.atlassian.net/wiki/spaces/JSDNXT/pages/1994096694/Applying+for+a+Workspace
- NIMH NDA eDAR prerequisites: https://nda.nih.gov/nda/tutorials/electronic-data-access-request?chapter=prerequisites-1
- NIH Data Use Certification agreement: https://nda.nih.gov/ndapublicweb/Documents/NDA+Data+Access+Request+DUC+FINAL.pdf
- Stanford PMH / BRAINnet datasets and current access statement: https://www.stanfordpmhw.com/datasets
- iSPOT-D randomized trial symptom-level EEG analysis: https://doi.org/10.1001/jamanetworkopen.2020.6653
- iSPOT-D study protocol: https://doi.org/10.1186/1745-6215-12-4

This document reports public provider instructions and published study details. Availability, eligibility, legal terms and application requirements must be reconfirmed with each provider; no relationship or approval is claimed.
