# Mycelix Compliance Matrix

> **Reliance warning — reviewed 2026-10-20:** This is a preliminary control-mapping document, not a compliance determination, certification, audit report, or assertion of production readiness. The [repository README](https://github.com/Luminous-Dynamics/mycelix/blob/main/README.md) describes Mycelix as a pre-alpha research project and says the repository has not been independently threat-modeled or externally audited. The previous percentage scores and “Production Ready” / “Audit Ready” labels were not substantiated in this document by a published scoring method or linked evidence register and are removed. Treat every implementation/control entry below as an unverified claim or candidate mapping until evidence is collected for an exact component version, configuration, and deployment.

> **Do not use this document as proof of GDPR, HIPAA, SOC 2, ISO 27001, NIST CSF, PCI DSS, or DORA compliance.** Applicability depends on jurisdiction, data, business process, contractual role, and deployment. Framework mapping does not create compliance. No certification or independent assurance report is claimed by this document.

## Framework mapping summary

| Framework | Current mapping status | External assurance |
|---|---|---|
| GDPR | Preliminary conceptual mapping; artifact-level evidence not inventoried | Not claimed |
| HIPAA | Scope-dependent; not a healthcare compliance determination | Not claimed |
| SOC 2 | Candidate Trust Services Criteria mapping; no auditor report linked here | Not claimed |
| ISO 27001 | Candidate control mapping; no certificate linked here | Not claimed |
| NIST Cybersecurity Framework | Candidate mapping; control implementation and evidence require verification | Not claimed |
| PCI DSS | Applicability must be determined from the exact cardholder-data environment; “not required” is not assumed | Not assessed here |
| EU DORA | Applicability and contractual role must be determined per institution and deployment | Not assessed here |

---

## GDPR control mapping (not a compliance determination)

### Article 5: Data Processing Principles

| Principle | Mycelix Implementation | Status |
|-----------|------------------------|--------|
| Lawfulness, fairness, transparency | Node consent on registration | Mapped — not independently verified |
| Purpose limitation | Gradients used only for FL | Mapped — not independently verified |
| Data minimization | No raw data transmitted | Mapped — not independently verified |
| Accuracy | Byzantine detection ensures quality | Mapped — not independently verified |
| Storage limitation | Configurable retention | Mapped — not independently verified |
| Integrity & confidentiality | TLS, encryption, signatures | Mapped — not independently verified |
| Accountability | Audit logging | Mapped — not independently verified |

### Article 17: Right to Erasure

```rust
// DHT tombstone implementation
pub async fn delete_participant_data(agent_id: &AgentPubKey) -> Result<()> {
    // Mark all entries as deleted
    let entries = get_entries_by_author(agent_id)?;
    for entry in entries {
        create_tombstone(entry.hash)?;
    }

    // Remove from active participants
    remove_from_training_round(agent_id)?;

    // Audit log
    emit_audit_event(AuditEventType::DataDeletion, agent_id)?;

    Ok(())
}
```

### Article 32: Security of Processing

| Measure | Implementation |
|---------|----------------|
| Encryption of personal data | AES-256-GCM at rest, TLS 1.3 in transit |
| Ongoing confidentiality | mTLS between nodes |
| Ongoing integrity | Ed25519 signatures on all gradients |
| Ongoing availability | Holochain DHT replication |
| Regular testing | Automated security tests in CI |

### Article 35: Data Protection Impact Assessment

| Risk | Likelihood | Impact | Mitigation | Residual Risk |
|------|------------|--------|------------|---------------|
| Gradient inversion | Medium | High | Differential privacy | Not validated |
| Model memorization | Not validated | High | Training limits | Not validated |
| Byzantine poisoning | High | High | Detection system | Not validated |
| Data exfiltration | Not validated | High | Encryption + auth | Not validated |

---

## HIPAA control mapping (scope-dependent; not a compliance determination)

### Administrative Safeguards (§164.308)

| Standard | Requirement | Implementation |
|----------|-------------|----------------|
| Security Management | Risk analysis | Annual assessment |
| Workforce Security | Authorization | RBAC + audit |
| Information Access | Access management | Role-based permissions |
| Security Awareness | Training | Onboarding program |
| Security Incident | Procedures | Incident response plan |
| Contingency Plan | Recovery | Backup + HA |
| Evaluation | Periodic review | Quarterly audits |

### Technical Safeguards (§164.312)

| Standard | Requirement | Implementation |
|----------|-------------|----------------|
| Access Control | Unique user ID | Node public keys |
| | Emergency access | Admin override (audited) |
| | Automatic logoff | Session timeout (15 min) |
| | Encryption/decryption | AES-256-GCM |
| Audit Controls | Logging | All access logged |
| Integrity | Mechanism to protect | Signatures + hashes |
| | Data authentication | Cryptographic verification |
| Transmission Security | Integrity controls | TLS 1.3 |
| | Encryption | Required for all traffic |

### Physical Safeguards (§164.310)

| Standard | Cloud Deployment | Self-Hosted |
|----------|------------------|-------------|
| Facility access | CSP responsibility | Customer responsibility |
| Workstation use | N/A for automated nodes | Security policies |
| Device controls | Encrypted volumes | Disk encryption required |

### Example HIPAA configuration (illustrative; not a verified safe baseline)

```toml
# config/hipaa.toml

[audit]
enabled = true
retention_days = 2190  # 6 years
log_all_access = true
log_phi_access = true

[encryption]
at_rest = "AES-256-GCM"
in_transit = "TLS-1.3"
key_management = "external"  # Use HSM or KMS

[access_control]
session_timeout_minutes = 15
max_failed_attempts = 3
lockout_duration_minutes = 30
require_mfa = true

[phi_protection]
# Never transmit or log PHI
raw_data_allowed = false
gradient_sanitization = true
differential_privacy_epsilon = 0.5  # Stricter for healthcare
```

---

## SOC 2 Trust Services Criteria mapping (not independently audited)

### Trust Services Criteria

#### CC1: Control Environment

| Control | Evidence |
|---------|----------|
| CC1.1 COSO commitment | Security policy document |
| CC1.2 Board oversight | Quarterly security reviews |
| CC1.3 Authority structure | RACI matrix for security |
| CC1.4 Competence commitment | Training requirements |
| CC1.5 Accountability | Performance metrics |

#### CC2: Communication and Information

| Control | Evidence |
|---------|----------|
| CC2.1 Information needs | Security documentation |
| CC2.2 Internal communication | Incident channels |
| CC2.3 External communication | Security advisories |

#### CC3: Risk Assessment

| Control | Evidence |
|---------|----------|
| CC3.1 Risk objectives | Threat model document |
| CC3.2 Risk identification | Quarterly risk reviews |
| CC3.3 Fraud risk | Byzantine detection |
| CC3.4 Change impact | Change management process |

#### CC4: Monitoring Activities

| Control | Evidence |
|---------|----------|
| CC4.1 Monitoring controls | Prometheus + Grafana |
| CC4.2 Deficiency evaluation | Alert review process |

#### CC5: Control Activities

| Control | Evidence |
|---------|----------|
| CC5.1 Control selection | Security architecture |
| CC5.2 Technology controls | Automated security |
| CC5.3 Policies & procedures | Security runbooks |

#### CC6: Logical and Physical Access

| Control | Evidence |
|---------|----------|
| CC6.1 Logical access security | RBAC + audit logs |
| CC6.2 Access provisioning | Onboarding/offboarding |
| CC6.3 Access removal | Automated deprovisioning |
| CC6.4 Access restriction | Network segmentation |
| CC6.5 System boundaries | Architecture diagrams |
| CC6.6 External threats | Byzantine detection, WAF |
| CC6.7 Access to data | Encryption + auth |
| CC6.8 Malicious software | Container scanning |

#### CC7: System Operations

| Control | Evidence |
|---------|----------|
| CC7.1 Vulnerability detection | Automated scanning |
| CC7.2 Anomaly monitoring | Byzantine detection |
| CC7.3 Change evaluation | Change management |
| CC7.4 Incident response | IR procedures |
| CC7.5 Recovery from incidents | Backup/restore tests |

#### CC8: Change Management

| Control | Evidence |
|---------|----------|
| CC8.1 Infrastructure changes | CI/CD pipeline |

#### CC9: Risk Mitigation

| Control | Evidence |
|---------|----------|
| CC9.1 Business continuity | DR plan |
| CC9.2 Vendor risk | Third-party assessment |

---

## NIST Cybersecurity Framework mapping (not independently verified)

### Identify (ID)

| Category | Implementation |
|----------|----------------|
| ID.AM Asset Management | Infrastructure as Code |
| ID.BE Business Environment | Architecture documentation |
| ID.GV Governance | Security policies |
| ID.RA Risk Assessment | Threat model, risk register |
| ID.RM Risk Management | Quarterly reviews |
| ID.SC Supply Chain | Dependency scanning |

### Protect (PR)

| Category | Implementation |
|----------|----------------|
| PR.AC Access Control | RBAC, mTLS, node auth |
| PR.AT Awareness Training | Security training program |
| PR.DS Data Security | Encryption, DP, signatures |
| PR.IP Information Protection | Data classification |
| PR.MA Maintenance | Patch management |
| PR.PT Protective Technology | Byzantine detection |

### Detect (DE)

| Category | Implementation |
|----------|----------------|
| DE.AE Anomalies Events | Prometheus alerting |
| DE.CM Security Monitoring | 24/7 monitoring |
| DE.DP Detection Processes | Automated detection |

### Respond (RS)

| Category | Implementation |
|----------|----------------|
| RS.RP Response Planning | Incident response plan |
| RS.CO Communications | Notification procedures |
| RS.AN Analysis | Forensic capabilities |
| RS.MI Mitigation | Containment procedures |
| RS.IM Improvements | Post-incident reviews |

### Recover (RC)

| Category | Implementation |
|----------|----------------|
| RC.RP Recovery Planning | Business continuity plan |
| RC.IM Improvements | Lessons learned |
| RC.CO Communications | Status updates |

---

## ISO 27001 Mapping

### Annex A Controls

| Control | Status | Notes |
|---------|--------|-------|
| A.5 Information security policies | Mapped — not independently verified | Documented |
| A.6 Organization of information security | Mapped — not independently verified | RACI defined |
| A.7 Human resource security | Gap noted — evidence/retest required | Requires HR integration |
| A.8 Asset management | Mapped — not independently verified | IaC inventory |
| A.9 Access control | Mapped — not independently verified | RBAC + audit |
| A.10 Cryptography | Mapped — not independently verified | Strong crypto throughout |
| A.11 Physical security | Gap noted — evidence/retest required | CSP dependent |
| A.12 Operations security | Mapped — not independently verified | Automated ops |
| A.13 Communications security | Mapped — not independently verified | TLS + mTLS |
| A.14 System acquisition | Mapped — not independently verified | Secure SDLC |
| A.15 Supplier relationships | Gap noted — evidence/retest required | Vendor assessment needed |
| A.16 Incident management | Mapped — not independently verified | IR procedures |
| A.17 Business continuity | Mapped — not independently verified | HA + DR |
| A.18 Compliance | Mapped — not independently verified | This matrix |

---

## Evidence collection — verification required

The repository has not established here that the commands and artifacts below are implemented in a releasable build or have been independently checked. Treat this section as a proposed evidence checklist until each item is demonstrated against a pinned version and deployment. Do not present examples as runnable CLI commands without verification.

### Candidate automated evidence (verify on pinned build)

```bash
# Illustrative command shape only — verify command availability/output against a pinned build.
mycelix compliance-report --framework gdpr > gdpr-evidence.json
mycelix compliance-report --framework hipaa > hipaa-evidence.json
mycelix compliance-report --framework soc2 > soc2-evidence.json

# Desired report should include once implemented and verified:
# - Access logs (90 days)
# - Configuration snapshots
# - Security scan results
# - Incident records
# - Change history
```

### Requested manual evidence (not evidence of current completion)

| Evidence Type | Collection Frequency | Owner |
|---------------|---------------------|-------|
| Risk assessments | Quarterly | Security Team |
| Penetration tests | Annual | External Vendor |
| Policy reviews | Annual | Compliance |
| Training records | Per employee | HR |
| Vendor assessments | Annual per vendor | Procurement |

---

## External assurance status

| Assurance item | Status as represented by this repository document | Evidence needed before making a claim |
|---|---|---|
| SOC 2 Type I / Type II | Not claimed; no independent auditor report is linked here | Defined system and service scope, control evidence, auditor report and applicable period |
| ISO 27001 | Not claimed; no certificate is linked here | Defined ISMS scope and current certificate from an accredited certification body |
| HIPAA | No attestation or blanket compliance claim | Determine applicability; document the covered service, required agreements, risk analysis, safeguards, and evidence with qualified counsel/compliance owners |
| Regulatory approval | Not claimed | Written evidence from the relevant authority, where an approval regime applies |

---

## Contact and accountability

Use the maintainer/contact route published in the repository README for review of this draft. This matrix does not establish that a dedicated compliance team or Data Protection Officer has been appointed. Before institutional diligence, identify the responsible legal entity, accountable control owners, and any statutory appointments and service-specific contact channels that apply.

---

*Last revised: 2026-10-20; no external assurance is claimed by this document.*
*Document version: 1.0.0*
