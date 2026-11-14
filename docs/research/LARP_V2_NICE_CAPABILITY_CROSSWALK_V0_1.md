# LARP v2 — NICE Workforce Capability Crosswalk v0.1

**Status:** research integration profile; no NIST endorsement, workforce certification or authorization integration claimed  
**Framework reference:** NIST NICE Framework Components v2.2.0, released 28 April 2026.

## 1. Purpose

NICE provides a common vocabulary for cybersecurity work and individual capabilities through work-role categories, work roles, competency areas, and Task, Knowledge and Skill (TKS) statements.

LARP can use NICE references to describe the capabilities needed to perform and independently oversee institutional duties. The crosswalk is evidence about capability, not authority.

Current component reference: https://www.nist.gov/news-events/news/2026/04/nice-releases-nice-framework-components-v220  
Current versions: https://www.nist.gov/itl/applied-cybersecurity/nice/nice-framework-resource-center/nice-framework-current-versions

## 2. Separation of capability and authority

Keep four distinct records:

1. **Duty:** what the institution needs done, and why.
2. **Capability evidence:** what a person or team can demonstrably perform.
3. **Appointment:** the locally authorized act assigning a bounded responsibility.
4. **Authority:** the exact constitutional/policy grant permitting defined decisions or effects.

A NICE work role, competency, TKS statement, course, certificate or skill assessment does not itself appoint someone or grant authority. A qualified person may have no authority; an authorized person may lack capability and require supervision or reassignment.

## 3. Suggested institutional duty families

These are LARP duty labels, not new NIST NICE work roles. Map each duty to the current NICE component identifiers only after reviewing the actual component data.

| Local duty family | Example responsibilities | Separation requirement |
|---|---|---|
| Authority operations | Maintain local authority/credential records and revocation | Cannot unilaterally approve own role or alter constitution |
| Independent assurance | Review control, evidence and execution claims | Must not be the sole owner of the process being reviewed |
| Incident response | Triage, contain and coordinate security incidents | Emergency capabilities are time-limited and reviewable |
| Recovery stewardship | Restore essential services and reconcile state | Service restoration cannot restore stale privileges |
| Privacy and rights review | Review purpose, disclosure, consent and redress | Must have a route independent of the data controller |
| Evidence custody | Preserve provenance, integrity and chain of custody | Custodian cannot certify the truth of its own evidence |
| Financial control | Review bounded commitments and treasury operations | Proposal, approval, execution and reconciliation are separated |

## 4. Crosswalk record

Each local duty mapping should include:
- stable local duty identifier and version;
- institution and jurisdiction;
- applicable contract(s);
- risk/criticality and required coverage;
- NICE Framework component version;
- NICE Work Role / Competency Area / Task / Knowledge / Skill identifiers, where actually applicable;
- mapping rationale and exclusions;
- required proficiency and evidence method;
- assessor identity, qualification and conflict status;
- last demonstrated date and review/expiry policy;
- supervision and dual-control requirements;
- succession coverage and alternate personnel;
- local appointment/authority reference, stored separately;
- challenge/correction path for the capability record.

Do not invent NICE identifiers or treat a loose semantic match as an official NIST crosswalk. Keep local duty taxonomy separate from the upstream framework and pin the exact upstream version.

## 5. Succession and resilience measures

The main institutional value is not employee ranking. It is whether essential duties can be performed safely when people are unavailable.

Report:
- critical duties with zero qualified coverage;
- critical duties dependent on one individual;
- independent reviewer coverage;
- separation-of-duty conflicts;
- time to qualify a successor;
- training and reassessment burden;
- revoked or stale capability evidence;
- time to staff essential degraded-mode operation;
- recovery quorum diversity;
- proportion of authority holders with a separately qualified reviewer.

Targets are locally risk-based. A universal workforce coverage percentage can be misleading where duties differ substantially in criticality.

## 6. Rights, privacy and anti-coercion constraints

- Keep capability evidence purpose-limited to the declared duty.
- Do not create a cross-institution social-credit or reputation score.
- Do not infer moral worth, honesty or legitimacy from skill proficiency.
- Do not make basic rights or public services depend on NICE mapping.
- Allow individuals to inspect and challenge inaccurate capability evidence.
- Do not expose sensitive assessment details beyond the legitimate need to know.
- Support alternative evidence routes and accessibility accommodations.
- Record conflicts of interest and assessor independence.
- Do not let the person who controls appointment authority unilaterally define or falsify the capability standard for an oversight role.

## 7. Benchmark plan

Run tabletop and synthetic incident exercises:
- founder/administrator unavailable;
- one recovery operator compromised;
- sole reviewer conflicted;
- two qualified people share a dependency;
- capability evidence expired during an incident;
- critical role has no successor;
- delegated task exceeds an individual's demonstrated scope;
- emergency service is restored while high-impact authority stays paused.

Measure duty coverage, unsafe delegation, separation-of-duty violation, recovery time, successor qualification time and residual single-person dependencies.

## 8. Integration boundary

NICE references can be attached to duty and capability evidence in LARP. LISE independently evaluates whether the local appointment and authority grant are valid, current, properly scoped and free from capture. Mycelix owner systems remain responsible for credential issuance, identity, appointment, revocation and recovery.

## 9. Validation status

This is a proposed crosswalk method only. It does not contain an official NIST mapping, certified competency assessment, workforce data, NIST review or production appointment/authorization flow.
