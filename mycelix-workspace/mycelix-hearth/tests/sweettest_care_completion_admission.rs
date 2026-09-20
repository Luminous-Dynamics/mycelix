// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Real-conductor qualification for profile-relative Care completion admission.
//!
//! Completion evidence is intentionally weaker than lifecycle admission:
//! an assignee may author revision-scoped evidence without gaining authority to
//! admit lifecycle completion under CreatorOrCurrentGuardianV1.

use holochain::prelude::*;
use holochain::sweettest::*;
use std::path::PathBuf;

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
enum HearthType {
    Nuclear,
    Extended,
    Chosen,
    Blended,
    Multigenerational,
    Intentional,
    CoPod,
    Custom(String),
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
enum MemberRole {
    Founder,
    Elder,
    Adult,
    Youth,
    Child,
    Guest,
    Ancestor,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
enum CareType {
    Childcare,
    Eldercare,
    PetCare,
    Chore,
    MealPrep,
    Medical,
    Emotional,
    Custom(String),
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
enum Recurrence {
    Daily,
    Weekly,
    Monthly,
    Custom(String),
}

#[derive(Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
enum CareCompletionAdmissionProfileV1 {
    CreatorOrCurrentGuardianV1,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct CreateHearthInput {
    name: String,
    description: String,
    hearth_type: HearthType,
    max_members: Option<u32>,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct InviteMemberInput {
    hearth_hash: ActionHash,
    invitee_agent: AgentPubKey,
    proposed_role: MemberRole,
    message: String,
    expires_at: Timestamp,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct AcceptInvitationInput {
    invitation_hash: ActionHash,
    display_name: String,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct CreateCareScheduleInput {
    hearth_hash: ActionHash,
    care_type: CareType,
    title: String,
    description: String,
    assigned_to: AgentPubKey,
    recurrence: Recurrence,
    notes: String,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct CompleteTaskV2Input {
    schedule_hash: ActionHash,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct AdmitCompletionV1Input {
    schedule_root_hash: ActionHash,
    evidence_hashes: Vec<ActionHash>,
    profile: CareCompletionAdmissionProfileV1,
}

#[derive(Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
struct CareCompletionAdmissionV1 {
    schema_version: u8,
    profile: CareCompletionAdmissionProfileV1,
    hearth_hash: ActionHash,
    schedule_root_hash: ActionHash,
    evidence_hashes: Vec<ActionHash>,
    actor_membership_hash: ActionHash,
}

fn hearth_dna_path() -> PathBuf {
    let mut path = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    path.pop();
    path.push("dna");
    path.push("mycelix_hearth.dna");
    path
}

fn schedule_input(
    hearth_hash: ActionHash,
    assigned_to: AgentPubKey,
    title: &str,
) -> CreateCareScheduleInput {
    CreateCareScheduleInput {
        hearth_hash,
        care_type: CareType::Chore,
        title: title.to_string(),
        description: "Completion admission conductor qualification".to_string(),
        assigned_to,
        recurrence: Recurrence::Weekly,
        notes: String::new(),
    }
}

async fn invite_and_accept(
    alice_conductor: &SweetConductor,
    alice: &SweetCell,
    member_conductor: &SweetConductor,
    member: &SweetCell,
    hearth_hash: ActionHash,
    role: MemberRole,
    display_name: &str,
) -> Record {
    let invitation: Record = alice_conductor
        .call(
            &alice.zome("hearth_kinship"),
            "invite_member",
            InviteMemberInput {
                hearth_hash,
                invitee_agent: member.agent_pubkey().clone(),
                proposed_role: role,
                message: format!("Join admission qualification as {display_name}"),
                expires_at: Timestamp::from_micros(
                    Timestamp::now().as_micros() + 86_400_000_000,
                ),
            },
        )
        .await;

    await_consistency([alice, member]).await.unwrap();

    let membership: Record = member_conductor
        .call(
            &member.zome("hearth_kinship"),
            "accept_invitation",
            AcceptInvitationInput {
                invitation_hash: invitation.action_address().clone(),
                display_name: display_name.to_string(),
            },
        )
        .await;

    await_consistency([alice, member]).await.unwrap();
    membership
}

#[tokio::test(flavor = "multi_thread")]
#[ignore = "requires packed Hearth DNA and Holochain conductor"]
async fn completion_admission_is_profile_relative_and_separate_from_evidence() {
    let dna_file = SweetDnaFile::from_bundle(&hearth_dna_path()).await.unwrap();
    let mut alice_conductor = SweetConductor::from_standard_config().await;
    let mut bob_conductor = SweetConductor::from_standard_config().await;
    let mut charlie_conductor = SweetConductor::from_standard_config().await;

    let (alice,) = alice_conductor
        .setup_app("care-admission-alice", &[dna_file.clone()])
        .await
        .unwrap()
        .into_tuple();
    let (bob,) = bob_conductor
        .setup_app("care-admission-bob", &[dna_file.clone()])
        .await
        .unwrap()
        .into_tuple();
    let (charlie,) = charlie_conductor
        .setup_app("care-admission-charlie", &[dna_file])
        .await
        .unwrap()
        .into_tuple();

    SweetConductor::exchange_peer_info([
        &alice_conductor,
        &bob_conductor,
        &charlie_conductor,
    ])
    .await;

    let hearth: Record = alice_conductor
        .call(
            &alice.zome("hearth_kinship"),
            "create_hearth",
            CreateHearthInput {
                name: "Completion Admission Hearth".to_string(),
                description: "Evidence and lifecycle admission separation".to_string(),
                hearth_type: HearthType::Chosen,
                max_members: Some(12),
            },
        )
        .await;
    let hearth_hash = hearth.action_address().clone();

    let bob_membership = invite_and_accept(
        &alice_conductor,
        &alice,
        &bob_conductor,
        &bob,
        hearth_hash.clone(),
        MemberRole::Adult,
        "Bob",
    )
    .await;
    let charlie_membership = invite_and_accept(
        &alice_conductor,
        &alice,
        &charlie_conductor,
        &charlie,
        hearth_hash.clone(),
        MemberRole::Youth,
        "Charlie",
    )
    .await;

    await_consistency([&alice, &bob, &charlie]).await.unwrap();

    // Alice authors the stable root. Charlie is the historical assignee and may
    // author revision-scoped completion evidence while Active.
    let schedule: Record = alice_conductor
        .call(
            &alice.zome("hearth_care"),
            "create_care_schedule",
            schedule_input(
                hearth_hash.clone(),
                charlie.agent_pubkey().clone(),
                "Admission subject",
            ),
        )
        .await;
    let root_hash = schedule.action_address().clone();

    await_consistency([&alice, &bob, &charlie]).await.unwrap();

    let evidence_one: Record = charlie_conductor
        .call(
            &charlie.zome("hearth_care_transitions"),
            "complete_task_v2",
            CompleteTaskV2Input {
                schedule_hash: root_hash.clone(),
            },
        )
        .await;
    let evidence_two: Record = charlie_conductor
        .call(
            &charlie.zome("hearth_care_transitions"),
            "complete_task_v2",
            CompleteTaskV2Input {
                schedule_hash: root_hash.clone(),
            },
        )
        .await;

    let evidence_one_hash = evidence_one.action_address().clone();
    let evidence_two_hash = evidence_two.action_address().clone();

    // Evidence authority is not lifecycle-admission authority. Charlie is an
    // Active Youth and the evidence author/assignee, but is neither root creator
    // nor current guardian under CreatorOrCurrentGuardianV1.
    let charlie_admission = charlie_conductor
        .call_fallible::<_, Record>(
            &charlie.zome("hearth_care_transitions"),
            "admit_completion_v1",
            AdmitCompletionV1Input {
                schedule_root_hash: root_hash.clone(),
                evidence_hashes: vec![evidence_one_hash.clone()],
                profile: CareCompletionAdmissionProfileV1::CreatorOrCurrentGuardianV1,
            },
        )
        .await;
    assert!(
        charlie_admission.is_err(),
        "completion evidence authoring must not grant lifecycle-admission authority"
    );

    // Bob is not the root creator, but is a current Adult guardian. Submit the
    // same evidence deliberately in reverse raw-byte order; the coordinator may
    // canonicalize set ordering but integrity independently validates the result.
    let mut submitted_evidence = vec![evidence_one_hash.clone(), evidence_two_hash.clone()];
    submitted_evidence.sort_by(|left, right| right.get_raw_39().cmp(left.get_raw_39()));

    let bob_admission_record: Record = bob_conductor
        .call(
            &bob.zome("hearth_care_transitions"),
            "admit_completion_v1",
            AdmitCompletionV1Input {
                schedule_root_hash: root_hash.clone(),
                evidence_hashes: submitted_evidence,
                profile: CareCompletionAdmissionProfileV1::CreatorOrCurrentGuardianV1,
            },
        )
        .await;
    let bob_admission_hash = bob_admission_record.action_address().clone();
    let bob_admission = bob_admission_record
        .entry()
        .to_app_option::<CareCompletionAdmissionV1>()
        .expect("admission must decode")
        .expect("admission must contain entry");

    assert_eq!(bob_admission.schema_version, 1);
    assert_eq!(
        bob_admission.profile,
        CareCompletionAdmissionProfileV1::CreatorOrCurrentGuardianV1
    );
    assert_eq!(bob_admission.hearth_hash, hearth_hash);
    assert_eq!(bob_admission.schedule_root_hash, root_hash);
    assert_eq!(
        bob_admission.actor_membership_hash,
        bob_membership.action_address().clone()
    );
    assert_eq!(bob_admission.evidence_hashes.len(), 2);
    assert!(bob_admission
        .evidence_hashes
        .windows(2)
        .all(|pair| pair[0].get_raw_39() < pair[1].get_raw_39()));
    assert!(bob_admission.evidence_hashes.contains(&evidence_one_hash));
    assert!(bob_admission.evidence_hashes.contains(&evidence_two_hash));

    // Equivalent positive admission by the root creator remains separate
    // immutable provenance; later projection may coalesce lifecycle equivalence.
    let alice_admission: Record = alice_conductor
        .call(
            &alice.zome("hearth_care_transitions"),
            "admit_completion_v1",
            AdmitCompletionV1Input {
                schedule_root_hash: root_hash.clone(),
                evidence_hashes: vec![evidence_one_hash.clone(), evidence_two_hash.clone()],
                profile: CareCompletionAdmissionProfileV1::CreatorOrCurrentGuardianV1,
            },
        )
        .await;

    await_consistency([&alice, &bob, &charlie]).await.unwrap();

    let admissions: Vec<Record> = alice_conductor
        .call(
            &alice.zome("hearth_care_transitions"),
            "get_schedule_completion_admissions",
            root_hash.clone(),
        )
        .await;
    assert_eq!(admissions.len(), 2);
    assert!(admissions.iter().any(|record| record.action_address() == &bob_admission_hash));
    assert!(admissions
        .iter()
        .any(|record| record.action_address() == alice_admission.action_address()));
    assert!(admissions.windows(2).all(|pair| {
        pair[0]
            .action()
            .timestamp()
            .cmp(pair[1].action().timestamp())
            .then_with(|| pair[0].action_address().cmp(pair[1].action_address()))
            != std::cmp::Ordering::Greater
    }));

    // Duplicate references are not silently deduplicated by the admission
    // construction boundary.
    let duplicate_evidence = bob_conductor
        .call_fallible::<_, Record>(
            &bob.zome("hearth_care_transitions"),
            "admit_completion_v1",
            AdmitCompletionV1Input {
                schedule_root_hash: root_hash.clone(),
                evidence_hashes: vec![evidence_one_hash.clone(), evidence_one_hash.clone()],
                profile: CareCompletionAdmissionProfileV1::CreatorOrCurrentGuardianV1,
            },
        )
        .await;
    assert!(duplicate_evidence.is_err(), "duplicate evidence hashes must fail closed");

    // Evidence concerning another stable root cannot be admitted under this root.
    let other_schedule: Record = alice_conductor
        .call(
            &alice.zome("hearth_care"),
            "create_care_schedule",
            schedule_input(
                hearth_hash.clone(),
                charlie.agent_pubkey().clone(),
                "Other admission root",
            ),
        )
        .await;
    let other_evidence: Record = charlie_conductor
        .call(
            &charlie.zome("hearth_care_transitions"),
            "complete_task_v2",
            CompleteTaskV2Input {
                schedule_hash: other_schedule.action_address().clone(),
            },
        )
        .await;

    let cross_root_admission = bob_conductor
        .call_fallible::<_, Record>(
            &bob.zome("hearth_care_transitions"),
            "admit_completion_v1",
            AdmitCompletionV1Input {
                schedule_root_hash: root_hash.clone(),
                evidence_hashes: vec![other_evidence.action_address().clone()],
                profile: CareCompletionAdmissionProfileV1::CreatorOrCurrentGuardianV1,
            },
        )
        .await;
    assert!(
        cross_root_admission.is_err(),
        "evidence from another stable root must not be admitted under this root"
    );

    // Membership revocation applies at admission time. Bob's earlier valid
    // admission remains immutable provenance, but a new admission is denied.
    let _: Record = bob_conductor
        .call(
            &bob.zome("hearth_kinship"),
            "leave_hearth",
            bob_membership.action_address().clone(),
        )
        .await;
    await_consistency([&alice, &bob, &charlie]).await.unwrap();

    let departed_guardian_admission = bob_conductor
        .call_fallible::<_, Record>(
            &bob.zome("hearth_care_transitions"),
            "admit_completion_v1",
            AdmitCompletionV1Input {
                schedule_root_hash: root_hash,
                evidence_hashes: vec![evidence_one_hash],
                profile: CareCompletionAdmissionProfileV1::CreatorOrCurrentGuardianV1,
            },
        )
        .await;
    assert!(
        departed_guardian_admission.is_err(),
        "departed guardian must not create a new completion admission"
    );

    // Keep the Charlie membership value live in the test subject so the setup
    // proves his evidence was authored while a real canonical Active membership
    // existed, not through a fixture identity.
    assert_eq!(
        charlie_membership.action().author(),
        charlie.agent_pubkey()
    );
}
