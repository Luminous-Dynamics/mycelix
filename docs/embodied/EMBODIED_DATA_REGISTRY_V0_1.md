# Embodied Data Registry v0.1

Status: source profile only. Parent program: EMB-DATA-000 / #2923. Preregistration: EMB-DATA-001T / #2930.

## Purpose

This registry freezes reviewed public source observations for the first external embodied-data families considered by Mycelix/Symthaea/Symtropy. It is an evidence/indexing artifact only.

```text
registry entry exists
!= dataset admitted
!= download authorized
!= training authorized
!= redistribution authorized
!= commercial use authorized
!= model quality established
!= robot action authorized
```

Rights observations are non-legal-advice summaries and always defer to the exact provider terms and a fresh intended-use review through EMB-DATA-RIGHTS-001/#2925.

## Registry identity

Profile: `mycelix:emb-data-registry:v0.1`

Review date: `2026-09-22`

Every entry preserves provider/resource family, reviewed source locator, resource kind, major modalities, observed rights/access state, split/benchmark concerns, first intended research role, owning issue and explicit nonclaims.

## Closed rights-observation vocabulary

- `RequiresExactTermsReview`
- `NonCommercialRestrictionObserved`
- `AttributionLicenseObserved`
- `DatasetSpecificAgreementObserved`
- `PerComponentTermsObserved`
- `SourceDatasetTermsInherited`
- `UserGeneratedCapture`
- `SyntheticFirstParty`
- `Unknown`

These states describe reviewed source material. None is a permission token.

## Entries

### Nymeria / NymeriaPlus

Provider family: Meta / Project Aria ecosystem.

Reviewed source: official `facebookresearch/nymeria_dataset` repository and dataset license files.

Resource kind: egocentric multimodal human-motion dataset.

Important observation: current NymeriaPlus terms use split licensing: audio is under a Meta non-commercial research license while the remaining non-audio data/code is described under CC BY-NC 4.0. The original Nymeria and NymeriaPlus have dataset-specific terms and must remain separately reviewed.

Rights observations: `NonCommercialRestrictionObserved`, `RequiresExactTermsReview`.

Owner: EMB-DATA-META-001/#2926.

### HOT3D

Provider family: Meta / Project Aria ecosystem.

Reviewed source: official HOT3D toolkit / Project Aria documentation.

Resource kind: egocentric hand-object interaction dataset and object assets.

Important observation: provider instructions distinguish licenses applicable to sequence data, hand annotations and 3D object models. MANO assets have their own external license dependency.

Rights observations: `PerComponentTermsObserved`, `RequiresExactTermsReview`.

Owner: EMB-DATA-META-001/#2926.

### Ego4D

Provider family: Ego4D consortium / Meta partnership.

Reviewed source: official Ego4D access documentation.

Resource kind: large-scale egocentric video/annotation benchmark dataset.

Important observation: access requires review/acceptance of a dataset license agreement; the access process and benchmark split are provider-defined. Current challenge documentation continues to use Ego4D v2.0.

Rights observations: `DatasetSpecificAgreementObserved`, `RequiresExactTermsReview`.

Owner: EMB-DATA-META-001/#2926.

### Ego-Exo4D

Provider family: Ego4D consortium / Meta Project Aria partnership.

Reviewed source: official Ego-Exo4D/Ego4D documentation and license materials.

Resource kind: synchronized ego/exocentric skilled-activity dataset with multimodal annotations.

Rights observations: `DatasetSpecificAgreementObserved`, `RequiresExactTermsReview`.

Owner: EMB-DATA-META-001/#2926.

### Open X-Embodiment

Provider family: Google DeepMind + contributing robotics labs.

Reviewed source: Google DeepMind Open X-Embodiment announcement/documentation.

Resource kind: aggregated multi-embodiment robot demonstration data.

Important observation: aggregation does not erase original contributing dataset identity, action semantics, split roles or source-specific rights.

Rights observations: `SourceDatasetTermsInherited`, `RequiresExactTermsReview`.

Owner: EMB-DATA-GOOGLE-001/#2927.

### Google Scanned Objects

Provider family: Google Research.

Reviewed source: Google Research Scanned Objects release description.

Resource kind: 3D scanned household object assets for simulation/robotics.

Important observation: the release states 1,030 scanned objects, about 13 GB, licensed under CC BY 4.0. Attribution obligations must survive conversion and derived simulation packaging.

Rights observations: `AttributionLicenseObserved`, `RequiresExactTermsReview`.

Owner: EMB-DATA-GOOGLE-001/#2927.

### LeRobotDataset v3

Provider family: Hugging Face LeRobot.

Reviewed source: official LeRobotDataset v3 documentation.

Resource kind: multimodal robotics dataset interchange/storage format.

Important observation: v3 uses tabular time-series, video shards and relational metadata; it supports streaming. Format compatibility does not establish a common license for datasets hosted in the format.

Rights observations: `SourceDatasetTermsInherited`, `RequiresExactTermsReview`.

Owner: EMB-DATA-LEROBOT-001/#2928.

### ROS 2 rosbag2 captures

Provider family: ROS 2 runtime/user capture.

Reviewed source: current rosbag2 documentation/profile selected by the recording system.

Resource kind: robot middleware capture/replay container.

Important observation: a bag is user-generated capture evidence. Dataset/use/privacy rights depend on the originating sensors, operators, people, environments and upstream assets; replay does not upgrade evidence class.

Rights observations: `UserGeneratedCapture`, `RequiresExactTermsReview`.

Owner: EMB-DATA-ROS-001/#2929.

### Symtropy synthetic episodes

Provider family: Luminous Dynamics / Symtropy.

Resource kind: simulation-generated embodied episode.

Important observation: synthetic outputs inherit every applicable source-asset/demonstration restriction. Synthetic generation cannot wash source rights or train/test restrictions.

Rights observations: `SyntheticFirstParty`, `SourceDatasetTermsInherited`.

Runtime owner: Symtropy EMB-SYNTH-001/#1428.

## Global invariants

1. rights evidence is not legal permission;
2. exact provider terms outrank this registry summary;
3. current review is required for a new intended use;
4. restricted source terms survive conversion, mixing and synthetic derivation;
5. train/validation/test/eval roles survive conversion;
6. missing modality is unknown, not zero/default;
7. provider locator is not semantic identity;
8. human motion is not humanoid actuator command;
9. source robot action is not target robot action;
10. synthetic evidence is not physical evidence;
11. dataset admission is not skill acquisition;
12. no registry state grants robot execution authority.

## Claim ceiling

A future qualification of this exact profile may establish only deterministic representation of these reviewed source observations as of 2026-09-22. It does not grant access/use rights, prove privacy or consent compliance, qualify any parser, authorize training, establish model quality, or establish humanoid capability/safety.