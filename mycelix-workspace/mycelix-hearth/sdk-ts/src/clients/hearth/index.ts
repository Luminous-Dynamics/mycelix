// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
export * from './types';
export { HearthError, classifyError } from './errors';
export type { HearthErrorCode } from './errors';
export { KinshipClient } from './kinship';
export type { KinshipSignalHandler } from './kinship';
export { DecisionsClient } from './decisions';
export type { DecisionSignalHandler } from './decisions';
export { GratitudeClient } from './gratitude';
export type { GratitudeSignalHandler } from './gratitude';
export { StoriesClient } from './stories';
export type { StorySignalHandler } from './stories';
export { CareClient } from './care';
export type { CareSignalHandler } from './care';
export { AutonomyClient } from './autonomy';
export { EmergencyClient } from './emergency';
export type { EmergencySignalHandler } from './emergency';
export { ResourcesClient } from './resources';
export type { ResourceSignalHandler } from './resources';
export { MilestonesClient } from './milestones';
export type { MilestoneSignalHandler } from './milestones';
export { RhythmsClient } from './rhythms';
export type { RhythmsSignalHandler } from './rhythms';
export { BridgeClient } from './bridge';
export type { BridgeSignalHandler } from './bridge';
export { ConsciousnessGateError, withGateRetry } from './consciousness-gate';
export type { ConsciousnessGateRejection } from './consciousness-gate';
export { HearthClient } from './hearth-client';

export { callHearthZome, equalBytes, equalCellId, isSignalForCell } from './cell-target';
export type { HearthCellTarget } from './cell-target';
export { HearthNetworkManager, HEARTH_NETWORK_SCHEMA, HEARTH_TEMPLATE_ROLE } from './network';
export type { HearthNetworkPropertiesV1, HearthNetworkInviteV1, CreateHearthNetworkInput, CreatedHearthNetwork } from './network';
