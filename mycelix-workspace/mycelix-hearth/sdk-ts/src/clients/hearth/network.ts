// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/** Per-hearth clone-cell lifecycle for isolated household DHTs. */

import type {
  AppClient,
  CellId,
  ClonedCell,
  MembraneProof,
} from '@holochain/client';
import type { HearthCellTarget } from './cell-target';

export const HEARTH_TEMPLATE_ROLE = 'hearth';
export const HEARTH_NETWORK_SCHEMA = 1 as const;

export interface HearthNetworkPropertiesV1 {
  readonly schema_version: typeof HEARTH_NETWORK_SCHEMA;
  readonly topology: 'one-clone-per-hearth';
  readonly network_id: string;
}

export interface CreateHearthNetworkInput {
  /** Local display name for the clone. */
  readonly name: string;
  /** Stable opaque identifier shared by the invited household members. */
  readonly networkId?: string;
  /** Random network seed shared out-of-band with invited household members. */
  readonly networkSeed?: string;
  readonly membraneProof?: MembraneProof;
}

export interface HearthNetworkInviteV1 {
  readonly schema_version: typeof HEARTH_NETWORK_SCHEMA;
  readonly role_name: typeof HEARTH_TEMPLATE_ROLE;
  readonly name: string;
  readonly network_seed: string;
  readonly properties: HearthNetworkPropertiesV1;
}

export interface CreatedHearthNetwork {
  readonly target: HearthCellTarget;
  readonly invite: HearthNetworkInviteV1;
  readonly clonedCell: ClonedCell;
}

function randomToken(byteLength: number): string {
  const cryptoApi = globalThis.crypto;
  if (!cryptoApi?.getRandomValues) {
    throw new Error('Secure randomness is required to create a Hearth network');
  }
  const bytes = new Uint8Array(byteLength);
  cryptoApi.getRandomValues(bytes);
  let binary = '';
  for (const byte of bytes) binary += String.fromCharCode(byte);
  return btoa(binary).replaceAll('+', '-').replaceAll('/', '_').replace(/=+$/u, '');
}

function validateOpaqueId(value: string, label: string): void {
  if (!/^[A-Za-z0-9._:-]{16,160}$/u.test(value)) {
    throw new Error(`${label} must be a 16-160 character opaque identifier`);
  }
}

export class HearthNetworkManager {
  constructor(private readonly client: AppClient) {}

  async create(input: CreateHearthNetworkInput): Promise<CreatedHearthNetwork> {
    const networkId = input.networkId ?? `hearth_${randomToken(24)}`;
    const networkSeed = input.networkSeed ?? `hearth-v1-${randomToken(32)}`;
    validateOpaqueId(networkId, 'networkId');
    validateOpaqueId(networkSeed, 'networkSeed');

    const properties: HearthNetworkPropertiesV1 = {
      schema_version: HEARTH_NETWORK_SCHEMA,
      topology: 'one-clone-per-hearth',
      network_id: networkId,
    };

    const clonedCell = await this.client.createCloneCell({
      role_name: HEARTH_TEMPLATE_ROLE,
      modifiers: {
        network_seed: networkSeed,
        properties,
      },
      membrane_proof: input.membraneProof,
      name: input.name,
    });

    return {
      clonedCell,
      target: this.toTarget(clonedCell),
      invite: {
        schema_version: HEARTH_NETWORK_SCHEMA,
        role_name: HEARTH_TEMPLATE_ROLE,
        name: input.name,
        network_seed: networkSeed,
        properties,
      },
    };
  }

  /** Join an existing hearth by reproducing its exact DNA modifiers. */
  async join(invite: HearthNetworkInviteV1, membraneProof?: MembraneProof): Promise<CreatedHearthNetwork> {
    this.validateInvite(invite);
    const clonedCell = await this.client.createCloneCell({
      role_name: HEARTH_TEMPLATE_ROLE,
      modifiers: {
        network_seed: invite.network_seed,
        properties: invite.properties,
      },
      membrane_proof: membraneProof,
      name: invite.name,
    });
    return { clonedCell, target: this.toTarget(clonedCell), invite };
  }

  async enable(target: HearthCellTarget): Promise<HearthCellTarget> {
    const cloneId = this.requireCloneId(target);
    const cell = await this.client.enableCloneCell({
      clone_cell_id: { type: 'clone_id', value: cloneId },
    });
    return this.toTarget(cell);
  }

  async disable(target: HearthCellTarget): Promise<void> {
    const cloneId = this.requireCloneId(target);
    await this.client.disableCloneCell({
      clone_cell_id: { type: 'clone_id', value: cloneId },
    });
  }

  target(cellId: CellId, cloneId: string, name?: string): HearthCellTarget {
    if (!cloneId) throw new Error('A clone id is required for a Hearth cell target');
    return { cellId, cloneId, name };
  }

  private toTarget(cell: ClonedCell): HearthCellTarget {
    return { cellId: cell.cell_id, cloneId: cell.clone_id, name: cell.name };
  }

  private requireCloneId(target: HearthCellTarget): string {
    if (!target.cloneId) {
      throw new Error('This operation requires a cloned Hearth cell, not the provisioned template role');
    }
    return target.cloneId;
  }

  private validateInvite(invite: HearthNetworkInviteV1): void {
    if (invite.schema_version !== HEARTH_NETWORK_SCHEMA) {
      throw new Error(`Unsupported Hearth network invite schema: ${invite.schema_version}`);
    }
    if (invite.role_name !== HEARTH_TEMPLATE_ROLE) {
      throw new Error('Hearth network invite references an unexpected role');
    }
    if (invite.properties.topology !== 'one-clone-per-hearth') {
      throw new Error('Hearth network invite does not require an isolated clone');
    }
    validateOpaqueId(invite.properties.network_id, 'networkId');
    validateOpaqueId(invite.network_seed, 'networkSeed');
  }
}
