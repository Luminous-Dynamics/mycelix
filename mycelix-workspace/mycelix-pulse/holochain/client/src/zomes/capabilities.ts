// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

import type { ActionHash, AgentPubKey, AppClient } from '@holochain/client';
import type {
  AuditAction,
  CapabilityGrantDeliveryOutcome,
  CapabilityProbeResult,
  GrantCapabilityInput,
  MailboxCapability,
} from '../types';

/**
 * Thin typed adapter for the capability zome's real 0.7 API.
 *
 * Important: the raw CapSecret stays inside Holochain. The grant call returns
 * only its application action hash; delivery is a separate acknowledged remote
 * call which creates a private CapClaim on the grantee's source chain.
 */
export class CapabilitiesZomeClient {
  constructor(
    private readonly client: AppClient,
    private readonly roleName = 'mycelix_mail',
    private readonly zomeName = 'mail_capabilities',
  ) {}

  private async callZome<T>(fnName: string, payload: unknown = null): Promise<T> {
    const result = await this.client.callZome({
      role_name: this.roleName,
      zome_name: this.zomeName,
      fn_name: fnName,
      payload,
    });
    return result as T;
  }

  /** Create the grant. Follow with deliverCapabilityGrant using the returned hash. */
  async grantCapability(input: GrantCapabilityInput): Promise<ActionHash> {
    if (input.expires_at != null) {
      throw new Error('Timed capability expiry is not currently qualified.');
    }

    return this.callZome<ActionHash>('grant_capability', {
      grantee: input.grantee,
      access_type: input.access_type,
      permissions: input.permissions,
      restrictions: input.restrictions ?? null,
      expires_at: null,
    });
  }

  /** Request private CapClaim creation on the grantee; returns only after acknowledgement. */
  async deliverCapabilityGrant(capabilityHash: ActionHash): Promise<void> {
    await this.callZome<void>('deliver_capability_grant', capabilityHash);
  }

  /**
   * Convenience wrapper which preserves the grant hash if delivery fails, so callers
   * can retry delivery instead of orphaning a live grant or discarding its identifier.
   */
  async grantCapabilityAndDeliver(
    input: GrantCapabilityInput,
  ): Promise<CapabilityGrantDeliveryOutcome> {
    const capabilityHash = await this.grantCapability(input);
    try {
      await this.deliverCapabilityGrant(capabilityHash);
      return {
        capability_hash: capabilityHash,
        delivery_acknowledged: true,
      };
    } catch {
      return {
        capability_hash: capabilityHash,
        delivery_acknowledged: false,
        // Remote/Holochain errors can echo serialized arguments. Do not surface raw
        // error text at a secret-handling boundary.
        delivery_error: 'Delivery was not acknowledged; retry deliverCapabilityGrant with this capability hash.',
      };
    }
  }

  /** Revoke conductor authorization; the app-entry flag alone is not the revocation. */
  async revokeCapability(
    capabilityHash: ActionHash,
    reason?: string,
  ): Promise<ActionHash> {
    return this.callZome<ActionHash>('revoke_capability', [
      capabilityHash,
      reason ?? null,
    ]);
  }

  /**
   * Reads the best currently visible application projection. This is advisory,
   * not proof of globally current DHT state or remote authorization. The zome
   * call may reject with an explicit unknown-state error while update evidence
   * is incomplete; callers must not coerce that error to false or true.
   */
  async verifyCapability(
    capabilityHash: ActionHash,
    action: AuditAction,
  ): Promise<boolean> {
    return this.callZome<boolean>('verify_capability', [capabilityHash, action]);
  }

  async getGrantedCapabilities(): Promise<Array<[ActionHash, MailboxCapability]>> {
    return this.callZome<Array<[ActionHash, MailboxCapability]>>('get_granted_capabilities');
  }

  async getReceivedCapabilities(): Promise<Array<[ActionHash, MailboxCapability]>> {
    return this.callZome<Array<[ActionHash, MailboxCapability]>>('get_received_capabilities');
  }

  /**
   * Probe performs a real remote call to the empty-response capability endpoint
   * using the recipient's private CapClaim; no inbox data is transferred.
   * FunctionNotGranted means the requested read probe is outside the capability's
   * scope. Unauthorized means the call was denied; by itself it does not prove
   * revocation rather than a secret/grant mismatch.
   */
  async probeRemoteCapability(capabilityHash: ActionHash): Promise<CapabilityProbeResult> {
    return this.callZome<CapabilityProbeResult>('probe_remote_capability', capabilityHash);
  }
}
