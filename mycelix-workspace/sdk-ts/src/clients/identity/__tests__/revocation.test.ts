import { describe, it, expect, vi } from 'vitest';
import { RevocationClient } from '../revocation';
import type { AppClient } from '@holochain/client';
import type { RevocationCheckResult } from '../types';

function createMockClient(): AppClient {
  return { callZome: vi.fn() } as unknown as AppClient;
}

describe('RevocationClient as-of result contract', () => {
  it('preserves effective, recorded, suspension and lookup metadata', async () => {
    const client = createMockClient();
    const revocation = new RevocationClient(client);
    const result: RevocationCheckResult = {
      credential_id: 'cred:revoked',
      status: 'Revoked',
      lookup_status: 'RecordObserved',
      reason: 'Key compromise',
      effective_from: 1_700_000_000_000_000,
      recorded_at: 1_700_000_001_000_000,
      suspension_end: null,
      checked_at: 1_700_000_002_000_000,
    };
    (client.callZome as ReturnType<typeof vi.fn>).mockResolvedValueOnce(result);

    const observed = await revocation.checkRevocationStatus('cred:revoked');

    expect(client.callZome).toHaveBeenCalledWith({
      role_name: 'identity',
      zome_name: 'revocation',
      fn_name: 'check_revocation_status',
      payload: 'cred:revoked',
    });
    expect(observed).toEqual(result);
    expect(observed.effective_from).toBe(result.effective_from);
    expect(observed.recorded_at).toBe(result.recorded_at);
    expect(observed.checked_at).toBeGreaterThan(observed.recorded_at!);
  });

  it('preserves no-record-observed as distinct from a record-backed Active result', async () => {
    const client = createMockClient();
    const revocation = new RevocationClient(client);
    const result: RevocationCheckResult = {
      credential_id: 'cred:not-observed',
      status: 'Active',
      lookup_status: 'NoRecordObserved',
      reason: null,
      effective_from: null,
      recorded_at: null,
      suspension_end: null,
      checked_at: 1_700_000_002_000_000,
    };
    (client.callZome as ReturnType<typeof vi.fn>).mockResolvedValueOnce(result);

    const observed = await revocation.checkRevocationStatus('cred:not-observed');

    expect(observed.status).toBe('Active');
    expect(observed.lookup_status).toBe('NoRecordObserved');
    expect(observed.effective_from).toBeNull();
    expect(observed.recorded_at).toBeNull();
  });
});
