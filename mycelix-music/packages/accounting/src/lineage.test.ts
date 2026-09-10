import { describe, expect, it } from 'vitest';
import { buildMoneyLineage, lineageObligationAuthorities, lineageRefs } from './lineage.js';
import { money } from './money.js';
import { createRoyaltyObligationAuthority } from './obligation-authority.js';

function obligation(id: string, usageEvidenceRef = `usage:${id}`) {
  return createRoyaltyObligationAuthority({
    id,
    beneficiaryId: 'creator:alice',
    amount: money(109n),
    observedAt: '2026-09-10T00:00:00Z',
    provenance: {
      usageEvidenceRef,
      rightsResolutionRef: `rights:${id}`,
      economicTermsRef: 'terms:t7',
    },
  });
}

const obligations = [obligation('obligation:o3'), obligation('obligation:o1'), obligation('obligation:o2')];
const base = {
  beneficiaryId: 'creator:alice',
  received: money(327n),
  paymentRef: 'payment:p91',
  settlementLineRef: 'settlement-line:sl88',
  nettingBatchRef: 'netting:n24',
  obligations,
  rightsAgreementRef: 'agreement:a29',
  repertoireSnapshotRef: 'repertoire:r11',
} as const;

describe('creator money lineage', () => {
  it('preserves the payment-to-immutable-debt-to-source-evidence chain', () => {
    const lineage = buildMoneyLineage(base);
    expect(lineageRefs(lineage, 'payment')).toEqual(['payment:p91']);
    expect(lineageRefs(lineage, 'royalty_obligation')).toEqual([
      'obligation:o1',
      'obligation:o2',
      'obligation:o3',
    ]);
    expect(lineageRefs(lineage, 'economic_terms')).toEqual(['terms:t7']);
    expect(lineageRefs(lineage, 'rights_resolution')).toEqual([
      'rights:obligation:o1',
      'rights:obligation:o2',
      'rights:obligation:o3',
    ]);
    expect(lineageRefs(lineage, 'usage_evidence_epoch')).toEqual([
      'usage:obligation:o1',
      'usage:obligation:o2',
      'usage:obligation:o3',
    ]);
    expect(lineageObligationAuthorities(lineage)).toEqual(
      [...obligations]
        .sort((a, b) => a.id.localeCompare(b.id))
        .map(item => ({ ref: item.id, authorityRoot: item.authorityRoot })),
    );
  });

  it('is deterministic when obligation input order changes', () => {
    const first = buildMoneyLineage(base);
    const second = buildMoneyLineage({ ...base, obligations: [...base.obligations].reverse() });
    expect(second.lineageRoot).toBe(first.lineageRoot);
  });

  it('changes lineage root when immutable debt provenance changes', () => {
    const first = buildMoneyLineage({ ...base, obligations: [obligation('obligation:o1', 'usage:a')] });
    const second = buildMoneyLineage({ ...base, obligations: [obligation('obligation:o1', 'usage:b')] });
    expect(second.lineageRoot).not.toBe(first.lineageRoot);
    expect(lineageObligationAuthorities(second)[0]?.authorityRoot).not.toBe(
      lineageObligationAuthorities(first)[0]?.authorityRoot,
    );
  });

  it('rejects duplicate obligation identities even when supplied as distinct objects', () => {
    expect(() => buildMoneyLineage({
      ...base,
      obligations: [obligation('obligation:o1'), obligation('obligation:o1')],
    })).toThrow(/ids must be unique/);
  });

  it('rejects debt belonging to another beneficiary', () => {
    const other = createRoyaltyObligationAuthority({
      id: 'obligation:other',
      beneficiaryId: 'creator:bob',
      amount: money(327n),
      observedAt: '2026-09-10T00:00:00Z',
      provenance: {
        usageEvidenceRef: 'usage:other',
        rightsResolutionRef: 'rights:other',
        economicTermsRef: 'terms:t7',
      },
    });
    expect(() => buildMoneyLineage({ ...base, obligations: [other] })).toThrow(/beneficiary must match/);
  });
});
