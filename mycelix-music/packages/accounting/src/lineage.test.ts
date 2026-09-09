import { describe, expect, it } from 'vitest';
import { buildMoneyLineage, lineageRefs } from './lineage.js';
import { money } from './money.js';

const base = {
  beneficiaryId: 'creator:alice',
  received: money(327n),
  paymentRef: 'payment:p91',
  settlementLineRef: 'settlement-line:sl88',
  nettingBatchRef: 'netting:n24',
  obligationRefs: ['obligation:o3', 'obligation:o1', 'obligation:o2'],
  economicTermsRef: 'terms:t7',
  rightsAgreementRef: 'agreement:a29',
  repertoireSnapshotRef: 'repertoire:r11',
  usageEvidenceEpochRef: 'usage:u407',
} as const;

describe('creator money lineage', () => {
  it('preserves the entire payment-to-usage chain', () => {
    const lineage = buildMoneyLineage(base);
    expect(lineageRefs(lineage, 'payment')).toEqual(['payment:p91']);
    expect(lineageRefs(lineage, 'royalty_obligation')).toEqual([
      'obligation:o1',
      'obligation:o2',
      'obligation:o3',
    ]);
    expect(lineageRefs(lineage, 'usage_evidence_epoch')).toEqual(['usage:u407']);
  });

  it('is deterministic when obligation input order changes', () => {
    const first = buildMoneyLineage(base);
    const second = buildMoneyLineage({ ...base, obligationRefs: [...base.obligationRefs].reverse() });
    expect(second.lineageRoot).toBe(first.lineageRoot);
  });

  it('rejects duplicate obligation lineage', () => {
    expect(() => buildMoneyLineage({ ...base, obligationRefs: ['obligation:o1', 'obligation:o1'] }))
      .toThrow(/unique/);
  });
});
