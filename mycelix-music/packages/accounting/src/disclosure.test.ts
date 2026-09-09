import { describe, expect, it } from 'vitest';
import { createSelectiveDisclosureBundle, verifySelectiveDisclosureBundle } from './disclosure.js';
import { buildMerkleCommitment } from './merkle.js';

const lines = [
  { id: 'obl:1', amountMinor: 100n, currency: 'USD' },
  { id: 'obl:2', amountMinor: 200n, currency: 'USD' },
  { id: 'obl:3', amountMinor: 300n, currency: 'USD' },
];
const root = buildMerkleCommitment(lines).root;
const policy = {
  policyId: 'disclosure:auditor:1',
  statementId: 'statement:1',
  audience: 'auditor:example',
  purpose: 'verify sampled royalty obligations',
  allowedSections: ['obligations'] as const,
  maxItemsPerSection: 2,
  notBefore: '2026-09-01T00:00:00Z',
  expiresAt: '2026-10-01T00:00:00Z',
};

describe('selective statement disclosure', () => {
  it('reveals only selected lines with independently verifiable inclusion proofs', () => {
    const bundle = createSelectiveDisclosureBundle({
      statementId: 'statement:1',
      section: 'obligations',
      expectedRoot: root,
      lines,
      indexes: [2, 0],
      policy,
      disclosedAt: '2026-09-10T00:00:00Z',
    });
    expect(bundle.items.map(item => item.index)).toEqual([0, 2]);
    expect(bundle.proofKind).toBe('merkle_inclusion_not_zero_knowledge');
    expect(verifySelectiveDisclosureBundle(bundle)).toBe(true);
  });

  it('refuses lines that do not reproduce the authoritative statement root', () => {
    expect(() => createSelectiveDisclosureBundle({
      statementId: 'statement:1',
      section: 'obligations',
      expectedRoot: root,
      lines: [{ id: 'tampered' }],
      indexes: [0],
      policy,
      disclosedAt: '2026-09-10T00:00:00Z',
    })).toThrow(/do not match expected statement root/);
  });

  it('enforces audience policy lifetime and item limits', () => {
    expect(() => createSelectiveDisclosureBundle({
      statementId: 'statement:1',
      section: 'obligations',
      expectedRoot: root,
      lines,
      indexes: [0, 1, 2],
      policy,
      disclosedAt: '2026-09-10T00:00:00Z',
    })).toThrow(/item limit/);

    expect(() => createSelectiveDisclosureBundle({
      statementId: 'statement:1',
      section: 'obligations',
      expectedRoot: root,
      lines,
      indexes: [0],
      policy,
      disclosedAt: '2026-10-01T00:00:00Z',
    })).toThrow(/not active/);
  });
});
