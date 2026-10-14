import { buildMerkleCommitment, type Digest } from './merkle.js';
import { type Money } from './money.js';

export interface ReportingProjectionSource {
  readonly statementId: string;
  /** SHA-256 root of the authoritative Mycelix statement commitment. */
  readonly statementCommitmentRoot: Digest;
  readonly generatedAt: string;
}

export interface RoyaltyReportingLine {
  readonly obligationId: string;
  readonly workReference: string;
  readonly beneficiaryReference: string;
  readonly amount: Money;
  readonly usageReference?: string;
  /** Two-letter territory code, normalized to uppercase. */
  readonly territory?: string;
}

export interface ReportingMoney {
  /** Exact integer minor units; never converted through JavaScript number. */
  readonly amountMinor: string;
  readonly currency: string;
}

export const ROYALTY_REPORTING_MAPPING_VERSION = 'mycelix-royalty-reporting-mapping-v1' as const;
export const REPORTING_STANDARD_CONFORMANCE = 'mapping_only_not_schema_conformant' as const;

interface ReportingProjectionBase {
  readonly authority: 'reporting_projection_only';
  readonly mappingVersion: typeof ROYALTY_REPORTING_MAPPING_VERSION;
  /**
   * This is a Mycelix field mapping, not an official DDEX/CISAC message. A
   * format-specific adapter must still map to the licensed/current schema,
   * apply profile rules, and validate against its official schema.
   */
  readonly standardConformance: typeof REPORTING_STANDARD_CONFORMANCE;
  /** Native amount unit is preserved; no FX conversion or ISO 4217 mapping occurs here. */
  readonly currencySemantics: 'source_accounting_unit_preserved_no_fx_conversion';
  readonly profileVersion: string;
  readonly source: ReportingProjectionSource;
}

export interface DdexRoyaltyReportingLine {
  readonly obligationReference: string;
  readonly workReference: string;
  readonly payeeReference: string;
  readonly royaltyAmount: ReportingMoney;
  readonly usageReference?: string;
  readonly territory?: string;
}

export interface DdexRoyaltyReportingProjection extends ReportingProjectionBase {
  readonly format: 'DDEX';
  readonly source: ReportingProjectionSource;
  readonly lines: readonly DdexRoyaltyReportingLine[];
  readonly projectionRoot: Digest;
}

export interface CisacCrdReportingLine {
  readonly distributionReference: string;
  readonly workReference: string;
  readonly interestedPartyReference: string;
  readonly royaltyAmount: ReportingMoney;
}

export interface CisacCrdReportingProjection extends ReportingProjectionBase {
  readonly format: 'CISAC_CRD';
  readonly source: ReportingProjectionSource;
  readonly lines: readonly CisacCrdReportingLine[];
  readonly projectionRoot: Digest;
}

const SHA256_HEX = /^[0-9a-f]{64}$/;
const REPORTING_ROOT_DOMAIN = 'mycelix-royalty-reporting-projection-v1';

function dataProperty(value: unknown, key: string, owner: string, optional = false): unknown {
  if (value === null || typeof value !== 'object') {
    throw new Error(`${owner} must be an object with own data properties`);
  }
  const descriptor = Object.getOwnPropertyDescriptor(value, key);
  if (descriptor === undefined) {
    if (optional) return undefined;
    throw new Error(`${owner}.${key} must be an own data property`);
  }
  if (!Object.prototype.hasOwnProperty.call(descriptor, 'value')) {
    throw new Error(`${owner}.${key} must not be accessor-backed`);
  }
  return descriptor.value;
}

function snapshotArrayData(value: unknown, label: string): readonly unknown[] {
  if (!Array.isArray(value)) throw new Error(`${label} must be an array`);
  const lengthDescriptor = Object.getOwnPropertyDescriptor(value, 'length');
  const length = lengthDescriptor?.value;
  if (!Number.isSafeInteger(length) || length < 0) {
    throw new Error(`${label} has an invalid array length`);
  }
  if (Object.getOwnPropertySymbols(value).length > 0) {
    throw new Error(`${label} must not contain symbol properties`);
  }
  const names = Object.getOwnPropertyNames(value);
  if (names.length !== length + 1 || names.some(key =>
    key !== 'length' && (!/^(0|[1-9][0-9]*)$/.test(key) || Number(key) >= length)
  )) {
    throw new Error(`${label} must not contain sparse indices or extra properties`);
  }
  const snapshot: unknown[] = [];
  for (let index = 0; index < length; index += 1) {
    snapshot.push(dataProperty(value, String(index), `${label}[${index}]`));
  }
  return Object.freeze(snapshot);
}

/**
 * Copy a projection into a detached data-only tree before hashing. Accessors,
 * prototypes, symbols, sparse arrays and cycles are rejected so verification
 * never hashes a different set of values from the ones it validated.
 */
function snapshotDataTree(value: unknown, label: string, ancestors = new Set<object>()): unknown {
  if (value === null || typeof value !== 'object') {
    if (typeof value === 'function') throw new Error(`${label} must not be a function`);
    return value;
  }
  const objectValue = value as object;
  if (ancestors.has(objectValue)) throw new Error(`${label} must not contain cycles`);
  ancestors.add(objectValue);
  try {
    if (Array.isArray(value)) {
      return Object.freeze(snapshotArrayData(value, label).map((entry, index) =>
        snapshotDataTree(entry, `${label}[${index}]`, ancestors)
      ));
    }
    const prototype = Object.getPrototypeOf(objectValue);
    if (prototype !== Object.prototype && prototype !== null) {
      throw new Error(`${label} must be a plain data object`);
    }
    if (Object.getOwnPropertySymbols(objectValue).length > 0) {
      throw new Error(`${label} must not contain symbol properties`);
    }
    const detached = Object.create(null) as Record<string, unknown>;
    for (const key of Object.getOwnPropertyNames(objectValue)) {
      const descriptor = Object.getOwnPropertyDescriptor(objectValue, key);
      if (descriptor === undefined) {
        throw new Error(`${label}.${key} must be an own data property`);
      }
      if (!Object.prototype.hasOwnProperty.call(descriptor, 'value')) {
        throw new Error(`${label}.${key} must not be accessor-backed`);
      }
      if (!descriptor.enumerable) {
        throw new Error(`${label}.${key} must be enumerable`);
      }
      detached[key] = snapshotDataTree(descriptor.value, `${label}.${key}`, ancestors);
    }
    return Object.freeze(detached);
  } finally {
    ancestors.delete(objectValue);
  }
}

function snapshotProjectionRecord(value: unknown, label: string): Record<string, unknown> {
  const snapshot = snapshotDataTree(value, label);
  if (snapshot === null || typeof snapshot !== 'object' || Array.isArray(snapshot)) {
    throw new Error(`${label} must be a plain projection object`);
  }
  return snapshot as Record<string, unknown>;
}

function required(label: string, value: unknown): string {
  if (typeof value !== 'string') throw new Error(`${label} must be a string`);
  const normalized = value.trim();
  if (!normalized) throw new Error(`${label} must be non-empty`);
  return normalized;
}

function validateSource(sourceInput: unknown): ReportingProjectionSource {
  const statementId = required(
    'reporting source statementId',
    dataProperty(sourceInput, 'statementId', 'reporting source'),
  );
  const rootInput = dataProperty(sourceInput, 'statementCommitmentRoot', 'reporting source');
  if (typeof rootInput !== 'string' || !SHA256_HEX.test(rootInput)) {
    throw new Error('reporting source statementCommitmentRoot must be a lowercase SHA-256 digest');
  }
  const generatedAt = required(
    'reporting source generatedAt',
    dataProperty(sourceInput, 'generatedAt', 'reporting source'),
  );
  const parsed = Date.parse(generatedAt);
  if (!Number.isFinite(parsed)) {
    throw new Error('reporting source generatedAt must be a valid timestamp');
  }
  return Object.freeze({
    statementId,
    statementCommitmentRoot: rootInput as Digest,
    generatedAt: new Date(parsed).toISOString(),
  });
}

function validateProfileVersion(profileVersion: string): string {
  return required('external reporting profileVersion', profileVersion);
}

function normalizedLines(linesInput: unknown): readonly RoyaltyReportingLine[] {
  const inputLines = snapshotArrayData(linesInput, 'reporting lines');
  const normalized = inputLines.map((lineInput, index) => {
    const label = `reporting line[${index}]`;
    const obligationId = required('reporting obligationId', dataProperty(lineInput, 'obligationId', label));
    const workReference = required('reporting workReference', dataProperty(lineInput, 'workReference', label));
    const beneficiaryReference = required('reporting beneficiaryReference', dataProperty(lineInput, 'beneficiaryReference', label));
    const amountInput = dataProperty(lineInput, 'amount', label);
    const amountMinor = dataProperty(amountInput, 'amountMinor', `${label} amount`);
    if (typeof amountMinor !== 'bigint') {
      throw new Error('reporting royalty amount must use exact bigint minor units');
    }
    const currency = required(
      'reporting currency',
      dataProperty(amountInput, 'currency', `${label} amount`),
    ).toUpperCase();
    const usageInput = dataProperty(lineInput, 'usageReference', label, true);
    const usageReference = usageInput === undefined
      ? undefined
      : required('reporting usageReference', usageInput);
    const territoryInput = dataProperty(lineInput, 'territory', label, true);
    const territory = territoryInput === undefined
      ? undefined
      : required('reporting territory', territoryInput).toUpperCase();
    if (territory !== undefined && !/^[A-Z]{2}$/.test(territory)) {
      throw new Error('reporting territory must be a two-letter uppercase code');
    }
    if (amountMinor < 0n) throw new Error('reporting royalty amount must be non-negative');
    return Object.freeze({
      obligationId,
      workReference,
      beneficiaryReference,
      amount: Object.freeze({ amountMinor, currency }),
      ...(usageReference === undefined ? {} : { usageReference }),
      ...(territory === undefined ? {} : { territory }),
    });
  }).sort((a, b) => a.obligationId < b.obligationId ? -1 : a.obligationId > b.obligationId ? 1 : 0);

  const seen = new Set<string>();
  for (const line of normalized) {
    if (seen.has(line.obligationId)) throw new Error('reporting projection contains duplicate obligation id');
    seen.add(line.obligationId);
  }
  return Object.freeze(normalized);
}

function reportingMoney(amount: Money): ReportingMoney {
  return Object.freeze({ amountMinor: amount.amountMinor.toString(10), currency: amount.currency });
}

function computeProjectionRoot(
  value: Omit<DdexRoyaltyReportingProjection, 'projectionRoot'> | Omit<CisacCrdReportingProjection, 'projectionRoot'>,
): Digest {
  return buildMerkleCommitment([{
    recordType: REPORTING_ROOT_DOMAIN,
    ...value,
  }]).root;
}

/**
 * Recompute a mapping projection's integrity commitment after storage or
 * transport. This detects edits relative to the carried root; it is not a
 * signature and does not authenticate the producer.
 */
export function assertDdexRoyaltyReportingProjection(
  projection: DdexRoyaltyReportingProjection,
): void {
  const snapshot = snapshotProjectionRecord(projection, 'DDEX mapping projection');
  const projectionRoot = snapshot.projectionRoot;
  const payload = Object.create(null) as Record<string, unknown>;
  for (const key of Object.keys(snapshot)) if (key !== 'projectionRoot') payload[key] = snapshot[key];
  if (typeof projectionRoot !== 'string' || !SHA256_HEX.test(projectionRoot) ||
      computeProjectionRoot(payload as Omit<DdexRoyaltyReportingProjection, 'projectionRoot'>) !== projectionRoot) {
    throw new Error('DDEX mapping projection root does not match its canonical contents');
  }
}

export function assertCisacCrdReportingProjection(
  projection: CisacCrdReportingProjection,
): void {
  const snapshot = snapshotProjectionRecord(projection, 'CISAC CRD mapping projection');
  const projectionRoot = snapshot.projectionRoot;
  const payload = Object.create(null) as Record<string, unknown>;
  for (const key of Object.keys(snapshot)) if (key !== 'projectionRoot') payload[key] = snapshot[key];
  if (typeof projectionRoot !== 'string' || !SHA256_HEX.test(projectionRoot) ||
      computeProjectionRoot(payload as Omit<CisacCrdReportingProjection, 'projectionRoot'>) !== projectionRoot) {
    throw new Error('CISAC CRD mapping projection root does not match its canonical contents');
  }
}

/**
 * Create a deterministic Mycelix mapping projection for a DDEX-related export.
 * This function does not produce an official DDEX XML/flat-file message or
 * assert conformance to RDR-R, ERN, or another DDEX profile.
 */
export function exportDdexRoyaltyReportingProjection(input: {
  profileVersion: string;
  source: ReportingProjectionSource;
  lines: readonly RoyaltyReportingLine[];
}): Readonly<DdexRoyaltyReportingProjection> {
  const profileVersion = validateProfileVersion(
    dataProperty(input, 'profileVersion', 'reporting input') as string,
  );
  const source = validateSource(dataProperty(input, 'source', 'reporting input'));
  const lineInputs = dataProperty(input, 'lines', 'reporting input');
  const lines = Object.freeze(normalizedLines(lineInputs).map(line => Object.freeze({
    obligationReference: line.obligationId,
    workReference: line.workReference,
    payeeReference: line.beneficiaryReference,
    royaltyAmount: reportingMoney(line.amount),
    ...(line.usageReference === undefined ? {} : { usageReference: line.usageReference }),
    ...(line.territory === undefined ? {} : { territory: line.territory }),
  })));
  const payload = Object.freeze({
    format: 'DDEX' as const,
    authority: 'reporting_projection_only' as const,
    mappingVersion: ROYALTY_REPORTING_MAPPING_VERSION,
    standardConformance: REPORTING_STANDARD_CONFORMANCE,
    currencySemantics: 'source_accounting_unit_preserved_no_fx_conversion' as const,
    profileVersion,
    source,
    lines,
  });
  return Object.freeze({ ...payload, projectionRoot: computeProjectionRoot(payload) });
}

/**
 * Create a deterministic Mycelix mapping projection for a CISAC CRD export.
 * This function does not produce a CRD distribution file or assert conformance
 * to the official CISAC CRD document pack or territory-specific requirements.
 */
export function exportCisacCrdReportingProjection(input: {
  profileVersion: string;
  source: ReportingProjectionSource;
  lines: readonly RoyaltyReportingLine[];
}): Readonly<CisacCrdReportingProjection> {
  const profileVersion = validateProfileVersion(
    dataProperty(input, 'profileVersion', 'reporting input') as string,
  );
  const source = validateSource(dataProperty(input, 'source', 'reporting input'));
  const lineInputs = dataProperty(input, 'lines', 'reporting input');
  const lines = Object.freeze(normalizedLines(lineInputs).map(line => Object.freeze({
    distributionReference: line.obligationId,
    workReference: line.workReference,
    interestedPartyReference: line.beneficiaryReference,
    royaltyAmount: reportingMoney(line.amount),
  })));
  const payload = Object.freeze({
    format: 'CISAC_CRD' as const,
    authority: 'reporting_projection_only' as const,
    mappingVersion: ROYALTY_REPORTING_MAPPING_VERSION,
    standardConformance: REPORTING_STANDARD_CONFORMANCE,
    currencySemantics: 'source_accounting_unit_preserved_no_fx_conversion' as const,
    profileVersion,
    source,
    lines,
  });
  return Object.freeze({ ...payload, projectionRoot: computeProjectionRoot(payload) });
}
