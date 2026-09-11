import { createHash } from 'node:crypto';
import type { Digest } from './merkle.js';

export const ACCOUNTING_WIRE_FORMAT = 'mycelix-accounting-wire' as const;
export const ACCOUNTING_WIRE_VERSION = 1 as const;
const WIRE_DIGEST_DOMAIN = 'mycelix-accounting-wire-v1\0';
const CANONICAL_INTEGER = /^-?(0|[1-9][0-9]*)$/;
const BINARY64_HEX = /^[0-9a-f]{16}$/;

export type AccountingWireNode =
  | readonly ['null']
  | readonly ['string', string]
  | readonly ['boolean', boolean]
  | readonly ['integer', string]
  | readonly ['number', string]
  | readonly ['array', readonly AccountingWireNode[]]
  | readonly ['object', readonly (readonly [string, AccountingWireNode])[]];

export interface AccountingWireEnvelopeV1 {
  readonly format: typeof ACCOUNTING_WIRE_FORMAT;
  readonly version: typeof ACCOUNTING_WIRE_VERSION;
  readonly value: AccountingWireNode;
}

/** Canonical finite IEEE-754 binary64, big-endian, lowercase hexadecimal. */
function canonicalBinary64Hex(value: number): string {
  if (!Number.isFinite(value)) throw new Error('accounting wire numbers must be finite');
  const bytes = Buffer.allocUnsafe(8);
  bytes.writeDoubleBE(value, 0);
  return bytes.toString('hex');
}

function decodeCanonicalBinary64Hex(value: unknown): number {
  if (typeof value !== 'string' || !BINARY64_HEX.test(value)) {
    throw new Error('accounting wire number must be canonical IEEE-754 binary64 hex');
  }
  const decoded = Buffer.from(value, 'hex').readDoubleBE(0);
  if (!Number.isFinite(decoded) || canonicalBinary64Hex(decoded) !== value) {
    throw new Error('accounting wire number must be canonical finite IEEE-754 binary64 hex');
  }
  return decoded;
}

/** Wire strings must be Unicode scalar sequences, never lone UTF-16 surrogates. */
function canonicalUnicodeString(label: string, value: string): string {
  for (let index = 0; index < value.length; index += 1) {
    const codeUnit = value.charCodeAt(index);
    if (codeUnit >= 0xd800 && codeUnit <= 0xdbff) {
      const next = value.charCodeAt(index + 1);
      if (!(next >= 0xdc00 && next <= 0xdfff)) {
        throw new Error(`${label} must not contain lone UTF-16 surrogates`);
      }
      index += 1;
      continue;
    }
    if (codeUnit >= 0xdc00 && codeUnit <= 0xdfff) {
      throw new Error(`${label} must not contain lone UTF-16 surrogates`);
    }
  }
  return value;
}

/** Language-neutral lexicographic order over UTF-8 key bytes. */
function compareUtf8Keys(left: string, right: string): number {
  return Buffer.compare(Buffer.from(left, 'utf8'), Buffer.from(right, 'utf8'));
}

function isPlainObject(value: object): boolean {
  const prototype = Object.getPrototypeOf(value);
  return prototype === Object.prototype || prototype === null;
}

export function encodeAccountingWireNode(value: unknown): AccountingWireNode {
  if (value === null) return Object.freeze(['null'] as const);
  if (typeof value === 'string') {
    return Object.freeze(['string', canonicalUnicodeString('accounting wire string', value)] as const);
  }
  if (typeof value === 'boolean') return Object.freeze(['boolean', value] as const);
  if (typeof value === 'bigint') return Object.freeze(['integer', value.toString(10)] as const);
  if (typeof value === 'number') return Object.freeze(['number', canonicalBinary64Hex(value)] as const);
  if (Array.isArray(value)) {
    return Object.freeze(['array', Object.freeze(value.map(encodeAccountingWireNode))] as const);
  }
  if (typeof value === 'object') {
    if (!isPlainObject(value)) {
      throw new Error('accounting wire objects must be plain records');
    }
    const record = value as Record<string, unknown>;
    const keys = Object.keys(record)
      .map(key => canonicalUnicodeString('accounting wire object key', key))
      .sort(compareUtf8Keys);
    const entries = keys.map(key => {
      if (record[key] === undefined) throw new Error('accounting wire values forbid undefined');
      return Object.freeze([key, encodeAccountingWireNode(record[key])] as const);
    });
    return Object.freeze(['object', Object.freeze(entries)] as const);
  }
  throw new Error(`unsupported accounting wire value: ${typeof value}`);
}

function expectArray(value: unknown, label: string): readonly unknown[] {
  if (!Array.isArray(value)) throw new Error(`${label} must be an array`);
  return value;
}

function canonicalIntegerText(value: unknown): string {
  if (typeof value !== 'string' || !CANONICAL_INTEGER.test(value) || value === '-0') {
    throw new Error('accounting wire integer must be canonical decimal text');
  }
  if (BigInt(value).toString(10) !== value) {
    throw new Error('accounting wire integer must be canonical decimal text');
  }
  return value;
}

export function decodeAccountingWireNode(node: unknown): unknown {
  const tuple = expectArray(node, 'accounting wire node');
  if (typeof tuple[0] !== 'string') throw new Error('accounting wire node tag must be a string');
  switch (tuple[0]) {
    case 'null':
      if (tuple.length !== 1) throw new Error('accounting wire null node has invalid arity');
      return null;
    case 'string':
      if (tuple.length !== 2 || typeof tuple[1] !== 'string') throw new Error('accounting wire string node is invalid');
      return canonicalUnicodeString('accounting wire string', tuple[1]);
    case 'boolean':
      if (tuple.length !== 2 || typeof tuple[1] !== 'boolean') throw new Error('accounting wire boolean node is invalid');
      return tuple[1];
    case 'integer':
      if (tuple.length !== 2) throw new Error('accounting wire integer node has invalid arity');
      return BigInt(canonicalIntegerText(tuple[1]));
    case 'number':
      if (tuple.length !== 2) throw new Error('accounting wire number node has invalid arity');
      return decodeCanonicalBinary64Hex(tuple[1]);
    case 'array': {
      if (tuple.length !== 2) throw new Error('accounting wire array node has invalid arity');
      const items = expectArray(tuple[1], 'accounting wire array payload');
      return Object.freeze(items.map(decodeAccountingWireNode));
    }
    case 'object': {
      if (tuple.length !== 2) throw new Error('accounting wire object node has invalid arity');
      const entries = expectArray(tuple[1], 'accounting wire object entries');
      const decodedEntries: Array<readonly [string, unknown]> = [];
      let previous: string | undefined;
      for (const entryValue of entries) {
        const entry = expectArray(entryValue, 'accounting wire object entry');
        if (entry.length !== 2 || typeof entry[0] !== 'string') {
          throw new Error('accounting wire object entry must contain string key and value node');
        }
        const key = canonicalUnicodeString('accounting wire object key', entry[0]);
        if (previous !== undefined && compareUtf8Keys(previous, key) >= 0) {
          throw new Error('accounting wire object keys must be strictly sorted and unique by UTF-8 bytes');
        }
        previous = key;
        decodedEntries.push(Object.freeze([key, decodeAccountingWireNode(entry[1])] as const));
      }
      return Object.freeze(Object.fromEntries(decodedEntries));
    }
    default:
      throw new Error(`unsupported accounting wire node tag: ${tuple[0]}`);
  }
}

export function createAccountingWireEnvelope(value: unknown): Readonly<AccountingWireEnvelopeV1> {
  return Object.freeze({
    format: ACCOUNTING_WIRE_FORMAT,
    version: ACCOUNTING_WIRE_VERSION,
    value: encodeAccountingWireNode(value),
  });
}

function assertEnvelope(value: unknown): Readonly<AccountingWireEnvelopeV1> {
  if (value === null || typeof value !== 'object' || Array.isArray(value)) {
    throw new Error('accounting wire envelope must be an object');
  }
  const record = value as Record<string, unknown>;
  const keys = Object.keys(record);
  if (keys.length !== 3 || keys[0] !== 'format' || keys[1] !== 'version' || keys[2] !== 'value') {
    throw new Error('accounting wire envelope fields must be exactly format, version, value in canonical order');
  }
  if (record.format !== ACCOUNTING_WIRE_FORMAT || record.version !== ACCOUNTING_WIRE_VERSION) {
    throw new Error('unsupported accounting wire format/version');
  }
  const decoded = decodeAccountingWireNode(record.value);
  return createAccountingWireEnvelope(decoded);
}

export function serializeAccountingWire(value: unknown): string {
  return JSON.stringify(createAccountingWireEnvelope(value));
}

export function deserializeAccountingWire(text: string): unknown {
  if (!text.length || text !== text.trim()) throw new Error('accounting wire text must be canonical JSON without surrounding whitespace');
  let parsed: unknown;
  try {
    parsed = JSON.parse(text) as unknown;
  } catch {
    throw new Error('accounting wire text must be valid JSON');
  }
  const canonical = assertEnvelope(parsed);
  const canonicalText = JSON.stringify(canonical);
  if (canonicalText !== text) {
    throw new Error('accounting wire text is not in canonical serialized form');
  }
  return decodeAccountingWireNode(canonical.value);
}

export function accountingWireDigestFromText(text: string): Digest {
  deserializeAccountingWire(text);
  return createHash('sha256').update(WIRE_DIGEST_DOMAIN + text, 'utf8').digest('hex');
}

export function accountingWireDigest(value: unknown): Digest {
  return accountingWireDigestFromText(serializeAccountingWire(value));
}
