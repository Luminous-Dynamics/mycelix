// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Cell-targeted Hearth calls.
 *
 * Household data must be written to a clone cell whose DNA modifiers identify
 * one hearth network. The provisioned `hearth` role is a template/control cell
 * and is deliberately not accepted by the unified SDK client.
 */

import type { AppClient, CellId, Signal } from '@holochain/client';

export interface HearthCellTarget {
  /** Exact clone cell to call. */
  readonly cellId: CellId;
  /** Optional conductor clone id, useful for lifecycle operations. */
  readonly cloneId: string;
  /** Human-readable local label; never used as an authorization claim. */
  readonly name?: string;
}

export async function callHearthZome<T>(
  client: AppClient,
  target: HearthCellTarget,
  zomeName: string,
  fnName: string,
  payload: unknown,
): Promise<T> {
  return client.callZome({
    cell_id: target.cellId,
    zome_name: zomeName,
    fn_name: fnName,
    payload,
  }) as Promise<T>;
}

/** Compare Holochain hashes without relying on object identity. */
export function equalBytes(left: Uint8Array, right: Uint8Array): boolean {
  if (left.byteLength !== right.byteLength) return false;
  for (let index = 0; index < left.byteLength; index += 1) {
    if (left[index] !== right[index]) return false;
  }
  return true;
}

export function equalCellId(left: CellId, right: CellId): boolean {
  return equalBytes(left[0], right[0]) && equalBytes(left[1], right[1]);
}

/** Prevent signals from one household clone leaking into another open client. */
export function isSignalForCell(signal: Signal, cellId: CellId): boolean {
  return signal.type === 'app' && equalCellId(signal.value.cell_id, cellId);
}
