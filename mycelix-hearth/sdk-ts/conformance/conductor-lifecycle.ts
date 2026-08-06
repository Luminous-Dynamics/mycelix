// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

/**
 * Opt-in real-conductor clone lifecycle harness.
 *
 * Required environment:
 *   HC_ADMIN_URL=ws://127.0.0.1:4444
 *   HC_APP_URL=ws://127.0.0.1:8888
 *
 * Optional:
 *   HC_APP_ID=mycelix-hearth
 *   HC_KEEP_CLONES=1
 */

import { AdminWebsocket, AppWebsocket } from '@holochain/client';
import type { AppInfo, ClonedCell } from '@holochain/client';
import { HearthNetworkManager } from '../src/clients/hearth/network.ts';

const appId = process.env.HC_APP_ID ?? 'mycelix-hearth';
const adminUrl = requiredUrl('HC_ADMIN_URL');
const appUrl = requiredUrl('HC_APP_URL');
const keepClones = process.env.HC_KEEP_CLONES === '1';

function requiredUrl(name: string): URL {
  const value = process.env[name];
  if (!value) throw new Error(`${name} is required`);
  return new URL(value);
}

function bytesKey(value: Uint8Array): string {
  return Buffer.from(value).toString('base64url');
}

function cloneFromInfo(info: AppInfo, cloneId: string): ClonedCell {
  for (const cells of Object.values(info.cell_info)) {
    for (const cell of cells) {
      if (cell.type === 'cloned' && cell.value.clone_id === cloneId) {
        return cell.value;
      }
    }
  }
  throw new Error(`clone ${cloneId} is absent from app_info`);
}

function assert(condition: unknown, message: string): asserts condition {
  if (!condition) throw new Error(message);
}

async function authenticatedApp(admin: AdminWebsocket): Promise<AppWebsocket> {
  const issued = await admin.issueAppAuthenticationToken({
    installed_app_id: appId,
    expiry_seconds: 300,
    single_use: true,
  });
  return AppWebsocket.connect({ url: appUrl, token: issued.token });
}

async function main(): Promise<void> {
  const admin = await AdminWebsocket.connect({ url: adminUrl });
  let app: AppWebsocket | undefined;
  const created: string[] = [];

  try {
    app = await authenticatedApp(admin);
    const networks = new HearthNetworkManager(app);
    const suffix = `${Date.now()}-${crypto.randomUUID()}`;

    const first = await networks.create({ name: `lifecycle-a-${suffix}` });
    created.push(first.target.cloneId);
    const second = await networks.create({ name: `lifecycle-b-${suffix}` });
    created.push(second.target.cloneId);

    assert(
      bytesKey(first.target.cellId[0]) !== bytesKey(second.target.cellId[0]),
      'independent Hearth clone modifiers must produce distinct DNA hashes',
    );

    let info = await app.appInfo();
    assert(cloneFromInfo(info, first.target.cloneId).enabled, 'first clone should start enabled');
    assert(cloneFromInfo(info, second.target.cloneId).enabled, 'second clone should start enabled');

    await networks.disable(first.target);
    info = await app.appInfo();
    assert(!cloneFromInfo(info, first.target.cloneId).enabled, 'disabled clone remained callable');
    assert(cloneFromInfo(info, second.target.cloneId).enabled, 'disabling one clone affected another');

    await app.client.close();
    app = await authenticatedApp(admin);
    info = await app.appInfo();
    assert(
      !cloneFromInfo(info, first.target.cloneId).enabled,
      'disabled state did not survive app-WebSocket reconnect',
    );
    assert(
      cloneFromInfo(info, second.target.cloneId).enabled,
      'enabled sibling clone disappeared across reconnect',
    );

    const reconnectedNetworks = new HearthNetworkManager(app);
    await reconnectedNetworks.enable(first.target);
    info = await app.appInfo();
    assert(cloneFromInfo(info, first.target.cloneId).enabled, 're-enabled clone is not enabled');

    console.log(JSON.stringify({
      ok: true,
      app_id: appId,
      first_clone: first.target.cloneId,
      second_clone: second.target.cloneId,
      checks: [
        'distinct-dna-hashes',
        'disable-isolation',
        'reconnect-state-persistence',
        're-enable',
      ],
    }, null, 2));
  } finally {
    if (!keepClones) {
      if (app) {
        const networks = new HearthNetworkManager(app);
        const info = await app.appInfo().catch(() => undefined);
        if (info) {
          for (const cloneId of created) {
            const clone = cloneFromInfo(info, cloneId);
            if (clone.enabled) {
              await networks.disable({
                cellId: clone.cell_id,
                cloneId: clone.clone_id,
                name: clone.name,
              }).catch(() => undefined);
            }
          }
        }
      }
      for (const cloneId of created) {
        await admin.deleteCloneCell({
          app_id: appId,
          clone_cell_id: { type: 'clone_id', value: cloneId },
        }).catch(() => undefined);
      }
    }
    if (app) await app.client.close().catch(() => undefined);
    await admin.client.close().catch(() => undefined);
  }
}

await main();
