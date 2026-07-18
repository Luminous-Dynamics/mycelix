import {
  AppWebsocket,
  CellType,
  type AppWebsocketConnectionOptions,
  type CellInfo,
} from "@holochain/client";
import { decodeZomePayload, encodeZomeResponse } from "./wire";

type MarketplaceRuntimeConfig = {
  url?: string;
  tokenBase64?: string;
  origin?: string;
  defaultTimeoutMs?: number;
};

type MarketplaceConnectionInfo = {
  installedAppId: string;
  agentPubKey: Uint8Array;
  hostSignerAvailable: boolean;
  configuredRoles: string[];
  activeRoles: string[];
};

export type MarketplaceRoleCapabilities = {
  configuredRoles: string[];
  activeRoles: string[];
};

/**
 * Reduce AppInfo cell inventory to explicit configured and active role sets.
 * Deferred Stem roles remain configured but are not treated as callable.
 */
export function summarizeRoleCapabilities(
  cellInfo: Record<string, CellInfo[]>,
): MarketplaceRoleCapabilities {
  const configuredRoles = Object.keys(cellInfo).sort();
  const activeRoles = configuredRoles.filter((roleName) =>
    cellInfo[roleName]?.some(
      (cell) =>
        cell.type === CellType.Provisioned || cell.type === CellType.Cloned,
    ),
  );
  return { configuredRoles, activeRoles };
}

declare global {
  interface Window {
    __MYCELIX_MARKETPLACE_CONFIG__?: MarketplaceRuntimeConfig;
    __HC_ZOME_CALL_SIGNER__?: unknown;
  }
}

let appClient: AppWebsocket | undefined;

function decodeBase64Url(value: string): Uint8Array {
  const normalized = value.replace(/-/g, "+").replace(/_/g, "/");
  const padded = normalized.padEnd(Math.ceil(normalized.length / 4) * 4, "=");
  const bytes = atob(padded);
  return Uint8Array.from(bytes, (character) => character.charCodeAt(0));
}

function connectionOptions(): AppWebsocketConnectionOptions {
  const config = window.__MYCELIX_MARKETPLACE_CONFIG__ ?? {};
  const options: AppWebsocketConnectionOptions = {};

  if (config.url) {
    options.url = new URL(config.url);
  }
  if (config.tokenBase64) {
    options.token = Array.from(decodeBase64Url(config.tokenBase64));
  }
  if (config.origin) {
    options.wsClientOptions = { origin: config.origin };
  }
  if (config.defaultTimeoutMs) {
    options.defaultTimeout = config.defaultTimeoutMs;
  }

  return options;
}

/**
 * Connect using either the Launcher/Tauri-injected environment or the explicit
 * `window.__MYCELIX_MARKETPLACE_CONFIG__` object.
 *
 * This bridge never requests conductor admin access and never generates or
 * persists signing keys. Zome-call signing remains the responsibility of the
 * official client's configured host signer or previously authorized signing
 * credentials.
 */
export async function connectMarketplace(): Promise<MarketplaceConnectionInfo> {
  appClient = await AppWebsocket.connect(connectionOptions());
  const info = await appClient.appInfo();
  const capabilities = summarizeRoleCapabilities(info.cell_info);
  if (!capabilities.activeRoles.includes("marketplace")) {
    await appClient.client.close();
    appClient = undefined;
    throw new Error(
      `Installed app ${info.installed_app_id} has no active marketplace role`,
    );
  }

  return {
    installedAppId: info.installed_app_id,
    agentPubKey: appClient.myPubKey,
    hostSignerAvailable: Boolean(window.__HC_ZOME_CALL_SIGNER__),
    configuredRoles: capabilities.configuredRoles,
    activeRoles: capabilities.activeRoles,
  };
}

export async function callMarketplaceZome(
  roleName: string,
  zomeName: string,
  functionName: string,
  payloadBytes: Uint8Array,
): Promise<Uint8Array> {
  if (!appClient) {
    throw new Error("Marketplace Holochain bridge is not connected");
  }

  const payload = decodeZomePayload(payloadBytes);
  const response = await appClient.callZome({
    role_name: roleName,
    zome_name: zomeName,
    fn_name: functionName,
    payload,
  });

  return encodeZomeResponse(response);
}

export async function disconnectMarketplace(): Promise<void> {
  if (appClient) {
    await appClient.client.close();
    appClient = undefined;
  }
}
