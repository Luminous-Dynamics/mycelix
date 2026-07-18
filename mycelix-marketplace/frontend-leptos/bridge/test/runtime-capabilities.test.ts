import assert from "node:assert/strict";
import test from "node:test";
import { CellType, type CellInfo } from "@holochain/client";
import { summarizeRoleCapabilities } from "../src/index";

function cell(type: CellType): CellInfo {
  return { type, value: {} } as CellInfo;
}

test("role discovery separates configured Stem roles from active cells", () => {
  const result = summarizeRoleCapabilities({
    marketplace: [cell(CellType.Provisioned)],
    finance: [{ type: "stem", value: {} } as unknown as CellInfo],
    identity: [cell(CellType.Cloned)],
  });

  assert.deepEqual(result.configuredRoles, ["finance", "identity", "marketplace"]);
  assert.deepEqual(result.activeRoles, ["identity", "marketplace"]);
});

test("a provisioned Finance role becomes active capability evidence", () => {
  const result = summarizeRoleCapabilities({
    marketplace: [cell(CellType.Provisioned)],
    finance: [cell(CellType.Provisioned)],
  });

  assert.deepEqual(result.activeRoles, ["finance", "marketplace"]);
});
