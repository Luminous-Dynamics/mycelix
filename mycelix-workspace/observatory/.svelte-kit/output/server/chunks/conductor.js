import "@holochain/client";
import { w as writable } from "./index.js";
const conductorStatus = writable("disconnected");
const conductorStatus$ = { subscribe: conductorStatus.subscribe };
export {
  conductorStatus$ as a,
  conductorStatus as c
};
