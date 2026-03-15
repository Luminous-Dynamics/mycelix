

export const index = 6;
let component_cache;
export const component = async () => component_cache ??= (await import('../entries/pages/epistemic-markets/_page.svelte.js')).default;
export const imports = ["_app/immutable/nodes/6.C_AaC83Y.js","_app/immutable/chunks/DMvb8rdj.js","_app/immutable/chunks/Ct2uKrlJ.js","_app/immutable/chunks/Bwzdl04Z.js"];
export const stylesheets = ["_app/immutable/assets/6.Dpyd8UR8.css"];
export const fonts = [];
