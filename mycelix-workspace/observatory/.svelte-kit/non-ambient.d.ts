
// this file is generated — do not edit it


declare module "svelte/elements" {
	export interface HTMLAttributes<T> {
		'data-sveltekit-keepfocus'?: true | '' | 'off' | undefined | null;
		'data-sveltekit-noscroll'?: true | '' | 'off' | undefined | null;
		'data-sveltekit-preload-code'?:
			| true
			| ''
			| 'eager'
			| 'viewport'
			| 'hover'
			| 'tap'
			| 'off'
			| undefined
			| null;
		'data-sveltekit-preload-data'?: true | '' | 'hover' | 'tap' | 'off' | undefined | null;
		'data-sveltekit-reload'?: true | '' | 'off' | undefined | null;
		'data-sveltekit-replacestate'?: true | '' | 'off' | undefined | null;
	}
}

export {};


declare module "$app/types" {
	type MatcherParam<M> = M extends (param : string) => param is (infer U extends string) ? U : string;

	export interface AppTypes {
		RouteId(): "/" | "/analytics" | "/attribution" | "/emergency" | "/epistemic-markets" | "/food" | "/governance" | "/mutual-aid" | "/mygov" | "/network" | "/tend" | "/value-anchor";
		RouteParams(): {
			
		};
		LayoutParams(): {
			"/": Record<string, never>;
			"/analytics": Record<string, never>;
			"/attribution": Record<string, never>;
			"/emergency": Record<string, never>;
			"/epistemic-markets": Record<string, never>;
			"/food": Record<string, never>;
			"/governance": Record<string, never>;
			"/mutual-aid": Record<string, never>;
			"/mygov": Record<string, never>;
			"/network": Record<string, never>;
			"/tend": Record<string, never>;
			"/value-anchor": Record<string, never>
		};
		Pathname(): "/" | "/analytics" | "/attribution" | "/emergency" | "/epistemic-markets" | "/food" | "/governance" | "/mutual-aid" | "/network" | "/tend" | "/value-anchor";
		ResolvedPathname(): `${"" | `/${string}`}${ReturnType<AppTypes['Pathname']>}`;
		Asset(): string & {};
	}
}