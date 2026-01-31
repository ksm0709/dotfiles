// Type definitions for @opencode-ai/plugin
// Temporary declarations until the actual package is installed

declare module "@opencode-ai/plugin" {
  export interface PluginContext {
    client: any;
  }

  export interface Plugin {
    (context: PluginContext): Promise<{
      tool?: Record<string, any>;
    }>;
  }

  export interface ToolSchema {
    string: (options?: { description?: string }) => any;
    enum: <T extends string>(values: T[], options?: { description?: string }) => any;
    array: (itemSchema: any, options?: { description?: string }) => any;
    object: (properties: Record<string, any>) => any;
    any: () => any;
    optional: (schema: any) => any;
  }

  export interface ToolConfig {
    description: string;
    args: Record<string, any>;
    execute: (args: any, ctx: any) => Promise<any>;
  }

  export interface ToolFunction {
    (config: ToolConfig): any;
    schema: ToolSchema;
  }

  export const tool: ToolFunction;
}
