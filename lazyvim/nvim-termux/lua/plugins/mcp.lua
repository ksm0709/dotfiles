return {
  "ravitemer/mcphub.nvim",
  dependencies = {
    "nvim-lua/plenary.nvim",
  },
  build = "npm install -g mcp-hub@latest", -- Installs `mcp-hub` node binary globally
  opts = {
    global_env = function(context)
      local env = {}
      env.ALLOWED_DIRECTORY = vim.fn.expand("~")
      env.DEFAULT_MINIMUM_TOKENS = "10000"
      env.GOOGLE_API_KEY = "${env: GEMINI_API_KEY}"

      -- Add context-aware variables
      if context.is_workspace_mode and context.workspace_root then
        env.WORKSPACE_ROOT = context.workspace_root
        env.WORKSPACE_PORT = tostring(context.port)
      end
      if context.config_files then
        env.CONFIG_FILES = table.concat(context.config_files, ":")
      end
      return env
    end,
    log = {
      level = vim.log.levels.INFO, -- DEBUG, INFO, WARN, ERROR
      to_file = true, -- ~/.local/state/nvim/mcphub.log
    },
    on_ready = function()
      vim.notify("MCP Hub is online!")
    end,
  },
}
