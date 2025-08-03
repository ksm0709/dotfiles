return {
  "Saghen/blink.cmp",
  -- Keymaps for better super-tab behaviors
  opts = {
    keymap = {
      ["<C-space>"] = { "show", "show_documentation", "hide_documentation" },
      ["<C-e>"] = { "hide", "fallback" },

      ["<Tab>"] = {
        function(cmp)
          if cmp.snippet_active() then
            return cmp.accept()
          else
            return cmp.select_and_accept()
          end
        end,
        "snippet_forward",
        "fallback",
      },
      ["<S-Tab>"] = { "snippet_backward", "fallback" },

      ["<Up>"] = { "select_prev", "fallback" },
      ["<Down>"] = { "select_next", "fallback" },
      ["<C-p>"] = { "select_prev", "fallback_to_mappings" },
      ["<C-n>"] = { "select_next", "fallback_to_mappings" },

      ["<C-b>"] = { "scroll_documentation_up", "fallback" },
      ["<C-f>"] = { "scroll_documentation_down", "fallback" },

      ["<C-k>"] = { "show_signature", "hide_signature", "fallback" },
    },
  },
  -- For avante.nvim
  --  {
  --    "saghen/blink.cmp",
  --    dependencies = {
  --      "kaiser-yang/blink-cmp-avante",
  --      -- ... other dependencies
  --    },
  --    opts = {
  --      sources = {
  --        -- add 'avante' to the list
  --        default = { "avante", "lsp", "path", "luasnip", "buffer" },
  --        providers = {
  --          avante = {
  --            module = "blink-cmp-avante",
  --            name = "avante",
  --            opts = {
  --              -- options for blink-cmp-avante
  --            },
  --          },
  --        },
  --      },
  --    },
  --  },
}
