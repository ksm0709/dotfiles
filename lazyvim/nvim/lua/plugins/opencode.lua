return {
  {
    "sudo-tee/opencode.nvim",
    config = function()
      require("opencode").setup({})
    end,
    dependencies = {
      "nvim-lua/plenary.nvim",
      {
        "MeanderingProgrammer/render-markdown.nvim",
        opts = {
          anti_conceal = { enabled = false },
          file_types = { "markdown", "opencode_output" },
        },
        ft = { "markdown", "Avante", "copilot-chat", "opencode_output" },
      },
      -- Optional, for file mentions and commands completion, pick only one
      "saghen/blink.cmp",
      -- 'hrsh7th/nvim-cmp',

      -- Optional, for file mentions picker, pick only one
      "folke/snacks.nvim",
      -- 'nvim-telescope/telescope.nvim',
      -- 'ibhagwan/fzf-lua',
      -- 'nvim_mini/mini.nvim',
    },
  },
  --{
  --  "NickvanDyke/opencode.nvim",
  --  dependencies = {
  --    -- Recommended for `ask()`, required for `toggle()` — otherwise optional
  --    { "folke/snacks.nvim", opts = { input = { enabled = true } } },
  --  },
  --  config = function()
  --    vim.g.opencode_opts = {
  --      -- Your configuration, if any — see `lua/opencode/config.lua`
  --    }

  --    -- Required for `vim.g.opencode_opts.auto_reload`
  --    vim.opt.autoread = true

  --    -- Recommended/example keymaps
  --    vim.keymap.set({ "n", "x" }, "<LocalLeader>oa", function()
  --      require("opencode").ask("@this: ", { submit = true })
  --    end, { desc = "Ask about this" })
  --    vim.keymap.set({ "n", "x" }, "<LocalLeader>o+", function()
  --      require("opencode").prompt("@this")
  --    end, { desc = "Add this" })
  --    vim.keymap.set({ "n", "x" }, "<LocalLeader>oe", function()
  --      require("opencode").prompt("Explain @this and its context", { submit = true })
  --    end, { desc = "Explain this" })
  --    vim.keymap.set({ "n", "x" }, "<LocalLeader>os", function()
  --      require("opencode").select()
  --    end, { desc = "Select prompt" })
  --    vim.keymap.set("n", "<LocalLeader>oo", function()
  --      require("opencode").toggle()
  --    end, { desc = "Toggle embedded" })
  --    vim.keymap.set("n", "<LocalLeader>on", function()
  --      require("opencode").command("session_new")
  --    end, { desc = "New session" })
  --    vim.keymap.set("n", "<LocalLeader>oi", function()
  --      require("opencode").command("session_interrupt")
  --    end, { desc = "Interrupt session" })
  --    vim.keymap.set("n", "<S-C-u>", function()
  --      require("opencode").command("messages_half_page_up")
  --    end, { desc = "Messages half page up" })
  --    vim.keymap.set("n", "<S-C-d>", function()
  --      require("opencode").command("messages_half_page_down")
  --    end, { desc = "Messages half page down" })
  --  end,
  --},
}
