return {
  {
    "Exafunction/windsurf.nvim",
    dependencies = {
      "nvim-lua/plenary.nvim",
      "saghen/blink.cmp",
    },
    config = function()
      require("codeium").setup({
        enable_cmp_source = false,
      })
    end,
  },
  {
    "olimorris/codecompanion.nvim",
    opts = {
      strategies = {
        chat = {
          adapter = "gemini",
        },
        inline = {
          adapter = "gemini",
        },
        cmd = {
          adapter = "gemini",
        },
      },
    },
    dependencies = {
      "nvim-lua/plenary.nvim",
      "nvim-treesitter/nvim-treesitter",
    },
    keys = {
      { "<C-a>", "<cmd>CodeCompanionActions<cr>", desc = "CodeCompanionActions", noremap = true, silent = true },
      { "ga", "<cmd>CodeCompanionChat Add<cr>", desc = "CodeCompanionChat Add", noremap = true, silent = true },
      {
        "<LocalLeader>a",
        "<cmd>CodeCompanionChat Toggle<cr>",
        desc = "CodeCompanionChat Toggle",
        noremap = true,
        silent = true,
      },
    },
  },
}
