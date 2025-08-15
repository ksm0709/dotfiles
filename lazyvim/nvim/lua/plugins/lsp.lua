return {
  {
    "mason-org/mason-lspconfig.nvim",
    opts = {
      ensure_installed = {
        --"clangd",
        "lua_ls",
      },
    },
    dependencies = {
      { "mason-org/mason.nvim", opts = {} },
      {
        "neovim/nvim-lspconfig",
        opts = {
          servers = {
            clangd = {
              mason = false,
            },
          },
        },
      },
    },
  },
}
