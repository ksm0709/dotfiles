return {
  {
    "tpope/vim-fugitive",
    cmd = {
      "G",
      "Git",
      "Gdiffsplit",
      "Gvdiffsplit",
      "Gedit",
      "Gsplit",
      "Gread",
      "Gwrite",
      "Ggrep",
      "Glgrep",
      "Gmove",
      "Gdelete",
      "Gremove",
      "Gbrowse",
    },
    -- Optionally, you can specify a config function to further customize Fugitive
    -- config = function()
    --   -- Your custom configuration here
    -- end,
  },
  {
    "lewis6991/gitsigns.nvim",
    opts = {
      current_line_blame = true,
    },
    keys = {
      { "<leader>gp", "<cmd>Gitsigns preview_hunk<CR>" },
    },
  },
}
