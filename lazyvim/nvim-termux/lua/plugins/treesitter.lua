return {
  {
    "nvim-treesitter/nvim-treesitter",
    lazy = false,
    opts = function()
      return {
        auto_install = true,
        hgighlight = {
          enable = true,
          additional_vim_regex_highlighting = false,
        },
      }
    end,
  },
}
