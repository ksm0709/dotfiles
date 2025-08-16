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
    lazy = false,
    opts = function()
      return {
        log_level = "INFO", -- TRACE|DEBUG|ERROR|INFO

        -- Adapter setup, define default model here. To change adapter, use "ga" in chat
        adapters = {
          opts = {
            show_model_choices = true,
          },
          gemini = function()
            return require("codecompanion.adapters").extend("gemini", {
              schema = {
                model = {
                  default = "gemini-2.5-pro",
                },
              },
            })
          end,
          openai = function()
            return require("codecompanion.adapters").extend("openai", {
              schema = {
                model = {
                  default = "gpt-4.1",
                },
              },
            })
          end,
        },
        language = "Korean",
        system_prompt = function()
          local path = vim.fn.expand("~/.config/nvim/prompts/system_prompt.md")
          if vim.fn.filereadable(path) == 1 then
            local f = io.open(path, "r")
            if f then
              local content = f:read("*a")
              f:close()
              return content
            end
          else
            vim.notify("System prompt file not readable at: " .. path, vim.log.levels.WARN)
          end
          return nil -- Fallback to default
        end,
        strategies = {
          chat = {
            adapter = {
              name = "gemini",
              model = "gemini-2.5-pro",
            },

            tools = {
              groups = {
                ["dev_tools"] = {
                  description = "Can run code, edit code and modify files",
                  tools = {
                    "task_master",
                    "sequentialthinking",
                    "insert_edit_into_file",
                    "read_file",
                    "filesystem",
                    "neovim",
                    "cmd_runner",
                    "tavily",
                    "list_code_usages",
                    "mcp",
                  },
                  opts = {
                    collapse_tools = true,
                  },
                },
              },
              opts = {
                -- List of default tools to use in new chatbuffer
                default_tools = {},
                auto_submit_errors = true, -- Send any errors to the LLM automatically
                auto_submit_success = true, -- Send any successful output to the LLM automatically
              },
            },
            roles = {
              ---The header name for the LLM's messages
              ---@type string|fun(adapter: CodeCompanion.Adapter): string
              llm = function(adapter)
                return "CodeCompanion (" .. adapter.model.name .. ")"
              end,

              ---The header name for your messages
              ---@type string
              user = "User",
            },
            icons = {
              chat_context = "📎️", -- You can also apply an icon to the fold
            },
            fold_context = true,
            show_settings = true, -- Show LLM settings at the top of the chat buffer?
            keymaps = {
              options = {
                modes = {
                  n = "?",
                },
                callback = "keymaps.options",
                description = "Options",
                hide = true,
              },
              completion = {
                modes = {
                  i = "<C-_>",
                },
                index = 1,
                callback = "keymaps.completion",
                description = "Completion Menu",
              },
              send = {
                modes = {
                  n = { "<CR>", "<C-s>" },
                  i = "<C-s>",
                },
                index = 2,
                callback = "keymaps.send",
                description = "Send",
              },
              regenerate = {
                modes = {
                  n = "<LocalLeader>r",
                },
                index = 3,
                callback = "keymaps.regenerate",
                description = "Regenerate the last response",
              },
              close = {
                modes = {
                  n = "<LocalLeader>q",
                },
                index = 4,
                callback = "keymaps.close",
                description = "Close Chat",
              },
              stop = {
                modes = {
                  n = "q",
                },
                index = 5,
                callback = "keymaps.stop",
                description = "Stop Request",
              },
              clear = {
                modes = {
                  n = "<LocalLeader>x",
                },
                index = 6,
                callback = "keymaps.clear",
                description = "Clear Chat",
              },
              codeblock = {
                modes = {
                  n = "gc",
                },
                index = 7,
                callback = "keymaps.codeblock",
                description = "Insert Codeblock",
              },
              yank_code = {
                modes = {
                  n = "gy",
                },
                index = 8,
                callback = "keymaps.yank_code",
                description = "Yank Code",
              },
              pin = {
                modes = {
                  n = "gp",
                },
                index = 9,
                callback = "keymaps.pin_context",
                description = "Pin context",
              },
              watch = {
                modes = {
                  n = "gw",
                },
                index = 10,
                callback = "keymaps.toggle_watch",
                description = "Watch Buffer",
              },
              next_chat = {
                modes = {
                  n = "}",
                },
                index = 11,
                callback = "keymaps.next_chat",
                description = "Next Chat",
              },
            },
          },
          inline = {
            adapter = {
              name = "gemini",
              model = "gemini-2.5-flash",
            },
          },
          cmd = {
            adapter = {
              name = "gemini",
              model = "gemini-2.5-flash",
            },
          },
        },

        extensions = {
          mcphub = {
            callback = "mcphub.extensions.codecompanion",
            opts = {
              make_vars = true,
              make_slash_commands = true,
              show_result_in_chat = true,
            },
          },
          spinner = {},
          history = {
            enabled = true,
            opts = {
              keymap = "<localleader>h",
              save_chat_keymap = "<localleader>s",
              auto_save = true,
            },
          },
        },
      }
    end,
    dependencies = {
      "nvim-lua/plenary.nvim",
      "nvim-treesitter/nvim-treesitter",
      "franco-ruggeri/codecompanion-spinner.nvim",
      "ravitemer/codecompanion-history.nvim",
      "ravitemer/mcphub.nvim",
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
