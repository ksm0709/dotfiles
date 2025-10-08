return {
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
        display = {
          action_palette = {
            width = 100,
            height = 20,
            prompt = "Prompt ", -- Prompt used for interactive LLM calls
            provider = "default", -- Can be "default", "telescope", "fzf_lua", "mini_pick" or "snacks". If not specified, the plugin will autodetect installed providers.
            opts = {
              show_default_actions = true, -- Show the default actions in the action palette?
              show_default_prompt_library = true, -- Show the default prompt library in the action palette?
              title = "CodeCompanion actions", -- The title of the action palette
            },
          },
        },
        strategies = {
          chat = {
            adapter = {
              name = "copilot",
              model = "gpt-4.1",
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
              clear = {
                modes = {
                  n = "<LocalLeader>x",
                },
                index = 6,
                callback = "keymaps.clear",
                description = "Clear Chat",
              },
            },
          },
          inline = {
            adapter = {
              name = "copilot",
              model = "gpt-4.1",
            },
          },
          cmd = {
            adapter = {
              model = "copilot",
              name = "gemini",
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
      "ravitemer/codecompanion-history.nvim",
      "ravitemer/mcphub.nvim",
      "j-hui/fidget.nvim",
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
    init = function()
      require("plugins.codecompanion.fidget-spinner"):init()
    end,
  },
}
