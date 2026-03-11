# oh-my-opencode profile switcher
# Source this file in your .bashrc:
#   source ~/.config/opencode/profiles/omo-profile.sh

_OMO_CONFIG_DIR="$HOME/.config/opencode"
_OMO_PROFILES_DIR="$_OMO_CONFIG_DIR/profiles"
_OMO_CONFIG_FILE="$_OMO_CONFIG_DIR/oh-my-opencode.json"

omo-profile() {
  local cmd="${1:-}"

  case "$cmd" in
    ls|list)
      echo "Available profiles:"
      for f in "$_OMO_PROFILES_DIR"/*.json; do
        [ -f "$f" ] || continue
        local name
        name="$(basename "$f" .json)"
        if [ -L "$_OMO_CONFIG_FILE" ] && \
           [ "$(readlink -f "$_OMO_CONFIG_FILE")" = "$(readlink -f "$f")" ]; then
          echo "  * $name  (active)"
        else
          echo "    $name"
        fi
      done
      ;;

    use|switch)
      local profile="${2:-}"
      if [ -z "$profile" ]; then
        echo "Usage: omo-profile use <profile-name>"
        echo "Run 'omo-profile ls' to see available profiles."
        return 1
      fi
      local target="$_OMO_PROFILES_DIR/${profile}.json"
      if [ ! -f "$target" ]; then
        echo "Error: profile '$profile' not found at $target"
        echo "Run 'omo-profile ls' to see available profiles."
        return 1
      fi
      # Back up if current config is a regular file (not symlink)
      if [ -f "$_OMO_CONFIG_FILE" ] && [ ! -L "$_OMO_CONFIG_FILE" ]; then
        echo "Backing up current config to profiles/backup-$(date +%Y%m%d%H%M%S).json"
        cp "$_OMO_CONFIG_FILE" "$_OMO_PROFILES_DIR/backup-$(date +%Y%m%d%H%M%S).json"
      fi
      ln -sf "$target" "$_OMO_CONFIG_FILE"
      echo "Switched to profile: $profile"
      ;;

    current)
      if [ -L "$_OMO_CONFIG_FILE" ]; then
        local target
        target="$(readlink -f "$_OMO_CONFIG_FILE")"
        echo "$(basename "$target" .json)"
      elif [ -f "$_OMO_CONFIG_FILE" ]; then
        echo "(custom — not linked to any profile)"
      else
        echo "(no config file found)"
      fi
      ;;

    diff)
      local profile="${2:-}"
      if [ -z "$profile" ]; then
        echo "Usage: omo-profile diff <profile-name>"
        return 1
      fi
      local target="$_OMO_PROFILES_DIR/${profile}.json"
      if [ ! -f "$target" ]; then
        echo "Error: profile '$profile' not found"
        return 1
      fi
      diff --color=auto <(python3 -m json.tool "$_OMO_CONFIG_FILE" 2>/dev/null || cat "$_OMO_CONFIG_FILE") \
                        <(python3 -m json.tool "$target" 2>/dev/null || cat "$target")
      ;;

    *)
      echo "omo-profile — oh-my-opencode profile switcher"
      echo ""
      echo "Commands:"
      echo "  ls, list        Show available profiles"
      echo "  use <name>      Switch to a profile"
      echo "  current         Show active profile"
      echo "  diff <name>     Compare current config with a profile"
      echo ""
      echo "Profiles directory: $_OMO_PROFILES_DIR"
      ;;
  esac
}

# Bash completion
_omo_profile_completions() {
  local cur="${COMP_WORDS[COMP_CWORD]}"
  local prev="${COMP_WORDS[COMP_CWORD-1]}"

  case "$prev" in
    omo-profile)
      COMPREPLY=($(compgen -W "ls list use switch current diff" -- "$cur"))
      ;;
    use|switch|diff)
      local profiles=""
      for f in "$_OMO_PROFILES_DIR"/*.json; do
        [ -f "$f" ] || continue
        profiles="$profiles $(basename "$f" .json)"
      done
      COMPREPLY=($(compgen -W "$profiles" -- "$cur"))
      ;;
  esac
}
complete -F _omo_profile_completions omo-profile
