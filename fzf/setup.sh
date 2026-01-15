#!/bin/bash

# Color codes
RED='\033[1;31m'
GREEN='\033[1;32m'
ORANGE='\033[1;33m'
BLUE='\033[1;34m'
NC='\033[0m'

echo -e "${BLUE}>> ${ORANGE}Installing fzf ... ${NC}"

# Clone fzf repository if not exists
if [ ! -d "$HOME/.fzf" ]; then
    git clone --depth 1 https://github.com/junegunn/fzf.git ~/.fzf
else
    echo -e "${BLUE}>> ${GREEN}fzf repository already exists. Updating...${NC}"
    cd ~/.fzf && git pull && cd -
fi

# Run install script
# --all: update shell configuration files (bash, zsh, fish)
# --key-bindings: enable key bindings
# --completion: enable fuzzy completion
# --no-update-rc: we will handle rc updates if needed, but --all usually handles it well
~/.fzf/install --all --key-bindings --completion --no-bash --no-zsh --no-fish

# Custom integration for ripgrep if available
if command -v rg &> /dev/null; then
    echo -e "${BLUE}>> ${GREEN}ripgrep found! Configuring fzf to use rg by default.${NC}"
    
    # Add to .bashrc if not present
    if ! grep -q "FZF_DEFAULT_COMMAND" ~/.bashrc; then
        echo '' >> ~/.bashrc
        echo '# fzf integration with ripgrep' >> ~/.bashrc
        echo 'export FZF_DEFAULT_COMMAND="rg --files --hidden --glob \"!.git/*\""' >> ~/.bashrc
        echo 'export FZF_CTRL_T_COMMAND="$FZF_DEFAULT_COMMAND"' >> ~/.bashrc
    fi
fi

echo -e "${BLUE}>> ${GREEN}fzf installation and configuration done!${NC}"
echo -e "${ORANGE}Please restart your shell or run 'source ~/.bashrc' to apply changes.${NC}"
