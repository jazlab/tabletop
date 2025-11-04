FROM tabletop/ros-base

# Install apt packages
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked <<EOT
set -e
sudo apt-get update && sudo apt-get upgrade -y
sudo apt-get install -y \
    git \
    git-lfs \
    gdb \
    bash-completion \
    vim \
    psmisc \
    iputils-ping \
    net-tools \
    iproute2 \
    usbutils \
    ripgrep \
    xarclock
EOT

# Install npm
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    <<EOT
set -e
sudo apt-get update && sudo apt-get upgrade -y
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.3/install.sh | bash
\. "$HOME/.nvm/nvm.sh"
nvm install 22
EOT

ARG TARGETARCH

# Install Neovim
RUN <<EOT
set -e
case $TARGETARCH in
    amd64)
        curl -fsSL -o /tmp/nvim.tar.gz https://github.com/neovim/neovim/releases/download/stable/nvim-linux-x86_64.tar.gz
        ;;
    arm64)
        curl -fsSL -o /tmp/nvim.tar.gz https://github.com/neovim/neovim/releases/download/stable/nvim-linux-arm64.tar.gz
        ;;
    *)
        echo "Unsupported architecture $TARGETARCH!"
        exit 1
        ;;
esac
sudo tar -xzf /tmp/nvim.tar.gz -C /opt
NVIM_DIRNAME=$(tar -tf /tmp/nvim.tar.gz | head -1 | cut -f1 -d"/")
sudo ln -s /opt/$NVIM_DIRNAME/bin/nvim /usr/bin/nvim
rm -f /tmp/nvim.tar.gz
EOT

# Install complete-alias
RUN <<EOT
set -e
curl -fsSL -o ~/complete-alias \
https://raw.githubusercontent.com/cykerway/complete-alias/refs/heads/master/complete_alias
chmod +x ~/complete-alias
echo ". ~/complete-alias" >> ~/.bash_completion
EOT

# Install kitten
RUN <<EOT
set -e
case $TARGETARCH in
    amd64)
        sudo curl -fsSL -o /usr/local/bin/kitten https://github.com/kovidgoyal/kitty/releases/latest/download/kitten-linux-amd64
        ;;
    arm64)
        sudo curl -fsSL -o /usr/local/bin/kitten https://github.com/kovidgoyal/kitty/releases/latest/download/kitten-linux-arm64
        ;;
    *)
        echo "Unsupported architecture $TARGETARCH!"
        exit 1
        ;;
esac
sudo chmod +x /usr/local/bin/kitten
EOT

# Install starship
RUN curl -sS https://starship.rs/install.sh | sudo sh -s -- -y

# Update .bashrc and .inputrc
RUN <<EOT
set -e
cat <<EOF >> ~/.bashrc
eval "\$(uv generate-shell-completion bash)"
eval "\$(uvx --generate-shell-completion bash)"
alias vim="nvim"
export EDITOR=nvim
set -o vi
complete -F _complete_alias "\${!BASH_ALIASES[@]}"
eval "\$(starship init bash)"
EOF
cat <<EOF >> ~/.inputrc
set editing-mode vi
set vi-ins-mode-string \1\e[5 q\2
set vi-cmd-mode-string \1\e[2 q\2
set show-mode-in-prompt on
EOT
