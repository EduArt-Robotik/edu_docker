FROM eduartrobotik/eduart-robot:0.3.0

WORKDIR /home/user

RUN echo "Installing Node Red" \
    cd /home/user \
    && wget https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.1/install.sh \
    && chmod u+x install.sh \
    && ./install.sh \
    && export NVM_DIR="$HOME/.nvm" \
    && [ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh" [ -s "$NVM_DIR/bash_completion" ] && \. "$NVM_DIR/bash_completion" \
    && nvm install node \
    && npm install -g --unsafe-perm node-red

CMD node-red
