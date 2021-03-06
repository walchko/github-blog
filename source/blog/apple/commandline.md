---
title: macOS Command Line
date: 2016-06-25
---

My `.bash_profile`:

```bash
export PATH=/usr/local/bin:/usr/local/sbin:/usr/bin:/usr/sbin:/bin:/sbin
export PATH="/usr/local/opt/python/libexec/bin:$PATH"
export PATH=$PATH:/Library/TeX/texbin
export EDITOR=`command -v pico`


USR="\[\033[01;32m\]"
HOST="\[\033[01;35m\]"
DIR="\[\033[01;34m\]"
END="\[\033[0m\]"

export PS1="$USR\u@$HOST\h $DIR\W$END \$ "

# keep sensative info in dropbox
TOKEN_FILE="/Users/${USER}/Dropbox/Share/kevin/tokens.sh"
if [ -r ${TOKEN_FILE} ]; then
    source ${TOKEN_FILE}
fi

alias df='df -h '
alias ls='ls -aGph'
alias cd..='cd ..'  # fix typing error
alias gitstatus='git remote update && git status'
alias sshraspberrypi="ssh -o UserKnownHostsFile=/dev/null pi@raspberrypi.local"
alias github-check="ssh-keygen -l -E md5 -f ~/.ssh/id_rsa.pub"

#   extract:  Extract most know archives with one command
#   ---------------------------------------------------------
extract () {
    if [ -f $1 ] ; then
      case $1 in
        *.tar.bz2)   tar xjf $1     ;;
        *.tar.gz)    tar xzf $1     ;;
        *.bz2)       bunzip2 $1     ;;
        *.rar)       unrar e $1     ;;
        *.gz)        gunzip $1      ;;
        *.tar)       tar xf $1      ;;
        *.tbz2)      tar xjf $1     ;;
        *.tgz)       tar xzf $1     ;;
        *.zip)       unzip $1       ;;
        *.Z)         uncompress $1  ;;
        *.7z)        7z x $1        ;;
        *)     echo "'$1' cannot be extracted via extract()" ;;
         esac
     else
         echo "'$1' is not a valid file"
     fi
}

# fully update pip
# -----------------------------------------------
pip-upgrade-all() {
    pip list --outdated | cut -d' ' -f1 | xargs pip install --upgrade
}

pip3-upgrade-all() {
    pip3 list --outdated | cut -d' ' -f1 | xargs pip3 install --upgrade
}
```
