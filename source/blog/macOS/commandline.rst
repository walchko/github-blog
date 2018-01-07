macOS Command Line
===================

:date: 2016-06-25
:summary: Setting up your command line on macOS

So I setup the following using `homebrew`:

* Bash 4
* `bash-git-prompt`
* `bash-completion`

Bash 4
-------

::

	brew install bash
	sudo bash -c "echo /usr/local/bin/bash >> /etc/shells"
	chsh -s /usr/local/bin/bash

	# test if it works
	echo $BASH && echo $BASH_VERSION

Git Prompt
-----------

To setup git prompt for bash, have a look at my `dotfiles <https://github.com/walchko/dotfiles>`_
and take a look at ``.git-prompt-colors.sh`` to see how I set it up. There are
lots of themes and defaults to play with.

::

	override_git_prompt_colors() {
	  GIT_PROMPT_THEME_NAME="Custom" # needed for reload optimization, should be unique

	  GIT_PROMPT_START_USER="${Green}\u@${BoldBlue}\\h ${Cyan}\W"
	  GIT_PROMPT_END_USER="${ResetColor} $ "
	  GIT_PROMPT_END_ROOT="${BoldRed} # "
	}

	# load the theme
	reload_git_prompt_colors "Custom"

Bash Completion
------------------

Ditto with bash completion, look at my dotfiles to show one way to do it::

	if [ -f $(brew --prefix)/etc/bash_completion ]; then
	    . $(brew --prefix)/etc/bash_completion
	fi
