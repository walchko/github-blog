# This Website

## Construction

This site is created using [node](https://nodejs.org/en/) and
[EJS](http://www.ejs.co/). The build script works as follows:

- `build.js`: builds the website using `pandoc`. It takes the following args:
    - github: to setup for github.io
	- surge: to setup for surge.sh
	- none: for just testing locally

## Setup

You need:

	brew install pandoc node
	brew cask install basictex

## Pygments

To setup the `css` for code, I do:

```bash
kevin@Logan pelican $ pygmentize -S default -f html > pygments.css
kevin@Logan pelican $ mv pygments.css themes/kevin/static/css/
kevin@Logan pelican $ make html
```
