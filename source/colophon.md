![](pics/ship.jpg)

This is the Repo that feeds my personal github page. This has the ability to
build a website from a combination of:

- [reStructuredText](http://docutils.sourceforge.net/rst.html)
- [Pandoc flavored markdown](http://pandoc.org/MANUAL.html#pandocs-markdown)
- [Jupyter Notebooks](https://jupyter.org/)
- The build tool is based on Python 3

I use to use Pelican, but that had issues with latex equations and took months
for them to fix it (sort of). Basically Pelican is a mass of crap that
limps along for people who cannot program. Also, I never founds a nice way to
include jupyter notebook content with it.

## Setup

You need:

```bash
brew install pandoc python
brew cask install basictex
pip install jinja2
```

## Build

The build script works as follows:

- `build.py`: builds the website using `pandoc` and `jupyter-nbconvert`

```bash
kevin@logan github-blog $ ./build.py
Cleaning out old html ------------------
>> Made topics.html
>> Copying file template.jinja2
==[Publications   ] ===============================
>> Copying file mimo-fuzzy-FCRAR.pdf
>> Copying file walchko-MS-EE.pdf
>> Copying file subjugator-FCRAR.pdf
>> Copying file Optimal-geo-lasercom.pdf
>> Copying file AIAA-Reconfigurable-AUV.pdf
>> Copying file walchko-nav-FCRAR.pdf
>> Copying file walchko-INS-FCRAR-2003.pdf
>> Copying file walchko-SM-FCRAR-2003.pdf
>> Copying file AUVSI-2001.pdf
>> Copying file AUVSI-2002.pdf
>> Copying file walchko-MS-ME.pdf
>> Copying file walchko-GSRP.pdf
>> Copying file embeded_linux_CF.pdf
>> Copying file walchko-PhD-ME.pdf
>> Copying file subjugator-sinkin-is-easy.pdf
>> Skipping folder old
==[pics           ] ===============================
>> Copying file 153d_CAV.png
>> Copying file space.png
>> Copying file ship.jpg
>> Copying file IJC.jpg
>> Copying file air_assault2.jpg
>> Copying file rack.png
>> Copying file ISAF.png
>> Copying file 53rd_INF.png
>> Copying file rocket.png
>> Copying file grim.png
>> Copying file Farnsworth.png
...
```

Now, you also need to do `pip install jupyter numpy matplotlib` if you are going to convert
jupyter notebooks into html.

## Pygments

To setup the `css` for code, I do:

```bash
kevin@Logan $ pygmentize -S default -f html > pygments.css
```
