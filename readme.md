![](pics/ship.jpg)

This is the Repo that feeds my personal github page. This has the ability to
build a website from a combination of:

- [reStructuredText](http://docutils.sourceforge.net/rst.html)
- [Markdown](http://pandoc.org/MANUAL.html#pandocs-markdown)
- [Jupyter Notebooks](https://jupyter.org/)

I use to use Pelican, but that had issues with latex equations and took months
for them to fix. Also, I never founds a nice way to do jupyter with it.

# Setup

First you need to install:

1. `brew install pandoc node python matplotlib`
1. `npm  install ejs`
1. `pip install -U jupyter matplotlib numpy`

# Build

1. Open a terminal window and navigate to the repo
1. run: `./build.js`
1. Look in the `html` folder and the website should be there

During the build process, you should see:

```bash
kevin@Dalek github-blog $ ./deploy_github.sh
Deploying to github
 * Wrote colophon: html/colophon.html
-------------------------------------------
Searching: source
-------------------------------------------
Searching: source/Publications
 * Copied: html/Publications/AIAA-Reconfigurable-AUV.pdf
 * Copied: html/Publications/AUVSI-2001.pdf
 * Copied: html/Publications/AUVSI-2002.pdf
 * Copied: html/Publications/Optimal-geo-lasercom.pdf
 * Copied: html/Publications/embeded_linux_CF.pdf
 * Copied: html/Publications/mimo-fuzzy-FCRAR.pdf
 * Copied: html/Publications/subjugator-FCRAR.pdf
 * Copied: html/Publications/subjugator-sinkin-is-easy.pdf
 * Copied: html/Publications/walchko-GSRP.pdf
 * Copied: html/Publications/walchko-INS-FCRAR-2003.pdf
 * Copied: html/Publications/walchko-MS-EE.pdf
 * Copied: html/Publications/walchko-MS-ME.pdf
 * Copied: html/Publications/walchko-PhD-ME.pdf
 * Copied: html/Publications/walchko-SM-FCRAR-2003.pdf
 * Copied: html/Publications/walchko-nav-FCRAR.pdf
 * Wrote: html/about.html
-------------------------------------------
Searching: source/blog
-------------------------------------------
Searching: source/blog/Arch_Linux
 * Wrote: html/blog/Arch_Linux/arch.html
-------------------------------------------
Searching: source/blog/Arch_Linux/pics
 * Copied: html/blog/Arch_Linux/pics/arch_linux.png
-------------------------------------------
Searching: source/blog/Computer_Vision
 >> skip: .ipynb_checkpoints
 * Wrote: html/blog/Computer_Vision/jupyter/Feature_Detection/Feature_Detection.html
 * Copied: html/blog/Computer_Vision/jupyter/Feature_Detection/afa.jpg
 * Copied: html/blog/Computer_Vision/jupyter/Feature_Detection/balls.jpg
 * Copied: html/blog/Computer_Vision/jupyter/Feature_Detection/calibration.jpg
 * Copied: html/blog/Computer_Vision/jupyter/Feature_Detection/coins.jpg
 * Copied: html/blog/Computer_Vision/jupyter/Feature_Detection/dnd.jpg
...
```
# Deployment

There are a couple scripts that allow you to deploy to multiple locations.

- `build.js` builds the website. If no args are given, then it builds it locally to `html` folder.
- `deploy_github.sh` performs a deployment to the `master` branch of a github pages repo. Github forces you to use the `master` branch on a personal website.
- `deploy_surge.sh` performs a deployment to surge.sh.

## [Surge](http://surge.sh/) Deployment

**Warning:** having issue with deploying pdf files.

Install surge: `npm install --global surge`

```bash
kevin@dalek html $ surge

    Welcome to Surge! (surge.sh)
    Please login or create an account by entering your email and password:

              email:
           password:
       project path: /Users/kevin/github/github-blog/html/
               size: 210 files, 36.0 MB
             domain: walchko.surge.sh
             upload: [====================] 100%, eta: 0.0s
   propagate on CDN: [====================] 100%
               plan: Free
              users: kevin.walchko@gmail.com
         IP Address: 45.55.110.124

    Success! Project is published and running at walchko.surge.sh
```

# ToDo

- jupyter headings have an annoyting &#182; which is the paragraph symbol ... I want to remove it
- move more code to gists so it displays nicely
- make it easier to swap out templates
