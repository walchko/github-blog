# My Blog

- `build.js` builds the website
- `deploy_github.sh` performs the process of copying the website to `gh-pages` and publishes the updated website.
- `deploy_surge.sh` performs a deployment to surge.sh. **Warning:** having issue with deploying pdf files.

## Setup Github

You will probably have to create a `gh-pages` branch. Then double check in the setting for the repo, that source is the `gh-pages branch`.

## Travis-CI

Grant access to your github repo so travis can clone and build it. You will also need to grant access for travis to write back to the repo under the `gh-pages` branch.

### Grant Push Access

- Generate a [Personal Access Token](https://help.github.com/articles/creating-a-personal-access-token-for-the-command-line/) on github, make sure to select **public_repo**.
- Add it to your travis account settings by setting [environment variable](https://docs.travis-ci.com/user/environment-variables#Defining-Variables-in-Repository-Settings) called GITHUB_TOKEN with a value of the personal access token (lots of numbers)

# Build Locally

First you need to install:

1. gitbash
1. node
1. pandoc
1. basictex (if we have latex equations or you are building pdfs)
1. jupyter notebooks (if you are using them)

## Build

1. Open a terminal window and navigate to the repo
1. run: `npm install`
    1. This installs ejs, you only have to do this once or again to update the library
1. run: `build.js`
1. Look in the `html` folder and the website should be there

During the build process, you should see:

```bash
kevin@Dalek ece382 $ node build.js
-------------------------------------------
Searching: source
-------------------------------------------
Searching: source/admin
Wrote: html/admin/c_style_guide.html
Wrote: html/admin/course_goals.html
Wrote: html/admin/course_letter.html
-------------------------------------------
Searching: source/admin/final_resources
Copied: html/admin/final_resources/ADC10AE0.jpg
Copied: html/admin/final_resources/ADC10CTL0_1.jpg
Copied: html/admin/final_resources/ADC10CTL0_2.jpg
Copied: html/admin/final_resources/ADC10CTL0_3.jpg
Copied: html/admin/final_resources/ADC10CTL1_1.jpg
Copied: html/admin/final_resources/ADC10CTL1_2.jpg
...
```


# [Surge](http://surge.sh/) Deployment

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
