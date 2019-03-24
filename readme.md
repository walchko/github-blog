![](pics/ship.jpg)

This is the Repo that feeds my personal github page. For details on how it
work, see [this page](https://github.com/walchko/github-blog/blob/master/source/colophon.md)
which describes the setup/usage.

# Deployment

I played around also with surge, but they are a pain-in-the-ass to get
PDFs approved for distribution on their site, so I gave up. These
instructions are just for posterity because I don't use them.

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

