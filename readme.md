![](pics/ship.jpg)

This is the Repo that feeds my personal github page. For details on how it
work, see [this page](https://github.com/walchko/github-blog/blob/master/source/colophon.md)
which describes the setup/usage.

# Deployment

1. Run `./build.py`
1. Check the `html` folder looks good
1. Run `./deploy.py`

Output of `build.py` will look something like this:

```
(venv) kevin@dalek github-blog % ./build.py
Cleaning out old html ------------------
==[python         ] ===============================
>> Markdown: vitualenv.html
>> Jupyter: comparing-file-storage.html
>> Markdown: zmq.html
>> Jupyter: attrs-slots-and-frozen-performance.html
>> Markdown: base64-serialization.html
>> Jupyter: DSP-Filtering.html
>> Markdown: determine-rpi.html
>> Jupyter: sparkline.html
>> Markdown: exceptions.html
>> Markdown: Python-Networking.html
>> Markdown: mkdocs.html
>> Markdown: Using-SSDP-to-Find-Rokus-on-a-Network.html
>> Jupyter: struct-binary-strings.html
>> Markdown: command-line-args.html
>> Markdown: Animated-Matplotlib.html
>> Markdown: poetry.html
>> Markdown: Temp-Files-and-Folders.html
>> Markdown: C-Python-Integration.html
...
```
