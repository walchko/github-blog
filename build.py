#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from subprocess import check_output  # call command line
# from subprocess import Popen, PIPE
import os              # make directories, change current dir, etc
import platform        # macOS or windows
import shutil          # move and delete files/folders
from glob import glob  # get contents of folder
from jinja2 import Environment, FileSystemLoader  # html templating
from collections import OrderedDict  # put toc in alphebetical order
# from pprint import pprint
from colorama import Fore,Back
import pathlib
from pprint import pprint
# import markdown
import pygments
from nbconvert import HTMLExporter
import nbformat
from slurm.files import rm, mkdir, find
from string import printable  # filter hiden unicode

devnull = open(os.devnull, 'w')
root = path = str(pathlib.Path().absolute())

SKIP_EXT = [
    ".bag", ".h5", ".pickle", ".gz", ".heic", ".old", ".old2",
    ".caffemodel", ".pnm", ".ps", ".html"
]

SKIP_FOLDERS = ['old', 'do_not_backup', 'deleteme',
    'large_dataset', 'draft', "__pycache__", ".ipynb_checkpoints"]


class Jupyter:
    def __init__(self, template):
        self.exporter = HTMLExporter()
        self.exporter.template_name = 'base'
        self.exporter.theme = "light"
        self.anchor_link_text = ""
        self.template = template

    def get_template_names(self):
        return self.exporter.get_template_names()

    def defaults(self):
        pprint(html_exporter.trait_values())

    def to_html(self, dest, file, to_main):
        """
        handle a jupyter notebook
        """
        (html, resources) = self.exporter.from_filename(file)
        # pprint(resources)
        html = self.template.render(info=html, path=to_main)

        fname, ext = os.path.splitext(file)
        htmlfile = f'{fname}.html'
        with open(f"{dest}/{fname}.html", 'w') as fd:
            fd.write(html)

        print(f"{Fore.GREEN}>> Jupyter: {htmlfile}{Fore.RESET}")


class Markdown:
    def __init__(self, template):
        self.template = template

    def filter_unicode(self, md):
        """
        Sometimes I have (historically) been bitten by invisible (unprintable)
        unicode sneaking into my work causing issues. This is far less common
        today, but just in case, this filters it out.
        """
        return ''.join(char for char in md if char in printable)

    def extractOneTag(self, text, tag):
        """
        Given a tag, this will return what is inside of it.
        """
        return text[text.find("<"+tag+">") + len("<"+tag+">"):text.find("</"+tag+">")]

    def to_html(self, dest, file, to_main):
        """
        Generate the html and save to disk
        """
        # yaml: sets up date/title ... not sure it is worth it
        # footnotes: something[^1] ... [^1]: http://somewhere.com
        # emoji: why not?
        EXTENTIONS = "markdown\
        +yaml_metadata_block\
        +footnotes\
        +emoji".replace(" ","")

        TEMPLATE = f"{root}/source/template.markdown.pandoc"

        # generate the body of the html
        html = run(f"pandoc -f {EXTENTIONS} --template {TEMPLATE} -t html5 {file}")
        html = self.template.render(info=html, path=to_main)

        fname, ext = os.path.splitext(file)

        with open(f"{dest}/{fname}.html", 'w') as fd:
            fd.write(html)
        print(f"{Fore.MAGENTA}>> Markdown: {fname}.html{Fore.RESET}")


def run(cmd):
    # given a command string, it runs it
    cmd = cmd.split()
    return check_output(cmd).decode("utf8")


def rmdir(path):
    if not isinstance(path, list):
        path = [path]
    for p in path:
        try:
            shutil.rmtree(p)
            # print(p)
        except FileNotFoundError:
            # folder was already deleted or doesn't exist ... it's ok
            pass
    # exit(0)


def pandoc(file, dest, template=None, format='html', to_main='.'):
    """
    dest - where html blog goes
    template - html template
    format - doing html
    to_main - get back to source directory. need to keep track of folder
    stucture for navigation bar across the top of the page
    """
    # handle files
    if os.path.isfile(file):
        try:
            f, ext = os.path.splitext(file)
        except Exception:
            print(f'{Fore.RED}*** this file has bad name: {file} ***{Fore.RESET}')
            exit(1)

        ext = ext.lower()

        if ext in ['.md']:
            # convert markdown to html
            mark.to_html(dest, file, to_main)

        elif ext == '.ipynb':
            # generate jupyter to html
            jup.to_html(dest, file, to_main)

        elif ext in SKIP_EXT:
            # print(f"{Fore.RED}*** {file}: won't copy to website ***{Fore.RESET}")
            pass

        else:
            path = dest + '/' + file
            # print(f'{Fore.CYAN}>> Copying file {file}{Fore.RESET}')
            shutil.copy(file, path)

    # let's handle directories
    elif os.path.isdir(file):
        if file.lower() in SKIP_FOLDERS:
            print(f'{Fore.YELLOW}>> Skipping folder {file}{Fore.RESET}')
            return

        # this must be a directory, let's change into it
        if to_main == "./..":
            print(f'==[{file:15}] ===============================')

        # make the destination path have the same folder
        path = dest + '/' + file
        mkdir(path)

        # change into it, get the files and recursively call pandoc
        os.chdir(file)
        files = glob('*')
        for f in files:
            pandoc(f, '../' + dest + '/' + file, template, format, to_main=to_main + '/..')
        os.chdir('../')
    else:
        print('***********************************')
        print(f'*** Unknown File Type: {file}')
        print('***********************************')
        # raise Exception()


def build_pages(template):
    # delete the old website so we don't miss anything when building
    print('Cleaning out old html ------------------')
    rmdir('html')
    mkdir('html')

    # change into source and recursively build website
    os.chdir("source")

    # grab files and folders
    files = glob("*")
    files.sort()

    # don't try to build html from the template, we use it another way!
    files.remove('template.jinja2')

    # for each file/directory in sourece build it into pdf or copy it
    for f in files:
        pandoc(f, '../html', template, 'html')

    # done
    os.chdir('..')

def getSubDir(path):
    # print(f"{Fore.CYAN}>>   {path}{Fore.RESET}")
    files = {}
    # os.chdir(path)
    fs = find(path,"*.html")
    for f in fs:
        bname = os.path.basename(f)
        name, ext = os.path.splitext(bname)
        name = name.replace('-', ' ').replace('_', ' ').title()
        # name = name.replace('-', ' ').replace('_', ' ')
        # print(name,f)
        files[name] = str(f).split("html/")[1]
    # return files
    return OrderedDict(sorted(files.items()))


def getDir(path):
    """
    Get and return a list of files and directories in this path
    """
    # print(f"{Fore.GREEN}>> {path}{Fore.RESET}")
    objs = glob(path)
    objs.sort()
    dirs = []
    for o in objs:
        if os.path.isdir(o):
            # don't save these folders
            if o.find('pics') >= 0 or o.find('static') >= 0:
                pass
            else:
                dirs.append(o)
        # elif os.path.isfile(o):
        #     files.append(o)
        # else:
        #     print(f"{Fore.RED}*** Unknown: {o} ***{Fore.RESET}")
    return dirs

def build_toc2(path, template):

    toc = {}

    dirs = getDir(path + "/*")
    # print(dirs)

    for d in dirs:
        dd = os.path.basename(d).replace('-', ' ').replace('_', ' ').title()
        # dd = os.path.basename(d).replace('-', ' ').replace('_', ' ')
        toc[dd] = getSubDir(d)

    toc = OrderedDict(sorted(toc.items()))

    html = template.render(TOC=toc, path='.')
    with open('html/topics.html', 'w') as fd:
        fd.write(html)
    print(f"{Fore.CYAN}>> Made topics.html{Fore.RESET}")
    # pprint(toc)

jup = None
mark = None

if __name__ == "__main__":
    # clean up the input
    rm(find("./",".DS_Store"))
    rm(find("./","deleteme"))
    rmdir(find("./",".ipynb_checkpoints"))
    rmdir(find("./","__pycache__"))

    fs = FileSystemLoader('./source')
    template = Environment(loader=fs, trim_blocks=True).get_template('template.jinja2')

    jup = Jupyter(template)
    mark = Markdown(template)

    build_pages(template)
    build_toc2("html/blog",template)
