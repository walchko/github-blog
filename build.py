#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from subprocess import check_output  # call command line
import os              # make directories, change current dir, etc
import platform        # macOS or windows
import shutil          # move and delete files/folders
from glob import glob  # get contents of folder
from jinja2 import Environment, FileSystemLoader  # html templating
from collections import OrderedDict  # put toc in alphebetical order
# from pprint import pprint
from colorama import Fore
import pathlib
from pprint import pprint

devnull = open(os.devnull, 'w')

SKIP_FOLDERS = ['old', 'do_not_backup', 'deleteme',
    'large_dataset', 'draft', "__pycache__", ".ipynb_checkpoints"]


def run(cmd):
    # given a command string, it runs it
    cmds = cmd.split()
    return check_output(cmds).decode("utf8")


def mkdir(path):
    try:
        os.mkdir(path)
    except FileExistsError:
        # folder was already created ... it's ok
        pass


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

def rm(fname):
    if fname is None:
        print(f"{Fore.RED}*** No file to remove ***{Fore.RESET}")
        return
    if not isinstance(fname, list):
        fname = [fname]
    for f in fname:
        try:
            os.remove(f)
            print(f"{Fore.RED}- {f}{Fore.RESET}")
        except FileNotFoundError:
            # folder was already deleted or doesn't exist ... it's ok
            pass

def find(path, fname):
    """Given a path, this will recursively search for a file (bob.txt) or
    pattern (*.txt). It returns an array of found file paths."""
    fn = []
    for p in pathlib.Path(path).rglob(fname):
        fn.append(p)
    return fn

# def zip_this_folder(folder, dest):
#     """
#     zip a folder
#     """
#     run(f'zip -r jupyter.zip {folder}')
#     shutil.move('jupyter.zip', dest)
#     print(f">> Zipped jupyter and moved to: {dest}")


def jupyter(f, dest, template, format, to_main, file):
    """
    handle a jupyter notebook
    """
    # zip_this_folder()
    # print("[[jupyter]] zip this path: {}".format(os.getcwd()))
    # zip_this_folder(os.getcwd(), dest)

    cmd = f"jupyter-nbconvert --to html --template full --log-level=0 --stdout {file}"
    html = run(cmd)
    html = template.render(info=html, path=to_main)
    with open(dest + '/' + f + '.html', 'w') as fd:
        fd.write(html)
    print(f"{Fore.MAGENTA}>> Made {f}.html{Fore.RESET}")


def markdown(f, dest, template, format, to_main, fmt):
    """
    handle a markdown document
     f- file
     dest - destination
     format - output: pdf or html
     to_main - path back to source folder
     fmt - ????
    """
    if format == 'pdf':
        run('pandoc -V geometry:margin=1in -s -o {}.pdf {}.{}'.format(f, f, fmt))
        shutil.move(f + '.pdf', dest + '/' + f + '.pdf')
        print(f">> Made {f}.pdf")

    elif format == 'html':
        if template is None:
            raise Exception("*** You need to pass a template to convert Markdown to HTML ***")
        # creates the html5 from markdown and sets pandoc.css to look pretty!
        if fmt == "md":
            html = run('pandoc -s -r markdown+simple_tables+table_captions+yaml_metadata_block --highlight-style=pygments -t html5 {}.{}'.format(f, fmt))
        else:
            html = run('pandoc --highlight-style=pygments -t html5 {}.{}'.format(f, fmt))
        html = template.render(info=html, path=to_main)
        with open(dest + '/' + f + '.html', 'w') as fd:
            fd.write(html)
        print(f"{Fore.MAGENTA}>> Made {f}.html{Fore.RESET}")


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
            # some files have multiple . in them and it is hard to get the ext
            # >>> '.'.join('a.b'.split('.')[:-1])
            # 'a'
            # >>> '.'.join('a.b.c'.split('.')[:-1])
            # 'a.b'
            f = '.'.join(file.split('.')[:-1])
            ext = file.split('.')[-1]
        except Exception:
            print(f'{Fore.RED}*** this file has an issue name.ext: {file} ***{Fore.RESET}')
            exit(1)

        ext = ext.lower()

        if ext in ['md', 'rst']:
            markdown(f, dest, template, format, to_main, ext)

        elif ext == 'html':
            print(f"{Fore.RED}*** {file}: left over html? You should erase it ***{Fore.RESET}")

        elif ext == 'ipynb':
            jupyter(f, dest, template, format, to_main, file)

        elif ext in ["bag", "h5", "pickle", "gz", "heic", "old", "old2", "caffemodel", "pnm", "ps"]:
            print(f"{Fore.RED}*** {file}: won't copy to website ***{Fore.RESET}")

        else:
            path = dest + '/' + file
            print(f'{Fore.CYAN}>> Copying file {file}{Fore.RESET}')
            shutil.copy(file, path)

    # let's handle directories
    elif os.path.isdir(file):
        if file.lower() in SKIP_FOLDERS:
            print(f'{Fore.YELLOW}>> Skipping folder {file}{Fore.RESET}')
            return

        # this must be a directory, let's change into it
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

    files = glob("*")

    # don't try to build html from the template, we use it another way!
    files.remove('template.jinja2')

    files = glob("*")

    # for each file/directory in sourece build it into pdf or copy it
    for f in files:
        pandoc(f, '../html', template, 'html')

    # make a pdf of the main page
    # pandoc('index.md', '../html', format='pdf')

    # done
    os.chdir('..')

def getSubDir(path):
    # print(f"{Fore.CYAN}>>   {path}{Fore.RESET}")
    files = {}
    # os.chdir(path)
    fs = find(path,"*.html")
    for f in fs:
        name = os.path.basename(f).split('.')[0]
        name = name.replace('-', ' ').replace('_', ' ').title()
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
        toc[dd] = getSubDir(d)

    toc = OrderedDict(sorted(toc.items()))

    html = template.render(TOC=toc, path='.')
    with open('html/topics.html', 'w') as fd:
        fd.write(html)
    print(f"{Fore.CYAN}>> Made topics.html{Fore.RESET}")
    # pprint(toc)

if __name__ == "__main__":
    # clean up the input
    rm(find("./",".DS_Store"))
    rm(find("./","deleteme"))
    rmdir(find("./",".ipynb_checkpoints"))
    rmdir(find("./","__pycache__"))

    template = Environment(loader=FileSystemLoader('./source'), trim_blocks=True).get_template('template.jinja2')

    build_pages(template)
    build_toc2("html/blog",template)
