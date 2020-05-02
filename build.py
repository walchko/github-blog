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

devnull = open(os.devnull, 'w')

SKIP_FOLDERS = ['old', 'do_not_backup', 'deleteme', 'large_dataset', 'draft']


def run(cmd):
    # given a command string, it runs it
    cmds = cmd.split()
    return check_output(cmds)


def mkdir(path):
    try:
        os.mkdir(path)
    except FileExistsError:
        # folder was already created ... it's ok
        pass


def rmdir(path):
    try:
        shutil.rmtree(path)
    except FileNotFoundError:
        # folder was already deleted or doesn't exist ... it's ok
        pass


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
    html = template.render(info=html.decode('utf8'), path=to_main)
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
        html = template.render(info=html.decode('utf8'), path=to_main)
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


def build_toc(template):
    toc = {}

    # def getDir(path):
    #     """
    #     Get and return a list of directories in this path
    #
    #     redundant with below?
    #     """
    #     files = glob(path)
    #     ret = []
    #     for f in files:
    #         if os.path.isdir(f):
    #             ret.append(f)
    #     if len(ret) == 0:
    #         ret = None
    #     return ret

    def getDirFile(path):
        """
        Get and return a list of files and directories in this path
        """
        objs = glob(path)
        objs.sort()
        dirs = []
        files = []
        for o in objs:
            if os.path.isdir(o):
                # don't save these folders
                if o.find('pics') >= 0 or o.find('static') >= 0:
                    pass
                else:
                    dirs.append(o)
            elif os.path.isfile(o):
                files.append(o)
            else:
                print(f"{Fore.RED}*** Unknown: {o} ***{Fore.RESET}")

        # if len(dirs) == 0:
        #     dirs = None
        #
        # if len(files) == 0:
        #     files = None

        return dirs, files

    blog, _ = getDirFile('blog/*')
    for b in blog:
        dirs, files = getDirFile(b + '/*')
        # print(dirs, files)
        jup = []
        # get folders with jupyter notebooks in them
        if dirs:
            for d in dirs:
                for ext in ['ipynb', 'md', 'rst']:
                    jj = glob(d + '/*.{}'.format(ext))
                    for j in jj:
                        jup.append(j)

                # jj = glob(d + '/*.md')
                # for j in jj:
                #     jup.append(j)
                #
                # jj = glob(d + '/*.rst')
                # for j in jj:
                #     jup.append(j)
        tmp = files + jup
        # print('tmp\n', tmp)
        key = b.replace('blog/', '')
        key = key.replace('-', ' ').replace('_', ' ')
        key = key[0].upper() + key[1:]
        # print("DEBUG: key ", key)
        toc[key] = tmp
        # get folders with markdown in them
        # jup = []
        # if dirs:
        #     for d in dirs:
        #         jj = glob(d + '/*.md')
        #         for j in jj:
        #             jup.append(j)
        # toc[b.replace('blog/', '')] = files + jup

    for key in toc.keys():
        links = toc[key]
        value = []
        for link in links:
            # print(link)
            pretty = link.split('/')[-1]
            pretty = pretty.replace('.md', '').replace('.rst', '').replace('.ipynb', '')
            pretty = pretty.replace('-', ' ').replace('_', ' ')
            pretty = pretty.title()  # capitalize first letter each word
            # print("DEBUG: pretty ", pretty)

            link = link.replace('.md', '').replace('.rst', '').replace('.ipynb', '')
            # print("DEBUG: link ", link)

            value.append((link, pretty))
        toc[key] = value

    mkdir('../html/blog')

    # pprint(toc)
    # print('='*50)

    # put the toc in alphabetical order so you can find things easier
    toc = OrderedDict(sorted(toc.items()))

    # pprint(toc)
    # exit(0)

    html = template.render(TOC=toc, path='.')
    with open('../html/topics.html', 'w') as fd:
        fd.write(html)
    print(f"{Fore.CYAN}>> Made topics.html{Fore.RESET}")
    # exit(0)


def main():
    # delete the old website so we don't miss anything when building
    print('Cleaning out old html ------------------')
    rmdir('html')
    mkdir('html')

    # change into source and recursively build website
    os.chdir("source")

    files = glob("*")

    # don't try to build html from the template, we use it another way!
    files.remove('template.jinja2')
    template = Environment(loader=FileSystemLoader('.'), trim_blocks=True).get_template('template.jinja2')

    # build toc
    build_toc(template)

    files = glob("*")

    # for each file/directory in sourece build it into pdf or copy it
    for f in files:
        pandoc(f, '../html', template, 'html')

    # make a pdf of the main page
    # pandoc('index.md', '../html', format='pdf')

    # done
    os.chdir('..')


if __name__ == "__main__":
    # this is a problem on macOS, where these stupid os files get found
    os.system("find . -type f -name '.DS_Store' -exec rm {} +")
    os.system("find . -type f -name 'deleteme' -exec rm {} +")
    main()
