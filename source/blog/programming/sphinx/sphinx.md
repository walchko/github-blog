---
title: Documenting Code with Sphinx
date: 7 Mar 2021
---

```
pip install Sphinx m2r2
```

```
sphinx-quickstart
```

```
docs/
 |
 +- config.py
 +- index.rst
 +- _build/
 +- _templates/
 +- _static/
      |
      +- logo.jpg
```

```
make clean && sphinx-build -E  -b html . _build/html
```

## index.rst

```reStructuredText
.. mdinclude:: ../readme.md

.. toctree::
   :hidden:
   :glob:

   *
 ```
 
 ## config.py
 
```python
# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------
# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
import os
import sys
sys.path.insert(0, os.path.abspath('..'))


# -- Project information -----------------------------------------------------

project = 'slurm'
copyright = '2014, Kevin Walchko'
author = 'Kevin J. Walchko'

# The full version, including alpha/beta/rc tags
# from importlib.metadata import version
# release = version("slurm")

source_suffix = ['.rst', '.md']
# default_role = "any"
# The master toctree document.
master_doc = "index"

# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    # "recommonmark",
    "autoapi.extension",
    "sphinx.ext.autodoc",
    "sphinx.ext.intersphinx",
    "sphinx.ext.viewcode",
    "sphinx.ext.napoleon",
    # "sphinx.ext.todo",
    'sphinx.ext.mathjax',
    "m2r2",
]

autoapi_dirs = ['../slurm']
autoapi_type = "python"
# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']
pygments_style = 'sphinx'

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = [
    '_build',
    'Thumbs.db',
    '.DS_Store'
]


# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = 'alabaster'
html_theme_options = {
    "description": "this is my description",
    "fixed_sidebar": True,
    'logo': 'logo.jpg',
    'github_user': 'walchko',
    'github_repo': 'slurm',
    "github_button": True,
    # "sidebar_width": "200px",
}

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['_static']
```
