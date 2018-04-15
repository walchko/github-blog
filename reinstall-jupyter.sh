#!/bin/bash

# fucking idiots keep changing names and it links to a specific version of
# python, so if you upgrade python, it breaks
PKGS="jupyter notebook nbformat nbconvert tornado widgetsnbextension ipywidgets \
ipython-genutils ipython ipykernel jupyter-client jupyter-core jupyter-console"

pip2 uninstall ${PKGS}
pip3 uninstall ${PKGS}

pip2 install -U jupyter
pip3 install -U jupyter
