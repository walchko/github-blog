---
title: Github Python Workflow
date: 27 Dec 2020
image: "https://i.pinimg.com/564x/14/09/fb/1409fbc18d377df6ff41d56cc230ff58.jpg"
image-height: "300px"
---

This is what works for me:

- I don't keep `poetry.lock` files in my repo, so I use my `pyproject.toml` as the key
- *Always* make sure to `source .venv/bin/activate` to setup your environment before calling python commands

```yaml
name: CheckPackage
on: [push]

jobs:
    build:
        runs-on: ubuntu-latest
        strategy:
          max-parallel: 5
          fail-fast: true
          matrix:
            python-version: [3.6, 3.7, 3.8, 3.9]
        steps:
            - uses: actions/checkout@master
            - name: Setup Python ${{ matrix.python-version }}
              uses: actions/setup-python@v2
              with:
                python-version: ${{ matrix.python-version }}
            - name: Install Poetry
              uses: snok/install-poetry@v1.1.1
              with:
                virtualenvs-create: true
                virtualenvs-in-project: true
              #----------------------------------------------
              #       load cached venv if cache exists
              #----------------------------------------------
            - name: Load cached venv
              id: cached-poetry-dependencies
              uses: actions/cache@master
              with:
                path: .venv
                key: venv-${{ runner.os }}-${{ hashFiles('pyproject.toml') }}
            - name: Install packages
              run: |
                echo "Installing dependencies"
                poetry install
                source .venv/bin/activate
                pip install -U pip setuptools wheel
              if: steps.cached-poetry-dependencies.outputs.cache-hit != 'true'
            - name: Show Module Info
              run: |
                source .venv/bin/activate
                pip show opencv_camera
            - name: Run PyTest
              run: |
                source .venv/bin/activate
                pytest tests/

```
