

# Contributing

<!----------------------------------------------------------------------------------------------------------------------
#
#   Table of Contents
#
# --------------------------------------------------------------------------------------------------------------------->
## Table of Contents
* [Getting Started](#getting-started)
  * [Installation](#installation)
* [Python Linter](#python-linter)
  * [Bandit](#bandit)
  * [Black](#black)
  * [docformatter](#docformatter)
  * [Flake8](#flake8)
  * [isort](#isort)
  * [pydocstyle](#pydocstyle)
  * [Pyright](#pyright)
  * [yamllint](#yamllint)
* [Metrics](#metrics)
  * [Radon](#radon)
  * [Xenon](#xenon)
* [Test](#test)
  * [Pytest](#pytest)

<!----------------------------------------------------------------------------------------------------------------------
#
#   Installation
#
# --------------------------------------------------------------------------------------------------------------------->
## Getting Started
### Installation
Create and activate a virtual environments (venv).
```shell
python3 -m venv .venv
source .venv/bin/activate
```

Install required python packages after activates.
```shell
pip3 install -r requirements.txt -r requirements_test.txt 
```

<!----------------------------------------------------------------------------------------------------------------------
#
#   Python Linter
#
# --------------------------------------------------------------------------------------------------------------------->
## Python Linter

<!----------------------------------------------------------
#   Bandit
# --------------------------------------------------------->
### Bandit
Bandit is a tool designed to find common security issues in Python code.
* Official: https://github.com/PyCQA/bandit
* Configuration: [pyproject.toml](pyproject.toml)
```shell
bandit -c pyproject.toml -r ./
```

<!----------------------------------------------------------
#   Black
# --------------------------------------------------------->
### Black
Black is the uncompromising Python code formatter.
* Official: https://github.com/psf/black
* Configuration: [pyproject.toml](pyproject.toml)
```shell
black --check ./
```

<!----------------------------------------------------------
#   docformatter
# --------------------------------------------------------->
### docformatter
docformatter is an automatic formatter for docstrings to follow a subset of the PEP 257 conventions.
* Official: https://github.com/PyCQA/docformatter
* Configuration: [pyproject.toml](pyproject.toml)
```shell
docformatter --check ./
```

<!----------------------------------------------------------
#   Flake8
# --------------------------------------------------------->
### Flake8
Flake8 is a wrapper tool to check your Python code against some of the style conventions in PEP 8.
* Official: https://github.com/PyCQA/flake8
* Configuration: [setup.cfg](setup.cfg)
```shell
flake8 -v ./
```

<!----------------------------------------------------------
#   isort
# --------------------------------------------------------->
### isort
isort is a Python utility / library to sort imports alphabetically, and automatically separated into sections and by type.
* Official: https://github.com/PyCQA/isort
* Configuration: [pyproject.toml](pyproject.toml)
```shell
isort --check ./
```

<!----------------------------------------------------------
#   pydocstyle
# --------------------------------------------------------->
### pydocstyle (In flake8)
pydocstyle is a static analysis tool for checking compliance with Python docstring conventions in PEP 257.
* Official: https://gitlab.com/pycqa/flake8-docstrings
* Configuration: [setup.cfg](setup.cfg)
```shell
# No commands.
# Runs with flake8 at the same time.
```

<!----------------------------------------------------------
#   Pyright
# --------------------------------------------------------->
### Pyright
Pyright is a fast type checker meant for large Python source bases.
* Official: https://github.com/microsoft/pyright
* Configuration: [pyproject.toml](pyproject.toml)
```shell
pyright --warnings ./
```

<!----------------------------------------------------------------------------------------------------------------------
#
#   Metrics
#
# --------------------------------------------------------------------------------------------------------------------->
## Metrics

<!----------------------------------------------------------
#   Radon
# --------------------------------------------------------->
### Radon
Radon is a Python tool that computes various metrics from the source code.
* Official: https://github.com/rubik/radon
* Configuration: [setup.cfg](setup.cfg)

#### Cyclomatic Complexity
See https://radon.readthedocs.io/en/latest/commandline.html#the-cc-command

| CC score | Rank | Risk                                    |
|----------|------|-----------------------------------------|
| 1 - 5    | A    | low - simple block                      |
| 6 - 10   | B    | low - well structured and stable block  |
| 11 - 20  | C    | moderate - slightly complex block       |
| 21 - 30  | D    | more than moderate - more complex block |
| 31 - 40  | E    | high - complex block, alarming          |
| 41+      | F    | very high - error-prone, unstable block |

Compute Cyclomatic Complexity (`.py`)
```shell
radon cc ./
```

#### Maintainability Index
See https://radon.readthedocs.io/en/latest/commandline.html#the-mi-command

| MI score | Rank | Maintainability |
|----------|------|-----------------|
| 100 - 20 | A    | Very high       |
| 19 - 10  | B	   | Medium          |
| 9 - 0    | C	   | Extremely low   |

Compute Maintainability Index score. (`.py`)
```shell
radon mi ./
```

Compute Maintainability Index score. (`.ipynb`)
```shell
radon mi --include-ipynb --ipynb-cells --exclude "*.py" ./
```

<!----------------------------------------------------------------------------------------------------------------------
#
#   Test
#
# --------------------------------------------------------------------------------------------------------------------->
## Test

<!----------------------------------------------------------
#   Pytest
# --------------------------------------------------------->
### Pytest
pytest is a mature full-featured Python testing tool that helps you write better programs.
* Official: https://github.com/pytest-dev/pytest
* Official: https://github.com/pytest-dev/pytest-cov
* Configuration: [pytest.ini](pytest.ini)

Basic (Skip slow processes)
```shell
pytest --cov ./
```

Full (None skip)
```shell
pytest --cov --all ./
```

<!----------------------------------------------------------------------------------------------------------------------
#
#   Shell
#
# --------------------------------------------------------------------------------------------------------------------->
## Shell

<!----------------------------------------------------------
#   Linter
# --------------------------------------------------------->
### Test Linter
Run all Linter commands above.

(Bandit, Black, docformatter, Flake8, isort, pydocstyle, Pyright, yamllint, xenon)

```shell
bash .tools/test-linter.sh
```
