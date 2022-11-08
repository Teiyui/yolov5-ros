from setuptools import setup


def _load_requires():
    return open("requirements.txt").read().splitlines()


setup(
    install_requires=_load_requires(),
)
