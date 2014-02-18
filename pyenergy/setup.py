import os
from setuptools import setup

def read(fname):
    return open(os.path.join(os.path.dirname(__file__), fname)).read()

setup(
    name = "pyenergy",
    version = "1.0",
    author = "James Pallister",
    author_email = "james.pallister@bristol.ac.uk",
    description = ("An interface to the MAGEEC energy monitor boards"),
    license = "GPLv3",
    keywords = "energy monitor MAGEEC",
    # url = "http://packages.python.org/an_example_pypi_project",
    package_dir={'':'src'},
    packages=['pyenergy', 'platformrun'],
    long_description=read('README.md'),
    install_requires=['pyusb>=1.0.0b1', 'docopt>=0.6.1'],
    entry_points = {
        'console_scripts': [
            'platformrun = platformrun:main',
            'platformrun-detect = platformrun.detect:main',
            'energytool = pyenergy.tools:main',
        ],
        # 'gui_scripts': [
        #     'interactive_graph = pyenergy.interactive_graph:main'
        # ]
    }
)
