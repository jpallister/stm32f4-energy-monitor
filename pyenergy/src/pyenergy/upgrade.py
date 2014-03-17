#!/usr/bin/python

"""Energy measurement upgrade tool.

This tools loads pickled objects, upgrades any Measurement types it finds
and then writes the objects back to the file. This is mostly useful for
data saved in the old namedtuple Measurement format, as other measurement
will get upgraded on load anyway.

Usage:
    measurement-upgrade [-v | -vv] FILENAME

Options
    --verbose -v        Be verbose
"""

import pyenergy
import logging
from docopt import docopt

logger = logging.getLogger(__name__)
logger.addHandler(logging.NullHandler())
warning = logger.warning
error = logger.error
info = logger.info


### Start ugly, ugly hack here ################################################
# This hack is here to maintain backwards compatability for Measurements
# already pickled, using the old namedtuple approach. The new Measurement
# class has at least the old functionality of the old tuple (except it is not)
# a tuple.
#
# This hack modifies the pickle module to use a different method of
# instantiating a class when it discovers an old namedtuple class. It then uses
# the arguments to create a new style Measurement class
import pickle, sys

def _reconstructor_hack(cls, base, state):
    if base is object:
        obj = object.__new__(cls)
    else:
        # Detect whether the cls and base are old style, and if so
        # create the object using its constructor, instead of tuple's __new__
        if cls == pyenergy.Measurement and base is tuple:
            if not hasattr(_reconstructor_hack, "warned"):
                warning("Loading from unversioned pickled file.")
                _reconstructor_hack.warned = True
            return pyenergy.Measurement(*state)
        obj = base.__new__(cls, state)
        if base.__init__ != object.__init__:
            base.__init__(obj, state)
    return obj

def find_class_hack(self, module, name):
    if name == "_reconstructor":
        return _reconstructor_hack

    __import__(module)
    mod = sys.modules[module]
    klass = getattr(mod, name)
    return klass

### End ugly hack #############################################################

def updateOldPickle(fname):

    oldfind = pickle.Unpickler.find_class
    pickle.Unpickler.find_class = find_class_hack

    objs = []
    f = open(fname, "r")
    while True:
        try:
            o = pickle.load(f)
            objs.append(o)
        except EOFError:
            break
    f.close()

    print o[0][0]

    f = open(fname, "w")
    for o in objs:
        pickle.dump(o, f)

    f.close()

    pickle.Unpickler.oldfind = find_class_hack

def main():
    arguments = docopt(__doc__)

    logging.basicConfig()

    if arguments['--verbose'] == 1:
        logging.getLogger('').setLevel(logging.INFO)
    elif arguments['--verbose']== 2:
        logging.getLogger('').setLevel(logging.DEBUG)

    updateOldPickle(arguments['FILENAME'])

if __name__ == "__main__":
    main()
