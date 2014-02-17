"""
Usage:
    platformrun-detect [options]

Options:
    -c --config FILE    Specify the tools configuration file
                        [default: ~/.platformrunrc]
    -p --prompt         Prompt for locations of tools not found
    --clean             Overwrite config with a clean configuration

This tool will help set up the configuration for platform run.
This will try and detect the prerequisites needed to run on
each platform and enter them into a configuration file.

"""
from docopt import docopt
import json, os, os.path, sys
import copy
import readline, glob

default_config = {
    'tools': {
        'arm_gdb' : 'arm-none-eabi-gdb',
        'stutil' : 'st-util',
        'avr_objcopy' : "avr-objcopy",
        'avrdude' : "avrdude",
        'pic32_objcopy' : 'pic32-objcopy',
        'pic32prog' : 'pic32prog',
        'mspdebug': 'mspdebug'
    },
    'platforms': {
        'stm32f0discovery': ['arm_gdb', 'stutil'],
        'stm32vldiscovery': ['arm_gdb', 'stutil'],
        'atmega328p': ['avr_objcopy', 'avrdude'],
        'pic32mx250f128b': ['pic32_objcopy', 'pic32prog'],
        'msp-exp430f5529' : ['mspdebug'],
        'msp-exp430fr5739' : ['mspdebug'],
    },
    'enabled': []
}

# This is directly taken from shutils.which (not yet backported to python 2.X)
def which(cmd, mode=os.F_OK | os.X_OK, path=None):
    """Given a command, mode, and a PATH string, return the path which
    conforms to the given mode on the PATH, or None if there is no such
    file.

    `mode` defaults to os.F_OK | os.X_OK. `path` defaults to the result
    of os.environ.get("PATH"), or can be overridden with a custom search
    path.

    """
    # Check that a given file can be accessed with the correct mode.
    # Additionally check that `file` is not a directory, as on Windows
    # directories pass the os.access check.
    def _access_check(fn, mode):
        return (os.path.exists(fn) and os.access(fn, mode)
                and not os.path.isdir(fn))

    # If we're given a path with a directory part, look it up directly rather
    # than referring to PATH directories. This includes checking relative to the
    # current directory, e.g. ./script
    if os.path.dirname(cmd):
        if _access_check(cmd, mode):
            return cmd
        return None

    if path is None:
        path = os.environ.get("PATH", os.defpath)
    if not path:
        return None
    path = path.split(os.pathsep)

    if sys.platform == "win32":
        # The current directory takes precedence on Windows.
        if not os.curdir in path:
            path.insert(0, os.curdir)

        # PATHEXT is necessary to check on Windows.
        pathext = os.environ.get("PATHEXT", "").split(os.pathsep)
        # See if the given file matches any of the expected path extensions.
        # This will allow us to short circuit when given "python.exe".
        # If it does match, only test that one, otherwise we have to try
        # others.
        if any(cmd.lower().endswith(ext.lower()) for ext in pathext):
            files = [cmd]
        else:
            files = [cmd + ext for ext in pathext]
    else:
        # On other platforms you don't have things like PATHEXT to tell you
        # what file suffixes are executable, so just pass on cmd as-is.
        files = [cmd]

    seen = set()
    for dir in path:
        normdir = os.path.normcase(dir)
        if not normdir in seen:
            seen.add(normdir)
            for thefile in files:
                name = os.path.join(dir, thefile)
                if _access_check(name, mode):
                    return name

def detect_prog(progname, prompt=False):

    def preinput(text):
        def f():
            readline.insert_text(text)
            readline.redisplay()
        readline.set_pre_input_hook(f)

    if not os.path.exists(progname):
        f = which(progname)
    else:
        f = progname

    if f is None and prompt:
        path = None

        preinput(progname)

        while True:
            path = raw_input("Enter location of \"{}\"\n > ".format(progname))

            if path == "":
                f = None
                break

            path = os.path.expanduser(path)

            if os.path.isdir(path):
                if os.path.exists(os.path.join(path, progname)):
                    f = os.path.join(path, progname)
                    break
                else:
                    print '"{}" cannot be found under directory "{}"'.format(progname,path)
            else:
                if os.path.exists(path):
                    f = path
                    break
                else:
                    print '"{}" cannot be found'.format(path)


    return f

def main():
    arguments = docopt(__doc__)

    # Set up file completion
    def complete(text, state):
        return (glob.glob(os.path.expanduser(text)+'*')+[None])[state]

    readline.set_completer_delims(' \t\n;')
    readline.parse_and_bind("tab: complete")
    readline.set_completer(complete)

    # Load existing config
    cfg = os.path.expanduser(arguments['--config'])
    if os.path.exists(cfg):
        config = json.load(open(cfg))
    else:
        print "Configuration file does not exist, creating new config"
        config = copy.deepcopy(default_config)

    # Try and detect if the programs exist
    for tool, name in default_config['tools'].items():
        if not isinstance(config['tools'][tool], str) and not isinstance(config['tools'][tool], unicode):
            config['tools'][tool] = default_config['tools'][tool]
        config['tools'][tool] = detect_prog(config['tools'][tool], prompt=arguments['--prompt'])

    # For each platform, check we have the dependencies
    config['enabled'] = []
    for platform in default_config['platforms'].keys():
        haveTools = True
        for tool in default_config['platforms'][platform]:
            if config['tools'][tool] is None:
                haveTools = False

        if haveTools:
            config['enabled'].append(platform)

    # Write out the config file
    json.dump(config, open(os.path.expanduser(arguments['--config']), "w+"),
        indent=2)

    # Display a summary
    print "\nSummary\n"
    for platform in default_config['platforms'].keys():
        print "{: <20}      {}".format(platform, "enabled" if platform in config['enabled'] else "disabled")

if __name__=="__main__":
    main()
