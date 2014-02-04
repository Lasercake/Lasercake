#!/usr/bin/env python
# (Python >= 2.6 including >= 3)

from __future__ import division
import re
import os
import sys
import getopt
import multiprocessing
import subprocess
import os.path
import shutil
import pwd
import grp
import traceback
from pipes import quote

def usage():
    return """Usage: $0 [options] <tag name>|git

  Example tag names are: Lasercake-0.22
  Does not do much interesting without options.

  (Serious) Limitations:
  Must be root because we chroot to Fedora and Debian containers.
  Must be on Fedora Linux because we create Fedora containers
    which is easiest to do, and do securely, on Fedora.
  Feel free to run this in a virtual machine.
  Requires about a gigabyte of total RAM+swap.  C++ compilation
    uses lots of RAM; cross-compiling C++ for Windows seems to use
    so much more.  If you don't have enough, this script will try
    to let you know, and you can add the --makeswap option.
  Can't build OS X binaries as easily and reproducibly as Linux and
    Windows binaries.  See "OS X conundrum" section below.

  Options:

  --update-host-fedora    Uses `yum` to update and to install
                          this script's dependencies.
  --mingw                 Builds Lasercake for 32 and 64 bit Windows.
  --linux                 Builds dynamically linked Lasercakes for
                          Linux x86 and x86_64.

  --git=https://github.com/Lasercake/Lasercake.git
                          Change the default place to fetch Lasercake
                          source from. Passed to `git clone` or `git pull`.
  --workdir=/root/Lasercake-build
                          Change the default place to create build chroots
                          and put build results.
  --makeswap=[/root/swapfile]
                          Ask this script to make a swapfile.  If the
                          default location is fine, pass `--makeswap=`
                          (Python 2.6's option parsing libraries don't
                          have proper optional-argument support, sorry.)
  --clean-distro-containers
                          By default, chroots created inside workdir
                          are kept as-is if they already exist.  This
                          option instead deletes any existing chroots
                          that are about to be used and rebuilds them.
  --no-clean-build-dirs
                          By default, the build directories inside the
                          chroots are deleted every time before building.
                          This option prevents that; potentially useful
                          for testing, at the risk of making builds less
                          reproducible.

  OS X conundrum
    There are freely redistributable ways to create Linux binaries (Linux)
    and Windows binaries (cross-compiled from Linux), but none for OS X
    that I know of.  All of the ways I know of require OS X, which is not
    freely redistributable.  Even having an OS X license doesn't make it
    technically or legally practical to create throwaway OS X virtual
    machines.  (It's possible, on a Mac, but I haven't found any
    guides for doing it in a secure and legal way.  Security is critical
    because our users trust us not to offer them compromised binaries.)
"""

standard_PATH = '/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin'
os.putenv('PATH', standard_PATH)

try:
    optlist, args = getopt.getopt(sys.argv[1:], '', [
        'update-host-fedora', 'mingw', 'linux', 'git=', 'workdir=',
        'makeswap=', 'clean-distro-containers', 'no-clean-build-dirs'
    ])
    optdict = dict(optlist)
    if '--help' in optdict:
        print(usage())
        exit(0)
    [tag] = args
except (getopt.GetoptError, ValueError):
    sys.stderr.write(usage())
    exit(1)

workdir = optdict.get('--workdir', "/root/Lasercake-build")
resultsdir = workdir+'/results'
local_source_clone_dir = workdir+'/Lasercake-source'
git_clone_from = optdict.get('--git',
                    'https://github.com/Lasercake/Lasercake.git')
cmakeflags = ['-DLTO=ON']
release_name = 'computed-in-prepare_workdir'
mingw_ram_estimate = 900
linux_ram_estimate = 300

def log_cmd(args, **kwargs):
    """ helper for cmd* """
    if isinstance(args, str): args = [args]
    assert all(isinstance(arg, str) for arg in args), "cmd-calling bug"
    sigil = '#' if os.geteuid() == 0 else '$'
    out = str(os.getpid())+sigil+' '
    if kwargs.get('shell', False):
        [arg] = args
        out += arg
    else:
        out += ' '.join(quote(arg) for arg in args)
    out +='\n'
    sys.stderr.write(out)

# Shorthand subprocess callers that log the commands to stderr.
def cmdstatus(args, **kwargs):
    """ logs 'args', then forwards to subprocess.call """
    log_cmd(args, **kwargs)
    return subprocess.call(args, **kwargs)

def cmd(args, **kwargs):
    """ logs 'args', then forwards to subprocess.check_call """
    log_cmd(args, **kwargs)
    return subprocess.check_call(args, **kwargs)

# Python 2.6 doesn't have subprocess.check_output, so include an implementation
# https://stackoverflow.com/questions/4814970/subprocess-check-output-doesnt-seem-to-exist-python-2-6-5
if "check_output" in dir(subprocess):
    check_output = subprocess.check_output
else:
    def check_output(*popenargs, **kwargs):
        if 'stdout' in kwargs:
            raise ValueError('stdout argument not allowed, it will be overridden.')
        process = subprocess.Popen(stdout=subprocess.PIPE, *popenargs, **kwargs)
        output, unused_err = process.communicate()
        retcode = process.poll()
        if retcode:
            cmd = kwargs.get("args")
            if cmd is None:
                cmd = popenargs[0]
            raise subprocess.CalledProcessError(retcode, cmd)
        return output

def cmdoutput(args, **kwargs):
    """ logs 'args', then forwards to subprocess.check_output """
    log_cmd(args, **kwargs)
    return check_output(args, **kwargs)

def cmdoutputline(args, **kwargs):
    """ cmdoutput() minus a trailing \n, like shell "$()". """
    val = cmdoutput(args, **kwargs)
    if bytes != str: val = val.decode()
    if val[-1:] == '\n':
        val = val[:-1]
    return val

def replace_environ(new_env):
    os.environ.clear()
    for k, v in new_env.items():
        os.environ[k] = v


def estimate_free_ram_megabytes():
    # /proc/meminfo is a series of sizes in kibibytes (2^10).
    # We return mebibytes (2^20).
    with open('/proc/meminfo', 'r') as f:
        meminfo = f.read()
    kibibytes = sum(map(int,
      re.findall(r'^(?:MemFree|Buffers|Cached): *([0-9]*)',
                  meminfo, re.MULTILINE)))
    return kibibytes // 1024
def free_disk_megabytes(location):
    s = os.statvfs('/')
    return (s.f_bsize * s.f_bavail) // (1024**2)
def parallelism(ram_megabytes_needed):
    freeish_ram_megabytes = estimate_free_ram_megabytes()
    processors = multiprocessing.cpu_count()
    return max(1, min(processors,
                      freeish_ram_megabytes // ram_megabytes_needed))
def require_ram_megabytes(ram_megabytes_needed):
    if (not cmdoutput('swapon')  # swapon of '' == no swap
        and estimate_free_ram_megabytes() < ram_megabytes_needed):
        raise OSError("Not enough RAM")
def require_disk_megabytes(location, disk_megabytes_needed):
    if free_disk_megabytes(location) < disk_megabytes_needed:
        raise OSError("Not enough disk space on "+location)


def get_unused_uidgid():
    """
    Returns a (uid, gid) pair of integers that is currently unused
    on the host system.
    """
    etc_passwd = pwd.getpwall()
    etc_group = grp.getgrall()
    used_uids = set(map(lambda l: l.pw_uid, etc_passwd))
    used_gids = set(map(lambda l: l.gr_gid, etc_group))
    uid = 2222
    while uid in used_uids:
        uid += 1
    gid = 2222
    while gid in used_gids:
        gid += 1
    return (uid, gid)

def subpython(function):
    pid = os.fork()
    if pid == 0:
        try:
          sys.stderr.write('subprocess beginning\n')
          function()
          sys.stderr.write('subprocess ending\n')
          os._exit(0)
        except:
          traceback.print_exc()
          os._exit(1)
    else:
        if os.waitpid(pid, 0)[1] != 0:
            raise OSError("subpython error")

def run_in_container_as_unnamed_user(
        container_root, scratchdir_within_root, function, clean=True):
    """
    Runs function() while chroot()ed to container_root
    and uid/gid set to a user who doesn't exist in the host or guest system
    with scratchdir_within_root removed if clean=True and, regardless,
    created as a directory and chown'ed to the nonexistent user and
    set to the current directory.
    """
    uid, gid = get_unused_uidgid()
    bindunmount = []
    try:
        cmd(['touch', container_root+'/etc/resolv.conf'])
        for f in ['/dev', '/proc', '/sys', '/etc/resolv.conf']:
            if cmdstatus(['mountpoint', '-q', container_root+f]) != 0:
                cmd(['mount', '--bind', f, container_root+f])
                bindunmount.append(container_root+f)
        def sub():
            replace_environ({
                'TERM': os.getenv('TERM'),
                'SHELL': '/bin/bash',
                'PATH': standard_PATH
            })
            os.chroot(container_root)
            os.chdir('/')
            if clean:
                shutil.rmtree(scratchdir_within_root, ignore_errors=True)
            if not os.path.isdir(scratchdir_within_root):
                os.makedirs(scratchdir_within_root)
            os.chown(scratchdir_within_root, uid, gid)
            os.chdir(scratchdir_within_root)
            os.setgroups([])
            os.setgid(gid)
            os.setuid(uid)
            function()
        subpython(sub)
    finally:
        for u in bindunmount:
            cmd(['umount', u])


def makeswap(path):
    if not os.path.exists(path):
        cmd(['dd', 'if=/dev/zero', 'of='+path, 'bs=1024', 'count=1048576'])
    cmd(['chmod', '600', path])
    cmd(['mkswap', path])
    cmd(['swapon', path])


def host_fedora_prepare():
    # Make sure /etc/pki/rpm-gpg/ (from fedora-release package) is up to date.
    cmd(['yum', 'update'])

    # git is used to fetch Lasercake source and create git version names.
    # debootstrap is used to build binaries for Linux.
    # debootstrap requires perl, debian-keyring and gpg1 to work fully.
    cmd(['yum', 'install',
          'git',
          'debootstrap', 'perl', 'debian-keyring', 'gnupg'])

def prepare_workdir():
    cmd(['mkdir', '-p', workdir, resultsdir])

    if not os.path.exists(local_source_clone_dir):
        cmd(['git', 'clone', git_clone_from, local_source_clone_dir])
    else:
        cmd(['git', 'pull', git_clone_from], cwd=local_source_clone_dir)
    if tag != "git":
        cmd(['git', 'reset', '--hard', 'tags/'+tag],
                        cwd=local_source_clone_dir)
    global release_name
    release_name = cmdoutputline(['git', 'describe'],
                        cwd=local_source_clone_dir)

def build_for_mingw_bare(srcdir, release_dir_name, bits, fedora_mingw_dir):
    """
    Builds a Lasercake release based on srcdir targeting mingw 32 or 64.
    It's 32 or 64 bits depending on 'bits'; it requires fedora_mingw_dir
    to point to the corresponding dir such as `/usr/i686-w64-mingw32`.
    The exact paths and DLLs within it are distro specific, and this
    function assumes Fedora.  Tested with Fedora 20 as of this writing.

    Does not require root or install or download anything;
    instead requires the environment to be prepared already.

    Assumes the current directory is the directory to build in.
    """
    # TODO: figure out how to successfully link statically here.
    # Hours of trying CMake configuration, linker arguments,
    # and searching the Web found lots of people trying to do it
    # but no complete examples of how to do it successfully.
    # Adding
    #   -ltiff -lpng -ljpeg -lz -limm32 -lwinmm -lws2_32
    # to the end of the g++ linker commandline fixed all the link errors
    # except the many where Lasercake code was trying to link to Qt.
    #        '-DCMAKE_EXE_LINKER_FLAGS=-static'
    # Also/alternative TODO: create an NSIS installer
    # (which might involve modifying CMakeLists.txt too).
    cmd(['mingw{}-cmake'.format(bits), srcdir] + cmakeflags)
    cmd(['make', '-j'+str(parallelism(mingw_ram_estimate))])
    shutil.rmtree(release_dir_name, ignore_errors=True)
    os.mkdir(release_dir_name)
    os.rename('Lasercake.exe', '{}/{}.exe'
              .format(release_dir_name, release_name))
    shutil.copy(srcdir+'/resources/ReadMe.rtf', release_dir_name)
    for dll in [
            'QtGui4.dll', 'QtCore4.dll', 'QtOpenGL4.dll',
            'libstdc++-6.dll', 'zlib1.dll', 'libpng16-16.dll',
            'libgcc_s_sjlj-1.dll' if int(bits)==32 else 'libgcc_s_seh-1.dll']:
        shutil.copy(fedora_mingw_dir+'/sys-root/mingw/bin/'+dll,
                    release_dir_name)
    cmd(['zip', '-r', release_dir_name+'.zip', release_dir_name])


def build_for_mingw():
    # Fedora is a good distro for building Windows binaries, because,
    # in addition to containing a mingw-w64 toolchain, Fedora (unlike Debian,
    # Ubuntu and Arch Linux as of 2013) contains pre-built mingw-w64 library
    # binaries for libraries we use (Qt).

    # For future reference: to make this Fedora a specific architecture,
    # `setarch i386 yum [args]`.
    root = workdir+'/fedora'
    if '--clean-distro-containers' in optdict:
        shutil.rmtree(root, ignore_errors=True)
    if not os.path.exists(root):
        cmd(['yum', '-y', '--releasever=20', '--installroot='+root,
            "--disablerepo='*'", '--enablerepo=fedora',
            'install',
            'systemd', 'passwd', 'yum', 'fedora-release', 'vim-minimal',
            'cmake', 'make', 'zip'] + [m+n
                for m in ['mingw32-', 'mingw64-']
                for n in ['gcc', 'gcc-c++', 'qt-static']])
    chroot_source_dir = '/'+release_name+'-source'
    cmd(['rsync', '-a', '--delete',
        local_source_clone_dir+'/', root+chroot_source_dir+'/'])
    for bits, fedora_mingw_dir in [('32', '/usr/i686-w64-mingw32'),
                                   ('64', '/usr/x86_64-w64-mingw32')]:
        release_dir_name = '{}-win{}'.format(release_name, bits)
        chroot_build_dir = ('/home/lasercake-builder/{}-build-{}'
                            .format(release_name, bits))
        run_in_container_as_unnamed_user(
            root,
            chroot_build_dir,
            lambda: build_for_mingw_bare(chroot_source_dir,
                        release_dir_name, bits, fedora_mingw_dir),
            clean = '--no-clean-build-dirs' not in optdict
            )
        shutil.copy(root+chroot_build_dir+'/'+release_dir_name+'.zip',
                    resultsdir)

def build_for_linux_bare(srcdir, release_dir_name):
    """
    Builds a Lasercake release based on srcdir targeting
    the current distro, dynamically linked.

    Does not require root or install or download anything;
    instead requires the environment to be prepared already.

    Assumes the current directory is the directory to build in.
    """
    cmd(['cmake', srcdir] + cmakeflags)
    cmd(['make', '-j'+str(parallelism(linux_ram_estimate))])
    shutil.rmtree(release_dir_name, ignore_errors=True)
    os.mkdir(release_dir_name)
    os.rename('Lasercake', '{}/{}'.format(release_dir_name, release_name))
    shutil.copy(srcdir+'/README.markdown', release_dir_name)
    cmd(['tar', '-czf', release_dir_name+'.tar.gz', release_dir_name])

def build_for_linux():
    # Debian Wheezy (Debian Stable as of 2013) has GCC 4.7 and Qt 4.8
    # and is roughly the oldest and stablest Linux distro we support.
    # This makes it a good choice for building Linux binaries on;
    # Linux binaries are more often forwards-compatible than
    # backwards-compatible.  Build against new glibc ---> likely can't
    # run on older glibc.

    for arch, debian_arch in [('x86', 'i386'),
                              ('x86_64', 'amd64')]:
        root = workdir+'/'+arch+'-wheezy'
        if '--clean-distro-containers' in optdict:
            shutil.rmtree(root, ignore_errors=True)
        if not os.path.exists(root):
            cmd(['debootstrap', '--arch='+debian_arch,
                '--include=build-essential,cmake,libqt4-dev,libqt4-opengl-dev',
                'wheezy', root,
                'http://ftp.us.debian.org/debian'])
        chroot_source_dir = '/'+release_name+'-source'
        cmd(['rsync', '-a', '--delete',
            local_source_clone_dir+'/', root+chroot_source_dir+'/'])
        release_dir_name = "{}-linux-{}-dynamic".format(release_name, arch)
        chroot_build_dir = ("/home/lasercake-builder/{}-build-{}"
                            .format(release_name, arch))
        run_in_container_as_unnamed_user(
            root,
            chroot_build_dir,
            lambda: build_for_linux_bare(chroot_source_dir, release_dir_name),
            clean = '--no-clean-build-dirs' not in optdict
            )
        shutil.copy(root+chroot_build_dir+'/'+release_dir_name+'.tar.gz',
                    resultsdir)

def main():
    if '--update-host-fedora' in optdict:
      host_fedora_prepare()
    if '--makeswap' in optdict:
        makeswap(optdict['--makeswap'] or '/root/swapfile')

    if '--mingw' in optdict:
        require_ram_megabytes(mingw_ram_estimate)
    if '--linux' in optdict:
        require_ram_megabytes(linux_ram_estimate)
    # require_disk_megabytes(4000ish?)
    # but what if the chroot distro instances already exist:
    # should we subtract their sizes, wasting the time to compute that?

    prepare_workdir()
    if '--mingw' in optdict:
        build_for_mingw()
    if '--linux' in optdict:
        build_for_linux()
    print()
    print("RESULTS OF THIS AND PAST RUNS:")
    print(resultsdir)
    print(os.listdir(resultsdir))


if __name__ == '__main__':
    main()
