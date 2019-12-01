#!/usr/bin/env python

# SPDX-License-Identifier: BSD-2-Clause
#
# Copyright (C) 2019 embedded brains GmbH
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import print_function

import pickle
import os
import re
import stat
import sys
from string import Template

try:
    import configparser
except:
    import ConfigParser as configparser

is_windows_host = os.name == "nt" or sys.platform in ["msys", "cygwin"]
default_prefix = "/opt/rtems/5"
compilers = ["gcc", "clang"]
items = {}
bsps = {}
top_level_groups = []


def no_unicode(value):
    if sys.version_info[0] > 2:
        return value
    if isinstance(value, unicode):
        return str(value)
    return value


class VersionControlKeyHeader:
    _content = None

    @staticmethod
    def write(bld, filename):
        if VersionControlKeyHeader._content is None:
            from waflib.Build import Context
            from waflib.Errors import WafError

            content = """/*
 * Automatically generated. Do not edit.
 */
#if !defined(_RTEMS_VERSION_VC_KEY_H_)
#define _RTEMS_VERSION_VC_KEY_H_
"""
            try:
                rev = bld.cmd_and_log(
                    "git rev-parse HEAD", quiet=Context.STDOUT
                ).strip()
                content += """#define RTEMS_VERSION_VC_KEY "{}"
""".format(
                    rev
                )
            except WafError:
                content += """/* No version control key found; release? */
"""
            content += """#endif
"""
            VersionControlKeyHeader._content = content
        f = bld.bldnode.make_node(filename)
        f.parent.mkdir()
        try:
            if content != f.read():
                f.write(VersionControlKeyHeader._content)
        except:
            f.write(VersionControlKeyHeader._content)


class EnvWrapper(object):
    def __init__(self, env):
        self._env = env

    def __getitem__(self, name):
        v = self._env[name]
        if isinstance(v, list):
            return " ".join(v)
        return v


class Item(object):
    def __init__(self, uid, data):
        self.uid = uid
        self.data = data
        self.links = self._init_links

    def _init_links(self):
        self._links = []
        for l in self.data["links"]:
            self._links.append(items[list(l.keys())[0]])
        self._links.sort(key=lambda x: x.data["order"])
        self.links = self._yield_links
        for l in self._links:
            yield l

    def _yield_links(self):
        for l in self._links:
            yield l

    @staticmethod
    def _is_enabled(enabled, enabled_by):
        if enabled_by:
            if isinstance(enabled_by, list):
                for e in enabled_by:
                    if Item._is_enabled(enabled, e):
                        return True
            elif isinstance(enabled_by, dict):
                if len(enabled_by) == 1:
                    if "and" in enabled_by:
                        for e in enabled_by["and"]:
                            if not Item._is_enabled(enabled, e):
                                return False
                        return True
                    elif "not" in enabled_by:
                        return not Item._is_enabled(enabled, enabled_by["not"])
                    elif "or" in enabled_by:
                        for e in enabled_by["or"]:
                            if Item._is_enabled(enabled, e):
                                return True
            else:
                return enabled_by in enabled
            return False
        return True

    def get_enabled_by(self):
        return self.data["enabled-by"]

    def defaults(self, enable, variant):
        if Item._is_enabled(enable, self.get_enabled_by()):
            for p in self.links():
                p.defaults(enable, variant)
            self.do_defaults(variant)

    def configure(self, conf, cic):
        if Item._is_enabled(conf.env.ENABLE, self.get_enabled_by()):
            self.prepare_configure(conf, cic)
            for p in self.links():
                p.configure(conf, cic)
            self.do_configure(conf, cic)

    def build(self, bld, bic):
        if Item._is_enabled(bld.env.ENABLE, self.get_enabled_by()):
            bic = self.prepare_build(bld, bic)
            for p in self.links():
                p.build(bld, bic)
            self.do_build(bld, bic)

    def do_defaults(self, variant):
        return

    def prepare_configure(self, conf, cic):
        return

    def do_configure(self, conf, cic):
        return

    def prepare_build(self, bld, bic):
        return bic

    def do_build(self, bld, bic):
        return

    def substitute(self, ctx, value):
        if isinstance(value, str):
            try:
                return Template(value).substitute(EnvWrapper(ctx.env))
            except Exception as e:
                ctx.fatal(
                    "In item '{}' substitution in '{}' failed: {}".format(
                        self.uid, value, e
                    )
                )
        return value

    def get(self, ctx, name):
        return self.substitute(ctx, self.data[name])

    def get_values(self, ctx, name):
        more = []
        for value in self.data[name]:
            more.extend(self.substitute(ctx, value).split())
        return more

    def install_target(self, bld):
        install_path = self.data["install-path"]
        if install_path:
            bld.install_files(install_path, self.get(bld, "target"))

    def install_files(self, bld):
        for install in self.data["install"]:
            bld.install_files(install["destination"], install["source"])

    def asm(self, bld, bic, source, target=None, deps=[], cppflags=[]):
        if target is None:
            target = os.path.splitext(source)[0] + ".o"
        bld(
            asflags=self.data["asflags"],
            before=["cstlib"],
            cppflags=cppflags + self.data["cppflags"],
            features="asm c",
            includes=bic.includes + self.data["includes"],
            rule="${CC} ${ASFLAGS} ${CPPFLAGS} ${DEFINES_ST:DEFINES} ${CPPPATH_ST:INCPATHS} -c ${SRC[0]} -o ${TGT}",
            source=[source] + deps,
            target=target,
        )
        return target

    def cc(self, bld, bic, source, target=None, deps=[], cppflags=[]):
        if target is None:
            target = os.path.splitext(source)[0] + ".o"
        bld(
            cflags=self.data["cflags"],
            cppflags=cppflags + self.data["cppflags"],
            features="c",
            includes=bic.includes + self.data["includes"],
            rule="${CC} ${CFLAGS} ${CPPFLAGS} ${DEFINES_ST:DEFINES} ${CPPPATH_ST:INCPATHS} -c ${SRC[0]} -o ${TGT}",
            source=[source] + deps,
            target=target,
        )
        return target

    def cxx(self, bld, bic, source, target=None, deps=[], cppflags=[]):
        if target is None:
            target = os.path.splitext(source)[0] + ".o"
        bld(
            cppflags=cppflags + self.data["cppflags"],
            cxxflags=self.data["cxxflags"],
            features="cxx",
            includes=bic.includes + self.data["includes"],
            rule="${CXX} ${CXXFLAGS} ${CPPFLAGS} ${DEFINES_ST:DEFINES} ${CPPPATH_ST:INCPATHS} -c ${SRC[0]} -o ${TGT}",
            source=[source] + deps,
            target=target,
        )
        return target

    def link(self, bld, bic, cmd, source, target):
        from waflib.Task import Task

        class link(Task):
            def __init__(self, item, bic, cmd, env):
                super(link, self).__init__(self, env=env)
                self.cmd = cmd
                self.ldflags = bic.ldflags + item.data["ldflags"]
                self.stlib = item.data["stlib"]
                self.use = (
                    item.data["use-before"] + bic.use + item.data["use-after"]
                )

            def run(self):
                cmd = [self.cmd]
                cmd.extend(self.env.LINKFLAGS)
                cmd.extend([i.abspath() for i in self.inputs])
                cmd.append("-o" + self.outputs[0].abspath())
                cmd.extend(self.ldflags)
                cmd.append("-L.")
                cmd.extend(["-l" + l for l in self.stlib])
                cmd.extend(["-l" + l for l in self.use])
                cmd.extend(self.env.LDFLAGS)
                return self.exec_command(cmd)

            def scan(self):
                return (
                    [
                        self.generator.bld.bldnode.make_node("lib" + u + ".a")
                        for u in self.use
                    ],
                    [],
                )

        tsk = link(self, bic, cmd, bld.env)
        tsk.set_inputs([bld.bldnode.make_node(s) for s in source])
        tsk.set_outputs(bld.bldnode.make_node(target))
        bld.add_to_group(tsk)
        return target

    def link_cc(self, bld, bic, source, target):
        return self.link(bld, bic, bld.env.LINK_CC[0], source, target)

    def link_cxx(self, bld, bic, source, target):
        return self.link(bld, bic, bld.env.LINK_CXX[0], source, target)

    def gnatmake(self, bld, bic, objdir, objs, main, target):
        from waflib.Task import Task

        class gnatmake(Task):
            def __init__(self, bld, bic, objdir, objs, main, target, item):
                super(gnatmake, self).__init__(self, env=bld.env)
                self.objdir = objdir
                self.objs = [bld.bldnode.make_node(o) for o in objs]
                self.main = bld.path.make_node(main)
                self.set_inputs(self.objs + [self.main])
                self.set_outputs(bld.bldnode.make_node(target))
                self.adaflags = item.data["adaflags"]
                self.adaincludes = []
                for i in item.data["adaincludes"]:
                    self.adaincludes.append(bld.bldnode.make_node(i))
                    self.adaincludes.append(bld.path.make_node(i))
                self.ldflags = bic.ldflags + item.data["ldflags"]
                self.stlib = item.data["stlib"]
                self.use = (
                    item.data["use-before"] + bic.use + item.data["use-after"]
                )

            def run(self):
                cwd = self.get_cwd()
                cmd = [
                    self.env.GNATMAKE[0],
                    "-D",
                    self.objdir,
                    "-bargs",
                    "-Mgnat_main",
                    "-margs",
                ]
                cmd.extend(self.adaflags)
                cmd.extend(["-I" + i.path_from(cwd) for i in self.adaincludes])
                cmd.append("-largs")
                cmd.extend([o.path_from(cwd) for o in self.objs])
                cmd.extend(self.env.LINKFLAGS)
                cmd.extend(self.ldflags)
                cmd.append("-L.")
                cmd.extend(["-l" + l for l in self.stlib])
                cmd.extend(["-l" + l for l in self.use])
                cmd.extend(self.env.LDFLAGS)
                cmd.extend(["-margs", "-a"])
                cmd.append(self.main.abspath())
                cmd.append("-o")
                cmd.append(self.outputs[0].abspath())
                return self.exec_command(cmd)

            def scan(self):
                return (
                    [
                        self.generator.bld.bldnode.make_node("lib" + u + ".a")
                        for u in self.use
                    ],
                    [],
                )

        tsk = gnatmake(bld, bic, objdir, objs, main, target, self)
        bld.add_to_group(tsk)
        return target

    def ar(self, bld, source, target):
        bld(rule="${AR} ${ARFLAGS} ${TGT} ${SRC}", source=source, target=target)
        return target

    def gzip(self, bld, source):
        target = source + ".gz"
        bld(rule="${GZIP} < ${SRC} > ${TGT}", source=source, target=target)
        return target

    def xz(self, bld, source):
        target = source + ".xz"
        bld(rule="${XZ} < ${SRC} > ${TGT}", source=source, target=target)
        return target

    def pax(self, bld, source, remove, target):
        def run(task):
            srcpath = bld.path.abspath() + "/"
            bldpath = bld.bldnode.abspath() + "/"
            cmd = [bld.env.PAX[0], "-w", "-f", task.outputs[0].abspath()]
            for r in remove:
                cmd.extend(
                    [
                        "-s",
                        "," + srcpath + r + ",,",
                        "-s",
                        "," + bldpath + r + ",,",
                    ]
                )
            cmd.extend([i.abspath() for i in task.inputs])
            return task.exec_command(cmd)

        bld(rule=run, source=source, target=target)
        return target

    def bin2c(self, bld, source, name=None):
        def run(task):
            cmd = [bld.env.BIN2C[0]]
            if name is not None:
                cmd.extend(["-N", name])
            cmd.append(task.inputs[0].abspath())
            cmd.append(task.outputs[0].abspath())
            return task.exec_command(cmd)

        path, base = os.path.split(source)
        target = path + "/" + base.replace(".", "-")
        target_c = target + ".c"
        target_h = target + ".h"
        bld(rule=run, source=source, target=[target_c, target_h])
        return target_c, target_h

    def rtems_syms(self, bld, source, target):
        bld(
            rule='${RTEMS_SYMS} -e -C ${CC} -c "${CFLAGS}" -o ${TGT} ${SRC}',
            source=source,
            target=target,
        )
        return target

    def rtems_rap(self, bld, base, objects, libs, target):
        def run(task):
            cmd = [
                bld.env.RTEMS_LD[0],
                "-C",
                bld.env.CC[0],
                "-c",
                " ".join(bld.env.CFLAGS),
                "-O",
                "rap",
                "-b",
                task.inputs[0].abspath(),
                "-e",
                "rtems_main",
                "-s",
                "-o",
            ]
            cmd.append(task.outputs[0].abspath())
            cmd.extend([i.abspath() for i in task.inputs[1:]])
            cmd.extend(["-l" + l for l in libs])
            return task.exec_command(cmd)

        bld(rule=run, source=[base] + objects, target=target)
        return target


class GroupItem(Item):
    def __init__(self, uid, data):
        super(GroupItem, self).__init__(uid, data)
        if self.data["top-level"]:
            top_level_groups.append(self)
            top_level_groups.sort(key=lambda x: x.data["order"])

    def prepare_build(self, bld, bic):
        return BuildItemContext(
            bic.includes + self.get_values(bld, "includes"),
            self.data["use-before"] + bic.use + self.data["use-after"],
            bic.ldflags + self.get_values(bld, "ldflags"),
            bic.objects,
        )

    def do_build(self, bld, bic):
        self.install_files(bld)


class ConfigFileItem(Item):
    def __init__(self, uid, data):
        super(ConfigFileItem, self).__init__(uid, data)

    def do_configure(self, conf, cic):
        content = self.substitute(conf, self.data["content"])
        f = conf.bldnode.make_node(
            conf.env.VARIANT + "/" + self.get(conf, "target")
        )
        f.parent.mkdir()
        f.write(content)
        conf.env.append_value("cfg_files", f.abspath())

    def do_build(self, bld, bic):
        self.install_target(bld)


class ConfigHeaderItem(Item):
    def __init__(self, uid, data):
        super(ConfigHeaderItem, self).__init__(uid, data)

    def do_configure(self, conf, cic):
        conf.env.include_key = self.data["include-headers"]
        conf.write_config_header(
            conf.env.VARIANT + "/" + self.get(conf, "target"),
            guard=self.data["guard"],
            headers=True,
        )
        conf.env.include_key = None

    def do_build(self, bld, bic):
        self.install_target(bld)


class StartFileItem(Item):
    def __init__(self, uid, data):
        super(StartFileItem, self).__init__(uid, data)

    def do_build(self, bld, bic):
        self.asm(bld, bic, self.data["source"], self.get(bld, "target"))
        self.install_target(bld)


class ObjectsItem(Item):
    def __init__(self, uid, data):
        super(ObjectsItem, self).__init__(uid, data)

    def do_build(self, bld, bic):
        bld.objects(
            cflags=self.data["cflags"],
            cppflags=self.data["cppflags"],
            cxxflags=self.data["cxxflags"],
            includes=bic.includes + self.data["includes"],
            source=self.data["source"],
            target=self.uid,
        )
        bic.objects.append(self.uid)
        self.install_files(bld)


class BSPItem(Item):
    def __init__(self, uid, data):
        super(BSPItem, self).__init__(uid, data)
        arch = data["arch"].strip()
        if not arch in bsps:
            bsps[arch] = {}
        bsp = data["bsp"].strip()
        bsps[arch][bsp] = self

    def prepare_configure(self, conf, cic):
        conf.env.BSP_FAMILY = self.data["family"]

    def prepare_build(self, bld, bic):
        return BuildItemContext(
            bic.includes + bld.env.BSP_INCLUDES.split(), [], [], []
        )

    def do_build(self, bld, bic):
        bld(
            cflags=self.data["cflags"],
            cppflags=self.data["cppflags"],
            features="c cstlib",
            includes=bic.includes + self.data["includes"],
            install_path="${BSP_LIBDIR}",
            source=self.data["source"],
            target="rtemsbsp",
            use=bic.objects,
        )
        self.install_files(bld)


class LibraryItem(Item):
    def __init__(self, uid, data):
        super(LibraryItem, self).__init__(uid, data)

    def prepare_build(self, bld, bic):
        return BuildItemContext(bic.includes, [], [], [])

    def do_build(self, bld, bic):
        bld(
            cflags=self.data["cflags"],
            cppflags=self.data["cppflags"],
            cxxflags=self.data["cxxflags"],
            features="c cxx cstlib",
            includes=bic.includes + self.data["includes"],
            install_path=self.data["install-path"],
            source=self.data["source"],
            target=self.get(bld, "target"),
            use=bic.objects,
        )
        self.install_files(bld)


class TestProgramItem(Item):
    def __init__(self, uid, data):
        super(TestProgramItem, self).__init__(uid, data)
        name = uid.split("-")[-1]
        self.exclude = "TEST_" + name + "_EXCLUDE"
        self.cppflags = "TEST_" + name + "_CPPFLAGS"

    def get_enabled_by(self):
        return [{"and": [{"not": self.exclude}, self.data["enabled-by"]]}]

    def prepare_build(self, bld, bic):
        return BuildItemContext(bic.includes, bic.use, bic.ldflags, [])

    def do_build(self, bld, bic):
        bld(
            cflags=self.data["cflags"],
            cppflags=bld.env[self.cppflags] + self.data["cppflags"],
            cxxflags=self.data["cxxflags"],
            features=self.data["features"],
            includes=bic.includes + self.data["includes"],
            install_path=None,
            ldflags=bic.ldflags + self.data["ldflags"],
            source=self.data["source"],
            stlib=self.data["stlib"],
            target=self.get(bld, "target"),
            use=self.data["use-before"] + bic.use + self.data["use-after"],
        )


class AdaTestProgramItem(TestProgramItem):
    def __init__(self, uid, data):
        super(AdaTestProgramItem, self).__init__(uid, data)

    def do_build(self, bld, bic):
        objs = []
        for s in self.data["source"]:
            objs.append(self.cc(bld, bic, s))
        self.gnatmake(
            bld,
            bic,
            self.data["ada-object-directory"],
            objs,
            self.data["ada-main"],
            self.data["target"],
        )


class OptionItem(Item):
    def __init__(self, uid, data):
        super(OptionItem, self).__init__(uid, data)

    @staticmethod
    def _is_variant(variants, variant):
        for pattern in variants:
            if re.search("^" + pattern + "$", variant):
                return True
        return False

    def default_value(self, variant):
        value = self.data["default"]
        for default in self.data["default-by-variant"]:
            if OptionItem._is_variant(default["variants"], variant):
                value = default["value"]
                break
        if value is None:
            return value
        if isinstance(value, list):
            return " ".join(value)
        elif isinstance(value, bool):
            return value
        else:
            return self.data["format"].format(value)

    def do_defaults(self, variant):
        value = self.default_value(variant)
        if value is None:
            return
        header = self.data["header"].strip()
        if header:
            print("# {}".format(header))
            print("")
        text = self.data["text"]
        if text:
            import textwrap

            tw = textwrap.TextWrapper()
            tw.drop_whitespace = True
            tw.initial_indent = "# "
            tw.subsequent_indent = "# "
            for line in tw.wrap(text):
                print(line)
        print("{} = {}".format(self.data["name"], value))

    def _do_append_test_cppflags(self, conf, name, state):
        conf.env.append_value("TEST_" + name.upper() + "_CPPFLAGS", state)

    def _append_test_cppflags(self, conf, cic, value, arg):
        self._do_append_test_cppflags(conf, arg, value)
        return value

    def _assert_aligned(self, conf, cic, value, arg):
        if value % arg != 0:
            conf.fatal(
                "Value '{}' for option '{}' is not aligned by '{}'".format(
                    value, self.data["name"], arg
                )
            )
        return value

    def _assert_in_interval(self, conf, cic, value, arg):
        if value < arg[0] or value > arg[1]:
            conf.fatal(
                "Value '{}' for option '{}' is not in closed interval [{}, {}]".format(
                    value, self.data["name"], arg[0], arg[1]
                )
            )
        return value

    def _assert_int8(self, conf, cic, value, arg):
        return self._assert_in_interval(conf, cic, value, [-128, 127])

    def _assert_int16(self, conf, cic, value, arg):
        return self._assert_in_interval(conf, cic, value, [-32768, 32767])

    def _assert_int32(self, conf, cic, value, arg):
        return self._assert_in_interval(
            conf, cic, value, [-2147483648, 2147483647]
        )

    def _assert_int64(self, conf, cic, value, arg):
        return self._assert_in_interval(
            conf, cic, value, [-9223372036854775808, 9223372036854775807]
        )

    def _assert_power_of_two(self, conf, cic, value, arg):
        if value <= 0 or (value & (value - 1)) != 0:
            conf.fatal(
                "Value '{}' for option '{}' is not a power of two".format(
                    value, self.data["name"]
                )
            )
        return value

    def _assert_uint8(self, conf, cic, value, arg):
        return self._assert_in_interval(conf, cic, value, [0, 255])

    def _assert_uint16(self, conf, cic, value, arg):
        return self._assert_in_interval(conf, cic, value, [0, 65535])

    def _assert_uint32(self, conf, cic, value, arg):
        return self._assert_in_interval(conf, cic, value, [0, 4294967295])

    def _assert_uint64(self, conf, cic, value, arg):
        return self._assert_in_interval(
            conf, cic, value, [0, 18446744073709551615]
        )

    def _check_cc(self, conf, cic, value, arg):
        result = conf.check_cc(
            fragment=arg["fragment"],
            cflags=arg["cflags"],
            msg="Checking for " + arg["message"],
            mandatory=False,
        )
        return value and result

    def _check_cxx(self, conf, cic, value, arg):
        result = conf.check_cxx(
            fragment=arg["fragment"],
            cxxflags=arg["cxxflags"],
            msg="Checking for " + arg["message"],
            mandatory=False,
        )
        return value and result

    def _define_condition(self, conf, cic, value, arg):
        name = self.data["name"] if arg is None else arg
        conf.define_cond(name, value)
        return value

    def _define(self, conf, cic, value, arg):
        name = self.data["name"] if arg is None else arg
        if value is not None:
            conf.define(name, value)
        else:
            conf.define_cond(name, False)
        return value

    def _define_unquoted(self, conf, cic, value, arg):
        name = self.data["name"] if arg is None else arg
        if value is not None:
            conf.define(name, value, quote=False)
        else:
            conf.define_cond(name, False)
        return value

    def _env_append(self, conf, cic, value, arg):
        name = self.data["name"] if arg is None else arg
        conf.env.append_value(name, value)
        return value

    def _env_assign(self, conf, cic, value, arg):
        name = self.data["name"] if arg is None else arg
        conf.env[name] = value
        return value

    def _env_enable(self, conf, cic, value, arg):
        if value:
            name = self.data["name"] if arg is None else arg
            conf.env.append_value("ENABLE", name)
        return value

    def _find_program(self, conf, cic, value, arg):
        return conf.find_program(value, path_list=cic.path_list)

    def _format_and_define(self, conf, cic, value, arg):
        name = self.data["name"] if arg is None else arg
        if value is not None:
            conf.define(name, self.data["format"].format(value), quote=False)
        else:
            conf.define_cond(name, False)
        return value

    def _get_boolean(self, conf, cic, value, arg):
        name = self.data["name"]
        try:
            value = cic.cp.getboolean(conf.variant, name)
            cic.add_option(name)
        except configparser.NoOptionError:
            value = self.default_value(conf.variant)
        except ValueError as ve:
            conf.fatal(
                "Invalid value for configuration option {}: {}".format(name, ve)
            )
        return value

    def _get_env(self, conf, cic, value, arg):
        return conf.env[arg]

    def _get_integer(self, conf, cic, value, arg):
        name = self.data["name"]
        try:
            value = cic.cp.get(conf.variant, name)
            cic.add_option(name)
        except configparser.NoOptionError:
            value = self.default_value(conf.variant)
            if value is None:
                return value
        try:
            return eval(value)
        except Exception as e:
            conf.fatal(
                "Value '{}' for option '{}' is an invalid integer expression: {}".format(
                    value, name, e
                )
            )

    def _get_string(self, conf, cic, value, arg):
        name = self.data["name"]
        try:
            value = cic.cp.get(conf.variant, name)
            cic.add_option(name)
            value = no_unicode(value)
        except configparser.NoOptionError:
            value = self.default_value(conf.variant)
        return value

    def _script(self, conf, cic, value, arg):
        exec(arg)
        return value

    def _test_state_benchmark(self, conf, name):
        self._do_append_test_cppflags(conf, name, "-DTEST_STATE_BENCHMARK=1")

    def _test_state_exclude(self, conf, name):
        conf.env.append_value("ENABLE", "TEST_" + name.upper() + "_EXCLUDE")

    def _test_state_expected_fail(self, conf, name):
        self._do_append_test_cppflags(
            conf, name, "-DTEST_STATE_EXPECTED_FAIL=1"
        )

    def _test_state_indeterminate(self, conf, name):
        self._do_append_test_cppflags(
            conf, name, "-DTEST_STATE_INDETERMINATE=1"
        )

    def _test_state_user_input(self, conf, name):
        self._do_append_test_cppflags(conf, name, "-DTEST_STATE_USER_INPUT=1")

    def _set_test_state(self, conf, cic, value, arg):
        actions = {
            "benchmark": self._test_state_benchmark,
            "exclude": self._test_state_exclude,
            "expected-fail": self._test_state_expected_fail,
            "indeterminate": self._test_state_indeterminate,
            "user-input": self._test_state_user_input,
        }
        for k, v in arg.items():
            actions[v](conf, k)
        return value

    def _set_value(self, conf, cic, value, arg):
        return arg

    def _split(self, conf, cic, value, arg):
        return value.split()

    def _substitute(self, conf, cic, value, arg):
        if isinstance(value, list):
            return [self.substitute(conf, v) for v in value]
        else:
            return self.substitute(conf, value)

    def do_configure(self, conf, cic):
        actions = {
            "append-test-cppflags": self._append_test_cppflags,
            "assert-aligned": self._assert_aligned,
            "assert-in-interval": self._assert_in_interval,
            "assert-int8": self._assert_int8,
            "assert-int16": self._assert_int16,
            "assert-int32": self._assert_int32,
            "assert-int64": self._assert_int64,
            "assert-power-of-two": self._assert_power_of_two,
            "assert-uint8": self._assert_uint8,
            "assert-uint16": self._assert_uint16,
            "assert-uint32": self._assert_uint32,
            "assert-uint64": self._assert_uint64,
            "check-cc": self._check_cc,
            "check-cxx": self._check_cxx,
            "define-condition": self._define_condition,
            "define": self._define,
            "define-unquoted": self._define_unquoted,
            "env-append": self._env_append,
            "env-assign": self._env_assign,
            "env-enable": self._env_enable,
            "find-program": self._find_program,
            "format-and-define": self._format_and_define,
            "get-boolean": self._get_boolean,
            "get-env": self._get_env,
            "get-integer": self._get_integer,
            "get-string": self._get_string,
            "script": self._script,
            "set-test-state": self._set_test_state,
            "set-value": self._set_value,
            "split": self._split,
            "substitute": self._substitute,
        }
        value = None
        for action in self.data["actions"]:
            for action_arg in action.items():
                value = actions[action_arg[0]](conf, cic, value, action_arg[1])


class ScriptItem(Item):
    def __init__(self, uid, data):
        super(ScriptItem, self).__init__(uid, data)

    def prepare_configure(self, conf, cic):
        script = self.data["prepare-configure"]
        if script:
            exec(script)

    def do_configure(self, conf, cic):
        script = self.data["do-configure"]
        if script:
            exec(script)

    def prepare_build(self, bld, bic):
        script = self.data["prepare-build"]
        if script:
            exec(script)
        return bic

    def do_build(self, bld, bic):
        script = self.data["do-build"]
        if script:
            exec(script)


class ConfigItemContext(object):
    def __init__(self, cp, path_list):
        self.cp = cp
        self.options = set()
        self.path_list = path_list

    def add_option(self, name):
        self.options.add(name.upper())


class BuildItemContext(object):
    def __init__(self, includes, use, ldflags, objects):
        self.includes = includes
        self.use = use
        self.ldflags = ldflags
        self.objects = objects


def is_one_item_newer(ctx, path, mtime):
    try:
        names = os.listdir(path)
    except Exception as e:
        ctx.fatal("Cannot list build specification directory: {}".format(e))
    for name in names:
        path2 = os.path.join(path, name)
        if name.endswith(".yml") and not name.startswith("."):
            mtime2 = os.path.getmtime(path2)
            if mtime <= mtime2:
                return True
        else:
            mode = os.lstat(path2).st_mode
            if stat.S_ISDIR(mode) and is_one_item_newer(ctx, path2, mtime):
                return True
    return False


def must_update_item_cache(ctx, path, cache_file):
    try:
        mtime = os.path.getmtime(cache_file)
        return is_one_item_newer(ctx, path, mtime)
    except:
        return True


def load_from_yaml(load, ctx, data_by_uid, path):
    try:
        names = os.listdir(path)
    except Exception as e:
        ctx.fatal("Cannot list build specification directory: {}".format(e))
    for name in names:
        path2 = os.path.join(path, name)
        if name.endswith(".yml") and not name.startswith("."):
            uid = os.path.basename(name).replace(".yml", "")
            with open(path2, "r") as f:
                data_by_uid[uid] = load(f.read())
        else:
            mode = os.lstat(path2).st_mode
            if stat.S_ISDIR(mode):
                load_from_yaml(load, ctx, data_by_uid, path2)


def load_items_in_directory(ctx, ctors, path):
    f = ctx.path.make_node(
        "build/c4che/" + re.sub(r"[^\w]", "_", path) + ".pickle"
    )
    f.parent.mkdir()
    cache_file = f.abspath()
    data_by_uid = {}
    if must_update_item_cache(ctx, path, cache_file):
        from waflib import Logs

        Logs.warn(
            "Regenerate build specification cache (needs a couple of seconds)..."
        )

        #
        # Do not use a system provided yaml module and instead import it from
        # the project.  This reduces the host system requirements to a simple
        # Python 2.7 or 3 installation without extra modules.
        #
        if sys.version_info[0] == 2:
            yaml_path = "yaml/lib"
        else:
            yaml_path = "yaml/lib3"
        sys.path += [yaml_path]
        from yaml import safe_load

        load_from_yaml(safe_load, ctx, data_by_uid, path)
        with open(cache_file, "wb") as f:
            pickle.dump(data_by_uid, f)
    else:
        with open(cache_file, "rb") as f:
            data_by_uid = pickle.load(f)
    for uid, data in data_by_uid.items():
        if data["type"] == "build":
            items[uid] = ctors[data["build-type"]](uid, data)


def load_items(ctx, specs):
    if items:
        return

    ctors = {
        "ada-test-program": lambda uid, data: AdaTestProgramItem(uid, data),
        "bsp": lambda uid, data: BSPItem(uid, data),
        "config-file": lambda uid, data: ConfigFileItem(uid, data),
        "config-header": lambda uid, data: ConfigHeaderItem(uid, data),
        "test-program": lambda uid, data: TestProgramItem(uid, data),
        "group": lambda uid, data: GroupItem(uid, data),
        "library": lambda uid, data: LibraryItem(uid, data),
        "objects": lambda uid, data: ObjectsItem(uid, data),
        "option": lambda uid, data: OptionItem(uid, data),
        "script": lambda uid, data: ScriptItem(uid, data),
        "start-file": lambda uid, data: StartFileItem(uid, data),
    }

    for path in specs:
        load_items_in_directory(ctx, ctors, path)


def load_items_from_options(ctx):
    specs = ctx.options.rtems_specs
    if specs is not None:
        specs = specs.split(",")
    else:
        specs = ["spec/build"]
    load_items(ctx, specs)
    return specs


def options(ctx):
    prefix = ctx.parser.get_option("--prefix")
    prefix.default = default_prefix
    prefix.help = "installation prefix [default: '{}']".format(default_prefix)
    rg = ctx.add_option_group("RTEMS options")
    rg.add_option(
        "--rtems-bsps",
        metavar="REGEX,...",
        help="a comma-separated list of Python regular expressions which select the desired BSP variants (e.g. 'sparc/erc32'); it may be used in the bsp_defaults and bsp_list commands",
    )
    rg.add_option(
        "--rtems-compiler",
        metavar="COMPILER",
        help="determines which compiler is used to list the BSP option defaults [default: 'gcc']; it may be used in the bsp_defaults command; valid compilers are: {}".format(
            ", ".join(compilers)
        ),
    )
    rg.add_option(
        "--rtems-config",
        metavar="CONFIG.INI,...",
        help="a comma-separated list of paths to the BSP configuration option files [default: 'config.ini']; default option values can be obtained via the bsp_defaults command; it may be used in the configure command",
    )
    rg.add_option(
        "--rtems-specs",
        metavar="SPECDIRS,...",
        help="a comma-separated list of directory paths to build specification items [default: 'spec/build']; it may be used in the bsp_defaults, bsp_list, and configure commands",
    )
    rg.add_option(
        "--rtems-tools",
        metavar="PREFIX,...",
        help="a comma-separated list of prefix paths to tools, e.g. compiler, linker, etc. [default: the installation prefix]; tools are searched in the prefix path and also in a 'bin' subdirectory of the prefix path; it may be used in the configure command",
    )


def check_environment(conf):
    for ev in [
        "AR",
        "AS",
        "ASFLAGS",
        "CC",
        "CFLAGS",
        "CPPFLAGS",
        "CXX",
        "CXXFLAGS",
        "IFLAGS",
        "LD",
        "LIB",
        "LINK_CC",
        "LINK_CXX",
        "LINKFLAGS",
        "MFLAGS",
        "RFLAGS",
        "WFLAGS",
    ]:
        if ev in os.environ:
            conf.msg("Environment variable set", ev, color="RED")


def load_config_files(ctx):
    cp = configparser.ConfigParser()
    files = ctx.options.rtems_config
    if files is not None:
        files = files.split(",")
    else:
        files = ["config.ini"]
    actual_files = cp.read(files)
    for o in files:
        if not o in actual_files:
            ctx.fatal("Option file '{}' was not readable".format(o))
    return cp


def inherit(conf, cp, bsp_map, arch, bsp, path):
    variant = arch + "/" + bsp
    if variant in path:
        path = " -> ".join(path + [variant])
        conf.fatal("Recursion in BSP options inheritance: {}".format(path))

    try:
        base = cp.get(variant, "INHERIT")
        cp.remove_option(variant, "INHERIT")
        base = no_unicode(base)
        base_variant = arch + "/" + base
        conf.msg(
            "Inherit options from '{}'".format(base_variant),
            variant,
            color="YELLOW",
        )
        if not cp.has_section(base_variant):
            if (not arch in bsps) or (not base in bsps[arch]):
                conf.fatal(
                    "BSP variant '{}' cannot inherit options from not existing variant '{}'".format(
                        variant, base_variant
                    )
                )
            bsp_map[bsp] = base
            return base
        top = inherit(conf, cp, bsp_map, arch, base, path + [variant])
        for i in cp.items(base_variant):
            name = i[0]
            if not cp.has_option(variant, name):
                cp.set(variant, name, i[1])
        bsp_map[bsp] = top
        return top
    except configparser.NoOptionError:
        return bsp_map.get(bsp, bsp)


def resolve_option_inheritance(conf, cp):
    bsp_map = {}
    for variant in cp.sections():
        variant = no_unicode(variant)
        try:
            arch, bsp = variant.split("/")
        except:
            conf.fatal(
                "Section name '{}' is a malformed 'arch/bsp' tuple".format(
                    variant
                )
            )
        inherit(conf, cp, bsp_map, arch, bsp, [])
    return bsp_map


def check_compiler(ctx, compiler):
    if compiler not in compilers:
        ctx.fatal(
            "Specified compiler '{}' is not one of {}".format(
                compiler, compilers
            )
        )


def get_compiler(conf, cp, variant):
    try:
        value = cp.get(conf.variant, "COMPILER")
        cp.remove_option(conf.variant, "COMPILER")
        value = no_unicode(value)
        check_compiler(conf, value)
    except configparser.NoOptionError:
        value = "gcc"
    return value


def configure_variant(conf, cp, bsp_map, path_list, variant):
    conf.msg("Configure board support package (BSP)", variant, color="YELLOW")

    conf.setenv(variant)
    arch, bsp_name = variant.split("/")
    bsp_base = bsp_map.get(bsp_name, bsp_name)

    conf.env["ARCH"] = arch
    conf.env["BSP_BASE"] = bsp_base
    conf.env["BSP_NAME"] = bsp_name
    conf.env["DEST_OS"] = "rtems"
    conf.env["ENABLE"] = [get_compiler(conf, cp, variant), arch, bsp_base]
    conf.env["TOP"] = conf.path.abspath()
    conf.env["VARIANT"] = variant

    cic = ConfigItemContext(cp, path_list)

    for group in top_level_groups:
        group.configure(conf, cic)

    try:
        bsp_item = bsps[arch][bsp_base]
    except KeyError:
        conf.fatal("No such base BSP: '{}'".format(variant))
    bsp_item.configure(conf, cic)

    options = set([o[0].upper() for o in cp.items(variant)])
    for o in options.difference(cic.options):
        conf.msg("Unknown configuration option", o.upper(), color="RED")


def check_forbidden_options(ctx, opts):
    for o in opts:
        if getattr(ctx.options, "rtems_" + o):
            ctx.fatal(
                "The --rtems-{} command line option is not allowed in the {} command".format(
                    o, ctx.cmd
                )
            )


def get_path_list(conf):
    path_list = []
    tools = conf.options.rtems_tools
    if tools is not None:
        for t in tools.split(","):
            path_list.extend([t + "/bin", t])
    path_list.append(conf.env.PREFIX + "/bin")
    path_list.extend(os.environ.get("PATH", "").split(os.pathsep))
    return path_list


def configure(conf):
    check_forbidden_options(conf, ["compiler"])
    check_environment(conf)
    conf.env["SPECS"] = load_items_from_options(conf)
    cp = load_config_files(conf)
    bsp_map = resolve_option_inheritance(conf, cp)
    path_list = get_path_list(conf)
    variant_list = []
    for variant in cp.sections():
        variant = no_unicode(variant)
        variant_list.append(variant)
        configure_variant(conf, cp, bsp_map, path_list, variant)
    conf.setenv("")
    conf.env["VARIANTS"] = variant_list


def append_variant_builds(bld):
    import waflib.Options
    from waflib.Build import (
        BuildContext,
        CleanContext,
        InstallContext,
        UninstallContext,
    )

    for var in bld.env["VARIANTS"]:
        for c in (BuildContext, CleanContext, InstallContext, UninstallContext):
            name = c.__name__.replace("Context", "").lower()

            class magic(c):
                cmd = name + "_" + var
                variant = var

        waflib.Options.commands.append(bld.cmd + "_" + var)


def long_command_line_workaround(bld):
    if is_windows_host:
        bld.load("long_gcc")


def build(bld):
    if not bld.variant:
        check_forbidden_options(bld, ["compiler", "config", "specs", "tools"])
        load_items(bld, bld.env.SPECS)
        append_variant_builds(bld)
        return
    long_command_line_workaround(bld)
    bic = BuildItemContext(bld.env.ARCH_INCLUDES.split(), [], [], [])
    bsps[bld.env.ARCH][bld.env.BSP_BASE].build(bld, bic)
    for group in top_level_groups:
        group.build(bld, bic)


def add_log_filter(name):
    msg = "'" + name + "' finished successfully"

    class Filter:
        def filter(self, rec):
            return not msg in rec.getMessage()

    import logging

    logging.getLogger("waflib").addFilter(Filter())


def get_white_list(ctx):
    white_list = ctx.options.rtems_bsps
    if white_list:
        white_list = white_list.split(",")
    return white_list


def is_in_white_list(variant, white_list):
    if not white_list:
        return True
    for pattern in white_list:
        if re.search(pattern, variant):
            return True
    return False


def no_matches_error(ctx, white_list):
    if white_list:
        ctx.fatal(
            "No BSP matches with the specified patterns: '{}'".format(
                "', '".join(white_list)
            )
        )
    else:
        ctx.fatal("The build specification contains no BSPs")


def bsp_defaults(ctx):
    """get all options with default values for base BSP variants"""
    check_forbidden_options(ctx, ["config", "tools"])
    add_log_filter(ctx.cmd)
    load_items_from_options(ctx)
    white_list = get_white_list(ctx)
    compiler = ctx.options.rtems_compiler
    if compiler is not None:
        check_compiler(ctx, compiler)
    else:
        compiler = "gcc"
    enable = [compiler]
    first = True
    for arch in sorted(bsps):
        for bsp in sorted(bsps[arch]):
            variant = arch + "/" + bsp
            if is_in_white_list(variant, white_list):
                if not first:
                    print("")
                first = False
                print("[{}]".format(variant))
                for group in top_level_groups:
                    group.defaults(enable, variant)
                bsps[arch][bsp].defaults(enable, variant)
    if first:
        no_matches_error(ctx, white_list)


def bsp_list(ctx):
    """lists base BSP variants"""
    check_forbidden_options(ctx, ["compiler", "config", "tools"])
    add_log_filter(ctx.cmd)
    load_items_from_options(ctx)
    white_list = get_white_list(ctx)
    first = True
    for arch in sorted(bsps):
        for bsp in sorted(bsps[arch]):
            variant = arch + "/" + bsp
            if is_in_white_list(variant, white_list):
                first = False
                print(variant)
    if first:
        no_matches_error(ctx, white_list)