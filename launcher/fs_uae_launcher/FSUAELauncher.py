from __future__ import division
from __future__ import print_function
from __future__ import absolute_import
from __future__ import unicode_literals

import os
import sys
import shutil
import hashlib
import ConfigParser
import fs_uae_launcher.fsui as fsui
from .ui.MainWindow import MainWindow
import fs_uae_launcher.fs as fs
from .Config import Config
from .ConfigurationScanner import ConfigurationScanner
from .Database import Database
from .Paths import Paths
from .ROMManager import ROMManager
from .Settings import Settings
from .UpdateManager import UpdateManager

class FSUAELauncher(fsui.Application):

    def on_create(self):
        print("FSUAELauncherApplication.on_create")

        self.parse_arguments()
        self.load_settings()
        self.config_startup_scan()
        self.kickstart_startup_scan()

        # FIXME: should now sanitize check some options -for instance,
        # - check if configured joysticks are still connected
        # - check if paths still exists, etc

        #Config.update_kickstart()

        icon = None
        def check_icon(path):
            path = os.path.join(path, "fs-uae-launcher.ico")
            if os.path.exists(path):
                return path
            return None
        if not icon:
            icon = check_icon("share/fs-uae-launcher")
        if not icon:
            icon = check_icon("launcher/share/fs-uae-launcher")
        # FIXME: should check data directories (XDG_DATA_DIRS) properly
        # instead
        if not icon:
            icon = check_icon(os.path.expanduser(
                    "~/.local/share/fs-uae-launcher"))
        if not icon:
            icon = check_icon("/usr/local/share/fs-uae-launcher")
        if not icon:
            icon = check_icon("/usr/share/fs-uae-launcher")
        if fs.macosx:
            # Icons come from the app bundles
            icon = None

        window = MainWindow(icon=icon)
        MainWindow.instance = window
        window.show()
        UpdateManager.run_update_check()

    def get_settings_file(self):
        return os.path.join(fs.get_app_data_dir(), "launcher.settings")

    def load_settings(self):
        path = self.get_settings_file()
        print("loading settings from " + repr(path))
        if not os.path.exists(path):
            print("settings file does not exist")
        cp = ConfigParser.ConfigParser()
        try:
            cp.read([path])
        except Exception, e:
            print(repr(e))
            return

        config = {}
        try:
            keys = cp.options("config")
        except ConfigParser.NoSectionError:
            keys = []
        for key in keys:
            config[key] = cp.get("config", key)
        for key, value in config.iteritems():
            print("loaded", key, value)
            Config.config[key] = value

        settings = {}
        try:
            keys = cp.options("settings")
        except ConfigParser.NoSectionError:
            keys = []
        for key in keys:
            settings[key] = cp.get("settings", key)
        for key, value in settings.iteritems():
            #if key in Settings.settings:
            #    # this setting is already initialized, possibly via
            #    # command line arguments
            #    pass
            #else:
            print("-- setting", key, value)
            Settings.settings[key] = value

    def parse_arguments(self):
        pass
        #for arg in sys.argv:
        #    if arg.startswith("--"):
        #        if "=" in arg:
        #            key, value = arg[2:].split("=", 1)
        #            key = key.replace("-", "_")
        #            if key == "base_dir":
        #                Settings.set("base_dir", value)

    def save_settings(self):
        path = self.get_settings_file()
        path = path + u".part"
        print("writing " + repr(path))

        cp = ConfigParser.ConfigParser()
        cp.add_section("settings")

        for key, value in Settings.settings.iteritems():
            #lines.append(u"{0} = {1}".format(key, value))
            cp.set("settings", key, value)

        cp.add_section("config")
        #lines.append(u"[config]")

        for key, value in Config.config.iteritems():
            if key.startswith("__"):
                # keys starting with __ are never saved
                continue
            #lines.append(u"{0} = {1}".format(key, value))
            cp.set("config", key, value)

        with open(path, "wb") as f:
            cp.write(f)
            #for line in lines:
            #    f.write(line + u"\n".encode("UTF-8"))
        print("moving to " + repr(self.get_settings_file()))
        shutil.move(path, self.get_settings_file())

    def get_dir_mtime_str(self, path):
        try:
            return str(int(os.path.getmtime(path)))
        except Exception:
            return "0"

    def config_startup_scan(self):
        print("config_startup_scan")
        configs_dir = Settings.get_configurations_dir()
        if Settings.get("configurations_dir_mtime") == \
                self.get_dir_mtime_str(configs_dir):
            print("... mtime not changed")
            return
        database = Database.get_instance()
        print("... database.find_local_configurations")
        local_configs = database.find_local_configurations()
        print("... walk configs_dir")
        for dir_path, dir_names, file_names in os.walk(configs_dir):
            for file_name in file_names:
                if not file_name.endswith(".fs-uae"):
                    continue
                path = Paths.join(dir_path, file_name)
                if path in local_configs:
                    local_configs[path] = None
                    # already exists in database
                    continue
                name, ext = os.path.splitext(file_name)
                search = ConfigurationScanner.create_configuration_search(name)
                name = ConfigurationScanner.create_configuration_name(name)
                print("[startup] adding config", path)
                database.add_configuration(path=path, uuid="", name=name,
                        scan=0, search=search)
        for path, id in local_configs.iteritems():
            if id is not None:
                print("[startup] removing configuration", path)
                database.delete_configuration(id=id)
        print("... commit")
        database.commit()
        Settings.set("configurations_dir_mtime",
                self.get_dir_mtime_str(configs_dir))

    def kickstart_startup_scan(self):
        print("kickstart_startup_scan")
        kickstarts_dir = Settings.get_kickstarts_dir()
        if Settings.get("kickstarts_dir_mtime") == \
                self.get_dir_mtime_str(kickstarts_dir):
            print("... mtime not changed")
            return
        database = Database.get_instance()
        print("... database.find_local_roms")
        local_roms = database.find_local_roms()
        print("... walk kickstarts_dir")
        for dir_path, dir_names, file_names in os.walk(kickstarts_dir):
            for file_name in file_names:
                if not file_name.endswith(".rom"):
                    continue
                path = Paths.join(dir_path, file_name)
                if path in local_roms:
                    local_roms[path] = None
                    # already exists in database
                    continue
                print("[startup] adding kickstart", path)
                ROMManager.add_rom_to_database(path, database)
        print(local_roms)
        for path, id in local_roms.iteritems():
            if id is not None:
                print("[startup] removing kickstart", path)
                database.delete_file(id=id)
        print("... commit")
        database.commit()
        Settings.set("kickstarts_dir_mtime",
                self.get_dir_mtime_str(kickstarts_dir))
