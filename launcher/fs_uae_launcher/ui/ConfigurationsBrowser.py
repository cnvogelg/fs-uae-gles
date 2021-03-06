from __future__ import division
from __future__ import print_function
from __future__ import absolute_import
from __future__ import unicode_literals

import os
import traceback
import fs_uae_launcher.fsui as fsui
from ..Config import Config
from ..Settings import Settings
from ..Database import Database
from ..I18N import _, ngettext

class ConfigurationsBrowser(fsui.VerticalItemView):

    def __init__(self, parent):
        fsui.VerticalItemView.__init__(self, parent)
        self.items = []
        self.icon = fsui.Image("fs_uae_launcher:res/fsuae_config_16.png")

        Settings.add_listener(self)
        self.update_search()

    def on_destroy(self):
        Settings.remove_listener(self)

    def on_select_item(self, index):
        self.load_configuration(self.items[index][0])

    def on_activate_item(self, index):
        from ..LaunchHandler import LaunchHandler
        LaunchHandler.start_game()

    def on_setting(self, key, value):
        if key == "config_search":
            self.update_search()
        if key == "config_refresh":
            self.update_search()

    def set_items(self, items):
        self.items = items
        #self.set_item_count(len(self.items))
        self.update()

    def get_item_count(self):
        return len(self.items)

    def get_item_text(self, index):
        return self.items[index][1]

    def get_item_icon(self, index):
        return self.icon

    #def on_get_item_tooltip(self, row, column):
    #    return self.items[row][1]
    #    #text = text.replace(u"\nAmiga \u00b7 ", "\n")

    def update_search(self):
        self.search = Settings.get("config_search").strip().lower()
        print("search for", self.search)

        database = Database.get_instance()
        items = database.search_configurations(self.search)
        #print(items)
        self.set_items(items)
        #self.set_items([list(x) for x in items])

    def load_configuration(self, configuration_id):
        database = Database.get_instance()
        config_info = database.get_config(configuration_id)
        if config_info["data"]:
            Config.load_data(config_info["data"])
        else:
            Config.load_file(config_info["path"])
