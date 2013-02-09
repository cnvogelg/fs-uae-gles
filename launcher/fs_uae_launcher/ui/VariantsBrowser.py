from __future__ import division
from __future__ import print_function
from __future__ import absolute_import
from __future__ import unicode_literals

import os
import traceback
import fs_uae_launcher.fsui as fsui
from ..Config import Config
from ..Signal import Signal
from ..Settings import Settings
from ..Database import Database
from ..I18N import _, ngettext
from ..fsgs.GameDatabase import GameDatabase
from ..fsgs.GameDatabaseClient import GameDatabaseClient
from .Constants import Constants

class LastVariants:

    def __init__(self):
        self.cache = {}
        Signal.add_listener("quit", self)

    def on_quit_signal(self):
        database = Database.get_instance()
        for key, value in self.cache.iteritems():
            database.set_last_game_variant(key, value)
        database.commit()

class VariantsBrowser(fsui.VerticalItemView):

    @staticmethod
    def use_horizontal_layout():
        return fsui.get_screen_size()[0] > 1024

    def __init__(self, parent):
        fsui.VerticalItemView.__init__(self, parent)
        #fsui.Group.__init__(self, parent)
        #self.layout = fsui.HorizontalLayout()
        #self.combo = fsui.ComboBox(self)
        #self.layout.add(self.combo, expand=True, fill=True)
        self.game_uuid = ""
        self.items = []
        self.last_variants = LastVariants()

        self.icon = fsui.Image("fs_uae_launcher:res/fsuae_config_16.png")
        self.adf_icon = fsui.Image("fs_uae_launcher:res/adf_game_16.png")
        self.ipf_icon = fsui.Image("fs_uae_launcher:res/ipf_game_16.png")
        self.cd_icon = fsui.Image("fs_uae_launcher:res/cd_game_16.png")
        self.hd_icon = fsui.Image("fs_uae_launcher:res/hd_game_16.png")

        Settings.add_listener(self)
        self.on_setting("parent_uuid", Settings.get("parent_uuid"))

    def on_destroy(self):
        Settings.remove_listener(self)
        #Signal.remove_listener("quit", self)

    def on_select_item(self, index):
        self.load_variant(self.items[index][1])
        variant_uuid = self.items[index][3]
        self.last_variants.cache[self.parent_uuid] = variant_uuid

        #database = Database.get_instance()
        #database.set_last_game_variant(self.game_uuid, variant_uuid)
        #database.commit()

    def on_activate_item(self, index):
        from ..LaunchHandler import LaunchHandler
        LaunchHandler.start_game()

    def on_setting(self, key, value):
        if key == "parent_uuid":
            self.parent_uuid = value
            if value:
                self.update_list(value)
            else:
                Settings.set("game_uuid", "")
                self.set_items([])

    def set_items(self, items):
        self.items = items
        self.update()

    def get_item_count(self):
        return len(self.items)

    def get_item_text(self, index):
        return self.items[index][2]

    def get_item_icon(self, index):
        name = self.items[index][2]
        if "IPF" in name:
            return self.ipf_icon
        if "ADF" in name:
            return self.adf_icon
        if "WHDLoad" in name:
            return self.hd_icon
        if "CD32" in name:
            return self.cd_icon
        if "CDTV" in name:
            return self.cd_icon
        return self.icon

    def update_list(self, game_uuid):
        #print("--- UPDATE LIST ---")
        #self.search = Settings.get("config_search").strip().lower()
        #print("search for", self.search)

        database = Database.get_instance()
        items = database.find_game_variants(game_uuid)
        #items = database.search_configurations(self.search)
        self.items = []
        for item in items:
            name = item[1]

            name = name.replace(u"\nAmiga \u00b7 ", "\n")
            #name = name.replace(u"\nCD32 \u00b7 ", "\n")
            #name = item[1].replace("\n", " \u00b7 ")

            # only show variant name (without game name)
            name = name.split("\n", 1)[-1]
            sort_key = (1000000 - item[3], 1000000 - item[4], name)
            self.items.append((sort_key, item[0], name, item[2], item[5]))
        self.items.sort()
        self.set_items(self.items)

        for i, item in enumerate(self.items):
            if item[4] == 5:
                self.select_item(i)
                break
        else:
            if len(self.items) > 0:
                self.select_item(0)
        """
        try:
            variant_uuid = self.last_variants.cache[game_uuid]
        except KeyError:
            variant_uuid = database.get_last_game_variant(game_uuid)
        if len(self.items) > 0:
            for i, item in enumerate(self.items):
                if item[3] == variant_uuid:
                    self.select_item(i)
                    break
            else:
                self.select_item(0)
        """

    def load_variant(self, configuration_id):
        database = Database.get_instance()
        config_info = database.get_config(configuration_id)
        print(config_info)

        game_uuid = config_info["uuid"]
        game_database = GameDatabase.get_instance()
        game_database_client = GameDatabaseClient(game_database)
        try:
            game_id = game_database_client.get_game_id(game_uuid)
        except Exception:
            # game not found:
            print("could not find game", repr(game_uuid))
            Config.load_default_config()
            return
        values = game_database_client.get_final_game_values(game_id)
        
        print(values)
        Config.load_values(values, uuid=game_uuid)

        variant_rating = 0
        if config_info["work_rating"] is not None:
            variant_rating = config_info["work_rating"] - 2
        if config_info["like_rating"]:
            variant_rating = config_info["like_rating"]
        Config.set("__variant_rating", str(variant_rating))
        Settings.set("config_changed", "0")
