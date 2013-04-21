from __future__ import division
from __future__ import print_function
from __future__ import absolute_import
from __future__ import unicode_literals

import os
import time
import hashlib
import weakref
import urllib2
import threading
import traceback
import fs_uae_launcher.fsui as fsui
from ..Settings import Settings
from ..Signal import Signal

SENTINEL = "3e31297d-2ae9-4014-9247-2495d40e5382"

class ImageLoader:

    instance = None

    @classmethod
    def get(cls):
        if not cls.instance:
            cls.instance = cls(SENTINEL)
        return cls.instance

    def __init__(self, sentinel):
        assert sentinel == SENTINEL
        self.stop_flag = False
        self.requests_lock = threading.Lock()
        self.requests = []
        threading.Thread(target=self.image_loader_thread).start()

        Signal.add_listener("quit", self)

    def on_quit_signal(self):
        print("ImageLoader.on_quit_signal")
        self.stop_flag = True

    def image_loader_thread(self):
        try:
            self._image_loader_thread()
        except Exception:
            traceback.print_exc()

    def load_image(self, path="", sha1="", size=None, on_load=None, **kwargs):
        request = ImageLoadRequest()
        request.path = path
        request.sha1 = sha1
        request.image = None
        request.size = size
        request.on_load = on_load
        #for key, value in kwargs.iteritems:
        #    setattr(request.data, key, value)
        request.args = kwargs
        with self.requests_lock:
            self.requests.append(weakref.ref(request))
        return request

    def _image_loader_thread(self):
    	while not self.stop_flag:
            #self.condition.wait()

            request = None
            with self.requests_lock:
                while len(self.requests) > 0:
                    request = self.requests.pop(0)()
                    if request is not None:
                        break
            if request:
                self.fill_request(request)
                request.notify()
            # FIXME: replace with condition
            time.sleep(0.01)

    def fill_request(self, request):
        try:
            self._fill_request(request)
        except Exception:
            traceback.print_exc()

    def get_cache_path_for_sha1(self, request, sha1):
        #print("get_cache_path_for_sha1", sha1)
        if request.args.get("is_cover", False):
            size_arg = "?size={0}".format(256)
            cache_ext = "_{0}".format(256)
        elif request.size:
            size_arg = "?w={0}&h={1}".format(request.size[0],
                    request.size[1])
            cache_ext = "_{0}x{1}".format(request.size[0],
                    request.size[1])
        else:
            size_arg = ""
            cache_ext = ""

        cache_dir = os.path.join(Settings.get_cache_dir(),
                "Images", sha1[:3])
        if not os.path.exists(cache_dir):
            os.makedirs(cache_dir)
        cache_file = os.path.join(cache_dir, sha1 + cache_ext)
        if os.path.exists(cache_file):
            # an old bug made it possible for 0-byte files to exist, so
            # we check for that here..
            if os.path.getsize(cache_file) > 0:
                return cache_file

        try:
            server = os.environ["FS_GAME_DATABASE_SERVER"]
        except KeyError:
            server = "fengestad.no"

        url = "http://fengestad.no/games/image/{0}{1}".format(
                sha1, size_arg)
        r = urllib2.urlopen(url)
        data = r.read()

        with open(cache_file, "wb") as f:
            f.write(data)
        return cache_file

    def _fill_request(self, request):
        if request.path is None:
            return
        path = ""
        if request.path.startswith("sha1:"):
            path = self.get_cache_path_for_sha1(request, request.path[5:])
            double_size = False
        else:
            path = request.path
            double_size = True

        if path:
            print("loading image from", request.path)
            image = fsui.Image(path)
            print(image.size, request.size)
            if request.size is not None:
                dest_size = request.size
            else:
                dest_size = image.size
            if image.size == dest_size:
                request.image = image
                return
            try:
                ratio = image.size[0] / image.size[1]
            except Exception:
                ratio = 1.0
            if ratio > 0.85 and ratio < 1.20:
                min_length = min(request.size)
                dest_size = (min_length, min_length)

            if double_size and image.size[0] < 400:
                image.resize((image.size[0] * 2,
                        image.size[1] * 2),
                        fsui.Image.NEAREST)
            image.resize(dest_size)
            #print(image)
            request.image = image

class ImageLoadRequest:

    def __init__(self):
        self.on_load = None
        self.size = None
        self.args = {}

    def notify(self):
        if self.on_load:
            def on_load_function():
                self.on_load(self)
            fsui.call_after(on_load_function)
