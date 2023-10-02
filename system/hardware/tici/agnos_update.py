#!/usr/bin/env python3
import os
import re
import datetime
import subprocess
import psutil
import shutil
import signal
import fcntl
import time
import threading
from collections import defaultdict
from pathlib import Path
from markdown_it import MarkdownIt

from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.common.time import system_time_valid
from openpilot.system.hardware import AGNOS, HARDWARE
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.alertmanager import set_offroad_alert
from openpilot.system.version import get_build_metadata

from openpilot.selfdrive.updated.updated import handle_agnos_update, OVERLAY_MERGED



handle_agnos_update()
