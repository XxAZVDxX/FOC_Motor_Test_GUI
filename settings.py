# -*- coding: utf-8 -*-
"""Persistence for user interface preferences (theme + language + zoom +
last motor speed/position commands).

Stored in ``config/settings.json`` next to the motor configuration files.
``ConfigManager.list_configs`` skips this file so it never shows up in the
configuration dropdown.
"""

import json
import os

import i18n
import theme

SETTINGS_DIR = "config"
SETTINGS_FILE = "settings.json"
SETTINGS_NAME = os.path.splitext(SETTINGS_FILE)[0]

_DEFAULTS = {
    "theme": theme.DEFAULT_THEME,
    "language": i18n.DEFAULT_LANGUAGE,
    "zoom": theme.DEFAULT_ZOOM,
    "speed_cmd": 0.0,
    "position_cmd": 0.0,
}

# 与 Motor Control 页签里输入框的上限保持一致
CMD_LIMIT = 100000.0


def settings_path(directory=SETTINGS_DIR):
    """Full path of the settings file for ``directory``."""
    return os.path.join(directory, SETTINGS_FILE)


def load(directory=SETTINGS_DIR):
    """Return the stored preferences, falling back to defaults.

    A missing or malformed file is not an error - the defaults are returned so
    the application always starts.
    """
    values = dict(_DEFAULTS)
    try:
        with open(settings_path(directory), "r", encoding="utf-8") as handle:
            stored = json.load(handle)
    except (OSError, ValueError):
        return values

    if not isinstance(stored, dict):
        return values

    valid_themes = {code for code, _ in theme.THEMES}
    if stored.get("theme") in valid_themes:
        values["theme"] = stored["theme"]
    if stored.get("language") in i18n.LANGUAGE_CODES:
        values["language"] = stored["language"]
    if isinstance(stored.get("zoom"), (int, float)) and not isinstance(
        stored.get("zoom"), bool
    ):
        values["zoom"] = theme.clamp_zoom(stored["zoom"])
    for key in ("speed_cmd", "position_cmd"):
        raw = stored.get(key)
        if isinstance(raw, (int, float)) and not isinstance(raw, bool):
            values[key] = max(-CMD_LIMIT, min(CMD_LIMIT, float(raw)))
    return values


def save(values, directory=SETTINGS_DIR):
    """Write ``values`` to the settings file.  Returns True on success."""
    path = settings_path(directory)
    try:
        os.makedirs(directory, exist_ok=True)
        with open(path, "w", encoding="utf-8") as handle:
            json.dump(values, handle, indent=2, ensure_ascii=False)
        return True
    except OSError:
        return False
