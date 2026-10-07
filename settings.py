# -*- coding: utf-8 -*-
"""Persistence for user interface preferences (theme + language).

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
}


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
