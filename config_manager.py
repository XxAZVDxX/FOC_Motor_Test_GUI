# -*- coding: utf-8 -*-

import os
import json


class ConfigManager:
    """Manages motor configuration JSON files: load, save, import, export."""

    def __init__(self, config_dir="./config"):
        self.config_dir = config_dir
        self._current_path = None  # path to the currently active config file
        if not os.path.exists(self.config_dir):
            os.makedirs(self.config_dir)

    # ── path helpers ──────────────────────────────────────────────

    @property
    def current_path(self):
        """Absolute path to the currently loaded/saved config, or None."""
        return self._current_path

    @property
    def current_name(self):
        """Display name of the current config (filename or 'Unsaved')."""
        if self._current_path:
            return os.path.basename(self._current_path)
        return "Unsaved"

    # ── file listing ──────────────────────────────────────────────

    def list_configs(self):
        """Return sorted list of .json filenames in the config directory."""
        if not os.path.exists(self.config_dir):
            return []
        files = [f for f in os.listdir(self.config_dir) if f.endswith('.json')]
        files.sort()
        return files

    # ── collect from GUI ──────────────────────────────────────────

    def collect_from_ui(self, main_window):
        """Build a config dict from the current state of the GUI widgets."""
        cfg = {
            'targets': {
                'iq': main_window.target_iq.value(),
                'id': main_window.target_id.value(),
                'speed': main_window.target_speed.value(),
                'position': main_window.target_position.value(),
                'uq': main_window.target_uq.value(),
                'ud': main_window.target_ud.value()
            },
            'pid': {},
            'limits': {
                'iq_max': main_window.limit_iq_max.value(),
                'iq_min': main_window.limit_iq_min.value(),
                'id_max': main_window.limit_id_max.value(),
                'id_min': main_window.limit_id_min.value(),
                'speed_max': main_window.limit_speed_max.value(),
                'speed_min': main_window.limit_speed_min.value(),
                'position_max': main_window.limit_position_max.value(),
                'position_min': main_window.limit_position_min.value()
            },
            'gear_ratio': main_window.gear_ratio_edit.text()
        }
        for name in ['Iq', 'Id', 'Speed', 'Position']:
            p, i, d, _, _ = main_window.pid_widgets[name]
            cfg['pid'][name] = {'p': p.value(), 'i': i.value(), 'd': d.value()}
        return cfg

    # ── apply to GUI ──────────────────────────────────────────────

    def apply_to_ui(self, main_window, cfg):
        """Push config dict values into the GUI widgets."""
        targets = cfg.get('targets', {})
        main_window.target_iq.setValue(targets.get('iq', 0))
        main_window.target_id.setValue(targets.get('id', 0))
        main_window.target_speed.setValue(targets.get('speed', 0))
        main_window.target_position.setValue(targets.get('position', 0))
        main_window.target_uq.setValue(targets.get('uq', 0))
        main_window.target_ud.setValue(targets.get('ud', 0))

        pid = cfg.get('pid', {})
        for name in ['Iq', 'Id', 'Speed', 'Position']:
            if name in pid:
                p, i, d, _, _ = main_window.pid_widgets[name]
                p.setValue(pid[name].get('p', 0))
                i.setValue(pid[name].get('i', 0))
                d.setValue(pid[name].get('d', 0))

        limits = cfg.get('limits', {})
        main_window.limit_iq_max.setValue(limits.get('iq_max', 0))
        main_window.limit_iq_min.setValue(limits.get('iq_min', 0))
        main_window.limit_id_max.setValue(limits.get('id_max', 0))
        main_window.limit_id_min.setValue(limits.get('id_min', 0))
        main_window.limit_speed_max.setValue(limits.get('speed_max', 0))
        main_window.limit_speed_min.setValue(limits.get('speed_min', 0))
        main_window.limit_position_max.setValue(limits.get('position_max', 0))
        main_window.limit_position_min.setValue(limits.get('position_min', 0))

        main_window.gear_ratio_edit.setText(cfg.get('gear_ratio', '1 : 1'))

    # ── load ──────────────────────────────────────────────────────

    def load_from_file(self, path):
        """Load a JSON config file, return the dict.  Raises on error."""
        with open(path, 'r', encoding='utf-8') as f:
            cfg = json.load(f)
        self._current_path = os.path.abspath(path)
        return cfg

    def load_and_apply(self, main_window, path):
        """Load a config file and apply it to the GUI."""
        cfg = self.load_from_file(path)
        self.apply_to_ui(main_window, cfg)
        return cfg

    # ── save ──────────────────────────────────────────────────────

    def save_to_file(self, cfg, path):
        """Write config dict to a JSON file.  Raises on error."""
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(cfg, f, indent=4)
        self._current_path = os.path.abspath(path)

    def save_current(self, main_window, path):
        """Collect current UI state and save to path."""
        cfg = self.collect_from_ui(main_window)
        self.save_to_file(cfg, path)

    # ── import (copy external file into config dir) ───────────────

    def import_file(self, src_path):
        """Copy an external JSON file into the config directory.
        Returns the destination path.  Raises on error."""
        basename = os.path.basename(src_path)
        dst = os.path.join(self.config_dir, basename)
        # avoid overwriting – append a number if needed
        if os.path.exists(dst):
            base, ext = os.path.splitext(basename)
            counter = 1
            while os.path.exists(dst):
                dst = os.path.join(self.config_dir, f"{base}_{counter}{ext}")
                counter += 1
        with open(src_path, 'r', encoding='utf-8') as fsrc:
            cfg = json.load(fsrc)
        with open(dst, 'w', encoding='utf-8') as fdst:
            json.dump(cfg, fdst, indent=4)
        return dst
