# -*- coding: utf-8 -*-
"""Light / dark theming for the Motor Control GUI.

Everything visual that cannot be expressed in a stylesheet (pyqtgraph plots,
QPainter based widgets, the OpenGL view) reads its colours from the same
palette dictionary, so a theme switch updates the whole application
consistently.
"""

import os
import re

import pyqtgraph as pg
from PyQt5.QtGui import QColor, QFont, QPalette

THEMES = [
    ("light", "Light"),
    ("dark", "Dark"),
]

DEFAULT_THEME = "light"

# ---------- view zoom ----------
# The stylesheet carries every metric, so scaling it scales the whole UI.
# Fractional pt/px values are accepted by Qt, which keeps the steps smooth.
MIN_ZOOM = 50
MAX_ZOOM = 250
ZOOM_STEP = 10
DEFAULT_ZOOM = 100
ZOOM_LEVELS = list(range(MIN_ZOOM, MAX_ZOOM + 1, ZOOM_STEP))
_BASE_FONT_PT = 9.0
_PLOT_FONT_PT = 12.0
_LEGEND_FONT_PT = 9.0
_LENGTH_RE = re.compile(r"(-?\d+(?:\.\d+)?)(px|pt)")

_ASSET_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets")
_CHECK_ICON = os.path.join(_ASSET_DIR, "check.svg").replace("\\", "/")

_PALETTES = {
    "light": {
        # chrome
        "window": "#f3f3f3",
        "surface": "#ffffff",
        "base": "#ffffff",
        "text": "#1f1f1f",
        "text_muted": "#6b6b6b",
        "text_disabled": "#a8a8a8",
        "border": "#c8c8c8",
        "border_soft": "#e0e0e0",
        "accent": "#0078d4",
        "accent_soft": "#e5f1fb",
        "accent_text": "#ffffff",
        "button": "#fbfbfb",
        "button_hover": "#eef6fd",
        "button_pressed": "#dbe9f5",
        "button_disabled": "#f0f0f0",
        "tab_inactive": "#e4e4e4",
        "scrollbar": "#c1c1c1",
        "scrollbar_hover": "#a4a4a4",
        # semantic
        "phase_a": "#d32f2f",
        "phase_b": "#2e7d32",
        "phase_c": "#1565c0",
        "status_ok": "#2e7d32",
        "status_error": "#d32f2f",
        "status_muted": "#6b6b6b",
        # plots / custom widgets
        "plot_bg": "#ffffff",
        "plot_axis": "#505050",
        "preview_face": "#f0f0f0",
        "preview_edge": "#505050",
        "preview_pointer": "#c83232",
        "preview_hub": "#000000",
        "preview_text": "#404040",
        "gl_bg": "w",
    },
    "dark": {
        # chrome
        "window": "#1e1e1e",
        "surface": "#252526",
        "base": "#2d2d30",
        "text": "#e6e6e6",
        "text_muted": "#9a9a9a",
        "text_disabled": "#6b6b6b",
        "border": "#3f3f46",
        "border_soft": "#333338",
        "accent": "#3794ff",
        "accent_soft": "#264f78",
        "accent_text": "#ffffff",
        "button": "#333337",
        "button_hover": "#3f3f46",
        "button_pressed": "#2a2a2e",
        "button_disabled": "#2a2a2e",
        "tab_inactive": "#2a2a2e",
        "scrollbar": "#4a4a52",
        "scrollbar_hover": "#5f5f6b",
        # semantic
        "phase_a": "#ff6b6b",
        "phase_b": "#51cf66",
        "phase_c": "#4dabf7",
        "status_ok": "#51cf66",
        "status_error": "#ff6b6b",
        "status_muted": "#9a9a9a",
        # plots / custom widgets
        "plot_bg": "#1a1a1a",
        "plot_axis": "#b0b0b0",
        "preview_face": "#2d2d30",
        "preview_edge": "#a0a0a0",
        "preview_pointer": "#e05252",
        "preview_hub": "#e6e6e6",
        "preview_text": "#b0b0b0",
        "gl_bg": "k",
    },
}

_QSS = """
QWidget {
    color: %(text)s;
    font-size: 9pt;
}

QMainWindow, QDialog {
    background-color: %(window)s;
}

/* ---------- menu bar ---------- */
QMenuBar {
    background-color: %(surface)s;
    border-bottom: 1px solid %(border)s;
    padding: 2px 4px;
}
QMenuBar::item {
    background: transparent;
    padding: 5px 10px;
    border-radius: 4px;
}
QMenuBar::item:selected {
    background-color: %(accent_soft)s;
}
QMenuBar::item:pressed {
    background-color: %(accent)s;
    color: %(accent_text)s;
}

QMenu {
    background-color: %(surface)s;
    border: 1px solid %(border)s;
    padding: 4px;
}
QMenu::item {
    padding: 5px 28px 5px 26px;
    border-radius: 4px;
}
QMenu::item:selected {
    background-color: %(accent)s;
    color: %(accent_text)s;
}
QMenu::item:disabled {
    color: %(text_disabled)s;
}
QMenu::separator {
    height: 1px;
    background: %(border)s;
    margin: 4px 8px;
}
QMenu::indicator {
    width: 13px;
    height: 13px;
    left: 7px;
}
QMenu::indicator:non-exclusive:unchecked {
    background-color: %(base)s;
    border: 1px solid %(border)s;
    border-radius: 3px;
}
QMenu::indicator:non-exclusive:checked {
    background-color: %(accent)s;
    border: 1px solid %(accent)s;
    border-radius: 3px;
    image: url(%(check_icon)s);
}
QMenu::indicator:non-exclusive:disabled {
    background-color: %(button_disabled)s;
    border-color: %(border_soft)s;
}

/* ---------- tabs ---------- */
QTabWidget::pane {
    border: 1px solid %(border)s;
    background-color: %(surface)s;
    top: -1px;
}
QTabWidget > QWidget {
    background-color: %(surface)s;
}
QTabWidget::tab-bar {
    left: 2px;
}
QTabBar::tab {
    background-color: %(tab_inactive)s;
    color: %(text_muted)s;
    border: 1px solid %(border)s;
    border-bottom: none;
    border-top-left-radius: 5px;
    border-top-right-radius: 5px;
    padding: 6px 14px;
    margin-right: 2px;
}
QTabBar::tab:selected {
    background-color: %(surface)s;
    color: %(accent)s;
    font-weight: bold;
    border-bottom: 2px solid %(accent)s;
}
QTabBar::tab:hover:!selected {
    background-color: %(accent_soft)s;
    color: %(text)s;
}

/* ---------- group boxes ---------- */
QGroupBox {
    background-color: %(surface)s;
    border: 1px solid %(border)s;
    border-radius: 6px;
    margin-top: 11px;
    padding: 10px 8px 8px 8px;
}
QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 10px;
    padding: 0 4px;
    color: %(accent)s;
    font-weight: bold;
}

/* ---------- buttons ---------- */
QPushButton {
    background-color: %(button)s;
    color: %(text)s;
    border: 1px solid %(border)s;
    border-radius: 5px;
    padding: 5px 12px;
    min-height: 17px;
}
QPushButton:hover {
    background-color: %(button_hover)s;
    border-color: %(accent)s;
}
QPushButton:pressed {
    background-color: %(button_pressed)s;
}
QPushButton:default {
    border-color: %(accent)s;
}
QPushButton:disabled {
    background-color: %(button_disabled)s;
    color: %(text_disabled)s;
    border-color: %(border_soft)s;
}

/* ---------- text inputs ---------- */
QLineEdit, QPlainTextEdit, QTextEdit, QSpinBox, QDoubleSpinBox, QComboBox {
    background-color: %(base)s;
    color: %(text)s;
    border: 1px solid %(border)s;
    border-radius: 5px;
    padding: 4px 6px;
    selection-background-color: %(accent)s;
    selection-color: %(accent_text)s;
}
QLineEdit:focus, QPlainTextEdit:focus, QTextEdit:focus,
QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus {
    border: 1px solid %(accent)s;
}
QLineEdit:disabled, QPlainTextEdit:disabled, QTextEdit:disabled,
QSpinBox:disabled, QDoubleSpinBox:disabled, QComboBox:disabled {
    background-color: %(button_disabled)s;
    color: %(text_disabled)s;
    border-color: %(border_soft)s;
}

/* ---------- combo boxes ---------- */
QComboBox::drop-down {
    border: none;
    width: 20px;
}
QComboBox::down-arrow {
    image: none;
    width: 0;
    height: 0;
    margin-right: 7px;
    border-left: 4px solid transparent;
    border-right: 4px solid transparent;
    border-top: 5px solid %(text_muted)s;
}
QComboBox::down-arrow:disabled {
    border-top-color: %(text_disabled)s;
}
QComboBox QAbstractItemView {
    background-color: %(surface)s;
    color: %(text)s;
    border: 1px solid %(border)s;
    outline: none;
    selection-background-color: %(accent)s;
    selection-color: %(accent_text)s;
}

/* ---------- list widgets ---------- */
QListWidget, QListView {
    background-color: %(base)s;
    color: %(text)s;
    border: 1px solid %(border)s;
    border-radius: 5px;
    outline: none;
    padding: 2px;
}
QListWidget::item, QListView::item {
    padding: 3px 6px;
    border-radius: 3px;
}
QListWidget::item:hover, QListView::item:hover {
    background-color: %(button_hover)s;
}
QListWidget::item:selected, QListView::item:selected {
    background-color: %(accent)s;
    color: %(accent_text)s;
}
QListWidget:disabled, QListView:disabled {
    background-color: %(button_disabled)s;
    color: %(text_disabled)s;
    border-color: %(border_soft)s;
}

/* ---------- spin boxes ---------- */
QSpinBox::up-button, QDoubleSpinBox::up-button {
    subcontrol-origin: border;
    subcontrol-position: top right;
    width: 16px;
    border-left: 1px solid %(border)s;
    border-top-right-radius: 5px;
    background-color: %(button)s;
}
QSpinBox::down-button, QDoubleSpinBox::down-button {
    subcontrol-origin: border;
    subcontrol-position: bottom right;
    width: 16px;
    border-left: 1px solid %(border)s;
    border-bottom-right-radius: 5px;
    background-color: %(button)s;
}
QSpinBox::up-button:hover, QDoubleSpinBox::up-button:hover,
QSpinBox::down-button:hover, QDoubleSpinBox::down-button:hover {
    background-color: %(button_hover)s;
}
QSpinBox::up-button:disabled, QDoubleSpinBox::up-button:disabled,
QSpinBox::down-button:disabled, QDoubleSpinBox::down-button:disabled {
    background-color: %(button_disabled)s;
}
QSpinBox::up-arrow, QDoubleSpinBox::up-arrow {
    image: none;
    width: 0;
    height: 0;
    border-left: 3px solid transparent;
    border-right: 3px solid transparent;
    border-bottom: 4px solid %(text_muted)s;
}
QSpinBox::down-arrow, QDoubleSpinBox::down-arrow {
    image: none;
    width: 0;
    height: 0;
    border-left: 3px solid transparent;
    border-right: 3px solid transparent;
    border-top: 4px solid %(text_muted)s;
}
QSpinBox::up-arrow:disabled, QDoubleSpinBox::up-arrow:disabled {
    border-bottom-color: %(text_disabled)s;
}
QSpinBox::down-arrow:disabled, QDoubleSpinBox::down-arrow:disabled {
    border-top-color: %(text_disabled)s;
}

/* ---------- check boxes ---------- */
QCheckBox {
    spacing: 6px;
}
QCheckBox::indicator {
    width: 15px;
    height: 15px;
    background-color: %(base)s;
    border: 1px solid %(border)s;
    border-radius: 3px;
}
QCheckBox::indicator:hover {
    border-color: %(accent)s;
}
QCheckBox::indicator:checked {
    background-color: %(accent)s;
    border-color: %(accent)s;
    image: url(%(check_icon)s);
}
QCheckBox::indicator:disabled {
    background-color: %(button_disabled)s;
    border-color: %(border_soft)s;
}

/* ---------- scroll bars ---------- */
QScrollBar:vertical {
    background: transparent;
    width: 12px;
    margin: 0;
    border: none;
}
QScrollBar::handle:vertical {
    background: %(scrollbar)s;
    min-height: 26px;
    border-radius: 5px;
    margin: 2px;
}
QScrollBar::handle:vertical:hover {
    background: %(scrollbar_hover)s;
}
QScrollBar:horizontal {
    background: transparent;
    height: 12px;
    margin: 0;
    border: none;
}
QScrollBar::handle:horizontal {
    background: %(scrollbar)s;
    min-width: 26px;
    border-radius: 5px;
    margin: 2px;
}
QScrollBar::handle:horizontal:hover {
    background: %(scrollbar_hover)s;
}
QScrollBar::add-line, QScrollBar::sub-line {
    width: 0;
    height: 0;
    background: transparent;
}
QScrollBar::add-page, QScrollBar::sub-page {
    background: transparent;
}

/* ---------- status bar ---------- */
QStatusBar {
    background-color: %(surface)s;
    border-top: 1px solid %(border)s;
    color: %(text_muted)s;
}
QStatusBar::item {
    border: none;
}
QStatusBar QLabel {
    color: %(text_muted)s;
    padding: 1px 4px;
}

/* ---------- misc ---------- */
QToolTip {
    background-color: %(surface)s;
    color: %(text)s;
    border: 1px solid %(border)s;
    padding: 4px;
}
QScrollArea {
    border: none;
}
QSplitter::handle {
    background-color: %(border_soft)s;
}
QLabel {
    background: transparent;
}
"""


def theme_names():
    """Return an ordered list of ``(code, display_name)`` theme pairs."""
    return list(THEMES)


def palette(theme_name):
    """Return a copy of the colour palette for ``theme_name``."""
    return dict(_PALETTES.get(theme_name, _PALETTES[DEFAULT_THEME]))


def clamp_zoom(zoom):
    """Coerce ``zoom`` into one of the supported percentages."""
    try:
        value = int(round(float(zoom)))
    except (TypeError, ValueError):
        return DEFAULT_ZOOM
    value = max(MIN_ZOOM, min(MAX_ZOOM, value))
    # Snap to the nearest preset so the zoom dropdown always has a matching entry.
    return min(ZOOM_LEVELS, key=lambda level: abs(level - value))


def _format_length(value):
    text = "%.2f" % value
    text = text.rstrip("0").rstrip(".")
    return text or "0"


def scale_qss(qss, factor):
    """Scale every ``px`` / ``pt`` metric in a stylesheet by ``factor``."""
    if factor == 1.0:
        return qss

    def _replace(match):
        value = float(match.group(1))
        unit = match.group(2)
        scaled = value * factor
        if value > 0 and scaled < 1.0:
            # Keep borders and paddings visible when zooming far out.
            scaled = 1.0
        return _format_length(scaled) + unit

    return _LENGTH_RE.sub(_replace, qss)


def stylesheet(theme_name, zoom=DEFAULT_ZOOM):
    """Return the full Qt stylesheet for ``theme_name`` scaled by ``zoom``."""
    colors = palette(theme_name)
    colors["check_icon"] = _CHECK_ICON
    return scale_qss(_QSS % colors, clamp_zoom(zoom) / 100.0)


def _build_qpalette(colors):
    """Build a QPalette so native-drawn elements match the stylesheet."""
    qp = QPalette()
    window = QColor(colors["window"])
    base = QColor(colors["base"])
    text = QColor(colors["text"])
    accent = QColor(colors["accent"])
    accent_text = QColor(colors["accent_text"])
    disabled = QColor(colors["text_disabled"])

    qp.setColor(QPalette.Window, window)
    qp.setColor(QPalette.WindowText, text)
    qp.setColor(QPalette.Base, base)
    qp.setColor(QPalette.AlternateBase, QColor(colors["surface"]))
    qp.setColor(QPalette.Text, text)
    qp.setColor(QPalette.Button, QColor(colors["button"]))
    qp.setColor(QPalette.ButtonText, text)
    qp.setColor(QPalette.BrightText, QColor(colors["status_error"]))
    qp.setColor(QPalette.Highlight, accent)
    qp.setColor(QPalette.HighlightedText, accent_text)
    qp.setColor(QPalette.ToolTipBase, QColor(colors["surface"]))
    qp.setColor(QPalette.ToolTipText, text)
    qp.setColor(QPalette.Link, accent)

    for role in (QPalette.WindowText, QPalette.Text, QPalette.ButtonText):
        qp.setColor(QPalette.Disabled, role, disabled)
    return qp


def apply_theme(app, theme_name, zoom=DEFAULT_ZOOM):
    """Apply ``theme_name`` to the whole application.  Returns the palette."""
    colors = palette(theme_name)
    app.setPalette(_build_qpalette(colors))
    app.setStyleSheet(stylesheet(theme_name, zoom))

    # pyqtgraph defaults for plots created after this point
    pg.setConfigOption("background", colors["plot_bg"])
    pg.setConfigOption("foreground", colors["plot_axis"])
    return colors


def _legend(plot_widget):
    """Return the legend of a ``PlotWidget`` (``PlotWidget.__getattr__`` hides it)."""
    plot_item = getattr(plot_widget, "plotItem", None)
    return getattr(plot_item, "legend", None) if plot_item is not None else None


def apply_plot_zoom(plot_widget, zoom=DEFAULT_ZOOM):
    """Scale the fonts of an existing pyqtgraph plot.

    pyqtgraph ignores both the application stylesheet and ``QApplication.font()``,
    so its axis ticks, axis labels and legend need to be sized explicitly.
    """
    factor = clamp_zoom(zoom) / 100.0
    family = plot_widget.font().family()

    for axis_name in ("left", "bottom", "right", "top"):
        axis = plot_widget.getAxis(axis_name)
        if axis is None:
            continue
        tick_font = QFont(family)
        tick_font.setPointSizeF(_PLOT_FONT_PT * factor)
        axis.setTickFont(tick_font)

        # setLabel() replaces labelStyle wholesale, so the axis colours have to be
        # re-passed.  siPrefixEnableRanges is private and must not be touched here.
        style = dict(axis.labelStyle)
        style["font-size"] = "%spt" % _format_length(_PLOT_FONT_PT * factor)
        axis.setLabel(
            axis.labelText,
            axis.labelUnits,
            unitPrefix=axis.labelUnitPrefix,
            unitPower=axis.unitPower,
            **style
        )

    legend = _legend(plot_widget)
    if legend is not None:
        legend.setLabelTextSize("%spt" % _format_length(_LEGEND_FONT_PT * factor))
        # 让图例立刻按新字号重新排布，而不是等下一次重绘
        legend.updateSize()


def apply_plot_theme(plot_widget, theme_name):
    """Re-colour an existing pyqtgraph ``PlotWidget`` after a theme change."""
    colors = palette(theme_name)
    plot_widget.setBackground(colors["plot_bg"])
    for axis_name in ("left", "bottom", "right", "top"):
        axis = plot_widget.getAxis(axis_name)
        if axis is None:
            continue
        axis.setPen(pg.mkPen(colors["plot_axis"]))
        axis.setTextPen(pg.mkPen(colors["plot_axis"]))
    legend = _legend(plot_widget)
    if legend is not None:
        for _sample, label in legend.items:
            label.setText(label.text, color=colors["plot_axis"])
        legend.setPen(pg.mkPen(colors["border"]))
        legend.setBrush(pg.mkBrush(colors["surface"]))
