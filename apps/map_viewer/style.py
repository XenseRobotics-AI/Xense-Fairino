"""Scoped desktop styling shared by map and workbench; no control behavior."""
import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, Gdk

_provider = None


def install_style():
    global _provider
    if _provider is not None:
        return
    screen = Gdk.Screen.get_default()
    if screen is None:
        return
    _provider = Gtk.CssProvider()
    _provider.load_from_data(b'''
    #map-studio { background: #f4f6f9; color: #223248; }
    #map-studio button { border-radius: 7px; padding: 7px 10px; }
    #map-studio button.suggested-action { background: #126f7d; color: white; border-color: #126f7d; }
    #map-studio button:disabled { color: #8a96a5; background: #edf0f4; border-color: #dde3ea; }
    #map-studio entry, #map-studio combobox { min-height: 29px; }
    #map-studio notebook { background: #ffffff; border: 1px solid #dce3eb; border-radius: 8px; }
    #map-studio notebook header { background: #eef2f6; }
    #map-studio notebook tab { padding: 9px 10px; }
    #map-studio notebook tab:checked { color: #086a78; border-bottom: 3px solid #168597; }
    #map-studio .map-metric { background: #ffffff; border: 1px solid #dfe5ed;
                              border-radius: 8px; padding: 5px 13px; }
    #map-studio .map-caption { color: #5d6d80; font-size: 11px; }
    #map-studio .map-notice { color: #42576d; font-size: 12px; }
    #map-workflow button { padding: 2px 7px; min-height: 22px; }
    #map-workflow combobox, #map-workflow entry { min-height: 24px; }
    #map-workflow expander { padding: 2px; }
    #map-studio toolbar { background: transparent; padding: 0; }
    #map-studio toolbar button { padding: 2px 5px; min-height: 22px; min-width: 22px; }
    #map-studio expander { color: #29435b; padding: 5px; }
    #map-manual-controls button { padding: 3px 7px; min-height: 20px; }
    #map-manual-controls spinbutton, #map-manual-controls entry { min-height: 24px; }
    ''')
    Gtk.StyleContext.add_provider_for_screen(screen, _provider, Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION)
