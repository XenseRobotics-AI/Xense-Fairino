"""GTK host adapter for embedding the preserved mapping keyboard controls."""
import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, Gdk


class PanelHost(Gtk.Box):
    disabled_style = None

    def __init__(self, owner, title='', **_):
        super().__init__(orientation=Gtk.Orientation.VERTICAL)
        if PanelHost.disabled_style is None:
            provider = Gtk.CssProvider()
            provider.load_from_data(b'''
                .mapping-control:disabled, .mapping-control:disabled * {
                    color: #737b85;
                    text-shadow: none;
                    -gtk-icon-shadow: none;
                    opacity: 1;
                }
                .mapping-control:disabled button,
                .mapping-control:disabled entry {
                    background-image: none;
                    background-color: #f2f3f5;
                    border-color: #d2d6dc;
                    box-shadow: none;
                }
            ''')
            Gtk.StyleContext.add_provider_for_screen(Gdk.Screen.get_default(), provider,
                                                     Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION)
            PanelHost.disabled_style = provider
        self.get_style_context().add_class('mapping-control')
        self.owner = owner
        self.handlers = {}
        self.input_enabled = False

    def add(self, child):
        self.pack_start(child, True, True, 0)

    def connect(self, name, callback, *args):
        if name in ('key-press-event', 'key-release-event', 'focus-in-event',
                    'focus-out-event', 'delete-event'):
            self.handlers[name] = (callback, args)
            return 0
        return super().connect(name, callback, *args)

    def dispatch(self, name, event):
        if name == 'key-press-event' and not self.is_sensitive():
            return False
        handler = self.handlers.get(name)
        return bool(handler and handler[0](self, event, *handler[1]))

    def get_focus(self):
        return self.owner.get_focus()

    def is_active(self):
        return self.input_enabled and self.owner.is_active()

    def set_default_size(self, *_):
        pass

    def present(self):
        pass
