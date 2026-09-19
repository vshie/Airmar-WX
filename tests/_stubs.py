"""Import-time stubs so tests can run without installing runtime deps.

The BlueOS container ships `pyserial`, `requests`, `flask`, `flask_cors`,
`waitress`, and `websockets`. Local dev boxes typically do not. Everything
below installs a minimal shim into `sys.modules` before `app.main` /
`app.mavlink_params` gets imported so we can exercise the pure-Python
routing / parameter logic without those wheels.
"""

from __future__ import annotations

import sys
import types


def _fake_flask_module():
    """Enough of Flask to let `from flask import ...` succeed."""
    mod = types.ModuleType('flask')

    class _App:
        static_folder = 'static'

        def __init__(self, *a, **k):
            pass

        def route(self, *a, **k):
            def deco(fn):
                return fn
            return deco

    class _Response:
        def __init__(self, *a, **k):
            pass

    def _jsonify(*a, **k):
        return {'args': a, 'kwargs': k}

    def _send_file(*a, **k):
        return None

    def _send_from_directory(*a, **k):
        return None

    class _RequestProxy:
        def get_json(self, *a, **k):
            return None
        args = {}

    mod.Flask = _App
    mod.jsonify = _jsonify
    mod.request = _RequestProxy()
    mod.send_file = _send_file
    mod.send_from_directory = _send_from_directory
    mod.Response = _Response
    return mod


def _fake_flask_cors():
    mod = types.ModuleType('flask_cors')

    def _CORS(*a, **k):
        return None

    mod.CORS = _CORS
    return mod


def _fake_serial():
    """`pyserial` shim. Only symbols main.py touches at import time."""
    mod = types.ModuleType('serial')

    class _Serial:
        def __init__(self, *a, **k):
            self.is_open = False

    mod.Serial = _Serial
    tools_mod = types.ModuleType('serial.tools')
    list_mod = types.ModuleType('serial.tools.list_ports')
    list_mod.comports = lambda: []
    tools_mod.list_ports = list_mod
    mod.tools = tools_mod
    sys.modules['serial.tools'] = tools_mod
    sys.modules['serial.tools.list_ports'] = list_mod
    return mod


def _fake_requests():
    """Minimal `requests` for import; tests that actually POST override this."""
    mod = types.ModuleType('requests')

    class _Session:
        def post(self, *a, **k):
            raise RuntimeError('_fake_requests.Session.post called in tests')

        def get(self, *a, **k):
            raise RuntimeError('_fake_requests.Session.get called in tests')

    mod.Session = _Session
    return mod


def install():
    """Install all stubs. Safe to call more than once."""
    for name, factory in (
        ('flask', _fake_flask_module),
        ('flask_cors', _fake_flask_cors),
        ('serial', _fake_serial),
        ('requests', _fake_requests),
    ):
        if name not in sys.modules:
            sys.modules[name] = factory()
