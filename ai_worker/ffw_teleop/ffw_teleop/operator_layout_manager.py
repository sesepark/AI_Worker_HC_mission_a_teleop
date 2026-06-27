import json
import os
import shutil
import subprocess
import time
import ctypes
import ctypes.util

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


DEFAULT_TARGETS = [
    {
        'name': 'rviz',
        'enabled': True,
        'title_patterns': ['teleop_operator_rviz', 'RViz', 'RViz2'],
    },
    {
        'name': 'mission_control',
        'enabled': True,
        'title_patterns': ['Mission Control'],
    },
    {
        'name': 'operator_image_viewer',
        'enabled': True,
        'title_patterns': ['Teleop Image Viewer'],
    },
]


class WmctrlWindowBackend:

    name = 'wmctrl'

    def __init__(self, path, logger):
        self.path = path
        self.logger = logger

    def list_windows(self):
        proc = subprocess.run(
            [self.path, '-lG'],
            check=False,
            text=True,
            capture_output=True,
        )
        if proc.returncode != 0:
            self.logger.warn(proc.stderr.strip() or 'wmctrl -lG failed')
            return []
        windows = []
        for line in proc.stdout.splitlines():
            parts = line.split(None, 7)
            if len(parts) < 8:
                continue
            try:
                geometry = {
                    'x': int(parts[2]),
                    'y': int(parts[3]),
                    'width': int(parts[4]),
                    'height': int(parts[5]),
                }
            except ValueError:
                continue
            windows.append({
                'id': parts[0],
                'desktop': parts[1],
                'host': parts[6],
                'title': parts[7],
                **geometry,
            })
        return windows

    def apply_geometry(self, match, geometry, dry_run=False):
        x, y, width, height = geometry
        self.logger.info(
            f'restore window -> {x},{y} {width}x{height} ({match["title"]})')
        if dry_run:
            return True
        subprocess.run(
            [self.path, '-ir', match['id'], '-b', 'remove,maximized_vert,maximized_horz'],
            check=False,
        )
        subprocess.run(
            [self.path, '-ir', match['id'], '-e', f'0,{x},{y},{width},{height}'],
            check=False,
        )
        return True

    def desktop_geometry(self):
        proc = subprocess.run(
            [self.path, '-d'],
            check=False,
            text=True,
            capture_output=True,
        )
        if proc.returncode != 0:
            return None
        for line in proc.stdout.splitlines():
            if '*' not in line:
                continue
            parts = line.split()
            for index, part in enumerate(parts):
                if part == 'WA:' and index + 2 < len(parts):
                    xy = parse_pair(parts[index + 1], ',')
                    size = parse_pair(parts[index + 2], 'x')
                    if xy and size:
                        return xy[0], xy[1], size[0], size[1]
                if part == 'DG:' and index + 1 < len(parts):
                    size = parse_pair(parts[index + 1], 'x')
                    if size:
                        return 0, 0, size[0], size[1]
        return None


class X11WindowBackend:

    name = 'libX11'

    def __init__(self, logger):
        self.logger = logger
        library_path = ctypes.util.find_library('X11')
        if not library_path:
            for candidate in (
                'libX11.so.6',
                '/usr/lib/x86_64-linux-gnu/libX11.so.6',
                '/usr/lib/aarch64-linux-gnu/libX11.so.6',
                '/opt/X11/lib/libX11.dylib',
            ):
                if os.path.exists(candidate):
                    library_path = candidate
                    break
        if not library_path:
            raise RuntimeError('libX11 runtime library was not found')
        self.x11 = ctypes.CDLL(library_path)
        self._configure_signatures()
        self.display = self.x11.XOpenDisplay(None)
        if not self.display:
            raise RuntimeError('XOpenDisplay failed; DISPLAY may be unset or inaccessible')
        self.root = self.x11.XDefaultRootWindow(self.display)
        self._atoms = {}

    def _configure_signatures(self):
        self.x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
        self.x11.XOpenDisplay.restype = ctypes.c_void_p
        self.x11.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
        self.x11.XDefaultRootWindow.restype = ctypes.c_ulong
        self.x11.XInternAtom.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
        self.x11.XInternAtom.restype = ctypes.c_ulong
        self.x11.XGetWindowProperty.argtypes = [
            ctypes.c_void_p,
            ctypes.c_ulong,
            ctypes.c_ulong,
            ctypes.c_long,
            ctypes.c_long,
            ctypes.c_int,
            ctypes.c_ulong,
            ctypes.POINTER(ctypes.c_ulong),
            ctypes.POINTER(ctypes.c_int),
            ctypes.POINTER(ctypes.c_ulong),
            ctypes.POINTER(ctypes.c_ulong),
            ctypes.POINTER(ctypes.POINTER(ctypes.c_ubyte)),
        ]
        self.x11.XGetWindowProperty.restype = ctypes.c_int
        self.x11.XFree.argtypes = [ctypes.c_void_p]
        self.x11.XFree.restype = ctypes.c_int
        self.x11.XFetchName.argtypes = [
            ctypes.c_void_p, ctypes.c_ulong, ctypes.POINTER(ctypes.c_char_p)]
        self.x11.XFetchName.restype = ctypes.c_int
        self.x11.XGetGeometry.argtypes = [
            ctypes.c_void_p,
            ctypes.c_ulong,
            ctypes.POINTER(ctypes.c_ulong),
            ctypes.POINTER(ctypes.c_int),
            ctypes.POINTER(ctypes.c_int),
            ctypes.POINTER(ctypes.c_uint),
            ctypes.POINTER(ctypes.c_uint),
            ctypes.POINTER(ctypes.c_uint),
            ctypes.POINTER(ctypes.c_uint),
        ]
        self.x11.XGetGeometry.restype = ctypes.c_int
        self.x11.XTranslateCoordinates.argtypes = [
            ctypes.c_void_p,
            ctypes.c_ulong,
            ctypes.c_ulong,
            ctypes.c_int,
            ctypes.c_int,
            ctypes.POINTER(ctypes.c_int),
            ctypes.POINTER(ctypes.c_int),
            ctypes.POINTER(ctypes.c_ulong),
        ]
        self.x11.XTranslateCoordinates.restype = ctypes.c_int
        self.x11.XMoveResizeWindow.argtypes = [
            ctypes.c_void_p,
            ctypes.c_ulong,
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_uint,
            ctypes.c_uint,
        ]
        self.x11.XMoveResizeWindow.restype = ctypes.c_int
        self.x11.XRaiseWindow.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
        self.x11.XRaiseWindow.restype = ctypes.c_int
        self.x11.XFlush.argtypes = [ctypes.c_void_p]
        self.x11.XFlush.restype = ctypes.c_int

    def _atom(self, name):
        if name not in self._atoms:
            self._atoms[name] = self.x11.XInternAtom(
                self.display, name.encode('utf-8'), False)
        return self._atoms[name]

    def _property(self, window_id, atom_name, long_length=8192):
        actual_type = ctypes.c_ulong()
        actual_format = ctypes.c_int()
        nitems = ctypes.c_ulong()
        bytes_after = ctypes.c_ulong()
        prop = ctypes.POINTER(ctypes.c_ubyte)()
        status = self.x11.XGetWindowProperty(
            self.display,
            ctypes.c_ulong(window_id),
            self._atom(atom_name),
            0,
            long_length,
            False,
            0,
            ctypes.byref(actual_type),
            ctypes.byref(actual_format),
            ctypes.byref(nitems),
            ctypes.byref(bytes_after),
            ctypes.byref(prop),
        )
        if status != 0 or not prop:
            return None, 0, 0
        try:
            if actual_format.value == 8:
                data = ctypes.string_at(prop, nitems.value)
            elif actual_format.value == 32:
                array = ctypes.cast(prop, ctypes.POINTER(ctypes.c_ulong))
                data = [int(array[index]) for index in range(nitems.value)]
            else:
                data = None
            return data, actual_format.value, int(nitems.value)
        finally:
            self.x11.XFree(prop)

    def _client_windows(self):
        for atom_name in ('_NET_CLIENT_LIST', '_NET_CLIENT_LIST_STACKING'):
            data, actual_format, _ = self._property(self.root, atom_name)
            if actual_format == 32 and data:
                return data
        return []

    def _window_title(self, window_id):
        for atom_name in ('_NET_WM_NAME', 'WM_NAME'):
            data, actual_format, _ = self._property(window_id, atom_name)
            if actual_format == 8 and data:
                title = data.split(b'\x00', 1)[0].decode('utf-8', errors='replace').strip()
                if title:
                    return title
        name = ctypes.c_char_p()
        if self.x11.XFetchName(self.display, ctypes.c_ulong(window_id), ctypes.byref(name)):
            if name.value:
                try:
                    return name.value.decode('utf-8', errors='replace').strip()
                finally:
                    self.x11.XFree(name)
        return ''

    def _window_geometry(self, window_id):
        root_return = ctypes.c_ulong()
        x = ctypes.c_int()
        y = ctypes.c_int()
        width = ctypes.c_uint()
        height = ctypes.c_uint()
        border = ctypes.c_uint()
        depth = ctypes.c_uint()
        ok = self.x11.XGetGeometry(
            self.display,
            ctypes.c_ulong(window_id),
            ctypes.byref(root_return),
            ctypes.byref(x),
            ctypes.byref(y),
            ctypes.byref(width),
            ctypes.byref(height),
            ctypes.byref(border),
            ctypes.byref(depth),
        )
        if not ok:
            return None
        abs_x = ctypes.c_int()
        abs_y = ctypes.c_int()
        child = ctypes.c_ulong()
        translated = self.x11.XTranslateCoordinates(
            self.display,
            ctypes.c_ulong(window_id),
            self.root,
            0,
            0,
            ctypes.byref(abs_x),
            ctypes.byref(abs_y),
            ctypes.byref(child),
        )
        return {
            'x': int(abs_x.value if translated else x.value),
            'y': int(abs_y.value if translated else y.value),
            'width': int(width.value),
            'height': int(height.value),
        }

    def list_windows(self):
        host = os.uname().nodename if hasattr(os, 'uname') else ''
        windows = []
        for window_id in self._client_windows():
            geometry = self._window_geometry(window_id)
            if geometry is None:
                continue
            title = self._window_title(window_id)
            windows.append({
                'id': hex(int(window_id)),
                '_window_id': int(window_id),
                'desktop': '',
                'host': host,
                'title': title,
                **geometry,
            })
        return windows

    def apply_geometry(self, match, geometry, dry_run=False):
        window_id = int(match.get('_window_id') or int(str(match['id']), 16))
        x, y, width, height = geometry
        self.logger.info(
            f'restore window via libX11 -> {x},{y} {width}x{height} ({match["title"]})')
        if dry_run:
            return True
        self.x11.XMoveResizeWindow(
            self.display,
            ctypes.c_ulong(window_id),
            int(x),
            int(y),
            int(width),
            int(height),
        )
        self.x11.XRaiseWindow(self.display, ctypes.c_ulong(window_id))
        self.x11.XFlush(self.display)
        return True

    def desktop_geometry(self):
        geometry = self._window_geometry(self.root)
        if geometry is None:
            return None
        return 0, 0, geometry['width'], geometry['height']


def parse_pair(value, separator):
    try:
        left, right = str(value).split(separator, 1)
        return int(left), int(right)
    except (TypeError, ValueError):
        return None


class OperatorLayoutManager(Node):

    def __init__(self):
        super().__init__('operator_layout_manager')
        self.declare_parameter('layout_config', '')
        self.declare_parameter('layout_store_path', '~/.config/ffw_teleop/operator_screen_layout.json')
        self.declare_parameter('action', 'restore')
        self.declare_parameter('command_topic', '/teleop/operator_layout/command')
        self.declare_parameter('status_topic', '/teleop/operator_layout/status')
        self.declare_parameter('initial_delay_s', 3.0)
        self.declare_parameter('retry_count', 24)
        self.declare_parameter('retry_interval_s', 0.5)
        self.declare_parameter('diagnostic_only', False)
        self.declare_parameter('dry_run', False)
        self.declare_parameter('wmctrl_path', 'wmctrl')

        self.layout_config = str(self.get_parameter('layout_config').value).strip()
        self.layout_store_path = os.path.expanduser(
            str(self.get_parameter('layout_store_path').value).strip())
        self.action = str(self.get_parameter('action').value).strip().lower() or 'restore'
        if self._as_bool(self.get_parameter('diagnostic_only').value):
            self.action = 'diagnostic'
        self.command_topic = str(self.get_parameter('command_topic').value).strip()
        self.status_topic = str(self.get_parameter('status_topic').value).strip()
        self.initial_delay_s = max(float(self.get_parameter('initial_delay_s').value), 0.0)
        self.retry_count = max(int(self.get_parameter('retry_count').value), 1)
        self.retry_interval_s = max(float(self.get_parameter('retry_interval_s').value), 0.1)
        self.dry_run = self._as_bool(self.get_parameter('dry_run').value)
        self.wmctrl_path = str(self.get_parameter('wmctrl_path').value).strip() or 'wmctrl'
        self.targets = self._load_targets(self.layout_config)
        self.status_pub = None
        if self.status_topic:
            self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.command_sub = None
        self.backend = None

    def run(self):
        if self.action == 'server':
            self._run_server()
            return
        if self.action == 'reset':
            self._reset_saved_layout()
            return
        backend = self._resolve_backend()
        if backend is None:
            message = self._backend_unavailable_message()
            self.get_logger().warn(message)
            self._publish_status('error', message, self.action)
            return

        if self.action == 'diagnostic':
            self._print_diagnostics(backend)
            return
        if self.action == 'save':
            self._save_current_layout(backend)
            return
        if self.action != 'restore':
            self.get_logger().warn(f'unknown layout action={self.action}; using restore')

        self._restore_saved_layout(backend)

    def _run_server(self):
        if self.command_topic:
            self.command_sub = self.create_subscription(
                String, self.command_topic, self._command_callback, 10)
        self.get_logger().info(
            f'operator layout manager server active: command={self.command_topic}, '
            f'status={self.status_topic}')
        backend = self._resolve_backend()
        if backend is None:
            message = self._backend_unavailable_message()
            self.get_logger().warn(message)
            self._publish_status('error', message, 'server')
        else:
            self.get_logger().info(f'operator layout backend: {backend.name}')
            self._restore_saved_layout(backend)
            self._publish_status(
                'ready',
                f'operator layout manager ready ({backend.name})',
                'server',
            )
        rclpy.spin(self)

    def _command_callback(self, msg):
        action = str(msg.data or '').strip().lower()
        aliases = {
            'save_layout': 'save',
            'restore_layout': 'restore',
            'reset_layout': 'reset',
            'clear': 'reset',
        }
        action = aliases.get(action, action)
        if action == 'reset':
            self._reset_saved_layout()
            return
        if action in ('save', 'restore', 'diagnostic'):
            self._run_layout_action(action)
            return
        message = f'unknown layout command: {msg.data!r}'
        self.get_logger().warn(message)
        self._publish_status('error', message, action)

    def _run_layout_action(self, action):
        backend = self._resolve_backend()
        if backend is None:
            message = self._backend_unavailable_message()
            self.get_logger().warn(message)
            self._publish_status('error', message, action)
            return
        if action == 'save':
            self._save_current_layout(backend)
        elif action == 'restore':
            self._restore_saved_layout(backend)
        elif action == 'diagnostic':
            self._print_diagnostics(backend)

    def _as_bool(self, value):
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value)

    def _resolve_wmctrl(self):
        if os.path.isabs(self.wmctrl_path) and os.path.exists(self.wmctrl_path):
            return self.wmctrl_path
        return shutil.which(self.wmctrl_path)

    def _resolve_backend(self):
        if self.backend is not None:
            return self.backend
        wmctrl = self._resolve_wmctrl()
        if wmctrl is not None:
            self.backend = WmctrlWindowBackend(wmctrl, self.get_logger())
            return self.backend
        try:
            self.backend = X11WindowBackend(self.get_logger())
            return self.backend
        except Exception as exc:
            self.get_logger().debug(f'libX11 layout backend unavailable: {exc}')
            return None

    def _backend_unavailable_message(self):
        return (
            'no X11 window layout backend is available; install wmctrl or make '
            'libX11 available in the environment'
        )

    def _publish_status(self, status, message, action='', details=None):
        if self.status_pub is None:
            return
        payload = {
            'stamp_sec': self.get_clock().now().nanoseconds / 1e9,
            'status': str(status),
            'action': str(action or ''),
            'message': str(message),
        }
        if details:
            payload['details'] = details
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.status_pub.publish(msg)

    def _load_targets(self, path):
        if not path:
            return DEFAULT_TARGETS
        try:
            import yaml
            with open(os.path.expanduser(path), 'r', encoding='utf-8') as stream:
                loaded = yaml.safe_load(stream) or {}
        except OSError as exc:
            self.get_logger().warn(f'failed to read layout target config {path}: {exc}')
            return DEFAULT_TARGETS
        except Exception as exc:
            self.get_logger().warn(f'failed to parse layout target config {path}: {exc}')
            return DEFAULT_TARGETS
        windows = loaded.get('windows')
        if not isinstance(windows, list):
            self.get_logger().warn(f'layout target config {path} has no windows list')
            return DEFAULT_TARGETS
        return windows

    def _enabled(self, window):
        return self._as_bool(window.get('enabled', True))

    def _list_windows(self, backend):
        return backend.list_windows()

    def _find_window(self, target, windows):
        for pattern in self._target_patterns(target):
            for window in windows:
                if pattern and pattern in window.get('title', ''):
                    return window
        return None

    def _target_patterns(self, target):
        patterns = [str(item) for item in target.get('title_patterns') or []]
        if not patterns:
            name = str(target.get('name') or '').strip()
            patterns = [name] if name else []
        title = str(target.get('title') or '').strip()
        if title and title not in patterns:
            patterns.insert(0, title)
        return patterns

    def _save_current_layout(self, backend):
        windows = self._list_windows(backend)
        saved_windows = []
        missing_windows = []
        for target in self.targets:
            if not self._enabled(target):
                continue
            match = self._find_window(target, windows)
            if match is None:
                missing_windows.append(str(target.get('name') or 'unnamed'))
                self.get_logger().warn(
                    f'cannot save layout for missing window: {target.get("name", "unnamed")}')
                continue
            patterns = self._target_patterns(target)
            if match['title'] not in patterns:
                patterns.insert(0, match['title'])
            saved_windows.append({
                'name': str(target.get('name') or match['title']),
                'enabled': True,
                'title_patterns': patterns,
                'title': match['title'],
                'x': match['x'],
                'y': match['y'],
                'width': match['width'],
                'height': match['height'],
            })
            self.get_logger().info(
                f'saved {target.get("name", match["title"])}: '
                f'{match["x"]},{match["y"]} {match["width"]}x{match["height"]}')

        payload = {
            'version': 1,
            'saved_at_sec': time.time(),
            'backend': backend.name,
            'windows': saved_windows,
        }
        if self.dry_run:
            self.get_logger().info(json.dumps(payload, sort_keys=True))
            self._publish_status(
                'ok',
                f'dry-run saved {len(saved_windows)} window layouts',
                'save',
                {'missing_windows': missing_windows},
            )
            return
        layout_dir = os.path.dirname(self.layout_store_path)
        if layout_dir:
            os.makedirs(layout_dir, exist_ok=True)
        with open(self.layout_store_path, 'w', encoding='utf-8') as stream:
            json.dump(payload, stream, indent=2, sort_keys=True)
            stream.write('\n')
        message = f'saved {len(saved_windows)} window layouts'
        self.get_logger().info(f'{message}: {self.layout_store_path}')
        self._publish_status(
            'ok',
            message,
            'save',
            {
                'path': self.layout_store_path,
                'missing_windows': missing_windows,
            },
        )

    def _restore_saved_layout(self, backend):
        if not os.path.exists(self.layout_store_path):
            message = f'no saved operator layout at {self.layout_store_path}; skipping restore'
            self.get_logger().info(message)
            self._publish_status(
                'idle',
                'no saved operator layout; skipped restore',
                'restore',
                {'path': self.layout_store_path},
            )
            return
        try:
            with open(self.layout_store_path, 'r', encoding='utf-8') as stream:
                payload = json.load(stream)
        except (OSError, json.JSONDecodeError) as exc:
            message = f'failed to read saved operator layout: {exc}'
            self.get_logger().warn(message)
            self._publish_status('error', message, 'restore')
            return
        saved_windows = payload.get('windows') or []
        pending = [window for window in saved_windows if self._enabled(window)]
        if not pending:
            message = 'saved operator layout has no enabled windows'
            self.get_logger().info(message)
            self._publish_status('idle', message, 'restore')
            return

        if self.initial_delay_s > 0.0:
            time.sleep(self.initial_delay_s)

        restored_count = 0
        for _ in range(self.retry_count):
            if not pending:
                break
            current = self._list_windows(backend)
            next_pending = []
            for window in pending:
                match = self._find_window(window, current)
                if match is None:
                    next_pending.append(window)
                    continue
                if self._apply_geometry(backend, match, window):
                    restored_count += 1
            pending = next_pending
            if pending:
                time.sleep(self.retry_interval_s)

        for window in pending:
            self.get_logger().warn(
                f'window not found for saved layout: {window.get("name", "unnamed")}')
        if pending:
            self._publish_status(
                'warn',
                f'restored {restored_count} windows; {len(pending)} not found',
                'restore',
                {
                    'missing_windows': [
                        str(window.get('name') or 'unnamed') for window in pending
                    ],
                },
            )
        else:
            self._publish_status(
                'ok',
                f'restored {restored_count} windows',
                'restore',
                {'path': self.layout_store_path},
            )

    def _apply_geometry(self, backend, match, target):
        geometry = self._safe_geometry(backend, target)
        if geometry is None:
            self.get_logger().warn(
                f'skipping invalid saved geometry for {target.get("name", "window")}')
            return False
        self.get_logger().info(
            f'restore {target.get("name", "window")} -> '
            f'{geometry[0]},{geometry[1]} {geometry[2]}x{geometry[3]} ({match["title"]})')
        return backend.apply_geometry(match, geometry, self.dry_run)

    def _safe_geometry(self, backend, target):
        try:
            x = int(target.get('x', 0))
            y = int(target.get('y', 0))
            width = int(target.get('width', 800))
            height = int(target.get('height', 600))
        except (TypeError, ValueError):
            return None

        width = max(width, 240)
        height = max(height, 180)
        desktop = backend.desktop_geometry()
        if desktop is None:
            return x, y, width, height

        desk_x, desk_y, desk_width, desk_height = desktop
        if desk_width < 240 or desk_height < 180:
            return x, y, width, height
        width = min(width, desk_width)
        height = min(height, desk_height)
        x_max = desk_x + max(desk_width - width, 0)
        y_max = desk_y + max(desk_height - height, 0)
        x = min(max(x, desk_x), x_max)
        y = min(max(y, desk_y), y_max)
        return x, y, width, height

    def _print_diagnostics(self, backend):
        windows = self._list_windows(backend)
        if not windows:
            message = f'no X11 windows reported by {backend.name}'
            self.get_logger().info(message)
            self._publish_status('idle', message, 'diagnostic')
            return
        for window in windows:
            self.get_logger().info(
                f'{window["id"]} x={window["x"]} y={window["y"]} '
                f'w={window["width"]} h={window["height"]} title={window["title"]}')
        self._publish_status(
            'ok',
            f'reported {len(windows)} X11 windows via {backend.name}',
            'diagnostic',
        )

    def _reset_saved_layout(self):
        if not self.layout_store_path:
            message = 'layout store path is empty'
            self.get_logger().warn(message)
            self._publish_status('error', message, 'reset')
            return
        if not os.path.exists(self.layout_store_path):
            message = 'no saved operator layout to reset'
            self.get_logger().info(message)
            self._publish_status(
                'idle',
                message,
                'reset',
                {'path': self.layout_store_path},
            )
            return
        try:
            os.remove(self.layout_store_path)
        except OSError as exc:
            message = f'failed to reset saved operator layout: {exc}'
            self.get_logger().warn(message)
            self._publish_status('error', message, 'reset')
            return
        message = 'deleted saved operator layout'
        self.get_logger().info(f'{message}: {self.layout_store_path}')
        self._publish_status(
            'ok',
            message,
            'reset',
            {'path': self.layout_store_path},
        )


def main(args=None):
    rclpy.init(args=args)
    node = OperatorLayoutManager()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
