"""Build isolated navigation plugins before changing any navigation runtime files.

The installer embeds this module; import has no build or ROS side effects.
Every requested package must build, install and pass its node-free pluginlib
probe before a usable installation prefix is returned.
"""
import base64
import datetime
import os
from pathlib import Path, PurePosixPath
import shutil
import subprocess


_NAVIGATION_PACKAGES = ('fairino_lookahead_controller', 'fairino_straight_planner')


def _build_plugins(runtime, package_sources, build_directory):
    # Validate every package before staging any source, including later packages.
    decoded = {}
    for name, sources in package_sources.items():
        if name not in _NAVIGATION_PACKAGES or not sources:
            raise ValueError('Invalid navigation plugin package: ' + str(name))
        decoded[name] = {}
        for relative, encoded in sources.items():
            path = PurePosixPath(relative)
            if path.is_absolute() or '..' in path.parts or not path.parts or str(path) != relative:
                raise ValueError('Invalid navigation plugin source path: ' + relative)
            decoded[name][relative] = base64.b64decode(encoded, validate=True)
    if not decoded:
        raise ValueError('Navigation plugin sources are empty')
    for tool in ('cmake', 'c++', 'make'):
        if shutil.which(tool) is None:
            raise RuntimeError('编译导航插件需要安装：' + tool)

    folder = Path(runtime) / build_directory / datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    install = folder/'install'
    for name, sources in decoded.items():
        for relative, content in sources.items():
            target = folder/'source'/name/relative
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(content)
    log = folder/'build.log'

    def run(command, env=None):
        try:
            with log.open('a') as stream:
                result = subprocess.run(command, env=env, stdout=stream,
                                        stderr=subprocess.STDOUT, timeout=600)
        except subprocess.TimeoutExpired as error:
            raise RuntimeError('导航插件构建/加载超时，未更新导航服务：' + str(log)) from error
        if result.returncode:
            raise RuntimeError('导航插件构建/加载失败，未更新导航服务：' + str(log) + '\n'
                               + log.read_text(errors='replace')[-5000:])

    # Each package has a separate source and build tree; the final overlay holds
    # both packages so switching it cannot silently drop the existing controller.
    for name in decoded:
        source, build = folder/'source'/name, folder/'build'/name
        run(['cmake', '-S', str(source), '-B', str(build), '-DCMAKE_BUILD_TYPE=Release',
             '-DBUILD_TESTING=OFF', '-DCMAKE_INSTALL_PREFIX=' + str(install)])
        run(['cmake', '--build', str(build), '--parallel', '2'])
        run(['cmake', '--install', str(build)])
        artifacts = (install/'lib'/('lib' + name + '.so'),
                     install/'lib'/name/'plugin_check',
                     install/'share'/name/'local_setup.bash')
        if name == 'fairino_lookahead_controller':
            artifacts += (install/'lib/libfairino_replan_bt_node.so',)
        if not all(path.is_file() for path in artifacts):
            raise RuntimeError('导航插件安装产物不完整，未更新导航服务：' + name + '：' + str(install))

    env = dict(os.environ)
    for key, prefix in (('AMENT_PREFIX_PATH', install), ('LD_LIBRARY_PATH', install/'lib')):
        env[key] = str(prefix) + (':' + env[key] if env.get(key) else '')
    # These executables load plugin types without creating ROS nodes or invoking
    # their lifecycle. Runtime files are changed only by the caller after return.
    for name in decoded:
        run([str(install/'lib'/name/'plugin_check')], env=env)
    return install


def build_navigation_plugins(runtime, package_sources):
    """Build and load-check the complete controller/planner overlay atomically."""
    if set(package_sources) != set(_NAVIGATION_PACKAGES):
        raise ValueError('Navigation overlay requires both controller and planner sources')
    ordered = {name: package_sources[name] for name in _NAVIGATION_PACKAGES}
    return _build_plugins(runtime, ordered, 'navigation-plugin-builds')


def build_controller(runtime, sources):
    """Keep the original single-controller build entry point for compatibility."""
    return _build_plugins(runtime, {'fairino_lookahead_controller': sources}, 'controller-builds')
