project(
    'platform-init',
    'cpp',
    meson_version: '>=1.7.0',
    version: '0.1',
    default_options: ['warning_level=3', 'cpp_std=c++23'],
)

gpiodcxx_dep = dependency('libgpiodcxx', default_options: ['bindings=cxx'])
systemd_dep = dependency('systemd')
libsystemd_dep = dependency('libsystemd')

cli11_dep = dependency('CLI11', required: true, include_type: 'system')

nvidia_srcs = ['nvidia/gb200.cpp']
util_srcs = ['i2c.cpp', 'gpio.cpp', 'utilities.cpp']

exe = executable(
    'platform',
    ['platform.cpp'] + nvidia_srcs + util_srcs,
    dependencies: [gpiodcxx_dep, libsystemd_dep, cli11_dep],
    include_directories: ['.', 'nvidia'],
    install: true,
    install_dir: get_option('libexecdir'),
)

systemd_system_unit_dir = systemd_dep.get_variable('systemd_system_unit_dir')
configure_file(
    input: 'platform_init.service.in',
    output: 'platform_init.service',
    install_dir: systemd_system_unit_dir,
    install: true,
    configuration: configuration_data(
        {'PLATFORM_NAME': get_option('platform-name')},
    ),
)
