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
sdbusplus = dependency('sdbusplus', include_type: 'system')

cli11_dep = dependency('CLI11', required: true, include_type: 'system')
i2c_dep = meson.get_compiler('cpp').find_library('i2c')

platform_srcs = files(
    'intel/acrp.cpp',
    'intel/jcrp.cpp',
    'meta/catalina.cpp',
    'nvidia/gb200.cpp',
    'nvidia/nvl32.cpp',
)
util_srcs = files('gpio.cpp', 'i2c.cpp', 'utilities.cpp')

installdir = join_paths(get_option('libexecdir'), 'platform-init')

executable(
    'platform',
    ['platform.cpp'] + platform_srcs + util_srcs,
    dependencies: [
        gpiodcxx_dep,
        libsystemd_dep,
        cli11_dep,
        i2c_dep,
        sdbusplus,
    ],
    include_directories: ['.', 'intel', 'meta', 'nvidia'],
    install: true,
    install_dir: installdir,
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

if get_option('cmos-reset').allowed()
    subdir('cmos-reset')
endif
