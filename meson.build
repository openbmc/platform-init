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
phosphor_dbus_interfaces_dep = dependency('phosphor-dbus-interfaces')
phosphor_logging_dep = dependency('phosphor-logging')

cli11_dep = dependency(
    'CLI11',
    include_type: 'system',
    default_options: ['tests=disabled'],
)
i2c_dep = meson.get_compiler('cpp').find_library('i2c')

platform_srcs = files(
    'platforms/intel/acrp.cpp',
    'platforms/intel/jcrp.cpp',
    'platforms/meta/catalina.cpp',
    'platforms/nvidia/gb200.cpp',
    'platforms/nvidia/nvl32.cpp',
    'platforms/nvidia/vr_nvl.cpp',
)

cmos_reset_srcs = files('cmos-reset/cmos_reset.cpp', 'cmos-reset/host_power.cpp')

util_srcs = files('gpio.cpp', 'i2c.cpp', 'utilities.cpp')

executable(
    'platform',
    ['platform.cpp'] + platform_srcs + cmos_reset_srcs + util_srcs,
    dependencies: [
        gpiodcxx_dep,
        libsystemd_dep,
        cli11_dep,
        i2c_dep,
        sdbusplus,
        phosphor_logging_dep,
        phosphor_dbus_interfaces_dep,
    ],
    include_directories: ['.', 'cmos-reset', 'platforms'],
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

configure_file(
    input: 'cmos-reset/xyz.openbmc_project.Software.Host.Updater0.service.in',
    output: 'xyz.openbmc_project.Software.Host.Updater0.service',
    install_dir: systemd_system_unit_dir,
    install: true,
    configuration: configuration_data(
        {'PLATFORM_NAME': get_option('platform-name')},
    ),
)

conf = configuration_data()
conf.set(
    'HOST_STATE_TRANSITION_TIMEOUT',
    get_option('host-state-transition-timeout'),
)
configure_file(output: 'config.h', configuration: conf)

