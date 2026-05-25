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
platform_name = get_option('platform-name')
mdio_opt = get_option('mdio')

libmnl = dependency(
    'libmnl',
    required: mdio_opt,
    fallback: ['libmnl', 'libmnl_dep'],
)
libftdi = dependency(
    'libftdi1',
    required: mdio_opt,
    include_type: 'system',
    fallback: ['libftdi1', 'libftdi1_dep'],
)
mdio_enabled = libmnl.found() and libftdi.found()

cli11_dep = dependency(
    'CLI11',
    include_type: 'system',
    default_options: ['tests=disabled'],
)
i2c_dep = meson.get_compiler('cpp').find_library('i2c')

platform_srcs = files(
    'platforms/intel/acrp.cpp',
    'platforms/intel/jcrp.cpp',
    'platforms/meta/ventura2.cpp',
    'platforms/nvidia/gb200.cpp',
    'platforms/nvidia/nvl32.cpp',
    'platforms/nvidia/vr_nvl.cpp',
)
util_srcs = files(
    'gpio.cpp',
    'i2c.cpp',
    'utilities.cpp',
)

if mdio_enabled
    util_srcs += files('ftdi_mdio.cpp', 'mdio.cpp')
endif

exe = executable(
    'platform',
    ['platform.cpp'] + platform_srcs + util_srcs,
    cpp_args: mdio_enabled ? ['-DENABLE_MDIO'] : [],
    dependencies: [
        gpiodcxx_dep,
        libsystemd_dep,
        cli11_dep,
        i2c_dep,
        sdbusplus,
        libmnl,
        libftdi,
    ],
    include_directories: ['.', 'platforms'],
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
        {'PLATFORM_NAME': platform_name},
    ),
)
