phosphor_dbus_interfaces_dep = dependency('phosphor-dbus-interfaces')

cmos_platform_srcs = files('../meta/catalina.cpp')
cmos_util_srcs = files('../gpio.cpp', '../utilities.cpp')

executable(
    'cmos-reset',
    ['cmos_reset.cpp', 'cmos_reset_main.cpp'] + cmos_platform_srcs + cmos_util_srcs,
    dependencies: [
        gpiodcxx_dep,
        libsystemd_dep,
        cli11_dep,
        sdbusplus,
        phosphor_dbus_interfaces_dep,
    ],
    include_directories: ['.', '..', '../meta'],
    install: true,
    install_dir: installdir,
)

systemd_system_unit_dir = systemd_dep.get_variable('systemd_system_unit_dir')

configure_file(
    input: 'xyz.openbmc_project.Software.Host.Updater0.service.in',
    output: 'xyz.openbmc_project.Software.Host.Updater0.service',
    install_dir: systemd_system_unit_dir,
    install: true,
    configuration: configuration_data(
        {'PLATFORM_NAME': get_option('platform-name')},
    ),
)
