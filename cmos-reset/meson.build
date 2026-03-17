phosphor_dbus_interfaces_dep = dependency('phosphor-dbus-interfaces')
phosphor_logging_dep = dependency('phosphor-logging')
sdbusplus_dep = dependency('sdbusplus')

cmos_platform_srcs = files('../meta/catalina.cpp')
cmos_util_srcs = files('../gpio.cpp', '../utilities.cpp')

executable(
    'cmos-reset',
    ['cmos_reset.cpp', 'cmos_reset_main.cpp', 'host_power.cpp'] + cmos_platform_srcs + cmos_util_srcs,
    dependencies: [
        gpiodcxx_dep,
        cli11_dep,
        phosphor_dbus_interfaces_dep,
        phosphor_logging_dep,
        sdbusplus_dep,
    ],
    include_directories: ['.', '..'],
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

conf = configuration_data()
conf.set(
    'HOST_STATE_TRANSITION_TIMEOUT',
    get_option('host-state-transition-timeout'),
)
configure_file(output: 'config.h', configuration: conf)
