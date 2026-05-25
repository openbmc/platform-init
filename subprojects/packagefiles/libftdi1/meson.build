project(
    'libftdi1',
    'c',
    version: '1.5',
    meson_version: '>=0.56.0',
    default_options: [
        'werror=false',
        'warning_level=0',
    ],
)

libusb_project = subproject(
    'libusb',
    default_options: [
        'tests=false',
        'werror=false',
        'warning_level=0',
    ],
)
libusb_dep = libusb_project.get_variable('libusb_dep')

version_conf = configuration_data()
version_conf.set('MAJOR_VERSION', '1')
version_conf.set('MINOR_VERSION', '5')
version_conf.set('VERSION_STRING', meson.project_version())
version_conf.set('SNAPSHOT_VERSION', 'unknown')

libftdi1_build_inc = include_directories('.')
libftdi1_src_inc = include_directories('src')

configure_file(
    input: 'src/ftdi_version_i.h.in',
    output: 'ftdi_version_i.h',
    configuration: version_conf,
)

libftdi1_lib = library(
    'ftdi1',
    files(
        'src/ftdi.c',
        'src/ftdi_stream.c',
    ),
    c_args: [
        '-Wno-error',
    ],
    dependencies: libusb_dep,
    include_directories: [
        libftdi1_build_inc,
        libftdi1_src_inc,
    ],
    override_options: [
        'werror=false',
        'warning_level=0',
    ],
)

libftdi1_dep = declare_dependency(
    dependencies: libusb_dep,
    include_directories: libftdi1_src_inc,
    link_with: libftdi1_lib,
)
