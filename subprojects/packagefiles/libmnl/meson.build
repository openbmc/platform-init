project(
    'libmnl',
    'c',
    version: '1.0.5',
    meson_version: '>=0.56.0',
    default_options: [
        'werror=false',
        'warning_level=0',
    ],
)

libmnl_inc = include_directories('include')
libmnl_build_inc = include_directories('.')

cc = meson.get_compiler('c')

config_h = configuration_data()
config_h.set(
    'HAVE_VISIBILITY_HIDDEN',
    cc.has_argument('-fvisibility=hidden') ? 1 : false,
)

configure_file(
    output: 'config.h',
    configuration: config_h,
)

libmnl_lib = library(
    'mnl',
    files(
        'src/attr.c',
        'src/callback.c',
        'src/nlmsg.c',
        'src/socket.c',
    ),
    c_args: [
        '-D_FILE_OFFSET_BITS=64',
        '-D_REENTRANT',
        '-Wno-error',
    ],
    include_directories: [
        libmnl_build_inc,
        libmnl_inc,
    ],
    override_options: [
        'werror=false',
        'warning_level=0',
    ],
)

libmnl_dep = declare_dependency(
    include_directories: libmnl_inc,
    link_with: libmnl_lib,
)
