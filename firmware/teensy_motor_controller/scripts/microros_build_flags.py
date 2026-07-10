Import("env")

microros_c_flags = [
    "-D_POSIX_TIMERS=1",
    "-Wno-implicit-function-declaration",
]

env.AppendUnique(CFLAGS=microros_c_flags)
