# PlatformIO pre-script for the cart and Sun firmware: stamp the build with its time as UTC Unix
# seconds (SOLAR_FW_BUILD_UTC). Unlike the compiler's local __DATE__/__TIME__ it never runs
# backwards (no daylight-saving jumps), so the Sun can safely update only carts whose build is
# OLDER than its image. Only the main file gets the define, so a build recompiles just that file.
import time

Import("env")

STAMP = int(time.time())


def add_stamp(env, node):
    return env.Object(node, CCFLAGS=env["CCFLAGS"] + ["-DSOLAR_FW_BUILD_UTC=%d" % STAMP])


env.AddBuildMiddleware(add_stamp, "*planet_main.cpp")
env.AddBuildMiddleware(add_stamp, "*sun_main.cpp")
