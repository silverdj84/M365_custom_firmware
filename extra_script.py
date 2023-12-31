Import("env", "projenv")

# access to global build environment
print(env)

# access to project build environment (is used source files in "src" folder)
print(projenv)

# Custom BIN from ELF
env.AddPostAction(
	"$BUILD_DIR/${PROGNAME}.elf",
	env.VerboseAction(" ".join([
				"$OBJCOPY",
				"-O",
				"binary",
				"$TARGET",
				"$BUILD_DIR/${PROGNAME}.bin"
			]), "Building $TARGET"))

# Custom HEX from ELF
env.AddPostAction(
    "$BUILD_DIR/${PROGNAME}.elf",
    env.VerboseAction(" ".join([
        "$OBJCOPY", "-O", "ihex", "-R", ".eeprom",
        "$BUILD_DIR/${PROGNAME}.elf", "$BUILD_DIR/${PROGNAME}.hex"
    ]), "Building $BUILD_DIR/${PROGNAME}.hex")
)
