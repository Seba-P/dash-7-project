# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 3.5

# Default target executed when no arguments are given to make.
default_target: all

.PHONY : default_target

# Allow only one "make -f Makefile2" at a time, but pass parallelism.
.NOTPARALLEL:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = "C:\Program Files (x86)\CMake\bin\cmake.exe"

# The command to remove a file.
RM = "C:\Program Files (x86)\CMake\bin\cmake.exe" -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = D:\SEBA\BACKUP\Projekty\ARM\dash-7-project\praca_inz\dash-7-project\stack

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = D:\SEBA\BACKUP\Projekty\ARM\dash-7-project\praca_inz\dash-7-project

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake cache editor..."
	"C:\Program Files (x86)\CMake\bin\cmake-gui.exe" -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache

.PHONY : edit_cache/fast

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake to regenerate build system..."
	"C:\Program Files (x86)\CMake\bin\cmake.exe" -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache

.PHONY : rebuild_cache/fast

# The main all target
all: cmake_check_build_system
	$(CMAKE_COMMAND) -E cmake_progress_start D:\SEBA\BACKUP\Projekty\ARM\dash-7-project\praca_inz\dash-7-project\CMakeFiles D:\SEBA\BACKUP\Projekty\ARM\dash-7-project\praca_inz\dash-7-project\CMakeFiles\progress.marks
	$(MAKE) -f CMakeFiles\Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start D:\SEBA\BACKUP\Projekty\ARM\dash-7-project\praca_inz\dash-7-project\CMakeFiles 0
.PHONY : all

# The main clean target
clean:
	$(MAKE) -f CMakeFiles\Makefile2 clean
.PHONY : clean

# The main clean target
clean/fast: clean

.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	$(MAKE) -f CMakeFiles\Makefile2 preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	$(MAKE) -f CMakeFiles\Makefile2 preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles\Makefile.cmake 1
.PHONY : depend

#=============================================================================
# Target rules for targets named framework

# Build rule for target.
framework: cmake_check_build_system
	$(MAKE) -f CMakeFiles\Makefile2 framework
.PHONY : framework

# fast build rule for target.
framework/fast:
	$(MAKE) -f framework\CMakeFiles\framework.dir\build.make framework/CMakeFiles/framework.dir/build
.PHONY : framework/fast

#=============================================================================
# Target rules for targets named PLATFORM

# Build rule for target.
PLATFORM: cmake_check_build_system
	$(MAKE) -f CMakeFiles\Makefile2 PLATFORM
.PHONY : PLATFORM

# fast build rule for target.
PLATFORM/fast:
	$(MAKE) -f framework\hal\platforms\platform\CMakeFiles\PLATFORM.dir\build.make framework/hal/platforms/platform/CMakeFiles/PLATFORM.dir/build
.PHONY : PLATFORM/fast

#=============================================================================
# Target rules for targets named CHIP_EFM32WG

# Build rule for target.
CHIP_EFM32WG: cmake_check_build_system
	$(MAKE) -f CMakeFiles\Makefile2 CHIP_EFM32WG
.PHONY : CHIP_EFM32WG

# fast build rule for target.
CHIP_EFM32WG/fast:
	$(MAKE) -f framework\hal\platforms\platform\chips\efm32wg\CMakeFiles\CHIP_EFM32WG.dir\build.make framework/hal/platforms/platform/chips/efm32wg/CMakeFiles/CHIP_EFM32WG.dir/build
.PHONY : CHIP_EFM32WG/fast

#=============================================================================
# Target rules for targets named CHIP_CC1101

# Build rule for target.
CHIP_CC1101: cmake_check_build_system
	$(MAKE) -f CMakeFiles\Makefile2 CHIP_CC1101
.PHONY : CHIP_CC1101

# fast build rule for target.
CHIP_CC1101/fast:
	$(MAKE) -f framework\hal\platforms\platform\chips\cc1101\CMakeFiles\CHIP_CC1101.dir\build.make framework/hal/platforms/platform/chips/cc1101/CMakeFiles/CHIP_CC1101.dir/build
.PHONY : CHIP_CC1101/fast

#=============================================================================
# Target rules for targets named FRAMEWORK_COMPONENT_cli

# Build rule for target.
FRAMEWORK_COMPONENT_cli: cmake_check_build_system
	$(MAKE) -f CMakeFiles\Makefile2 FRAMEWORK_COMPONENT_cli
.PHONY : FRAMEWORK_COMPONENT_cli

# fast build rule for target.
FRAMEWORK_COMPONENT_cli/fast:
	$(MAKE) -f framework\components\cli\CMakeFiles\FRAMEWORK_COMPONENT_cli.dir\build.make framework/components/cli/CMakeFiles/FRAMEWORK_COMPONENT_cli.dir/build
.PHONY : FRAMEWORK_COMPONENT_cli/fast

#=============================================================================
# Target rules for targets named FRAMEWORK_COMPONENT_console

# Build rule for target.
FRAMEWORK_COMPONENT_console: cmake_check_build_system
	$(MAKE) -f CMakeFiles\Makefile2 FRAMEWORK_COMPONENT_console
.PHONY : FRAMEWORK_COMPONENT_console

# fast build rule for target.
FRAMEWORK_COMPONENT_console/fast:
	$(MAKE) -f framework\components\console\CMakeFiles\FRAMEWORK_COMPONENT_console.dir\build.make framework/components/console/CMakeFiles/FRAMEWORK_COMPONENT_console.dir/build
.PHONY : FRAMEWORK_COMPONENT_console/fast

#=============================================================================
# Target rules for targets named FRAMEWORK_COMPONENT_crc

# Build rule for target.
FRAMEWORK_COMPONENT_crc: cmake_check_build_system
	$(MAKE) -f CMakeFiles\Makefile2 FRAMEWORK_COMPONENT_crc
.PHONY : FRAMEWORK_COMPONENT_crc

# fast build rule for target.
FRAMEWORK_COMPONENT_crc/fast:
	$(MAKE) -f framework\components\crc\CMakeFiles\FRAMEWORK_COMPONENT_crc.dir\build.make framework/components/crc/CMakeFiles/FRAMEWORK_COMPONENT_crc.dir/build
.PHONY : FRAMEWORK_COMPONENT_crc/fast

#=============================================================================
# Target rules for targets named FRAMEWORK_COMPONENT_fec

# Build rule for target.
FRAMEWORK_COMPONENT_fec: cmake_check_build_system
	$(MAKE) -f CMakeFiles\Makefile2 FRAMEWORK_COMPONENT_fec
.PHONY : FRAMEWORK_COMPONENT_fec

# fast build rule for target.
FRAMEWORK_COMPONENT_fec/fast:
	$(MAKE) -f framework\components\fec\CMakeFiles\FRAMEWORK_COMPONENT_fec.dir\build.make framework/components/fec/CMakeFiles/FRAMEWORK_COMPONENT_fec.dir/build
.PHONY : FRAMEWORK_COMPONENT_fec/fast

#=============================================================================
# Target rules for targets named FRAMEWORK_COMPONENT_fifo

# Build rule for target.
FRAMEWORK_COMPONENT_fifo: cmake_check_build_system
	$(MAKE) -f CMakeFiles\Makefile2 FRAMEWORK_COMPONENT_fifo
.PHONY : FRAMEWORK_COMPONENT_fifo

# fast build rule for target.
FRAMEWORK_COMPONENT_fifo/fast:
	$(MAKE) -f framework\components\fifo\CMakeFiles\FRAMEWORK_COMPONENT_fifo.dir\build.make framework/components/fifo/CMakeFiles/FRAMEWORK_COMPONENT_fifo.dir/build
.PHONY : FRAMEWORK_COMPONENT_fifo/fast

#=============================================================================
# Target rules for targets named FRAMEWORK_COMPONENT_log

# Build rule for target.
FRAMEWORK_COMPONENT_log: cmake_check_build_system
	$(MAKE) -f CMakeFiles\Makefile2 FRAMEWORK_COMPONENT_log
.PHONY : FRAMEWORK_COMPONENT_log

# fast build rule for target.
FRAMEWORK_COMPONENT_log/fast:
	$(MAKE) -f framework\components\log\CMakeFiles\FRAMEWORK_COMPONENT_log.dir\build.make framework/components/log/CMakeFiles/FRAMEWORK_COMPONENT_log.dir/build
.PHONY : FRAMEWORK_COMPONENT_log/fast

#=============================================================================
# Target rules for targets named FRAMEWORK_COMPONENT_node_globals

# Build rule for target.
FRAMEWORK_COMPONENT_node_globals: cmake_check_build_system
	$(MAKE) -f CMakeFiles\Makefile2 FRAMEWORK_COMPONENT_node_globals
.PHONY : FRAMEWORK_COMPONENT_node_globals

# fast build rule for target.
FRAMEWORK_COMPONENT_node_globals/fast:
	$(MAKE) -f framework\components\node_globals\CMakeFiles\FRAMEWORK_COMPONENT_node_globals.dir\build.make framework/components/node_globals/CMakeFiles/FRAMEWORK_COMPONENT_node_globals.dir/build
.PHONY : FRAMEWORK_COMPONENT_node_globals/fast

#=============================================================================
# Target rules for targets named FRAMEWORK_COMPONENT_random

# Build rule for target.
FRAMEWORK_COMPONENT_random: cmake_check_build_system
	$(MAKE) -f CMakeFiles\Makefile2 FRAMEWORK_COMPONENT_random
.PHONY : FRAMEWORK_COMPONENT_random

# fast build rule for target.
FRAMEWORK_COMPONENT_random/fast:
	$(MAKE) -f framework\components\random\CMakeFiles\FRAMEWORK_COMPONENT_random.dir\build.make framework/components/random/CMakeFiles/FRAMEWORK_COMPONENT_random.dir/build
.PHONY : FRAMEWORK_COMPONENT_random/fast

#=============================================================================
# Target rules for targets named FRAMEWORK_COMPONENT_scheduler

# Build rule for target.
FRAMEWORK_COMPONENT_scheduler: cmake_check_build_system
	$(MAKE) -f CMakeFiles\Makefile2 FRAMEWORK_COMPONENT_scheduler
.PHONY : FRAMEWORK_COMPONENT_scheduler

# fast build rule for target.
FRAMEWORK_COMPONENT_scheduler/fast:
	$(MAKE) -f framework\components\scheduler\CMakeFiles\FRAMEWORK_COMPONENT_scheduler.dir\build.make framework/components/scheduler/CMakeFiles/FRAMEWORK_COMPONENT_scheduler.dir/build
.PHONY : FRAMEWORK_COMPONENT_scheduler/fast

#=============================================================================
# Target rules for targets named FRAMEWORK_COMPONENT_shell

# Build rule for target.
FRAMEWORK_COMPONENT_shell: cmake_check_build_system
	$(MAKE) -f CMakeFiles\Makefile2 FRAMEWORK_COMPONENT_shell
.PHONY : FRAMEWORK_COMPONENT_shell

# fast build rule for target.
FRAMEWORK_COMPONENT_shell/fast:
	$(MAKE) -f framework\components\shell\CMakeFiles\FRAMEWORK_COMPONENT_shell.dir\build.make framework/components/shell/CMakeFiles/FRAMEWORK_COMPONENT_shell.dir/build
.PHONY : FRAMEWORK_COMPONENT_shell/fast

#=============================================================================
# Target rules for targets named FRAMEWORK_COMPONENT_timer

# Build rule for target.
FRAMEWORK_COMPONENT_timer: cmake_check_build_system
	$(MAKE) -f CMakeFiles\Makefile2 FRAMEWORK_COMPONENT_timer
.PHONY : FRAMEWORK_COMPONENT_timer

# fast build rule for target.
FRAMEWORK_COMPONENT_timer/fast:
	$(MAKE) -f framework\components\timer\CMakeFiles\FRAMEWORK_COMPONENT_timer.dir\build.make framework/components/timer/CMakeFiles/FRAMEWORK_COMPONENT_timer.dir/build
.PHONY : FRAMEWORK_COMPONENT_timer/fast

#=============================================================================
# Target rules for targets named d7ap

# Build rule for target.
d7ap: cmake_check_build_system
	$(MAKE) -f CMakeFiles\Makefile2 d7ap
.PHONY : d7ap

# fast build rule for target.
d7ap/fast:
	$(MAKE) -f modules\d7ap\CMakeFiles\d7ap.dir\build.make modules/d7ap/CMakeFiles/d7ap.dir/build
.PHONY : d7ap/fast

#=============================================================================
# Target rules for targets named continuous_tx.axf

# Build rule for target.
continuous_tx.axf: cmake_check_build_system
	$(MAKE) -f CMakeFiles\Makefile2 continuous_tx.axf
.PHONY : continuous_tx.axf

# fast build rule for target.
continuous_tx.axf/fast:
	$(MAKE) -f apps\continuous_tx\CMakeFiles\continuous_tx.axf.dir\build.make apps/continuous_tx/CMakeFiles/continuous_tx.axf.dir/build
.PHONY : continuous_tx.axf/fast

#=============================================================================
# Target rules for targets named continuous_tx.elf

# Build rule for target.
continuous_tx.elf: cmake_check_build_system
	$(MAKE) -f CMakeFiles\Makefile2 continuous_tx.elf
.PHONY : continuous_tx.elf

# fast build rule for target.
continuous_tx.elf/fast:
	$(MAKE) -f apps\continuous_tx\CMakeFiles\continuous_tx.elf.dir\build.make apps/continuous_tx/CMakeFiles/continuous_tx.elf.dir/build
.PHONY : continuous_tx.elf/fast

#=============================================================================
# Target rules for targets named flash-continuous_tx

# Build rule for target.
flash-continuous_tx: cmake_check_build_system
	$(MAKE) -f CMakeFiles\Makefile2 flash-continuous_tx
.PHONY : flash-continuous_tx

# fast build rule for target.
flash-continuous_tx/fast:
	$(MAKE) -f apps\continuous_tx\CMakeFiles\flash-continuous_tx.dir\build.make apps/continuous_tx/CMakeFiles/flash-continuous_tx.dir/build
.PHONY : flash-continuous_tx/fast

#=============================================================================
# Target rules for targets named d7ap_test.axf

# Build rule for target.
d7ap_test.axf: cmake_check_build_system
	$(MAKE) -f CMakeFiles\Makefile2 d7ap_test.axf
.PHONY : d7ap_test.axf

# fast build rule for target.
d7ap_test.axf/fast:
	$(MAKE) -f apps\d7ap_test\CMakeFiles\d7ap_test.axf.dir\build.make apps/d7ap_test/CMakeFiles/d7ap_test.axf.dir/build
.PHONY : d7ap_test.axf/fast

#=============================================================================
# Target rules for targets named d7ap_test.elf

# Build rule for target.
d7ap_test.elf: cmake_check_build_system
	$(MAKE) -f CMakeFiles\Makefile2 d7ap_test.elf
.PHONY : d7ap_test.elf

# fast build rule for target.
d7ap_test.elf/fast:
	$(MAKE) -f apps\d7ap_test\CMakeFiles\d7ap_test.elf.dir\build.make apps/d7ap_test/CMakeFiles/d7ap_test.elf.dir/build
.PHONY : d7ap_test.elf/fast

#=============================================================================
# Target rules for targets named flash-d7ap_test

# Build rule for target.
flash-d7ap_test: cmake_check_build_system
	$(MAKE) -f CMakeFiles\Makefile2 flash-d7ap_test
.PHONY : flash-d7ap_test

# fast build rule for target.
flash-d7ap_test/fast:
	$(MAKE) -f apps\d7ap_test\CMakeFiles\flash-d7ap_test.dir\build.make apps/d7ap_test/CMakeFiles/flash-d7ap_test.dir/build
.PHONY : flash-d7ap_test/fast

#=============================================================================
# Target rules for targets named efm32_sensor.axf

# Build rule for target.
efm32_sensor.axf: cmake_check_build_system
	$(MAKE) -f CMakeFiles\Makefile2 efm32_sensor.axf
.PHONY : efm32_sensor.axf

# fast build rule for target.
efm32_sensor.axf/fast:
	$(MAKE) -f apps\efm32_sensor\CMakeFiles\efm32_sensor.axf.dir\build.make apps/efm32_sensor/CMakeFiles/efm32_sensor.axf.dir/build
.PHONY : efm32_sensor.axf/fast

#=============================================================================
# Target rules for targets named efm32_sensor.elf

# Build rule for target.
efm32_sensor.elf: cmake_check_build_system
	$(MAKE) -f CMakeFiles\Makefile2 efm32_sensor.elf
.PHONY : efm32_sensor.elf

# fast build rule for target.
efm32_sensor.elf/fast:
	$(MAKE) -f apps\efm32_sensor\CMakeFiles\efm32_sensor.elf.dir\build.make apps/efm32_sensor/CMakeFiles/efm32_sensor.elf.dir/build
.PHONY : efm32_sensor.elf/fast

#=============================================================================
# Target rules for targets named flash-efm32_sensor

# Build rule for target.
flash-efm32_sensor: cmake_check_build_system
	$(MAKE) -f CMakeFiles\Makefile2 flash-efm32_sensor
.PHONY : flash-efm32_sensor

# fast build rule for target.
flash-efm32_sensor/fast:
	$(MAKE) -f apps\efm32_sensor\CMakeFiles\flash-efm32_sensor.dir\build.make apps/efm32_sensor/CMakeFiles/flash-efm32_sensor.dir/build
.PHONY : flash-efm32_sensor/fast

#=============================================================================
# Target rules for targets named gateway.axf

# Build rule for target.
gateway.axf: cmake_check_build_system
	$(MAKE) -f CMakeFiles\Makefile2 gateway.axf
.PHONY : gateway.axf

# fast build rule for target.
gateway.axf/fast:
	$(MAKE) -f apps\gateway\CMakeFiles\gateway.axf.dir\build.make apps/gateway/CMakeFiles/gateway.axf.dir/build
.PHONY : gateway.axf/fast

#=============================================================================
# Target rules for targets named gateway.elf

# Build rule for target.
gateway.elf: cmake_check_build_system
	$(MAKE) -f CMakeFiles\Makefile2 gateway.elf
.PHONY : gateway.elf

# fast build rule for target.
gateway.elf/fast:
	$(MAKE) -f apps\gateway\CMakeFiles\gateway.elf.dir\build.make apps/gateway/CMakeFiles/gateway.elf.dir/build
.PHONY : gateway.elf/fast

#=============================================================================
# Target rules for targets named flash-gateway

# Build rule for target.
flash-gateway: cmake_check_build_system
	$(MAKE) -f CMakeFiles\Makefile2 flash-gateway
.PHONY : flash-gateway

# fast build rule for target.
flash-gateway/fast:
	$(MAKE) -f apps\gateway\CMakeFiles\flash-gateway.dir\build.make apps/gateway/CMakeFiles/flash-gateway.dir/build
.PHONY : flash-gateway/fast

#=============================================================================
# Target rules for targets named noise_logger.axf

# Build rule for target.
noise_logger.axf: cmake_check_build_system
	$(MAKE) -f CMakeFiles\Makefile2 noise_logger.axf
.PHONY : noise_logger.axf

# fast build rule for target.
noise_logger.axf/fast:
	$(MAKE) -f apps\noise_logger\CMakeFiles\noise_logger.axf.dir\build.make apps/noise_logger/CMakeFiles/noise_logger.axf.dir/build
.PHONY : noise_logger.axf/fast

#=============================================================================
# Target rules for targets named noise_logger.elf

# Build rule for target.
noise_logger.elf: cmake_check_build_system
	$(MAKE) -f CMakeFiles\Makefile2 noise_logger.elf
.PHONY : noise_logger.elf

# fast build rule for target.
noise_logger.elf/fast:
	$(MAKE) -f apps\noise_logger\CMakeFiles\noise_logger.elf.dir\build.make apps/noise_logger/CMakeFiles/noise_logger.elf.dir/build
.PHONY : noise_logger.elf/fast

#=============================================================================
# Target rules for targets named flash-noise_logger

# Build rule for target.
flash-noise_logger: cmake_check_build_system
	$(MAKE) -f CMakeFiles\Makefile2 flash-noise_logger
.PHONY : flash-noise_logger

# fast build rule for target.
flash-noise_logger/fast:
	$(MAKE) -f apps\noise_logger\CMakeFiles\flash-noise_logger.dir\build.make apps/noise_logger/CMakeFiles/flash-noise_logger.dir/build
.PHONY : flash-noise_logger/fast

#=============================================================================
# Target rules for targets named per_test.axf

# Build rule for target.
per_test.axf: cmake_check_build_system
	$(MAKE) -f CMakeFiles\Makefile2 per_test.axf
.PHONY : per_test.axf

# fast build rule for target.
per_test.axf/fast:
	$(MAKE) -f apps\per_test\CMakeFiles\per_test.axf.dir\build.make apps/per_test/CMakeFiles/per_test.axf.dir/build
.PHONY : per_test.axf/fast

#=============================================================================
# Target rules for targets named per_test.elf

# Build rule for target.
per_test.elf: cmake_check_build_system
	$(MAKE) -f CMakeFiles\Makefile2 per_test.elf
.PHONY : per_test.elf

# fast build rule for target.
per_test.elf/fast:
	$(MAKE) -f apps\per_test\CMakeFiles\per_test.elf.dir\build.make apps/per_test/CMakeFiles/per_test.elf.dir/build
.PHONY : per_test.elf/fast

#=============================================================================
# Target rules for targets named flash-per_test

# Build rule for target.
flash-per_test: cmake_check_build_system
	$(MAKE) -f CMakeFiles\Makefile2 flash-per_test
.PHONY : flash-per_test

# fast build rule for target.
flash-per_test/fast:
	$(MAKE) -f apps\per_test\CMakeFiles\flash-per_test.dir\build.make apps/per_test/CMakeFiles/flash-per_test.dir/build
.PHONY : flash-per_test/fast

#=============================================================================
# Target rules for targets named phy_test.axf

# Build rule for target.
phy_test.axf: cmake_check_build_system
	$(MAKE) -f CMakeFiles\Makefile2 phy_test.axf
.PHONY : phy_test.axf

# fast build rule for target.
phy_test.axf/fast:
	$(MAKE) -f apps\phy_test\CMakeFiles\phy_test.axf.dir\build.make apps/phy_test/CMakeFiles/phy_test.axf.dir/build
.PHONY : phy_test.axf/fast

#=============================================================================
# Target rules for targets named phy_test.elf

# Build rule for target.
phy_test.elf: cmake_check_build_system
	$(MAKE) -f CMakeFiles\Makefile2 phy_test.elf
.PHONY : phy_test.elf

# fast build rule for target.
phy_test.elf/fast:
	$(MAKE) -f apps\phy_test\CMakeFiles\phy_test.elf.dir\build.make apps/phy_test/CMakeFiles/phy_test.elf.dir/build
.PHONY : phy_test.elf/fast

#=============================================================================
# Target rules for targets named flash-phy_test

# Build rule for target.
flash-phy_test: cmake_check_build_system
	$(MAKE) -f CMakeFiles\Makefile2 flash-phy_test
.PHONY : flash-phy_test

# fast build rule for target.
flash-phy_test/fast:
	$(MAKE) -f apps\phy_test\CMakeFiles\flash-phy_test.dir\build.make apps/phy_test/CMakeFiles/flash-phy_test.dir/build
.PHONY : flash-phy_test/fast

#=============================================================================
# Target rules for targets named simple_leds.axf

# Build rule for target.
simple_leds.axf: cmake_check_build_system
	$(MAKE) -f CMakeFiles\Makefile2 simple_leds.axf
.PHONY : simple_leds.axf

# fast build rule for target.
simple_leds.axf/fast:
	$(MAKE) -f apps\simple_leds\CMakeFiles\simple_leds.axf.dir\build.make apps/simple_leds/CMakeFiles/simple_leds.axf.dir/build
.PHONY : simple_leds.axf/fast

#=============================================================================
# Target rules for targets named simple_leds.elf

# Build rule for target.
simple_leds.elf: cmake_check_build_system
	$(MAKE) -f CMakeFiles\Makefile2 simple_leds.elf
.PHONY : simple_leds.elf

# fast build rule for target.
simple_leds.elf/fast:
	$(MAKE) -f apps\simple_leds\CMakeFiles\simple_leds.elf.dir\build.make apps/simple_leds/CMakeFiles/simple_leds.elf.dir/build
.PHONY : simple_leds.elf/fast

#=============================================================================
# Target rules for targets named flash-simple_leds

# Build rule for target.
flash-simple_leds: cmake_check_build_system
	$(MAKE) -f CMakeFiles\Makefile2 flash-simple_leds
.PHONY : flash-simple_leds

# fast build rule for target.
flash-simple_leds/fast:
	$(MAKE) -f apps\simple_leds\CMakeFiles\flash-simple_leds.dir\build.make apps/simple_leds/CMakeFiles/flash-simple_leds.dir/build
.PHONY : flash-simple_leds/fast

# Help Target
help:
	@echo The following are some of the valid targets for this Makefile:
	@echo ... all (the default if no target is provided)
	@echo ... clean
	@echo ... depend
	@echo ... edit_cache
	@echo ... rebuild_cache
	@echo ... framework
	@echo ... PLATFORM
	@echo ... CHIP_EFM32WG
	@echo ... CHIP_CC1101
	@echo ... FRAMEWORK_COMPONENT_cli
	@echo ... FRAMEWORK_COMPONENT_console
	@echo ... FRAMEWORK_COMPONENT_crc
	@echo ... FRAMEWORK_COMPONENT_fec
	@echo ... FRAMEWORK_COMPONENT_fifo
	@echo ... FRAMEWORK_COMPONENT_log
	@echo ... FRAMEWORK_COMPONENT_node_globals
	@echo ... FRAMEWORK_COMPONENT_random
	@echo ... FRAMEWORK_COMPONENT_scheduler
	@echo ... FRAMEWORK_COMPONENT_shell
	@echo ... FRAMEWORK_COMPONENT_timer
	@echo ... d7ap
	@echo ... continuous_tx.axf
	@echo ... continuous_tx.elf
	@echo ... flash-continuous_tx
	@echo ... d7ap_test.axf
	@echo ... d7ap_test.elf
	@echo ... flash-d7ap_test
	@echo ... efm32_sensor.axf
	@echo ... efm32_sensor.elf
	@echo ... flash-efm32_sensor
	@echo ... gateway.axf
	@echo ... gateway.elf
	@echo ... flash-gateway
	@echo ... noise_logger.axf
	@echo ... noise_logger.elf
	@echo ... flash-noise_logger
	@echo ... per_test.axf
	@echo ... per_test.elf
	@echo ... flash-per_test
	@echo ... phy_test.axf
	@echo ... phy_test.elf
	@echo ... flash-phy_test
	@echo ... simple_leds.axf
	@echo ... simple_leds.elf
	@echo ... flash-simple_leds
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles\Makefile.cmake 0
.PHONY : cmake_check_build_system

