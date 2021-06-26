# INCLUDE_DIRS - Directories used to search for include files.
#                 This generates a -I for each directory
# SRC_DIRS - Directories containing source files to build into the library
# SRC_FILES - Specific source files to build into library. Helpful when not all code
#             in a directory can be built for test (hopefully a temporary situation)
# TEST_SRC_DIRS - Directories containing unit test code build into the unit test runner
#             These do not go in a library. They are explicitly included in the test runner
# TEST_SRC_FILES - Specific source files to build into the unit test runner
#             These do not go in a library. They are explicitly included in the test runner
# MOCKS_SRC_DIRS - Directories containing mock source files to build into the test runner
#             These do not go in a library. They are explicitly included in the test runner
# ------
#
MAKEFILE_DIR=$(dir $(realpath $(firstword $(MAKEFILE_LIST))))
PROJECT_DIR=$(realpath $(MAKEFILE_DIR)..)
TEST_DIR=
CPPUTEST_HOME=cpputest

# project source files path
SRC_FILES=$(PROJECT_DIR)/Core/Src/find.c \
		  $(PROJECT_DIR)/Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_init_f32.c \
		  $(PROJECT_DIR)/Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_add_f32.c \
		  $(PROJECT_DIR)/Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_f32.c \
		  $(PROJECT_DIR)/Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_sub_f32.c \
		  $(PROJECT_DIR)/Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_f32.c \
		  $(PROJECT_DIR)/Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_inverse_f32.c \
		  $(PROJECT_DIR)/Core/Src/kalman.c \
		  $(PROJECT_DIR)/Core/Src/auxiliary_mat.c \
		  $(PROJECT_DIR)/Core/Src/battery_model.c \
		  $(PROJECT_DIR)/Core/Src/crc.c \
		  $(PROJECT_DIR)/Core/Src/fsm_charger.c \
		  $(PROJECT_DIR)/Core/Src/fsm.c


SRC_DIRS=

# project header files path
INCLUDE_DIRS=$(PROJECT_DIR)/Core/Inc \
			 $(PROJECT_DIR)/Drivers/STM32F4xx_HAL_Driver/Inc \
			 $(PROJECT_DIR)/Drivers/CMSIS/DSP/Include \
			 $(PROJECT_DIR)/Drivers/CMSIS/Device/ST/STM32F4xx/Include \
			 $(PROJECT_DIR)/Drivers/CMSIS/Include \
			 $(CPPUTEST_HOME)/include/ \
			 $(PROJECT_DIR)/Test/inc

# add code coverage
CPPUTEST_USE_GCOV=Y

# specify objs dir
CPPUTEST_OBJS_DIR=objs

## specify where the test code is located
TEST_SRC_DIRS=src

## what to call the test binary
TEST_TARGET=bms_test

# run MakefileWorker.mk with the variables defined here
include $(CPPUTEST_HOME)/build/MakefileWorker.mk
