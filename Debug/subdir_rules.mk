################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1210/ccs/tools/compiler/ti-cgt-c2000_22.6.0.LTS/bin/cl2000" -v28 -ml -mt --float_support=softlib --include_path="C:/TestBed/Control_ACIM_F28335_v1" --include_path="C:/ti/controlSUITE/development_kits/~SupportFiles/F2833x_headers" --include_path="C:/ti/controlSUITE/device_support/f2833x/v132/DSP2833x_common/include" --include_path="C:/ti/controlSUITE/device_support/f2833x/v132/DSP2833x_headers/include" --include_path="C:/ti/controlSUITE/libs/app_libs/motor_control/drivers/f2833x_v2.0/~Docs" --include_path="C:/ti/controlSUITE/libs/app_libs/motor_control/math_blocks/v4.0/~Docs" --include_path="C:/ti/controlSUITE/libs/math/IQmath/v15c/include" --include_path="C:/TestBed/Control_ACIM_F28335_v1/controlBlocks" --include_path="C:/ti/ccs1210/ccs/tools/compiler/ti-cgt-c2000_22.6.0.LTS/include" -g --diag_warning=225 --diag_wrap=off --display_error_number --abi=coffabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

%.obj: ../%.asm $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1210/ccs/tools/compiler/ti-cgt-c2000_22.6.0.LTS/bin/cl2000" -v28 -ml -mt --float_support=softlib --include_path="C:/TestBed/Control_ACIM_F28335_v1" --include_path="C:/ti/controlSUITE/development_kits/~SupportFiles/F2833x_headers" --include_path="C:/ti/controlSUITE/device_support/f2833x/v132/DSP2833x_common/include" --include_path="C:/ti/controlSUITE/device_support/f2833x/v132/DSP2833x_headers/include" --include_path="C:/ti/controlSUITE/libs/app_libs/motor_control/drivers/f2833x_v2.0/~Docs" --include_path="C:/ti/controlSUITE/libs/app_libs/motor_control/math_blocks/v4.0/~Docs" --include_path="C:/ti/controlSUITE/libs/math/IQmath/v15c/include" --include_path="C:/TestBed/Control_ACIM_F28335_v1/controlBlocks" --include_path="C:/ti/ccs1210/ccs/tools/compiler/ti-cgt-c2000_22.6.0.LTS/include" -g --diag_warning=225 --diag_wrap=off --display_error_number --abi=coffabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


