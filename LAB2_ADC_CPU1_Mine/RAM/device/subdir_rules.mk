################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
device/%.obj: ../device/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1220/ccs/tools/compiler/ti-cgt-c2000_22.6.0.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 --fp_mode=relaxed --include_path="C:/Users/LG/workspace_CCS12/LAB2_ADC_CPU1_Mine" --include_path="C:/ti/ccs1220/ccs/tools/compiler/ti-cgt-c2000_22.6.0.LTS/include" --include_path="C:/Users/LG/workspace_CCS12/LAB2_ADC_CPU1_Mine/device" --include_path="C:/Users/LG/workspace_CCS12/LAB2_ADC_CPU1_Mine/device/device_support/common/Include" --include_path="C:/Users/LG/workspace_CCS12/LAB2_ADC_CPU1_Mine/device/device_support/headers/Include" --include_path="C:/Users/LG/workspace_CCS12/LAB2_ADC_CPU1_Mine/device/driverlib" --advice:performance=all --define=DEBUG --define=CPU1 --define=_DUAL_HEADERS -g --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --gen_data_subsections=on --abi=coffabi --preproc_with_compile --preproc_dependency="device/$(basename $(<F)).d_raw" --obj_directory="device" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


