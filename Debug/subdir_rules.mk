################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'MSP430 Compiler - building file: "$<"'
	"F:/ccs/tools/compiler/ti-cgt-msp430_21.6.2.LTS/bin/cl430" -vmsp --use_hw_mpy=none --include_path="F:/ccs/ccs_base/msp430/include" --include_path="C:/Users/devma/workspace_ccstheia/projeto-final" --include_path="F:/ccs/tools/compiler/ti-cgt-msp430_21.6.2.LTS/include" --advice:power=all --define=__MSP430G2553__ -g --printf_support=minimal --diag_warning=225 --diag_wrap=off --display_error_number --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


