################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
eQEP_Calc/eqep_ex1_calculation.obj: ../eQEP_Calc/eqep_ex1_calculation.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.6.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 -Ooff --include_path="D:/AcademicRelated/Y4S1/FYP/Codes/Delfino/SmartWinch" --include_path="D:/AcademicRelated/Y4S1/FYP/Codes/Delfino/SmartWinch/device" --include_path="C:/ti/c2000/C2000Ware_1_00_03_00/driverlib/f2837xs/driverlib" --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.6.LTS/include" --define=DEBUG --define=CPU1 --diag_suppress=10063 --diag_warning=225 --diag_wrap=off --display_error_number --preproc_with_compile --preproc_dependency="eQEP_Calc/eqep_ex1_calculation.d_raw" --obj_directory="eQEP_Calc" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


