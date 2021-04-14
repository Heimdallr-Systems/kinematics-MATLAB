###########################################################################
## Makefile generated for component 'Robot_Control'. 
## 
## Makefile     : Robot_Control_rtw.mk
## Generated on : Wed Apr 14 15:29:02 2021
## Final product: ./Robot_Control.a
## Product type : static-library
## 
###########################################################################

###########################################################################
## MACROS
###########################################################################

# Macro Descriptions:
# PRODUCT_NAME            Name of the system to build
# MAKEFILE                Name of this makefile
# MODELLIB                Static library target

PRODUCT_NAME              = Robot_Control
MAKEFILE                  = Robot_Control_rtw.mk
MATLAB_ROOT               = $(MATLAB_WORKSPACE)/D/Program_Files/MATLAB/R2021a
MATLAB_BIN                = $(MATLAB_WORKSPACE)/D/Program_Files/MATLAB/R2021a/bin
MATLAB_ARCH_BIN           = $(MATLAB_BIN)/win64
START_DIR                 = $(MATLAB_WORKSPACE)/D/Desktop/Capstone/kinematics-MATLAB/SpringProject/codegen/lib/Robot_Control
TGT_FCN_LIB               = GNU
SOLVER_OBJ                = 
CLASSIC_INTERFACE         = 0
MODEL_HAS_DYNAMICALLY_LOADED_SFCNS = 
RELATIVE_PATH_TO_ANCHOR   = .
C_STANDARD_OPTS           = 
CPP_STANDARD_OPTS         = 
MODELLIB                  = Robot_Control.a

###########################################################################
## TOOLCHAIN SPECIFICATIONS
###########################################################################

# Toolchain Name:          GNU GCC for NVIDIA Embedded Processors
# Supported Version(s):    
# ToolchainInfo Version:   2021a
# Specification Revision:  1.0
# 

#-----------
# MACROS
#-----------

CCOUTPUTFLAG  = --output_file=
LDOUTPUTFLAG  = --output_file=
XCOMPILERFLAG =  

TOOLCHAIN_SRCS = 
TOOLCHAIN_INCS = 
TOOLCHAIN_LIBS = -lm -lm -lstdc++

#------------------------
# BUILD TOOL COMMANDS
#------------------------

# Assembler: GNU GCC for NVIDIA Embedded Processors Assembler
AS = as

# C Compiler: GNU GCC for NVIDIA Embedded Processors C Compiler
CC = gcc

# Linker: GNU GCC for NVIDIA Embedded Processors Linker
LD = gcc

# C++ Compiler: GNU GCC for NVIDIA Embedded Processors C++ Compiler
CPP = g++

# C++ Linker: GNU GCC for NVIDIA Embedded Processors C++ Linker
CPP_LD = g++

# Archiver: GNU GCC for NVIDIA Embedded Processors Archiver
AR = ar

# MEX Tool: MEX Tool
MEX_PATH = $(MATLAB_ARCH_BIN)
MEX = "$(MEX_PATH)/mex"

# Download: Download
DOWNLOAD =

# Execute: Execute
EXECUTE = $(PRODUCT)

# Builder: Make Tool
MAKE = make


#-------------------------
# Directives/Utilities
#-------------------------

ASDEBUG             = -g
AS_OUTPUT_FLAG      = -o
CDEBUG              = -g
C_OUTPUT_FLAG       = -o
LDDEBUG             = -g
OUTPUT_FLAG         = -o
CPPDEBUG            = -g
CPP_OUTPUT_FLAG     = -o
CPPLDDEBUG          = -g
OUTPUT_FLAG         = -o
ARDEBUG             =
STATICLIB_OUTPUT_FLAG =
MEX_DEBUG           = -g
RM                  =
ECHO                = echo
MV                  =
RUN                 =

#--------------------------------------
# "Faster Runs" Build Configuration
#--------------------------------------

ARFLAGS              = -r
ASFLAGS              = -c \
                       $(ASFLAGS_ADDITIONAL) \
                       $(INCLUDES)
CFLAGS               = -c \
                       -MMD -MP -MF"$(@:%.o=%.dep)" -MT"$@"  \
                       -O2
CPPFLAGS             = -c \
                       -MMD -MP -MF"$(@:%.o=%.dep)" -MT"$@"  \
                       -fpermissive  \
                       -O2
CPP_LDFLAGS          = -lrt -pthread -ldl
CPP_SHAREDLIB_LDFLAGS  = -shared  \
                         -lrt -pthread -ldl
DOWNLOAD_FLAGS       =
EXECUTE_FLAGS        =
LDFLAGS              = -lrt -pthread -ldl
MEX_CPPFLAGS         =
MEX_CPPLDFLAGS       =
MEX_CFLAGS           =
MEX_LDFLAGS          =
MAKE_FLAGS           = -f $(MAKEFILE)
SHAREDLIB_LDFLAGS    = -shared  \
                       -lrt -pthread -ldl



###########################################################################
## OUTPUT INFO
###########################################################################

PRODUCT = ./Robot_Control.a
PRODUCT_TYPE = "static-library"
BUILD_TYPE = "Static Library"

###########################################################################
## INCLUDE PATHS
###########################################################################

INCLUDES_BUILDINFO = -I$(START_DIR) -I$(MATLAB_WORKSPACE)/D/Desktop/Capstone/kinematics-MATLAB/SpringProject -I$(MATLAB_WORKSPACE)/C/ProgramData/MATLAB/SupportPackages/R2021a/toolbox/target/supportpackages/nvidia/include -I$(MATLAB_WORKSPACE)/C/ProgramData/MATLAB/SupportPackages/R2021a/toolbox/target/supportpackages/nvidia/sources/utils -I$(MATLAB_ROOT)/toolbox/coder/rtiostream/src/utils -I$(MATLAB_ROOT)/extern/include

INCLUDES = $(INCLUDES_BUILDINFO)

###########################################################################
## DEFINES
###########################################################################

DEFINES_ = -D__MW_TARGET_USE_HARDWARE_RESOURCES_H__ -DMW_DL_DATA_PATH="$(START_DIR)" -DMW_SCHED_OTHER=1
DEFINES_CUSTOM = 
DEFINES_SKIPFORSIL = -D__linux__ -DARM_PROJECT -D_USE_TARGET_UDP_ -D_RUNONTARGETHARDWARE_BUILD_ -DSTACK_SIZE=200000
DEFINES_STANDARD = -DMODEL=Robot_Control

DEFINES = $(DEFINES_) $(DEFINES_CUSTOM) $(DEFINES_SKIPFORSIL) $(DEFINES_STANDARD)

###########################################################################
## SOURCE FILES
###########################################################################

SRCS = $(START_DIR)/Robot_Control_data.cpp $(START_DIR)/Robot_Control_initialize.cpp $(START_DIR)/CPos_wrt_B.cpp $(START_DIR)/rotz.cpp $(START_DIR)/rotx.cpp $(START_DIR)/CPos_wrt_I.cpp $(START_DIR)/isequal.cpp $(START_DIR)/IK_Solver_BodyRot_BodyPos.cpp $(START_DIR)/combineVectorElements.cpp $(START_DIR)/findTrue4Elem.cpp $(START_DIR)/svd.cpp $(START_DIR)/svd1.cpp $(START_DIR)/xzsvdc.cpp $(START_DIR)/xnrm2.cpp $(START_DIR)/abs.cpp $(START_DIR)/abs1.cpp $(START_DIR)/power.cpp $(START_DIR)/eml_int_forloop_overflow_check.cpp $(START_DIR)/xdotc.cpp $(START_DIR)/xdot.cpp $(START_DIR)/det.cpp $(START_DIR)/ixamax.cpp $(START_DIR)/diag.cpp $(START_DIR)/norm.cpp $(START_DIR)/error.cpp $(START_DIR)/Robot_Control.cpp $(START_DIR)/eye.cpp $(START_DIR)/roty.cpp $(START_DIR)/inpolygon.cpp $(START_DIR)/manipulability.cpp $(START_DIR)/contactJacobians.cpp $(START_DIR)/recursiveKin.cpp $(START_DIR)/skew.cpp $(START_DIR)/prod.cpp $(START_DIR)/find_pgon_goal.cpp $(START_DIR)/cross.cpp $(START_DIR)/Leg_Controller_B.cpp $(START_DIR)/atan2.cpp $(START_DIR)/atan21.cpp $(START_DIR)/angle.cpp $(START_DIR)/angle1.cpp $(START_DIR)/mldivide.cpp $(START_DIR)/lusolve.cpp $(START_DIR)/ifWhileCond.cpp $(START_DIR)/step_planner_intelligent.cpp $(START_DIR)/centroid_codeGen.cpp $(START_DIR)/sum.cpp $(START_DIR)/blockedSummation.cpp $(START_DIR)/all.cpp $(START_DIR)/allOrAny.cpp $(START_DIR)/CallTheDead.cpp $(START_DIR)/cos.cpp $(START_DIR)/cos1.cpp $(START_DIR)/sin.cpp $(START_DIR)/sin1.cpp $(START_DIR)/sumprod.cpp $(START_DIR)/floor.cpp $(START_DIR)/sqrt.cpp $(START_DIR)/sqrt1.cpp $(START_DIR)/xscal.cpp $(START_DIR)/xaxpy.cpp $(START_DIR)/xrotg.cpp $(START_DIR)/xrot.cpp $(START_DIR)/xswap.cpp $(START_DIR)/xgetrf.cpp $(START_DIR)/xzgetrf.cpp $(START_DIR)/xgeru.cpp $(START_DIR)/xger.cpp $(START_DIR)/getUp.cpp $(START_DIR)/sort.cpp $(START_DIR)/sortIdx.cpp $(START_DIR)/sign.cpp $(START_DIR)/sign1.cpp $(START_DIR)/exp.cpp $(START_DIR)/exp1.cpp $(START_DIR)/Body_Pose_Controller.cpp $(START_DIR)/circshift.cpp $(START_DIR)/Leg_Controller.cpp $(START_DIR)/Robot_Control_rtwutil.cpp $(MATLAB_WORKSPACE)/C/ProgramData/MATLAB/SupportPackages/R2021a/toolbox/target/supportpackages/nvidia/sources/utils/MW_nvidia_init.c

ALL_SRCS = $(SRCS)

###########################################################################
## OBJECTS
###########################################################################

OBJS = Robot_Control_data.cpp.o Robot_Control_initialize.cpp.o CPos_wrt_B.cpp.o rotz.cpp.o rotx.cpp.o CPos_wrt_I.cpp.o isequal.cpp.o IK_Solver_BodyRot_BodyPos.cpp.o combineVectorElements.cpp.o findTrue4Elem.cpp.o svd.cpp.o svd1.cpp.o xzsvdc.cpp.o xnrm2.cpp.o abs.cpp.o abs1.cpp.o power.cpp.o eml_int_forloop_overflow_check.cpp.o xdotc.cpp.o xdot.cpp.o det.cpp.o ixamax.cpp.o diag.cpp.o norm.cpp.o error.cpp.o Robot_Control.cpp.o eye.cpp.o roty.cpp.o inpolygon.cpp.o manipulability.cpp.o contactJacobians.cpp.o recursiveKin.cpp.o skew.cpp.o prod.cpp.o find_pgon_goal.cpp.o cross.cpp.o Leg_Controller_B.cpp.o atan2.cpp.o atan21.cpp.o angle.cpp.o angle1.cpp.o mldivide.cpp.o lusolve.cpp.o ifWhileCond.cpp.o step_planner_intelligent.cpp.o centroid_codeGen.cpp.o sum.cpp.o blockedSummation.cpp.o all.cpp.o allOrAny.cpp.o CallTheDead.cpp.o cos.cpp.o cos1.cpp.o sin.cpp.o sin1.cpp.o sumprod.cpp.o floor.cpp.o sqrt.cpp.o sqrt1.cpp.o xscal.cpp.o xaxpy.cpp.o xrotg.cpp.o xrot.cpp.o xswap.cpp.o xgetrf.cpp.o xzgetrf.cpp.o xgeru.cpp.o xger.cpp.o getUp.cpp.o sort.cpp.o sortIdx.cpp.o sign.cpp.o sign1.cpp.o exp.cpp.o exp1.cpp.o Body_Pose_Controller.cpp.o circshift.cpp.o Leg_Controller.cpp.o Robot_Control_rtwutil.cpp.o MW_nvidia_init.c.o

ALL_OBJS = $(OBJS)

###########################################################################
## PREBUILT OBJECT FILES
###########################################################################

PREBUILT_OBJS = 

###########################################################################
## LIBRARIES
###########################################################################

LIBS = 

###########################################################################
## SYSTEM LIBRARIES
###########################################################################

SYSTEM_LIBS = $(LDFLAGS_CUSTOMLIBFLAGS) -lm -lstdc++

###########################################################################
## ADDITIONAL TOOLCHAIN FLAGS
###########################################################################

#---------------
# C Compiler
#---------------

CFLAGS_BASIC = $(DEFINES) $(INCLUDES)

CFLAGS += $(CFLAGS_BASIC)

#-----------------
# C++ Compiler
#-----------------

CPPFLAGS_BASIC = $(DEFINES) $(INCLUDES)

CPPFLAGS += $(CPPFLAGS_BASIC)

###########################################################################
## INLINED COMMANDS
###########################################################################


DERIVED_SRCS = $(subst .o,.dep,$(OBJS))

build:

%.dep:



-include codertarget_assembly_flags.mk
-include *.dep


###########################################################################
## PHONY TARGETS
###########################################################################

.PHONY : all build clean info prebuild download execute


all : build
	echo "### Successfully generated all binary outputs."


build : prebuild $(PRODUCT)


prebuild : 


download : $(PRODUCT)


execute : download


###########################################################################
## FINAL TARGET
###########################################################################

#---------------------------------
# Create a static library         
#---------------------------------

$(PRODUCT) : $(OBJS) $(PREBUILT_OBJS)
	echo "### Creating static library "$(PRODUCT)" ..."
	$(AR) $(ARFLAGS)  $(PRODUCT) $(OBJS)
	echo "### Created: $(PRODUCT)"


###########################################################################
## INTERMEDIATE TARGETS
###########################################################################

#---------------------
# SOURCE-TO-OBJECT
#---------------------

%.c.o : %.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.s.o : %.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.cpp.o : %.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.c.o : $(RELATIVE_PATH_TO_ANCHOR)/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.s.o : $(RELATIVE_PATH_TO_ANCHOR)/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.cpp.o : $(RELATIVE_PATH_TO_ANCHOR)/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.c.o : $(START_DIR)/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.s.o : $(START_DIR)/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.cpp.o : $(START_DIR)/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.c.o : $(MATLAB_WORKSPACE)/D/Desktop/Capstone/kinematics-MATLAB/SpringProject/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.s.o : $(MATLAB_WORKSPACE)/D/Desktop/Capstone/kinematics-MATLAB/SpringProject/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.cpp.o : $(MATLAB_WORKSPACE)/D/Desktop/Capstone/kinematics-MATLAB/SpringProject/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


Robot_Control_data.cpp.o : $(START_DIR)/Robot_Control_data.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


Robot_Control_initialize.cpp.o : $(START_DIR)/Robot_Control_initialize.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


CPos_wrt_B.cpp.o : $(START_DIR)/CPos_wrt_B.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


rotz.cpp.o : $(START_DIR)/rotz.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


rotx.cpp.o : $(START_DIR)/rotx.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


CPos_wrt_I.cpp.o : $(START_DIR)/CPos_wrt_I.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


isequal.cpp.o : $(START_DIR)/isequal.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


IK_Solver_BodyRot_BodyPos.cpp.o : $(START_DIR)/IK_Solver_BodyRot_BodyPos.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


combineVectorElements.cpp.o : $(START_DIR)/combineVectorElements.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


findTrue4Elem.cpp.o : $(START_DIR)/findTrue4Elem.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


svd.cpp.o : $(START_DIR)/svd.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


svd1.cpp.o : $(START_DIR)/svd1.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


xzsvdc.cpp.o : $(START_DIR)/xzsvdc.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


xnrm2.cpp.o : $(START_DIR)/xnrm2.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


abs.cpp.o : $(START_DIR)/abs.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


abs1.cpp.o : $(START_DIR)/abs1.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


power.cpp.o : $(START_DIR)/power.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


eml_int_forloop_overflow_check.cpp.o : $(START_DIR)/eml_int_forloop_overflow_check.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


xdotc.cpp.o : $(START_DIR)/xdotc.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


xdot.cpp.o : $(START_DIR)/xdot.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


det.cpp.o : $(START_DIR)/det.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


ixamax.cpp.o : $(START_DIR)/ixamax.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


diag.cpp.o : $(START_DIR)/diag.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


norm.cpp.o : $(START_DIR)/norm.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


error.cpp.o : $(START_DIR)/error.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


Robot_Control.cpp.o : $(START_DIR)/Robot_Control.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


eye.cpp.o : $(START_DIR)/eye.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


roty.cpp.o : $(START_DIR)/roty.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


inpolygon.cpp.o : $(START_DIR)/inpolygon.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


manipulability.cpp.o : $(START_DIR)/manipulability.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


contactJacobians.cpp.o : $(START_DIR)/contactJacobians.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


recursiveKin.cpp.o : $(START_DIR)/recursiveKin.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


skew.cpp.o : $(START_DIR)/skew.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


prod.cpp.o : $(START_DIR)/prod.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


find_pgon_goal.cpp.o : $(START_DIR)/find_pgon_goal.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


cross.cpp.o : $(START_DIR)/cross.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


Leg_Controller_B.cpp.o : $(START_DIR)/Leg_Controller_B.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


atan2.cpp.o : $(START_DIR)/atan2.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


atan21.cpp.o : $(START_DIR)/atan21.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


angle.cpp.o : $(START_DIR)/angle.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


angle1.cpp.o : $(START_DIR)/angle1.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


mldivide.cpp.o : $(START_DIR)/mldivide.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


lusolve.cpp.o : $(START_DIR)/lusolve.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


ifWhileCond.cpp.o : $(START_DIR)/ifWhileCond.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


step_planner_intelligent.cpp.o : $(START_DIR)/step_planner_intelligent.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


centroid_codeGen.cpp.o : $(START_DIR)/centroid_codeGen.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


sum.cpp.o : $(START_DIR)/sum.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


blockedSummation.cpp.o : $(START_DIR)/blockedSummation.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


all.cpp.o : $(START_DIR)/all.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


allOrAny.cpp.o : $(START_DIR)/allOrAny.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


CallTheDead.cpp.o : $(START_DIR)/CallTheDead.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


cos.cpp.o : $(START_DIR)/cos.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


cos1.cpp.o : $(START_DIR)/cos1.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


sin.cpp.o : $(START_DIR)/sin.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


sin1.cpp.o : $(START_DIR)/sin1.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


sumprod.cpp.o : $(START_DIR)/sumprod.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


floor.cpp.o : $(START_DIR)/floor.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


sqrt.cpp.o : $(START_DIR)/sqrt.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


sqrt1.cpp.o : $(START_DIR)/sqrt1.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


xscal.cpp.o : $(START_DIR)/xscal.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


xaxpy.cpp.o : $(START_DIR)/xaxpy.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


xrotg.cpp.o : $(START_DIR)/xrotg.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


xrot.cpp.o : $(START_DIR)/xrot.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


xswap.cpp.o : $(START_DIR)/xswap.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


xgetrf.cpp.o : $(START_DIR)/xgetrf.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


xzgetrf.cpp.o : $(START_DIR)/xzgetrf.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


xgeru.cpp.o : $(START_DIR)/xgeru.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


xger.cpp.o : $(START_DIR)/xger.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


getUp.cpp.o : $(START_DIR)/getUp.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


sort.cpp.o : $(START_DIR)/sort.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


sortIdx.cpp.o : $(START_DIR)/sortIdx.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


sign.cpp.o : $(START_DIR)/sign.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


sign1.cpp.o : $(START_DIR)/sign1.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


exp.cpp.o : $(START_DIR)/exp.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


exp1.cpp.o : $(START_DIR)/exp1.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


Body_Pose_Controller.cpp.o : $(START_DIR)/Body_Pose_Controller.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


circshift.cpp.o : $(START_DIR)/circshift.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


Leg_Controller.cpp.o : $(START_DIR)/Leg_Controller.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


Robot_Control_rtwutil.cpp.o : $(START_DIR)/Robot_Control_rtwutil.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


MW_nvidia_init.c.o : $(MATLAB_WORKSPACE)/C/ProgramData/MATLAB/SupportPackages/R2021a/toolbox/target/supportpackages/nvidia/sources/utils/MW_nvidia_init.c
	$(CC) $(CFLAGS) -o "$@" "$<"


###########################################################################
## DEPENDENCIES
###########################################################################

$(ALL_OBJS) : rtw_proj.tmw $(MAKEFILE)


###########################################################################
## MISCELLANEOUS TARGETS
###########################################################################

info : 
	echo "### PRODUCT = $(PRODUCT)"
	echo "### PRODUCT_TYPE = $(PRODUCT_TYPE)"
	echo "### BUILD_TYPE = $(BUILD_TYPE)"
	echo "### INCLUDES = $(INCLUDES)"
	echo "### DEFINES = $(DEFINES)"
	echo "### ALL_SRCS = $(ALL_SRCS)"
	echo "### ALL_OBJS = $(ALL_OBJS)"
	echo "### LIBS = $(LIBS)"
	echo "### MODELREF_LIBS = $(MODELREF_LIBS)"
	echo "### SYSTEM_LIBS = $(SYSTEM_LIBS)"
	echo "### TOOLCHAIN_LIBS = $(TOOLCHAIN_LIBS)"
	echo "### ASFLAGS = $(ASFLAGS)"
	echo "### CFLAGS = $(CFLAGS)"
	echo "### LDFLAGS = $(LDFLAGS)"
	echo "### SHAREDLIB_LDFLAGS = $(SHAREDLIB_LDFLAGS)"
	echo "### CPPFLAGS = $(CPPFLAGS)"
	echo "### CPP_LDFLAGS = $(CPP_LDFLAGS)"
	echo "### CPP_SHAREDLIB_LDFLAGS = $(CPP_SHAREDLIB_LDFLAGS)"
	echo "### ARFLAGS = $(ARFLAGS)"
	echo "### MEX_CFLAGS = $(MEX_CFLAGS)"
	echo "### MEX_CPPFLAGS = $(MEX_CPPFLAGS)"
	echo "### MEX_LDFLAGS = $(MEX_LDFLAGS)"
	echo "### MEX_CPPLDFLAGS = $(MEX_CPPLDFLAGS)"
	echo "### DOWNLOAD_FLAGS = $(DOWNLOAD_FLAGS)"
	echo "### EXECUTE_FLAGS = $(EXECUTE_FLAGS)"
	echo "### MAKE_FLAGS = $(MAKE_FLAGS)"


clean : 
	$(ECHO) "### Deleting all derived files..."
	$(RM) $(PRODUCT)
	$(RM) $(ALL_OBJS)
	$(RM) *.c.dep
	$(RM) *.cpp.dep
	$(ECHO) "### Deleted all derived files."


