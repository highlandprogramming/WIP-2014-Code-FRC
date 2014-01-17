# Wind River Workbench generated Makefile.
# Do not edit!!!
#
# The file ".wrmakefile" is the template used by the Wind River Workbench to
# generate the makefiles of this project. Add user-specific build targets and
# make rules only(!) in this project's ".wrmakefile" file. These will then be
# automatically dumped into the makefiles.

WIND_HOME := $(subst \,/,$(WIND_HOME))
WIND_BASE := $(subst \,/,$(WIND_BASE))
WIND_USR := $(subst \,/,$(WIND_USR))
WRVX_COMPBASE := $(subst \,/,$(WRVX_COMPBASE))

all : pre_build main_all post_build

_clean ::
	@echo "make: removing targets and objects of `pwd`"

TRACE=0
TRACEON=$(TRACE:0=@)
TRACE_FLAG=$(TRACEON:1=)

MAKEFILE := Makefile

FLEXIBLE_BUILD := 1

BUILD_SPEC = PPC32diab
DEBUG_MODE = 1
ifeq ($(DEBUG_MODE),1)
MODE_DIR := Debug
else
MODE_DIR := NonDebug
endif
OBJ_DIR := .
WS_ROOT_DIR := C:/WindRiver/workspace
PRJ_ROOT_DIR := $(WS_ROOT_DIR)/Rivendell



#Global Build Macros
PROJECT_TYPE = DKM
DEFINES = 
EXPAND_DBG = 0


#BuildSpec specific Build Macros
VX_CPU_FAMILY = ppc
CPU = PPC32
TOOL_FAMILY = diab
TOOL = diab
TOOL_PATH = 
CC_ARCH_SPEC = -tPPCFH:vxworks63 -Xstmw-slow -DPPC32_fp60x
LIBPATH = 
LIBS = 

IDE_INCLUDES = -I$(WIND_BASE)/target/h -I$(WIND_BASE)/target/h/wrn/coreip 

IDE_LIBRARIES = 

IDE_DEFINES = 



#BuildTool flags
ifeq ($(DEBUG_MODE),1)
DEBUGFLAGS_C-Compiler = -g
DEBUGFLAGS_C++-Compiler = -g
DEBUGFLAGS_Linker = -g
DEBUGFLAGS_Partial-Image-Linker = 
DEBUGFLAGS_Librarian = 
DEBUGFLAGS_Assembler = -g
else
DEBUGFLAGS_C-Compiler = -XO -Xsize-opt
DEBUGFLAGS_C++-Compiler = -XO -Xsize-opt
DEBUGFLAGS_Linker = -XO -Xsize-opt
DEBUGFLAGS_Partial-Image-Linker = 
DEBUGFLAGS_Librarian = 
DEBUGFLAGS_Assembler = -XO -Xsize-opt
endif


#Project Targets
PROJECT_TARGETS = Rivendell/$(MODE_DIR)/Rivendell.out \
	 Rivendell_partialImage/$(MODE_DIR)/Rivendell_partialImage.o


#Rules

# Rivendell
ifeq ($(DEBUG_MODE),1)
Rivendell/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -g
Rivendell/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -g
Rivendell/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -g
Rivendell/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
Rivendell/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
Rivendell/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -g
else
Rivendell/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -XO -Xsize-opt
Rivendell/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -XO -Xsize-opt
Rivendell/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -XO -Xsize-opt
Rivendell/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
Rivendell/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
Rivendell/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -XO -Xsize-opt
endif
Rivendell/$(MODE_DIR)/% : IDE_INCLUDES = -I$(WIND_BASE)/target/h -I$(WIND_BASE)/target/h/wrn/coreip 
Rivendell/$(MODE_DIR)/% : IDE_LIBRARIES = 
Rivendell/$(MODE_DIR)/% : IDE_DEFINES = 
Rivendell/$(MODE_DIR)/% : PROJECT_TYPE = DKM
Rivendell/$(MODE_DIR)/% : DEFINES = 
Rivendell/$(MODE_DIR)/% : EXPAND_DBG = 0
Rivendell/$(MODE_DIR)/% : VX_CPU_FAMILY = ppc
Rivendell/$(MODE_DIR)/% : CPU = PPC32
Rivendell/$(MODE_DIR)/% : TOOL_FAMILY = diab
Rivendell/$(MODE_DIR)/% : TOOL = diab
Rivendell/$(MODE_DIR)/% : TOOL_PATH = 
Rivendell/$(MODE_DIR)/% : CC_ARCH_SPEC = -tPPCFH:vxworks63 -Xstmw-slow -DPPC32_fp60x
Rivendell/$(MODE_DIR)/% : LIBPATH = 
Rivendell/$(MODE_DIR)/% : LIBS = 
Rivendell/$(MODE_DIR)/% : OBJ_DIR := Rivendell/$(MODE_DIR)

OBJECTS_Rivendell = Rivendell_partialImage/$(MODE_DIR)/Rivendell_partialImage.o

Rivendell/$(MODE_DIR)/Rivendell.out : $(OBJECTS_Rivendell)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@";rm -f "$@";ddump -Ng $(OBJECTS_Rivendell) $(IDE_LIBRARIES) | tclsh $(WIND_BASE)/host/resource/hutils/tcl/munch.tcl -c ppc > $(OBJ_DIR)/ctdt.c; $(TOOL_PATH)dcc $(DEBUGFLAGS_Linker) $(CC_ARCH_SPEC) -Xdollar-in-ident -Xforce-declarations $(ADDED_CFLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES)  -o $(OBJ_DIR)/ctdt.o -c $(OBJ_DIR)/ctdt.c; $(TOOL_PATH)dld -tPPCFH:vxworks63 -X -r5 -r4 -o "$@" $(OBJ_DIR)/ctdt.o $(OBJECTS_Rivendell) $(IDE_LIBRARIES) $(LIBPATH) $(LIBS)  $(ADDED_LIBPATH) $(ADDED_LIBS) && if [ "$(EXPAND_DBG)" = "1" ]; then plink "$@";fi

Rivendell/$(MODE_DIR)/Rivendell_compile_file : $(FILE) ;

_clean :: Rivendell/$(MODE_DIR)/Rivendell_clean

Rivendell/$(MODE_DIR)/Rivendell_clean : 
	$(TRACE_FLAG)if [ -d "Rivendell" ]; then cd "Rivendell"; rm -rf $(MODE_DIR); fi


# Rivendell_partialImage
ifeq ($(DEBUG_MODE),1)
Rivendell_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -g
Rivendell_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -g
Rivendell_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -g
Rivendell_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
Rivendell_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
Rivendell_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -g
else
Rivendell_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -XO -Xsize-opt
Rivendell_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -XO -Xsize-opt
Rivendell_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -XO -Xsize-opt
Rivendell_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
Rivendell_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
Rivendell_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -XO -Xsize-opt
endif
Rivendell_partialImage/$(MODE_DIR)/% : IDE_INCLUDES = -I$(WIND_BASE)/target/h -I$(WIND_BASE)/target/h/wrn/coreip 
Rivendell_partialImage/$(MODE_DIR)/% : IDE_LIBRARIES = 
Rivendell_partialImage/$(MODE_DIR)/% : IDE_DEFINES = 
Rivendell_partialImage/$(MODE_DIR)/% : PROJECT_TYPE = DKM
Rivendell_partialImage/$(MODE_DIR)/% : DEFINES = 
Rivendell_partialImage/$(MODE_DIR)/% : EXPAND_DBG = 0
Rivendell_partialImage/$(MODE_DIR)/% : VX_CPU_FAMILY = ppc
Rivendell_partialImage/$(MODE_DIR)/% : CPU = PPC32
Rivendell_partialImage/$(MODE_DIR)/% : TOOL_FAMILY = diab
Rivendell_partialImage/$(MODE_DIR)/% : TOOL = diab
Rivendell_partialImage/$(MODE_DIR)/% : TOOL_PATH = 
Rivendell_partialImage/$(MODE_DIR)/% : CC_ARCH_SPEC = -tPPCFH:vxworks63 -Xstmw-slow -DPPC32_fp60x
Rivendell_partialImage/$(MODE_DIR)/% : LIBPATH = 
Rivendell_partialImage/$(MODE_DIR)/% : LIBS = 
Rivendell_partialImage/$(MODE_DIR)/% : OBJ_DIR := Rivendell_partialImage/$(MODE_DIR)

Rivendell_partialImage/$(MODE_DIR)/Objects/Rivendell/Imladris.o : $(PRJ_ROOT_DIR)/Imladris.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)dcc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -W:c:,-Xclib-optim-off -Xansi -Xlocal-data-area-static-only -Xforce-declarations  -Xmake-dependency=0xd $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


OBJECTS_Rivendell_partialImage = Rivendell_partialImage/$(MODE_DIR)/Objects/Rivendell/Imladris.o

Rivendell_partialImage/$(MODE_DIR)/Rivendell_partialImage.o : $(OBJECTS_Rivendell_partialImage)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)dld -tPPCFH:vxworks63 -X -r5  -o "$@" $(OBJECTS_Rivendell_partialImage) $(LIBPATH) $(LIBS) $(ADDED_LIBPATH) $(ADDED_LIBS)  && if [ "$(EXPAND_DBG)" = "1" ]; then plink "$@";fi

Rivendell_partialImage/$(MODE_DIR)/Rivendell_partialImage_compile_file : $(FILE) ;

_clean :: Rivendell_partialImage/$(MODE_DIR)/Rivendell_partialImage_clean

Rivendell_partialImage/$(MODE_DIR)/Rivendell_partialImage_clean : 
	$(TRACE_FLAG)if [ -d "Rivendell_partialImage" ]; then cd "Rivendell_partialImage"; rm -rf $(MODE_DIR); fi

force : 

DEP_FILES := Rivendell_partialImage/$(MODE_DIR)/Objects/Rivendell/Imladris.d
-include $(DEP_FILES)



-include $(PRJ_ROOT_DIR)/*.makefile

-include *.makefile

JOBS?=1
ifeq ($(JOBS),1)
main_all : external_build  $(PROJECT_TARGETS)
	@echo "make: built targets of `pwd`"
else
main_all : external_build 
	@$(MAKE) -f $(MAKEFILE) --jobs $(JOBS) $(MFLAGS) $(PROJECT_TARGETS);\
	echo "make: built targets of `pwd`"
endif 

# entry point for extending the build
external_build ::
	@echo ""

# main entry point for pre processing prior to the build
pre_build :: $(PRE_BUILD_STEP) generate_sources
	@echo ""

# entry point for generating sources prior to the build
generate_sources ::
	@echo ""

# main entry point for post processing after the build
post_build :: $(POST_BUILD_STEP) deploy_output
	@echo ""

# entry point for deploying output after the build
deploy_output ::
	@echo ""

clean :: external_clean $(CLEAN_STEP) _clean

# entry point for extending the build clean
external_clean ::
	@echo ""
