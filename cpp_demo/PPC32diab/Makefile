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
PRJ_ROOT_DIR := $(WS_ROOT_DIR)/cpp_demo



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
PROJECT_TARGETS = cpp_demo/$(MODE_DIR)/cpp_demo.out \
	 cpp_demo_partialImage/$(MODE_DIR)/cpp_demo_partialImage.o


#Rules

# cpp_demo
ifeq ($(DEBUG_MODE),1)
cpp_demo/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -g
cpp_demo/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -g
cpp_demo/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -g
cpp_demo/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
cpp_demo/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
cpp_demo/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -g
else
cpp_demo/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -XO -Xsize-opt
cpp_demo/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -XO -Xsize-opt
cpp_demo/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -XO -Xsize-opt
cpp_demo/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
cpp_demo/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
cpp_demo/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -XO -Xsize-opt
endif
cpp_demo/$(MODE_DIR)/% : IDE_INCLUDES = -I$(WIND_BASE)/target/h -I$(WIND_BASE)/target/h/wrn/coreip 
cpp_demo/$(MODE_DIR)/% : IDE_LIBRARIES = 
cpp_demo/$(MODE_DIR)/% : IDE_DEFINES = 
cpp_demo/$(MODE_DIR)/% : PROJECT_TYPE = DKM
cpp_demo/$(MODE_DIR)/% : DEFINES = 
cpp_demo/$(MODE_DIR)/% : EXPAND_DBG = 0
cpp_demo/$(MODE_DIR)/% : VX_CPU_FAMILY = ppc
cpp_demo/$(MODE_DIR)/% : CPU = PPC32
cpp_demo/$(MODE_DIR)/% : TOOL_FAMILY = diab
cpp_demo/$(MODE_DIR)/% : TOOL = diab
cpp_demo/$(MODE_DIR)/% : TOOL_PATH = 
cpp_demo/$(MODE_DIR)/% : CC_ARCH_SPEC = -tPPCFH:vxworks63 -Xstmw-slow -DPPC32_fp60x
cpp_demo/$(MODE_DIR)/% : LIBPATH = 
cpp_demo/$(MODE_DIR)/% : LIBS = 
cpp_demo/$(MODE_DIR)/% : OBJ_DIR := cpp_demo/$(MODE_DIR)

OBJECTS_cpp_demo = cpp_demo_partialImage/$(MODE_DIR)/cpp_demo_partialImage.o

cpp_demo/$(MODE_DIR)/cpp_demo.out : $(OBJECTS_cpp_demo)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@";rm -f "$@";ddump -Ng $(OBJECTS_cpp_demo) $(IDE_LIBRARIES) | tclsh $(WIND_BASE)/host/resource/hutils/tcl/munch.tcl -c ppc > $(OBJ_DIR)/ctdt.c; $(TOOL_PATH)dcc $(DEBUGFLAGS_Linker) $(CC_ARCH_SPEC) -Xdollar-in-ident -Xforce-declarations $(ADDED_CFLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES)  -o $(OBJ_DIR)/ctdt.o -c $(OBJ_DIR)/ctdt.c; $(TOOL_PATH)dld -tPPCFH:vxworks63 -X -r5 -r4 -o "$@" $(OBJ_DIR)/ctdt.o $(OBJECTS_cpp_demo) $(IDE_LIBRARIES) $(LIBPATH) $(LIBS)  $(ADDED_LIBPATH) $(ADDED_LIBS) && if [ "$(EXPAND_DBG)" = "1" ]; then plink "$@";fi

cpp_demo/$(MODE_DIR)/cpp_demo_compile_file : $(FILE) ;

_clean :: cpp_demo/$(MODE_DIR)/cpp_demo_clean

cpp_demo/$(MODE_DIR)/cpp_demo_clean : 
	$(TRACE_FLAG)if [ -d "cpp_demo" ]; then cd "cpp_demo"; rm -rf $(MODE_DIR); fi


# cpp_demo_partialImage
ifeq ($(DEBUG_MODE),1)
cpp_demo_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -g
cpp_demo_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -g
cpp_demo_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -g
cpp_demo_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
cpp_demo_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
cpp_demo_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -g
else
cpp_demo_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -XO -Xsize-opt
cpp_demo_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -XO -Xsize-opt
cpp_demo_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -XO -Xsize-opt
cpp_demo_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
cpp_demo_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
cpp_demo_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -XO -Xsize-opt
endif
cpp_demo_partialImage/$(MODE_DIR)/% : IDE_INCLUDES = -I$(WIND_BASE)/target/h -I$(WIND_BASE)/target/h/wrn/coreip 
cpp_demo_partialImage/$(MODE_DIR)/% : IDE_LIBRARIES = 
cpp_demo_partialImage/$(MODE_DIR)/% : IDE_DEFINES = 
cpp_demo_partialImage/$(MODE_DIR)/% : PROJECT_TYPE = DKM
cpp_demo_partialImage/$(MODE_DIR)/% : DEFINES = 
cpp_demo_partialImage/$(MODE_DIR)/% : EXPAND_DBG = 0
cpp_demo_partialImage/$(MODE_DIR)/% : VX_CPU_FAMILY = ppc
cpp_demo_partialImage/$(MODE_DIR)/% : CPU = PPC32
cpp_demo_partialImage/$(MODE_DIR)/% : TOOL_FAMILY = diab
cpp_demo_partialImage/$(MODE_DIR)/% : TOOL = diab
cpp_demo_partialImage/$(MODE_DIR)/% : TOOL_PATH = 
cpp_demo_partialImage/$(MODE_DIR)/% : CC_ARCH_SPEC = -tPPCFH:vxworks63 -Xstmw-slow -DPPC32_fp60x
cpp_demo_partialImage/$(MODE_DIR)/% : LIBPATH = 
cpp_demo_partialImage/$(MODE_DIR)/% : LIBS = 
cpp_demo_partialImage/$(MODE_DIR)/% : OBJ_DIR := cpp_demo_partialImage/$(MODE_DIR)

cpp_demo_partialImage/$(MODE_DIR)/Objects/cpp_demo/allocator.o : $(PRJ_ROOT_DIR)/allocator.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)dcc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -W:c:,-Xclib-optim-off -Xansi -Xlocal-data-area-static-only -Xforce-declarations  -Xmake-dependency=0xd $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


cpp_demo_partialImage/$(MODE_DIR)/Objects/cpp_demo/array.o : $(PRJ_ROOT_DIR)/array.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)dcc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -W:c:,-Xclib-optim-off -Xansi -Xlocal-data-area-static-only -Xforce-declarations  -Xmake-dependency=0xd $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


cpp_demo_partialImage/$(MODE_DIR)/Objects/cpp_demo/exception.o : $(PRJ_ROOT_DIR)/exception.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)dcc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -W:c:,-Xclib-optim-off -Xansi -Xlocal-data-area-static-only -Xforce-declarations  -Xmake-dependency=0xd $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


cpp_demo_partialImage/$(MODE_DIR)/Objects/cpp_demo/list.o : $(PRJ_ROOT_DIR)/list.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)dcc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -W:c:,-Xclib-optim-off -Xansi -Xlocal-data-area-static-only -Xforce-declarations  -Xmake-dependency=0xd $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


cpp_demo_partialImage/$(MODE_DIR)/Objects/cpp_demo/main.o : $(PRJ_ROOT_DIR)/main.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)dcc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -W:c:,-Xclib-optim-off -Xansi -Xlocal-data-area-static-only -Xforce-declarations  -Xmake-dependency=0xd $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


cpp_demo_partialImage/$(MODE_DIR)/Objects/cpp_demo/multipleInhert.o : $(PRJ_ROOT_DIR)/multipleInhert.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)dcc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -W:c:,-Xclib-optim-off -Xansi -Xlocal-data-area-static-only -Xforce-declarations  -Xmake-dependency=0xd $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


cpp_demo_partialImage/$(MODE_DIR)/Objects/cpp_demo/static.o : $(PRJ_ROOT_DIR)/static.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)dcc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -W:c:,-Xclib-optim-off -Xansi -Xlocal-data-area-static-only -Xforce-declarations  -Xmake-dependency=0xd $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


cpp_demo_partialImage/$(MODE_DIR)/Objects/cpp_demo/string.o : $(PRJ_ROOT_DIR)/string.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)dcc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -W:c:,-Xclib-optim-off -Xansi -Xlocal-data-area-static-only -Xforce-declarations  -Xmake-dependency=0xd $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


cpp_demo_partialImage/$(MODE_DIR)/Objects/cpp_demo/virtualBase.o : $(PRJ_ROOT_DIR)/virtualBase.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)dcc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -W:c:,-Xclib-optim-off -Xansi -Xlocal-data-area-static-only -Xforce-declarations  -Xmake-dependency=0xd $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


OBJECTS_cpp_demo_partialImage = cpp_demo_partialImage/$(MODE_DIR)/Objects/cpp_demo/allocator.o \
	 cpp_demo_partialImage/$(MODE_DIR)/Objects/cpp_demo/array.o \
	 cpp_demo_partialImage/$(MODE_DIR)/Objects/cpp_demo/exception.o \
	 cpp_demo_partialImage/$(MODE_DIR)/Objects/cpp_demo/list.o \
	 cpp_demo_partialImage/$(MODE_DIR)/Objects/cpp_demo/main.o \
	 cpp_demo_partialImage/$(MODE_DIR)/Objects/cpp_demo/multipleInhert.o \
	 cpp_demo_partialImage/$(MODE_DIR)/Objects/cpp_demo/static.o \
	 cpp_demo_partialImage/$(MODE_DIR)/Objects/cpp_demo/string.o \
	 cpp_demo_partialImage/$(MODE_DIR)/Objects/cpp_demo/virtualBase.o

cpp_demo_partialImage/$(MODE_DIR)/cpp_demo_partialImage.o : $(OBJECTS_cpp_demo_partialImage)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)dld -tPPCFH:vxworks63 -X -r5  -o "$@" $(OBJECTS_cpp_demo_partialImage) $(LIBPATH) $(LIBS) $(ADDED_LIBPATH) $(ADDED_LIBS)  && if [ "$(EXPAND_DBG)" = "1" ]; then plink "$@";fi

cpp_demo_partialImage/$(MODE_DIR)/cpp_demo_partialImage_compile_file : $(FILE) ;

_clean :: cpp_demo_partialImage/$(MODE_DIR)/cpp_demo_partialImage_clean

cpp_demo_partialImage/$(MODE_DIR)/cpp_demo_partialImage_clean : 
	$(TRACE_FLAG)if [ -d "cpp_demo_partialImage" ]; then cd "cpp_demo_partialImage"; rm -rf $(MODE_DIR); fi

force : 

DEP_FILES := cpp_demo_partialImage/$(MODE_DIR)/Objects/cpp_demo/allocator.d cpp_demo_partialImage/$(MODE_DIR)/Objects/cpp_demo/array.d cpp_demo_partialImage/$(MODE_DIR)/Objects/cpp_demo/exception.d \
	 cpp_demo_partialImage/$(MODE_DIR)/Objects/cpp_demo/list.d cpp_demo_partialImage/$(MODE_DIR)/Objects/cpp_demo/main.d cpp_demo_partialImage/$(MODE_DIR)/Objects/cpp_demo/multipleInhert.d \
	 cpp_demo_partialImage/$(MODE_DIR)/Objects/cpp_demo/static.d cpp_demo_partialImage/$(MODE_DIR)/Objects/cpp_demo/string.d cpp_demo_partialImage/$(MODE_DIR)/Objects/cpp_demo/virtualBase.d
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
