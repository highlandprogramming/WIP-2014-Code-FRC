# Wind River Workbench generated Makefile.
# Do not edit!!!
#
# The file ".wrmakefile" is the template used by the Wind River Workbench to
# generate the makefiles of this project. Add user-specific build targets and
# make rules only(!) in this project's ".wrmakefile" file. These will then be
# automatically dumped into the makefiles.

WIND_HOME := $(subst \,/,$(WIND_HOME))

all : pre_build main_all post_build

_clean ::
	@echo "make: removing targets and objects of `pwd`"

TRACE=0
TRACEON=$(TRACE:0=@)
TRACE_FLAG=$(TRACEON:1=)

MAKEFILE := Makefile

FLEXIBLE_BUILD := 1

BUILD_SPEC = Windows-gnu-native-3.x-4.x
DEBUG_MODE = 1
ifeq ($(DEBUG_MODE),1)
MODE_DIR := Debug
else
MODE_DIR := NonDebug
endif
OBJ_DIR := .
WS_ROOT_DIR := C:/WindRiver/workspace
PRJ_ROOT_DIR := $(WS_ROOT_DIR)/hello_world_Native



#Global Build Macros
PROJECT_TYPE = APP
DEFINES = 


#BuildSpec specific Build Macros
TOOL_PATH = 
LIBPATH = 
LIBS = 
ADAPT_DEPS = 

IDE_INCLUDES = 

IDE_LIBRARIES = 

IDE_DEFINES = 



#BuildTool flags
ifeq ($(DEBUG_MODE),1)
DEBUGFLAGS_C-Compiler = -g
DEBUGFLAGS_C++-Compiler = -g
DEBUGFLAGS_C-Linker = 
DEBUGFLAGS_C++-Linker = 
DEBUGFLAGS_Librarian = 
DEBUGFLAGS_Assembler = -g
else
DEBUGFLAGS_C-Compiler = 
DEBUGFLAGS_C++-Compiler = 
DEBUGFLAGS_C-Linker = 
DEBUGFLAGS_C++-Linker = 
DEBUGFLAGS_Librarian = 
DEBUGFLAGS_Assembler = 
endif


#Project Targets
PROJECT_TARGETS = hello_world_Native/$(MODE_DIR)/hello_world_Native.exe


#Rules

# hello_world_Native
ifeq ($(DEBUG_MODE),1)
hello_world_Native/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -g
hello_world_Native/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -g
hello_world_Native/$(MODE_DIR)/% : DEBUGFLAGS_C-Linker = 
hello_world_Native/$(MODE_DIR)/% : DEBUGFLAGS_C++-Linker = 
hello_world_Native/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
hello_world_Native/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -g
else
hello_world_Native/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = 
hello_world_Native/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = 
hello_world_Native/$(MODE_DIR)/% : DEBUGFLAGS_C-Linker = 
hello_world_Native/$(MODE_DIR)/% : DEBUGFLAGS_C++-Linker = 
hello_world_Native/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
hello_world_Native/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = 
endif
hello_world_Native/$(MODE_DIR)/% : IDE_INCLUDES = 
hello_world_Native/$(MODE_DIR)/% : IDE_LIBRARIES = 
hello_world_Native/$(MODE_DIR)/% : IDE_DEFINES = 
hello_world_Native/$(MODE_DIR)/% : PROJECT_TYPE = APP
hello_world_Native/$(MODE_DIR)/% : DEFINES = 
hello_world_Native/$(MODE_DIR)/% : TOOL_PATH = 
hello_world_Native/$(MODE_DIR)/% : LIBPATH = 
hello_world_Native/$(MODE_DIR)/% : LIBS = 
hello_world_Native/$(MODE_DIR)/% : ADAPT_DEPS = 
hello_world_Native/$(MODE_DIR)/% : OBJ_DIR := hello_world_Native/$(MODE_DIR)

hello_world_Native/$(MODE_DIR)/Objects/hello_world_Native/helloworld.o : $(PRJ_ROOT_DIR)/helloworld.c $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)gcc $(DEBUGFLAGS_C-Compiler) -ansi -fno-builtin -Wall $(ADDED_CFLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) $(DEFINES) -MD -MP -c -o "$@" "$<"


OBJECTS_hello_world_Native = hello_world_Native/$(MODE_DIR)/Objects/hello_world_Native/helloworld.o

hello_world_Native/$(MODE_DIR)/hello_world_Native.exe : $(OBJECTS_hello_world_Native)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)g++ $(DEBUGFLAGS_C++-Linker)  -o "$@" $(OBJECTS_hello_world_Native) $(ADDED_OBJECTS) $(IDE_LIBRARIES) $(LIBPATH) $(LIBS) $(ADDED_LIBPATH) $(ADDED_LIBS)

hello_world_Native/$(MODE_DIR)/hello_world_Native_compile_file : $(FILE) ;

_clean :: hello_world_Native/$(MODE_DIR)/hello_world_Native_clean

hello_world_Native/$(MODE_DIR)/hello_world_Native_clean : 
	$(TRACE_FLAG)if [ -d "hello_world_Native" ]; then cd "hello_world_Native"; rm -rf $(MODE_DIR); fi

force : 

DEP_FILES := hello_world_Native/$(MODE_DIR)/Objects/hello_world_Native/helloworld.d
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
