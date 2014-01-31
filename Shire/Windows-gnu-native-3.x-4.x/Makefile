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
PRJ_ROOT_DIR := $(WS_ROOT_DIR)/Shire



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
PROJECT_TARGETS = Shire/$(MODE_DIR)/Shire.exe


#Rules

# Shire
ifeq ($(DEBUG_MODE),1)
Shire/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -g
Shire/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -g
Shire/$(MODE_DIR)/% : DEBUGFLAGS_C-Linker = 
Shire/$(MODE_DIR)/% : DEBUGFLAGS_C++-Linker = 
Shire/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
Shire/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -g
else
Shire/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = 
Shire/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = 
Shire/$(MODE_DIR)/% : DEBUGFLAGS_C-Linker = 
Shire/$(MODE_DIR)/% : DEBUGFLAGS_C++-Linker = 
Shire/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
Shire/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = 
endif
Shire/$(MODE_DIR)/% : IDE_INCLUDES = 
Shire/$(MODE_DIR)/% : IDE_LIBRARIES = 
Shire/$(MODE_DIR)/% : IDE_DEFINES = 
Shire/$(MODE_DIR)/% : PROJECT_TYPE = APP
Shire/$(MODE_DIR)/% : DEFINES = 
Shire/$(MODE_DIR)/% : TOOL_PATH = 
Shire/$(MODE_DIR)/% : LIBPATH = 
Shire/$(MODE_DIR)/% : LIBS = 
Shire/$(MODE_DIR)/% : ADAPT_DEPS = 
Shire/$(MODE_DIR)/% : OBJ_DIR := Shire/$(MODE_DIR)

Shire/$(MODE_DIR)/Objects/Shire/A\ Hobbit.o : $(PRJ_ROOT_DIR)/A\ Hobbit.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)g++ $(DEBUGFLAGS_C++-Compiler) -ansi -fno-builtin -Wall $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) $(DEFINES) -MD -MP -c -o "$@" "$<"


OBJECTS_Shire = Shire/$(MODE_DIR)/Objects/Shire/A\ Hobbit.o

Shire/$(MODE_DIR)/Shire.exe : $(OBJECTS_Shire)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)g++ $(DEBUGFLAGS_C++-Linker)  -o "$@" $(OBJECTS_Shire) $(ADDED_OBJECTS) $(IDE_LIBRARIES) $(LIBPATH) $(LIBS) $(ADDED_LIBPATH) $(ADDED_LIBS)

Shire/$(MODE_DIR)/Shire_compile_file : $(FILE) ;

_clean :: Shire/$(MODE_DIR)/Shire_clean

Shire/$(MODE_DIR)/Shire_clean : 
	$(TRACE_FLAG)if [ -d "Shire" ]; then cd "Shire"; rm -rf $(MODE_DIR); fi

force : 

DEP_FILES := Shire/$(MODE_DIR)/Objects/Shire/A\ Hobbit.d
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
