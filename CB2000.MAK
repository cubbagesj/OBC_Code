# Microsoft Visual C++ generated build script - Do not modify

PROJ = CB2000
DEBUG = 0
PROGTYPE = 6
CALLER = 
ARGS = 
DLLS = 
D_RCDEFINES = -d_DEBUG
R_RCDEFINES = -dNDEBUG
ORIGIN = MSVC
ORIGIN_VER = 1.00
PROJPATH = C:\CB2000\
USEMFC = 0
CC = cl
CPP = cl
CXX = cl
CCREATEPCHFLAG = 
CPPCREATEPCHFLAG = 
CUSEPCHFLAG = 
CPPUSEPCHFLAG = 
FIRSTC =             
FIRSTCPP = CESCC.CPP   
RC = rc
CFLAGS_D_DEXE = /nologo /G3 /FPc /W3 /Zi /AH /Od /D "_DEBUG" /D "_DOS" /FR /Fd"CB2000.PDB"
CFLAGS_R_DEXE = /nologo /Gs /G3 /FPc87 /W3 /AH /Od /D "NDEBUG" /D "_DOS" /FR 
LFLAGS_D_DEXE = /NOLOGO /NOI /STACK:32000 /ONERROR:NOEXE /CO 
LFLAGS_R_DEXE = /NOLOGO /NOI /STACK:16000 /ONERROR:NOEXE 
LIBS_D_DEXE = oldnames llibce 
LIBS_R_DEXE = oldnames llibc7 
RCFLAGS = /nologo
RESFLAGS = /nologo
RUNFLAGS = 
OBJS_EXT = 
LIBS_EXT = TE5650H.LIB 
!if "$(DEBUG)" == "1"
CFLAGS = $(CFLAGS_D_DEXE)
LFLAGS = $(LFLAGS_D_DEXE)
LIBS = $(LIBS_D_DEXE)
MAPFILE = nul
RCDEFINES = $(D_RCDEFINES)
!else
CFLAGS = $(CFLAGS_R_DEXE)
LFLAGS = $(LFLAGS_R_DEXE)
LIBS = $(LIBS_R_DEXE)
MAPFILE = nul
RCDEFINES = $(R_RCDEFINES)
!endif
!if [if exist MSVC.BND del MSVC.BND]
!endif
SBRS = CESCC.SBR \
		UIO48.SBR \
		DMM32.SBR \
		CB2000.SBR


TE5650H_DEP = 

CESCC_DEP = c:\cb2000\cescc.h \
	c:\cb2000\uio48.h \
	c:\cb2000\te5650.h


UIO48_DEP = c:\cb2000\uio48.h


DMM32_DEP = c:\cb2000\dmm32.h


CB2000_DEP = c:\cb2000\cb2000.h \
	c:\cb2000\cescc.h \
	c:\cb2000\dmm32.h \
	c:\cb2000\ln200.h \
	c:\cb2000\uio48.h \
	c:\cb2000\te5650.h


all:	$(PROJ).EXE $(PROJ).BSC

CESCC.OBJ:	CESCC.CPP $(CESCC_DEP)
	$(CPP) $(CFLAGS) $(CPPCREATEPCHFLAG) /c CESCC.CPP

UIO48.OBJ:	UIO48.CPP $(UIO48_DEP)
	$(CPP) $(CFLAGS) $(CPPUSEPCHFLAG) /c UIO48.CPP

DMM32.OBJ:	DMM32.CPP $(DMM32_DEP)
	$(CPP) $(CFLAGS) $(CPPUSEPCHFLAG) /c DMM32.CPP

CB2000.OBJ:	CB2000.CPP $(CB2000_DEP)
	$(CPP) $(CFLAGS) $(CPPUSEPCHFLAG) /c CB2000.CPP

$(PROJ).EXE::	CESCC.OBJ UIO48.OBJ DMM32.OBJ CB2000.OBJ $(OBJS_EXT) $(DEFFILE)
	echo >NUL @<<$(PROJ).CRF
CESCC.OBJ +
UIO48.OBJ +
DMM32.OBJ +
CB2000.OBJ +
$(OBJS_EXT)
$(PROJ).EXE
$(MAPFILE)
c:\msvc\lib\+
c:\msvc\mfc\lib\+
TE5650H.LIB+
$(LIBS)
$(DEFFILE);
<<
	link $(LFLAGS) @$(PROJ).CRF

run: $(PROJ).EXE
	$(PROJ) $(RUNFLAGS)


$(PROJ).BSC: $(SBRS)
	bscmake @<<
/o$@ $(SBRS)
<<
