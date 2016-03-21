# Microsoft Visual C++ generated build script - Do not modify

PROJ = DMMTEST
DEBUG = 0
PROGTYPE = 6
CALLER = 
ARGS = 
DLLS = 
D_RCDEFINES = -d_DEBUG
R_RCDEFINES = -dNDEBUG
ORIGIN = MSVC
ORIGIN_VER = 1.00
PROJPATH = C:\DMMTEST\
USEMFC = 0
CC = cl
CPP = cl
CXX = cl
CCREATEPCHFLAG = 
CPPCREATEPCHFLAG = 
CUSEPCHFLAG = 
CPPUSEPCHFLAG = 
FIRSTC = PC.C        
FIRSTCPP = DMMTEST.CPP 
RC = rc
CFLAGS_D_DEXE = /nologo /G2 /W3 /Zi /AL /Od /D "_DEBUG" /D "_DOS" /Fc /FR /Fd"DMMTEST.PDB"
CFLAGS_R_DEXE = /nologo /Gs /G2 /FPc /Zp1 /W3 /AL /Od /D "NDEBUG" /D "_DOS" /Fc /FR 
LFLAGS_D_DEXE = /NOLOGO /NOI /STACK:5120 /ONERROR:NOEXE /CO 
LFLAGS_R_DEXE = /NOLOGO /NOI /STACK:8000 /ONERROR:NOEXE 
LIBS_D_DEXE = oldnames llibce 
LIBS_R_DEXE = oldnames llibce 
RCFLAGS = /nologo
RESFLAGS = /nologo
RUNFLAGS = 
OBJS_EXT = 
LIBS_EXT = ..\TECH80\C\TE5650L.LIB 
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
SBRS = DMMTEST.SBR \
		ESCCAUTO.SBR \
		PC.SBR \
		ETHER.SBR \
		UIO48.SBR \
		RS232.SBR


TE5650L_DEP = 

DMMTEST_DEP = c:\dmmtest\dmmtest.h \
	c:\dmmtest\esccauto.h \
	c:\dmmtest\dmm32.h \
	c:\dmmtest\rs232.h \
	c:\dmmtest\uio48.h \
	c:\dmmtest\ether.h \
	c:\dmmtest\pc.c \
	c:\dmmtest\pc.h \
	c:\dmmtest\commands.h \
	c:\dmmtest\te5650.h


ESCCAUTO_DEP = c:\dmmtest\esccauto.h \
	c:\dmmtest\dmmtest.h


PC_DEP = c:\dmmtest\pc.h


ETHER_DEP = c:\dmmtest\pktdrv.h \
	c:\dmmtest\ether.h \
	c:\dmmtest\dmmtest.h


UIO48_DEP = c:\dmmtest\uio48.h


RS232_DEP = c:\dmmtest\pc.h \
	c:\dmmtest\rs232.h \
	c:\dmmtest\dmmtest.h \
	c:\dmmtest\dmm32.h


all:	$(PROJ).EXE $(PROJ).BSC

DMMTEST.OBJ:	DMMTEST.CPP $(DMMTEST_DEP)
	$(CPP) $(CFLAGS) $(CPPCREATEPCHFLAG) /c DMMTEST.CPP

ESCCAUTO.OBJ:	ESCCAUTO.CPP $(ESCCAUTO_DEP)
	$(CPP) $(CFLAGS) $(CPPUSEPCHFLAG) /c ESCCAUTO.CPP

PC.OBJ:	PC.C $(PC_DEP)
	$(CC) $(CFLAGS) $(CCREATEPCHFLAG) /c PC.C

ETHER.OBJ:	ETHER.CPP $(ETHER_DEP)
	$(CPP) $(CFLAGS) $(CPPUSEPCHFLAG) /c ETHER.CPP

UIO48.OBJ:	UIO48.CPP $(UIO48_DEP)
	$(CPP) $(CFLAGS) $(CPPUSEPCHFLAG) /c UIO48.CPP

RS232.OBJ:	RS232.CPP $(RS232_DEP)
	$(CPP) $(CFLAGS) $(CPPUSEPCHFLAG) /c RS232.CPP

$(PROJ).EXE::	DMMTEST.OBJ ESCCAUTO.OBJ PC.OBJ ETHER.OBJ UIO48.OBJ RS232.OBJ $(OBJS_EXT) $(DEFFILE)
	echo >NUL @<<$(PROJ).CRF
DMMTEST.OBJ +
ESCCAUTO.OBJ +
PC.OBJ +
ETHER.OBJ +
UIO48.OBJ +
RS232.OBJ +
$(OBJS_EXT)
$(PROJ).EXE
$(MAPFILE)
C:\MSVC\LIB\+
C:\MSVC\MFC\LIB\+
..\TECH80\C\TE5650L.LIB+
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
