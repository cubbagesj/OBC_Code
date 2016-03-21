# Microsoft Visual C++ generated build script - Do not modify

PROJ = CBA
DEBUG = 0
PROGTYPE = 6
CALLER = 
ARGS = 
DLLS = 
D_RCDEFINES = -d_DEBUG
R_RCDEFINES = -dNDEBUG
ORIGIN = MSVC
ORIGIN_VER = 1.00
PROJPATH = C:\CBA\
USEMFC = 0
CC = cl
CPP = cl
CXX = cl
CCREATEPCHFLAG = 
CPPCREATEPCHFLAG = 
CUSEPCHFLAG = 
CPPUSEPCHFLAG = 
FIRSTC = PC.C        
FIRSTCPP = CBA.CPP     
RC = rc
CFLAGS_D_DEXE = /nologo /G2 /W3 /Zi /AM /Od /D "_DEBUG" /D "_DOS" /FR /Fd"CB8.PDB"  /Fp"CB8.PCH"
CFLAGS_R_DEXE = /nologo /Gs /G2 /Zp1 /W3 /AM /Ox /D "NDEBUG" /D "_DOS" /FR 
LFLAGS_D_DEXE = /NOLOGO /NOI /STACK:5120 /ONERROR:NOEXE /CO 
LFLAGS_R_DEXE = /NOLOGO /NOI /STACK:8000 /ONERROR:NOEXE 
LIBS_D_DEXE = oldnames mlibce 
LIBS_R_DEXE = oldnames mlibce 
RCFLAGS = /nologo
RESFLAGS = /nologo
RUNFLAGS = 
OBJS_EXT = 
LIBS_EXT = ..\TECH80\C\TE5650H.LIB 
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
SBRS = CBA.SBR \
		ESCCAUTO.SBR \
		ETHER.SBR \
		PC.SBR \
		RS232CB8.SBR \
		UIO48.SBR


TE5650H_DEP = 

CBA_DEP = c:\cba\cba.h \
	c:\cba\esccauto.h \
	c:\cba\dmm32.h \
	c:\cba\rs232.h \
	c:\cba\uio48.h \
	c:\cba\ether.h \
	c:\cba\pc.c \
	c:\cba\pc.h \
	c:\cba\commands.h \
	c:\cba\te5650.h


ESCCAUTO_DEP = c:\cba\esccauto.h \
	c:\cba\dmmtest.h


ETHER_DEP = c:\cba\pktdrv.h \
	c:\cba\ether.h \
	c:\cba\dmmtest.h


PC_DEP = c:\cba\pc.h


RS232CB8_DEP = c:\cba\pc.h \
	c:\cba\rs232.h \
	c:\cba\cba.h \
	c:\cba\dmm32.h


UIO48_DEP = c:\cba\uio48.h


all:	$(PROJ).EXE $(PROJ).BSC

CBA.OBJ:	CBA.CPP $(CBA_DEP)
	$(CPP) $(CFLAGS) $(CPPCREATEPCHFLAG) /c CBA.CPP

ESCCAUTO.OBJ:	ESCCAUTO.CPP $(ESCCAUTO_DEP)
	$(CPP) $(CFLAGS) $(CPPUSEPCHFLAG) /c ESCCAUTO.CPP

ETHER.OBJ:	ETHER.CPP $(ETHER_DEP)
	$(CPP) $(CFLAGS) $(CPPUSEPCHFLAG) /c ETHER.CPP

PC.OBJ:	PC.C $(PC_DEP)
	$(CC) $(CFLAGS) $(CCREATEPCHFLAG) /c PC.C

RS232CB8.OBJ:	RS232CB8.CPP $(RS232CB8_DEP)
	$(CPP) $(CFLAGS) $(CPPUSEPCHFLAG) /c RS232CB8.CPP

UIO48.OBJ:	UIO48.CPP $(UIO48_DEP)
	$(CPP) $(CFLAGS) $(CPPUSEPCHFLAG) /c UIO48.CPP

$(PROJ).EXE::	CBA.OBJ ESCCAUTO.OBJ ETHER.OBJ PC.OBJ RS232CB8.OBJ UIO48.OBJ $(OBJS_EXT) $(DEFFILE)
	echo >NUL @<<$(PROJ).CRF
CBA.OBJ +
ESCCAUTO.OBJ +
ETHER.OBJ +
PC.OBJ +
RS232CB8.OBJ +
UIO48.OBJ +
$(OBJS_EXT)
$(PROJ).EXE
$(MAPFILE)
C:\MSVC\LIB\+
..\TECH80\C\TE5650H.LIB+
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
