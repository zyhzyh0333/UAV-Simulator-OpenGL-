CC = clang++
TARGET = test.app/Contents/MacOS/test
TARGET_PATH = test.app/Contents/MacOS
IDIR = include
ODIR = obj
$(shell mkdir obj)
SRCDIR = src
LDIR = lib
OSDIR = lib/osx
DATA = lib/resources


# Modify lib path by OS
ifeq ($(OS),Windows_NT)
 OSDIR = lib/windows
 TARGET = a.out
else
 $(shell mkdir -p $(TARGET_PATH))
endif

CFLAGS = -std=c++1z -O2 -g -Wno-unused-parameter -Wno-unused-variable -Wno-unused-command-line-argument -Wno-deprecated -framework Cocoa -framework OpenGL -I$(IDIR) -I$(LDIR) -I$(OSDIR) -I$(DATA)
_DEPS = Map.h UAV.h Camera.h Draw.h Control.h Point.h
_OBJ = Map.o UAV.o Camera.o Draw.o Control.o Point.o
_OSLIBS = fssimplewindowobjc.o fssimplewindowcpp.o yssimplesound.o yssimplesound_macosx_cpp.o yssimplesound_macosx_objc.o
_LIBS = GraphicFont.o DrawingUtilNG.o yspng.o
DEPS = $(patsubst %,$(IDIR)/%,$(_DEPS))
OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))
OSLIB = $(patsubst %,$(OSDIR)/%,$(_OSLIBS))
LIB = $(patsubst %,$(LDIR)/%,$(_LIBS))

$(TARGET): main.cpp  $(OSLIB) $(LIB) $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS) -Wall -Wextra

$(LDIR)/%.o: $(LDIR)/%.cpp
	$(CC) -c -o $@ $< $(CFLAGS)

$(LDIR)/%.o: $(LDIR)/%.c
	$(CC) -c -o $@ $< $(CLFAGS) -Wall -Wextra

$(ODIR)/%.o: $(SRCDIR)/%.cpp
	$(CC) -c -o $@ $< $(CFLAGS) -Wall -Wextra

$(OSDIR)/%.o: $(OSDIR)/%.cpp
	$(CC) -c -o $@ $< $(CFLAGS)

$(OSDIR)/%.o: $(OSDIR)/%.m
	clang -c -o $@ $< -Wno-unused-parameter -Wno-unused-command-line-argument -Wno-deprecated -Wno-unused-variable

.PHONY: run clean
run: 
	$(TARGET)

clean:
	$(RM) -r *.o $(ODIR) *~ test.app $(LDIR)/*.o $(OSDIR)/*.o

