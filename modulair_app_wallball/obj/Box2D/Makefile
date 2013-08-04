# BOX2D Compilation ------------------------------------------------------------------------------

BOX2D_SOURCE = ../../src/Box2D/Box2D.h
BOX2D_SOURCE += ../../src/Box2D/Collision/*.cpp
BOX2D_SOURCE += ../../src/Box2D/Collision/Shapes/*.cpp
BOX2D_SOURCE += ../../src/Box2D/Common/*.cpp
BOX2D_SOURCE += ../../src/Box2D/Dynamics/*.cpp
BOX2D_SOURCE += ../../src/Box2D/Dynamics/Contacts/*.cpp
BOX2D_SOURCE += ../../src/Box2D/Dynamics/Joints/*.cpp
BOX2D_SOURCE += ../../src/Box2D/Rope/*.cpp

BOX2D_LIB = ../../lib/Box2D.a

CC = g++

#------------------------------------------------------------------------------

../../lib/Box2D.a:
	$(CC) -I../../src $(BOX2D_SOURCE) -c
	ar rs $(BOX2D_LIB) *.o
	
clean:
	rm -f $(MYPROGRAM)
	rm -f *.o
	rm -f $(BOX2D_LIB)