CXXFLAGS =	-O2 -g -fmessage-length=0 -std=c++11 -pthread

OBJS =		main.o RobotDetection.o RobotOverseer.o RobotObject.o vect2d.o Gui.o RobotCalc.o RobotControl.o

LIBS =		-lpthread -lbluetooth `pkg-config --libs opencv`

TARGET =	RobotSupervision.out

$(TARGET):	$(OBJS)
	$(CXX) -o $(TARGET) $(OBJS) $(LIBS)

all:	$(TARGET)

clean:
	rm -f $(OBJS) $(TARGET)
