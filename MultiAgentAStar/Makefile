CC=gcc
CXX=g++
RM=rm -f
CXXFLAGS+=-std=c++11 -g


include Makefile.conf
OBJS=$(subst .cpp,.o,$(SOURCES))

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(OBJS) -o $(TARGET) $(CXXFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< 

clean:
	@$(RM) $(OBJS)

dist-clean: clean
	@$(RM) $(TARGET)
