
all: test.o cube_class.o face_class.o helper.o
	g++ --std=c++0x test.o cube_class.o face_class.o helper.o -o main

test.o: test.cpp cube_class.hpp face_class.hpp helper.hpp
	g++ test.cpp -c

main.o: main.cpp cube_class.hpp face_class.hpp helper.hpp
	g++ main.cpp -c

cube_class.o: cube_class.cpp helper.hpp face_class.hpp 
	g++ cube_class.cpp -c

face_class.o: face_class.cpp face_class.hpp 
	g++ face_class.cpp -c

helper.o: helper.cpp helper.hpp
	g++ helper.cpp -c

clean:
	rm main *.o
