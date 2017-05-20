
all: main.o face_class.o row_class.o helper.o
	g++ --std=c++0x main.o face_class.o row_class.o helper.o -o main

main.o: main.cpp helper.hpp face_class.hpp row_class.hpp
	g++ main.cpp -c

face_class.o: face_class.cpp face_class.hpp 
	g++ face_class.cpp -c

row_class.o: row_class.cpp row_class.hpp
	g++ row_class.cpp -c

helper.o: helper.cpp helper.hpp
	g++ helper.cpp -c

clean:
	rm main *.o
