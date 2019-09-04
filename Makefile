all: program

softPi.o: softPi.c softPi.h softPiMacros.h
	gcc -Wall -c softPi.c

program: program.c softPi.o
	gcc -Wall -o program program.c softPi.o -pthread

clean:
	rm program softPi.o
