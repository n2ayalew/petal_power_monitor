CC=gcc
CFLAGS= -I -std=c99
LDFLAGS=-lm -lpthread
DEPS = measure.h spiComm.h

%.o: %.c $(DEPS)
		$(CC) -c -o $@ $< $(CFLAGS)

measure: measure.o spiComm.o 
		$(CC) -o measure measure.o spiComm.o $(LDFLAGS)

.PHONY: clean

clean:
	rm *.o
