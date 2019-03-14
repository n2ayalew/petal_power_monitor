CC=gcc
CFLAGS= -I -std=c99 -Wall -g
LDFLAGS=-lm -lpthread -lcurl
DEPS = measure.h spiComm.h

%.o: %.c $(DEPS)
		$(CC) -c -o $@ $< $(CFLAGS)

all: measure runpy

measure: measure.o spiComm.o 
		$(CC) -o measure measure.o spiComm.o $(LDFLAGS)

runpy: runpy.o
	$(CC) -o runpy runpy.o

.PHONY: clean all

clean:
	rm *.o
