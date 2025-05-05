.PHONY: clean

CC = gcc

CFLAGS = -Wall -g -fpic

csv: csv_capture.c
	$(CC) $(CFLAGS) -o csvcap csv_capture.c -lcurses


clean:
	rm ./*.csv


