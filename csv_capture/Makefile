.PHONY: clean

CC = gcc

CFLAGS = -Wall -g -fpic

csv: csv_capture.c
	$(CC) $(CFLAGS) -o csvcap  csv_capture.c -lncursesw -lc


clean:
	rm ./*.csv


