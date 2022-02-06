CC = cc -g
CFLAGS = -Wall -ansi -pedantic

SRCS = yanova.c curterm.c
OBJS = $(SRCS:.c=.o)

all: yanova

yanova: $(OBJS)
	$(CC) -o $@ $(OBJS)
	wc $(OBJS:.o=.c)
	ls -l $@

clean:
	$(RM) yanova
	$(RM) $(OBJS)
	$(RM) make.log
