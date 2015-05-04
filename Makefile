#CFLAGS=-O2 -g -Wall -W `pkg-config --cflags librtlsdr`
RTLSDR=/c/tmp/nico/hack/sdr/RelWithDebInfo/rtl-sdr-release
CFLAGS=-O0 -g -Wall -Werror -I $(RTLSDR)
#LIBS=`pkg-config --libs librtlsdr` -lpthread -lm
LIBS=$(RTLSDR)/x32/rtlsdr.dll -lpthread -lm
CC=gcc
PROGNAME=dump433.exe

all: $(PROGNAME)

%.o: %.c
	$(CC) $(CFLAGS) -c $<

OBJS=dump433.o
$(PROGNAME): $(OBJS)
	$(CC) -g -o $@ $^ $(LIBS)

clean:
	rm -f *.o $(PROGNAME)
