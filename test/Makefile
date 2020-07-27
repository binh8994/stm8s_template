# We're assuming POSIX conformance
.POSIX:

#Compiler
CC = sdcc

PLATFORM = stm8
PNAME = main
ODIR = build

MAINSRC = $(PNAME).c

EXTRASRCS = \
	stm8s_it.c \
	../sources/STM8S_StdPeriph_Lib/Libraries/STM8S_StdPeriph_Driver/src/stm8s_gpio.c \
	../sources/STM8S_StdPeriph_Lib/Libraries/STM8S_StdPeriph_Driver/src/stm8s_clk.c \
	

RELS = $(EXTRASRCS:.c=.rel)

INCLUDES = \
	-I./ \
	-I../sources/STM8S_StdPeriph_Lib/Libraries/STM8S_StdPeriph_Driver/inc

CFLAGS   = \
	-DSTM8S003 \
	-m$(PLATFORM)

LIBS = -l$(PLATFORM)

all: clean $(PNAME)

$(PNAME): $(MAINSRC) $(RELS)
	@mkdir -p $(ODIR)
	$(CC) $(INCLUDES) $(CFLAGS) $(LIBS) $(MAINSRC) $(wildcard $(ODIR)/*.rel) -o$(ODIR)/

.c.rel:
	@mkdir -p $(ODIR)
	$(CC) -c $(INCLUDES) $(CFLAGS) $(LIBS) $< -o$(ODIR)/

.SUFFIXES: .c .rel

.PHONY:	clean flash

clean:
	@echo "Removing $(ODIR)..."
	@rm -rf $(ODIR)
	@echo "Done."

flash:
	@echo "Uploading resulted " $(PNAME)".ihx to target board..." 
	stm8flash -c stlinkv2 -p stm8s003f3 -w $(ODIR)/$(PNAME).ihx
	@echo "Flash completed."