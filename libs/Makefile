# libs Makefile

include ../Makefile.common
LIBS+=libfreertos.a
#LIBS+=libfrcm3.a
CFLAGSlib+=-c


all: libs

libs: $(LIBS)

libfreertos.a:
		@echo -n "Building $@ ..."
		@cd $(FREERTOSDIR)/Source/portable/MemMang && $(CC) $(CFLAGSlib) *.c		
		@cd $(FREERTOSDIR)/Source/portable/GCC/ARM_CM3 && $(CC) $(CFLAGSlib) *.c		
		@cd $(FREERTOSDIR)/Source && $(CC) $(CFLAGSlib) *.c		
	$(AR) cr $(LIBDIR)/$@ \
		$(FREERTOSDIR)/Source/portable/MemMang/*.o \
		$(FREERTOSDIR)/Source/portable/GCC/ARM_CM3/*.o \
		$(FREERTOSDIR)/Source/*.o
		@echo "$@ done."

libfrcm3.a:
		@echo -n "Building $@ ..."
		@echo -n "Extracting obj files into the temp dir"
		@cd $(LIBDIR)/tmpobj && $(AR) x $(OPENCM3LIB)
		@echo -n "Compiling Free RTOS"
		@cd $(FREERTOSDIR)/Source/portable/MemMang && $(CC) $(CFLAGSlib) *.c		
		@cd $(FREERTOSDIR)/Source/portable/GCC/ARM_CM3 && $(CC) $(CFLAGSlib) *.c		
		@cd $(FREERTOSDIR)/Source && $(CC) $(CFLAGSlib) *.c		
	$(AR) cr $(LIBDIR)/$@ \
		$(LIBDIR)/tmpobj/*.o \
		$(FREERTOSDIR)/Source/portable/MemMang/*.o \
		$(FREERTOSDIR)/Source/portable/GCC/ARM_CM3/*.o \
		$(FREERTOSDIR)/Source/*.o
		@echo "$@ done."

.PHONY: libs clean tshow

clean:
	rm -f $(FREERTOSDIR)/Source/portable/MemMang/*.o
	rm -f $(FREERTOSDIR)/Source/portable/GCC/ARM_CM3/*.o
	rm -f $(FREERTOSDIR)/Source/*.o
	rm -f $(LIBS)

tshow:
	@echo "######################################################################################################"
	@echo "######## optimize settings: $(InfoTextLib), $(InfoTextSrc)"
	@echo "######################################################################################################"
	