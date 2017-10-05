# Systeme d'exploitation :
###########################

OS = $(shell uname -s)

# Compilateur a utiliser :
##########################

CC     = gcc

# Repertoire include :
#########################

INCLUDE   = include

# Options de compilation : (-g = pour deverminer)
#################################################

CFLAGS = -g -Wall -I${INCLUDE} -O2

# Librairies a utiliser :
#########################

ifeq ($(OS),Linux)	# Pour Linux

LIBS = -lm -I/usr/include/libusb-1.0  -lusb-1.0 

else	# Pour Mac OSX

LIBS = -lm

endif

# Liste de fichiers source (.c) a compiler :
############################################

CFILES = source/capture_sound.c
# Liste de modules objets (generee a partir de CFILES) :
########################################################

OFILES = $(CFILES:.c=.o)

# Nom du fichier executable :
#############################

CIBLE  = capture_sound

# edition de liens (rassembler tous les objets -> executable)
#############################################################

$(CIBLE): $(OFILES)
	$(CC) $(OFILES) ${LIBS} ${CFLAGS} -o $(CIBLE)


# Definitions de cibles particulieres :
#
# "make depend" : genere la liste des dependances
# "make clean"  : efface les fichiers .o et .x
#################################################

depend:
	@echo " *** MISE A JOUR DES DEPENDANCES ***"
	@(sed '/^# DO NOT DELETE THIS LINE/q' Makefile && \
	  $(CC) -MM $(CFLAGS) $(CFILES) | \
	  egrep -v "/usr/include" \
	 ) >Makefile.new
	@mv Makefile.new Makefile

clean:
	@echo " *** EFFACE MODULES OBJET ET EXECUTABLE ***"
	@/bin/rm -f  capture_sound source/*.o

#
# -- Regles de dependances generees par "make depend"
#####################################################
# DO NOT DELETE THIS LINE
capture_sound.o: source/capture_sound.c
