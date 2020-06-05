# software requirements
# sudo apt-get install binutils gcc-avr avr-libc uisp avrdude

# ---------------------- file config ---------------------------------
# file name to compile
MAIN_FILE = avr-basic

# source path
SRC_PATH = src

# build path for compiled fils
BUILD_PATH = build


# ----------------------- avr config ---------------------------------
# avr mcu type
MCU = attiny45

# define mcu clockspeed
F_CPU = 1000000UL


# -------------------- programmer config -----------------------------
# programmer usb port
PORT = /dev/ttyACM0

# programmer type
#PGMR = avrispmkII
PGMR = stk500

# baud rate
#BAUD = -b 19200
BAUD = -B16


# -------------------------- falgs -----------------------------------
# compiler flags
CFLAGS += -DF_CPU=$(F_CPU)



# ------------------------ pipelines ---------------------------------
all:
	make clean
	make compile

clean:
	mkdir -p build 
	rm -f build/*.o
	rm -f build/*.hex
	rm -f build/*.elf


compile:
	avr-gcc -g -Os -mmcu=$(MCU) $(CFLAGS) -c $(SRC_PATH)/$(MAIN_FILE).c -o $(BUILD_PATH)/$(MAIN_FILE).o
	avr-gcc -g -mmcu=$(MCU) -o $(BUILD_PATH)/$(MAIN_FILE).elf $(BUILD_PATH)/$(MAIN_FILE).o
	avr-objcopy -j .text -j .data -O ihex $(BUILD_PATH)/$(MAIN_FILE).elf $(BUILD_PATH)/$(MAIN_FILE).hex
	avr-size --format=avr --mcu=$(MCU) $(BUILD_PATH)/$(MAIN_FILE).elf 


flash:
	sudo avrdude -c $(PGMR) -p $(MCU) -P $(PORT) $(BAUD) -U flash:w:$(BUILD_PATH)/$(MAIN_FILE).hex