MicroController Toolchain initialisation on Rasberry Pi

Connect through ssh to you RasberryPi and perform the following steps:

Make sure you pulled the latest software version form the duckietown software master branch!

On the Rasberry Pi, execute the following commands (Note, the $ indicates a command, which has to be executed, but the $ must not be typed):

install avrdude and gcc

    $ sudo apt-get install bison autoconf flex
    $ sudo apt-get install gcc-avr binutils-avr gdb-avr avr-libc avrdude

Copy the avrdude config file

    $ cd ~/duckietown/hardware/software/_avrdudeconfig/
    $ sudo cp avrdude.conf /etc/avrdude.conf

Test avrdude and set fuses

    $ cd ~/duckietown/hardware/traffic-light/software
    $ make fuses

if there is the message "make: warning:  Clock skew detected.  Your build may be incomplete." or the make process is not stopping and many debugging messages are showed, try the following:

Press Ctrl+C to stop the current commant.

    $ find -exec touch \{\} \;

This ensures that the modification time of all files is set to the current time. Make decides, which files have to be compiled by comparing the source file time with the executable file time. If the executable file time lies in the future regarding the current system time, not all modified files are compiled. This could happen when the clock of the Rasberry Pi is not set correctly and the file timestamps of, e.g., a github pull are used.

    $ make clean

This removes all temporary files, so every thing has to be compiled freshly.

if the output of `$ make fuses` is at the end like
 
    avrdude: verifying ...
    avrdude: 1 bytes of efuse verified

    avrdude: safemode: Fuses OK (E:FF, H:DF, L:E2)

    avrdude done.  Thank you.

the connection to the MC works and the fuses could be written. The fuses are some low lowlevel settings, which just have to be set once for each microcontroller.

Compile the programm and download it to the MC

    $ make

the output should look like:

    ...

    Errors: none
    -------- end --------

    sudo avrdude -p attiny861 -c linuxgpio -P  -q -U flash:w:main.hex 

    avrdude: AVR device initialized and ready to accept instructions

    Reading | ################################################## | 100% 0.00s

    avrdude: Device signature = 0x1e930d (probably t861)
    avrdude: NOTE: "flash" memory has been specified, an erase cycle will be performed
         To disable this feature, specify the -D option.
    avrdude: erasing chip
    avrdude: reading input file "main.hex"
    avrdude: input file main.hex auto detected as Intel Hex
    avrdude: writing flash (2220 bytes):

    Writing | ################################################## | 100% 0.75s

    avrdude: 2220 bytes of flash written
    avrdude: verifying flash memory against main.hex:
    avrdude: load data flash data from input file main.hex:
    avrdude: input file main.hex auto detected as Intel Hex
    avrdude: input file main.hex contains 2220 bytes
    avrdude: reading on-chip flash data:

    Reading | ################################################## | 100% 0.58s

    avrdude: verifying ...
    avrdude: 2220 bytes of flash verified

    avrdude: safemode: Fuses OK (E:FF, H:DF, L:E2)

    avrdude done.  Thank you.


With that, the MC should work. To change the MC programm, just edit the files, e.g with vim. With `$ make` you can compile and download the programm to the MC again.


Hint: in vim with the key [i] one can start editing mode, with [ESC], [:],[w],[q],[enter] one can leave edit mode and save the current file and close vim

