# stm8s_template
The project template for stm8s, compile with sdcc 4.0.0

### Install ###
Post: https://www.codementor.io/@hbendali/
getting-started-with-stm8-development-tools-on-gnu-linux-zu59yo35x
Post: https://chisight.wordpress.com/2018/02/07/installing-sdcc-on-debian-or-ubuntu/
Refer: https://github.com/gicking/STM8-SPL_SDCC_patch/blob/master/STM8S_StdPeriph_Lib_V2.3.1_sdcc.patch

###  sdcc ###
Download: http://sourceforge.net/projects/sdcc/files/snapshot_builds/amd64-unknown-linux2.5/sdcc-snapshot-amd64-unknown-linux2.5-20200113-11515.tar.bz2/download

sudo apt install build-essential libboost-all-dev bison flex texinfo
tar -xvjf sdcc.tar.bz2
cd sdcc
./configure --disable-pic14-port --disable-pic16-port
make
sudo make install

### stm8flash ###
git clone https://github.com/vdudouyt/stm8flash.git
cd stm8flash
make
sudo make install

### OpenOCD ###
git clone https://git.code.sf.net/p/openocd/code openocd-code
cd openocd-code/
./bootstrap
./configure
make
sudo make install

### stm8-gdb ###
Download: https://sourceforge.net/projects/stm8-binutils-gdb/files/stm8-binutils-gdb-sources-2018-03-04.tar.gz/download

cd stm8-binutils-gdb-sources
./patch_binutils.sh
./configure_binutils.sh
cd binutils-2.30
make
sudo make install

### Build ###
sdcc -lstm8 -mstm8 --opt-code-size --std-sdcc99 --nogcse --all-callee-saves --debug --verbose --stack-auto --fverbose-asm --float-reent --no-peep -I./ -I./STM8S_StdPeriph_Driver/inc -D STM8S003 ./stm8_blinky.c

### Debug ###
openocd -f /usr/local/share/openocd/scripts/interface/stlink.cfg -f /usr/local/share/openocd/scripts/target/stm8s.cfg -c "init" -c "reset halt"

stm8-gdb test.elf --tui
start
