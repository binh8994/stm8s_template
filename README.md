# stm8s_template
The project template for stm8s, compile with sdcc 4.0.0

### Install ###
Post: https://www.codementor.io/@hbendali/getting-started-with-stm8-development-tools-on-gnu-linux-zu59yo35x<br>
Post: https://chisight.wordpress.com/2018/02/07/installing-sdcc-on-debian-or-ubuntu/<br>
Refer: https://github.com/gicking/STM8-SPL_SDCC_patch/blob/master/STM8S_StdPeriph_Lib_V2.3.1_sdcc.patch<br>

###  sdcc ###
Download: http://sourceforge.net/projects/sdcc/files/snapshot_builds/amd64-unknown-linux2.5/sdcc-snapshot-amd64-unknown-linux2.5-20200113-11515.tar.bz2/download<br>

sudo apt install build-essential libboost-all-dev bison flex texinfo<br>
tar -xvjf sdcc.tar.bz2<br>
cd sdcc<br>
./configure --disable-pic14-port --disable-pic16-port<br>
make<br>
sudo make install<br>

### stm8flash ###
git clone https://github.com/vdudouyt/stm8flash.git<br>
cd stm8flash<br>
make<br>
sudo make install<br>

### OpenOCD ###
git clone https://git.code.sf.net/p/openocd/code openocd-code<br>
cd openocd-code/<br>
./bootstrap<br>
./configure<br>
make<br>
sudo make install<br>

### stm8-gdb ###
Download: https://sourceforge.net/projects/stm8-binutils-gdb/files/stm8-binutils-gdb-sources-2018-03-04.tar.gz/download<br>

cd stm8-binutils-gdb-sources<br>
./patch_binutils.sh<br>
./configure_binutils.sh<br>
cd binutils-2.30<br>
make<br>
sudo make install<br>

### Build ###
sdcc -lstm8 -mstm8 --opt-code-size --std-sdcc99 --nogcse --all-callee-saves --debug --verbose --stack-auto --fverbose-asm --float-reent --no-peep -I./ -I./STM8S_StdPeriph_Driver/inc -D STM8S003 ./stm8_blinky.c<br>

### Debug ###
openocd -f /usr/local/share/openocd/scripts/interface/stlink.cfg -f /usr/local/share/openocd/scripts/target/stm8s.cfg -c "init" -c "reset halt"<br>

stm8-gdb test.elf --tui<br>
start<br>
