echo copying h files...
if [ ! -d "include" ]; then
mkdir include
fi
cp -r UDG_ShrimpIII/UDG_ShrimpIII.h include

echo Generating Raspberry Pi binaries...
echo compiling source files for Raspberry pi...
g++ -c -std=c++0x UDG_ShrimpIII/UDG_ShrimpIII.cpp  UDG_ShrimpIII/SerialPort.cpp
echo linking and packaging...   
ar rvs libUDG_ShrimpIII.a UDG_ShrimpIII.o  SerialPort.o
ar rvs UDG_ShrimpIII.a UDG_ShrimpIII.o  SerialPort.o
echo cleaning...
rm *.o
if [ ! -d "bin/RaspberryPi" ]; then
mkdir -p bin/RaspberryPi
fi
mv libUDG_ShrimpIII.a bin/RaspberryPi
mv UDG_ShrimpIII.a bin/RaspberryPi