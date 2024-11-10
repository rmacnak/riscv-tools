run_test: out/rv64 out/rv64/test out/rv32/test
	./out/rv64/test
	./out/rv32/test

CXXFLAGS += -O2 -g -std=c++17 -Werror -Wall -Wextra -Wnon-virtual-dtor -Wvla -Wno-unused-parameter -DDEBUG -fno-rtti -fno-exceptions -I.

out/rv64:
	mkdir -p out/rv64

out/rv64/assembler.o: vm/assembler_riscv.cc vm/*.h Makefile out/rv64
	$(CXX) $(CXXFLAGS) -DXLEN=64 -c -o out/rv64/assembler.o vm/assembler_riscv.cc

out/rv64/assert.o: vm/assert.cc vm/*.h Makefile out/rv64
	$(CXX) $(CXXFLAGS) -DXLEN=64 -c -o out/rv64/assert.o vm/assert.cc

out/rv64/disassembler.o: vm/disassembler_riscv.cc vm/*.h Makefile out/rv64
	$(CXX) $(CXXFLAGS) -DXLEN=64 -c -o out/rv64/disassembler.o vm/disassembler_riscv.cc

out/rv64/simulator.o: vm/simulator_riscv.cc vm/*.h Makefile out/rv64
	$(CXX) $(CXXFLAGS) -DXLEN=64 -c -o out/rv64/simulator.o vm/simulator_riscv.cc

out/rv64/os_macos.o: vm/os_macos.cc vm/*.h Makefile out/rv64
	$(CXX) $(CXXFLAGS) -DXLEN=64 -c -o out/rv64/os_macos.o vm/os_macos.cc

out/rv64/os_linux.o: vm/os_linux.cc vm/*.h Makefile out/rv64
	$(CXX) $(CXXFLAGS) -DXLEN=64 -c -o out/rv64/os_linux.o vm/os_linux.cc

out/rv64/test.o: vm/test.cc vm/*.h Makefile out/rv64
	$(CXX) $(CXXFLAGS) -DXLEN=64 -c -o out/rv64/test.o vm/test.cc

out/rv64/test: out/rv64/simulator.o out/rv64/disassembler.o out/rv64/assembler.o out/rv64/test.o out/rv64/assert.o out/rv64/os_macos.o out/rv64/os_linux.o
	$(CXX) $(CXXFLAGS) -lm -o out/rv64/test out/rv64/simulator.o out/rv64/disassembler.o out/rv64/assembler.o out/rv64/test.o out/rv64/assert.o out/rv64/os_macos.o out/rv64/os_linux.o

out/rv32:
	mkdir -p out/rv32

out/rv32/assembler.o: vm/assembler_riscv.cc vm/*.h Makefile out/rv32
	$(CXX) $(CXXFLAGS) -DXLEN=32 -c -o out/rv32/assembler.o vm/assembler_riscv.cc

out/rv32/assert.o: vm/assert.cc vm/*.h Makefile out/rv32
	$(CXX) $(CXXFLAGS) -DXLEN=32 -c -o out/rv32/assert.o vm/assert.cc

out/rv32/disassembler.o: vm/disassembler_riscv.cc vm/*.h Makefile out/rv32
	$(CXX) $(CXXFLAGS) -DXLEN=32 -c -o out/rv32/disassembler.o vm/disassembler_riscv.cc

out/rv32/simulator.o: vm/simulator_riscv.cc vm/*.h Makefile out/rv32
	$(CXX) $(CXXFLAGS) -DXLEN=32 -c -o out/rv32/simulator.o vm/simulator_riscv.cc

out/rv32/os_macos.o: vm/os_macos.cc vm/*.h Makefile out/rv32
	$(CXX) $(CXXFLAGS) -DXLEN=32 -c -o out/rv32/os_macos.o vm/os_macos.cc

out/rv32/os_linux.o: vm/os_linux.cc vm/*.h Makefile out/rv32
	$(CXX) $(CXXFLAGS) -DXLEN=32 -c -o out/rv32/os_linux.o vm/os_linux.cc

out/rv32/test.o: vm/test.cc vm/*.h Makefile out/rv32
	$(CXX) $(CXXFLAGS) -DXLEN=32 -c -o out/rv32/test.o vm/test.cc

out/rv32/test: out/rv32/simulator.o out/rv32/disassembler.o out/rv32/assembler.o out/rv32/test.o out/rv32/assert.o out/rv32/os_macos.o out/rv32/os_linux.o
	$(CXX) $(CXXFLAGS) -lm -o out/rv32/test out/rv32/simulator.o out/rv32/disassembler.o out/rv32/assembler.o out/rv32/test.o out/rv32/assert.o out/rv32/os_macos.o out/rv32/os_linux.o
