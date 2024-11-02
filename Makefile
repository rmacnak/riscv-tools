run_test: out out/test_rv64 out/test_rv32
	./out/test_rv64
	./out/test_rv32

out:
	mkdir -p out

CFLAGS64 = -O2 -g -std=c++17 -Werror -Wall -Wextra -Wnon-virtual-dtor -Wvla -Wno-unused-parameter -DDEBUG -DXLEN=64 -fno-rtti -fno-exceptions -I.

out/test_rv64: out/simulator64.o out/disassembler64.o out/assembler64.o out/test64.o out/assert64.o out/os_macos64.o out/os_linux64.o
	$(CXX) $(CFLAGS64) -lm -o out/test_rv64 out/simulator64.o out/disassembler64.o out/assembler64.o out/test64.o out/assert64.o out/os_macos64.o out/os_linux64.o

out/assembler64.o: vm/assembler_riscv.cc vm/*.h Makefile
	$(CXX) $(CFLAGS64) -c -o out/assembler64.o vm/assembler_riscv.cc

out/assert64.o: vm/assert.cc vm/*.h Makefile
	$(CXX) $(CFLAGS64) -c -o out/assert64.o vm/assert.cc

out/disassembler64.o: vm/disassembler_riscv.cc vm/*.h Makefile
	$(CXX) $(CFLAGS64) -c -o out/disassembler64.o vm/disassembler_riscv.cc

out/simulator64.o: vm/simulator_riscv.cc vm/*.h Makefile
	$(CXX) $(CFLAGS64) -c -o out/simulator64.o vm/simulator_riscv.cc

out/os_macos64.o: vm/os_macos.cc vm/*.h Makefile
	$(CXX) $(CFLAGS64) -c -o out/os_macos64.o vm/os_macos.cc

out/os_linux64.o: vm/os_linux.cc vm/*.h Makefile
	$(CXX) $(CFLAGS64) -c -o out/os_linux64.o vm/os_linux.cc

out/test64.o: vm/test.cc vm/*.h Makefile
	$(CXX) $(CFLAGS64) -c -o out/test64.o vm/test.cc

CFLAGSc64 = -O2 -g -std=c++17 -Werror -Wall -Wextra -Wnon-virtual-dtor -Wvla -Wno-unused-parameter -DDEBUG -DXLEN=32 -fno-rtti -fno-exceptions -I.

out/test_rv32: out/simulatorc64.o out/disassemblerc64.o out/assemblerc64.o out/testc64.o out/assertc64.o out/os_macosc64.o out/os_linuxc64.o
	$(CXX) $(CFLAGSc64) -lm -o out/test_rv32 out/simulatorc64.o out/disassemblerc64.o out/assemblerc64.o out/testc64.o out/assertc64.o out/os_macosc64.o out/os_linuxc64.o

out/assemblerc64.o: vm/assembler_riscv.cc vm/*.h Makefile
	$(CXX) $(CFLAGSc64) -c -o out/assemblerc64.o vm/assembler_riscv.cc

out/assertc64.o: vm/assert.cc vm/*.h Makefile
	$(CXX) $(CFLAGSc64) -c -o out/assertc64.o vm/assert.cc

out/disassemblerc64.o: vm/disassembler_riscv.cc vm/*.h Makefile
	$(CXX) $(CFLAGSc64) -c -o out/disassemblerc64.o vm/disassembler_riscv.cc

out/simulatorc64.o: vm/simulator_riscv.cc vm/*.h Makefile
	$(CXX) $(CFLAGSc64) -c -o out/simulatorc64.o vm/simulator_riscv.cc

out/os_macosc64.o: vm/os_macos.cc vm/*.h Makefile
	$(CXX) $(CFLAGSc64) -c -o out/os_macosc64.o vm/os_macos.cc

out/os_linuxc64.o: vm/os_linux.cc vm/*.h Makefile
	$(CXX) $(CFLAGSc64) -c -o out/os_linuxc64.o vm/os_linux.cc

out/testc64.o: vm/test.cc vm/*.h Makefile
	$(CXX) $(CFLAGSc64) -c -o out/testc64.o vm/test.cc

