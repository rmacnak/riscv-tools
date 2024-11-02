// Copyright (c) 2017, the Newspeak project authors. Please see the AUTHORS file
// for details. All rights reserved. Use of this source code is governed by a
// BSD-style license that can be found in the LICENSE file.

#include <math.h>
#include <stdarg.h>
#include <stdio.h>

#include <limits>
#include <sstream>
#include <string>

#include "vm/assembler_riscv.h"
#include "vm/macroassembler_riscv.h"
#include "vm/assert.h"
#include "vm/disassembler_riscv.h"
#include "vm/globals.h"
#include "vm/simulator_riscv.h"

namespace psoup {

class Expect {
 public:
  Expect(const char* file, int line) : file_(file), line_(line) {}

  void Fail(const char* format, ...) PRINTF_ATTRIBUTE(2, 3);

  template <typename E, typename A>
  void Equals(const E& expected, const A& actual);

  template <typename E, typename A>
  void StringEquals(const E& expected, const A& actual);

  template <typename T>
  void BitwiseEquals(const T& expected, const T& actual);

 private:
  const char* const file_;
  const int line_;

  DISALLOW_IMPLICIT_CONSTRUCTORS(Expect);
};

#define EXPECT_EQ(expected, actual)                                            \
  psoup::Expect(__FILE__, __LINE__).Equals((expected), (actual))

#define EXPECT_STREQ(expected, actual)                                         \
  psoup::Expect(__FILE__, __LINE__).StringEquals((expected), (actual))

#define EXPECT_BITEQ(expected, actual)                                  \
  psoup::Expect(__FILE__, __LINE__).BitwiseEquals((expected), (actual))

void Expect::Fail(const char* format, ...) {
  fprintf(stderr, "%s:%d: error: ", file_, line_);
  va_list arguments;
  va_start(arguments, format);
  vfprintf(stderr, format, arguments);
  va_end(arguments);
  fprintf(stderr, "\n");
  fflush(stderr);
  abort();
}

template <typename E, typename A>
void Expect::Equals(const E& expected, const A& actual) {
  if (actual == expected)
    return;
  std::ostringstream ess, ass;
  ess << expected;
  ass << actual;
  std::string es = ess.str(), as = ass.str();
  Fail("expected: <%s> but was: <%s>", es.c_str(), as.c_str());
}

template <typename E, typename A>
void Expect::StringEquals(const E& expected, const A& actual) {
  std::ostringstream ess, ass;
  ess << expected;
  ass << actual;
  std::string es = ess.str(), as = ass.str();
  if (as == es)
    return;
  Fail("expected:\n<\"%s\">\nbut was:\n<\"%s\">", es.c_str(), as.c_str());
}

template <typename T>
void Expect::BitwiseEquals(const T& expected, const T& actual) {
  if (memcmp(&expected, &actual, sizeof(T)) == 0)
    return;
  std::ostringstream ess, ass;
  ess << expected;
  ass << actual;
  std::string es = ess.str(), as = ass.str();
  Fail("expected: <%s> but was: <%s>", es.c_str(), as.c_str());
}

class UnitTest;

UnitTest* tests_;

class UnitTest {
 public:
  UnitTest(const char* name, void (*function)(void))
      : name_(name), function_(function), next_(tests_) {
    tests_ = this;
  }

  void Run() {
    printf("%s\n", name_);
    function_();
  }

  UnitTest* next() { return next_; }

 private:
  const char* name_;
  void (*function_)(void);
  UnitTest* next_;
};

#define UNIT_TEST(name)                                                        \
  void UnitTest_##name();                                                      \
  static const UnitTest kRegister##name(#name, UnitTest_##name);               \
  void UnitTest_##name()

#define __ assembler.

UNIT_TEST(LoadUpperImmediate) {
  Assembler assembler(RV_G);
  __ lui(A0, 42 << 16);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  002a0537 lui a0, 2752512\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(42 << 16, simulator.Call(buffer));
}

UNIT_TEST(AddUpperImmediatePC) {
  Assembler assembler(RV_G);
  __ auipc(A0, 0);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  00000517 auipc a0, 0\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(buffer, Memory::ToHost<void>(simulator.Call(buffer)));
}

UNIT_TEST(JumpAndLink) {
  Assembler assembler(RV_G);
  Label label1, label2;
  __ jal(T4, &label1);  // Forward.
  __ sub(A0, T0, T1);
  __ ret();
  __ trap();

  __ Bind(&label2);
  __ li(T1, 7);
  __ jalr(ZERO, T5);
  __ trap();

  __ Bind(&label1);
  __ li(T0, 4);
  __ jal(T5, &label2);  // Backward.
  __ jalr(ZERO, T4);
  __ trap();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  01c00eef jal t4, +28\n"
      "  40628533 sub a0, t0, t1\n"
      "  00008067 ret\n"
      "  00000000 trap\n"
      "  00700313 li t1, 7\n"
      "  000f0067 jr t5\n"
      "  00000000 trap\n"
      "  00400293 li t0, 4\n"
      "  ff1fff6f jal t5, -16\n"
      "  000e8067 jr t4\n"
      "  00000000 trap\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(-3, simulator.Call(buffer));
}

UNIT_TEST(Jump) {
  Assembler assembler(RV_G);
  Label label1, label2;
  __ j(&label1);  // Forward.
  __ trap();
  __ Bind(&label2);
  __ li(T1, 7);
  __ sub(A0, T0, T1);
  __ ret();
  __ Bind(&label1);
  __ li(T0, 4);
  __ j(&label2);  // Backward.
  __ trap();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  0140006f j +20\n"
      "  00000000 trap\n"
      "  00700313 li t1, 7\n"
      "  40628533 sub a0, t0, t1\n"
      "  00008067 ret\n"
      "  00400293 li t0, 4\n"
      "  ff1ff06f j -16\n"
      "  00000000 trap\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(-3, simulator.Call(buffer));
}

UNIT_TEST(JumpAndLinkRegister) {
  Assembler assembler(RV_G);
  /* 00 */ __ jalr(T4, A1, 28);  // Forward.
  /* 04 */ __ sub(A0, T0, T1);
  /* 08 */ __ ret();
  /* 12 */ __ trap();

  /* 16 */ __ li(T1, 7);
  /* 20 */ __ jalr(ZERO, T5);
  /* 24 */ __ trap();

  /* 28 */ __ li(T0, 4);
  /* 32 */ __ jalr(T5, A1, 16);  // Backward.
  /* 36 */ __ jalr(ZERO, T4);
  /* 40 */ __ trap();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  01c58ee7 jalr t4, 28(a1)\n"
      "  40628533 sub a0, t0, t1\n"
      "  00008067 ret\n"
      "  00000000 trap\n"
      "  00700313 li t1, 7\n"
      "  000f0067 jr t5\n"
      "  00000000 trap\n"
      "  00400293 li t0, 4\n"
      "  01058f67 jalr t5, 16(a1)\n"
      "  000e8067 jr t4\n"
      "  00000000 trap\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(-3, simulator.Call(buffer, 0, Memory::ToGuest(buffer)));
}

UNIT_TEST(JumpRegister) {
  Assembler assembler(RV_G);
  /* 00 */ __ jr(A1, 20);  // Forward.
  /* 04 */ __ trap();
  /* 08 */ __ li(T1, 7);
  /* 12 */ __ sub(A0, T0, T1);
  /* 16 */ __ ret();
  /* 20 */ __ li(T0, 4);
  /* 24 */ __ jr(A1, 8);  // Backward.
  /* 28 */ __ trap();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  01458067 jr 20(a1)\n"
      "  00000000 trap\n"
      "  00700313 li t1, 7\n"
      "  40628533 sub a0, t0, t1\n"
      "  00008067 ret\n"
      "  00400293 li t0, 4\n"
      "  00858067 jr 8(a1)\n"
      "  00000000 trap\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(-3, simulator.Call(buffer, 0, Memory::ToGuest(buffer)));
}

UNIT_TEST(BranchEqualForward) {
  Assembler assembler(RV_G);
  Label label;
  __ beq(A0, A1, &label);
  __ li(A0, 3);
  __ ret();
  __ Bind(&label);
  __ li(A0, 4);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  00b50663 beq a0, a1, +12\n"
      "  00300513 li a0, 3\n"
      "  00008067 ret\n"
      "  00400513 li a0, 4\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(4, simulator.Call(buffer, 1, 1));
  EXPECT_EQ(3, simulator.Call(buffer, 1, 0));
  EXPECT_EQ(3, simulator.Call(buffer, 1, -1));
  EXPECT_EQ(3, simulator.Call(buffer, 0, 1));
  EXPECT_EQ(4, simulator.Call(buffer, 0, 0));
  EXPECT_EQ(3, simulator.Call(buffer, 0, -1));
  EXPECT_EQ(3, simulator.Call(buffer, -1, 1));
  EXPECT_EQ(3, simulator.Call(buffer, -1, 0));
  EXPECT_EQ(4, simulator.Call(buffer, -1, -1));
}

UNIT_TEST(BranchNotEqualForward) {
  Assembler assembler(RV_G);
  Label label;
  __ bne(A0, A1, &label);
  __ li(A0, 3);
  __ ret();
  __ Bind(&label);
  __ li(A0, 4);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  00b51663 bne a0, a1, +12\n"
      "  00300513 li a0, 3\n"
      "  00008067 ret\n"
      "  00400513 li a0, 4\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(3, simulator.Call(buffer, 1, 1));
  EXPECT_EQ(4, simulator.Call(buffer, 1, 0));
  EXPECT_EQ(4, simulator.Call(buffer, 1, -1));
  EXPECT_EQ(4, simulator.Call(buffer, 0, 1));
  EXPECT_EQ(3, simulator.Call(buffer, 0, 0));
  EXPECT_EQ(4, simulator.Call(buffer, 0, -1));
  EXPECT_EQ(4, simulator.Call(buffer, -1, 1));
  EXPECT_EQ(4, simulator.Call(buffer, -1, 0));
  EXPECT_EQ(3, simulator.Call(buffer, -1, -1));
}

UNIT_TEST(BranchLessThanForward) {
  Assembler assembler(RV_G);
  Label label;
  __ blt(A0, A1, &label);
  __ li(A0, 3);
  __ ret();
  __ Bind(&label);
  __ li(A0, 4);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  00b54663 blt a0, a1, +12\n"
      "  00300513 li a0, 3\n"
      "  00008067 ret\n"
      "  00400513 li a0, 4\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(3, simulator.Call(buffer, 1, 1));
  EXPECT_EQ(3, simulator.Call(buffer, 1, 0));
  EXPECT_EQ(3, simulator.Call(buffer, 1, -1));
  EXPECT_EQ(4, simulator.Call(buffer, 0, 1));
  EXPECT_EQ(3, simulator.Call(buffer, 0, 0));
  EXPECT_EQ(3, simulator.Call(buffer, 0, -1));
  EXPECT_EQ(4, simulator.Call(buffer, -1, 1));
  EXPECT_EQ(4, simulator.Call(buffer, -1, 0));
  EXPECT_EQ(3, simulator.Call(buffer, -1, -1));
}

UNIT_TEST(BranchLessOrEqualForward) {
  Assembler assembler(RV_G);
  Label label;
  __ ble(A0, A1, &label);
  __ li(A0, 3);
  __ ret();
  __ Bind(&label);
  __ li(A0, 4);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  00a5d663 ble a0, a1, +12\n"
      "  00300513 li a0, 3\n"
      "  00008067 ret\n"
      "  00400513 li a0, 4\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(4, simulator.Call(buffer, 1, 1));
  EXPECT_EQ(3, simulator.Call(buffer, 1, 0));
  EXPECT_EQ(3, simulator.Call(buffer, 1, -1));
  EXPECT_EQ(4, simulator.Call(buffer, 0, 1));
  EXPECT_EQ(4, simulator.Call(buffer, 0, 0));
  EXPECT_EQ(3, simulator.Call(buffer, 0, -1));
  EXPECT_EQ(4, simulator.Call(buffer, -1, 1));
  EXPECT_EQ(4, simulator.Call(buffer, -1, 0));
  EXPECT_EQ(4, simulator.Call(buffer, -1, -1));
}

UNIT_TEST(BranchGreaterThanForward) {
  Assembler assembler(RV_G);
  Label label;
  __ bgt(A0, A1, &label);
  __ li(A0, 3);
  __ ret();
  __ Bind(&label);
  __ li(A0, 4);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  00a5c663 blt a1, a0, +12\n"
      "  00300513 li a0, 3\n"
      "  00008067 ret\n"
      "  00400513 li a0, 4\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(3, simulator.Call(buffer, 1, 1));
  EXPECT_EQ(4, simulator.Call(buffer, 1, 0));
  EXPECT_EQ(4, simulator.Call(buffer, 1, -1));
  EXPECT_EQ(3, simulator.Call(buffer, 0, 1));
  EXPECT_EQ(3, simulator.Call(buffer, 0, 0));
  EXPECT_EQ(4, simulator.Call(buffer, 0, -1));
  EXPECT_EQ(3, simulator.Call(buffer, -1, 1));
  EXPECT_EQ(3, simulator.Call(buffer, -1, 0));
  EXPECT_EQ(3, simulator.Call(buffer, -1, -1));
}

UNIT_TEST(BranchGreaterOrEqualForward) {
  Assembler assembler(RV_G);
  Label label;
  __ bge(A0, A1, &label);
  __ li(A0, 3);
  __ ret();
  __ Bind(&label);
  __ li(A0, 4);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  00b55663 ble a1, a0, +12\n"
      "  00300513 li a0, 3\n"
      "  00008067 ret\n"
      "  00400513 li a0, 4\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(4, simulator.Call(buffer, 1, 1));
  EXPECT_EQ(4, simulator.Call(buffer, 1, 0));
  EXPECT_EQ(4, simulator.Call(buffer, 1, -1));
  EXPECT_EQ(3, simulator.Call(buffer, 0, 1));
  EXPECT_EQ(4, simulator.Call(buffer, 0, 0));
  EXPECT_EQ(4, simulator.Call(buffer, 0, -1));
  EXPECT_EQ(3, simulator.Call(buffer, -1, 1));
  EXPECT_EQ(3, simulator.Call(buffer, -1, 0));
  EXPECT_EQ(4, simulator.Call(buffer, -1, -1));
}

UNIT_TEST(BranchLessThanUnsignedForward) {
  Assembler assembler(RV_G);
  Label label;
  __ bltu(A0, A1, &label);
  __ li(A0, 3);
  __ ret();
  __ Bind(&label);
  __ li(A0, 4);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  00b56663 bltu a0, a1, +12\n"
      "  00300513 li a0, 3\n"
      "  00008067 ret\n"
      "  00400513 li a0, 4\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(3, simulator.Call(buffer, 1, 1));
  EXPECT_EQ(3, simulator.Call(buffer, 1, 0));
  EXPECT_EQ(4, simulator.Call(buffer, 1, -1));
  EXPECT_EQ(4, simulator.Call(buffer, 0, 1));
  EXPECT_EQ(3, simulator.Call(buffer, 0, 0));
  EXPECT_EQ(4, simulator.Call(buffer, 0, -1));
  EXPECT_EQ(3, simulator.Call(buffer, -1, 1));
  EXPECT_EQ(3, simulator.Call(buffer, -1, 0));
  EXPECT_EQ(3, simulator.Call(buffer, -1, -1));
}

UNIT_TEST(BranchLessOrEqualUnsignedForward) {
  Assembler assembler(RV_G);
  Label label;
  __ bleu(A0, A1, &label);
  __ li(A0, 3);
  __ ret();
  __ Bind(&label);
  __ li(A0, 4);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  00a5f663 bleu a0, a1, +12\n"
      "  00300513 li a0, 3\n"
      "  00008067 ret\n"
      "  00400513 li a0, 4\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(4, simulator.Call(buffer, 1, 1));
  EXPECT_EQ(3, simulator.Call(buffer, 1, 0));
  EXPECT_EQ(4, simulator.Call(buffer, 1, -1));
  EXPECT_EQ(4, simulator.Call(buffer, 0, 1));
  EXPECT_EQ(4, simulator.Call(buffer, 0, 0));
  EXPECT_EQ(4, simulator.Call(buffer, 0, -1));
  EXPECT_EQ(3, simulator.Call(buffer, -1, 1));
  EXPECT_EQ(3, simulator.Call(buffer, -1, 0));
  EXPECT_EQ(4, simulator.Call(buffer, -1, -1));
}

UNIT_TEST(BranchGreaterThanUnsignedForward) {
  Assembler assembler(RV_G);
  Label label;
  __ bgtu(A0, A1, &label);
  __ li(A0, 3);
  __ ret();
  __ Bind(&label);
  __ li(A0, 4);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  00a5e663 bltu a1, a0, +12\n"
      "  00300513 li a0, 3\n"
      "  00008067 ret\n"
      "  00400513 li a0, 4\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(3, simulator.Call(buffer, 1, 1));
  EXPECT_EQ(4, simulator.Call(buffer, 1, 0));
  EXPECT_EQ(3, simulator.Call(buffer, 1, -1));
  EXPECT_EQ(3, simulator.Call(buffer, 0, 1));
  EXPECT_EQ(3, simulator.Call(buffer, 0, 0));
  EXPECT_EQ(3, simulator.Call(buffer, 0, -1));
  EXPECT_EQ(4, simulator.Call(buffer, -1, 1));
  EXPECT_EQ(4, simulator.Call(buffer, -1, 0));
  EXPECT_EQ(3, simulator.Call(buffer, -1, -1));
}

UNIT_TEST(BranchGreaterOrEqualUnsignedForward) {
  Assembler assembler(RV_G);
  Label label;
  __ bgeu(A0, A1, &label);
  __ li(A0, 3);
  __ ret();
  __ Bind(&label);
  __ li(A0, 4);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  00b57663 bleu a1, a0, +12\n"
      "  00300513 li a0, 3\n"
      "  00008067 ret\n"
      "  00400513 li a0, 4\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(4, simulator.Call(buffer, 1, 1));
  EXPECT_EQ(4, simulator.Call(buffer, 1, 0));
  EXPECT_EQ(3, simulator.Call(buffer, 1, -1));
  EXPECT_EQ(3, simulator.Call(buffer, 0, 1));
  EXPECT_EQ(4, simulator.Call(buffer, 0, 0));
  EXPECT_EQ(3, simulator.Call(buffer, 0, -1));
  EXPECT_EQ(4, simulator.Call(buffer, -1, 1));
  EXPECT_EQ(4, simulator.Call(buffer, -1, 0));
  EXPECT_EQ(4, simulator.Call(buffer, -1, -1));
}

UNIT_TEST(LoadByte) {
  {
    Assembler assembler(RV_G);
    __ lb(A0, Address(A0, 0));
    __ ret();

    void* buffer = assembler.buffer();
    size_t size = assembler.size();

    Disassembler disassembler(RV_G);
    char* disassembly = disassembler.Disassemble(buffer, size);
    EXPECT_STREQ(
        "  00050503 lb a0, 0(a0)\n"
        "  00008067 ret\n",
        disassembly);
    free(disassembly);

    uint8_t* values =
        reinterpret_cast<uint8_t*>(Memory::Allocate(3 * sizeof(uint8_t)));
    values[0] = 0xAB;
    values[1] = 0xCD;
    values[2] = 0xEF;

    Simulator simulator;
    EXPECT_EQ(-51, simulator.Call(buffer, Memory::ToGuest(&values[1])));
  }
  {
    Assembler assembler(RV_G);
    __ lb(A0, Address(A0, 1));
    __ ret();

    void* buffer = assembler.buffer();
    size_t size = assembler.size();

    Disassembler disassembler(RV_G);
    char* disassembly = disassembler.Disassemble(buffer, size);
    EXPECT_STREQ(
        "  00150503 lb a0, 1(a0)\n"
        "  00008067 ret\n",
        disassembly);
    free(disassembly);

    uint8_t* values =
        reinterpret_cast<uint8_t*>(Memory::Allocate(3 * sizeof(uint8_t)));
    values[0] = 0xAB;
    values[1] = 0xCD;
    values[2] = 0xEF;

    Simulator simulator;
    EXPECT_EQ(-17, simulator.Call(buffer, Memory::ToGuest(&values[1])));
  }
  {
    Assembler assembler(RV_G);
    __ lb(A0, Address(A0, -1));
    __ ret();

    void* buffer = assembler.buffer();
    size_t size = assembler.size();

    Disassembler disassembler(RV_G);
    char* disassembly = disassembler.Disassemble(buffer, size);
    EXPECT_STREQ(
        "  fff50503 lb a0, -1(a0)\n"
        "  00008067 ret\n",
        disassembly);
    free(disassembly);

    uint8_t* values =
        reinterpret_cast<uint8_t*>(Memory::Allocate(3 * sizeof(uint8_t)));
    values[0] = 0xAB;
    values[1] = 0xCD;
    values[2] = 0xEF;

    Simulator simulator;
    EXPECT_EQ(-85, simulator.Call(buffer, Memory::ToGuest(&values[1])));
  }
}

UNIT_TEST(LoadByteUnsigned) {
  {
    Assembler assembler(RV_G);
    __ lbu(A0, Address(A0, 0));
    __ ret();

    void* buffer = assembler.buffer();
    size_t size = assembler.size();

    Disassembler disassembler(RV_G);
    char* disassembly = disassembler.Disassemble(buffer, size);
    EXPECT_STREQ(
        "  00054503 lbu a0, 0(a0)\n"
        "  00008067 ret\n",
        disassembly);
    free(disassembly);

    uint8_t* values =
        reinterpret_cast<uint8_t*>(Memory::Allocate(3 * sizeof(uint8_t)));
    values[0] = 0xAB;
    values[1] = 0xCD;
    values[2] = 0xEF;

    Simulator simulator;
    EXPECT_EQ(0xCD, simulator.Call(buffer, Memory::ToGuest(&values[1])));
  }
  {
    Assembler assembler(RV_G);
    __ lbu(A0, Address(A0, 1));
    __ ret();

    void* buffer = assembler.buffer();
    size_t size = assembler.size();

    Disassembler disassembler(RV_G);
    char* disassembly = disassembler.Disassemble(buffer, size);
    EXPECT_STREQ(
        "  00154503 lbu a0, 1(a0)\n"
        "  00008067 ret\n",
        disassembly);
    free(disassembly);

    uint8_t* values =
        reinterpret_cast<uint8_t*>(Memory::Allocate(3 * sizeof(uint8_t)));
    values[0] = 0xAB;
    values[1] = 0xCD;
    values[2] = 0xEF;

    Simulator simulator;
    EXPECT_EQ(0xEF, simulator.Call(buffer, Memory::ToGuest(&values[1])));
  }
  {
    Assembler assembler(RV_G);
    __ lbu(A0, Address(A0, -1));
    __ ret();

    void* buffer = assembler.buffer();
    size_t size = assembler.size();

    Disassembler disassembler(RV_G);
    char* disassembly = disassembler.Disassemble(buffer, size);
    EXPECT_STREQ(
        "  fff54503 lbu a0, -1(a0)\n"
        "  00008067 ret\n",
        disassembly);
    free(disassembly);

    uint8_t* values =
        reinterpret_cast<uint8_t*>(Memory::Allocate(3 * sizeof(uint8_t)));
    values[0] = 0xAB;
    values[1] = 0xCD;
    values[2] = 0xEF;

    Simulator simulator;
    EXPECT_EQ(0xAB, simulator.Call(buffer, Memory::ToGuest(&values[1])));
  }
}

UNIT_TEST(LoadHalfword) {
  {
    Assembler assembler(RV_G);
    __ lh(A0, Address(A0, 0));
    __ ret();

    void* buffer = assembler.buffer();
    size_t size = assembler.size();

    Disassembler disassembler(RV_G);
    char* disassembly = disassembler.Disassemble(buffer, size);
    EXPECT_STREQ(
        "  00051503 lh a0, 0(a0)\n"
        "  00008067 ret\n",
        disassembly);
    free(disassembly);

    uint16_t* values =
        reinterpret_cast<uint16_t*>(Memory::Allocate(3 * sizeof(uint16_t)));
    values[0] = 0xAB01;
    values[1] = 0xCD02;
    values[2] = 0xEF03;

    Simulator simulator;
    EXPECT_EQ(-13054, simulator.Call(buffer, Memory::ToGuest(&values[1])));
  }
  {
    Assembler assembler(RV_G);
    __ lh(A0, Address(A0, 2));
    __ ret();

    void* buffer = assembler.buffer();
    size_t size = assembler.size();

    Disassembler disassembler(RV_G);
    char* disassembly = disassembler.Disassemble(buffer, size);
    EXPECT_STREQ(
        "  00251503 lh a0, 2(a0)\n"
        "  00008067 ret\n",
        disassembly);
    free(disassembly);

    uint16_t* values =
        reinterpret_cast<uint16_t*>(Memory::Allocate(3 * sizeof(uint16_t)));
    values[0] = 0xAB01;
    values[1] = 0xCD02;
    values[2] = 0xEF03;

    Simulator simulator;
    EXPECT_EQ(-4349, simulator.Call(buffer, Memory::ToGuest(&values[1])));
  }
  {
    Assembler assembler(RV_G);
    __ lh(A0, Address(A0, -2));
    __ ret();

    void* buffer = assembler.buffer();
    size_t size = assembler.size();

    Disassembler disassembler(RV_G);
    char* disassembly = disassembler.Disassemble(buffer, size);
    EXPECT_STREQ(
        "  ffe51503 lh a0, -2(a0)\n"
        "  00008067 ret\n",
        disassembly);
    free(disassembly);

    uint16_t* values =
        reinterpret_cast<uint16_t*>(Memory::Allocate(3 * sizeof(uint16_t)));
    values[0] = 0xAB01;
    values[1] = 0xCD02;
    values[2] = 0xEF03;

    Simulator simulator;
    EXPECT_EQ(-21759, simulator.Call(buffer, Memory::ToGuest(&values[1])));
  }
}

UNIT_TEST(LoadHalfwordUnsigned) {
  {
    Assembler assembler(RV_G);
    __ lhu(A0, Address(A0, 0));
    __ ret();

    void* buffer = assembler.buffer();
    size_t size = assembler.size();

    Disassembler disassembler(RV_G);
    char* disassembly = disassembler.Disassemble(buffer, size);
    EXPECT_STREQ(
        "  00055503 lhu a0, 0(a0)\n"
        "  00008067 ret\n",
        disassembly);
    free(disassembly);

    uint16_t* values =
        reinterpret_cast<uint16_t*>(Memory::Allocate(3 * sizeof(uint16_t)));
    values[0] = 0xAB01;
    values[1] = 0xCD02;
    values[2] = 0xEF03;

    Simulator simulator;
    EXPECT_EQ(0xCD02, simulator.Call(buffer, Memory::ToGuest(&values[1])));
  }
  {
    Assembler assembler(RV_G);
    __ lhu(A0, Address(A0, 2));
    __ ret();

    void* buffer = assembler.buffer();
    size_t size = assembler.size();

    Disassembler disassembler(RV_G);
    char* disassembly = disassembler.Disassemble(buffer, size);
    EXPECT_STREQ(
        "  00255503 lhu a0, 2(a0)\n"
        "  00008067 ret\n",
        disassembly);
    free(disassembly);

    uint16_t* values =
        reinterpret_cast<uint16_t*>(Memory::Allocate(3 * sizeof(uint16_t)));
    values[0] = 0xAB01;
    values[1] = 0xCD02;
    values[2] = 0xEF03;

    Simulator simulator;
    EXPECT_EQ(0xEF03, simulator.Call(buffer, Memory::ToGuest(&values[1])));
  }
  {
    Assembler assembler(RV_G);
    __ lhu(A0, Address(A0, -2));
    __ ret();

    void* buffer = assembler.buffer();
    size_t size = assembler.size();

    Disassembler disassembler(RV_G);
    char* disassembly = disassembler.Disassemble(buffer, size);
    EXPECT_STREQ(
        "  ffe55503 lhu a0, -2(a0)\n"
        "  00008067 ret\n",
        disassembly);
    free(disassembly);

    uint16_t* values =
        reinterpret_cast<uint16_t*>(Memory::Allocate(3 * sizeof(uint16_t)));
    values[0] = 0xAB01;
    values[1] = 0xCD02;
    values[2] = 0xEF03;

    Simulator simulator;
    EXPECT_EQ(0xAB01, simulator.Call(buffer, Memory::ToGuest(&values[1])));
  }
}

UNIT_TEST(LoadWord) {
  {
    Assembler assembler(RV_G);
    __ lw(A0, Address(A0, 0));
    __ ret();

    void* buffer = assembler.buffer();
    size_t size = assembler.size();

    Disassembler disassembler(RV_G);
    char* disassembly = disassembler.Disassemble(buffer, size);
    EXPECT_STREQ(
        "  00052503 lw a0, 0(a0)\n"
        "  00008067 ret\n",
        disassembly);
    free(disassembly);

    uint32_t* values =
        reinterpret_cast<uint32_t*>(Memory::Allocate(3 * sizeof(uint32_t)));
    values[0] = 0xAB010203;
    values[1] = 0xCD020405;
    values[2] = 0xEF030607;

    Simulator simulator;
    EXPECT_EQ(-855505915, simulator.Call(buffer, Memory::ToGuest(&values[1])));
  }
  {
    Assembler assembler(RV_G);
    __ lw(A0, Address(A0, 4));
    __ ret();

    void* buffer = assembler.buffer();
    size_t size = assembler.size();

    Disassembler disassembler(RV_G);
    char* disassembly = disassembler.Disassemble(buffer, size);
    EXPECT_STREQ(
        "  00452503 lw a0, 4(a0)\n"
        "  00008067 ret\n",
        disassembly);
    free(disassembly);

    uint32_t* values =
        reinterpret_cast<uint32_t*>(Memory::Allocate(3 * sizeof(uint32_t)));
    values[0] = 0xAB010203;
    values[1] = 0xCD020405;
    values[2] = 0xEF030607;

    Simulator simulator;
    EXPECT_EQ(-285014521, simulator.Call(buffer, Memory::ToGuest(&values[1])));
  }
  {
    Assembler assembler(RV_G);
    __ lw(A0, Address(A0, -4));
    __ ret();

    void* buffer = assembler.buffer();
    size_t size = assembler.size();

    Disassembler disassembler(RV_G);
    char* disassembly = disassembler.Disassemble(buffer, size);
    EXPECT_STREQ(
        "  ffc52503 lw a0, -4(a0)\n"
        "  00008067 ret\n",
        disassembly);
    free(disassembly);

    uint32_t* values =
        reinterpret_cast<uint32_t*>(Memory::Allocate(3 * sizeof(uint32_t)));
    values[0] = 0xAB010203;
    values[1] = 0xCD020405;
    values[2] = 0xEF030607;

    Simulator simulator;
    EXPECT_EQ(-1425997309, simulator.Call(buffer, Memory::ToGuest(&values[1])));
  }
}

UNIT_TEST(StoreWord) {
  {
    Assembler assembler(RV_G);
    __ sw(A1, Address(A0, 0));
    __ ret();

    void* buffer = assembler.buffer();
    size_t size = assembler.size();

    Disassembler disassembler(RV_G);
    char* disassembly = disassembler.Disassemble(buffer, size);
    EXPECT_STREQ(
        "  00b52023 sw a1, 0(a0)\n"
        "  00008067 ret\n",
        disassembly);
    free(disassembly);

    uint32_t* values =
        reinterpret_cast<uint32_t*>(Memory::Allocate(3 * sizeof(uint32_t)));
    values[0] = 0;
    values[1] = 0;
    values[2] = 0;

    Simulator simulator;
    simulator.Call(buffer, Memory::ToGuest(&values[1]), 0xCD020405);
    EXPECT_EQ(0u, values[0]);
    EXPECT_EQ(0xCD020405, values[1]);
    EXPECT_EQ(0u, values[2]);
  }
  {
    Assembler assembler(RV_G);
    __ sw(A1, Address(A0, 4));
    __ ret();

    void* buffer = assembler.buffer();
    size_t size = assembler.size();

    Disassembler disassembler(RV_G);
    char* disassembly = disassembler.Disassemble(buffer, size);
    EXPECT_STREQ(
        "  00b52223 sw a1, 4(a0)\n"
        "  00008067 ret\n",
        disassembly);
    free(disassembly);

    uint32_t* values =
        reinterpret_cast<uint32_t*>(Memory::Allocate(3 * sizeof(uint32_t)));
    values[0] = 0;
    values[1] = 0;
    values[2] = 0;

    Simulator simulator;
    simulator.Call(buffer, Memory::ToGuest(&values[1]), 0xEF030607);
    EXPECT_EQ(0u, values[0]);
    EXPECT_EQ(0u, values[1]);
    EXPECT_EQ(0xEF030607, values[2]);
  }
  {
    Assembler assembler(RV_G);
    __ sw(A1, Address(A0, -4));
    __ ret();

    void* buffer = assembler.buffer();
    size_t size = assembler.size();

    Disassembler disassembler(RV_G);
    char* disassembly = disassembler.Disassemble(buffer, size);
    EXPECT_STREQ(
        "  feb52e23 sw a1, -4(a0)\n"
        "  00008067 ret\n",
        disassembly);
    free(disassembly);

    uint32_t* values =
        reinterpret_cast<uint32_t*>(Memory::Allocate(3 * sizeof(uint32_t)));
    values[0] = 0;
    values[1] = 0;
    values[2] = 0;

    Simulator simulator;
    simulator.Call(buffer, Memory::ToGuest(&values[1]), 0xAB010203);
    EXPECT_EQ(0xAB010203, values[0]);
    EXPECT_EQ(0u, values[1]);
    EXPECT_EQ(0u, values[2]);
  }
}

#if XLEN >= 64
UNIT_TEST(LoadWordUnsigned) {
  {
    Assembler assembler(RV_G);
    __ lwu(A0, Address(A0, 0));
    __ ret();

    void* buffer = assembler.buffer();
    size_t size = assembler.size();

    Disassembler disassembler(RV_G);
    char* disassembly = disassembler.Disassemble(buffer, size);
    EXPECT_STREQ(
        "  00056503 lwu a0, 0(a0)\n"
        "  00008067 ret\n",
        disassembly);
    free(disassembly);

    uint32_t* values =
        reinterpret_cast<uint32_t*>(Memory::Allocate(3 * sizeof(uint32_t)));
    values[0] = 0xAB010203;
    values[1] = 0xCD020405;
    values[2] = 0xEF030607;

    Simulator simulator;
    EXPECT_EQ(0xCD020405, simulator.Call(buffer, Memory::ToGuest(&values[1])));
  }
  {
    Assembler assembler(RV_G);
    __ lwu(A0, Address(A0, 4));
    __ ret();

    void* buffer = assembler.buffer();
    size_t size = assembler.size();

    Disassembler disassembler(RV_G);
    char* disassembly = disassembler.Disassemble(buffer, size);
    EXPECT_STREQ(
        "  00456503 lwu a0, 4(a0)\n"
        "  00008067 ret\n",
        disassembly);
    free(disassembly);

    uint32_t* values =
        reinterpret_cast<uint32_t*>(Memory::Allocate(3 * sizeof(uint32_t)));
    values[0] = 0xAB010203;
    values[1] = 0xCD020405;
    values[2] = 0xEF030607;

    Simulator simulator;
    EXPECT_EQ(0xEF030607, simulator.Call(buffer, Memory::ToGuest(&values[1])));
  }
  {
    Assembler assembler(RV_G);
    __ lwu(A0, Address(A0, -4));
    __ ret();

    void* buffer = assembler.buffer();
    size_t size = assembler.size();

    Disassembler disassembler(RV_G);
    char* disassembly = disassembler.Disassemble(buffer, size);
    EXPECT_STREQ(
        "  ffc56503 lwu a0, -4(a0)\n"
        "  00008067 ret\n",
        disassembly);
    free(disassembly);

    uint32_t* values =
        reinterpret_cast<uint32_t*>(Memory::Allocate(3 * sizeof(uint32_t)));
    values[0] = 0xAB010203;
    values[1] = 0xCD020405;
    values[2] = 0xEF030607;

    Simulator simulator;
    EXPECT_EQ(0xAB010203, simulator.Call(buffer, Memory::ToGuest(&values[1])));
  }
}

UNIT_TEST(LoadDoubleWord) {
  {
    Assembler assembler(RV_G);
    __ ld(A0, Address(A0, 0));
    __ ret();

    void* buffer = assembler.buffer();
    size_t size = assembler.size();

    Disassembler disassembler(RV_G);
    char* disassembly = disassembler.Disassemble(buffer, size);
    EXPECT_STREQ(
        "  00053503 ld a0, 0(a0)\n"
        "  00008067 ret\n",
        disassembly);
    free(disassembly);

    uint64_t* values =
        reinterpret_cast<uint64_t*>(Memory::Allocate(3 * sizeof(uint64_t)));
    values[0] = 0xAB01020304050607;
    values[1] = 0xCD02040505060708;
    values[2] = 0xEF03060708090A0B;

    Simulator simulator;
    EXPECT_EQ(-3674369926375274744,
              simulator.Call(buffer, Memory::ToGuest(&values[1])));
  }
  {
    Assembler assembler(RV_G);
    __ ld(A0, Address(A0, 8));
    __ ret();

    void* buffer = assembler.buffer();
    size_t size = assembler.size();

    Disassembler disassembler(RV_G);
    char* disassembly = disassembler.Disassemble(buffer, size);
    EXPECT_STREQ(
        "  00853503 ld a0, 8(a0)\n"
        "  00008067 ret\n",
        disassembly);
    free(disassembly);

    uint64_t* values =
        reinterpret_cast<uint64_t*>(Memory::Allocate(3 * sizeof(uint64_t)));
    values[0] = 0xAB01020304050607;
    values[1] = 0xCD02040505060708;
    values[2] = 0xEF03060708090A0B;

    Simulator simulator;
    EXPECT_EQ(-1224128046445295093,
              simulator.Call(buffer, Memory::ToGuest(&values[1])));
  }
  {
    Assembler assembler(RV_G);
    __ ld(A0, Address(A0, -8));
    __ ret();

    void* buffer = assembler.buffer();
    size_t size = assembler.size();

    Disassembler disassembler(RV_G);
    char* disassembly = disassembler.Disassemble(buffer, size);
    EXPECT_STREQ(
        "  ff853503 ld a0, -8(a0)\n"
        "  00008067 ret\n",
        disassembly);
    free(disassembly);

    uint64_t* values =
        reinterpret_cast<uint64_t*>(Memory::Allocate(3 * sizeof(uint64_t)));
    values[0] = 0xAB01020304050607;
    values[1] = 0xCD02040505060708;
    values[2] = 0xEF03060708090A0B;

    Simulator simulator;
    EXPECT_EQ(-6124611806271568377,
              simulator.Call(buffer, Memory::ToGuest(&values[1])));
  }
}

UNIT_TEST(StoreDoubleWord) {
  {
    Assembler assembler(RV_G);
    __ sd(A1, Address(A0, 0));
    __ ret();

    void* buffer = assembler.buffer();
    size_t size = assembler.size();

    Disassembler disassembler(RV_G);
    char* disassembly = disassembler.Disassemble(buffer, size);
    EXPECT_STREQ(
        "  00b53023 sd a1, 0(a0)\n"
        "  00008067 ret\n",
        disassembly);
    free(disassembly);

    uint64_t* values =
        reinterpret_cast<uint64_t*>(Memory::Allocate(3 * sizeof(uint64_t)));
    values[0] = 0;
    values[1] = 0;
    values[2] = 0;

    Simulator simulator;
    simulator.Call(buffer, Memory::ToGuest(&values[1]), 0xCD02040505060708);
    EXPECT_EQ(0u, values[0]);
    EXPECT_EQ(0xCD02040505060708, values[1]);
    EXPECT_EQ(0u, values[2]);
  }
  {
    Assembler assembler(RV_G);
    __ sd(A1, Address(A0, 8));
    __ ret();

    void* buffer = assembler.buffer();
    size_t size = assembler.size();

    Disassembler disassembler(RV_G);
    char* disassembly = disassembler.Disassemble(buffer, size);
    EXPECT_STREQ(
        "  00b53423 sd a1, 8(a0)\n"
        "  00008067 ret\n",
        disassembly);
    free(disassembly);

    uint64_t* values =
        reinterpret_cast<uint64_t*>(Memory::Allocate(3 * sizeof(uint64_t)));
    values[0] = 0;
    values[1] = 0;
    values[2] = 0;

    Simulator simulator;
    simulator.Call(buffer, Memory::ToGuest(&values[1]), 0xEF03060708090A0B);
    EXPECT_EQ(0u, values[0]);
    EXPECT_EQ(0u, values[1]);
    EXPECT_EQ(0xEF03060708090A0B, values[2]);
  }
  {
    Assembler assembler(RV_G);
    __ sd(A1, Address(A0, -8));
    __ ret();

    void* buffer = assembler.buffer();
    size_t size = assembler.size();

    Disassembler disassembler(RV_G);
    char* disassembly = disassembler.Disassemble(buffer, size);
    EXPECT_STREQ(
        "  feb53c23 sd a1, -8(a0)\n"
        "  00008067 ret\n",
        disassembly);
    free(disassembly);

    uint64_t* values =
        reinterpret_cast<uint64_t*>(Memory::Allocate(3 * sizeof(uint64_t)));
    values[0] = 0;
    values[1] = 0;
    values[2] = 0;

    Simulator simulator;
    simulator.Call(buffer, Memory::ToGuest(&values[1]), 0xAB01020304050607);
    EXPECT_EQ(0xAB01020304050607, values[0]);
    EXPECT_EQ(0u, values[1]);
    EXPECT_EQ(0u, values[2]);
  }
}
#endif

UNIT_TEST(AddImmediate1) {
  Assembler assembler(RV_G);
  __ addi(A0, A0, 42);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  02a50513 addi a0, a0, 42\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(42, simulator.Call(buffer, 0));
  EXPECT_EQ(40, simulator.Call(buffer, -2));
  EXPECT_EQ(0, simulator.Call(buffer, -42));
}

UNIT_TEST(AddImmediate2) {
  Assembler assembler(RV_G);
  __ addi(A0, A0, -42);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  fd650513 addi a0, a0, -42\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(-42, simulator.Call(buffer, 0));
  EXPECT_EQ(-44, simulator.Call(buffer, -2));
  EXPECT_EQ(38, simulator.Call(buffer, 80));
}

UNIT_TEST(SetLessThanImmediate1) {
  Assembler assembler(RV_G);
  __ slti(A0, A0, 7);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  00752513 slti a0, a0, 7\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(1, simulator.Call(buffer, 6));
  EXPECT_EQ(0, simulator.Call(buffer, 7));
  EXPECT_EQ(0, simulator.Call(buffer, 8));
  EXPECT_EQ(1, simulator.Call(buffer, -6));
  EXPECT_EQ(1, simulator.Call(buffer, -7));
  EXPECT_EQ(1, simulator.Call(buffer, -8));
}

UNIT_TEST(SetLessThanImmediate2) {
  Assembler assembler(RV_G);
  __ slti(A0, A0, -7);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  ff952513 slti a0, a0, -7\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0, simulator.Call(buffer, 6));
  EXPECT_EQ(0, simulator.Call(buffer, 7));
  EXPECT_EQ(0, simulator.Call(buffer, 8));
  EXPECT_EQ(0, simulator.Call(buffer, -6));
  EXPECT_EQ(0, simulator.Call(buffer, -7));
  EXPECT_EQ(1, simulator.Call(buffer, -8));
}

UNIT_TEST(SetLessThanImmediateUnsigned1) {
  Assembler assembler(RV_G);
  __ sltiu(A0, A0, 7);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  00753513 sltiu a0, a0, 7\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(1, simulator.Call(buffer, 6));
  EXPECT_EQ(0, simulator.Call(buffer, 7));
  EXPECT_EQ(0, simulator.Call(buffer, 8));
  EXPECT_EQ(0, simulator.Call(buffer, -6));
  EXPECT_EQ(0, simulator.Call(buffer, -7));
  EXPECT_EQ(0, simulator.Call(buffer, -8));
}

UNIT_TEST(SetLessThanImmediateUnsigned2) {
  Assembler assembler(RV_G);
  __ sltiu(A0, A0, -7);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  ff953513 sltiu a0, a0, -7\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(1, simulator.Call(buffer, 6));
  EXPECT_EQ(1, simulator.Call(buffer, 7));
  EXPECT_EQ(1, simulator.Call(buffer, 8));
  EXPECT_EQ(0, simulator.Call(buffer, -6));
  EXPECT_EQ(0, simulator.Call(buffer, -7));
  EXPECT_EQ(1, simulator.Call(buffer, -8));
}

UNIT_TEST(XorImmediate1) {
  Assembler assembler(RV_G);
  __ xori(A0, A0, 42);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  02a54513 xori a0, a0, 42\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(42, simulator.Call(buffer, 0));
  EXPECT_EQ(43, simulator.Call(buffer, 1));
  EXPECT_EQ(32, simulator.Call(buffer, 10));
  EXPECT_EQ(-43, simulator.Call(buffer, -1));
  EXPECT_EQ(-36, simulator.Call(buffer, -10));
}

UNIT_TEST(XorImmediate2) {
  Assembler assembler(RV_G);
  __ xori(A0, A0, -42);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  fd654513 xori a0, a0, -42\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(-42, simulator.Call(buffer, 0));
  EXPECT_EQ(-41, simulator.Call(buffer, 1));
  EXPECT_EQ(-36, simulator.Call(buffer, 10));
  EXPECT_EQ(41, simulator.Call(buffer, -1));
  EXPECT_EQ(32, simulator.Call(buffer, -10));
}

UNIT_TEST(OrImmediate1) {
  Assembler assembler(RV_G);
  __ ori(A0, A0, -6);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  ffa56513 ori a0, a0, -6\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(-6, simulator.Call(buffer, 0));
  EXPECT_EQ(-5, simulator.Call(buffer, 1));
  EXPECT_EQ(-5, simulator.Call(buffer, 11));
  EXPECT_EQ(-1, simulator.Call(buffer, -1));
  EXPECT_EQ(-1, simulator.Call(buffer, -11));
}

UNIT_TEST(OrImmediate2) {
  Assembler assembler(RV_G);
  __ ori(A0, A0, 6);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  00656513 ori a0, a0, 6\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(6, simulator.Call(buffer, 0));
  EXPECT_EQ(7, simulator.Call(buffer, 1));
  EXPECT_EQ(15, simulator.Call(buffer, 11));
  EXPECT_EQ(-1, simulator.Call(buffer, -1));
  EXPECT_EQ(-9, simulator.Call(buffer, -11));
}

UNIT_TEST(AndImmediate1) {
  Assembler assembler(RV_G);
  __ andi(A0, A0, -6);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  ffa57513 andi a0, a0, -6\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0, simulator.Call(buffer, 0));
  EXPECT_EQ(0, simulator.Call(buffer, 1));
  EXPECT_EQ(10, simulator.Call(buffer, 11));
  EXPECT_EQ(-6, simulator.Call(buffer, -1));
  EXPECT_EQ(-16, simulator.Call(buffer, -11));
}

UNIT_TEST(AndImmediate2) {
  Assembler assembler(RV_G);
  __ andi(A0, A0, 6);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  00657513 andi a0, a0, 6\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0, simulator.Call(buffer, 0));
  EXPECT_EQ(0, simulator.Call(buffer, 1));
  EXPECT_EQ(2, simulator.Call(buffer, 11));
  EXPECT_EQ(6, simulator.Call(buffer, -1));
  EXPECT_EQ(4, simulator.Call(buffer, -11));
}

UNIT_TEST(ShiftLeftLogicalImmediate) {
  Assembler assembler(RV_G);
  __ slli(A0, A0, 2);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  00251513 slli a0, a0, 0x2\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(84, simulator.Call(buffer, 21));
  EXPECT_EQ(4, simulator.Call(buffer, 1));
  EXPECT_EQ(0, simulator.Call(buffer, 0));
  EXPECT_EQ(-4, simulator.Call(buffer, -1));
  EXPECT_EQ(-84, simulator.Call(buffer, -21));
}

UNIT_TEST(ShiftLeftLogicalImmediate2) {
  Assembler assembler(RV_G);
  __ slli(A0, A0, XLEN - 1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
#if XLEN == 32
  EXPECT_STREQ(
      "  01f51513 slli a0, a0, 0x1f\n"  ///CHECK
      "  00008067 ret\n",
      disassembly);
#elif XLEN == 64
  EXPECT_STREQ(
      "  03f51513 slli a0, a0, 0x3f\n"   ///CHECK
      "  00008067 ret\n",
      disassembly);
#endif
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0, simulator.Call(buffer, 2));
  EXPECT_EQ(kMinIntX, simulator.Call(buffer, 1));
  EXPECT_EQ(0, simulator.Call(buffer, 0));
  EXPECT_EQ(kMinIntX, simulator.Call(buffer, -1));
  EXPECT_EQ(0, simulator.Call(buffer, -2));
}

UNIT_TEST(ShiftRightLogicalImmediate) {
  Assembler assembler(RV_G);
  __ srli(A0, A0, 2);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  00255513 srli a0, a0, 0x2\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(5, simulator.Call(buffer, 21));
  EXPECT_EQ(0, simulator.Call(buffer, 1));
  EXPECT_EQ(0, simulator.Call(buffer, 0));
  EXPECT_EQ(static_cast<intx_t>(static_cast<uintx_t>(-1) >> 2),
            simulator.Call(buffer, -1));
  EXPECT_EQ(static_cast<intx_t>(static_cast<uintx_t>(-21) >> 2),
            simulator.Call(buffer, -21));
}

UNIT_TEST(ShiftRightLogicalImmediate2) {
  Assembler assembler(RV_G);
  __ srli(A0, A0, XLEN - 1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
#if XLEN == 32
  EXPECT_STREQ(
      "  01f55513 srli a0, a0, 0x1f\n"  //CHECK
      "  00008067 ret\n",
      disassembly);
#elif XLEN == 64
  EXPECT_STREQ(
      "  03f55513 srli a0, a0, 0x3f\n" //CHECK
      "  00008067 ret\n",
      disassembly);
#endif
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0, simulator.Call(buffer, 21));
  EXPECT_EQ(0, simulator.Call(buffer, 1));
  EXPECT_EQ(0, simulator.Call(buffer, 0));
  EXPECT_EQ(1, simulator.Call(buffer, -1));
  EXPECT_EQ(1, simulator.Call(buffer, -21));
}

UNIT_TEST(ShiftRightArithmeticImmediate) {
  Assembler assembler(RV_G);
  __ srai(A0, A0, 2);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  40255513 srai a0, a0, 0x2\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(5, simulator.Call(buffer, 21));
  EXPECT_EQ(0, simulator.Call(buffer, 1));
  EXPECT_EQ(0, simulator.Call(buffer, 0));
  EXPECT_EQ(-1, simulator.Call(buffer, -1));
  EXPECT_EQ(-6, simulator.Call(buffer, -21));
}

UNIT_TEST(ShiftRightArithmeticImmediate2) {
  Assembler assembler(RV_G);
  __ srai(A0, A0, XLEN - 1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
#if XLEN == 32
  EXPECT_STREQ(
      "  41f55513 srai a0, a0, 0x1f\n"  // CHECK
      "  00008067 ret\n",
      disassembly);
#elif XLEN == 64
  EXPECT_STREQ(
      "  43f55513 srai a0, a0, 0x3f\n"  // CHECK
      "  00008067 ret\n",
      disassembly);
#endif
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0, simulator.Call(buffer, 21));
  EXPECT_EQ(0, simulator.Call(buffer, 1));
  EXPECT_EQ(0, simulator.Call(buffer, 0));
  EXPECT_EQ(-1, simulator.Call(buffer, -1));
  EXPECT_EQ(-1, simulator.Call(buffer, -21));
}

UNIT_TEST(Add) {
  Assembler assembler(RV_G);
  __ add(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  00b50533 add a0, a0, a1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(24, simulator.Call(buffer, 7, 17));
  EXPECT_EQ(-10, simulator.Call(buffer, 7, -17));
  EXPECT_EQ(10, simulator.Call(buffer, -7, 17));
  EXPECT_EQ(-24, simulator.Call(buffer, -7, -17));
  EXPECT_EQ(24, simulator.Call(buffer, 17, 7));
  EXPECT_EQ(10, simulator.Call(buffer, 17, -7));
  EXPECT_EQ(-10, simulator.Call(buffer, -17, 7));
  EXPECT_EQ(-24, simulator.Call(buffer, -17, -7));
}

UNIT_TEST(Subtract) {
  Assembler assembler(RV_G);
  __ sub(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  40b50533 sub a0, a0, a1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(-10, simulator.Call(buffer, 7, 17));
  EXPECT_EQ(24, simulator.Call(buffer, 7, -17));
  EXPECT_EQ(-24, simulator.Call(buffer, -7, 17));
  EXPECT_EQ(10, simulator.Call(buffer, -7, -17));
  EXPECT_EQ(10, simulator.Call(buffer, 17, 7));
  EXPECT_EQ(24, simulator.Call(buffer, 17, -7));
  EXPECT_EQ(-24, simulator.Call(buffer, -17, 7));
  EXPECT_EQ(-10, simulator.Call(buffer, -17, -7));
}

UNIT_TEST(ShiftLeftLogical) {
  Assembler assembler(RV_G);
  __ sll(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  00b51533 sll a0, a0, a1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(2176, simulator.Call(buffer, 17, 7));
  EXPECT_EQ(-2176, simulator.Call(buffer, -17, 7));
  EXPECT_EQ(34, simulator.Call(buffer, 17, 1));
  EXPECT_EQ(-34, simulator.Call(buffer, -17, 1));
  EXPECT_EQ(17, simulator.Call(buffer, 17, 0));
  EXPECT_EQ(-17, simulator.Call(buffer, -17, 0));
}

UNIT_TEST(SetLessThan) {
  Assembler assembler(RV_G);
  __ slt(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  00b52533 slt a0, a0, a1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0, simulator.Call(buffer, 7, 7));
  EXPECT_EQ(0, simulator.Call(buffer, -7, -7));
  EXPECT_EQ(1, simulator.Call(buffer, 7, 17));
  EXPECT_EQ(0, simulator.Call(buffer, 7, -17));
  EXPECT_EQ(1, simulator.Call(buffer, -7, 17));
  EXPECT_EQ(0, simulator.Call(buffer, -7, -17));
  EXPECT_EQ(0, simulator.Call(buffer, 17, 7));
  EXPECT_EQ(0, simulator.Call(buffer, 17, -7));
  EXPECT_EQ(1, simulator.Call(buffer, -17, 7));
  EXPECT_EQ(1, simulator.Call(buffer, -17, -7));
}

UNIT_TEST(SetLessThanUnsigned) {
  Assembler assembler(RV_G);
  __ sltu(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  00b53533 sltu a0, a0, a1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0, simulator.Call(buffer, 7, 7));
  EXPECT_EQ(0, simulator.Call(buffer, -7, -7));
  EXPECT_EQ(1, simulator.Call(buffer, 7, 17));
  EXPECT_EQ(1, simulator.Call(buffer, 7, -17));
  EXPECT_EQ(0, simulator.Call(buffer, -7, 17));
  EXPECT_EQ(0, simulator.Call(buffer, -7, -17));
  EXPECT_EQ(0, simulator.Call(buffer, 17, 7));
  EXPECT_EQ(1, simulator.Call(buffer, 17, -7));
  EXPECT_EQ(0, simulator.Call(buffer, -17, 7));
  EXPECT_EQ(1, simulator.Call(buffer, -17, -7));
}

UNIT_TEST(Xor) {
  Assembler assembler(RV_G);
  __ xor_(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  00b54533 xor a0, a0, a1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(22, simulator.Call(buffer, 7, 17));
  EXPECT_EQ(-24, simulator.Call(buffer, 7, -17));
  EXPECT_EQ(-24, simulator.Call(buffer, -7, 17));
  EXPECT_EQ(22, simulator.Call(buffer, -7, -17));
  EXPECT_EQ(22, simulator.Call(buffer, 17, 7));
  EXPECT_EQ(-24, simulator.Call(buffer, 17, -7));
  EXPECT_EQ(-24, simulator.Call(buffer, -17, 7));
  EXPECT_EQ(22, simulator.Call(buffer, -17, -7));
}

UNIT_TEST(ShiftRightLogical) {
  Assembler assembler(RV_G);
  __ srl(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  00b55533 srl a0, a0, a1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0, simulator.Call(buffer, 17, 7));
  EXPECT_EQ(static_cast<intx_t>(static_cast<uintx_t>(-17) >> 7),
            simulator.Call(buffer, -17, 7));
  EXPECT_EQ(8, simulator.Call(buffer, 17, 1));
  EXPECT_EQ(static_cast<intx_t>(static_cast<uintx_t>(-17) >> 1),
            simulator.Call(buffer, -17, 1));
  EXPECT_EQ(17, simulator.Call(buffer, 17, 0));
  EXPECT_EQ(-17, simulator.Call(buffer, -17, 0));
}

UNIT_TEST(ShiftRightArithmetic) {
  Assembler assembler(RV_G);
  __ sra(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  40b55533 sra a0, a0, a1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0, simulator.Call(buffer, 17, 7));
  EXPECT_EQ(-1, simulator.Call(buffer, -17, 7));
  EXPECT_EQ(8, simulator.Call(buffer, 17, 1));
  EXPECT_EQ(-9, simulator.Call(buffer, -17, 1));
  EXPECT_EQ(17, simulator.Call(buffer, 17, 0));
  EXPECT_EQ(-17, simulator.Call(buffer, -17, 0));
}

UNIT_TEST(Or) {
  Assembler assembler(RV_G);
  __ or_(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  00b56533 or a0, a0, a1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(23, simulator.Call(buffer, 7, 17));
  EXPECT_EQ(-17, simulator.Call(buffer, 7, -17));
  EXPECT_EQ(-7, simulator.Call(buffer, -7, 17));
  EXPECT_EQ(-1, simulator.Call(buffer, -7, -17));
  EXPECT_EQ(23, simulator.Call(buffer, 17, 7));
  EXPECT_EQ(-7, simulator.Call(buffer, 17, -7));
  EXPECT_EQ(-17, simulator.Call(buffer, -17, 7));
  EXPECT_EQ(-1, simulator.Call(buffer, -17, -7));
}

UNIT_TEST(And) {
  Assembler assembler(RV_G);
  __ and_(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  00b57533 and a0, a0, a1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(1, simulator.Call(buffer, 7, 17));
  EXPECT_EQ(7, simulator.Call(buffer, 7, -17));
  EXPECT_EQ(17, simulator.Call(buffer, -7, 17));
  EXPECT_EQ(-23, simulator.Call(buffer, -7, -17));
  EXPECT_EQ(1, simulator.Call(buffer, 17, 7));
  EXPECT_EQ(17, simulator.Call(buffer, 17, -7));
  EXPECT_EQ(7, simulator.Call(buffer, -17, 7));
  EXPECT_EQ(-23, simulator.Call(buffer, -17, -7));
}

UNIT_TEST(Fence) {
  Assembler assembler(RV_G);
  __ fence();
  __ fence(kRead, kWrite);
  __ fence(kInput, kOutput);
  __ fence(kMemory, kMemory);
  __ fence(kAll, kAll);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  0ff0000f fence\n"
      "  0210000f fence r,w\n"
      "  0840000f fence i,o\n"
      "  0330000f fence rw,rw\n"
      "  0ff0000f fence\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  simulator.Call(buffer);
}

UNIT_TEST(InstructionFence) {
  Assembler assembler(RV_G);
  __ fencei();
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  0000100f fence.i\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  simulator.Call(buffer);
}

UNIT_TEST(EnvironmentCall) {
  Assembler assembler(RV_G);
  __ ecall();
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  00000073 ecall\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  // Not running: would trap.
}

UNIT_TEST(EnvironmentBreak) {
  Assembler assembler(RV_G);
  __ ebreak();
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  00100073 ebreak\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  // Not running: would trap.
}

UNIT_TEST(ControlStatusRegisters) {
  Assembler assembler(RV_G);
  __ csrrw(T0, 0x123, S1);
  __ csrrs(T1, 0x123, S2);
  __ csrrc(T2, 0x123, S3);
  __ csrr(T3, 0x123);
  __ csrw(0x123, S4);
  __ csrs(0x123, S5);
  __ csrc(0x123, S6);
  __ csrrwi(T1, 0x123, 1);
  __ csrrsi(T2, 0x123, 2);
  __ csrrci(T3, 0x123, 3);
  __ csrwi(0x123, 4);
  __ csrsi(0x123, 5);
  __ csrci(0x123, 6);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  123492f3 csrrw t0, 0x123, s1\n"
      "  12392373 csrrs t1, 0x123, s2\n"
      "  1239b3f3 csrrc t2, 0x123, s3\n"
      "  12302e73 csrr t3, 0x123\n"
      "  123a1073 csrw 0x123, s4\n"
      "  123aa073 csrs 0x123, s5\n"
      "  123b3073 csrc 0x123, s6\n"
      "  1230d373 csrrwi t1, 0x123, 1\n"
      "  123163f3 csrrsi t2, 0x123, 2\n"
      "  1231fe73 csrrci t3, 0x123, 3\n"
      "  12325073 csrwi 0x123, 4\n"
      "  1232e073 csrsi 0x123, 5\n"
      "  12337073 csrci 0x123, 6\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  // Not running: would trap.
}

UNIT_TEST(Nop) {
  Assembler assembler(RV_G);
  __ nop();
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  00000013 nop\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(123, simulator.Call(buffer, 123));
}

UNIT_TEST(Move) {
  Assembler assembler(RV_G);
  __ mv(A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  00058513 mv a0, a1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(36, simulator.Call(buffer, 42, 36));
}

UNIT_TEST(Not) {
  Assembler assembler(RV_G);
  __ not_(A0, A0);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  fff54513 not a0, a0\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(~42, simulator.Call(buffer, 42));
  EXPECT_EQ(~-42, simulator.Call(buffer, -42));
}

UNIT_TEST(Negate) {
  Assembler assembler(RV_G);
  __ neg(A0, A0);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  40a00533 neg a0, a0\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(-42, simulator.Call(buffer, 42));
  EXPECT_EQ(42, simulator.Call(buffer, -42));
}

UNIT_TEST(SetNotEqualToZero) {
  Assembler assembler(RV_G);
  __ snez(A0, A0);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  00a03533 snez a0, a0\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(1, simulator.Call(buffer, -42));
  EXPECT_EQ(0, simulator.Call(buffer, 0));
  EXPECT_EQ(1, simulator.Call(buffer, 42));
}

UNIT_TEST(SetEqualToZero) {
  Assembler assembler(RV_G);
  __ seqz(A0, A0);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  00153513 seqz a0, a0\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0, simulator.Call(buffer, -42));
  EXPECT_EQ(1, simulator.Call(buffer, 0));
  EXPECT_EQ(0, simulator.Call(buffer, 42));
}

UNIT_TEST(SetLessThanZero) {
  Assembler assembler(RV_G);
  __ sltz(A0, A0);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  00052533 sltz a0, a0\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(1, simulator.Call(buffer, -42));
  EXPECT_EQ(0, simulator.Call(buffer, 0));
  EXPECT_EQ(0, simulator.Call(buffer, 42));
}

UNIT_TEST(SetGreaterThanZero) {
  Assembler assembler(RV_G);
  __ sgtz(A0, A0);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  00a02533 sgtz a0, a0\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0, simulator.Call(buffer, -42));
  EXPECT_EQ(0, simulator.Call(buffer, 0));
  EXPECT_EQ(1, simulator.Call(buffer, 42));
}

UNIT_TEST(BranchEqualZero) {
  Assembler assembler(RV_G);
  Label label;
  __ beqz(A0, &label);
  __ li(A0, 3);
  __ ret();
  __ Bind(&label);
  __ li(A0, 4);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  00050663 beqz a0, +12\n"
      "  00300513 li a0, 3\n"
      "  00008067 ret\n"
      "  00400513 li a0, 4\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(3, simulator.Call(buffer, -42));
  EXPECT_EQ(4, simulator.Call(buffer, 0));
  EXPECT_EQ(3, simulator.Call(buffer, 42));
}

UNIT_TEST(BranchNotEqualZero) {
  Assembler assembler(RV_G);
  Label label;
  __ bnez(A0, &label);
  __ li(A0, 3);
  __ ret();
  __ Bind(&label);
  __ li(A0, 4);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  00051663 bnez a0, +12\n"
      "  00300513 li a0, 3\n"
      "  00008067 ret\n"
      "  00400513 li a0, 4\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(4, simulator.Call(buffer, -42));
  EXPECT_EQ(3, simulator.Call(buffer, 0));
  EXPECT_EQ(4, simulator.Call(buffer, 42));
}

UNIT_TEST(BranchLessOrEqualZero) {
  Assembler assembler(RV_G);
  Label label;
  __ blez(A0, &label);
  __ li(A0, 3);
  __ ret();
  __ Bind(&label);
  __ li(A0, 4);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  00a05663 blez a0, +12\n"
      "  00300513 li a0, 3\n"
      "  00008067 ret\n"
      "  00400513 li a0, 4\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(4, simulator.Call(buffer, -42));
  EXPECT_EQ(4, simulator.Call(buffer, 0));
  EXPECT_EQ(3, simulator.Call(buffer, 42));
}

UNIT_TEST(BranchGreaterOrEqualZero) {
  Assembler assembler(RV_G);
  Label label;
  __ bgez(A0, &label);
  __ li(A0, 3);
  __ ret();
  __ Bind(&label);
  __ li(A0, 4);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  00055663 bgez a0, +12\n"
      "  00300513 li a0, 3\n"
      "  00008067 ret\n"
      "  00400513 li a0, 4\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(3, simulator.Call(buffer, -42));
  EXPECT_EQ(4, simulator.Call(buffer, 0));
  EXPECT_EQ(4, simulator.Call(buffer, 42));
}

UNIT_TEST(BranchLessThanZero) {
  Assembler assembler(RV_G);
  Label label;
  __ bltz(A0, &label);
  __ li(A0, 3);
  __ ret();
  __ Bind(&label);
  __ li(A0, 4);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  00054663 bltz a0, +12\n"
      "  00300513 li a0, 3\n"
      "  00008067 ret\n"
      "  00400513 li a0, 4\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(4, simulator.Call(buffer, -42));
  EXPECT_EQ(3, simulator.Call(buffer, 0));
  EXPECT_EQ(3, simulator.Call(buffer, 42));
}

UNIT_TEST(BranchGreaterThanZero) {
  Assembler assembler(RV_G);
  Label label;
  __ bgtz(A0, &label);
  __ li(A0, 3);
  __ ret();
  __ Bind(&label);
  __ li(A0, 4);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  00a04663 bgtz a0, +12\n"
      "  00300513 li a0, 3\n"
      "  00008067 ret\n"
      "  00400513 li a0, 4\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(3, simulator.Call(buffer, -42));
  EXPECT_EQ(3, simulator.Call(buffer, 0));
  EXPECT_EQ(4, simulator.Call(buffer, 42));
}

#if XLEN >= 64
UNIT_TEST(AddImmediateWord1) {
  Assembler assembler(RV_G);
  __ addiw(A0, A0, 42);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  02a5051b addiw a0, a0, 42\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(42, simulator.Call(buffer, 0));
  EXPECT_EQ(40, simulator.Call(buffer, -2));
  EXPECT_EQ(0, simulator.Call(buffer, -42));
}

UNIT_TEST(AddImmediateWord2) {
  Assembler assembler(RV_G);
  __ addiw(A0, A0, -42);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  fd65051b addiw a0, a0, -42\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(-42, simulator.Call(buffer, 0));
  EXPECT_EQ(-44, simulator.Call(buffer, -2));
  EXPECT_EQ(38, simulator.Call(buffer, 80));
}

UNIT_TEST(ShiftLeftLogicalImmediateWord) {
  Assembler assembler(RV_G);
  __ slliw(A0, A0, 2);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  0025151b slliw a0, a0, 0x2\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(84, simulator.Call(buffer, 21));
  EXPECT_EQ(4, simulator.Call(buffer, 1));
  EXPECT_EQ(0, simulator.Call(buffer, 0));
  EXPECT_EQ(-4, simulator.Call(buffer, -1));
  EXPECT_EQ(-84, simulator.Call(buffer, -21));
}

UNIT_TEST(ShiftRightLogicalImmediateWord) {
  Assembler assembler(RV_G);
  __ srliw(A0, A0, 2);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  0025551b srliw a0, a0, 0x2\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(5, simulator.Call(buffer, 21));
  EXPECT_EQ(0, simulator.Call(buffer, 1));
  EXPECT_EQ(0, simulator.Call(buffer, 0));
  EXPECT_EQ(sign_extend(static_cast<uint32_t>(-1) >> 2),
            simulator.Call(buffer, -1));
  EXPECT_EQ(sign_extend(static_cast<uint32_t>(-21) >> 2),
            simulator.Call(buffer, -21));
}

UNIT_TEST(ShiftRightArithmeticImmediateWord) {
  Assembler assembler(RV_G);
  __ sraiw(A0, A0, 2);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  4025551b sraiw a0, a0, 0x2\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(5, simulator.Call(buffer, 21));
  EXPECT_EQ(0, simulator.Call(buffer, 1));
  EXPECT_EQ(0, simulator.Call(buffer, 0));
  EXPECT_EQ(-1, simulator.Call(buffer, -1));
  EXPECT_EQ(-6, simulator.Call(buffer, -21));
}

UNIT_TEST(AddWord) {
  Assembler assembler(RV_G);
  __ addw(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  00b5053b addw a0, a0, a1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(24, simulator.Call(buffer, 7, 17));
  EXPECT_EQ(-10, simulator.Call(buffer, 7, -17));
  EXPECT_EQ(10, simulator.Call(buffer, -7, 17));
  EXPECT_EQ(-24, simulator.Call(buffer, -7, -17));
  EXPECT_EQ(24, simulator.Call(buffer, 17, 7));
  EXPECT_EQ(10, simulator.Call(buffer, 17, -7));
  EXPECT_EQ(-10, simulator.Call(buffer, -17, 7));
  EXPECT_EQ(-24, simulator.Call(buffer, -17, -7));
  EXPECT_EQ(3, simulator.Call(buffer, 0x200000002, 0x100000001));
}

UNIT_TEST(SubtractWord) {
  Assembler assembler(RV_G);
  __ subw(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  40b5053b subw a0, a0, a1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(-10, simulator.Call(buffer, 7, 17));
  EXPECT_EQ(24, simulator.Call(buffer, 7, -17));
  EXPECT_EQ(-24, simulator.Call(buffer, -7, 17));
  EXPECT_EQ(10, simulator.Call(buffer, -7, -17));
  EXPECT_EQ(10, simulator.Call(buffer, 17, 7));
  EXPECT_EQ(24, simulator.Call(buffer, 17, -7));
  EXPECT_EQ(-24, simulator.Call(buffer, -17, 7));
  EXPECT_EQ(-10, simulator.Call(buffer, -17, -7));
  EXPECT_EQ(1, simulator.Call(buffer, 0x200000002, 0x100000001));
}

UNIT_TEST(ShiftLeftLogicalWord) {
  Assembler assembler(RV_G);
  __ sllw(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  00b5153b sllw a0, a0, a1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(2176, simulator.Call(buffer, 17, 7));
  EXPECT_EQ(-2176, simulator.Call(buffer, -17, 7));
  EXPECT_EQ(34, simulator.Call(buffer, 17, 1));
  EXPECT_EQ(-34, simulator.Call(buffer, -17, 1));
  EXPECT_EQ(17, simulator.Call(buffer, 17, 0));
  EXPECT_EQ(-17, simulator.Call(buffer, -17, 0));
  EXPECT_EQ(0x10, simulator.Call(buffer, 0x10000001, 4));
}

UNIT_TEST(ShiftRightLogicalWord) {
  Assembler assembler(RV_G);
  __ srlw(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  00b5553b srlw a0, a0, a1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0, simulator.Call(buffer, 17, 7));
  EXPECT_EQ(sign_extend(static_cast<uint32_t>(-17) >> 7),
            simulator.Call(buffer, -17, 7));
  EXPECT_EQ(8, simulator.Call(buffer, 17, 1));
  EXPECT_EQ(sign_extend(static_cast<uint32_t>(-17) >> 1),
            simulator.Call(buffer, -17, 1));
  EXPECT_EQ(17, simulator.Call(buffer, 17, 0));
  EXPECT_EQ(-17, simulator.Call(buffer, -17, 0));
}

UNIT_TEST(ShiftRightArithmeticWord) {
  Assembler assembler(RV_G);
  __ sraw(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  40b5553b sraw a0, a0, a1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0, simulator.Call(buffer, 17, 7));
  EXPECT_EQ(-1, simulator.Call(buffer, -17, 7));
  EXPECT_EQ(8, simulator.Call(buffer, 17, 1));
  EXPECT_EQ(-9, simulator.Call(buffer, -17, 1));
  EXPECT_EQ(17, simulator.Call(buffer, 17, 0));
  EXPECT_EQ(-17, simulator.Call(buffer, -17, 0));
}

UNIT_TEST(NegateWord) {
  Assembler assembler(RV_G);
  __ negw(A0, A0);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  40a0053b negw a0, a0\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0, simulator.Call(buffer, 0));
  EXPECT_EQ(-42, simulator.Call(buffer, 42));
  EXPECT_EQ(42, simulator.Call(buffer, -42));
  EXPECT_EQ(1, simulator.Call(buffer, 0x10FFFFFFFF));
}

UNIT_TEST(SignExtendWord) {
  Assembler assembler(RV_G);
  __ sextw(A0, A0);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  0005051b sext.w a0, a0\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0, simulator.Call(buffer, 0));
  EXPECT_EQ(42, simulator.Call(buffer, 42));
  EXPECT_EQ(-42, simulator.Call(buffer, -42));
  EXPECT_EQ(-1, simulator.Call(buffer, 0x10FFFFFFFF));
}
#endif  // XLEN >= 64

UNIT_TEST(Multiply) {
  Assembler assembler(RV_G);
  __ mul(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  02b50533 mul a0, a0, a1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(68, simulator.Call(buffer, 4, 17));
  EXPECT_EQ(-68, simulator.Call(buffer, -4, 17));
  EXPECT_EQ(-68, simulator.Call(buffer, 4, -17));
  EXPECT_EQ(68, simulator.Call(buffer, -4, -17));
  EXPECT_EQ(68, simulator.Call(buffer, 17, 4));
  EXPECT_EQ(-68, simulator.Call(buffer, -17, 4));
  EXPECT_EQ(-68, simulator.Call(buffer, 17, -4));
  EXPECT_EQ(68, simulator.Call(buffer, -17, -4));
}

UNIT_TEST(MultiplyHigh) {
  Assembler assembler(RV_G);
  __ mulh(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  02b51533 mulh a0, a0, a1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0, simulator.Call(buffer, 4, 17));
  EXPECT_EQ(-1, simulator.Call(buffer, -4, 17));
  EXPECT_EQ(-1, simulator.Call(buffer, 4, -17));
  EXPECT_EQ(0, simulator.Call(buffer, -4, -17));
  EXPECT_EQ(0, simulator.Call(buffer, 17, 4));
  EXPECT_EQ(-1, simulator.Call(buffer, -17, 4));
  EXPECT_EQ(-1, simulator.Call(buffer, 17, -4));
  EXPECT_EQ(0, simulator.Call(buffer, -17, -4));
}

UNIT_TEST(MultiplyHighSignedUnsigned) {
  Assembler assembler(RV_G);
  __ mulhsu(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  02b52533 mulhsu a0, a0, a1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0, simulator.Call(buffer, 4, 17));
  EXPECT_EQ(-1, simulator.Call(buffer, -4, 17));
  EXPECT_EQ(3, simulator.Call(buffer, 4, -17));
  EXPECT_EQ(-4, simulator.Call(buffer, -4, -17));
  EXPECT_EQ(0, simulator.Call(buffer, 17, 4));
  EXPECT_EQ(-1, simulator.Call(buffer, -17, 4));
  EXPECT_EQ(16, simulator.Call(buffer, 17, -4));
  EXPECT_EQ(-17, simulator.Call(buffer, -17, -4));
}

UNIT_TEST(MultiplyHighUnsigned) {
  Assembler assembler(RV_G);
  __ mulhu(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  02b53533 mulhu a0, a0, a1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0, simulator.Call(buffer, 4, 17));
  EXPECT_EQ(16, simulator.Call(buffer, -4, 17));
  EXPECT_EQ(3, simulator.Call(buffer, 4, -17));
  EXPECT_EQ(-21, simulator.Call(buffer, -4, -17));
  EXPECT_EQ(0, simulator.Call(buffer, 17, 4));
  EXPECT_EQ(3, simulator.Call(buffer, -17, 4));
  EXPECT_EQ(16, simulator.Call(buffer, 17, -4));
  EXPECT_EQ(-21, simulator.Call(buffer, -17, -4));
}

UNIT_TEST(Divide) {
  Assembler assembler(RV_G);
  __ div(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  02b54533 div a0, a0, a1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0, simulator.Call(buffer, 4, 17));
  EXPECT_EQ(0, simulator.Call(buffer, -4, 17));
  EXPECT_EQ(0, simulator.Call(buffer, 4, -17));
  EXPECT_EQ(0, simulator.Call(buffer, -4, -17));
  EXPECT_EQ(4, simulator.Call(buffer, 17, 4));
  EXPECT_EQ(-4, simulator.Call(buffer, -17, 4));
  EXPECT_EQ(-4, simulator.Call(buffer, 17, -4));
  EXPECT_EQ(4, simulator.Call(buffer, -17, -4));
}

UNIT_TEST(DivideUnsigned) {
  Assembler assembler(RV_G);
  __ divu(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  02b55533 divu a0, a0, a1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0, simulator.Call(buffer, 4, 17));
#if XLEN == 32
  EXPECT_EQ(252645134, simulator.Call(buffer, -4, 17));
#else
  EXPECT_EQ(1085102592571150094, simulator.Call(buffer, -4, 17));
#endif
  EXPECT_EQ(0, simulator.Call(buffer, 4, -17));
  EXPECT_EQ(1, simulator.Call(buffer, -4, -17));
  EXPECT_EQ(4, simulator.Call(buffer, 17, 4));
#if XLEN == 32
  EXPECT_EQ(1073741819, simulator.Call(buffer, -17, 4));
#else
  EXPECT_EQ(4611686018427387899, simulator.Call(buffer, -17, 4));
#endif
  EXPECT_EQ(0, simulator.Call(buffer, 17, -4));
  EXPECT_EQ(0, simulator.Call(buffer, -17, -4));
}

UNIT_TEST(Remainder) {
  Assembler assembler(RV_G);
  __ rem(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  02b56533 rem a0, a0, a1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(4, simulator.Call(buffer, 4, 17));
  EXPECT_EQ(-4, simulator.Call(buffer, -4, 17));
  EXPECT_EQ(4, simulator.Call(buffer, 4, -17));
  EXPECT_EQ(-4, simulator.Call(buffer, -4, -17));
  EXPECT_EQ(1, simulator.Call(buffer, 17, 4));
  EXPECT_EQ(-1, simulator.Call(buffer, -17, 4));
  EXPECT_EQ(1, simulator.Call(buffer, 17, -4));
  EXPECT_EQ(-1, simulator.Call(buffer, -17, -4));
}

UNIT_TEST(RemainderUnsigned) {
  Assembler assembler(RV_G);
  __ remu(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  02b57533 remu a0, a0, a1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(4, simulator.Call(buffer, 4, 17));
  EXPECT_EQ(14, simulator.Call(buffer, -4, 17));
  EXPECT_EQ(4, simulator.Call(buffer, 4, -17));
  EXPECT_EQ(13, simulator.Call(buffer, -4, -17));
  EXPECT_EQ(1, simulator.Call(buffer, 17, 4));
  EXPECT_EQ(3, simulator.Call(buffer, -17, 4));
  EXPECT_EQ(17, simulator.Call(buffer, 17, -4));
  EXPECT_EQ(-17, simulator.Call(buffer, -17, -4));
}

#if XLEN >= 64
UNIT_TEST(MultiplyWord) {
  Assembler assembler(RV_G);
  __ mulw(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  02b5053b mulw a0, a0, a1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(68, simulator.Call(buffer, 4, 17));
  EXPECT_EQ(-68, simulator.Call(buffer, -4, 17));
  EXPECT_EQ(-68, simulator.Call(buffer, 4, -17));
  EXPECT_EQ(68, simulator.Call(buffer, -4, -17));
  EXPECT_EQ(68, simulator.Call(buffer, 17, 4));
  EXPECT_EQ(-68, simulator.Call(buffer, -17, 4));
  EXPECT_EQ(-68, simulator.Call(buffer, 17, -4));
  EXPECT_EQ(68, simulator.Call(buffer, -17, -4));
}

UNIT_TEST(DivideWord) {
  Assembler assembler(RV_G);
  __ divw(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  02b5453b divw a0, a0, a1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0, simulator.Call(buffer, 4, 17));
  EXPECT_EQ(0, simulator.Call(buffer, -4, 17));
  EXPECT_EQ(0, simulator.Call(buffer, 4, -17));
  EXPECT_EQ(0, simulator.Call(buffer, -4, -17));
  EXPECT_EQ(4, simulator.Call(buffer, 17, 4));
  EXPECT_EQ(-4, simulator.Call(buffer, -17, 4));
  EXPECT_EQ(-4, simulator.Call(buffer, 17, -4));
  EXPECT_EQ(4, simulator.Call(buffer, -17, -4));
}

UNIT_TEST(DivideUnsignedWord) {
  Assembler assembler(RV_G);
  __ divuw(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  02b5553b divuw a0, a0, a1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0, simulator.Call(buffer, 4, 17));
  EXPECT_EQ(252645134, simulator.Call(buffer, -4, 17));
  EXPECT_EQ(0, simulator.Call(buffer, 4, -17));
  EXPECT_EQ(1, simulator.Call(buffer, -4, -17));
  EXPECT_EQ(4, simulator.Call(buffer, 17, 4));
  EXPECT_EQ(1073741819, simulator.Call(buffer, -17, 4));
  EXPECT_EQ(0, simulator.Call(buffer, 17, -4));
  EXPECT_EQ(0, simulator.Call(buffer, -17, -4));
}

UNIT_TEST(RemainderWord) {
  Assembler assembler(RV_G);
  __ remw(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  02b5653b remw a0, a0, a1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(4, simulator.Call(buffer, 4, 17));
  EXPECT_EQ(-4, simulator.Call(buffer, -4, 17));
  EXPECT_EQ(4, simulator.Call(buffer, 4, -17));
  EXPECT_EQ(-4, simulator.Call(buffer, -4, -17));
  EXPECT_EQ(1, simulator.Call(buffer, 17, 4));
  EXPECT_EQ(-1, simulator.Call(buffer, -17, 4));
  EXPECT_EQ(1, simulator.Call(buffer, 17, -4));
  EXPECT_EQ(-1, simulator.Call(buffer, -17, -4));
}

UNIT_TEST(RemainderUnsignedWord) {
  Assembler assembler(RV_G);
  __ remuw(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  02b5753b remuw a0, a0, a1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(4, simulator.Call(buffer, 4, 17));
  EXPECT_EQ(14, simulator.Call(buffer, -4, 17));
  EXPECT_EQ(4, simulator.Call(buffer, 4, -17));
  EXPECT_EQ(13, simulator.Call(buffer, -4, -17));
  EXPECT_EQ(1, simulator.Call(buffer, 17, 4));
  EXPECT_EQ(3, simulator.Call(buffer, -17, 4));
  EXPECT_EQ(17, simulator.Call(buffer, 17, -4));
  EXPECT_EQ(-17, simulator.Call(buffer, -17, -4));
}
#endif

UNIT_TEST(LoadReserveStoreConditionalWord_Success) {
  Assembler assembler(RV_G);
  __ lrw(T0, Address(A0));
  __ addi(T0, T0, 1);
  __ scw(A0, T0, Address(A0));
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  100522af lr.w t0, (a0)\n"
      "  00128293 addi t0, t0, 1\n"
      "  1855252f sc.w a0, t0, (a0)\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  int32_t* value =
      reinterpret_cast<int32_t*>(Memory::Allocate(sizeof(int32_t)));
  *value = 0b1100;

  Simulator simulator;
  EXPECT_EQ(0, simulator.Call(buffer, Memory::ToGuest(value)));
  EXPECT_EQ(0b1101, *value);
}

UNIT_TEST(LoadReserveStoreConditionalWord_Failure) {
  Assembler assembler(RV_G);
  __ li(T0, 42);
  __ scw(A0, T0, Address(A0));
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  02a00293 li t0, 42\n"
      "  1855252f sc.w a0, t0, (a0)\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  int32_t* value =
      reinterpret_cast<int32_t*>(Memory::Allocate(sizeof(int32_t)));
  *value = 0b1100;

  Simulator simulator;
  EXPECT_EQ(false, 0 == simulator.Call(buffer, Memory::ToGuest(value)));
  EXPECT_EQ(0b1100, *value);
}

UNIT_TEST(AmoSwapWord) {
  Assembler assembler(RV_G);
  __ amoswapw(A0, A1, Address(A0));
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  08b5252f amoswap.w a0, a1, (a0)\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  int32_t* value =
      reinterpret_cast<int32_t*>(Memory::Allocate(sizeof(int32_t)));
  *value = 0b1100;

  Simulator simulator;
  EXPECT_EQ(0b1100, simulator.Call(buffer, Memory::ToGuest(value), 0b1010));
  EXPECT_EQ(0b1010, *value);
}

UNIT_TEST(AmoAddWord) {
  Assembler assembler(RV_G);
  __ amoaddw(A0, A1, Address(A0));
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  00b5252f amoadd.w a0, a1, (a0)\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  int32_t* value =
      reinterpret_cast<int32_t*>(Memory::Allocate(sizeof(int32_t)));
  *value = 42;

  Simulator simulator;
  EXPECT_EQ(42, simulator.Call(buffer, Memory::ToGuest(value), 10));
  EXPECT_EQ(52, *value);
}

UNIT_TEST(AmoXorWord) {
  Assembler assembler(RV_G);
  __ amoxorw(A0, A1, Address(A0));
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  20b5252f amoxor.w a0, a1, (a0)\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  int32_t* value =
      reinterpret_cast<int32_t*>(Memory::Allocate(sizeof(int32_t)));
  *value = 0b1100;

  Simulator simulator;
  EXPECT_EQ(0b1100, simulator.Call(buffer, Memory::ToGuest(value), 0b1010));
  EXPECT_EQ(0b0110, *value);
}

UNIT_TEST(AmoAndWord) {
  Assembler assembler(RV_G);
  __ amoandw(A0, A1, Address(A0));
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  60b5252f amoand.w a0, a1, (a0)\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  int32_t* value =
      reinterpret_cast<int32_t*>(Memory::Allocate(sizeof(int32_t)));
  *value = 0b1100;

  Simulator simulator;
  EXPECT_EQ(0b1100, simulator.Call(buffer, Memory::ToGuest(value), 0b1010));
  EXPECT_EQ(0b1000, *value);
}

UNIT_TEST(AmoOrWord) {
  Assembler assembler(RV_G);
  __ amoorw(A0, A1, Address(A0));
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  40b5252f amoor.w a0, a1, (a0)\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  int32_t* value =
      reinterpret_cast<int32_t*>(Memory::Allocate(sizeof(int32_t)));
  *value = 0b1100;

  Simulator simulator;
  EXPECT_EQ(0b1100, simulator.Call(buffer, Memory::ToGuest(value), 0b1010));
  EXPECT_EQ(0b1110, *value);
}

UNIT_TEST(AmoMinWord) {
  Assembler assembler(RV_G);
  __ amominw(A0, A1, Address(A0));
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  80b5252f amomin.w a0, a1, (a0)\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  int32_t* value =
      reinterpret_cast<int32_t*>(Memory::Allocate(sizeof(int32_t)));
  *value = -7;

  Simulator simulator;
  EXPECT_EQ(-7, simulator.Call(buffer, Memory::ToGuest(value), -4));
  EXPECT_EQ(-7, *value);
  EXPECT_EQ(-7, simulator.Call(buffer, Memory::ToGuest(value), -7));
  EXPECT_EQ(-7, *value);
  EXPECT_EQ(-7, simulator.Call(buffer, Memory::ToGuest(value), -11));
  EXPECT_EQ(-11, *value);
}

UNIT_TEST(AmoMaxWord) {
  Assembler assembler(RV_G);
  __ amomaxw(A0, A1, Address(A0));
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  a0b5252f amomax.w a0, a1, (a0)\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  int32_t* value =
      reinterpret_cast<int32_t*>(Memory::Allocate(sizeof(int32_t)));
  *value = -7;

  Simulator simulator;
  EXPECT_EQ(-7, simulator.Call(buffer, Memory::ToGuest(value), -11));
  EXPECT_EQ(-7, *value);
  EXPECT_EQ(-7, simulator.Call(buffer, Memory::ToGuest(value), -7));
  EXPECT_EQ(-7, *value);
  EXPECT_EQ(-7, simulator.Call(buffer, Memory::ToGuest(value), -4));
  EXPECT_EQ(-4, *value);
}

UNIT_TEST(AmoMinUnsignedWord) {
  Assembler assembler(RV_G);
  __ amominuw(A0, A1, Address(A0));
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  c0b5252f amominu.w a0, a1, (a0)\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  int32_t* value =
      reinterpret_cast<int32_t*>(Memory::Allocate(sizeof(int32_t)));
  *value = -7;

  Simulator simulator;
  EXPECT_EQ(sign_extend(static_cast<uint32_t>(-7)),
            simulator.Call(buffer, Memory::ToGuest(value), -4));
  EXPECT_EQ(-7, *value);
  EXPECT_EQ(sign_extend(static_cast<uint32_t>(-7)),
            simulator.Call(buffer, Memory::ToGuest(value), -7));
  EXPECT_EQ(-7, *value);
  EXPECT_EQ(sign_extend(static_cast<uint32_t>(-7)),
            simulator.Call(buffer, Memory::ToGuest(value), -11));
  EXPECT_EQ(-11, *value);
}

UNIT_TEST(AmoMaxUnsignedWord) {
  Assembler assembler(RV_G);
  __ amomaxuw(A0, A1, Address(A0));
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  e0b5252f amomaxu.w a0, a1, (a0)\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  int32_t* value =
      reinterpret_cast<int32_t*>(Memory::Allocate(sizeof(int32_t)));
  *value = -7;

  Simulator simulator;
  EXPECT_EQ(sign_extend(static_cast<uint32_t>(-7)),
            simulator.Call(buffer, Memory::ToGuest(value), -11));
  EXPECT_EQ(-7, *value);
  EXPECT_EQ(sign_extend(static_cast<uint32_t>(-7)),
            simulator.Call(buffer, Memory::ToGuest(value), -7));
  EXPECT_EQ(-7, *value);
  EXPECT_EQ(sign_extend(static_cast<uint32_t>(-7)),
            simulator.Call(buffer, Memory::ToGuest(value), -4));
  EXPECT_EQ(-4, *value);
}

#if XLEN >= 64
UNIT_TEST(LoadReserveStoreConditionalDoubleWord_Success) {
  Assembler assembler(RV_G);
  __ lrd(T0, Address(A0));
  __ addi(T0, T0, 1);
  __ scd(A0, T0, Address(A0));
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  100532af lr.d t0, (a0)\n"
      "  00128293 addi t0, t0, 1\n"
      "  1855352f sc.d a0, t0, (a0)\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  int64_t* value =
      reinterpret_cast<int64_t*>(Memory::Allocate(sizeof(int64_t)));
  *value = 0b1100;

  Simulator simulator;
  EXPECT_EQ(0, simulator.Call(buffer, Memory::ToGuest(value)));
  EXPECT_EQ(0b1101, *value);
}

UNIT_TEST(LoadReserveStoreConditionalDoubleWord_Failure) {
  Assembler assembler(RV_G);
  __ li(T0, 42);
  __ scd(A0, T0, Address(A0));
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  02a00293 li t0, 42\n"
      "  1855352f sc.d a0, t0, (a0)\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  int64_t* value =
      reinterpret_cast<int64_t*>(Memory::Allocate(sizeof(int64_t)));
  *value = 0b1100;

  Simulator simulator;
  EXPECT_EQ(false, 0 == simulator.Call(buffer, Memory::ToGuest(value)));
  EXPECT_EQ(0b1100, *value);
}

UNIT_TEST(AmoSwapDoubleWord) {
  Assembler assembler(RV_G);
  __ amoswapd(A0, A1, Address(A0));
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  08b5352f amoswap.d a0, a1, (a0)\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  int64_t* value =
      reinterpret_cast<int64_t*>(Memory::Allocate(sizeof(int64_t)));
  *value = 0b1100;

  Simulator simulator;
  EXPECT_EQ(0b1100, simulator.Call(buffer, Memory::ToGuest(value), 0b1010));
  EXPECT_EQ(0b1010, *value);
}

UNIT_TEST(AmoAddDoubleWord) {
  Assembler assembler(RV_G);
  __ amoaddd(A0, A1, Address(A0));
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  00b5352f amoadd.d a0, a1, (a0)\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  int64_t* value =
      reinterpret_cast<int64_t*>(Memory::Allocate(sizeof(int64_t)));
  *value = 42;

  Simulator simulator;
  EXPECT_EQ(42, simulator.Call(buffer, Memory::ToGuest(value), 10));
  EXPECT_EQ(52, *value);
}

UNIT_TEST(AmoXorDoubleWord) {
  Assembler assembler(RV_G);
  __ amoxord(A0, A1, Address(A0));
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  20b5352f amoxor.d a0, a1, (a0)\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  int64_t* value =
      reinterpret_cast<int64_t*>(Memory::Allocate(sizeof(int64_t)));
  *value = 0b1100;

  Simulator simulator;
  EXPECT_EQ(0b1100, simulator.Call(buffer, Memory::ToGuest(value), 0b1010));
  EXPECT_EQ(0b0110, *value);
}

UNIT_TEST(AmoAndDoubleWord) {
  Assembler assembler(RV_G);
  __ amoandd(A0, A1, Address(A0));
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  60b5352f amoand.d a0, a1, (a0)\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  int64_t* value =
      reinterpret_cast<int64_t*>(Memory::Allocate(sizeof(int64_t)));
  *value = 0b1100;

  Simulator simulator;
  EXPECT_EQ(0b1100, simulator.Call(buffer, Memory::ToGuest(value), 0b1010));
  EXPECT_EQ(0b1000, *value);
}

UNIT_TEST(AmoOrDoubleWord) {
  Assembler assembler(RV_G);
  __ amoord(A0, A1, Address(A0));
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  40b5352f amoor.d a0, a1, (a0)\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  int64_t* value =
      reinterpret_cast<int64_t*>(Memory::Allocate(sizeof(int64_t)));
  *value = 0b1100;

  Simulator simulator;
  EXPECT_EQ(0b1100, simulator.Call(buffer, Memory::ToGuest(value), 0b1010));
  EXPECT_EQ(0b1110, *value);
}

UNIT_TEST(AmoMinDoubleWord) {
  Assembler assembler(RV_G);
  __ amomind(A0, A1, Address(A0));
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  80b5352f amomin.d a0, a1, (a0)\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  int64_t* value =
      reinterpret_cast<int64_t*>(Memory::Allocate(sizeof(int64_t)));
  *value = -7;

  Simulator simulator;
  EXPECT_EQ(-7, simulator.Call(buffer, Memory::ToGuest(value), -4));
  EXPECT_EQ(-7, *value);
  EXPECT_EQ(-7, simulator.Call(buffer, Memory::ToGuest(value), -7));
  EXPECT_EQ(-7, *value);
  EXPECT_EQ(-7, simulator.Call(buffer, Memory::ToGuest(value), -11));
  EXPECT_EQ(-11, *value);
}

UNIT_TEST(AmoMaxDoubleWord) {
  Assembler assembler(RV_G);
  __ amomaxd(A0, A1, Address(A0));
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  a0b5352f amomax.d a0, a1, (a0)\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  int64_t* value =
      reinterpret_cast<int64_t*>(Memory::Allocate(sizeof(int64_t)));
  *value = -7;

  Simulator simulator;
  EXPECT_EQ(-7, simulator.Call(buffer, Memory::ToGuest(value), -11));
  EXPECT_EQ(-7, *value);
  EXPECT_EQ(-7, simulator.Call(buffer, Memory::ToGuest(value), -7));
  EXPECT_EQ(-7, *value);
  EXPECT_EQ(-7, simulator.Call(buffer, Memory::ToGuest(value), -4));
  EXPECT_EQ(-4, *value);
}

UNIT_TEST(AmoMinUnsignedDoubleWord) {
  Assembler assembler(RV_G);
  __ amominud(A0, A1, Address(A0));
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  c0b5352f amominu.d a0, a1, (a0)\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  int64_t* value =
      reinterpret_cast<int64_t*>(Memory::Allocate(sizeof(int64_t)));
  *value = -7;

  Simulator simulator;
  EXPECT_EQ(-7, simulator.Call(buffer, Memory::ToGuest(value), -4));
  EXPECT_EQ(-7, *value);
  EXPECT_EQ(-7, simulator.Call(buffer, Memory::ToGuest(value), -7));
  EXPECT_EQ(-7, *value);
  EXPECT_EQ(-7, simulator.Call(buffer, Memory::ToGuest(value), -11));
  EXPECT_EQ(-11, *value);
}

UNIT_TEST(AmoMaxUnsignedDoubleWord) {
  Assembler assembler(RV_G);
  __ amomaxud(A0, A1, Address(A0));
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  e0b5352f amomaxu.d a0, a1, (a0)\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  int64_t* value =
      reinterpret_cast<int64_t*>(Memory::Allocate(sizeof(int64_t)));
  *value = -7;

  Simulator simulator;
  EXPECT_EQ(-7, simulator.Call(buffer, Memory::ToGuest(value), -11));
  EXPECT_EQ(-7, *value);
  EXPECT_EQ(-7, simulator.Call(buffer, Memory::ToGuest(value), -7));
  EXPECT_EQ(-7, *value);
  EXPECT_EQ(-7, simulator.Call(buffer, Memory::ToGuest(value), -4));
  EXPECT_EQ(-4, *value);
}
#endif

UNIT_TEST(LoadSingleFloat) {
  Assembler assembler(RV_G);
  __ flw(FA0, Address(A0, 1 * sizeof(float)));
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  00452507 flw fa0, 4(a0)\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  float* data = reinterpret_cast<float*>(Memory::Allocate(3 * sizeof(float)));
  data[0] = 1.7f;
  data[1] = 2.8f;
  data[2] = 3.9f;

  Simulator simulator;
  EXPECT_EQ(data[1], simulator.CallF(buffer, Memory::ToGuest(data)));
}

UNIT_TEST(StoreSingleFloat) {
  Assembler assembler(RV_G);
  __ fsw(FA0, Address(A0, 1 * sizeof(float)));
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  00a52227 fsw fa0, 4(a0)\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  float* data = reinterpret_cast<float*>(Memory::Allocate(3 * sizeof(float)));
  data[0] = 1.7f;
  data[1] = 2.8f;
  data[2] = 3.9f;

  Simulator simulator;
  simulator.CallF(buffer, Memory::ToGuest(data), 4.2f);
  EXPECT_EQ(4.2f, data[1]);
}

UNIT_TEST(SingleMultiplyAdd) {
  Assembler assembler(RV_G);
  __ fmadds(FA0, FA0, FA1, FA2);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  60b50543 fmadd.s fa0, fa0, fa1, fa2\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(22.0, simulator.CallF(buffer, 3.0, 5.0, 7.0));
  EXPECT_EQ(-8.0, simulator.CallF(buffer, -3.0, 5.0, 7.0));
  EXPECT_EQ(-8.0, simulator.CallF(buffer, 3.0, -5.0, 7.0));
  EXPECT_EQ(8.0, simulator.CallF(buffer, 3.0, 5.0, -7.0));

  EXPECT_EQ(26.0, simulator.CallF(buffer, 7.0, 3.0, 5.0));
  EXPECT_EQ(-16.0, simulator.CallF(buffer, -7.0, 3.0, 5.0));
  EXPECT_EQ(-16.0, simulator.CallF(buffer, 7.0, -3.0, 5.0));
  EXPECT_EQ(16.0, simulator.CallF(buffer, 7.0, 3.0, -5.0));
}

UNIT_TEST(SingleMultiplySubtract) {
  Assembler assembler(RV_G);
  __ fmsubs(FA0, FA0, FA1, FA2);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  60b50547 fmsub.s fa0, fa0, fa1, fa2\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(8.0, simulator.CallF(buffer, 3.0, 5.0, 7.0));
  EXPECT_EQ(-22.0, simulator.CallF(buffer, -3.0, 5.0, 7.0));
  EXPECT_EQ(-22.0, simulator.CallF(buffer, 3.0, -5.0, 7.0));
  EXPECT_EQ(22.0, simulator.CallF(buffer, 3.0, 5.0, -7.0));

  EXPECT_EQ(16.0, simulator.CallF(buffer, 7.0, 3.0, 5.0));
  EXPECT_EQ(-26.0, simulator.CallF(buffer, -7.0, 3.0, 5.0));
  EXPECT_EQ(-26.0, simulator.CallF(buffer, 7.0, -3.0, 5.0));
  EXPECT_EQ(26.0, simulator.CallF(buffer, 7.0, 3.0, -5.0));
}

UNIT_TEST(SingleNegateMultiplySubtract) {
  Assembler assembler(RV_G);
  __ fnmsubs(FA0, FA0, FA1, FA2);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  60b5054b fnmsub.s fa0, fa0, fa1, fa2\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(-8.0, simulator.CallF(buffer, 3.0, 5.0, 7.0));
  EXPECT_EQ(22.0, simulator.CallF(buffer, -3.0, 5.0, 7.0));
  EXPECT_EQ(22.0, simulator.CallF(buffer, 3.0, -5.0, 7.0));
  EXPECT_EQ(-22.0, simulator.CallF(buffer, 3.0, 5.0, -7.0));

  EXPECT_EQ(-16.0, simulator.CallF(buffer, 7.0, 3.0, 5.0));
  EXPECT_EQ(26.0, simulator.CallF(buffer, -7.0, 3.0, 5.0));
  EXPECT_EQ(26.0, simulator.CallF(buffer, 7.0, -3.0, 5.0));
  EXPECT_EQ(-26.0, simulator.CallF(buffer, 7.0, 3.0, -5.0));
}

UNIT_TEST(SingleNegateMultiplyAdd) {
  Assembler assembler(RV_G);
  __ fnmadds(FA0, FA0, FA1, FA2);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  60b5054f fnmadd.s fa0, fa0, fa1, fa2\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(-22.0, simulator.CallF(buffer, 3.0, 5.0, 7.0));
  EXPECT_EQ(8.0, simulator.CallF(buffer, -3.0, 5.0, 7.0));
  EXPECT_EQ(8.0, simulator.CallF(buffer, 3.0, -5.0, 7.0));
  EXPECT_EQ(-8.0, simulator.CallF(buffer, 3.0, 5.0, -7.0));

  EXPECT_EQ(-26.0, simulator.CallF(buffer, 7.0, 3.0, 5.0));
  EXPECT_EQ(16.0, simulator.CallF(buffer, -7.0, 3.0, 5.0));
  EXPECT_EQ(16.0, simulator.CallF(buffer, 7.0, -3.0, 5.0));
  EXPECT_EQ(-16.0, simulator.CallF(buffer, 7.0, 3.0, -5.0));
}

UNIT_TEST(SingleAdd) {
  Assembler assembler(RV_G);
  __ fadds(FA0, FA0, FA1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  00b50553 fadd.s fa0, fa0, fa1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(8.0f, simulator.CallF(buffer, 3.0f, 5.0f));
  EXPECT_EQ(2.0f, simulator.CallF(buffer, -3.0f, 5.0f));
  EXPECT_EQ(-2.0f, simulator.CallF(buffer, 3.0f, -5.0f));
  EXPECT_EQ(-8.0f, simulator.CallF(buffer, -3.0f, -5.0f));

  EXPECT_EQ(10.0f, simulator.CallF(buffer, 7.0f, 3.0f));
  EXPECT_EQ(-4.0f, simulator.CallF(buffer, -7.0f, 3.0f));
  EXPECT_EQ(4.0f, simulator.CallF(buffer, 7.0f, -3.0f));
  EXPECT_EQ(-10.0f, simulator.CallF(buffer, -7.0f, -3.0f));
}

UNIT_TEST(SingleSubtract) {
  Assembler assembler(RV_G);
  __ fsubs(FA0, FA0, FA1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  08b50553 fsub.s fa0, fa0, fa1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(-2.0f, simulator.CallF(buffer, 3.0f, 5.0f));
  EXPECT_EQ(-8.0f, simulator.CallF(buffer, -3.0f, 5.0f));
  EXPECT_EQ(8.0f, simulator.CallF(buffer, 3.0f, -5.0f));
  EXPECT_EQ(2.0f, simulator.CallF(buffer, -3.0f, -5.0f));

  EXPECT_EQ(4.0f, simulator.CallF(buffer, 7.0f, 3.0f));
  EXPECT_EQ(-10.0f, simulator.CallF(buffer, -7.0f, 3.0f));
  EXPECT_EQ(10.0f, simulator.CallF(buffer, 7.0f, -3.0f));
  EXPECT_EQ(-4.0f, simulator.CallF(buffer, -7.0f, -3.0f));
}

UNIT_TEST(SingleMultiply) {
  Assembler assembler(RV_G);
  __ fmuls(FA0, FA0, FA1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  10b50553 fmul.s fa0, fa0, fa1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(15.0f, simulator.CallF(buffer, 3.0f, 5.0f));
  EXPECT_EQ(-15.0f, simulator.CallF(buffer, -3.0f, 5.0f));
  EXPECT_EQ(-15.0f, simulator.CallF(buffer, 3.0f, -5.0f));
  EXPECT_EQ(15.0f, simulator.CallF(buffer, -3.0f, -5.0f));

  EXPECT_EQ(21.0f, simulator.CallF(buffer, 7.0f, 3.0f));
  EXPECT_EQ(-21.0f, simulator.CallF(buffer, -7.0f, 3.0f));
  EXPECT_EQ(-21.0f, simulator.CallF(buffer, 7.0f, -3.0f));
  EXPECT_EQ(21.0f, simulator.CallF(buffer, -7.0f, -3.0f));
}

UNIT_TEST(SingleDivide) {
  Assembler assembler(RV_G);
  __ fdivs(FA0, FA0, FA1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  18b50553 fdiv.s fa0, fa0, fa1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(2.0f, simulator.CallF(buffer, 10.0f, 5.0f));
  EXPECT_EQ(-2.0f, simulator.CallF(buffer, -10.0f, 5.0f));
  EXPECT_EQ(-2.0f, simulator.CallF(buffer, 10.0f, -5.0f));
  EXPECT_EQ(2.0f, simulator.CallF(buffer, -10.0f, -5.0f));
}

UNIT_TEST(SingleSquareRoot) {
  Assembler assembler(RV_G);
  __ fsqrts(FA0, FA0);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  58050553 fsqrt.s fa0, fa0\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0.0f, simulator.CallF(buffer, 0.0f));
  EXPECT_EQ(1.0f, simulator.CallF(buffer, 1.0f));
  EXPECT_EQ(2.0f, simulator.CallF(buffer, 4.0f));
  EXPECT_EQ(3.0f, simulator.CallF(buffer, 9.0f));
}

UNIT_TEST(SingleSignInject) {
  Assembler assembler(RV_G);
  __ fsgnjs(FA0, FA0, FA1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  20b50553 fsgnj.s fa0, fa0, fa1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(3.0f, simulator.CallF(buffer, 3.0f, 5.0f));
  EXPECT_EQ(3.0f, simulator.CallF(buffer, -3.0f, 5.0f));
  EXPECT_EQ(-3.0f, simulator.CallF(buffer, 3.0f, -5.0f));
  EXPECT_EQ(-3.0f, simulator.CallF(buffer, -3.0f, -5.0f));
}

UNIT_TEST(SingleNegatedSignInject) {
  Assembler assembler(RV_G);
  __ fsgnjns(FA0, FA0, FA1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  20b51553 fsgnjn.s fa0, fa0, fa1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(-3.0f, simulator.CallF(buffer, 3.0f, 5.0f));
  EXPECT_EQ(-3.0f, simulator.CallF(buffer, -3.0f, 5.0f));
  EXPECT_EQ(3.0f, simulator.CallF(buffer, 3.0f, -5.0f));
  EXPECT_EQ(3.0f, simulator.CallF(buffer, -3.0f, -5.0f));
}

UNIT_TEST(SingleXorSignInject) {
  Assembler assembler(RV_G);
  __ fsgnjxs(FA0, FA0, FA1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  20b52553 fsgnjx.s fa0, fa0, fa1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(3.0f, simulator.CallF(buffer, 3.0f, 5.0f));
  EXPECT_EQ(-3.0f, simulator.CallF(buffer, -3.0f, 5.0f));
  EXPECT_EQ(-3.0f, simulator.CallF(buffer, 3.0f, -5.0f));
  EXPECT_EQ(3.0f, simulator.CallF(buffer, -3.0f, -5.0f));
}

UNIT_TEST(SingleMin) {
  Assembler assembler(RV_G);
  __ fmins(FA0, FA0, FA1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  28b50553 fmin.s fa0, fa0, fa1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(1.0f, simulator.CallF(buffer, 3.0f, 1.0f));
  EXPECT_EQ(3.0f, simulator.CallF(buffer, 3.0f, 3.0f));
  EXPECT_EQ(3.0f, simulator.CallF(buffer, 3.0f, 5.0f));
  EXPECT_EQ(-1.0f, simulator.CallF(buffer, 3.0f, -1.0f));
  EXPECT_EQ(-3.0f, simulator.CallF(buffer, 3.0f, -3.0f));
  EXPECT_EQ(-5.0f, simulator.CallF(buffer, 3.0f, -5.0f));
  EXPECT_EQ(-3.0f, simulator.CallF(buffer, -3.0f, 1.0f));
  EXPECT_EQ(-3.0f, simulator.CallF(buffer, -3.0f, 3.0f));
  EXPECT_EQ(-3.0f, simulator.CallF(buffer, -3.0f, 5.0f));
  EXPECT_EQ(-3.0f, simulator.CallF(buffer, -3.0f, -1.0f));
  EXPECT_EQ(-3.0f, simulator.CallF(buffer, -3.0f, -3.0f));
  EXPECT_EQ(-5.0f, simulator.CallF(buffer, -3.0f, -5.0f));

  EXPECT_BITEQ(-0.0f, simulator.CallF(buffer, 0.0f, -0.0f));
  EXPECT_BITEQ(-0.0f, simulator.CallF(buffer, -0.0f, 0.0f));

  float qNAN = std::numeric_limits<float>::quiet_NaN();
  EXPECT_EQ(3.0f, simulator.CallF(buffer, 3.0f, qNAN));
  EXPECT_EQ(3.0f, simulator.CallF(buffer, qNAN, 3.0f));
  EXPECT_EQ(-3.0f, simulator.CallF(buffer, -3.0f, qNAN));
  EXPECT_EQ(-3.0f, simulator.CallF(buffer, qNAN, -3.0f));

  float sNAN = std::numeric_limits<float>::signaling_NaN();
  EXPECT_EQ(3.0f, simulator.CallF(buffer, 3.0f, sNAN));
  EXPECT_EQ(3.0f, simulator.CallF(buffer, sNAN, 3.0f));
  EXPECT_EQ(-3.0f, simulator.CallF(buffer, -3.0f, sNAN));
  EXPECT_EQ(-3.0f, simulator.CallF(buffer, sNAN, -3.0f));

  EXPECT_BITEQ(qNAN, simulator.CallF(buffer, qNAN, qNAN));
  EXPECT_BITEQ(qNAN, simulator.CallF(buffer, sNAN, sNAN));
  EXPECT_BITEQ(qNAN, simulator.CallF(buffer, qNAN, sNAN));
  EXPECT_BITEQ(qNAN, simulator.CallF(buffer, sNAN, qNAN));
}

UNIT_TEST(SingleMax) {
  Assembler assembler(RV_G);
  __ fmaxs(FA0, FA0, FA1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  28b51553 fmax.s fa0, fa0, fa1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(3.0f, simulator.CallF(buffer, 3.0f, 1.0f));
  EXPECT_EQ(3.0f, simulator.CallF(buffer, 3.0f, 3.0f));
  EXPECT_EQ(5.0f, simulator.CallF(buffer, 3.0f, 5.0f));
  EXPECT_EQ(3.0f, simulator.CallF(buffer, 3.0f, -1.0f));
  EXPECT_EQ(3.0f, simulator.CallF(buffer, 3.0f, -3.0f));
  EXPECT_EQ(3.0f, simulator.CallF(buffer, 3.0f, -5.0f));
  EXPECT_EQ(1.0f, simulator.CallF(buffer, -3.0f, 1.0f));
  EXPECT_EQ(3.0f, simulator.CallF(buffer, -3.0f, 3.0f));
  EXPECT_EQ(5.0f, simulator.CallF(buffer, -3.0f, 5.0f));
  EXPECT_EQ(-1.0f, simulator.CallF(buffer, -3.0f, -1.0f));
  EXPECT_EQ(-3.0f, simulator.CallF(buffer, -3.0f, -3.0f));
  EXPECT_EQ(-3.0f, simulator.CallF(buffer, -3.0f, -5.0f));

  EXPECT_BITEQ(0.0f, simulator.CallF(buffer, 0.0f, -0.0f));
  EXPECT_BITEQ(0.0f, simulator.CallF(buffer, -0.0f, 0.0f));

  float qNAN = std::numeric_limits<float>::quiet_NaN();
  EXPECT_EQ(3.0f, simulator.CallF(buffer, 3.0f, qNAN));
  EXPECT_EQ(3.0f, simulator.CallF(buffer, qNAN, 3.0f));
  EXPECT_EQ(-3.0f, simulator.CallF(buffer, -3.0f, qNAN));
  EXPECT_EQ(-3.0f, simulator.CallF(buffer, qNAN, -3.0f));

  float sNAN = std::numeric_limits<float>::signaling_NaN();
  EXPECT_EQ(3.0f, simulator.CallF(buffer, 3.0f, sNAN));
  EXPECT_EQ(3.0f, simulator.CallF(buffer, sNAN, 3.0f));
  EXPECT_EQ(-3.0f, simulator.CallF(buffer, -3.0f, sNAN));
  EXPECT_EQ(-3.0f, simulator.CallF(buffer, sNAN, -3.0f));

  EXPECT_BITEQ(qNAN, simulator.CallF(buffer, qNAN, qNAN));
  EXPECT_BITEQ(qNAN, simulator.CallF(buffer, sNAN, sNAN));
  EXPECT_BITEQ(qNAN, simulator.CallF(buffer, qNAN, sNAN));
  EXPECT_BITEQ(qNAN, simulator.CallF(buffer, sNAN, qNAN));
}

UNIT_TEST(SingleEqual) {
  Assembler assembler(RV_G);
  __ feqs(A0, FA0, FA1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  a0b52553 feq.s a0, fa0, fa1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0, simulator.CallI(buffer, 3.0f, 1.0f));
  EXPECT_EQ(1, simulator.CallI(buffer, 3.0f, 3.0f));
  EXPECT_EQ(0, simulator.CallI(buffer, 3.0f, 5.0f));
  EXPECT_EQ(0, simulator.CallI(buffer, 3.0f, -1.0f));
  EXPECT_EQ(0, simulator.CallI(buffer, 3.0f, -3.0f));
  EXPECT_EQ(0, simulator.CallI(buffer, 3.0f, -5.0f));
  EXPECT_EQ(0, simulator.CallI(buffer, -3.0f, 1.0f));
  EXPECT_EQ(0, simulator.CallI(buffer, -3.0f, 3.0f));
  EXPECT_EQ(0, simulator.CallI(buffer, -3.0f, 5.0f));
  EXPECT_EQ(0, simulator.CallI(buffer, -3.0f, -1.0f));
  EXPECT_EQ(1, simulator.CallI(buffer, -3.0f, -3.0f));
  EXPECT_EQ(0, simulator.CallI(buffer, -3.0f, -5.0f));

  float qNAN = std::numeric_limits<float>::quiet_NaN();
  EXPECT_EQ(0, simulator.CallI(buffer, 3.0f, qNAN));
  EXPECT_EQ(0, simulator.CallI(buffer, qNAN, 3.0f));
  EXPECT_EQ(0, simulator.CallI(buffer, -3.0f, qNAN));
  EXPECT_EQ(0, simulator.CallI(buffer, qNAN, -3.0f));
}

UNIT_TEST(SingleLessThan) {
  Assembler assembler(RV_G);
  __ flts(A0, FA0, FA1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  a0b51553 flt.s a0, fa0, fa1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0, simulator.CallI(buffer, 3.0f, 1.0f));
  EXPECT_EQ(0, simulator.CallI(buffer, 3.0f, 3.0f));
  EXPECT_EQ(1, simulator.CallI(buffer, 3.0f, 5.0f));
  EXPECT_EQ(0, simulator.CallI(buffer, 3.0f, -1.0f));
  EXPECT_EQ(0, simulator.CallI(buffer, 3.0f, -3.0f));
  EXPECT_EQ(0, simulator.CallI(buffer, 3.0f, -5.0f));
  EXPECT_EQ(1, simulator.CallI(buffer, -3.0f, 1.0f));
  EXPECT_EQ(1, simulator.CallI(buffer, -3.0f, 3.0f));
  EXPECT_EQ(1, simulator.CallI(buffer, -3.0f, 5.0f));
  EXPECT_EQ(1, simulator.CallI(buffer, -3.0f, -1.0f));
  EXPECT_EQ(0, simulator.CallI(buffer, -3.0f, -3.0f));
  EXPECT_EQ(0, simulator.CallI(buffer, -3.0f, -5.0f));

  float qNAN = std::numeric_limits<float>::quiet_NaN();
  EXPECT_EQ(0, simulator.CallI(buffer, 3.0f, qNAN));
  EXPECT_EQ(0, simulator.CallI(buffer, qNAN, 3.0f));
  EXPECT_EQ(0, simulator.CallI(buffer, -3.0f, qNAN));
  EXPECT_EQ(0, simulator.CallI(buffer, qNAN, -3.0f));
}

UNIT_TEST(SingleLessOrEqual) {
  Assembler assembler(RV_G);
  __ fles(A0, FA0, FA1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  a0b50553 fle.s a0, fa0, fa1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0, simulator.CallI(buffer, 3.0f, 1.0f));
  EXPECT_EQ(1, simulator.CallI(buffer, 3.0f, 3.0f));
  EXPECT_EQ(1, simulator.CallI(buffer, 3.0f, 5.0f));
  EXPECT_EQ(0, simulator.CallI(buffer, 3.0f, -1.0f));
  EXPECT_EQ(0, simulator.CallI(buffer, 3.0f, -3.0f));
  EXPECT_EQ(0, simulator.CallI(buffer, 3.0f, -5.0f));
  EXPECT_EQ(1, simulator.CallI(buffer, -3.0f, 1.0f));
  EXPECT_EQ(1, simulator.CallI(buffer, -3.0f, 3.0f));
  EXPECT_EQ(1, simulator.CallI(buffer, -3.0f, 5.0f));
  EXPECT_EQ(1, simulator.CallI(buffer, -3.0f, -1.0f));
  EXPECT_EQ(1, simulator.CallI(buffer, -3.0f, -3.0f));
  EXPECT_EQ(0, simulator.CallI(buffer, -3.0f, -5.0f));

  float qNAN = std::numeric_limits<float>::quiet_NaN();
  EXPECT_EQ(0, simulator.CallI(buffer, 3.0f, qNAN));
  EXPECT_EQ(0, simulator.CallI(buffer, qNAN, 3.0f));
  EXPECT_EQ(0, simulator.CallI(buffer, -3.0f, qNAN));
  EXPECT_EQ(0, simulator.CallI(buffer, qNAN, -3.0f));
}

UNIT_TEST(SingleClassify) {
  Assembler assembler(RV_G);
  __ fclasss(A0, FA0);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  e0051553 fclass.s a0, fa0\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  // Neg infinity
  EXPECT_EQ(1 << 0,
            simulator.CallI(buffer, -std::numeric_limits<float>::infinity()));
  // Neg normal
  EXPECT_EQ(1 << 1, simulator.CallI(buffer, -1.0f));
  // Neg subnormal
  EXPECT_EQ(1 << 2,
            simulator.CallI(buffer, -std::numeric_limits<float>::min() / 2.0f));
  // Neg zero
  EXPECT_EQ(1 << 3, simulator.CallI(buffer, -0.0f));
  // Pos zero
  EXPECT_EQ(1 << 4, simulator.CallI(buffer, 0.0f));
  // Pos subnormal
  EXPECT_EQ(1 << 5,
            simulator.CallI(buffer, std::numeric_limits<float>::min() / 2.0f));
  // Pos normal
  EXPECT_EQ(1 << 6, simulator.CallI(buffer, 1.0f));
  // Pos infinity
  EXPECT_EQ(1 << 7,
            simulator.CallI(buffer, std::numeric_limits<float>::infinity()));
  // Signaling NaN
  EXPECT_EQ(1 << 8, simulator.CallI(
                        buffer, std::numeric_limits<float>::signaling_NaN()));
  // Queit NaN
  EXPECT_EQ(1 << 9,
            simulator.CallI(buffer, std::numeric_limits<float>::quiet_NaN()));
}

UNIT_TEST(ConvertSingleToWord) {
  Assembler assembler(RV_G);
  __ fcvtws(A0, FA0);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  c0050553 fcvt.w.s a0, fa0\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(-42, simulator.CallI(buffer, static_cast<float>(-42)));
  EXPECT_EQ(0, simulator.CallI(buffer, static_cast<float>(0)));
  EXPECT_EQ(42, simulator.CallI(buffer, static_cast<float>(42)));
  EXPECT_EQ(sign_extend(kMinInt32),
            simulator.CallI(buffer, static_cast<float>(kMinInt32)));
  EXPECT_EQ(sign_extend(kMaxInt32),
            simulator.CallI(buffer, static_cast<float>(kMaxInt32)));
  EXPECT_EQ(sign_extend(kMaxInt32),
            simulator.CallI(buffer, static_cast<float>(kMaxUint32)));
  EXPECT_EQ(sign_extend(kMinInt32),
            simulator.CallI(buffer, static_cast<float>(kMinInt64)));
  EXPECT_EQ(sign_extend(kMaxInt32),
            simulator.CallI(buffer, static_cast<float>(kMaxInt64)));
  EXPECT_EQ(sign_extend(kMaxInt32),
            simulator.CallI(buffer, static_cast<float>(kMaxUint64)));
  EXPECT_EQ(sign_extend(kMinInt32),
            simulator.CallI(buffer, -std::numeric_limits<float>::infinity()));
  EXPECT_EQ(sign_extend(kMaxInt32),
            simulator.CallI(buffer, std::numeric_limits<float>::infinity()));
  EXPECT_EQ(
      sign_extend(kMaxInt32),
      simulator.CallI(buffer, std::numeric_limits<float>::signaling_NaN()));
}


UNIT_TEST(ConvertSingleToWord_RNE) {
  Assembler assembler(RV_G);
  __ fcvtws(A0, FA0, RNE);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  c0050553 fcvt.w.s a0, fa0\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(-44, simulator.CallI(buffer, -43.6f));
  EXPECT_EQ(-44, simulator.CallI(buffer, -43.5f));
  EXPECT_EQ(-43, simulator.CallI(buffer, -43.4f));
  EXPECT_EQ(-43, simulator.CallI(buffer, -43.0f));
  EXPECT_EQ(-43, simulator.CallI(buffer, -42.6f));
  EXPECT_EQ(-42, simulator.CallI(buffer, -42.5f));
  EXPECT_EQ(-42, simulator.CallI(buffer, -42.4f));
  EXPECT_EQ(-42, simulator.CallI(buffer, -42.0f));
  EXPECT_EQ(0, simulator.CallI(buffer, -0.0f));
  EXPECT_EQ(0, simulator.CallI(buffer, +0.0f));
  EXPECT_EQ(42, simulator.CallI(buffer, 42.0f));
  EXPECT_EQ(42, simulator.CallI(buffer, 42.4f));
  EXPECT_EQ(42, simulator.CallI(buffer, 42.5f));
  EXPECT_EQ(43, simulator.CallI(buffer, 42.6f));
  EXPECT_EQ(43, simulator.CallI(buffer, 43.0f));
  EXPECT_EQ(43, simulator.CallI(buffer, 43.4f));
  EXPECT_EQ(44, simulator.CallI(buffer, 43.5f));
  EXPECT_EQ(44, simulator.CallI(buffer, 43.6f));
}

UNIT_TEST(ConvertSingleToWord_RTZ) {
  Assembler assembler(RV_G);
  __ fcvtws(A0, FA0, RTZ);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  c0051553 fcvt.w.s a0, fa0, rtz\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(-43, simulator.CallI(buffer, -43.6f));
  EXPECT_EQ(-43, simulator.CallI(buffer, -43.5f));
  EXPECT_EQ(-43, simulator.CallI(buffer, -43.4f));
  EXPECT_EQ(-43, simulator.CallI(buffer, -43.0f));
  EXPECT_EQ(-42, simulator.CallI(buffer, -42.6f));
  EXPECT_EQ(-42, simulator.CallI(buffer, -42.5f));
  EXPECT_EQ(-42, simulator.CallI(buffer, -42.4f));
  EXPECT_EQ(-42, simulator.CallI(buffer, -42.0f));
  EXPECT_EQ(0, simulator.CallI(buffer, -0.0f));
  EXPECT_EQ(0, simulator.CallI(buffer, +0.0f));
  EXPECT_EQ(42, simulator.CallI(buffer, 42.0f));
  EXPECT_EQ(42, simulator.CallI(buffer, 42.4f));
  EXPECT_EQ(42, simulator.CallI(buffer, 42.5f));
  EXPECT_EQ(42, simulator.CallI(buffer, 42.6f));
  EXPECT_EQ(43, simulator.CallI(buffer, 43.0f));
  EXPECT_EQ(43, simulator.CallI(buffer, 43.4f));
  EXPECT_EQ(43, simulator.CallI(buffer, 43.5f));
  EXPECT_EQ(43, simulator.CallI(buffer, 43.6f));
}

UNIT_TEST(ConvertSingleToWord_RDN) {
  Assembler assembler(RV_G);
  __ fcvtws(A0, FA0, RDN);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  c0052553 fcvt.w.s a0, fa0, rdn\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(-44, simulator.CallI(buffer, -43.6f));
  EXPECT_EQ(-44, simulator.CallI(buffer, -43.5f));
  EXPECT_EQ(-44, simulator.CallI(buffer, -43.4f));
  EXPECT_EQ(-43, simulator.CallI(buffer, -43.0f));
  EXPECT_EQ(-43, simulator.CallI(buffer, -42.6f));
  EXPECT_EQ(-43, simulator.CallI(buffer, -42.5f));
  EXPECT_EQ(-43, simulator.CallI(buffer, -42.4f));
  EXPECT_EQ(-42, simulator.CallI(buffer, -42.0f));
  EXPECT_EQ(0, simulator.CallI(buffer, -0.0f));
  EXPECT_EQ(0, simulator.CallI(buffer, +0.0f));
  EXPECT_EQ(42, simulator.CallI(buffer, 42.0f));
  EXPECT_EQ(42, simulator.CallI(buffer, 42.4f));
  EXPECT_EQ(42, simulator.CallI(buffer, 42.5f));
  EXPECT_EQ(42, simulator.CallI(buffer, 42.6f));
  EXPECT_EQ(43, simulator.CallI(buffer, 43.0f));
  EXPECT_EQ(43, simulator.CallI(buffer, 43.4f));
  EXPECT_EQ(43, simulator.CallI(buffer, 43.5f));
  EXPECT_EQ(43, simulator.CallI(buffer, 43.6f));
}

UNIT_TEST(ConvertSingleToWord_RUP) {
  Assembler assembler(RV_G);
  __ fcvtws(A0, FA0, RUP);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  c0053553 fcvt.w.s a0, fa0, rup\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(-43, simulator.CallI(buffer, -43.6f));
  EXPECT_EQ(-43, simulator.CallI(buffer, -43.5f));
  EXPECT_EQ(-43, simulator.CallI(buffer, -43.4f));
  EXPECT_EQ(-43, simulator.CallI(buffer, -43.0f));
  EXPECT_EQ(-42, simulator.CallI(buffer, -42.6f));
  EXPECT_EQ(-42, simulator.CallI(buffer, -42.5f));
  EXPECT_EQ(-42, simulator.CallI(buffer, -42.4f));
  EXPECT_EQ(-42, simulator.CallI(buffer, -42.0f));
  EXPECT_EQ(0, simulator.CallI(buffer, -0.0f));
  EXPECT_EQ(0, simulator.CallI(buffer, +0.0f));
  EXPECT_EQ(42, simulator.CallI(buffer, 42.0f));
  EXPECT_EQ(43, simulator.CallI(buffer, 42.4f));
  EXPECT_EQ(43, simulator.CallI(buffer, 42.5f));
  EXPECT_EQ(43, simulator.CallI(buffer, 42.6f));
  EXPECT_EQ(43, simulator.CallI(buffer, 43.0f));
  EXPECT_EQ(44, simulator.CallI(buffer, 43.5f));
  EXPECT_EQ(44, simulator.CallI(buffer, 43.5f));
  EXPECT_EQ(44, simulator.CallI(buffer, 43.6f));
}

UNIT_TEST(ConvertSingleToWord_RMM) {
  Assembler assembler(RV_G);
  __ fcvtws(A0, FA0, RMM);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  c0054553 fcvt.w.s a0, fa0, rmm\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(-44, simulator.CallI(buffer, -43.6f));
  EXPECT_EQ(-44, simulator.CallI(buffer, -43.5f));
  EXPECT_EQ(-43, simulator.CallI(buffer, -43.4f));
  EXPECT_EQ(-43, simulator.CallI(buffer, -43.0f));
  EXPECT_EQ(-43, simulator.CallI(buffer, -42.6f));
  EXPECT_EQ(-43, simulator.CallI(buffer, -42.5f));
  EXPECT_EQ(-42, simulator.CallI(buffer, -42.4f));
  EXPECT_EQ(-42, simulator.CallI(buffer, -42.0f));
  EXPECT_EQ(0, simulator.CallI(buffer, -0.0f));
  EXPECT_EQ(0, simulator.CallI(buffer, +0.0f));
  EXPECT_EQ(42, simulator.CallI(buffer, 42.0f));
  EXPECT_EQ(42, simulator.CallI(buffer, 42.4f));
  EXPECT_EQ(43, simulator.CallI(buffer, 42.5f));
  EXPECT_EQ(43, simulator.CallI(buffer, 42.6f));
  EXPECT_EQ(43, simulator.CallI(buffer, 43.0f));
  EXPECT_EQ(43, simulator.CallI(buffer, 43.4f));
  EXPECT_EQ(44, simulator.CallI(buffer, 43.5f));
  EXPECT_EQ(44, simulator.CallI(buffer, 43.6f));
}

UNIT_TEST(ConvertSingleToUnsignedWord) {
  Assembler assembler(RV_G);
  __ fcvtwus(A0, FA0);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  c0150553 fcvt.wu.s a0, fa0\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0, simulator.CallI(buffer, static_cast<float>(-42)));
  EXPECT_EQ(0, simulator.CallI(buffer, static_cast<float>(0)));
  EXPECT_EQ(42, simulator.CallI(buffer, static_cast<float>(42)));
  EXPECT_EQ(sign_extend(0),
            simulator.CallI(buffer, static_cast<float>(kMinInt32)));
  // float loss of precision
  EXPECT_EQ(-2147483648,
            simulator.CallI(buffer, static_cast<float>(kMaxInt32)));
  EXPECT_EQ(sign_extend(kMaxUint32),
            simulator.CallI(buffer, static_cast<float>(kMaxUint32)));
  EXPECT_EQ(sign_extend(0),
            simulator.CallI(buffer, static_cast<float>(kMinInt64)));
  EXPECT_EQ(sign_extend(kMaxUint32),
            simulator.CallI(buffer, static_cast<float>(kMaxInt64)));
  EXPECT_EQ(sign_extend(kMaxUint32),
            simulator.CallI(buffer, static_cast<float>(kMaxUint64)));
  EXPECT_EQ(sign_extend(0),
            simulator.CallI(buffer, -std::numeric_limits<float>::infinity()));
  EXPECT_EQ(sign_extend(kMaxUint32),
            simulator.CallI(buffer, std::numeric_limits<float>::infinity()));
  EXPECT_EQ(
      sign_extend(kMaxUint32),
      simulator.CallI(buffer, std::numeric_limits<float>::signaling_NaN()));
}

UNIT_TEST(ConvertWordToSingle) {
  Assembler assembler(RV_G);
  __ fcvtsw(FA0, A0);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  d0050553 fcvt.s.w fa0, a0\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(-42.0f, simulator.CallF(buffer, sign_extend(-42)));
  EXPECT_EQ(0.0f, simulator.CallF(buffer, sign_extend(0)));
  EXPECT_EQ(42.0f, simulator.CallF(buffer, sign_extend(42)));
  EXPECT_EQ(static_cast<float>(kMinInt32),
            simulator.CallF(buffer, sign_extend(kMinInt32)));
  EXPECT_EQ(static_cast<float>(kMaxInt32),
            simulator.CallF(buffer, sign_extend(kMaxInt32)));
  EXPECT_EQ(-1.0f, simulator.CallF(buffer, sign_extend(kMaxUint32)));
}

UNIT_TEST(ConvertUnsignedWordToSingle) {
  Assembler assembler(RV_G);
  __ fcvtswu(FA0, A0);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  d0150553 fcvt.s.wu fa0, a0\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(
      static_cast<float>(static_cast<uint32_t>(static_cast<int32_t>(-42))),
      simulator.CallF(buffer, sign_extend(-42)));
  EXPECT_EQ(0.0f, simulator.CallF(buffer, sign_extend(0)));
  EXPECT_EQ(42.0f, simulator.CallF(buffer, sign_extend(42)));
  EXPECT_EQ(static_cast<float>(static_cast<uint32_t>(kMinInt32)),
            simulator.CallF(buffer, sign_extend(kMinInt32)));
  EXPECT_EQ(static_cast<float>(kMaxInt32),
            simulator.CallF(buffer, sign_extend(kMaxInt32)));
  EXPECT_EQ(static_cast<float>(kMaxUint32),
            simulator.CallF(buffer, sign_extend(kMaxUint32)));
}

UNIT_TEST(SingleMove) {
  Assembler assembler(RV_G);
  __ fmvs(FA0, FA1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  20b58553 fmv.s fa0, fa1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(36.0f, simulator.CallF(buffer, 42.0f, 36.0f));
  EXPECT_EQ(std::numeric_limits<float>::infinity(),
            simulator.CallF(buffer, -std::numeric_limits<float>::infinity(),
                            std::numeric_limits<float>::infinity()));
}

UNIT_TEST(SingleAbsoluteValue) {
  Assembler assembler(RV_G);
  __ fabss(FA0, FA0);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  20a52553 fabs.s fa0, fa0\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0.0f, simulator.CallF(buffer, 0.0f));
  EXPECT_EQ(0.0f, simulator.CallF(buffer, -0.0f));
  EXPECT_EQ(42.0f, simulator.CallF(buffer, 42.0f));
  EXPECT_EQ(42.0f, simulator.CallF(buffer, -42.0f));
  EXPECT_EQ(std::numeric_limits<float>::infinity(),
            simulator.CallF(buffer, std::numeric_limits<float>::infinity()));
  EXPECT_EQ(std::numeric_limits<float>::infinity(),
            simulator.CallF(buffer, -std::numeric_limits<float>::infinity()));
}

UNIT_TEST(SingleNegate) {
  Assembler assembler(RV_G);
  __ fnegs(FA0, FA0);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  20a51553 fneg.s fa0, fa0\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(-0.0f, simulator.CallF(buffer, 0.0f));
  EXPECT_EQ(0.0f, simulator.CallF(buffer, -0.0f));
  EXPECT_EQ(-42.0f, simulator.CallF(buffer, 42.0f));
  EXPECT_EQ(42.0f, simulator.CallF(buffer, -42.0f));
  EXPECT_EQ(-std::numeric_limits<float>::infinity(),
            simulator.CallF(buffer, std::numeric_limits<float>::infinity()));
  EXPECT_EQ(std::numeric_limits<float>::infinity(),
            simulator.CallF(buffer, -std::numeric_limits<float>::infinity()));
}

UNIT_TEST(BitCastSingleToInteger) {
  Assembler assembler(RV_G);
  __ fmvxw(A0, FA0);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  e0050553 fmv.x.w a0, fa0\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(bit_cast<int32_t>(0.0f), simulator.CallI(buffer, 0.0f));
  EXPECT_EQ(bit_cast<int32_t>(-0.0f), simulator.CallI(buffer, -0.0f));
  EXPECT_EQ(bit_cast<int32_t>(42.0f), simulator.CallI(buffer, 42.0f));
  EXPECT_EQ(bit_cast<int32_t>(-42.0f), simulator.CallI(buffer, -42.0f));
  EXPECT_EQ(bit_cast<int32_t>(std::numeric_limits<float>::quiet_NaN()),
            simulator.CallI(buffer, std::numeric_limits<float>::quiet_NaN()));
  EXPECT_EQ(
      bit_cast<int32_t>(std::numeric_limits<float>::signaling_NaN()),
      simulator.CallI(buffer, std::numeric_limits<float>::signaling_NaN()));
  EXPECT_EQ(bit_cast<int32_t>(std::numeric_limits<float>::infinity()),
            simulator.CallI(buffer, std::numeric_limits<float>::infinity()));
  EXPECT_EQ(bit_cast<int32_t>(-std::numeric_limits<float>::infinity()),
            simulator.CallI(buffer, -std::numeric_limits<float>::infinity()));
}

UNIT_TEST(BitCastIntegerToSingle) {
  Assembler assembler(RV_G);
  __ fmvwx(FA0, A0);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  f0050553 fmv.w.x fa0, a0\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0.0f,
            simulator.CallF(buffer, sign_extend(bit_cast<int32_t>(0.0f))));
  EXPECT_EQ(-0.0f,
            simulator.CallF(buffer, sign_extend(bit_cast<int32_t>(-0.0f))));
  EXPECT_EQ(42.0f,
            simulator.CallF(buffer, sign_extend(bit_cast<int32_t>(42.0f))));
  EXPECT_EQ(-42.0f,
            simulator.CallF(buffer, sign_extend(bit_cast<int32_t>(-42.0f))));
  EXPECT_EQ(true, isnan(simulator.CallF(
                      buffer, sign_extend(bit_cast<int32_t>(
                                  std::numeric_limits<float>::quiet_NaN())))));
  EXPECT_EQ(true,
            isnan(simulator.CallF(
                buffer, sign_extend(bit_cast<int32_t>(
                            std::numeric_limits<float>::signaling_NaN())))));
  EXPECT_EQ(
      std::numeric_limits<float>::infinity(),
      simulator.CallF(buffer, sign_extend(bit_cast<int32_t>(
                                  std::numeric_limits<float>::infinity()))));
  EXPECT_EQ(
      -std::numeric_limits<float>::infinity(),
      simulator.CallF(buffer, sign_extend(bit_cast<int32_t>(
                                  -std::numeric_limits<float>::infinity()))));
}

#if XLEN >= 64
UNIT_TEST(ConvertSingleToDoubleWord) {
  Assembler assembler(RV_G);
  __ fcvtls(A0, FA0);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  c0250553 fcvt.l.s a0, fa0\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(-42, simulator.CallI(buffer, static_cast<float>(-42)));
  EXPECT_EQ(0, simulator.CallI(buffer, static_cast<float>(0)));
  EXPECT_EQ(42, simulator.CallI(buffer, static_cast<float>(42)));
  EXPECT_EQ(static_cast<int64_t>(kMinInt32),
            simulator.CallI(buffer, static_cast<float>(kMinInt32)));
  // float loses precision:
  EXPECT_EQ(static_cast<int64_t>(kMaxInt32) + 1,
            simulator.CallI(buffer, static_cast<float>(kMaxInt32)));
  EXPECT_EQ(static_cast<int64_t>(kMaxUint32) + 1,
            simulator.CallI(buffer, static_cast<float>(kMaxUint32)));
  EXPECT_EQ(kMinInt64, simulator.CallI(buffer, static_cast<float>(kMinInt64)));
  EXPECT_EQ(kMaxInt64, simulator.CallI(buffer, static_cast<float>(kMaxInt64)));
  EXPECT_EQ(kMaxInt64, simulator.CallI(buffer, static_cast<float>(kMaxUint64)));
  EXPECT_EQ(kMinInt64,
            simulator.CallI(buffer, -std::numeric_limits<float>::infinity()));
  EXPECT_EQ(kMaxInt64,
            simulator.CallI(buffer, std::numeric_limits<float>::infinity()));
  EXPECT_EQ(
      kMaxInt64,
      simulator.CallI(buffer, std::numeric_limits<float>::signaling_NaN()));
}

UNIT_TEST(ConvertSingleToUnsignedDoubleWord) {
  Assembler assembler(RV_G);
  __ fcvtlus(A0, FA0);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  c0350553 fcvt.lu.s a0, fa0\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0, simulator.CallI(buffer, static_cast<float>(-42)));
  EXPECT_EQ(0, simulator.CallI(buffer, static_cast<float>(0)));
  EXPECT_EQ(42, simulator.CallI(buffer, static_cast<float>(42)));
  EXPECT_EQ(static_cast<int64_t>(static_cast<uint64_t>(0)),
            simulator.CallI(buffer, static_cast<float>(kMinInt32)));
  EXPECT_EQ(static_cast<int64_t>(static_cast<uint64_t>(kMaxInt32) + 1),
            simulator.CallI(buffer, static_cast<float>(kMaxInt32)));
  EXPECT_EQ(static_cast<int64_t>(static_cast<uint64_t>(kMaxUint32) + 1),
            simulator.CallI(buffer, static_cast<float>(kMaxUint32)));
  EXPECT_EQ(static_cast<int64_t>(static_cast<uint64_t>(0)),
            simulator.CallI(buffer, static_cast<float>(kMinInt64)));
  EXPECT_EQ(static_cast<int64_t>(static_cast<uint64_t>(kMaxInt64) + 1),
            simulator.CallI(buffer, static_cast<float>(kMaxInt64)));
  EXPECT_EQ(static_cast<int64_t>(static_cast<uint64_t>(kMaxUint64)),
            simulator.CallI(buffer, static_cast<float>(kMaxUint64)));
  EXPECT_EQ(static_cast<int64_t>(static_cast<uint64_t>(0)),
            simulator.CallI(buffer, -std::numeric_limits<float>::infinity()));
  EXPECT_EQ(static_cast<int64_t>(static_cast<uint64_t>(kMaxUint64)),
            simulator.CallI(buffer, std::numeric_limits<float>::infinity()));
  EXPECT_EQ(
      static_cast<int64_t>(static_cast<uint64_t>(kMaxUint64)),
      simulator.CallI(buffer, std::numeric_limits<float>::signaling_NaN()));
}

UNIT_TEST(ConvertDoubleWordToSingle) {
  Assembler assembler(RV_G);
  __ fcvtsl(FA0, A0);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  d0250553 fcvt.s.l fa0, a0\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0.0f, simulator.CallF(buffer, sign_extend(0)));
  EXPECT_EQ(42.0f, simulator.CallF(buffer, sign_extend(42)));
  EXPECT_EQ(-42.0f, simulator.CallF(buffer, sign_extend(-42)));
  EXPECT_EQ(static_cast<float>(kMinInt32),
            simulator.CallF(buffer, sign_extend(kMinInt32)));
  EXPECT_EQ(static_cast<float>(kMaxInt32),
            simulator.CallF(buffer, sign_extend(kMaxInt32)));
  EXPECT_EQ(static_cast<float>(sign_extend(kMaxUint32)),
            simulator.CallF(buffer, sign_extend(kMaxUint32)));
  EXPECT_EQ(static_cast<float>(kMinInt64),
            simulator.CallF(buffer, sign_extend(kMinInt64)));
  EXPECT_EQ(static_cast<float>(kMaxInt64),
            simulator.CallF(buffer, sign_extend(kMaxInt64)));
  EXPECT_EQ(static_cast<float>(sign_extend(kMaxUint64)),
            simulator.CallF(buffer, sign_extend(kMaxUint64)));
}

UNIT_TEST(ConvertUnsignedDoubleWordToSingle) {
  Assembler assembler(RV_G);
  __ fcvtslu(FA0, A0);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  d0350553 fcvt.s.lu fa0, a0\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0.0f, simulator.CallF(buffer, sign_extend(0)));
  EXPECT_EQ(42.0f, simulator.CallF(buffer, sign_extend(42)));
  EXPECT_EQ(static_cast<float>(static_cast<uint64_t>(sign_extend(-42))),
            simulator.CallF(buffer, sign_extend(-42)));
  EXPECT_EQ(static_cast<float>(static_cast<uint64_t>(sign_extend(kMinInt32))),
            simulator.CallF(buffer, sign_extend(kMinInt32)));
  EXPECT_EQ(static_cast<float>(static_cast<uint64_t>(sign_extend(kMaxInt32))),
            simulator.CallF(buffer, sign_extend(kMaxInt32)));
  EXPECT_EQ(static_cast<float>(static_cast<uint64_t>(sign_extend(kMaxUint32))),
            simulator.CallF(buffer, sign_extend(kMaxUint32)));
  EXPECT_EQ(static_cast<float>(static_cast<uint64_t>(sign_extend(kMinInt64))),
            simulator.CallF(buffer, sign_extend(kMinInt64)));
  EXPECT_EQ(static_cast<float>(static_cast<uint64_t>(sign_extend(kMaxInt64))),
            simulator.CallF(buffer, sign_extend(kMaxInt64)));
  EXPECT_EQ(static_cast<float>(kMaxUint64),
            simulator.CallF(buffer, sign_extend(kMaxUint64)));
}
#endif

UNIT_TEST(LoadDoubleFloat) {
  Assembler assembler(RV_G);
  __ fld(FA0, Address(A0, 1 * sizeof(double)));
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  00853507 fld fa0, 8(a0)\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  double* data =
      reinterpret_cast<double*>(Memory::Allocate(3 * sizeof(double)));
  data[0] = 1.7;
  data[1] = 2.8;
  data[2] = 3.9;

  Simulator simulator;
  EXPECT_EQ(data[1], simulator.CallD(buffer, Memory::ToGuest(data)));
}

UNIT_TEST(StoreDoubleFloat) {
  Assembler assembler(RV_G);
  __ fsd(FA0, Address(A0, 1 * sizeof(double)));
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  00a53427 fsd fa0, 8(a0)\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  double* data =
      reinterpret_cast<double*>(Memory::Allocate(3 * sizeof(double)));
  data[0] = 1.7;
  data[1] = 2.8;
  data[2] = 3.9;

  Simulator simulator;
  simulator.CallD(buffer, Memory::ToGuest(data), 4.2);
  EXPECT_EQ(4.2, data[1]);
}

UNIT_TEST(DoubleMultiplyAdd) {
  Assembler assembler(RV_G);
  __ fmaddd(FA0, FA0, FA1, FA2);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  62b50543 fmadd.d fa0, fa0, fa1, fa2\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(22.0, simulator.CallD(buffer, 3.0, 5.0, 7.0));
  EXPECT_EQ(-8.0, simulator.CallD(buffer, -3.0, 5.0, 7.0));
  EXPECT_EQ(-8.0, simulator.CallD(buffer, 3.0, -5.0, 7.0));
  EXPECT_EQ(8.0, simulator.CallD(buffer, 3.0, 5.0, -7.0));

  EXPECT_EQ(26.0, simulator.CallD(buffer, 7.0, 3.0, 5.0));
  EXPECT_EQ(-16.0, simulator.CallD(buffer, -7.0, 3.0, 5.0));
  EXPECT_EQ(-16.0, simulator.CallD(buffer, 7.0, -3.0, 5.0));
  EXPECT_EQ(16.0, simulator.CallD(buffer, 7.0, 3.0, -5.0));
}

UNIT_TEST(DoubleMultiplySubtract) {
  Assembler assembler(RV_G);
  __ fmsubd(FA0, FA0, FA1, FA2);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  62b50547 fmsub.d fa0, fa0, fa1, fa2\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(8.0, simulator.CallD(buffer, 3.0, 5.0, 7.0));
  EXPECT_EQ(-22.0, simulator.CallD(buffer, -3.0, 5.0, 7.0));
  EXPECT_EQ(-22.0, simulator.CallD(buffer, 3.0, -5.0, 7.0));
  EXPECT_EQ(22.0, simulator.CallD(buffer, 3.0, 5.0, -7.0));

  EXPECT_EQ(16.0, simulator.CallD(buffer, 7.0, 3.0, 5.0));
  EXPECT_EQ(-26.0, simulator.CallD(buffer, -7.0, 3.0, 5.0));
  EXPECT_EQ(-26.0, simulator.CallD(buffer, 7.0, -3.0, 5.0));
  EXPECT_EQ(26.0, simulator.CallD(buffer, 7.0, 3.0, -5.0));
}

UNIT_TEST(DoubleNegateMultiplySubtract) {
  Assembler assembler(RV_G);
  __ fnmsubd(FA0, FA0, FA1, FA2);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  62b5054b fnmsub.d fa0, fa0, fa1, fa2\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(-8.0, simulator.CallD(buffer, 3.0, 5.0, 7.0));
  EXPECT_EQ(22.0, simulator.CallD(buffer, -3.0, 5.0, 7.0));
  EXPECT_EQ(22.0, simulator.CallD(buffer, 3.0, -5.0, 7.0));
  EXPECT_EQ(-22.0, simulator.CallD(buffer, 3.0, 5.0, -7.0));

  EXPECT_EQ(-16.0, simulator.CallD(buffer, 7.0, 3.0, 5.0));
  EXPECT_EQ(26.0, simulator.CallD(buffer, -7.0, 3.0, 5.0));
  EXPECT_EQ(26.0, simulator.CallD(buffer, 7.0, -3.0, 5.0));
  EXPECT_EQ(-26.0, simulator.CallD(buffer, 7.0, 3.0, -5.0));
}

UNIT_TEST(DoubleNegateMultiplyAdd) {
  Assembler assembler(RV_G);
  __ fnmaddd(FA0, FA0, FA1, FA2);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  62b5054f fnmadd.d fa0, fa0, fa1, fa2\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(-22.0, simulator.CallD(buffer, 3.0, 5.0, 7.0));
  EXPECT_EQ(8.0, simulator.CallD(buffer, -3.0, 5.0, 7.0));
  EXPECT_EQ(8.0, simulator.CallD(buffer, 3.0, -5.0, 7.0));
  EXPECT_EQ(-8.0, simulator.CallD(buffer, 3.0, 5.0, -7.0));

  EXPECT_EQ(-26.0, simulator.CallD(buffer, 7.0, 3.0, 5.0));
  EXPECT_EQ(16.0, simulator.CallD(buffer, -7.0, 3.0, 5.0));
  EXPECT_EQ(16.0, simulator.CallD(buffer, 7.0, -3.0, 5.0));
  EXPECT_EQ(-16.0, simulator.CallD(buffer, 7.0, 3.0, -5.0));
}

UNIT_TEST(DoubleAdd) {
  Assembler assembler(RV_G);
  __ faddd(FA0, FA0, FA1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  02b50553 fadd.d fa0, fa0, fa1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(8.0, simulator.CallD(buffer, 3.0, 5.0));
  EXPECT_EQ(2.0, simulator.CallD(buffer, -3.0, 5.0));
  EXPECT_EQ(-2.0, simulator.CallD(buffer, 3.0, -5.0));
  EXPECT_EQ(-8.0, simulator.CallD(buffer, -3.0, -5.0));

  EXPECT_EQ(10.0, simulator.CallD(buffer, 7.0, 3.0));
  EXPECT_EQ(-4.0, simulator.CallD(buffer, -7.0, 3.0));
  EXPECT_EQ(4.0, simulator.CallD(buffer, 7.0, -3.0));
  EXPECT_EQ(-10.0, simulator.CallD(buffer, -7.0, -3.0));
}

UNIT_TEST(DoubleSubtract) {
  Assembler assembler(RV_G);
  __ fsubd(FA0, FA0, FA1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  0ab50553 fsub.d fa0, fa0, fa1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(-2.0, simulator.CallD(buffer, 3.0, 5.0));
  EXPECT_EQ(-8.0, simulator.CallD(buffer, -3.0, 5.0));
  EXPECT_EQ(8.0, simulator.CallD(buffer, 3.0, -5.0));
  EXPECT_EQ(2.0, simulator.CallD(buffer, -3.0, -5.0));

  EXPECT_EQ(4.0, simulator.CallD(buffer, 7.0, 3.0));
  EXPECT_EQ(-10.0, simulator.CallD(buffer, -7.0, 3.0));
  EXPECT_EQ(10.0, simulator.CallD(buffer, 7.0, -3.0));
  EXPECT_EQ(-4.0, simulator.CallD(buffer, -7.0, -3.0));
}

UNIT_TEST(DoubleMultiply) {
  Assembler assembler(RV_G);
  __ fmuld(FA0, FA0, FA1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  12b50553 fmul.d fa0, fa0, fa1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(15.0, simulator.CallD(buffer, 3.0, 5.0));
  EXPECT_EQ(-15.0, simulator.CallD(buffer, -3.0, 5.0));
  EXPECT_EQ(-15.0, simulator.CallD(buffer, 3.0, -5.0));
  EXPECT_EQ(15.0, simulator.CallD(buffer, -3.0, -5.0));

  EXPECT_EQ(21.0, simulator.CallD(buffer, 7.0, 3.0));
  EXPECT_EQ(-21.0, simulator.CallD(buffer, -7.0, 3.0));
  EXPECT_EQ(-21.0, simulator.CallD(buffer, 7.0, -3.0));
  EXPECT_EQ(21.0, simulator.CallD(buffer, -7.0, -3.0));
}

UNIT_TEST(DoubleDivide) {
  Assembler assembler(RV_G);
  __ fdivd(FA0, FA0, FA1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  1ab50553 fdiv.d fa0, fa0, fa1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(2.0, simulator.CallD(buffer, 10.0, 5.0));
  EXPECT_EQ(-2.0, simulator.CallD(buffer, -10.0, 5.0));
  EXPECT_EQ(-2.0, simulator.CallD(buffer, 10.0, -5.0));
  EXPECT_EQ(2.0, simulator.CallD(buffer, -10.0, -5.0));
}

UNIT_TEST(DoubleSquareRoot) {
  Assembler assembler(RV_G);
  __ fsqrtd(FA0, FA0);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  5a050553 fsqrt.d fa0, fa0\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0.0, simulator.CallD(buffer, 0.0));
  EXPECT_EQ(1.0, simulator.CallD(buffer, 1.0));
  EXPECT_EQ(2.0, simulator.CallD(buffer, 4.0));
  EXPECT_EQ(3.0, simulator.CallD(buffer, 9.0));
}

UNIT_TEST(DoubleSignInject) {
  Assembler assembler(RV_G);
  __ fsgnjd(FA0, FA0, FA1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  22b50553 fsgnj.d fa0, fa0, fa1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(3.0, simulator.CallD(buffer, 3.0, 5.0));
  EXPECT_EQ(3.0, simulator.CallD(buffer, -3.0, 5.0));
  EXPECT_EQ(-3.0, simulator.CallD(buffer, 3.0, -5.0));
  EXPECT_EQ(-3.0, simulator.CallD(buffer, -3.0, -5.0));
}

UNIT_TEST(DoubleNegatedSignInject) {
  Assembler assembler(RV_G);
  __ fsgnjnd(FA0, FA0, FA1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  22b51553 fsgnjn.d fa0, fa0, fa1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(-3.0, simulator.CallD(buffer, 3.0, 5.0));
  EXPECT_EQ(-3.0, simulator.CallD(buffer, -3.0, 5.0));
  EXPECT_EQ(3.0, simulator.CallD(buffer, 3.0, -5.0));
  EXPECT_EQ(3.0, simulator.CallD(buffer, -3.0, -5.0));
}

UNIT_TEST(DoubleXorSignInject) {
  Assembler assembler(RV_G);
  __ fsgnjxd(FA0, FA0, FA1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  22b52553 fsgnjx.d fa0, fa0, fa1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(3.0, simulator.CallD(buffer, 3.0, 5.0));
  EXPECT_EQ(-3.0, simulator.CallD(buffer, -3.0, 5.0));
  EXPECT_EQ(-3.0, simulator.CallD(buffer, 3.0, -5.0));
  EXPECT_EQ(3.0, simulator.CallD(buffer, -3.0, -5.0));
}

UNIT_TEST(DoubleMin) {
  Assembler assembler(RV_G);
  __ fmind(FA0, FA0, FA1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  2ab50553 fmin.d fa0, fa0, fa1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(1.0, simulator.CallD(buffer, 3.0, 1.0));
  EXPECT_EQ(3.0, simulator.CallD(buffer, 3.0, 3.0));
  EXPECT_EQ(3.0, simulator.CallD(buffer, 3.0, 5.0));
  EXPECT_EQ(-1.0, simulator.CallD(buffer, 3.0, -1.0));
  EXPECT_EQ(-3.0, simulator.CallD(buffer, 3.0, -3.0));
  EXPECT_EQ(-5.0, simulator.CallD(buffer, 3.0, -5.0));
  EXPECT_EQ(-3.0, simulator.CallD(buffer, -3.0, 1.0));
  EXPECT_EQ(-3.0, simulator.CallD(buffer, -3.0, 3.0));
  EXPECT_EQ(-3.0, simulator.CallD(buffer, -3.0, 5.0));
  EXPECT_EQ(-3.0, simulator.CallD(buffer, -3.0, -1.0));
  EXPECT_EQ(-3.0, simulator.CallD(buffer, -3.0, -3.0));
  EXPECT_EQ(-5.0, simulator.CallD(buffer, -3.0, -5.0));

  EXPECT_BITEQ(-0.0, simulator.CallD(buffer, 0.0, -0.0));
  EXPECT_BITEQ(-0.0, simulator.CallD(buffer, -0.0, 0.0));

  double qNAN = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(3.0, simulator.CallD(buffer, 3.0, qNAN));
  EXPECT_EQ(3.0, simulator.CallD(buffer, qNAN, 3.0));
  EXPECT_EQ(-3.0, simulator.CallD(buffer, -3.0, qNAN));
  EXPECT_EQ(-3.0, simulator.CallD(buffer, qNAN, -3.0));

  double sNAN = std::numeric_limits<double>::signaling_NaN();
  EXPECT_EQ(3.0, simulator.CallD(buffer, 3.0, sNAN));
  EXPECT_EQ(3.0, simulator.CallD(buffer, sNAN, 3.0));
  EXPECT_EQ(-3.0, simulator.CallD(buffer, -3.0, sNAN));
  EXPECT_EQ(-3.0, simulator.CallD(buffer, sNAN, -3.0));

  EXPECT_BITEQ(qNAN, simulator.CallD(buffer, qNAN, qNAN));
  EXPECT_BITEQ(qNAN, simulator.CallD(buffer, sNAN, sNAN));
  EXPECT_BITEQ(qNAN, simulator.CallD(buffer, qNAN, sNAN));
  EXPECT_BITEQ(qNAN, simulator.CallD(buffer, sNAN, qNAN));
}

UNIT_TEST(DoubleMax) {
  Assembler assembler(RV_G);
  __ fmaxd(FA0, FA0, FA1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  2ab51553 fmax.d fa0, fa0, fa1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(3.0, simulator.CallD(buffer, 3.0, 1.0));
  EXPECT_EQ(3.0, simulator.CallD(buffer, 3.0, 3.0));
  EXPECT_EQ(5.0, simulator.CallD(buffer, 3.0, 5.0));
  EXPECT_EQ(3.0, simulator.CallD(buffer, 3.0, -1.0));
  EXPECT_EQ(3.0, simulator.CallD(buffer, 3.0, -3.0));
  EXPECT_EQ(3.0, simulator.CallD(buffer, 3.0, -5.0));
  EXPECT_EQ(1.0, simulator.CallD(buffer, -3.0, 1.0));
  EXPECT_EQ(3.0, simulator.CallD(buffer, -3.0, 3.0));
  EXPECT_EQ(5.0, simulator.CallD(buffer, -3.0, 5.0));
  EXPECT_EQ(-1.0, simulator.CallD(buffer, -3.0, -1.0));
  EXPECT_EQ(-3.0, simulator.CallD(buffer, -3.0, -3.0));
  EXPECT_EQ(-3.0, simulator.CallD(buffer, -3.0, -5.0));

  EXPECT_BITEQ(0.0, simulator.CallD(buffer, 0.0, -0.0));
  EXPECT_BITEQ(0.0, simulator.CallD(buffer, -0.0, 0.0));

  double qNAN = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(3.0, simulator.CallD(buffer, 3.0, qNAN));
  EXPECT_EQ(3.0, simulator.CallD(buffer, qNAN, 3.0));
  EXPECT_EQ(-3.0, simulator.CallD(buffer, -3.0, qNAN));
  EXPECT_EQ(-3.0, simulator.CallD(buffer, qNAN, -3.0));

  double sNAN = std::numeric_limits<double>::signaling_NaN();
  EXPECT_EQ(3.0, simulator.CallD(buffer, 3.0, sNAN));
  EXPECT_EQ(3.0, simulator.CallD(buffer, sNAN, 3.0));
  EXPECT_EQ(-3.0, simulator.CallD(buffer, -3.0, sNAN));
  EXPECT_EQ(-3.0, simulator.CallD(buffer, sNAN, -3.0));

  EXPECT_BITEQ(qNAN, simulator.CallD(buffer, qNAN, qNAN));
  EXPECT_BITEQ(qNAN, simulator.CallD(buffer, sNAN, sNAN));
  EXPECT_BITEQ(qNAN, simulator.CallD(buffer, qNAN, sNAN));
  EXPECT_BITEQ(qNAN, simulator.CallD(buffer, sNAN, qNAN));
}

UNIT_TEST(DoubleToSingle) {
  Assembler assembler(RV_G);
  __ fcvtsd(FA0, FA0);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  40150553 fcvt.s.d fa0, fa0\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0.0f, simulator.CallF(buffer, 0.0));
  EXPECT_EQ(42.0f, simulator.CallF(buffer, 42.0));
  EXPECT_EQ(-42.0f, simulator.CallF(buffer, -42.0));
  EXPECT_EQ(true, isnan(simulator.CallF(
                      buffer, std::numeric_limits<double>::quiet_NaN())));
  EXPECT_EQ(true, isnan(simulator.CallF(
                      buffer, std::numeric_limits<double>::signaling_NaN())));
  EXPECT_EQ(std::numeric_limits<float>::infinity(),
            simulator.CallF(buffer, std::numeric_limits<double>::infinity()));
  EXPECT_EQ(-std::numeric_limits<float>::infinity(),
            simulator.CallF(buffer, -std::numeric_limits<double>::infinity()));
}

UNIT_TEST(SingleToDouble) {
  Assembler assembler(RV_G);
  __ fcvtds(FA0, FA0);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  42050553 fcvt.d.s fa0, fa0\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0.0, simulator.CallD(buffer, 0.0f));
  EXPECT_EQ(42.0, simulator.CallD(buffer, 42.0f));
  EXPECT_EQ(-42.0, simulator.CallD(buffer, -42.0f));
  EXPECT_EQ(true, isnan(simulator.CallD(
                      buffer, std::numeric_limits<float>::quiet_NaN())));
  EXPECT_EQ(true, isnan(simulator.CallD(
                      buffer, std::numeric_limits<float>::signaling_NaN())));
  EXPECT_EQ(std::numeric_limits<double>::infinity(),
            simulator.CallD(buffer, std::numeric_limits<float>::infinity()));
  EXPECT_EQ(-std::numeric_limits<double>::infinity(),
            simulator.CallD(buffer, -std::numeric_limits<float>::infinity()));
}

UNIT_TEST(NaNBoxing) {
  Assembler assembler(RV_G);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ("  00008067 ret\n", disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(true, isnan(simulator.CallD(buffer, 42.0f)));
}

UNIT_TEST(DoubleEqual) {
  Assembler assembler(RV_G);
  __ feqd(A0, FA0, FA1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  a2b52553 feq.d a0, fa0, fa1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0, simulator.CallI(buffer, 3.0, 1.0));
  EXPECT_EQ(1, simulator.CallI(buffer, 3.0, 3.0));
  EXPECT_EQ(0, simulator.CallI(buffer, 3.0, 5.0));
  EXPECT_EQ(0, simulator.CallI(buffer, 3.0, -1.0));
  EXPECT_EQ(0, simulator.CallI(buffer, 3.0, -3.0));
  EXPECT_EQ(0, simulator.CallI(buffer, 3.0, -5.0));
  EXPECT_EQ(0, simulator.CallI(buffer, -3.0, 1.0));
  EXPECT_EQ(0, simulator.CallI(buffer, -3.0, 3.0));
  EXPECT_EQ(0, simulator.CallI(buffer, -3.0, 5.0));
  EXPECT_EQ(0, simulator.CallI(buffer, -3.0, -1.0));
  EXPECT_EQ(1, simulator.CallI(buffer, -3.0, -3.0));
  EXPECT_EQ(0, simulator.CallI(buffer, -3.0, -5.0));

  double qNAN = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(0, simulator.CallI(buffer, 3.0, qNAN));
  EXPECT_EQ(0, simulator.CallI(buffer, qNAN, 3.0));
  EXPECT_EQ(0, simulator.CallI(buffer, -3.0, qNAN));
  EXPECT_EQ(0, simulator.CallI(buffer, qNAN, -3.0));
}

UNIT_TEST(DoubleLessThan) {
  Assembler assembler(RV_G);
  __ fltd(A0, FA0, FA1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  a2b51553 flt.d a0, fa0, fa1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0, simulator.CallI(buffer, 3.0, 1.0));
  EXPECT_EQ(0, simulator.CallI(buffer, 3.0, 3.0));
  EXPECT_EQ(1, simulator.CallI(buffer, 3.0, 5.0));
  EXPECT_EQ(0, simulator.CallI(buffer, 3.0, -1.0));
  EXPECT_EQ(0, simulator.CallI(buffer, 3.0, -3.0));
  EXPECT_EQ(0, simulator.CallI(buffer, 3.0, -5.0));
  EXPECT_EQ(1, simulator.CallI(buffer, -3.0, 1.0));
  EXPECT_EQ(1, simulator.CallI(buffer, -3.0, 3.0));
  EXPECT_EQ(1, simulator.CallI(buffer, -3.0, 5.0));
  EXPECT_EQ(1, simulator.CallI(buffer, -3.0, -1.0));
  EXPECT_EQ(0, simulator.CallI(buffer, -3.0, -3.0));
  EXPECT_EQ(0, simulator.CallI(buffer, -3.0, -5.0));

  double qNAN = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(0, simulator.CallI(buffer, 3.0, qNAN));
  EXPECT_EQ(0, simulator.CallI(buffer, qNAN, 3.0));
  EXPECT_EQ(0, simulator.CallI(buffer, -3.0, qNAN));
  EXPECT_EQ(0, simulator.CallI(buffer, qNAN, -3.0));
}

UNIT_TEST(DoubleLessOrEqual) {
  Assembler assembler(RV_G);
  __ fled(A0, FA0, FA1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  a2b50553 fle.d a0, fa0, fa1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0, simulator.CallI(buffer, 3.0, 1.0));
  EXPECT_EQ(1, simulator.CallI(buffer, 3.0, 3.0));
  EXPECT_EQ(1, simulator.CallI(buffer, 3.0, 5.0));
  EXPECT_EQ(0, simulator.CallI(buffer, 3.0, -1.0));
  EXPECT_EQ(0, simulator.CallI(buffer, 3.0, -3.0));
  EXPECT_EQ(0, simulator.CallI(buffer, 3.0, -5.0));
  EXPECT_EQ(1, simulator.CallI(buffer, -3.0, 1.0));
  EXPECT_EQ(1, simulator.CallI(buffer, -3.0, 3.0));
  EXPECT_EQ(1, simulator.CallI(buffer, -3.0, 5.0));
  EXPECT_EQ(1, simulator.CallI(buffer, -3.0, -1.0));
  EXPECT_EQ(1, simulator.CallI(buffer, -3.0, -3.0));
  EXPECT_EQ(0, simulator.CallI(buffer, -3.0, -5.0));

  double qNAN = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(0, simulator.CallI(buffer, 3.0, qNAN));
  EXPECT_EQ(0, simulator.CallI(buffer, qNAN, 3.0));
  EXPECT_EQ(0, simulator.CallI(buffer, -3.0, qNAN));
  EXPECT_EQ(0, simulator.CallI(buffer, qNAN, -3.0));
}

UNIT_TEST(DoubleClassify) {
  Assembler assembler(RV_G);
  __ fclassd(A0, FA0);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  e2051553 fclass.d a0, fa0\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  // Neg infinity
  EXPECT_EQ(1 << 0,
            simulator.CallI(buffer, -std::numeric_limits<double>::infinity()));
  // Neg normal
  EXPECT_EQ(1 << 1, simulator.CallI(buffer, -1.0));
  // Neg subnormal
  EXPECT_EQ(1 << 2,
            simulator.CallI(buffer, -std::numeric_limits<double>::min() / 2.0));
  // Neg zero
  EXPECT_EQ(1 << 3, simulator.CallI(buffer, -0.0));
  // Pos zero
  EXPECT_EQ(1 << 4, simulator.CallI(buffer, 0.0));
  // Pos subnormal
  EXPECT_EQ(1 << 5,
            simulator.CallI(buffer, std::numeric_limits<double>::min() / 2.0));
  // Pos normal
  EXPECT_EQ(1 << 6, simulator.CallI(buffer, 1.0));
  // Pos infinity
  EXPECT_EQ(1 << 7,
            simulator.CallI(buffer, std::numeric_limits<double>::infinity()));
  // Signaling NaN
  EXPECT_EQ(1 << 8, simulator.CallI(
                        buffer, std::numeric_limits<double>::signaling_NaN()));
  // Queit NaN
  EXPECT_EQ(1 << 9,
            simulator.CallI(buffer, std::numeric_limits<double>::quiet_NaN()));
}

UNIT_TEST(ConvertDoubleToWord) {
  Assembler assembler(RV_G);
  __ fcvtwd(A0, FA0);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  c2050553 fcvt.w.d a0, fa0\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(-42, simulator.CallI(buffer, static_cast<double>(-42)));
  EXPECT_EQ(0, simulator.CallI(buffer, static_cast<double>(0)));
  EXPECT_EQ(42, simulator.CallI(buffer, static_cast<double>(42)));
  EXPECT_EQ(sign_extend(kMinInt32),
            simulator.CallI(buffer, static_cast<double>(kMinInt32)));
  EXPECT_EQ(sign_extend(kMaxInt32),
            simulator.CallI(buffer, static_cast<double>(kMaxInt32)));
  EXPECT_EQ(sign_extend(kMaxInt32),
            simulator.CallI(buffer, static_cast<double>(kMaxUint32)));
  EXPECT_EQ(sign_extend(kMinInt32),
            simulator.CallI(buffer, static_cast<double>(kMinInt64)));
  EXPECT_EQ(sign_extend(kMaxInt32),
            simulator.CallI(buffer, static_cast<double>(kMaxInt64)));
  EXPECT_EQ(sign_extend(kMaxInt32),
            simulator.CallI(buffer, static_cast<double>(kMaxUint64)));
  EXPECT_EQ(sign_extend(kMinInt32),
            simulator.CallI(buffer, -std::numeric_limits<double>::infinity()));
  EXPECT_EQ(sign_extend(kMaxInt32),
            simulator.CallI(buffer, std::numeric_limits<double>::infinity()));
  EXPECT_EQ(
      sign_extend(kMaxInt32),
      simulator.CallI(buffer, std::numeric_limits<double>::signaling_NaN()));
}

UNIT_TEST(ConvertDoubleToUnsignedWord) {
  Assembler assembler(RV_G);
  __ fcvtwud(A0, FA0);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  c2150553 fcvt.wu.d a0, fa0\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0, simulator.CallI(buffer, static_cast<double>(-42)));
  EXPECT_EQ(0, simulator.CallI(buffer, static_cast<double>(0)));
  EXPECT_EQ(42, simulator.CallI(buffer, static_cast<double>(42)));
  EXPECT_EQ(sign_extend(0),
            simulator.CallI(buffer, static_cast<double>(kMinInt32)));
  EXPECT_EQ(sign_extend(kMaxInt32),
            simulator.CallI(buffer, static_cast<double>(kMaxInt32)));
  EXPECT_EQ(sign_extend(kMaxUint32),
            simulator.CallI(buffer, static_cast<double>(kMaxUint32)));
  EXPECT_EQ(sign_extend(0),
            simulator.CallI(buffer, static_cast<double>(kMinInt64)));
  EXPECT_EQ(sign_extend(kMaxUint32),
            simulator.CallI(buffer, static_cast<double>(kMaxInt64)));
  EXPECT_EQ(sign_extend(kMaxUint32),
            simulator.CallI(buffer, static_cast<double>(kMaxUint64)));
  EXPECT_EQ(sign_extend(0),
            simulator.CallI(buffer, -std::numeric_limits<double>::infinity()));
  EXPECT_EQ(sign_extend(kMaxUint32),
            simulator.CallI(buffer, std::numeric_limits<double>::infinity()));
  EXPECT_EQ(
      sign_extend(kMaxUint32),
      simulator.CallI(buffer, std::numeric_limits<double>::signaling_NaN()));
}

UNIT_TEST(ConvertWordToDouble) {
  Assembler assembler(RV_G);
  __ fcvtdw(FA0, A0);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  d2050553 fcvt.d.w fa0, a0\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(-42.0, simulator.CallD(buffer, sign_extend(-42)));
  EXPECT_EQ(0.0, simulator.CallD(buffer, sign_extend(0)));
  EXPECT_EQ(42.0, simulator.CallD(buffer, sign_extend(42)));
  EXPECT_EQ(static_cast<double>(kMinInt32),
            simulator.CallD(buffer, sign_extend(kMinInt32)));
  EXPECT_EQ(static_cast<double>(kMaxInt32),
            simulator.CallD(buffer, sign_extend(kMaxInt32)));
  EXPECT_EQ(-1.0, simulator.CallD(buffer, sign_extend(kMaxUint32)));
}

UNIT_TEST(ConvertUnsignedWordToDouble) {
  Assembler assembler(RV_G);
  __ fcvtdwu(FA0, A0);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  d2150553 fcvt.d.wu fa0, a0\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(
      static_cast<double>(static_cast<uint32_t>(static_cast<int32_t>(-42))),
      simulator.CallD(buffer, sign_extend(-42)));
  EXPECT_EQ(0.0, simulator.CallD(buffer, sign_extend(0)));
  EXPECT_EQ(42.0, simulator.CallD(buffer, sign_extend(42)));
  EXPECT_EQ(static_cast<double>(static_cast<uint32_t>(kMinInt32)),
            simulator.CallD(buffer, sign_extend(kMinInt32)));
  EXPECT_EQ(static_cast<double>(kMaxInt32),
            simulator.CallD(buffer, sign_extend(kMaxInt32)));
  EXPECT_EQ(static_cast<double>(kMaxUint32),
            simulator.CallD(buffer, sign_extend(kMaxUint32)));
}

UNIT_TEST(DoubleMove) {
  Assembler assembler(RV_G);
  __ fmvd(FA0, FA1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  22b58553 fmv.d fa0, fa1\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(36.0, simulator.CallD(buffer, 42.0, 36.0));
  EXPECT_EQ(std::numeric_limits<double>::infinity(),
            simulator.CallD(buffer, -std::numeric_limits<double>::infinity(),
                            std::numeric_limits<double>::infinity()));
}

UNIT_TEST(DoubleAbsoluteValue) {
  Assembler assembler(RV_G);
  __ fabsd(FA0, FA0);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  22a52553 fabs.d fa0, fa0\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0.0, simulator.CallD(buffer, 0.0));
  EXPECT_EQ(0.0, simulator.CallD(buffer, -0.0));
  EXPECT_EQ(42.0, simulator.CallD(buffer, 42.0));
  EXPECT_EQ(42.0, simulator.CallD(buffer, -42.0));
  EXPECT_EQ(std::numeric_limits<double>::infinity(),
            simulator.CallD(buffer, std::numeric_limits<double>::infinity()));
  EXPECT_EQ(std::numeric_limits<double>::infinity(),
            simulator.CallD(buffer, -std::numeric_limits<double>::infinity()));
}

UNIT_TEST(DoubleNegate) {
  Assembler assembler(RV_G);
  __ fnegd(FA0, FA0);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  22a51553 fneg.d fa0, fa0\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(-0.0, simulator.CallD(buffer, 0.0));
  EXPECT_EQ(0.0, simulator.CallD(buffer, -0.0));
  EXPECT_EQ(-42.0, simulator.CallD(buffer, 42.0));
  EXPECT_EQ(42.0, simulator.CallD(buffer, -42.0));
  EXPECT_EQ(-std::numeric_limits<double>::infinity(),
            simulator.CallD(buffer, std::numeric_limits<double>::infinity()));
  EXPECT_EQ(std::numeric_limits<double>::infinity(),
            simulator.CallD(buffer, -std::numeric_limits<double>::infinity()));
}

#if XLEN >= 64
UNIT_TEST(ConvertDoubleToDoubleWord) {
  Assembler assembler(RV_G);
  __ fcvtld(A0, FA0);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  c2250553 fcvt.l.d a0, fa0\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(-42, simulator.CallI(buffer, static_cast<double>(-42)));
  EXPECT_EQ(0, simulator.CallI(buffer, static_cast<double>(0)));
  EXPECT_EQ(42, simulator.CallI(buffer, static_cast<double>(42)));
  EXPECT_EQ(static_cast<int64_t>(kMinInt32),
            simulator.CallI(buffer, static_cast<double>(kMinInt32)));
  EXPECT_EQ(static_cast<int64_t>(kMaxInt32),
            simulator.CallI(buffer, static_cast<double>(kMaxInt32)));
  EXPECT_EQ(static_cast<int64_t>(kMaxUint32),
            simulator.CallI(buffer, static_cast<double>(kMaxUint32)));
  EXPECT_EQ(kMinInt64, simulator.CallI(buffer, static_cast<double>(kMinInt64)));
  EXPECT_EQ(kMaxInt64, simulator.CallI(buffer, static_cast<double>(kMaxInt64)));
  EXPECT_EQ(kMaxInt64,
            simulator.CallI(buffer, static_cast<double>(kMaxUint64)));
  EXPECT_EQ(kMinInt64,
            simulator.CallI(buffer, -std::numeric_limits<double>::infinity()));
  EXPECT_EQ(kMaxInt64,
            simulator.CallI(buffer, std::numeric_limits<double>::infinity()));
  EXPECT_EQ(
      kMaxInt64,
      simulator.CallI(buffer, std::numeric_limits<double>::signaling_NaN()));
}

UNIT_TEST(ConvertDoubleToDoubleWord_RNE) {
  Assembler assembler(RV_G);
  __ fcvtld(A0, FA0, RNE);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  c2250553 fcvt.l.d a0, fa0\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(-44, simulator.CallI(buffer, -43.6));
  EXPECT_EQ(-44, simulator.CallI(buffer, -43.5));
  EXPECT_EQ(-43, simulator.CallI(buffer, -43.4));
  EXPECT_EQ(-43, simulator.CallI(buffer, -43.0));
  EXPECT_EQ(-43, simulator.CallI(buffer, -42.6));
  EXPECT_EQ(-42, simulator.CallI(buffer, -42.5));
  EXPECT_EQ(-42, simulator.CallI(buffer, -42.4));
  EXPECT_EQ(-42, simulator.CallI(buffer, -42.0));
  EXPECT_EQ(0, simulator.CallI(buffer, -0.0));
  EXPECT_EQ(0, simulator.CallI(buffer, +0.0));
  EXPECT_EQ(42, simulator.CallI(buffer, 42.0));
  EXPECT_EQ(42, simulator.CallI(buffer, 42.4));
  EXPECT_EQ(42, simulator.CallI(buffer, 42.5));
  EXPECT_EQ(43, simulator.CallI(buffer, 42.6));
  EXPECT_EQ(43, simulator.CallI(buffer, 43.0));
  EXPECT_EQ(43, simulator.CallI(buffer, 43.4));
  EXPECT_EQ(44, simulator.CallI(buffer, 43.5));
  EXPECT_EQ(44, simulator.CallI(buffer, 43.6));
}

UNIT_TEST(ConvertDoubleToDoubleWord_RTZ) {
  Assembler assembler(RV_G);
  __ fcvtld(A0, FA0, RTZ);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  c2251553 fcvt.l.d a0, fa0, rtz\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(-43, simulator.CallI(buffer, -43.6));
  EXPECT_EQ(-43, simulator.CallI(buffer, -43.5));
  EXPECT_EQ(-43, simulator.CallI(buffer, -43.4));
  EXPECT_EQ(-43, simulator.CallI(buffer, -43.0));
  EXPECT_EQ(-42, simulator.CallI(buffer, -42.6));
  EXPECT_EQ(-42, simulator.CallI(buffer, -42.5));
  EXPECT_EQ(-42, simulator.CallI(buffer, -42.4));
  EXPECT_EQ(-42, simulator.CallI(buffer, -42.0));
  EXPECT_EQ(0, simulator.CallI(buffer, -0.0));
  EXPECT_EQ(0, simulator.CallI(buffer, +0.0));
  EXPECT_EQ(42, simulator.CallI(buffer, 42.0));
  EXPECT_EQ(42, simulator.CallI(buffer, 42.4));
  EXPECT_EQ(42, simulator.CallI(buffer, 42.5));
  EXPECT_EQ(42, simulator.CallI(buffer, 42.6));
  EXPECT_EQ(43, simulator.CallI(buffer, 43.0));
  EXPECT_EQ(43, simulator.CallI(buffer, 43.4));
  EXPECT_EQ(43, simulator.CallI(buffer, 43.5));
  EXPECT_EQ(43, simulator.CallI(buffer, 43.6));
}

UNIT_TEST(ConvertDoubleToDoubleWord_RDN) {
  Assembler assembler(RV_G);
  __ fcvtld(A0, FA0, RDN);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  c2252553 fcvt.l.d a0, fa0, rdn\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(-44, simulator.CallI(buffer, -43.6));
  EXPECT_EQ(-44, simulator.CallI(buffer, -43.5));
  EXPECT_EQ(-44, simulator.CallI(buffer, -43.4));
  EXPECT_EQ(-43, simulator.CallI(buffer, -43.0));
  EXPECT_EQ(-43, simulator.CallI(buffer, -42.6));
  EXPECT_EQ(-43, simulator.CallI(buffer, -42.5));
  EXPECT_EQ(-43, simulator.CallI(buffer, -42.4));
  EXPECT_EQ(-42, simulator.CallI(buffer, -42.0));
  EXPECT_EQ(0, simulator.CallI(buffer, -0.0));
  EXPECT_EQ(0, simulator.CallI(buffer, +0.0));
  EXPECT_EQ(42, simulator.CallI(buffer, 42.0));
  EXPECT_EQ(42, simulator.CallI(buffer, 42.4));
  EXPECT_EQ(42, simulator.CallI(buffer, 42.5));
  EXPECT_EQ(42, simulator.CallI(buffer, 42.6));
  EXPECT_EQ(43, simulator.CallI(buffer, 43.0));
  EXPECT_EQ(43, simulator.CallI(buffer, 43.4));
  EXPECT_EQ(43, simulator.CallI(buffer, 43.5));
  EXPECT_EQ(43, simulator.CallI(buffer, 43.6));
}

UNIT_TEST(ConvertDoubleToDoubleWord_RUP) {
  Assembler assembler(RV_G);
  __ fcvtld(A0, FA0, RUP);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  c2253553 fcvt.l.d a0, fa0, rup\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(-43, simulator.CallI(buffer, -43.6));
  EXPECT_EQ(-43, simulator.CallI(buffer, -43.5));
  EXPECT_EQ(-43, simulator.CallI(buffer, -43.4));
  EXPECT_EQ(-43, simulator.CallI(buffer, -43.0));
  EXPECT_EQ(-42, simulator.CallI(buffer, -42.6));
  EXPECT_EQ(-42, simulator.CallI(buffer, -42.5));
  EXPECT_EQ(-42, simulator.CallI(buffer, -42.4));
  EXPECT_EQ(-42, simulator.CallI(buffer, -42.0));
  EXPECT_EQ(0, simulator.CallI(buffer, -0.0));
  EXPECT_EQ(0, simulator.CallI(buffer, +0.0));
  EXPECT_EQ(42, simulator.CallI(buffer, 42.0));
  EXPECT_EQ(43, simulator.CallI(buffer, 42.4));
  EXPECT_EQ(43, simulator.CallI(buffer, 42.5));
  EXPECT_EQ(43, simulator.CallI(buffer, 42.6));
  EXPECT_EQ(43, simulator.CallI(buffer, 43.0));
  EXPECT_EQ(44, simulator.CallI(buffer, 43.5));
  EXPECT_EQ(44, simulator.CallI(buffer, 43.5));
  EXPECT_EQ(44, simulator.CallI(buffer, 43.6));
}

UNIT_TEST(ConvertDoubleToDoubleWord_RMM) {
  Assembler assembler(RV_G);
  __ fcvtld(A0, FA0, RMM);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  c2254553 fcvt.l.d a0, fa0, rmm\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(-44, simulator.CallI(buffer, -43.6));
  EXPECT_EQ(-44, simulator.CallI(buffer, -43.5));
  EXPECT_EQ(-43, simulator.CallI(buffer, -43.4));
  EXPECT_EQ(-43, simulator.CallI(buffer, -43.0));
  EXPECT_EQ(-43, simulator.CallI(buffer, -42.6));
  EXPECT_EQ(-43, simulator.CallI(buffer, -42.5));
  EXPECT_EQ(-42, simulator.CallI(buffer, -42.4));
  EXPECT_EQ(-42, simulator.CallI(buffer, -42.0));
  EXPECT_EQ(0, simulator.CallI(buffer, -0.0));
  EXPECT_EQ(0, simulator.CallI(buffer, +0.0));
  EXPECT_EQ(42, simulator.CallI(buffer, 42.0));
  EXPECT_EQ(42, simulator.CallI(buffer, 42.4));
  EXPECT_EQ(43, simulator.CallI(buffer, 42.5));
  EXPECT_EQ(43, simulator.CallI(buffer, 42.6));
  EXPECT_EQ(43, simulator.CallI(buffer, 43.0));
  EXPECT_EQ(43, simulator.CallI(buffer, 43.4));
  EXPECT_EQ(44, simulator.CallI(buffer, 43.5));
  EXPECT_EQ(44, simulator.CallI(buffer, 43.6));
}

UNIT_TEST(ConvertDoubleToUnsignedDoubleWord) {
  Assembler assembler(RV_G);
  __ fcvtlud(A0, FA0);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  c2350553 fcvt.lu.d a0, fa0\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0, simulator.CallI(buffer, static_cast<double>(-42)));
  EXPECT_EQ(0, simulator.CallI(buffer, static_cast<double>(0)));
  EXPECT_EQ(42, simulator.CallI(buffer, static_cast<double>(42)));
  EXPECT_EQ(static_cast<int64_t>(static_cast<uint64_t>(0)),
            simulator.CallI(buffer, static_cast<double>(kMinInt32)));
  EXPECT_EQ(static_cast<int64_t>(static_cast<uint64_t>(kMaxInt32)),
            simulator.CallI(buffer, static_cast<double>(kMaxInt32)));
  EXPECT_EQ(static_cast<int64_t>(static_cast<uint64_t>(kMaxUint32)),
            simulator.CallI(buffer, static_cast<double>(kMaxUint32)));
  EXPECT_EQ(static_cast<int64_t>(static_cast<uint64_t>(0)),
            simulator.CallI(buffer, static_cast<double>(kMinInt64)));
  EXPECT_EQ(static_cast<int64_t>(static_cast<uint64_t>(kMaxInt64) + 1),
            simulator.CallI(buffer, static_cast<double>(kMaxInt64)));
  EXPECT_EQ(static_cast<int64_t>(static_cast<uint64_t>(kMaxUint64)),
            simulator.CallI(buffer, static_cast<double>(kMaxUint64)));
  EXPECT_EQ(static_cast<int64_t>(static_cast<uint64_t>(0)),
            simulator.CallI(buffer, -std::numeric_limits<double>::infinity()));
  EXPECT_EQ(static_cast<int64_t>(static_cast<uint64_t>(kMaxUint64)),
            simulator.CallI(buffer, std::numeric_limits<double>::infinity()));
  EXPECT_EQ(
      static_cast<int64_t>(static_cast<uint64_t>(kMaxUint64)),
      simulator.CallI(buffer, std::numeric_limits<double>::signaling_NaN()));
}

UNIT_TEST(BitCastDoubleToInteger) {
  Assembler assembler(RV_G);
  __ fmvxd(A0, FA0);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  e2050553 fmv.x.d a0, fa0\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(bit_cast<int64_t>(0.0), simulator.CallI(buffer, 0.0));
  EXPECT_EQ(bit_cast<int64_t>(-0.0), simulator.CallI(buffer, -0.0));
  EXPECT_EQ(bit_cast<int64_t>(42.0), simulator.CallI(buffer, 42.0));
  EXPECT_EQ(bit_cast<int64_t>(-42.0), simulator.CallI(buffer, -42.0));
  EXPECT_EQ(bit_cast<int64_t>(std::numeric_limits<double>::quiet_NaN()),
            simulator.CallI(buffer, std::numeric_limits<double>::quiet_NaN()));
  EXPECT_EQ(
      bit_cast<int64_t>(std::numeric_limits<double>::signaling_NaN()),
      simulator.CallI(buffer, std::numeric_limits<double>::signaling_NaN()));
  EXPECT_EQ(bit_cast<int64_t>(std::numeric_limits<double>::infinity()),
            simulator.CallI(buffer, std::numeric_limits<double>::infinity()));
  EXPECT_EQ(bit_cast<int64_t>(-std::numeric_limits<double>::infinity()),
            simulator.CallI(buffer, -std::numeric_limits<double>::infinity()));
}

UNIT_TEST(ConvertDoubleWordToDouble) {
  Assembler assembler(RV_G);
  __ fcvtdl(FA0, A0);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  d2250553 fcvt.d.l fa0, a0\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0.0, simulator.CallD(buffer, sign_extend(0)));
  EXPECT_EQ(42.0, simulator.CallD(buffer, sign_extend(42)));
  EXPECT_EQ(-42.0, simulator.CallD(buffer, sign_extend(-42)));
  EXPECT_EQ(static_cast<double>(kMinInt32),
            simulator.CallD(buffer, sign_extend(kMinInt32)));
  EXPECT_EQ(static_cast<double>(kMaxInt32),
            simulator.CallD(buffer, sign_extend(kMaxInt32)));
  EXPECT_EQ(static_cast<double>(sign_extend(kMaxUint32)),
            simulator.CallD(buffer, sign_extend(kMaxUint32)));
  EXPECT_EQ(static_cast<double>(kMinInt64),
            simulator.CallD(buffer, sign_extend(kMinInt64)));
  EXPECT_EQ(static_cast<double>(kMaxInt64),
            simulator.CallD(buffer, sign_extend(kMaxInt64)));
  EXPECT_EQ(static_cast<double>(sign_extend(kMaxUint64)),
            simulator.CallD(buffer, sign_extend(kMaxUint64)));
}

UNIT_TEST(ConvertUnsignedDoubleWordToDouble) {
  Assembler assembler(RV_G);
  __ fcvtdlu(FA0, A0);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  d2350553 fcvt.d.lu fa0, a0\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0.0, simulator.CallD(buffer, sign_extend(0)));
  EXPECT_EQ(42.0, simulator.CallD(buffer, sign_extend(42)));
  EXPECT_EQ(static_cast<double>(static_cast<uint64_t>(sign_extend(-42))),
            simulator.CallD(buffer, sign_extend(-42)));
  EXPECT_EQ(static_cast<double>(static_cast<uint64_t>(sign_extend(kMinInt32))),
            simulator.CallD(buffer, sign_extend(kMinInt32)));
  EXPECT_EQ(static_cast<double>(static_cast<uint64_t>(sign_extend(kMaxInt32))),
            simulator.CallD(buffer, sign_extend(kMaxInt32)));
  EXPECT_EQ(static_cast<double>(static_cast<uint64_t>(sign_extend(kMaxUint32))),
            simulator.CallD(buffer, sign_extend(kMaxUint32)));
  EXPECT_EQ(static_cast<double>(static_cast<uint64_t>(sign_extend(kMinInt64))),
            simulator.CallD(buffer, sign_extend(kMinInt64)));
  EXPECT_EQ(static_cast<double>(static_cast<uint64_t>(sign_extend(kMaxInt64))),
            simulator.CallD(buffer, sign_extend(kMaxInt64)));
  EXPECT_EQ(static_cast<double>(kMaxUint64),
            simulator.CallD(buffer, sign_extend(kMaxUint64)));
}

UNIT_TEST(BitCastIntegerToDouble) {
  Assembler assembler(RV_G);
  __ fmvdx(FA0, A0);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  f2050553 fmv.d.x fa0, a0\n"
      "  00008067 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0.0, simulator.CallD(buffer, bit_cast<int64_t>(0.0)));
  EXPECT_EQ(-0.0, simulator.CallD(buffer, bit_cast<int64_t>(-0.0)));
  EXPECT_EQ(42.0, simulator.CallD(buffer, bit_cast<int64_t>(42.0)));
  EXPECT_EQ(-42.0, simulator.CallD(buffer, bit_cast<int64_t>(-42.0)));
  EXPECT_EQ(true, isnan(simulator.CallD(
                      buffer, bit_cast<int64_t>(
                                  std::numeric_limits<double>::quiet_NaN()))));
  EXPECT_EQ(true,
            isnan(simulator.CallD(
                buffer, bit_cast<int64_t>(
                            std::numeric_limits<double>::signaling_NaN()))));
  EXPECT_EQ(
      std::numeric_limits<double>::infinity(),
      simulator.CallD(
          buffer, bit_cast<int64_t>(std::numeric_limits<double>::infinity())));
  EXPECT_EQ(
      -std::numeric_limits<double>::infinity(),
      simulator.CallD(
          buffer, bit_cast<int64_t>(-std::numeric_limits<double>::infinity())));
}
#endif

UNIT_TEST(Fibonacci) {
  Assembler assembler(RV_G);
  Label fib, base, done;
  __ Bind(&fib);
  __ subi(SP, SP, sizeof(uintx_t) * 4);
  __ sx(RA, Address(SP, 3 * sizeof(uintx_t)));
  __ sx(A0, Address(SP, 2 * sizeof(uintx_t)));
  __ subi(A0, A0, 1);
  __ blez(A0, &base);

  __ jal(&fib);
  __ sx(A0, Address(SP, 1 * sizeof(uintx_t)));
  __ lx(A0, Address(SP, 2 * sizeof(uintx_t)));
  __ subi(A0, A0, 2);
  __ jal(&fib);
  __ lx(A1, Address(SP, 1 * sizeof(uintx_t)));
  __ add(A0, A0, A1);
  __ j(&done);

  __ Bind(&base);
  __ li(A0, 1);

  __ Bind(&done);
  __ lx(RA, Address(SP, 3 * sizeof(uintx_t)));
  __ addi(SP, SP, sizeof(uintx_t) * 4);
  __ ret();
  __ trap();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
#if XLEN == 32
  EXPECT_STREQ(
      "  ff010113 addi sp, sp, -16\n"
      "  00112623 sw ra, 12(sp)\n"
      "  00a12423 sw a0, 8(sp)\n"
      "  fff50513 addi a0, a0, -1\n"
      "  02a05263 blez a0, +36\n"
      "  fedff0ef jal -20\n"
      "  00a12223 sw a0, 4(sp)\n"
      "  00812503 lw a0, 8(sp)\n"
      "  ffe50513 addi a0, a0, -2\n"
      "  fddff0ef jal -36\n"
      "  00412583 lw a1, 4(sp)\n"
      "  00b50533 add a0, a0, a1\n"
      "  0080006f j +8\n"
      "  00100513 li a0, 1\n"
      "  00c12083 lw ra, 12(sp)\n"
      "  01010113 addi sp, sp, 16\n"
      "  00008067 ret\n"
      "  00000000 trap\n",
      disassembly);
#elif XLEN == 64
  EXPECT_STREQ(
      "  fe010113 addi sp, sp, -32\n"
      "  00113c23 sd ra, 24(sp)\n"
      "  00a13823 sd a0, 16(sp)\n"
      "  fff50513 addi a0, a0, -1\n"
      "  02a05263 blez a0, +36\n"
      "  fedff0ef jal -20\n"
      "  00a13423 sd a0, 8(sp)\n"
      "  01013503 ld a0, 16(sp)\n"
      "  ffe50513 addi a0, a0, -2\n"
      "  fddff0ef jal -36\n"
      "  00813583 ld a1, 8(sp)\n"
      "  00b50533 add a0, a0, a1\n"
      "  0080006f j +8\n"
      "  00100513 li a0, 1\n"
      "  01813083 ld ra, 24(sp)\n"
      "  02010113 addi sp, sp, 32\n"
      "  00008067 ret\n"
      "  00000000 trap\n",
      disassembly);
#else
#error Unimplemented
#endif
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(1, simulator.Call(buffer, 0));
  EXPECT_EQ(1, simulator.Call(buffer, 1));
  EXPECT_EQ(2, simulator.Call(buffer, 2));
  EXPECT_EQ(3, simulator.Call(buffer, 3));
  EXPECT_EQ(5, simulator.Call(buffer, 4));
  EXPECT_EQ(8, simulator.Call(buffer, 5));
  EXPECT_EQ(13, simulator.Call(buffer, 6));
}

static intx_t TestRuntimeEntry(intx_t a, intx_t b, intx_t c, intx_t d) {
  EXPECT_EQ(10, a);
  EXPECT_EQ(20, b);
  EXPECT_EQ(30, c);
  EXPECT_EQ(40, d);
  return 50;
}

UNIT_TEST(Redirection) {
  struct Thread {
    uintx_t nil_;
    uintx_t false_;
    uintx_t true_;
    uintx_t runtime_entry_;
  };

  Assembler assembler(RV_G);
  __ subi(SP, SP, sizeof(uintx_t) * 1);
  __ sx(RA, Address(SP, 0 * sizeof(uintx_t)));
  __ mv(T0, A0);
  __ li(A0, 10);
  __ li(A1, 20);
  __ li(A2, 30);
  __ li(A3, 40);
  __ lx(RA, Address(T0, offsetof(Thread, runtime_entry_)));
  __ jalr(RA);
  __ lx(RA, Address(SP, 0 * sizeof(uintx_t)));
  __ addi(SP, SP, sizeof(uintx_t) * 1);
  __ ret();
  __ trap();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_G);
  char* disassembly = disassembler.Disassemble(buffer, size);
#if XLEN == 32
  EXPECT_STREQ(
      "  ffc10113 addi sp, sp, -4\n"
      "  00112023 sw ra, 0(sp)\n"
      "  00050293 mv t0, a0\n"
      "  00a00513 li a0, 10\n"
      "  01400593 li a1, 20\n"
      "  01e00613 li a2, 30\n"
      "  02800693 li a3, 40\n"
      "  00c2a083 lw ra, 12(t0)\n"
      "  000080e7 jalr ra\n"
      "  00012083 lw ra, 0(sp)\n"
      "  00410113 addi sp, sp, 4\n"
      "  00008067 ret\n"
      "  00000000 trap\n",
      disassembly);
#elif XLEN == 64
  EXPECT_STREQ(
      "  ff810113 addi sp, sp, -8\n"
      "  00113023 sd ra, 0(sp)\n"
      "  00050293 mv t0, a0\n"
      "  00a00513 li a0, 10\n"
      "  01400593 li a1, 20\n"
      "  01e00613 li a2, 30\n"
      "  02800693 li a3, 40\n"
      "  0182b083 ld ra, 24(t0)\n"
      "  000080e7 jalr ra\n"
      "  00013083 ld ra, 0(sp)\n"
      "  00810113 addi sp, sp, 8\n"
      "  00008067 ret\n"
      "  00000000 trap\n",
      disassembly);
#else
#error Unimplemented
#endif
  free(disassembly);

  Redirection* redirection = new (Memory::Allocate(sizeof(Redirection)))
      Redirection(&TestRuntimeEntry);
  Thread* thread = reinterpret_cast<Thread*>(Memory::Allocate(sizeof(Thread)));
  thread->runtime_entry_ = Memory::ToGuest(redirection->entry_point());

  Simulator simulator;
  EXPECT_EQ(50, simulator.Call(buffer, Memory::ToGuest(thread)));
}

UNIT_TEST(CompressedLoadStoreWordSP) {
  {
    Assembler assembler(RV_GC);
    __ subi(SP, SP, 256);
    __ sw(A1, Address(SP, 0));
    __ lw(A0, Address(SP, 0));
    __ addi(SP, SP, 256);
    __ ret();

    void* buffer = assembler.buffer();
    size_t size = assembler.size();

    Disassembler disassembler(RV_GC);
    char* disassembly = disassembler.Disassemble(buffer, size);
    EXPECT_STREQ(
        "      7111 addi sp, sp, -256\n"
        "      c02e sw a1, 0(sp)\n"
        "      4502 lw a0, 0(sp)\n"
        "      6111 addi sp, sp, 256\n"
        "      8082 ret\n",
        disassembly);
    free(disassembly);

    Simulator simulator;
    EXPECT_EQ(sign_extend(0xAB010203), simulator.Call(buffer, 0, 0xAB010203));
    EXPECT_EQ(sign_extend(0xCD020405), simulator.Call(buffer, 0, 0xCD020405));
    EXPECT_EQ(sign_extend(0xEF030607), simulator.Call(buffer, 0, 0xEF030607));
  }
  {
    Assembler assembler(RV_GC);
    __ subi(SP, SP, 256);
    __ sw(A1, Address(SP, 4));
    __ lw(A0, Address(SP, 4));
    __ addi(SP, SP, 256);
    __ ret();

    void* buffer = assembler.buffer();
    size_t size = assembler.size();

    Disassembler disassembler(RV_GC);
    char* disassembly = disassembler.Disassemble(buffer, size);
    EXPECT_STREQ(
        "      7111 addi sp, sp, -256\n"
        "      c22e sw a1, 4(sp)\n"
        "      4512 lw a0, 4(sp)\n"
        "      6111 addi sp, sp, 256\n"
        "      8082 ret\n",
        disassembly);
    free(disassembly);

    Simulator simulator;
    EXPECT_EQ(sign_extend(0xAB010203), simulator.Call(buffer, 0, 0xAB010203));
    EXPECT_EQ(sign_extend(0xCD020405), simulator.Call(buffer, 0, 0xCD020405));
    EXPECT_EQ(sign_extend(0xEF030607), simulator.Call(buffer, 0, 0xEF030607));
  }
}

UNIT_TEST(CompressedLoadStoreSPLarge) {
  Assembler assembler(RV_GC);
#if XLEN == 32
  __ lw(A0, Address(SP, 252));
  __ sw(A0, Address(SP, 252));
#elif XLEN == 64
  __ ld(A0, Address(SP, 504));
  __ sd(A0, Address(SP, 504));
#endif

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GC);
  char* disassembly = disassembler.Disassemble(buffer, size);
#if XLEN == 32
  EXPECT_STREQ(
      "      557e lw a0, 252(sp)\n"
      "      dfaa sw a0, 252(sp)\n",
      disassembly);
#elif XLEN == 64
  EXPECT_STREQ(
      "      757e ld a0, 504(sp)\n"
      "      ffaa sd a0, 504(sp)\n",
      disassembly);
#endif
  free(disassembly);
}

#if XLEN == 32
UNIT_TEST(CompressedLoadStoreSingleFloatSP) {
  {
    Assembler assembler(RV_GC);
    __ subi(SP, SP, 256);
    __ fsw(FA1, Address(SP, 0));
    __ flw(FA0, Address(SP, 0));
    __ addi(SP, SP, 256);
    __ ret();

    void* buffer = assembler.buffer();
    size_t size = assembler.size();

    Disassembler disassembler(RV_GC);
    char* disassembly = disassembler.Disassemble(buffer, size);
    EXPECT_STREQ(
        "      7111 addi sp, sp, -256\n"
        "      e02e fsw fa1, 0(sp)\n"
        "      6502 flw fa0, 0(sp)\n"
        "      6111 addi sp, sp, 256\n"
        "      8082 ret\n",
        disassembly);
    free(disassembly);

    Simulator simulator;
    EXPECT_EQ(1.7f, simulator.CallF(buffer, 0.0f, 1.7f));
    EXPECT_EQ(2.8f, simulator.CallF(buffer, 0.0f, 2.8f));
    EXPECT_EQ(3.9f, simulator.CallF(buffer, 0.0f, 3.9f));
  }
  {
    Assembler assembler(RV_GC);
    __ subi(SP, SP, 256);
    __ fsw(FA1, Address(SP, 4));
    __ flw(FA0, Address(SP, 4));
    __ addi(SP, SP, 256);
    __ ret();

    void* buffer = assembler.buffer();
    size_t size = assembler.size();

    Disassembler disassembler(RV_GC);
    char* disassembly = disassembler.Disassemble(buffer, size);
    EXPECT_STREQ(
        "      7111 addi sp, sp, -256\n"
        "      e22e fsw fa1, 4(sp)\n"
        "      6512 flw fa0, 4(sp)\n"
        "      6111 addi sp, sp, 256\n"
        "      8082 ret\n",
        disassembly);
    free(disassembly);

    Simulator simulator;
    EXPECT_EQ(1.7f, simulator.CallF(buffer, 0.0f, 1.7f));
    EXPECT_EQ(2.8f, simulator.CallF(buffer, 0.0f, 2.8f));
    EXPECT_EQ(3.9f, simulator.CallF(buffer, 0.0f, 3.9f));
  }
}
#endif

UNIT_TEST(CompressedLoadStoreDoubleFloatSP) {
  {
    Assembler assembler(RV_GC);
    __ subi(SP, SP, 256);
    __ fsd(FA1, Address(SP, 0));
    __ fld(FA0, Address(SP, 0));
    __ addi(SP, SP, 256);
    __ ret();

    void* buffer = assembler.buffer();
    size_t size = assembler.size();

    Disassembler disassembler(RV_GC);
    char* disassembly = disassembler.Disassemble(buffer, size);
    EXPECT_STREQ(
        "      7111 addi sp, sp, -256\n"
        "      a02e fsd fa1, 0(sp)\n"
        "      2502 fld fa0, 0(sp)\n"
        "      6111 addi sp, sp, 256\n"
        "      8082 ret\n",
        disassembly);
    free(disassembly);

    Simulator simulator;
    EXPECT_EQ(1.7, simulator.CallD(buffer, 0.0, 1.7));
    EXPECT_EQ(2.8, simulator.CallD(buffer, 0.0, 2.8));
    EXPECT_EQ(3.9, simulator.CallD(buffer, 0.0, 3.9));
  }
  {
    Assembler assembler(RV_GC);
    __ subi(SP, SP, 256);
    __ fsd(FA1, Address(SP, 8));
    __ fld(FA0, Address(SP, 8));
    __ addi(SP, SP, 256);
    __ ret();

    void* buffer = assembler.buffer();
    size_t size = assembler.size();

    Disassembler disassembler(RV_GC);
    char* disassembly = disassembler.Disassemble(buffer, size);
    EXPECT_STREQ(
        "      7111 addi sp, sp, -256\n"
        "      a42e fsd fa1, 8(sp)\n"
        "      2522 fld fa0, 8(sp)\n"
        "      6111 addi sp, sp, 256\n"
        "      8082 ret\n",
        disassembly);
    free(disassembly);

    Simulator simulator;
    EXPECT_EQ(1.7, simulator.CallD(buffer, 0.0, 1.7));
    EXPECT_EQ(2.8, simulator.CallD(buffer, 0.0, 2.8));
    EXPECT_EQ(3.9, simulator.CallD(buffer, 0.0, 3.9));
  }
}

#if XLEN >= 64
UNIT_TEST(CompressedLoadStoreDoubleWordSP) {
  {
    Assembler assembler(RV_GC);
    __ subi(SP, SP, 256);
    __ sd(A1, Address(SP, 0));
    __ ld(A0, Address(SP, 0));
    __ addi(SP, SP, 256);
    __ ret();

    void* buffer = assembler.buffer();
    size_t size = assembler.size();

    Disassembler disassembler(RV_GC);
    char* disassembly = disassembler.Disassemble(buffer, size);
    EXPECT_STREQ(
        "      7111 addi sp, sp, -256\n"
        "      e02e sd a1, 0(sp)\n"
        "      6502 ld a0, 0(sp)\n"
        "      6111 addi sp, sp, 256\n"
        "      8082 ret\n",
        disassembly);
    free(disassembly);

    Simulator simulator;
    EXPECT_EQ((intx_t)0xAB01020304050607,
              simulator.Call(buffer, 0, 0xAB01020304050607));
    EXPECT_EQ((intx_t)0xCD02040505060708,
              simulator.Call(buffer, 0, 0xCD02040505060708));
    EXPECT_EQ((intx_t)0xEF03060708090A0B,
              simulator.Call(buffer, 0, 0xEF03060708090A0B));
  }
  {
    Assembler assembler(RV_GC);
    __ subi(SP, SP, 256);
    __ sd(A1, Address(SP, 8));
    __ ld(A0, Address(SP, 8));
    __ addi(SP, SP, 256);
    __ ret();

    void* buffer = assembler.buffer();
    size_t size = assembler.size();

    Disassembler disassembler(RV_GC);
    char* disassembly = disassembler.Disassemble(buffer, size);
    EXPECT_STREQ(
        "      7111 addi sp, sp, -256\n"
        "      e42e sd a1, 8(sp)\n"
        "      6522 ld a0, 8(sp)\n"
        "      6111 addi sp, sp, 256\n"
        "      8082 ret\n",
        disassembly);
    free(disassembly);

    Simulator simulator;
    EXPECT_EQ((intx_t)0xAB01020304050607,
              simulator.Call(buffer, 0, 0xAB01020304050607));
    EXPECT_EQ((intx_t)0xCD02040505060708,
              simulator.Call(buffer, 0, 0xCD02040505060708));
    EXPECT_EQ((intx_t)0xEF03060708090A0B,
              simulator.Call(buffer, 0, 0xEF03060708090A0B));
  }
}
#endif

UNIT_TEST(CompressedLoadWord) {
  {
    Assembler assembler(RV_GC);
    __ lw(A0, Address(A0, 0));
    __ ret();

    void* buffer = assembler.buffer();
    size_t size = assembler.size();

    Disassembler disassembler(RV_GC);
    char* disassembly = disassembler.Disassemble(buffer, size);
    EXPECT_STREQ(
        "      4108 lw a0, 0(a0)\n"
        "      8082 ret\n",
        disassembly);
    free(disassembly);

    uint32_t* values =
        reinterpret_cast<uint32_t*>(Memory::Allocate(3 * sizeof(uint32_t)));
    values[0] = 0xAB010203;
    values[1] = 0xCD020405;
    values[2] = 0xEF030607;

    Simulator simulator;
    EXPECT_EQ(-855505915, simulator.Call(buffer, Memory::ToGuest(&values[1])));
  }
  {
    Assembler assembler(RV_GC);
    __ lw(A0, Address(A0, 4));
    __ ret();

    void* buffer = assembler.buffer();
    size_t size = assembler.size();

    Disassembler disassembler(RV_GC);
    char* disassembly = disassembler.Disassemble(buffer, size);
    EXPECT_STREQ(
        "      4148 lw a0, 4(a0)\n"
        "      8082 ret\n",
        disassembly);
    free(disassembly);

    uint32_t* values =
        reinterpret_cast<uint32_t*>(Memory::Allocate(3 * sizeof(uint32_t)));
    values[0] = 0xAB010203;
    values[1] = 0xCD020405;
    values[2] = 0xEF030607;

    Simulator simulator;
    EXPECT_EQ(-285014521, simulator.Call(buffer, Memory::ToGuest(&values[1])));
  }
}

UNIT_TEST(CompressedStoreWord) {
  {
    Assembler assembler(RV_GC);
    __ sw(A1, Address(A0, 0));
    __ ret();

    void* buffer = assembler.buffer();
    size_t size = assembler.size();

    Disassembler disassembler(RV_GC);
    char* disassembly = disassembler.Disassemble(buffer, size);
    EXPECT_STREQ(
        "      c10c sw a1, 0(a0)\n"
        "      8082 ret\n",
        disassembly);
    free(disassembly);

    uint32_t* values =
        reinterpret_cast<uint32_t*>(Memory::Allocate(3 * sizeof(uint32_t)));
    values[0] = 0;
    values[1] = 0;
    values[2] = 0;

    Simulator simulator;
    simulator.Call(buffer, Memory::ToGuest(&values[1]), 0xCD020405);
    EXPECT_EQ(0u, values[0]);
    EXPECT_EQ(0xCD020405, values[1]);
    EXPECT_EQ(0u, values[2]);
  }
  {
    Assembler assembler(RV_GC);
    __ sw(A1, Address(A0, 4));
    __ ret();

    void* buffer = assembler.buffer();
    size_t size = assembler.size();

    Disassembler disassembler(RV_GC);
    char* disassembly = disassembler.Disassemble(buffer, size);
    EXPECT_STREQ(
        "      c14c sw a1, 4(a0)\n"
        "      8082 ret\n",
        disassembly);
    free(disassembly);

    uint32_t* values =
        reinterpret_cast<uint32_t*>(Memory::Allocate(3 * sizeof(uint32_t)));
    values[0] = 0;
    values[1] = 0;
    values[2] = 0;

    Simulator simulator;
    simulator.Call(buffer, Memory::ToGuest(&values[1]), 0xEF030607);
    EXPECT_EQ(0u, values[0]);
    EXPECT_EQ(0u, values[1]);
    EXPECT_EQ(0xEF030607, values[2]);
  }
}

#if XLEN == 32
UNIT_TEST(CompressedLoadSingleFloat) {
  Assembler assembler(RV_GC);
  __ flw(FA0, Address(A0, 1 * sizeof(float)));
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GC);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "      6148 flw fa0, 4(a0)\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  float* data = reinterpret_cast<float*>(Memory::Allocate(3 * sizeof(float)));
  data[0] = 1.7f;
  data[1] = 2.8f;
  data[2] = 3.9f;

  Simulator simulator;
  EXPECT_EQ(data[1], simulator.CallF(buffer, Memory::ToGuest(data)));
}

UNIT_TEST(CompressedStoreSingleFloat) {
  Assembler assembler(RV_GC);
  __ fsw(FA0, Address(A0, 1 * sizeof(float)));
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GC);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "      e148 fsw fa0, 4(a0)\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  float* data = reinterpret_cast<float*>(Memory::Allocate(3 * sizeof(float)));
  data[0] = 1.7f;
  data[1] = 2.8f;
  data[2] = 3.9f;

  Simulator simulator;
  simulator.CallF(buffer, Memory::ToGuest(data), 4.2f);
  EXPECT_EQ(4.2f, data[1]);
}
#endif

#if XLEN >= 64
UNIT_TEST(CompressedLoadDoubleWord) {
  {
    Assembler assembler(RV_GC);
    __ ld(A0, Address(A0, 0));
    __ ret();

    void* buffer = assembler.buffer();
    size_t size = assembler.size();

    Disassembler disassembler(RV_GC);
    char* disassembly = disassembler.Disassemble(buffer, size);
    EXPECT_STREQ(
        "      6108 ld a0, 0(a0)\n"
        "      8082 ret\n",
        disassembly);
    free(disassembly);

    uint64_t* values =
        reinterpret_cast<uint64_t*>(Memory::Allocate(3 * sizeof(uint64_t)));
    values[0] = 0xAB01020304050607;
    values[1] = 0xCD02040505060708;
    values[2] = 0xEF03060708090A0B;

    Simulator simulator;
    EXPECT_EQ(-3674369926375274744,
              simulator.Call(buffer, Memory::ToGuest(&values[1])));
  }
  {
    Assembler assembler(RV_GC);
    __ ld(A0, Address(A0, 8));
    __ ret();

    void* buffer = assembler.buffer();
    size_t size = assembler.size();

    Disassembler disassembler(RV_GC);
    char* disassembly = disassembler.Disassemble(buffer, size);
    EXPECT_STREQ(
        "      6508 ld a0, 8(a0)\n"
        "      8082 ret\n",
        disassembly);
    free(disassembly);

    uint64_t* values =
        reinterpret_cast<uint64_t*>(Memory::Allocate(3 * sizeof(uint64_t)));
    values[0] = 0xAB01020304050607;
    values[1] = 0xCD02040505060708;
    values[2] = 0xEF03060708090A0B;

    Simulator simulator;
    EXPECT_EQ(-1224128046445295093,
              simulator.Call(buffer, Memory::ToGuest(&values[1])));
  }
}

UNIT_TEST(CompressedStoreDoubleWord) {
  {
    Assembler assembler(RV_GC);
    __ sd(A1, Address(A0, 0));
    __ ret();

    void* buffer = assembler.buffer();
    size_t size = assembler.size();

    Disassembler disassembler(RV_GC);
    char* disassembly = disassembler.Disassemble(buffer, size);
    EXPECT_STREQ(
        "      e10c sd a1, 0(a0)\n"
        "      8082 ret\n",
        disassembly);
    free(disassembly);

    uint64_t* values =
        reinterpret_cast<uint64_t*>(Memory::Allocate(3 * sizeof(uint64_t)));
    values[0] = 0;
    values[1] = 0;
    values[2] = 0;

    Simulator simulator;
    simulator.Call(buffer, Memory::ToGuest(&values[1]), 0xCD02040505060708);
    EXPECT_EQ(0u, values[0]);
    EXPECT_EQ(0xCD02040505060708, values[1]);
    EXPECT_EQ(0u, values[2]);
  }
  {
    Assembler assembler(RV_GC);
    __ sd(A1, Address(A0, 8));
    __ ret();

    void* buffer = assembler.buffer();
    size_t size = assembler.size();

    Disassembler disassembler(RV_GC);
    char* disassembly = disassembler.Disassemble(buffer, size);
    EXPECT_STREQ(
        "      e50c sd a1, 8(a0)\n"
        "      8082 ret\n",
        disassembly);
    free(disassembly);

    uint64_t* values =
        reinterpret_cast<uint64_t*>(Memory::Allocate(3 * sizeof(uint64_t)));
    values[0] = 0;
    values[1] = 0;
    values[2] = 0;

    Simulator simulator;
    simulator.Call(buffer, Memory::ToGuest(&values[1]), 0xEF03060708090A0B);
    EXPECT_EQ(0u, values[0]);
    EXPECT_EQ(0u, values[1]);
    EXPECT_EQ(0xEF03060708090A0B, values[2]);
  }
}
#endif

UNIT_TEST(CompressedLoadDoubleFloat) {
  Assembler assembler(RV_GC);
  __ fld(FA0, Address(A0, 1 * sizeof(double)));
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GC);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "      2508 fld fa0, 8(a0)\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  double* data =
      reinterpret_cast<double*>(Memory::Allocate(3 * sizeof(double)));
  data[0] = 1.7;
  data[1] = 2.8;
  data[2] = 3.9;

  Simulator simulator;
  EXPECT_EQ(data[1], simulator.CallD(buffer, Memory::ToGuest(data)));
}

UNIT_TEST(CompressedStoreDoubleFloat) {
  Assembler assembler(RV_GC);
  __ fsd(FA0, Address(A0, 1 * sizeof(double)));
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GC);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "      a508 fsd fa0, 8(a0)\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  double* data =
      reinterpret_cast<double*>(Memory::Allocate(3 * sizeof(double)));
  data[0] = 1.7;
  data[1] = 2.8;
  data[2] = 3.9;

  Simulator simulator;
  simulator.CallD(buffer, Memory::ToGuest(data), 4.2);
  EXPECT_EQ(4.2, data[1]);
}

#if XLEN == 32
UNIT_TEST(CompressedJumpAndLink) {
  Assembler assembler(RV_GC);
  Label label1, label2;
  __ mv(T3, RA);
  __ jal(&label1, Assembler::kNearJump);  // Forward.
  __ sub(A0, T0, T1);
  __ mv(RA, T3);
  __ ret();
  __ trap();

  __ Bind(&label2);
  __ mv(T5, RA);
  __ li(T1, 7);
  __ jr(T5);
  __ trap();

  __ Bind(&label1);
  __ mv(T4, RA);
  __ li(T0, 4);
  __ jal(&label2, Assembler::kNearJump);  // Backward.
  __ mv(RA, T4);
  __ jr(T4);
  __ trap();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GC);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "      8e06 mv t3, ra\n"
      "      2811 jal +20\n"
      "  40628533 sub a0, t0, t1\n"
      "      80f2 mv ra, t3\n"
      "      8082 ret\n"
      "      0000 trap\n"
      "      8f06 mv t5, ra\n"
      "      431d li t1, 7\n"
      "      8f02 jr t5\n"
      "      0000 trap\n"
      "      8e86 mv t4, ra\n"
      "      4291 li t0, 4\n"
      "      3fd5 jal -12\n"
      "      80f6 mv ra, t4\n"
      "      8e82 jr t4\n"
      "      0000 trap\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(-3, simulator.Call(buffer));
}
#endif

UNIT_TEST(CompressedJump) {
  Assembler assembler(RV_GC);
  Label label1, label2;
  __ j(&label1, Assembler::kNearJump);  // Forward.
  __ trap();
  __ Bind(&label2);
  __ li(T1, 7);
  __ sub(A0, T0, T1);
  __ ret();
  __ Bind(&label1);
  __ li(T0, 4);
  __ j(&label2, Assembler::kNearJump);  // Backward.
  __ trap();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GC);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "      a031 j +12\n"
      "      0000 trap\n"
      "      431d li t1, 7\n"
      "  40628533 sub a0, t0, t1\n"
      "      8082 ret\n"
      "      4291 li t0, 4\n"
      "      bfdd j -10\n"
      "      0000 trap\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(-3, simulator.Call(buffer));
}

UNIT_TEST(CompressedJumpAndLinkRegister) {
  Assembler assembler(RV_GC);
  Label label1, label2;
  __ mv(T3, RA);
  __ jalr(A1);  // Forward.
  __ sub(A0, T0, T1);
  __ jr(T3);
  __ trap();

  __ Bind(&label2);
  __ mv(T5, RA);
  __ li(T1, 7);
  __ jr(T5);
  __ trap();

  __ Bind(&label1);
  __ mv(T4, RA);
  __ li(T0, 4);
  __ jalr(A2);  // Backward.
  __ jr(T4);
  __ trap();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GC);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "      8e06 mv t3, ra\n"
      "      9582 jalr a1\n"
      "  40628533 sub a0, t0, t1\n"
      "      8e02 jr t3\n"
      "      0000 trap\n"
      "      8f06 mv t5, ra\n"
      "      431d li t1, 7\n"
      "      8f02 jr t5\n"
      "      0000 trap\n"
      "      8e86 mv t4, ra\n"
      "      4291 li t0, 4\n"
      "      9602 jalr a2\n"
      "      8e82 jr t4\n"
      "      0000 trap\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(-3, simulator.Call(buffer, 0,
                               Memory::ToGuest(buffer) + label1.Position(),
                               Memory::ToGuest(buffer) + label2.Position()));
}

UNIT_TEST(CompressedJumpRegister) {
  Assembler assembler(RV_GC);
  Label label;
  __ jr(A1);
  __ trap();
  __ Bind(&label);
  __ li(A0, 42);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GC);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "      8582 jr a1\n"
      "      0000 trap\n"
      "  02a00513 li a0, 42\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(42, simulator.Call(buffer, 0,
                               Memory::ToGuest(buffer) + label.Position()));
}

UNIT_TEST(CompressedBranchEqualZero) {
  Assembler assembler(RV_GC);
  Label label;
  __ beqz(A0, &label, Assembler::kNearJump);
  __ li(A0, 3);
  __ ret();
  __ Bind(&label);
  __ li(A0, 4);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GC);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "      c119 beqz a0, +6\n"
      "      450d li a0, 3\n"
      "      8082 ret\n"
      "      4511 li a0, 4\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(3, simulator.Call(buffer, -42));
  EXPECT_EQ(4, simulator.Call(buffer, 0));
  EXPECT_EQ(3, simulator.Call(buffer, 42));
}

UNIT_TEST(CompressedBranchNotEqualZero) {
  Assembler assembler(RV_GC);
  Label label;
  __ bnez(A0, &label, Assembler::kNearJump);
  __ li(A0, 3);
  __ ret();
  __ Bind(&label);
  __ li(A0, 4);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GC);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "      e119 bnez a0, +6\n"
      "      450d li a0, 3\n"
      "      8082 ret\n"
      "      4511 li a0, 4\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(4, simulator.Call(buffer, -42));
  EXPECT_EQ(3, simulator.Call(buffer, 0));
  EXPECT_EQ(4, simulator.Call(buffer, 42));
}

UNIT_TEST(CompressedLoadImmediate) {
  Assembler assembler(RV_GC);
  __ li(A0, -7);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GC);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "      5565 li a0, -7\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(-7, simulator.Call(buffer));
}

UNIT_TEST(CompressedLoadUpperImmediate) {
  Assembler assembler(RV_GC);
  __ lui(A0, 7 << 12);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GC);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "      651d lui a0, 28672\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(7 << 12, simulator.Call(buffer));
}

UNIT_TEST(CompressedAddImmediate) {
  Assembler assembler(RV_GC);
  __ addi(A0, A0, 19);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GC);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "      054d addi a0, a0, 19\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(42, simulator.Call(buffer, 23));
}

#if XLEN == 64
UNIT_TEST(CompressedAddImmediateWord) {
  Assembler assembler(RV_GC);
  __ addiw(A0, A0, 19);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GC);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "      254d addiw a0, a0, 19\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(19, simulator.Call(buffer, 0xFFFFFFFF00000000));
  EXPECT_EQ(-237, simulator.Call(buffer, 0x00000000FFFFFF00));
}
#endif

UNIT_TEST(CompressedAddImmediateSP16) {
  Assembler assembler(RV_GC);
  __ addi(SP, SP, -128);
  __ addi(SP, SP, +128);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GC);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "      7119 addi sp, sp, -128\n"
      "      6109 addi sp, sp, 128\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0, simulator.Call(buffer, 0));
}

UNIT_TEST(CompressedAddImmediateSP4N) {
  Assembler assembler(RV_GC);
  __ addi(A1, SP, 36);
  __ sub(A0, A1, SP);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GC);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "      104c addi a1, sp, 36\n"
      "  40258533 sub a0, a1, sp\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(36, simulator.Call(buffer));
}

UNIT_TEST(CompressedShiftLeftLogicalImmediate) {
  Assembler assembler(RV_GC);
  __ slli(A0, A0, 3);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GC);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "      050e slli a0, a0, 3\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0, simulator.Call(buffer, 0));
  EXPECT_EQ(336, simulator.Call(buffer, 42));
  EXPECT_EQ(15872, simulator.Call(buffer, 1984));
  EXPECT_EQ(-336, simulator.Call(buffer, -42));
  EXPECT_EQ(-15872, simulator.Call(buffer, -1984));
}

UNIT_TEST(CompressedShiftRightLogicalImmediate) {
  Assembler assembler(RV_GC);
  __ srli(A0, A0, 3);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GC);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "      810d srli a0, a0, 3\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0, simulator.Call(buffer, 0));
  EXPECT_EQ(5, simulator.Call(buffer, 42));
  EXPECT_EQ(248, simulator.Call(buffer, 1984));
  EXPECT_EQ(static_cast<intx_t>(static_cast<uintx_t>(-42) >> 3),
            simulator.Call(buffer, -42));
  EXPECT_EQ(static_cast<intx_t>(static_cast<uintx_t>(-1984) >> 3),
            simulator.Call(buffer, -1984));
}

UNIT_TEST(CompressedShiftRightArithmeticImmediate) {
  Assembler assembler(RV_GC);
  __ srai(A0, A0, 3);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GC);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "      850d srai a0, a0, 3\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0, simulator.Call(buffer, 0));
  EXPECT_EQ(5, simulator.Call(buffer, 42));
  EXPECT_EQ(248, simulator.Call(buffer, 1984));
  EXPECT_EQ(-6, simulator.Call(buffer, -42));
  EXPECT_EQ(-248, simulator.Call(buffer, -1984));
}

UNIT_TEST(CompressedAndImmediate) {
  Assembler assembler(RV_GC);
  __ andi(A0, A0, 6);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GC);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "      8919 andi a0, a0, 6\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0, simulator.Call(buffer, 0));
  EXPECT_EQ(2, simulator.Call(buffer, 43));
  EXPECT_EQ(0, simulator.Call(buffer, 1984));
  EXPECT_EQ(6, simulator.Call(buffer, -42));
  EXPECT_EQ(0, simulator.Call(buffer, -1984));
}

UNIT_TEST(CompressedAndImmediate2) {
  Assembler assembler(RV_GC);
  __ andi(A0, A0, -6);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GC);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "      9969 andi a0, a0, -6\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0, simulator.Call(buffer, 0));
  EXPECT_EQ(42, simulator.Call(buffer, 43));
  EXPECT_EQ(1984, simulator.Call(buffer, 1984));
  EXPECT_EQ(-46, simulator.Call(buffer, -42));
  EXPECT_EQ(-1984, simulator.Call(buffer, -1984));
}

UNIT_TEST(CompressedMove) {
  Assembler assembler(RV_GC);
  __ mv(A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GC);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "      852e mv a0, a1\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(42, simulator.Call(buffer, 0, 42));
}

UNIT_TEST(CompressedAdd) {
  Assembler assembler(RV_GC);
  __ add(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GC);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "      952e add a0, a0, a1\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(24, simulator.Call(buffer, 7, 17));
  EXPECT_EQ(-10, simulator.Call(buffer, 7, -17));
  EXPECT_EQ(10, simulator.Call(buffer, -7, 17));
  EXPECT_EQ(-24, simulator.Call(buffer, -7, -17));
  EXPECT_EQ(24, simulator.Call(buffer, 17, 7));
  EXPECT_EQ(10, simulator.Call(buffer, 17, -7));
  EXPECT_EQ(-10, simulator.Call(buffer, -17, 7));
  EXPECT_EQ(-24, simulator.Call(buffer, -17, -7));
}

UNIT_TEST(CompressedAnd) {
  Assembler assembler(RV_GC);
  __ and_(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GC);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "      8d6d and a0, a0, a1\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(1, simulator.Call(buffer, 7, 17));
  EXPECT_EQ(7, simulator.Call(buffer, 7, -17));
  EXPECT_EQ(17, simulator.Call(buffer, -7, 17));
  EXPECT_EQ(-23, simulator.Call(buffer, -7, -17));
  EXPECT_EQ(1, simulator.Call(buffer, 17, 7));
  EXPECT_EQ(17, simulator.Call(buffer, 17, -7));
  EXPECT_EQ(7, simulator.Call(buffer, -17, 7));
  EXPECT_EQ(-23, simulator.Call(buffer, -17, -7));
}

UNIT_TEST(CompressedOr) {
  Assembler assembler(RV_GC);
  __ or_(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GC);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "      8d4d or a0, a0, a1\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(23, simulator.Call(buffer, 7, 17));
  EXPECT_EQ(-17, simulator.Call(buffer, 7, -17));
  EXPECT_EQ(-7, simulator.Call(buffer, -7, 17));
  EXPECT_EQ(-1, simulator.Call(buffer, -7, -17));
  EXPECT_EQ(23, simulator.Call(buffer, 17, 7));
  EXPECT_EQ(-7, simulator.Call(buffer, 17, -7));
  EXPECT_EQ(-17, simulator.Call(buffer, -17, 7));
  EXPECT_EQ(-1, simulator.Call(buffer, -17, -7));
}

UNIT_TEST(CompressedXor) {
  Assembler assembler(RV_GC);
  __ xor_(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GC);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "      8d2d xor a0, a0, a1\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(22, simulator.Call(buffer, 7, 17));
  EXPECT_EQ(-24, simulator.Call(buffer, 7, -17));
  EXPECT_EQ(-24, simulator.Call(buffer, -7, 17));
  EXPECT_EQ(22, simulator.Call(buffer, -7, -17));
  EXPECT_EQ(22, simulator.Call(buffer, 17, 7));
  EXPECT_EQ(-24, simulator.Call(buffer, 17, -7));
  EXPECT_EQ(-24, simulator.Call(buffer, -17, 7));
  EXPECT_EQ(22, simulator.Call(buffer, -17, -7));
}

UNIT_TEST(CompressedSubtract) {
  Assembler assembler(RV_GC);
  __ sub(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GC);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "      8d0d sub a0, a0, a1\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(-10, simulator.Call(buffer, 7, 17));
  EXPECT_EQ(24, simulator.Call(buffer, 7, -17));
  EXPECT_EQ(-24, simulator.Call(buffer, -7, 17));
  EXPECT_EQ(10, simulator.Call(buffer, -7, -17));
  EXPECT_EQ(10, simulator.Call(buffer, 17, 7));
  EXPECT_EQ(24, simulator.Call(buffer, 17, -7));
  EXPECT_EQ(-24, simulator.Call(buffer, -17, 7));
  EXPECT_EQ(-10, simulator.Call(buffer, -17, -7));
}

#if XLEN >= 64
UNIT_TEST(CompressedAddWord) {
  Assembler assembler(RV_GC);
  __ addw(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GC);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "      9d2d addw a0, a0, a1\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(24, simulator.Call(buffer, 7, 17));
  EXPECT_EQ(-10, simulator.Call(buffer, 7, -17));
  EXPECT_EQ(10, simulator.Call(buffer, -7, 17));
  EXPECT_EQ(-24, simulator.Call(buffer, -7, -17));
  EXPECT_EQ(24, simulator.Call(buffer, 17, 7));
  EXPECT_EQ(10, simulator.Call(buffer, 17, -7));
  EXPECT_EQ(-10, simulator.Call(buffer, -17, 7));
  EXPECT_EQ(-24, simulator.Call(buffer, -17, -7));
  EXPECT_EQ(3, simulator.Call(buffer, 0x200000002, 0x100000001));
}

UNIT_TEST(CompressedSubtractWord) {
  Assembler assembler(RV_GC);
  __ subw(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GC);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "      9d0d subw a0, a0, a1\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(-10, simulator.Call(buffer, 7, 17));
  EXPECT_EQ(24, simulator.Call(buffer, 7, -17));
  EXPECT_EQ(-24, simulator.Call(buffer, -7, 17));
  EXPECT_EQ(10, simulator.Call(buffer, -7, -17));
  EXPECT_EQ(10, simulator.Call(buffer, 17, 7));
  EXPECT_EQ(24, simulator.Call(buffer, 17, -7));
  EXPECT_EQ(-24, simulator.Call(buffer, -17, 7));
  EXPECT_EQ(-10, simulator.Call(buffer, -17, -7));
  EXPECT_EQ(1, simulator.Call(buffer, 0x200000002, 0x100000001));
}
#endif

UNIT_TEST(CompressedNop) {
  Assembler assembler(RV_GC);
  __ nop();
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GC);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "      0001 nop\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(123, simulator.Call(buffer, 123));
}

UNIT_TEST(CompressedEnvironmentBreak) {
  Assembler assembler(RV_GC);
  __ ebreak();
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GC);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "      9002 ebreak\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  // Not running: would trap.
}

#if XLEN >= 64
UNIT_TEST(AddUnsignedWord) {
  Assembler assembler(RV_GCB);
  __ adduw(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GCB);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  08b5053b add.uw a0, a0, a1\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0x200000001,
            simulator.Call(buffer, 0x1, 0x200000000));
  EXPECT_EQ(0x200000001,
            simulator.Call(buffer, 0x100000001, 0x200000000));
  EXPECT_EQ(0x2FFFFFFFF,
            simulator.Call(buffer, -0x1, 0x200000000));
}
#endif

UNIT_TEST(Shift1Add) {
  Assembler assembler(RV_GCB);
  __ sh1add(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GCB);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  20b52533 sh1add a0, a0, a1\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(1002, simulator.Call(buffer, 1, 1000));
  EXPECT_EQ(1000, simulator.Call(buffer, 0, 1000));
  EXPECT_EQ(998, simulator.Call(buffer, -1, 1000));
}

UNIT_TEST(Shift2Add) {
  Assembler assembler(RV_GCB);
  __ sh2add(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GCB);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  20b54533 sh2add a0, a0, a1\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(1004, simulator.Call(buffer, 1, 1000));
  EXPECT_EQ(1000, simulator.Call(buffer, 0, 1000));
  EXPECT_EQ(996, simulator.Call(buffer, -1, 1000));
}

UNIT_TEST(Shift3Add) {
  Assembler assembler(RV_GCB);
  __ sh3add(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GCB);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  20b56533 sh3add a0, a0, a1\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(1008, simulator.Call(buffer, 1, 1000));
  EXPECT_EQ(1000, simulator.Call(buffer, 0, 1000));
  EXPECT_EQ(992, simulator.Call(buffer, -1, 1000));
}

#if XLEN >= 64
UNIT_TEST(Shift1AddUnsignedWord) {
  Assembler assembler(RV_GCB);
  __ sh1adduw(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GCB);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  20b5253b sh1add.uw a0, a0, a1\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(1002, simulator.Call(buffer, 1, 1000));
  EXPECT_EQ(1002, simulator.Call(buffer, 0x100000001, 1000));
  EXPECT_EQ(1000, simulator.Call(buffer, 0, 1000));
  EXPECT_EQ(8589935590, simulator.Call(buffer, -1, 1000));
}

UNIT_TEST(Shift2AddUnsignedWord) {
  Assembler assembler(RV_GCB);
  __ sh2adduw(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GCB);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  20b5453b sh2add.uw a0, a0, a1\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(1004, simulator.Call(buffer, 1, 1000));
  EXPECT_EQ(1004, simulator.Call(buffer, 0x100000001, 1000));
  EXPECT_EQ(1000, simulator.Call(buffer, 0, 1000));
  EXPECT_EQ(17179870180, simulator.Call(buffer, -1, 1000));
}

UNIT_TEST(Shift3AddUnsignedWord) {
  Assembler assembler(RV_GCB);
  __ sh3adduw(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GCB);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  20b5653b sh3add.uw a0, a0, a1\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(1008, simulator.Call(buffer, 1, 1000));
  EXPECT_EQ(1008, simulator.Call(buffer, 0x100000001, 1000));
  EXPECT_EQ(1000, simulator.Call(buffer, 0, 1000));
  EXPECT_EQ(34359739360, simulator.Call(buffer, -1, 1000));
}

UNIT_TEST(ShiftLeftLogicalImmediateUnsignedWord) {
  Assembler assembler(RV_GCB);
  __ slliuw(A0, A0, 8);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GCB);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  0885151b slli.uw a0, a0, 0x8\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0x100, simulator.Call(buffer, 0x1));
  EXPECT_EQ(0x1000000000, simulator.Call(buffer, 0x10000000));
  EXPECT_EQ(0, simulator.Call(buffer, 0x100000000));
  EXPECT_EQ(0x100, simulator.Call(buffer, 0x100000001));
}
#endif

UNIT_TEST(AndNot) {
  Assembler assembler(RV_GCB);
  __ andn(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GCB);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  40b57533 andn a0, a0, a1\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(6, simulator.Call(buffer, 7, 17));
  EXPECT_EQ(0, simulator.Call(buffer, 7, -17));
  EXPECT_EQ(-24, simulator.Call(buffer, -7, 17));
  EXPECT_EQ(16, simulator.Call(buffer, -7, -17));
  EXPECT_EQ(16, simulator.Call(buffer, 17, 7));
  EXPECT_EQ(0, simulator.Call(buffer, 17, -7));
  EXPECT_EQ(-24, simulator.Call(buffer, -17, 7));
  EXPECT_EQ(6, simulator.Call(buffer, -17, -7));
}

UNIT_TEST(OrNot) {
  Assembler assembler(RV_GCB);
  __ orn(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GCB);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  40b56533 orn a0, a0, a1\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(-17, simulator.Call(buffer, 7, 17));
  EXPECT_EQ(23, simulator.Call(buffer, 7, -17));
  EXPECT_EQ(-1, simulator.Call(buffer, -7, 17));
  EXPECT_EQ(-7, simulator.Call(buffer, -7, -17));
  EXPECT_EQ(-7, simulator.Call(buffer, 17, 7));
  EXPECT_EQ(23, simulator.Call(buffer, 17, -7));
  EXPECT_EQ(-1, simulator.Call(buffer, -17, 7));
  EXPECT_EQ(-17, simulator.Call(buffer, -17, -7));
}

UNIT_TEST(XorNot) {
  Assembler assembler(RV_GCB);
  __ xnor(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GCB);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  40b54533 xnor a0, a0, a1\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(-23, simulator.Call(buffer, 7, 17));
  EXPECT_EQ(23, simulator.Call(buffer, 7, -17));
  EXPECT_EQ(23, simulator.Call(buffer, -7, 17));
  EXPECT_EQ(-23, simulator.Call(buffer, -7, -17));
  EXPECT_EQ(-23, simulator.Call(buffer, 17, 7));
  EXPECT_EQ(23, simulator.Call(buffer, 17, -7));
  EXPECT_EQ(23, simulator.Call(buffer, -17, 7));
  EXPECT_EQ(-23, simulator.Call(buffer, -17, -7));
}

UNIT_TEST(CountLeadingZeroes) {
  Assembler assembler(RV_GCB);
  __ clz(A0, A0);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GCB);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  60051513 clz a0, a0\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(XLEN, simulator.Call(buffer, 0));
  EXPECT_EQ(XLEN-1, simulator.Call(buffer, 1));
  EXPECT_EQ(XLEN-2, simulator.Call(buffer, 2));
  EXPECT_EQ(XLEN-3, simulator.Call(buffer, 4));
  EXPECT_EQ(XLEN-8, simulator.Call(buffer, 240));
  EXPECT_EQ(0, simulator.Call(buffer, -1));
  EXPECT_EQ(0, simulator.Call(buffer, -2));
  EXPECT_EQ(0, simulator.Call(buffer, -4));
  EXPECT_EQ(0, simulator.Call(buffer, -240));
}

UNIT_TEST(CountTrailingZeroes) {
  Assembler assembler(RV_GCB);
  __ ctz(A0, A0);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GCB);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  60151513 ctz a0, a0\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(XLEN, simulator.Call(buffer, 0));
  EXPECT_EQ(0, simulator.Call(buffer, 1));
  EXPECT_EQ(1, simulator.Call(buffer, 2));
  EXPECT_EQ(2, simulator.Call(buffer, 4));
  EXPECT_EQ(4, simulator.Call(buffer, 240));
  EXPECT_EQ(0, simulator.Call(buffer, -1));
  EXPECT_EQ(1, simulator.Call(buffer, -2));
  EXPECT_EQ(2, simulator.Call(buffer, -4));
  EXPECT_EQ(4, simulator.Call(buffer, -240));
}

UNIT_TEST(CountPopulation) {
  Assembler assembler(RV_GCB);
  __ cpop(A0, A0);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GCB);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  60251513 cpop a0, a0\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0, simulator.Call(buffer, 0));
  EXPECT_EQ(1, simulator.Call(buffer, 1));
  EXPECT_EQ(3, simulator.Call(buffer, 7));
  EXPECT_EQ(4, simulator.Call(buffer, 30));
  EXPECT_EQ(XLEN, simulator.Call(buffer, -1));
  EXPECT_EQ(XLEN-2, simulator.Call(buffer, -7));
  EXPECT_EQ(XLEN-4, simulator.Call(buffer, -30));
}

#if XLEN >= 64
UNIT_TEST(CountLeadingZeroesWord) {
  Assembler assembler(RV_GCB);
  __ clzw(A0, A0);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GCB);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  6005151b clzw a0, a0\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(32, simulator.Call(buffer, 0));
  EXPECT_EQ(31, simulator.Call(buffer, 1));
  EXPECT_EQ(30, simulator.Call(buffer, 2));
  EXPECT_EQ(29, simulator.Call(buffer, 4));
  EXPECT_EQ(24, simulator.Call(buffer, 240));
  EXPECT_EQ(0, simulator.Call(buffer, -1));
  EXPECT_EQ(0, simulator.Call(buffer, -2));
  EXPECT_EQ(0, simulator.Call(buffer, -4));
  EXPECT_EQ(0, simulator.Call(buffer, -240));
}

UNIT_TEST(CountTrailingZeroesWord) {
  Assembler assembler(RV_GCB);
  __ ctzw(A0, A0);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GCB);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  6015151b ctzw a0, a0\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(32, simulator.Call(buffer, 0));
  EXPECT_EQ(0, simulator.Call(buffer, 1));
  EXPECT_EQ(1, simulator.Call(buffer, 2));
  EXPECT_EQ(2, simulator.Call(buffer, 4));
  EXPECT_EQ(4, simulator.Call(buffer, 240));
  EXPECT_EQ(0, simulator.Call(buffer, -1));
  EXPECT_EQ(1, simulator.Call(buffer, -2));
  EXPECT_EQ(2, simulator.Call(buffer, -4));
  EXPECT_EQ(4, simulator.Call(buffer, -240));
}

UNIT_TEST(CountPopulationWord) {
  Assembler assembler(RV_GCB);
  __ cpopw(A0, A0);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GCB);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  6025151b cpopw a0, a0\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0, simulator.Call(buffer, 0));
  EXPECT_EQ(1, simulator.Call(buffer, 1));
  EXPECT_EQ(3, simulator.Call(buffer, 7));
  EXPECT_EQ(4, simulator.Call(buffer, 30));
  EXPECT_EQ(32, simulator.Call(buffer, -1));
  EXPECT_EQ(30, simulator.Call(buffer, -7));
  EXPECT_EQ(28, simulator.Call(buffer, -30));
  EXPECT_EQ(0, simulator.Call(buffer, 0x7FFFFFFF00000000));
}
#endif

UNIT_TEST(Max) {
  Assembler assembler(RV_GCB);
  __ max(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GCB);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  0ab56533 max a0, a0, a1\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(17, simulator.Call(buffer, 7, 17));
  EXPECT_EQ(17, simulator.Call(buffer, -7, 17));
  EXPECT_EQ(7, simulator.Call(buffer, 7, -17));
  EXPECT_EQ(-7, simulator.Call(buffer, -7, -17));
}

UNIT_TEST(MaxUnsigned) {
  Assembler assembler(RV_GCB);
  __ maxu(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GCB);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  0ab57533 maxu a0, a0, a1\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(17, simulator.Call(buffer, 7, 17));
  EXPECT_EQ(-7, simulator.Call(buffer, -7, 17));
  EXPECT_EQ(-17, simulator.Call(buffer, 7, -17));
  EXPECT_EQ(-7, simulator.Call(buffer, -7, -17));
}

UNIT_TEST(Min) {
  Assembler assembler(RV_GCB);
  __ min(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GCB);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  0ab54533 min a0, a0, a1\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(7, simulator.Call(buffer, 7, 17));
  EXPECT_EQ(-7, simulator.Call(buffer, -7, 17));
  EXPECT_EQ(-17, simulator.Call(buffer, 7, -17));
  EXPECT_EQ(-17, simulator.Call(buffer, -7, -17));
}

UNIT_TEST(MinUnsigned) {
  Assembler assembler(RV_GCB);
  __ minu(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GCB);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  0ab55533 minu a0, a0, a1\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(7, simulator.Call(buffer, 7, 17));
  EXPECT_EQ(17, simulator.Call(buffer, -7, 17));
  EXPECT_EQ(7, simulator.Call(buffer, 7, -17));
  EXPECT_EQ(-17, simulator.Call(buffer, -7, -17));
}

UNIT_TEST(SignExtendByte) {
  Assembler assembler(RV_GCB);
  __ sextb(A0, A0);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GCB);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  60451513 sext.b a0, a0\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(1, simulator.Call(buffer, 1));
  EXPECT_EQ(127, simulator.Call(buffer, 127));
  EXPECT_EQ(-128, simulator.Call(buffer, 128));
}

UNIT_TEST(SignExtendHalfWord) {
  Assembler assembler(RV_GCB);
  __ sexth(A0, A0);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GCB);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  60551513 sext.h a0, a0\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0, simulator.Call(buffer, 0));
  EXPECT_EQ(0x7BCD, simulator.Call(buffer, 0x12347BCD));
  EXPECT_EQ(-1, simulator.Call(buffer, 0xFFFF));
  EXPECT_EQ(-1, simulator.Call(buffer, -1));
}

UNIT_TEST(ZeroExtendHalfWord) {
  Assembler assembler(RV_GCB);
  __ zexth(A0, A0);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GCB);
  char* disassembly = disassembler.Disassemble(buffer, size);
#if XLEN == 32
  EXPECT_STREQ(
      "  08054533 zext.h a0, a0\n"
      "      8082 ret\n",
      disassembly);
#else
  EXPECT_STREQ(
      "  0805453b zext.h a0, a0\n"
      "      8082 ret\n",
      disassembly);
#endif
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0, simulator.Call(buffer, 0));
  EXPECT_EQ(0xABCD, simulator.Call(buffer, 0x1234ABCD));
  EXPECT_EQ(0xFFFF, simulator.Call(buffer, 0xFFFF));
  EXPECT_EQ(0xFFFF, simulator.Call(buffer, -1));
}

UNIT_TEST(RotateRight) {
  Assembler assembler(RV_GCB);
  __ ror(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GCB);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  60b55533 ror a0, a0, a1\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
#if XLEN == 32
  EXPECT_EQ(static_cast<intx_t>(0x12345678),
            simulator.Call(buffer, 0x12345678, 0));
  EXPECT_EQ(static_cast<intx_t>(0x81234567),
            simulator.Call(buffer, 0x12345678, 4));
  EXPECT_EQ(static_cast<intx_t>(0x23456781),
            simulator.Call(buffer, 0x12345678, 28));
  EXPECT_EQ(static_cast<intx_t>(0x81234567),
            simulator.Call(buffer, 0x12345678, 36));
#else
  EXPECT_EQ(static_cast<intx_t>(0x0123456789ABCDEF),
            simulator.Call(buffer, 0x0123456789ABCDEF, 0));
  EXPECT_EQ(static_cast<intx_t>(0xF0123456789ABCDE),
            simulator.Call(buffer, 0x0123456789ABCDEF, 4));
  EXPECT_EQ(static_cast<intx_t>(0x123456789ABCDEF0),
            simulator.Call(buffer, 0x0123456789ABCDEF, 60));
  EXPECT_EQ(static_cast<intx_t>(0xF0123456789ABCDE),
            simulator.Call(buffer, 0x0123456789ABCDEF, 68));
#endif
}

UNIT_TEST(RotateLeft) {
  Assembler assembler(RV_GCB);
  __ rol(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GCB);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  60b51533 rol a0, a0, a1\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
#if XLEN == 32
  EXPECT_EQ(static_cast<intx_t>(0x12345678),
            simulator.Call(buffer, 0x12345678, 0));
  EXPECT_EQ(static_cast<intx_t>(0x23456781),
            simulator.Call(buffer, 0x12345678, 4));
  EXPECT_EQ(static_cast<intx_t>(0x81234567),
            simulator.Call(buffer, 0x12345678, 28));
  EXPECT_EQ(static_cast<intx_t>(0x23456781),
            simulator.Call(buffer, 0x12345678, 36));
#else
  EXPECT_EQ(static_cast<intx_t>(0x0123456789ABCDEF),
            simulator.Call(buffer, 0x0123456789ABCDEF, 0));
  EXPECT_EQ(static_cast<intx_t>(0x123456789ABCDEF0),
            simulator.Call(buffer, 0x0123456789ABCDEF, 4));
  EXPECT_EQ(static_cast<intx_t>(0xF0123456789ABCDE),
            simulator.Call(buffer, 0x0123456789ABCDEF, 60));
  EXPECT_EQ(static_cast<intx_t>(0x123456789ABCDEF0),
            simulator.Call(buffer, 0x0123456789ABCDEF, 68));
#endif
}

UNIT_TEST(RotateRightImmediate) {
  Assembler assembler(RV_GCB);
  __ rori(A0, A0, 4);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GCB);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  60455513 rori a0, a0, 0x4\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
#if XLEN == 32
  EXPECT_EQ(static_cast<intx_t>(0x81234567),
            simulator.Call(buffer, 0x12345678));
#else
  EXPECT_EQ(static_cast<intx_t>(0xF0123456789ABCDE),
            simulator.Call(buffer, 0x0123456789ABCDEF));
#endif
}

#if XLEN >= 64
UNIT_TEST(RotateRightWord) {
  Assembler assembler(RV_GCB);
  __ rorw(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GCB);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  60b5553b rorw a0, a0, a1\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(sign_extend(0x12345678),
            simulator.Call(buffer, 0x12345678, 0));
  EXPECT_EQ(sign_extend(0x81234567),
            simulator.Call(buffer, 0x12345678, 4));
  EXPECT_EQ(sign_extend(0x23456781),
            simulator.Call(buffer, 0x12345678, 28));
  EXPECT_EQ(sign_extend(0x81234567),
            simulator.Call(buffer, 0x12345678, 36));
}

UNIT_TEST(RotateLeftWord) {
  Assembler assembler(RV_GCB);
  __ rolw(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GCB);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  60b5153b rolw a0, a0, a1\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(sign_extend(0x12345678),
            simulator.Call(buffer, 0x12345678, 0));
  EXPECT_EQ(sign_extend(0x23456781),
            simulator.Call(buffer, 0x12345678, 4));
  EXPECT_EQ(sign_extend(0x81234567),
            simulator.Call(buffer, 0x12345678, 28));
  EXPECT_EQ(sign_extend(0x23456781),
            simulator.Call(buffer, 0x12345678, 36));
}

UNIT_TEST(RotateRightImmediateWord) {
  Assembler assembler(RV_GCB);
  __ roriw(A0, A0, 4);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GCB);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  6045551b roriw a0, a0, 0x4\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(sign_extend(0x81234567), simulator.Call(buffer, 0x12345678));
}
#endif

UNIT_TEST(OrCombineBytes) {
  Assembler assembler(RV_GCB);
  __ orcb(A0, A0);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GCB);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  28755513 orc.b a0, a0\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0, simulator.Call(buffer, 0));
  EXPECT_EQ(-1, simulator.Call(buffer, -1));
  EXPECT_EQ(0x00FF00FF, simulator.Call(buffer, 0x00010001));
#if XLEN >= 64
  EXPECT_EQ(0x00FF00FF00FF00FF, simulator.Call(buffer, 0x0001000100010001));
#endif
}

UNIT_TEST(ByteReverse) {
  Assembler assembler(RV_GCB);
  __ rev8(A0, A0);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GCB);
  char* disassembly = disassembler.Disassemble(buffer, size);
#if XLEN == 32
  EXPECT_STREQ(
      "  69855513 rev8 a0, a0\n"
      "      8082 ret\n",
      disassembly);
#else
  EXPECT_STREQ(
      "  6b855513 rev8 a0, a0\n"
      "      8082 ret\n",
      disassembly);
#endif
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0, simulator.Call(buffer, 0));
  EXPECT_EQ(-1, simulator.Call(buffer, -1));
#if XLEN == 32
  EXPECT_EQ(0x11223344, simulator.Call(buffer, 0x44332211));
#elif XLEN == 64
  EXPECT_EQ(0x1122334455667788, simulator.Call(buffer, 0x8877665544332211));
#endif
}

UNIT_TEST(CarrylessMultiply) {
  Assembler assembler(RV_GC | RV_Zbc);
  __ clmul(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GC | RV_Zbc);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  0ab51533 clmul a0, a0, a1\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
#if XLEN == 32
  EXPECT_EQ(0x55555555, simulator.Call(buffer, -1, -1));
#else
  EXPECT_EQ(0x5555555555555555, simulator.Call(buffer, -1, -1));
#endif
  EXPECT_EQ(0, simulator.Call(buffer, -1, 0));
  EXPECT_EQ(-1, simulator.Call(buffer, -1, 1));
  EXPECT_EQ(0, simulator.Call(buffer, 0, -1));
  EXPECT_EQ(0, simulator.Call(buffer, 0, 0));
  EXPECT_EQ(0, simulator.Call(buffer, 0, 1));
  EXPECT_EQ(-1, simulator.Call(buffer, 1, -1));
  EXPECT_EQ(0, simulator.Call(buffer, 1, 0));
  EXPECT_EQ(1, simulator.Call(buffer, 1, 1));

  EXPECT_EQ(4, simulator.Call(buffer, 2, 2));
  EXPECT_EQ(5, simulator.Call(buffer, 3, 3));
  EXPECT_EQ(16, simulator.Call(buffer, 4, 4));
  EXPECT_EQ(20, simulator.Call(buffer, 6, 6));
}

UNIT_TEST(CarrylessMultiplyHigh) {
  Assembler assembler(RV_GC | RV_Zbc);
  __ clmulh(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GC | RV_Zbc);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  0ab53533 clmulh a0, a0, a1\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
#if XLEN == 32
  EXPECT_EQ(0x55555555, simulator.Call(buffer, -1, -1));
#else
  EXPECT_EQ(0x5555555555555555, simulator.Call(buffer, -1, -1));
#endif
  EXPECT_EQ(0, simulator.Call(buffer, -1, 0));
  EXPECT_EQ(0, simulator.Call(buffer, -1, 1));
  EXPECT_EQ(0, simulator.Call(buffer, 0, -1));
  EXPECT_EQ(0, simulator.Call(buffer, 0, 0));
  EXPECT_EQ(0, simulator.Call(buffer, 0, 1));
  EXPECT_EQ(0, simulator.Call(buffer, 1, -1));
  EXPECT_EQ(0, simulator.Call(buffer, 1, 0));
  EXPECT_EQ(0, simulator.Call(buffer, 1, 1));

  EXPECT_EQ(0, simulator.Call(buffer, 2, 2));
  EXPECT_EQ(0, simulator.Call(buffer, 3, 3));
  EXPECT_EQ(0, simulator.Call(buffer, 4, 4));
  EXPECT_EQ(0, simulator.Call(buffer, 6, 6));
}

UNIT_TEST(CarrylessMultiplyReversed) {
  Assembler assembler(RV_GC | RV_Zbc);
  __ clmulr(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GC | RV_Zbc);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  0ab52533 clmulr a0, a0, a1\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
#if XLEN == 32
  EXPECT_EQ(-0x55555556, simulator.Call(buffer, -1, -1));
#else
  EXPECT_EQ(-0x5555555555555556, simulator.Call(buffer, -1, -1));
#endif
  EXPECT_EQ(0, simulator.Call(buffer, -1, 0));
  EXPECT_EQ(1, simulator.Call(buffer, -1, 1));
  EXPECT_EQ(0, simulator.Call(buffer, 0, -1));
  EXPECT_EQ(0, simulator.Call(buffer, 0, 0));
  EXPECT_EQ(0, simulator.Call(buffer, 0, 1));
  EXPECT_EQ(1, simulator.Call(buffer, 1, -1));
  EXPECT_EQ(0, simulator.Call(buffer, 1, 0));
  EXPECT_EQ(0, simulator.Call(buffer, 1, 1));

  EXPECT_EQ(0, simulator.Call(buffer, 2, 2));
  EXPECT_EQ(0, simulator.Call(buffer, 3, 3));
  EXPECT_EQ(0, simulator.Call(buffer, 4, 4));
  EXPECT_EQ(0, simulator.Call(buffer, 6, 6));
}

UNIT_TEST(BitClear) {
  Assembler assembler(RV_GCB);
  __ bclr(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GCB);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  48b51533 bclr a0, a0, a1\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(42, simulator.Call(buffer, 42, 0));
  EXPECT_EQ(40, simulator.Call(buffer, 42, 1));
  EXPECT_EQ(42, simulator.Call(buffer, 42, 2));
  EXPECT_EQ(34, simulator.Call(buffer, 42, 3));
  EXPECT_EQ(42, simulator.Call(buffer, 42, 4));
  EXPECT_EQ(10, simulator.Call(buffer, 42, 5));
  EXPECT_EQ(42, simulator.Call(buffer, 42, 6));
  EXPECT_EQ(42, simulator.Call(buffer, 42, 7));
  EXPECT_EQ(42, simulator.Call(buffer, 42, 8));

  EXPECT_EQ(42, simulator.Call(buffer, 42, 64));
  EXPECT_EQ(40, simulator.Call(buffer, 42, 65));
}

UNIT_TEST(BitClearImmediate) {
  Assembler assembler(RV_GCB);
  __ bclri(A0, A0, 3);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GCB);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  48351513 bclri a0, a0, 0x3\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0, simulator.Call(buffer, 0));
  EXPECT_EQ(7, simulator.Call(buffer, 7));
  EXPECT_EQ(0, simulator.Call(buffer, 8));
  EXPECT_EQ(1, simulator.Call(buffer, 9));
  EXPECT_EQ(-15, simulator.Call(buffer, -7));
  EXPECT_EQ(-16, simulator.Call(buffer, -8));
  EXPECT_EQ(-9, simulator.Call(buffer, -9));
}

UNIT_TEST(BitClearImmediate2) {
  Assembler assembler(RV_GCB);
  __ bclri(A0, A0, XLEN - 1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GCB);
  char* disassembly = disassembler.Disassemble(buffer, size);
#if XLEN == 32
  EXPECT_STREQ(
      "  49f51513 bclri a0, a0, 0x1f\n"
      "      8082 ret\n",
      disassembly);
#elif XLEN == 64
  EXPECT_STREQ(
      "  4bf51513 bclri a0, a0, 0x3f\n"
      "      8082 ret\n",
      disassembly);
#endif
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0, simulator.Call(buffer, 0));
  EXPECT_EQ(1, simulator.Call(buffer, 1));
  EXPECT_EQ(kMaxIntX, simulator.Call(buffer, -1));
}

UNIT_TEST(BitExtract) {
  Assembler assembler(RV_GCB);
  __ bext(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GCB);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  48b55533 bext a0, a0, a1\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0, simulator.Call(buffer, 42, 0));
  EXPECT_EQ(1, simulator.Call(buffer, 42, 1));
  EXPECT_EQ(0, simulator.Call(buffer, 42, 2));
  EXPECT_EQ(1, simulator.Call(buffer, 42, 3));
  EXPECT_EQ(0, simulator.Call(buffer, 42, 4));
  EXPECT_EQ(1, simulator.Call(buffer, 42, 5));
  EXPECT_EQ(0, simulator.Call(buffer, 42, 6));
  EXPECT_EQ(0, simulator.Call(buffer, 42, 7));
  EXPECT_EQ(0, simulator.Call(buffer, 42, 8));

  EXPECT_EQ(0, simulator.Call(buffer, 42, 64));
  EXPECT_EQ(1, simulator.Call(buffer, 42, 65));
}

UNIT_TEST(BitExtractImmediate) {
  Assembler assembler(RV_GCB);
  __ bexti(A0, A0, 3);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GCB);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  48355513 bexti a0, a0, 0x3\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0, simulator.Call(buffer, 0));
  EXPECT_EQ(0, simulator.Call(buffer, 7));
  EXPECT_EQ(1, simulator.Call(buffer, 8));
  EXPECT_EQ(1, simulator.Call(buffer, 9));
  EXPECT_EQ(1, simulator.Call(buffer, -7));
  EXPECT_EQ(1, simulator.Call(buffer, -8));
  EXPECT_EQ(0, simulator.Call(buffer, -9));
}

UNIT_TEST(BitExtractImmediate2) {
  Assembler assembler(RV_GCB);
  __ bexti(A0, A0, XLEN - 1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GCB);
  char* disassembly = disassembler.Disassemble(buffer, size);
#if XLEN == 32
  EXPECT_STREQ(
      "  49f55513 bexti a0, a0, 0x1f\n"
      "      8082 ret\n",
      disassembly);
#elif XLEN == 64
  EXPECT_STREQ(
      "  4bf55513 bexti a0, a0, 0x3f\n"
      "      8082 ret\n",
      disassembly);
#endif
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(0, simulator.Call(buffer, 0));
  EXPECT_EQ(0, simulator.Call(buffer, 1));
  EXPECT_EQ(1, simulator.Call(buffer, -1));
}

UNIT_TEST(BitInvert) {
  Assembler assembler(RV_GCB);
  __ binv(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GCB);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  68b51533 binv a0, a0, a1\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(43, simulator.Call(buffer, 42, 0));
  EXPECT_EQ(40, simulator.Call(buffer, 42, 1));
  EXPECT_EQ(46, simulator.Call(buffer, 42, 2));
  EXPECT_EQ(34, simulator.Call(buffer, 42, 3));
  EXPECT_EQ(58, simulator.Call(buffer, 42, 4));
  EXPECT_EQ(10, simulator.Call(buffer, 42, 5));
  EXPECT_EQ(106, simulator.Call(buffer, 42, 6));
  EXPECT_EQ(170, simulator.Call(buffer, 42, 7));
  EXPECT_EQ(298, simulator.Call(buffer, 42, 8));

  EXPECT_EQ(43, simulator.Call(buffer, 42, 64));
  EXPECT_EQ(40, simulator.Call(buffer, 42, 65));
}

UNIT_TEST(BitInvertImmediate) {
  Assembler assembler(RV_GCB);
  __ binvi(A0, A0, 3);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GCB);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  68351513 binvi a0, a0, 0x3\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(8, simulator.Call(buffer, 0));
  EXPECT_EQ(15, simulator.Call(buffer, 7));
  EXPECT_EQ(0, simulator.Call(buffer, 8));
  EXPECT_EQ(1, simulator.Call(buffer, 9));
  EXPECT_EQ(-15, simulator.Call(buffer, -7));
  EXPECT_EQ(-16, simulator.Call(buffer, -8));
  EXPECT_EQ(-1, simulator.Call(buffer, -9));
}

UNIT_TEST(BitInvertImmediate2) {
  Assembler assembler(RV_GCB);
  __ binvi(A0, A0, XLEN - 1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GCB);
  char* disassembly = disassembler.Disassemble(buffer, size);
#if XLEN == 32
  EXPECT_STREQ(
      "  69f51513 binvi a0, a0, 0x1f\n"
      "      8082 ret\n",
      disassembly);
#elif XLEN == 64
  EXPECT_STREQ(
      "  6bf51513 binvi a0, a0, 0x3f\n"
      "      8082 ret\n",
      disassembly);
#endif
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(kMinIntX, simulator.Call(buffer, 0));
  EXPECT_EQ(kMinIntX + 1, simulator.Call(buffer, 1));
  EXPECT_EQ(kMaxIntX, simulator.Call(buffer, -1));
}

UNIT_TEST(BitSet) {
  Assembler assembler(RV_GCB);
  __ bset(A0, A0, A1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GCB);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  28b51533 bset a0, a0, a1\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(43, simulator.Call(buffer, 42, 0));
  EXPECT_EQ(42, simulator.Call(buffer, 42, 1));
  EXPECT_EQ(46, simulator.Call(buffer, 42, 2));
  EXPECT_EQ(42, simulator.Call(buffer, 42, 3));
  EXPECT_EQ(58, simulator.Call(buffer, 42, 4));
  EXPECT_EQ(42, simulator.Call(buffer, 42, 5));
  EXPECT_EQ(106, simulator.Call(buffer, 42, 6));
  EXPECT_EQ(170, simulator.Call(buffer, 42, 7));
  EXPECT_EQ(298, simulator.Call(buffer, 42, 8));

  EXPECT_EQ(43, simulator.Call(buffer, 42, 64));
  EXPECT_EQ(42, simulator.Call(buffer, 42, 65));
}

UNIT_TEST(BitSetImmediate) {
  Assembler assembler(RV_GCB);
  __ bseti(A0, A0, 3);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GCB);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  28351513 bseti a0, a0, 0x3\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(8, simulator.Call(buffer, 0));
  EXPECT_EQ(15, simulator.Call(buffer, 7));
  EXPECT_EQ(8, simulator.Call(buffer, 8));
  EXPECT_EQ(9, simulator.Call(buffer, 9));
  EXPECT_EQ(-7, simulator.Call(buffer, -7));
  EXPECT_EQ(-8, simulator.Call(buffer, -8));
  EXPECT_EQ(-1, simulator.Call(buffer, -9));
}

UNIT_TEST(BitSetImmediate2) {
  Assembler assembler(RV_GCB);
  __ bseti(A0, A0, XLEN - 1);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GCB);
  char* disassembly = disassembler.Disassemble(buffer, size);
#if XLEN == 32
  EXPECT_STREQ(
      "  29f51513 bseti a0, a0, 0x1f\n"
      "      8082 ret\n",
      disassembly);
#elif XLEN == 64
  EXPECT_STREQ(
      "  2bf51513 bseti a0, a0, 0x3f\n"
      "      8082 ret\n",
      disassembly);
#endif
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(kMinIntX, simulator.Call(buffer, 0));
  EXPECT_EQ(kMinIntX + 1, simulator.Call(buffer, 1));
  EXPECT_EQ(-1, simulator.Call(buffer, -1));
}

UNIT_TEST(MacroLoadImmediate_MaxInt32) {
  MacroAssembler assembler(RV_GC);
  __ LoadImmediate(A0, kMaxInt32);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GC);
  char* disassembly = disassembler.Disassemble(buffer, size);
#if XLEN == 32
  EXPECT_STREQ(
      "  80000537 lui a0, -2147483648\n"
      "      157d addi a0, a0, -1\n"
      "      8082 ret\n",
      disassembly);
#elif XLEN == 64
  EXPECT_STREQ(
      "  80000537 lui a0, -2147483648\n"
      "      357d addiw a0, a0, -1\n"
      "      8082 ret\n",
      disassembly);
#endif
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(kMaxInt32, simulator.Call(buffer));
}

UNIT_TEST(MacroLoadImmediate_MinInt32) {
  MacroAssembler assembler(RV_GC);
  __ LoadImmediate(A0, kMinInt32);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GC);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  80000537 lui a0, -2147483648\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(kMinInt32, simulator.Call(buffer));
}

UNIT_TEST(MacroAddBranchOverflow) {
  MacroAssembler assembler(RV_GC);
  Label overflow;
  __ AddBranchOverflow(A0, A1, A0, &overflow);
  __ ret();
  __ Bind(&overflow);
  __ li(A0, 0);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GC);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  00052793 slti a5, a0, 0\n"
      "      952e add a0, a0, a1\n"
      "  00b52833 slt a6, a0, a1\n"
      "  01079363 bne a5, a6, +6\n"
      "      8082 ret\n"
      "      4501 li a0, 0\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(kMaxIntX-1, simulator.Call(buffer, kMaxIntX, -1));
  EXPECT_EQ(kMaxIntX, simulator.Call(buffer, kMaxIntX, 0));
  EXPECT_EQ(0, simulator.Call(buffer, kMaxIntX, 1));

  EXPECT_EQ(0, simulator.Call(buffer, kMinIntX, -1));
  EXPECT_EQ(kMinIntX+1, simulator.Call(buffer, kMinIntX, 1));
  EXPECT_EQ(kMinIntX, simulator.Call(buffer, kMinIntX, 0));
}

  /*
UNIT_TEST(MacroSubBranchOverflow) {
  MacroAssembler assembler(RV_GC);
  Label overflow;
  __ SubBranchOverflow(A0, A1, A2, &overflow);
  __ ret();
  __ Bind(&overflow);
  __ li(A0, 0);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GC);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  00052793 slti a5, a0, 0\n"
      "      952e add a0, a0, a1\n"
      "  00b52833 slt a6, a0, a1\n"
      "  01079363 bne a5, a6, +6\n"
      "      8082 ret\n"
      "      4501 li a0, 0\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(kMaxIntX-1, simulator.Call(buffer, 0, kMaxIntX, -1));
  EXPECT_EQ(kMaxIntX, simulator.Call(buffer, 0, kMaxIntX, 0));
  EXPECT_EQ(0, simulator.Call(buffer, 0, kMaxIntX, 1));

  EXPECT_EQ(0, simulator.Call(buffer, 0, kMinIntX, -1));
  EXPECT_EQ(kMinIntX+1, simulator.Call(buffer, 0, kMinIntX, 1));
  EXPECT_EQ(kMinIntX, simulator.Call(buffer, 0, kMinIntX, 0));
}

UNIT_TEST(MacroMulBranchOverflow) {
  MacroAssembler assembler(RV_GC);
  Label overflow;
  __ MulBranchOverflow(A0, A1, A2, &overflow);
  __ ret();
  __ Bind(&overflow);
  __ li(A0, 0);
  __ ret();

  void* buffer = assembler.buffer();
  size_t size = assembler.size();

  Disassembler disassembler(RV_GC);
  char* disassembly = disassembler.Disassemble(buffer, size);
  EXPECT_STREQ(
      "  00052793 slti a5, a0, 0\n"
      "      952e add a0, a0, a1\n"
      "  00b52833 slt a6, a0, a1\n"
      "  01079363 bne a5, a6, +6\n"
      "      8082 ret\n"
      "      4501 li a0, 0\n"
      "      8082 ret\n",
      disassembly);
  free(disassembly);

  Simulator simulator;
  EXPECT_EQ(kMaxIntX-1, simulator.Call(buffer, 0, kMaxIntX, -1));
  EXPECT_EQ(kMaxIntX, simulator.Call(buffer, 0, kMaxIntX, 0));
  EXPECT_EQ(0, simulator.Call(buffer, 0, kMaxIntX, 1));

  EXPECT_EQ(0, simulator.Call(buffer, 0, kMinIntX, -1));
  EXPECT_EQ(kMinIntX+1, simulator.Call(buffer, 0, kMinIntX, 1));
  EXPECT_EQ(kMinIntX, simulator.Call(buffer, 0, kMinIntX, 0));
  }*/

#define TEST_ENCODING(type, name)                                         \
  UNIT_TEST(name##Encoding) {                                                  \
    for (intptr_t v = -(1 << 21); v <= (1 << 21); v++) {                       \
      type value = static_cast<type>(v);                                       \
      if (!Is##name(value)) continue;                                          \
      int32_t encoded = Encode##name(value);                                   \
      type decoded = Decode##name(encoded);                                    \
      EXPECT_EQ(value, decoded);                                               \
    }                                                                          \
  }                                                                            \

TEST_ENCODING(Register, Rd)
TEST_ENCODING(Register, Rs1)
TEST_ENCODING(Register, Rs2)
TEST_ENCODING(FRegister, FRd)
TEST_ENCODING(FRegister, FRs1)
TEST_ENCODING(FRegister, FRs2)
TEST_ENCODING(FRegister, FRs3)
TEST_ENCODING(Funct2, Funct2)
TEST_ENCODING(Funct3, Funct3)
TEST_ENCODING(Funct5, Funct5)
TEST_ENCODING(Funct7, Funct7)
TEST_ENCODING(Funct12, Funct12)
TEST_ENCODING(RoundingMode, RoundingMode)
TEST_ENCODING(intptr_t, BTypeImm)
TEST_ENCODING(intptr_t, JTypeImm)
TEST_ENCODING(intptr_t, ITypeImm)
TEST_ENCODING(intptr_t, STypeImm)
TEST_ENCODING(intptr_t, UTypeImm)

TEST_ENCODING(Register, CRd)
TEST_ENCODING(Register, CRs1)
TEST_ENCODING(Register, CRs2)
TEST_ENCODING(Register, CRdp)
TEST_ENCODING(Register, CRs1p)
TEST_ENCODING(Register, CRs2p)
TEST_ENCODING(FRegister, CFRd)
TEST_ENCODING(FRegister, CFRs1)
TEST_ENCODING(FRegister, CFRs2)
TEST_ENCODING(FRegister, CFRdp)
TEST_ENCODING(FRegister, CFRs1p)
TEST_ENCODING(FRegister, CFRs2p)
TEST_ENCODING(intptr_t, CSPLoad4Imm)
TEST_ENCODING(intptr_t, CSPLoad8Imm)
TEST_ENCODING(intptr_t, CSPStore4Imm)
TEST_ENCODING(intptr_t, CSPStore8Imm)
TEST_ENCODING(intptr_t, CMem4Imm)
TEST_ENCODING(intptr_t, CMem8Imm)
TEST_ENCODING(intptr_t, CJImm)
TEST_ENCODING(intptr_t, CBImm)
TEST_ENCODING(intptr_t, CIImm)
TEST_ENCODING(intptr_t, CUImm)
TEST_ENCODING(intptr_t, CI16Imm)
TEST_ENCODING(intptr_t, CI4SPNImm)

#undef TEST_ENCODING

}  // namespace psoup

int main(int argc, char** argv) {
  psoup::Memory::Startup(4 * MB);
  for (psoup::UnitTest* test = psoup::tests_; test != nullptr;
       test = test->next()) {
    test->Run();
  }
  psoup::Memory::Shutdown();
  return 0;
}
