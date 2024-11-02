// Copyright (c) 2017, the Newspeak project authors. Please see the AUTHORS file
// for details. All rights reserved. Use of this source code is governed by a
// BSD-style license that can be found in the LICENSE file.

#ifndef VM_DISASSEMBLER_RISCV_H_
#define VM_DISASSEMBLER_RISCV_H_

#include "vm/constants_riscv.h"

namespace psoup {

class TextBuffer {
 public:
  TextBuffer();
  ~TextBuffer();

  void Print(const char* format, ...) PRINTF_ATTRIBUTE(2, 3);
  char* Steal();

 private:
  char* buffer_;
  intptr_t size_;
  intptr_t capacity_;
};

// We deviate from objdump in two places:
//  - branches display displacements instead of targets so our output is
//    position independent for tests.
class Disassembler {
 public:
  explicit Disassembler(ExtensionSet extensions = RV_G)
      : extensions_(extensions) {}

  bool Supports(Extension extension) const {
    return extensions_.Includes(extension);
  }
  bool Supports(ExtensionSet extensions) const {
    return extensions_.IncludesAll(extensions);
  }

  char* Disassemble(void* buffer, size_t size);
  char* Disassemble(Instruction instr) {
    DisassembleInstruction(instr);
    return buffer_.Steal();
  }
  char* Disassemble(CInstruction instr) {
    DisassembleInstruction(instr);
    return buffer_.Steal();
  }

 private:
  void DisassembleInstruction(Instruction instr);
  void DisassembleInstruction(CInstruction instr);
  void DisassembleLUI(Instruction instr);
  void DisassembleAUIPC(Instruction instr);
  void DisassembleJAL(Instruction instr);
  void DisassembleJALR(Instruction instr);
  void DisassembleBRANCH(Instruction instr);
  void DisassembleLOAD(Instruction instr);
  void DisassembleSTORE(Instruction instr);
  void DisassembleOPIMM(Instruction instr);
  void DisassembleOPIMM32(Instruction instr);
  void DisassembleOP(Instruction instr);
  void DisassembleOP_0(Instruction instr);
  void DisassembleOP_SUB(Instruction instr);
  void DisassembleOP_MULDIV(Instruction instr);
  void DisassembleOP_SHADD(Instruction instr);
  void DisassembleOP_MINMAXCLMUL(Instruction instr);
  void DisassembleOP_ROTATE(Instruction instr);
  void DisassembleOP_BCLRBEXT(Instruction instr);
  void DisassembleOP_CZERO(Instruction instr);
  void DisassembleOP32(Instruction instr);
  void DisassembleOP32_0(Instruction instr);
  void DisassembleOP32_SUB(Instruction instr);
  void DisassembleOP32_MULDIV(Instruction instr);
  void DisassembleOP32_SHADD(Instruction instr);
  void DisassembleOP32_ADDUW(Instruction instr);
  void DisassembleOP32_ROTATE(Instruction instr);
  void DisassembleMISCMEM(Instruction instr);
  void DisassembleSYSTEM(Instruction instr);
  void DisassembleAMO(Instruction instr);
  void DisassembleAMO32(Instruction instr);
  void DisassembleAMO64(Instruction instr);
  void DisassembleLOADFP(Instruction instr);
  void DisassembleSTOREFP(Instruction instr);
  void DisassembleFMADD(Instruction instr);
  void DisassembleFMSUB(Instruction instr);
  void DisassembleFNMADD(Instruction instr);
  void DisassembleFNMSUB(Instruction instr);
  void DisassembleOPFP(Instruction instr);

  void UnknownInstruction(Instruction instr);
  void UnknownInstruction(CInstruction instr);

  void Print(const char* format, Instruction instr, ExtensionSet extension);
  void Print(const char* format, CInstruction instr, ExtensionSet extension);
  const char* PrintOption(const char* format, Instruction instr);
  const char* PrintOption(const char* format, CInstruction instr);

  const ExtensionSet extensions_;
  TextBuffer buffer_;
};

}  // namespace psoup

#endif  // VM_DISASSEMBLER_RISCV_H_
