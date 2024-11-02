// Copyright (c) 2017, the Newspeak project authors. Please see the AUTHORS file
// for details. All rights reserved. Use of this source code is governed by a
// BSD-style license that can be found in the LICENSE file.

#include "vm/simulator_riscv.h"

#include <math.h>
#include <sys/mman.h>

#include <limits>

#include "vm/assert.h"
#include "vm/disassembler_riscv.h"
#include "vm/os.h"

namespace psoup {

#if XLEN != HOST_XLEN
uintptr_t Memory::guest_base_ = 0;
uintptr_t Memory::guest_size_ = 0;
uintptr_t Memory::top_ = 0;
#endif

void Memory::Startup(size_t size) {
#if XLEN != HOST_XLEN
  void* result =
      mmap(0, size, PROT_READ | PROT_WRITE, MAP_ANON | MAP_PRIVATE, -1, 0);
  if (result == MAP_FAILED) {
    perror("mmap");
    abort();
  }

  guest_base_ = top_ = reinterpret_cast<uintptr_t>(result);
  guest_size_ = size;
#endif
}

void Memory::Shutdown() {
#if XLEN != HOST_XLEN
  int result = munmap(reinterpret_cast<void*>(guest_base_), guest_size_);
  if (result != 0) {
    perror("munmap");
    abort();
  }
#endif
}

void* Memory::Allocate(size_t size) {
#if XLEN == HOST_XLEN
  return malloc(size);
#else
  uintptr_t result = top_;
  top_ += size;
  top_ = (top_ + 7) & ~7;
  return reinterpret_cast<void*>(result);
#endif
}

void Memory::Free(void* ptr) {
#if XLEN == HOST_XLEN
  free(ptr);
#else
  // UNIMPLEMENTED: Leaking...
#endif
}

#if defined(USING_SIMULATOR)
static const size_t kStackSize = 4096;

Simulator::Simulator()
    : random_(42),
      trace_(false),
      pc_(0),
      instret_(0),
      reserved_address_(0),
      reserved_value_(0),
      fcsr_(0) {
  xregs_[0] = 0;
  for (intptr_t i = 1; i < kNumRegisters; i++) {
    xregs_[i] = random_.NextUInt64();
  }
  for (intptr_t i = 0; i < kNumFRegisters; i++) {
    fregs_[i] = bit_cast<double>(random_.NextUInt64());
  }

  stack_ = Memory::Allocate(kStackSize);
  set_xreg(SP, Memory::ToGuest(stack_) + kStackSize);
}

Simulator::~Simulator() {
  Memory::Free(stack_);
}

void Simulator::PrepareCall(PreservedRegisters* preserved) {
#if defined(DEBUG)
  for (intptr_t i = 0; i < kNumRegisters; i++) {
    preserved->xregs[i] = xregs_[i];
    if (kVolatileRegisters.Includes(Register(i))) {
      xregs_[i] = random_.NextUInt64();
    }
  }
  for (intptr_t i = 0; i < kNumFRegisters; i++) {
    preserved->fregs[i] = fregs_[i];
    if (kVolatileFRegisters.Includes(FRegister(i))) {
      fregs_[i] = bit_cast<double>(random_.NextUInt64());
    }
  }
#endif
}

void Simulator::RunCall(void* function, PreservedRegisters* preserved) {
  reserved_address_ = reserved_value_ = 0;  // Clear reservation.

  const uintx_t kEndSimulationPC = -2;
  set_xreg(RA, kEndSimulationPC);

  pc_ = Memory::ToGuest(function);
  while (pc_ != kEndSimulationPC) {
    uint16_t parcel = *Memory::ToHost<uint16_t>(pc_);
    if (IsCInstruction(parcel)) {
      CInstruction instr(parcel);
      if (trace_) {
        Disassembler disassembler;
        char* str = disassembler.Disassemble(instr);
        OS::Print("  %p %s", Memory::ToHost<uint16_t>(pc_), str);
        free(str);
      }
      Interpret(instr);
    } else {
      Instruction instr(*Memory::ToHost<uint32_t>(pc_));
      if (trace_) {
        Disassembler disassembler;
        char* str = disassembler.Disassemble(instr);
        OS::Print("  %p %s", Memory::ToHost<uint32_t>(pc_), str);
        free(str);
      }
      Interpret(instr);
    }
    instret_++;
  }

#if defined(DEBUG)
  for (intptr_t i = 0; i < kNumRegisters; i++) {
    if (!kVolatileRegisters.Includes(Register(i))) {
      ASSERT(xregs_[i] == preserved->xregs[i]);
    }
  }
  for (intptr_t i = 0; i < kNumFRegisters; i++) {
    if (!kVolatileFRegisters.Includes(FRegister(i))) {
      ASSERT(bit_cast<uint64_t>(fregs_[i]) ==
             bit_cast<uint64_t>(preserved->fregs[i]));
    }
  }
#endif
}

void Simulator::PrintRegisters() {
  ASSERT(kNumRegisters == kNumFRegisters);
  for (intptr_t i = 0; i < kNumRegisters; i++) {
#if XLEN == 32
    OS::Print("%4s: %8x %11d", kRegisterNames[i], xregs_[i], xregs_[i]);
#elif XLEN == 64
    OS::Print("%4s: %16" Px64 " %20" Pd64, kRegisterNames[i], xregs_[i],
              xregs_[i]);
#endif
    OS::Print("  %4s: %lf\n", kFRegisterNames[i], fregs_[i]);
  }
#if XLEN == 32
  OS::Print("  pc: %8x\n", pc_);
#elif XLEN == 64
  OS::Print("  pc: %16" Px64 "\n", pc_);
#endif
}

void Simulator::Interpret(Instruction instr) {
  switch (instr.opcode()) {
    case LUI:
      InterpretLUI(instr);
      break;
    case AUIPC:
      InterpretAUIPC(instr);
      break;
    case JAL:
      InterpretJAL(instr);
      break;
    case JALR:
      InterpretJALR(instr);
      break;
    case BRANCH:
      InterpretBRANCH(instr);
      break;
    case LOAD:
      InterpretLOAD(instr);
      break;
    case STORE:
      InterpretSTORE(instr);
      break;
    case OPIMM:
      InterpretOPIMM(instr);
      break;
    case OPIMM32:
      InterpretOPIMM32(instr);
      break;
    case OP:
      InterpretOP(instr);
      break;
    case OP32:
      InterpretOP32(instr);
      break;
    case MISCMEM:
      InterpretMISCMEM(instr);
      break;
    case SYSTEM:
      InterpretSYSTEM(instr);
      break;
    case AMO:
      InterpretAMO(instr);
      break;
    case LOADFP:
      InterpretLOADFP(instr);
      break;
    case STOREFP:
      InterpretSTOREFP(instr);
      break;
    case FMADD:
      InterpretFMADD(instr);
      break;
    case FMSUB:
      InterpretFMSUB(instr);
      break;
    case FNMADD:
      InterpretFNMADD(instr);
      break;
    case FNMSUB:
      InterpretFNMSUB(instr);
      break;
    case OPFP:
      InterpretOPFP(instr);
      break;
    default:
      IllegalInstruction(instr);
  }
}

void Simulator::Interpret(CInstruction instr) {
  switch (instr.opcode()) {
    case C_LWSP: {
      uintx_t addr = get_xreg(SP) + instr.spload4_imm();
      set_xreg(instr.rd(), *Memory::ToHost<int32_t>(addr));
      break;
    }
#if XLEN == 32
    case C_FLWSP: {
      uintx_t addr = get_xreg(SP) + instr.spload4_imm();
      set_fregs(instr.frd(), *Memory::ToHost<float>(addr));
      break;
    }
#else
    case C_LDSP: {
      uintx_t addr = get_xreg(SP) + instr.spload8_imm();
      set_xreg(instr.rd(), *Memory::ToHost<int64_t>(addr));
      break;
    }
#endif
    case C_FLDSP: {
      uintx_t addr = get_xreg(SP) + instr.spload8_imm();
      set_fregd(instr.frd(), *Memory::ToHost<double>(addr));
      break;
    }
    case C_SWSP: {
      uintx_t addr = get_xreg(SP) + instr.spstore4_imm();
      *Memory::ToHost<uint32_t>(addr) = get_xreg(instr.rs2());
      break;
    }
#if XLEN == 32
    case C_FSWSP: {
      uintx_t addr = get_xreg(SP) + instr.spstore4_imm();
      *Memory::ToHost<float>(addr) = get_fregs(instr.frs2());
      break;
    }
#else
    case C_SDSP: {
      uintx_t addr = get_xreg(SP) + instr.spstore8_imm();
      *Memory::ToHost<uint64_t>(addr) = get_xreg(instr.rs2());
      break;
    }
#endif
    case C_FSDSP: {
      uintx_t addr = get_xreg(SP) + instr.spstore8_imm();
      *Memory::ToHost<double>(addr) = get_fregd(instr.frs2());
      break;
    }
    case C_LW: {
      uintx_t addr = get_xreg(instr.rs1p()) + instr.mem4_imm();
      set_xreg(instr.rdp(), *Memory::ToHost<int32_t>(addr));
      break;
    }
#if XLEN == 32
    case C_FLW: {
      uintx_t addr = get_xreg(instr.rs1p()) + instr.mem4_imm();
      set_fregs(instr.frdp(), *Memory::ToHost<float>(addr));
      break;
    }
#else
    case C_LD: {
      uintx_t addr = get_xreg(instr.rs1p()) + instr.mem8_imm();
      set_xreg(instr.rdp(), *Memory::ToHost<int64_t>(addr));
      break;
    }
#endif
    case C_FLD: {
      uintx_t addr = get_xreg(instr.rs1p()) + instr.mem8_imm();
      set_fregd(instr.frdp(), *Memory::ToHost<double>(addr));
      break;
    }
    case C_SW: {
      uintx_t addr = get_xreg(instr.rs1p()) + instr.mem4_imm();
      *Memory::ToHost<uint32_t>(addr) = get_xreg(instr.rs2p());
      break;
    }
#if XLEN == 32
    case C_FSW: {
      uintx_t addr = get_xreg(instr.rs1p()) + instr.mem4_imm();
      *Memory::ToHost<float>(addr) = get_fregs(instr.frs2p());
      break;
    }
#else
    case C_SD: {
      uintx_t addr = get_xreg(instr.rs1p()) + instr.mem8_imm();
      *Memory::ToHost<uint64_t>(addr) = get_xreg(instr.rs2p());
      break;
    }
#endif
    case C_FSD: {
      uintx_t addr = get_xreg(instr.rs1p()) + instr.mem8_imm();
      *Memory::ToHost<double>(addr) = get_fregd(instr.frs2p());
      break;
    }
    case C_J: {
      pc_ += sign_extend((int32_t)instr.j_imm());
      return;
    }
#if XLEN == 32
    case C_JAL: {
      set_xreg(RA, pc_ + instr.length());
      pc_ += sign_extend((int32_t)instr.j_imm());
      return;
    }
#endif
    case C_JR: {
      if (instr.encoding() & (C_JALR ^ C_JR)) {
        if ((instr.rs1() == ZERO) && (instr.rs2() == ZERO)) {
          FATAL("Encountered EBREAK");
        } else if (instr.rs2() == ZERO) {
          // JALR
          uintx_t target = get_xreg(instr.rs1());
          set_xreg(RA, pc_ + instr.length());
          pc_ = target;
          return;
        } else {
          // ADD
          set_xreg(instr.rd(), get_xreg(instr.rs1()) + get_xreg(instr.rs2()));
        }
      } else {
        if ((instr.rd() != ZERO) && (instr.rs2() != ZERO)) {
          // MV
          set_xreg(instr.rd(), get_xreg(instr.rs2()));
        } else if (instr.rs2() != ZERO) {
          IllegalInstruction(instr);
        } else {
          // JR
          pc_ = get_xreg(instr.rs1());
          return;
        }
      }
      break;
    }
    case C_BEQZ:
      if (get_xreg(instr.rs1p()) == 0) {
        pc_ += instr.b_imm();
        return;
      }
      break;
    case C_BNEZ:
      if (get_xreg(instr.rs1p()) != 0) {
        pc_ += instr.b_imm();
        return;
      }
      break;
    case C_LI:
      if (instr.rd() == ZERO) {
        IllegalInstruction(instr);
      } else {
        set_xreg(instr.rd(), sign_extend(instr.i_imm()));
      }
      break;
    case C_LUI:
      if (instr.rd() == SP) {
        if (instr.i16_imm() == 0) {
          IllegalInstruction(instr);
        } else {
          set_xreg(instr.rd(),
                   get_xreg(instr.rs1()) + sign_extend(instr.i16_imm()));
        }
      } else if ((instr.rd() == ZERO) || (instr.u_imm() == 0)) {
        IllegalInstruction(instr);
      } else {
        set_xreg(instr.rd(), sign_extend(instr.u_imm()));
      }
      break;
    case C_ADDI:
      set_xreg(instr.rd(), get_xreg(instr.rs1()) + instr.i_imm());
      break;
#if XLEN >= 64
    case C_ADDIW: {
      uint32_t a = get_xreg(instr.rs1());
      uint32_t b = instr.i_imm();
      set_xreg(instr.rd(), sign_extend(a + b));
      break;
    }
#endif  // XLEN >= 64
    case C_ADDI4SPN:
      if (instr.i4spn_imm() == 0) {
        IllegalInstruction(instr);
      } else {
        set_xreg(instr.rdp(), get_xreg(SP) + instr.i4spn_imm());
      }
      break;
    case C_SLLI:
      if (instr.rd() == ZERO) {
        IllegalInstruction(instr);
      } else {
        set_xreg(instr.rd(), get_xreg(instr.rs1()) << instr.shamt());
      }
      break;
    case C_MISCALU:
      // Note MISCALU has a different notion of rsd′ than other instructions,
      // so use rs1′ instead.
      switch (instr.encoding() & C_MISCALU_MASK) {
        case C_SRLI:
          if (instr.shamt() >= XLEN) {
            IllegalInstruction(instr);
          } else {
            set_xreg(instr.rs1p(),  get_xreg(instr.rs1p()) >> instr.shamt());
          }
          break;
        case C_SRAI:
          if (instr.shamt() >= XLEN) {
            IllegalInstruction(instr);
          } else {
            set_xreg(instr.rs1p(),
                     static_cast<intx_t>(get_xreg(instr.rs1p())) >>
                         instr.i_imm());
          }
          break;
        case C_ANDI:
          set_xreg(instr.rs1p(), get_xreg(instr.rs1p()) & instr.i_imm());
          break;
        case C_RR:
          switch (instr.encoding() & C_RR_MASK) {
            case C_AND:
              set_xreg(instr.rs1p(),
                       get_xreg(instr.rs1p()) & get_xreg(instr.rs2p()));
              break;
            case C_OR:
              set_xreg(instr.rs1p(),
                       get_xreg(instr.rs1p()) | get_xreg(instr.rs2p()));
              break;
            case C_XOR:
              set_xreg(instr.rs1p(),
                       get_xreg(instr.rs1p()) ^ get_xreg(instr.rs2p()));
              break;
            case C_SUB:
              set_xreg(instr.rs1p(),
                       get_xreg(instr.rs1p()) - get_xreg(instr.rs2p()));
              break;
            case C_ADDW: {
              uint32_t a = get_xreg(instr.rs1p());
              uint32_t b = get_xreg(instr.rs2p());
              set_xreg(instr.rs1p(), sign_extend(a + b));
              break;
            }
            case C_SUBW: {
              uint32_t a = get_xreg(instr.rs1p());
              uint32_t b = get_xreg(instr.rs2p());
              set_xreg(instr.rs1p(), sign_extend(a - b));
              break;
            }
            default:
              IllegalInstruction(instr);
          }
          break;
        default:
          IllegalInstruction(instr);
      }
      break;
    default:
      IllegalInstruction(instr);
  }
  pc_ += instr.length();
}

void Simulator::InterpretLUI(Instruction instr) {
  set_xreg(instr.rd(), sign_extend(instr.utype_imm()));
  pc_ += instr.length();
}

void Simulator::InterpretAUIPC(Instruction instr) {
  set_xreg(instr.rd(), pc_ + sign_extend(instr.utype_imm()));
  pc_ += instr.length();
}

void Simulator::InterpretJAL(Instruction instr) {
  set_xreg(instr.rd(), pc_ + instr.length());
  pc_ += sign_extend(instr.jtype_imm());
}

void Simulator::InterpretJALR(Instruction instr) {
  uintx_t base = get_xreg(instr.rs1());
  uintx_t offset = static_cast<uintx_t>(instr.itype_imm());
  set_xreg(instr.rd(), pc_ + instr.length());
  pc_ = base + offset;
}

void Simulator::InterpretBRANCH(Instruction instr) {
  switch (instr.funct3()) {
    case BEQ:
      if (get_xreg(instr.rs1()) == get_xreg(instr.rs2())) {
        pc_ += instr.btype_imm();
      } else {
        pc_ += instr.length();
      }
      break;
    case BNE:
      if (get_xreg(instr.rs1()) != get_xreg(instr.rs2())) {
        pc_ += instr.btype_imm();
      } else {
        pc_ += instr.length();
      }
      break;
    case BLT:
      if (static_cast<intx_t>(get_xreg(instr.rs1())) <
          static_cast<intx_t>(get_xreg(instr.rs2()))) {
        pc_ += instr.btype_imm();
      } else {
        pc_ += instr.length();
      }
      break;
    case BGE:
      if (static_cast<intx_t>(get_xreg(instr.rs1())) >=
          static_cast<intx_t>(get_xreg(instr.rs2()))) {
        pc_ += instr.btype_imm();
      } else {
        pc_ += instr.length();
      }
      break;
    case BLTU:
      if (static_cast<uintx_t>(get_xreg(instr.rs1())) <
          static_cast<uintx_t>(get_xreg(instr.rs2()))) {
        pc_ += instr.btype_imm();
      } else {
        pc_ += instr.length();
      }
      break;
    case BGEU:
      if (static_cast<uintx_t>(get_xreg(instr.rs1())) >=
          static_cast<uintx_t>(get_xreg(instr.rs2()))) {
        pc_ += instr.btype_imm();
      } else {
        pc_ += instr.length();
      }
      break;
    default:
      IllegalInstruction(instr);
  }
}

void Simulator::InterpretLOAD(Instruction instr) {
  uintx_t addr = get_xreg(instr.rs1()) + instr.itype_imm();
  switch (instr.funct3()) {
    case LB:
      set_xreg(instr.rd(), *Memory::ToHost<int8_t>(addr));
      break;
    case LH:
      set_xreg(instr.rd(), *Memory::ToHost<int16_t>(addr));
      break;
    case LW:
      set_xreg(instr.rd(), *Memory::ToHost<int32_t>(addr));
      break;
    case LBU:
      set_xreg(instr.rd(), *Memory::ToHost<uint8_t>(addr));
      break;
    case LHU:
      set_xreg(instr.rd(), *Memory::ToHost<uint16_t>(addr));
      break;
#if XLEN >= 64
    case LWU:
      set_xreg(instr.rd(), *Memory::ToHost<uint32_t>(addr));
      break;
    case LD:
      set_xreg(instr.rd(), *Memory::ToHost<int64_t>(addr));
      break;
#endif  // XLEN >= 64
    default:
      IllegalInstruction(instr);
  }
  pc_ += instr.length();
}

void Simulator::InterpretLOADFP(Instruction instr) {
  uintx_t addr = get_xreg(instr.rs1()) + instr.itype_imm();
  switch (instr.funct3()) {
    case S:
      set_fregs(instr.frd(), *Memory::ToHost<float>(addr));
      break;
    case D:
      set_fregd(instr.frd(), *Memory::ToHost<double>(addr));
      break;
    default:
      IllegalInstruction(instr);
  }
  pc_ += instr.length();
}

void Simulator::InterpretSTORE(Instruction instr) {
  uintx_t addr = get_xreg(instr.rs1()) + instr.stype_imm();
  switch (instr.funct3()) {
    case SB:
      *Memory::ToHost<uint8_t>(addr) = get_xreg(instr.rs2());
      break;
    case SH:
      *Memory::ToHost<uint16_t>(addr) = get_xreg(instr.rs2());
      break;
    case SW:
      *Memory::ToHost<uint32_t>(addr) = get_xreg(instr.rs2());
      break;
#if XLEN >= 64
    case SD:
      *Memory::ToHost<uint64_t>(addr) = get_xreg(instr.rs2());
      break;
#endif  // XLEN >= 64
    default:
      IllegalInstruction(instr);
  }
  pc_ += instr.length();
}

void Simulator::InterpretSTOREFP(Instruction instr) {
  uintx_t addr = get_xreg(instr.rs1()) + instr.stype_imm();
  switch (instr.funct3()) {
    case S:
      *Memory::ToHost<float>(addr) = get_fregs(instr.frs2());
      break;
    case D:
      *Memory::ToHost<double>(addr) = get_fregd(instr.frs2());
      break;
    default:
      IllegalInstruction(instr);
  }
  pc_ += instr.length();
}

static uintx_t clz(uintx_t a) {
  for (int bit = XLEN - 1; bit >= 0; bit--) {
    if ((a & (static_cast<uintx_t>(1) << bit)) != 0) {
      return XLEN - bit - 1;
    }
  }
  return XLEN;
}

static uintx_t ctz(uintx_t a) {
  for (int bit = 0; bit < XLEN; bit++) {
    if ((a & (static_cast<uintx_t>(1) << bit)) != 0) {
      return bit;
    }
  }
  return XLEN;
}

static uintx_t cpop(uintx_t a) {
  uintx_t count = 0;
  for (int bit = 0; bit < XLEN; bit++) {
    if ((a & (static_cast<uintx_t>(1) << bit)) != 0) {
      count++;
    }
  }
  return count;
}

static uintx_t clzw(uint32_t a) {
  for (int bit = 32 - 1; bit >= 0; bit--) {
    if ((a & (static_cast<uint32_t>(1) << bit)) != 0) {
      return 32 - bit - 1;
    }
  }
  return 32;
}

static uintx_t ctzw(uint32_t a) {
  for (int bit = 0; bit < 32; bit++) {
    if ((a & (static_cast<uint32_t>(1) << bit)) != 0) {
      return bit;
    }
  }
  return 32;
}

static uintx_t cpopw(uint32_t a) {
  uintx_t count = 0;
  for (int bit = 0; bit < 32; bit++) {
    if ((a & (static_cast<uint32_t>(1) << bit)) != 0) {
      count++;
    }
  }
  return count;
}

static intx_t max(intx_t a, intx_t b) {
  return a > b ? a : b;
}
static uintx_t maxu(uintx_t a, uintx_t b) {
  return a > b ? a : b;
}
static intx_t min(intx_t a, intx_t b) {
  return a < b ? a : b;
}
static uintx_t minu(uintx_t a, uintx_t b) {
  return a < b ? a : b;
}
static uintx_t clmul(uintx_t a, uintx_t b) {
  uintx_t result = 0;
  for (int bit = 0; bit < XLEN; bit++) {
    if (((b >> bit) & 1) != 0) {
      result ^= a << bit;
    }
  }
  return result;
}
static uintx_t clmulh(uintx_t a, uintx_t b) {
  uintx_t result = 0;
  for (int bit = 1; bit < XLEN; bit++) {
    if (((b >> bit) & 1) != 0) {
      result ^= a >> (XLEN - bit);
    }
  }
  return result;
}
static uintx_t clmulr(uintx_t a, uintx_t b) {
  uintx_t result = 0;
  for (int bit = 0; bit < XLEN; bit++) {
    if (((b >> bit) & 1) != 0) {
      result ^= a >> (XLEN - bit - 1);
    }
  }
  return result;
}
static uintx_t sextb(uintx_t a) {
  return static_cast<intx_t>(a << (XLEN - 8)) >> (XLEN - 8);
}
static uintx_t sexth(uintx_t a) {
  return static_cast<intx_t>(a << (XLEN - 16)) >> (XLEN - 16);
}
static uintx_t zexth(uintx_t a) {
  return a << (XLEN - 16) >> (XLEN - 16);
}
static uintx_t ror(uintx_t a, uintx_t b) {
  uintx_t r = b & (XLEN - 1);
  uintx_t l = (XLEN - r) & (XLEN - 1);
  return (a << l) | (a >> r);
}
static uintx_t rol(uintx_t a, uintx_t b) {
  uintx_t l = b & (XLEN - 1);
  uintx_t r = (XLEN - l) & (XLEN - 1);
  return (a << l) | (a >> r);
}
static uintx_t rorw(uintx_t a, uintx_t b) {
  uint32_t r = b & (XLEN - 1);
  uint32_t l = (XLEN - r) & (XLEN - 1);
  uint32_t x = a;
  return sign_extend((x << l) | (x >> r));
}
static uintx_t rolw(uintx_t a, uintx_t b) {
  uint32_t l = b & (XLEN - 1);
  uint32_t r = (XLEN - l) & (XLEN - 1);
  uint32_t x = a;
  return sign_extend((x << l) | (x >> r));
}
static uintx_t orcb(uintx_t a) {
  uintx_t result = 0;
  for (int shift = 0; shift < XLEN; shift += 8) {
    if (((a >> shift) & 0xFF) != 0) {
      result |= static_cast<uintx_t>(0xFF) << shift;
    }
  }
  return result;
}
static uintx_t rev8(uintx_t a) {
  uintx_t result = 0;
  for (int shift = 0; shift < XLEN; shift += 8) {
    result <<= 8;
    result |= (a >> shift) & 0xFF;
  }
  return result;
}
static uintx_t bclr(uintx_t a, uintx_t b) {
  return a & ~(static_cast<uintx_t>(1) << (b & (XLEN - 1)));
}
static uintx_t bext(uintx_t a, uintx_t b) {
  return (a >> (b & (XLEN - 1))) & 1;
}
static uintx_t binv(uintx_t a, uintx_t b) {
  return a ^ (static_cast<uintx_t>(1) << (b & (XLEN - 1)));
}
static uintx_t bset(uintx_t a, uintx_t b) {
  return a | (static_cast<uintx_t>(1) << (b & (XLEN - 1)));
}

void Simulator::InterpretOPIMM(Instruction instr) {
  switch (instr.funct3()) {
    case ADDI:
      set_xreg(instr.rd(), get_xreg(instr.rs1()) + instr.itype_imm());
      break;
    case SLTI: {
      set_xreg(instr.rd(), static_cast<intx_t>(get_xreg(instr.rs1())) <
                                   static_cast<intx_t>(instr.itype_imm())
                               ? 1
                               : 0);
      break;
    }
    case SLTIU:
      set_xreg(instr.rd(), static_cast<uintx_t>(get_xreg(instr.rs1())) <
                                   static_cast<uintx_t>(instr.itype_imm())
                               ? 1
                               : 0);
      break;
    case XORI:
      set_xreg(instr.rd(), get_xreg(instr.rs1()) ^ instr.itype_imm());
      break;
    case ORI:
      set_xreg(instr.rd(), get_xreg(instr.rs1()) | instr.itype_imm());
      break;
    case ANDI:
      set_xreg(instr.rd(), get_xreg(instr.rs1()) & instr.itype_imm());
      break;
    case SLLI:
      if (instr.funct7() == COUNT) {
        if (instr.shamt() == 0b00000) {
          set_xreg(instr.rd(), clz(get_xreg(instr.rs1())));
        } else if (instr.shamt() == 0b00001) {
          set_xreg(instr.rd(), ctz(get_xreg(instr.rs1())));
        } else if (instr.shamt() == 0b00010) {
          set_xreg(instr.rd(), cpop(get_xreg(instr.rs1())));
        } else if (instr.shamt() == 0b00100) {
          set_xreg(instr.rd(), sextb(get_xreg(instr.rs1())));
        } else if (instr.shamt() == 0b00101) {
          set_xreg(instr.rd(), sexth(get_xreg(instr.rs1())));
        } else {
          IllegalInstruction(instr);
        }
      } else if ((instr.funct7() & 0b1111110) == BCLRBEXT) {
        set_xreg(instr.rd(), bclr(get_xreg(instr.rs1()), instr.shamt()));
      } else if ((instr.funct7() & 0b1111110) == BINV) {
        set_xreg(instr.rd(), binv(get_xreg(instr.rs1()), instr.shamt()));
      } else if ((instr.funct7() & 0b1111110) == BSET) {
        set_xreg(instr.rd(), bset(get_xreg(instr.rs1()), instr.shamt()));
      } else {
        set_xreg(instr.rd(), get_xreg(instr.rs1()) << instr.shamt());
      }
      break;
    case SRI:
      if ((instr.funct7() & 0b1111110) == SRA) {
        set_xreg(instr.rd(),
                 static_cast<intx_t>(get_xreg(instr.rs1())) >> instr.shamt());
      } else if ((instr.funct7() & 0b1111110) == ROTATE) {
        set_xreg(instr.rd(), ror(get_xreg(instr.rs1()), instr.shamt()));
      } else if (instr.funct7() == 0b0010100) {
        set_xreg(instr.rd(), orcb(get_xreg(instr.rs1())));
#if XLEN == 32
      } else if (instr.funct7() == 0b0110100) {
#else
      } else if (instr.funct7() == 0b0110101) {
#endif
        set_xreg(instr.rd(), rev8(get_xreg(instr.rs1())));
      } else if ((instr.funct7() & 0b1111110) == BCLRBEXT) {
        set_xreg(instr.rd(), bext(get_xreg(instr.rs1()), instr.shamt()));
      } else {
        set_xreg(instr.rd(),
                 static_cast<uintx_t>(get_xreg(instr.rs1())) >> instr.shamt());
      }
      break;
    default:
      IllegalInstruction(instr);
  }
  pc_ += instr.length();
}

void Simulator::InterpretOPIMM32(Instruction instr) {
  switch (instr.funct3()) {
    case ADDI: {
      uint32_t a = get_xreg(instr.rs1());
      uint32_t b = instr.itype_imm();
      set_xreg(instr.rd(), sign_extend(a + b));
      break;
    }
    case SLLI: {
      if (instr.funct7() == SLLIUW) {
        uintx_t a = static_cast<uint32_t>(get_xreg(instr.rs1()));
        uintx_t b = instr.shamt();
        set_xreg(instr.rd(), a << b);
      } else if (instr.funct7() == COUNT) {
        if (instr.shamt() == 0b00000) {
          set_xreg(instr.rd(), clzw(get_xreg(instr.rs1())));
        } else if (instr.shamt() == 0b00001) {
          set_xreg(instr.rd(), ctzw(get_xreg(instr.rs1())));
        } else if (instr.shamt() == 0b00010) {
          set_xreg(instr.rd(), cpopw(get_xreg(instr.rs1())));
        } else {
          IllegalInstruction(instr);
        }
      } else {
        uint32_t a = get_xreg(instr.rs1());
        uint32_t b = instr.shamt();
        set_xreg(instr.rd(), sign_extend(a << b));
      }
      break;
    }
    case SRI:
      if (instr.funct7() == SRA) {
        int32_t a = get_xreg(instr.rs1());
        int32_t b = instr.shamt();
        set_xreg(instr.rd(), sign_extend(a >> b));
      } else if (instr.funct7() == ROTATE) {
        set_xreg(instr.rd(), rorw(get_xreg(instr.rs1()), instr.shamt()));
      } else {
        uint32_t a = get_xreg(instr.rs1());
        uint32_t b = instr.shamt();
        set_xreg(instr.rd(), sign_extend(a >> b));
      }
      break;
    default:
      IllegalInstruction(instr);
  }
  pc_ += instr.length();
}

void Simulator::InterpretOP(Instruction instr) {
  switch (instr.funct7()) {
    case 0:
      InterpretOP_0(instr);
      break;
    case SUB:
      InterpretOP_SUB(instr);
      break;
    case MULDIV:
      InterpretOP_MULDIV(instr);
      break;
    case SHADD:
      InterpretOP_SHADD(instr);
      break;
    case MINMAXCLMUL:
      InterpretOP_MINMAXCLMUL(instr);
      break;
    case ROTATE:
      InterpretOP_ROTATE(instr);
      break;
    case BCLRBEXT:
      InterpretOP_BCLRBEXT(instr);
      break;
    case BINV:
      set_xreg(instr.rd(), binv(get_xreg(instr.rs1()), get_xreg(instr.rs2())));
      pc_ += instr.length();
      break;
    case BSET:
      set_xreg(instr.rd(), bset(get_xreg(instr.rs1()), get_xreg(instr.rs2())));
      pc_ += instr.length();
      break;
#if XLEN == 32
    case 0b0000100:
      set_xreg(instr.rd(), zexth(get_xreg(instr.rs1())));
      pc_ += instr.length();
      break;
#endif
    default:
      IllegalInstruction(instr);
  }
}

void Simulator::InterpretOP_0(Instruction instr) {
  switch (instr.funct3()) {
    case ADD:
      set_xreg(instr.rd(), get_xreg(instr.rs1()) + get_xreg(instr.rs2()));
      break;
    case SLL: {
      uintx_t shamt = get_xreg(instr.rs2()) & (XLEN - 1);
      set_xreg(instr.rd(), get_xreg(instr.rs1()) << shamt);
      break;
    }
    case SLT:
      set_xreg(instr.rd(), static_cast<intx_t>(get_xreg(instr.rs1())) <
                                   static_cast<intx_t>(get_xreg(instr.rs2()))
                               ? 1
                               : 0);
      break;
    case SLTU:
      set_xreg(instr.rd(), static_cast<uintx_t>(get_xreg(instr.rs1())) <
                                   static_cast<uintx_t>(get_xreg(instr.rs2()))
                               ? 1
                               : 0);
      break;
    case XOR:
      set_xreg(instr.rd(), get_xreg(instr.rs1()) ^ get_xreg(instr.rs2()));
      break;
    case SR: {
      uintx_t shamt = get_xreg(instr.rs2()) & (XLEN - 1);
      set_xreg(instr.rd(),
               static_cast<uintx_t>(get_xreg(instr.rs1())) >> shamt);
      break;
    }
    case OR:
      set_xreg(instr.rd(), get_xreg(instr.rs1()) | get_xreg(instr.rs2()));
      break;
    case AND:
      set_xreg(instr.rd(), get_xreg(instr.rs1()) & get_xreg(instr.rs2()));
      break;
    default:
      IllegalInstruction(instr);
  }
  pc_ += instr.length();
}

static intx_t mul(intx_t a, intx_t b) {
  return a * b;
}

static intx_t mulh(intx_t a, intx_t b) {
  const uintx_t kLoMask = (static_cast<uintx_t>(1) << (XLEN / 2)) - 1;
  const uintx_t kHiShift = XLEN / 2;

  uintx_t a_lo = a & kLoMask;
  intx_t a_hi = a >> kHiShift;
  uintx_t b_lo = b & kLoMask;
  intx_t b_hi = b >> kHiShift;

  uintx_t x = a_lo * b_lo;
  intx_t y = a_hi * b_lo;
  intx_t z = a_lo * b_hi;
  intx_t w = a_hi * b_hi;

  intx_t r0 = (x >> kHiShift) + y;
  intx_t r1 = (r0 & kLoMask) + z;
  return w + (r0 >> kHiShift) + (r1 >> kHiShift);
}

static uintx_t mulhu(uintx_t a, uintx_t b) {
  const uintx_t kLoMask = (static_cast<uintx_t>(1) << (XLEN / 2)) - 1;
  const uintx_t kHiShift = XLEN / 2;

  uintx_t a_lo = a & kLoMask;
  uintx_t a_hi = a >> kHiShift;
  uintx_t b_lo = b & kLoMask;
  uintx_t b_hi = b >> kHiShift;

  uintx_t x = a_lo * b_lo;
  uintx_t y = a_hi * b_lo;
  uintx_t z = a_lo * b_hi;
  uintx_t w = a_hi * b_hi;

  uintx_t r0 = (x >> kHiShift) + y;
  uintx_t r1 = (r0 & kLoMask) + z;
  return w + (r0 >> kHiShift) + (r1 >> kHiShift);
}

static uintx_t mulhsu(intx_t a, uintx_t b) {
  const uintx_t kLoMask = (static_cast<uintx_t>(1) << (XLEN / 2)) - 1;
  const uintx_t kHiShift = XLEN / 2;

  uintx_t a_lo = a & kLoMask;
  intx_t a_hi = a >> kHiShift;
  uintx_t b_lo = b & kLoMask;
  uintx_t b_hi = b >> kHiShift;

  uintx_t x = a_lo * b_lo;
  intx_t y = a_hi * b_lo;
  uintx_t z = a_lo * b_hi;
  intx_t w = a_hi * b_hi;

  intx_t r0 = (x >> kHiShift) + y;
  uintx_t r1 = (r0 & kLoMask) + z;
  return w + (r0 >> kHiShift) + (r1 >> kHiShift);
}

static intx_t div(intx_t a, intx_t b) {
  if (b == 0) {
    return -1;
  } else if (b == -1 && a == kMinIntX) {
    return kMinIntX;
  } else {
    return a / b;
  }
}

static uintx_t divu(uintx_t a, uintx_t b) {
  if (b == 0) {
    return kMaxUIntX;
  } else {
    return a / b;
  }
}

static intx_t rem(intx_t a, intx_t b) {
  if (b == 0) {
    return a;
  } else if (b == -1 && a == kMinIntX) {
    return 0;
  } else {
    return a % b;
  }
}

static uintx_t remu(uintx_t a, uintx_t b) {
  if (b == 0) {
    return a;
  } else {
    return a % b;
  }
}

#if XLEN >= 64
static int32_t mulw(int32_t a, int32_t b) {
  return a * b;
}

static int32_t divw(int32_t a, int32_t b) {
  if (b == 0) {
    return -1;
  } else if (b == -1 && a == kMinInt32) {
    return kMinInt32;
  } else {
    return a / b;
  }
}

static uint32_t divuw(uint32_t a, uint32_t b) {
  if (b == 0) {
    return kMaxUInt32;
  } else {
    return a / b;
  }
}

static int32_t remw(int32_t a, int32_t b) {
  if (b == 0) {
    return a;
  } else if (b == -1 && a == kMinInt32) {
    return 0;
  } else {
    return a % b;
  }
}

static uint32_t remuw(uint32_t a, uint32_t b) {
  if (b == 0) {
    return a;
  } else {
    return a % b;
  }
}
#endif  // XLEN >= 64

void Simulator::InterpretOP_MULDIV(Instruction instr) {
  switch (instr.funct3()) {
    case MUL:
      set_xreg(instr.rd(), mul(get_xreg(instr.rs1()), get_xreg(instr.rs2())));
      break;
    case MULH:
      set_xreg(instr.rd(), mulh(get_xreg(instr.rs1()), get_xreg(instr.rs2())));
      break;
    case MULHSU:
      set_xreg(instr.rd(),
               mulhsu(get_xreg(instr.rs1()), get_xreg(instr.rs2())));
      break;
    case MULHU:
      set_xreg(instr.rd(), mulhu(get_xreg(instr.rs1()), get_xreg(instr.rs2())));
      break;
    case DIV:
      set_xreg(instr.rd(), div(get_xreg(instr.rs1()), get_xreg(instr.rs2())));
      break;
    case DIVU:
      set_xreg(instr.rd(), divu(get_xreg(instr.rs1()), get_xreg(instr.rs2())));
      break;
    case REM:
      set_xreg(instr.rd(), rem(get_xreg(instr.rs1()), get_xreg(instr.rs2())));
      break;
    case REMU:
      set_xreg(instr.rd(), remu(get_xreg(instr.rs1()), get_xreg(instr.rs2())));
      break;
    default:
      IllegalInstruction(instr);
  }
  pc_ += instr.length();
}

void Simulator::InterpretOP_SUB(Instruction instr) {
  switch (instr.funct3()) {
    case ADD:
      set_xreg(instr.rd(), get_xreg(instr.rs1()) - get_xreg(instr.rs2()));
      break;
    case SR: {
      uintx_t shamt = get_xreg(instr.rs2()) & (XLEN - 1);
      set_xreg(instr.rd(), static_cast<intx_t>(get_xreg(instr.rs1())) >> shamt);
      break;
    }
    case AND:
      set_xreg(instr.rd(), get_xreg(instr.rs1()) & ~get_xreg(instr.rs2()));
      break;
    case OR:
      set_xreg(instr.rd(), get_xreg(instr.rs1()) | ~get_xreg(instr.rs2()));
      break;
    case XOR:
      set_xreg(instr.rd(), get_xreg(instr.rs1()) ^ ~get_xreg(instr.rs2()));
      break;
    default:
      IllegalInstruction(instr);
  }
  pc_ += instr.length();
}

void Simulator::InterpretOP_SHADD(Instruction instr) {
  switch (instr.funct3()) {
    case SH1ADD:
      set_xreg(instr.rd(), (get_xreg(instr.rs1()) << 1) +
                            get_xreg(instr.rs2()));
      break;
    case SH2ADD:
      set_xreg(instr.rd(), (get_xreg(instr.rs1()) << 2) +
                            get_xreg(instr.rs2()));
      break;
    case SH3ADD:
      set_xreg(instr.rd(), (get_xreg(instr.rs1()) << 3) +
                            get_xreg(instr.rs2()));
      break;
    default:
      IllegalInstruction(instr);
  }
  pc_ += instr.length();
}

void Simulator::InterpretOP_MINMAXCLMUL(Instruction instr) {
  switch (instr.funct3()) {
    case MAX:
      set_xreg(instr.rd(), max(get_xreg(instr.rs1()), get_xreg(instr.rs2())));
      break;
    case MAXU:
      set_xreg(instr.rd(), maxu(get_xreg(instr.rs1()), get_xreg(instr.rs2())));
      break;
    case MIN:
      set_xreg(instr.rd(), min(get_xreg(instr.rs1()), get_xreg(instr.rs2())));
      break;
    case MINU:
      set_xreg(instr.rd(), minu(get_xreg(instr.rs1()), get_xreg(instr.rs2())));
      break;
    case CLMUL:
      set_xreg(instr.rd(), clmul(get_xreg(instr.rs1()), get_xreg(instr.rs2())));
      break;
    case CLMULH:
      set_xreg(instr.rd(), clmulh(get_xreg(instr.rs1()), get_xreg(instr.rs2())));
      break;
    case CLMULR:
      set_xreg(instr.rd(), clmulr(get_xreg(instr.rs1()), get_xreg(instr.rs2())));
      break;
    default:
      IllegalInstruction(instr);
  }
  pc_ += instr.length();
}

void Simulator::InterpretOP_ROTATE(Instruction instr) {
  switch (instr.funct3()) {
    case ROR:
      set_xreg(instr.rd(), ror(get_xreg(instr.rs1()), get_xreg(instr.rs2())));
      break;
    case ROL:
      set_xreg(instr.rd(), rol(get_xreg(instr.rs1()), get_xreg(instr.rs2())));
      break;
    default:
      IllegalInstruction(instr);
  }
  pc_ += instr.length();
}

void Simulator::InterpretOP_BCLRBEXT(Instruction instr) {
  switch (instr.funct3()) {
    case BCLR:
      set_xreg(instr.rd(), bclr(get_xreg(instr.rs1()), get_xreg(instr.rs2())));
      break;
    case BEXT:
      set_xreg(instr.rd(), bext(get_xreg(instr.rs1()), get_xreg(instr.rs2())));
      break;
    default:
      IllegalInstruction(instr);
  }
  pc_ += instr.length();
}

void Simulator::InterpretOP32(Instruction instr) {
  switch (instr.funct7()) {
#if XLEN >= 64
    case 0:
      InterpretOP32_0(instr);
      break;
    case SUB:
      InterpretOP32_SUB(instr);
      break;
    case MULDIV:
      InterpretOP32_MULDIV(instr);
      break;
    case SHADD:
      InterpretOP32_SHADD(instr);
      break;
    case ADDUW:
      InterpretOP32_ADDUW(instr);
      break;
    case ROTATE:
      InterpretOP32_ROTATE(instr);
      break;
#endif  // XLEN >= 64
    default:
      IllegalInstruction(instr);
  }
}

void Simulator::InterpretOP32_0(Instruction instr) {
  switch (instr.funct3()) {
#if XLEN >= 64
    case ADD: {
      uint32_t a = get_xreg(instr.rs1());
      uint32_t b = get_xreg(instr.rs2());
      set_xreg(instr.rd(), sign_extend(a + b));
      break;
    }
    case SLL: {
      uint32_t a = get_xreg(instr.rs1());
      uint32_t b = get_xreg(instr.rs2()) & (32 - 1);
      set_xreg(instr.rd(), sign_extend(a << b));
      break;
    }
    case SR: {
      uint32_t b = get_xreg(instr.rs2()) & (32 - 1);
      uint32_t a = get_xreg(instr.rs1());
      set_xreg(instr.rd(), sign_extend(a >> b));
      break;
    }
#endif  // XLEN >= 64
    default:
      IllegalInstruction(instr);
  }
  pc_ += instr.length();
}

void Simulator::InterpretOP32_SUB(Instruction instr) {
  switch (instr.funct3()) {
#if XLEN >= 64
    case ADD: {
      uint32_t a = get_xreg(instr.rs1());
      uint32_t b = get_xreg(instr.rs2());
      set_xreg(instr.rd(), sign_extend(a - b));
      break;
    }
    case SR: {
      uint32_t b = get_xreg(instr.rs2()) & (32 - 1);
      int32_t a = get_xreg(instr.rs1());
      set_xreg(instr.rd(), sign_extend(a >> b));
      break;
    }
#endif  // XLEN >= 64
    default:
      IllegalInstruction(instr);
  }
  pc_ += instr.length();
}

void Simulator::InterpretOP32_MULDIV(Instruction instr) {
  switch (instr.funct3()) {
#if XLEN >= 64
    case MULW:
      set_xreg(instr.rd(),
               sign_extend(mulw(get_xreg(instr.rs1()), get_xreg(instr.rs2()))));
      break;
    case DIVW:
      set_xreg(instr.rd(),
               sign_extend(divw(get_xreg(instr.rs1()), get_xreg(instr.rs2()))));
      break;
    case DIVUW:
      set_xreg(instr.rd(), sign_extend(divuw(get_xreg(instr.rs1()),
                                             get_xreg(instr.rs2()))));
      break;
    case REMW:
      set_xreg(instr.rd(),
               sign_extend(remw(get_xreg(instr.rs1()), get_xreg(instr.rs2()))));
      break;
    case REMUW:
      set_xreg(instr.rd(), sign_extend(remuw(get_xreg(instr.rs1()),
                                             get_xreg(instr.rs2()))));
      break;
#endif  // XLEN >= 64
    default:
      IllegalInstruction(instr);
  }
  pc_ += instr.length();
}

void Simulator::InterpretOP32_SHADD(Instruction instr) {
  switch (instr.funct3()) {
    case SH1ADD: {
      uintx_t a = static_cast<uint32_t>(get_xreg(instr.rs1()));
      uintx_t b = get_xreg(instr.rs2());
      set_xreg(instr.rd(), (a << 1) + b);
      break;
    }
    case SH2ADD: {
      uintx_t a = static_cast<uint32_t>(get_xreg(instr.rs1()));
      uintx_t b = get_xreg(instr.rs2());
      set_xreg(instr.rd(), (a << 2) + b);
      break;
    }
    case SH3ADD: {
      uintx_t a = static_cast<uint32_t>(get_xreg(instr.rs1()));
      uintx_t b = get_xreg(instr.rs2());
      set_xreg(instr.rd(), (a << 3) + b);
      break;
    }
    default:
      IllegalInstruction(instr);
  }
  pc_ += instr.length();
}


void Simulator::InterpretOP32_ADDUW(Instruction instr) {
  switch (instr.funct3()) {
#if XLEN >= 64
    case F3_0: {
      uintx_t a = static_cast<uint32_t>(get_xreg(instr.rs1()));
      uintx_t b = get_xreg(instr.rs2());
      set_xreg(instr.rd(), a + b);
      break;
    }
    case ZEXT:
      set_xreg(instr.rd(), zexth(get_xreg(instr.rs1())));
      break;
#endif  // XLEN >= 64
    default:
      IllegalInstruction(instr);
  }
  pc_ += instr.length();
}

void Simulator::InterpretOP32_ROTATE(Instruction instr) {
  switch (instr.funct3()) {
    case ROR:
      set_xreg(instr.rd(), rorw(get_xreg(instr.rs1()), get_xreg(instr.rs2())));
      break;
    case ROL:
      set_xreg(instr.rd(), rolw(get_xreg(instr.rs1()), get_xreg(instr.rs2())));
      break;
    default:
      IllegalInstruction(instr);
  }
  pc_ += instr.length();
}

void Simulator::InterpretMISCMEM(Instruction instr) {
  switch (instr.funct3()) {
    case FENCE:
      std::atomic_thread_fence(std::memory_order_acq_rel);
      break;
    case FENCEI:
      // Nothing to do: simulated instructions are data on the host.
      break;
    default:
      IllegalInstruction(instr);
  }
  pc_ += instr.length();
}

void Simulator::InterpretSYSTEM(Instruction instr) {
  switch (instr.funct3()) {
    case 0:
      switch (instr.funct12()) {
        case ECALL:
          InterpretECALL(instr);
          return;
        case EBREAK:
          FATAL("Encountered EBREAK");
          break;
        default:
          IllegalInstruction(instr);
      }
      break;
    case CSRRW: {
      if (instr.rd() == ZERO) {
        // No read effect.
        CSRWrite(instr.csr(), get_xreg(instr.rs1()));
      } else {
        intx_t result = CSRRead(instr.csr());
        CSRWrite(instr.csr(), get_xreg(instr.rs1()));
        set_xreg(instr.rd(), result);
      }
      break;
    }
    case CSRRS: {
      intx_t result = CSRRead(instr.csr());
      if (instr.rs1() == ZERO) {
        // No write effect.
      } else {
        CSRSet(instr.csr(), get_xreg(instr.rs1()));
      }
      set_xreg(instr.rd(), result);
      break;
    }
    case CSRRC: {
      intx_t result = CSRRead(instr.csr());
      if (instr.rs1() == ZERO) {
        // No write effect.
      } else {
        CSRClear(instr.csr(), get_xreg(instr.rs1()));
      }
      set_xreg(instr.rd(), result);
      break;
    }
    case CSRRWI: {
      if (instr.rd() == ZERO) {
        // No read effect.
        CSRWrite(instr.csr(), instr.zimm());
      } else {
        intx_t result = CSRRead(instr.csr());
        CSRWrite(instr.csr(), instr.zimm());
        set_xreg(instr.rd(), result);
      }
      break;
    }
    case CSRRSI: {
      intx_t result = CSRRead(instr.csr());
      if (instr.zimm() == 0) {
        // No write effect.
      } else {
        CSRSet(instr.csr(), instr.zimm());
      }
      set_xreg(instr.rd(), result);
      break;
    }
    case CSRRCI: {
      intx_t result = CSRRead(instr.csr());
      if (instr.zimm() == 0) {
        // No write effect.
      } else {
        CSRClear(instr.csr(), instr.zimm());
      }
      set_xreg(instr.rd(), result);
      break;
    }
    default:
      IllegalInstruction(instr);
  }
  pc_ += instr.length();
}

void Simulator::InterpretECALL(Instruction instr) {
  Redirection* redirection = Memory::ToHost<Redirection>(pc_);
  uintx_t ra = get_xreg(RA);
  uintx_t result = redirection->target_(get_xreg(A0), get_xreg(A1),
                                        get_xreg(A2), get_xreg(A3));

#if defined(DEBUG)
  for (intptr_t i = 0; i < kNumRegisters; i++) {
    if (kVolatileRegisters.Includes(Register(i))) {
      xregs_[i] = random_.NextUInt64();
    }
  }
  for (intptr_t i = 0; i < kNumFRegisters; i++) {
    if (kVolatileFRegisters.Includes(FRegister(i))) {
      fregs_[i] = bit_cast<double>(random_.NextUInt64());
    }
  }
#endif

  set_xreg(A0, result);

  reserved_address_ = reserved_value_ = 0;  // Clear reservation.

  pc_ = ra;
}

void Simulator::InterpretAMO(Instruction instr) {
  switch (instr.funct3()) {
    case WIDTH32:
      InterpretAMO32(instr);
      break;
    case WIDTH64:
      InterpretAMO64(instr);
      break;
    default:
      IllegalInstruction(instr);
  }
}

// Note: This implementation does not give full LR/SC semantics because it
// suffers from the ABA problem.

template <typename type>
void Simulator::InterpretLR(Instruction instr) {
  uintx_t addr = get_xreg(instr.rs1());
  if ((addr & (sizeof(type) - 1)) != 0) {
    FATAL("Misaligned atomic memory operation");
  }
  std::atomic<type>* atomic = Memory::ToHost<std::atomic<type>>(addr);
  reserved_address_ = addr;
  reserved_value_ = atomic->load(instr.memory_order());
  set_xreg(instr.rd(), reserved_value_);
}

template <typename type>
void Simulator::InterpretSC(Instruction instr) {
  uintx_t addr = get_xreg(instr.rs1());
  if ((addr & (sizeof(type) - 1)) != 0) {
    FATAL("Misaligned atomic memory operation");
  }
  std::atomic<type>* atomic = Memory::ToHost<std::atomic<type>>(addr);
  if (addr != reserved_address_) {
    set_xreg(instr.rd(), 1);
    return;
  }
  type expected = reserved_value_;
  type desired = get_xreg(instr.rs2());
  bool success =
      atomic->compare_exchange_strong(expected, desired, instr.memory_order());
  set_xreg(instr.rd(), success ? 0 : 1);
}

template <typename type>
void Simulator::InterpretAMOSWAP(Instruction instr) {
  uintx_t addr = get_xreg(instr.rs1());
  if ((addr & (sizeof(type) - 1)) != 0) {
    FATAL("Misaligned atomic memory operation");
  }
  std::atomic<type>* atomic = Memory::ToHost<std::atomic<type>>(addr);
  type desired = get_xreg(instr.rs2());
  type result = atomic->exchange(desired, instr.memory_order());
  set_xreg(instr.rd(), sign_extend(result));
}

template <typename type>
void Simulator::InterpretAMOADD(Instruction instr) {
  uintx_t addr = get_xreg(instr.rs1());
  if ((addr & (sizeof(type) - 1)) != 0) {
    FATAL("Misaligned atomic memory operation");
  }
  std::atomic<type>* atomic = Memory::ToHost<std::atomic<type>>(addr);
  type arg = get_xreg(instr.rs2());
  type result = atomic->fetch_add(arg, instr.memory_order());
  set_xreg(instr.rd(), sign_extend(result));
}

template <typename type>
void Simulator::InterpretAMOXOR(Instruction instr) {
  uintx_t addr = get_xreg(instr.rs1());
  if ((addr & (sizeof(type) - 1)) != 0) {
    FATAL("Misaligned atomic memory operation");
  }
  std::atomic<type>* atomic = Memory::ToHost<std::atomic<type>>(addr);
  type arg = get_xreg(instr.rs2());
  type result = atomic->fetch_xor(arg, instr.memory_order());
  set_xreg(instr.rd(), sign_extend(result));
}

template <typename type>
void Simulator::InterpretAMOAND(Instruction instr) {
  uintx_t addr = get_xreg(instr.rs1());
  if ((addr & (sizeof(type) - 1)) != 0) {
    FATAL("Misaligned atomic memory operation");
  }
  std::atomic<type>* atomic = Memory::ToHost<std::atomic<type>>(addr);
  type arg = get_xreg(instr.rs2());
  type result = atomic->fetch_and(arg, instr.memory_order());
  set_xreg(instr.rd(), sign_extend(result));
}

template <typename type>
void Simulator::InterpretAMOOR(Instruction instr) {
  uintx_t addr = get_xreg(instr.rs1());
  if ((addr & (sizeof(type) - 1)) != 0) {
    FATAL("Misaligned atomic memory operation");
  }
  std::atomic<type>* atomic = Memory::ToHost<std::atomic<type>>(addr);
  type arg = get_xreg(instr.rs2());
  type result = atomic->fetch_or(arg, instr.memory_order());
  set_xreg(instr.rd(), sign_extend(result));
}

template <typename type>
void Simulator::InterpretAMOMIN(Instruction instr) {
  uintx_t addr = get_xreg(instr.rs1());
  if ((addr & (sizeof(type) - 1)) != 0) {
    FATAL("Misaligned atomic memory operation");
  }
  std::atomic<type>* atomic = Memory::ToHost<std::atomic<type>>(addr);
  type expected = atomic->load(std::memory_order_relaxed);
  type compare = get_xreg(instr.rs2());
  type desired;
  do {
    desired = expected < compare ? expected : compare;
  } while (
      !atomic->compare_exchange_weak(expected, desired, instr.memory_order()));
  set_xreg(instr.rd(), sign_extend(expected));
}

template <typename type>
void Simulator::InterpretAMOMAX(Instruction instr) {
  uintx_t addr = get_xreg(instr.rs1());
  if ((addr & (sizeof(type) - 1)) != 0) {
    FATAL("Misaligned atomic memory operation");
  }
  std::atomic<type>* atomic = Memory::ToHost<std::atomic<type>>(addr);
  type expected = atomic->load(std::memory_order_relaxed);
  type compare = get_xreg(instr.rs2());
  type desired;
  do {
    desired = expected > compare ? expected : compare;
  } while (
      !atomic->compare_exchange_weak(expected, desired, instr.memory_order()));
  set_xreg(instr.rd(), sign_extend(expected));
}

void Simulator::InterpretAMO32(Instruction instr) {
  switch (instr.funct5()) {
    case LR:
      InterpretLR<int32_t>(instr);
      break;
    case SC:
      InterpretSC<int32_t>(instr);
      break;
    case AMOSWAP:
      InterpretAMOSWAP<int32_t>(instr);
      break;
    case AMOADD:
      InterpretAMOADD<int32_t>(instr);
      break;
    case AMOXOR:
      InterpretAMOXOR<int32_t>(instr);
      break;
    case AMOAND:
      InterpretAMOAND<int32_t>(instr);
      break;
    case AMOOR:
      InterpretAMOOR<int32_t>(instr);
      break;
    case AMOMIN:
      InterpretAMOMIN<int32_t>(instr);
      break;
    case AMOMAX:
      InterpretAMOMAX<int32_t>(instr);
      break;
    case AMOMINU:
      InterpretAMOMIN<uint32_t>(instr);
      break;
    case AMOMAXU:
      InterpretAMOMAX<uint32_t>(instr);
      break;
    default:
      IllegalInstruction(instr);
  }
  pc_ += instr.length();
}

void Simulator::InterpretAMO64(Instruction instr) {
  switch (instr.funct5()) {
#if XLEN >= 64
    case LR:
      InterpretLR<int64_t>(instr);
      break;
    case SC:
      InterpretSC<int64_t>(instr);
      break;
    case AMOSWAP:
      InterpretAMOSWAP<int64_t>(instr);
      break;
    case AMOADD:
      InterpretAMOADD<int64_t>(instr);
      break;
    case AMOXOR:
      InterpretAMOXOR<int64_t>(instr);
      break;
    case AMOAND:
      InterpretAMOAND<int64_t>(instr);
      break;
    case AMOOR:
      InterpretAMOOR<int64_t>(instr);
      break;
    case AMOMIN:
      InterpretAMOMIN<int64_t>(instr);
      break;
    case AMOMAX:
      InterpretAMOMAX<int64_t>(instr);
      break;
    case AMOMINU:
      InterpretAMOMIN<uint64_t>(instr);
      break;
    case AMOMAXU:
      InterpretAMOMAX<uint64_t>(instr);
      break;
#endif  // XLEN >= 64
    default:
      IllegalInstruction(instr);
  }
  pc_ += instr.length();
}

void Simulator::InterpretFMADD(Instruction instr) {
  switch (instr.funct2()) {
    case F2_S: {
      float rs1 = get_fregs(instr.frs1());
      float rs2 = get_fregs(instr.frs2());
      float rs3 = get_fregs(instr.frs3());
      set_fregs(instr.frd(), (rs1 * rs2) + rs3);
      break;
    }
    case F2_D: {
      double rs1 = get_fregd(instr.frs1());
      double rs2 = get_fregd(instr.frs2());
      double rs3 = get_fregd(instr.frs3());
      set_fregd(instr.frd(), (rs1 * rs2) + rs3);
      break;
    }
    default:
      IllegalInstruction(instr);
  }
  pc_ += instr.length();
}

void Simulator::InterpretFMSUB(Instruction instr) {
  switch (instr.funct2()) {
    case F2_S: {
      float rs1 = get_fregs(instr.frs1());
      float rs2 = get_fregs(instr.frs2());
      float rs3 = get_fregs(instr.frs3());
      set_fregs(instr.frd(), (rs1 * rs2) - rs3);
      break;
    }
    case F2_D: {
      double rs1 = get_fregd(instr.frs1());
      double rs2 = get_fregd(instr.frs2());
      double rs3 = get_fregd(instr.frs3());
      set_fregd(instr.frd(), (rs1 * rs2) - rs3);
      break;
    }
    default:
      IllegalInstruction(instr);
  }
  pc_ += instr.length();
}

void Simulator::InterpretFNMSUB(Instruction instr) {
  switch (instr.funct2()) {
    case F2_S: {
      float rs1 = get_fregs(instr.frs1());
      float rs2 = get_fregs(instr.frs2());
      float rs3 = get_fregs(instr.frs3());
      set_fregs(instr.frd(), -(rs1 * rs2) + rs3);
      break;
    }
    case F2_D: {
      double rs1 = get_fregd(instr.frs1());
      double rs2 = get_fregd(instr.frs2());
      double rs3 = get_fregd(instr.frs3());
      set_fregd(instr.frd(), -(rs1 * rs2) + rs3);
      break;
    }
    default:
      IllegalInstruction(instr);
  }
  pc_ += instr.length();
}

void Simulator::InterpretFNMADD(Instruction instr) {
  switch (instr.funct2()) {
    case F2_S: {
      float rs1 = get_fregs(instr.frs1());
      float rs2 = get_fregs(instr.frs2());
      float rs3 = get_fregs(instr.frs3());
      set_fregs(instr.frd(), -(rs1 * rs2) - rs3);
      break;
    }
    case F2_D: {
      double rs1 = get_fregd(instr.frs1());
      double rs2 = get_fregd(instr.frs2());
      double rs3 = get_fregd(instr.frs3());
      set_fregd(instr.frd(), -(rs1 * rs2) - rs3);
      break;
    }
    default:
      IllegalInstruction(instr);
  }
  pc_ += instr.length();
}

// "For the purposes of these instructions only, the value −0.0 is considered to
//  be less than the value +0.0. If both inputs are NaNs, the result is the
//  canonical NaN. If only one operand is a NaN, the result is the non-NaN
//  operand."
static double rv_fmin(double x, double y) {
  if (isnan(x) && isnan(y)) return std::numeric_limits<double>::quiet_NaN();
  if (isnan(x)) return y;
  if (isnan(y)) return x;
  if (x == y) return signbit(x) ? x : y;
  return fmin(x, y);
}

static double rv_fmax(double x, double y) {
  if (isnan(x) && isnan(y)) return std::numeric_limits<double>::quiet_NaN();
  if (isnan(x)) return y;
  if (isnan(y)) return x;
  if (x == y) return signbit(x) ? y : x;
  return fmax(x, y);
}

static float rv_fminf(float x, float y) {
  if (isnan(x) && isnan(y)) return std::numeric_limits<float>::quiet_NaN();
  if (isnan(x)) return y;
  if (isnan(y)) return x;
  if (x == y) return signbit(x) ? x : y;
  return fminf(x, y);
}

static float rv_fmaxf(float x, float y) {
  if (isnan(x) && isnan(y)) return std::numeric_limits<float>::quiet_NaN();
  if (isnan(x)) return y;
  if (isnan(y)) return x;
  if (x == y) return signbit(x) ? y : x;
  return fmaxf(x, y);
}

static bool is_quiet(float x) {
  // Warning: This is true on Intel/ARM, but not everywhere.
  return (bit_cast<uint32_t>(x) & (static_cast<uint32_t>(1) << 22)) != 0;
}

static uintx_t fclass(float x) {
  ASSERT(!is_quiet(std::numeric_limits<float>::signaling_NaN()));
  ASSERT(is_quiet(std::numeric_limits<float>::quiet_NaN()));

  switch (fpclassify(x)) {
    case FP_INFINITE:
      return signbit(x) ? kFClassNegInfinity : kFClassPosInfinity;
    case FP_NAN:
      return is_quiet(x) ? kFClassQuietNan : kFClassSignallingNan;
    case FP_ZERO:
      return signbit(x) ? kFClassNegZero : kFClassPosZero;
    case FP_SUBNORMAL:
      return signbit(x) ? kFClassNegSubnormal : kFClassPosSubnormal;
    case FP_NORMAL:
      return signbit(x) ? kFClassNegNormal : kFClassPosNormal;
    default:
      UNREACHABLE();
      return 0;
  }
}

static bool is_quiet(double x) {
  // Warning: This is true on Intel/ARM, but not everywhere.
  return (bit_cast<uint64_t>(x) & (static_cast<uint64_t>(1) << 51)) != 0;
}

static uintx_t fclass(double x) {
  ASSERT(!is_quiet(std::numeric_limits<double>::signaling_NaN()));
  ASSERT(is_quiet(std::numeric_limits<double>::quiet_NaN()));

  switch (fpclassify(x)) {
    case FP_INFINITE:
      return signbit(x) ? kFClassNegInfinity : kFClassPosInfinity;
    case FP_NAN:
      return is_quiet(x) ? kFClassQuietNan : kFClassSignallingNan;
    case FP_ZERO:
      return signbit(x) ? kFClassNegZero : kFClassPosZero;
    case FP_SUBNORMAL:
      return signbit(x) ? kFClassNegSubnormal : kFClassPosSubnormal;
    case FP_NORMAL:
      return signbit(x) ? kFClassNegNormal : kFClassPosNormal;
    default:
      UNREACHABLE();
      return 0;
  }
}

static float roundevenf(float x) {
  float rounded = roundf(x);
  if (fabsf(x - rounded) == 0.5f) {  // Tie
    if (fmodf(rounded, 2) != 0) {    // Not even
      if (rounded > 0.0f) {
        rounded -= 1.0f;
      } else {
        rounded += 1.0f;
      }
      ASSERT(fmodf(rounded, 2) == 0);
    }
  }
  return rounded;
}

static double roundeven(double x) {
  double rounded = round(x);
  if (fabs(x - rounded) == 0.5f) {  // Tie
    if (fmod(rounded, 2) != 0) {    // Not even
      if (rounded > 0.0f) {
        rounded -= 1.0f;
      } else {
        rounded += 1.0f;
      }
      ASSERT(fmod(rounded, 2) == 0);
    }
  }
  return rounded;
}

static float Round(float x, RoundingMode rounding) {
  switch (rounding) {
    case RNE:  // Round to Nearest, ties to Even
      return roundevenf(x);
    case RTZ:  // Round towards Zero
      return truncf(x);
    case RDN:  // Round Down (toward negative infinity)
      return floorf(x);
    case RUP:  // Round Up (toward positive infinity)
      return ceilf(x);
    case RMM:  // Round to nearest, ties to Max Magnitude
      return roundf(x);
    case DYN:  // Dynamic rounding mode
      UNIMPLEMENTED();
    default:
      FATAL("Invalid rounding mode");
  }
}

static double Round(double x, RoundingMode rounding) {
  switch (rounding) {
    case RNE:  // Round to Nearest, ties to Even
      return roundeven(x);
    case RTZ:  // Round towards Zero
      return trunc(x);
    case RDN:  // Round Down (toward negative infinity)
      return floor(x);
    case RUP:  // Round Up (toward positive infinity)
      return ceil(x);
    case RMM:  // Round to nearest, ties to Max Magnitude
      return round(x);
    case DYN:  // Dynamic rounding mode
      UNIMPLEMENTED();
    default:
      FATAL("Invalid rounding mode");
  }
}

static int32_t fcvtws(float x, RoundingMode rounding) {
  if (x < static_cast<float>(kMinInt32)) {
    return kMinInt32;  // Negative infinity.
  }
  if (x < static_cast<float>(kMaxInt32)) {
    return static_cast<int32_t>(Round(x, rounding));
  }
  return kMaxInt32;  // Positive infinity, NaN.
}

static uint32_t fcvtwus(float x, RoundingMode rounding) {
  if (x < static_cast<float>(0)) {
    return 0;  // Negative infinity.
  }
  if (x < static_cast<float>(kMaxUint32)) {
    return static_cast<uint32_t>(Round(x, rounding));
  }
  return kMaxUint32;  // Positive infinity, NaN.
}

#if XLEN >= 64
static int64_t fcvtls(float x, RoundingMode rounding) {
  if (x < static_cast<float>(kMinInt64)) {
    return kMinInt64;  // Negative infinity.
  }
  if (x < static_cast<float>(kMaxInt64)) {
    return static_cast<int64_t>(Round(x, rounding));
  }
  return kMaxInt64;  // Positive infinity, NaN.
}

static uint64_t fcvtlus(float x, RoundingMode rounding) {
  if (x < static_cast<float>(0.0)) {
    return 0;  // Negative infinity.
  }
  if (x < static_cast<float>(kMaxUint64)) {
    return static_cast<uint64_t>(Round(x, rounding));
  }
  return kMaxUint64;  // Positive infinity, NaN.
}
#endif  // XLEN >= 64

static int32_t fcvtwd(double x, RoundingMode rounding) {
  if (x < static_cast<double>(kMinInt32)) {
    return kMinInt32;  // Negative infinity.
  }
  if (x < static_cast<double>(kMaxInt32)) {
    return static_cast<int32_t>(Round(x, rounding));
  }
  return kMaxInt32;  // Positive infinity, NaN.
}

static uint32_t fcvtwud(double x, RoundingMode rounding) {
  if (x < static_cast<double>(0)) {
    return 0;  // Negative infinity.
  }
  if (x < static_cast<double>(kMaxUint32)) {
    return static_cast<uint32_t>(Round(x, rounding));
  }
  return kMaxUint32;  // Positive infinity, NaN.
}

#if XLEN >= 64
static int64_t fcvtld(double x, RoundingMode rounding) {
  if (x < static_cast<double>(kMinInt64)) {
    return kMinInt64;  // Negative infinity.
  }
  if (x < static_cast<double>(kMaxInt64)) {
    return static_cast<int64_t>(Round(x, rounding));
  }
  return kMaxInt64;  // Positive infinity, NaN.
}

static uint64_t fcvtlud(double x, RoundingMode rounding) {
  if (x < static_cast<double>(0.0)) {
    return 0;  // Negative infinity.
  }
  if (x < static_cast<double>(kMaxUint64)) {
    return static_cast<uint64_t>(Round(x, rounding));
  }
  return kMaxUint64;  // Positive infinity, NaN.
}
#endif  // XLEN >= 64

void Simulator::InterpretOPFP(Instruction instr) {
  switch (instr.funct7()) {
    case FADDS: {
      float rs1 = get_fregs(instr.frs1());
      float rs2 = get_fregs(instr.frs2());
      set_fregs(instr.frd(), rs1 + rs2);
      break;
    }
    case FSUBS: {
      float rs1 = get_fregs(instr.frs1());
      float rs2 = get_fregs(instr.frs2());
      set_fregs(instr.frd(), rs1 - rs2);
      break;
    }
    case FMULS: {
      float rs1 = get_fregs(instr.frs1());
      float rs2 = get_fregs(instr.frs2());
      set_fregs(instr.frd(), rs1 * rs2);
      break;
    }
    case FDIVS: {
      float rs1 = get_fregs(instr.frs1());
      float rs2 = get_fregs(instr.frs2());
      set_fregs(instr.frd(), rs1 / rs2);
      break;
    }
    case FSQRTS: {
      float rs1 = get_fregs(instr.frs1());
      set_fregs(instr.frd(), sqrtf(rs1));
      break;
    }
    case FSGNJS: {
      const uint32_t kSignMask = static_cast<uint32_t>(1) << 31;
      uint32_t rs1 = bit_cast<uint32_t>(get_fregs(instr.frs1()));
      uint32_t rs2 = bit_cast<uint32_t>(get_fregs(instr.frs2()));
      uint32_t result;
      switch (instr.funct3()) {
        case J:
          result = (rs1 & ~kSignMask) | (rs2 & kSignMask);
          break;
        case JN:
          result = (rs1 & ~kSignMask) | (~rs2 & kSignMask);
          break;
        case JX:
          result = (rs1 & ~kSignMask) | ((rs1 ^ rs2) & kSignMask);
          break;
        default:
          IllegalInstruction(instr);
      }
      set_fregs(instr.frd(), bit_cast<float>(result));
      break;
    }
    case FMINMAXS: {
      float rs1 = get_fregs(instr.frs1());
      float rs2 = get_fregs(instr.frs2());
      switch (instr.funct3()) {
        case FMIN:
          set_fregs(instr.frd(), rv_fminf(rs1, rs2));
          break;
        case FMAX:
          set_fregs(instr.frd(), rv_fmaxf(rs1, rs2));
          break;
        default:
          IllegalInstruction(instr);
      }
      break;
    }
    case FCMPS: {
      float rs1 = get_fregs(instr.frs1());
      float rs2 = get_fregs(instr.frs2());
      switch (instr.funct3()) {
        case FEQ:
          set_xreg(instr.rd(), rs1 == rs2 ? 1 : 0);
          break;
        case FLT:
          set_xreg(instr.rd(), rs1 < rs2 ? 1 : 0);
          break;
        case FLE:
          set_xreg(instr.rd(), rs1 <= rs2 ? 1 : 0);
          break;
        default:
          IllegalInstruction(instr);
      }
      break;
    }
    case FCLASSS:  // = FMVXW
      switch (instr.funct3()) {
        case 1:
          // fclass.s
          set_xreg(instr.rd(), fclass(get_fregs(instr.frs1())));
          break;
        case 0:
          // fmv.x.s
          set_xreg(instr.rd(),
                   sign_extend(bit_cast<int32_t>(get_fregs(instr.frs1()))));
          break;
        default:
          IllegalInstruction(instr);
      }
      break;
    case FCVTintS:
      switch (instr.rs2().encoding()) {
        case W:
          set_xreg(instr.rd(), sign_extend(fcvtws(get_fregs(instr.frs1()),
                                                  instr.rounding())));
          break;
        case WU:
          set_xreg(instr.rd(), sign_extend(fcvtwus(get_fregs(instr.frs1()),
                                                   instr.rounding())));
          break;
#if XLEN >= 64
        case L:
          set_xreg(instr.rd(), sign_extend(fcvtls(get_fregs(instr.frs1()),
                                                  instr.rounding())));
          break;
        case LU:
          set_xreg(instr.rd(), sign_extend(fcvtlus(get_fregs(instr.frs1()),
                                                   instr.rounding())));
          break;
#endif  // XLEN >= 64
        default:
          IllegalInstruction(instr);
      }
      break;
    case FCVTSint:
      switch (instr.rs2().encoding()) {
        case W:
          set_fregs(
              instr.frd(),
              static_cast<float>(static_cast<int32_t>(get_xreg(instr.rs1()))));
          break;
        case WU:
          set_fregs(
              instr.frd(),
              static_cast<float>(static_cast<uint32_t>(get_xreg(instr.rs1()))));
          break;
#if XLEN >= 64
        case L:
          set_fregs(
              instr.frd(),
              static_cast<float>(static_cast<int64_t>(get_xreg(instr.rs1()))));
          break;
        case LU:
          set_fregs(
              instr.frd(),
              static_cast<float>(static_cast<uint64_t>(get_xreg(instr.rs1()))));
          break;
#endif  // XLEN >= 64
        default:
          IllegalInstruction(instr);
      }
      break;
    case FMVWX:
      set_fregs(instr.frd(),
                bit_cast<float>(static_cast<int32_t>(get_xreg(instr.rs1()))));
      break;
    case FADDD: {
      double rs1 = get_fregd(instr.frs1());
      double rs2 = get_fregd(instr.frs2());
      set_fregd(instr.frd(), rs1 + rs2);
      break;
    }
    case FSUBD: {
      double rs1 = get_fregd(instr.frs1());
      double rs2 = get_fregd(instr.frs2());
      set_fregd(instr.frd(), rs1 - rs2);
      break;
    }
    case FMULD: {
      double rs1 = get_fregd(instr.frs1());
      double rs2 = get_fregd(instr.frs2());
      set_fregd(instr.frd(), rs1 * rs2);
      break;
    }
    case FDIVD: {
      double rs1 = get_fregd(instr.frs1());
      double rs2 = get_fregd(instr.frs2());
      set_fregd(instr.frd(), rs1 / rs2);
      break;
    }
    case FSQRTD: {
      double rs1 = get_fregd(instr.frs1());
      set_fregd(instr.frd(), sqrt(rs1));
      break;
    }
    case FSGNJD: {
      const uint64_t kSignMask = static_cast<uint64_t>(1) << 63;
      uint64_t rs1 = bit_cast<uint64_t>(get_fregd(instr.frs1()));
      uint64_t rs2 = bit_cast<uint64_t>(get_fregd(instr.frs2()));
      uint64_t result;
      switch (instr.funct3()) {
        case J:
          result = (rs1 & ~kSignMask) | (rs2 & kSignMask);
          break;
        case JN:
          result = (rs1 & ~kSignMask) | (~rs2 & kSignMask);
          break;
        case JX:
          result = (rs1 & ~kSignMask) | ((rs1 ^ rs2) & kSignMask);
          break;
        default:
          IllegalInstruction(instr);
      }
      set_fregd(instr.frd(), bit_cast<double>(result));
      break;
    }
    case FMINMAXD: {
      double rs1 = get_fregd(instr.frs1());
      double rs2 = get_fregd(instr.frs2());
      switch (instr.funct3()) {
        case FMIN:
          set_fregd(instr.frd(), rv_fmin(rs1, rs2));
          break;
        case FMAX:
          set_fregd(instr.frd(), rv_fmax(rs1, rs2));
          break;
        default:
          IllegalInstruction(instr);
      }
      break;
    }
    case FCVTS: {
      switch (instr.rs2().encoding()) {
        case 1:
          set_fregs(instr.frd(), static_cast<float>(get_fregd(instr.frs1())));
          break;
        default:
          IllegalInstruction(instr);
      }
      break;
    }
    case FCVTD: {
      switch (instr.rs2().encoding()) {
        case 0:
          set_fregd(instr.frd(), static_cast<double>(get_fregs(instr.frs1())));
          break;
        default:
          IllegalInstruction(instr);
      }
      break;
    }

    case FCMPD: {
      double rs1 = get_fregd(instr.frs1());
      double rs2 = get_fregd(instr.frs2());
      switch (instr.funct3()) {
        case FEQ:
          set_xreg(instr.rd(), rs1 == rs2 ? 1 : 0);
          break;
        case FLT:
          set_xreg(instr.rd(), rs1 < rs2 ? 1 : 0);
          break;
        case FLE:
          set_xreg(instr.rd(), rs1 <= rs2 ? 1 : 0);
          break;
        default:
          IllegalInstruction(instr);
      }
      break;
    }
    case FCLASSD:  // = FMVXD
      switch (instr.funct3()) {
        case 1:
          // fclass.d
          set_xreg(instr.rd(), fclass(get_fregd(instr.frs1())));
          break;
#if XLEN >= 64
        case 0:
          // fmv.x.d
          set_xreg(instr.rd(), bit_cast<int64_t>(get_fregd(instr.frs1())));
          break;
#endif  // XLEN >= 64
        default:
          IllegalInstruction(instr);
      }
      break;
    case FCVTintD:
      switch (instr.rs2().encoding()) {
        case W:
          set_xreg(instr.rd(), sign_extend(fcvtwd(get_fregd(instr.frs1()),
                                                  instr.rounding())));
          break;
        case WU:
          set_xreg(instr.rd(), sign_extend(fcvtwud(get_fregd(instr.frs1()),
                                                   instr.rounding())));
          break;
#if XLEN >= 64
        case L:
          set_xreg(instr.rd(), sign_extend(fcvtld(get_fregd(instr.frs1()),
                                                  instr.rounding())));
          break;
        case LU:
          set_xreg(instr.rd(), sign_extend(fcvtlud(get_fregd(instr.frs1()),
                                                   instr.rounding())));
          break;
#endif  // XLEN >= 64
        default:
          IllegalInstruction(instr);
      }
      break;
    case FCVTDint:
      switch (instr.rs2().encoding()) {
        case W:
          set_fregd(
              instr.frd(),
              static_cast<double>(static_cast<int32_t>(get_xreg(instr.rs1()))));
          break;
        case WU:
          set_fregd(instr.frd(), static_cast<double>(static_cast<uint32_t>(
                                     get_xreg(instr.rs1()))));
          break;
#if XLEN >= 64
        case L:
          set_fregd(
              instr.frd(),
              static_cast<double>(static_cast<int64_t>(get_xreg(instr.rs1()))));
          break;
        case LU:
          set_fregd(instr.frd(), static_cast<double>(static_cast<uint64_t>(
                                     get_xreg(instr.rs1()))));
          break;
#endif  // XLEN >= 64
        default:
          IllegalInstruction(instr);
      }
      break;
#if XLEN >= 64
    case FMVDX:
      set_fregd(instr.frd(), bit_cast<double>(get_xreg(instr.rs1())));
      break;
#endif  // XLEN >= 64
    default:
      IllegalInstruction(instr);
  }
  pc_ += instr.length();
}

enum ControlStatusRegister {
  fflags = 0x001,
  frm = 0x002,
  fcsr = 0x003,
  cycle = 0xC00,
  time = 0xC01,
  instret = 0xC02,
#if XLEN == 32
  cycleh = 0xC80,
  timeh = 0xC81,
  instreth = 0xC82,
#endif
};

void Simulator::IllegalInstruction(Instruction instr) {
  PrintRegisters();
  FATAL("Illegal instruction: 0x%08x", instr.encoding());
}

void Simulator::IllegalInstruction(CInstruction instr) {
  PrintRegisters();
  FATAL("Illegal instruction: 0x%04x", instr.encoding());
}

intx_t Simulator::CSRRead(uint16_t csr) {
  switch (csr) {
    case fcsr:
      return fcsr_;
    case cycle:
      return instret_ / 2;
    case time:
      return 0;
    case instret:
      return instret_;
#if XLEN == 32
    case cycleh:
      return (instret_ / 2) >> 32;
    case timeh:
      return 0;
    case instreth:
      return instret_ >> 32;
#endif
    default:
      FATAL("Unknown CSR: %d", csr);
  }
}

void Simulator::CSRWrite(uint16_t csr, intx_t value) {
  UNIMPLEMENTED();
}

void Simulator::CSRSet(uint16_t csr, intx_t mask) {
  UNIMPLEMENTED();
}

void Simulator::CSRClear(uint16_t csr, intx_t mask) {
  UNIMPLEMENTED();
}
#endif  // defined(USING_SIMULATOR)

}  // namespace psoup
