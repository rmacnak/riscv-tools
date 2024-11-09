// Copyright (c) 2017, the Newspeak project authors. Please see the AUTHORS file
// for details. All rights reserved. Use of this source code is governed by a
// BSD-style license that can be found in the LICENSE file.

#ifndef VM_SIMULATOR_RISCV_H_
#define VM_SIMULATOR_RISCV_H_

#include "vm/constants_riscv.h"
#include "vm/random.h"

namespace psoup {

#if !defined(__riscv)
#define USING_SIMULATOR 1
#endif

class Memory {
 public:
  static void Startup(size_t size);
  static void Shutdown();

  static void* Allocate(size_t size);
  static void Free(void* ptr);

  static intx_t ToGuest(const void* ptr) {
#if XLEN == HOST_XLEN
    return reinterpret_cast<uintx_t>(ptr);
#else
    uintptr_t host_addr = reinterpret_cast<uintptr_t>(ptr);
    ASSERT(host_addr >= guest_base_);
    uintptr_t guest_addr = host_addr - guest_base_;
    ASSERT(guest_addr <= guest_size_);
    return static_cast<uintx_t>(guest_addr);
#endif
  }

  template <typename T>
  static T* ToHost(intx_t addr) {
#if XLEN == HOST_XLEN
    return reinterpret_cast<T*>(addr);
#else
    uintptr_t guest_addr = static_cast<uintptr_t>(static_cast<uintx_t>(addr));
    ASSERT(guest_addr <= guest_size_);
    uintptr_t host_addr = guest_addr + guest_base_;
    return reinterpret_cast<T*>(host_addr);
#endif
  }

 private:
#if XLEN != HOST_XLEN
  static uintptr_t guest_base_;
  static uintptr_t guest_size_;
  static uintptr_t top_;
#endif
};

// Allows callouts from the simulation to the host.
#if defined(USING_SIMULATOR)
class Redirection {
 public:
  typedef intx_t (*Function)(intx_t, intx_t, intx_t, intx_t);

  explicit Redirection(Function target)
      : ecall_instr_(ECALL << 20 | SYSTEM), target_(target) {}

  void* entry_point() { return &ecall_instr_; }

 private:
  uint32_t ecall_instr_;
  Function const target_;

  friend class Simulator;
};
#else
class Redirection {
 public:
  typedef intx_t (*Function)(intx_t, intx_t, intx_t, intx_t);

  explicit Redirection(Function target) : target_(target) {}

  void* entry_point() const { return reinterpret_cast<void*>(target_); }

 private:
  Function target_;
};
#endif

// Simulates one hart.
//
// Limitations:
//  - LR/SC are implemented in terms of std::atomic::compare_exchange_strong.
//    This means an LR/SC pair may incorrectly succeed in an ABA situation.
//
//  - csr not implemented
//  - fcsr not implemented -> dynamic rounding not implemented
#if defined(USING_SIMULATOR)
class Simulator {
 public:
  Simulator();
  ~Simulator();

  intx_t Call(void* function,
              intx_t arg0 = 0,
              intx_t arg1 = 0,
              intx_t arg2 = 0) {
    PreservedRegisters preserved;
    PrepareCall(&preserved);
    set_xreg(A0, arg0);
    set_xreg(A1, arg1);
    set_xreg(A2, arg2);
    RunCall(function, &preserved);
    return get_xreg(A0);
  }

  intx_t CallI(void* function, double arg0, double arg1 = 0.0) {
    PreservedRegisters preserved;
    PrepareCall(&preserved);
    set_fregd(FA0, arg0);
    set_fregd(FA1, arg1);
    RunCall(function, &preserved);
    return get_xreg(A0);
  }
  intx_t CallI(void* function, float arg0, float arg1 = 0.0f) {
    PreservedRegisters preserved;
    PrepareCall(&preserved);
    set_fregs(FA0, arg0);
    set_fregs(FA1, arg1);
    RunCall(function, &preserved);
    return get_xreg(A0);
  }

  double CallD(void* function, intx_t arg0, intx_t arg1 = 0) {
    PreservedRegisters preserved;
    PrepareCall(&preserved);
    set_xreg(A0, arg0);
    set_xreg(A1, arg1);
    RunCall(function, &preserved);
    return get_fregd(FA0);
  }
  double CallD(void* function,
               double arg0,
               double arg1 = 0.0,
               double arg2 = 0.0) {
    PreservedRegisters preserved;
    PrepareCall(&preserved);
    set_fregd(FA0, arg0);
    set_fregd(FA1, arg1);
    set_fregd(FA2, arg2);
    RunCall(function, &preserved);
    return get_fregd(FA0);
  }
  double CallD(void* function, intx_t arg0, double arg1) {
    PreservedRegisters preserved;
    PrepareCall(&preserved);
    set_xreg(A0, arg0);
    set_fregd(FA0, arg1);
    RunCall(function, &preserved);
    return get_fregd(FA0);
  }
  double CallD(void* function, float arg0) {
    PreservedRegisters preserved;
    PrepareCall(&preserved);
    set_fregs(FA0, arg0);
    RunCall(function, &preserved);
    return get_fregd(FA0);
  }

  float CallF(void* function, intx_t arg0, intx_t arg1 = 0) {
    PreservedRegisters preserved;
    PrepareCall(&preserved);
    set_xreg(A0, arg0);
    set_xreg(A1, arg1);
    RunCall(function, &preserved);
    return get_fregs(FA0);
  }
  float CallF(void* function,
              float arg0,
              float arg1 = 0.0f,
              float arg2 = 0.0f) {
    PreservedRegisters preserved;
    PrepareCall(&preserved);
    set_fregs(FA0, arg0);
    set_fregs(FA1, arg1);
    set_fregs(FA2, arg2);
    RunCall(function, &preserved);
    return get_fregs(FA0);
  }
  float CallF(void* function, intx_t arg0, float arg1) {
    PreservedRegisters preserved;
    PrepareCall(&preserved);
    set_xreg(A0, arg0);
    set_fregs(FA0, arg1);
    RunCall(function, &preserved);
    return get_fregs(FA0);
  }
  float CallF(void* function, double arg0) {
    PreservedRegisters preserved;
    PrepareCall(&preserved);
    set_fregd(FA0, arg0);
    RunCall(function, &preserved);
    return get_fregs(FA0);
  }

  void PrintRegisters();

 private:
  struct PreservedRegisters {
    uintx_t xregs[kNumRegisters];
    double fregs[kNumFRegisters];
  };
  void PrepareCall(PreservedRegisters* preserved);
  void RunCall(void* function, PreservedRegisters* preserved);

  void Interpret(Instruction instr);
  void Interpret(CInstruction instr);
  void InterpretLUI(Instruction instr);
  void InterpretAUIPC(Instruction instr);
  void InterpretJAL(Instruction instr);
  void InterpretJALR(Instruction instr);
  void InterpretBRANCH(Instruction instr);
  void InterpretLOAD(Instruction instr);
  void InterpretSTORE(Instruction instr);
  void InterpretOPIMM(Instruction instr);
  void InterpretOPIMM32(Instruction instr);
  void InterpretOP(Instruction instr);
  void InterpretOP_0(Instruction instr);
  void InterpretOP_SUB(Instruction instr);
  void InterpretOP_MULDIV(Instruction instr);
  void InterpretOP_SHADD(Instruction instr);
  void InterpretOP_MINMAXCLMUL(Instruction instr);
  void InterpretOP_ROTATE(Instruction instr);
  void InterpretOP_BCLRBEXT(Instruction instr);
  void InterpretOP_CZERO(Instruction instr);
  void InterpretOP32(Instruction instr);
  void InterpretOP32_0(Instruction instr);
  void InterpretOP32_SUB(Instruction instr);
  void InterpretOP32_MULDIV(Instruction instr);
  void InterpretOP32_SHADD(Instruction instr);
  void InterpretOP32_ADDUW(Instruction instr);
  void InterpretOP32_ROTATE(Instruction instr);
  void InterpretMISCMEM(Instruction instr);
  void InterpretSYSTEM(Instruction instr);
  void InterpretECALL(Instruction instr);
  void InterpretAMO(Instruction instr);
  void InterpretAMO8(Instruction instr);
  void InterpretAMO16(Instruction instr);
  void InterpretAMO32(Instruction instr);
  void InterpretAMO64(Instruction instr);
  template <typename type>
  void InterpretLR(Instruction instr);
  template <typename type>
  void InterpretSC(Instruction instr);
  template <typename type>
  void InterpretAMOSWAP(Instruction instr);
  template <typename type>
  void InterpretAMOADD(Instruction instr);
  template <typename type>
  void InterpretAMOXOR(Instruction instr);
  template <typename type>
  void InterpretAMOAND(Instruction instr);
  template <typename type>
  void InterpretAMOOR(Instruction instr);
  template <typename type>
  void InterpretAMOMIN(Instruction instr);
  template <typename type>
  void InterpretAMOMAX(Instruction instr);
  template <typename type>
  void InterpretAMOMINU(Instruction instr);
  template <typename type>
  void InterpretAMOMAXU(Instruction instr);
  template <typename type>
  void InterpretLOADORDERED(Instruction instr);
  template <typename type>
  void InterpretSTOREORDERED(Instruction instr);
  void InterpretLOADFP(Instruction instr);
  void InterpretSTOREFP(Instruction instr);
  void InterpretFMADD(Instruction instr);
  void InterpretFMSUB(Instruction instr);
  void InterpretFNMADD(Instruction instr);
  void InterpretFNMSUB(Instruction instr);
  void InterpretOPFP(Instruction instr);
  void IllegalInstruction(Instruction instr);
  void IllegalInstruction(CInstruction instr);

  template <typename type>
  type MemoryRead(uintx_t addr, Register base);
  template <typename type>
  void MemoryWrite(uintx_t addr, type value, Register base);

  intx_t CSRRead(uint16_t csr);
  void CSRWrite(uint16_t csr, intx_t value);
  void CSRSet(uint16_t csr, intx_t mask);
  void CSRClear(uint16_t csr, intx_t mask);

  uintx_t get_xreg(Register rs) const { return xregs_[rs.encoding()]; }
  void set_xreg(Register rd, uintx_t value) {
    if (rd != ZERO) {
      xregs_[rd.encoding()] = value;
    }
  }

  double get_fregd(FRegister rs) const { return fregs_[rs.encoding()]; }
  void set_fregd(FRegister rd, double value) { fregs_[rd.encoding()] = value; }

  static constexpr uint64_t kNaNBox = 0xFFFFFFFF00000000;

  float get_fregs(FRegister rs) const {
    uint64_t bits64 = bit_cast<uint64_t>(fregs_[rs.encoding()]);
    if ((bits64 & kNaNBox) != kNaNBox) {
      // When the register value isn't a valid NaN, the canonical NaN is used
      // instead.
      return bit_cast<float>(0x7fc00000);
    }
    uint32_t bits32 = static_cast<uint32_t>(bits64);
    return bit_cast<float>(bits32);
  }
  void set_fregs(FRegister rd, float value) {
    uint32_t bits32 = bit_cast<uint32_t>(value);
    uint64_t bits64 = static_cast<uint64_t>(bits32);
    bits64 |= kNaNBox;
    fregs_[rd.encoding()] = bit_cast<double>(bits64);
  }

  void* stack_;
  uintx_t stack_base_;
  Random random_;
  bool trace_;

  // I state
  uintx_t pc_;
  uintx_t xregs_[kNumRegisters];
  uint64_t instret_;

  // A state
  uintx_t reserved_address_;
  uintx_t reserved_value_;

  // F/D state
  double fregs_[kNumFRegisters];
  uint32_t fcsr_;
};
#else
class Simulator {
 public:
  intx_t Call(void* function,
              intx_t arg0 = 0,
              intx_t arg1 = 0,
              intx_t arg2 = 0) {
    typedef intx_t (*Function)(intx_t, intx_t, intx_t);
    return reinterpret_cast<Function>(function)(arg0, arg1, arg2);
  }

  intx_t CallI(void* function, double arg0, double arg1 = 0.0) {
    typedef intx_t (*Function)(double, double);
    return reinterpret_cast<Function>(function)(arg0, arg1);
  }
  intx_t CallI(void* function, float arg0, float arg1 = 0.0f) {
    typedef intx_t (*Function)(float, float);
    return reinterpret_cast<Function>(function)(arg0, arg1);
  }

  double CallD(void* function, intx_t arg0, intx_t arg1 = 0) {
    typedef double (*Function)(intx_t, intx_t);
    return reinterpret_cast<Function>(function)(arg0, arg1);
  }
  double CallD(void* function,
               double arg0,
               double arg1 = 0.0,
               double arg2 = 0.0) {
    typedef double (*Function)(double, double, double);
    return reinterpret_cast<Function>(function)(arg0, arg1, arg2);
  }
  double CallD(void* function, intx_t arg0, double arg1) {
    typedef double (*Function)(intx_t, double);
    return reinterpret_cast<Function>(function)(arg0, arg1);
  }
  double CallD(void* function, float arg0) {
    typedef double (*Function)(float);
    return reinterpret_cast<Function>(function)(arg0);
  }

  float CallF(void* function, intx_t arg0, intx_t arg1 = 0) {
    typedef float (*Function)(intx_t, intx_t);
    return reinterpret_cast<Function>(function)(arg0, arg1);
  }
  float CallF(void* function,
              float arg0,
              float arg1 = 0.0f,
              float arg2 = 0.0f) {
    typedef float (*Function)(float, float, float);
    return reinterpret_cast<Function>(function)(arg0, arg1, arg2);
  }
  float CallF(void* function, intx_t arg0, float arg1) {
    typedef float (*Function)(intx_t, float);
    return reinterpret_cast<Function>(function)(arg0, arg1);
  }
  float CallF(void* function, double arg0) {
    typedef float (*Function)(double);
    return reinterpret_cast<Function>(function)(arg0);
  }
};
#endif

}  // namespace psoup

#endif  // VM_SIMULATOR_RISCV_H_
