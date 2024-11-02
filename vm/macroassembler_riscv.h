// Copyright (c) 2017, the Newspeak project authors. Please see the AUTHORS file
// for details. All rights reserved. Use of this source code is governed by a
// BSD-style license that can be found in the LICENSE file.

#ifndef VM_MACROASSEMBLER_RISCV_H_
#define VM_MACROASSEMBLER_RISCV_H_

#include "vm/assembler_riscv.h"

namespace psoup {

constexpr Register TMP = A5;
constexpr Register TMP2 = A6;

inline intx_t ImmLo(intx_t imm) {
  return imm << (XLEN - 12) >> (XLEN - 12);
}
inline intx_t ImmHi(intx_t imm) {
  return imm - ImmLo(imm);
}

// Higher level functions that may produce multiple instructions.
class MacroAssembler : public Assembler {
 public:
  explicit MacroAssembler(ExtensionSet extensions = RV_G)
      : Assembler(extensions) {}

  void Push(Register rs) {
    ASSERT(rs != SP);
    addi(SP, SP, -sizeof(uintx_t));
    sx(rs, Address(SP, 0));
  }
  void Pop(Register rd) {
    ASSERT(rd != SP);
    lx(rd, Address(SP, 0));
    addi(SP, SP, sizeof(uintx_t));
  }

  void LoadImmediate(Register rd, intx_t imm) {
    intx_t lo = ImmLo(imm);
    intx_t hi = ImmHi(imm);
    uintx_t uhi = hi;
    if (hi == 0) {
      addi(rd, ZERO, imm);
    } else if (IsUTypeImm(hi)) {
      lui(rd, hi);
      if (lo != 0) {
        addi(rd, rd, lo);
      }
    } else if (IsUTypeImm(imm ^ lo)) {
      lui(rd, imm ^ lo);
      xori(rd, rd, lo);
    } else if (Supports(RV_Zbs) && Utils::IsPowerOfTwo(uhi)) {
      bseti(rd, ZERO, Utils::ShiftForPowerOfTwo(uhi));
      if (lo != 0) {
        addi(rd, rd, lo);
      }
    } else {
      int shift = 12;
      while ((hi >> (shift + 1) << (shift + 1)) == hi) {
        shift++;
      }
      LoadImmediate(rd, hi >> shift);
      slli(rd, rd, shift);
      if (lo != 0) {
        addi(rd, rd, lo);
      }
    }
  }

  Address PrepareLargeOffset(Address address) {
    if (IsITypeImm(address.offset())) {
      return address;
    } else {
      intx_t lo = ImmLo(address.offset());
      intx_t hi = ImmHi(address.offset());
      lui(TMP, hi);
      add(TMP, TMP, address.base());
      return Address(TMP, lo);
    }
  }

  void Load(Register rd, Address address) {
    lx(rd, PrepareLargeOffset(address));
  }

  void Store(Register rs, Address address) {
    sx(rs, PrepareLargeOffset(address));
  }

  void Move(Register rd, Register rs) {
    if (rd != rs) {
      mv(rd, rs);
    }
  }

  void AddImmediate(Register rd, Register rs1, intptr_t imm) {
    if (imm == 0) {
      Move(rd, rs1);
    } else if (IsITypeImm(imm)) {
      addi(rd, rs1, imm);
    } else {
      ASSERT(rs1 != TMP);
      LoadImmediate(TMP, imm);
      add(rd, rs1, TMP);
    }
  }

  void SubImmediate(Register rd, Register rs1, intptr_t imm) {
    if (imm == 0) {
      Move(rd, rs1);
    } else if (IsITypeImm(-imm)) {
      addi(rd, rs1, -imm);
    } else {
      ASSERT(rs1 != TMP);
      LoadImmediate(TMP, imm);
      sub(rd, rs1, TMP);
    }
  }

  void MulImmediate(Register rd, Register rs1, intptr_t imm) {
    if (imm == 1) {
      Move(rd, rs1);
    } else if (Utils::IsPowerOfTwo(imm)) {
      slli(rd, rs1, Utils::ShiftForPowerOfTwo(imm));
    } else {
      LoadImmediate(TMP, imm);
      mul(rd, rs1, TMP);
    }
  }

  void AndImmediate(Register rd, Register rs1, intptr_t imm) {
    uintx_t uimm = imm;
    if (imm == -1) {
      Move(rd, rs1);
#if XLEN >= 64
    } else if (static_cast<int32_t>(imm) == -1) {
      sextw(rd, rs1);
#endif
    } else if (IsITypeImm(imm)) {
      andi(rd, rs1, imm);
    } else if (Supports(RV_Zbs) && Utils::IsPowerOfTwo(~uimm)) {
      bclri(rd, rs1, Utils::ShiftForPowerOfTwo(~uimm));
    } else if (Utils::IsPowerOfTwo(uimm + 1)) {
      intptr_t shift = Utils::ShiftForPowerOfTwo(uimm + 1);
      if (Supports(RV_Zbb) && (shift == 16)) {
        zexth(rd, rs1);
#if XLEN >= 64
      } else if (Supports(RV_Zba) && (shift == 32)) {
        zextw(rd, rs1);
#endif
      } else {
        slli(rd, rs1, XLEN - shift);
        srli(rd, rd, XLEN - shift);
      }
    } else {
      ASSERT(rs1 != TMP);
      LoadImmediate(TMP, imm);
      and_(rd, rs1, TMP);
    }
  }

  void OrImmediate(Register rd, Register rs1, intptr_t imm) {
    uintx_t uimm = imm;
    if (imm == 0) {
      Move(rd, rs1);
    } else if (IsITypeImm(imm)) {
      ori(rd, rs1, imm);
    } else if (Supports(RV_Zbs) && Utils::IsPowerOfTwo(uimm)) {
      bseti(rd, rs1, Utils::ShiftForPowerOfTwo(uimm));
    } else {
      ASSERT(rs1 != TMP);
      LoadImmediate(TMP, imm);
      or_(rd, rs1, TMP);
    }
  }

  void XorImmediate(Register rd, Register rs1, intptr_t imm) {
    uintx_t uimm = imm;
    if (imm == 0) {
      Move(rd, rs1);
    } else if (IsITypeImm(imm)) {
      xori(rd, rs1, imm);
    } else if (Supports(RV_Zbs) && Utils::IsPowerOfTwo(uimm)) {
      binvi(rd, rs1, Utils::ShiftForPowerOfTwo(uimm));
    } else {
      ASSERT(rs1 != TMP);
      LoadImmediate(TMP, imm);
      xor_(rd, rs1, TMP);
    }
  }

  void tbnz(Register rs1, intptr_t bit, Label* label) {
    if (bit == XLEN - 1) {
      bltz(rs1, label);
    } else {
      slli(TMP, rs1, XLEN - 1 - bit);
      bltz(TMP, label);
    }
  }

  void AddBranchOverflow(Register rd,
                         Register rs1,
                         Register rs2,
                         Label* on_overflow) {
    ASSERT(rd != TMP);
    ASSERT(rd != TMP2);
    ASSERT(rs1 != TMP);
    ASSERT(rs1 != TMP2);
    ASSERT(rs2 != TMP);
    ASSERT(rs2 != TMP2);

    if ((rd == rs1) && (rd == rs2)) {
      ASSERT(rs1 == rs2);
      mv(TMP, rs1);
      add(rd, rs1, rs2);  // rs1, rs2 destroyed
      xor_(TMP, TMP, rd);  // TMP negative if sign changed
      bltz(TMP, on_overflow);
    } else if (rs1 == rs2) {
      ASSERT(rd != rs1);
      ASSERT(rd != rs2);
      add(rd, rs1, rs2);
      xor_(TMP, rd, rs1);  // TMP negative if sign changed
      bltz(TMP, on_overflow);
    } else if (rd == rs1) {
      ASSERT(rs1 != rs2);
      slti(TMP, rs1, 0);
      add(rd, rs1, rs2);  // rs1 destroyed
      slt(TMP2, rd, rs2);
      bne(TMP, TMP2, on_overflow);
    } else if (rd == rs2) {
      ASSERT(rs1 != rs2);
      slti(TMP, rs2, 0);
      add(rd, rs1, rs2);  // rs2 destroyed
      slt(TMP2, rd, rs1);
      bne(TMP, TMP2, on_overflow);
    } else {
      add(rd, rs1, rs2);
      slti(TMP, rs2, 0);
      slt(TMP2, rd, rs1);
      bne(TMP, TMP2, on_overflow);
    }
  }

  void AddImmediateBranchOverflow(Register rd,
                                  Register rs1,
                                  intptr_t imm,
                                  Label* on_overflow) {
    ASSERT(rd != rs1);
    AddImmediate(rd, rs1, imm);
    if (imm > 0) {
      blt(rd, rs1, on_overflow);
    } else if (imm < 0) {
      bgt(rd, rs1, on_overflow);
    }
  }

  void SubImmediateBranchOverflow(Register rd,
                                  Register rs1,
                                  intptr_t imm,
                                  Label* on_overflow) {
    AddImmediateBranchOverflow(rd, rs1, -imm, on_overflow);
  }

  void SubBranchOverflow(Register rd,
                         Register rs1,
                         Register rs2,
                         Label* on_overflow) {
    ASSERT(rd != rs1);
    ASSERT(rd != rs2);
    ASSERT(rd != TMP);
    ASSERT(rd != TMP2);
    ASSERT(rs1 != TMP);
    ASSERT(rs1 != TMP2);
    ASSERT(rs2 != TMP);
    ASSERT(rs2 != TMP2);
    sub(rd, rs1, rs2);
    if (rs1 != rs2) {
      slti(TMP, rs1, 0);
      slt(TMP2, rd, rs1);
      bne(TMP, TMP2, on_overflow);
    }
  }

  void MulBranchOverflow(Register rd,
                         Register rs1,
                         Register rs2,
                         Label* on_overflow) {
    ASSERT(rd != rs1);
    ASSERT(rd != rs2);
    ASSERT(rd != TMP);
    ASSERT(rd != TMP2);
    ASSERT(rs1 != TMP);
    ASSERT(rs1 != TMP2);
    ASSERT(rs2 != TMP);
    ASSERT(rs2 != TMP2);

    // Macro-op fusion: when both products are needed, the recommended sequence
    // is mulh first.
    mulh(TMP, rs1, rs2);
    mul(rd, rs1, rs2);
    xor_(TMP, TMP, rd);
    blt(TMP, ZERO, on_overflow);

    // OR:
    srai(TMP2, rd, XLEN - 1);
    bne(TMP, TMP2, on_overflow);
  }

  void DivBranchOverflow(Register rd,
                         Register rs1,
                         Register rs2,
                         Label* on_overflow) {
    // If rs1 == -2^(XLEN-1) && rs2 == -1.
    Label done;
    div(rd, rs1, rs2);
    addi(TMP, rs2, 1);
    bne(TMP, ZERO, &done, kNearJump);
    addi(TMP, rs1, -1);
    beq(TMP, ZERO, on_overflow);
    Bind(&done);
  }

  void Call(Address addr) {
    lx(RA, addr);
    jalr(RA);
  }

  void AddPair(Register rd_hi,
               Register rd_lo,
               Register rs1_hi,
               Register rs1_lo,
               Register rs2_hi,
               Register rs2_lo) {
    ASSERT(rd_lo != rs1_lo);
    add(rd_lo, rs1_lo, rs2_lo);
    sltu(TMP, rs1_lo, rd_lo);  // Carry
    add(rd_hi, rs1_hi, rs2_hi);
    add(rd_hi, rd_hi, TMP);
  }
  void SubPair(Register rd_hi,
               Register rd_lo,
               Register rs1_hi,
               Register rs1_lo,
               Register rs2_hi,
               Register rs2_lo) {
    ASSERT(rd_lo != rs1_lo);
    sub(rd_lo, rs1_lo, rs2_lo);
    sltu(TMP, rs1_lo, rd_lo);  // Borrow
    sub(rd_hi, rs1_hi, rs2_hi);
    sub(rd_hi, rd_hi, TMP);
  }
  void MulPair(Register rd_hi,
               Register rd_lo,
               Register rs1_hi,
               Register rs1_lo,
               Register rs2_hi,
               Register rs2_lo) {
  }
  void ShiftLeftPair(Register rd_hi,
                     Register rd_lo,
                     Register rs_hi,
                     Register rs_lo,
                     intptr_t shift) {
    //rd_hi = rs_hi << shift;
    //| rs_lo >> (XLEN - shift);
    //rd_lo = rs_lo << shift;
  }

  void BranchAnyMask(Register rs, intptr_t mask, Label* label) {
    andi(TMP, rs, mask);
    bnez(TMP, label);
  }

  void BranchAllMask(Register rs, intptr_t mask, Label* label) {
    LoadImmediate(TMP, mask);
    and_(TMP2, rs, TMP);
    beq(TMP2, TMP, label);
  }
};


#if 0
constexpr Register THR = S1;
constexpr Register NIL = S2;
constexpr Register FALSE = S3;
constexpr Register TRUE = S4;
constexpr Register RCVR = S5;
constexpr Register ARG1 = S6;
constexpr Register ARG2 = S7;
constexpr Register TMP = T0;
constexpr Register PP = T1;
constexpr Register CODE = T2;  // At call
constexpr Register DATA = T3;  // At call

constexpr Register DISPATCH = T4;  // Interp global.
constexpr Register IP = T5;        // Interp global.
constexpr Register BYTE = T6;      // At call.

class JITAssembler : public MacroAssembler {
 public:
  void EnterFrame(intptr_t size) {
    addi(SP, SP, 4 * kWordSize + size);
    sw(RA, Address(SP, 3 * kWordSize + size));
    sw(FP, Address(SP, 2 * kWordSize + size));
    sw(CODE, Address(SP, 1 * kWordSize + size));
    sw(PP, Address(SP, 0 * kWordSize + size));
    addi(FP, SP, -(2 * kWordSize + size));
    lw(PP, Address(CODE, 0000));
  }

  void LeaveFrame(intptr_t stack_depth) {
    lw(RA, Address(FP, kWordSize));
    lw(PP, Address(FP, -kWordSize));
    lw(FP, Address(FP, 0));
    addi(SP, SP, stack_depth + 4 * kWordSize);
    ret();
  }

  void Prologue() {
    Label stack_overflow;
    lw(TMP, Address(THR, 0000));
    bltu(SP, TMP, &stack_overflow);
  }

  void Send(intptr_t num_args) {
    lw(RCVR, Address(SP, -kWordSize * num_args));
    lw(CODE, Address(PP, 0000));
    lw(RA, Address(CODE, 0000));
    lw(S9, Address(PP, 0000));
    jalr(RA);
    addi(SP, SP, -kWordSize * num_args);
  }

  void PopJumpTrue() {
    Label on_true, on_false, must_be_boolean;
    lw(TMP, Address(SP, 0));
    beq(TMP, TRUE, &on_true);
    beq(TMP, FALSE, &on_false);
    // <call must be boolean>
  }

  void StoreIntoObject(Register object, Register value) {
    intptr_t kObjectAlignmentLog2 = kWordSizeLog2 + 1;
    intptr_t kNewObjectAlignmentOffset = kWordSize;

    //      g h
    // old=x001
    // new=x101
    // smi=xxx0

    // ~(obj:2) & val:2 & val:0 & bit2

    Label done;
    slli(TMP, value, kObjectAlignmentLog2 - 1);
    and_(TMP, TMP, value);
    not_(TMP2, object);
    and_(TMP, TMP, TMP2);
    andi(TMP, TMP, kNewObjectAlignmentOffset);
    bnez(TMP, &done);

    Label done;
    slli(TMP, value, kObjectAlignmentLog2 - 1);
    and_(TMP, TMP, value);
    not_(TMP, TMP) or_(TMP, TMP, object);
    andi(TMP, TMP, kNewObjectAlignmentOffset);
    beqz(TMP, &done);

    /*
    andi(TMP, value, kSmiTag);
    bne(TMP, ZERO, &done);
    ldu(TMP, Address(value, Object::header_offset()));
    ldu(TMP2, Address(object, Object::header_offset()));
    slli(TMP, 2);
    and(TMP, TMP, TMP2);
    beq(ZERO, TMP, &done);
    */

    // < call stub >
    Bind(&done);
  }

  void BranchIfSmi(Register object, Label* label) {
    andi(TMP, object, 1);
    beqz(TMP, label);
  }
  void BranchIfNotSmi(Register object, Label* label) {
    andi(TMP, object, 1);
    bnez(TMP, label);
  }

  void LoadHeapClassId(Register cid, Register object) {
    lhu(cid, Address(object, 1));
  }

  void LoadClassId(Register cid, Register object) {
    intptr_t kSmiCid = 12;

    Label smi, done;
    BranchIfSmi(object, &smi);

    LoadHeapClassId(cid, object);
    j(&done);

    Bind(&smi);
    li(cid, kSmiCid);
    Bind(&done);
  }

  void MonomorphicCheck() {
    Label miss;

    LoadClassId(TMP, RCVR);
    bne(TMP, DATA, &miss);

    Bind(&miss);
  }

  void Intrinsic_Getter() {
    lw(RCVR, Address(RCVR, 0));
    ret();
  }

  void Intrinsic_Setter() {
    sw(ARG1, Address(RCVR, 0));
    StoreIntoObject(RCVR, ARG1);
    ret();
  }

  void Intrinsic_ArrayAt() {
    Label slow;

    BranchIfNotSmi(ARG1, &slow);

    lw(T2, Address(RCVR, 5));  // Size as Smi.
    intptr_t kSmiOne = 2;
    subi(T3, ARG1, kSmiOne);  // 0-origin index
    bgeu(T3, T2, &slow);

    slli(T4, ARG1, 1);  // index scaled to elements
    add(T5, T4, RCVR);
    lw(RCVR, Address(T5, 1));
    ret();

    Bind(&slow);
  }

  void Return(Register result) {
    Move(A0, result);  // Likely no-op.
    lw(RA, Address(SP, 0));
    lw(PP, Address(SP, 0));
    lw(S0, Address(SP, 0));
    lw(S1, Address(SP, 0));
    lw(S2, Address(SP, 0));
    lw(S3, Address(SP, 0));
    addi(SP, SP, 0);
    ret();
  }

  void NonLocalReturn() {
    // Move(A0, result);  // Likely no-op.
    lw(T0, Address(THR, 0));
    jr(T0);  // Maybe not a tail so the NLR stub can look at metadata for this
             // method?
  }

  struct Thread {
    uword nil_;
    uword false_;
    uword true_;
    uword scheduler_;

    uword non_local_return_;
    uword base_return_;
    uword stack_overflow_;
    uword monomorphic_miss_;
    uword must_be_boolean_;

    uword ordinary_send_;
    uword self_send_;
    uword super_send_;
    uword outer_send_;
    uword implicit_send_;
    uword enclosing_object_;
    uword create_array_;
    uword create_closure_;

    uword bytecode_dispatch_table_;
  };

  Register table;
  Register ip;
  Register byte;

  void Dispatch() {
    lw(byte, Address(ip, 0));
    addi(byte, byte, 1);
    slli(T0, byte, kWordSizeLog2);  // T0 = offset
    add(T0, table, T0);             // T0 = address
    lw(T0, Address(T0, 0));         // T0 = bytecode entry
    jr(T0);
  }
  void InterpretPushTemp() {
    subi(T0, byte, 32);           // T0 = index
    slli(T0, T0, kWordSizeLog2);  // T0 = offset
    lw(T1, Address(FP, 0));       // T1 = method
    lw(T1, Address(T1, 0));       // T1 = literals
    add(T1, T1, T0);              // T1 = address
    lw(T1, Address(T1, 0));       // T1 = literal
    Push(T1);
    Dispatch();
  }

  void SendLookup() {
    Label miss;

    LoadClassId(T0, RCVR);   // T0 <- receiver cid
    lw(T1, Address(FP, 0));  // T1 <- frame method
    lw(T1, Address(T1, 0));  // T1 <- literals
    addi(T1, T1, 0);         // T1 <- addr
    lw(T1, Address(T1, 0));  // T1 <- selector

    srai(T2, T1, kWordSizeLog2);
    xor_(T2, T2, T0);  // T2 <- hash

    andi(T3, T3, 511);            // T3 <- probe1
    slli(T3, T3, 4 * kWordSize);  // T3 <- cache offset
    add(T3, T3, T6);              // T3 <- cache entry addr
    lw(T4, Address(T2, 0));       // T4 <- cache cid
    lw(T5, Address(T2, 4));       // T5 <- cache sel
    bne(T4, T0, &miss);
    bne(T5, T1, &miss);

    lw(T0, Address(T2, 8));  // T0 <- cache method
    // build frame, jump
  }
};
#endif

}  // namespace psoup

#endif  // VM_MACROASSEMBLER_RISCV_H_
