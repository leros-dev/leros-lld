//===- Leros.cpp ----------------------------------------------------------===//
//
//                             The LLVM Linker
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "InputFiles.h"
#include "Target.h"

using namespace llvm;
using namespace llvm::object;
using namespace llvm::support::endian;
using namespace llvm::ELF;
using namespace lld;
using namespace lld::elf;

namespace {

class Leros final : public TargetInfo {
public:
  Leros();
  virtual uint32_t calcEFlags() const override;
  RelExpr getRelExpr(RelType Type, const Symbol &S,
                     const uint8_t *Loc) const override;
  void relocateOne(uint8_t *Loc, RelType Type, uint64_t Val) const override;
};

} // end anonymous namespace

Leros::Leros() { NoneRel = R_LEROS_NONE; }

static uint32_t getEFlags(InputFile *F) {
  if (Config->Is64)
    return cast<ObjFile<ELF64LE>>(F)->getObj().getHeader()->e_flags;
  return cast<ObjFile<ELF32LE>>(F)->getObj().getHeader()->e_flags;
}

uint32_t Leros::calcEFlags() const {
  assert(!ObjectFiles.empty());
  uint32_t Target = getEFlags(ObjectFiles.front());
  return Target;
}

RelExpr Leros::getRelExpr(const RelType Type, const Symbol &S,
                          const uint8_t *Loc) const {
  switch (Type) {
  case R_LEROS_BYTE0:
  case R_LEROS_BYTE1:
  case R_LEROS_BYTE2:
  case R_LEROS_BYTE3:
    return R_ABS;
  case R_LEROS_BRANCH:
    return R_PC;
  default:
    return R_ABS;
  }
}

void Leros::relocateOne(uint8_t *Loc, const RelType Type,
                        const uint64_t Val) const {
  uint16_t Insn = read16le(Loc) & 0xFF00;

  // Todo: Check alignment for byte relocations, if the opcode is related to
  // control flow

  switch (Type) {
  case R_LEROS_32:
    write32le(Loc, Val);
    return;
  case R_LEROS_64:
    write64le(Loc, Val);
    return;
  case R_LEROS_BYTE0: {
    checkInt(Loc, static_cast<int64_t>(Val), 32, Type);
    Insn |= Val & 0xFF;
    break;
  }
  case R_LEROS_BYTE1: {
    checkInt(Loc, static_cast<int64_t>(Val), 32, Type);
    Insn |= (Val >> 8) & 0xFF;
    break;
  }
  case R_LEROS_BYTE2: {
    checkInt(Loc, static_cast<int64_t>(Val), 32, Type);
    Insn |= (Val >> 16) & 0xFF;
    break;
  }
  case R_LEROS_BYTE3: {
    checkInt(Loc, static_cast<int64_t>(Val), 32, Type);
    Insn |= (Val >> 24) & 0xFF;
    break;
  }
  case R_LEROS_BRANCH: {
    // Verify that it is representable as a 13-bit immediate,
    // with lsb = 0
    checkInt(Loc, static_cast<int64_t>(Val), 13, Type);
    checkAlignment(Loc, Val, 2, Type);
    Insn |= (Val >> 1) & 0xfff;
    break;
  }
  default:
    error(getErrorLocation(Loc) +
          "unimplemented relocation: " + toString(Type));
    return;
  }
  write16le(Loc, Insn);
}

TargetInfo *elf::getLerosTargetInfo() {
  static Leros Target;
  return &Target;
}
