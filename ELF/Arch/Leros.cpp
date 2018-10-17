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
    return R_PC;
  default:
    return R_ABS;
  }
}

// Extract bits V[Begin:End], where range is inclusive, and Begin must be < 63.
static uint32_t extractBits(uint64_t V, uint32_t Begin, uint32_t End) {
  return (V & ((1ULL << (Begin + 1)) - 1)) >> End;
}

void Leros::relocateOne(uint8_t *Loc, const RelType Type,
                        const uint64_t Val) const {
  checkInt(Loc, static_cast<int64_t>(Val), 8, Type);
  checkAlignment(Loc, Val, 2, Type);
  uint16_t Insn = read16le(Loc) & 0xFF00;

  switch (Type) {
  case R_LEROS_BYTE0: {
    Insn |= extractBits(Val, 0, 7);
    break;
  }
  case R_LEROS_BYTE1: {
    Insn |= extractBits(Val, 8, 15);
    break;
  }
  case R_LEROS_BYTE2: {
    Insn |= extractBits(Val, 16, 23);
    break;
  }
  case R_LEROS_BYTE3: {
    Insn |= extractBits(Val, 24, 31);
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
