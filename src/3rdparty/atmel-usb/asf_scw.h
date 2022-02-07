#ifndef ASF_SCW_H
#define ASF_SCW_H

#include <assert.h>
#include <stdbool.h>

typedef uint16_t	le16_t;
typedef uint32_t	le32_t;
typedef uint32_t	iram_size_t;

enum status_code {
	STATUS_OK = 0,
	STATUS_ERR_DENIED
};

#define	COMPILER_PRAGMA(arg)		_Pragma(#arg)
#define	COMPILER_PACK_SET(alignment)	COMPILER_PRAGMA(pack(alignment))
#define	COMPILER_PACK_RESET()		COMPILER_PRAGMA(pack())

#define	PACKED				__attribute__((packed))
#define	ALIGNED(a)			__attribute__((aligned(a)))
#define	PACKED_ALIGNED(a)i		__attribute__((packed,aligned(a)))

#define	Assert		assert
#define	clz(u)		((u) ? __builtin_ctz(u) : 32)
#define	ctz(u)		__builtin_ctz(u)
#define	Min(a, b)	(((a) < (b)) ? (a) : (b))
#define	Max(a, b)	(((a) > (b)) ? (a) : (b))

#define	LE16(x)		(x)
#define	CPU_TO_LE16(x)	(x)
#define	cpu_to_le16(x)	(x)
#define	le16_to_cpu(x)	(x)
#define	LE32(x)		(x)
#define	CPU_TO_LE32(x)	(x)

#define UNUSED(v)	(void)(v)

#define	Rd_bits(value, mask)		((value) & (mask))
#define	Wr_bits(lvalue, mask, bits)	((lvalue) = ((lvalue) & ~(mask)) |\
					 ((bits  ) &  (mask)))
#define	Clr_bits(lvalue, mask)		((lvalue) &= ~(mask))
#define	Set_bits(lvalue, mask)		((lvalue) |= (mask))
#define	Tst_bits( value, mask)		(Rd_bits(value, mask) != 0)
#define	Rd_bitfield( value, mask)	(Rd_bits( value, mask) >> ctz(mask))
#define	Wr_bitfield(lvalue, mask, bitfield) (Wr_bits(lvalue, mask, (uint32_t)(bitfield) << ctz(mask)))

#endif /* ASF_SCW */
