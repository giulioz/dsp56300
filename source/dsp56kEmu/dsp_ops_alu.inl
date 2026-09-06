#pragma once

#include "dsp.h"
#include "registers.h"

namespace dsp56k
{
	constexpr int64_t g_alu_max_56		=  0x7FFFFFFFFFFFFF;
	constexpr int64_t g_alu_min_56		= -0x80000000000000;
	constexpr uint64_t g_alu_max_56_u	=  0xffffffffffffff;

	// _____________________________________________________________________________
	// alu_and
	//
	void DSP::alu_and( bool ab, TWord _val )
	{
		TReg56& d = ab ? reg.b : reg.a;

		d.var &= (TInt64(isSixteenBitArithmetic() ? _val & 0xffff00 : _val)<<(24 + g_aluShift)) | static_cast<TInt64>(0xFF000000FFFFFF00ull);

		// S L E U N Z V C
		// v - - - * * * -
		sr_toggle( CCR_N, bittest( d, 47 + g_aluShift ) );
		sr_toggle( CCR_Z, (d.var & (0xffffff000000ull << g_aluShift)) == 0 );
		sr_clear( CCR_V );
	}


	// _____________________________________________________________________________
	// alu_or
	//
	void DSP::alu_or( bool ab, TWord _val )
	{
		TReg56& d = ab ? reg.b : reg.a;

		d.var |= (TInt64(_val)<<(24 + g_aluShift));
		if(isSixteenBitArithmetic()) d.var &= ~(uint64_t(0xff) << (24 + g_aluShift));

		// S L E U N Z V C
		// v - - - * * * -
		sr_toggle( CCR_N, bittest( d, 47 + g_aluShift ) );
		sr_toggle( CCR_Z, (d.var & (0xffffff000000ull << g_aluShift)) == 0 );
		sr_clear( CCR_V );
	}

	// _____________________________________________________________________________
	// alu_eor
	//
	void DSP::alu_eor( bool ab, TWord _val )
	{
		TReg56& d = ab ? reg.b : reg.a;

		d.var ^= (TInt64(_val)<<(24 + g_aluShift));
		if(isSixteenBitArithmetic()) d.var &= ~(uint64_t(0xff) << (24 + g_aluShift));

		// S L E U N Z V C
		// v - - - * * * -
		sr_toggle( CCR_N, bittest( d, 47 + g_aluShift ) );
		sr_toggle( CCR_Z, (d.var & (0xffffff000000ull << g_aluShift)) == 0 );
		sr_clear( CCR_V );
	}

	// _____________________________________________________________________________
	// alu_add
	//
	void DSP::alu_add( bool ab, const TReg56& _val )
	{
		TReg56& d = ab ? reg.b : reg.a;

		const uint64_t d64 = arithmeticValue(d);
		const uint64_t value = arithmeticValue(_val);
		const uint64_t res = d64 + value;
		const bool overflow = (~(d64 ^ value) & (d64 ^ res)) >> 63;

		arithmeticResult(d, res);

		const auto carry = int(res < d64);	// carry out of the accumulator = 64-bit unsigned overflow

		// S L E U N Z V C

		sr_toggle(CCRB_C, Bit(carry));
		sr_toggle(CCR_V, overflow);
		limit_arithmeticSaturation(d);
		sr_z_update(d);
		sr_l_update_by_v();

//		sr_s_update();
//		sr_e_update(d);
//		sr_u_update(d);
//		sr_n_update(d);

		setCCRDirty(ab, d, CCR_E | CCR_U | CCR_N);

	//	dumpCCCC();
	}

	// _____________________________________________________________________________
	// alu_cmp
	//
	void DSP::alu_cmp( bool ab, const TReg56& _val, bool _magnitude )
	{
		TReg56& d = ab ? reg.b : reg.a;

		const TReg56 oldD = d;

		uint64_t d64 = arithmeticValue(d);

		uint64_t val = arithmeticValue(_val);

		if( _magnitude )
		{
			const auto d64Signed = static_cast<int64_t>(arithmeticValue(d));
			if(d64Signed < 0)
				d64 = uint64_t(0) - d64;

			const auto valSigned = static_cast<int64_t>(arithmeticValue(_val));
			if(valSigned < 0)
				val = uint64_t(0) - val;
		}

		const auto res = static_cast<uint64_t>(d64) - static_cast<uint64_t>(val);

		const auto carry = val > d64;	// borrow out of the accumulator

		arithmeticResult(d, res);

		sr_toggle(CCR_V, ((d64 ^ val) & (d64 ^ res)) >> 63);
		limit_arithmeticSaturation(d);
		sr_z_update(d);
		sr_l_update_by_v();
		sr_toggle(CCR_C, carry);

		setCCRDirty(ab, d, CCR_E | CCR_U | CCR_N);

		d = oldD;
	}
	// _____________________________________________________________________________
	// alu_cmpu
	//
	void DSP::alu_cmpu( bool ab, const TReg56& _val )
	{
		// CMPU compares two 48 bit UNSIGNED operands. The accumulator extension takes no part in the
		// operation, and a 24 bit source has already been left-aligned and zero-filled by the decoder.
		// E and U are unchanged; writing N below cancels only its pending cache entry.

		const TReg56& d = ab ? reg.b : reg.a;

		constexpr uint64_t mask48 = 0x0000ffffffffffffull;

		const uint64_t s2 = (static_cast<uint64_t>(d.var)    >> g_aluShift) & mask48;
		const uint64_t s1 = (static_cast<uint64_t>(_val.var) >> g_aluShift) & mask48;

		const uint64_t res = (s2 - s1) & mask48;

		sr_toggle( CCR_Z, res == 0 );
		sr_toggle( CCRB_N, Bit((res >> 47) & 1) );
		sr_clear ( CCR_V );			// "always cleared"
		sr_toggle( CCR_C, s1 > s2 );	// borrow out of bit 47
	}

	// _____________________________________________________________________________
	// alu_sub
	//
	void DSP::alu_sub( bool ab, const TReg56& _val )
	{
		TReg56& d = ab ? reg.b : reg.a;

		const uint64_t d64 = arithmeticValue(d);
		const uint64_t value = arithmeticValue(_val);
		const uint64_t res = d64 - value;

		const auto carry = value > d64;	// borrow out of the accumulator

		arithmeticResult(d, res);

		// S L E U N Z V C
		sr_toggle(CCR_C, carry);
		sr_toggle(CCR_V, ((d64 ^ value) & (d64 ^ res)) >> 63);

		limit_arithmeticSaturation(d);
		sr_z_update(d);
		sr_l_update_by_v();
		setCCRDirty(ab, d, CCR_E | CCR_U | CCR_N);
	}

	// _____________________________________________________________________________
	// alu_asr
	//
	void DSP::alu_asr( bool abDst, bool abSrc, int _shiftAmount )
	{
		const TReg56& dSrc = abSrc ? reg.b : reg.a;

		const TInt64 d64 = static_cast<int64_t>(arithmeticValue(dSrc));

		const bool carry = isSixteenBitArithmetic() && _shiftAmount >= 40
			? d64 < 0 : _shiftAmount && bittest(d64, _shiftAmount - 1 + (isSixteenBitArithmetic() ? 24 : g_aluShift));
		sr_toggle(CCR_C, carry);

		const TInt64 res = d64 >> _shiftAmount;

		TReg56& d = abDst ? reg.b : reg.a;

		arithmeticResult(d, res);	// discards the bits shifted below the accumulator

		// S L E U N Z V C

		sr_z_update(d);
		sr_clear(CCR_V);
		//sr_l_update_by_v();
		setCCRDirty(abDst, d, CCR_E | CCR_U | CCR_N);
	}

	// _____________________________________________________________________________
	// alu_asl
	//
	void DSP::alu_asl( bool abDst, bool abSrc, int _shiftAmount )
	{
		const TReg56& dSrc = abSrc ? reg.b : reg.a;

		const TInt64 d64 = static_cast<int64_t>(arithmeticValue(dSrc));

		sr_toggle( CCR_C, _shiftAmount && ((d64 & (TInt64(1)<<(56 + g_aluShift - _shiftAmount))) != 0) );

		const TInt64 res = d64 << _shiftAmount;

		TReg56& d = abDst ? reg.b : reg.a;

		arithmeticResult(d, res);

		// Overflow: Set if Bit 55 is changed any time during the shift operation, cleared otherwise.
		// What that means for us is that all bits that are shifted out need to be identical to not overflow
		int64_t overflowMaskI = 0x8000000000000000;
		overflowMaskI >>= _shiftAmount;
		uint64_t overflowMaskU = overflowMaskI;
		if constexpr (g_aluShift == 0)
			overflowMaskU >>= 8;	// right-aligned the window has to come down to bit 55
		const uint64_t v = d64 & overflowMaskU;
		const bool isOverflow = v != overflowMaskU && v != 0;

		// S L E U N Z V C
		sr_z_update(d);
		sr_toggle(CCR_V, isOverflow);
		sr_l_update_by_v();
		setCCRDirty(abDst, d, CCR_E | CCR_U | CCR_N);
	}

	// _____________________________________________________________________________
	// alu_lsl
	//
	void DSP::alu_lsl(bool ab, int _shiftAmount)
	{
		const unsigned pad = isSixteenBitArithmetic() ? 8 : 0, width = 24 - pad;
		const uint32_t value = (ab ? b1() : a1()).toWord() >> pad;
		const uint32_t mask = (1u << width) - 1;
		const bool carry = _shiftAmount > 0 && unsigned(_shiftAmount) <= width && ((value >> (width - _shiftAmount)) & 1);
		const auto result = _shiftAmount >= int(width) ? 0 : (value << _shiftAmount) & mask;
		if(ab) b1(TReg24(result << pad)); else a1(TReg24(result << pad));
		sr_toggle(CCR_C, carry);
		sr_toggle(CCR_N, (result & (1u << (width - 1))) != 0);
		sr_toggle(CCR_Z, result == 0);
		sr_clear(CCR_V);
	}

	void DSP::alu_lsr(bool ab, int _shiftAmount)
	{
		const unsigned pad = isSixteenBitArithmetic() ? 8 : 0, width = 24 - pad;
		const uint32_t value = (ab ? b1() : a1()).toWord() >> pad;
		const bool carry = _shiftAmount > 0 && unsigned(_shiftAmount) <= width && ((value >> (_shiftAmount - 1)) & 1);
		const auto result = _shiftAmount >= int(width) ? 0 : value >> _shiftAmount;
		if(ab) b1(TReg24(result << pad)); else a1(TReg24(result << pad));
		sr_toggle(CCR_C, carry);
		sr_toggle(CCR_N, (result & (1u << (width - 1))) != 0);
		sr_toggle(CCR_Z, result == 0);
		sr_clear(CCR_V);
	}

	void DSP::alu_shiftedArithmetic(bool ab, bool _left, bool _subtract)
	{
		TReg56& d = ab ? reg.b : reg.a;
		const uint64_t original = arithmeticValue(d);
		const uint64_t source = arithmeticValue(ab ? reg.a : reg.b);
		const uint64_t shifted = _left ? original << 1 :
			(static_cast<uint64_t>(static_cast<int64_t>(original) >> 1) &
				(~uint64_t(0) << (isSixteenBitArithmetic() ? 24 : g_aluShift)));
		const uint64_t result = _subtract ? shifted - source : shifted + source;
		const bool shiftOverflow = _left && ((original ^ shifted) >> 63);
		const bool overflow = shiftOverflow ||
			(((_subtract ? shifted ^ source : ~(shifted ^ source)) & (shifted ^ result)) >> 63);
		arithmeticResult(d, result);
		sr_toggle(CCR_C, _subtract ? source > shifted : result < shifted);
		sr_toggle(CCR_V, overflow);
		limit_arithmeticSaturation(d);
		sr_z_update(d);
		sr_l_update_by_v();
		setCCRDirty(ab, d, CCR_E | CCR_U | CCR_N);
	}

	void DSP::alu_addl(bool ab)
	{
		alu_shiftedArithmetic(ab, true, false);
	}

	void DSP::alu_addr(bool ab)
	{
		alu_shiftedArithmetic(ab, false, false);
	}

	void DSP::alu_rotate(const bool ab, const bool _right)
	{
		const unsigned pad = isSixteenBitArithmetic() ? 8 : 0, width = 24 - pad;
		const uint32_t value = (ab ? b1() : a1()).toWord() >> pad;
		const uint32_t carry = sr_val(CCRB_C);
		const uint32_t result = (_right ? (value >> 1) | (carry << (width - 1)) : (value << 1) | carry) & ((1u << width) - 1);
		if(ab) b1(TReg24(result << pad)); else a1(TReg24(result << pad));
		sr_toggle(CCR_N, (result & (1u << (width - 1))) != 0);
		sr_toggle(CCR_Z, result == 0);
		sr_clear(CCR_V);
		sr_toggle(CCR_C, ((value >> (_right ? 0 : width - 1)) & 1) != 0);
	}

	void DSP::alu_clr(bool ab)
	{
		TReg56& dst = ab ? reg.b : reg.a;
		dst.var = 0;

		sr_clear( static_cast<CCRMask>(CCR_E | CCR_N | CCR_V) );
		sr_set( static_cast<CCRMask>(CCR_U | CCR_Z) );
		// TODO: SR_L and SR_S are changed according to standard definition, but that should mean that no update is required?!
	}

	// _____________________________________________________________________________
	// alu_bclr
	//
	TWord DSP::alu_bclr( TWord _bit, TWord _val )
	{
		sr_toggle( CCR_C, bittest(_val,_bit) );

		_val &= ~(1<<_bit);

		return _val;
	}

	// _____________________________________________________________________________
	// alu_mpy
	//
	void DSP::alu_mpy( bool ab, const TReg24& _s1, const TReg24& _s2, bool _negate, bool _accumulate, bool _round )
	{
	//	assert( sr_test(SR_S0) == 0 && sr_test(SR_S1) == 0 );

		const int64_t s1 = (isSixteenBitArithmetic() ? (_s1.signextend<int64_t>() & ~255ll) : _s1.signextend<int64_t>());
		const int64_t s2 = (isSixteenBitArithmetic() ? (_s2.signextend<int64_t>() & ~255ll) : _s2.signextend<int64_t>());

		uint64_t res = static_cast<uint64_t>(s1 * s2);

		// fractional multiplication requires one post-shift; the same shift scales the product
		// into the left-aligned ALU domain before it meets the accumulator
		res <<= (1 + g_aluShift);

		if( _negate )
			res = uint64_t(0) - res;

		TReg56& d = ab ? reg.b : reg.a;

		if( _accumulate )
			res += arithmeticValue(d);

		arithmeticResult(d, res);

		// Rounded products must not saturate or set L at an intermediate stage.
		if(_round) { alu_rnd(ab); return; }
		sr_v_update(res,d);
		limit_arithmeticSaturation(d);
		sr_z_update(d);

		sr_l_update_by_v();

//		sr_s_update();
//		sr_e_update(d);
//		sr_u_update(d);
//		sr_n_update(d);

		setCCRDirty(ab, d, CCR_E | CCR_U | CCR_N);
	}
	// _____________________________________________________________________________
	// alu_mpysuuu
	//
	void DSP::alu_mpysuuu( bool ab, TReg24 _s1, TReg24 _s2, bool _negate, bool _accumulate, bool _suuu )
	{
	//	assert( sr_test(SR_S0) == 0 && sr_test(SR_S1) == 0 );

		if(isSixteenBitArithmetic()) { _s1.var &= ~255; _s2.var &= ~255; }

		TInt64 res;

		if( _suuu )
			res = TInt64( TUInt64(_s1.var) * TUInt64(_s2.var) );
		else
			res = _s1.signextend<TInt64>() * TUInt64(_s2.var);

		// fractional multiplication requires one post-shift; the same shift scales the product
		// into the left-aligned ALU domain before it meets the accumulator
		res <<= (1 + g_aluShift);

		if( _negate )
			res = -res;

		TReg56& d = ab ? reg.b : reg.a;

		const TReg56 old = d;

		if( _accumulate )
			res += static_cast<int64_t>(arithmeticValue(d));

		arithmeticResult(d, res);

		// Update SR
		sr_z_update( d );
		sr_v_update(res,d);

		sr_l_update_by_v();
		setCCRDirty(ab, d, CCR_E | CCR_U | CCR_N);
	}
	// _____________________________________________________________________________
	// alu_dmac
	//
	void DSP::alu_dmac( bool ab, TReg24 _s1, TReg24 _s2, bool _negate, bool srcUnsigned, bool dstUnsigned )
	{
		assert( sr_test(SR_S0) == 0 && sr_test(SR_S1) == 0 );

		TInt64 res;

		if( srcUnsigned && dstUnsigned )	res = TInt64( TUInt64(_s1.var) * TUInt64(_s2.var) );
		else if( srcUnsigned )				res = TUInt64(_s1.var) * _s2.signextend<TInt64>();
		else if( dstUnsigned )				res = TUInt64(_s2.var) * _s1.signextend<TInt64>();
		else								res = _s2.signextend<TInt64>() * _s1.signextend<TInt64>();

		// fractional multiplication requires one post-shift; the same shift scales the product
		// into the left-aligned ALU domain before it meets the accumulator
		res <<= (1 + g_aluShift);

		if( _negate )
			res = -res;

		TReg56& d = ab ? reg.b : reg.a;

		const TReg56 old = d;

		TInt64 dShifted = aluSignextend(d) >> 24;

		res += dShifted;

	//	LOG( "DMAC  " << std::hex << old.var << std::hex << " + " << _s1.var << " * " << std::hex << _s2.var << " = " << std::hex << res );

		d.var = res;
		aluMask(d);

		// Update SR
		sr_z_update( d );
		sr_v_update(res,d);

		sr_l_update_by_v();
		setCCRDirty(ab, d, CCR_E | CCR_U | CCR_N);
	}

	// _____________________________________________________________________________
	// alu_mac
	//
	void DSP::alu_mac( bool ab, TReg24 _s1, TReg24 _s2, bool _negate, bool _uu )
	{
		assert( sr_test(SR_S0) == 0 && sr_test(SR_S1) == 0 );

		if(isSixteenBitArithmetic()) { _s1.var &= ~255; _s2.var &= ~255; }

		TInt64 res;

		if( _uu )
			res = TInt64( TUInt64(_s1.var) * TUInt64(_s2.var) );
		else
			res = _s1.signextend<TInt64>() * TUInt64(_s2.var);

		// fractional multiplication requires one post-shift; the same shift scales the product
		// into the left-aligned ALU domain before it meets the accumulator
		res <<= (1 + g_aluShift);

		if( _negate )
			res = -res;

		TReg56& d = ab ? reg.b : reg.a;

		res += arithmeticValue(d);

		const TReg56 old = d;

		arithmeticResult(d, res);

		// Update SR
		sr_z_update( d );
		sr_v_update(res,d);

		sr_l_update_by_v();
		setCCRDirty(ab, d, CCR_E | CCR_U | CCR_N);
	}

	// _____________________________________________________________________________
	// alu_rnd
	//
	void DSP::alu_rnd(bool ab)
	{
		TReg56& _alu = ab ? reg.b : reg.a;

		auto value = arithmeticValue(_alu);
		int64_t rounder = 0x800000ll << (g_aluShift + (isSixteenBitArithmetic() ? 8 : 0));	// the rounding position moves up with the ALU

		if		(sr_test_noCache(SR_S1)) rounder>>=1;
		else if	(sr_test_noCache(SR_S0)) rounder<<=1;
		
		value += rounder;

		const auto mask = (rounder<<1)-1;		// all the bits to the right of, and including the rounding position

		if (!sr_test_noCache(SR_RM))			// convergent rounding. If all mask bits are cleared
		{
			if (!(value & mask))
				value&=~(rounder<<1);		// then the bit to the left of the rounding position is cleared in the result
		}

		value&=~mask;						// all bits to the right of and including the rounding position are cleared.

		const auto res = value;

		arithmeticResult(_alu, value);

		sr_v_update(res, _alu);
		limit_arithmeticSaturation(_alu, true);
		sr_z_update(_alu);

		sr_l_update_by_v();
		setCCRDirty(ab, _alu, CCR_E | CCR_U | CCR_N);
	}
	
	inline bool DSP::alu_multiply(const TWord _op)
	{
		const auto round = _op & 0x1;
		const auto mulAcc = (_op>>1) & 0x1;
		const auto negative = (_op>>2) & 0x1;
		const auto ab = (_op>>3) & 0x1;
		const auto qqq = (_op>>4) & 0x7;

		TReg24 s1, s2;

		decode_QQQ_read(s1, s2, qqq);

		alu_mpy(ab, s1, s2, negative, mulAcc, round);

		return true;
	}

	// __________________
	//

	inline void DSP::op_Abs(const TWord op)
	{
		const auto D = getFieldValue<Abs, Field_d>(op);
		alu_abs(D);
	}

	template<TWord ab> void DSP::opCE_Abs(const TWord op)
	{
		alu_abs(ab);
	}

	inline void DSP::op_ADC(const TWord op)
	{
		errNotImplemented("ADC");
	}
	inline void DSP::op_Add_SD(const TWord op)
	{
		const auto D = getFieldValue<Add_SD, Field_d>(op);
		const auto JJJ = getFieldValue<Add_SD, Field_JJJ>(op);
		alu_add(D, decode_JJJ_read_56(JJJ, !D));
	}
	inline void DSP::op_Add_xx(const TWord op)
	{
		const auto iiiiii	= getFieldValue<Add_xx,Field_iiiiii>(op);
		const auto ab		= getFieldValue<Add_xx,Field_d>(op);

		alu_add( ab, toAluOperand(iiiiii) );
	}
	inline void DSP::op_Add_xxxx(const TWord op)
	{
		const auto ab = getFieldValue<Add_xxxx,Field_d>(op);

		const TReg56 r56 = toAluOperand(TReg24(immediateDataExt<Add_xxxx>()));

		alu_add( ab, r56 );
	}
	inline void DSP::op_Addl(const TWord op)
	{
		alu_addl(getFieldValue<Addl, Field_d>(op));
	}
	inline void DSP::op_Addr(const TWord op)
	{		
		alu_addr(getFieldValue<Addr, Field_d>(op));
	}
	inline void DSP::op_And_SD(const TWord op)
	{
		const auto D = getFieldValue<And_SD, Field_d>(op);
		const auto JJ = getFieldValue<And_SD, Field_JJ>(op);
		alu_and(D, decode_JJ_read(JJ).var);
	}
	template<TWord D, TWord JJ>
	void DSP::opCE_And_SD(const TWord op)
	{
		alu_and(D ? true : false, decode_JJ_read(JJ).var);
	}
	inline void DSP::op_And_xx(const TWord op)
	{
		const auto ab		= getFieldValue<And_xx,Field_d>(op);
		const TWord xxxx	= getFieldValue<And_xx,Field_iiiiii>(op);

		alu_and(ab, xxxx );
	}
	inline void DSP::op_And_xxxx(const TWord op)
	{
		const auto ab = getFieldValue<And_xxxx,Field_d>(op);
		const TWord xxxx = immediateDataExt<And_xxxx>();

		alu_and( ab, xxxx );		
	}
	inline void DSP::op_Andi(const TWord op)
	{
		const TWord ee		= getFieldValue<Andi,Field_EE>(op);
		const TWord iiiiii	= getFieldValue<Andi,Field_iiiiiiii>(op);

		TReg8 val = decode_EE_read(ee);
		val.var &= iiiiii;
		decode_EE_write(ee,val);			
	}
	inline void DSP::op_Asl_D(const TWord op)
	{
		const auto D = getFieldValue<Asl_D, Field_d>(op);
		alu_asl(D, D, 1);
	}
	template<TWord D> void DSP::opCE_Asl_D(const TWord op)
	{
		alu_asl(D, D, 1);
	}
	inline void DSP::op_Asl_ii(const TWord op)
	{
		const TWord shiftAmount	= getFieldValue<Asl_ii,Field_iiiiii>(op);

		const bool abDst		= getFieldValue<Asl_ii,Field_D>(op);
		const bool abSrc		= getFieldValue<Asl_ii,Field_S>(op);

		alu_asl( abDst, abSrc, shiftAmount );					
	}
	inline void DSP::op_Asl_S1S2D(const TWord op)
	{
		const TWord sss = getFieldValue<Asl_S1S2D,Field_sss>(op);
		const bool abDst = getFieldValue<Asl_S1S2D,Field_D>(op);
		const bool abSrc = getFieldValue<Asl_S1S2D,Field_S>(op);

		const TWord shiftAmount = decode_sss_read<TWord>( sss ) & 0x3f;

		alu_asl( abDst, abSrc, shiftAmount );
	}
	inline void DSP::op_Asr_D(const TWord op)
	{
		const auto D = getFieldValue<Asr_D, Field_d>(op);
		alu_asr(D, D, 1);
	}
	inline void DSP::op_Asr_ii(const TWord op)
	{		
		const TWord shiftAmount	= getFieldValue<Asr_ii,Field_iiiiii>(op);

		const bool abDst		= getFieldValue<Asr_ii,Field_D>(op);
		const bool abSrc		= getFieldValue<Asr_ii,Field_S>(op);

		alu_asr( abDst, abSrc, shiftAmount );
	}
	inline void DSP::op_Asr_S1S2D(const TWord op)
	{
		const TWord sss = getFieldValue<Asr_S1S2D,Field_sss>(op);
		const bool abDst = getFieldValue<Asr_S1S2D,Field_D>(op);
		const bool abSrc = getFieldValue<Asr_S1S2D,Field_S>(op);

		const auto shiftAmount = decode_sss_read<TWord>( sss ) & 0x3f;

		alu_asr( abDst, abSrc, shiftAmount );			
	}
	inline void DSP::op_Clb(const TWord op)
	{
		const auto S = getFieldValue<Clb, Field_S>(op);
		const auto D = getFieldValue<Clb, Field_D>(op);

		const TReg56& s = S ? reg.b : reg.a;
		TReg56& d = D ? reg.b : reg.a;

		int count;

		if(s.var == 0)
		{
			// Special case: source is 0, result is 0
			count = 0;
		}
		else
		{
			// bit 55 has to sit at the MSB of the 64-bit value; left-aligned it already does
			const auto shifted = static_cast<int64_t>(s.var << (8 - g_aluShift));

			// If MSB is 1, invert to count leading ones as leading zeros
			auto val = static_cast<uint64_t>(shifted < 0 ? ~shifted : shifted);

			// Ensure we get a valid BSR result by setting low byte
			val |= 0xff;

			// Find highest set bit (equivalent to BSR on x86)
			int bsr = 0;
			for(int bit = 63; bit >= 0; --bit)
			{
				if(val & (static_cast<uint64_t>(1) << bit))
				{
					bsr = bit;
					break;
				}
			}

			count = bsr - (64 - 9 - 1);  // range: -47 to +8
		}

		d.var = static_cast<TInt64>(count) << (24 + g_aluShift);
		aluMask(d);

		// N: Set if bit 47 (= bit 23 of the 24-bit result) is set
		sr_toggle(CCR_N, bittest(d, 47 + g_aluShift));
		// Z: Set if result is zero
		sr_toggle(CCR_Z, count == 0);
		// V: Always cleared
		sr_clear(CCR_V);
	}
	inline void DSP::op_Clr(const TWord op)
	{
		const auto D = getFieldValue<Clr, Field_d>(op);
		alu_clr(D);
	}
	inline void DSP::op_Cmp_S1S2(const TWord op)
	{
		const auto D = getFieldValue<Cmp_S1S2, Field_d>(op);
		const auto JJJ = getFieldValue<Cmp_S1S2, Field_JJJ>(op);
		alu_cmp(D, decode_JJJ_read_56(JJJ, !D), false);
	}
	inline void DSP::op_Cmp_xxS2(const TWord op)
	{
		const TWord iiiiii = getFieldValue<Cmp_xxS2,Field_iiiiii>(op);
		
		const TReg56 r56 = toAluOperand(TReg24(iiiiii));

		alu_cmp( bittest(op,3), r56, false );
	}
	inline void DSP::op_Cmp_xxxxS2(const TWord op)
	{
		const TReg24 s( signextend<int,24>( immediateDataExt<Cmp_xxxxS2>() ) );

		const TReg56 r56 = toAluOperand(s);

		alu_cmp( bittest(op,3), r56, false );
	}
	inline void DSP::op_Cmpm_S1S2(const TWord op)
	{
		const auto D = getFieldValue<Cmpm_S1S2, Field_d>(op);
		const auto JJJ = getFieldValue<Cmpm_S1S2, Field_JJJ>(op);
		alu_cmp(D, decode_JJJ_read_56(JJJ, !D), true);
	}
	inline void DSP::op_Cmpu_S1S2(const TWord op)
	{
		const auto D = getFieldValue<Cmpu_S1S2, Field_d>(op);
		const auto ggg = getFieldValue<Cmpu_S1S2, Field_ggg>(op);
		// ggg only defines 0 (the other accumulator) and 4..7 (x0, y0, x1, y1). Those encodings are
		// identical to the ones JJJ uses, so the existing decoder covers every valid CMPU operand.
		assert((ggg == 0 || ggg >= 4) && "invalid ggg value for CMPU");
		alu_cmpu(D, decode_JJJ_read_56(ggg, !D));
	}
	inline void DSP::op_Dec(const TWord op)
	{
		auto ab = getFieldValue<Dec,Field_d>(op);
		TReg56& d = ab ? reg.b : reg.a;

		const auto old = d;
		const auto res = (d.var -= (TInt64(1) << g_aluShift));

		aluMask(d);

		sr_z_update(d);
		sr_v_update(res,d);
		sr_l_update_by_v();
		sr_c_update_arithmetic(old,d);
		sr_toggle( CCR_C, bittest(d, 47 + g_aluShift) != bittest(old, 47 + g_aluShift) );
		setCCRDirty(ab, d, CCR_E | CCR_U | CCR_N);
	}

	inline void DSP::op_Div(const TWord op)
	{
		const TWord jj	= getFieldValue<Div,Field_JJ>(op);
		const auto ab	= getFieldValue<Div,Field_d>(op);

		TReg56& d = ab ? reg.b : reg.a;

		const TReg24 s24 = decode_JJ_read( jj );

		const auto msbOld = bitvalue<55 + g_aluShift>(d);
		
		const auto c = msbOld != bitvalue<23>(s24);
		
		d.var <<= 1;
		d.var |= static_cast<TInt64>(sr_test_noCache(CCR_C) ? 1 : 0) << g_aluShift;	// carry enters at the accumulator LSB

		const auto msbNew = bitvalue<55 + g_aluShift>(d);

		if( c )
			d.var = ((d.var + (signextend<TInt64,24>(s24.var) << (24 + g_aluShift)) )&static_cast<TInt64>(0x00ffffffff000000ull << g_aluShift)) | (d.var & (0xffffffll << g_aluShift));
		else
			d.var = ((d.var - (signextend<TInt64,24>(s24.var) << (24 + g_aluShift)) )&static_cast<TInt64>(0x00ffffffff000000ull << g_aluShift)) | (d.var & (0xffffffll << g_aluShift));

		sr_toggle( CCRB_C, !bitvalue<55 + g_aluShift>(d) );	// Set if bit 55 of the result is cleared.
		sr_toggle( CCRB_V, msbNew != msbOld );	// Set if the MSB of the destination operand is changed as a result of the instructions left shift operation.

		if(msbNew != msbOld)
			sr_set(CCR_L);						// Set if the Overflow bit (V) is set.

//		LOG( "DIV: d" << std::hex << debugOldD.var << " s" << std::hex << debugOldS.var << " =>" << std::hex << d.var );
	}
	inline void DSP::op_Dmac(const TWord op)
	{
		const auto ss			= getFieldValue<Dmac,Field_S, Field_s>(op);
		const bool ab			= getFieldValue<Dmac,Field_d>(op);
		const bool negate		= getFieldValue<Dmac,Field_k>(op);

		const TWord qqqq		= getFieldValue<Dmac,Field_QQQQ>(op);

		const bool s1Unsigned	= ss > 1;
		const bool s2Unsigned	= ss > 0;

		TReg24 s1, s2;
		decode_QQQQ_read( s1, s2, qqqq );

		alu_dmac( ab, s1, s2, negate, s1Unsigned, s2Unsigned );
	}
	inline void DSP::op_Eor_SD(const TWord op)
	{
		const auto D = getFieldValue<Or_SD, Field_d>(op);
		const auto JJ = getFieldValue<Or_SD, Field_JJ>(op);
		alu_eor(D, decode_JJ_read(JJ).var);
	}
	inline void DSP::op_Eor_xx(const TWord op)
	{
		const auto ab = getFieldValue<Eor_xx, Field_d>(op);
		const TWord xxxx = getFieldValue<Eor_xx, Field_iiiiii>(op);
		alu_eor(ab, xxxx);
	}
	inline void DSP::op_Eor_xxxx(const TWord op)
	{
		const auto ab = getFieldValue<Eor_xxxx, Field_d>(op);
		const TWord xxxx = immediateDataExt<Eor_xxxx>();
		alu_eor(ab, xxxx);
	}
	inline void DSP::op_Extract_S1S2(const TWord op)
	{
		const auto sss = getFieldValue<Extract_S1S2, Field_SSS>(op);
		const auto widthOffset = decode_sss_read<TWord>(sss);
		const bool abDst = getFieldValue<Extract_S1S2, Field_D>(op);
		const bool abSrc = getFieldValue<Extract_S1S2, Field_s>(op);
		alu_extract(abDst, abSrc, widthOffset);
	}
	inline void DSP::op_Extract_CoS2(const TWord op)
	{
		const auto widthOffset = fetchOpWordB();
		const bool abDst = getFieldValue<Extract_CoS2, Field_D>(op);
		const bool abSrc = getFieldValue<Extract_CoS2, Field_s>(op);
		alu_extract(abDst, abSrc, widthOffset);
	}

	inline void DSP::alu_extract(const bool abDst, const bool abSrc, const TWord widthOffset)
	{
		const auto width = (widthOffset >> (sr_test(SR_SA) ? 16 : 12)) & 0x3f;
		const auto offset = (widthOffset >> (sr_test(SR_SA) ? 8 : 0)) & 0x3f;
		const TReg56& dSrc = abSrc ? reg.b : reg.a;
		TReg56& dDst = abDst ? reg.b : reg.a;

		if(!width)
			dDst.var = 0;
		else
		{
			const auto mask = TReg56::bitMask >> (56 - width);
			auto field = (static_cast<uint64_t>(dSrc.var) >> (offset + g_aluShift)) & mask;
			if(field & (uint64_t(1) << (width - 1)))
				field |= TReg56::bitMask ^ mask;
			dDst.var = static_cast<TInt64>(field << g_aluShift);
		}

		sr_clear(CCR_C);
		sr_clear(CCR_V);
		sr_z_update(dDst);
		setCCRDirty(abDst, dDst, CCR_E | CCR_U | CCR_N);
	}

	inline void DSP::alu_extractu(bool abDst, bool abSrc, const TWord widthOffset)
	{
		const auto width = (widthOffset >> (sr_test(SR_SA) ? 16 : 12)) & 0x3f;
		const auto offset = (widthOffset >> (sr_test(SR_SA) ? 8 : 0)) & 0x3f;

		const TReg56& dSrc = abSrc ? reg.b : reg.a;
		TReg56& dDst = abDst ? reg.b : reg.a;
		if(!width)
			dDst.var = 0;
		else
		{
			const auto mask = TReg56::bitMask >> (56 - width);
			dDst.var = static_cast<TInt64>(((static_cast<uint64_t>(dSrc.var) >> (offset + g_aluShift)) & mask) << g_aluShift);
		}

		sr_clear(CCR_C);
		sr_clear(CCR_V);
		sr_z_update(dDst);
		setCCRDirty(abDst, dDst, CCR_E | CCR_U | CCR_N);
	}
	inline void DSP::op_Extractu_S1S2(const TWord op)
	{
		const auto sss = getFieldValue<Extractu_S1S2, Field_SSS>(op);
		const auto widthOffset = decode_sss_read<TWord>(sss);

		const bool abDst = getFieldValue<Extractu_S1S2, Field_D>(op);
		const bool abSrc = getFieldValue<Extractu_S1S2, Field_s>(op);

		alu_extractu(abDst, abSrc, widthOffset);
	}
	inline void DSP::op_Extractu_CoS2(const TWord op)
	{
		const TWord width_offset = fetchOpWordB();

		const bool abDst = getFieldValue<Extractu_CoS2, Field_D>(op);
		const bool abSrc = getFieldValue<Extractu_CoS2, Field_s>(op);

		alu_extractu(abDst, abSrc, width_offset);
	}
	inline void DSP::op_Inc(const TWord op)
	{
		const auto ab = getFieldValue<Inc,Field_d>(op);
		TReg56& d = ab ? reg.b : reg.a;

		const auto old = d;

		const auto res = (d.var += (TInt64(1) << g_aluShift));

		aluMask(d);

		sr_z_update(d);
		sr_v_update(res,d);
		sr_l_update_by_v();
		sr_c_update_arithmetic(old,d);	// TODO: what? C updated two times?!
		sr_toggle( CCR_C, bittest(d, 47 + g_aluShift) != bittest(old, 47 + g_aluShift) );
		setCCRDirty(ab, d, CCR_E | CCR_U | CCR_N);
	}

	inline void DSP::alu_insert(bool abDst, const TWord src, const TWord widthOffset)
	{
		const auto width = (widthOffset >> (sr_test(SR_SA) ? 16 : 12)) & 0x3f;

		// the offset is relative to the 56-bit value, so it moves up with the ALU
		const uint64_t offset = ((widthOffset >> (sr_test(SR_SA) ? 8 : 0)) & 0x3f) + g_aluShift;

		const auto mask = width ? (uint64_t(1) << width) - 1 : 0;

		uint64_t s = src & mask;
		s <<= offset;

		TReg56& dReg = abDst ? reg.b : reg.a;
		auto& d = reinterpret_cast<uint64_t&>(dReg.var);

		d &= ~(static_cast<uint64_t>(mask) << offset);
		d |= s;

		sr_clear(CCR_C);
		sr_clear(CCR_V);
		sr_z_update(dReg);
		setCCRDirty(abDst, dReg, CCR_E | CCR_U | CCR_N);
	}

	inline void DSP::op_Insert_S1S2(const TWord op)
	{
		const auto D   = getFieldValue<Insert_S1S2, Field_D>(op);
		const auto qqq = getFieldValue<Insert_S1S2, Field_qqq>(op);
		const auto sss = getFieldValue<Insert_S1S2, Field_SSS>(op);

		const auto src = decode_qqq_read(qqq);
		const auto co = decode_sss_read<TWord>(sss);

		alu_insert(D, src.toWord(), co);
	}
	inline void DSP::op_Insert_CoS2(const TWord op)
	{
		const auto D   = getFieldValue<Insert_CoS2, Field_D>(op);
		const auto qqq = getFieldValue<Insert_CoS2, Field_qqq>(op);

		const auto src = decode_qqq_read(qqq);

		alu_insert(D, src.toWord(), fetchOpWordB());
	}

	inline void DSP::op_Lsl_D(const TWord op)
	{
		const auto D = getFieldValue<Lsl_D,Field_D>(op);
		alu_lsl(D, 1);
	}
	inline void DSP::op_Lsl_ii(const TWord op)
	{
		const auto shiftAmount = getFieldValue<Lsl_ii,Field_iiiii>(op);
		const auto abDst = getFieldValue<Lsl_ii,Field_D>(op);

		alu_lsl(abDst, shiftAmount);
	}
	inline void DSP::op_Lsl_SD(const TWord op)
	{
		const auto sss   = getFieldValue<Lsl_SD,Field_sss>(op);
		const auto abDst = getFieldValue<Lsl_SD,Field_D>(op);

		const TWord shiftAmount = decode_sss_read<TWord>( sss ) & 0x3f;
		alu_lsl(abDst, shiftAmount);
	}
	inline void DSP::op_Lsr_D(const TWord op)
	{
		const auto D = getFieldValue<Lsr_D,Field_D>(op);
		alu_lsr(D, 1);
	}
	inline void DSP::op_Lsr_ii(const TWord op)
	{
		const auto shiftAmount = getFieldValue<Lsr_ii,Field_iiiii>(op);
		const auto abDst = getFieldValue<Lsr_ii,Field_D>(op);

		alu_lsr(abDst, shiftAmount);
	}
	inline void DSP::op_Lsr_SD(const TWord op)
	{
		const auto sss   = getFieldValue<Lsr_SD,Field_sss>(op);
		const auto abDst = getFieldValue<Lsr_SD,Field_D>(op);

		const TWord shiftAmount = decode_sss_read<TWord>( sss ) & 0x3f;
		alu_lsr(abDst, shiftAmount);
	}
	inline void DSP::op_Mac_S1S2(const TWord op)
	{
		alu_multiply(op);
	}
	inline void DSP::op_Mac_S(const TWord op)
	{
		const TWord sssss	= getFieldValue<Mac_S,Field_sssss>(op);
		const TWord qq		= getFieldValue<Mac_S,Field_QQ>(op);
		const bool	ab		= getFieldValue<Mac_S,Field_d>(op);
		const bool	negate	= getFieldValue<Mac_S,Field_k>(op);

		const TReg24 s1 = decode_QQ_read( qq );
		const TReg24 s2( decode_sssss(sssss) );

		alu_mpy(ab, s1, s2, negate, true);
	}
	inline void DSP::op_Maci_xxxx(const TWord op)
	{
		const bool	ab		= getFieldValue<Maci_xxxx,Field_d>(op);
		const bool	negate	= getFieldValue<Maci_xxxx,Field_k>(op);
		const TWord qq		= getFieldValue<Maci_xxxx,Field_qq>(op);

		const TReg24 s		= TReg24(immediateDataExt<Maci_xxxx>());

		const TReg24 reg	= decode_qq_read(qq);

		alu_mpy( ab, reg, s, negate, true );
	}
	inline void DSP::op_Macsu(const TWord op)
	{
		const bool uu			= getFieldValue<Macsu,Field_s>(op);
		const bool ab			= getFieldValue<Macsu,Field_d>(op);
		const bool negate		= getFieldValue<Macsu,Field_k>(op);
		const TWord qqqq		= getFieldValue<Macsu,Field_QQQQ>(op);

		TReg24 s1, s2;
		decode_QQQQ_read( s1, s2, qqqq );

		alu_mac( ab, s1, s2, negate, uu );
	}
	inline void DSP::op_Macr_S1S2(const TWord op)
	{
		alu_multiply(op);
	}
	inline void DSP::op_Macr_S(const TWord op)
	{
		const TWord sssss	= getFieldValue<Macr_S,Field_sssss>(op);
		const bool ab		= getFieldValue<Macr_S,Field_d>(op);
		const bool negate	= getFieldValue<Macr_S,Field_k>(op);
		const TWord qq		= getFieldValue<Macr_S,Field_QQ>(op);

		const TReg24 s1 = decode_QQ_read( qq );
		const TReg24 s2( decode_sssss(sssss) );

		alu_mpy(ab, s1, s2, negate, true, true);
	}
	inline void DSP::op_Macri_xxxx(const TWord op)
	{
		errNotImplemented("MACRI");		
	}
	inline void DSP::op_Max(const TWord op)
	{
		const auto a = signextend<int64_t, 56>(aluA().var);
		const auto b = signextend<int64_t, 56>(aluB().var);

		if(a >= b)
		{
			setALU(true, aluA());
			sr_clear(CCR_C);
		}
		else
		{
			sr_set(CCR_C);
		}
	}
	inline void DSP::op_Maxm(const TWord op)
	{
		const auto a = std::abs(signextend<int64_t, 56>(aluA().var));
		const auto b = std::abs(signextend<int64_t, 56>(aluB().var));

		if(a >= b)
		{
			setALU(true, aluA());
			sr_clear(CCR_C);
		}
		else
		{
			sr_set(CCR_C);
		}
	}
	inline void DSP::op_Merge(const TWord op)
	{
		errNotImplemented("MERGE");		
	}
	inline void DSP::op_Mpy_S1S2D(const TWord op)
	{
		alu_multiply(op);
	}
	inline void DSP::op_Mpy_SD(const TWord op)
	{
		const int sssss		= getFieldValue<Mpy_SD,Field_sssss>(op);
		const TWord QQ		= getFieldValue<Mpy_SD,Field_QQ>(op);
		const bool ab		= getFieldValue<Mpy_SD,Field_d>(op);
		const bool negate	= getFieldValue<Mpy_SD,Field_k>(op);

		const TReg24 s1 = decode_QQ_read(QQ);
		const TReg24 s2 = TReg24( decode_sssss(sssss) );

		alu_mpy( ab, s1, s2, negate, false );
	}
	inline void DSP::op_Mpy_su(const TWord op)
	{
		const bool ab		= getFieldValue<Mpy_su,Field_d>(op);
		const bool negate	= getFieldValue<Mpy_su,Field_k>(op);
		const bool uu		= getFieldValue<Mpy_su,Field_s>(op);
		const TWord qqqq	= getFieldValue<Mpy_su,Field_QQQQ>(op);

		TReg24 s1, s2;
		decode_QQQQ_read( s1, s2, qqqq );

		alu_mpysuuu( ab, s1, s2, negate, false, uu );
	}
	inline void DSP::op_Mpyi(const TWord op)
	{
		const bool	ab		= getFieldValue<Mpyi,Field_d>(op);
		const bool	negate	= getFieldValue<Mpyi,Field_k>(op);
		const TWord qq		= getFieldValue<Mpyi,Field_qq>(op);

		const TReg24 s		= TReg24(immediateDataExt<Mpyi>());

		const TReg24 reg	= decode_qq_read(qq);

		alu_mpy( ab, reg, s, negate, false );
	}
	inline void DSP::op_Mpyr_S1S2D(const TWord op)
	{
		alu_multiply(op);
	}
	inline void DSP::op_Mpyr_SD(const TWord op)
	{
		const int sssss		= getFieldValue<Mpyr_SD,Field_sssss>(op);
		const TWord QQ		= getFieldValue<Mpyr_SD,Field_QQ>(op);
		const bool ab		= getFieldValue<Mpyr_SD,Field_d>(op);
		const bool negate	= getFieldValue<Mpyr_SD,Field_k>(op);

		const TReg24 s1 = decode_QQ_read(QQ);
		const TReg24 s2 = TReg24( decode_sssss(sssss) );

		alu_mpy(ab, s1, s2, negate, false, true);
	}
	inline void DSP::op_Mpyri(const TWord op)
	{
		errNotImplemented("MPYRI");
	}
	inline void DSP::op_Neg(const TWord op)
	{
		const auto D = getFieldValue<Neg, Field_d>(op);
		alu_neg(D);
	}
	inline void DSP::op_Not(const TWord op)
	{
		const auto D = getFieldValue<Not, Field_d>(op);
		alu_not(D);
	}
	inline void DSP::op_Or_SD(const TWord op)
	{
		const auto D = getFieldValue<Or_SD, Field_d>(op);
		const auto JJ = getFieldValue<Or_SD, Field_JJ>(op);
		alu_or(D, decode_JJ_read(JJ).var);
	}
	inline void DSP::op_Or_xx(const TWord op)
	{
		const auto ab		= getFieldValue<Or_xx,Field_d>(op);
		const TWord xxxx	= getFieldValue<Or_xx,Field_iiiiii>(op);

		alu_or(ab, xxxx);
	}
	inline void DSP::op_Or_xxxx(const TWord op)
	{
		const auto ab = getFieldValue<Or_xxxx,Field_d>(op);
		const TWord xxxx = immediateDataExt<Or_xxxx>();

		alu_or( ab, xxxx );
	}
	inline void DSP::op_Ori(const TWord op)
	{
		const TWord iiiiiiii = getFieldValue<Ori,Field_iiiiiiii>(op);
		const TWord ee = getFieldValue<Ori,Field_EE>(op);

		switch( ee )
		{
		case 0:	mr ( TReg8( mr().var | iiiiiiii) );	break;
		case 1:	ccr( TReg8(ccr().var | iiiiiiii) );	break;
		case 2:	com( TReg8(com().var | iiiiiiii) );	break;
		case 3:	eom( TReg8(eom().var | iiiiiiii) );	break;
		}
	}
	inline void DSP::op_Rnd(const TWord op)
	{
		const auto D = getFieldValue<Rnd, Field_d>(op);
		alu_rnd(D);
	}
	inline void DSP::op_Rol(const TWord op)
	{
		const auto D = getFieldValue<Rol, Field_d>(op);
		alu_rotate(D, false);
	}
	inline void DSP::op_Ror(const TWord op)
	{
		alu_rotate(getFieldValue<Ror, Field_d>(op), true);
	}
	inline void DSP::op_Sbc(const TWord op)
	{
		errNotImplemented("SBC");
	}
	inline void DSP::op_Sub_SD(const TWord op)
	{
		const auto D = getFieldValue<Sub_SD, Field_d>(op);
		const auto JJJ = getFieldValue<Sub_SD, Field_JJJ>(op);
		alu_sub(D, decode_JJJ_read_56(JJJ, !D));
	}
	inline void DSP::op_Sub_xx(const TWord op)
	{
		const auto ab		= getFieldValue<Sub_xx,Field_d>(op);
		const TWord iiiiii	= getFieldValue<Sub_xx,Field_iiiiii>(op);

		alu_sub( ab, toAluOperand(iiiiii) );
	}
	inline void DSP::op_Sub_xxxx(const TWord op)
	{
		const auto ab = getFieldValue<Sub_xxxx,Field_d>(op);

		const TReg56 r56 = toAluOperand(TReg24(immediateDataExt<Sub_xxxx>()));

		alu_sub( ab, r56 );
	}
	inline void DSP::op_Subl(const TWord op)
	{
		alu_shiftedArithmetic(getFieldValue<Subl, Field_d>(op), true, true);
	}
	inline void DSP::op_Subr(const TWord op)
	{
		alu_shiftedArithmetic(getFieldValue<Subr, Field_d>(op), false, true);
	}

	inline void DSP::op_Tfr(const TWord op)
	{
		const auto D = getFieldValue<Tfr, Field_d>(op);
		const auto JJJ = getFieldValue<Tfr, Field_JJJ>(op);
		alu_tfr(D, decode_JJJ_read_56(JJJ, !D));
	}
	inline void DSP::op_Tst(const TWord op)
	{
		const auto D = getFieldValue<Tst, Field_d>(op);
		alu_tst(D);
	}
	inline void DSP::op_Vsl(const TWord op)
	{
		errNotImplemented("VSL");		
	}
}
