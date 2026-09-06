// Independent architectural expectations from DSP56300FM rev. 5:
// sections 3.4/5.4, tables 3-3/3-4/5-1, figure 3-10; chapter 13 instruction definitions.
// Architectural regression coverage, independent of emulator arithmetic helpers.
// Passing these cases is not a full hardware-conformance certificate.
#include "dsp56kEmu/assembler.h"
#include "dsp56kEmu/dsp.h"
#include "dsp56kEmu/peripherals.h"

#include <array>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace
{
	using namespace dsp56k;
	constexpr uint64_t Mask56 = 0x00ffffffffffffffull;
	constexpr uint64_t DefinedSA = 0x00ffffff00ffff00ull;
	DefaultMemoryValidator validator;

	struct Rig
	{
		Memory memory{validator, 0x10000, 0xc00000, 0x800000};
		PeripheralsNop px, py;
		DSP dsp{memory, &px, &py};
		Assembler assembler;
		int engine;
		unsigned checks = 0, failures = 0;

		explicit Rig(int _engine) : engine(_engine)
		{
			dsp.setUseJit(engine != 0);
			auto config = dsp.getJit().getConfig();
			config.linkJitBlocks = engine == 2;
			config.maxDoIterations = 64;
			dsp.getJit().setConfig(config);
			emit(0xff0, "jmp $ff0");
		}
		uint64_t a() const { TReg56 value; dsp.readReg(Reg_A, value); return value.var; }
		const char* name() const { return engine == 0 ? "interpreter" : engine == 1 ? "JIT unlinked" : "JIT linked"; }
		TWord emit(TWord pc, const std::string& instruction)
		{
			// The local assembler does not implement long-memory MOVE syntax yet.
			if(instruction == "move l:$6,a") { dsp.memWriteP(pc, 0x488600); return pc + 1; }
			if(instruction == "move a,l:$7") { dsp.memWriteP(pc, 0x480700); return pc + 1; }
			const auto result = assembler.assemble(instruction.c_str());
			if(!result.success()) { std::fprintf(stderr, "Assembly failed: %s\n", instruction.c_str()); std::exit(2); }
			if(instruction == "cmp b,a") check("CMP encoding", result.word[0], 0x200005);
			if(instruction == "tfr b,a") check("TFR encoding", result.word[0], 0x200001);
			if(instruction == "mpy x0,y0,a") check("signed MPY encoding", result.word[0], 0x2000d0);
			if(instruction == "mac x0,y0,a") check("signed MAC encoding", result.word[0], 0x2000d2);
			for(unsigned i = 0; i < result.wordCount; ++i) dsp.memWriteP(pc++, result.word[i]);
			return pc;
		}
		void start(TWord pc, TWord sr)
		{
			dsp.getSR(); // materialize and clear the interpreter CCR cache before installing the fixture
			dsp.regs().sr.var = sr;
			dsp.setPC(TReg24(static_cast<int>(pc)));
			dsp.getJit().checkModeChange();
		}
		void run()
		{
			unsigned steps = 0;
			while(dsp.getPC().var != 0xff0 && ++steps < 10000)
			{
				if(engine) dsp.execJit(); else dsp.execInterpreter();
			}
			if(steps >= 10000) { std::fprintf(stderr, "%s did not finish\n", name()); std::exit(2); }
		}
		void check(const char* label, uint64_t actual, uint64_t expected)
		{
			++checks;
			if(actual == expected) return;
			++failures;
			std::printf("FAIL %-12s %-28s actual=%014llx expected=%014llx\n", name(), label,
				static_cast<unsigned long long>(actual), static_cast<unsigned long long>(expected));
		}
	};

	void serialClock(Rig& r)
	{
		struct ClockProbe : EsxiClock
		{
			using EsxiClock::EsxiClock;
			using EsxiClock::getLastClock;
		} clock(r.px);
		clock.setDSP(&r.dsp);
		clock.setClockSource(EsxiClock::ClockSource::Cycles);
		clock.setExternalClockFrequency(4000000);
		clock.setSamplerate(44100);
		clock.setPCTL(0x040012);
		const auto origin = r.dsp.getCycles();
		bool exact = true;
		for(uint64_t slot = 1; slot <= 88200; ++slot)
		{
			const auto deadline = origin + (slot * 76000000 + 88199) / 88200;
			const auto previous = clock.getLastClock();
			r.dsp.fastForward(0, static_cast<TWord>(deadline - r.dsp.getCycles() - 1));
			clock.exec();
			exact &= clock.getLastClock() == previous;
			r.dsp.fastForward(0, 1);
			clock.exec();
			exact &= clock.getLastClock() == deadline;
		}
		r.check("serial 88200 exact slots", exact, true);
		r.check("serial one second", clock.getLastClock() - origin, 76000000);
		// Late service must consume one event at a time without resetting phase.
		r.dsp.fastForward(0, 760000);
		for(unsigned i = 0; i < 882; ++i) clock.exec();
		r.check("serial delayed catch-up", clock.getLastClock() - origin, 76760000);
		clock.setEnabled(false);
		r.dsp.fastForward(0, 1234);
		clock.exec();
		r.check("serial disabled", clock.getLastClock() - origin, 76760000);
		clock.setEnabled(true);
		r.check("serial restart interval", clock.getCyclesPerSample(), 862);
		clock.setSpeedPercent(125);
		const auto overclockOrigin = r.dsp.getCycles();
		r.dsp.fastForward(0, 950000);
		for(unsigned i = 0; i < 882; ++i) clock.exec();
		r.check("serial 125 percent", clock.getLastClock() - overclockOrigin, 950000);
		clock.setCyclesPerSample(100);
		r.check("serial fixed period", clock.getCyclesPerSample(), 125);
		clock.setSpeedPercent(100);
		r.check("serial fixed normal", clock.getCyclesPerSample(), 100);
		clock.setCyclesPerSample(0);
		clock.setSamplerate(48000);
		const auto rateOrigin = r.dsp.getCycles();
		r.dsp.fastForward(0, 760000);
		for(unsigned i = 0; i < 960; ++i) clock.exec();
		r.check("serial 48 kHz", clock.getLastClock() - rateOrigin, 760000);
		clock.setSamplerate(44100);
		clock.setExternalClockFrequency(4000001);
		clock.setPCTL(0x240012); // divide PLL by 3: frequency is not an integer Hz
		const auto pllOrigin = r.dsp.getCycles();
		r.dsp.fastForward(0, 25333340);
		for(unsigned i = 0; i < 88200; ++i) clock.exec();
		r.check("serial fractional PLL", clock.getLastClock() - pllOrigin, 25333340);
	}

	// Architectural 40-bit arithmetic value -> the defined bits of A2:A1:A0.
	uint64_t packSA(uint64_t value)
	{
		return ((value & 0xff00000000ull) << 16) | ((value & 0xffff0000ull) << 16) |
			((value & 0xffffull) << 8);
	}

	void transfersAndArithmetic(Rig& rig)
	{
		// Mode is established before execution; two NOPs also avoid relying on
		// the unsupported immediate mode-transition sequences in the firmware.
		auto program = [&](const char* instruction)
		{
			auto pc = rig.emit(0x200, "nop");
			pc = rig.emit(pc, "nop");
			pc = rig.emit(pc, instruction);
			rig.emit(pc, "jmp $ff0");
			rig.start(0x200, SR_SA | 0x300);
		};
		rig.memory.set(MemArea_X, 6, 0x1234);
		rig.memory.set(MemArea_Y, 6, 0x5678);
		program("move l:$6,a"); rig.run();
		rig.check("SA long load, defined bits", rig.a() & DefinedSA, 0x00123400567800ull);
		program("move a0,x:$5"); rig.run();
		rig.check("SA long load -> A0 bus", rig.memory.get(MemArea_X, 5), 0x5678);

		rig.dsp.writeReg(Reg_A, TReg56(static_cast<TReg56::MyType>(0x00123400567800ull)));
		program("move a,l:$7"); rig.run();
		rig.check("SA long store high", rig.memory.get(MemArea_X, 7), 0x1234);
		rig.check("SA long store low", rig.memory.get(MemArea_Y, 7), 0x5678);

		rig.dsp.writeReg(Reg_A, TReg56(static_cast<TReg56::MyType>(packSA(0x1234ffff))));
		rig.dsp.writeReg(Reg_B, TReg56(static_cast<TReg56::MyType>(packSA(1))));
		program("add b,a"); rig.run();
		rig.check("SA add LSP carry into MSP", rig.a() & DefinedSA, packSA(0x12350000));

		rig.dsp.writeReg(Reg_A, TReg56(static_cast<TReg56::MyType>(packSA(0x12340000))));
		rig.dsp.writeReg(Reg_B, TReg56(static_cast<TReg56::MyType>(packSA(1))));
		program("sub b,a"); rig.run();
		rig.check("SA sub LSP borrow", rig.a() & DefinedSA, packSA(0x1233ffff));

		rig.dsp.writeReg(Reg_A, TReg56(static_cast<TReg56::MyType>(packSA(0x8000))));
		program("asl a"); rig.run();
		rig.check("SA shift across word gap", rig.a() & DefinedSA, packSA(0x10000));

		rig.dsp.writeReg(Reg_X0, TReg24(0x100)); rig.dsp.writeReg(Reg_Y0, TReg24(0x100)); // 16-bit fractional raw operands 1, 1
		program("mpy x0,y0,a"); rig.run();
		rig.check("SA fractional multiply", rig.a() & DefinedSA, packSA(2));
	}

	void partialConditionCodeWrites(Rig& rig)
	{
		for(bool sa : {false, true})
			for(const char* instruction : {"and x0,a", "or x0,a", "eor x0,a", "lsl a", "lsr a", "rol a", "ror a", "clr a"})
			{
				// TST B leaves lazy E/U/N. The next instruction writes N and must
				// keep its own result when the remaining flags are materialized.
				const uint64_t a = sa ? packSA(0x20000000) : 0x00200000000000ull;
				const uint64_t b = sa ? packSA(0xffffffffffull) : Mask56;
				rig.dsp.writeReg(Reg_A, TReg56(static_cast<TReg56::MyType>(a)));
				rig.dsp.writeReg(Reg_B, TReg56(static_cast<TReg56::MyType>(b)));
				rig.dsp.writeReg(Reg_X0, TReg24(0x200000));
				auto pc = rig.emit(0x2a0, "tst b");
				pc = rig.emit(pc, instruction); rig.emit(pc, "jmp $ff0");
				rig.start(0x2a0, 0x300 | (sa ? SR_SA : 0)); rig.run();
				const bool zero = instruction[0] == 'e' || instruction[0] == 'c';
				rig.check("partial CCR after arithmetic", rig.dsp.getSR().var & 0x3f, CCR_U | (zero ? CCR_Z : 0));
			}
	}

	void logicalWordOperations(Rig& rig)
	{
		for(bool sa : {false, true})
		{
			const unsigned width = sa ? 16 : 24, pad = sa ? 8 : 0;
			const uint32_t mask = (1u << width) - 1, sign = 1u << (width - 1);
			const std::array<uint32_t, 7> values{0, 1, 2, sign - 1, sign, sign + 1, mask};
			for(const auto value : values)
				for(const char* instruction : {"rol a", "ror a", "lsl #0,a", "lsl #1,a", "lsl #8,a", "lsl #16,a",
					"lsl #24,a", "lsl #23,a", "lsr #0,a", "lsr #1,a", "lsr #8,a", "lsr #16,a", "lsr #24,a", "lsr #23,a"})
					for(unsigned carry : {0u, 1u})
					{
						const bool rotate = instruction[0] == 'r', right = instruction[2] == 'r';
						const unsigned count = rotate ? 1 : std::strtoul(instruction + 5, nullptr, 10);
						if(count > width) continue; // FM limits logical shifts to the operand width.
						uint64_t result = right ? value >> count : uint64_t(value) << count;
						if(rotate) result |= right ? carry << (width - 1) : carry;
						result &= mask;
						const unsigned out = !count || count > width ? 0 : (value >> (right ? count - 1 : width - count)) & 1;
						const uint64_t preserved = 0x5b000000abcdefull;
						// Dirty padding must not enter the arithmetic unit; A0 and A2 are unchanged.
						rig.dsp.writeReg(Reg_A, TReg56(static_cast<TReg56::MyType>(preserved | (uint64_t((value << pad) | (sa ? 0xa5 : 0)) << 24))));
						auto pc = rig.emit(0x280, instruction); rig.emit(pc, "jmp $ff0");
						rig.start(0x280, 0x300 | (sa ? SR_SA : 0) | 0x72 | carry); rig.run();
						rig.check(instruction, rig.a(), preserved | (result << (24 + pad)));
						const unsigned flags = 0x70 | out | (result == 0 ? CCR_Z : 0) | (result & sign ? CCR_N : 0);
						rig.check("logical shift/rotate CCR", rig.dsp.getSR().var & 0x7f, flags);
						if(!rotate)
						{
							// Register-controlled shifts use a different native lowering path.
							rig.dsp.writeReg(Reg_A, TReg56(static_cast<TReg56::MyType>(preserved | (uint64_t((value << pad) | (sa ? 0xa5 : 0)) << 24))));
							rig.dsp.writeReg(Reg_X0, TReg24(count));
							pc = rig.emit(0x280, right ? "lsr x0,a" : "lsl x0,a"); rig.emit(pc, "jmp $ff0");
							rig.start(0x280, 0x300 | (sa ? SR_SA : 0) | 0x72 | carry); rig.run();
							rig.check("register logical shift", rig.a(), preserved | (result << (24 + pad)));
							rig.check("register logical shift CCR", rig.dsp.getSR().var & 0x7f, flags);
						}
					}
			for(const auto a : values)
				for(const auto b : values)
					for(const char* instruction : {"and x0,a", "or x0,a", "eor x0,a"})
					{
						const uint32_t result = instruction[0] == 'a' ? a & b : instruction[0] == 'o' ? a | b : a ^ b;
						const uint64_t preserved = 0x5b000000abcdefull;
						rig.dsp.writeReg(Reg_A, TReg56(static_cast<TReg56::MyType>(preserved | (uint64_t((a << pad) | (sa ? 0xa5 : 0)) << 24))));
						rig.dsp.writeReg(Reg_X0, TReg24((b << pad) | (sa ? 0x5a : 0)));
						auto pc = rig.emit(0x280, instruction); rig.emit(pc, "jmp $ff0");
						rig.start(0x280, 0x373 | (sa ? SR_SA : 0)); rig.run();
						rig.check(instruction, rig.a(), preserved | (uint64_t(result) << (24 + pad)));
						rig.check("logical CCR", rig.dsp.getSR().var & 0x7f, 0x71 | (result == 0 ? CCR_Z : 0) | (result & sign ? CCR_N : 0));
					}
		}
	}

	// FM table 5-1 and ADD/SUB/CMP: full accumulator overflow sets V and sticky L.
	// Test each engine against integer arithmetic, including a following operation
	// that clears V but must retain L. CMP must leave both operands intact.
	void arithmeticOverflow(Rig& rig)
	{
		for(bool sa : {false, true})
		{
			const unsigned width = sa ? 40 : 56;
			const uint64_t sign = uint64_t(1) << (width - 1), mask = sign * 2 - 1;
			const std::array<uint64_t, 7> values{0, 1, sign - 2, sign - 1, sign, sign + 1, mask};
			auto architectural = [&](uint64_t value) { return sa ? packSA(value) : value; };
			for(const char* instruction : {"add b,a", "sub b,a", "cmp b,a"})
				for(const auto a : values)
					for(const auto b : values)
						for(bool sticky : {false, true})
						{
							const bool add = instruction[0] == 'a', compare = instruction[0] == 'c';
							const uint64_t result = (add ? a + b : a - b) & mask;
							const bool overflow = ((add ? ~(a ^ b) : (a ^ b)) & (a ^ result) & sign) != 0;
							const unsigned flags = (add ? a + b > mask : b > a) |
								(overflow ? CCR_V : 0) | (overflow || sticky ? CCR_L : 0) |
								(result == 0 ? CCR_Z : 0) | (result & sign ? CCR_N : 0);
							auto pc = rig.emit(0x240, instruction);
							rig.emit(pc, "jmp $ff0");
							rig.dsp.writeReg(Reg_A, TReg56(static_cast<TReg56::MyType>(architectural(a))));
							rig.dsp.writeReg(Reg_B, TReg56(static_cast<TReg56::MyType>(architectural(b))));
							rig.start(0x240, 0x300 | (sa ? SR_SA : 0) | CCR_V | (sticky ? CCR_L : 0));
							rig.run();
							rig.check(instruction, rig.a(), architectural(compare ? a : result));
							rig.check("arithmetic C,V,Z,N,L", rig.dsp.getSR().var & 0x4f, flags);
						}
			// Keep the producer and consumer in the same compiled block.
			auto pc = rig.emit(0x240, "add b,a");
			pc = rig.emit(pc, "move sr,x:$8");
			pc = rig.emit(pc, "sub b,a");
			pc = rig.emit(pc, "move sr,x:$9");
			pc = rig.emit(pc, "sub b,a");
			rig.emit(pc, "jmp $ff0");
			rig.dsp.writeReg(Reg_A, TReg56(static_cast<TReg56::MyType>(architectural(sign - 1))));
			rig.dsp.writeReg(Reg_B, TReg56(static_cast<TReg56::MyType>(architectural(1))));
			rig.start(0x240, 0x300 | (sa ? SR_SA : 0)); rig.run();
			rig.check("ADD overflow observed in block", rig.memory.get(MemArea_X, 8) & 0x42, 0x42);
			rig.check("SUB overflow observed in block", rig.memory.get(MemArea_X, 9) & 0x42, 0x42);
			rig.check("L sticks after V clears", rig.dsp.getSR().var & 0x42, 0x40);
			pc = rig.emit(0x240, "add b,a");
			pc = rig.emit(pc, "sub b,a");
			pc = rig.emit(pc, "sub b,a");
			rig.emit(pc, "jmp $ff0");
			rig.dsp.writeReg(Reg_A, TReg56(static_cast<TReg56::MyType>(architectural(sign - 1))));
			rig.dsp.writeReg(Reg_B, TReg56(static_cast<TReg56::MyType>(architectural(1))));
			rig.start(0x240, 0x300 | (sa ? SR_SA : 0)); rig.run();
			rig.check("L sticks without intermediate SR read", rig.dsp.getSR().var & 0x42, 0x40);
		}
	}

	// FM 13-9/10 and 13-174/175: shift happens before add/subtract. V includes
	// either left-shift overflow or arithmetic overflow; C is unspecified when
	// the initial left shift overflows. Expectations use architectural integers.
	void shiftedArithmetic(Rig& rig)
	{
		for(bool sa : {false, true})
		{
			const unsigned width = sa ? 40 : 56;
			const uint64_t sign = uint64_t(1) << (width - 1), mask = sign * 2 - 1;
			const std::array<uint64_t, 11> values{0, 1, 3, sign / 2 - 1, sign / 2,
				sign - 1, sign, sign + 1, sign + sign / 2, mask - 1, mask};
			auto physical = [&](uint64_t value) { return sa ? packSA(value) : value; };
			for(bool left : {false, true})
				for(bool subtract : {false, true})
					for(bool destinationB : {false, true})
						for(const auto d : values)
							for(const auto s : values)
								for(bool sticky : {false, true})
								{
									const uint64_t shifted = left ? (d * 2) & mask : (d >> 1) | (d & sign);
									const uint64_t result = (subtract ? shifted - s : shifted + s) & mask;
									const bool shiftOverflow = left && ((d ^ shifted) & sign);
									const bool overflow = shiftOverflow ||
										(((subtract ? shifted ^ s : ~(shifted ^ s)) & (shifted ^ result) & sign) != 0);
									const unsigned flags = (subtract ? s > shifted : shifted + s > mask) |
										(overflow ? CCR_V : 0) | (overflow || sticky ? CCR_L : 0) |
										(result == 0 ? CCR_Z : 0) | (result & sign ? CCR_N : 0);
									const std::string instruction = std::string(subtract ? "sub" : "add") +
										(left ? "l " : "r ") + (destinationB ? "a,b" : "b,a");
									const auto pc = rig.emit(0x250, instruction);
									rig.emit(pc, "jmp $ff0");
									rig.dsp.writeReg(Reg_A, TReg56(physical(destinationB ? s : d)));
									rig.dsp.writeReg(Reg_B, TReg56(physical(destinationB ? d : s)));
									rig.start(0x250, 0x300 | (sa ? SR_SA : 0) | CCR_V | CCR_Z | CCR_C |
										(sticky ? CCR_L : 0));
									rig.run();
									TReg56 a, b;
									rig.dsp.readReg(Reg_A, a); rig.dsp.readReg(Reg_B, b);
									rig.check(instruction.c_str(), destinationB ? b.var : a.var, physical(result));
									rig.check("shift/add source preserved", destinationB ? a.var : b.var, physical(s));
									const unsigned flagMask = shiftOverflow ? 0x4e : 0x4f;
									rig.check("shift/add C,V,Z,N,L", rig.dsp.getSR().var & flagMask, flags & flagMask);
								}
		}
	}

	// FM 4.5.3: legal offsets stay within a circular buffer, including one
	// starting at address zero. Whole-block jumps are a separate manual case.
	void moduloAddressing(Rig& rig)
	{
		for(int size : {3, 8, 32768})
		{
			unsigned block = 1;
			while(block < unsigned(size)) block *= 2;
			for(unsigned base : {0u, 0x40000u, 0x1000000u - block})
				for(int position : {0, size / 2, size - 1})
					for(int offset : {1 - size, -1, 0, 1, size - 1})
						for(bool subtract : {false, true})
						{
							const auto pc = rig.emit(0x270, subtract ? "move (r0)-n0" : "move (r0)+n0");
							rig.emit(pc, "jmp $ff0");
							rig.dsp.writeReg(Reg_M0, TReg24(size - 1));
							rig.dsp.writeReg(Reg_R0, TReg24(base + position));
							rig.dsp.writeReg(Reg_N0, TReg24(offset & 0xffffff));
							rig.start(0x270, 0x35f); rig.run();
							const int expected = (position + (subtract ? -offset : offset) + size) % size;
							rig.check("modulo legal offset", rig.dsp.regs().r[0].var, base + expected);
							rig.check("modulo preserves CCR", rig.dsp.getSR().var & 0xff, 0x5f);
						}
		}
	}

	// FM table 5-1: transfer S observes both pre-scaling accumulators.
	void transferScalingFlag(Rig& rig)
	{
		for(bool sa : {false, true})
			for(TWord scaling : {TWord(0), TWord(SR_S0), TWord(SR_S1)})
				for(uint64_t a : {0ull, 0x00100000000000ull, 0x00200000000000ull,
					0x00400000000000ull, 0x00600000000000ull, 0xffc00000000000ull})
					for(uint64_t b : {0ull, 0x00400000000000ull})
						for(bool sticky : {false, true})
							for(const char* instruction : {"move a,x0", "move b,y0", "move a,x:$7",
								"move b,y:$7", "move a,l:$7", "clr a a,x0", "clr b b,y0"})
							{
								const unsigned bit = scaling == SR_S0 ? 45 : scaling == SR_S1 ? 47 : 46;
								const bool expected = sticky || ((((a >> bit) ^ (a >> (bit - 1))) |
									((b >> bit) ^ (b >> (bit - 1)))) & 1);
								const auto pc = rig.emit(0x280, instruction); rig.emit(pc, "jmp $ff0");
								rig.dsp.writeReg(Reg_A, TReg56(a)); rig.dsp.writeReg(Reg_B, TReg56(b));
								const TWord initial = 0x300 | scaling | (sa ? SR_SA : 0) | (sticky ? CCR_S : 0);
								rig.start(0x280, initial); rig.run();
								rig.check(instruction, (rig.dsp.getSR().var & CCR_S) != 0, expected);
								rig.check("transfer preserves mode", rig.dsp.getSR().var & 0xffff00, initial & 0xffff00);
							}
	}

	void modifierHighByte(Rig& rig)
	{
		for(int size : {3, 8, 32768})
			for(TWord high : {0u, 0x550000u, 0xff0000u})
				for(unsigned setup = 0; setup < 5; ++setup)
					for(TWord base : {0u, 0x120000u})
						for(int position : {0, size - 1})
							for(bool subtract : {false, true})
								for(bool unit : {false, true})
								{
									const TWord modifier = high | (size - 1);
									const int offset = unit ? 1 : size - 1;
									auto pc = TWord(0x290);
									rig.dsp.writeReg(Reg_M0, TReg24(setup ? 0xffffff : modifier));
									rig.dsp.writeReg(Reg_M1, TReg24(modifier));
									rig.dsp.writeReg(Reg_X0, TReg24(modifier));
									rig.memory.set(MemArea_X, 8, modifier);
									if(setup == 1)
									{
										char instruction[40]; std::snprintf(instruction, sizeof(instruction), "move #>$%x,m0", modifier);
										pc = rig.emit(pc, instruction);
									}
									if(setup == 2) pc = rig.emit(pc, "move x0,m0");
									if(setup == 3) pc = rig.emit(pc, "move x:$8,m0");
									if(setup == 4) pc = rig.emit(pc, "move m1,m0");
									pc = rig.emit(pc, std::string("move (r0)") + (subtract ? "-" : "+") + (unit ? "" : "n0"));
									rig.emit(pc, "jmp $ff0");
									rig.dsp.writeReg(Reg_R0, TReg24(base + position));
									rig.dsp.writeReg(Reg_N0, TReg24(offset));
									rig.start(0x290, 0x35f); rig.run();
									const int expected = (position + (subtract ? -offset : offset) + size) % size;
									rig.check("Mn ignores high byte", rig.dsp.regs().r[0].var, base + expected);
									rig.check("Mn retains high byte", rig.dsp.regs().m[0].var, modifier);
									rig.check("Mn preserves Nn", rig.dsp.regs().n[0].var, offset);
									rig.check("Mn preserves CCR", rig.dsp.getSR().var & 0xff, 0x5f);
								}
	}

	// FM table 3-1, section 3.4.2: saturation is after arithmetic/rounding,
	// uses three specific result bits, and ignores transfer scaling for its constants.
	uint64_t saturatedResult(uint64_t physical, bool sa, bool rounded)
	{
		const unsigned pattern = ((physical >> 53) & 4) | ((physical >> 47) & 2) | ((physical >> 47) & 1);
		if(pattern == 0 || pattern == 7) return physical;
		if(pattern >= 4) return 0xff800000000000ull;
		return rounded ? (sa ? 0x007fff00000000ull : 0x007fffff000000ull) :
			(sa ? 0x007fff00ffff00ull : 0x007fffffffffffull);
	}

	void saturationArithmetic(Rig& rig)
	{
		for(bool sa : {false, true})
		{
			const unsigned width = sa ? 40 : 56;
			const uint64_t mask = (uint64_t(1) << width) - 1, fraction = uint64_t(1) << (sa ? 31 : 47);
			auto physical = [&](uint64_t value) { return sa ? packSA(value & mask) : value & mask; };
			const std::array<uint64_t, 8> values{0, 1, fraction - 1, fraction, fraction + 1,
				mask - fraction, mask - fraction + 1, mask};
			for(const char* instruction : {"add b,a", "sub b,a", "cmp b,a", "addl b,a", "addr b,a", "subl b,a", "subr b,a"})
				for(uint64_t a : values)
					for(uint64_t b : values)
						for(TWord scaling : {TWord(0), TWord(SR_S0), TWord(SR_S1)})
							for(bool sticky : {false, true})
							{
								const std::string op(instruction);
								uint64_t shifted = a;
								if(op[3] == 'l') shifted = (a * 2) & mask;
								if(op[3] == 'r') shifted = (a >> 1) | (a & (uint64_t(1) << (width - 1)));
								const uint64_t raw = physical(op[0] == 'a' ? shifted + b : shifted - b);
								const uint64_t expected = saturatedResult(raw, sa, false);
								const bool overflow = raw != expected;
								const unsigned flags = (overflow ? CCR_V : 0) | (overflow || sticky ? CCR_L : 0) |
									(expected == 0 ? CCR_Z : 0) | (expected >> 55 ? CCR_N : 0);
								const auto pc = rig.emit(0x2a0, instruction); rig.emit(pc, "jmp $ff0");
								rig.dsp.writeReg(Reg_A, TReg56(physical(a))); rig.dsp.writeReg(Reg_B, TReg56(physical(b)));
								rig.start(0x2a0, 0x300 | SR_SM | scaling | (sa ? SR_SA : 0) | CCR_V | (sticky ? CCR_L : 0));
								rig.run();
								rig.check(instruction, rig.a(), op[0] == 'c' ? physical(a) : expected);
								rig.check((op + " SM V,L,N,Z").c_str(), rig.dsp.getSR().var & 0x4e, flags);
							}

			auto round = [&](uint64_t value, TWord mode)
			{
				const uint64_t rounder = uint64_t(1) << ((sa ? 15 : 23) + (mode == SR_S0 ? 1 : mode == SR_S1 ? -1 : 0));
				value += rounder;
				if(!(value & (2 * rounder - 1))) value &= ~(2 * rounder);
				return value & ~(2 * rounder - 1) & mask;
			};
			for(const char* instruction : {"mpy x0,y0,a", "mac x0,y0,a", "mpyr x0,y0,a", "macr x0,y0,a", "rnd a"})
				for(int64_t x : {int64_t(-0x800000), int64_t(-0x400000), int64_t(0x400000), int64_t(0x7fff00)})
					for(int64_t y : {int64_t(-0x800000), int64_t(0x400000)})
						for(uint64_t a : values)
							for(TWord scaling : {TWord(0), TWord(SR_S0), TWord(SR_S1)})
							{
								const std::string op(instruction);
								const bool isRound = op == "rnd a" || op[3] == 'r';
								const int64_t product = 2 * (sa ? x / 256 : x) * (sa ? y / 256 : y);
								uint64_t value = op == "rnd a" ? a : uint64_t(product) + (op[1] == 'a' ? a : 0);
								value = isRound ? round(value, scaling) : value & mask;
								const uint64_t raw = physical(value), expected = saturatedResult(raw, sa, isRound);
								const auto pc = rig.emit(0x2a0, instruction); rig.emit(pc, "jmp $ff0");
								rig.dsp.writeReg(Reg_A, TReg56(physical(a)));
								rig.dsp.writeReg(Reg_X0, TReg24(x & 0xffffff)); rig.dsp.writeReg(Reg_Y0, TReg24(y & 0xffffff));
								rig.start(0x2a0, 0x301 | SR_SM | scaling | (sa ? SR_SA : 0)); rig.run();
								rig.check(instruction, rig.a(), expected);
								rig.check("SM multiply/round V,L", rig.dsp.getSR().var & 0x42, raw == expected ? 0 : 0x42);
								rig.check("SM multiply/round C", rig.dsp.getSR().var & 1, 1);
							}
		}
	}

	void recycledBlockFlags(Rig& rig)
	{
		if(!rig.engine) return;
		auto config = rig.dsp.getJit().getConfig();
		config.linkJitBlocks = false;
		rig.dsp.getJit().setConfig(config);
		rig.dsp.getJit().destroyAllBlocks();
		auto pc = rig.emit(0x630, "add b,a"); rig.emit(pc, "jmp $ff0");
		rig.dsp.writeReg(Reg_A, TReg56(uint64_t(1))); rig.dsp.writeReg(Reg_B, TReg56(uint64_t(1)));
		rig.start(0x630, 0x300); rig.run();
		// Free a block that writes arithmetic flags, then reuse its metadata
		// for a jump-only successor. It must no longer claim to overwrite Z/N.
		rig.dsp.getJit().destroy(0x630);
		config.linkJitBlocks = rig.engine == 2;
		rig.dsp.getJit().setConfig(config);
		rig.dsp.getJit().create(0xff0, false);
		pc = rig.emit(0x620, "sub b,a"); rig.emit(pc, "jmp $ff0");
		rig.dsp.writeReg(Reg_A, TReg56(uint64_t(1))); rig.dsp.writeReg(Reg_B, TReg56(uint64_t(1)));
		rig.start(0x620, 0x300); rig.run();
		rig.check("recycled jump preserves Z", rig.dsp.getSR().var & CCR_Z, CCR_Z);
		rig.check("recycled jump result", rig.a(), 0);
	}

	void unaryArithmetic(Rig& rig)
	{
		for(bool sa : {false, true})
			for(bool sm : {false, true})
				for(bool destinationB : {false, true})
					for(bool absolute : {false, true})
					{
						const unsigned width = sa ? 40 : 56;
						const uint64_t sign = uint64_t(1) << (width - 1), mask = sign * 2 - 1;
						const uint64_t fraction = uint64_t(1) << (sa ? 31 : 47);
						for(uint64_t value : {uint64_t(0), uint64_t(1), fraction - 1, fraction, fraction + 1,
							sign - 1, sign, sign + 1, mask - fraction, mask - fraction + 1, mask})
							for(bool sticky : {false, true})
							{
								const uint64_t integer = (absolute && !(value & sign) ? value : uint64_t(0) - value) & mask;
								const uint64_t raw = sa ? packSA(integer) : integer;
								const uint64_t expected = sm ? saturatedResult(raw, sa, false) : raw;
								const bool overflow = sm ? raw != expected : value == sign;
								const unsigned flags = CCR_C | (overflow ? CCR_V : 0) | (sticky || overflow ? CCR_L : 0) |
									(expected == 0 ? CCR_Z : 0) | (expected >> 55 ? CCR_N : 0);
								const std::string instruction = std::string(absolute ? "abs " : "neg ") + (destinationB ? "b" : "a");
								const auto pc = rig.emit(0x2c0, instruction); rig.emit(pc, "jmp $ff0");
								rig.dsp.writeReg(destinationB ? Reg_B : Reg_A,
									TReg56(sa ? packSA(value) | 0x5a0000aaull : value));
								rig.start(0x2c0, 0x300 | (sa ? SR_SA : 0) | (sm ? SR_SM : 0) | CCR_C | CCR_V |
									(sticky ? CCR_L : 0)); rig.run();
								TReg56 result; rig.dsp.readReg(destinationB ? Reg_B : Reg_A, result);
								rig.check(instruction.c_str(), result.var, expected);
								rig.check("unary C,V,L,N,Z", rig.dsp.getSR().var & 0x4f, flags);
							}
					}
	}

	void saturationExclusions(Rig& rig)
	{
		// FM 3.2.3: these operations must behave identically with SM clear/set.
		// This checks the exclusion, not the independent arithmetic of each opcode.
		for(bool sa : {false, true})
			for(const char* instruction : {"tfr b,a", "asl #1,a,a", "asr #1,a,a", "lsr #1,a",
				"and x0,a", "cmpu b,a", "mpyuu x0,y0,a", "mpysu x0,y0,a", "macsu y1,x0,a"})
				for(uint64_t input : {0x00008000000000ull, 0x01000000000000ull, 0xfe800000000000ull})
				{
					uint64_t baseline = 0;
					TWord baselineFlags = 0;
					const auto pc = rig.emit(0x2e0, instruction); rig.emit(pc, "jmp $ff0");
					for(bool sm : {false, true})
					{
						rig.dsp.writeReg(Reg_A, TReg56(input)); rig.dsp.writeReg(Reg_B, TReg56(input));
						rig.dsp.writeReg(Reg_X0, TReg24(0x800000)); rig.dsp.writeReg(Reg_Y0, TReg24(0x800000));
						rig.dsp.writeReg(Reg_Y1, TReg24(0x7fffff));
						rig.start(0x2e0, 0x300 | (sa ? SR_SA : 0) | (sm ? SR_SM : 0) | CCR_C | CCR_V);
						rig.run();
						if(!sm)
						{
							baseline = rig.a(); baselineFlags = rig.dsp.getSR().var & 0xff;
						}
						else
						{
							rig.check(instruction, rig.a(), baseline);
							rig.check("SM exclusion CCR", rig.dsp.getSR().var & 0xff, baselineFlags);
						}
					}
				}
	}

	void saturationAndTransferTables(Rig& rig)
	{
		// FM table 3-1 and section 3.4.2. Adding zero isolates the saturation
		// stage and avoids ordinary accumulator-width overflow.
		for(bool sa : {false, true})
			for(unsigned pattern = 0; pattern < 8; ++pattern)
				for(TWord scaling : {TWord(0), TWord(SR_S0), TWord(SR_S1)})
				{
					const uint64_t input = (uint64_t(pattern >> 2 ? 0xfe : 0) << 48) |
						(uint64_t((pattern >> 1) & 1) << 48) | (uint64_t(pattern & 1) << 47);
					const bool saturated = pattern != 0 && pattern != 7;
					const uint64_t expected = !saturated ? input : pattern < 4 ?
						(sa ? 0x007fff00ffff00ull : 0x007fffffffffffull) : 0xff800000000000ull;
					const auto pc = rig.emit(0x260, "add b,a"); rig.emit(pc, "jmp $ff0");
					rig.dsp.writeReg(Reg_A, TReg56(input)); rig.dsp.writeReg(Reg_B, TReg56(uint64_t(0)));
					rig.start(0x260, 0x300 | SR_SM | (sa ? SR_SA : 0) | scaling | CCR_V);
					rig.run();
					rig.check("SM table 3-1 accumulator", rig.a(), expected);
					rig.check("SM table 3-1 V,L", rig.dsp.getSR().var & (CCR_V | CCR_L),
						saturated ? CCR_V | CCR_L : 0);
				}
		// FM table 5-1: S is sticky and observes both accumulators when an
		// accumulator is moved to XDB/YDB, using the pre-scaling value.
		for(TWord scaling : {TWord(0), TWord(SR_S0), TWord(SR_S1)})
			for(uint64_t a : {uint64_t(0), 0x00400000000000ull})
				for(uint64_t b : {uint64_t(0), 0x00400000000000ull})
					for(bool sticky : {false, true})
					{
						const unsigned bit = scaling == SR_S0 ? 45 : scaling == SR_S1 ? 47 : 46;
						const bool expected = sticky || (((a >> bit) ^ (a >> (bit - 1)) |
							(b >> bit) ^ (b >> (bit - 1))) & 1);
						const auto pc = rig.emit(0x260, "move a,x0"); rig.emit(pc, "jmp $ff0");
						rig.dsp.writeReg(Reg_A, TReg56(a)); rig.dsp.writeReg(Reg_B, TReg56(b));
						rig.start(0x260, 0x300 | scaling | (sticky ? CCR_S : 0)); rig.run();
						rig.check("transfer sticky S", (rig.dsp.getSR().var & CCR_S) != 0, expected);
					}
	}

	void arithmeticBoundaries(Rig& rig)
	{
		constexpr uint64_t mask40 = 0xffffffffffull;
		constexpr uint64_t sign40 = 0x8000000000ull;
		const std::array<uint64_t, 10> values{0, 1, 0x7fff, 0x8000, 0xffff, 0x10000,
			0x7fffffff, 0x7fffffffff, 0x8000000000, 0xffffffffff};
		auto program = [&](const std::string& instruction, TWord extraSR = 0)
		{
			auto pc = rig.emit(0x200, "nop");
			pc = rig.emit(pc, "nop");
			pc = rig.emit(pc, instruction);
			rig.emit(pc, "jmp $ff0");
			rig.start(0x200, SR_SA | 0x300 | extraSR);
		};
		auto setA = [&](uint64_t value) { rig.dsp.writeReg(Reg_A, TReg56(static_cast<TReg56::MyType>(packSA(value)))); };
		auto setB = [&](uint64_t value) { rig.dsp.writeReg(Reg_B, TReg56(static_cast<TReg56::MyType>(packSA(value)))); };
		for(const auto a : values)
		{
			for(const auto b : values)
				for(bool subtract : {false, true})
				{
					setA(a); setB(b);
					program(subtract ? "sub b,a" : "add b,a"); rig.run();
					const auto result = (subtract ? a - b : a + b) & mask40;
					rig.check(subtract ? "SA SUB boundary" : "SA ADD boundary", rig.a(), packSA(result));
					const unsigned flags = (subtract ? b > a : a + b > mask40) |
						(result == 0 ? 4 : 0) | (result & sign40 ? 8 : 0);
					rig.check("SA ADD/SUB C,N,Z", rig.dsp.getSR().var & 13, flags);
				}
			for(unsigned shift : {1u, 7u, 8u, 15u, 16u, 17u, 31u, 39u, 40u, 41u, 63u})
				for(bool right : {false, true})
				{
					setA(a);
					program(std::string(right ? "asr #" : "asl #") + std::to_string(shift) + ",a,a"); rig.run();
					const int64_t signedA = (a & sign40) ? static_cast<int64_t>(a | ~mask40) : static_cast<int64_t>(a);
					const auto result = (right ? static_cast<uint64_t>(signedA >> shift) : a << shift) & mask40;
					rig.check(right ? "SA ASR boundary" : "SA ASL boundary", rig.a(), packSA(result));
					const bool carry = right ? (shift >= 40 ? (a & sign40) != 0 : ((a >> (shift - 1)) & 1))
						: shift > 40 ? false : ((a >> (40 - shift)) & 1);
					rig.check("SA shift carry", rig.dsp.getSR().var & 1, carry);
				}
		}
		// Comparison must neither alter its source nor write its temporary result to A.
		setA(0x10000); setB(1);
		auto pc = rig.emit(0x200, "cmp b,a");
		pc = rig.emit(pc, "move b0,x:$5");
		rig.emit(pc, "jmp $ff0");
		rig.start(0x200, SR_SA | 0x300); rig.run();
		rig.check("SA CMP preserves source", rig.memory.get(MemArea_X, 5), 1);
		rig.check("SA CMP preserves destination", rig.a(), packSA(0x10000));
		rig.check("SA CMP C,N,Z", rig.dsp.getSR().var & 13, 0);

		// Long transfers cover sign extension, limiting and scaling across the word gap.
		for(const auto a : values)
			for(TWord scaling : {0u, TWord(SR_S0), TWord(SR_S1)})
			{
				const int64_t signedA = (a & sign40) ? static_cast<int64_t>(a | ~mask40) : static_cast<int64_t>(a);
				const int64_t scaled = scaling == SR_S0 ? signedA >> 1 : scaling == SR_S1 ? signedA * 2 : signedA;
				// Transfer saturation retains the source sign, including scaler overflow (FM 3.1.6.2).
				const int64_t number = scaled;
				const int64_t limited = number < -0x80000000ll ? -0x80000000ll : number > 0x7fffffffll ? 0x7fffffffll : number;
				setA(a); program("move a,l:$7", scaling); rig.run();
				rig.check("SA long transfer high", rig.memory.get(MemArea_X, 7), (limited >> 16) & 0xffffff);
				rig.check("SA long transfer low", rig.memory.get(MemArea_Y, 7), limited & 0xffff);
				rig.check("SA long transfer limiter", rig.dsp.getSR().var & 0x40, number != limited ? 0x40 : 0);
			}
		for(uint64_t a : {0x7fffull, 0x8000ull, 0x8001ull, 0x18000ull, 0xfffffe8000ull, 0xffffff8000ull})
			for(TWord mode : {0u, TWord(SR_RM), TWord(SR_S0), TWord(SR_S1)})
			{
				const uint64_t rounder = mode == SR_S0 ? 0x10000 : mode == SR_S1 ? 0x4000 : 0x8000;
				const uint64_t mask = rounder * 2 - 1;
				auto result = a + rounder;
				if(!(mode & SR_RM) && !(result & mask)) result &= ~(rounder * 2);
				result &= ~mask;
				setA(a); program("rnd a", mode); rig.run();
				rig.check("SA rounding and scaling", rig.a(), packSA(result & mask40));
			}

		for(int x : {1, -1, 0x1234, -0x1234, 0x7fff, -0x8000})
			for(int y : {1, -1, 0x4567, -0x8000})
				for(bool mac : {false, true})
				{
					const uint64_t initial = mac ? 0x1234ffff : 0;
					setA(initial);
					rig.dsp.writeReg(Reg_X0, TReg24(((x & 0xffff) << 8) | 0x5a));
					rig.dsp.writeReg(Reg_Y0, TReg24(((y & 0xffff) << 8) | 0xa5));
					program(mac ? "mac x0,y0,a" : "mpy x0,y0,a"); rig.run();
					const uint64_t result = (initial + int64_t(x) * y * 2) & mask40;
					rig.check(mac ? "SA MAC signed" : "SA MPY signed", rig.a(), packSA(result));
				}
	}

	void normalization(Rig& rig)
	{
		for(uint64_t value : {0ull, 1ull, 0x123456789abcdefull & Mask56,
			0x80000000000000ull, 0xffffffffffffffull})
			for(int count : {-8, -1, 0, 1, 8})
				for(unsigned carry : {0u, 1u})
				{
					const int64_t signedValue = int64_t(value << 8) >> 8;
					const uint64_t expected = (count < 0 ? value << -count : uint64_t(signedValue >> count)) & Mask56;
					rig.dsp.writeReg(Reg_A, TReg56(static_cast<int64_t>(value)));
					rig.dsp.writeReg(Reg_X0, TReg24(count & 0xffffff));
					auto pc = rig.emit(0x880, "normf x0,a");
					rig.emit(pc, "jmp $ff0"); rig.start(0x880, 0xc00300 | carry); rig.run();
					rig.check("NORMF result", rig.a(), expected);
					rig.check("NORMF preserves MR/EMR and C", rig.dsp.getSR().var & 0xffff01, 0xc00300 | carry);
				}
		// The successor must declare its incoming carry read so block linking
		// cannot remove the ASL carry computation in the predecessor.
		rig.emit(0x8a0, "asl a"); rig.emit(0x8a1, "jmp $8b0");
		rig.emit(0x8b0, "normf x0,a"); rig.emit(0x8b1, "jmp $ff0");
		rig.dsp.writeReg(Reg_A, TReg56(int64_t(0x80000000000000ull)));
		rig.dsp.writeReg(Reg_X0, TReg24(1));
		rig.start(0x8a0, 0x300); rig.run();
		rig.check("NORMF incoming linked carry", rig.dsp.getSR().var & 1, 1);
	}

	void leadingBits(Rig& rig)
	{
		// CLB, manual p. 13-42: signed count in MSP, N/Z from that result.
		for(uint64_t value : {0ull, 1ull, 0x800000ull, 0x400000000000ull, 0x800000000000ull,
			0x80000000000000ull, 0x7fffffffffffffull, 0xffffffffffffffull})
			for(bool same : {false, true})
			{
				unsigned leading = 0;
				const bool sign = (value >> 55) & 1;
				for(int bit = 55; bit >= 0 && bool((value >> bit) & 1) == sign; --bit) ++leading;
				const int count = value ? 9 - int(leading) : 0;
				const uint64_t expected = (uint64_t(int64_t(count)) << 24) & Mask56;
				rig.dsp.writeReg(Reg_A, TReg56(static_cast<int64_t>(value)));
				rig.dsp.writeReg(Reg_B, TReg56(int64_t(0x1000000)));
				auto pc = rig.emit(0x850, same ? "clb a,a" : "clb a,b");
				if(!same) pc = rig.emit(pc, "move b,a");
				rig.emit(pc, "jmp $ff0"); rig.start(0x850, 0x3f3); rig.run();
				rig.check("CLB signed count", rig.a(), expected);
				rig.check("CLB result flags", rig.dsp.getSR().var & 0xff,
					0xf1 | (count < 0 ? 8 : 0) | (count == 0 ? 4 : 0));
			}
	}

	void fullWidthShifts(Rig& rig)
	{
		for(uint64_t value : {0ull, 1ull, 0x800000ull, 0x800000000000ull,
			0x80000000000000ull, 0x7fffffffffffffull, 0xffffffffffffffull})
			for(unsigned count : {0u, 1u, 7u, 8u, 16u, 24u, 40u})
				for(bool right : {false, true})
					for(bool dynamic : {false, true})
					{
						const int64_t signedValue = int64_t(value << 8) >> 8;
						const uint64_t expected = (right ? uint64_t(signedValue >> count) : value << count) & Mask56;
						const unsigned carry = count ? (value >> (right ? count - 1 : 56 - count)) & 1 : 0;
						rig.dsp.writeReg(Reg_A, TReg56(static_cast<int64_t>(value)));
						rig.dsp.writeReg(Reg_X0, TReg24(static_cast<int>(count)));
						auto pc = rig.emit(0x800, std::string(right ? "asr " : "asl ") +
							(dynamic ? "x0" : "#" + std::to_string(count)) + ",a,a");
						rig.emit(pc, "jmp $ff0"); rig.start(0x800, 0x301); rig.run();
						rig.check(right ? "56-bit ASR result" : "56-bit ASL result", rig.a(), expected);
						rig.check(right ? "56-bit ASR carry" : "56-bit ASL carry", rig.dsp.getSR().var & 1, carry);
					}
	}

	void repeatedCachedBody(Rig& rig)
	{
		// Compile the body first, as linked encoder branches do before reaching REP.
		// The REP and its body must still execute as one indivisible instruction pair.
		rig.emit(0x700, "rep #24");
		rig.emit(0x701, "div x0,a");
		rig.emit(0x702, "jmp $ff0");
		rig.start(0x700, 0x300);
		if(rig.engine) rig.dsp.getJit().create(0x701, false);
		rig.dsp.writeReg(Reg_A, TReg56(static_cast<TReg56::MyType>(0x595da00ull)));
		rig.dsp.writeReg(Reg_X0, TReg24(0x17a5));
		rig.run();
		rig.check("REP with previously cached DIV", rig.a(), 0x103001e3cull);

		TWord base = 0x720;
		for(bool dynamic : {false, true})
			for(unsigned count : {1u, 2u, 8u})
			{
				rig.emit(base, dynamic ? "rep x0" : "rep #" + std::to_string(count));
				rig.emit(base + 1, "asl a");
				rig.emit(base + 2, "jmp $ff0");
				rig.start(base, 0x300);
				if(rig.engine) rig.dsp.getJit().create(base + 1, false);
				rig.dsp.writeReg(Reg_A, TReg56(int64_t(1)));
				rig.dsp.writeReg(Reg_X0, TReg24(static_cast<int>(count)));
				rig.dsp.regs().lc.var = 0x1234;
				rig.run();
				rig.check("REP cached ASL count", rig.a(), uint64_t(1) << count);
				rig.check("REP restores LC", rig.dsp.regs().lc.var, 0x1234);
				// A body write must invalidate the whole pair, including cached REP.
				rig.emit(base + 1, "asr a");
				rig.start(base, 0x300); rig.run();
				rig.check("REP body write invalidates pair", rig.a(), 1);
				base += 0x10;
			}

		// Compiling the body follows a backward branch into REP while the body
		// is still being generated. It must defer that link to the dispatcher.
		rig.emit(0x790, "rep #2");
		rig.emit(0x791, "asl a");
		auto end = rig.emit(0x792, "jcc $790");
		rig.emit(end, "jmp $ff0");
		rig.start(0x790, 0x300);
		if(rig.engine) rig.dsp.getJit().create(0x791, false);
		rig.dsp.writeReg(Reg_A, TReg56(int64_t(1)));
		rig.run();
		rig.check("REP backward link to active body", rig.a(), 0);
		rig.check("REP backward link carry", rig.dsp.getSR().var & 1, 1);

		// REP must begin after an addressing-mode change used by its body.
		rig.dsp.writeReg(Reg_M0, TReg24(0xffffff));
		rig.dsp.regs().r[0].var = 6;
		auto pc = rig.emit(0x7b0, "move #3,m0");
		pc = rig.emit(pc, "rep #2");
		pc = rig.emit(pc, "move x:(r0)+,a");
		rig.emit(pc, "jmp $ff0"); rig.start(0x7b0, 0x300); rig.run();
		rig.check("REP body after modulo change", rig.dsp.regs().r[0].var, 4);

	}

	void division(Rig& rig)
	{
		// Follow the manual's algorithm in architectural integers, independently
		// of emulator helpers and native JIT accumulator representation.
		// Only valid fractional dividends 0 <= D < |S| and nonzero divisors.
		for(unsigned count : {1u, 2u, 8u, 16u, 24u})
		{
			auto pc = rig.emit(0x600, "rep #" + std::to_string(count));
			pc = rig.emit(pc, "div x0,a");
			rig.emit(pc, "jmp $ff0");
			for(int32_t divisor : {1, 2, 3, 0x12345, 0x400000, 0x7fffff, -1, -3, -0x12345, -0x800000})
			{
				const uint64_t bound = uint64_t(std::abs(divisor)) << 24;
				for(uint64_t dividend : {uint64_t(0), uint64_t(1), bound / 2, bound - 1})
					for(unsigned initial : {0u, 1u, 0xbcu, 0xfdu})
					{
						auto value = dividend;
						auto flags = initial;
						for(unsigned i = 0; i < count; ++i)
						{
							const bool oldSign = (value >> 55) & 1;
							const auto shifted = ((value << 1) | (flags & 1)) & Mask56;
							const bool overflow = oldSign != bool((shifted >> 55) & 1);
							const int64_t term = int64_t(divisor) * (int64_t(1) << 24);
							value = (shifted + (oldSign != (divisor < 0) ? uint64_t(term) : uint64_t(-term))) & Mask56;
							flags = (flags & ~3u) | (overflow ? 0x42u : 0u) | (((value >> 55) & 1) ? 0u : 1u);
						}
						rig.dsp.writeReg(Reg_A, TReg56(static_cast<TReg56::MyType>(dividend)));
						rig.dsp.writeReg(Reg_X0, TReg24(divisor & 0xffffff));
						rig.start(0x600, initial | 0x300);
						rig.run();
						rig.check("DIV accumulator", rig.a() & Mask56, value);
						rig.check("DIV CCR", rig.dsp.getSR().var & 0xff, flags);
					}
			}
		}
	}
}

int main(int argc, char** argv)
{
	const bool tablesOnly = argc == 2 && std::string(argv[1]) == "--tables";
	if(argc > 1 && !tablesOnly)
	{
		std::fprintf(stderr, "Usage: %s [--tables]\n", argv[0]);
		return 2;
	}
	unsigned failed = 0;
	for(int engine = 0; engine < (dsp56k::g_useJIT ? 3 : 1); ++engine)
	{
		Rig rig(engine);
		if(tablesOnly)
		{
			saturationAndTransferTables(rig);
			std::printf("%s: %u table checks, %u failures\n", rig.name(), rig.checks, rig.failures);
			failed += rig.failures;
			continue;
		}
		normalization(rig);
		leadingBits(rig);
		fullWidthShifts(rig);
		repeatedCachedBody(rig);
		division(rig);
		transfersAndArithmetic(rig);
		arithmeticBoundaries(rig);
		arithmeticOverflow(rig);
		shiftedArithmetic(rig);
		moduloAddressing(rig);
		transferScalingFlag(rig);
		modifierHighByte(rig);
		saturationArithmetic(rig);
		recycledBlockFlags(rig);
		unaryArithmetic(rig);
		saturationAndTransferTables(rig);
		saturationExclusions(rig);
		logicalWordOperations(rig);
		partialConditionCodeWrites(rig);
		serialClock(rig);
		std::printf("%s: %u checks, %u failures\n", rig.name(), rig.checks, rig.failures);
		failed += rig.failures;
	}
	return failed ? 1 : 0;
}
