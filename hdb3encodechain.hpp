#include <stdint.h> //for uint32_t

template<typename T>
class ETIFrameConsumer {
public:
	T next;

	void consumeETIFrame(uint8_t *framedata, int frame_length) {
		int i = 0;
		while (i < frame_length) {
			uint8_t b = framedata[i];
			next.consumeBit((b & (1 << 0)) > 0);
			next.consumeBit((b & (1 << 1)) > 0);
			next.consumeBit((b & (1 << 2)) > 0);
			next.consumeBit((b & (1 << 3)) > 0);
			
			next.consumeBit((b & (1 << 4)) > 0);
			next.consumeBit((b & (1 << 5)) > 0);
			next.consumeBit((b & (1 << 6)) > 0);
			next.consumeBit((b & (1 << 7)) > 0);
			++i;
		}
		while (i < 6144) {
			next.consumeBit((0x55 & (1 << 0)) > 0);
			next.consumeBit((0x55 & (1 << 1)) > 0);
			next.consumeBit((0x55 & (1 << 2)) > 0);
			next.consumeBit((0x55 & (1 << 3)) > 0);
			
			next.consumeBit((0x55 & (1 << 4)) > 0);
			next.consumeBit((0x55 & (1 << 5)) > 0);
			next.consumeBit((0x55 & (1 << 6)) > 0);
			next.consumeBit((0x55 & (1 << 7)) > 0);
			++i;
		}
	}
};

template<typename T>
class ETIFrameBitConsumer {
protected:
	int current_zero_count = 0;
	bool last_one_negative = true;
	bool odd_pairs = false;
public:
	T next;
	void consumeBit(bool bit) {
		if (bit) {
			switch(current_zero_count) {
				case 3: next.consumeTertiary(0);
				case 2: next.consumeTertiary(0);
				case 1: next.consumeTertiary(0);
				case 0: current_zero_count = 0;
				default: break;
			}
			if (last_one_negative) {
				next.consumeTertiary(1);
				last_one_negative = false;
			} else {
				next.consumeTertiary(-1);
				last_one_negative = true;
			}
			odd_pairs = !odd_pairs;
		} else {
			current_zero_count += 1;
			if (current_zero_count == 4) {
				if (odd_pairs) {
					next.consumeTertiary(0);
					next.consumeTertiary(0);
					next.consumeTertiary(0);
					next.consumeTertiary(last_one_negative ? -1 : 1);
					odd_pairs = false;
				} else {
					next.consumeTertiary(last_one_negative ? 1 : -1); //This is not a violation
					next.consumeTertiary(0);
					next.consumeTertiary(0);
					next.consumeTertiary(last_one_negative ? 1 : -1); //This is a violation.
					last_one_negative = !last_one_negative;
				}
				current_zero_count = 0;
			}
		}
	}
};

template<typename T>
class HDB3TertiaryConsumer {
protected:
	int count = 0;
	uint32_t current_out_p = 0;
	uint32_t current_out_m = 0;
public:
	T next;
	void consumeTertiary(int tertiary) {
		current_out_p <<= 2;
		current_out_m <<= 2;
		if (tertiary > 0) {
			current_out_p |= 1;
		} else if (tertiary < 0) {
			current_out_m |= 1;
		}
		count += 1;
		if (count == 16) {
			count = 0;
			next.consumeEncodedHdb3(current_out_p, current_out_m);
			current_out_m = current_out_p = 0;
		}
		
	}
};

class EncodedHDB3WordConsumer {
public:
	void consumeEncodedHdb3(uint32_t out_p, uint32_t out_m) {

	}
};

typedef ETIFrameConsumer<ETIFrameBitConsumer<HDB3TertiaryConsumer<EncodedHDB3WordConsumer>>> EncodingChain;