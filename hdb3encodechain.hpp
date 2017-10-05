#include <stdint.h> //for uint32_t
#include <assert.h>
#include <stdio.h>

template<typename T>
class ETIFrameConsumer {
public:
	T next;

	void consumeETIFrame(uint8_t *framedata, int frame_length) {
		assert(frame_length >= 0);
		int i = 0;
		while (i < frame_length) {
			uint8_t b = framedata[i];
			next.consumeBit((b & (1 << 7)) > 0);
			next.consumeBit((b & (1 << 6)) > 0);
			next.consumeBit((b & (1 << 5)) > 0);
			next.consumeBit((b & (1 << 4)) > 0);
		
			next.consumeBit((b & (1 << 3)) > 0);
			next.consumeBit((b & (1 << 2)) > 0);
			next.consumeBit((b & (1 << 1)) > 0);
			next.consumeBit((b & (1 << 0)) > 0);
			++i;
		}
		if (frame_length < 6144) {
			next.consumePaddingBytes(6144 - frame_length);
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
	void consumePaddingBytes(int nBytes) {
		if (nBytes >= 1) {
			//0x55
			consumeBit(false);
			consumeBit(true);
			consumeBit(false);
			consumeBit(true);

			consumeBit(false);
			consumeBit(true);
			consumeBit(false);
			consumeBit(true);
			next.consumePaddingBytes(nBytes - 1, last_one_negative);
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

	void consumePaddingBytes(int nBytes, bool last_one_negative) {
		assert(count == 0 || count == 8);
		if (nBytes >= 1) {
			//Padding is 0x         5         5 
			//           0b  0 1  0 1  0 1  0 1 
			//               0 +  0 -  0 +  0 - 
			//           p: 0001 0000 0001 0000
			//           m: 0000 0001 0000 0001
			if (count == 8) {
				current_out_p <<= 16;
				current_out_m <<= 16;
				if (last_one_negative) {

					current_out_p |= 0x1010; //0b0100 0100
					current_out_m |= 0x0101; //0b0001 0001
					assert(current_out_p == 0x10101010);
					assert(current_out_m == 0x01010101);
				} else{
					current_out_p |= 0x0101; //0b0001 0001
					current_out_m |= 0x1010; //0b0100 0100
					assert(current_out_p == 0x01010101);
					assert(current_out_m == 0x10101010);
				}
				next.consumeEncodedHdb3(current_out_p, current_out_m);
				count = 0;
				current_out_m = current_out_p = 0;
				assert((nBytes - 1) % 2 == 0);
				next.consumePadding((nBytes - 1)/2, current_out_p, current_out_m);
			} else {
				assert(nBytes % 2 == 0);
				if (last_one_negative) {
					next.consumePadding(nBytes/2, 0x10101010, 0x01010101);
				} else {
					next.consumePadding(nBytes/2, 0x01010101, 0x10101010);
				}
			}

		}
	}
};

