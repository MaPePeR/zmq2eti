#pragma once
#include <assert.h>

class HDB3Context {
public:
	inline bool hasOutput() {
		return output_pattern_pos != 0;
	}
	inline bool inputData(bool bit) {
		assert(!this->hasOutput());
		if (bit) {
			output_pattern = last_one_negative ? ZZZP : ZZZN;
			output_pattern_pos = 1 << current_zero_count;
			current_zero_count = 0;
			odd_pairs = !odd_pairs;
			return true;
		} else {
			current_zero_count += 1;
			if (current_zero_count == 4) {
				if (odd_pairs) {
					output_pattern = last_one_negative ? ZZZN : ZZZP;
					output_pattern_pos = 1 << 3;
				} else {
					output_pattern = last_one_negative ? PZZP : NZZN;
					output_pattern_pos = 1 << 3;
				}
				current_zero_count = 0;
				odd_pairs = 0;
				return true;
			}
		}
		return false;
	}
	inline int getOutput() {
		assert(this->hasOutput());
		int isnz = (output_pattern & output_pattern_pos) > 0;
		output_pattern_pos >>= 1;
		if (isnz) {
			last_one_negative = (output_pattern & 0x10) > 0;
			return last_one_negative ? -1 : 1;
		} else {
			return 0;
		}
	}
	inline void close() {

	}
protected:
	int output_pattern_pos = 0;
	enum {ZZZZ=0x00,PZZP=0x09,NZZN=0x19,ZZZP=0x01,ZZZN=0x11} output_pattern = ZZZZ;
	int current_zero_count = 0;
	int last_one_negative = 1;
	int odd_pairs = 0;
};

template <int (*func)()>
int get_next_HDB3() {
	static int output_pattern_pos = 0;
	static enum {ZZZZ=0x00,PZZP=0x09,NZZN=0x19,ZZZP=0x01,ZZZN=0x11} output_pattern = ZZZZ;
	static int current_zero_count = 0;
	static int last_one_negative = 1;
	static int odd_pairs = 0;

	while (output_pattern_pos == 0) {
		int next_bit = func();
		if (next_bit) {
			output_pattern = last_one_negative ? ZZZP : ZZZN;
			output_pattern_pos = 1<<current_zero_count;
			current_zero_count = 0;
			odd_pairs = !odd_pairs;
		} else {
			current_zero_count += 1;
			if (current_zero_count == 4) {
				if (odd_pairs) {
					output_pattern = last_one_negative ? ZZZN : ZZZP;
					output_pattern_pos = 1<<3;
				} else {
					output_pattern = last_one_negative ? PZZP : NZZN;
					output_pattern_pos = 1<<3;
				}
				current_zero_count = 0;
				odd_pairs = 0;
			}
		}
	}
	assert (output_pattern_pos > 0);
	int isnz = (output_pattern & output_pattern_pos) > 0;
	output_pattern_pos >>= 1;
	if (isnz) {
		last_one_negative = (output_pattern & 0x10) > 0;
		return last_one_negative ? -1 : 1;
	} else {
		return 0;
	}
}