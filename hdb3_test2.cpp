#include <string.h>
#include <stdio.h>
#include "hdb3encodechain.hpp"

const int testcases = 3;

template <typename T>
class TestStringConsumer {
public:
	T next;
	void consumeString(const char* s) {
		int length = strlen(s);
		for (int j = 0; j < length; j++) {
			next.consumeBit(s[j] == '1');
		}
	}
};

class HDB3TestConsumer {
	
	int index;
public:
	char testOutput[100];
	void consumeTertiary(int tertiary) {
		if (tertiary > 0)
			testOutput[index] = '+';
		else if (tertiary < 0)
			testOutput[index] = '-';
		else if (tertiary == 0) 
			testOutput[index] = '0';
		index++;
	}
	void reset() {
		index = 0;
		memset(testOutput, 0, 100 * sizeof(char));
	}

};

constexpr const char *testdata[]= {
	"10000110",
	"101000001100001100000001",
	"1010000100001100001110000111100001010000"
};

const char *expected[] = {
	"+000+−+0",
	"+0−+00+0−+−00−+−+00+000−",
	"+0−+00+−000−+−+00+−+−000−+−+−+00+-0+-00-"
};

int main() {
	for (int i = 0; i < testcases; i++) {
		printf("%s\n", testdata[i]);
		TestStringConsumer<ETIFrameBitConsumer<HDB3TestConsumer>> context;
		context.next.next.reset();
		context.consumeString(testdata[i]);
		printf("%s", context.next.next.testOutput);
		printf("\n%s\n", expected[i]);
		context.next.next.reset();

	}	
}