#include "hdb3.h"
#include <string.h>
#include <stdio.h>
const int testcases = 3;
constexpr const char *testdata[]= {
	"10000110",
	"101000001100001100000001",
	"1010000100001100001110000111100001010000"
};

#define GET_DATA(i) \
int get_data_ ## i () { \
	static int pos = 0; \
	if (pos > strlen(testdata[i])) return pos++ & 1; \
	return testdata[i][pos++] == '1'; \
}

GET_DATA(0)
GET_DATA(1)
GET_DATA(2)

const char *expected[] = {
	"+000+−+0",
	"+0−+00+0−+−00−+−+00+000−",
	"+0−+00+−000−+−+00+−+−000−+−+−+00+-0+-00-"
};

#define TEST_FUNCTION(N) \
void test_function_ ## N() {   \
	printf("%s\n", testdata[N]); \
	for (int j = 0; j < strlen(testdata[N]); j++) { \
		int hdb3 = get_next_HDB3<get_data_ ## N>(); \
		printf("%c", hdb3 > 0 ? '+' : (hdb3 < 0 ? '-' : '0')); \
	} \
	printf("\n%s\n", expected[N]);  \
}
TEST_FUNCTION(0)
TEST_FUNCTION(1)
TEST_FUNCTION(2)
int main() {
	test_function_0();
	test_function_1();
	test_function_2();
	
}