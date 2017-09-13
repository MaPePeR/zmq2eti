#include "hdb3.h"
#include <string.h>
#include <stdio.h>
const int testcases = 3;
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
		HDB3Context ctx = HDB3Context();
		for (int j = 0; j < strlen(testdata[i]); j++) {
			ctx.inputData(testdata[i][j] == '1');
			while(ctx.hasOutput()) {
				int hdb3 = ctx.getOutput();
				printf("%c", hdb3 > 0 ? '+' : (hdb3 < 0 ? '-' : '0'));
			}
		}
		ctx.close();
		while(ctx.hasOutput()) {
			int hdb3 = ctx.getOutput();
			printf("%c", hdb3 > 0 ? '+' : (hdb3 < 0 ? '-' : '0'));
		}
		printf("\n%s\n", expected[i]);
	}	
}