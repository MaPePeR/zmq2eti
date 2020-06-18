"""
Input	10000110
AMI		+0000−+0
HDB3	+000V−+0
		+000+−+0
Input	101000001100001100000001
AMI		+0−00000+−0000+−0000000+
HDB3	+0−B00V0−+B00V+−B00V000−
		+0−+00+0−+−00−+−+00+000−
Input	1010000100001100001110000111100001010000
AMI		+0−0000+0000−+0000−+−0000+−+−0000+0−0000
HDB3	+0−B00V−000V+−B00V−+−000V+−+−B00V+0−B00V
		+0−+00+−000−+−+00+−+−000−+−+−+00+-0+-00-
"""

inputs = {
	"10000110": "+000+−+0",
	"101000001100001100000001": "+0−+00+0−+−00−+−+00+000−",
	"1010000100001100001110000111100001010000" : "+0−+00+−000−+−+00+−+−000−+−+−+00+-0+-00-"
}

class HDB3:
	def __init__(self, data):
		self.data = data
		self.last_one_negative = True
		self.odd_pairs = False
	def generate(self):
		count = 0
		for bit in self.data:
			if bit:
				if count > 0:
					for i in range(count):
						yield '0'
					count = 0
				self.last_one_negative = not self.last_one_negative
				self.odd_pairs = not self.odd_pairs
				yield '-' if self.last_one_negative else '+'
			else:
				count += 1
				if (count == 4):
					if self.odd_pairs:
						yield '0'
						yield '0'
						yield '0'
						yield '-' if self.last_one_negative else '+'
					else:
						yield '-' if not self.last_one_negative else '+'
						yield '0'
						yield '0'
						yield '-' if not self.last_one_negative else '+'
						self.last_one_negative = not self.last_one_negative
					self.odd_pairs = False
					count = 0
		for i in range(count):
			yield '0'
				

for d,expected in inputs.items():
	h = HDB3(map(lambda c: c == '1', d))
	out = ''.join(h.generate())
	print(d)
	print(out)
	print(expected)
