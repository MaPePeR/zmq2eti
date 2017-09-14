import zmq

import time
import socket
import struct

class TokenBucket(object):
    """An implementation of the token bucket algorithm from http://code.activestate.com/recipes/511490-implementation-of-the-token-bucket-algorithm/

    >>> bucket = TokenBucket(80, 0.5)
    >>> print bucket.consume(10)
    True
    >>> print bucket.consume(90)
    False
    """
    def __init__(self, tokens, fill_rate):
        """tokens is the total tokens in the bucket. fill_rate is the
        rate in tokens/second that the bucket will be refilled."""
        self.capacity = float(tokens)
        self._tokens = float(0)
        self.fill_rate = float(fill_rate)
        self.timestamp = time.time()
        self.overflowcount = 0;

    def consume(self, tokens):
        """Consume tokens from the bucket. Returns True if there were
        sufficient tokens otherwise False."""
        if tokens <= self.tokens:
            self._tokens -= tokens
        else:
            return False
        return True

    def get_tokens(self):
        now = time.time()
        if self._tokens < self.capacity:
            delta = self.fill_rate * (now - self.timestamp)
            if self.capacity < self._tokens + delta:
                print("Bucket overflow! %d: %d" % (self.overflowcount, self.capacity -( self._tokens + delta)))
                self.overflowcount += 1
            self._tokens = min(self.capacity, self._tokens + delta)
        self.timestamp = now
        return self._tokens
    tokens = property(get_tokens)


NUM_FRAMES_PER_ZMQ_MESSAGE = 4
PACKET_SIZE = 6144 * NUM_FRAMES_PER_ZMQ_MESSAGE #Bytes
#DATA_RATE = 256000 #Byte/s
DATA_RATE = 256*1000 #Byte/s
if __name__ == "__main__":
    data = bytearray()
    for i in range(6144 * 4):
        #data.append(0b11111111)
        data.append(0b00000000)
    tb = TokenBucket(PACKET_SIZE * 4, DATA_RATE)
    # uint32_t version;uint16_t buflen[4]; uint8_t buf[NUM_FRAMES_PER_ZMQ_MESSAGE*6144];
    

    print("Creating context...")
    zmq_ctx = zmq.Context.instance()
    print("Creating socket")
    s = zmq_ctx.socket(zmq.PUB)
    #port = s.bind_to_random_port("tcp://*")
    s.bind("tcp://*:50473")
    print("Publishing on port %d" % 50473)
    i = 0
    try:
        while True:
            if tb.consume(PACKET_SIZE): 
                packet = struct.pack("IHHHH" + str(len(data)) + "s", i % 2048, 1, 1, 1, 1, data)
                i = (i + 1) % 2048
                s.send(packet)
            else:
                time.sleep(24*10**-3 / 4)
    except Exception as e:
        print(e)
        s.close()