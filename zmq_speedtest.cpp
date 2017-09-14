#include <zmq.h>
#include "odr-dabOutput.h"
#include <time.h>
#include <stdlib.h>
#include <assert.h>

#define ZMQ_MESSAGE_BUFFER_SIZE (1)
void *zmq_ctx;
void *zmq_sock;
struct zmq_dab_message_t zmq_msg_buffer[ZMQ_MESSAGE_BUFFER_SIZE];
int main(int argc, const char* argv[]) {
	zmq_ctx = zmq_init(1);
	if (!zmq_ctx) {
		fprintf(stderr, "%s\n", "Error initializing zmq_ctx.");
		exit(1);
	}
	zmq_sock = zmq_socket(zmq_ctx, ZMQ_SUB);
	if (!zmq_sock) {
		fprintf(stderr, "%s\n", "Error creating zmq_socket.");
		exit(1);
	}
	if (zmq_connect(zmq_sock, argv[1]) < 0) {
		fprintf(stderr, "Failed to connect to zmq-endpoint %s\n", argv[1]);
		exit(1);
	}
	int rc = zmq_setsockopt (zmq_sock, ZMQ_SUBSCRIBE, "", 0);
	assert (rc == 0);
	size_t bytes = 0;
	time_t start_time = time(0);
	while(1) {
		zmq_recv(zmq_sock, &zmq_msg_buffer[0], sizeof(*zmq_msg_buffer), 0);
		bytes += 6144 * NUM_FRAMES_PER_ZMQ_MESSAGE;
		printf("Avg: %10.2f\n", bytes * 1.0f / (time(0) - start_time));
	}
}
