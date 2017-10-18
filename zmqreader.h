#include <sys/time.h>
#include <zmq.h>
#include <thread>
#include <assert.h>
#include "odr-dabOutput.h"

class ZMQReader {
	void *zmq_ctx;
	static constexpr int BUFFER_COUNT = 40;
	const char *zmq_endpoint;
	void *zmq_sock_odr_sub;
	void *zmq_sock_inproc_thread;
	void *zmq_sock_inproc_main;
	std::thread thread;
	zmq_dab_message_t zmq_msg_buffer[BUFFER_COUNT];
	int buffer_read_pos = 0;
	int buffer_write_pos = -1;
public:
	volatile bool keep_running = true;
	ZMQReader(void *zmq_ctx_, const char *zmq_endpoint_) :  zmq_ctx(zmq_ctx_), zmq_endpoint(zmq_endpoint_) {
		zmq_sock_inproc_main = zmq_socket(zmq_ctx, ZMQ_PAIR);
		if (!zmq_sock_inproc_main) {
			fprintf(stderr, "%s\n", "Error creating zmq_socket.");
			exit(1);
		}
		int val = 2;
		int rc = zmq_setsockopt(zmq_sock_inproc_main, ZMQ_RCVHWM, &val, sizeof(int));
		assert (rc == 0);
		if (zmq_bind(zmq_sock_inproc_main, "inproc://#1") < 0) {
			fprintf(stderr, "%s\n", "Failed to connect to inproc.");
			exit(1);
		}
		thread = std::thread([this] { this->read_thread(); });
	}

	inline int bufferSize() {
		return (BUFFER_COUNT + buffer_write_pos - buffer_read_pos) % BUFFER_COUNT;
	}

	void read_thread() {
		zmq_sock_odr_sub = zmq_socket(zmq_ctx, ZMQ_SUB);
		if (!zmq_sock_odr_sub) {
			fprintf(stderr, "%s\n", "Error creating zmq_socket.");
			exit(1);
		}
		if (zmq_connect(zmq_sock_odr_sub, zmq_endpoint) < 0) {
			fprintf(stderr, "Failed to connect to zmq-endpoint %s\n", zmq_endpoint);
			exit(1);
		}
		int rc = zmq_setsockopt (zmq_sock_odr_sub, ZMQ_SUBSCRIBE, "", 0);
		assert (rc == 0);

		zmq_sock_inproc_thread = zmq_socket(zmq_ctx, ZMQ_PAIR);
		if (!zmq_sock_inproc_thread) {
			fprintf(stderr, "%s\n", "Error creating zmq_socket.");
			exit(1);
		}

		int val = 2;
		rc = zmq_setsockopt(zmq_sock_inproc_thread, ZMQ_SNDHWM, &val, sizeof(int));
		assert (rc == 0);

		if (zmq_connect(zmq_sock_inproc_thread, "inproc://#1") < 0) {
			fprintf(stderr, "%s\n", "Failed to connect to inproc.");
			exit(1);
		}


		if (buffer_write_pos < 0) {
			//Prebuffering
			struct timeval start_time;
			uint64_t frames = 0;
			gettimeofday(&start_time, NULL);
			for (buffer_write_pos = 0; buffer_write_pos < BUFFER_COUNT / 2 && keep_running; buffer_write_pos++) {
				int rc = zmq_recv(zmq_sock_odr_sub, &zmq_msg_buffer[buffer_write_pos], sizeof(*zmq_msg_buffer), 0);
				assert(rc > 0 && (size_t)rc <= sizeof(*zmq_msg_buffer));
				frames += NUM_FRAMES_PER_ZMQ_MESSAGE;
			}
		}
		printf("Prebuffering done... Have %d Messages\n", bufferSize());

		zmq_pollitem_t poll[2];
		poll[0].socket = zmq_sock_odr_sub;
		poll[0].fd = 0;
		poll[0].events = ZMQ_POLLIN;
		poll[0].revents = 0;
		poll[1].socket = zmq_sock_inproc_thread;
		poll[1].fd = 0;
		poll[1].events = ZMQ_POLLOUT;
		poll[1].revents = 0;
		int poll_rc = 1;
		while(poll_rc >= 0 && keep_running) {
			poll_rc = zmq_poll (poll, 2, 500);
			if (poll_rc > 0) {
				//Something changed
				if (poll[0].revents & ZMQ_POLLIN) {
					poll[0].revents = 0;
					assert(buffer_write_pos != (BUFFER_COUNT + buffer_read_pos - 1) % BUFFER_COUNT); //Buffer is not full!
					//We received something
					int rc = zmq_recv(zmq_sock_odr_sub, &zmq_msg_buffer[buffer_write_pos], sizeof(*zmq_msg_buffer), 0);
					assert(rc > 0 && (size_t)rc <= sizeof(*zmq_msg_buffer));
					buffer_write_pos = (buffer_write_pos + 1) % BUFFER_COUNT;
					if (buffer_write_pos == (BUFFER_COUNT + buffer_read_pos - 1) % BUFFER_COUNT) {
						//Our ringbuffer is full.
						printf("Buffer full\n");
						poll[0].events = 0;
					}
				}
				if (poll[1].revents & ZMQ_POLLOUT) {
					poll[1].revents = 0;
					//We can send something
					zmq_dab_message_t* output = &zmq_msg_buffer[buffer_read_pos];
					if (buffer_read_pos == buffer_write_pos) {
						//TODO: can send, but have nothing to send.
						int rc = zmq_recv(zmq_sock_odr_sub, &zmq_msg_buffer[buffer_write_pos], sizeof(*zmq_msg_buffer), 0);
						assert(rc > 0 && (size_t)rc <= sizeof(*zmq_msg_buffer));
						rc = zmq_send(zmq_sock_inproc_thread, &output, sizeof(zmq_dab_message_t *), 0);
						assert(rc > 0 && (size_t)rc == sizeof(zmq_dab_message_t *));
						buffer_write_pos = (buffer_write_pos + 1) % BUFFER_COUNT;
						buffer_read_pos = (buffer_read_pos + 1) % BUFFER_COUNT;						
					} else {
						int rc = zmq_send(zmq_sock_inproc_thread, &output, sizeof(zmq_dab_message_t *), 0);
						assert(rc > 0 && (size_t)rc == sizeof(zmq_dab_message_t *));
						buffer_read_pos = (buffer_read_pos + 1) % BUFFER_COUNT;

						if (buffer_write_pos != (BUFFER_COUNT + buffer_read_pos - 1) % BUFFER_COUNT) {
							//Our ringbuffer is not full: enable polling for input
							poll[0].events = ZMQ_POLLIN;
						}
					}
				}
			}
		}
	}

	inline void tryReceive() {

	}

	zmq_dab_message_t *getNextMessage() {

		zmq_dab_message_t *output;
		zmq_recv(zmq_sock_inproc_main, &output, sizeof(zmq_dab_message_t*), 0);
		return output;
	}

	~ZMQReader() {

		printf("Waiting for thread to stop...");
		keep_running = false;
		thread.join();
		if (zmq_sock_odr_sub) {
			printf("Closing ZMQ-Socket\n");
			zmq_close(zmq_sock_odr_sub);
		}

		if (zmq_sock_inproc_thread) {
			printf("Closing ZMQ-Socket\n");
			zmq_close(zmq_sock_inproc_thread);
		}

		if (zmq_sock_inproc_main) {
			printf("Closing ZMQ-Socket\n");
			zmq_close(zmq_sock_inproc_main);
		}
	}
};