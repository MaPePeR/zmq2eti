#pragma once 

#define NUM_FRAMES_PER_ZMQ_MESSAGE 4
/* A concatenation of four ETI frames,
 * whose maximal size is 6144.
 *
 * If we transmit four frames in one zmq message,
 * we do not risk breaking ETI vs. transmission frame
 * phase.
 *
 * The frames are concatenated in buf, and
 * their sizes is given in the buflen array.
 *
 * Most of the time, the buf will not be completely
 * filled
 */
struct zmq_dab_message_t
{
    zmq_dab_message_t()
    {
        /* set buf lengths to invalid */
        buflen[0] = -1;
        buflen[1] = -1;
        buflen[2] = -1;
        buflen[3] = -1;

        version = 1;
    }
    uint32_t version;
    int16_t buflen[NUM_FRAMES_PER_ZMQ_MESSAGE];
    /* The head stops here. Use the macro below to calculate
     * the head size.
     */

    uint8_t  buf[NUM_FRAMES_PER_ZMQ_MESSAGE*6144];

    /* The packet is then followed with metadata appended to it,
     * according to dabOutput/metadata.h
     */
};

#define ZMQ_DAB_MESSAGE_HEAD_LENGTH (4 + NUM_FRAMES_PER_ZMQ_MESSAGE*2)