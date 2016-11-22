# /** current system time in seconds */
# fri_float_t timestamp;
float32 timestamp

# /** state of interface (monitor or command FRI_STATE) */
# fri_uint16_t state;
uint16 state

# /** quality of communication (FRI_QUALITY) */
# fri_uint16_t quality;
uint16 quality

# /** configured sample time for sent measurment packets */
# fri_float_t desiredMsrSampleTime;
float32 desiredMsrSampleTime
    
# /** configured sample time for received command packets */
# fri_float_t desiredCmdSampleTime;
float32 desiredCmdSampleTime

# /** configured safety limits */
# fri_float_t safetyLimits;
float32 safetyLimits

############################################
#
# /** communication statistics */
# tFriIntfStatistics stat;
#
#
# /** avg. answer rate (should be close to 1) */
# fri_float_t answerRate;
float32 answerRate

# /** avg. latency in seconds (should be as small as possible) */
# fri_float_t latency;
float32 latency

# /** avg. jitter in seconds (should be as small as possible) */
# fri_float_t jitter;
float32 jitter

# /** avg. missing answer packets (should be as small as possible) */
# fri_float_t missRate;
float32 missRate

# /** absolute missing answer packets */
# fri_uint32_t missCounter;
uint32 missCounter

