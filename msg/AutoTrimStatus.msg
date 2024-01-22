uint64 timestamp                # time since system start (microseconds)
uint64 timestamp_sample

float32[3] trim_estimate
float32[3] trim_estimate_var
float32[3] trim_test
float32[3] trim_test_var
float32[3] trim_validated

uint8 STATE_IDLE = 0
uint8 STATE_SAMPLING = 1
uint8 STATE_SAMPLING_TEST = 2
uint8 STATE_VERIFICATION = 3
uint8 STATE_COMPLETE = 4
uint8 STATE_FAIL = 5

uint8 state
