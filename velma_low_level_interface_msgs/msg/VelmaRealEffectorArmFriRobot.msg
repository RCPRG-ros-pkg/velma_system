# /** power state of drives (bitfield) */
# fri_uint16_t power;
uint16 power

# /** selected control strategy (FRI_CTRL) */
# fri_uint16_t control;
uint16 control

# /** drive errors (leads to power off) */
# fri_uint16_t error;
uint16 error

# /** drive warnings */
# fri_uint16_t warning;
uint16 warning

# /** temperature of drives */
# fri_float_t temperature[LBR_MNJ];
float32[7] temperature

