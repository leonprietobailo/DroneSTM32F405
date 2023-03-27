
int16_t acc_z_corr, acc_z_old;


void compute_hover_throttle(void) {
  acc_z_corr = acc_z - 4096;
  if (acc_z != 0){
    if(acc_z_corr * acc_z_old < 0) {      // If result is negative there has been a change of sign 
      hover_throttle = throttle;        // When we are around acc_z = 0 that means the net forces on the drone are 0. This happens when the thrust provided by the mottors = weight or when the drone is on the ground normal = weight.

      voltage_str[n_str] = battery_voltage*100;
      throttle_str[n_str] = throttle;
      n_str++;

      if(n_str == length_str){
        n_str = 0;
      }
//        Serial.print(n_str);
//        Serial.print('\t');
//        Serial.print(voltage_str[n_str-1]);
//        Serial.print('\t');
//        Serial.print(throttle_str[n_str-1]);
//        Serial.print('\n');     
    }
    acc_z_old = acc_z_corr;
  }
}
