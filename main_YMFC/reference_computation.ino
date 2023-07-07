void reference_computation(){
	ref_mode_management();
	ref_gen();
}

void ref_mode_management(){
	if (remote_channel[3] < 1100 && remote_channel[4] < 1100) fm = FM_mounting;
	if (fm == FM_mounting && remote_channel[3] < 1100 && remote_channel[4] > 1450) fm = FM_stable;
	if (fm >=2 && remote_channel[3] < 1050 && remote_channel[4] > 1950) fm = FM_disabled;
	if (fm >=2 && remote_channel[6] < 1500) fm = FM_stable;
	if (fm >=2 && remote_channel[6] >= 1500) fm = FM_alt_hold;
}

void ref_gen(){

	if(fm == FM_disabled) led_on();

	if(fm == FM_mounting){
		throttle = 950;
		led_off();
		angle_pitch = angle_pitch_acc;
	    angle_roll = angle_roll_acc;
	    pid_i_mem_roll = 0;
	    pid_last_roll_d_error = 0;
	    pid_i_mem_pitch = 0;
	    pid_last_pitch_d_error = 0;
	    pid_i_mem_yaw = 0;
	    pid_last_yaw_d_error = 0;
	}

	if(fm == FM_stable){
		throttle = remote_channel[3];
	    pid_i_mem_altitude = 0;
	    pid_last_altitude_d_error = 0;
	    pid_altitude_setpoint = actual_pressure;
	}

	if(fm == FM_alt_hold){
    	throttle = -63.4 * battery_voltage + 2203;

    	if(remote_channel[3] > 1750) pid_altitude_setpoint -= 1.0 / 250.0;
    	else if(remote_channel[3] < 1250 && distance > 50) pid_altitude_setpoint += 1.0 / 250.0;
	}
}
