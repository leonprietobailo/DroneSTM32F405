bool timer_started = false;
bool collision_enable = false;
bool flying = false;

void manage_throttle() {
	
	hoverThrottle = -63.4 * battery_voltage + 2183;

 if (distance > 50){
  flying = true;
 }

 if (flying)
 {
  if (distance > 60){
	  collision_enable = false;
	}
  else{
    collision_enable = true;
  }
 }
	
  if (hoverThrottle > Mando_canal[3] && collision_enable){
  	throttle = hoverThrottle;
  	if (Mando_canal[3] < 1200){
  		start_timer();
  	}
  	else{
  		stop_timer();
  	}
  }
  else{
  	throttle = Mando_canal[3];
  	pid_i_mem_altitude = 0;
  	pid_last_altitude_d_error= 0;
  	stop_timer();
  }
}

int16_t timer_count;
float substract_throttle = 0;

void start_timer(){
	
	if(!timer_started){
		timer_started = true;
		timer_count = millis();
	}
	
	if (millis() - timer_count > 5000 && timer_started){
	 substract_throttle += 0.1;
	 throttle -= substract_throttle;
   if (throttle < 1250){
    start = 0;
    stop_timer();
    flying = false;
    collision_enable = false;
   }
	}
}

void stop_timer(){
	substract_throttle = 0;
	timer_count = 0;
	timer_started = false;
}
