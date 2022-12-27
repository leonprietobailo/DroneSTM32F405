
void read_RC() {
  if (contador_flaco == 18) {
    for (int i = 1; i <= numero_canales; i++) {
      Mando_canal[i] = map(pulso_instante[2 * i] - pulso_instante[2 * i - 1], 600, 1600, 1000, 2000);
    }
  }
}

void read_PPM() {
  if (micros() - pulso_instante[contador_flaco - 1] > 2500) contador_flaco = 0;
  pulso_instante[contador_flaco] = micros();
  contador_flaco++;
}
