const clockwise = true;

motor_init();

motor_full_steps(50, clockwise);
esp_wait(10000);
