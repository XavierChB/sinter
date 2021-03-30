ultrasonic_init();

while (true) {
    display(ultrasonic_measure_cm(50));
    esp_wait(200);
}
