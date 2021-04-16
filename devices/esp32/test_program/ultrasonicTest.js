ultrasonic_init();

while (true) {
    display(ultrasonic_measure_cm(100));
    esp_wait(200);
}
