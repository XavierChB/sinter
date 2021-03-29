color_sensor_init(5, false);

display("init successful");

while (true) {
    const x = color_get_rgb();
    const y = color_get_rgbc_raw();
    const lux = color_get_lux();
    const ct = color_get_color_temperature();
    display(x);
    display(y);
    display(lux);
    display(ct);
    esp_wait(1000);
}
