init_led();

const led_channels = enum_list(0, 5);
const no_led_channels = 6;
const turn_off_led = () => map(ch => led_off(ch), led_channels);

function to_binary(num) {
    if (num === 0) {
        return null;
    } else {
        return pair(num % 2, to_binary(math_floor(num / 2)));
    }
}

map(ch => {
    led_on(ch);
    esp_wait(100);
}, led_channels);

map(ch => {
    led_off(no_led_channels - 1 - ch);
    esp_wait(100);
}, led_channels);

for (let k = 0; k < 2; k = k + 1) {
    const goal = k % 2 === 0 ? 5000 : 0;
    led_fade(0, goal, 3000);
    esp_wait(500);
    led_fade(1, goal, 2500);
    esp_wait(500);
    led_fade(2, goal, 2000);
    esp_wait(500);
    led_fade(3, goal, 1500);
    esp_wait(500);
    led_fade(4, goal, 1000);
    esp_wait(500);
    led_fade(5, goal, 500);
    esp_wait(1000);
}

for (let l = 0; l < 4096; l = l + 1) {
    display(l % 64);
    const x = l % 64;
    if (x === 0) {
        turn_off_led();
    } else {
    }
    const res = to_binary(x);
    const len = length(res);
    accumulate((cur, index) => {
        cur > 0 ? led_on(index) : led_off(index);
        return index + 1;
    }, 6 - len, res);
    esp_wait(150);
}

turn_off_led();
