src="$1"
filename="${src%.*}"

node ~/CP3108/sinter/tools/compiler/svmc.js "$src" -i [esp_init_led, esp_led_on, esp_led_off] -o "$filename".svm
