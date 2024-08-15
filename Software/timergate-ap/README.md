| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C6 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | -------- |

# Timergate Access Point

## Overview


### Build and Flash

After the webpage design work has been finished, you should compile them by running following commands:

```bash
cd path_to_this_example/front/web-demo
npm install
npm run build
```

After a while, you will see a `dist` directory which contains all the website files (e.g. html, js, css, images).

Run `idf.py -p PORT flash monitor` to build and flash the project..

(To exit the serial monitor, type ``Ctrl-]``.)

See the [Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html) for full steps to configure and use ESP-IDF to build projects.

### Extra steps to do for deploying website by semihost

We need to run the latest version of OpenOCD which should support semihost feature when we test this deploy mode:

```bash
openocd-esp32/bin/openocd -s openocd-esp32/share/openocd/scripts -f board/esp32-wrover-kit-3.3v.cfg
```

## Example Output

### Render webpage in browser

In your browser, enter the URL where the website located (e.g. `http://esp-home.local`). You can also enter the IP address that ESP32 obtained if your operating system currently don't have support for mDNS service.

Besides that, this example also enables the NetBIOS feature with the domain name `esp-home`. If your OS supports NetBIOS and has enabled it (e.g. Windows has native support for NetBIOS), then the URL `http://esp-home` should also work.

![esp_home_local](https://dl.espressif.com/dl/esp-idf/docs/_static/esp_home_local.gif)

### ESP monitor output

In the *Light* page, after we set up the light color and click on the check button, the browser will send a post request to ESP32, and in the console, we just print the color value.

```bash
I (6115) example_connect: Connected to Ethernet
I (6115) example_connect: IPv4 address: 192.168.2.151
I (6325) esp-home: Partition size: total: 1920401, used: 1587575
I (6325) esp-rest: Starting HTTP Server
I (128305) esp-rest: File sending complete
I (128565) esp-rest: File sending complete
I (128855) esp-rest: File sending complete
I (129525) esp-rest: File sending complete
I (129855) esp-rest: File sending complete
I (137485) esp-rest: Light control: red = 50, green = 85, blue = 28
```

## Troubleshooting

1. Error occurred when building example: `...front/web-demo/dist doesn't exit. Please run 'npm run build' in ...front/web-demo`.
   * When you choose to deploy website to SPI flash, make sure the `dist` directory has been generated before you building this example.

(For any technical queries, please open an [issue](https://github.com/espressif/esp-idf/issues) on GitHub. We will get back to you as soon as possible.)
