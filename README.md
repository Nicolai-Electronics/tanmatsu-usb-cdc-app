# Tanmatsu USB-serial app

This app turns Tanmatsu into a USB-to-serial (CDC) adapter. It switches the on-board USB 1.1 PHY to device mode, exposes a USB CDC-ACM interface to the host, and bridges it to a UART on the CATT connector:

- CATT D5 -> UART TX (device output)
- CATT D1 -> UART RX (device input)

The host's requested baud rate, data bits, parity and stop bits are applied to the UART live. The screen shows the current port settings and running TX/RX byte counters, refreshed independently of the data path so the display never slows down the bridge. Press F1 to return to the launcher.

Build using `make build`. For more information visit the [documentation website](https://docs.tanmatsu.cloud).

## License

The contents of this repository may be considered in the public domain or [CC0-1.0](https://creativecommons.org/publicdomain/zero/1.0) licensed at your disposal.

At Nicolai Electronics we love open source so we recommend licensing your work based on this template under terms of the [MIT license](https://opensource.org/license/mit). The MIT license allows others to build upon your work without restrictions while also making sure you retain your attribution.
