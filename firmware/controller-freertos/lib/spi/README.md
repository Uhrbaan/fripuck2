# SPI connections

![](https://www.gctronic.com/doc/images/9/9b/comm_overall_epuck2E.jpg)
This small library contains all the code necessary to manage all the SPI/I2S connections (drawn in a reddish-brown in the image above, where the connections to the microphones use a variant of the SPI protocol called I2S). 
This includes the following connections: 

- **`SPI1`**: The `SPI1` connects the the controller to the radio chip and to the encoder (the encoder is a sensor that tracks how much the wheels have turned). 

- **`SPI2/I2S2`**: Connects the controller to microphones n°1 and 2

- **`SPI3/I2S3`**: Connects the controller to the microphones n°3 and 4