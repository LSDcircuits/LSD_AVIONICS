This dir contains all different formats used to transfer data:

Reason:
theres a high ammount of data being transfered between cores and used by a OS to read data from the MCU. 
To ligthen the job having formatted binary outputs will ease the processor instead of using printf & is more easy than openocd on pico.

Debug format: (uart string for debugging)
used for:
  - checking filter gain values
  - watermark check
  - pipeline verification
  - raw data
  
Data format (MISO):(output data used for external devices)
used for:
  - Output via SPI (embedded devices)
  - Output via I2C (embedded devices)
  - output via uart (embedded devices & (OS/APPS))
  - Output via USB (printf for test tools)
