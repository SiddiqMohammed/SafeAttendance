/***************************************************
  This is a library example for the MLX90614 Temp Sensor

  Designed specifically to work with the MLX90614 sensors in the
  adafruit shop
  ----> https://www.adafruit.com/products/1747 3V version
  ----> https://www.adafruit.com/products/1748 5V version

  These sensors use I2C to communicate, 2 pins are required to
  interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_MLX90614.h>

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

int temp;

void setup() {
  Serial.begin(9600);

//  Serial.println("Adafruit MLX90614 test");

  mlx.begin();
}

void loop() {
  //  Serial.print("Temp = "); Serial.print(mlx.readAmbientTempC());
  //  Serial.print("*C\tObject = ");
  temp = mlx.readObjectTempC(), DEC;
  //  Serial.print(mlx.readObjectTempC(), DEC);
  temp = int(temp);
//  Serial.print(temp);

  if (temp > 35) {
//    Serial.print(" HIGH TEMP");
    Serial.print("1");
  }
  else if (temp < 29){
//    Serial.print(" NO Readings");
    Serial.print("2");
  }
  else {
//    Serial.print(" Good to go");
    Serial.print("3");
  }

  //  Serial.println("*C");

  //  Serial.print("Ambient = "); Serial.print(mlx.readAmbientTempF());
  //  Serial.print("*F\tObject = "); Serial.print(mlx.readObjectTempF()); Serial.println("*F");

  Serial.println();
  delay(100);
}
