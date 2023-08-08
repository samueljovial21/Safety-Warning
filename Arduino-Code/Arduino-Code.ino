// this header is needed for Bluetooth Serial -> works ONLY on ESP32
#include "BluetoothSerial.h"

// init Class:
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

int incoming;
int button;
int value;

#include <MPU6050_tockn.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
TinyGPSPlus gps;
SoftwareSerial serial_gps(16, 17);  //TX and RX
double latitude, longitude;

#define LedRed 25
#define LedYellow 33
#define LedGreen 32

#define ANALOG_IN_PIN 27
#define RELAY 23

#define buzzer1 12
#define buzzer2 13

#define getar 14

#define rxPin 4
#define txPin 2
HardwareSerial sim800(1);


MPU6050 mpu6050(Wire);
int sudut;

long timer = 0;

String pesan;

int present_condition = 0;
int previous_condition = 0;

bool getaran = false;

int kecepatan;

// Volt Sensor Things :
// Floats for ADC voltage & Input voltage
float adc_voltage = 0.0;
float in_voltage = 0.0;
// Floats for resistor values in divider (in ohms)
float R1 = 30000.0;
float R2 = 7500.0;
// Float for Reference Voltage
float ref_voltage = 5.0;
// Integer for ADC value
int adc_value = 0;


bool mode = true;  // True for riding mode & false for parking mode

String saran;
int hasil = 0;

int satelit = 0;

// Millis
unsigned long interval = 1000;     // the time we need to wait
unsigned long previousMillis = 0;  // millis() returns an unsigned long.

// State Aktuator
bool buzz1State = false;
bool buzz2State = false;

bool redState = false;
bool yellowState = false;
bool greenState = false;

void setup() {
  Serial.begin(115200);

  SerialBT.begin("ESPSafetyWarning");  //Name of your Bluetooth interface -> will show up on your phone
  Serial.println("The device started, now you can pair with bluetooth!");

  pinMode(LedRed, OUTPUT);
  pinMode(LedYellow, OUTPUT);
  pinMode(LedGreen, OUTPUT);

  pinMode(RELAY, OUTPUT);

  pinMode(buzzer1, OUTPUT);
  pinMode(buzzer2, OUTPUT);

  pinMode(getar, INPUT);

  digitalWrite(RELAY, LOW);

  pesan = "";

  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  serial_gps.begin(9600);

  //Begin serial communication with Arduino and SIM800L
  sim800.begin(9600, SERIAL_8N1, rxPin, txPin);

  sim800.println("AT+CMGF=1");  // Configuring TEXT mode
  delay(1000);
  sim800.println("AT+CSQ");
  delay(1000);
  sim800.println("AT+CNMI=1,2,0,0,0");
  delay(1000);
}

void loop() {
  if (SerialBT.available()) {
    incoming = SerialBT.read();

    button = floor(incoming / 10);
    value = incoming % 10;
  }

  if (button == 1) {
    if (value == 1) {
      mode = true;
    } else {
      mode = false;
    }
  }

  saran = "Tidak ada saran berkendara";

  // Volt Sensor Things :
  // Read the Analog Input
  adc_value = analogRead(ANALOG_IN_PIN);
  // Determine voltage at ADC input
  adc_voltage = (adc_value * ref_voltage) / 1024.0;
  // Calculate voltage at divider input
  in_voltage = adc_voltage / (R2 / (R1 + R2));
  int temp = in_voltage;


  getSudut();

  boolean newData = false;
  for (unsigned long start = millis(); millis() - start < 250;) {
    while (serial_gps.available()) {
      if (gps.encode(serial_gps.read())) {
        newData = true;
      }
    }
  }

  if (newData == true) {
    newData = false;
    if (gps.satellites.value() != 0) {
      kecepatan = gps.speed.kmph();
      satelit = gps.satellites.value();
    }
  }

  if (Serial.available()) {
    sim800.write(Serial.read());
  }

  if (sim800.available() > 0) {
    pesan = sim800.readStringUntil('\n');
    pesan.toLowerCase();
    Serial.println(pesan);
    if (pesan == "get location\r") {
      getLink();
    } else if (pesan == "relay off\r") {
      mode = false;
      notifikasi();
    }
  }

  // Cek Getaran
  previous_condition = present_condition;
  present_condition = digitalRead(getar);
  if (previous_condition != present_condition) {
    getaran = true;
  } else {
    getaran = false;
  }

  unsigned long currentMillis = millis();  // grab current time

  
  // Mode Safety
  if (mode) {  // Mode Berkendara
    digitalWrite(RELAY, HIGH);
    // Logika Kecepatan

    // Logic Batas Kecepatan
    if ((kecepatan >= 50) && (kecepatan < 70)) {
      saran = "Kurangi kecepatan Anda";
      if (!buzz1State) {
        interval = 4000;
      } else {
        interval = 1000;
      }

      if ((unsigned long)(currentMillis - previousMillis) >= interval) {

        buzz1State = !buzz1State;
        digitalWrite(buzzer1, buzz1State);
        // save the "current" time
        previousMillis = millis();
      }
    } else if ((kecepatan >= 70) && (kecepatan < 100)) {
      saran = "Anda berada di kecepatan berbahaya";
      if (!buzz1State) {
        interval = 1000;
      } else {
        interval = 1000;
      }

      if ((unsigned long)(currentMillis - previousMillis) >= interval) {

        buzz1State = !buzz1State;
        digitalWrite(buzzer1, buzz1State);
        // save the "current" time
        previousMillis = millis();
      }
    } else {
      digitalWrite(buzzer1, false);
    }

    // Peringatan Kecepatan dan Sudut
    if ((kecepatan >= 30) && (kecepatan < 50)) {
      if ((sudut >= 30) && (sudut < 35)) {
        hasil = sudut - 29;
        saran = "Anda berkendara terlalu miring sebesar " + String(hasil) + "°";
        digitalWrite(LedRed, LOW);
        digitalWrite(LedYellow, HIGH);
        digitalWrite(LedGreen, LOW);
        if (!buzz2State) {
          interval = 1000;
        } else {
          interval = 1000;
        }

        if ((unsigned long)(currentMillis - previousMillis) >= interval) {

          buzz2State = !buzz2State;
          digitalWrite(buzzer2, buzz2State);
          // save the "current" time
          previousMillis = millis();
        }
      } else if (sudut < 30) {
        digitalWrite(LedRed, LOW);
        digitalWrite(LedYellow, LOW);
        digitalWrite(LedGreen, HIGH);
        digitalWrite(buzzer2, false);
      } else if (sudut >= 35) {
        hasil = sudut - 29;
        saran = "Anda berkendara terlalu miring sebesar " + String(hasil) + "°";
        digitalWrite(LedRed, HIGH);
        digitalWrite(LedYellow, LOW);
        digitalWrite(LedGreen, LOW);
        if (!buzz2State) {
          interval = 500;
        } else {
          interval = 500;
        }

        if ((unsigned long)(currentMillis - previousMillis) >= interval) {

          buzz2State = !buzz2State;
          digitalWrite(buzzer2, buzz2State);
          // save the "current" time
          previousMillis = millis();
        }
      }
    } else if ((kecepatan >= 50) && (kecepatan < 70)) {
      if ((sudut >= 45) && (sudut < 50)) {
        hasil = sudut - 44;
        saran = "Anda berkendara terlalu miring sebesar " + String(hasil) + "°";
        digitalWrite(LedRed, LOW);
        digitalWrite(LedYellow, HIGH);
        digitalWrite(LedGreen, LOW);
        if (!buzz2State) {
          interval = 1000;
        } else {
          interval = 1000;
        }

        if ((unsigned long)(currentMillis - previousMillis) >= interval) {

          buzz2State = !buzz2State;
          digitalWrite(buzzer2, buzz2State);
          // save the "current" time
          previousMillis = millis();
        }
      } else if (sudut < 45) {
        digitalWrite(LedRed, LOW);
        digitalWrite(LedYellow, LOW);
        digitalWrite(LedGreen, HIGH);
        digitalWrite(buzzer2, false);
      } else if (sudut >= 50) {
        hasil = sudut - 44;
        saran = "Anda berkendara terlalu miring sebesar " + String(hasil) + "°";
        digitalWrite(LedRed, HIGH);
        digitalWrite(LedYellow, LOW);
        digitalWrite(LedGreen, LOW);
        if (!buzz2State) {
          interval = 500;
        } else {
          interval = 500;
        }

        if ((unsigned long)(currentMillis - previousMillis) >= interval) {

          buzz2State = !buzz2State;
          digitalWrite(buzzer2, buzz2State);
          // save the "current" time
          previousMillis = millis();
        }
      }
    } else if ((kecepatan >= 60) && (kecepatan < 70)) {
      if ((sudut >= 50) && (sudut < 55)) {
        hasil = sudut - 49;
        saran = "Anda berkendara terlalu miring sebesar " + String(hasil) + "°";
        digitalWrite(LedRed, LOW);
        digitalWrite(LedYellow, HIGH);
        digitalWrite(LedGreen, LOW);
        if (!buzz2State) {
          interval = 1000;
        } else {
          interval = 1000;
        }

        if ((unsigned long)(currentMillis - previousMillis) >= interval) {

          buzz2State = !buzz2State;
          digitalWrite(buzzer2, buzz2State);
          // save the "current" time
          previousMillis = millis();
        }
      } else if (sudut < 50) {
        digitalWrite(LedRed, LOW);
        digitalWrite(LedYellow, LOW);
        digitalWrite(LedGreen, HIGH);
        digitalWrite(buzzer2, false);
      } else if (sudut >= 55) {
        hasil = sudut - 49;
        saran = "Anda berkendara terlalu miring sebesar " + String(hasil) + "°";
        digitalWrite(LedRed, HIGH);
        digitalWrite(LedYellow, LOW);
        digitalWrite(LedGreen, LOW);
        if (!buzz2State) {
          interval = 500;
        } else {
          interval = 500;
        }

        if ((unsigned long)(currentMillis - previousMillis) >= interval) {

          buzz2State = !buzz2State;
          digitalWrite(buzzer2, buzz2State);
          // save the "current" time
          previousMillis = millis();
        }
      }
    } else if ((kecepatan >= 70) && (kecepatan < 80)) {
      if ((sudut >= 60) && (sudut < 65)) {
        hasil = sudut - 59;
        saran = "Anda berkendara terlalu miring sebesar " + String(hasil) + "°";
        digitalWrite(LedRed, LOW);
        digitalWrite(LedYellow, HIGH);
        digitalWrite(LedGreen, LOW);
        if (!buzz2State) {
          interval = 1000;
        } else {
          interval = 1000;
        }

        if ((unsigned long)(currentMillis - previousMillis) >= interval) {

          buzz2State = !buzz2State;
          digitalWrite(buzzer2, buzz2State);
          // save the "current" time
          previousMillis = millis();
        }
      } else if (sudut < 60) {
        digitalWrite(LedRed, LOW);
        digitalWrite(LedYellow, LOW);
        digitalWrite(LedGreen, HIGH);
        digitalWrite(buzzer2, false);
      } else if (sudut >= 65) {
        hasil = sudut - 59;
        saran = "Anda berkendara terlalu miring sebesar " + String(hasil) + "°";
        digitalWrite(LedRed, HIGH);
        digitalWrite(LedYellow, LOW);
        digitalWrite(LedGreen, LOW);
        if (!buzz2State) {
          interval = 500;
        } else {
          interval = 500;
        }

        if ((unsigned long)(currentMillis - previousMillis) >= interval) {

          buzz2State = !buzz2State;
          digitalWrite(buzzer2, buzz2State);
          // save the "current" time
          previousMillis = millis();
        }
      }
    } else if ((kecepatan >= 80) && (kecepatan < 90)) {
      if ((sudut >= 70) && (sudut < 75)) {
        hasil = sudut - 69;
        saran = "Anda berkendara terlalu miring sebesar " + String(hasil) + "°";
        digitalWrite(LedRed, LOW);
        digitalWrite(LedYellow, HIGH);
        digitalWrite(LedGreen, LOW);
        if (!buzz2State) {
          interval = 1000;
        } else {
          interval = 1000;
        }

        if ((unsigned long)(currentMillis - previousMillis) >= interval) {

          buzz2State = !buzz2State;
          digitalWrite(buzzer2, buzz2State);
          // save the "current" time
          previousMillis = millis();
        }
      } else if (sudut < 70) {
        digitalWrite(LedRed, LOW);
        digitalWrite(LedYellow, LOW);
        digitalWrite(LedGreen, HIGH);
        digitalWrite(buzzer2, false);
      } else if (sudut >= 75) {
        hasil = sudut - 69;
        saran = "Anda berkendara terlalu miring sebesar " + String(hasil) + "°";
        digitalWrite(LedRed, HIGH);
        digitalWrite(LedYellow, LOW);
        digitalWrite(LedGreen, LOW);
        if (!buzz2State) {
          interval = 500;
        } else {
          interval = 500;
        }

        if ((unsigned long)(currentMillis - previousMillis) >= interval) {

          buzz2State = !buzz2State;
          digitalWrite(buzzer2, buzz2State);
          // save the "current" time
          previousMillis = millis();
        }
      }
    } else {
      digitalWrite(buzzer2, false);
      digitalWrite(LedGreen, HIGH);
      digitalWrite(LedRed, LOW);
      digitalWrite(LedYellow, LOW);
    }
  } else {  // Mode Parkir
    digitalWrite(RELAY, LOW);

    // Kasus 4
    if ((getaran) && (kecepatan >= 2)) {
      positipMaling();
      getLink();
      digitalWrite(LedRed, HIGH);
      digitalWrite(LedYellow, LOW);
      digitalWrite(LedGreen, LOW);
      if ((!buzz2State) || (!buzz1State)) {
        interval = 500;
      } else {
        interval = 500;
      }

      if ((unsigned long)(currentMillis - previousMillis) >= interval) {

        buzz2State = !buzz2State;
        buzz1State = !buzz1State;
        digitalWrite(buzzer2, buzz2State);
        digitalWrite(buzzer1, buzz1State);
        // save the "current" time
        previousMillis = millis();
      }
    }

    // Kasus 3
    if ((getaran) && (kecepatan < 2)) {
      digitalWrite(LedRed, LOW);
      digitalWrite(LedYellow, LOW);
      digitalWrite(LedGreen, LOW);
      if ((!buzz2State) || (!buzz1State)) {
        interval = 1000;
      } else {
        interval = 1000;
      }

      if ((unsigned long)(currentMillis - previousMillis) >= interval) {

        buzz2State = !buzz2State;
        buzz1State = !buzz1State;
        digitalWrite(buzzer2, buzz2State);
        digitalWrite(buzzer1, buzz1State);
        // save the "current" time
        previousMillis = millis();
      }
    }

    // Kasus 2
    if ((!getaran) && (kecepatan >= 2)) {
      digitalWrite(LedRed, LOW);
      digitalWrite(LedYellow, LOW);
      digitalWrite(LedGreen, LOW);
      if ((!buzz2State) || (!buzz1State)) {
        interval = 500;
      } else {
        interval = 500;
      }

      if ((unsigned long)(currentMillis - previousMillis) >= interval) {

        buzz2State = !buzz2State;
        buzz1State = !buzz1State;
        digitalWrite(buzzer2, buzz2State);
        digitalWrite(buzzer1, buzz1State);
        // save the "current" time
        previousMillis = millis();
      }
    }

    // Kasus 1
    if ((!getaran) && (kecepatan < 2)) {
      digitalWrite(LedRed, LOW);
      digitalWrite(LedYellow, LOW);
      digitalWrite(LedGreen, LOW);
      digitalWrite(buzzer2, LOW);
      digitalWrite(buzzer1, LOW);
    }
  }

  String statusGetaran;
  if (getaran) {
    statusGetaran = "Ada Getaran";
  } else {
    statusGetaran = "Tidak ada Getaran";
  }

  String nyala = "Mode Parkir Menyala";

  if (mode) {
    SerialBT.print(String(satelit) + "-" + String(kecepatan) + " km/jam" + "-" + String(sudut) + "°" + "-" + saran);
  } else {
    if (getaran) {
      SerialBT.print(String(satelit) + "-" + "Diputus" + "-" + statusGetaran + "-" + " ");
      // delay(5000);
    } else {
      SerialBT.print(String(satelit) + "-" + "Diputus" + "-" + statusGetaran + "-" + " ");
    }
  }
}

void getLink() {
  boolean newData = false;
  for (unsigned long start = millis(); millis() - start < 2000;) {
    while (serial_gps.available()) {
      if (gps.encode(serial_gps.read())) {
        newData = true;
      }
    }
  }

  if (newData) {
    Serial.print(gps.location.lat(), 6);
    Serial.println(gps.location.lng(), 6);
    delay(300);

    sim800.print("AT+CMGF=1\r");
    delay(1000);
    sim800.print("AT+CMGS=\"+6281263178388\"\r");
    delay(1000);
    sim800.print("http://maps.google.com/maps?q=loc:");
    sim800.print(gps.location.lat(), 6);
    sim800.print(",");
    sim800.print(gps.location.lng(), 6);
    delay(100);
    sim800.write(0x1A);  //ascii code for ctrl-26 //sim800.println((char)26); //ascii code for ctrl-26
    delay(1000);
    newData = false;
  }

  else {
    sim800.println("AT+CMGF=1");
    delay(1000);
    sim800.println("AT+CMGS=\"+6281263178388\"\r");
    delay(1000);
    sim800.print("Belum ada link");
    delay(100);
    sim800.println((char)26);
    delay(1000);
  }
}

void notifikasi() {
  sim800.println("AT+CMGF=1");
  delay(1000);
  sim800.println("AT+CMGS=\"+6281263178388\"\r");
  delay(1000);
  sim800.print("Kendaraan Anda sekarang berhasil diubah ke mode parkir.");
  delay(100);
  sim800.println((char)26);
  delay(1000);
}

void positipMaling() {
  sim800.println("AT+CMGF=1");
  delay(1000);
  sim800.println("AT+CMGS=\"+6281263178388\"\r");
  delay(1000);
  sim800.println("Terindikasi Pencurian!!!");
  delay(100);
  sim800.println((char)26);
  delay(1000);
}

void getSudut() {
  mpu6050.update();
  if (millis() - timer > 10) {
    sudut = mpu6050.getAngleY();
    if (sudut < 0) {
      sudut *= -1;
      sudut -= 4;
    }
    timer = millis();
  }
}
