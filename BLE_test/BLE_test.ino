#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
int scanTime = 5; //Em Segundos
int nivelRSSI = -10; //Ajustar conforme o ambiente
String dispositivosAutorizados = "42:11:d8:35:28:e8"; //MAC do seu dispositivo BLE
bool dispositivoPresente = false;

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      String dispositivosEncontrados = advertisedDevice.getAddress().toString().c_str();
      Serial.println(dispositivosEncontrados);
      if (dispositivosEncontrados == dispositivosAutorizados  
                    && advertisedDevice.getRSSI() > nivelRSSI) {
        dispositivoPresente = true;
      } else {
        dispositivoPresente = false;
      }
    }
};

void scanBLE() {
  Serial.println("entrou scan");
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  BLEScanResults foundDevices = pBLEScan->start(scanTime);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  BLEDevice::init("BLE_test");
}

void loop() {
  // put your main code here, to run repeatedly:
  scanBLE();
  Serial.println(dispositivoPresente);
  delay(2000);
}
