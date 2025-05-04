// BLE service code disabled for test
/*
#include "ble_service.h"
#include <BLEDevice.h>
#include <BLEServer.h>

static BLEServer* pServer = nullptr;
static bool bleConnected = false;
static const char* deviceName = "GeoAtom";

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) override { bleConnected = true; }
    void onDisconnect(BLEServer* pServer) override { bleConnected = false; }
};

void initBLEService() {
    BLEDevice::init(deviceName);
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    // Add services/characteristics as needed
    BLEDevice::startAdvertising();
}

bool isBLEConnected() {
    return bleConnected;
}

const char* getBLEDeviceName() {
    return deviceName;
}
*/ 