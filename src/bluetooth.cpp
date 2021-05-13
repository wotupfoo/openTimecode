/**
 * A BLE client example that is rich in capabilities.
 * There is a lot new capabilities implemented.
 * author unknown
 * updated by chegewara
 */
#include <Arduino.h>
#include "BLEDevice.h"

// GATT services
static BLEUUID gatt_service_generic_UUID("1800"); // GATT Generic service
static BLEUUID gatt_service_time_UUID("1805");	  // GATT Time service
static BLEUUID gatt_service_device_UUID("180A");  // GATT Device Information service
static BLEUUID gatt_service_battery_UUID("180F"); // GATT Battery service
// Select what service to connect to...
#define serviceUUID gatt_service_time_UUID

// GATT service characteristics
static BLEUUID gatt_time_current_UUID("2A2B");
static BLEUUID gatt_time_localinfo_UUID("2A0F");
static BLEUUID gatt_bat_level_UUID("2A19");
static BLEUUID gatt_device_ModelNumber_UUID("2A24");
static BLEUUID gatt_device_Manufacturer_UUID("2A29");
// Select what characteristic to monitor...
#define charUUID gatt_time_current_UUID

static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic *pRemoteCharacteristic;
static BLEAdvertisedDevice *myDevice;

static void notifyCallback(
	BLERemoteCharacteristic *pBLERemoteCharacteristic,
	uint8_t *pData,
	size_t length,
	bool isNotify)
{
	Serial.print("Notify callback for characteristic ");
	Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
	Serial.print(" of data length ");
	Serial.println(length);
	Serial.print("data: ");
	Serial.println((char *)pData);
}

class MyClientCallback : public BLEClientCallbacks
{
	void onConnect(BLEClient *pclient)
	{
		//Serial.println("BLEClient onConnect");
	}

	void onDisconnect(BLEClient *pclient)
	{
		connected = false;
		//Serial.println("BLEClient onDisconnect");
	}
};

bool connectToServer()
{
	Serial.print("Forming a connection to : \"");
	Serial.print(myDevice->getAddress().toString().c_str());
	Serial.print("\" : ");
	Serial.print(myDevice->getName().c_str());
	Serial.println("\"");

	BLEClient *pClient = BLEDevice::createClient();
	if (pClient == NULL)
	{
		Serial.println(" - Create client failed ");
		return false;
	}
	Serial.println(" - Created client");

	pClient->setClientCallbacks(new MyClientCallback());

	// Connect to the remote BLE Server.
	bool result = pClient->connect(myDevice); // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
	if (result == false)
	{
		Serial.println(" - Connect to client failed ");
		return false;
	}
	Serial.println(" - Connected to server");

	BLERemoteService *pRemoteGATTService = pClient->getService(gatt_service_generic_UUID);
	if (pRemoteGATTService == nullptr)
	{
		Serial.print("Failed to find the GATT Generic service UUID: ");
		Serial.println(serviceUUID.toString().c_str());
		pClient->disconnect();
		return false;
	}
	Serial.println(" - Found the GATT Generic service");

	// Obtain a reference to the service we are after in the remote BLE server.
	BLERemoteService *pRemoteService = pClient->getService(serviceUUID);
	if (pRemoteService == nullptr)
	{
		Serial.print("Failed to find our service UUID: ");
		Serial.println(serviceUUID.toString().c_str());
		pClient->disconnect();
		return false;
	}
	Serial.print(" - Found our service UUID:  ");
	Serial.print(serviceUUID.toString().c_str());
	Serial.println("  (GATT Time Service)");

	// Obtain a reference to the characteristic in the service of the remote BLE server.
	pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
	if (pRemoteCharacteristic == nullptr)
	{
		Serial.print("Failed to find our characteristic UUID: ");
		Serial.println(charUUID.toString().c_str());
		pClient->disconnect();
		return false;
	}
	Serial.print(" - Found our characteristic UUID:  ");
	Serial.print(charUUID.toString().c_str());
	Serial.println("  (GATT Time Service - Current Time)");

	// Read the value of the characteristic.
	if (pRemoteCharacteristic->canRead())
	{
		std::string value = pRemoteCharacteristic->readValue();
		Serial.print(" - The characteristic value was: ");
		Serial.println(value.c_str());
	}
#if 1
	pClient->disconnect();
	connected = false;
	return false;
#else
	if (pRemoteCharacteristic->canNotify())
		pRemoteCharacteristic->registerForNotify(notifyCallback);

	connected = true;
#endif
	return true;
}
/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
	/**
	 * Called for each advertising BLE server.
	 */
	void onResult(BLEAdvertisedDevice advertisedDevice)
	{
		// Does it have a name?
		Serial.print("BLE Device found: \"");
		if ((strcmp(advertisedDevice.getName().c_str(), "") != 0) /*&&
				(strcmp(advertisedDevice.getAddress().toString().c_str(),"28:5a:4d:c0:eb:07") != 0) &&
				(strcmp(advertisedDevice.getAddress().toString().c_str(),"64:d5:01:92:f4:19") != 0) */
		)
		{
			Serial.print(advertisedDevice.getName().c_str());
			Serial.print("\" : ");
			Serial.print(advertisedDevice.getAddress().toString().c_str());
			Serial.println("  ");

#if 1
			myDevice = new BLEAdvertisedDevice(advertisedDevice);
			doConnect = true;
			doScan = true;
#else
			// We have found a device, let us now see if it contains the service we are looking for.
			if (advertisedDevice.haveServiceUUID())
			{
				Serial.println("\t\tDevice has Service UUIDs");
				// Is it Advertising the desired Service? - eg GATT Time service
				if (advertisedDevice.isAdvertisingService(serviceUUID))
				{
					myDevice = new BLEAdvertisedDevice(advertisedDevice);
					doConnect = true;
					doScan = true;
					Serial.println("\t\tFound GATT Time Service - and is advertising it");
				} // has adversiting service
				else
				{
					// Walk through the Services (aka Attributes)
					for (int i = 0; i < advertisedDevice.getServiceUUIDCount(); i++)
					{
						if (advertisedDevice.getServiceUUID(i).equals(serviceUUID))
						{
							// Found the GATT TIME SERVICE
							Serial.println("\t\t\tFound GATT Time Service - and is NOT advertising it");
							for (int j = 0; advertisedDevice.getServiceDataUUIDCount(); j++)
							{
								if (advertisedDevice.getServiceDataUUID(j).equals(charUUID))
								{
									// FOUND THE GATT TIME SERVICE DATA "currentTime" Characteristic
									Serial.print("\t\t\t\tFound GATT Time Service Characteristic \"currenttime\" = ");
									Serial.println(advertisedDevice.getServiceData(j).c_str());
								} // if ServiceDataUUID
							}	  // for ServiceData's
						}		  // if ServiceUUID
					}			  // for Service's
				}				  // doesn't have advertising Service
			}					  // has ServiceUUIDs
#endif
		} // has a Device name
		else
		{
			Serial.println("\" NO NAME - skipping since devices with no name probably don't have the service we want anyhow");
		} // doesn't have Device name
	}	  // onResult
};		  // MyAdvertisedDeviceCallbacks

void bluetooth_setup()
{
	BLEDevice::init("ArduinoBLE");

	// Retrieve a Scanner and set the callback we want to use to be informed when we
	// have detected a new device.  Specify that we want active scanning and start the
	// scan to run for 10 seconds.
	BLEScan *pBLEScan = BLEDevice::getScan();
#if 0 // Regular callback engine
	//pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
#else // Manual walking through the results
	while (1)
	{
		pBLEScan->setInterval(1000);
		pBLEScan->setWindow(1000);
		pBLEScan->setActiveScan(true);
		Serial.println("Scanning for Bluetooth Devices for 30 seconds");
		pBLEScan->start(30, true);

		BLEScanResults theScanResults = pBLEScan->getResults();
		uint32_t theScanCount = theScanResults.getCount();
		Serial.print("BLEScan - detected ");
		Serial.print(theScanCount);
		Serial.println(" devices");
		for (uint32_t i = 0; i < theScanCount; i++)
		{
			BLEAdvertisedDevice theAvDevice = theScanResults.getDevice(i);
			BLEAddress theAddress(theAvDevice.getAddress());
			Serial.print(i);
			Serial.print(" ");
			Serial.print(theAvDevice.getRSSI());
			Serial.print("dBm\t");
			Serial.print(theAddress.toString().c_str());
			Serial.print("\t");
			Serial.print(theAvDevice.getName().c_str());
			Serial.println("");
		}
		Serial.println("-----------------------------------------------------------");
		esp_log_level_set("*", ESP_LOG_INFO);
		for (uint32_t i = 0; i < theScanCount; i++)
		{
			BLEAdvertisedDevice theAvDevice = theScanResults.getDevice(i);
			BLEAddress theAddress(theAvDevice.getAddress());
			Serial.print(i);
			Serial.print("\t");
			Serial.print(theAddress.toString().c_str());
			Serial.print("\t");
			Serial.print(theAvDevice.getName().c_str());
			Serial.println("");

			myDevice = new BLEAdvertisedDevice(theAvDevice);
			if (connectToServer())
			{
				Serial.println("Monitoring the server's service characteristic");
				break;
			}
		}
	}
#endif
} // End of setup.

// This is the Arduino main loop function.
void bluetooth_loop()
{

	// If the flag "doConnect" is true then we have scanned for and found the desired
	// BLE Server with which we wish to connect.  Now we connect to it.  Once we are
	// connected we set the connected flag to be true.
	if (doConnect == true)
	{
		if (connectToServer())
		{
			Serial.println("We are now connected to the BLE Server.");
		}
		else
		{
			Serial.println("We have failed to connect to the server; there is nothin more we will do.");
		}
		doConnect = false;
	}

	// If we are connected to a peer BLE Server, update the characteristic each time we are reached
	// with the current time since boot.
	if (connected)
	{
		pRemoteCharacteristic->readValue();
		Serial.println(pRemoteCharacteristic->readValue().c_str());
	}
	else if (doScan)
	{
		BLEDevice::getScan()->start(0); // this is just example to start scan after disconnect, most likely there is better way to do it in arduino
	}

	delay(1000); // Delay a second between loops.
} // End of loop
