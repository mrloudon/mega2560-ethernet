#include <Arduino.h>
#include <Ethernet.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ArduinoJson.h>

#define LED1 12
#define LED2 48
#define LOCAL_PORT 8889
#define REMOTE_PORT 8888

byte mac[] = {
    0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(192, 168, 20, 177);
IPAddress remoteAddress(192, 168, 20, 255);
TaskHandle_t blinkTask1Handle, udpRxTaskHandle, led2TaskHandle, serverTaskHandle;
SemaphoreHandle_t ethernetMutex, serialMutex, i2cMutex;

unsigned int localPort = 8888; // local port to listen on

// buffers for receiving and sending data
char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; // buffer to hold incoming packet,
char ReplyBuffer[] = "acknowledged";       // a string to send back

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;
EthernetServer server(80);
LiquidCrystal_I2C lcd(0x27, 16, 2);

void blinkTask1(void *pvParameters);
void led2Task(void *pvParameters);
void udpRxTask(void *pvParameters);
void serverTask(void *pvParameters);
void i2cScan();

void setup()
{
  Serial.begin(9600);
  Serial.println("\nRunning...");
  Wire.begin();

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);

  i2cScan();

  lcd.init();
  lcd.setCursor(0, 0);
  lcd.print("N received:");
  lcd.setCursor(0, 1);
  lcd.print("N sent:");

  Ethernet.begin(mac, ip);
  if (Ethernet.hardwareStatus() == EthernetNoHardware)
  {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    while (true)
    {
      delay(1); // do nothing, no point running without Ethernet hardware
    }
  }
  if (Ethernet.linkStatus() == LinkOFF)
  {
    Serial.println("Ethernet cable is not connected.");
  }
  Serial.print("Ethernet hardware status: ");
  Serial.print(Ethernet.hardwareStatus());
  Serial.println(" (1 = W5100, 2 = W5200, 3 = W5500)");
  Udp.begin(LOCAL_PORT);
  ethernetMutex = xSemaphoreCreateMutex();
  serialMutex = xSemaphoreCreateMutex();
  i2cMutex = xSemaphoreCreateMutex();
  xTaskCreate(blinkTask1, "B1", 256, NULL, 2, &blinkTask1Handle);
  xTaskCreate(udpRxTask, "RX", 256, NULL, 2, &udpRxTaskHandle);
  xTaskCreate(led2Task, "L2", 64, NULL, 2, &led2TaskHandle);
  xTaskCreate(serverTask, "SV", 1024, NULL, 2, &serverTaskHandle);
}

void loop()
{
}

void blinkTask1(void *pvParameters)
{
  uint8_t n = 0;
  while (1)
  {
    digitalWrite(LED1, HIGH);
    vTaskDelay(pdMS_TO_TICKS(500));
    digitalWrite(LED1, LOW);
    vTaskDelay(pdMS_TO_TICKS(1000));
    if (xSemaphoreTake(ethernetMutex, portMAX_DELAY) == pdTRUE)
    {
      Udp.beginPacket(remoteAddress, REMOTE_PORT);
      Udp.write("Hello RTOS World!\n");
      Udp.endPacket();
      xSemaphoreGive(ethernetMutex);
      n++;
      if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE)
      {
        lcd.setCursor(7, 1);
        lcd.print(n);
        lcd.print("  ");
        xSemaphoreGive(i2cMutex);
      }
      xTaskNotifyGive(led2TaskHandle);
    }
  }
}

void udpRxTask(void *pvParameters)
{
  uint16_t n = 0;
  while (1)
  {
    if (xSemaphoreTake(ethernetMutex, portMAX_DELAY) == pdTRUE)
    {
      int packetSize = Udp.parsePacket(); // Udp.parsePacket() needs to be called BEFORE Udp.available()
      if (Udp.available())
      {
        IPAddress remote = Udp.remoteIP();
        // read the packet into packetBuffer
        Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
        n++;

        if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE)
        {
          Serial.print("Received packet of size ");
          Serial.println(packetSize);
          Serial.print("From ");
          for (int i = 0; i < 4; i++)
          {
            Serial.print(remote[i], DEC);
            if (i < 3)
            {
              Serial.print(".");
            }
          }
          Serial.print(", port ");
          Serial.println(Udp.remotePort());
          Serial.println("Contents:");
          Serial.println(packetBuffer);
          xSemaphoreGive(serialMutex);
        }

        if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE)
        {
          lcd.setCursor(13, 0);
          lcd.print(n);
          vTaskDelay(pdMS_TO_TICKS(500));
          lcd.print("   ");
          xSemaphoreGive(i2cMutex);
        }
      }
      xSemaphoreGive(ethernetMutex);
    }
    taskYIELD();
  }
}

void led2Task(void *pvParameters)
{
  while (1)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    digitalWrite(LED2, HIGH);
    vTaskDelay(pdMS_TO_TICKS(100));
    digitalWrite(LED2, LOW);
  }
}

void i2cScan()
{
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++)
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}

void serverTask(void *pvParameters)
{
  uint8_t n = 0;
  char buf[10];
  server.begin();
  while (1)
  {
    if (xSemaphoreTake(ethernetMutex, portMAX_DELAY) == pdTRUE)
    {
      EthernetClient client = server.available();
      if (!client)
      {
        xSemaphoreGive(ethernetMutex);
        yield();
        continue;
      }
  
      while(client.available()){
        client.read(); // dump any content
      }
      n++;
      sprintf(buf, "Count: %3u", n);
      client.println(F("HTTP/1.0 200 OK"));
      client.println(F("Content-Type: text/plain"));
      client.println(F("Connection: close"));
      client.print(F("Content-Length: "));
      client.println(strlen(buf));
      client.println();
      client.print(buf);
      client.stop();

      xSemaphoreGive(ethernetMutex);
      yield();
    }
  }
};