/***
 *  This example shows LoRaWan protocol joining the network in OTAA mode, class A, region EU868.
 *  Device will send uplink every 10 seconds.
***/

#define OTAA_PERIOD   (10000)
/*************************************

   LoRaWAN band setting:
     RAK_REGION_EU433
     RAK_REGION_CN470
     RAK_REGION_RU864
     RAK_REGION_IN865
     RAK_REGION_EU868
     RAK_REGION_US915
     RAK_REGION_AU915
     RAK_REGION_KR920
     RAK_REGION_AS923

 *************************************/
#define OTAA_BAND     (RAK_REGION_AS923)
#define OTAA_DEVEUI   {0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x02}
#define OTAA_APPEUI   {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define OTAA_APPKEY   {0xEA, 0x24, 0x7D, 0x16, 0x97, 0x3E, 0xA2, 0x65, 0x7B, 0x6F, 0xD1, 0x78, 0x87, 0xC2, 0xE4, 0x34}

/** Packet buffer for sending */
/*uint8_t collected_data[64] = { 0 };*/


void recvCallback(SERVICE_LORA_RECEIVE_T * data){
  
  if (data->BufferSize > 0) { //資料長度大於0，
    char ascii[data->BufferSize]; //儲存資料的陣列
    Serial.println("Something received!");
    for (int i = 0; i < data->BufferSize; i++) {
        Serial.printf("0x%02X ", data->Buffer[i]); //%02X為不足兩位會自動補0
    }
    Serial.println(""); //等同於Serial.print("\r\n");
    Serial.printf("ASCII: ");
    for (int i = 0; i < data->BufferSize; i++){
        ascii[i] = data->Buffer[i]; //將資料放進陣列裡面
        Serial.printf("%c",ascii[i]);
    }
    Serial.println("");
    // 通過 UART1 發送數據給 STM32
    for (int i = 0; i < data->BufferSize; i++) {
      Serial1.write(data->Buffer[i]); //將資料透過UART1傳送到STM32
    }
    Serial.println("Data sent to STM32 via UART1");
  }
}

void joinCallback(int32_t status){
    Serial.printf("Join status: %d\r\n", status);
}

/*************************************
 * enum type for LoRa Event
    RAK_LORAMAC_STATUS_OK = 0,
    RAK_LORAMAC_STATUS_ERROR,
    RAK_LORAMAC_STATUS_TX_TIMEOUT,
    RAK_LORAMAC_STATUS_RX1_TIMEOUT,
    RAK_LORAMAC_STATUS_RX2_TIMEOUT,
    RAK_LORAMAC_STATUS_RX1_ERROR,
    RAK_LORAMAC_STATUS_RX2_ERROR,
    RAK_LORAMAC_STATUS_JOIN_FAIL,
    RAK_LORAMAC_STATUS_DOWNLINK_REPEATED,
    RAK_LORAMAC_STATUS_TX_DR_PAYLOAD_SIZE_ERROR,
    RAK_LORAMAC_STATUS_DOWNLINK_TOO_MANY_FRAMES_LOSS,
    RAK_LORAMAC_STATUS_ADDRESS_FAIL,
    RAK_LORAMAC_STATUS_MIC_FAIL,
    RAK_LORAMAC_STATUS_MULTICAST_FAIL,
    RAK_LORAMAC_STATUS_BEACON_LOCKED,
    RAK_LORAMAC_STATUS_BEACON_LOST,
    RAK_LORAMAC_STATUS_BEACON_NOT_FOUND,
 *************************************/

void sendCallback(int32_t status){
    if (status == LORAMAC_STATUS_OK) {
        Serial.println("Successfully sent");
    } else {
        Serial.println("Sending failed");
    }
}

void setup(){
    Serial.begin(115200, RAK_AT_MODE);
    Serial1.begin(115200, RAK_CUSTOM_MODE); //UART1 OPEN
    delay(2000);

    Serial.println("RAKwireless LoRaWan OTAA Example");
    Serial.println("------------------------------------------------------");
  
    if(api.lorawan.nwm.get() != 1)
    {
        Serial.printf("Set Node device work mode %s\r\n",
            api.lorawan.nwm.set(1) ? "Success" : "Fail");
        api.system.reboot();
    }

    // OTAA Device EUI MSB first
    uint8_t node_device_eui[8] = OTAA_DEVEUI;
    // OTAA Application EUI MSB first
    uint8_t node_app_eui[8] = OTAA_APPEUI;
    // OTAA Application Key MSB first
    uint8_t node_app_key[16] = OTAA_APPKEY;
  
    if (!api.lorawan.appeui.set(node_app_eui, 8)) {
        Serial.printf("LoRaWan OTAA - set application EUI is incorrect! \r\n"); //錯誤偵測
        return;
    }
    if (!api.lorawan.appkey.set(node_app_key, 16)) {
        Serial.printf("LoRaWan OTAA - set application key is incorrect! \r\n");//錯誤偵測
        return;
    }
    if (!api.lorawan.deui.set(node_device_eui, 8)) {
        Serial.printf("LoRaWan OTAA - set device EUI is incorrect! \r\n");//錯誤偵測
        return;
    }
  
    if (!api.lorawan.band.set(OTAA_BAND)) {
        Serial.printf("LoRaWan OTAA - set band is incorrect! \r\n");
        return;
    }
    if (!api.lorawan.deviceClass.set(RAK_LORA_CLASS_C)) {
        Serial.printf("LoRaWan OTAA - set device class is incorrect! \r\n");//錯誤檢測
        return;
    }
    if (!api.lorawan.njm.set(RAK_LORA_OTAA))	// Set the network join mode to OTAA
    {
        Serial.printf("LoRaWan OTAA - set network join mode is incorrect! \r\n");//OTAA加入模式失敗
        return;
    }
    if (!api.lorawan.join())	// Join to Gateway
    {
        Serial.printf("LoRaWan OTAA - join fail! \r\n");
        return;
    }
  
    /** Wait for Join success */
    while (api.lorawan.njs.get() == 0) {
        Serial.print("Wait for LoRaWAN join...");
        api.lorawan.join();
        delay(10000);
    }
  
    if (!api.lorawan.adr.set(true)) {
        Serial.printf("LoRaWan OTAA - set adaptive data rate is incorrect! \r\n"); //設定自適應資料頻寬(ADR)模式
        return;
    }
    if (!api.lorawan.rety.set(1)) {
        Serial.printf("LoRaWan OTAA - set retry times is incorrect! \r\n"); //設定嘗試自動重新連線模式
        return;
    }
    if (!api.lorawan.cfm.set(1)) {
        Serial.printf("LoRaWan OTAA - set confirm mode is incorrect! \r\n"); //設定上傳資料模式為confirm，回傳為0表示失敗
        return;
    }
  
    /** Check LoRaWan Status*/
    Serial.printf("Duty cycle is %s\r\n", api.lorawan.dcs.get()? "ON" : "OFF");	// Check Duty Cycle status
    Serial.printf("Packet is %s\r\n", api.lorawan.cfm.get()? "CONFIRMED" : "UNCONFIRMED");	// Check Confirm status
    uint8_t assigned_dev_addr[4] = { 0 };
    api.lorawan.daddr.get(assigned_dev_addr, 4);
    Serial.printf("Device Address is %02X%02X%02X%02X\r\n", assigned_dev_addr[0], assigned_dev_addr[1], assigned_dev_addr[2], assigned_dev_addr[3]);	// Check Device Address
    Serial.printf("Uplink period is %ums\r\n", OTAA_PERIOD);
    Serial.println("");
    api.lorawan.registerRecvCallback(recvCallback);
    api.lorawan.registerJoinCallback(joinCallback);
    api.lorawan.registerSendCallback(sendCallback);
}


void uplink_routine(){
    /** Payload of Uplink */
    uint8_t temp,humi;
    uint8_t rxBuffer[2]={0}; //儲存透過UART傳送過來的資料
    uint8_t AppDataBuffer[242]={0}; //合併相關資料用的緩衝區
    
    if(Serial1.available()){ //如果UART1可以動作，就往下執行
      rxBuffer[2]={0};
      Serial1.readBytes(rxBuffer, 2);
      temp = rxBuffer[0];
      humi = rxBuffer[1];
      Serial.printf("Receive Data: %d,%d\r\n",temp,humi);
    }

    int len = snprintf((char*)AppDataBuffer, 6, "%d,%d",temp,humi); //合併溫濕度資料，格式為：temp,humi
    Serial.println("Data Packet:");
    for (int i = 0; i < len; i++) {
      Serial.printf("0x%02X ",AppDataBuffer[i]); //印出資料
    }
    Serial.println(""); //等同於Serial.print("\r\n");
    /** Send the data package */
    if (api.lorawan.send(5,(uint8_t*)&AppDataBuffer, 3, false, 1)) { //呼叫api.lorawan.send來上傳資料，並確認相關狀態
      Serial.println("Sending is requested");
    } else {
      Serial.println("Sending failed");
    }
}

void loop(){
    static uint64_t last = 0;
    static uint64_t elapsed;
  
    if ((elapsed = millis() - last) > OTAA_PERIOD) {
        uplink_routine(); //LoRaWAN SendData Callback Function
  
        last = millis();
    }
}
