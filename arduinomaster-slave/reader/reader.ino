double myFreq = 923000000; 
uint16_t sf = 12, bw = 0, cr = 0, preamble = 8, txPower = 22;

char slaveReply[] = "Pong from Slave";

void hexDump(uint8_t * buf, uint16_t len) {
  char alphabet[17] = "0123456789abcdef";
  Serial.print(F("   +------------------------------------------------+ +----------------+\r\n"));
  Serial.print(F("   |.0 .1 .2 .3 .4 .5 .6 .7 .8 .9 .a .b .c .d .e .f | |      ASCII     |\r\n"));
  for (uint16_t i = 0; i < len; i += 16) {
    if (i % 128 == 0)
      Serial.print(F("   +------------------------------------------------+ +----------------+\r\n"));
    char s[] = "|                                                | |                |\r\n";
    uint8_t ix = 1, iy = 52;
    for (uint8_t j = 0; j < 16; j++) {
      if (i + j < len) {
        uint8_t c = buf[i + j];
        s[ix++] = alphabet[(c >> 4) & 0x0F];
        s[ix++] = alphabet[c & 0x0F];
        ix++;
        if (c > 31 && c < 128) s[iy++] = c;
        else s[iy++] = '.';
      }
    }
    uint8_t index = i / 16;
    if (i < 256) Serial.write(' ');
    Serial.print(index, HEX);
    Serial.write('.');
    Serial.print(s);
  }
  Serial.print(F("   +------------------------------------------------+ +----------------+\r\n"));
}

void recv_cb(rui_lora_p2p_recv_t data) {
  if (data.BufferSize == 0) {
    return;
  }
  Serial.println("\n<<< Message from Master Received >>>");
  char buff[92];
  sprintf(buff, "Incoming message, length: %d, RSSI: %d, SNR: %d", data.BufferSize, data.Rssi, data.Snr);
  Serial.println(buff);
  hexDump(data.Buffer, data.BufferSize);

  Serial.println(">>> Sending Slave Pong <<<");
  bool send_result = api.lorawan.psend(sizeof(slaveReply) - 1, (uint8_t*)slaveReply);
  if (!send_result) {
    Serial.println("P2P send reply FAILED");
  }
}

void send_cb(void) {
  Serial.println("Slave reply sent. Returning to listening mode.");
  api.lorawan.precv(65534); 
}

void setup() {
  Serial.begin(115200);
  while(!Serial); 

  Serial.println("RAK P2P Slave Node");
  Serial.println("------------------------------------------------------");

  Serial.printf("Set Node device work mode %s\r\n", api.lorawan.nwm.set(0) ? "Success" : "Fail");
  Serial.printf("Set P2P mode frequency %3.3f: %s\r\n", (myFreq / 1e6), api.lorawan.pfreq.set(myFreq) ? "Success" : "Fail");
  Serial.printf("Set P2P mode spreading factor %d: %s\r\n", sf, api.lorawan.psf.set(sf) ? "Success" : "Fail");
  Serial.printf("Set P2P mode bandwidth %d: %s\r\n", bw, api.lorawan.pbw.set(bw) ? "Success" : "Fail");
  Serial.printf("Set P2P mode code rate 4/%d: %s\r\n", (cr + 5), api.lorawan.pcr.set(0) ? "Success" : "Fail");
  Serial.printf("Set P2P mode preamble length %d: %s\r\n", preamble, api.lorawan.ppl.set(8) ? "Success" : "Fail");
  Serial.printf("Set P2P mode tx power %d: %s\r\n", txPower, api.lorawan.ptp.set(22) ? "Success" : "Fail");
  
  api.lorawan.registerPRecvCallback(recv_cb);
  api.lorawan.registerPSendCallback(send_cb);

  Serial.println("Setup Complete. Listening for messages...");
  api.lorawan.precv(65534);
}

void loop() {
}