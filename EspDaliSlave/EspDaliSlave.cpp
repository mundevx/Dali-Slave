



#include <Arduino.h>

#include <inttypes.h>
#include <driver/gptimer.h>
#include <driver/gpio.h>
#include <esp_timer.h>
#include <functional>

#include "EspDaliSlave.h"

#define IS_INVERTED
#ifdef IS_INVERTED
	#define DALI_BUS_LOW() digitalWrite(this->tx_pin,LOW); this->tx_bus_low=1
	#define DALI_BUS_HIGH() digitalWrite(this->tx_pin,HIGH); this->tx_bus_low=0
	#define DALI_IS_BUS_LOW() (digitalRead(this->rx_pin)==LOW)
#else
	#define DALI_BUS_LOW() digitalWrite(this->tx_pin,HIGH); this->tx_bus_low=1
	#define DALI_BUS_HIGH() digitalWrite(this->tx_pin,LOW); this->tx_bus_low=0
	#define DALI_IS_BUS_LOW() (digitalRead(this->rx_pin)==HIGH)
#endif
#define DALI_BAUD 1200
#define DALI_TE ((1000000+(DALI_BAUD))/(2*(DALI_BAUD)))  //417us
#define DALI_TE_MIN (80*DALI_TE)/100  
#define DALI_TE_MAX (120*DALI_TE)/100  
#define DALI_IS_TE(x) ((DALI_TE_MIN)<=(x) && (x)<=(DALI_TE_MAX))
#define DALI_IS_2TE(x) ((2*(DALI_TE_MIN))<=(x) && (x)<=(2*(DALI_TE_MAX)))

#define DALI_HOOK_COUNT 3
static Dali *IsrTimerHooks[DALI_HOOK_COUNT+1];


bool IRAM_ATTR timerISR(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) {
  for(uint8_t i=0; i<DALI_HOOK_COUNT; i++) {
    if(IsrTimerHooks[i] == NULL) return false;
    IsrTimerHooks[i]->ISR_timer();
  }
  return false;
}



/*
void IRAM_ATTR onTimer() {
  for(uint8_t i=0; i<DALI_HOOK_COUNT; i++) {
    if(IsrTimerHooks[i] == NULL) return;
    IsrTimerHooks[i]->ISR_timer();
  }
}
*/
void IRAM_ATTR Dali::ISR_timer() {
  portENTER_CRITICAL_ISR(&timerMux);
  if(this->bus_idle_te_cnt < 0xff) this->bus_idle_te_cnt++;
  
  //send starbit, message bytes, 2 stop bits. 
  switch(this->tx_state) {
    case IDLE: 
      break;
    case START: 
      //wait for timeslot, then send start bit
      if(this->bus_idle_te_cnt >= 22) {
        DALI_BUS_LOW();
        this->tx_state = START_X;
      }
      break;
    case START_X: 
      DALI_BUS_HIGH();
      this->tx_pos = 0;
      this->tx_state = BIT;
      break;
    case BIT: 
      if(this->tx_msg[this->tx_pos>>3] & 1<<(7-(this->tx_pos&0x7))) {DALI_BUS_LOW();} else {DALI_BUS_HIGH();}
      this->tx_state = BIT_X;
      break;
    case BIT_X: 
      if(this->tx_msg[this->tx_pos>>3] & 1<<(7-(this->tx_pos&0x7))) {DALI_BUS_HIGH();} else {DALI_BUS_LOW();}
      this->tx_pos++;
      if(this->tx_pos < this->tx_len) {this->tx_state = BIT;} else {this->tx_state = STOP1;}
      break;  	
    case STOP1: 
      DALI_BUS_HIGH();
      this->tx_state = STOP1_X;
	  //Serial.println("STOP1");
      break;  
    case STOP1_X: 
      this->tx_state = STOP2;
	  //Serial.println("STOP1_X");
      break;  
    case STOP2: 
      this->tx_state = STOP2_X;
	  //Serial.println("STOP2");
      break;  
    case STOP2_X: 
      this->tx_state = STOP3;
	  //Serial.println("STOP2_X");
      break;  
    case STOP3: 
      this->bus_idle_te_cnt = 0; 
      this->tx_state = IDLE;
      this->rx_state = RX_IDLE;
	  //Serial.println("STOP3");
      break;  	
  }
 
  //handle receiver stop bits
  if(this->rx_state == RX_BIT && this->bus_idle_te_cnt > 4) {
    //received two stop bits, got message in rx_msg + rx_len
    uint8_t bitlen = (this->rx_len+1)>>1;
    if((bitlen & 0x7) == 0) {
      uint8_t len = bitlen>>3;
      if(this->EventHandlerReceivedData != NULL) this->EventHandlerReceivedData(this, (uint8_t*)this->rx_msg, len);
	  
	  processCommand((uint8_t*)this->rx_msg);
    } else {
      //Serial.println("invalid bitlen");  
      Serial.println(this->rx_msg[0], HEX);
    } 
    this->rx_state = RX_IDLE;	
  }
  portEXIT_CRITICAL_ISR(&timerMux);
}

void IRAM_ATTR Dali::ISR_pinchange() {
	

 portENTER_CRITICAL_ISR(&timerMux);
  uint32_t ts = micros(); //get timestamp of change
  this->bus_idle_te_cnt = 0; //reset idle counter
  uint8_t bus_low = DALI_IS_BUS_LOW();

  //exit if transmitting
  if(this->tx_state != IDLE) {
    //check tx collision
    if(bus_low && !this->tx_bus_low) {
      this->tx_state = IDLE; //stop transmitter
      this->tx_collision = 1; //mark collision
    }
    portEXIT_CRITICAL_ISR(&timerMux);
    return;
  }

  //no bus change, ignore
  if(bus_low == this->rx_last_bus_low) {
    portEXIT_CRITICAL_ISR(&timerMux);
    return;
  }

  //store values for next loop
  uint32_t dt = ts - this->rx_last_change_ts;
  //Serial.printf("Delta=%uus\n", dt);
  this->rx_last_change_ts = ts;
  this->rx_last_bus_low = bus_low;

  switch(this->rx_state) {
    case RX_IDLE: 
      if(bus_low) {
        this->rx_state = RX_START;
      }
      break;	  
    case RX_START: 
      if(bus_low || !DALI_IS_TE(dt)) {
        this->rx_state = RX_IDLE;
      } else {
        this->rx_len = -1;
        for(uint8_t i=0; i<7; i++) this->rx_msg[0] = 0;		  
        this->rx_state = RX_BIT;
      }
      break;
    case RX_BIT:
      if(DALI_IS_TE(dt)) {
        //got a single Te pulse
        this->push_halfbit(bus_low);
      } else if(DALI_IS_2TE(dt)) {
        //got a double Te pulse
        this->push_halfbit(bus_low);
        this->push_halfbit(bus_low);
      } else {
        //got something else -> no good
        this->rx_state = RX_IDLE;
      }		
      break;
  }
  portEXIT_CRITICAL_ISR(&timerMux);
}

void Dali::push_halfbit(uint8_t bit) {
  bit = (~bit)&1;
  if((this->rx_len & 1) == 0) {
    uint8_t i = this->rx_len>>4;
    if(i < 3) {
      this->rx_msg[i] = (this->rx_msg[i]<<1) | bit;
    }
  }
  this->rx_len++;
}

void Dali::loadAddressesFromEEPROM() {
	/*
    EEPROM.begin(EEPROM_SIZE);
    
    // Load Slave 1
    slaves[0].shortAddress = EEPROM.read(EEPROM_ADDR_SHORT_ADDR_1);
    slaves[0].randomAddress = 0;
    for (int i = 0; i < 3; i++) {
        slaves[0].randomAddress <<= 8;
        slaves[0].randomAddress |= EEPROM.read(EEPROM_ADDR_RANDOM_ADDR_1 + i);
    }
    
    // Load Slave 2
    slaves[1].shortAddress = EEPROM.read(EEPROM_ADDR_SHORT_ADDR_2);
    slaves[1].randomAddress = 0;
    for (int i = 0; i < 3; i++) {
        slaves[1].randomAddress <<= 8;
        slaves[1].randomAddress |= EEPROM.read(EEPROM_ADDR_RANDOM_ADDR_2 + i);
    }
    
    EEPROM.end();
    
    // Validate and generate new addresses if needed
    for (int i = 0; i < 2; i++) {
        if (slaves[i].randomAddress == 0 || slaves[i].randomAddress == 0xFFFFFF) {
            generateRandomAddress(i);
        }
        Serial.printf("Slave %d - SHORT: 0x%X, RANDOM: %06X\n", 
                     i+1, slaves[i].shortAddress, slaves[i].randomAddress);
    }
	*/
}

void Dali::saveSlaveToEEPROM(uint8_t slaveIndex) {
	/*
    if (slaveIndex > 1) return;
    
	
	
    EEPROM.begin(EEPROM_SIZE);
    
    if (slaveIndex == 0) {
        EEPROM.write(EEPROM_ADDR_SHORT_ADDR_1, slaves[0].shortAddress);
        for (int i = 0; i < 3; i++) {
            EEPROM.write(EEPROM_ADDR_RANDOM_ADDR_1 + i, (slaves[0].randomAddress >> (16 - i*8)) & 0xFF);
        }
    } else {
        EEPROM.write(EEPROM_ADDR_SHORT_ADDR_2, slaves[1].shortAddress);
        for (int i = 0; i < 3; i++) {
            EEPROM.write(EEPROM_ADDR_RANDOM_ADDR_2 + i, (slaves[1].randomAddress >> (16 - i*8)) & 0xFF);
        }
    }
    
    EEPROM.commit();
    EEPROM.end();
	*/
}

void Dali::factoryReset(){
	for(int i=0; i<2; i++){
		slaves[i].shortAddress = 0xff;
		slaves[i].randomAddress = 0;
		slaves[i].groups[0] = 0;
		slaves[i].groups[1] = 0;
		for(int j=0; j<2; j++){
			slaves[i].scenes[j] = 0;
		}
		slaves[i].actualLevel = 0;
		slaves[i].addressAssigned = false;
		slaves[i].inCommissioningMode = false;
		slaves[i].addr_index = 0;
	}
    EEPROM.put(EEPROM_ADDR_SLAVE_STRUCT, slaves); // Write entire array at address 0
    EEPROM.commit(); // Save changes
}

void Dali::beginEEPROM() {
	EEPROM.begin(EEPROM_SIZE);	
}
// Write all slaves to EEPROM
void Dali::writeAllToEEPROM() {
	
    EEPROM.put(EEPROM_ADDR_SLAVE_STRUCT, slaves); // Write entire array at address 0
    EEPROM.commit(); // Save changes
	//EEPROM.end();
}

// Erase all EEPROM data (fill with 0xFF)
void Dali::eraseAllEEPROM() {
	//EEPROM.begin(EEPROM_SIZE);
    for (int i = EEPROM_ADDR_SLAVE_STRUCT; i < EEPROM_SIZE; i++) {
        EEPROM.write(i, 0xFF);
    }
    EEPROM.commit();
	//EEPROM.end();
}

// Read all slaves from EEPROM
void Dali::readAllFromEEPROM() {
	//EEPROM.begin(EEPROM_SIZE);
    EEPROM.get(EEPROM_ADDR_SLAVE_STRUCT, slaves);
	//EEPROM.end();
	
    for (int i = 0; i < 2; i++) {
        if (slaves[i].randomAddress == 0 || slaves[i].randomAddress == 0xFFFFFF) {
            generateRandomAddress(i);
        }
        Serial.printf("Slave %d - SHORT: 0x%X, RANDOM: %06X\n", 
                     i+1, slaves[i].shortAddress, slaves[i].randomAddress);
					 
		if(slaves[i].groups[0] == 0xff) slaves[i].groups[0] = 0;
		if(slaves[i].groups[1] == 0xff) slaves[i].groups[1] = 0;
		
		if(slaves[i].randomAddress == 0xffff) slaves[i].randomAddress = 0x0000;
		for(int j=0; j<16; j++){
		 if(slaves[i].scenes[j] == 0xff) slaves[i].scenes[j] = 0;
		}
    }	
}


void Dali::begin(int8_t tx_pin, int8_t rx_pin) {
  this->tx_pin = tx_pin;
  this->rx_pin = rx_pin;
  this->tx_state = IDLE;
  this->rx_state = RX_IDLE;
  
  // Initialize commissioning state
    for (uint8_t i = 0; i < 2; i++) {
        slaves[i].inCommissioningMode = false;
    }
    
	
  beginEEPROM();
  readAllFromEEPROM();	
  // Setup TX
  if(this->tx_pin >= 0) {
    pinMode(this->tx_pin, OUTPUT); 

	DALI_BUS_HIGH();

    	
    
    // Configure GPTimer for ESP32-C6
    gptimer_config_t timer_config = {
      .clk_src = GPTIMER_CLK_SRC_DEFAULT,
      .direction = GPTIMER_COUNT_UP,
      .resolution_hz = 1000000, // 1MHz, 1us per tick
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));
    
    // Set alarm value
    gptimer_alarm_config_t alarm_config = {
      .alarm_count = DALI_TE, // 417us
      .reload_count = 0,
      .flags = {
        .auto_reload_on_alarm = true,
      },
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));
    
    // Register event callback
    gptimer_event_callbacks_t cbs = {
      .on_alarm = timerISR,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, this));
    
    // Enable timer
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    ESP_ERROR_CHECK(gptimer_start(gptimer));
    
    // Setup timer interrupt hooks
    for(uint8_t i=0; i<DALI_HOOK_COUNT; i++) {
      if(IsrTimerHooks[i] == NULL) {
        IsrTimerHooks[i] = this;
        break;
      }
    }
  }else{
	  
	pinMode(3, OUTPUT); 
	digitalWrite(3, HIGH); this->tx_bus_low=0;
  }
   
  // Setup RX  
  if(this->rx_pin >= 0) {
    pinMode(this->rx_pin, INPUT);    

    // Setup pin change interrupt
    attachInterruptArg(digitalPinToInterrupt(this->rx_pin), 
      [](void* arg) {
        static_cast<Dali*>(arg)->ISR_pinchange();
      }, 
      this, 
      CHANGE);
  }
  maxLevel = 254;
  //loadAddressesFromEEPROM();
  
        gpio_pulldown_t pd_enable = GPIO_PULLDOWN_DISABLE;
		gpio_pullup_t pu_enable = GPIO_PULLUP_ENABLE;
		
        uint32_t pins = 0; 
        for(int j=0; j<2; j++){
			 pins |= 1ULL << led_pins[j];
             pins |= 1ULL << load_pins[j];
        }         
        gpio_config_t io_conf;
        // Configure the output pin
        io_conf.intr_type = GPIO_INTR_DISABLE;       // Disable interrupt
        io_conf.mode = GPIO_MODE_OUTPUT;             // Set as output mode
        io_conf.pin_bit_mask = pins;                 // Bit mask for the output pin
        io_conf.pull_down_en = pd_enable;                    // Disable pull-down mode
        io_conf.pull_up_en = pu_enable;                      // Disable pull-up mode
        gpio_config(&io_conf);
	
	
	
    // Initialize slave states
    for (int i = 0; i < 2; i++) {
      slaves[i].addressAssigned = (slaves[i].shortAddress != 0xFF);
	  #if(DEVICE_TYPE == DALI_RELAY)
		if(slaves[i].actualLevel > 0){
			//digitalWrite(led_pins[i], HIGH); 
			//digitalWrite(load_pins[i], HIGH);
			gpio_set_level(led_pins[i], HIGH);
			gpio_set_level(load_pins[i], HIGH);			
		}else{
			//digitalWrite(led_pins[i], LOW); 
			//digitalWrite(load_pins[i], LOW);
			gpio_set_level(led_pins[i], LOW);
			gpio_set_level(load_pins[i], LOW);			
		}
	  #endif		
    } 
	
}

uint8_t Dali::send(uint8_t* tx_msg, uint8_t tx_len_bytes) {
  if(tx_len_bytes > 3) return 2;
  if(this->tx_state != IDLE) return 1; 
  if(this->tx_pin >= 0) {
	  for(uint8_t i=0; i<tx_len_bytes; i++) this->tx_msg[i] = tx_msg[i];
	  this->tx_len = tx_len_bytes<<3;
	  this->tx_collision = 0;
	  this->tx_state = START;
  }
  return 0;
}

uint8_t Dali::sendwait(uint8_t* tx_msg, uint8_t tx_len_bytes, uint32_t timeout_ms) {
  if(tx_len_bytes > 3) return 2;
  uint32_t ts = millis();
  //wait for idle
  
  while(this->tx_state != IDLE) {
    if(millis() - ts > timeout_ms) return 1; 
  }
  //start transmit
  if(this->send(tx_msg, tx_len_bytes)) return 2;
  //wait for completion
  while(this->tx_state != IDLE) {
    if(millis() - ts > timeout_ms) return 3; 
  }
  //wait for answer
  //TODO
  return 0;
}

uint8_t Dali::sendwait_int(uint16_t tx_msg, uint32_t timeout_ms) {
  uint8_t m[3];
  m[0] = tx_msg>>8;
  m[1] = tx_msg&0xff;
  return sendwait(m, 2, timeout_ms);
}  

uint8_t Dali::sendwait_byte(uint8_t tx_msg, uint32_t timeout_ms) {
  uint8_t m[3];
  m[0] = tx_msg;
  return sendwait(m, 1, timeout_ms);
}  

void Dali::sendResponse(uint8_t response) {
    if (tx_pin < 0) return; // No TX pin configured
	portENTER_CRITICAL_ISR(&timerMux);
	sendwait_byte(response, 10);
	portEXIT_CRITICAL_ISR(&timerMux);
}

bool Dali::checkAddress(uint8_t addressByte) {
    activeSlaveIndex = 0xff;
    slaves[0].addr_index = 0;
	slaves[1].addr_index = 0;
	/*
		0AAA_AAAS: Sending to a Short Address AAAAAA (0 .. 63)
		100A_AAAS: Sending to a Group Address AAAA (0 .. 15)
		1111_111S: Sending to Broadcast
		
		0000_0010 xxxx_xxxx: Short Address 1, Arc power = xxxx_xxxx
		1000_0100 xxxx_xxxx: Group Address 2, Arc power = xxxx_xxxx
		1111_1110 xxxx_xxxx: Broadcast, Arc power = xxxx_xxxx
		0000_0011 xxxx_xxxx: Short Address 1, Command xxxx_xxxx
		1010_0000 xxxx_xxxx: Special Command
		
	*/
    // Broadcast commands (0xFE, 0xFF)
    if ((addressByte & 0xFE) == 0xFE) {
        slaves[0].addr_index = 1; // Both slaves
		slaves[1].addr_index = 1;
		//Serial.print("BROADCAST_ADDR: "); Serial.println(addressByte & 0xFE, DEC);
        return true;
    }
    
    // Group commands (0x80-0x9F)
    if (addressByte >= 0x80 && addressByte <= 0x9F) {
		Serial.println("Group commands");
		uint8_t addressType = (addressByte >> 7) & 0x01;  // Y bit
        uint8_t address = addressByte >> 1;               // Don't mask yet - we need full value fir
	
	    bool anyInGroup = false;
	    uint8_t group = address & 0x0F;  // Lower 4 bits
		uint8_t bank = (address >> 4) & 0x01; // Bit 4 (groups 0-7 or 8-15)
		for (uint8_t i = 0; i < 2; i++) {
			anyInGroup = (slaves[i].groups[bank] & (1 << group)) != 0;
			if(anyInGroup){
				slaves[i].addr_index = 1;
				activeSlaveIndex |= 1<<i;
				return anyInGroup;
			}else continue;
		}
		return anyInGroup;
    }

    // Short addresses (0x00-0x7F)
    uint8_t shortAddr = (addressByte >> 1) & 0x3F;
	//Serial.print("SHORT_ADDR: "); Serial.println(shortAddr, DEC);
    for (uint8_t i = 0; i < 2; i++) {
        if (slaves[i].shortAddress == shortAddr) {
            slaves[i].addr_index = 1;
			activeSlaveIndex = i;
            return true;
        }
    }
    
    return false;
}

void Dali::setActualLevel(uint8_t addr_index, uint8_t level) {
	set_level_flag = true;
	public_addr_index = addr_index;
	
	if(addr_index == 3){	
		slaves[0].actualLevel = level;
		slaves[1].actualLevel = level;
	    // For now, we just store the value
		portENTER_CRITICAL_ISR(&timerMux);
		#if(DEVICE_TYPE == DALI_RELAY)
			if(slaves[0].actualLevel > 0){
				gpio_set_level(led_pins[0], HIGH);
				gpio_set_level(load_pins[0], HIGH);	
				gpio_set_level(led_pins[1], HIGH);
				gpio_set_level(load_pins[1], HIGH);	
				//Serial.println("Turning ON"); 			
			}else{			
				gpio_set_level(led_pins[0], LOW);
				gpio_set_level(load_pins[0], LOW);
				gpio_set_level(led_pins[1], LOW);
				gpio_set_level(load_pins[1], LOW);				
				//Serial.println("Turning OFF");			
			}
		#else
			// Here you would implement the actual dimming control
			analogWrite(led_pins[0], level);
			analogWrite(load_pins[0], level);
			analogWrite(led_pins[1], level);
			analogWrite(load_pins[1], level);			
		#endif
		portEXIT_CRITICAL_ISR(&timerMux);		
	}else{
		if((slaves[0].addr_index & 0x01) == 0x01){
			slaves[0].actualLevel = level;
			// For now, we just store the value
			portENTER_CRITICAL_ISR(&timerMux);
			#if(DEVICE_TYPE == DALI_RELAY)
				if(slaves[0].actualLevel > 0){
					gpio_set_level(led_pins[0], HIGH);
					gpio_set_level(load_pins[0], HIGH);				
					//Serial.println("Turning ON"); 			
				}else{			
					gpio_set_level(led_pins[0], LOW);
					gpio_set_level(load_pins[0], LOW);				
					//Serial.println("Turning OFF");			
				}
			#else
				// Here you would implement the actual dimming control
				analogWrite(led_pins[0], level);
				analogWrite(load_pins[0], level);		
			#endif
			portEXIT_CRITICAL_ISR(&timerMux);		
		}
		if((slaves[1].addr_index & 0x01) == 0x01){
			slaves[1].actualLevel = level;
			// For now, we just store the value
			portENTER_CRITICAL_ISR(&timerMux);
			#if(DEVICE_TYPE == DALI_RELAY)
				if(slaves[1].actualLevel > 0){
					gpio_set_level(led_pins[1], HIGH);
					gpio_set_level(load_pins[1], HIGH);				
					//Serial.println("Turning ON"); 			
				}else{			
					gpio_set_level(led_pins[1], LOW);
					gpio_set_level(load_pins[1], LOW);				
					//Serial.println("Turning OFF");			
				}
			#else
				// Here you would implement the actual dimming control
				analogWrite(led_pins[1], level);
				analogWrite(load_pins[1], level);		
			#endif
			portEXIT_CRITICAL_ISR(&timerMux);
		}	
	}
}



bool Dali::isInGroup(uint8_t groupNum, uint8_t index) {
	if (groupNum < 8) {
		//Serial.print("groupNum1:"); Serial.println(groupNum, BIN);
		return (slaves[index].groups[0] & (1<<groupNum)); // Check group 0
	} else if (groupNum < 16) {
		//Serial.print("groupNum2:");Serial.println(slaves[index].groups[0]);
		return (slaves[index].groups[1] & (1<<(groupNum - 8))); // Check group 1
	}
}

void Dali::processCommand(uint8_t* data) {
	
    // Print the received buffer (2 bytes)
	/*
	Serial.println("--------------------");
    Serial.print("Rxd DALI CMD: ");
    Serial.print("0x");
    //if (data[0] < 0x10) Serial.print("0"); // Leading zero for single-digit hex
    Serial.print(data[0], HEX);
    Serial.print(" ");
    Serial.print("0x");
    //if (data[1] < 0x10) Serial.print("0"); // Leading zero for single-digit hex
    Serial.print(data[1], HEX);
    Serial.println();   
	*/
	
    uint8_t addressByte = data[0];
    uint8_t commandByte = data[1];
	
    // DT8 commands use 10100101 (0xA5) pattern
    if ((addressByte & 0b11100000) == 0b10100000) {  // Match all 0xAx and 0xBx
		//uint8_t commandType = (addressByte >> 4) & 0x01;  // 0 for A, 1 for B
		//uint8_t command = addressByte & 0x0F;  // Full lower nibble as command
		/*
        // DT8 commands use 10100101 (0xA5) pattern
        if (addressByte == 0xA5) {
			Serial.println("Handling DT8 Commands!!");
            //processDT8Command(command, data);
            
        }else{*/
            //Serial.println("Handling Commissioning Commands!!");			
			handleCommissioningCommand(addressByte, commandByte);
			
		//}
		return;
	}

    bool isAddressedToMe = checkAddress(addressByte);
    if (!isAddressedToMe) {
        Serial.println("Command not for us, ignore");
        return;
    }
    
    bool isDirectArcPower = !(addressByte & 0x01); // S bit is 0
	
    if (isDirectArcPower) {
        // Direct arc power control
		//Serial.println("-->Direct arc power control");
        setActualLevel(activeSlaveIndex, commandByte);
    } else {
        // Indirect command
		//Serial.println("-->Indirect command");
		
		//Serial.print("SlaveIndex: "); Serial.println(activeSlaveIndex, DEC);
        switch (commandByte) {
            // Basic commands
            case 0x00: // Off
                setActualLevel(activeSlaveIndex, 0);
                break;
            case 0x0A:
				// Group commands (0x80-0x8F = OFF, 0x90-0x9F = ON)			
                setActualLevel(activeSlaveIndex, maxLevel);
				break;			
			case 0x01: // Up
				setActualLevel(activeSlaveIndex, min<uint8_t>(actualLevel + 10, maxLevel));
				break;
				
			case 0x02: // Down
				setActualLevel(activeSlaveIndex, max<uint8_t>(actualLevel - 10, minLevel));
				break;
                
            case 0x05: // Max level			
                setActualLevel(activeSlaveIndex, maxLevel);
                break;
                
            case 0x06: // Min level
                setActualLevel(activeSlaveIndex, minLevel);
                break;
                
            // Configuration commands
            case 0x20: // Reset
                // Reset to default values
                maxLevel = 254;
                minLevel = 1;
                powerOnLevel = 254;
                systemFailureLevel = 254;
                fadeTime = 1;
                fadeRate = 1;
                slaves[0].groups[0] = slaves[0].groups[1] = 0;
				slaves[1].groups[0] = slaves[1].groups[1] = 0;
                memset(scenes, 0, sizeof(scenes));
                break;
                
            case 0x21: // Store DTR
                dtr = commandByte;
                break;
                
            case 0x2A: // Store max level
                maxLevel = dtr;
                break;
                
            case 0x2B: // Store min level
                minLevel = dtr;
                break;
                
            case 0x2C: // Store system failure level
                systemFailureLevel = dtr;
                break;
                
            case 0x2D: // Store power on level
                powerOnLevel = dtr;
                break;
                
            case 0x2E: // Store fade time
                fadeTime = dtr;
                break;
                
            case 0x2F: // Store fade rate
                fadeRate = dtr;
                break;
                
            case 0x80: // Store short address
                shortAddress = dtr & 0x3F;
                break;
                
            // Query commands
            case 0x90: // Query status
                sendResponse(status);
                break;
                
            case 0x91: // Query operating
                sendResponse(actualLevel > 0 ? 0xFF : 0x00);
                break;
                
            case 0x98: // Query DTR
                sendResponse(dtr);
                break;
                
            case 0xA0: // Query actual level
                sendResponse(actualLevel);
                break;
                
            case 0xA1: // Query max level
                sendResponse(maxLevel);
                break;
                
            case 0xA2: // Query min level
                sendResponse(minLevel);
                break;
                
            case 0xA3: // Query power on level
                sendResponse(powerOnLevel);
                break;
                
            case 0xA4: // Query system failure level
                sendResponse(systemFailureLevel);
                break;
                
            case 0xA5: // Query fade time/rate
                sendResponse((fadeTime << 4) | (fadeRate & 0x0F));
                break;
                
            case 0xB0 ... 0xBF: // Query scene level
                sendResponse(scenes[commandByte - 0xB0]);
                break;
                
            case 0xC0: // Query groups 0-7
                sendResponse(slaves[activeSlaveIndex].groups[0]);
                break;
                
            case 0xC1: // Query groups 8-15
                sendResponse(slaves[activeSlaveIndex].groups[1]);
                break;
                
            case 0xC2: // Query random address H
                sendResponse((randomAddress >> 16) & 0xFF);
                break;
                
            case 0xC3: // Query random address M
                sendResponse((randomAddress >> 8) & 0xFF);
                break;
                
            case 0xC4: // Query random address L
                sendResponse(randomAddress & 0xFF);
                break;
                
            // Scene commands
            case 0x40 ... 0x4F: // Store scene
			    if(activeSlaveIndex < 2){
					slaves[activeSlaveIndex].scenes[commandByte - 0x40] = actualLevel;
				}
                break;
                
            case 0x50 ... 0x5F: // Remove scene
			    if(activeSlaveIndex < 2){
					slaves[activeSlaveIndex].scenes[commandByte - 0x50] = 0;
				}
                break;
                
            case 0x10 ... 0x1F: // Recall scene
                setActualLevel(activeSlaveIndex, slaves[activeSlaveIndex].scenes[commandByte - 0x10]);
                break;
               
            // Group commands
            case 0x60 ... 0x6F: // Add to group
				if(activeSlaveIndex < 2){
					//Serial.println("--->++Add to group!!");
					if (commandByte <= 0x67) {
						slaves[activeSlaveIndex].groups[0] |= (1 << (commandByte - 0x60));
					} else {
						slaves[activeSlaveIndex].groups[1] |= (1 << (commandByte - 0x68));
					}
				}
                break;
                
            case 0x70 ... 0x7F: // Remove from group			    	
				if(activeSlaveIndex < 2){	
                    //Serial.println("--->--Remove from group!!");  				
					if (commandByte <= 0x77) {
						slaves[activeSlaveIndex].groups[0] &= ~(1 << (commandByte - 0x70));
					} else {
						slaves[activeSlaveIndex].groups[1] &= ~(1 << (commandByte - 0x78));
					}
				}
                break;  
			
            default:
                // Unsupported command
                break;
        }
    }
	writeAllToEEPROM();
}

void Dali::generateRandomAddress(uint8_t slaveIndex) {
    if (slaveIndex >= 2) return;
    
    do {
        // Generate 24-bit random address
        slaves[slaveIndex].randomAddress = esp_random() & 0x00FFFFFF;
        
        // Ensure valid address (not 0 or 0xFFFFFF)
    } while (slaves[slaveIndex].randomAddress == 0 || 
             slaves[slaveIndex].randomAddress == 0x00FFFFFF ||
             (slaveIndex == 1 && slaves[slaveIndex].randomAddress == slaves[0].randomAddress) ||
             (slaveIndex == 0 && slaves[slaveIndex].randomAddress == slaves[1].randomAddress));
    /*
    Serial.printf("Generated random address for slave %d: %06X\n", 
                 slaveIndex + 1, 
                 slaves[slaveIndex].randomAddress);*/
}

/*
void Dali::generateRandomAddress(uint8_t slaveIndex) {
    if (slaveIndex > 1) return; // Only support 2 slaves (index 0 and 1)
    
    do {
        // Generate 24-bit random address (masking to ensure 3-byte range)
        slaves[slaveIndex].randomAddress = esp_random() & 0x00FFFFFF;
        // Ensure address isn't:
        // - 0x000000 (invalid)
        // - 0xFFFFFF (reserved)
        // - Matching the other slave's address
    } while (slaves[slaveIndex].randomAddress == 0 || 
             slaves[slaveIndex].randomAddress == 0x00FFFFFF ||
             (slaveIndex == 1 && slaves[slaveIndex].randomAddress == slaves[0].randomAddress) ||
             (slaveIndex == 0 && slaves[slaveIndex].randomAddress == slaves[1].randomAddress));

    Serial.printf("Slave %d new random address: %06X\n", 
                 slaveIndex + 1, 
                 slaves[slaveIndex].randomAddress);
    
    // Save immediately to EEPROM
	writeAllToEEPROM();
}*/

bool addressAssigned = false;
/*
bool Dali::compareWithSearchAddress(uint8_t slaveIndex) {
    if (slaves[slaveIndex].addressAssigned){
		Serial.println("Already addressed - ignoring compare");
		return false;
    }
    uint32_t search_addr = (searchAddress[0] << 16) | 
                         (searchAddress[1] << 8) | 
                          searchAddress[2];
    
    return (slaves[slaveIndex].randomAddress <= search_addr);
}*/

uint32_t Dali::getSearchAddress() {
    return (searchAddress[0] << 16) | (searchAddress[1] << 8) | searchAddress[2];
}

bool Dali::shouldRespondToCompare(uint8_t slaveIndex) {
    if (slaveIndex >= 2) return false;
    return slaves[slaveIndex].randomAddress <= getSearchAddress();
}

bool Dali::isExactMatch(uint8_t slaveIndex) {
    if (slaveIndex >= 2) return false;
    return slaves[slaveIndex].randomAddress == getSearchAddress();
}

bool Dali::compareWithSearchAddress(uint8_t slaveIndex) {
    if (slaveIndex >= 2) return false;
    
    uint32_t slaveAddr = slaves[slaveIndex].randomAddress;
    uint32_t searchAddr = getSearchAddress();
    
    // Only respond if we're in commissioning mode and our address <= search address
    return slaves[slaveIndex].inCommissioningMode && (slaveAddr <= searchAddr);
}

void Dali::handleCommissioningCommand(uint8_t cmd, uint8_t data) {
    //Serial.printf("Commissioning CMD: 0x%02X, Data: 0x%02X\n", cmd, data);
    switch (cmd) {
        case CMD_INITIALIZE:  // 0xA1
            // Enter commissioning mode for both slaves
            for (uint8_t i = 0; i < 2; i++) {
                slaves[i].inCommissioningMode = true;
                slaves[i].addressAssigned = false;
                generateRandomAddress(i);
                //Serial.printf("Slave %d random addr: %06X\n", i+1, slaves[i].randomAddress);
            }
            break;
            
        case CMD_RANDOMIZE:  // 0xA3
            // Randomize both slaves
            for (uint8_t i = 0; i < 2; i++) {
                if (slaves[i].inCommissioningMode) {
                    generateRandomAddress(i);
                    //Serial.printf("Slave %d new random addr: %06X\n", i+1, slaves[i].randomAddress);
                }
            }
            break;
            
        case CMD_COMPARE:  // 0xA5
            // Only respond if we're in commissioning mode and address <= search address
            for (uint8_t i = 0; i < 2; i++) {
                if (slaves[i].inCommissioningMode && shouldRespondToCompare(i)) {
                    sendResponse(0x00);
                    //Serial.printf("Slave %d responded to compare\n", i+1);
                }
            }
            break;
            
        case CMD_SEARCHADDRH:  // 0xA9
            searchAddress[0] = data;
            break;
            
        case CMD_SEARCHADDRM:  // 0xAB
            searchAddress[1] = data;
            break;
            
        case CMD_SEARCHADDRL:  // 0xAD
            searchAddress[2] = data;
            //Serial.printf("New search address: %06X\n", getSearchAddress());
            break;
            
        case CMD_PROGRAMSHORT:  // 0xB1
            // Only accept valid short addresses (0-63)
            if (data > 63) {
                //Serial.println("Invalid short address (must be 0-63)");
                break;
            }
            
            // Program address to matching slave
            for (uint8_t i = 0; i < 2; i++) {
                if (slaves[i].inCommissioningMode && isExactMatch(i)) {
                    slaves[i].shortAddress = data;
                    slaves[i].addressAssigned = true;
                    slaves[i].inCommissioningMode = false;
					
                    //Serial.printf("Slave %d assigned address: %d\n", i+1, slaves[i].shortAddress);
                }
            }
            break;
        case CMD_VERIFYSHORT:
            for (uint8_t i = 0; i < 2; i++) {
				if (slaves[i].shortAddress == data) {
					slaves[i].shortAddress = (data >> 1);
					sendResponse(0x00);
					//Serial.println("verify address");
					writeAllToEEPROM();
				}
            }
            break;
            
		default: break;	
    }
}


