
#include <WiFi.h>
#include <WebServer.h>
#include <EEPROM.h>
#include "EspDaliSlave.h"
#include <esp_task_wdt.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

WebServer server(80);
// Create the DALI interfaces
Dali dali2; 
#define DALI_ADDR(short_addr)    ((short_addr << 1) | 0x01)

// DALI Commands
#define DALI_CMD_OFF             0x00
#define DALI_CMD_UP              0x01
#define DALI_CMD_DOWN            0x02
#define DALI_CMD_STEP_UP         0x03
#define DALI_CMD_STEP_DOWN       0x04
#define DALI_CMD_RECALL_MAX      0x05
#define DALI_CMD_RECALL_MIN      0x06
#define DALI_CMD_STEP_DOWN_OFF   0x07
#define DALI_CMD_ON_STEP_UP      0x08
#define DALI_CMD_ENABLE_DAPC     0x09
#define DALI_CMD_GO_TO_LEVEL     0x10
#define DALI_CMD_RESET           0x20
#define DALI_CMD_STORE_ACTUAL    0x21
#define DALI_CMD_STORE_MAX       0x2A
#define DALI_CMD_STORE_MIN       0x2B

// GPIO pins for switches
#define SWITCH1_PIN             4
#define SWITCH2_PIN             5

#define SWITCH3_PIN             1
#define SWITCH4_PIN             0

#define DEBOUNCE_DELAY_MS       100
#define LONG_PRESS_DURATION_MS  5000  // 5 seconds for long press

unsigned long pressStartTime=0;

// Task handles
TaskHandle_t switchTaskHandle = NULL;

void switchTask(void *parameter) {
  bool lastState1 = digitalRead(SWITCH1_PIN);
  bool lastState2 = digitalRead(SWITCH2_PIN);
  bool lastState3 = digitalRead(SWITCH3_PIN);
  bool lastState4 = digitalRead(SWITCH4_PIN);

  unsigned long lastDebounceTime1 = 0;
  unsigned long lastDebounceTime2 = 0;
  unsigned long lastDebounceTime3 = 0;
  unsigned long lastDebounceTime4 = 0;

  Serial.println("Task Working...");
  pinMode(9, OUTPUT); 
  while (1) {
    bool currentState1 = digitalRead(SWITCH1_PIN);
    bool currentState2 = digitalRead(SWITCH2_PIN);
    bool currentState3 = digitalRead(SWITCH3_PIN);
    bool currentState4 = digitalRead(SWITCH4_PIN);    
    unsigned long now = millis();
      
    // Debug raw pin states
    // Serial.printf("Raw states - SW1: %d, SW2: %d\n", currentState1, currentState2);

    // Check switch 1
    if (currentState1 != lastState1) {
      lastDebounceTime1 = now;
      
      //Serial.println(lastDebounceTime1);
    }
    if ((now - lastDebounceTime1) > DEBOUNCE_DELAY_MS) {
       
    }else{
      if (currentState1 != lastState1) {
        lastState1 = currentState1;
        if (currentState1 == LOW) { // Pressed (assuming pull-up)
          //Serial.println("SWITCH1_PIN Pressed!!");
          // Add your switch 1 pressed action here
          if(pressStartTime == 0)
          pressStartTime = now;
        } else {
          //Serial.println("SWITCH1_PIN Released!!");

          bool isLongPress = (now - pressStartTime) >= LONG_PRESS_DURATION_MS;
          // Serial.println(now - pressStartTime, DEC);
          if(isLongPress) {
            pressStartTime = 0;
            // Serial.println("SWITCH1_PIN Long Pressed!!");
            // Add your long press action for switch 1 here
            dali2.factoryReset();
            esp_restart();
          } else{
            // Add your switch 1 released action here
            if(dali2.slaves[0].actualLevel == 0) dali2.slaves[0].actualLevel = 254;
            else dali2.slaves[0].actualLevel = 0;
            dali2.slaves[0].addr_index = 0x01; dali2.slaves[1].addr_index = 0x00;
            dali2.setActualLevel(0, dali2.slaves[0].actualLevel);
          }
        }
      }
    }

    // Check switch 2
    if (currentState2 != lastState2) {
      lastDebounceTime2 = now;
    }
    if ((now - lastDebounceTime2) > DEBOUNCE_DELAY_MS) {

    }else{
      if (currentState2 != lastState2) {
        lastState2 = currentState2;
        if (currentState2 == LOW) { // Pressed (assuming pull-up)
          //Serial.println("SWITCH2_PIN Pressed!!");
          // Add your switch 2 pressed action here
        } else {
          //Serial.println("SWITCH2_PIN Released!!");
          // Add your switch 2 released action here
          if(dali2.slaves[1].actualLevel == 0) dali2.slaves[1].actualLevel = 254;
          else dali2.slaves[1].actualLevel = 0;
          dali2.slaves[0].addr_index = 0x00; dali2.slaves[1].addr_index = 0x01;
          dali2.setActualLevel(1, dali2.slaves[1].actualLevel);          
        }
      }      
    }
    /////////////////////Read pins from relay board////////////////////

        // Check switch 3
    if (currentState3 != lastState3) {
      lastDebounceTime3 = now;
      //Serial.println(lastDebounceTime3);
    }
    if ((now - lastDebounceTime3) > DEBOUNCE_DELAY_MS) {

    }else{
      if (currentState3 != lastState3) {
        lastState3 = currentState3;
        if (currentState3 == LOW) { // Pressed (assuming pull-up)
          //Serial.println("SWITCH3_PIN Pressed!!");
          // Add your switch 3 pressed action here
        } else {
          //Serial.println("SWITCH3_PIN Released!!");
          // Add your switch 1 released action here
          if(dali2.slaves[0].actualLevel == 0) dali2.slaves[0].actualLevel = 254;
          else dali2.slaves[0].actualLevel = 0;
          dali2.slaves[0].addr_index = 0x01; dali2.slaves[1].addr_index = 0x00;
          dali2.setActualLevel(0, dali2.slaves[0].actualLevel);

        }
      }
    }

    // Check switch 4
    if (currentState4 != lastState4) {
      lastDebounceTime4 = now;
    }
    if ((now - lastDebounceTime4) > DEBOUNCE_DELAY_MS) {

    }else{
      if (currentState4 != lastState4) {
        lastState4 = currentState4;
        if (currentState4 == LOW) { // Pressed (assuming pull-up)
          //Serial.println("SWITCH4_PIN Pressed!!");
          // Add your switch 4 pressed action here
        } else {
          //Serial.println("SWITCH4_PIN Released!!");
          // Add your switch 4 released action here
          if(dali2.slaves[1].actualLevel == 0) dali2.slaves[1].actualLevel = 254;
          else dali2.slaves[1].actualLevel = 0;
          dali2.slaves[0].addr_index = 0x00; dali2.slaves[1].addr_index = 0x01;
          dali2.setActualLevel(1, dali2.slaves[1].actualLevel);          
        }
      }      
    }

    digitalWrite(9, LOW);
    vTaskDelay(pdMS_TO_TICKS(5)); // Small delay to prevent busy waiting
    digitalWrite(9, HIGH);
    vTaskDelay(pdMS_TO_TICKS(5)); // Small delay to prevent busy waiting
  }
}


/*
// Task handles
TaskHandle_t switchTaskHandle = NULL;

void handleSwitchAction(uint8_t switchNum, bool isLongPress) {
  switch(switchNum) {
    case 1:
      if(isLongPress) {
        Serial.println("SWITCH1_PIN Long Pressed!!");
        // Add your long press action for switch 1 here
        dali2.eraseAllEEPROM();
      } else {
        Serial.println("SWITCH1_PIN Short Pressed!!");
        if(dali2.slaves[0].actualLevel == 0) dali2.slaves[0].actualLevel = 254;
        else dali2.slaves[0].actualLevel = 0;
        dali2.slaves[0].addr_index = 0x01; dali2.slaves[1].addr_index = 0x00;
        dali2.setActualLevel(0, dali2.slaves[0].actualLevel);
      }
      break;
      
    case 2:
      if(isLongPress) {
        Serial.println("SWITCH2_PIN Long Pressed!!");
        // Add your long press action for switch 2 here
        dali2.eraseAllEEPROM();
      } else {
        Serial.println("SWITCH2_PIN Short Pressed!!");
        if(dali2.slaves[1].actualLevel == 0) dali2.slaves[1].actualLevel = 254;
        else dali2.slaves[1].actualLevel = 0;
        dali2.slaves[0].addr_index = 0x00; dali2.slaves[1].addr_index = 0x01;
        dali2.setActualLevel(1, dali2.slaves[1].actualLevel);
      }
      break;
      
    case 3:
      if(isLongPress) {
        Serial.println("SWITCH3_PIN Long Pressed!!");
        // Add your long press action for switch 3 here
      } else {
        Serial.println("SWITCH3_PIN Short Pressed!!");
        if(dali2.slaves[0].actualLevel == 0) dali2.slaves[0].actualLevel = 254;
        else dali2.slaves[0].actualLevel = 0;
        dali2.slaves[0].addr_index = 0x01; dali2.slaves[1].addr_index = 0x00;
        dali2.setActualLevel(0, dali2.slaves[0].actualLevel);
      }
      break;
      
    case 4:
      if(isLongPress) {
        Serial.println("SWITCH4_PIN Long Pressed!!");
        // Add your long press action for switch 4 here
      } else {
        Serial.println("SWITCH4_PIN Short Pressed!!");
        if(dali2.slaves[1].actualLevel == 0) dali2.slaves[1].actualLevel = 254;
        else dali2.slaves[1].actualLevel = 0;
        dali2.slaves[0].addr_index = 0x00; dali2.slaves[1].addr_index = 0x01;
        dali2.setActualLevel(1, dali2.slaves[1].actualLevel);
      }
      break;
  }
}

void switchTask(void *parameter) {
  struct SwitchState {
    bool lastState;
    bool currentState;
    unsigned long pressStartTime;
    bool pressDetected;
  };
  
  SwitchState switches[4] = {
    {digitalRead(SWITCH1_PIN), digitalRead(SWITCH1_PIN), 0, false},
    {digitalRead(SWITCH2_PIN), digitalRead(SWITCH2_PIN), 0, false},
    {digitalRead(SWITCH3_PIN), digitalRead(SWITCH3_PIN), 0, false},
    {digitalRead(SWITCH4_PIN), digitalRead(SWITCH4_PIN), 0, false}
  };
  
  unsigned long lastDebounceTime[4] = {0};
  const uint8_t switchPins[4] = {SWITCH1_PIN, SWITCH2_PIN, SWITCH3_PIN, SWITCH4_PIN};

  Serial.println("Task Working...");

  while (1) {
    unsigned long now = millis();

    for (int i = 0; i < 4; i++) {
      switches[i].currentState = digitalRead(switchPins[i]);

      // Check for state change
      if (switches[i].currentState != switches[i].lastState) {
        lastDebounceTime[i] = now;
      }

      // Debounce check
      if ((now - lastDebounceTime[i]) > DEBOUNCE_DELAY_MS) {
        // State has changed
        if (switches[i].currentState != switches[i].lastState) {
          switches[i].lastState = switches[i].currentState;

          // Button pressed (assuming pull-up)
          if (switches[i].currentState == LOW) {
            switches[i].pressStartTime = now;
            switches[i].pressDetected = true;
          } 
          // Button released
          else if (switches[i].pressDetected) {
            switches[i].pressDetected = false;
            
            // Check if it was a long press
            bool isLongPress = (now - switches[i].pressStartTime) >= LONG_PRESS_DURATION_MS;
            handleSwitchAction(i+1, isLongPress);
          }
        }
      }

      // Check for ongoing long press
      if (switches[i].pressDetected && (now - switches[i].pressStartTime) >= LONG_PRESS_DURATION_MS) {
        switches[i].pressDetected = false; // Prevent repeated triggers
        handleSwitchAction(i+1, true);     // Trigger long press action
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to prevent busy waiting
  }
}
*/
//////////////////////////////////////////////////////////////////////////

// Turn ON device with short address 1
void turnOnAddress1() {
  uint8_t addr = DALI_ADDR(1); // Convert to DALI address format
  uint16_t command = (addr << 8) | DALI_CMD_ON_STEP_UP;
  
  uint8_t status = dali2.sendwait_int(command, 100); // 100ms timeout
  if(status != dali2.DALI_SUCCESS) {
    Serial.printf("Error sending ON command: %d\n", status);
  } else {
    Serial.println("ON command sent successfully to address 1");
  }
}

// Turn OFF device with short address 1
void turnOffAddress1() {
  uint8_t addr = DALI_ADDR(1); // Convert to DALI address format
  uint16_t command = (addr << 8) | DALI_CMD_OFF;
  
  uint8_t status = dali2.sendwait_int(command, 100); // 100ms timeout
  if(status != dali2.DALI_SUCCESS) {
    Serial.printf("Error sending OFF command: %d\n", status);
  } else {
    Serial.println("OFF command sent successfully to address 1");
  }
}


// Callback to handle received data on dali2 interface
void dali2_receiver(Dali *d, uint8_t *data, uint8_t len) {
  // Serial.print("RX: ");
  // if(len >= 2) Serial.println((int)(data[0]<<8) + data[1], HEX); 
  // else Serial.println((int)data[0], HEX);
}



void handleRoot() {
  Serial.println("Handling root request");
  
  String html = R"=====(
  <!DOCTYPE html>
  <html>
  <head>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>DALI Slave Configuration</title>
    <style>
      body { font-family: Arial, sans-serif; margin: 20px; }
      .container { display: flex; flex-wrap: wrap; }
      .column { flex: 50%; padding: 10px; box-sizing: border-box; }
      select { width: 100%; padding: 8px; margin: 8px 0; }
      button { 
        background-color: #4CAF50; 
        color: white; 
        padding: 10px 15px; 
        border: none; 
        cursor: pointer; 
        width: 100%;
        margin-bottom: 10px;
      }
      .toggle-btn {
        background-color: #008CBA;
      }
      .toggle-btn.on {
        background-color: #4CAF50;
      }
      .toggle-btn.off {
        background-color: #f44336;
      }
      button:hover { opacity: 0.8; }
      @media (max-width: 600px) { .column { flex: 100%; } }
    </style>
    <script>
  const slave1Addr = )=====" + String(dali2.slaves[0].shortAddress) + R"=====(;
  const slave2Addr = )=====" + String(dali2.slaves[1].shortAddress) + R"=====(;

  function updateDropdowns() {
    const slave1 = document.getElementById('slave1');
    const slave2 = document.getElementById('slave2');
    const currentSlave2 = slave2.value;

    let options = '';
    for(let i = parseInt(slave1.value) + 1; i <= 63; i++) {
      let selected = (i == slave2Addr) ? 'selected' : '';
      options += `<option value="${i}" ${selected}>${i}</option>`;
    }
    slave2.innerHTML = options;

    if(currentSlave2 > slave1.value) {
      slave2.value = currentSlave2;
    }
  }

  function submitForm(formId) {
    const form = document.getElementById(formId);
    const xhr = new XMLHttpRequest();
    xhr.open("POST", form.action, true);
    xhr.onload = function() {
      if(xhr.status == 200) {
        showAlert(xhr.responseText);
      } else {
        showAlert("Error: " + xhr.responseText);
      }
    };
    xhr.send(new FormData(form));
    return false;
  }

  function showAlert(message) {
    const alertBox = document.createElement('div');
    alertBox.style.position = 'fixed';
    alertBox.style.top = '20px';
    alertBox.style.left = '50%';
    alertBox.style.transform = 'translateX(-50%)';
    alertBox.style.backgroundColor = '#4CAF50';
    alertBox.style.color = 'white';
    alertBox.style.padding = '15px';
    alertBox.style.borderRadius = '5px';
    alertBox.style.zIndex = '1000';
    alertBox.style.boxShadow = '0 4px 8px rgba(0,0,0,0.2)';
    alertBox.textContent = message;

    document.body.appendChild(alertBox);

    setTimeout(() => {
      alertBox.style.opacity = '0';
      setTimeout(() => document.body.removeChild(alertBox), 500);
    }, 3000);
  }

  function toggleSlave(slaveNum) {
  const btn = document.getElementById('toggle' + slaveNum);
  const newState = btn.classList.contains('off') ? '1' : '0';

  // Get selected address directly from dropdown
  const addr = (slaveNum === 1)
    ? document.getElementById('slave1').value
    : document.getElementById('slave2').value;

  const xhr = new XMLHttpRequest();
  xhr.open("GET", `/control?addr=${addr}&value=${newState}`, true);
  xhr.onload = function() {
    if(xhr.status == 200) {
      if(newState == '1') {
        btn.classList.remove('off');
        btn.classList.add('on');
        btn.textContent = 'Slave ' + slaveNum + ' ON';
      } else {
        btn.classList.remove('on');
        btn.classList.add('off');
        btn.textContent = 'Slave ' + slaveNum + ' OFF';
      }
      showAlert(xhr.responseText);
    } else {
      showAlert("Error: " + xhr.responseText);
    }
  };
  xhr.send();
}
</script>
  </head>
  <body>
    <h2>DALI Slave Configuration</h2>
    <div class="container">
      <div class="column">
        <h3>Slave 1 Address</h3>
        <form id="form1" action="/save1" method="post" onsubmit="return submitForm('form1')">
          <select id="slave1" name="slave1" onchange="updateDropdowns()">
  )=====";
  
  // Generate options for slave1 (1-62)
  for(int i = 1; i <= 62; i++) {
    html += "<option value='" + String(i) + "'";
    if(i == dali2.slaves[0].shortAddress) html += " selected";
    html += ">" + String(i) + "</option>";
  }
  
  html += R"=====(
          </select>
          <button type="submit">Save Address 1</button>
          <button id="toggle1" class="toggle-btn off" type="button" onclick="toggleSlave(1)">
            Slave 1 OFF
          </button>
        </form>
      </div>
      
      <div class="column">
        <h3>Slave 2 Address</h3>
        <form id="form2" action="/save2" method="post" onsubmit="return submitForm('form2')">
          <select id="slave2" name="slave2">
  )=====";
  
  // Generate options for slave2 (slave1Addr+1 to 63)
  for(int i = dali2.slaves[0].shortAddress + 1; i <= 63; i++) {
    html += "<option value='" + String(i) + "'";
    if(i == dali2.slaves[1].shortAddress) html += " selected";
    html += ">" + String(i) + "</option>";
  }
  
  html += R"=====(
          </select>
          <button type="submit">Save Address 2</button>
          <button id="toggle2" class="toggle-btn off" type="button" onclick="toggleSlave(2)">
            Slave 2 OFF
          </button>
        </form>
      </div>
    </div>
  </body>
  </html>
  )=====";
  
  server.send(200, "text/html", html);
}

void handleSave1() {
  Serial.println("Handling save1 request");
  if(server.hasArg("slave1")) {
    int newAddr = server.arg("slave1").toInt();
    Serial.print("newAddr1: "); Serial.println(newAddr, HEX);
    if(newAddr >= 1 && newAddr <= 62) {
      dali2.slaves[0].shortAddress = newAddr;
      dali2.writeAllToEEPROM();
      
      // Update slave2 if needed
      if(dali2.slaves[1].shortAddress <= dali2.slaves[0].shortAddress) {
        dali2.slaves[1].shortAddress = dali2.slaves[0].shortAddress + 1;
        dali2.writeAllToEEPROM();
      }
      server.send(200, "text/plain", "Slave 1 address saved successfully!!");
      Serial.println("Saved Slave 1 address: " + String(dali2.slaves[0].shortAddress));
    } else {
      server.send(400, "text/plain", "Invalid address (must be 1-62)");
      Serial.println("Invalid Slave 1 address attempted: " + String(newAddr));
    }
  } else {
    server.send(400, "text/plain", "Missing address parameter");
    Serial.println("Missing Slave 1 address parameter");
  }
}

void handleSave2() {
  Serial.println("Handling save2 request");
  if(server.hasArg("slave2")) {
    int newAddr = server.arg("slave2").toInt();
    Serial.print("newAddr2: "); Serial.println(newAddr, HEX);
    if(newAddr >= 1 && newAddr <= 63) {
      dali2.slaves[1].shortAddress = newAddr;
      dali2.writeAllToEEPROM();
      server.send(200, "text/plain", "Slave 2 address saved successfully!!");
      Serial.println("Saved Slave 2 address: " + String(dali2.slaves[1].shortAddress));
    } else {
      server.send(400, "text/plain", "Invalid address (must be > Slave 1 and <= 63)");
      Serial.println("Invalid Slave 2 address attempted: " + String(newAddr));
    }
  } else {
    server.send(400, "text/plain", "Missing address parameter");
    Serial.println("Missing Slave 2 address parameter");
  }
}

void handleControl() {
  Serial.println("Handling control request");
  if(server.hasArg("addr") && server.hasArg("value")) {
    uint8_t addr = server.arg("addr").toInt();
    bool value = server.arg("value").toInt() == 1;
    
    // Determine which slave is being controlled
    if(addr == dali2.slaves[0].shortAddress) {
      dali2.slaves[0].actualLevel = value;
      dali2.slaves[0].addr_index = 0x01; dali2.slaves[1].addr_index = 0x00;
      dali2.setActualLevel(0, value);
      Serial.println("Slave 1 (" + String(addr) + ") set to: " + String(value));
      server.send(200, "text/plain", "Slave 1 control successful!");
    } 
    else if(addr == dali2.slaves[1].shortAddress) {
      dali2.slaves[1].actualLevel = value;
      dali2.slaves[0].addr_index = 0x00; dali2.slaves[1].addr_index = 0x01;
      dali2.setActualLevel(1, value);
      Serial.println("Slave 2 (" + String(addr) + ") set to: " + String(value));
      server.send(200, "text/plain", "Slave 2 control successful!");
    }
    else {
      server.send(400, "text/plain", "Invalid address");
      Serial.println("Invalid control address: " + String(addr));
    }
  } else {
    server.send(400, "text/plain", "Missing parameters");
    Serial.println("Missing control parameters");
  }
}

void handleNotFound() {
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i = 0; i < server.args(); i++) {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
  Serial.println(message);
}

void setup() {
  Serial.begin(115200);
  Serial.println("DALI Master/Slave Demo");

// Configure switch pins with pull-up resistors
  pinMode(SWITCH1_PIN, INPUT_PULLUP);
  pinMode(SWITCH2_PIN, INPUT_PULLUP);

 // Create switch monitoring task
  xTaskCreate(
    switchTask,       // Task function
    "SwitchTask",     // Task name
    2048,            // Stack size
    NULL,            // Parameter
    1,               // Priority
    &switchTaskHandle // Task handle
  ); 

  //dali1.begin(18, -1); // Using GPIO18 for TX, no RX
  dali2.begin(3, 19); // Using GPIO19 for RX, no TX
  
  // Attach a received data handler
  dali2.EventHandlerReceivedData = dali2_receiver;

  //if(dali2.slaves[0].shortAddress == 0xFF || dali2.slaves[0].shortAddress > 63) dali2.slaves[0].shortAddress = 1;
  //if(dali2.slaves[1].shortAddress == 0xFF || dali2.slaves[1].shortAddress > 63) dali2.slaves[1].shortAddress = 2;

  // Ensure slave2 > slave1
  if(dali2.slaves[1].shortAddress <= dali2.slaves[0].shortAddress) {
    dali2.slaves[1].shortAddress = dali2.slaves[0].shortAddress + 1;
    if(dali2.slaves[1].shortAddress > 63) dali2.slaves[1].shortAddress = 63;
  }

  // Start AP
  WiFi.softAP("NUOS DALI Slave", "DALI1234");
  Serial.println("AP IP: " + WiFi.softAPIP().toString());

  // Server routes
  server.on("/", HTTP_GET, handleRoot);
  server.on("/save1", HTTP_POST, handleSave1);
  server.on("/save2", HTTP_POST, handleSave2);
  server.on("/control", HTTP_GET, handleControl);
  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("Http Server Started!!");

}

void loop() {
  server.handleClient();
  vTaskDelay(1 / portTICK_PERIOD_MS);

}
