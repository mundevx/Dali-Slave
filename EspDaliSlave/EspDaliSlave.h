
#include <Arduino.h>
#include "driver/gpio.h"
#include <inttypes.h>

#include <esp_timer.h>
#include <functional>
#include <EEPROM.h>

#define DALI_SLAVE_ADDRESS      0x01

// In your header file:
#define EEPROM_SIZE 64
#define EEPROM_ADDR_SHORT_ADDR_1   0  // Slave 1 short address (1 byte)
#define EEPROM_ADDR_RANDOM_ADDR_1  1  // Slave 1 random address (3 bytes)
#define EEPROM_ADDR_SHORT_ADDR_2   4  // Slave 2 short address (1 byte)
#define EEPROM_ADDR_RANDOM_ADDR_2  5  // Slave 2 random address (3 bytes)
#define EEPROM_ADDR_SLAVE_STRUCT   0



enum {
	CMD_RESET				= 0b00100000,
	CMD_INITIALIZE			= 0b10100101,
	CMD_RANDOMIZE			= 0b10100111,
	CMD_SEARCHADDRH			= 0b10110001,
	CMD_SEARCHADDRM			= 0b10110011,
	CMD_SEARCHADDRL			= 0b10110101,
	CMD_COMPARE				= 0b10101001,
	CMD_WITHDRAW			= 0b10101011,
	CMD_TERMINATE			= 0b10100001,
	CMD_PROGRAMSHORT	= 0b10110111,
	CMD_VERIFYSHORT	= 0b10111001,

	CONTROL_BROADCAST	= 0b11111110,
};

/*		
// DALI Commissioning commands
#define CMD_INITIALIZE 0xA5
#define CMD_RANDOMIZE 0xA7
#define CMD_COMPARE 0xA9
#define CMD_WITHDRAW 0xAB
#define CMD_SEARCHADDRH 0xB1
#define CMD_SEARCHADDRM 0xB3
#define CMD_SEARCHADDRL 0xB5
#define CMD_PROGRAMSHORT 0xB7
#define CMD_VERIFYSHORT 0xB9
#define CMD_QUERYSHORT 0xBB

*/

#define DALI_RELAY         0
#define DALI_DIMMER        1
#define DEVICE_TYPE        DALI_RELAY
//#define DEVICE_TYPE        DALI_DIMMER


class Dali {
public:

  enum ErrorCode {
    DALI_SUCCESS = 0,
    DALI_ERR_BUSY,
    DALI_ERR_MSG_TOO_LONG,
    DALI_ERR_TIMEOUT
  };
  
  typedef void (*EventHandlerReceivedDataFuncPtr)(Dali *sender, uint8_t *data, uint8_t len);
  EventHandlerReceivedDataFuncPtr EventHandlerReceivedData;
  /*
	struct Slave {
        uint8_t shortAddress;
        uint32_t randomAddress;
        bool addressAssigned;
        uint8_t groups[2];
        uint8_t scenes[16];
		uint8_t actualLevel;
		uint8_t addr_index;
    };
    
    Slave slaves[2];  // Two independent slaves
*/

struct {
    uint8_t shortAddress;
    uint32_t randomAddress;
    uint8_t groups[2];
    uint8_t scenes[16];
    uint8_t actualLevel;
    bool addressAssigned;
    bool inCommissioningMode;  // Add this line
    uint8_t addr_index;
} slaves[2];
	
  void begin(int8_t tx_pin, int8_t rx_pin);
  uint8_t send(uint8_t* tx_msg, uint8_t tx_len_bytes);
  uint8_t sendwait(uint8_t* tx_msg, uint8_t tx_len_bytes, uint32_t timeout_ms=500);
  uint8_t sendwait_int(uint16_t tx_msg, uint32_t timeout_ms=500);
  uint8_t sendwait_byte(uint8_t tx_msg, uint32_t timeout_ms=500);
  void IRAM_ATTR ISR_timer();
  void IRAM_ATTR ISR_pinchange();

  #define DALI_HOOK_COUNT 3

	// DALI state variables
	uint8_t shortAddress = DALI_SLAVE_ADDRESS;
	uint8_t groups[2] = {0, 0}; // Groups 0-7 and 8-15
	uint8_t scenes[16] = {0};
	uint8_t dtr = 0; // Data Transfer Register
	uint8_t actualLevel = 0;
	uint8_t maxLevel = 254;
	uint8_t minLevel = 1;
	uint8_t powerOnLevel = 254;
	uint8_t systemFailureLevel = 254;
	uint8_t fadeTime = 1; // Default fade time
	uint8_t fadeRate = 1; // Default fade rate
	uint8_t status = 0;
	uint8_t deviceType = 0x81; // Default to Switch Device

	// Physical parameters
	uint8_t physicalMinLevel = 1;
	
	 // Helper functions
    void processCommand(uint8_t* data);
    void sendResponse(uint8_t response);
    bool checkAddress(uint8_t addressByte);
    void setActualLevel(uint8_t addr_index, uint8_t level);

	bool set_level_flag = false;

#define LED1        GPIO_NUM_21
#define LED2        GPIO_NUM_12
#define LOAD_3_PIN  GPIO_NUM_20
#define LOAD_4_PIN  GPIO_NUM_7

const gpio_num_t led_pins[2] = {LED1, LED2};
const gpio_num_t load_pins[2] = {LOAD_3_PIN, LOAD_4_PIN};

uint8_t public_addr_index = 0;
void writeAllToEEPROM();
void eraseAllEEPROM();
void readAllFromEEPROM();
void saveShortAddr(uint8_t addr);
void factoryReset();
private:
  //static constexpr size_t EEPROM_SIZE_SLAVES = sizeof(slaves);
  enum tx_stateEnum { IDLE=0,START,START_X,BIT,BIT_X,STOP1,STOP1_X,STOP2,STOP2_X,STOP3};
  uint8_t tx_pin;
  uint8_t tx_msg[3];
  uint8_t tx_len;
  volatile uint8_t tx_pos;
  volatile tx_stateEnum tx_state;
  volatile uint8_t tx_collision;
  volatile uint8_t tx_bus_low;
 uint32_t getSearchAddress();
	uint8_t txBuffer[1] = {0};
	uint8_t txBitCount = 0;
	uint8_t txByteCount = 0;
    void beginEEPROM();
// Commissioning state
    bool inCommissioningMode = false;
    uint32_t randomAddress = 0;
    uint8_t searchAddress[3] = {0}; // H, M, L
    
    // EEPROM methods
    void loadAddressesFromEEPROM();
    //void saveAddressToEEPROM();
	void saveSlaveToEEPROM(uint8_t slaveIndex);
	
// Write all slaves to EEPROM

    uint8_t activeSlaveIndex;  // Currently addressed slave
    bool isInGroup(uint8_t groupNum, uint8_t index);
	
    // Commissioning methods
    void generateRandomAddress(uint8_t slaveIndex);
    void handleCommissioningCommand(uint8_t cmd, uint8_t data);
    bool compareWithSearchAddress(uint8_t slaveIndex);
bool shouldRespondToCompare(uint8_t slaveIndex);
bool isExactMatch(uint8_t slaveIndex);
	
  enum rx_stateEnum { RX_IDLE,RX_START,RX_BIT};
  uint8_t rx_pin;
  volatile uint8_t rx_last_bus_low;
  volatile uint32_t rx_last_change_ts;
  volatile rx_stateEnum rx_state;
  volatile uint8_t rx_msg[3];
  volatile int8_t rx_len;
  volatile uint8_t rx_last_halfbit;

  volatile uint8_t bus_idle_te_cnt;

  void push_halfbit(uint8_t bit);
  
  gptimer_handle_t gptimer = NULL;
  portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
};