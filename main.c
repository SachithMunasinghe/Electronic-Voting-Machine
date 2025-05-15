#include <avr/io.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <string.h>
#include <stdlib.h>

// Define LCD I2C address (for example, 0x27, but this might vary)
#define LCD_ADDR 0x27

// Pins for the LEDs, debug and Buzzer
#define LED_PIN_Reg PD3  	// LED connected to PD3
#define LED_PIN_Vote PD5 	// LED connected to PD5
#define LED_PIN_RR PD6		// LED connected to PD6
#define Buzzer_PIN PD7		// Buzzer connected to PD7
#define DEBUG_PIN PD4

// Pins for buttons
#define Button_PIN_A PC0	// Button for vote A
#define Button_PIN_B PC1	// Button for vote B
#define Button_PIN_C PC2	// Button for vote C
#define Button_PIN_D PC3	// Button for vote D
#define Button_PIN_Reg PB0	// Button for register
#define Button_PIN_St PB1	// Button for start
#define Button_PIN_Result PB6	// Button for result
#define Button_PIN_Reset PB7	// Button for reset

#define MAX_LEN 16
#define SCROLL_DELAY 200 // Adjust scroll delay for smoothness

// Status codes
#define MI_OK                 0
#define MI_NOTAGERR           1
#define MI_ERR                2

// Admin ID (hardcoded)
uint8_t adminID[5] = {0xA3, 0x9B, 0x42, 0x28, 0x52};

// EEPROM storage for user IDs and user count
uint8_t userCount EEMEM = 0;  // Stored in EEPROM
uint8_t storedIDs[20][5] EEMEM; // Can store up to 20 IDs

// RAM storage for voting status (not using EEPROM)
uint8_t votedUsers[20] = {0};  // 0: not voted, 1: already voted

// Vote counters stored in variables
uint8_t voteA = 0;
uint8_t voteB = 0;
uint8_t voteC = 0;
uint8_t voteD = 0;

// MFRC522 command set
#define PCD_IDLE              0x00
#define PCD_AUTHENT           0x0E
#define PCD_RECEIVE           0x08
#define PCD_TRANSMIT          0x04
#define PCD_TRANSCEIVE        0x0C
#define PCD_RESETPHASE        0x0F
#define PCD_CALCCRC           0x03

// Mifare_One card command set
#define PICC_REQIDL           0x26
#define PICC_REQALL           0x52
#define PICC_ANTICOLL         0x93

// MFRC522 registers
#define CommandReg            0x01
#define CommIEnReg            0x02
#define CommIrqReg            0x04
#define ErrorReg              0x06
#define Status1Reg            0x07
#define Status2Reg            0x08
#define FIFODataReg           0x09
#define FIFOLevelReg          0x0A
#define ControlReg            0x0C
#define BitFramingReg         0x0D
#define ModeReg               0x11
#define TxModeReg             0x12
#define RxModeReg             0x13
#define TxControlReg          0x14
#define TxASKReg              0x15
#define TModeReg              0x2A
#define TPrescalerReg         0x2B
#define TReloadRegH           0x2C
#define TReloadRegL           0x2D

// Function prototypes
void initSPI(void);
void initButton(void);
void SPI_send(char data);
char SPI_receive(void);
void initLEDs_Buzzer(void);
void blinkLED_Buzzer(uint8_t led_pin);
void LCD_Init(void);
void LCD_SendCommand(uint8_t cmd);
void LCD_SendData(uint8_t data);
void LCD_Print(char *str);
void LCD_ScrollText(const char *str, uint8_t row, uint8_t scrollDist, uint8_t *index);
void LCD_SetPosition(uint8_t row, uint8_t col);
void LCD_Clear(void);
void MFRC522_init(void);
void MFRC522_reset(void);
void MFRC522_write(uint8_t addr, uint8_t val);
uint8_t MFRC522_read(uint8_t addr);
void MFRC522_setBitMask(uint8_t reg, uint8_t mask);
void MFRC522_clearBitMask(uint8_t reg, uint8_t mask);
void MFRC522_antennaOn(void);
uint8_t MFRC522_request(uint8_t reqMode, uint8_t *TagType);
uint8_t MFRC522_toCard(uint8_t command, uint8_t *sendData, uint8_t sendLen, uint8_t *backData, uint8_t *backLen);
uint8_t MFRC522_anticoll(uint8_t *serNum);
uint8_t isRegistered(uint8_t *serial);
void storeCard(uint8_t *serial);
uint8_t checkAdmin(uint8_t *serial);
uint8_t getUserIndex(uint8_t *serial);
void i2c_init(void);
void i2c_start(uint8_t address);
void i2c_stop(void);
void i2c_write(uint8_t data);
int8_t isButtonPressed(uint8_t button_pin, uint8_t port);
void resetEEPROM(void);  // Function for EEPROM reset

int main(void) {
        uint8_t serial[5];
	uint8_t status;
    	uint8_t str[MAX_LEN];
    	uint8_t scrollIndex = 0;  // Keeps track of the scroll position

    	initSPI();
    	initLEDs_Buzzer(); // Initialize both LEDs	
    	MFRC522_init();
    	i2c_init();  // Initialize I2C communication for LCD:
    	LCD_Init();
    	initButton(); 
        blinkLED_Buzzer(LED_PIN_Vote);
	// Main loop: Scrolling display until register is pressed
initial_state:
    while (1) {
        // Non-blocking scrolling for first line
        LCD_ScrollText("Electronic Voting Machine", 0, 1, &scrollIndex);  // First line scrolls
        LCD_SetPosition(1, 0);  // Fixed message on second line
        LCD_Print("Press register");
        
        // Check for reset button press
        if (isButtonPressed(Button_PIN_Reset, 'B')) {
            blinkLED_Buzzer(LED_PIN_RR);
            LCD_Clear();
            LCD_SetPosition(0, 0);
            LCD_Print("Tap Admin ID");

            // Set up a timer for 1 seconds
            uint16_t timeout = 1000 / 10;  // 2000ms, with _delay_ms(10)

            while (timeout > 0) {
                // Actively scan for RFID card
                status = MFRC522_request(0x26, serial);  // Request RFID card scan
                if (status == MI_OK && MFRC522_anticoll(serial) == MI_OK) {
                    if (checkAdmin(serial)) {
                        // Admin verified, reset EEPROM
                        LCD_Clear();
                        LCD_SetPosition(0, 0);
                        LCD_Print("Admin Verified");
                        blinkLED_Buzzer(LED_PIN_RR);
                        _delay_ms(50);
                        
                        LCD_Clear();
                        LCD_SetPosition(0, 0);
                        LCD_Print("Please wait");
                        resetEEPROM();  // Clear EEPROM after Admin verification
                        LCD_Clear();
                        LCD_SetPosition(0, 0);
                        LCD_Print("System Reseted");
                        LCD_SetPosition(1, 0);
                        LCD_Print("Successfully");
                        blinkLED_Buzzer(LED_PIN_RR);
                        _delay_ms(50);
                        break;  // Exit loop after reset
                    } else {
                        // Admin authentication failed
                        LCD_Clear();
                        LCD_SetPosition(0, 0);
                        LCD_Print("Auth Failed");
                        blinkLED_Buzzer(LED_PIN_RR);
                        _delay_ms(50);
                        break;  // Exit loop after failed auth
                    }
                }

                // Reduce the timeout and wait for 10ms between scans
                _delay_ms(10);
                timeout--;
            }
        }
        
        // Check for result button press
        if (isButtonPressed(Button_PIN_Result, 'B')) {
            uint8_t totalVoted = voteA + voteB + voteC + voteD;
            if (totalVoted > 0) {
                // Admin authentication for displaying result
                blinkLED_Buzzer(LED_PIN_RR);
                LCD_Clear();
                LCD_SetPosition(0, 0);
                LCD_Print("Tap Admin ID");

                // Wait for admin authentication with timeout (0.5 second)
                uint16_t timeout = 500 / 10;  // 1000ms, with _delay_ms(10)
                uint8_t adminVerified = 0;

                while (timeout > 0) {
                    status = MFRC522_request(0x26, serial);  // Request RFID card scan
                    if (status == MI_OK && MFRC522_anticoll(serial) == MI_OK) {
                        if (checkAdmin(serial)) {
                            adminVerified = 1;
                            // Admin authenticated, display result
                            blinkLED_Buzzer(LED_PIN_RR);

                            // Show the result for 6 seconds (3 alternations of 2 seconds)
                            uint16_t resultDisplayTime = 6000 / 1000; // 6 seconds in 1-second increments
                            while (resultDisplayTime > 0) {
                                // Display the total votes for 1 second
                                LCD_Clear();
                                LCD_SetPosition(0, 0);
                                char resultMsg1[MAX_LEN];
                                snprintf(resultMsg1, MAX_LEN, "  A:%d      B:%d", voteA, voteB);
                                LCD_Print(resultMsg1);
                                LCD_SetPosition(1, 0);
                                char resultMsg2[MAX_LEN];
                                snprintf(resultMsg2, MAX_LEN, "  C:%d      D:%d", voteC, voteD);
                                LCD_Print(resultMsg2);
                                _delay_ms(1000);

                                // Display the registered and voted count for 1 second
                                LCD_Clear();
                                LCD_SetPosition(0, 0);
                                char regMsg[MAX_LEN];
                                snprintf(regMsg, MAX_LEN, "Registered :%d", eeprom_read_byte(&userCount));
                                LCD_Print(regMsg);
                                LCD_SetPosition(1, 0);
                                char votedMsg[MAX_LEN];
                                snprintf(votedMsg, MAX_LEN, "Voted :%d", totalVoted);
                                LCD_Print(votedMsg);
                                _delay_ms(1000);

                                // Decrement the result display time by 2 seconds (1 second per view)
                                resultDisplayTime -= 2;
                            }
                            break;
                        }
                    }
                    _delay_ms(10);
                    timeout--;
                }

                if (!adminVerified) {
                    // Admin authentication failed or timeout
                    LCD_Clear();
                    LCD_SetPosition(0, 0);
                    LCD_Print("Auth Failed");
                    blinkLED_Buzzer(LED_PIN_RR);
                    _delay_ms(50);
                }
            } else {
                // No results to display
                LCD_Clear();
                LCD_SetPosition(0, 0);
                LCD_Print("No Results");
                LCD_SetPosition(1, 0);
                LCD_Print("To Display");
                blinkLED_Buzzer(LED_PIN_RR);
                _delay_ms(100); // Display "No results" message for 2 seconds
            }
        }
        
        // Check for register button press frequently for quick response
        if (isButtonPressed(Button_PIN_Reg, 'B')) {
            blinkLED_Buzzer(LED_PIN_Reg);
            LCD_Clear();  // Clear the screen when registration button is pressed
            // Enter registration mode when register button is pressed
            LCD_SetPosition(0, 0);
            LCD_Print("Tap Admin RFID");
            

            while (1) {
                status = MFRC522_request(0x26, serial);  // Request IDLE cards
                if (status == MI_OK && MFRC522_anticoll(serial) == MI_OK) {
                    if (checkAdmin(serial)) {
                        blinkLED_Buzzer(LED_PIN_RR);
                        LCD_Clear();
                        LCD_SetPosition(0, 0);
                        LCD_Print("Admin Verified");
                        _delay_ms(100);
                        
                        LCD_Clear();  // Clear the screen for the next message
                        LCD_SetPosition(0, 0);
                        LCD_Print("Registration");

                        LCD_SetPosition(1, 0);
                        LCD_Print("Tap your RFID");

                        // Registration Mode: Register users
                        while (!isButtonPressed(Button_PIN_St, 'B')) {
                            //LCD_Clear();  // Clear the screen for the next message
                            LCD_SetPosition(0, 0);
                            LCD_Print("Registration");
                            LCD_SetPosition(1, 0);
                            LCD_Print("Tap your RFID");
                            
                            status = MFRC522_request(0x26, serial);
                            if (status == MI_OK && MFRC522_anticoll(serial) == MI_OK) {
                                if (!isRegistered(serial)) {
                                    storeCard(serial);
                                    LCD_Clear();  // Clear the screen after successful registration
                                    LCD_SetPosition(0, 0);
                                    LCD_Print("Registration");
                                    LCD_SetPosition(1, 0);
                                    LCD_Print("Successful");
                                    blinkLED_Buzzer(LED_PIN_Reg);
                                    _delay_ms(100);
                                } else {
                                    
                                    LCD_Clear();  // Clear the screen if already registered
                                    LCD_SetPosition(0, 0);
                                    LCD_Print("Already");
                                    LCD_SetPosition(1, 0);
                                    LCD_Print("Registered");
                                    blinkLED_Buzzer(LED_PIN_Reg);
                                    _delay_ms(100);
                        
                                }
                            }
                        }
                        break;
                    } else {
                        // Admin authentication failed, retry
                        LCD_Clear();  // Clear the screen for retry
                        LCD_SetPosition(0, 0);
                        LCD_Print("Auth Failed");
                        LCD_SetPosition(1, 0);
                        LCD_Print("Try again");
                        blinkLED_Buzzer(LED_PIN_RR);
                        _delay_ms(100);
                        LCD_Clear(); 
                        LCD_SetPosition(0, 0);
                        LCD_Print("Tap Admin RFID");
                    }
                }
            }
        }
        
        // Check for Start button press
        if (isButtonPressed(Button_PIN_St, 'B')) {
            uint8_t userCountValue = eeprom_read_byte(&userCount); // Get the user count from EEPROM
            if (userCountValue == 0) {
                // No users registered, display a message and prevent voting
                blinkLED_Buzzer(LED_PIN_Vote);
                LCD_Clear();
                LCD_SetPosition(0, 0);
                LCD_Print("No Voters");
                LCD_SetPosition(1, 0);
                LCD_Print("Register first");
                _delay_ms(100);  // Show the message for 0.1 seconds
                continue;  // Return to the main loop
            }
            blinkLED_Buzzer(LED_PIN_RR);
            LCD_Clear();  // Clear the screen when start button is pressed

            // Admin authentication before starting voting
            LCD_SetPosition(0, 0);
            LCD_Print("Tap Admin ID");

            while (1) {
                status = MFRC522_request(0x26, serial);  // Request card scan
                if (status == MI_OK && MFRC522_anticoll(serial) == MI_OK) {
                    if (checkAdmin(serial)) {
                        blinkLED_Buzzer(LED_PIN_RR);
                        LCD_Clear();  // Clear the screen after successful admin authentication
                        LCD_SetPosition(0, 0);
                        LCD_Print("Admin Verified");
                        _delay_ms(100);  // Give some time to show verification message
                        break;  // Break to start voting after admin check
                    } else {
                        // Admin authentication failed, retry
                        LCD_Clear();  // Clear the screen for retry
                        LCD_SetPosition(0, 0);
                        LCD_Print("Auth Failed");
                        _delay_ms(100);
                        LCD_Clear(); 
                        LCD_SetPosition(0, 0);
                        LCD_Print("Tap Admin ID");
                    }
                }
            }
            // Voting mode begins here
            LCD_Clear();
            LCD_SetPosition(0, 0);
            LCD_Print("Tap your RFID");
            LCD_SetPosition(1, 0);
            LCD_Print("To Vote");
            
            while (1) {
                status = MFRC522_request(0x26, serial);  // Request RFID scan for user
                if (status == MI_OK && MFRC522_anticoll(serial) == MI_OK) {
                    uint8_t userIndex = getUserIndex(serial);  // Find the user in registered IDs
                    if (userIndex != 255) {  // User is registered
                        if (votedUsers[userIndex] == 0) {  // Check if the user has not voted
                            LCD_Clear();
                            LCD_SetPosition(0, 0);
                            LCD_Print("Please vote");
                            blinkLED_Buzzer(LED_PIN_Reg);
                            
                            while (1) {
                                if (isButtonPressed(Button_PIN_A, 'C')) {
                                    voteA++;
                                    blinkLED_Buzzer(LED_PIN_Vote);
                                    LCD_Clear();
                                    LCD_SetPosition(0, 0);
                                    LCD_Print("Thank for voting");
                                    votedUsers[userIndex] = 1;  // Mark the user as having voted
                                    _delay_ms(100);
                                    LCD_Clear();
                                    LCD_SetPosition(0, 0);
                                    LCD_Print("Tap your RFID");
                                    LCD_SetPosition(1, 0);
                                    LCD_Print("To Vote");
                                    break;
                                    
                                } else if (isButtonPressed(Button_PIN_B, 'C')) {
                                    voteB++;
                                    blinkLED_Buzzer(LED_PIN_Vote);
                                    LCD_Clear();
                                    LCD_SetPosition(0, 0);
                                    LCD_Print("Thank for voting");
                                    votedUsers[userIndex] = 1;  // Mark the user as having voted
                                    _delay_ms(100);
                                    LCD_Clear();
                                    LCD_SetPosition(0, 0);
                                    LCD_Print("Tap your RFID");
                                    LCD_SetPosition(1, 0);
                                    LCD_Print("To Vote");
                                    break;
                                    
                                } else if (isButtonPressed(Button_PIN_C, 'C')) {
                                    voteC++;
                                    blinkLED_Buzzer(LED_PIN_Vote);
                                    LCD_Clear();
                                    LCD_SetPosition(0, 0);
                                    LCD_Print("Thank for voting");
                                    votedUsers[userIndex] = 1;  // Mark the user as having voted
                                    _delay_ms(100);
                                    LCD_Clear();
                                    LCD_SetPosition(0, 0);
                                    LCD_Print("Tap your RFID");
                                    LCD_SetPosition(1, 0);
                                    LCD_Print("To Vote");
                                    break;
                                    
                                } else if (isButtonPressed(Button_PIN_D, 'C')) {
                                    voteD++;
                                    blinkLED_Buzzer(LED_PIN_Vote);
                                    LCD_Clear();
                                    LCD_SetPosition(0, 0);
                                    LCD_Print("Thank for voting");
                                    votedUsers[userIndex] = 1;  // Mark the user as having voted
                                    _delay_ms(100);
                                    LCD_Clear();
                                    LCD_SetPosition(0, 0);
                                    LCD_Print("Tap your RFID");
                                    LCD_SetPosition(1, 0);
                                    LCD_Print("To Vote");
                                    break;
                                }
                            }
                        } else {
                            LCD_Clear();
                            LCD_SetPosition(0, 0);
                            LCD_Print("Already voted");
                            blinkLED_Buzzer(LED_PIN_Reg);
                            _delay_ms(100);
                            
                            LCD_Clear();
                            LCD_SetPosition(0, 0);
                            LCD_Print("Tap your RFID");
                            LCD_SetPosition(1, 0);
                            LCD_Print("To Vote");
                        }
                    } else {
                        LCD_Clear();
                        LCD_SetPosition(0, 0);
                        LCD_Print("Invalid Vote");
                        blinkLED_Buzzer(LED_PIN_Reg);
                        _delay_ms(100);
                        
                        LCD_Clear();
                        LCD_SetPosition(0, 0);
                        LCD_Print("Tap your RFID");
                        LCD_SetPosition(1, 0);
                        LCD_Print("To Vote");
                    }
                }

                // Check for Result button press
                if (isButtonPressed(Button_PIN_Result, 'B')) {
                    LCD_Clear();
                    LCD_SetPosition(0, 0);
                    LCD_Print("Tap Admin ID");

                    // Wait for admin authentication with timeout (0.5 second)
                    uint16_t timeout = 500 / 10;  // 1000ms, with _delay_ms(10)
                    uint8_t adminVerified = 0;

                    while (timeout > 0) {
                        status = MFRC522_request(0x26, serial);  // Request RFID card scan
                        if (status == MI_OK && MFRC522_anticoll(serial) == MI_OK) {
                            if (checkAdmin(serial)) {
                                adminVerified = 1;
                                // Admin authenticated, display result
                                blinkLED_Buzzer(LED_PIN_RR);
                                uint8_t totalVoted = voteA + voteB + voteC + voteD;
                                uint8_t userNum = eeprom_read_byte(&userCount);
                                
                                // Show the result for 6 seconds (3 alternations of 2 seconds)
                                uint16_t resultDisplayTime = 6000 / 1000; // 6 seconds in 1-second increments
                                while (resultDisplayTime > 0) {
                                    // Display the total votes for 1 second
                                    LCD_Clear();
                                    LCD_SetPosition(0, 0);
                                    char resultMsg1[MAX_LEN];
                                    snprintf(resultMsg1, MAX_LEN, "  A:%d      B:%d", voteA, voteB);
                                    LCD_Print(resultMsg1);
                                    LCD_SetPosition(1, 0);
                                    char resultMsg2[MAX_LEN];
                                    snprintf(resultMsg2, MAX_LEN, "  C:%d      D:%d", voteC, voteD);
                                    LCD_Print(resultMsg2);
                                    _delay_ms(1000);

                                    // Display the registered and voted count for 1 second
                                    LCD_Clear();
                                    LCD_SetPosition(0, 0);
                                    char regMsg[MAX_LEN];
                                    snprintf(regMsg, MAX_LEN, "Registered :%d", userNum);
                                    LCD_Print(regMsg);
                                    LCD_SetPosition(1, 0);
                                    char votedMsg[MAX_LEN];
                                    snprintf(votedMsg, MAX_LEN, "Voted :%d", totalVoted);
                                    LCD_Print(votedMsg);
                                    _delay_ms(1000);

                                    // Decrement the result display time by 2 seconds (1 second per view)
                                    resultDisplayTime -= 2;
                                    
                                }
                                // After showing the result, return to the initial state (registration mode)
                                goto initial_state;
                            } else {
                                // Admin authentication failed
                                LCD_Clear();
                                LCD_SetPosition(0, 0);
                                LCD_Print("Auth Failed");
                                blinkLED_Buzzer(LED_PIN_RR);
                                _delay_ms(50);
                                break;  // Exit loop after failed auth
                            }
                        }

                        // Reduce the timeout and wait for 10ms between scans
                        _delay_ms(10);
                        timeout--;
                    }

                    if (!adminVerified) {
                        // Admin authentication failed or timeout
                        LCD_Clear();
                        LCD_SetPosition(0, 0);
                        LCD_Print("Tap your RFID");
                        LCD_SetPosition(1, 0);
                        LCD_Print("To Vote");
                    }
                }
            }
        }

        _delay_ms(SCROLL_DELAY); // Added delay to reduce processing overhead and enable smooth scrolling
    }
}

// Function to initialize buttons with internal pull-up resistors
void initButton(void) {
    // Set buttons connected to PORTC (A, B, C, D) as input with pull-up
    DDRC &= ~((1 << Button_PIN_A) | (1 << Button_PIN_B) | (1 << Button_PIN_C) | (1 << Button_PIN_D)); // Set PORTC pins as input
    PORTC |= (1 << Button_PIN_A) | (1 << Button_PIN_B) | (1 << Button_PIN_C) | (1 << Button_PIN_D);   // Enable pull-up resistors on PORTC pins

    // Set buttons connected to PORTB (Register, Start, Result, Reset) as input with pull-up
    DDRB &= ~((1 << Button_PIN_Reg) | (1 << Button_PIN_St) | (1 << Button_PIN_Result) | (1 << Button_PIN_Reset)); // Set PORTB pins as input
    PORTB |= (1 << Button_PIN_Reg) | (1 << Button_PIN_St) | (1 << Button_PIN_Result) | (1 << Button_PIN_Reset);   // Enable pull-up resistors on PORTB pins
}

void initSPI(void) {
    DDRB = (1<<PB3) | (1<<PB5) | (1<<PB2);
    SPCR = (1<<SPE) | (1<<MSTR) | (1<<SPR0);
}

void SPI_send(char data) {
    SPDR = data;
    while(!(SPSR & (1<<SPIF)));
}

char SPI_receive(void) {
    SPI_send(0x00);
    while(!(SPSR & (1<<SPIF)));
    return SPDR;
}

void MFRC522_init(void) {
    MFRC522_reset();
    MFRC522_write(TModeReg, 0x8D);
    MFRC522_write(TPrescalerReg, 0x3E);
    MFRC522_write(TReloadRegL, 30);
    MFRC522_write(TReloadRegH, 0);
    MFRC522_write(TxASKReg, 0x40);
    MFRC522_write(ModeReg, 0x3D);
    MFRC522_antennaOn();
}

void MFRC522_reset(void) {
    MFRC522_write(CommandReg, PCD_RESETPHASE);
}

void MFRC522_write(uint8_t addr, uint8_t val) {
    PORTB &= ~(1<<PB2);
    SPI_send((addr<<1)&0x7E);
    SPI_send(val);
    PORTB |= (1<<PB2);
}

uint8_t MFRC522_read(uint8_t addr) {
    uint8_t val;
    PORTB &= ~(1<<PB2);
    SPI_send(((addr<<1)&0x7E) | 0x80);
    val = SPI_receive();
    PORTB |= (1<<PB2);
    return val;
}

void MFRC522_setBitMask(uint8_t reg, uint8_t mask) {
    uint8_t tmp = MFRC522_read(reg);
    MFRC522_write(reg, tmp | mask);
}

void MFRC522_clearBitMask(uint8_t reg, uint8_t mask) {
    uint8_t tmp = MFRC522_read(reg);
    MFRC522_write(reg, tmp & (~mask));
}

void MFRC522_antennaOn(void) {
    uint8_t temp = MFRC522_read(TxControlReg);
    if (!(temp & 0x03)) {
        MFRC522_setBitMask(TxControlReg, 0x03);
    }
}

uint8_t MFRC522_request(uint8_t reqMode, uint8_t *TagType) {
    uint8_t status;
    uint8_t backBits;

    MFRC522_write(BitFramingReg, 0x07);
    TagType[0] = reqMode;
    status = MFRC522_toCard(PCD_TRANSCEIVE, TagType, 1, TagType, &backBits);

    if ((status != MI_OK) || (backBits != 0x10)) {
        status = MI_ERR;
    }

    return status;
}

uint8_t MFRC522_toCard(uint8_t command, uint8_t *sendData, uint8_t sendLen, uint8_t *backData, uint8_t *backLen) {
    uint8_t status = MI_ERR;
    uint8_t irqEn = 0x00;
    uint8_t waitIRq = 0x00;
    uint8_t lastBits;
    uint8_t n;
    uint8_t i;

    switch (command) {
        case PCD_AUTHENT:
            irqEn = 0x12;
            waitIRq = 0x10;
            break;
        case PCD_TRANSCEIVE:
            irqEn = 0x77;
            waitIRq = 0x30;
            break;
        default:
            break;
    }

    MFRC522_write(CommIEnReg, irqEn | 0x80);
    MFRC522_clearBitMask(CommIrqReg, 0x80);
    MFRC522_setBitMask(FIFOLevelReg, 0x80);

    MFRC522_write(CommandReg, PCD_IDLE);

    for (i = 0; i < sendLen; i++) {
        MFRC522_write(FIFODataReg, sendData[i]);
    }

    MFRC522_write(CommandReg, command);
    if (command == PCD_TRANSCEIVE) {
        MFRC522_setBitMask(BitFramingReg, 0x80);
    }

    i = 2000;
    do {
        n = MFRC522_read(CommIrqReg);
        i--;
    } while ((i != 0) && !(n & 0x01) && !(n & waitIRq));

    MFRC522_clearBitMask(BitFramingReg, 0x80);

    if (i != 0) {
        if (!(MFRC522_read(ErrorReg) & 0x1B)) {
            status = MI_OK;
            if (n & irqEn & 0x01) {
                status = MI_NOTAGERR;
            }

            if (command == PCD_TRANSCEIVE) {
                n = MFRC522_read(FIFOLevelReg);
                lastBits = MFRC522_read(ControlReg) & 0x07;
                if (lastBits) {
                    *backLen = (n - 1) * 8 + lastBits;
                } else {
                    *backLen = n * 8;
                }

                if (n == 0) {
                    n = 1;
                }
                if (n > MAX_LEN) {
                    n = MAX_LEN;
                }

                for (i = 0; i < n; i++) {
                    backData[i] = MFRC522_read(FIFODataReg);
                }
            }
        } else {
            status = MI_ERR;
        }
    }

    return status;
}

uint8_t MFRC522_anticoll(uint8_t *serNum) {
    uint8_t status;
    uint8_t i;
    uint8_t serNumCheck = 0;
    uint8_t unLen;

    MFRC522_write(BitFramingReg, 0x00);
    serNum[0] = PICC_ANTICOLL;
    serNum[1] = 0x20;
    status = MFRC522_toCard(PCD_TRANSCEIVE, serNum, 2, serNum, &unLen);

    if (status == MI_OK) {
        for (i = 0; i < 4; i++) {
            serNumCheck ^= serNum[i];
        }
        if (serNumCheck != serNum[i]) {
            status = MI_ERR;
        }
    }

    return status;
}

void initLEDs_Buzzer(void) {
    	// Initialize both LEDs and the buzzer
	DDRD |= (1<<LED_PIN_Reg) | (1<<LED_PIN_Vote) | (1<<LED_PIN_RR) | (1<<Buzzer_PIN);
}

void blinkLED_Buzzer(uint8_t led_pin) {
    // Blink the specified LED
	if (led_pin == LED_PIN_Reg) {
        	PORTD |= (1<<led_pin);
		PORTD |= (1<<Buzzer_PIN);
        	_delay_ms(50);         
        	PORTD &= ~(1<<led_pin);
		PORTD &= ~(1<<Buzzer_PIN);

    	} else if (led_pin == LED_PIN_Vote) {
        	PORTD |= (1<<led_pin);
		PORTD |= (1<<Buzzer_PIN);
        	_delay_ms(50);         
        	PORTD &= ~(1<<led_pin);
		PORTD &= ~(1<<Buzzer_PIN);

    	}else if (led_pin == LED_PIN_RR) {
		PORTD |= (1<<led_pin);
		PORTD |= (1<<Buzzer_PIN);
		_delay_ms(50);
		PORTD &= ~(1<<led_pin);
		PORTD &= ~(1<<Buzzer_PIN);
	}
}

void i2c_init(void) {
    // Initialize I2C (TWI) interface
    TWSR = 0x00; // Prescaler value of 1
    TWBR = ((F_CPU/100000UL) - 16) / 2; // SCL frequency 100kHz
}

void i2c_start(uint8_t address) {
    // Send start condition
    TWCR = (1<<TWSTA) | (1<<TWEN) | (1<<TWINT);
    while (!(TWCR & (1<<TWINT))); // Wait for start to be transmitted

    // Load slave address into data register
    TWDR = address;
    TWCR = (1<<TWEN) | (1<<TWINT); // Clear TWINT to start transmission
    while (!(TWCR & (1<<TWINT))); // Wait for end of transmission
}

void i2c_stop(void) {
    // Send stop condition
    TWCR = (1<<TWSTO) | (1<<TWEN) | (1<<TWINT);
    while (TWCR & (1<<TWSTO)); // Wait for stop to be transmitted
}

void i2c_write(uint8_t data) {
    TWDR = data;
    TWCR = (1<<TWEN) | (1<<TWINT);
    while (!(TWCR & (1<<TWINT))); // Wait for end of transmission
}

void LCD_Init(void) {
    // Initialize LCD (Assuming 4-bit mode)
    _delay_ms(50); // Wait for more than 40ms after VCC rises to 4.5V
    LCD_SendCommand(0x03);
    _delay_ms(5);  // Wait for more than 4.1ms
    LCD_SendCommand(0x03);
    _delay_us(150);
    LCD_SendCommand(0x03);
    LCD_SendCommand(0x02); // Set to 4-bit mode
    LCD_SendCommand(0x28); // Function set: 4-bit, 2 lines, 5x8 dots
    LCD_SendCommand(0x0C); // Display ON, cursor OFF, blink OFF
    LCD_SendCommand(0x06); // Entry mode set: increment automatically, no shift
    LCD_SendCommand(0x01); // Clear display
    _delay_ms(2);  // Delay after clearing display
}

void LCD_SendCommand(uint8_t cmd) {
    // Send a command to the LCD
    i2c_start(LCD_ADDR<<1);
    i2c_write((cmd & 0xF0) | 0x08); // Send high nibble
    i2c_write((cmd & 0xF0) | 0x0C); // Enable bit high
    i2c_write((cmd & 0xF0) | 0x08); // Enable bit low
    i2c_write((cmd << 4) | 0x08);   // Send low nibble
    i2c_write((cmd << 4) | 0x0C);   // Enable bit high
    i2c_write((cmd << 4) | 0x08);   // Enable bit low
    i2c_stop();
}

void LCD_SendData(uint8_t data) {
    // Send data to the LCD
    i2c_start(LCD_ADDR<<1);
    i2c_write((data & 0xF0) | 0x09); // Send high nibble
    i2c_write((data & 0xF0) | 0x0D); // Enable bit high
    i2c_write((data & 0xF0) | 0x09); // Enable bit low
    i2c_write((data << 4) | 0x09);   // Send low nibble
    i2c_write((data << 4) | 0x0D);   // Enable bit high
    i2c_write((data << 4) | 0x09);   // Enable bit low
    i2c_stop();
}

void LCD_Print(char *str) {
    while (*str) {
        LCD_SendData(*str++);
    }
}

void LCD_SetPosition(uint8_t row, uint8_t col) {
    uint8_t pos = (row == 0) ? (0x80 + col) : (0xC0 + col);
    LCD_SendCommand(pos);
}

void LCD_Clear(void) {
    LCD_SendCommand(0x01);  // Send clear display command
    _delay_ms(2);           // Wait for the command to execute (it takes a bit of time)
}

// Scroll text on the specified row
void LCD_ScrollText(const char *str, uint8_t row, uint8_t scrollDist, uint8_t *index) {
    uint8_t len = strlen(str);
    char buffer[MAX_LEN + 1];

    char *scrollText = malloc(len + MAX_LEN + 1);
    if (scrollText == NULL) return;

    strcpy(scrollText, str);
    strcat(scrollText, "                "); // Adding extra spaces for smooth scroll

    // Copy the portion of the string to display
    strncpy(buffer, scrollText + *index, MAX_LEN);
    buffer[MAX_LEN] = '\0';  // Null-terminate

    LCD_SetPosition(row, 0);  // Set the row position
    LCD_Print(buffer);  // Print the current scroll position

    // Increment the scroll index
    (*index)++;
    if (*index >= len + scrollDist) {
        *index = 0;  // Reset the index once the entire text has scrolled
    }

    free(scrollText);  // Free the allocated memory
}

void storeCard(uint8_t *serial) {
    uint8_t userNum = eeprom_read_byte(&userCount);
    eeprom_write_block(serial, storedIDs[userNum], 5);
    eeprom_write_byte(&userCount, userNum + 1);
    votedUsers[userNum] = 0;  // Mark as not voted in RAM
}

uint8_t getUserIndex(uint8_t *serial) {
    uint8_t userNum = eeprom_read_byte(&userCount);
    uint8_t storedSerial[5];
    for (uint8_t i = 0; i < userNum; i++) {
        eeprom_read_block(storedSerial, storedIDs[i], 5);
        if (memcmp(serial, storedSerial, 5) == 0) {
            return i;  // Return the index if user is found
        }
    }
    return 255;  // Return 255 if user is not found (invalid)
}

uint8_t isRegistered(uint8_t *serial) {
    uint8_t userNum = eeprom_read_byte(&userCount);
    uint8_t storedSerial[5];
    for (int i = 0; i < userNum; i++) {
        eeprom_read_block(storedSerial, storedIDs[i], 5);
        if (memcmp(serial, storedSerial, 5) == 0) {
            return 1;
        }
    }
    return 0;
}

uint8_t checkAdmin(uint8_t *serial) {
    return memcmp(serial, adminID, 5) == 0;
}

int8_t isButtonPressed(uint8_t button_pin, uint8_t port) {
    static uint8_t previousState = 1;  // Keeps track of the previous button state (unpressed)
    uint8_t currentState;

    // Check if the button is connected to PORTB or PORTC
    if (port == 'B') {
        currentState = (PINB & (1 << button_pin)) ? 1 : 0;  // For buttons on PORTB
    } else if (port == 'C') {
        currentState = (PINC & (1 << button_pin)) ? 1 : 0;  // For buttons on PORTC
    }

    // Detect a falling edge (button press)
    if (previousState == 1 && currentState == 0) {
        _delay_ms(10);  // Debouncing delay
        if (currentState == 0) {  // Check if the button is still pressed
            previousState = 0;  // Update previous state to pressed
            return 1;  // Button was pressed
        }
    } else if (currentState == 1) {
        previousState = 1;  // Button is unpressed
    }

    return 0;  // Button is not pressed
}

void resetEEPROM(void) {
    // Reset the user count to 0
    eeprom_write_byte(&userCount, 0);
    _delay_ms(10); // Wait for EEPROM write to complete

    // Clear all stored IDs in EEPROM
    for (int i = 0; i < 20; i++) {
        uint8_t emptyID[5] = {0, 0, 0, 0, 0};
        eeprom_write_block(emptyID, storedIDs[i], 5);
        _delay_ms(10); // Wait for EEPROM write to complete
    }
    // Reset vote counts
    voteA = 0;
    voteB = 0;
    voteC = 0;
    voteD = 0;

    // Optionally reset the votedUsers array (if stored in EEPROM)
    for (int i = 0; i < 20; i++) {
        votedUsers[i] = 0;  // Reset in RAM
        // If you store votedUsers in EEPROM, add EEPROM persistence here:
        // eeprom_write_byte(&votedUsers[i], 0);
        _delay_ms(10); // Wait for EEPROM write to complete
    }
}
