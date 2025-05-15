#include <avr/io.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>  // Added for sprintf

// Define LCD I2C address (for example, 0x27, but this might vary)
#define LCD_ADDR 0x27

// Pins for the LEDs, debug, and Buzzer
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

// Status codes for RFID
#define MI_OK                 0
#define MI_NOTAGERR           1
#define MI_ERR                2

// Hardcoded Admin ID (example placeholder)
uint8_t adminID[5] = {0xDE, 0xAD, 0xBE, 0xEF, 0x01};

// EEPROM storage for user IDs
uint8_t userCount EEMEM = 0;
uint8_t storedIDs[20][5] EEMEM; // Can store up to 20 IDs

// Function prototypes
void initSPI(void);
void SPI_send(char data);
char SPI_receive(void);
void MFRC522_init(void);
void MFRC522_reset(void);
void MFRC522_write(uint8_t addr, uint8_t val);
uint8_t MFRC522_read(uint8_t addr);
void MFRC522_antennaOn(void);
uint8_t MFRC522_request(uint8_t reqMode, uint8_t *TagType);
uint8_t MFRC522_anticoll(uint8_t *serNum);
uint8_t MFRC522_toCard(uint8_t command, uint8_t *sendData, uint8_t sendLen, uint8_t *backData, uint8_t *backLen);
void MFRC522_clearBitMask(uint8_t reg, uint8_t mask);
void MFRC522_setBitMask(uint8_t reg, uint8_t mask);
void i2c_init(void);
void i2c_start(uint8_t address);
void i2c_stop(void);
void i2c_write(uint8_t data);
void LCD_Init(void);
void LCD_SendCommand(uint8_t cmd);
void LCD_SendData(uint8_t data);
void LCD_Print(char *str);
void LCD_SetPosition(uint8_t row, uint8_t col);
void LCD_Clear(void);
void displayUID(uint8_t *uid);

// Function to display a UID on the LCD
void displayUID(uint8_t *uid) {
    char uidStr[17];  // String to hold UID
    sprintf(uidStr, "%02X%02X%02X%02X%02X", uid[0], uid[1], uid[2], uid[3], uid[4]); // Format UID as hex string
    LCD_Clear();
    LCD_SetPosition(0, 0);
    LCD_Print("Admin UID:");
    LCD_SetPosition(1, 0);
    LCD_Print(uidStr);  // Display UID on the LCD
    _delay_ms(2000);    // Wait for 2 seconds
}

// Function to initialize the SPI interface
void initSPI(void) {
    DDRB = (1 << PB3) | (1 << PB5) | (1 << PB2);  // Set MOSI, SCK, and SS as output
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0); // Enable SPI as Master, set clock rate
}

// Function to send data over SPI
void SPI_send(char data) {
    SPDR = data;
    while (!(SPSR & (1 << SPIF))); // Wait for transmission complete
}

// Function to receive data from SPI
char SPI_receive(void) {
    SPI_send(0x00);  // Send dummy data to read from slave
    while (!(SPSR & (1 << SPIF))); // Wait for reception complete
    return SPDR;
}

// Function to initialize the RFID MFRC522 module
void MFRC522_init(void) {
    MFRC522_reset();
    MFRC522_write(0x2A, 0x8D);  // TModeReg
    MFRC522_write(0x2B, 0x3E);  // TPrescalerReg
    MFRC522_write(0x2C, 30);    // TReloadRegL
    MFRC522_write(0x2D, 0);     // TReloadRegH
    MFRC522_write(0x15, 0x40);  // TxASKReg
    MFRC522_write(0x11, 0x3D);  // ModeReg
    MFRC522_antennaOn();
}

// Reset the MFRC522 module
void MFRC522_reset(void) {
    MFRC522_write(0x01, 0x0F);  // PCD_RESETPHASE
}

// Write a value to a register of MFRC522
void MFRC522_write(uint8_t addr, uint8_t val) {
    PORTB &= ~(1 << PB2); // Select MFRC522
    SPI_send((addr << 1) & 0x7E); // Send address
    SPI_send(val);                // Send value
    PORTB |= (1 << PB2);          // Deselect MFRC522
}

// Read a value from a register of MFRC522
uint8_t MFRC522_read(uint8_t addr) {
    uint8_t val;
    PORTB &= ~(1 << PB2);          // Select MFRC522
    SPI_send(((addr << 1) & 0x7E) | 0x80);  // Send address + read command
    val = SPI_receive();           // Receive value
    PORTB |= (1 << PB2);           // Deselect MFRC522
    return val;
}

// Turn on the antenna for MFRC522
void MFRC522_antennaOn(void) {
    uint8_t temp = MFRC522_read(0x14); // TxControlReg
    if (!(temp & 0x03)) {
        MFRC522_write(0x14, temp | 0x03);
    }
}

// Request a tag from the RFID reader
uint8_t MFRC522_request(uint8_t reqMode, uint8_t *TagType) {
    uint8_t status;
    uint8_t backBits;  // The received bit count
    MFRC522_write(0x0D, 0x07);  // BitFramingReg
    TagType[0] = reqMode;
    status = MFRC522_toCard(0x0C, TagType, 1, TagType, &backBits);  // PCD_TRANSCEIVE
    if ((status != MI_OK) || (backBits != 0x10)) {
        status = MI_ERR;
    }
    return status;
}

// Anti-collision detection (get unique serial number)
uint8_t MFRC522_anticoll(uint8_t *serNum) {
    uint8_t status;
    uint8_t i;
    uint8_t serNumCheck = 0;
    uint8_t unLen;
    MFRC522_write(0x0D, 0x00);  // BitFramingReg
    serNum[0] = 0x93;  // PICC_ANTICOLL
    serNum[1] = 0x20;
    status = MFRC522_toCard(0x0C, serNum, 2, serNum, &unLen);  // PCD_TRANSCEIVE
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

// Send command to MFRC522 and handle response
uint8_t MFRC522_toCard(uint8_t command, uint8_t *sendData, uint8_t sendLen, uint8_t *backData, uint8_t *backLen) {
    uint8_t status = MI_ERR;
    uint8_t irqEn = 0x00;
    uint8_t waitIRq = 0x00;
    uint8_t lastBits;
    uint8_t n;
    uint16_t i;  // Changed to uint16_t to avoid integer truncation warning

    switch (command) {
        case 0x0E:  // PCD_AUTHENT
            irqEn = 0x12;
            waitIRq = 0x10;
            break;
        case 0x0C:  // PCD_TRANSCEIVE
            irqEn = 0x77;
            waitIRq = 0x30;
            break;
        default:
            break;
    }

    MFRC522_write(0x02, irqEn | 0x80); // CommIEnReg
    MFRC522_clearBitMask(0x04, 0x80); // CommIrqReg
    MFRC522_setBitMask(0x0A, 0x80);   // FIFOLevelReg

    MFRC522_write(0x01, 0x00);  // CommandReg

    for (i = 0; i < sendLen; i++) {
        MFRC522_write(0x09, sendData[i]);  // FIFODataReg
    }

    MFRC522_write(0x01, command); // CommandReg
    if (command == 0x0C) {  // PCD_TRANSCEIVE
        MFRC522_setBitMask(0x0D, 0x80);  // BitFramingReg
    }

    i = 2000;  // Timeout counter
    do {
        n = MFRC522_read(0x04);  // CommIrqReg
        i--;
    } while ((i != 0) && !(n & 0x01) && !(n & waitIRq));

    MFRC522_clearBitMask(0x0D, 0x80);  // BitFramingReg

    if (i != 0) {
        if (!(MFRC522_read(0x06) & 0x1B)) {  // ErrorReg
            status = MI_OK;
            if (n & irqEn & 0x01) {
                status = MI_NOTAGERR;
            }

            if (command == 0x0C) {  // PCD_TRANSCEIVE
                n = MFRC522_read(0x0A);  // FIFOLevelReg
                lastBits = MFRC522_read(0x0C) & 0x07;  // ControlReg
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
                    backData[i] = MFRC522_read(0x09);  // FIFODataReg
                }
            }
        } else {
            status = MI_ERR;
        }
    }

    return status;
}

// Set specific bits of a register
void MFRC522_setBitMask(uint8_t reg, uint8_t mask) {
    uint8_t tmp = MFRC522_read(reg);
    MFRC522_write(reg, tmp | mask);
}

// Clear specific bits of a register
void MFRC522_clearBitMask(uint8_t reg, uint8_t mask) {
    uint8_t tmp = MFRC522_read(reg);
    MFRC522_write(reg, tmp & (~mask));
}

// Function to initialize I2C
void i2c_init(void) {
    TWSR = 0x00; // Prescaler value of 1
    TWBR = ((F_CPU / 100000UL) - 16) / 2; // SCL frequency 100kHz
}

// Function to start I2C communication
void i2c_start(uint8_t address) {
    TWCR = (1 << TWSTA) | (1 << TWEN) | (1 << TWINT); // Send start condition
    while (!(TWCR & (1 << TWINT))); // Wait for start to be transmitted
    TWDR = address;  // Load address
    TWCR = (1 << TWEN) | (1 << TWINT); // Start transmission
    while (!(TWCR & (1 << TWINT))); // Wait for end of transmission
}

// Function to stop I2C communication
void i2c_stop(void) {
    TWCR = (1 << TWSTO) | (1 << TWEN) | (1 << TWINT);  // Send stop condition
    while (TWCR & (1 << TWSTO));  // Wait for stop to be transmitted
}

// Function to write data over I2C
void i2c_write(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT))); // Wait for end of transmission
}

// Function to initialize the LCD (Assuming 4-bit mode)
void LCD_Init(void) {
    _delay_ms(50);  // Wait for more than 40ms after VCC rises to 4.5V
    LCD_SendCommand(0x03);
    _delay_ms(5);
    LCD_SendCommand(0x03);
    _delay_us(150);
    LCD_SendCommand(0x03);
    LCD_SendCommand(0x02);  // Set to 4-bit mode
    LCD_SendCommand(0x28);  // Function set: 4-bit, 2 lines, 5x8 dots
    LCD_SendCommand(0x0C);  // Display ON, cursor OFF, blink OFF
    LCD_SendCommand(0x06);  // Entry mode set: increment automatically, no shift
    LCD_SendCommand(0x01);  // Clear display
    _delay_ms(2);  // Delay after clearing display
}

// Send a command to the LCD
void LCD_SendCommand(uint8_t cmd) {
    i2c_start(LCD_ADDR << 1);
    i2c_write((cmd & 0xF0) | 0x08);  // Send high nibble
    i2c_write((cmd & 0xF0) | 0x0C);  // Enable bit high
    i2c_write((cmd & 0xF0) | 0x08);  // Enable bit low
    i2c_write((cmd << 4) | 0x08);    // Send low nibble
    i2c_write((cmd << 4) | 0x0C);    // Enable bit high
    i2c_write((cmd << 4) | 0x08);    // Enable bit low
    i2c_stop();
}

// Send data to the LCD
void LCD_SendData(uint8_t data) {
    i2c_start(LCD_ADDR << 1);
    i2c_write((data & 0xF0) | 0x09);  // Send high nibble
    i2c_write((data & 0xF0) | 0x0D);  // Enable bit high
    i2c_write((data & 0xF0) | 0x09);  // Enable bit low
    i2c_write((data << 4) | 0x09);    // Send low nibble
    i2c_write((data << 4) | 0x0D);    // Enable bit high
    i2c_write((data << 4) | 0x09);    // Enable bit low
    i2c_stop();
}

// Print a string on the LCD
void LCD_Print(char *str) {
    while (*str) {
        LCD_SendData(*str++);
    }
}

// Set cursor position on the LCD
void LCD_SetPosition(uint8_t row, uint8_t col) {
    uint8_t pos = (row == 0) ? (0x80 + col) : (0xC0 + col);
    LCD_SendCommand(pos);
}

// Clear the LCD display
void LCD_Clear(void) {
    LCD_SendCommand(0x01);  // Send clear display command
    _delay_ms(2);           // Wait for the command to execute
}

int main(void) {
    uint8_t serial[5];
    uint8_t status;

    // Initialization
    initSPI();
    MFRC522_init();
    LCD_Init();
    i2c_init();

    // Main loop
    while (1) {
        LCD_Clear();
        LCD_SetPosition(0, 0);
        LCD_Print("Tap Admin Card");

        // Wait for a card to be tapped
        while (1) {
            status = MFRC522_request(0x26, serial);  // Request card
            if (status == MI_OK && MFRC522_anticoll(serial) == MI_OK) {
                // Display the tapped card's UID on the LCD
                displayUID(serial);
                break;  // Exit loop after one card tap
            }
        }

        _delay_ms(2000);  // Delay to give time for the user to see the UID
    }
}

