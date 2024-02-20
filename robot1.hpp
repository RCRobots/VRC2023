
// commands
const int LCD_CLEARDISPLAY = 0x01;
const int LCD_RETURNHOME = 0x02;
const int LCD_ENTRYMODESET = 0x04;
const int LCD_DISPLAYCONTROL = 0x08;
const int LCD_CURSORSHIFT = 0x10;
const int LCD_FUNCTIONSET = 0x20;
const int LCD_SETCGRAMADDR = 0x40;
const int LCD_SETDDRAMADDR = 0x80;

// flags for display entry mode
const int LCD_ENTRYSHIFTINCREMENT = 0x01;
const int LCD_ENTRYLEFT = 0x02;

// flags for display and cursor control
const int LCD_BLINKON = 0x01;
const int LCD_CURSORON = 0x02;
const int LCD_DISPLAYON = 0x04;

// flags for display and cursor shift
const int LCD_MOVERIGHT = 0x04;
const int LCD_DISPLAYMOVE = 0x08;

// flags for function set
const int LCD_5x10DOTS = 0x04;
const int LCD_2LINE = 0x08;
const int LCD_8BITMODE = 0x10;

// flag for backlight control
const int LCD_BACKLIGHT = 0x08;

const int LCD_ENABLE_BIT = 0x04;

#define LCD_DELAY_US 6000
// Modes for lcd_send_byte
#define LCD_CHARACTER  1
#define LCD_COMMAND    0
#define LCD_MAX_LINES  2
#define LCD_MAX_CHARS  16

// Character positions in LCD display
#define DB_STATUS_LINE 2
#define DB_IR_FRONT_LEFT 1
#define DB_IR_FRONT_RIGHT 2
#define DB_IR_FRONT_CENTER 3
#define DB_IR_FRONT_CENTER_LEFT 4
#define DB_IR_FRONT_CENTER_RIGHT 5
#define DB_US_LEFT 6
#define DB_US_RIGHT 7
#define DB_US_FRONT 8
#define DB_US_BACK 9
#define DB_WHEEL_ENCODEr 10
#define DB_PIR_PRESENT 11
#define DB_DRIVE_FORWARD 12
#define DB_TURN_LEFT 13
#define DB_TURN_RIGHT 14
#define DB_DRIVE_BACKUP 15





