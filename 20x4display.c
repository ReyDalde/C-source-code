/*
 * Sample program  by Rey S. Dalde
 * Oct. 16, 2021
 * With 20x4 LCD
 * Displaying date, time and system temperature
*/

#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>


// Define some device parameters
#define I2C_ADDR   0x27 // I2C device address

// Define some device constants
#define LCD_CHR  1 // Mode - Sending data
#define LCD_CMD  0 // Mode - Sending command

#define LINE1  0x80 // 1st line
#define LINE2  0xC0 // 2nd line
#define LINE3  0x94 // 3rd line
#define LINE4  0xD4 // 4th line

#define LCD_BACKLIGHT   0x08  // On
// LCD_BACKLIGHT = 0x00  # Off

#define ENABLE  0b00000100 // Enable bit

void lcd_init(void);
void lcd_byte(int bits, int mode);
void lcd_toggle_enable(int bits);

void lcdLoc(int line); //move cursor
void ClrLcd(void); // clr LCD return home
void LCDdisplayStr(const char *s); // display string in LCD
int fd;  // seen by all subroutines

int main()   {
  
  float millideg;
  FILE *thermal;
  int i; 
  char buf[20];        
   
  time_t rawtime;
  struct tm * timeinfo;

  if (wiringPiSetup () == -1) exit (1);

  fd = wiringPiI2CSetup(I2C_ADDR);

  lcd_init(); // setup LCD
  
  ClrLcd();
  
  while (1)   {
    
    thermal = fopen("/sys/class/thermal/thermal_zone0/temp","r");  
    fscanf(thermal,"%f",&millideg);
    fclose(thermal);
    
    
    time ( &rawtime );
    timeinfo = localtime ( &rawtime ); //get current date and time
    lcdLoc(LINE1);
    LCDdisplayStr("LCD Disp I2C Program");
    lcdLoc(LINE2+3);
    LCDdisplayStr("in C language");
    lcdLoc(LINE3);
    LCDdisplayStr(asctime (timeinfo)); //display date & time
    
    lcdLoc(LINE4);
    LCDdisplayStr("Temp.:");
    for (i=0;i++;i<20) buf[i]=0; // clear buffer string
    gcvt(millideg / 1000.00, 20, buf); //get system temperature
    for (i=4;i++;i<20) buf[i]=0; // clear excess string
    lcdLoc(LINE4+7);
    LCDdisplayStr(buf); //display system temperature
    lcdLoc(LINE4+12);
    LCDdisplayStr(" C"); //in centigrade
    delay(1000);       // every 1 second
    
  }
  return 0;
}

// clr lcd go home loc 0x80
void ClrLcd(void)   {
  lcd_byte(0x01, LCD_CMD);
  lcd_byte(0x02, LCD_CMD);
}

// go to location on LCD
void lcdLoc(int line)   {
  lcd_byte(line, LCD_CMD);
}


// this allows use of upto 20 char of string
void LCDdisplayStr(const char *s)   {
  
  int i=0;
  while ( *s ) {
    i++;
    if (i>20) break;
    lcd_byte(*(s++), LCD_CHR);
  }  
}

void lcd_byte(int bits, int mode)   {

  //Send byte to data pins
  // bits = the data
  // mode = 1 for data, 0 for command
  int bits_high;
  int bits_low;
  // uses the two half byte writes to LCD
  bits_high = mode | (bits & 0xF0) | LCD_BACKLIGHT ;
  bits_low = mode | ((bits << 4) & 0xF0) | LCD_BACKLIGHT ;

  // High bits
  wiringPiI2CReadReg8(fd, bits_high);
  lcd_toggle_enable(bits_high);

  // Low bits
  wiringPiI2CReadReg8(fd, bits_low);
  lcd_toggle_enable(bits_low);
}

void lcd_toggle_enable(int bits)   {
  // Toggle enable pin on LCD display
  delayMicroseconds(500);
  wiringPiI2CReadReg8(fd, (bits | ENABLE));
  delayMicroseconds(500);
  wiringPiI2CReadReg8(fd, (bits & ~ENABLE));
  delayMicroseconds(500);
}


void lcd_init()   {
  // Initialise display
  lcd_byte(0x33, LCD_CMD); // Initialise
  lcd_byte(0x32, LCD_CMD); // Initialise
  lcd_byte(0x06, LCD_CMD); // Cursor move direction
  lcd_byte(0x0C, LCD_CMD); // 0x0F On, Blink Off
  lcd_byte(0x28, LCD_CMD); // Data length, number of lines, font size
  lcd_byte(0x01, LCD_CMD); // Clear display
  delayMicroseconds(500);
}
