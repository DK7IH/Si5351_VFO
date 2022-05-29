/*****************************************************************/
/*                  Si5351 clock oscillator demo code            */
/*  ************************************************************ */
/*  MUC:              ATMEL AVR ATmega 32  16MHz                 */
/*                                                               */
/*  Compiler:         GCC (GNU AVR C-Compiler)                   */
/*  Author:           Peter Baier (DK7IH)                       */
/*  Last change:      OCT 2017                                   */
/*****************************************************************/
//TWI
//PC4=SDA, PC5=SCL: I²C-Bus lines: 

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <util/twi.h>
#include <avr/pgmspace.h>

#define INTERFREQUENCY 9000000

#define OLEDADDR 0x78
#define OLEDCMD 0x00   //Command follows
#define OLEDDATA 0x40  //Data follows

#define FONTW 6
#define FONTH 8

#define S_SETLOWCOLUMN           0x00
#define S_SETHIGHCOLUMN          0x10
#define S_PAGEADDR               0xB0
#define S_SEGREMAP               0xA0

#define S_LCDWIDTH               128
#define S_LCDHEIGHT              64 

//////////////////////////////////////
//   L   C   D   
//////////////////////////////////////
int oledtype = 1; //SH1106=1; SSD1306=0

// Font 6x8 for OLED
const char font[97][6] PROGMEM={
{0x00,0x00,0x00,0x00,0x00,0x00},	// 0x20
{0x00,0x00,0x06,0x5F,0x06,0x00},	// 0x21
{0x00,0x07,0x03,0x00,0x07,0x03},	// 0x22
{0x00,0x24,0x7E,0x24,0x7E,0x24},	// 0x23
{0x00,0x24,0x2B,0x6A,0x12,0x00},	// 0x24
{0x00,0x63,0x13,0x08,0x64,0x63},	// 0x25
{0x00,0x36,0x49,0x56,0x20,0x50},	// 0x26
{0x00,0x00,0x07,0x03,0x00,0x00},	// 0x27
{0x00,0x00,0x3E,0x41,0x00,0x00},	// 0x28
{0x00,0x00,0x41,0x3E,0x00,0x00},	// 0x29
{0x00,0x08,0x3E,0x1C,0x3E,0x08},	// 0x2A
{0x00,0x08,0x08,0x3E,0x08,0x08},	// 0x2B
{0x00,0x00,0xE0,0x60,0x00,0x00},	// 0x2C
{0x00,0x08,0x08,0x08,0x08,0x08},	// 0x2D
{0x00,0x00,0x60,0x60,0x00,0x00},	// 0x2E
{0x00,0x20,0x10,0x08,0x04,0x02},	// 0x2F
{0x00,0x3E,0x51,0x49,0x45,0x3E},	// 0x30
{0x00,0x00,0x42,0x7F,0x40,0x00},	// 0x31
{0x00,0x62,0x51,0x49,0x49,0x46},	// 0x32
{0x00,0x22,0x49,0x49,0x49,0x36},	// 0x33
{0x00,0x18,0x14,0x12,0x7F,0x10},	// 0x34
{0x00,0x2F,0x49,0x49,0x49,0x31},	// 0x35
{0x00,0x3C,0x4A,0x49,0x49,0x30},	// 0x36
{0x00,0x01,0x71,0x09,0x05,0x03},	// 0x37
{0x00,0x36,0x49,0x49,0x49,0x36},	// 0x38
{0x00,0x06,0x49,0x49,0x29,0x1E},	// 0x39
{0x00,0x00,0x6C,0x6C,0x00,0x00},	// 0x3A
{0x00,0x00,0xEC,0x6C,0x00,0x00},	// 0x3B
{0x00,0x08,0x14,0x22,0x41,0x00},	// 0x3C
{0x00,0x24,0x24,0x24,0x24,0x24},	// 0x3D
{0x00,0x00,0x41,0x22,0x14,0x08},	// 0x3E
{0x00,0x02,0x01,0x59,0x09,0x06},	// 0x3F
{0x00,0x3E,0x41,0x5D,0x55,0x1E},	// 0x40
{0x00,0x7E,0x11,0x11,0x11,0x7E},	// 0x41
{0x00,0x7F,0x49,0x49,0x49,0x36},	// 0x42
{0x00,0x3E,0x41,0x41,0x41,0x22},	// 0x43
{0x00,0x7F,0x41,0x41,0x41,0x3E},	// 0x44
{0x00,0x7F,0x49,0x49,0x49,0x41},	// 0x45
{0x00,0x7F,0x09,0x09,0x09,0x01},	// 0x46
{0x00,0x3E,0x41,0x49,0x49,0x7A},	// 0x47
{0x00,0x7F,0x08,0x08,0x08,0x7F},	// 0x48
{0x00,0x00,0x41,0x7F,0x41,0x00},	// 0x49
{0x00,0x30,0x40,0x40,0x40,0x3F},	// 0x4A
{0x00,0x7F,0x08,0x14,0x22,0x41},	// 0x4B
{0x00,0x7F,0x40,0x40,0x40,0x40},	// 0x4C
{0x00,0x7F,0x02,0x04,0x02,0x7F},	// 0x4D
{0x00,0x7F,0x02,0x04,0x08,0x7F},	// 0x4E
{0x00,0x3E,0x41,0x41,0x41,0x3E},	// 0x4F
{0x00,0x7F,0x09,0x09,0x09,0x06},	// 0x50
{0x00,0x3E,0x41,0x51,0x21,0x5E},	// 0x51
{0x00,0x7F,0x09,0x09,0x19,0x66},	// 0x52
{0x00,0x26,0x49,0x49,0x49,0x32},	// 0x53
{0x00,0x01,0x01,0x7F,0x01,0x01},	// 0x54
{0x00,0x3F,0x40,0x40,0x40,0x3F},	// 0x55
{0x00,0x1F,0x20,0x40,0x20,0x1F},	// 0x56
{0x00,0x3F,0x40,0x3C,0x40,0x3F},	// 0x57
{0x00,0x63,0x14,0x08,0x14,0x63},	// 0x58
{0x00,0x07,0x08,0x70,0x08,0x07},	// 0x59
{0x00,0x71,0x49,0x45,0x43,0x00},	// 0x5A
{0x00,0x00,0x7F,0x41,0x41,0x00},	// 0x5B
{0x00,0x02,0x04,0x08,0x10,0x20},	// 0x5C
{0x00,0x00,0x41,0x41,0x7F,0x00},	// 0x5D
{0x00,0x04,0x02,0x01,0x02,0x04},	// 0x5E
{0x80,0x80,0x80,0x80,0x80,0x80},	// 0x5F
{0x00,0x00,0x03,0x07,0x00,0x00},	// 0x60
{0x00,0x20,0x54,0x54,0x54,0x78},	// 0x61
{0x00,0x7F,0x44,0x44,0x44,0x38},	// 0x62
{0x00,0x38,0x44,0x44,0x44,0x28},	// 0x63
{0x00,0x38,0x44,0x44,0x44,0x7F},	// 0x64
{0x00,0x38,0x54,0x54,0x54,0x08},	// 0x65
{0x00,0x08,0x7E,0x09,0x09,0x00},	// 0x66
{0x00,0x18,0xA4,0xA4,0xA4,0x7C},	// 0x67
{0x00,0x7F,0x04,0x04,0x78,0x00},	// 0x68
{0x00,0x00,0x00,0x7D,0x40,0x00},	// 0x69
{0x00,0x40,0x80,0x84,0x7D,0x00},	// 0x6A
{0x00,0x7F,0x10,0x28,0x44,0x00},	// 0x6B
{0x00,0x00,0x00,0x7F,0x40,0x00},	// 0x6C
{0x00,0x7C,0x04,0x18,0x04,0x78},	// 0x6D
{0x00,0x7C,0x04,0x04,0x78,0x00},	// 0x6E
{0x00,0x38,0x44,0x44,0x44,0x38},	// 0x6F
{0x00,0xFC,0x44,0x44,0x44,0x38},	// 0x70
{0x00,0x38,0x44,0x44,0x44,0xFC},	// 0x71
{0x00,0x44,0x78,0x44,0x04,0x08},	// 0x72
{0x00,0x08,0x54,0x54,0x54,0x20},	// 0x73
{0x00,0x04,0x3E,0x44,0x24,0x00},	// 0x74
{0x00,0x3C,0x40,0x20,0x7C,0x00},	// 0x75
{0x00,0x1C,0x20,0x40,0x20,0x1C},	// 0x76
{0x00,0x3C,0x60,0x30,0x60,0x3C},	// 0x77
{0x00,0x6C,0x10,0x10,0x6C,0x00},	// 0x78
{0x00,0x9C,0xA0,0x60,0x3C,0x00},	// 0x79
{0x00,0x64,0x54,0x54,0x4C,0x00},	// 0x7A
{0x00,0x08,0x3E,0x41,0x41,0x00},	// 0x7B
{0x00,0x00,0x00,0x77,0x00,0x00},	// 0x7C
{0x00,0x00,0x41,0x41,0x3E,0x08},	// 0x7D
{0x00,0x02,0x01,0x02,0x01,0x00},	// 0x7E
{0x00,0x3C,0x26,0x23,0x26,0x3C},	// 0x7F
{0x00,0x1E,0xA1,0xE1,0x21,0x12}};	// 0x80
///////////////////////////
//     DECLARATIONS
///////////////////////////
//
//OLED
void oled_command(int value);
void oled_data(unsigned int*, unsigned int);
void oled_gotoxy(unsigned int, unsigned int);
void oled_cls(int);
void oled_init(void);
void oled_byte(unsigned char);
void oled_putchar1(unsigned int x, unsigned int y, unsigned char ch, int);
void oled_putchar2(unsigned int x, unsigned int y, unsigned char ch, int);
void oled_putnumber(int, int, long, int, int, int);
void oled_putstring(int, int, char*, char, int);
void oled_write_section(int, int, int, int);

int int2asc(long, int, char*, int);
void show_frequency(long);
int calc_tuningfactor(void);

/////////////////////
//Defines for Si5351
/////////////////////
#define SI5351_ADDRESS 0xC0 // 0b11000000 for my module. Others may vary! The 0x60 did NOT work with my module!
#define PLLRATIO 36
#define CFACTOR 1048575

//Set of Si5351A register addresses
#define CLK_ENABLE_CONTROL       3
#define PLLX_SRC				15
#define CLK0_CONTROL            16 
#define CLK1_CONTROL            17
#define CLK2_CONTROL            18
#define SYNTH_PLL_A             26
#define SYNTH_PLL_B             34
#define SYNTH_MS_0              42
#define SYNTH_MS_1              50
#define SYNTH_MS_2              58
#define PLL_RESET              177
#define XTAL_LOAD_CAP          183

#define CPUCLK 16

//TWI
void twi_init(void);
void twi_start(void);
void twi_stop(void);
void twi_write(uint8_t);
uint8_t twi_get_status(void);

//SI5351 Declarations & frequency
void si5351_write(int, int);
void si5351_start(void);
void si5351_set_freq(int, unsigned long);
int main(void);

void wait_ms(int);

//Tuning
int calc_tuningfactor(void);

int tuningcount = 0;
int tuning = 0;
int laststate = 0; //Last state of rotary encoder

//Seconds counting
long runseconds10 =  0;

// Cheap & dirty delay
void wait_ms(int ms)
{
    int t1, t2;

    for(t1 = 0; t1 < ms; t1++)
    {
        for(t2 = 0; t2 < 137 * CPUCLK; t2++)
        {
            asm volatile ("nop" ::);
        }   
     }    
}

///////////////////////////
//
//         TWI
//
///////////////////////////
void twi_init(void)
{
	
    //set SCL to 400kHz
    TWSR = 0x00;
    TWBR = 0x28;
    
    //enable TWI
    TWCR = (1<<TWEN);
    
}

//Send start signal
void twi_start(void)
{
    TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
    while ((TWCR & (1<<TWINT)) == 0);
}

//send stop signal
void twi_stop(void)
{
    TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
}

void twi_write(uint8_t u8data)
{
    TWDR = u8data;
    TWCR = (1<<TWINT)|(1<<TWEN);
    while ((TWCR & (1<<TWINT)) == 0);
}

uint8_t twi_get_status(void)
{
    uint8_t status;
    //mask status
    status = TWSR & 0xF8;
    return status;
}

////////////////////////////////
//
// Si5351A commands
//
///////////////////////////////
void si5351_write(int reg_addr, int reg_value)
{
    twi_start();
    twi_write(SI5351_ADDRESS);
    twi_write(reg_addr);
    twi_write(reg_value);
    twi_stop();
} 

// Set PLLs (VCOs) to internal clock rate of 900 MHz
// Equation fVCO = fXTAL * (a+b/c) (=> AN619 p. 3
void si5351_start(void)
{
    unsigned long a, b, c;
    unsigned long p1, p2;//, p3;
    
    // Init clock chip
    si5351_write(XTAL_LOAD_CAP, 0xD2);      // Set crystal load capacitor to 10pF (default), 
                                          // for bits 5:0 see also AN619 p. 60
    si5351_write(CLK_ENABLE_CONTROL, 0x00); // Enable all outputs
    si5351_write(CLK0_CONTROL, 0x0F);       // Set PLLA to CLK0, 8 mA output
    si5351_write(CLK1_CONTROL, 0x2F);       // Set PLLB to CLK1, 8 mA output
    si5351_write(CLK2_CONTROL, 0x2F);       // Set PLLB to CLK2, 8 mA output
    si5351_write(PLL_RESET, 0xA0);          // Reset PLLA and PLLB

    // Set VCOs of PLLA and PLLB to 650 MHz
    a = PLLRATIO;     // Division factor 650/25 MHz !!!!
    b = 0;            // Numerator, sets b/c=0
    c = CFACTOR;      //Max. resolution, but irrelevant in this case (b=0)

    //Formula for splitting up the numbers to register data, see AN619
    p1 = 128 * a + (unsigned long) (128 * b / c) - 512;
    p2 = 128 * b - c * (unsigned long) (128 * b / c);
    //p3  = c;
  
    //Write data to registers PLLA and PLLB so that both VCOs are set to 900MHz intermal freq
    si5351_write(SYNTH_PLL_A, 0xFF);
    si5351_write(SYNTH_PLL_A + 1, 0xFF);
    si5351_write(SYNTH_PLL_A + 2, (p1 & 0x00030000) >> 16);
    si5351_write(SYNTH_PLL_A + 3, (p1 & 0x0000FF00) >> 8);
    si5351_write(SYNTH_PLL_A + 4, (p1 & 0x000000FF));
    si5351_write(SYNTH_PLL_A + 5, 0xF0 | ((p2 & 0x000F0000) >> 16));
    si5351_write(SYNTH_PLL_A + 6, (p2 & 0x0000FF00) >> 8);
    si5351_write(SYNTH_PLL_A + 7, (p2 & 0x000000FF));

    si5351_write(SYNTH_PLL_B, 0xFF);
    si5351_write(SYNTH_PLL_B + 1, 0xFF);
    si5351_write(SYNTH_PLL_B + 2, (p1 & 0x00030000) >> 16);
    si5351_write(SYNTH_PLL_B + 3, (p1 & 0x0000FF00) >> 8);
    si5351_write(SYNTH_PLL_B + 4, (p1 & 0x000000FF));
    si5351_write(SYNTH_PLL_B + 5, 0xF0 | ((p2 & 0x000F0000) >> 16));
    si5351_write(SYNTH_PLL_B + 6, (p2 & 0x0000FF00) >> 8);
    si5351_write(SYNTH_PLL_B + 7, (p2 & 0x000000FF));

}

void si5351_set_freq(int synth, unsigned long f)
{
    unsigned long freq = f + INTERFREQUENCY;	
    unsigned long  a, b, c = CFACTOR; 
    unsigned long f_xtal = 25000000;
    double fdiv = (double) (f_xtal * PLLRATIO) / freq; //division factor fvco/freq (will be integer part of a+b/c)
    double rm; //remainder
    unsigned long p1, p2;
  
    a = (unsigned long) fdiv;
    rm = fdiv - a;  //(equiv. to fractional part b/c)
    b = rm * c;
    p1  = 128 * a + (unsigned long) (128 * b / c) - 512;
    p2 = 128 * b - c * (unsigned long) (128 * b / c);
      
    //Write data to multisynth registers of synth n
    si5351_write(synth, 0xFF);      //1048575 MSB
    si5351_write(synth + 1, 0xFF);  //1048575 LSB
    si5351_write(synth + 2, (p1 & 0x00030000) >> 16);
    si5351_write(synth + 3, (p1 & 0x0000FF00) >> 8);
    si5351_write(synth + 4, (p1 & 0x000000FF));
    si5351_write(synth + 5, 0xF0 | ((p2 & 0x000F0000) >> 16));
    si5351_write(synth + 6, (p2 & 0x0000FF00) >> 8);
    si5351_write(synth + 7, (p2 & 0x000000FF));
}

////////////////////////////////
//
// OLED routines
//
///////////////////////////////

//Send comand to OLED
void oled_command(int value)
{
   twi_start();
   twi_write(OLEDADDR); //Device address
   twi_write(OLEDCMD);  //Command follows
   twi_write(value);    //Send value
   twi_stop();
} 

//Send a 'number' bytes of data to display - from RAM
void oled_data(unsigned int *data, unsigned int number)
{
   int t1;
   twi_start();
   twi_write(OLEDADDR); //Device address
   twi_write(OLEDDATA); //Data follows
   
   for(t1 = 0; t1 < number; t1++)
   {
      twi_write(data[t1]); //send the byte(s)
   }   
   twi_stop ();   
}

//Set "cursor" to current position to screen
void oled_gotoxy(unsigned int x, unsigned int y)
{
   int x2 = x;
   
   if(oledtype)
   {
	   x += 2; //SH1106
   }   
   twi_start();
   twi_write(OLEDADDR); //Select display  I2C address
   twi_write(OLEDCMD);  //Be ready for command
   twi_write(S_PAGEADDR + y); //Select display row
   twi_write(S_SETLOWCOLUMN + (x2 & 0x0F)); //Col addr lo byte
   twi_write(S_SETHIGHCOLUMN + ((x2 >> 4) & 0x0F)); //Col addr hi byte
   twi_stop();
}

void oled_cls(int invert)
{
    unsigned int row, col;

    //Just fill the memory with zeros
    for(row = 0; row < S_LCDHEIGHT / 8; row++)
    {
        oled_gotoxy(0, row); //Set OLED address
        twi_start();
        twi_write(OLEDADDR); //Select OLED
        twi_write(OLEDDATA); //Data follows
        for(col = 0; col < S_LCDWIDTH; col++)
        {
            if(!invert)
            {
                twi_write (0); //normal
            }   
            else
            {
                twi_write(255); //inverse
            } 
        }
        twi_stop();
    }
    oled_gotoxy(0, 0); //Return to 0, 0
}

//Write number of bitmaps to one row of screen
void oled_write_section(int x1, int x2, int row, int number)
{
    int t1;
    oled_gotoxy(x1, row);
    	
    twi_start();
    twi_write(OLEDADDR); //Device address
    twi_write(OLEDDATA); //Data follows
   
    for(t1 = x1; t1 < x2; t1++)
    {
       twi_write(number); //send the byte(s)
    }    
    twi_stop ();   
}


//Initialize OLED
void oled_init(void)
{
    oled_command(0xAE); // Display OFF
	oled_command(0x20); // Set Memory Addressing Mode
    oled_command(0x00); // HOR
    
    oled_command(0xB0);    // Set Page Start Address for Page Addressing Mode, 0-7
    oled_command(0xC8);    // Set COM Output Scan Direction
    oled_command(0x00);    // --set low column address
    oled_command(0x10);    // --set high column address
    oled_command(0x40);    // --set start line address
    oled_command(0x81);
    oled_command(0xFF);    // Set contrast control register
    oled_command(0xA1);    // Set Segment Re-map. A0=address mapped; A1=address 127 mapped.
    oled_command(0xA6);    // Set display mode. A6=Normal; A7=Inverse
    oled_command(0xA8);
    oled_command(0x3F);    // Set multiplex ratio(1 to 64)
    oled_command(0xA4);    // Output RAM to Display
					       // 0xA4=Output follows RAM content; 0xA5,Output ignores RAM content
    oled_command(0xD3);
    oled_command(0x00);    // Set display offset. 00 = no offset
    oled_command(0xD5);    // --set display clock divide ratio/oscillator frequency
    oled_command(0xF0);    // --set divide ratio
    oled_command(0xD9); 
    oled_command(0x22);    // Set pre-charge period
    oled_command(0xDA);
    oled_command(0x12);    // Set com pins hardware configuration
    oled_command(0xDB);    // --set vcomh
    oled_command(0x20);    // 0x20,0.77xVcc
    oled_command(0x8D);
    oled_command(0x14);    // Set DC-DC enabl
    oled_command(0xAF);    //Display ON
   
} 

//Write 1 byte pattern to screen using vertical orientation 
void oled_byte(unsigned char value)
{
   twi_start();
   twi_write(OLEDADDR); //Device address
   twi_write(OLEDDATA); //Data follows
   twi_write(value);
   twi_stop ();   
}

//Write character to screen (normal size);
void oled_putchar1(unsigned int x, unsigned int y, unsigned char ch, int invert)
{
	int t0;
		
	oled_gotoxy(x, y);
	for(t0 = 0; t0 < FONTW; t0++)
	{
		if(!invert)
		{
            oled_byte(pgm_read_byte(&font[ch - 32][t0]));
        }
        else    
        {
            oled_byte(~pgm_read_byte(&font[ch - 32][t0]));
        }
        
	}
}		

//Write character to screen (DOUBLE size);
void oled_putchar2(unsigned int x, unsigned int y, unsigned char ch, int invert)
{
	int t0, t1;
	char c;
	int i[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	
	for(t0 = 0; t0 < FONTW; t0++)
	{
		for(t1 = 0; t1 < 8; t1++)
		{
			if(!invert)
			{
				c = pgm_read_byte(&font[ch - 32][t0]);
			}
			else
			{
				c = ~pgm_read_byte(&font[ch - 32][t0]);
			}
				
		    if(c & (1 << t1))
		    {
			    i[t0] += (1 << (t1 * 2));
			    i[t0] += (1 << (t1 * 2 + 1));
		    }	
	    }
	}
	
	oled_gotoxy(x, y);
	for(t0 = 0; t0 < FONTW; t0++)
	{		
	    oled_byte(i[t0] & 0xFF);
	    oled_byte(i[t0] & 0xFF);
	}
	
	oled_gotoxy(x, y + 1);
	for(t0 = 0; t0 < FONTW; t0++)
	{		
	    oled_byte((i[t0] & 0xFF00) >> 8);
	    oled_byte((i[t0] & 0xFF00) >> 8);
	}
}		

//Print string in given size
//lsize=0 => normal height, lsize=1 => double height
void oled_putstring(int col, int row, char *s, char lsize, int inv)
{
    int c = col;
	
	while(*s)
	{
	    if(!lsize)
		{
	        oled_putchar1(c, row, *s++, inv);
		}
        else
        {
            oled_putchar2(c, row, *s++, inv);
		}	
		c += (lsize + 1) * FONTW;
	}
}

//Print an integer/long to OLED
void oled_putnumber(int col, int row, long num, int dec, int lsize, int inv)
{
    char *s = malloc(16);
	if(s != NULL)
	{
	    int2asc(num, dec, s, 16);
	    oled_putstring(col, row, s, lsize, inv);
	    free(s);
	}	
}


/////////////////////////////////
//
// STRING FUNCTIONS
//
////////////////////////////////
//INT 2 ASC
int int2asc(long num, int dec, char *buf, int buflen)
{
    int i, c, xp = 0, neg = 0;
    long n, dd = 1E09;

    if(!num)
	{
	    *buf++ = '0';
		*buf = 0;
		return 1;
	}	
		
    if(num < 0)
    {
     	neg = 1;
	    n = num * -1;
    }
    else
    {
	    n = num;
    }

    //Fill buffer with \0
    for(i = 0; i < 12; i++)
    {
	    *(buf + i) = 0;
    }

    c = 9; //Max. number of displayable digits
    while(dd)
    {
	    i = n / dd;
	    n = n - i * dd;
	
	    *(buf + 9 - c + xp) = i + 48;
	    dd /= 10;
	    if(c == dec && dec)
	    {
	        *(buf + 9 - c + ++xp) = '.';
	    }
	    c--;
    }

    //Search for 1st char different from '0'
    i = 0;
    while(*(buf + i) == 48)
    {
	    *(buf + i++) = 32;
    }

    //Add minus-sign if neccessary
    if(neg)
    {
	    *(buf + --i) = '-';
    }
 
    //Eleminate leading spaces
    c = 0;
    while(*(buf + i))
    {
	    *(buf + c++) = *(buf + i++);
    }
    *(buf + c) = 0;
	
	return c;
} 

/////////////////////////////////
//
// DISPLAY FUNCTIONS
//
////////////////////////////////
//Current frequency (double letter height)
void show_frequency(long f)
{
	char *buf;
	int t1 = 0;
	int row = 4, col = 5;
	
	if(f == -1)
	{
		oled_putstring(col, row, "-----.--", 1, 0);
		return;
	}
		
	buf = malloc(16);
	
	//Init buffer string
	for(t1 = 0; t1 < 16; t1++)
	{
	    *(buf + t1) = 0;
	}
	
	int2asc(f, 3, buf, 16);
	
	//Display buffer
	for(t1 = 0; *(buf + t1); t1++)
	{
		oled_putchar2(col + t1 * 12, row, *(buf + t1), 0);   
	}	
		
	free(buf);
}

///////////////////////////
 //       ISR Timer       // 
///////////////////////////
ISR(TIMER1_COMPA_vect)
{
    runseconds10++; 
    tuningcount = 0;
}

/////////////////////
//Rotary encoder
////////////////////
ISR(PCINT0_vect)
{ 
	int gray = (PINB & 0xC0) >> 6;         //Read PB6 and PB7
    	
    int state = (gray >> 1) ^ gray;        //Convert from Gray code to binary

    if (state != laststate)                //Compare states
    {        
        tuning += ((laststate - state) & 0x03) - 2; // Results in -1 or +1
        laststate = state;
        tuningcount += 2;
    } 
	PCIFR |=  (1 << PCIF0);               //Clear pin change interrupt flag.
}

//Calc increment/decrement rate from tuning speed
int calc_tuningfactor(void)
{
	return (tuningcount); 
}	  

int main(void) 
{
			
    unsigned long f_vfo = 14212789;
    
    PORTB = (1 << PB6)| (1 << PB7); //Pullup Rs for rotary encoder
    
    //Set Timer 1 as counter for 10th seconds
    TCCR1A = 0;             // normal mode, no PWM
    TCCR1B = (1 << CS10) | (1 << CS12) | (1<<WGM12);   // Prescaler = 1/1024 based on system clock 16 MHz
                                                       // 15625 incs/sec
                                                       // and enable reset of counter register
	OCR1AH = (781 >> 8);                             //Load compare values to registers
    OCR1AL = (781 & 0x00FF);
	TIMSK1 |= (1<<OCIE1A);
	
	//Interrupt definitions for rotary encoder PB6 and PB7 (PCINT6 and PCINT7)
	PCMSK0 |= (1<<PCINT6)|(1<<PCINT7);    //enable encoder pins as interrupt source
	PCICR |= (1<<PCIE0);                      // enable pin change interupts 
			
    //TWI	
	wait_ms(100);
	twi_init();
	wait_ms(100);
	
	//OLED
	oled_init();
	wait_ms(20);
	oled_cls(0);	
	
	si5351_start();
	si5351_set_freq(SYNTH_MS_0, f_vfo);
	si5351_set_freq(SYNTH_MS_1, INTERFREQUENCY + 1500);
	si5351_set_freq(SYNTH_MS_2, INTERFREQUENCY - 1500);
	
	//First display
	oled_putstring(0, 0, "Si5351 VFO V1.2", 0, 0);
	oled_putstring(0, 1, "by Peter (DK7IH)", 0, 0);
	show_frequency(f_vfo);
	
	sei();
	
	for(;;)
    {
		//TUNING		
		if(tuning > 1)  
		{    
		    f_vfo += calc_tuningfactor();  
		    si5351_set_freq(SYNTH_MS_0, f_vfo);
		    show_frequency(f_vfo);
			tuning = 0;
		}
		
		if(tuning < -1)
		{
		    f_vfo -= calc_tuningfactor();  
		    si5351_set_freq(SYNTH_MS_0, f_vfo);
		    show_frequency(f_vfo);
			tuning = 0;
		}
		
    }
	
	return 0;
}

