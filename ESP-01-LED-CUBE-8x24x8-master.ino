/*
Project : ESP-01 ESP8266 CONTROL LED CUBE 8x24x8 (8x8x8 RED & 8x8x8 GREEN & 8x8x8 BLUE)
Version : 0.0
Date    : 05.July.2020
Author  : tuenhidiy
Email   : tuenhi.n2012@gmail.com
*/
#include <ESP8266WiFi.h>
#include <avr/pgmspace.h>

#define DATA_Pin        1 // ESP-01 ESP8266 - GPIO1 - TX
#define CLOCK_Pin       2 // ESP-01 ESP8266 - GPIO2
#define LATCH_Pin       0 // ESP-01 ESP8266 - GPIO0
#define BLANK_Pin       3 // ESP-01 ESP8266 - GPIO3 - RX

#define ARRAY_SIZE(a)   (sizeof(a)/sizeof(a[0]))
#define CUBE_XSIZE      8 //  Cube size - X axis
#define CUBE_YSIZE      24 // Cube size - Y axis
#define CUBE_ZSIZE      8 //  Cube size - Z axis


#define AXIS_X          0
#define AXIS_Y          1
#define AXIS_Z          2

#define PATH            0
#define ELLIPSE         1

byte anodelevel;
byte cube[CUBE_ZSIZE][CUBE_YSIZE];  // cube[8][24]
byte fb[CUBE_ZSIZE][CUBE_YSIZE];

// byte anode[8]= {B00000001, B00000010, B00000100, B00001000, B00010000, B00100000, B01000000, B10000000}; // MSBFIRST
byte anode[8]= {B10000000, B01000000, B00100000, B00010000,B00001000, B00000100, B00000010, B00000001};     // LSBFIRST                 

typedef uint8_t     type_polarity;

enum polarity_t : uint8_t
  {
    POSITIVE,
    NEGATIVE,
  };

uint32_t timeStart;
unsigned long start;
#define DEMO_RUNTIME  20000

const unsigned char font[760] = {
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x60,0xfa,0xfa,0x60,0x00,0x00,0x00,
0x00,0xe0,0xe0,0x00,0xe0,0xe0,0x00,0x00,
0x28,0xfe,0xfe,0x28,0xfe,0xfe,0x28,0x00,
0x24,0x74,0xd6,0xd6,0x5c,0x48,0x00,0x00,
0x62,0x66,0x0c,0x18,0x30,0x66,0x46,0x00,
0x0c,0x5e,0xf2,0xba,0xec,0x5e,0x12,0x00,
0x20,0xe0,0xc0,0x00,0x00,0x00,0x00,0x00,
0x00,0x38,0x7c,0xc6,0x82,0x00,0x00,0x00,
0x00,0x82,0xc6,0x7c,0x38,0x00,0x00,0x00,
0x10,0x54,0x7c,0x38,0x38,0x7c,0x54,0x10,
0x10,0x10,0x7c,0x7c,0x10,0x10,0x00,0x00,
0x00,0x05,0x07,0x06,0x00,0x00,0x00,0x00,
0x10,0x10,0x10,0x10,0x10,0x10,0x00,0x00,
0x00,0x00,0x06,0x06,0x00,0x00,0x00,0x00,
0x06,0x0c,0x18,0x30,0x60,0xc0,0x80,0x00,
0x7c,0xfe,0x9a,0xb2,0xfe,0x7c,0x00,0x00,
0x42,0x42,0xfe,0xfe,0x02,0x02,0x00,0x00,
0x46,0xce,0x9a,0x92,0xf6,0x66,0x00,0x00,
0x44,0xc6,0x92,0x92,0xfe,0x6c,0x00,0x00,
0x18,0x38,0x68,0xc8,0xfe,0xfe,0x08,0x00,
0xe4,0xe6,0xa2,0xa2,0xbe,0x9c,0x00,0x00,
0x3c,0x7e,0xd2,0x92,0x9e,0x0c,0x00,0x00,
0xc0,0xc6,0x8e,0x98,0xf0,0xe0,0x00,0x00,
0x6c,0xfe,0x92,0x92,0xfe,0x6c,0x00,0x00,
0x60,0xf2,0x92,0x96,0xfc,0x78,0x00,0x00,
0x00,0x00,0x36,0x36,0x00,0x00,0x00,0x00,
0x00,0x05,0x37,0x36,0x00,0x00,0x00,0x00,
0x10,0x38,0x6c,0xc6,0x82,0x00,0x00,0x00,
0x28,0x28,0x28,0x28,0x28,0x28,0x00,0x00,
0x00,0x82,0xc6,0x6c,0x38,0x10,0x00,0x00,
0x40,0xc0,0x8a,0x9a,0xf0,0x60,0x00,0x00,
0x7c,0xfe,0x82,0xba,0xba,0xf8,0x78,0x00,
0x3e,0x7e,0xc8,0xc8,0x7e,0x3e,0x00,0x00,
0x82,0xfe,0xfe,0x92,0x92,0xfe,0x6c,0x00,
0x38,0x7c,0xc6,0x82,0x82,0xc6,0x44,0x00,
0x82,0xfe,0xfe,0x82,0xc6,0xfe,0x38,0x00,
0x82,0xfe,0xfe,0x92,0xba,0x82,0xc6,0x00,
0x82,0xfe,0xfe,0x92,0xb8,0x80,0xc0,0x00,
0x38,0x7c,0xc6,0x82,0x8a,0xce,0x4e,0x00,
0xfe,0xfe,0x10,0x10,0xfe,0xfe,0x00,0x00,
0x00,0x82,0xfe,0xfe,0x82,0x00,0x00,0x00,
0x0c,0x0e,0x02,0x82,0xfe,0xfc,0x80,0x00,
0x82,0xfe,0xfe,0x10,0x38,0xee,0xc6,0x00,
0x82,0xfe,0xfe,0x82,0x02,0x06,0x0e,0x00,
0xfe,0xfe,0x60,0x30,0x60,0xfe,0xfe,0x00,
0xfe,0xfe,0x60,0x30,0x18,0xfe,0xfe,0x00,
0x38,0x7c,0xc6,0x82,0xc6,0x7c,0x38,0x00,
0x82,0xfe,0xfe,0x92,0x90,0xf0,0x60,0x00,
0x78,0xfc,0x84,0x8e,0xfe,0x7a,0x00,0x00,
0x82,0xfe,0xfe,0x98,0x9c,0xf6,0x62,0x00,
0x64,0xe6,0xb2,0x9a,0xde,0x4c,0x00,0x00,
0xc0,0x82,0xfe,0xfe,0x82,0xc0,0x00,0x00,
0xfe,0xfe,0x02,0x02,0xfe,0xfe,0x00,0x00,
0xf8,0xfc,0x06,0x06,0xfc,0xf8,0x00,0x00,
0xfe,0xfe,0x0c,0x18,0x0c,0xfe,0xfe,0x00,
0xc6,0xee,0x38,0x10,0x38,0xee,0xc6,0x00,
0xe0,0xf2,0x1e,0x1e,0xf2,0xe0,0x00,0x00,
0xe6,0xce,0x9a,0xb2,0xe2,0xc6,0x8e,0x00,
0x00,0xfe,0xfe,0x82,0x82,0x00,0x00,0x00,
0x80,0xc0,0x60,0x30,0x18,0x0c,0x06,0x00,
0x00,0x82,0x82,0xfe,0xfe,0x00,0x00,0x00,
0x10,0x30,0x60,0xc0,0x60,0x30,0x10,0x00,
0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
0x00,0x00,0xc0,0xe0,0x20,0x00,0x00,0x00,
0x04,0x2e,0x2a,0x2a,0x3c,0x1e,0x02,0x00,
0x82,0xfc,0xfe,0x22,0x22,0x3e,0x1c,0x00,
0x1c,0x3e,0x22,0x22,0x36,0x14,0x00,0x00,
0x0c,0x1e,0x12,0x92,0xfc,0xfe,0x02,0x00,
0x1c,0x3e,0x2a,0x2a,0x3a,0x18,0x00,0x00,
0x12,0x7e,0xfe,0x92,0xc0,0x40,0x00,0x00,
0x19,0x3d,0x25,0x25,0x1f,0x3e,0x20,0x00,
0x82,0xfe,0xfe,0x10,0x20,0x3e,0x1e,0x00,
0x00,0x22,0xbe,0xbe,0x02,0x00,0x00,0x00,
0x02,0x23,0x21,0xbf,0xbe,0x00,0x00,0x00,
0x82,0xfe,0xfe,0x08,0x1c,0x36,0x22,0x00,
0x00,0x82,0xfe,0xfe,0x02,0x00,0x00,0x00,
0x3e,0x3e,0x30,0x18,0x30,0x3e,0x1e,0x00,
0x3e,0x3e,0x20,0x20,0x3e,0x1e,0x00,0x00,
0x1c,0x3e,0x22,0x22,0x3e,0x1c,0x00,0x00,
0x21,0x3f,0x1f,0x25,0x24,0x3c,0x18,0x00,
0x18,0x3c,0x24,0x25,0x1f,0x3f,0x21,0x00,
0x22,0x3e,0x1e,0x22,0x38,0x18,0x00,0x00,
0x12,0x3a,0x2a,0x2a,0x2e,0x24,0x00,0x00,
0x00,0x20,0x7c,0xfe,0x22,0x24,0x00,0x00,
0x3c,0x3e,0x02,0x02,0x3c,0x3e,0x02,0x00,
0x38,0x3c,0x06,0x06,0x3c,0x38,0x00,0x00,
0x3c,0x3e,0x06,0x0c,0x06,0x3e,0x3c,0x00,
0x22,0x36,0x1c,0x08,0x1c,0x36,0x22,0x00,
0x39,0x3d,0x05,0x05,0x3f,0x3e,0x00,0x00,
0x32,0x26,0x2e,0x3a,0x32,0x26,0x00,0x00,
0x10,0x10,0x7c,0xee,0x82,0x82,0x00,0x00,
0x00,0x00,0x00,0xee,0xee,0x00,0x00,0x00,
0x82,0x82,0xee,0x7c,0x10,0x10,0x00,0x00,
0x40,0xc0,0x80,0xc0,0x40,0xc0,0x80,0x00,
 };

const unsigned char paths[60] PROGMEM =
{
0xF7,0xD7,0xB7,0x97,0x77,0x57,0x37,0x17,0x16,0x15,0x14,0x13,0x12,0x11,0x10,0x0F,0x0E,0x0D,0x0C,0x0B,0x0A,0x09,0x08,0x07,0x06,0x05,0x04,0x03,0x02,0x01,
0x00,0x20,0x40,0x60,0x80,0xA0,0xC0,0xE0,0xE1,0xE2,0xE3,0xE4,0xE5,0xE6,0xE7,0xE8,0xE9,0xEA,0xEB,0xEC,0xED,0xEE,0xEF,0xF0,0xF1,0xF2,0xF3,0xF4,0xF5,0xF6   
};

const unsigned char ellipse[38] PROGMEM =
{
0xF1,0xF2,0xF3,0xF4,0xD5,0xB6,0x97,0x77,0x56,0x35,0x14,0x13,0x12,0x11,0x10,0x0F,0x0E,0x0D,0x0C,
0x0B,0x0A,0x09,0x08,0x07,0x06,0x05,0x04,0x03,0x22,0x41,0x60,0x80,0xA1,0xC2,0xE3,0xE4,0xE5,0xE6
};

void ICACHE_RAM_ATTR timer1_ISR(void);  
void DIY_shiftOut(uint8_t DATA);

void setvoxel(int x, int y, int z);
void clrvoxel(int x, int y, int z);
void tmpsetvoxel(int x, int y, int z);
void tmpclrvoxel(int x, int y, int z);

unsigned char inrange(int x, int y, int z);
unsigned char getvoxel(int x, int y, int z);
void flpvoxel(int x, int y, int z);
void altervoxel(int x, int y, int z, int state);
    
void setplane_z(int z);
void clrplane_z(int z);
void setplane_x(int x);
void clrplane_x(int x);
void setplane_y(int y);
void clrplane_y(int y);

void setplane (byte axis, unsigned char i);
void clrplane (byte axis, unsigned char i);

void setline_z(int x, int y, int z1, int z2);
void setline_x(int z, int y, int x1, int x2);
void setline_y(int z, int x, int y1, int y2);
void clrline_z(int x, int y, int z1, int z2);
void clrline_x(int z, int y, int x1, int x2);
void clrline_y(int z, int x, int y1, int y2);
void fill(unsigned char pattern);
void clearfast();
void tmpfill(unsigned char pattern);
char flipbyte(char byte);
char byteline(int start, int end);
void argorder(int ix1, int ix2, int *ox1, int *ox2);  

void drawCircle(int x0, int y0, int z0, int radius);
void shift (byte axis, int direction);
    
void bomb(int delayx, int iterations);
void line_3d (int x1, int y1, int z1, int x2, int y2, int z2);
void clear_line_3d(int x1, int y1, int z1, int x2, int y2, int z2);

float distance3d(float x1, float y1, float z1, float x2, float y2, float z2);

void fireworks(int iterations, int n, int delayx);
void effect_random_filler(int delayx, int state);
  
void copyPlane(byte axis, uint8_t cordFrom, uint8_t cordTo);
void flagwave();
void TranslateScroll8(byte axis, int8_t value);


void EffectScrollingSine();
void Bresenham3D(int8_t x1, int8_t y1, int8_t z1, const int8_t x2, const int8_t y2, const int8_t z2, const byte modex);
void CalcLine3D(int8_t x1, int8_t y1, int8_t z1, const int8_t x2, const int8_t y2, const int8_t z2, byte modex);
void DrawLine3D(int8_t x0, int8_t y0, int8_t z0, const int8_t x1, const int8_t y1, const int8_t z1);
void EraseLine3D(int8_t x0, int8_t y0, int8_t z0, const int8_t x1, const int8_t y1, const int8_t z1);

void CornerToCorner();
void sinwaveTwo();
void wipe_out();
    
int checkConstrains(int value, int min, int max);
void TransitionShift(byte axis, const type_polarity polarity, const uint8_t delayms);
void TransitionScroll(byte axis, const type_polarity polarity, const uint8_t delayx);

void effect_pathmove(unsigned char *path, int length);
void font_getpath (unsigned char path, unsigned char *destination);
    
void font_getchar (char chr, unsigned char dst[8]);
void effect_allshape_text(int delayx, char *str, int modex);
void effect_outer_wall(int iterations, int delayx);
    
unsigned char bitswap(unsigned char x);
void drawRPrism(uint8_t x, uint8_t y, uint8_t z, int8_t dx, int8_t dy, int8_t dz);
void clrRPrism(uint8_t x, uint8_t y, uint8_t z, int8_t dx, int8_t dy, int8_t dz);

void drawCube(uint8_t x, uint8_t y, uint8_t z, int8_t size);
void effect_rain(int iterations);
void effect_wormsqueeze(int size, int axis, int direction, int iterations, int delayx);

void crazy_straw_mode(int delayx, int iterations);

void draw_positions_axis(byte axis, unsigned char positions[128], int invert);
void effect_boxside_randsend_parallel(byte axis, int origin, int delayx, int modex);
void effect_axis_updown_randsuspend(byte axis, int delayx, int sleep, int invert);

void oscillation(int interactions);
void boingboing(uint16_t iterations, int delayx, unsigned char modex, unsigned char drawmode);
void squarespiral(int iterations, int Delay);
void brownian(int interactions);
void folder();

void setup () 
{
  anodelevel = 0;
   
  pinMode(DATA_Pin, OUTPUT);
  pinMode(CLOCK_Pin, OUTPUT);
  pinMode(LATCH_Pin, OUTPUT);
  //pinMode(BLANK_Pin, OUTPUT);
  
  timer1_isr_init();
  timer1_attachInterrupt(timer1_ISR);
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);
  timer1_write(1000);
  clearfast();
}
   

void loop()
{
  clearfast();
  CornerToCorner();
  
  clearfast();
  squarespiral(200, 80);
   
  clearfast();
  effect_outer_wall(2, 25);
  
  clearfast();
  effect_rand_patharound(1000,25);
  
  clearfast();
  effect_allshape_text(75,"HAPPY NEW YEAR 2020", 0);
  clearfast();
  effect_allshape_text(75,"TUENHI DIY CHANNEL", 1);
  
  clearfast();
  effect_axis_updown_randsuspend(AXIS_Y, 100, 500, 0);
  effect_axis_updown_randsuspend(AXIS_Y, 100, 500, 1);
  effect_axis_updown_randsuspend(AXIS_Y, 100, 500, 0);
  effect_axis_updown_randsuspend(AXIS_Y, 100, 500, 1);
  
  clearfast();
  effect_axis_updown_randsuspend(AXIS_X, 100, 500, 0);
  effect_axis_updown_randsuspend(AXIS_X, 100, 500, 1);
  effect_axis_updown_randsuspend(AXIS_X, 100, 500, 0);
  effect_axis_updown_randsuspend(AXIS_X, 100, 500, 1);
  
  clearfast();
  crazy_straw_mode(15, 75);
  
  clearfast();
  fireworks(10,50,50);
  
  clearfast();
  flagwave();
  
  clearfast();
  shrinkCube_random(15);
  
  clearfast();
  bomb(75, 6);
  
  clearfast();
  effect_boxside_randsend_parallel(AXIS_Y, 0 , 75, 1);
  delay(1500);
  effect_boxside_randsend_parallel(AXIS_Y, 1 , 75, 1);
  clearfast();
  
  effect_boxside_randsend_parallel(AXIS_X, 0 , 75, 1);
  delay(1500);
  effect_boxside_randsend_parallel(AXIS_X, 1 , 75, 1);
  
  clearfast();
  oscillation(640);
  
  clearfast();
  effect_wormsqueeze(2, AXIS_Z, -1, 300, 75);
  
  clearfast();
  effect_rain (300);
  
  clearfast();
  EffectScrollingSine();
  
  
  clearfast();
  folder();
  
  clearfast();
  wipe_out();
  
  clearfast();
  sinwaveTwo();
  
  clearfast();
  boingboing(500, 40, 0x01, 0x02);
  boingboing(300, 40, 0x01, 0x01);
  boingboing(300, 40, 0x02, 0x03);
  
  clearfast();
  brownian(300);
  
  clearfast();
}

void DIY_shiftOut(uint8_t DATA)
{
    for (uint8_t i = 0; i<8; i++)  
    {
      digitalWrite(DATA_Pin, !!(DATA & (1 << (i))));
      digitalWrite(CLOCK_Pin,HIGH);
      digitalWrite(CLOCK_Pin,LOW);                
    }
}


void ICACHE_RAM_ATTR timer1_ISR(void)
{
    digitalWrite(BLANK_Pin, HIGH);
    
    // Cathodes scanning
    for (byte i = 0; i<24; i++)
    {
      DIY_shiftOut((cube[anodelevel][i]));
    }
    // Layer - Anodes scanning
    DIY_shiftOut(anode[anodelevel]);
    
    delayMicroseconds(5);
    digitalWrite(LATCH_Pin, HIGH);
    delayMicroseconds(5);    
    digitalWrite(LATCH_Pin, LOW);
    delayMicroseconds(5);    

    digitalWrite(BLANK_Pin, LOW);
    delayMicroseconds(5);
    anodelevel++;   
    if (anodelevel==8) anodelevel=0;
    pinMode(BLANK_Pin, OUTPUT);        
    timer1_write(1000);      
}

unsigned char inrange(int x, int y, int z)
{
  if (x >= 0 && x < CUBE_XSIZE && y >= 0 && y < CUBE_YSIZE && z >= 0 && z < CUBE_ZSIZE)
  {
    return 1;
  } else
  {
    return 0;
  }
}
// Set a single voxel to ON
void setvoxel(int x, int y, int z)
{
  if (inrange(x,y,z))
    cube[z][y] |= (1 << x);
}

// Set a single voxel to OFF
void clrvoxel(int x, int y, int z)
{
  if (inrange(x,y,z))  
    cube[z][y] &= ~(1 << x);
}

void fill(unsigned char pattern)
{
  int z;
  int y;
  for (z=0; z<CUBE_ZSIZE; z++)
  {
    for (y=0;y<CUBE_YSIZE; y++)
    {      
      cube[z][y] = pattern;
    }
  }
}

void clearfast()
{
  memset(cube, 0, sizeof(cube[0][0]) * 8 * 24);
}

void tmpsetvoxel(int x, int y, int z)
{
  if (inrange(x,y,z))
  fb[z][y] |= (1 << x);
}

void tmpclrvoxel(int x, int y, int z)
{
  if (inrange(x,y,z))
  fb[z][y] &= ~(1 << x);
}

unsigned char getvoxel(int x, int y, int z)
{
  if (inrange(x,y,z))
  {
    if (cube[z][y] & (1 << x))
    {
      return 1;
    } else
    {
      return 0;
    }
  } else
  {
    return 0;
  }
}

void altervoxel(int x, int y, int z, int state)
{
  if (state == 1)
  {
    setvoxel(x,y,z);
  } else
  {
    clrvoxel(x,y,z);
  }
}

void flpvoxel(int x, int y, int z)
{
  if (inrange(x, y, z))   
    cube[z][y] ^= (1 << x);
}

void setplane_z (int z)
{
  int i;
  if (z>=0 && z<CUBE_ZSIZE)
  {
    for (i=0;i<24;i++)
    {
    cube[z][i] = 0xff;
    }    
  }
  
}

void clrplane_z (int z)
{
  int i;
  if (z>=0 && z<CUBE_ZSIZE)
  {
    for (i=0;i<24;i++)
    {
      cube[z][i] = 0x00;
    }
  }

}

void setplane_x (int x)
{
  int z;
  int y;
  if (x>=0 && x<CUBE_XSIZE)
  {
    for (z=0;z<CUBE_ZSIZE;z++)
    {
      for (y=0;y<CUBE_YSIZE;y++)
      {
        cube[z][y] |= (1 << x);
      }
    }
  }
}

void clrplane_x (int x)
{
  int z;
  int y;
  if (x>=0 && x<CUBE_XSIZE)
  {
    for (z=0;z<CUBE_ZSIZE;z++)
    {
      for (y=0;y<CUBE_YSIZE;y++)
      { 
        cube[z][y] &= ~(1 << x);
      }
    }
  }
}

void setplane_y (int y)
{
  int z;
  if (y>=0 && y<CUBE_YSIZE)
  {
    for (z=0;z<CUBE_ZSIZE;z++)
    {      
      cube[z][y] = 0xff;
    }
  } 
}

void clrplane_y (int y)
{
  int z;
  if (y>=0 && y<CUBE_YSIZE)
  {
    for (z=0;z<CUBE_ZSIZE;z++)
    {    
      cube[z][y] = 0x00; 
    }
  }
}

void setplane (byte axis, unsigned char i)
{
    switch (axis)
    {
        case AXIS_X:
            setplane_x(i);
            break;
        
       case AXIS_Y:
            setplane_y(i);
            break;

       case AXIS_Z:
            setplane_z(i);
            break;
    }
}

void clrplane (byte axis, unsigned char i)
{
    switch (axis)
    {
        case AXIS_X:
            clrplane_x(i);
            break;
        
       case AXIS_Y:
            clrplane_y(i);
            break;

       case AXIS_Z:
            clrplane_z(i);
            break;
    }
}

void shift (byte axis, int direction)
{
  int i, x ,y;
  int ii, iii;
  int state;

  for (i = 0; i < CUBE_ZSIZE; i++)
  {
    if (direction == -1)
    {
      ii = i;
    } else
    {
      ii = (7-i);
    } 
  
  
    for (x = 0; x < CUBE_XSIZE; x++)
    {
      for (y = 0; y < CUBE_YSIZE; y++)
      {
        if (direction == -1)
        {
          iii = ii+1;
        } else
        {
          iii = ii-1;
        }
        
        if (axis == AXIS_Z)
        {
          state = getvoxel(x,y,iii);
          altervoxel(x,y,ii,state);
        }
        
        if (axis == AXIS_Y)
        {
          state = getvoxel(x,iii,y);
          altervoxel(x,ii,y,state);
        }
        
        if (axis == AXIS_X)
        {
          state = getvoxel(iii,y,x);
          altervoxel(ii,y,x,state);
        }
      }
    }
  }
  
  if (direction == -1)
  {
    i = 7;
  } else
  {
    i = 0;
  } 
  
  for (x = 0; x < CUBE_XSIZE; x++)
  {
    for (y = 0; y < CUBE_YSIZE; y++)
    {
      if (axis == AXIS_Z)
        clrvoxel(x,y,i);
        
      if (axis == AXIS_Y)
        clrvoxel(x,i,y);
      
      if (axis == AXIS_X)
        clrvoxel(i,y,x);
    }
  }
}

void argorder(int ix1, int ix2, int *ox1, int *ox2)
{
  if (ix1>ix2)
  {
    int tmp;
    tmp = ix1;
    ix1= ix2;
    ix2 = tmp;
  }
  *ox1 = ix1;
  *ox2 = ix2;
}

char byteline (int start, int end)
{
  return ((0xff<<start) & ~(0xff<<(end+1)));
}

char flipbyte (char byte)
{
  char flop = 0x00;

  flop = (flop & 0b11111110) | (0b00000001 & (byte >> 7));
  flop = (flop & 0b11111101) | (0b00000010 & (byte >> 5));
  flop = (flop & 0b11111011) | (0b00000100 & (byte >> 3));
  flop = (flop & 0b11110111) | (0b00001000 & (byte >> 1));
  flop = (flop & 0b11101111) | (0b00010000 & (byte << 1));
  flop = (flop & 0b11011111) | (0b00100000 & (byte << 3));
  flop = (flop & 0b10111111) | (0b01000000 & (byte << 5));
  flop = (flop & 0b01111111) | (0b10000000 & (byte << 7));
  return flop;
}
void tmpfill (unsigned char pattern)
{
  int z;
  int y;
  for (z=0;z<CUBE_ZSIZE;z++)
  {
    for (y=0;y<CUBE_YSIZE;y++)
    {
      fb[z][y] = pattern;
    }
  }
}


// Set or clear exactly 1536 voxels in a random order.
void effect_random_filler (int delayx, int state)
{
  int x,y,z;
  int loop = 0;
  
  
  if (state == 1)
  {
    clearfast();
  } else
  {
    fill(0xff);
  }
  
  while (loop<1535)
  {
    x = rand()%8;
    y = rand()%24;
    z = rand()%8;

    if ((state == 0 && getvoxel(x,y,z) == 0x01) || (state == 1 && getvoxel(x,y,z) == 0x00))
    {
      altervoxel(x,y,z,state);
      delay(delayx);
      loop++;
    } 
  }
}

void bomb(int delayx, int iterations)
{
  int i;
  uint8_t j,x,y,r;
  
  for(i=0;i<iterations;i++)
  {
    x = rand()%8;
    y = rand()%24;
    setvoxel(x,y,CUBE_ZSIZE-1);
    for(j=0;j<CUBE_ZSIZE;j++)
    {
      delay(delayx);
      shift(AXIS_Z,-1);
    }
    for(r=1;r<=CUBE_YSIZE;r++)
    {
      drawCircle(x,y,0,r);
      delay(delayx);
      fill(0x00);
    }
    delay(800);
  }
}


float distance3d(float x1, float y1, float z1, float x2, float y2, float z2)
{  
  float dist;
  dist = sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2));

  return dist;
}
///////////////////
void line_3d(int x1, int y1, int z1, int x2, int y2, int z2)
{
  int i, dx, dy, dz, l, m, n, x_inc, y_inc, z_inc,
  err_1, err_2, dx2, dy2, dz2;
  int pixel[3];
  pixel[0] = x1;
  pixel[1] = y1;
  pixel[2] = z1;
  dx = x2 - x1;
  dy = y2 - y1;
  dz = z2 - z1;
  x_inc = (dx < 0) ? -1 : 1;
  l = abs(dx);
  y_inc = (dy < 0) ? -1 : 1;
  m = abs(dy);
  z_inc = (dz < 0) ? -1 : 1;
  n = abs(dz);
  dx2 = l << 1;
  dy2 = m << 1;
  dz2 = n << 1;
  if ((l >= m) && (l >= n))
  {
    err_1 = dy2 - l;
    err_2 = dz2 - l;
    for (i = 0; i < l; i++)
    {
      setvoxel(pixel[0],pixel[1],pixel[2]);
      if (err_1 > 0)
      {
        pixel[1] += y_inc;
        err_1 -= dx2;
      }
      if (err_2 > 0)
      {
        pixel[2] += z_inc;
        err_2 -= dx2;
      }
      err_1 += dy2;
      err_2 += dz2;
      pixel[0] += x_inc;
    }
  } 
  else if ((m >= l) && (m >= n))
  {
    err_1 = dx2 - m;
    err_2 = dz2 - m;
    for (i = 0; i < m; i++) 
    {
      setvoxel(pixel[0],pixel[1],pixel[2]);
      if (err_1 > 0)
      {
        pixel[0] += x_inc;
        err_1 -= dy2;
      }
      if (err_2 > 0)
      {
        pixel[2] += z_inc;
        err_2 -= dy2;
      }
      err_1 += dx2;
      err_2 += dz2;
      pixel[1] += y_inc;
    }
  }
  else
  {
    err_1 = dy2 - n;
    err_2 = dx2 - n;
    for (i = 0; i < n; i++)
    {
      setvoxel(pixel[0],pixel[1],pixel[2]);
      if (err_1 > 0)
    {
      pixel[1] += y_inc;
      err_1 -= dz2;
    }
    if (err_2 > 0)
    {
      pixel[0] += x_inc;
      err_2 -= dz2;
    }
    err_1 += dy2;
    err_2 += dx2;
    pixel[2] += z_inc;
    }
  }
  setvoxel(pixel[0],pixel[1],pixel[2]);
}

void clear_line_3d(int x1, int y1, int z1, int x2, int y2, int z2)
{
  int i, dx, dy, dz, l, m, n, x_inc, y_inc, z_inc,
  err_1, err_2, dx2, dy2, dz2;
  int pixel[3];
  pixel[0] = x1;
  pixel[1] = y1;
  pixel[2] = z1;
  dx = x2 - x1;
  dy = y2 - y1;
  dz = z2 - z1;
  x_inc = (dx < 0) ? -1 : 1;
  l = abs(dx);
  y_inc = (dy < 0) ? -1 : 1;
  m = abs(dy);
  z_inc = (dz < 0) ? -1 : 1;
  n = abs(dz);
  dx2 = l << 1;
  dy2 = m << 1;
  dz2 = n << 1;
  if ((l >= m) && (l >= n)) {
  err_1 = dy2 - l;
  err_2 = dz2 - l;
  for (i = 0; i < l; i++) {
  clrvoxel(pixel[0],pixel[1],pixel[2]);
  if (err_1 > 0) {
  pixel[1] += y_inc;
  err_1 -= dx2;
  }
  if (err_2 > 0) {
  pixel[2] += z_inc;
  err_2 -= dx2;
  }
  err_1 += dy2;
  err_2 += dz2;
  pixel[0] += x_inc;
  }
  } else if ((m >= l) && (m >= n)) {
  err_1 = dx2 - m;
  err_2 = dz2 - m;
  for (i = 0; i < m; i++) {
  clrvoxel(pixel[0],pixel[1],pixel[2]);
  if (err_1 > 0) {
  pixel[0] += x_inc;
  err_1 -= dy2;
  }
  if (err_2 > 0) {
  pixel[2] += z_inc;
  err_2 -= dy2;
  }
  err_1 += dx2;
  err_2 += dz2;
  pixel[1] += y_inc;
  }
  } else {
  err_1 = dy2 - n;
  err_2 = dx2 - n;
  for (i = 0; i < n; i++) {
  clrvoxel(pixel[0],pixel[1],pixel[2]);
  if (err_1 > 0) {
  pixel[1] += y_inc;
  err_1 -= dz2;
  }
  if (err_2 > 0) {
  pixel[0] += x_inc;
  err_2 -= dz2;
  }
  err_1 += dy2;
  err_2 += dx2;
  pixel[2] += z_inc;
  }
  }
  clrvoxel(pixel[0],pixel[1],pixel[2]);
}

void drawCircle(int x0, int y0, int z0, int radius)
{
  int r = radius, t = 0;
  int radiusError = 1-r;
 
  while(r >= t)
  {
    setvoxel(r + x0, t + y0,z0);
    setvoxel(t + x0, r + y0,z0);
    setvoxel(-r + x0, t + y0,z0);
    setvoxel(-t + x0, r + y0,z0);
    setvoxel(-r + x0, -t + y0,z0);
    setvoxel(-t + x0, -r + y0,z0);
    setvoxel(r + x0, -t + y0,z0);
    setvoxel(t + x0, -r + y0,z0);
    t++;
    if (radiusError<0)
    {
      radiusError += 2 * t + 1;
    }
    else{
      r--;
      radiusError += 2 * (t - r + 1);
    }
  }
}

void copyPlane(byte axis, uint8_t cordFrom, uint8_t cordTo)
// copy the plane from level cordFrom to cordTo
{
  clrplane(axis, cordTo);    // clear the destination plane
  switch (axis)
  {
  case AXIS_Z:
    for (uint8_t i = 0; i < CUBE_XSIZE; i++)
    {
      for (uint8_t j = 0; j < CUBE_YSIZE; j++)
      {
        if (getvoxel(i, j, cordFrom))
        
          setvoxel(i, j, cordTo);
        else
          clrvoxel(i, j, cordTo);
      }
    }
    break;
  case AXIS_Y:
    for (uint8_t i = 0; i < CUBE_XSIZE; i++)
    {
      for (uint8_t j = 0; j < CUBE_ZSIZE; j++)
      {
        
        if (getvoxel(i, cordFrom, j))
        
          setvoxel(i, cordTo, j);
        else
          clrvoxel(i, cordTo, j);
      }
    }
    break;
  case AXIS_X:
    for (uint8_t i = 0; i < CUBE_YSIZE; i++)
    {
      for (uint8_t j = 0; j < CUBE_ZSIZE; j++)
      {

        if (getvoxel(cordFrom, i, j))
        
          setvoxel(cordTo, i, j);
        else
          clrvoxel(cordTo, i, j);
      }
    }
    break;
  }
}

// Scroll the outer sides of the cube around an axis by value
void TranslateScroll8(byte axis, int8_t value)
{
  uint8_t i, j, k, l, x, y, z, prev_x, prev_y, prev_z, dx, dy, dz, start_state1, start_state2, start_state3;
  int8_t positive, negative;
  int8_t increment_x[4];
  int8_t increment_y[4];
  int8_t increment_z[4];

  positive = (value > 0);
  negative = (value < 0);

  switch (axis)
  {
  default:
  case AXIS_X:
    increment_x[0] = 0;
    increment_x[1] = 0;
    increment_x[2] = 0;
    increment_x[3] = 0;

    increment_y[0] = positive;
    increment_y[1] = negative;
    increment_y[2] = -positive;
    increment_y[3] = -negative;

    increment_z[0] = negative;
    increment_z[1] = positive;
    increment_z[2] = -negative;
    increment_z[3] = -positive;

    dx = 1;
    dy = 0;
    dz = 0;
    break;

  case AXIS_Y:
    increment_x[0] = positive;
    increment_x[1] = negative;
    increment_x[2] = -positive;
    increment_x[3] = -negative;

    increment_y[0] = 0;
    increment_y[1] = 0;
    increment_y[2] = 0;
    increment_y[3] = 0;

    increment_z[0] = negative;
    increment_z[1] = positive;
    increment_z[2] = -negative;
    increment_z[3] = -positive;

    dx = 0;
    dy = 1;
    dz = 0;
    break;

  case AXIS_Z:
    increment_x[0] = negative;
    increment_x[1] = positive;
    increment_x[2] = -negative;
    increment_x[3] = -positive;

    increment_y[0] = positive;
    increment_y[1] = negative;
    increment_y[2] = -positive;
    increment_y[3] = -negative;

    increment_z[0] = 0;
    increment_z[1] = 0;
    increment_z[2] = 0;
    increment_z[3] = 0;

    dx = 0;
    dy = 0;
    dz = 1;
    break;
  }

  value -= ((value >= 0) - 1) & (value << 1);

  for (l = 0; l<value; l++)
  {
    x = 0;
    y = 0;
    z = 0;

    for (i = 0; i < 8; i++)
    {
      start_state1 = getvoxel(x, y, z);
      start_state2 = getvoxel(x, 15-y, 7-z);
      start_state3 = getvoxel(7-x, 23-y, z);
      for (j = 0; j < 4; j++)
      {
        for (k = 0; k < 7; k++)
        {
          prev_x = x;
          prev_y = y;
          prev_z = z;

          x += increment_x[j];
          y += increment_y[j];
          z += increment_z[j];

          altervoxel(prev_x, prev_y, prev_z,getvoxel(x, y, z));
          altervoxel(prev_x, 15-prev_y, 7-prev_z,getvoxel(x, 15-y, 7-z));
          altervoxel(7-prev_x, 23-prev_y, prev_z,getvoxel(7-x, 23-y, z));
        }
      }

      altervoxel(prev_x, prev_y, prev_z, start_state1);
      altervoxel(prev_x, 15-prev_y, 7-prev_z, start_state2);
      altervoxel(7-prev_x, 23-prev_y, prev_z, start_state3);
      x += dx;
      y += dy;
      z += dz;
    }
  }
}


void Bresenham3D(int8_t x1, int8_t y1, int8_t z1, const int8_t x2, const int8_t y2, const int8_t z2, const byte modex)
{ 
  int8_t i, dx, dy, dz, l, m, n, x_inc, y_inc, z_inc, err_1, err_2, dx2, dy2, dz2;
  int8_t point[3];
  
  point[0] = x1;
  point[1] = y1;
  point[2] = z1;
  dx = x2 - x1;
  dy = y2 - y1;
  dz = z2 - z1;
  x_inc = (dx < 0) ? -1 : 1;
  l = abs(dx);
  y_inc = (dy < 0) ? -1 : 1;
  m = abs(dy);
  z_inc = (dz < 0) ? -1 : 1;
  n = abs(dz);
  dx2 = l << 1;
  dy2 = m << 1;
  dz2 = n << 1;
  
  if ((l >= m) && (l >= n)) {
      err_1 = dy2 - l;
      err_2 = dz2 - l;
      for (i = 0; i < l; i++) {
          if (modex) setvoxel(point[0], point[1], point[2]); else clrvoxel(point[0], point[1], point[2]);
          if (err_1 > 0) {
              point[1] += y_inc;
              err_1 -= dx2;
          }
          if (err_2 > 0) {
              point[2] += z_inc;
              err_2 -= dx2;
          }
          err_1 += dy2;
          err_2 += dz2;
          point[0] += x_inc;
      }
  } else if ((m >= l) && (m >= n)) {
      err_1 = dx2 - m;
      err_2 = dz2 - m;
      for (i = 0; i < m; i++) {
          if (modex) setvoxel(point[0], point[1], point[2]); else clrvoxel(point[0], point[1], point[2]);
          if (err_1 > 0) {
              point[0] += x_inc;
              err_1 -= dy2;
          }
          if (err_2 > 0) {
              point[2] += z_inc;
              err_2 -= dy2;
          }
          err_1 += dx2;
          err_2 += dz2;
          point[1] += y_inc;
      }
  } else {
      err_1 = dy2 - n;
      err_2 = dx2 - n;
      for (i = 0; i < n; i++) {
          if (modex) setvoxel(point[0], point[1], point[2]); else clrvoxel(point[0], point[1], point[2]);
          if (err_1 > 0) {
              point[1] += y_inc;
              err_1 -= dz2;
          }
          if (err_2 > 0) {
              point[0] += x_inc;
              err_2 -= dz2;
          }
          err_1 += dy2;
          err_2 += dx2;
          point[2] += z_inc;
      }
  }
  if (modex) setvoxel(point[0], point[1], point[2]); else clrvoxel(point[0], point[1], point[2]);
}


void CalcLine3D(int8_t x1, int8_t y1, int8_t z1, const int8_t x2, const int8_t y2, const int8_t z2, byte modex)
{
  Bresenham3D(x1,y1,z1, x2,y2,z2, modex);
}

void DrawLine3D(int8_t x0, int8_t y0, int8_t z0, const int8_t x1, const int8_t y1, const int8_t z1)
{
  CalcLine3D(x0,y0,z0, x1,y1,z1, 1);
}

void EraseLine3D(int8_t x0, int8_t y0, int8_t z0, const int8_t x1, const int8_t y1, const int8_t z1)
{
  CalcLine3D(x0,y0,z0, x1,y1,z1, 0);
}


/*------------------------------KEVIN DARRAH-----------------------------------------------*/

void sinwaveTwo(){
  int sinewavearray[8], addr, sinemult[8], colselect, addrt;
  int sinewavearrayOLD[8], subZ=-7, subT=7, multi=0;//random(-1, 2);
  sinewavearray[0]=0;
  sinemult[0]=1;
  sinewavearray[1]=1;
  sinemult[1]=1; 
  sinewavearray[2]=2;
  sinemult[2]=1;
  sinewavearray[3]=3;
  sinemult[3]=1;
  sinewavearray[4]=4;
  sinemult[4]=1;
  sinewavearray[5]=5;
  sinemult[5]=1;
  sinewavearray[6]=6;
  sinemult[6]=1;
  sinewavearray[7]=7;
  sinemult[7]=1;
  
  start=millis();
      
  while(millis()-start<30000)
  {
    for(addr=0; addr<8; addr++)
    {
      if(sinewavearray[addr]==7)
      {
        sinemult[addr]=-1;
      }
      if(sinewavearray[addr]==0)
      {
        sinemult[addr]=1;     
      }
    sinewavearray[addr] = sinewavearray[addr] + sinemult[addr];
    }

    for(addr=0; addr<8; addr++)
    {
      clrvoxel(0, addr, sinewavearrayOLD[addr]);
      clrvoxel(addr, 0, sinewavearrayOLD[addr]);
      clrvoxel(7, subT-addr, sinewavearrayOLD[addr]);
      clrvoxel(subT-addr, 7, sinewavearrayOLD[addr]);     
      setvoxel(0, addr, sinewavearray[addr]);
      setvoxel(addr, 0, sinewavearray[addr]);
      setvoxel(7, subT-addr,sinewavearray[addr]);
      setvoxel(subT-addr, 7, sinewavearray[addr]);
  
      clrvoxel(7, addr + 8, sinewavearrayOLD[addr]);
      clrvoxel(7-addr, 8, sinewavearrayOLD[addr]);
      clrvoxel(0, 15-addr, sinewavearrayOLD[addr]);
      clrvoxel(addr, 15, sinewavearrayOLD[addr]);     
      setvoxel(7, addr + 8, sinewavearray[addr]);
      setvoxel(7-addr, 8, sinewavearray[addr]);
      setvoxel(0, 15-addr,sinewavearray[addr]);
      setvoxel(addr, 15, sinewavearray[addr]);

      clrvoxel(7, addr + 16, sinewavearrayOLD[addr]);
      clrvoxel(7-addr, 16, sinewavearrayOLD[addr]);
      clrvoxel(0, 23-addr, sinewavearrayOLD[addr]);
      clrvoxel(addr, 23, sinewavearrayOLD[addr]);     
      setvoxel(7, addr + 16, sinewavearray[addr]);
      setvoxel(7-addr, 16, sinewavearray[addr]);
      setvoxel(0, 23-addr,sinewavearray[addr]);
      setvoxel(addr, 23, sinewavearray[addr]);    
    }
    
    for(addr=1; addr<7; addr++)
    {   
      clrvoxel(1, addr, sinewavearrayOLD[addr+multi*1]);
      clrvoxel(addr, 1, sinewavearrayOLD[addr+multi*1]);
      clrvoxel(6, subT-addr, sinewavearrayOLD[addr+multi*1]);
      clrvoxel(subT-addr, 6, sinewavearrayOLD[addr+multi*1]);  
      setvoxel(1, addr, sinewavearray[addr+multi*1]);
      setvoxel(addr, 1, sinewavearray[addr+multi*1]);
      setvoxel(6, subT-addr,sinewavearray[addr+multi*1]);
      setvoxel(subT-addr, 6, sinewavearray[addr+multi*1]);
  
      clrvoxel(6, addr+8, sinewavearrayOLD[addr+multi*1]);
      clrvoxel(7-addr, 9, sinewavearrayOLD[addr+multi*1]);
      clrvoxel(1, 15-addr, sinewavearrayOLD[addr+multi*1]);
      clrvoxel(addr, 14, sinewavearrayOLD[addr+multi*1]);  
      setvoxel(6, addr + 8, sinewavearray[addr+multi*1]);
      setvoxel(7-addr, 9, sinewavearray[addr+multi*1]);
      setvoxel(1, 15-addr,sinewavearray[addr+multi*1]);
      setvoxel(addr, 14, sinewavearray[addr+multi*1]);   

      clrvoxel(6, addr+16, sinewavearrayOLD[addr+multi*1]);
      clrvoxel(7-addr, 17, sinewavearrayOLD[addr+multi*1]);
      clrvoxel(1, 23-addr, sinewavearrayOLD[addr+multi*1]);
      clrvoxel(addr, 22, sinewavearrayOLD[addr+multi*1]);  
      setvoxel(6, addr + 16, sinewavearray[addr+multi*1]);
      setvoxel(7-addr, 17, sinewavearray[addr+multi*1]);
      setvoxel(1, 23-addr,sinewavearray[addr+multi*1]);
      setvoxel(addr, 22, sinewavearray[addr+multi*1]);         
    }
 
    for(addr=2; addr<6; addr++)
    {   
      clrvoxel(2, addr, sinewavearrayOLD[addr+multi*2]);
      clrvoxel(addr, 2, sinewavearrayOLD[addr+multi*2]);
      clrvoxel(5, subT-addr, sinewavearrayOLD[addr+multi*2]);
      clrvoxel(subT-addr, 5, sinewavearrayOLD[addr+multi*2]);  
      setvoxel(2, addr, sinewavearray[addr+multi*2]);
      setvoxel(addr, 2, sinewavearray[addr+multi*2]);
      setvoxel(5, subT-addr,sinewavearray[addr+multi*2]);
      setvoxel(subT-addr, 5, sinewavearray[addr+multi*2]);
  
      clrvoxel(5, addr+8, sinewavearrayOLD[addr+multi*2]);
      clrvoxel(7-addr, 10, sinewavearrayOLD[addr+multi*2]);
      clrvoxel(2, 15-addr, sinewavearrayOLD[addr+multi*2]);
      clrvoxel(addr, 13, sinewavearrayOLD[addr+multi*2]);  
      setvoxel(5, addr+8, sinewavearray[addr+multi*2]);
      setvoxel(7-addr, 10, sinewavearray[addr+multi*2]);
      setvoxel(2, 15-addr,sinewavearray[addr+multi*2]);
      setvoxel(addr, 13, sinewavearray[addr+multi*2]);

      clrvoxel(5, addr+16, sinewavearrayOLD[addr+multi*2]);
      clrvoxel(7-addr, 18, sinewavearrayOLD[addr+multi*2]);
      clrvoxel(2, 23-addr, sinewavearrayOLD[addr+multi*2]);
      clrvoxel(addr, 21, sinewavearrayOLD[addr+multi*2]);  
      setvoxel(5, addr+16, sinewavearray[addr+multi*2]);
      setvoxel(7-addr, 18, sinewavearray[addr+multi*2]);
      setvoxel(2, 23-addr,sinewavearray[addr+multi*2]);
      setvoxel(addr, 21, sinewavearray[addr+multi*2]);      
    }  
    for(addr=3; addr<5; addr++)
    {   
      clrvoxel(3, addr, sinewavearrayOLD[addr+multi*3]);
      clrvoxel(addr, 3, sinewavearrayOLD[addr+multi*3]);
      clrvoxel(4, subT-addr, sinewavearrayOLD[addr+multi*3]);
      clrvoxel(subT-addr, 4, sinewavearrayOLD[addr+multi*3]);  
      setvoxel(3, addr, sinewavearray[addr+multi*3]);
      setvoxel(addr, 3, sinewavearray[addr+multi*3]);
      setvoxel(4, subT-addr,sinewavearray[addr+multi*3]);
      setvoxel(subT-addr, 4, sinewavearray[addr+multi*3]);   
      
      clrvoxel(4, addr+8, sinewavearrayOLD[addr+multi*3]);
      clrvoxel(7-addr, 11, sinewavearrayOLD[addr+multi*3]);
      clrvoxel(3, 15-addr, sinewavearrayOLD[addr+multi*3]);
      clrvoxel(addr, 12, sinewavearrayOLD[addr+multi*3]);  
      setvoxel(4, addr+8, sinewavearray[addr+multi*3]);
      setvoxel(7-addr, 11, sinewavearray[addr+multi*3]);
      setvoxel(3, 15-addr,sinewavearray[addr+multi*3]);
      setvoxel(addr, 12, sinewavearray[addr+multi*3]); 

      clrvoxel(4, addr+16, sinewavearrayOLD[addr+multi*3]);
      clrvoxel(7-addr, 19, sinewavearrayOLD[addr+multi*3]);
      clrvoxel(3, 23-addr, sinewavearrayOLD[addr+multi*3]);
      clrvoxel(addr, 20, sinewavearrayOLD[addr+multi*3]);  
      setvoxel(4, addr+16, sinewavearray[addr+multi*3]);
      setvoxel(7-addr, 19, sinewavearray[addr+multi*3]);
      setvoxel(3, 23-addr,sinewavearray[addr+multi*3]);
      setvoxel(addr, 20, sinewavearray[addr+multi*3]);       
    }  
     
  for(addr=0; addr<8; addr++) 
    sinewavearrayOLD[addr]=sinewavearray[addr];
    delay(50);   
  } 
}


void wipe_out(){
  int xxx=0, yyy=0, zzz=0;
  int fx=random(8), fy=random(24), fz=random(8), direct, fxm=1, fym=1, fzm=1, fxo=0, fyo=0, fzo=0;
  int  ftx=random(8), fty=random(24), ftz=random(8), ftxm=1, ftym=1, ftzm=1, ftxo=0, ftyo=0, ftzo=0;
  for(xxx=0; xxx<8; xxx++)
  {
    for(yyy=0; yyy<24; yyy++)
    {
      for(zzz=0; zzz<8; zzz++)
      {
        clrvoxel(xxx, yyy, zzz);
      }
    }
  }
    
  start=millis();
      
  while(millis()-start<25000)
  {
    //fx=random(8); fy=random(8); fz=random(8);

    clrvoxel(fxo, fyo, fzo);
    clrvoxel(fxo, fyo, fzo+1);
    clrvoxel(fxo, fyo, fzo-1);
    clrvoxel(fxo+1, fyo, fzo);
    clrvoxel(fxo-1, fyo, fzo);
    clrvoxel(fxo, fyo+1, fzo);
    clrvoxel(fxo, fyo-1, fzo);
    
    clrvoxel(ftxo, ftyo, ftzo);
    clrvoxel(ftxo, ftyo, ftzo+1);
    clrvoxel(ftxo, ftyo, ftzo-1);
    clrvoxel(ftxo+1, ftyo, ftzo);
    clrvoxel(ftxo-1, ftyo, ftzo);
    clrvoxel(ftxo, ftyo+1, ftzo);
    clrvoxel(ftxo, ftyo-1, ftzo);

    setvoxel(ftx, fty, ftz);
    setvoxel(ftx, fty, ftz+1);
    setvoxel(ftx, fty, ftz-1);
    setvoxel(ftx+1, fty, ftz);
    setvoxel(ftx-1, fty, ftz);
    setvoxel(ftx, fty+1, ftz);
    setvoxel(ftx, fty-1, ftz);     
    
    setvoxel(fx, fy, fz);
    setvoxel(fx, fy, fz+1);
    setvoxel(fx, fy, fz-1);
    setvoxel(fx+1, fy, fz);
    setvoxel(fx-1, fy, fz);
    setvoxel(fx, fy+1, fz);
    setvoxel(fx, fy-1, fz);  
       
    delay(10);
     
    fxo=fx;
    fyo=fy;
    fzo=fz; 
    
    ftxo=ftx;
    ftyo=fty;
    ftzo=ftz; 
 
    direct=random(3);
    if(direct==0)
    fx= fx+fxm;
    if(direct==1)
    fy= fy+fym;  
    if(direct==2)
    fz= fz+fzm;  
    if(fx<0){
    fx=0; fxm=1;}
    if(fx>7){
    fx=7; fxm=-1;}  
    if(fy<0){
    fy=0; fym=1;}
    if(fy>23){
    fy=23; fym=-1;}    
    if(fz<0){
    fz=0; fzm=1;}
    if(fz>7){
    fz=7; fzm=-1;}  
  
    direct=random(3);
    if(direct==0)
    ftx= ftx+ftxm;
    if(direct==1)
    fty= fty+ftym;  
    if(direct==2)
    ftz= ftz+ftzm;  
    if(ftx<0){
    ftx=0; ftxm=1;}
    if(ftx>7){
    ftx=7; ftxm=-1;}  
    if(fty<0){
    fty=0; ftym=1;}
    if(fty>23){
    fty=23; ftym=-1;}    
    if(ftz<0){
    ftz=0; ftzm=1;}
    if(ftz>7){
    ftz=7; ftzm=-1;} 
  }//while
  for(xxx=0; xxx<8; xxx++)
  {
    for(yyy=0; yyy<24; yyy++)
    {
      for(zzz=0; zzz<8; zzz++)
      {
        clrvoxel(xxx, yyy, zzz);
      }
    }
  }  
}


int checkConstrains(int value, int min, int max) {
  if(value < min) {
    return min;
  } else if (value > max) {
    return max;
  } else {
    return value;
  }
}


void drawRPrism(uint8_t x, uint8_t y, uint8_t z, int8_t dx, int8_t dy, int8_t dz)
{
  // top rectangle
  line_3d(x, y, z, x, y+dy, z);
  line_3d(x, y+dy, z, x+dx, y+dy, z);
  line_3d(x+dx, y+dy, z, x+dx, y, z);
  line_3d(x+dx, y, z, x, y, z);

  // bottom rectangle
  line_3d(x, y, z+dz, x, y+dy, z+dz);
  line_3d(x, y+dy, z+dz, x+dx, y+dy, z+dz);
  line_3d(x+dx, y+dy, z+dz, x+dx, y, z+dz);
  line_3d(x+dx, y, z+dz, x, y, z+dz);

  // joining verticals
  line_3d(x, y, z, x, y, z+dz);
  line_3d(x, y+dy, z, x, y+dy, z+dz);
  line_3d(x+dx, y+dy, z, x+dx, y+dy, z+dz);
  line_3d(x+dx, y, z, x+dx, y, z+dz);
}

void clrRPrism(uint8_t x, uint8_t y, uint8_t z, int8_t dx, int8_t dy, int8_t dz)
{
  // top rectangle
  clear_line_3d(x, y, z, x, y+dy, z);
  clear_line_3d(x, y+dy, z, x+dx, y+dy, z);
  clear_line_3d(x+dx, y+dy, z, x+dx, y, z);
  clear_line_3d(x+dx, y, z, x, y, z);

  // bottom rectangle
  clear_line_3d(x, y, z+dz, x, y+dy, z+dz);
  clear_line_3d(x, y+dy, z+dz, x+dx, y+dy, z+dz);
  clear_line_3d(x+dx, y+dy, z+dz, x+dx, y, z+dz);
  clear_line_3d(x+dx, y, z+dz, x, y, z+dz);

  // joining verticals
  clear_line_3d(x, y, z, x, y, z+dz);
  clear_line_3d(x, y+dy, z, x, y+dy, z+dz);
  clear_line_3d(x+dx, y+dy, z, x+dx, y+dy, z+dz);
  clear_line_3d(x+dx, y, z, x+dx, y, z+dz);
}

void drawCube(uint8_t x, uint8_t y, uint8_t z, int8_t size) { drawRPrism(x, y, z, size-1, size-1, size-1); };

void boingboing(uint16_t iterations, int delayx, unsigned char modex, unsigned char drawmode)
{
  clearfast();   // Blank the cube

  int x, y, z;    // Current coordinates for the point
  int dx, dy, dz; // Direction of movement
  int lol, i;   // lol?
  unsigned char crash_x, crash_y, crash_z;

  y = rand()%24;
  x = rand()%8;
  z = rand()%8;

  // Coordinate array for the snake.
  int snake[8][3];
  for (i=0;i<8;i++)
  {
    snake[i][0] = x;
    snake[i][1] = y;
    snake[i][2] = z;
  }
  
  
  dx = 1;
  dy = 1;
  dz = 1;
  
  while(iterations)
  {
    crash_x = 0;
    crash_y = 0;
    crash_z = 0;
  

    // Let's mix things up a little:
    if (rand()%3 == 0)
    {
      // Pick a random axis, and set the speed to a random number.
      lol = rand()%3;
      if (lol == 0)
        dx = rand()%3 - 1;
      
      if (lol == 1)
        dy = rand()%3 - 1;
        
      if (lol == 2)
        dz = rand()%3 - 1;
    }

      // The point has reached 0 on the x-axis and is trying to go to -1
        // aka a crash
    if (dx == -1 && x == 0)
    {
      crash_x = 0x01;
      if (rand()%3 == 1)
      {
        dx = 1;
      } else
      {
        dx = 0;
      }
    }
    
        // y axis 0 crash
    if (dy == -1 && y == 0)
    {
      crash_y = 0x01;
      if (rand()%3 == 1)
      {
        dy = 1;
      } else
      {
        dy = 0;
      }
    }
    
        // z axis 0 crash
    if (dz == -1 && z == 0)
    {
      crash_z = 0x01;
      if (rand()%3 == 1)
      {
        dz = 1;
      } else
      {
        dz = 0;
      }
    }
      
        // x axis 7 crash
    if (dx == 1 && x == 7)
    {
      crash_x = 0x01;
      if (rand()%3 == 1)
      {
        dx = -1;
      } else
      {
        dx = 0;
      }
    }
    
        // y axis 7 crash
    if (dy == 1 && y == 23)
    {
      crash_y = 0x01;
      if (rand()%3 == 1)
      {
        dy = -1;
      } else
      {
        dy = 0;
      }
    }
    
        // z azis 7 crash
    if (dz == 1 && z == 7)
    {
      crash_z = 0x01;
      if (rand()%3 == 1)
      {
        dz = -1;
      } else
      {
        dz = 0;
      }
    }
    
    // mode bit 0 sets crash action enable
    if (modex | 0x01)
    {
      if (crash_x)
      {
        if (dy == 0)
        {
          if (y == 23)
          {
            dy = -1;
          } else if (y == 0)
          {
            dy = +1;
          } else
          {
            if (rand()%2 == 0)
            {
              dy = -1;
            } else
            {
              dy = 1;
            }
          }
        }
        if (dz == 0)
        {
          if (z == 7)
          {
            dz = -1;
          } else if (z == 0)
          {
            dz = 1;
          } else
          {
            if (rand()%2 == 0)
            {
              dz = -1;
            } else
            {
              dz = 1;
            }
          } 
        }
      }
      
      if (crash_y)
      {
        if (dx == 0)
        {
          if (x == 7)
          {
            dx = -1;
          } else if (x == 0)
          {
            dx = 1;
          } else
          {
            if (rand()%2 == 0)
            {
              dx = -1;
            } else
            {
              dx = 1;
            }
          }
        }
        if (dz == 0)
        {
          if (z == 3)
          {
            dz = -1;
          } else if (z == 0)
          {
            dz = 1;
          } else
          {
            if (rand()%2 == 0)
            {
              dz = -1;
            } else
            {
              dz = 1;
            }
          } 
        }
      }
      
      if (crash_z)
      {
        if (dy == 0)
        {
          if (y == 23)
          {
            dy = -1;
          } else if (y == 0)
          {
            dy = 1;
          } else
          {
            if (rand()%2 == 0)
            {
              dy = -1;
            } else
            {
              dy = 1;
            }
          } 
        }
        if (dx == 0)
        {
          if (x == 7)
          {
            dx = -1;
          } else if (x == 0)
          {
            dx = 1;
          } else
          {
            if (rand()%2 == 0)
            {
              dx = -1;
            } else
            {
              dx = 1;
            }
          } 
        }
      }
    }
    
    // mode bit 1 sets corner avoid enable
    if (modex | 0x02)
    {
      if (  // We are in one of 8 corner positions
        (x == 0 && y == 0 && z == 0) ||
        (x == 0 && y == 0 && z == 7) ||
        (x == 0 && y == 23 && z == 0) ||
        (x == 0 && y == 23 && z == 7) ||
        (x == 7 && y == 0 && z == 0) ||
        (x == 7 && y == 0 && z == 7) ||
        (x == 7 && y == 23 && z == 0) ||
        (x == 7 && y == 23 && z == 7)
      )
      {
        // At this point, the voxel would bounce
        // back and forth between this corner,
        // and the exact opposite corner
        // We don't want that!
      
        // So we alter the trajectory a bit,
        // to avoid corner stickyness
        lol = rand()%3;
        if (lol == 0)
          dx = 0;
        
        if (lol == 1)
          dy = 0;
          
        if (lol == 2)
          dz = 0;
      }
    }

        // one last sanity check
        if (x == 0 && dx == -1)
            dx = 1;
  
        if (y == 0 && dy == -1)
            dy = 1;
  
        if (z == 0 && dz == -1)
            dz = 1;
  
        if (x == 7 && dx == 1)
            dx = -1;
  
        if (y == 23 && dy == 1)
            dy = -1;
  
        if (z == 7 && dz == 1)
            dz = -1;
  
  
    // Finally, move the voxel.
    x = x + dx;
    y = y + dy;
    z = z + dz;
    
    if (drawmode == 0x01) // show one voxel at time
    {
      setvoxel(z, y, x);
      delay(delayx);
      clrvoxel(z, y, x);  
    } else if (drawmode == 0x02) // flip the voxel in question
    {
      flpvoxel(z,y,x);
      delay(delayx);
    } if (drawmode == 0x03) // draw a snake
    {
      for (i=7;i>=0;i--)
      {
        snake[i][0] = snake[i-1][0];
        snake[i][1] = snake[i-1][1];
        snake[i][2] = snake[i-1][2];
      }
      snake[0][0] = x;
      snake[0][1] = y;
      snake[0][2] = z;
        
      for (i=0;i<8;i++)
      {
        setvoxel(snake[i][2], snake[i][1], snake[i][0]);
      }
      delay(delayx);
      for (i=0;i<8;i++)
      {
        clrvoxel(snake[i][2], snake[i][1], snake[i][0]);
      }
    }
    iterations--;
  }
}


void squarespiral(int iterations, int Delay)
{
int loc = 0;
int iter = 0;
while (iter <= iterations)
{

  for (loc =0;loc < 7; loc ++)
  {
    shift(AXIS_Z,-1);
    setvoxel(0,loc,7);
    setvoxel(0,loc+8,7);
    setvoxel(0,loc+16,7);

    
    setvoxel(loc,7,7);
    setvoxel(loc,15,7);
    setvoxel(loc,23,7);
    
    setvoxel(7,7-loc,7);
    setvoxel(7,15-loc,7);
    setvoxel(7,23-loc,7);
    
    setvoxel(7-loc,0,7);
    setvoxel(7-loc,8,7); 
    setvoxel(7-loc,16,7);
    
    delay(Delay);
    iter++;
  }
  loc = 0;
  }
}

void brownian(int interactions)
// 2x2 cube doing brownian motion
{
  uint8_t x=0 , y=0 , z=0 ; 
  int8_t dx, dy, dz;

  for (int i=0; i<interactions; i++)
  {
    clearfast();
    drawCube(x, y, z, 2);
    delay(100);

    dx = random(3) - 1;
    dy = random(10) - 1;
    dz = random(3) - 1;
    if ((x + dx >= 0) && (x + dx + 1 < 8)) x += dx;
    if ((y + dy >= 0) && (y + dy + 1 < 24)) y += dy;
    if ((z + dz >= 0) && (z + dz + 1 < 8)) z += dz;
  }    
}

void folder()
{
  int xx, yy, zz, pullback[16], state=0, backorfront=7;
  
  int folderaddr[16], LED_Old[16], oldpullback[16], ranx=random(16), rany=random(16), ranz=random(16), ranselect;
  int bot=0, top=1, right=0, left=0, back=0, front=0, side=0, side_select;
  
  folderaddr[0]=-7;
  folderaddr[1]=-6;
  folderaddr[2]=-5;
  folderaddr[3]=-4;
  folderaddr[4]=-3;
  folderaddr[5]=-2;
  folderaddr[6]=-1;
  folderaddr[7]=0;
  
  for(xx=0; xx<8; xx++)
  {
  oldpullback[xx]=0;
  pullback[xx]=0;
  }
    
  start=millis();
  while(millis()-start<50000)
  { 
    if(top==1)
      {
      if(side==0)
      {
        //top to left-side
        for(yy=0; yy<8; yy++)
        {
          for(xx=0; xx<8; xx++)
          {
            clrvoxel(checkConstrains(7-LED_Old[yy], 0, 7), checkConstrains(yy-oldpullback[yy], 0, 7),xx);
            setvoxel(checkConstrains(7-folderaddr[yy], 0, 7), checkConstrains(yy-pullback[yy], 0, 7),xx);
            
            clrvoxel(7-checkConstrains(7-LED_Old[yy], 0, 7), checkConstrains(yy-oldpullback[yy], 0, 7)+8,xx);
            setvoxel(7-checkConstrains(7-folderaddr[yy], 0, 7), checkConstrains(yy-pullback[yy], 0, 7)+8,xx);
            
            clrvoxel(7-checkConstrains(7-LED_Old[yy], 0, 7), checkConstrains(yy-oldpullback[yy], 0, 7)+16,xx);
            setvoxel(7-checkConstrains(7-folderaddr[yy], 0, 7), checkConstrains(yy-pullback[yy], 0, 7)+16,xx);
          }
        }
      }
      if(side==2)
      {
        //top to back-side
        for(yy=0; yy<8; yy++)
        {
            for(xx=0; xx<8; xx++)
            {
              clrvoxel(checkConstrains(7-LED_Old[yy], 0, 7), xx, checkConstrains(yy-oldpullback[yy], 0, 7));
              setvoxel(checkConstrains(7-folderaddr[yy], 0, 7), xx, checkConstrains(yy-pullback[yy], 0, 7));
            
              clrvoxel(7-checkConstrains(7-LED_Old[yy], 0, 7), xx+8, checkConstrains(yy-oldpullback[yy], 0, 7));
              setvoxel(7-checkConstrains(7-folderaddr[yy], 0, 7), xx+8, checkConstrains(yy-pullback[yy], 0, 7));
            
              clrvoxel(7-checkConstrains(7-LED_Old[yy], 0, 7), xx+16, checkConstrains(yy-oldpullback[yy], 0, 7));
              setvoxel(7-checkConstrains(7-folderaddr[yy], 0, 7), xx+16, checkConstrains(yy-pullback[yy], 0, 7));
            }
          }
        }
      if(side==3)
      {
        //top-side to front-side
        for(yy=0; yy<8; yy++)
        {
          for(xx=0; xx<8; xx++)
          {
              clrvoxel(checkConstrains(7-LED_Old[7-yy], 0, 7), xx, checkConstrains(yy+oldpullback[yy], 0, 7));
              setvoxel(checkConstrains(7-folderaddr[7-yy], 0, 7), xx, checkConstrains(yy+pullback[yy], 0, 7));
              
              clrvoxel(7-checkConstrains(7-LED_Old[7-yy], 0, 7), xx+8, checkConstrains(yy+oldpullback[yy], 0, 7));
              setvoxel(7-checkConstrains(7-folderaddr[7-yy], 0, 7), xx+8, checkConstrains(yy+pullback[yy], 0, 7));
            
              clrvoxel(7-checkConstrains(7-LED_Old[7-yy], 0, 7), xx+16, checkConstrains(yy+oldpullback[yy], 0, 7));
              setvoxel(7-checkConstrains(7-folderaddr[7-yy], 0, 7), xx+16, checkConstrains(yy+pullback[yy], 0, 7));
    
          }
        }
      }
      if(side==1)
      {
        //top-side to right
        for(yy=0; yy<8; yy++)
        {
          for(xx=0; xx<8; xx++)
          {
            clrvoxel(checkConstrains(7-LED_Old[7-yy], 0, 7), checkConstrains(yy+oldpullback[yy], 0, 7),xx);
            setvoxel(checkConstrains(7-folderaddr[7-yy], 0, 7), checkConstrains(yy+pullback[yy], 0, 7),xx);
          
            clrvoxel(7-checkConstrains(7-LED_Old[7-yy], 0, 7), checkConstrains(yy+oldpullback[yy], 0, 7)+8,xx);
            setvoxel(7-checkConstrains(7-folderaddr[7-yy], 0, 7), checkConstrains(yy+pullback[yy], 0, 7)+8,xx);
          
            clrvoxel(7-checkConstrains(7-LED_Old[7-yy], 0, 7), checkConstrains(yy+oldpullback[yy], 0, 7)+16,xx);
            setvoxel(7-checkConstrains(7-folderaddr[7-yy], 0, 7), checkConstrains(yy+pullback[yy], 0, 7)+16,xx);  
          }
        }
      }
    }//top
    
    if(right==1)
    {
      if(side==4)
      {
        //right-side to top
        for(yy=0; yy<8; yy++)
        {
          for(xx=0; xx<8; xx++)
          {
            clrvoxel(checkConstrains(yy+oldpullback[7-yy], 0, 7), checkConstrains(7-LED_Old[7-yy], 0, 7),xx);
            setvoxel(checkConstrains(yy+pullback[7-yy], 0, 7), checkConstrains(7-folderaddr[7-yy], 0, 7),xx);
            
            clrvoxel(7-checkConstrains(yy+oldpullback[7-yy], 0, 7), checkConstrains(7-LED_Old[7-yy], 0, 7)+8,xx);
            setvoxel(7-checkConstrains(yy+pullback[7-yy], 0, 7), checkConstrains(7-folderaddr[7-yy], 0, 7)+8,xx);
          
            clrvoxel(7-checkConstrains(yy+oldpullback[7-yy], 0, 7), checkConstrains(7-LED_Old[7-yy], 0, 7)+16,xx);
            setvoxel(7-checkConstrains(yy+pullback[7-yy], 0, 7), checkConstrains(7-folderaddr[7-yy], 0, 7)+16,xx);  
          }
        }
      }
      if(side==3)
      {
        //right-side to front-side
        for(yy=0; yy<8; yy++)
        {
          for(xx=0; xx<8; xx++)
          {
            clrvoxel(xx, checkConstrains(7-LED_Old[7-yy], 0, 7), checkConstrains(yy+oldpullback[yy], 0, 7));
            setvoxel(xx, checkConstrains(7-folderaddr[7-yy], 0, 7), checkConstrains(yy+pullback[yy], 0, 7));
          
            clrvoxel(7-xx, checkConstrains(7-LED_Old[7-yy], 0, 7)+8, checkConstrains(yy+oldpullback[yy], 0, 7));
            setvoxel(7-xx, checkConstrains(7-folderaddr[7-yy], 0, 7)+8, checkConstrains(yy+pullback[yy], 0, 7));
          
            clrvoxel(7-xx, checkConstrains(7-LED_Old[7-yy], 0, 7)+16, checkConstrains(yy+oldpullback[yy], 0, 7));
            setvoxel(7-xx, checkConstrains(7-folderaddr[7-yy], 0, 7)+16, checkConstrains(yy+pullback[yy], 0, 7));  
          }
        }
      }
      if(side==2)
      {
        //right-side to back-side
        for(yy=0; yy<8; yy++)
        {
          for(xx=0; xx<8; xx++)
          {
            clrvoxel(xx, checkConstrains(7-LED_Old[yy], 0, 7), checkConstrains(yy-oldpullback[yy], 0, 7));
            setvoxel(xx, checkConstrains(7-folderaddr[yy], 0, 7), checkConstrains(yy-pullback[yy], 0, 7));
          
            clrvoxel(7-xx, checkConstrains(7-LED_Old[yy], 0, 7)+8, checkConstrains(yy-oldpullback[yy], 0, 7));
            setvoxel(7-xx, checkConstrains(7-folderaddr[yy], 0, 7)+8, checkConstrains(yy-pullback[yy], 0, 7));
          
            clrvoxel(7-xx, checkConstrains(7-LED_Old[yy], 0, 7)+16, checkConstrains(yy-oldpullback[yy], 0, 7));
            setvoxel(7-xx, checkConstrains(7-folderaddr[yy], 0, 7)+16, checkConstrains(yy-pullback[yy], 0, 7));  
          }
        }
      }
      if(side==5)
      {
        //right-side to bottom
        for(yy=0; yy<8; yy++)
        {
          for(xx=0; xx<8; xx++)
          {
            clrvoxel(checkConstrains(yy-oldpullback[yy], 0, 7), checkConstrains(7-LED_Old[yy], 0, 7),xx);
            setvoxel(checkConstrains(yy-pullback[yy], 0, 7), checkConstrains(7-folderaddr[yy], 0, 7),xx);
          
            clrvoxel(7-checkConstrains(yy-oldpullback[yy], 0, 7), checkConstrains(7-LED_Old[yy], 0, 7)+8,xx);
            setvoxel(7-checkConstrains(yy-pullback[yy], 0, 7), checkConstrains(7-folderaddr[yy], 0, 7)+8,xx);
          
            clrvoxel(7-checkConstrains(yy-oldpullback[yy], 0, 7), checkConstrains(7-LED_Old[yy], 0, 7)+16,xx);
            setvoxel(7-checkConstrains(yy-pullback[yy], 0, 7), checkConstrains(7-folderaddr[yy], 0, 7)+16,xx);      
          }
        }
      }
    }//right
    
    if(left==1)
    {
      if(side==4)
      {
        //left-side to top
        for(yy=0; yy<8; yy++)
        {
          for(xx=0; xx<8; xx++)
          {
            clrvoxel(checkConstrains(yy+oldpullback[yy], 0, 7), checkConstrains(LED_Old[7-yy], 0, 7),xx );
            setvoxel(checkConstrains(yy+pullback[yy], 0, 7), checkConstrains(folderaddr[7-yy], 0, 7),xx);
          
            clrvoxel(7-checkConstrains(yy+oldpullback[yy], 0, 7), checkConstrains(LED_Old[7-yy], 0, 7)+8,xx );
            setvoxel(7-checkConstrains(yy+pullback[yy], 0, 7), checkConstrains(folderaddr[7-yy], 0, 7)+8,xx);
          
            clrvoxel(7-checkConstrains(yy+oldpullback[yy], 0, 7), checkConstrains(LED_Old[7-yy], 0, 7)+16,xx );
            setvoxel(7-checkConstrains(yy+pullback[yy], 0, 7), checkConstrains(folderaddr[7-yy], 0, 7)+16,xx);  
          }
        }
      }
      if(side==3)
      {
        //left-side to front-side
        for(yy=0; yy<8; yy++)
        {
          for(xx=0; xx<8; xx++)
          {
            clrvoxel(xx, checkConstrains(LED_Old[7-yy], 0, 7), checkConstrains(yy+oldpullback[yy], 0, 7));
            setvoxel(xx, checkConstrains(folderaddr[7-yy], 0, 7), checkConstrains(yy+pullback[yy], 0, 7));
          
            clrvoxel(7-xx, checkConstrains(LED_Old[7-yy], 0, 7)+8, checkConstrains(yy+oldpullback[yy], 0, 7));
            setvoxel(7-xx, checkConstrains(folderaddr[7-yy], 0, 7)+8, checkConstrains(yy+pullback[yy], 0, 7));
          
            clrvoxel(7-xx, checkConstrains(LED_Old[7-yy], 0, 7)+16, checkConstrains(yy+oldpullback[yy], 0, 7));
            setvoxel(7-xx, checkConstrains(folderaddr[7-yy], 0, 7)+16, checkConstrains(yy+pullback[yy], 0, 7));  
          }
        }
      }
      if(side==2)
      {
        //left-side to back-side
        for(yy=0; yy<8; yy++)
        {
          for(xx=0; xx<8; xx++)
          {
            clrvoxel(xx,checkConstrains(LED_Old[yy], 0, 7), checkConstrains(yy-oldpullback[yy], 0, 7));
            setvoxel(xx, checkConstrains(folderaddr[yy], 0, 7), checkConstrains(yy-pullback[yy], 0, 7));
          
            clrvoxel(7-xx,checkConstrains(LED_Old[yy], 0, 7)+8, checkConstrains(yy-oldpullback[yy], 0, 7));
            setvoxel(7-xx, checkConstrains(folderaddr[yy], 0, 7)+8, checkConstrains(yy-pullback[yy], 0, 7));
          
            clrvoxel(7-xx,checkConstrains(LED_Old[yy], 0, 7)+16, checkConstrains(yy-oldpullback[yy], 0, 7));
            setvoxel(7-xx, checkConstrains(folderaddr[yy], 0, 7)+16, checkConstrains(yy-pullback[yy], 0, 7));  
          }
        }
      }
      if(side==5)
      {
        //left-side to bottom
        for(yy=0; yy<8; yy++)
        {
          for(xx=0; xx<8; xx++)
          {
            clrvoxel(checkConstrains(yy-oldpullback[yy], 0, 7), checkConstrains(LED_Old[yy], 0, 7),xx);
            setvoxel(checkConstrains(yy-pullback[yy], 0, 7), checkConstrains(folderaddr[yy], 0, 7),xx);
          
            clrvoxel(7-checkConstrains(yy-oldpullback[yy], 0, 7), checkConstrains(LED_Old[yy], 0, 7)+8,xx);
            setvoxel(7-checkConstrains(yy-pullback[yy], 0, 7), checkConstrains(folderaddr[yy], 0, 7)+8,xx);
          
            clrvoxel(7-checkConstrains(yy-oldpullback[yy], 0, 7), checkConstrains(LED_Old[yy], 0, 7)+16,xx);
            setvoxel(7-checkConstrains(yy-pullback[yy], 0, 7), checkConstrains(folderaddr[yy], 0, 7)+16,xx);  
          }
        }
      }
    }//left
  
  
    if(back==1)
    {
      if(side==1)
      {
        //back-side to right-side
        for(yy=0; yy<8; yy++)
        {
          for(xx=0; xx<8; xx++)
          {
            clrvoxel(xx, checkConstrains(yy+oldpullback[yy], 0, 7), checkConstrains(LED_Old[7-yy], 0, 7));
            setvoxel(xx, checkConstrains(yy+pullback[yy], 0, 7), checkConstrains(folderaddr[7-yy], 0, 7));
            
            clrvoxel(7-xx, checkConstrains(yy+oldpullback[yy], 0, 7)+8, checkConstrains(LED_Old[7-yy], 0, 7));
            setvoxel(7-xx, checkConstrains(yy+pullback[yy], 0, 7)+8, checkConstrains(folderaddr[7-yy], 0, 7));
          
            clrvoxel(7-xx, checkConstrains(yy+oldpullback[yy], 0, 7)+16, checkConstrains(LED_Old[7-yy], 0, 7));
            setvoxel(7-xx, checkConstrains(yy+pullback[yy], 0, 7)+16, checkConstrains(folderaddr[7-yy], 0, 7));  
          }
        }
      }
      if(side==4)
      {
        // back-side to top-side
        for(yy=0; yy<8; yy++)
        {
          for(xx=0; xx<8; xx++)
          {
            clrvoxel(checkConstrains(yy+oldpullback[yy], 0, 7),xx, checkConstrains(LED_Old[7-yy], 0, 7));
            setvoxel(checkConstrains(yy+pullback[yy], 0, 7), xx, checkConstrains(folderaddr[7-yy], 0, 7));
          
            clrvoxel(7-checkConstrains(yy+oldpullback[yy], 0, 7),xx+8, checkConstrains(LED_Old[7-yy], 0, 7));
            setvoxel(7-checkConstrains(yy+pullback[yy], 0, 7), xx+8, checkConstrains(folderaddr[7-yy], 0, 7));
          
            clrvoxel(7-checkConstrains(yy+oldpullback[yy], 0, 7),xx+16, checkConstrains(LED_Old[7-yy], 0, 7));
            setvoxel(7-checkConstrains(yy+pullback[yy], 0, 7), xx+16, checkConstrains(folderaddr[7-yy], 0, 7));      
          }
        }
      }
      if(side==5)
      {
        // back-side to bottom
        for(yy=0; yy<8; yy++)
        {
          for(xx=0; xx<8; xx++)
          {
            clrvoxel(checkConstrains(yy-oldpullback[yy], 0, 7), xx, checkConstrains(LED_Old[yy], 0, 7));
            setvoxel(checkConstrains(yy-pullback[yy], 0, 7), xx, checkConstrains(folderaddr[yy], 0, 7));
          
            clrvoxel(7-checkConstrains(yy-oldpullback[yy], 0, 7), xx+8, checkConstrains(LED_Old[yy], 0, 7));
            setvoxel(7-checkConstrains(yy-pullback[yy], 0, 7), xx+8, checkConstrains(folderaddr[yy], 0, 7));
          
            clrvoxel(7-checkConstrains(yy-oldpullback[yy], 0, 7), xx+16, checkConstrains(LED_Old[yy], 0, 7));
            setvoxel(7-checkConstrains(yy-pullback[yy], 0, 7), xx+16, checkConstrains(folderaddr[yy], 0, 7));  
          }
        }
      }//state1
      if(side==0)
      {
        //back-side to left-side
        for(yy=0; yy<8; yy++)
        {
          for(xx=0; xx<8; xx++)
          {
            clrvoxel(xx, checkConstrains(yy-oldpullback[yy], 0, 7), checkConstrains(LED_Old[yy], 0, 7));
            setvoxel(xx, checkConstrains(yy-pullback[yy], 0, 7), checkConstrains(folderaddr[yy], 0, 7));
          
            clrvoxel(7-xx, checkConstrains(yy-oldpullback[yy], 0, 7)+8, checkConstrains(LED_Old[yy], 0, 7));
            setvoxel(7-xx, checkConstrains(yy-pullback[yy], 0, 7)+8, checkConstrains(folderaddr[yy], 0, 7));
          
            clrvoxel(7-xx, checkConstrains(yy-oldpullback[yy], 0, 7)+16, checkConstrains(LED_Old[yy], 0, 7));
            setvoxel(7-xx, checkConstrains(yy-pullback[yy], 0, 7)+16, checkConstrains(folderaddr[yy], 0, 7));  
          }
        }
      }
    }//back
    if(bot==1)
    {
      if(side==1)
      {
        // bottom-side to right-side
        for(yy=0; yy<8; yy++)
        {
          for(xx=0; xx<8; xx++)
          {
            clrvoxel(checkConstrains(LED_Old[7-yy], 0, 7), checkConstrains(yy+oldpullback[yy], 0, 7),xx);
            setvoxel(checkConstrains(folderaddr[7-yy], 0, 7), checkConstrains(yy+pullback[yy], 0, 7),xx);
          
            clrvoxel(7-checkConstrains(LED_Old[7-yy], 0, 7), checkConstrains(yy+oldpullback[yy], 0, 7)+8,xx);
            setvoxel(7-checkConstrains(folderaddr[7-yy], 0, 7), checkConstrains(yy+pullback[yy], 0, 7)+8,xx);
          
            clrvoxel(7-checkConstrains(LED_Old[7-yy], 0, 7), checkConstrains(yy+oldpullback[yy], 0, 7)+16,xx);
            setvoxel(7-checkConstrains(folderaddr[7-yy], 0, 7), checkConstrains(yy+pullback[yy], 0, 7)+16,xx);  
          }
        }
      }
      if(side==3)
      {
        //bottom to front-side
        for(yy=0; yy<8; yy++)
        {
          for(xx=0; xx<8; xx++)
          {
            clrvoxel(checkConstrains(LED_Old[7-yy], 0, 7), xx, checkConstrains(yy+oldpullback[yy], 0, 7));
            setvoxel(checkConstrains(folderaddr[7-yy], 0, 7), xx, checkConstrains(yy+pullback[yy], 0, 7));
          
            clrvoxel(7-checkConstrains(LED_Old[7-yy], 0, 7), xx+8, checkConstrains(yy+oldpullback[yy], 0, 7));
            setvoxel(7-checkConstrains(folderaddr[7-yy], 0, 7), xx+8, checkConstrains(yy+pullback[yy], 0, 7));
          
            clrvoxel(7-checkConstrains(LED_Old[7-yy], 0, 7), xx+16, checkConstrains(yy+oldpullback[yy], 0, 7));
            setvoxel(7-checkConstrains(folderaddr[7-yy], 0, 7), xx+16, checkConstrains(yy+pullback[yy], 0, 7));  
          }
        }
      }
      if(side==2)
      {
        //bottom to back-side
        for(yy=0; yy<8; yy++)
        {
          for(xx=0; xx<8; xx++)
          {
            clrvoxel(checkConstrains(LED_Old[yy], 0, 7), xx, checkConstrains(yy-oldpullback[yy], 0, 7));
            setvoxel(checkConstrains(folderaddr[yy], 0, 7), xx, checkConstrains(yy-pullback[yy], 0, 7));
          
            clrvoxel(7-checkConstrains(LED_Old[yy], 0, 7), xx+8, checkConstrains(yy-oldpullback[yy], 0, 7));
            setvoxel(7-checkConstrains(folderaddr[yy], 0, 7), xx+8, checkConstrains(yy-pullback[yy], 0, 7));
          
            clrvoxel(7-checkConstrains(LED_Old[yy], 0, 7), xx+16, checkConstrains(yy-oldpullback[yy], 0, 7));
            setvoxel(7-checkConstrains(folderaddr[yy], 0, 7), xx+16, checkConstrains(yy-pullback[yy], 0, 7));  
          }
        }
      }
      if(side==0)
      {
        //bottom to left-side
        for(yy=0; yy<8; yy++)
        {
          for(xx=0; xx<8; xx++)
          {
            clrvoxel(checkConstrains(LED_Old[yy], 0, 7), checkConstrains(yy-oldpullback[yy], 0, 7),xx);
            setvoxel(checkConstrains(folderaddr[yy], 0, 7), checkConstrains(yy-pullback[yy], 0, 7),xx);
          
            clrvoxel(7-checkConstrains(LED_Old[yy], 0, 7), checkConstrains(yy-oldpullback[yy], 0, 7)+8,xx);
            setvoxel(7-checkConstrains(folderaddr[yy], 0, 7), checkConstrains(yy-pullback[yy], 0, 7)+8,xx);
          
            clrvoxel(7-checkConstrains(LED_Old[yy], 0, 7), checkConstrains(yy-oldpullback[yy], 0, 7)+16,xx);
            setvoxel(7-checkConstrains(folderaddr[yy], 0, 7), checkConstrains(yy-pullback[yy], 0, 7)+16,xx);  
          }
        }
      }
    }//bot
       
    if(front==1)
    {      
      if(side==0)
      {
        //front-side to left-side
        for(yy=0; yy<8; yy++)
        {
          for(xx=0; xx<8; xx++)
          {
            clrvoxel(xx, checkConstrains(yy-oldpullback[yy], 0, 7), checkConstrains(7-LED_Old[yy], 0, 7));
            setvoxel(xx, checkConstrains(yy-pullback[yy], 0, 7), checkConstrains(7-folderaddr[yy], 0, 7));
          
            clrvoxel(7-xx, checkConstrains(yy-oldpullback[yy], 0, 7)+8, checkConstrains(7-LED_Old[yy], 0, 7));
            setvoxel(7-xx, checkConstrains(yy-pullback[yy], 0, 7)+8, checkConstrains(7-folderaddr[yy], 0, 7));
          
            clrvoxel(7-xx, checkConstrains(yy-oldpullback[yy], 0, 7)+16, checkConstrains(7-LED_Old[yy], 0, 7));
            setvoxel(7-xx, checkConstrains(yy-pullback[yy], 0, 7)+16, checkConstrains(7-folderaddr[yy], 0, 7));  
          }
        }
      }
      if(side==5)
      {
        // front-side to bottom
        for(yy=0; yy<8; yy++)
        {
          for(xx=0; xx<8; xx++)
          {
            clrvoxel(checkConstrains(yy-oldpullback[yy], 0, 7),xx, checkConstrains(7-LED_Old[yy], 0, 7));
            setvoxel(checkConstrains(yy-pullback[yy], 0, 7),xx, checkConstrains(7-folderaddr[yy], 0, 7));
          
            clrvoxel(7-checkConstrains(yy-oldpullback[yy], 0, 7),xx+8, checkConstrains(7-LED_Old[yy], 0, 7));
            setvoxel(7-checkConstrains(yy-pullback[yy], 0, 7),xx+8, checkConstrains(7-folderaddr[yy], 0, 7));
          
            clrvoxel(7-checkConstrains(yy-oldpullback[yy], 0, 7),xx+16, checkConstrains(7-LED_Old[yy], 0, 7));
            setvoxel(7-checkConstrains(yy-pullback[yy], 0, 7),xx+16, checkConstrains(7-folderaddr[yy], 0, 7));  
          }
        }
      }
      if(side==4)
      {
        // front-side to top-side
        for(yy=0; yy<8; yy++)
        {
          for(xx=0; xx<8; xx++)
          {
            clrvoxel(checkConstrains(yy+oldpullback[yy], 0, 7),xx, checkConstrains(7-checkConstrains(LED_Old[7-yy], 0, 7), 0, 7));
            setvoxel(checkConstrains(yy+pullback[yy], 0, 7),xx, checkConstrains(7-folderaddr[7-yy], 0, 7));
          
            clrvoxel(7-checkConstrains(yy+oldpullback[yy], 0, 7),xx+8, checkConstrains(7-checkConstrains(LED_Old[7-yy], 0, 7), 0, 7));
            setvoxel(7-checkConstrains(yy+pullback[yy], 0, 7),xx+8, checkConstrains(7-folderaddr[7-yy], 0, 7));
          
            clrvoxel(7-checkConstrains(yy+oldpullback[yy], 0, 7),xx+16, checkConstrains(7-checkConstrains(LED_Old[7-yy], 0, 7), 0, 7));
            setvoxel(7-checkConstrains(yy+pullback[yy], 0, 7),xx+16, checkConstrains(7-folderaddr[7-yy], 0, 7));  
          }
        }
      }
      if(side==1)
      {
      //front-side to right-side
        for(yy=0; yy<8; yy++)
        {
          for(xx=0; xx<8; xx++)
          {
            clrvoxel(xx, checkConstrains(yy+oldpullback[yy], 0, 7), checkConstrains(7-checkConstrains(LED_Old[7-yy], 0, 7), 0, 7));
            setvoxel(xx, checkConstrains(yy+pullback[yy], 0, 7), checkConstrains(7-folderaddr[7-yy], 0, 7));
          
            clrvoxel(7-xx, checkConstrains(yy+oldpullback[yy], 0, 7)+8, checkConstrains(7-checkConstrains(LED_Old[7-yy], 0, 7), 0, 7));
            setvoxel(7-xx, checkConstrains(yy+pullback[yy], 0, 7)+8, checkConstrains(7-folderaddr[7-yy], 0, 7));
          
            clrvoxel(7-xx, checkConstrains(yy+oldpullback[yy], 0, 7)+16, checkConstrains(7-checkConstrains(LED_Old[7-yy], 0, 7), 0, 7));
            setvoxel(7-xx, checkConstrains(yy+pullback[yy], 0, 7)+16, checkConstrains(7-folderaddr[7-yy], 0, 7));  
          }
        }
      }
    }//front 

  delay(30);//               DELAY   DELAY  DELAY
  for(xx=0; xx<8; xx++)
  {
  LED_Old[xx]=folderaddr[xx];
  oldpullback[xx]=pullback[xx];
  }  
 
  if(folderaddr[7]==7)
  {
    // pullback=8;
    for(zz=0; zz<8; zz++)
    pullback[zz] = pullback[zz]+1;
 
    if(pullback[7]==8)
    {//finished with fold
      delay(50);
     //state++;
     //if(state==4)
     //state=0;     
     ranselect= random(3);
     if(ranselect==0)
     {
     ranx=0;
     rany=random(1,16);
     ranz=random(1,16);
     }
     if(ranselect==1)
     {
     ranx=random(1,16);
     rany=0;
     ranz=random(1,16);
     }
     if(ranselect==2)
     {
     ranx=random(1,16);
     rany=random(1,16);
     ranz=0;
     }     
     
     side_select=random(3);
     
     if(top==1)
      {//                 TOP
      top=0; 
      if(side==0)
      {//top to left
      left=1;
      if(side_select==0) side=2;
      if(side_select==1) side=3;
      //if(side_select==2) side=4;
      if(side_select==2) side=5;
      } else    
      if(side==1)
      {//top to right
      right=1;
      if(side_select==0) side=5;
      if(side_select==1) side=2;
      if(side_select==2) side=3;
      //if(side_select==3) side=4;
      } else  
      if(side==2)
      {//top to back
        back=1;
        if(side_select==0) side=0;
        if(side_select==1) side=1;
        if(side_select==2) side=5;
        //if(side_select==3) side=4;
        } else      
      if(side==3)
      {//top to front
        front=1;
        if(side_select==0) side=0;
        if(side_select==1) side=1;
        if(side_select==2) side=5;
        //if(side_select==3) side=4;
        }   
      } else//top
    if(bot==1){//                 BOTTOM
     bot=0; 
     if(side==0){//bot to left
      left=1;
      if(side_select==0) side=2;
      if(side_select==1) side=3;
      if(side_select==2) side=4;
      //if(side_select==3) side=5;
    } else    
     if(side==1){//bot to right
      right=1;
      //if(side_select==0) side=5;
      if(side_select==0) side=2;
      if(side_select==1) side=3;
      if(side_select==2) side=4;} else  
     if(side==2){//bot to back
      back=1;
      if(side_select==0) side=0;
      if(side_select==1) side=1;
      //if(side_select==2) side=5;
      if(side_select==2) side=4;} else      
      if(side==3){//bot to front
      front=1;
      if(side_select==0) side=0;
      if(side_select==1) side=1;
      //if(side_select==2) side=5;
      if(side_select==2) side=4;}   
     } else//bot
    if(right==1){//                 RIGHT
     right=0; 
     if(side==4){//right to top
      top=1;
      if(side_select==0) side=2;
      if(side_select==1) side=3;
      if(side_select==2) side=0;
      //if(side_select==3) side=1;
    } else    
     if(side==5){//right to bot
      bot=1;
      if(side_select==0) side=0;
      if(side_select==1) side=2;
      if(side_select==2) side=3;
      //if(side_select==3) side=1;
    } 
      else  
     if(side==2){//right to back
      back=1;
      if(side_select==0) side=0;
      //if(side_select==1) side=1;
      if(side_select==1) side=5;
      if(side_select==2) side=4;} else      
      if(side==3){//right to front
      front=1;
      if(side_select==0) side=0;
      //if(side_select==1) side=1;
      if(side_select==1) side=5;
      if(side_select==2) side=4;}   
     } else//bot
      if(left==1){//                 LEFT
     left=0; 
     if(side==4){//left to top
      top=1;
      //if(side_select==0) side=2;
      if(side_select==0) side=3;
      if(side_select==1) side=2;
      if(side_select==2) side=1;} else    
      if(side==5){//left to bot
      bot=1;
      //if(side_select==0) side=0;
      if(side_select==0) side=2;
      if(side_select==1) side=3;
      if(side_select==2) side=1;} else  
      if(side==2){//left to back
      back=1;
      //if(side_select==0) side=0;
      if(side_select==0) side=1;
      if(side_select==1) side=5;
      if(side_select==2) side=4;} else      
      if(side==3){//left to front
      front=1;
      //if(side_select==0) side=0;
      if(side_select==0) side=1;
      if(side_select==1) side=5;
      if(side_select==2) side=4;}   
     } else//bot
      if(front==1){//                 front
      front=0; 
      if(side==4){//front to top
      top=1;
      if(side_select==0) side=2;
      //if(side_select==1) side=3;
      if(side_select==1) side=0;
      if(side_select==2) side=1;} else    
      if(side==5){//front to bot
      bot=1;
      if(side_select==0) side=0;
      if(side_select==1) side=2;
      //if(side_select==2) side=3;
      if(side_select==2) side=1;} else  
      if(side==0){//front to left
      left=1;
      if(side_select==0) side=2;
     // if(side_select==1) side=3;
      if(side_select==1) side=5;
      if(side_select==2) side=4;} else      
      if(side==1){//front to right
      right=1;
      if(side_select==0) side=2;
     // if(side_select==1) side=3;
      if(side_select==1) side=5;
      if(side_select==2) side=4;}   
      } else//bot
      if(back==1){//                 back
      back=0; 
      if(side==4){//back to top
      top=1;
      //if(side_select==0) side=2;
      if(side_select==0) side=3;
      if(side_select==1) side=0;
      if(side_select==2) side=1;} else    
      if(side==5){//back to bot
      bot=1;
      if(side_select==0) side=0;
      //if(side_select==1) side=2;
      if(side_select==1) side=3;
      if(side_select==2) side=1;} else  
      if(side==0){//back to left
      left=1;
      //if(side_select==0) side=2;
      if(side_select==0) side=3;
      if(side_select==1) side=5;
      if(side_select==2) side=4;} else      
      if(side==1){//back to right
      right=1;
      //if(side_select==0) side=2;
      if(side_select==0) side=3;
      if(side_select==1) side=5;
      if(side_select==2) side=4;}   
     } //bot  
   // for(yy=0; yy<8; yy++)
  //for(xx=0; xx<8; xx++)
  //LED(LED_Old[yy], xx, yy-oldpullback[yy], 0, 0, 0);
  for(xx=0; xx<8; xx++){
  oldpullback[xx]=0;
  pullback[xx]=0;}
 
  folderaddr[0]=-8;
  folderaddr[1]=-7;
  folderaddr[2]=-6;
  folderaddr[3]=-5;
  folderaddr[4]=-4;
  folderaddr[5]=-3;
  folderaddr[6]=-2;
  folderaddr[7]=-1;

    }//pullback==7
  }//folderaddr==7    

  if(folderaddr[7]!=7)
  for(zz=0; zz<8; zz++)
  folderaddr[zz] = folderaddr[zz]+1;
  
  }//while 
}//folder

void effect_rain(int iterations)
{
  int i, ii;
  int rnd_x;
  int rnd_y;
  int rnd_num;
  
  for (ii=0;ii<iterations;ii++)
  {
    rnd_num = rand()%8;
    
    for (i=0; i < rnd_num;i++)
    {
      rnd_x = rand()%8;
      rnd_y = rand()%24;
      setvoxel(rnd_x,rnd_y,7);
    }
    
    delay(50);
    shift(AXIS_Z,-1);
  }
}

void effect_wormsqueeze(int size, int axis, int direction, int iterations, int delayx)
{
  int x, y, i,j,k, dx, dy;
  int cube_sizex, cube_sizey;
  int origin = 0;
  
  if (direction == -1)
    origin = 7;
  
  cube_sizex = 8 - (size-1);
  cube_sizey = 24 - (size-1);
  
  
  x = rand()%cube_sizex;
  y = rand()%cube_sizey;
  
  for (i=0; i<iterations; i++)
  {
    dx = ((rand()%3)-1);
    dy = ((rand()%3)-1);
  
    if ((x+dx) >= 0 && (x+dx) < cube_sizex)
      x += dx;
      
    if ((y+dy) >= 0 && (y+dy) < cube_sizey)
      y += dy;
  
    shift(axis, direction);
    

    for (j=0; j<size;j++)
    {
      for (k=0; k<size;k++)
      {
        if (axis == AXIS_Z)
          setvoxel(x+j,y+k,origin);
          
        if (axis == AXIS_Y)
          setvoxel(x+j,origin,y+k);
          
        if (axis == AXIS_X)
          setvoxel(origin,y+j,x+k);
      }
    }
    
    delay(delayx);
  }
}

//CRAZY STRAW
void crazy_straw_mode(int delayx, int iterations){
  int8_t  i,j,p=0,pp=0,
          xStart=0,yStart=0,zStart=0,
          xEnd=0,yEnd=0,zEnd=0,
          xDest=0,yDest=0,zDest=0,
          xx,yy,zz;

  //start point & straw random generation
  if(rand()%2) xStart=0;
  else xStart=CUBE_XSIZE-1;
  
  if(rand()%2) yStart=0;
  else yStart=CUBE_YSIZE-1;
  
  if(rand()%2) zStart=0;
  else zStart=CUBE_ZSIZE-1;
  
  if(xStart>0) xEnd=0;
  else xEnd=CUBE_XSIZE-1;
  yEnd=yStart;
  zEnd=zStart;
  
  for(i=0;i<iterations;i++){
    while(1){
      xx=xEnd;
      yy=yEnd;
      zz=zEnd;
      
      do p=rand()%3;
      while(pp==p);
      pp=p;
      
      if(p==0){
        if(xEnd==0) xx=CUBE_XSIZE-1;
        else xx=0;
      }
      if(p==1){
        if(yEnd==0) yy=CUBE_YSIZE-1;
        else yy=0;
      }
      if(p==2){
        if(zEnd==0) zz=CUBE_ZSIZE-1;
        else zz=0;
      }
      
      if((xx==xStart && yy==yStart && zz==zStart) || (xx==xEnd && yy==yEnd && zz==zEnd)) continue;
      else{
        xDest=xx;
        yDest=yy;
        zDest=zz;
        break;
      } 
    }
    
    for(j=0;j<CUBE_YSIZE;j++){
      if(xDest!=xEnd){
        if(xEnd==0) xx=j/3;
        if(xEnd==CUBE_XSIZE-1) xx=xEnd-j/3;
      }
      
      if(yDest!=yEnd){
        if(yEnd==0) yy=j;
        if(yEnd==CUBE_YSIZE-1) yy=yEnd-j;
      }
      
      if(zDest!=zEnd){
        if(zEnd==0) zz=j/3;
        if(zEnd==CUBE_ZSIZE-1) zz=zEnd-j/3;
      }
      
      line_3d(xStart,yStart,zStart,xx,yy,zz);
      delay(delayx);
      clearfast();
      
      if(j==CUBE_YSIZE-1){
        xEnd=xStart;
        yEnd=yStart;
        zEnd=zStart;
        xStart=xx;
        yStart=yy;
        zStart=zz;
      }
    }
  }
}

void draw_positions_axis (byte axis, unsigned char positions[128], int invert)
{
  int x, y, z, p;
  fill(0x00);
  if (axis == AXIS_Y)
  { 
    for (x=0; x<24; x++)
    {
      for (y=0; y<8; y++)
      {
        if (invert)
        {
          p = (23-positions[(x*8)+y]);
        } else
        {       
          p = positions[(x*8)+y];
        }
        setvoxel(y,p,x);
      }
    }
  }
    
  else if (axis == AXIS_X)
     { 
    for (y=0; y<24; y++)
    {
      for (x=0; x<8; x++)
      {
        if (invert)
        {
          p = (7-positions[(y*8)+x]);
        } else
        {       
          p = positions[(y*8)+x];
        }
        setvoxel(p,y,x);
      }
    }
  }  
       
  else //if (axis == AXIS_Z)
  { 
    for (y=0; y<24; y++)
      {    
    for (x=0; x<8; x++)
    {

        if (invert)
        {
          p = (7-positions[(y*8)+x]);
        } else
        {       
          p = positions[(y*8)+x];
        }
        setvoxel(x,y,p);
      }
    }
  }  
}

void effect_boxside_randsend_parallel(byte axis, int origin, int delayx, int modex)
{
  int i;
  int done;
  unsigned char cubepos[192];
  unsigned char pos[192];
  int notdone = 1;
  int notdone2 = 1;
  int sent = 0;
  
  for (i=0;i<192;i++)
  {
    pos[i] = 0;
  }
  
  while (notdone)
  {
    if (modex == 1)
    {
      notdone2 = 1;
      while (notdone2 && sent<192)
      {
        i = rand()%192;
        if (pos[i] == 0)
        {
          sent++;
          pos[i] += 1;
          notdone2 = 0;
        }
      }
    } else if (modex == 2)
    {
      if (sent<192)
      {
        pos[sent] += 1;
        sent++;
      }
    }
    
    done = 0;
    for (i=0;i<192;i++)
    {
    if (axis == AXIS_Y)
    {
      if (pos[i] > 0 && pos[i] <23)
      {
        pos[i] += 1;
      }
        
      if (pos[i] == 23)
        done++;
    }
    else
    {
      if (pos[i] > 0 && pos[i] <7)
      {
        pos[i] += 1;
      }
        
      if (pos[i] == 7)
        done++; 
    }
    }
    
    if (done == 192)
      notdone = 0;
    
    for (i=0;i<192;i++)
    {
      if (origin == 0)
      {
        cubepos[i] = pos[i];
      } else
      {
        if (axis == AXIS_Y)
        cubepos[i] = (23-pos[i]);
        else
        cubepos[i] = (7-pos[i]);
      }
    }      
    draw_positions_axis(axis,cubepos,0);
    delay(delayx);
  } 
}

void effect_axis_updown_randsuspend (byte axis, int delayx, int sleep, int invert)
{
  unsigned char positions[192];
  unsigned char destinations[192];

  int i,px;
 if ((axis==AXIS_X) || (axis==AXIS_Z))
 {
    // Set 128 random positions
  for (i=0; i<192; i++)
  {
    positions[i] = 0; // Set all starting positions to 0
    destinations[i] = rand()%8;
  }

    // Loop 8 times to allow destination 7 to reach all the way
  for (i=0; i<8; i++)
  {
        // For every iteration, move all position one step closer to their destination
    for (px=0; px<192; px++)
    {
      if (positions[px]<destinations[px])
      {
        positions[px]++;
      }
    }
        // Draw the positions and take a nap
    draw_positions_axis (axis, positions,invert);
    delay(delayx);
  }
  
    // Set all destinations to 7 (opposite from the side they started out)
  for (i=0; i<192; i++)
  {
    destinations[i] = 7;
  }
  
    // Suspend the positions in mid-air for a while
  delay(sleep);
  
    // Then do the same thing one more time
  for (i=0; i<8; i++)
  {
    for (px=0; px<192; px++)
    {
      if (positions[px]<destinations[px])
      {
        positions[px]++;
      }
      if (positions[px]>destinations[px])
      {
        positions[px]--;
      }
    }
    draw_positions_axis (axis, positions,invert);
    delay(delayx);
  }
}
else
{
  // Set 64 random positions
  for (i=0; i<64; i++)
  {
    positions[i] = 0; // Set all starting positions to 0
    destinations[i] = rand()%24;
  }

    // Loop 8 times to allow destination 7 to reach all the way
  for (i=0; i<24; i++)
  {
        // For every iteration, move all position one step closer to their destination
    for (px=0; px<64; px++)
    {
      if (positions[px]<destinations[px])
      {
        positions[px]++;
      }
    }
        // Draw the positions and take a nap
    draw_positions_axis (axis, positions,invert);
    delay(delayx/2);
  }
  
    // Set all destinations to 7 (opposite from the side they started out)
  for (i=0; i<64; i++)
  {
    destinations[i] = 23;
  }
  
    // Suspend the positions in mid-air for a while
  delay(sleep);
  
    // Then do the same thing one more time
  for (i=0; i<23; i++)
  {
    for (px=0; px<64; px++)
    {
      if (positions[px]<destinations[px])
      {
        positions[px]++;
      }
      if (positions[px]>destinations[px])
      {
        positions[px]--;
      }
    }
    draw_positions_axis (axis, positions,invert);
    delay(delayx/2);
  }
}
}

void oscillation(int interactions)
{
  uint8_t curZ = 0, dz = 1;
  clearfast();
  for (int i=0; i<interactions; i++)
  {
      for (uint8_t x = 7; x > 0; x--)
      copyPlane(AXIS_X, x - 1, x);
      clrplane(AXIS_X, 0);      

      line_3d(0, 0, curZ, 0, 23, 7 - curZ);
      curZ += dz;
      if (curZ == 8 || curZ == 0) dz = -dz;
      delay(20);
  }
}


void shrinkCube_random(int interactions)
// Shrink a cube from largest to smallest in the corner. 
// Random corners picked for each turn
{
  const uint16_t delayx = 40;
  uint8_t cur = 0;
  uint8_t x, y, z, dx, dy, dz;
  int8_t corners[8][6] = // corner coordinates and delta multipliers for size 
  {
    { 0, 0, 0 , 1, 1, 1 },
    { 0, 23, 0 , 1, -1, 1 },
    { 7, 0, 0, -1, 1, 1 },
    { 0, 0, 7, 1, 1, -1 },
    { 7, 23, 0, -1, -1, 1 },
    { 0, 23, 7, 1, -1, -1 },
    { 7, 0, 7, -1, 1, -1 },
    { 7, 23, 7, -1, -1, -1 }
  };  
  clearfast();

  // Grown the cube from max size to min size then back to max,
  // selected another corner and repeat

  for (int i=0; i<interactions; i++)
  {
    x = corners[cur][0];
    y = corners[cur][1];
    z = corners[cur][2];
    // growth phase
    for (uint8_t i = 1; i < 24; i++)
    {
      dx = (i * corners[cur][3])/3;
      dy = i * corners[cur][4];
      dz = (i * corners[cur][5])/3;
      drawRPrism(x, y, z, dx, dy, dz);
      delay(delayx);
      clrRPrism(x, y, z, dx, dy, dz);
    }

    // pick another corner
    cur = random(ARRAY_SIZE(corners));
    x = corners[cur][0];
    y = corners[cur][1];
    z = corners[cur][2];

    // shrink down to this new corner
    for (uint8_t i = 23; i >= 1; i--)
    {
      dx = (i * corners[cur][3])/3;
      dy = i * corners[cur][4];
      dz = (i * corners[cur][5])/3;
      drawRPrism(x, y, z, dx, dy, dz);
      delay(delayx);
      clrRPrism(x, y, z, dx, dy, dz);
    }
  }
}

void EffectScrollingSine()
{
  uint8_t i, k;
  float stepx = 0;

  for (k = 0; k < 16; k++)
  {
    for (i = 0; i < 64; i++)
    {
      stepx += PI / 14;
      setvoxel(1, 0, 4 + 3.5 * sin(stepx));
      setvoxel(1, 15, 4 + 3.5 * sin(stepx));
      setvoxel(1, 23, 4 + 3.5 * sin(stepx));
      delay(25);
      TranslateScroll8(AXIS_Z, -1);
      EraseLine3D(1, 0, 0, 1, 0, 7);
    }    
  }  
}


void CornerToCorner()
{   
  clearfast();

  unsigned long STEP_TIME = 500;
  unsigned long timeSinceLastStep = 0;
  int whatstep = 0;
  boolean growing = true;
  boolean xHigh = random(2);
  boolean yHigh = random(2);
  boolean zHigh = random(2);   
  timeStart = millis();
  while (millis() - timeStart <= DEMO_RUNTIME)
  {
    unsigned long dt = 1;

    timeSinceLastStep += dt;

  if (timeSinceLastStep > STEP_TIME)
  {
    if(growing)
    {
      for(byte x = 0 ; x < 8 ; x++)
      {
        for(byte y = 0 ; y < 24 ; y++)
        {
          for(byte z = 0 ; z < 8 ; z++)
          {
            int xDist = x;
            if(xHigh)
            {
              xDist = 7 -x;
            }
            int yDist = y;
            if(yHigh)
            {
              yDist = 23 -y;
            }
            int zDist = z;
            if(zHigh)
            {
              zDist = 7 -z;
            }
            if(((xDist + yDist + zDist) <= whatstep))
            setvoxel(x, y, z);
            else
            clrvoxel(x, y, z);
          }
        }
      }
      whatstep++;
      if(whatstep > 21)
      {
        whatstep = 0;
        growing = false;
      }
    }
    else
    {
      for(byte x = 0 ; x < 8 ; x++)
      {
        for(byte y = 0 ; y < 24 ; y++)
        {
          for(byte z = 0 ; z < 8 ; z++)
          {
            int xDist = x;
            if(xHigh)
            {
              xDist = 7 -x;
            }
            int yDist = y;
            if(yHigh)
            {
              yDist = 23 -y;
            }
            int zDist = z;
            if(zHigh)
            {
              zDist = 7 -z;
            }
            if ((!(xDist + yDist + zDist) <= whatstep))
         
            setvoxel(x, y, z);
            else
            clrvoxel(x, y, z);            
          }
        }
      }
      whatstep++;
      if(whatstep > 21)
      {
        whatstep = 0;
        growing = true;
        xHigh = random(2);
        yHigh = random(2);
        zHigh = random(2);
      }
    }
    timeSinceLastStep -= STEP_TIME;
    delay(15);
  }
  }
}

void flagwave()
{
  uint8_t curY = 0, dy = 1;  
  clearfast();
  timeStart = millis();
  while (millis() - timeStart <= DEMO_RUNTIME)
  {
    // shift all the planes back by one
    for (uint8_t x = 7; x > 0; x--)
    copyPlane(AXIS_X, x - 1, x);
    clrplane(AXIS_X, 0);
    line_3d(0, curY, 0, 0, curY, 7);
    curY += dy;
    if (curY == (CUBE_YSIZE-1) || curY == 0) dy = -dy;
    delay(50);
  }
}


void fireworks (int iterations, int n, int delayx)
{
  clearfast();
  int i,f,e;
  float origin_x = 3.5;
  float origin_y = 11;
  float origin_z = 3.5;
  int rand_y, rand_x, rand_z;
  float slowrate, gravity;
  // Particles and their position, x,y,z and their movement, dx, dy, dz
  float particles[n][6];
  for (i=0; i<iterations; i++)
  {
    origin_x = rand()%4;
    origin_y = rand()%20;
    origin_z = rand()%2;
    origin_z +=5;
    origin_x +=2;
    origin_y +=2;
    // shoot a particle up in the air
    for (e=0;e<origin_z;e++)
    {
      setvoxel(origin_x,origin_y,e);
      delay(10+20*e);
      clearfast();
    }
    // Fill particle array
    for (f=0; f<n; f++)
    {
      // Position
      particles[f][0] = origin_x;
      particles[f][1] = origin_y;
      particles[f][2] = origin_z;      
      rand_x = rand()%200;
      rand_y = rand()%200;
      rand_z = rand()%200;
      // Movement
      particles[f][3] = 1-(float)rand_x/100; // dx
      particles[f][4] = 1-(float)rand_y/100; // dy
      particles[f][5] = 1-(float)rand_z/100; // dz
    }
    // explode
    for (e=0; e<25; e++)
    {
      slowrate = 1+tan((e+0.1)/20)*10;      
      gravity = tan((e+0.1)/20)/2;
      for (f=0; f<n; f++)
      {
        particles[f][0] += particles[f][3]/slowrate;
        particles[f][1] += particles[f][4]/slowrate;
        particles[f][2] += particles[f][5]/slowrate;
        particles[f][2] -= gravity;
        setvoxel(particles[f][0],particles[f][1],particles[f][2]);
      }
      delay(delayx);
      clearfast();
    }
  }
}


void TransitionShift(byte axis, const type_polarity polarity, const uint8_t delayms)
{
  uint8_t x;

  for (x = 0; x < 8; x++)
  {
    shift(axis, ((polarity == POSITIVE) << 1) - 1);
    delay(delayms);
  }
}

void TransitionScroll(byte axis, const type_polarity polarity, const uint8_t delayx)
{
  uint8_t x;

  for (x = 0; x < 7; x++)
  {
    TranslateScroll8(axis, ((polarity == POSITIVE) << 1) - 1);
    delay(delayx);
  }
}

unsigned char bitswap( unsigned char aByte ) 
{ 
return (aByte & 0x80 ? 0x01 : 0) |
       (aByte & 0x40 ? 0x02 : 0) |
       (aByte & 0x20 ? 0x04 : 0) |
       (aByte & 0x10 ? 0x08 : 0) |
       (aByte & 0x08 ? 0x10 : 0) |
       (aByte & 0x04 ? 0x20 : 0) |
       (aByte & 0x02 ? 0x40 : 0) |
       (aByte & 0x01 ? 0x80 : 0);   
}

void font_getchar (char chr, unsigned char dst[8])
{
  int i;
  chr -= 32;
    
    for (i = 0; i < 8; i++)
    {
    dst[i] = font[(chr*8)+i];
    }
  
}

void effect_rand_patharound(int iterations, int delayx)
{
  int z, dz, i;
  z = 4;
  unsigned char path[60];
  
  font_getpath(0,path);
  
  for (i = 0; i < iterations; i++)
  {
    dz = ((rand()%3)-1);
    z += dz;
    
    if (z>7)
      z = 7;
      
    if (z<0)
      z = 0;
    
    effect_pathmove(path,60);
    setvoxel(((path[0]>>5) & 0x07), (path[0] & 0x1f),z);
    delay(delayx);
  }
}

void effect_pathmove(unsigned char *path, int length)
{
  int i,z;
  unsigned char state;
  
  for (i=(length-1);i>=1;i--)
  {
    for (z=0;z<8;z++)
    {
    
      state = getvoxel(((path[(i-1)]>>5) & 0x07), (path[(i-1)] & 0x1f), z);
      altervoxel(((path[i]>>5) & 0x07), (path[i] & 0x1f), z, state);
    }
  }
  for (i=0;i<8;i++)
    clrvoxel(((path[0]>>5) & 0x07), (path[0] & 0x1f),i);
}


void font_getpath(unsigned char path, unsigned char *destination)
{
  int i;
  int offset = 0;
  
  if (path == 1)
    offset=sizeof(paths);
  
  for (i = 0; i < sizeof(paths); i++)
    destination[i] = pgm_read_byte(&paths[i+offset]);
}


void font_getEllipse(unsigned char path, unsigned char *destination)
{
  int i;
  int offset = 0;
  
  if (path == 1)
    offset=sizeof(ellipse);
  
  for (i = 0; i < sizeof(ellipse); i++)
    destination[i] = pgm_read_byte(&ellipse[i+offset]);
}

void effect_allshape_text (int delayx, char *str, int mode)
{
  int z, i,ii;
  z = 4;
  unsigned char Paths[60], Ellipse[38];
  
  font_getpath(0, Paths);
  font_getEllipse(0, Ellipse);
  
  unsigned char chr[8];
  unsigned char stripe;
  
  switch (mode)
  {
    case 0:
  
      while (*str)
        {
            font_getchar(*str++, chr);
            
            for (ii=0;ii<8;ii++)
            {
              stripe = bitswap(chr[ii]);
              for (z=0;z<8;z++)
              {
                if ((stripe>>(7-z)) & 0x01)
                {
                  setvoxel(((Paths[0]>>5) & 0x07), (Paths[0] & 0x1f),z);
                } else
                {
                  clrvoxel(((Paths[0]>>5) & 0x07), (Paths[0] & 0x1f),z);
                }
                
              }
              effect_pathmove(Paths,60);
              delay(delayx);
            }
          
            effect_pathmove(Paths,60);
            delay(delayx);
          }
          for (i=0;i<60;i++)
          {
            effect_pathmove(Paths,60);
            delay(delayx);
          }
          break;
          
    case 1:
  
          while (*str)
          {
            font_getchar(*str++, chr);
            
            for (ii=0;ii<8;ii++)
            {
              stripe = bitswap(chr[ii]);
              
              for (z=0;z<8;z++)
              {
                if ((stripe>>(7-z)) & 0x01)
                {
                  setvoxel(((Ellipse[0]>>5) & 0x07), (Ellipse[0] & 0x1f),z);
                } else
                {
                  clrvoxel(((Ellipse[0]>>5) & 0x07), (Ellipse[0] & 0x1f),z);
                }
                
              }
              effect_pathmove(Ellipse,38);
              delay(delayx);
            }
          
            effect_pathmove(Ellipse,38);
            delay(delayx);
          }
          for (i=0;i<38;i++)
          {
            effect_pathmove(Ellipse,38);
            delay(delayx);
          }
          break;        
  }
}

void effect_outer_wall(int iterations, int delayx)
{
  unsigned char path[60];
  
  font_getpath(0,path);
  
  for (int inter = 0; inter < iterations; inter++)
  {
    for (byte z=0; z<CUBE_ZSIZE; z++)
      {
        for (byte i=0;i<60;i++)
        { 
          setvoxel(((path[i]>>5) & 0x07), (path[i] & 0x1f),z);
          delay(delayx);
        }
      }
    delay(delayx);
    for (byte z=0; z<CUBE_ZSIZE; z++)
      {
        for (byte i=0;i<60;i++)
        {        
          clrvoxel(((path[i]>>5) & 0x07), (path[i] & 0x1f),7-z);
          delay(delayx);
        }
    }      
  }   
}
