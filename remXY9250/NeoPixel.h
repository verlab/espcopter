#include <Adafruit_NeoPixel.h>

#define PIXELSPIN   2
#define NUMPIXELS   12
#define CALIBRATIONTIME 20000

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIXELSPIN, NEO_GRB + NEO_KHZ800);

unsigned long pixelsInterval=50;  // the time we need to wait
unsigned long colorWipePreviousMillis=0;
unsigned long theaterChasePreviousMillis=0;
unsigned long theaterChaseRainbowPreviousMillis=0;
unsigned long rainbowPreviousMillis=0;
unsigned long rainbowCyclesPreviousMillis=0;

int theaterChaseQ = 0;
int theaterChaseRainbowQ = 0;
int theaterChaseRainbowCycles = 0;
int rainbowCycles = 0;
int rainbowCycleCycles = 0;

uint16_t currentPixel = 0;// what pixel are we operating on

void NeoPixelsetup() {
  currentPixel = 0;
  
  pixels.begin(); // This initializes the NeoPixel library.
  pixels.show(); // This sends the updated pixel color to the hardware.
      
}




uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return pixels.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return pixels.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return pixels.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c){
  pixels.setPixelColor(currentPixel,c);
  pixels.show();
  currentPixel++;
  if(currentPixel == NUMPIXELS){
    currentPixel = 0;
  }
}

void rainbow() {
  for(uint16_t i=0; i<pixels.numPixels(); i++) {
    pixels.setPixelColor(i, Wheel((i+rainbowCycles) & 255));
  }
  pixels.show();
  rainbowCycles++;
  if(rainbowCycles >= 256) rainbowCycles = 0;
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle() {
  uint16_t i;

  //for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< pixels.numPixels(); i++) {
      pixels.setPixelColor(i, Wheel(((i * 256 / pixels.numPixels()) + rainbowCycleCycles) & 255));
    }
    pixels.show();

  rainbowCycleCycles++;
  if(rainbowCycleCycles >= 256*5) rainbowCycleCycles = 0;
}

//Theatre-style crawling lights.
void theaterChase(uint32_t c) {
  for (int i=0; i < pixels.numPixels(); i=i+3) {
      pixels.setPixelColor(i+theaterChaseQ, c);    //turn every third pixel on
    }
    pixels.show();
    for (int i=0; i < pixels.numPixels(); i=i+3) {
      pixels.setPixelColor(i+theaterChaseQ, 0);        //turn every third pixel off
    }
    theaterChaseQ++;
    if(theaterChaseQ >= 3) theaterChaseQ = 0;
}


//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow() {
  for (int i=0; i < pixels.numPixels(); i=i+3) {
    pixels.setPixelColor(i+theaterChaseRainbowQ, Wheel( (i+theaterChaseRainbowCycles) % 255));    //turn every third pixel on
  }
      
  pixels.show();
  for (int i=0; i < pixels.numPixels(); i=i+3) {
    pixels.setPixelColor(i+theaterChaseRainbowQ, 0);        //turn every third pixel off        
  }      
  theaterChaseRainbowQ++;
  theaterChaseRainbowCycles++;
  if(theaterChaseRainbowQ >= 3) theaterChaseRainbowQ = 0;
  if(theaterChaseRainbowCycles >= 256) theaterChaseRainbowCycles = 0;
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.

void neoPixel() {
          
   if ((unsigned long)(millis() - rainbowPreviousMillis) >= pixelsInterval) {
       rainbowPreviousMillis = millis();
       rainbow();
    }
            
}
