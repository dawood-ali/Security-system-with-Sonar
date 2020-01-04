#include <Servo.h>
#include <UTFTGLUE.h>
#include <Adafruit_GFX.h>
#include <TouchScreen.h>  //touchscreen library
#include <MCUFRIEND_kbv.h> //touchscreen specific library because it detects
//the kind of shield you are using which is really important

#define LCD_CS A3 //chip select is set to A3
#define LCD_CD A2 //command data goes to A2
#define LCD_WR A1 //writing to LCD is set to A1
#define LCD_RD A0 //lcd reading is set to A0
#define LCD_RST A4 //LCD resetting the screen.
#define trigPin 41
#define echoPin 37

#define sensor 25
#define LED 53
MCUFRIEND_kbv tft{LCD_CS,LCD_CD,LCD_WR,LCD_RD,LCD_RST};

#define BLACK 0x0000
#define WHITE 0xFFFF
#define BLUE 0x001F
#define RED 0xF800
#define GREEN 0x07E0
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define YELLOW 0xFFE0

#define YP A3 //input in the y
#define XM A2 // input in the x
#define YM 9 //output digital
#define XP 8 //output digital

uint16_t r = 11;
uint16_t g = 170;
uint16_t b = 6;
uint16_t color_lines= tft.color565(r,g,b);
int i = 25;
int angle = 155;

float x = 10;
float y = 10;

int max_distance = 10;


void point_map(float x, float y) {
  uint16_t pixel_x= 239+((x/double(max_distance))*double(240));
  uint16_t pixel_y= 320-((y/double(max_distance))*double(320));

  for(int i=255; i>=0; i--) {
    uint16_t color_dot= tft.color565(i,0,0);
    tft.fillCircle(pixel_x,pixel_y,4,color_dot);
  }
  }


Servo myservo;

void setup() {

  Serial.begin (9600);
  pinMode(LED, OUTPUT);
  pinMode(sensor, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  myservo.attach(31);

    tft.reset();

    uint16_t ID=tft.readID();

    tft.begin(ID);
    tft.setRotation(1);
    tft.fillScreen(BLACK);

    while(i<=355) {
      tft.drawCircle(239,320,i,color_lines);
      i+=40;
    }
    i=25;
    tft.drawLine(239,320,239,0,color_lines);
}

void loop() {

  while (digitalRead(sensor) == HIGH) {
    digitalWrite(LED, HIGH);
    float duration, distance;
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);

    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    duration = pulseIn(echoPin, HIGH);
    distance = (duration / 2) * 0.000344;

      while(i<=355) {
        tft.drawCircle(239,320,i,color_lines);
        i+=40;
      }
      i=25;
      tft.drawLine(239,320,239,0,color_lines);
    
    
    if (angle > 0) {
      myservo.write(angle);

      x = distance * cos(double(angle) * double(PI / 180));
      y = distance * sin(double(angle) * double(PI / 180));

      point_map(x,y);

      angle=angle-10;
      Serial.println(angle);

    }
    if (angle <= 0 && angle >= -155) {
      myservo.write(-angle);

      x = distance * cos(double(-angle) * double(PI / 180));
      y = distance * sin(double(-angle) * double(PI / 180));

      point_map(x,y);

      angle=angle-10;
      Serial.println(angle);
    }

    if (angle == -155) {
      angle = -angle;
    }

    delay(5);

  }
  digitalWrite(LED,LOW);
  

}
