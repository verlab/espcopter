boolean isMouseTracking=false;

float x = 0;
float y = 0;

int joyOutputRange = 10;  //Maximum value for full horiz or vert position where centered is 0.

float curJoyDisplayWidth;
float curJoyDisplayHeight;

float maxJoyRange = 200;     //Maximum joystick range
float curJoyAngle;     //Current joystick angle
float curJoyRange;     //Current joystick range
float joyDisplayCenterX;  //Joystick displayed Center X
float joyDisplayCenterY;  //Joystick displayed Center Y

Textlabel joystickLabel;

int windowX, windowY;

int getCenterLocation;
int getCenterX;
int getCenterY;

int isMouseTrackingFirst =1;
void setupJoystick(int centerx, int centery)
{
  joyDisplayCenterX = centerx;
  joyDisplayCenterY = centery;
  curJoyDisplayWidth = maxJoyRange * .85;
  curJoyDisplayHeight = curJoyDisplayWidth;
  maxJoyRange = curJoyDisplayWidth / 2;

  joystickLabel = cp5.addTextlabel("label")
    .setText("Joystick")
      .setPosition(joyDisplayCenterX - 25, joyDisplayCenterY + maxJoyRange + 5)
        .setFont(font)
          ;
}

void joystickDraw()
{

  float dx = mouseX - joyDisplayCenterX;
  float dy = mouseY - joyDisplayCenterY;

  if ((mouseButton == LEFT) && dist(mouseX, mouseY, joyDisplayCenterX, joyDisplayCenterY) <= 50 ) {
    isMouseTracking = true;
    x_axis = 0;
    y_axis = 0;
  } 
  else {
   if ((mouseButton == RIGHT))            
   isMouseTracking = false;
  }

  if (isMouseTracking == true)
  {   
    curJoyAngle = atan2(dy, dx);
    curJoyRange = dist(mouseX, mouseY, joyDisplayCenterX, joyDisplayCenterY);
   
    MouseInfo.getPointerInfo();
    Point pt = MouseInfo.getPointerInfo().getLocation();
    int absMouseX = (int)pt.getX();
    int absMouseY = (int)pt.getY();

    // println("absMouseX = " + absMouseX );
    if(getCenterLocation == 1){
      getCenterX=   absMouseX;
      getCenterY=   absMouseY;
      getCenterLocation =0;
    }
    //absMouseX = constrain(absMouseX ,round(getCenterX)-100,round(getCenterX)+100);
  //  absMouseY = constrain(absMouseY ,round(getCenterY)-100,round(getCenterY)+100);
    
    absMouseX = constrain(absMouseX ,820-100,820+100);
    absMouseY = constrain(absMouseY ,730-100,730+100);
    
     
    if(isMouseTrackingFirst < 10){
      absMouseX = 820;
      absMouseY =730;
     isMouseTrackingFirst++;
    }
    
    rbt.mouseMove(absMouseX, absMouseY);
    
    
    
  
  }else {
    getCenterLocation =1;
    curJoyRange = 0;
    isMouseTrackingFirst=1;
  }

  noStroke();
  fill(2, 52, 77);
  ellipse(joyDisplayCenterX, joyDisplayCenterY, curJoyDisplayHeight, curJoyDisplayWidth);

  joystickButton(joyDisplayCenterX, joyDisplayCenterY, curJoyAngle);
  y = (joyOutputRange*(cos(curJoyAngle) * curJoyRange)/ maxJoyRange);
  x = (joyOutputRange*(-(sin(curJoyAngle) * curJoyRange)/maxJoyRange));
     
}

void joystickButton(float x, float y, float a)
{
    
  pushMatrix();
  translate(x-x_axis, y+y_axis);
  rotate(a);

  if (curJoyRange > maxJoyRange)
    curJoyRange = maxJoyRange;

  if (isMouseTracking) {
    fill(0, 180, 234);
  } 
  else {
    fill(1, 108, 158);
  }
  ellipse(curJoyRange * 0.5, 0, maxJoyRange * 1.5, maxJoyRange * 1.5);

  popMatrix();
}
void mouseWheel(MouseEvent event) {
  int e = event.getCount();
  if(e == 1 && throttleValue > 0 ){
  knobTurretRotation.setValue(throttleValue = throttleValue - 5);
  }if(e == -1 && throttleValue < 100) {
  knobTurretRotation.setValue(throttleValue = throttleValue + 5);
  }
    
    
}