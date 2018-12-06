
ControlP5 cp5;

CheckBox chkStatus;

Toggle tgl1, tgl2, tgl3, tgl4, tgl12, tgl20, tgl21, tgl22,tgl23, tgl5;
Slider rudder,aileronTrim, elevatorTrim; 
Textlabel lblConsole;
Textarea txtConsole;
Println console;
Chart myChart;
Slider2D sslider;
Knob knobTurretRotation;
DropdownList d1;
  PFont pfont;
  ControlFont font;
  
int clorWheelweight = 30;
int clorWheelheight = 525;//525

int xDegree = 500;
int yDegree = 30;

PImage img; 
PShape quad;

void setupUI() {

   pfont = loadFont("Standard0757-48.vlw");  // use true/false for smooth/no-smooth
   font = new ControlFont(pfont, 10);
   img = loadImage("drone5.jpg"); 
   quad = loadShape("Design3kart.obj");
  smooth();
  noStroke();


  // TODO, agregar area de radar, toggle de on off, joistick, toggle autonomo, toggle de sonido, toggle para luz frontal
  cp5 = new ControlP5(this);

sslider = cp5.addSlider2D("wave")
         .setPosition(980+xDegree,490+yDegree)
         .setSize(200,200)
         .setMinMax(0,0,100,100)
         .setValue(0,0)
         //.disableCrosshair()
         ;

      d1 = cp5.addDropdownList("myList-d1")
          .setPosition(1550, 270)
           .setSize(150,200)
          ;
           d1.setBackgroundColor(color(190));
           d1.setItemHeight(20);
           d1.setBarHeight(15);
           d1.getCaptionLabel().set("dropdown");
        //  customize(d1); 
 
      cp5.addButton("TakeCoordinat")
     .setValue(0)
     .setPosition(980+xDegree,715+yDegree)
     .setSize(200,19)
     ;
 
     cp5.addSlider("Time")
     .setSize(20,245)
     .setPosition(930+xDegree,475+yDegree)
     .setRange(1000,10000) // values can range from big to small as well
     .setNumberOfTickMarks(10)
     .setSliderMode(Slider.FLEXIBLE)
     .setValue(4)
     ;
     
     cp5.addSlider("Atitute")
     .setSize(20,245)
     .setPosition(855+xDegree,475+yDegree)
     .setRange(0,30) // values can range from big to small as well
     .setNumberOfTickMarks(31)
     .setSliderMode(Slider.FLEXIBLE)
     .setValue(4)
     ;
  // On/Off
          
      tgl1 = cp5.addToggle("joystick")
     .setPosition(770+xDegree,500+yDegree)
     .setSize(50,20)
     .setValue(true)
     .setMode(ControlP5.SWITCH)
     ;

  // Autonomous
  tgl2 = cp5.addToggle("tglAuto")
    .setCaptionLabel("AUTONOMOUS")
      .setPosition(770+xDegree, 600+yDegree)
        .setSize(60, 30)
          .setValue(false);

  // Light
 tgl5 = cp5.addToggle("tglLight")
   .setCaptionLabel("LIGHT")
     .setPosition(770+xDegree, 700+yDegree)
       .setSize(60, 30)
         .setValue(false);
         
          tgl3 = cp5.addToggle("Connect")
   .setCaptionLabel("Connect")
     .setPosition(220+xDegree, yDegree)
       .setSize(60, 30)
         .setValue(false);
 
 
  knobTurretRotation = cp5.addKnob("Motor Power")
    .setRange(0, 100)
      .setValue(0)
        .setPosition(450+xDegree, 490+yDegree)
          .setRadius(60)
            .setNumberOfTickMarks(50)
              .setTickMarkLength(7)
                .snapToTickMarks(true)
                // .setColorForeground(color(255))
                // .setColorBackground(color(0, 160, 100))
                // .setColorActive(color(255,255,0))
                .setDragDirection(Knob.HORIZONTAL);
            
            
       //ColorWheel    
     cp5.addColorWheel("colorWheel", clorWheelweight , clorWheelheight , 225 ).setRGB(color(128,0,255));
      noStroke();
     
     
    rudder= cp5.addSlider("slider")
     .setPosition(415+xDegree,640+yDegree)
     .setSize(200,20)
     .setRange(-30,30)
     .setValue(0)
     .setSliderMode(Slider.FLEXIBLE)
     ;
     
  
    aileronTrim = cp5.addSlider("sliderTicks1")
      .setSize(20,200)
     .setPosition(60+xDegree,475+yDegree)
     .setRange(-50,50) // values can range from big to small as well
     .setNumberOfTickMarks(101)
     .setSliderMode(Slider.FLEXIBLE)
     .setValue(-4)
     ;
     
     
     elevatorTrim= cp5.addSlider("sliderTicks2")
      .setSize(200,22)
     .setPosition(117+xDegree,700+yDegree)
     .setRange(-50,50) // values can range from big to small as well
     .setNumberOfTickMarks(101)
     .setSliderMode(Slider.FLEXIBLE)
     .setValue(10)
     ;
     
      elevatorTrim= cp5.addSlider("BoundRate")
      .setSize(200,22)
     .setPosition(200,440)
     .setRange(500,0) // values can range from big to small as well
     .setValue(500)
     .setNumberOfTickMarks(11)
     .setSliderMode(Slider.FLEXIBLE)
     ;
     tgl12 = cp5.addToggle("Take__Data")
     .setPosition(40,440)
     .setSize(50,20)
     .setValue(true)
     .setMode(ControlP5.SWITCH)
     ;
     
      tgl20 = cp5.addToggle("ARM__DISARM")
     .setPosition(850,525)
     .setSize(50,20)
     .setValue(false)
     .setMode(ControlP5.SWITCH)
     ;
     
      tgl21 = cp5.addToggle("Mode_1")
     .setPosition(850,575)
     .setSize(50,20)
     .setValue(false)
     .setMode(ControlP5.SWITCH)
     ;
     
      tgl22 = cp5.addToggle("Mode_2")
     .setPosition(850,625)
     .setSize(50,20)
     .setValue(false)
     .setMode(ControlP5.SWITCH)
     ;
    
      tgl23 = cp5.addToggle("Mode_3")
     .setPosition(850,675)
     .setSize(50,20)
     .setValue(false)
     .setMode(ControlP5.SWITCH)
     ;
     
      /*    
     cp5.addSlider("mode")
      .setSize(15,100)
     .setPosition(367+xDegree,540+yDegree)
     .setRange(0,5) // values can range from big to small as well
     .setNumberOfTickMarks(6)
     //.setSliderMode(Slider.FLEXIBLE)
     .setValue(0)
     ; */
          
     
     cp5.addSlider("sliderTicks3")
      .setSize(20,200)
     .setPosition(680+xDegree,475+yDegree)
     .setRange(-30,30) // values can range from big to small as well
     .setNumberOfTickMarks(31)
     .setSliderMode(Slider.FLEXIBLE)
     .setValue(4)
     ;
     
     
     cp5.addSlider("sliderTicks4")
      .setSize(200,22)
     .setPosition(415+xDegree,700+yDegree)
     .setRange(-30,30) // values can range from big to small as well
     .setValue(2)
     .setNumberOfTickMarks(31)
     .setSliderMode(Slider.FLEXIBLE)
     ;
     
     myChart = cp5.addChart("dataflow")
               .setPosition(-200+xDegree, yDegree+500)
               .setSize(200, 100)
               .setRange(-20, 20)
               .setView(Chart.LINE) // use Chart.LINE, Chart.PIE, Chart.AREA, Chart.BAR_CENTERED
               .setStrokeWeight(1.5)
               .setColorCaptionLabel(color(40))
               ;
               
     cp5.addTextfield("textValue")
     .setPosition(-200+xDegree,630+yDegree)
     .setSize(200,40)
     .setFont(createFont("arial",20))
     .setAutoClear(false)
     ;
     
     cp5.addBang("clear")
     .setPosition(-200+xDegree,700+yDegree)
     .setSize(200,30)
     .getCaptionLabel().align(ControlP5.CENTER, ControlP5.CENTER)
     ; 
     
   myChart.addDataSet("incoming");
   myChart.setData("incoming", new float[100]);


  knobTurretRotation.getCaptionLabel();
  knobTurretRotation.getValueLabel();

  
}
public void clear() {
  cp5.get(Textfield.class,"textValue").clear();
}
void controlEvent(ControlEvent theEvent) {
  if(theEvent.isAssignableFrom(Textfield.class)) {
    println(theEvent.getStringValue());
    myClient.write(theEvent.getStringValue());
  }
}
void colorWheel(int theValue) {
 println("from client: " + command);
  myClient.write(command);
}