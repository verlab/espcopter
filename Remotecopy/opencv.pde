import gab.opencv.*;
import processing.video.*;
import java.awt.Rectangle;

Capture video;
OpenCV opencv;
PImage src, colorFilteredImage;
ArrayList<Contour> contours;
ArrayList<Contour> contours_1;
ArrayList<Contour> contours_2;
ArrayList<Contour> contours_3;

// <1> Set the range of Hue values for our filter
int rangeLow = 20;
int rangeHigh = 35;
int weightVideo= 760; 

int maxColors = 4;
int[] hues;
int[] colors;
int rangeWidth = 10;

PImage[] outputs;
int colorToChange = -1;

void opencCvinit(){
String[] cameras = Capture.list();

 if (cameras.length == 0) {
    println("There are no cameras available for capture.");
    exit();
  } else {
    println("Available cameras:");
    for (int i = 0; i < cameras.length; i++) {
      println(cameras[i]);
    }
  }
  
 video = new Capture(this, 640, 480,cameras[14]); //,cameras[14])
  opencv = new OpenCV(this,video.width, video.height);
  
  contours = new ArrayList<Contour>();
   contours_1 = new ArrayList<Contour>();
    contours_2 = new ArrayList<Contour>();
     contours_3 = new ArrayList<Contour>();
     
  colors = new int[maxColors];
  hues = new int[maxColors];
  outputs = new PImage[maxColors];
    video.start();

}
void openCvvideo(){


  if (video.available()) {
    video.read();
  }

  // <2> Load the new frame of our movie in to OpenCV
  opencv.loadImage(video);
  
  // Tell OpenCV to use color information
  opencv.useColor();
  src = opencv.getSnapshot();
  
  // <3> Tell OpenCV to work in HSV color space.
  opencv.useColor(HSB);
  
  detectColors();
  
  // Show images
  image(src, 760, 0);
  for (int i=0; i<outputs.length; i++) {
    if (outputs[i] != null) {
      image(outputs[i], width-src.width/4, i*src.height/4, src.width/4, src.height/4);
      
      noStroke();
      fill(colors[i]);
      rect(src.width+760, i*src.height/4, 30, src.height/4);
    }
  }
  
  // Print text if new color expected
  textSize(20);
  stroke(255);
  fill(255);
  
  if (colorToChange > -1) {
    text("click to change color " + colorToChange, 770, 25);
  } else {
    text("press key [1-4] to select color", 770, 25);
  }
  
  displayContoursBoundingBoxes();
}

//////////////////////
// Detect Functions///
//////////////////////

void detectColors() {
    
  for (int i=0; i<hues.length; i++) {
    
    if (hues[i] <= 0) continue;
    
    opencv.loadImage(src);
    opencv.useColor(HSB);
    
    // <4> Copy the Hue channel of our image into 
    //     the gray channel, which we process.
    opencv.setGray(opencv.getH().clone());
    
    int hueToDetect = hues[i];
    //println("index " + i + " - hue to detect: " + hueToDetect);
    
    // <5> Filter the image based on the range of 
    //     hue values that match the object we want to track.
    opencv.inRange(hueToDetect-rangeWidth/2, hueToDetect+rangeWidth/2);
    
    //opencv.dilate();
    opencv.erode();
    
    // TO DO:
    // Add here some image filtering to detect blobs better
    
    // <6> Save the processed image for reference.
    outputs[i] = opencv.getSnapshot();
  }
  
  // <7> Find contours in our range image.
  //     Passing 'true' sorts them by descending area.
  if (outputs[0] != null) {

    opencv.loadImage(outputs[0]);
    contours = opencv.findContours(true,true);
  }
    if (outputs[1] != null) {
    opencv.loadImage(outputs[1]);
    contours_1 = opencv.findContours(true,true);
  }
    if (outputs[2] != null) {
    opencv.loadImage(outputs[2]);
    contours_2 = opencv.findContours(true,true);
  }
    if (outputs[3] != null) {
    opencv.loadImage(outputs[3]);
    contours_3 = opencv.findContours(true,true);
  }
  
  
}

void displayContoursBoundingBoxes() {
  
  if (contours.size() > 0.8) {
    // <9> Get the first contour, which will be the largest one
    Contour biggestContour = contours.get(0);
    
    // <10> Find the bounding box of the largest contour,
    //      and hence our object.
    Rectangle r = biggestContour.getBoundingBox();
    
    // <11> Draw the bounding box of our object
    noFill(); 
    strokeWeight(2); 
    stroke(255, 0, 0);
    rect(r.x+weightVideo, r.y, r.width, r.height);
    
    // <12> Draw a dot in the middle of the bounding box, on the object.

    fill(255, 0, 0);
    ellipse(r.x + r.width/2+weightVideo, r.y + r.height/2, 30, 30);
    fill(255, 255, 0);
   textSize(20); 
   text( r.x + r.width/2 +","+ r.y + r.height/2, (int)  r.x + r.width/2+weightVideo, r.y + r.height/2-20); 
   
   float xAxis_ = r.x + r.width/2;
   float yAxis_ =  r.y + r.height/2;
   yAxis_ =round(map(yAxis_, 0, 480, 0,100));
   xAxis_ =round(map(xAxis_, 0, 640, 0,100));

   
   pid(xAxis_, yAxis_ , 0);
   
   text( (int)xAxis_ +","+ (int)yAxis_, 1100, 25); 
}

  if (contours_1.size() > 0.5) {
    // <9> Get the first contour, which will be the largest one
    Contour biggestContour = contours_1.get(0);
    
    // <10> Find the bounding box of the largest contour,
    //      and hence our object.
    Rectangle r = biggestContour.getBoundingBox();
    
    // <11> Draw the bounding box of our object
    noFill(); 
    strokeWeight(2); 
    stroke(0, 255, 0);
    rect(r.x+weightVideo, r.y, r.width, r.height);
    
    // <12> Draw a dot in the middle of the bounding box, on the object.
    noStroke(); 
    fill(0, 255, 0);
    ellipse(r.x + r.width/2+weightVideo, r.y + r.height/2, 30, 30);
  
   fill(0, 255, 0);
   textSize(20); 
   text( r.x + r.width/2 +","+ r.y + r.height/2, (int)  r.x + r.width/2+weightVideo, r.y + r.height/2-20); 
   
   float xAxis_ = r.x + r.width/2;
   float yAxis_ =  r.y + r.height/2;
   yAxis_ =round(map(yAxis_, 0, 480, 0,100));
   xAxis_ =round(map(xAxis_, 0, 640, 0,100));
   
   
  // pid(xAxis_, yAxis_,1);
   
   text( (int)xAxis_ +","+ (int)yAxis_, 1100+75, 25); 
 
}
/*
  if (contours_2.size() > 10) {
    // <9> Get the first contour, which will be the largest one
    Contour biggestContour = contours_2.get(0);
    
    // <10> Find the bounding box of the largest contour,
    //      and hence our object.
    Rectangle r = biggestContour.getBoundingBox();
    
    // <11> Draw the bounding box of our object
    noFill(); 
    strokeWeight(2); 
    stroke(0, 0, 255);
    rect(r.x+weightVideo, r.y, r.width, r.height);
    
    // <12> Draw a dot in the middle of the bounding box, on the object.
    noStroke(); 
    fill(0, 0, 255);
    ellipse(r.x + r.width/2+weightVideo, r.y + r.height/2, 30, 30);
  
   fill(0, 0, 255);
   textSize(20); 
   text( r.x + r.width/2 +","+ r.y + r.height/2, (int)  r.x + r.width/2+weightVideo, r.y + r.height/2-20); 
   
   float xAxis_ = r.x + r.width/2;
   float yAxis_ =  r.y + r.height/2;
   yAxis_ =round(map(yAxis_, 0, 480, 0,100));
   xAxis_ =round(map(xAxis_, 0, 640, 0,100));
   
   
   pid(xAxis_, yAxis_,2);
   
   text( (int)xAxis_ +","+ (int)yAxis_, 1100+150, 25); 
   
    fill(0, 255, 255);
    ellipse(weightVideo+320, 240, 30, 30);
}
  if (contours_3.size() > 10) {
    // <9> Get the first contour, which will be the largest one
    Contour biggestContour = contours_3.get(0);
    
    // <10> Find the bounding box of the largest contour,
    //      and hence our object.
    Rectangle r = biggestContour.getBoundingBox();
    
    // <11> Draw the bounding box of our object
    noFill(); 
    strokeWeight(2); 
    stroke(255, 255, 255);
    rect(r.x+weightVideo, r.y, r.width, r.height);
    
    // <12> Draw a dot in the middle of the bounding box, on the object.
    noStroke(); 
    fill(255, 255, 255);
    ellipse(r.x + r.width/2+weightVideo, r.y + r.height/2, 30, 30);
  
   fill(255, 255, 255);
   textSize(20); 
   text( r.x + r.width/2 +","+ r.y + r.height/2, (int)  r.x + r.width/2+weightVideo, r.y + r.height/2-20); 
   
   float xAxis_ = r.x + r.width/2;
   float yAxis_ =  r.y + r.height/2;
   yAxis_ =round(map(yAxis_, 0, 480, 0,100));
   xAxis_ =round(map(xAxis_, 0, 640, 0,100));
   
   
   pid(xAxis_, yAxis_,3);
   
   text( (int)xAxis_ +","+ (int)yAxis_, 1100+225, 25); 
   
   fill(0, 255, 255);
   ellipse(weightVideo+320, 240, 30, 30);
}
*/
}

//////////////////////
// Keyboard / Mouse
//////////////////////

void mousePressed() {
    
  if (colorToChange > -1) {
    
    color c = get(mouseX, mouseY);
    println("r: " + red(c) + " g: " + green(c) + " b: " + blue(c));
   
    int hue = int(map(hue(c), 0, 255, 0, 180));
    
    colors[colorToChange-1] = c;
    hues[colorToChange-1] = hue;
    
    println("color index " + (colorToChange-1) + ", value: " + hue);
  }
}

void keyPressed() {
  
  if (key == '1') {
    colorToChange = 1;
    
  } else if (key == '2') {
    colorToChange = 2;
    
  } else if (key == '3') {
    colorToChange = 3;
    
  } else if (key == '4') {
    colorToChange = 4;
  }
}

void keyReleased() {
  colorToChange = -1; 
}
