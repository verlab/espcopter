class points {
  int x, y;
  int r;
 int line;
  float dx, dy;
  float red;
  float green;
  float blue;
  int colorPoint;

  // Constructor
  points(int _x, int _y, int r_, int line_, int colorPoint_) {
    x = _x;
    y = _y;
    r = r_;
    line = line_;
    colorPoint = colorPoint_;
  }
  int getX(){
    return round(map(x, 0, 100, 0,640));
  }
   int getY(){
    return round(map(y, 0, 100, 0,480));
  }
   int getTiming(){
    return r;
  }
   int getCollor(){
    return colorPoint;
  }
  void setCollor(int colorPoint_){
  colorPoint = colorPoint_;
  }
 
  void update() {
   int x_x =round(map(x, 0, 100, 0,640));
   int y_y =round(map(y, 0, 100, 0,480));
   noStroke();
   if(getCollor() == 0){
   fill(255, 0, 0);
   }else if(getCollor() == 1){
   fill(255, 140, 0);
   }else if(getCollor() == 2){
   fill(0, 255, 0);
   }
   
   ellipse(weightVideo+x_x, y_y, 15, 15);
 
   if(0 < line){
   points b = points.get(line-1);
   fill(255, 255, 255);
   stroke(255);
   line(weightVideo+x_x, y_y, weightVideo+(b.getX()), b.getY());

   }
   
  }
  
  
}