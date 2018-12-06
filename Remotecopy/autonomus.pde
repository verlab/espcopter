



int pointerweight = 740;
int pointerHeight = 0;

int[][] PointArray = new int[50][2];
int pointerCounter = -1;

int rangeOto = 5;

ArrayList<points> points = new ArrayList<points>();


void customize(DropdownList ddl) { // dropdownlist
  // a convenience function to customize a DropdownList
   pointerCounter = pointerCounter +1 ;
  
  
   
  
  
   if(pointerCounter == 1){
   points b;
   PointArray[0+pointerCounter][0] =  round(sslider.getArrayValue()[0]);
   PointArray[0+pointerCounter][1] =  round(sslider.getArrayValue()[1]);
   b = new points(round(sslider.getArrayValue()[0]),round(sslider.getArrayValue()[1]), timing, pointerCounter,0);
   points.add(b);  
   }else{
   points b;
   int xAxis= constrain(round(sslider.getArrayValue()[0])-round(map(points.get(pointerCounter-2).getX(), 0, 640, 0,100)),-rangeOto,rangeOto)  + round(map(points.get(pointerCounter-2).getX(), 0, 640, 0,100));
   int yAxis= constrain(round(sslider.getArrayValue()[1])-round(map(points.get(pointerCounter-2).getY(), 0, 480, 0,100)),-rangeOto,rangeOto)  + round(map(points.get(pointerCounter-2).getY(), 0, 480, 0,100));
   PointArray[0+pointerCounter][0] = xAxis;
   PointArray[0+pointerCounter][1] = yAxis;
   b = new points(xAxis,yAxis, timing, pointerCounter,0);
   points.add(b);
   
   }  
   
  //println(round(sslider.getArrayValue()[0]) + " count " + pointerCounter + " point " + points.get(pointerCounter-1).getX() + " MAP " +round(map(points.get(pointerCounter-1).getX(), 0, 640, 0,100)));
   
   round(map(x, 0, 100, 0,640));
   
   ddl.addItem("Point  " +pointerCounter, pointerCounter); 
   ddl.setColorBackground(color(60));
   ddl.setColorActive(color(255, 128));
   
}
void autonomus(){

  textSize(15); 
  fill(255, 0, 0);
  rect(pointerweight+6.4*round(sslider.getArrayValue()[0]), 4.8*round(sslider.getArrayValue()[1])+pointerHeight, 40, 4, 10);
  rect(pointerweight+18+6.4*round(sslider.getArrayValue()[0]), 4.8*round(sslider.getArrayValue()[1])+pointerHeight-18, 4, 40, 10);
  text(sslider.getArrayValue()[0]+","+  sslider.getArrayValue()[1], (int)  pointerweight+25 + round(6.4*round(sslider.getArrayValue()[0])), pointerHeight+25 + round(4.8*round(sslider.getArrayValue()[1]))); 

   if(ifMission){
    String[] lines = loadStrings("missionBase.txt");
    println("there are " + lines.length + " lines");
    for (int i = 0 ; i < lines.length; i++) {
    println(lines[i]);
    int[] XYcor = int(split(lines[i], ' '));

    points b;
    b = new points(XYcor[0],XYcor[1], XYcor[2], XYcor[3], 0);
    points.add(b);
    
    }
  ifMission = false;
}
    
    
   for (int i = 0; i<points.size(); i++) {
      points b = points.get(i);
      b.update();
    }
    
    
    
     
    

}
public void TakeCoordinat(int theValueCordinat) {
customize(d1); 
}