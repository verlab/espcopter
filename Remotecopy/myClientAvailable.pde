
String  inString;
int datain;
 float   dt;
 float   x_gyr;  //Gyroscope data
float   y_gyr;
float   z_gyr;
float   x_acc;  //Accelerometer data
float   y_acc;
float   z_acc;
float   x_fil;  //Filtered data
float   y_fil;
float   z_fil;
float   w_fil;
 void myClientAvaible(){
   
    pushMatrix();
    translate(430, 5, 350);
    rotateX(radians(x_gyr));
    rotateY(radians(y_gyr));
     
   shape(quad, 200, 300);
   popMatrix();
   fill(50, 50, 50);
   
     rect(0, 0, 745, 480, 7);

   
  int satir= 70;
   textSize(20);
  String accStr = "x_acc = " + (int) x_acc;
  String accStr2 ="y_acc = " + (int) y_acc;
  String accStr3 ="z_acc = " + (int) z_acc;
   
  String gyrStr = "x_gyr = " + (int) x_gyr;
  String gyrStr2 ="y_gyr = " + (int) y_gyr;
  String gyrStr3 ="z_gyr = " + (int) z_gyr;
  
  String filStr = "Degree = " + (int) x_fil;
  String filStr2 ="Z Sensor = " + (int) y_fil;
  String filStr3 ="Altitude = " + (float) z_fil;

  
    fill(249, 250, 50);
 text("Gyroscope:", (int) 30, satir);
  text(gyrStr, (int) 30, satir+25);
  text(gyrStr2, (int) 30, satir+50);
  text(gyrStr3, (int) 30, satir+75);

  fill(56, 140, 206);
  text("Accelerometre:", (int) 30, satir+125);
  text(accStr, (int) 30, satir+150); 
  text(accStr2, (int) 30, satir+175);
  text(accStr3, (int) 30, satir+200);
    
  fill(83, 175, 93);
  text("Other Sensors:", (int) 30, satir+250);
  text(filStr, (int) 30, satir+275);
  text(filStr2, (int) 30, satir+300);
  text(filStr3, (int) 30, satir+325);

  
   if (myClient.available() > 0) { 
   inString = (myClient.readString());
   println("calculation from string: " + inString);
    

  try {
    // Parse the data
    String[] dataStrings = split(inString, '#');
    for (int i = 0; i < dataStrings.length; i++) {
      String type = dataStrings[i].substring(0, 4);
      String dataval = dataStrings[i].substring(4);
    if (type.equals("DEL:")) {
        dt = float(dataval);
      } else if (type.equals("ACC:")) {
        String data[] = split(dataval, ',');
        x_acc = float(data[0]);
        y_acc = float(data[1]);
        z_acc = float(data[2]);
      } else if (type.equals("GYR:")) {
        String data[] = split(dataval, ',');
        x_gyr = float(data[0]);
        y_gyr = float(data[1]);
      } else if (type.equals("FIL:")) {
        String data[] = split(dataval, ',');
        x_fil = float(data[0]);
        y_fil = float(data[1]);
        z_fil = float(data[2]);
      }
    }
  } catch (Exception e) {
      //println("Caught Exception");
  }
    }
 }

 