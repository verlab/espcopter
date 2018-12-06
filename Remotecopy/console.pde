import org.gamecontrolplus.gui.*;
import org.gamecontrolplus.*;
import net.java.games.input.*;

ControlIO control;
ControlDevice stick;
int throttle_jstc,throttle_jstc_old, rudder_jstc, elevator_jstc, aileron_jstc = 0;
boolean L1On,L2On, R1On, R2On, star ,oval, square,trigon, keyRight, keyLeft;

void consoleinit()
{
    // Initialise the ControlIO
  control = ControlIO.getInstance(this);
  // Find a device that matches the configuration file
  stick = control.getMatchedDevice("espcopterv2");
  if (stick == null) {
    println("No suitable device configured");
    System.exit(-1); // End the program NOW!
  }
  
  
}
public void getUserInput() {
  if(ifConsole){
  throttle_jstc_old = throttle_jstc;
  throttle_jstc = round(map(stick.getSlider("thorttel").getValue(), 0, -1, 0, 20));//throttle rudder 
  throttle_jstc *= 5;
  rudder_jstc = round(map(stick.getSlider("rudder").getValue(), -1, 1, -10, 10));
  elevator_jstc = round(map(stick.getSlider("elevator").getValue(), -1, 1, -10, 10));
  aileron_jstc = round(map(stick.getSlider("aileron").getValue(), -1, 1, -10, 10));
  
 // R1On = stick.getButton("a").pressed();
  //R2On = stick.getButton("y").pressed();
  //L1On = stick.getButton("b").pressed();
  //L2On = stick.getButton("x").pressed();
  
  // star = stick.getButton("star").pressed();
  // trigon = stick.getButton("trigon").pressed();
  // square = stick.getButton("square").pressed();
  // oval = stick.getButton("oval").pressed();
  // keyRight = stick.getButton("keyRight").pressed();
  // keyLeft = stick.getButton("keyLeft").pressed();
}
}