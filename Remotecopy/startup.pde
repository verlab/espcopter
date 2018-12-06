int counter = 1;
void startup(){
 if(counter < 20){
     XAxisBase = XAxis;
     YAxisBase = XAxis;
     qxBase = qx;
     sxBase = throttle ;
    
    counter +=  1 ;
    println("Waiting  "+counter );
  }

}