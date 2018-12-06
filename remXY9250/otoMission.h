#define otoMission

#define missionTime 4600

#ifdef otoMission
unsigned long baslagincZamani = 0;  
/*

                               __
                             /    \
                           /        \
                          |          |           
                          |          |
                          |   ++++   |
                          |          |
                          |          |
                          |          |
                          |Trim_Pitch|
                          

  ___________________                        ___________________
 /                   |                      |                   \
/       ----         |                      |       ++++         \
\                    |                      |                    /
 \____Trim_Roll______|                      |____Trim_Roll______/
 

                           Trim_Roll
                          |          |           
                          |          |
                          |          |
                          |   ----   |
                          |          |
                          |          |
                          |          |
                           \        /
                             \ __ /


                      
*/

unsigned long kaydedilenZaman = 0;  

int saymayaBasla = 1;
int stopMission = 1;

int gorevNo = 0;

int sure[] = {  // milisaniye
  500,
  1000,
  1000,
  1000,
  1000
};

int rollDegeri[] = {  
  0,
  0,
  25,
 -10, //
 -25
};

int pitchDegeri[] = {  
  0,
  25,
 -10, //
 -25,
  10  //
};

int yukseklikDegeri[] = {  
  250,
  250,
  250,
  250,
  250
};

void setup_(){

//Trim_Roll = 500; // -1750, 1750
//Trim_Pitch = 140; // -1750, 1750

}

void loop_(){
  if(flyMode_3 == 1 && stopMission == 1){
  unsigned long suAnkiZaman = millis();

  if(saymayaBasla == 1){
  kaydedilenZaman = suAnkiZaman; 
  baslagincZamani = suAnkiZaman;
  saymayaBasla = 0;
  }


  if( gorevNo < 5){
    
  if (suAnkiZaman - kaydedilenZaman >= sure[gorevNo] ) {
      kaydedilenZaman = suAnkiZaman;

    RX_roll =  rollDegeri[gorevNo];
    RX_pitch = pitchDegeri[gorevNo];
    targetOto = yukseklikDegeri[gorevNo];

   
    gorevNo = gorevNo + 1 ;
    }
  }

  if (suAnkiZaman - baslagincZamani >= missionTime ) {
    
    stopFlightControl = 0;
    stopMission = 0;
  }
  
}}
#endif  
