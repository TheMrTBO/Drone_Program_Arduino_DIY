#include <Servo.h>
#include <Wire.h>
// Plan 
// FL   FR
//   \ /               
//   / \                  
// BL   BR  
            
Servo motFL,motFR,motBL,motBR;

// valeurs des gain pour le pid
float pid_p_gain_roll = 0.6;               
float pid_i_gain_roll = 0;              
float pid_d_gain_roll = 8 ;                
int pid_max_roll = 400;                    

float pid_p_gain_pitch = pid_p_gain_roll;  
float pid_i_gain_pitch = pid_i_gain_roll;  
float pid_d_gain_pitch = pid_d_gain_roll;  
int pid_max_pitch = pid_max_roll;          

float pid_p_gain_yaw = 3;                
float pid_i_gain_yaw = 0.02;               
float pid_d_gain_yaw = 0;                
int pid_max_yaw = 400;

//variable du pid
float pid_error_temp; 
float pid_output_roll, pid_output_pitch, pid_output_yaw;
  
//interrupt
volatile unsigned long time_startThrottle,time_startRoll,time_startPitch,time_startYaw; /*variable volatiles ou unsigned long encadre et int interdit les nb <0*/
volatile int last_interrupt_timeThrottle,last_interrupt_timeRoll,last_interrupt_timePitch,last_interrupt_timeYaw;  
volatile unsigned int pulseThrottle,pulseRoll,pulsePitch,pulseYaw;
volatile unsigned int throttle,roll,pitch,yaw,pulse;

//pour gyro
int cal_nb;
float gyro_pitch,gyro_yaw,gyro_roll;
float gyro_roll_cal,gyro_pitch_cal,gyro_yaw_cal;
byte highByte,lowByte;

//initCommand
int start;
float pid_i_mem_roll, pid_last_roll_d_error; 
float pid_i_mem_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_last_yaw_d_error;

//commandDPS
float pid_roll_setpoint, pid_pitch_setpoint, pid_yaw_setpoint;
// convertion données du gyro en dps dans la loop
float gyro_roll_input, gyro_pitch_input, gyro_yaw_input;

//PWM fourni aux esc
int FRpwm,BRpwm,BLpwm,FLpwm;

void setup(){
  Serial.begin(9600);
  Wire.begin();
  gyro_para();
  gyro_setup();
  pinMode(2,INPUT);                 //CH1 Roll      interrupt 0
  pinMode(3,INPUT);                 //CH2 Pitch     interrupt 1
  pinMode(18,INPUT);                //CH3 Throttle  interrupt 5
  pinMode(19,INPUT);                //CH4 Yaw       interrupt 4
  
  pinMode(50,OUTPUT);               // pin servomoteur
  pinMode(51,OUTPUT); 
  pinMode(52,OUTPUT);
  pinMode(53,OUTPUT);

  attachInterrupt(0,CH1,CHANGE);    // CHANGE active attachinterrupt a chaque variation PWM(HIGH ou LOW)
  attachInterrupt(1,CH2,CHANGE);
  attachInterrupt(5,CH3,CHANGE);
  attachInterrupt(4,CH4,CHANGE);
  
  motFL.attach(50);                 // attach servo aux pin
  motFR.attach(51);
  motBL.attach(52); 
  motBR.attach(53);
}

void loop() {
  gyro_signalen();
  // on prends un pourcentage (20%) pour pas que le changement soit trop brute
  gyro_roll_input = (gyro_roll_input * 0.8) + ((gyro_roll*0.0175) * 0.2);             //Gyro en entre mit en dps pour le calcule PID
  gyro_pitch_input = (gyro_pitch_input * 0.8) + ((gyro_pitch *0.0175) * 0.2);         
  gyro_yaw_input = (gyro_yaw_input * 0.8) + ((gyro_yaw *0.0175) * 0.2); 
         
  initCommand ();
  commandDPS();
  calcul_pid();

  if (start == 2){                                                                    //les moteurs démarre.
    if (throttle > 1800) throttle = 1800;                                             //on a besoin d'un peu de marge pour controler le throttle
    FRpwm = throttle + pid_output_pitch + pid_output_roll - pid_output_yaw;           //on calcul la pulsation pour FR 
    BRpwm = throttle - pid_output_pitch + pid_output_roll + pid_output_yaw;           //on calcul la pulsation pour BR 
    BLpwm = throttle - pid_output_pitch - pid_output_roll - pid_output_yaw;           //on calcul la pulsation pour BL 
    FLpwm = throttle + pid_output_pitch - pid_output_roll + pid_output_yaw;           //on calcul la pulsation pour FL 
  // verifier ???
  if (FRpwm < 1100) FRpwm = 1100;                                         //garder les moteurs en marche
  if (BRpwm < 1100) BRpwm = 1100;                                         
  if (BLpwm < 1100) BLpwm = 1100;                                         
  if (FLpwm < 1100) FLpwm = 1100;

  if (FRpwm > 2000) FRpwm = 2000;                                         //garder les moteurs en marche
  if (BRpwm > 2000) BRpwm = 2000;                                         
  if (BLpwm > 2000) BLpwm = 2000;                                         
  if (FLpwm > 2000) FLpwm = 2000;
  }
  else{
    FRpwm=1000;           //dans le cas ou le start=!2
    BRpwm=1000;
    BLpwm=1000;
    FLpwm=1000;
  }
  motFL.writeMicroseconds(FRpwm); //fournir le pwm aux esc
  motBR.writeMicroseconds(BRpwm);
  motBL.writeMicroseconds(BLpwm);
  motFR.writeMicroseconds(FRpwm);
}
  
void CH1() { 
    last_interrupt_timeRoll = micros();/* change de valeur a chaque interruption*/
    //HIGH
    if(digitalRead(2) == HIGH) 
    { 
        time_startRoll = last_interrupt_timeRoll; /*debut de la pulsation HIGH*/
    } 
    // LOW 
    else
    { 
      pulseRoll = ( last_interrupt_timeRoll - time_startRoll); /*difference pour avoir la pulsation */
      if (pulseRoll > 0)
      {
        roll = pulseRoll;
      }
    }  
}
void CH2() {
    last_interrupt_timePitch = micros();/* change de valeur a chaque interruption*/
    //HIGH
    if(digitalRead(3) == HIGH) 
    { 
        time_startPitch = last_interrupt_timePitch; /*debut de la pulsation HIGH*/
    } 
    // LOW 
    else
    { 
      pulsePitch = ( last_interrupt_timePitch - time_startPitch); /*difference pour avoir la pulsation */
      if (pulsePitch > 0)
      {
        pitch = pulsePitch;
      }
    }  
}
void CH3() {
    last_interrupt_timeThrottle = micros();/* change de valeur a chaque interruption*/
    //HIGH
    if(digitalRead(18) == HIGH) 
    { 
        time_startThrottle = last_interrupt_timeThrottle; /*debut de la pulsation HIGH*/
    } 
    // LOW 
    else
    { 
      pulseThrottle = ( last_interrupt_timeThrottle - time_startThrottle); /*difference pour avoir la pulsation */
      if (pulseThrottle > 0)
      {
        throttle = pulseThrottle;
      }
    }  
}
void CH4() {
    last_interrupt_timeYaw = micros();/* change de valeur a chaque interruption*/
    //HIGH
    if(digitalRead(19) == HIGH) 
    { 
        time_startYaw = last_interrupt_timeYaw; /*debut de la pulsation HIGH*/
    } 
    // LOW 
    else
    { 
      pulseYaw = ( last_interrupt_timeYaw - time_startYaw); /*difference pour avoir la pulsation */
      if (pulseYaw > 0)
      {
        yaw = pulseYaw;
      }
    }  
}
void gyro_setup() { //à la suite de gyro_para
  for (cal_nb = 0; cal_nb < 2000 ; cal_nb ++){    //prends 2000 mesures pour calibrer
    gyro_signalen();                                 //appel la fct lisant les données des axes
    gyro_roll_cal += gyro_roll;                      //on somme les valeur sur chaque axes
    gyro_pitch_cal += gyro_pitch;                        
    gyro_yaw_cal += gyro_yaw;
    delay(3);
    }
  gyro_roll_cal /= 2000;                               //on divise par le nb de valeurs
  gyro_pitch_cal /= 2000;                                    
  gyro_yaw_cal /= 2000;
}
void gyro_signalen() {
  Wire.beginTransmission(0x69);                 //Début de la communication avec le gyro (adresse I2C 0x69)
  Wire.write(168);                              //commence la lecture du registre 28h et auto-incrementation avec chaque lecture
  Wire.endTransmission();                       //fin de la communication avec le gyro
  Wire.requestFrom(0x69, 6);                    //Demande 6 octets (6*8=48bits, 16bits par axe) au gyro
  while(Wire.available() < 6);                  //on attends que les 6 octects soient recus
  lowByte = Wire.read();                        //le premier octet recu et l'octet bas des données angulaire
  highByte = Wire.read();                       //le second octet recu et l'octet haut des données angulaire
  gyro_roll = ((highByte<<8)|lowByte);          //multiplie l'octet haut par 256=2**8 ("shift left by 8bits") et ajoute le l'octet bas
  lowByte = Wire.read();                                       
  highByte = Wire.read();                                      
  gyro_pitch = ((highByte<<8)|lowByte);         // même procédé pour les autres angles              
  lowByte = Wire.read();                                       
  highByte = Wire.read();                                      
  gyro_yaw = ((highByte<<8)|lowByte);
                        
  if(cal_nb == 2000){
    gyro_roll -= gyro_roll_cal;               //on retire l'offset
    gyro_pitch -= gyro_pitch_cal;                           
    gyro_yaw -= gyro_yaw_cal; 
   }   
}
void gyro_para(){ //après Wire.begin()
  Wire.beginTransmission(0x69);               //Début de la communication avec le gyro (adresse I2C 0x69)
  Wire.write(0x20);                           //on veut écrire sur REG1 (20 hex)
  Wire.write(0x0F);                           //on configure les bits du registre à 00001111 (ce qui réveille le gyro et active les 3 axes)
  Wire.endTransmission();                     //fin de la communication avec le gyro
  delay(1000);
  Wire.beginTransmission(0x69);               //Début de la communication avec le gyro (adresse I2C 0x69)
  Wire.write(0x23);                           //on veut écrire sur REG4 (23 hex)
  Wire.write(0x90);                           //on configure les bits du registre à 10010000 
                                              //(cache à jour en permanence & configure la sensi à 500dps ) 
  Wire.endTransmission();                     //fin de la communication avec le gyro
  }
void initCommand (){                          // a mettre dans la loop principal
  
  
  if(throttle < 1050 && yaw < 1050)start = 1;       //pour activer les moteurs: throttle en bas et yaw à gauche (etape 1).
  
  if(start == 1 && throttle < 1050 && yaw > 1450){  //quand yaw est a sa position central demarrer les moteurs (etape 2).
    
    start = 2;
    //initialise les PID
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
  }
  
  if(start == 2 && throttle < 1050 && yaw > 1950)start = 0; //Stop les moteurs: throttle en bas et yaw a droite.
  }
void commandDPS(){
  //Le pid mit en dps est determiné par roll mit en dps
  //ici on divise le max roll par 6 pour avoir au max 82dps ( (500-8)/6 = 82d/s ).
  pid_roll_setpoint = 0;
  
  // on a besoin d'une marge de 16us pour de meilleurs resultats.
  if(roll > 1508)pid_roll_setpoint = (roll - 1508)/6.0;
  else if(roll < 1492)pid_roll_setpoint = (roll - 1492)/6.0;
  
  pid_pitch_setpoint = 0;
  
  if(pitch > 1508)pid_pitch_setpoint = (pitch - 1508)/6.0;
  else if(pitch < 1492)pid_pitch_setpoint = (pitch - 1492)/6.0;
  
  pid_yaw_setpoint = 0;
  
  if(yaw > 1050){ //ne pas tanger quand on eteint les moteurs
  if(yaw > 1508)pid_yaw_setpoint = (yaw - 1508)/6.0;
  else if(yaw < 1492)pid_yaw_setpoint = (yaw - 1492)/6.0;
  }
}
void calcul_pid(){
  //calcul du roll
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;                                               // P
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;                                                 // I
  
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

                                                                                                      //D
  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;
  
  pid_last_roll_d_error = pid_error_temp;
  
  //calcul du pitch
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;                                                
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;
  
  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;
    
  pid_last_pitch_d_error = pid_error_temp;
    
  //calcul du yaw
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;
  
  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;
    
  pid_last_yaw_d_error = pid_error_temp;
}
