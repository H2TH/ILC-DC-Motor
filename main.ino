/*
*Iterative Learning Control for the position control of a DC motor
*@author: Hoang T.Hua
*
/*




#include<stdio.h>

#define pinEncoderA 18
#define pinEncoderB 19
#define pinL298In1 7
#define pinL298In2 8

float K_ILC = 0.1;                    //K_p of ILC controller
float Kd_ILC = 0.1;                   //K_d of ILC controller
float T_ILC = 0.01;                   //Sampling time

/*
      Parameters of PID controller
*/
float Kp = 0.00814111545730487;
float Ki = 0.00325762772587089;
float integral;
float Kd = 0.000542676605189655;

int x;                  //Number of sampling points per iteration for mapping Y_REF() and Input()
float U_PID, U_PID1;    //PID control output
int e, e_pre, e_pre2;   //error in calculation of PID at sampling t, t-1, and t-2 respectively 
float U_ILC;            //ILC control output
float U_old[157];       //Saving ILC control output of the last iteration
int e_ilc;              //Intermediate variable to save error at sampling point
int de[157];            //Saving error of the last  iterration
float U[157];           //Control signal

double Y_REF[157];       //Reference signal                  
int pulseNumber = 0;    //Initial motor position

int i;                  //Iterations
int t;                  //Sample points




/*
  @brief:   Recording motor's position
  @param:   State of chanelB of encoder
  @return:  pulseNumer (position) of the motor
*/
int chanelB;
void PulseCounter() 
{
  chanelB = digitalRead(pinEncoderB);
	if (chanelB == 1)
	{
        pulseNumber++;
    } 
	else 
	{
        pulseNumber--;
    }
}

/*
  @brief: Creating reference signal
*/
void Y_ref()
{
  for (x=0;x<157;x++)
  {
    Y_REF[x] = (1-cos(4*x*0.01))*(374/2);
    Y_REF[x] = int(Y_REF[x]);
  }
}

/*
  @brief: Creating pre-control signal for the first iteration
*/
void Input()
{
  for (x=0;x<157;x++)
  {
    U[x]=0;
    de[x]=Y_REF[x] - pulseNumber;
  }
}

/*
  @brief:   Driving L298 module
  @param:   Control input
  @return:  None
*/
void Control(int u)
{
  if (u > 0)
  {
    analogWrite(pinL298In1,u);
    analogWrite(pinL298In2,0);
  }
  else
  {
    analogWrite(pinL298In1,0);
    analogWrite(pinL298In2,abs(u));
  }
}

/*
  @brief: PID calculation
*/
void PID_control(int u_ilc)
{
    e_pre = e;
    e_pre2 = e_pre;
    e = u_ilc - pulseNumber;
    U_PID1= U_PID;
    U_PID = U_PID1 + Kp*(e - e_pre) + Ki*(0.01/2)*(e + e_pre) + (Kd/0.01)*(e - 2*e_pre + e_pre2);
    U_PID = constrain(U_PID,-255,255);
}

/*
  @brief: ILC calculation
*/
void ILC()
{
  for(i=1;i<20;i++) //20 iterations
  {
    pulseNumber=0;  //initial potion of motor is set to 0
    
    //First iteration
    if(i==1)
    {
      for(t=0;t<157;t++)
      {
        if(t!=0)
        {
          U_ILC=((K_ILC*Y_REF[t+1]) + ((Kd_ILC*(Y_REF[t+1]- Y_REF[t]))/T_ILC)) + U[t];
        }
        else
        {
          U_ILC=((K_ILC*Y_REF[t]) + ((Kd_ILC*(Y_REF[t])/T_ILC))) + U[t];
        }
      
      U_old[t] = int (U_ILC); //Saving control input of current iteration
      PID_control(U_old);     //Calculating control input through a cascade ILC-PID controller
      U[t] = U_PID;           //Control input
      
      int now = millis();     
      while(millis() - now <= 10)
      {
        Control(U[t]);
        //Serial.print("Control="); Serial.println(U[t]); 
      }
      
      Serial.println(pulseNumber);         
      e_ilc = Y_REF[t] - pulseNumber;   //Saving error at sampling point to an intermediate variable
      de[t] = e_ilc;                    //Saving errir at sampling point
      }
    }            
    //end first iteration
    
    //Next iterations
    else
    {
        for(t=0;t<157;t++)
        {
            if(t!=0)
            {
                U_ILC=((K_ILC*de[t+1]) + ((Kd_ILC*(de[t+1]-de[t]))/T_ILC)) + U[t];
            }
            else
            {
                U_ILC=((K_ILC*de[t]) + ((Kd_ILC*(de[t])/T_ILC))) + U[t];
            }
      
      U_old[t] = int (U_ILC);
      PID_control(U_old);
      U[t]=U_PID;

      int now = millis();
      while(millis() - now <= 10)
      {
        Control(U[t]);
      }
      Serial.println(pulseNumber);         
      e_ilc=Y_REF[t] - pulseNumber;
      de[t]=e_ilc;
      }
      Serial.println("Overrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr");
      //End of the iteration  
    }
    //End of all iterations       
  }
}

void setup() 
{
  // put your setup code here, to run once:
  pinMode(pinEncoderA, INPUT);
  pinMode(pinEncoderB, INPUT);
  pinMode(pinL298In1, OUTPUT);
  pinMode(pinL298In2, OUTPUT);
  Serial.begin(9600);
  attachInterrupt(5, PulseCounter,FALLING);
}

void loop() {
  
  Y_ref();  //Creating reference;
  Input();  //Pre-control signal
  ILC();    //Learning phase
  
  //After completing the learning phase, using control input from the last iteration to execute the task repeatedly 
  while(1)
  {
    for(t=0;t<157;t++)
    {
      Control(U[t]);
      delay(10);
    }
  }
}
    
