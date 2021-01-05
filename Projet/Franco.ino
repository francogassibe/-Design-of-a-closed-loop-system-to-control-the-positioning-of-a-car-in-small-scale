//CODIGO OPTIMIZADO //
//VERIFICAR UNA VEZ MAS POR LAS DUDAS//
#include <PID_v1.h>

const int Emisor = 9;   //Pin digital 2 para el Trigger del sensor
const int Receptor = 8;   //Pin digital 3 para el echo del sensor
long tiempodeonda; 
double distancia;
double vmax, Var_max,var;
double Kp,Ki,Kd, dkp=6,dki=3,dkd=0.5,kpinic=8,kie=2.5,kde=0.91,sum1=0;
double salidapwm;
const double Tiempo_cambio=1.5*1000000; //en microsegundos
const double paso_ojos=150000;//en microsegundos (0,3s)
double n=1;//exponente del cociente variaciaion

int c=0,i=0,j=0;//voler Cmax un entero (falta porque tiempo_cambio es double)
const int Cmax=Tiempo_cambio/paso_ojos;
int v[Cmax];
int IN1=5;    // Input3 conectada al pin 5+
int IN2=4;    // Input4 conectada al pin 4 +
int IN3=7;
int IN4=2;
int ENA=3;    // ENA conectada al pin 3 de Arduino
int ENB=6;
double Setpoint = 15.0;
int ledPin = 13;

PID miPID(&distancia,&salidapwm,&Setpoint,1,1,1,DIRECT);

//-------------------------------------------------------------------------

void setup() {
    Serial.begin(9600);
    pinMode(Emisor, OUTPUT); //pin como salida
    pinMode(Receptor, INPUT);  //pin como entrada
    digitalWrite(Emisor, LOW);//Inicializamos el pin con 0
    pinMode (ENA, OUTPUT);
    pinMode (IN1, OUTPUT);
    pinMode (IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ledPin, OUTPUT);
    miPID.SetMode(AUTOMATIC);
    miPID.SetOutputLimits(-255,255);
    if ((70-Setpoint)>(Setpoint-0)){
        vmax=70-Setpoint;
    }
    else {
        vmax=Setpoint;
    }
    Var_max=abs(vmax)*sqrt(Cmax)/Cmax;//al cuadrado
}

//------------------------------------------------------------------------------------

void loop(){
    digitalWrite(Emisor, HIGH);
    delayMicroseconds(paso_ojos);          
    digitalWrite(Emisor, LOW);
    tiempodeonda = pulseIn(Receptor, HIGH);
    distancia = tiempodeonda/59; //Variable de retroalimentacion
    if (c<Cmax){
        v[c]=distancia;
        sum1+=(Setpoint-v[c])*(Setpoint-v[c]);
        var=(sqrt(sum1))/(c+1);
        c++;
    }
    if(c==Cmax){
        sum1-=(Setpoint-v[j])*(Setpoint-v[j]);
        v[j]=distancia;
        sum1+=(Setpoint-v[j])*(Setpoint-v[j]);
        var=(sqrt(sum1))/Cmax;
        j++;
        if (j==Cmax){
          j=0;
        }
    }
    if(var==0){
       digitalWrite(ledPin, HIGH);
        c=0;//podria prender un led azul o rosa para indicarlo
        sum1=0;
    } else{
        digitalWrite(ledPin, LOW);
    }
   
Kp=kpinic-((1-(var/Var_max))*dkp);
Ki=kie-((var/Var_max))*dki;
Kd=kde+sqrt(((var/Var_max)))*dkd;
    PID miPID(&distancia,&salidapwm,&Setpoint,Kp,Ki,Kd,DIRECT);
    miPID.SetMode(AUTOMATIC);
    miPID.SetOutputLimits(-255,255);
    miPID.Compute();
   // Serial.print(distancia);  
   // Serial.print("\t");
   
    Serial.print(var);  
   // Serial.print("\t");
    //Serial.print(Ki);
   // Serial.print(Kd);
       
    Serial.println();
    if(salidapwm<0){
      
        digitalWrite(IN1,LOW);
        digitalWrite(IN2,HIGH);
        digitalWrite(IN3,HIGH);
        digitalWrite(IN4,LOW);
        analogWrite(ENA,abs(salidapwm));
        analogWrite(ENB,abs(salidapwm));
    }
    else{
        digitalWrite(IN1,HIGH);
        digitalWrite(IN2,LOW);
        digitalWrite(IN3,LOW);
        digitalWrite(IN4,HIGH);
        analogWrite(ENA,0.4*abs(salidapwm));
        analogWrite(ENB,0.4*abs(salidapwm));
    }
    
}
