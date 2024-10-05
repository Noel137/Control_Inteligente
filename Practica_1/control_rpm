unsigned long dt_us = 200;    // muestreo en micro-segundos
float dt = dt_us*0.000001;    // muestro en segudos
unsigned long t1 = 0, t2 = 0; // Tiempos para intervalos

unsigned long rt_p =0;
unsigned long rt=0;
unsigned long interval=0;
float alpha = 0.05;            // Coeficiente de filtro
unsigned long rpm=0, rpm_f=0;
unsigned long rpm_des=0;

float kp = 0.0581, kd = 0.075, ki = 0.3; //PID AGRESIVO

float e = 0, de = 0, inte = 0;
float e_norm; 
float u = 0, usat = 0;        // SEÑAL DE CONTROL
float PWM = 0;                  // SEÑAL PWM
float th_des = 0;             // CONSIGAN DEL CONTROL


float rpm_p=0, rpm_d=0;
float rpm_calculado= 0;            // Consigna de velocidad (RPM)
float pulses_per_rev = 341; // Pulsos por revolución (ajustar según tu encoder)
float revs=0, revsp;

int sen1=5;
int sen2=6;

String consigna;

void setup() {
  
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(3), CH_A, RISING);

  // Salidas de control PWM
  pinMode(sen1,OUTPUT);
  pinMode(sen2,OUTPUT);
}

void loop() {
  // El código se debe escribir entre la toma de los tiempo
  t1 = micros();  //Muestra de tiempo 1
    
      if(Serial.available()> 0){
      consigna = Serial.readStringUntil('\n');
      rpm_des = consigna.toFloat();  // Leer consigna deseada en RPM *********************************
      }
      
      rpm = rpm_calculado;
      rpm_d = (rpm-rpm_p)/dt;

      // Filtro de la velocidad 
      rpm_f = alpha*rpm_d + (1-alpha)*rpm_f;
      

      e = rpm_des - rpm;             // ERROR 
      de = - rpm_f;                // DERIVADA DEL ERROR
      inte = inte + e*dt;          // INTEGRAL DEL ERROR      
          
      e_norm = e / 290; 
      
      if(e_norm>0.3){
        if(rpm_des < rpm){
          PWM = 0;
        }
        else{
          PWM = 255;
        }
      }

      if(0.3 >= e_norm && e_norm > 0.2){//PID agresivo
        kp = 0.981;
        kd = 0.035; 
        ki = 27;
      }
      else if( e_norm>0.1){// PID suave
        kp = 0.98; 
        kd = 0.00141291;
        ki = 10.72; 
      }
      else{//PI suave
        kp = 0.2;
        kd = 0;
        ki = 10;
      }

      u = kp*e + kd*de + ki*inte;  // VOLTAJE
      usat = constrain(u,0 ,12); // SATURACION
      PWM = usat*21.25; // 255/6V = 42.5
      
             // MANDAR SEÑAL DE CONTROL COMO PWM
      if(PWM>0){
      analogWrite(5,PWM);
      digitalWrite(10, HIGH);
      digitalWrite(11, LOW);
      
      }
      

      rpm_p = rpm;
      revsp = revs;
      Serial.print(rpm);      
      Serial.print(',');
      Serial.println(PWM);
  t2 = micros();  //Muestra de tiempo 2
  while((t2-t1)<dt_us){
    t2 = micros();
  }
  //Serial.println(t2-t1);
}

void CH_A(){
  rt= micros();
  interval = rt - rt_p;
  rt_p = rt;
  rpm_calculado = 60*1000000/(342*interval);
}
