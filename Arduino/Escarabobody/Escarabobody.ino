#include <Servo.h>


// DEFINICION DE MOTORES

// DERECHOS
#define motorDD 2
#define ADD 22
#define TDD 23

#define motorTD 3
#define ATD 24
#define TTD 25

// IZQUIERDOS
#define motorDI 4
#define ADI 26
#define TDI 27

#define motorTI 5
#define ATI 28
#define TTI 29


// SERVOS
Servo servoI, servoD, servoP;

// LEDS
#define PW  30
#define CON 31
#define DER 32
#define IZ  33
#define EN  34
#define PIN 35



//  SENSOR DISTANCIA
#define eco 36
#define trig 37

// PARAMETROS DE CONTROL
#define avance    'a'
#define reversa   'r'
#define derecha   'd'
#define izquierda 'i'
#define parada    's'
#define cue       'c'
#define pinza     'p'
#define led       'l'

void setup() {

  Serial.begin(115200);

  //INICIO DE MOTORES
  pinMode(motorDD, OUTPUT);
  pinMode(ADD, OUTPUT);
  pinMode(TDD, OUTPUT);

  pinMode(motorTD, OUTPUT);
  pinMode(ATD, OUTPUT);
  pinMode(TTD, OUTPUT);

  pinMode(motorDI, OUTPUT);
  pinMode(ADI, OUTPUT);
  pinMode(TDI, OUTPUT);

  pinMode(motorTI, OUTPUT);
  pinMode(ATI, OUTPUT);
  pinMode(TTI, OUTPUT);

  //INICIO DE SERVOS
  servoD.attach(6);
  servoI.attach(7);
  servoP.attach(8);

  servoD.write(85);
  servoI.write(95);
  servoP.write(170);



  //INICIO SENSOR
  pinMode(eco, INPUT);
  pinMode(trig, OUTPUT);

  //INICIO LEDS
  pinMode(PW, OUTPUT);
  pinMode(CON, OUTPUT);
  pinMode(DER, OUTPUT);
  pinMode(IZ, OUTPUT);
  pinMode(EN, OUTPUT);
  pinMode(PIN, OUTPUT);
}

String recibido;
char modo;
char parametro[10];
int valor;
int posI, posD;
long duracion, distancia;
void loop() {
  digitalWrite(PW, HIGH);
  if (Serial.available()) {
    recibido = Serial.readString();
    if (recibido.length() > 0 ) {                      //HAND SHAKE
      digitalWrite(CON, HIGH);
      Serial.println("HELLO");

      while (1) {                                   //INICIO DEL PROGRAMA
        if (Serial.available())
        {
          recibido = Serial.readString();
          modo = recibido[0];
          recibido.substring(1).toCharArray(parametro, 10);
          valor = atoi(parametro);
          switch (modo) {

            /*************** CONTROL DE LOS MOTORES   ***********************/
            case parada:                            //PARADA
              digitalWrite(motorDD, LOW);
              digitalWrite(motorTD, LOW);
              digitalWrite(motorDI, LOW);
              digitalWrite(motorTI, LOW);

              digitalWrite(DER, LOW);
              digitalWrite(IZ, LOW);
              break;
            case avance:                             //AVANCE
              digitalWrite(ADD, HIGH);
              digitalWrite(TDD, LOW);
              analogWrite(motorDD, valor);

              digitalWrite(ATD, HIGH);
              digitalWrite(TTD, LOW);
              analogWrite(motorTD, valor);

              digitalWrite(ADI, HIGH);
              digitalWrite(TDI, LOW);
              analogWrite(motorDI, valor);

              digitalWrite(ATI, HIGH);
              digitalWrite(TTI, LOW);
              analogWrite(motorTI, valor);

              digitalWrite(DER, HIGH);
              digitalWrite(IZ, HIGH);
              break;
            case reversa:                             //REVERSA
              digitalWrite(ADD, LOW);
              digitalWrite(TDD, HIGH);
              analogWrite(motorDD, valor);

              digitalWrite(ATD, LOW);
              digitalWrite(TTD, HIGH);
              analogWrite(motorTD, valor);

              digitalWrite(ADI, LOW);
              digitalWrite(TDI, HIGH);
              analogWrite(motorDI, valor);

              digitalWrite(ATI, LOW);
              digitalWrite(TTI, HIGH);
              analogWrite(motorTI, valor);

              digitalWrite(DER, LOW);
              digitalWrite(IZ, LOW);
              break;
            case derecha:                             //DERECHA
              digitalWrite(ADD, LOW);
              digitalWrite(TDD, HIGH);
              analogWrite(motorDD, valor);

              digitalWrite(ATD, LOW);
              digitalWrite(TTD, HIGH);
              analogWrite(motorTD, valor);

              digitalWrite(ADI, HIGH);
              digitalWrite(TDI, LOW);
              analogWrite(motorDI, valor);

              digitalWrite(ATI, HIGH);
              digitalWrite(TTI, LOW);
              analogWrite(motorTI, valor);

              digitalWrite(DER, HIGH);
              digitalWrite(IZ, LOW);
              break;
            case izquierda:                               //IZQUIERDA
              digitalWrite(ADD, HIGH);
              digitalWrite(TDD, LOW);
              digitalWrite(motorDD, valor);

              digitalWrite(ATD, HIGH);
              digitalWrite(TTD, LOW);
              analogWrite(motorTD, valor);

              digitalWrite(ADI, LOW);
              digitalWrite(TDI, HIGH);
              analogWrite(motorDI, valor);

              digitalWrite(ATI, LOW);
              digitalWrite(TTI, HIGH);
              analogWrite(motorTI, valor);

              digitalWrite(DER, LOW);
              digitalWrite(IZ, HIGH);
              break;





            /***************** MOVIMIENTO CUELLO Y PINZA  ********************/
            case cue:


              posI = valor;
              posD = 179 - posI;

              servoI.write(posI);
              servoD.write(posD);



              break;
            case pinza:


              servoP.write(valor);

              break;


            /************* ENCONTRADO **************/
            case led:
              if (valor == 1) {
                digitalWrite(EN, HIGH);
              } else {
                digitalWrite(EN, LOW);
              }
          }
        }
        digitalWrite(trig, LOW);
        delayMicroseconds(4);
        digitalWrite(trig, HIGH);
        delayMicroseconds(10);
        digitalWrite(trig, LOW);

        duracion = pulseIn(eco, HIGH);

        distancia = duracion * 10 / 292 / 2;
        Serial.println(distancia);
      }
    }
  }
}