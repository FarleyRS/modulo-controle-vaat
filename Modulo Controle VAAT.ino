//Classe responsavel por instanciar os motores
class MotorDC {
  int spd = 200, pin1, pin2;

public:
  void Pinout(int in1, int in2) {
    pin1 = in1;
    pin2 = in2;
    pinMode(pin1, OUTPUT);
    pinMode(pin2, OUTPUT);
  }

  void Speed(int in1) {
    spd = in1;
  }

  void Frente() {
    digitalWrite(pin1, spd);
    digitalWrite(pin2, 0);
  }

  void ParaTras() {  // Backward é o método para fazer o motor girar para trás
    digitalWrite(pin1, 0);
    digitalWrite(pin2, spd);
  }
  void Parar() {  // Stop é o metodo para fazer o motor ficar parado.
    digitalWrite(pin1, 0);
    digitalWrite(pin2, 0);
  }
};

//Bibliotecas

#include <SoftwareSerial.h>
#include <TinyGPS.h>

//Pinos do GPS
SoftwareSerial mySerial(10, 11);
TinyGPS gps;

//Variaveis de longitude e latitude do GPS
float flat, flon;
unsigned long age;

//Pinos do Ultrassonico
int triggerPin = 3, echoPin = 2;

//
MotorDC Motor1, Motor2, Motor3, Motor4;



void setup() {
  //Configuração de comunicação serial
  Serial.begin(9600);
  mySerial.begin(9600);
  delay(1000);

  // Set pinos ultrassonico
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);


  Motor1.Pinout(22, 24);
  Motor2.Pinout(26, 28);
  Motor3.Pinout(30, 32);
  Motor4.Pinout(34, 36);
}

void loop() {
  ControleAlt();
}

// Ler os dados do sensor de distancia ultrassonico
long readUltrasonicDistance() {
  // Medindo a distância do obstáculo
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  long distance = duration * 0.034 / 2;

  return distance;
}


//Controle alternativo do veiculo
void ControleAlt() {
  Serial.println("Iniciando controle alternativo");

  if (40 > readUltrasonicDistance()) {
    Serial.println("desviando");
    parar();
    delay(2000);

    girarDireita();
    delay(2000);
    andarParaFrente();
    delay(2000);
    girarEsquerda();
    delay(2000);
    andarParaFrente();
    delay(2000);
    girarDireita();
    delay(2000);

    ControleAlt();
  } else {
    Serial.println("andando para frente");
    andarParaTras();
  }
}


// Método para andar para frente
void andarParaFrente() {
  Motor1.Frente();
  Motor2.Frente();
  Motor3.Frente();
  Motor4.Frente();
}

// Método para girar para a esquerda
void girarEsquerda() {
  Motor1.Frente();
  Motor2.Frente();
  Motor3.ParaTras();
  Motor4.ParaTras();
}

// Método para girar para a direita
void girarDireita() {
  Motor1.ParaTras();
  Motor2.ParaTras();
  Motor3.Frente();
  Motor4.Frente();
}

// Método para parar
void parar() {
  Motor1.Parar();
  Motor2.Parar();
  Motor3.Parar();
  Motor4.Parar();
}

// Método para andar para trás
void andarParaTras() {
  Motor1.ParaTras();
  Motor2.ParaTras();
  Motor3.ParaTras();
  Motor4.ParaTras();
}