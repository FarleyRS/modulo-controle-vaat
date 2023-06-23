// Classe responsável por instanciar os motores
class MotorDC {
  int speed = 200;
  int pin1, pin2;

public:
  void Pinout(int in1, int in2) {
    pin1 = in1;
    pin2 = in2;
    pinMode(pin1, OUTPUT);
    pinMode(pin2, OUTPUT);
  }

  void Speed(int spd) {
    speed = spd;
  }

  void Frente() {
    digitalWrite(pin1, LOW);
    analogWrite(pin2, speed);
  }

  void ParaTras() {
    analogWrite(pin1, speed);
    digitalWrite(pin2, LOW);
  }

  void Parar() {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
  }
};

// Pinos do Ultrassônico
int triggerPin = 3;
int echoPin = 2;

MotorDC Motor1, Motor2, Motor3, Motor4;

void setup() {
  // Configuração de comunicação serial
  Serial.begin(9600);
  mySerial.begin(9600);
  delay(1000);

  // Configuração dos pinos do ultrassônico
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Setando os pinos de cada motor
  Motor1.Pinout(22, 24);
  Motor2.Pinout(26, 28);
  Motor3.Pinout(30, 32);
  Motor4.Pinout(34, 36);
}

void loop() {
  ControleAlt();
}

// Ler os dados do sensor de distância ultrassônico
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

// Controle alternativo do veículo
void ControleAlt() {
  Serial.println("Iniciando controle");

  if (readUltrasonicDistance() < 40) {
    Serial.println("Desviando");
    Parar();
    delay(1000);

    GirarDireita();
    delay(2000);
    AndarParaFrente();
    delay(2000);
    GirarEsquerda();
    delay(2000);
    AndarParaFrente();
    delay(2000);
    GirarDireita();
    delay(2000);

    ControleAlt();
  } else {
    Serial.println("Andando para frente");
    AndarParaFrente();
  }
}

// Método para andar para frente
void AndarParaFrente() {
  Motor1.Frente();
  Motor2.Frente();
  Motor3.Frente();
  Motor4.Frente();
}

// Método para girar para a esquerda
void GirarEsquerda() {
  Motor1.Frente();
  Motor2.Frente();
  Motor3.ParaTras();
  Motor4.ParaTras();
}

// Método para girar para a direita
void GirarDireita() {
  Motor1.ParaTras();
  Motor2.ParaTras();
  Motor3.Frente();
  Motor4.Frente();
}

// Método para parar
void Parar() {
  Motor1.Parar();
  Motor2.Parar();
  Motor3.Parar();
  Motor4.Parar();
}
