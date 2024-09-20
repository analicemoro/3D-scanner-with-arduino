#include <MobaTools.h>

// Define os pinos de controle do motor
#define STEP_PIN 8
#define DIR_PIN 9
//#define ENABLE_PIN 4

// Define os parâmetros do motor
#define STEPS_PER_REVOLUTION 200   // Número de passos por revolução do motor

// Cria um objeto Stepper4 para controlar o motor
MoToStepper stepper(STEPS_PER_REVOLUTION, STEPDIR);

// Variáveis para armazenar os comandos
int speed = 0;
int steps = 0;
bool newCommand = false;

void setup() {
  // Inicializa a comunicação serial
  Serial.begin(9600);
  while (!Serial) {
    ; // Espera pela inicialização da porta serial
  }

  // Define os pinos do motor
  stepper.attach(STEP_PIN, DIR_PIN);
  //stepper.attachEnable(ENABLE_PIN, 10, LOW); // Se precisar de um pino de habilitação
  stepper.setSpeed(1000);    // Velocidade inicial
  stepper.setRampLen(100);   // Comprimento da rampa de aceleração/desaceleração

  Serial.println("Setup completo. Envie comandos no formato: velocidade,passos");
}

void loop() {
  // Verifica se há nova entrada serial
  if (Serial.available() > 0) {
    // Lê a entrada do usuário
    String input = Serial.readStringUntil('\n');
    input.trim();  // Remove espaços em branco

    // Divide a entrada em velocidade e passos
    int separatorIndex = input.indexOf(',');
    if (separatorIndex != -1) {
      String speedString = input.substring(0, separatorIndex);
      String stepsString = input.substring(separatorIndex + 1);

      // Converte os valores para inteiros
      speed = speedString.toInt();
      steps = stepsString.toInt();

      // Debugging
      Serial.print("Speed: ");
      Serial.println(speed);
      Serial.print("Steps: ");
      Serial.println(steps);

      // Define a velocidade do motor
      stepper.setSpeed(speed * STEPS_PER_REVOLUTION / 60);  // Converte RPM para passos por segundo

      // Marca que um novo comando foi recebido
      newCommand = true;
    } else {
      Serial.println("Entrada inválida. Use o formato: velocidade,passos");
    }
  }

  // Executa o comando se houver um novo
  if (newCommand && !stepper.moving()) {
    stepper.move(steps);  // Move passos para frente
    Serial.println("Movendo.");

    // Inicia o movimento
    stepper.doSteps(steps);

    // Marca que o comando está em execução
    newCommand = false;
  }

  // Adicione um pequeno delay para evitar travamento do loop
  delay(10);
}
