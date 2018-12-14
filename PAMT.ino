#include <avr/io.h>             //permite chamar os registradores
#include <avr/interrupt.h>      //permite chamar as interrupção
#include <OneWire.h>            //para temperatura
#include <DallasTemperature.h>  // biblioteca necessária para o ds18b20
#include <SoftwareSerial.h>     //Comunicação serial para a placa
#include <TimerOne.h>           //Temporizador
#include <Wire.h>               //para lcd I2C
#include <LiquidCrystal_I2C.h>  // biblioteca necessária para o display LCD

//PINOS DOS SENSORES
#define TEMP 3                  //Pino temperatura
  float temperatura = 0;        //Estado inicial temperatura 0°C
  boolean tempCheck = false;    //Controle se o valor da temperatura mudou
  float tempLocal=0;            //Armazena valor travado da temperatura

#define TEMPO_BR 1000000         //(1000000 = 1Segundo)
#define CONTA_TICK 2            // 2S de ticks temporizados
  volatile long tik_tak_count = CONTA_TICK;
  volatile bool em_isr_longo = false;

#define DELAY 2000              //Delay geral LCD
  int idelay=0;                 //Controle tempo delay LCD
  boolean initBemVindoLCD=true; //Controle mensagem boas vindas
  String tipoMSG;               //Variável controle tipo de mensagme para LCD
  int voltaLoop=0;              //Variável para identificar quantos volta no loop
  boolean msgBluetooth = false; //Identifica uso da mensagem bluetooth

#define R 4
#define S 5
#define T 6
#define Trst  1 //ms
  boolean FR = false;
  boolean FS = false;
  boolean FT = false;

//PINOS ALARME
#define BZ A2
#define LED A1
  boolean alerta = false;       //Variável de controle estado da atuação
  unsigned int tempoAlerta=0;   //Variável controle tempo atuação

//PINO ATUADOR
#define RELE A0

//DECLARAÇÕES DAS FUNÇÕES
void ligarMotor();
void desligarMotor();
void desligarMotorBT();

// pinos utilizados para a conexão do display 20x4
LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7,3, POSITIVE);

OneWire ourWire(TEMP);                //Configura uma instância onewire
DallasTemperature sensors(&ourWire);  // passa a temperatura para o dallastemperature
SoftwareSerial minhaSoftSerial(7, 8); // pinos RX/TX para o modulo

void setup() {
  cli();                              //desabilita todas as interrupção
  //Configurar o timer 0 para executar uma interrupção ISR(TIMER0_COMPA_vect) para 1ms
  TCCR0A = (1<<WGM01);                //modo CTC
  OCR0A = 250;
  TCCR0B = (1<<CS01)|(1<<CS00);       //Clk/64 = 250kHz
  TIMSK0 = (1<<OCIE0A);               //habilita a interrupção TIMER0_COMPA_vect

  //declaração dos pinos do alerta
  pinMode(BZ, OUTPUT);
  pinMode(LED, OUTPUT);

  //declaação dos pinos do Réle
  pinMode(RELE, OUTPUT);
  digitalWrite(RELE, LOW); //Inicia motor desligado

  //declaração dos pinos dos sensores de fase
  pinMode(R, INPUT);
  pinMode(S, INPUT);
  pinMode(T, INPUT);

  //inicializa o sensor de temperatura
  sensors.begin();
  sensors.setResolution(9);

  // inicia o lcd
  lcd.begin(20,4);
  lcd.backlight();

  //incicialização da porta serial
  Serial.begin(9600); //porta serial (hardware) para monitorar os caracteres enviados

  //incicialização do bluetooth
  minhaSoftSerial.begin(9600);

  Timer1.initialize(TEMPO_BR);
  Timer1.attachInterrupt(tempoISR);

  sei(); //habilita interrupção

  Serial.println("Motor pronto para ligar."); // mensagem de inicializacao

}

void loop() {
 /* 1 - LIGAR e DESLIGAR O MOTOR VIA BLUETOOTH */
  // Serial ativa com caracter para ler
  boolean MSS = minhaSoftSerial.available();

  // Testa se minhaSoftSerial está disponível
  if(MSS){
    //Salva caracter lido
    char caracter = minhaSoftSerial.read();

  // ligando via bluetooth se for A
  if(caracter == 'A'){
    tipoMSG = String("CT");

    // 1.1 testa se a temperatura está OK
    if(tempLocal >= 40){
      tipoMSG = String("MPT");
      desligarMotor(); // Desligar o motor
    }
    // exibe mensagem "checando fases..."
    tipoMSG = String("CFA");
    // 1.2 testa se as fases(RST) estão OK
    if(FR == LOW){
      tipoMSG = String("SFR");
      desligarMotor(); // Desligar o motor
    }

    // Se estiver sem fase S, mostra erro e sai
    if(FS == LOW){
      tipoMSG = String("SFS");
      desligarMotor(); // Desligar o motor
    }

    // Se estiver sem fase T
    if(FT == LOW){
      tipoMSG = String("SFT");
      desligarMotor(); // Desligar o motor
    }

    // Se tudo estiver OK
    if(!alerta){
      tipoMSG = String("LMA");
      ligarMotor(); // Liga motor
      tipoMSG = String("MAIN");
    }
  } //FIM IF

  // desligar o motor via bluetooth se for a
  if(caracter == 'a'){
    tipoMSG = String("DOM");
    desligarMotorBT(); // DESLIGAR o motor
  }

  } //Fim MSS

  /* 2 - VERIFICAR O STATUS RECORRENTE DOS ATUADORES*/
    // 2.1 testa se a temperatura está OK
    if(tempLocal >= 40){
      tipoMSG = String("MPT");
      desligarMotor(); // Desligar o motor
    }

    // 2.2 testa se as fases(RST) estão OK
    if(FR == LOW){
      tipoMSG = String("SFR");
      desligarMotor(); // Desligar o motor
    }

    // Se estiver sem fase S
    if(FS == LOW){
      tipoMSG = String("SFS");
      desligarMotor(); // Desligar o motor
    }

    // Se estiver sem fase T
    if(FT == LOW){
      tipoMSG = String("SFT");
      desligarMotor(); // Desligar o motor
    }

    // Se queda das fases RST juntas
    if(FR==LOW&&FS==LOW&&FT==LOW){
      tipoMSG = String("QRST");
    }

    //Exibe mensagem "MOTOR FUNCIONANDO"
    if(!alerta&&msgBluetooth){
      tipoMSG = String("MAIN");
      msgBluetooth=true;

      // Exibe temperatura atual
      if(voltaLoop % 4 == 0){
        tipoMSG = String("MED");
        voltaLoop=0;
      }
    }
    voltaLoop++;//Variável controle

    //Exibe mensagem bluetooth, se tudo OK
    if(tempLocal<40&&FR==HIGH&&FS==HIGH&&FT==HIGH&&!msgBluetooth){
      alerta=false;
      tipoMSG = String("MPPSL"); //Motor Pronto Para Ser Ligado
    }

  // Chama as mensagens LCD
  mensagensLCD();
  // Chama alerta atuadores
  alertaAtuadores();

} //Fim void loop

ISR(TIMER0_COMPA_vect){ //executa a cada 1ms
  static unsigned int ifase=0;
  tempoAlerta++;        //Variável controle da atuação
  idelay++;             //Variável de controle LCD

  //leitura de falta de fase
  if(ifase++ >= Trst){
    FR = digitalRead(R);
    FS = digitalRead(S);
    FT = digitalRead(T);
    ifase =0;
  }

} //Fim do TIMER0_COMPA_vect

// LOW - DESATRACAR RELÉ
void desligarMotor(){
  digitalWrite(RELE, LOW);
  alerta=true; msgBluetooth=false;
}

// HIGH - ATRACAR RELÉ,
void ligarMotor(){
  digitalWrite(RELE, HIGH);
  alerta=false; msgBluetooth=true;
}

// LOW - DESATRACAR RELÉ. Desliga motor via bluetooth
void desligarMotorBT(){
  digitalWrite(RELE, LOW);
  alerta=false; msgBluetooth=false;
}

// atuação sinal visual(LED) e sonoro(BUZZER)
void alertaAtuadores(){
  if(!alerta){
    digitalWrite(LED, LOW);
    digitalWrite(BZ, LOW);
    tempoAlerta=0;
    return;
  }
  if(tempoAlerta<=3000){
      if(tempoAlerta<=1000){
        digitalWrite(LED, HIGH);
        digitalWrite(BZ, HIGH);
      }
      if(tempoAlerta+500==1500){
        digitalWrite(LED, LOW);
        digitalWrite(BZ, LOW);
      }
  }else{
    tempoAlerta=0;
  }
}

// Mensagens do LCD
void mensagensLCD(){
  // Define tipo mensagen
  if(temperatura > 0 && temperatura != 85){
    tempLocal = temperatura;
    tempCheck = true; //Exibe temperatura
  } else {
    tempCheck = false;//Exibe mensagem "Verificando temp..."
  }

// mensagem de bem-vindo Executa 1 vez
if(initBemVindoLCD){
  lcd.setCursor(0, 0);
  lcd.print("::::PROJETO PAMT::::");
  lcd.setCursor(0, 1);
  lcd.print("    BEM-VINDO AO    ");
  lcd.setCursor(0, 2);
  lcd.print(" PROTETOR AUTOMATICO");
  lcd.setCursor(0, 3);
  lcd.print(" DE MOTOR TRIFASICO ");
  initBemVindoLCD=false;
}
//verifica qual mensagem dever se exibida
if(idelay>=DELAY){
  if(tipoMSG == "CT"){
      lcd.setCursor(0, 0);
      lcd.print("::::PROJETO PAMT::::");
      lcd.setCursor(0, 1);
      lcd.print("--------------------");
      lcd.setCursor(0, 2);
      lcd.print("     CHECANDO...    ");
      lcd.setCursor(0, 3);
      lcd.print("   TEMPERATURA...   ");

    } else if(tipoMSG == "MAIN"){
      lcd.setCursor(0, 0);
      lcd.print("::::PROJETO PAMT::::");
      lcd.setCursor(0, 1);
      lcd.print("ESTADO DO MOTOR:  OK");
      lcd.setCursor(0, 2);
      lcd.print("FASES R/S/T:      OK");
      lcd.setCursor(0, 3);
      lcd.print("TEMPERATURA: ");
        if(tempLocal>0){
          lcd.print(String(tempLocal,0));
          lcd.setCursor(15, 3);
          lcd.write(223); // Exibe símbolo '°'
          lcd.setCursor(16, 3);
          lcd.print("C   ");
        }else{
          lcd.print("AGUARDE");
        }

    } else if(tipoMSG == "MPT"){
      lcd.setCursor(0, 0);
      lcd.print("::::PROJETO PAMT::::");
      lcd.setCursor(0, 1);
      lcd.print(">> ALARME TERMICO <<");
      lcd.setCursor(0, 2);
      lcd.print("MOTOR: DESLIGADO    ");
      lcd.setCursor(0, 3);
      lcd.print("TEMPERATURA: ");
      lcd.print(String(tempLocal,0));
      lcd.setCursor(15, 3);
      lcd.write(223); // Exibe símbolo '°'
      lcd.setCursor(16, 3);
      lcd.print("C   ");


    } else if(tipoMSG == "CFA"){
      lcd.setCursor(0, 0);
      lcd.print("::::PROJETO PAMT::::");
      lcd.setCursor(0, 1);
      lcd.print("  CHECANDO FASES... ");
      lcd.setCursor(0, 2);
      lcd.print("     AGUARDE...     ");
      lcd.setCursor(0, 3);
      lcd.print("    FASES R/S/T     ");

    } else if(tipoMSG == "SFR"){
      lcd.setCursor(0, 0);
      lcd.print("::::PROJETO PAMT::::");
      lcd.setCursor(0, 1);
      lcd.print(">> ALARME DE FASE <<");
      lcd.setCursor(0, 2);
      lcd.print("ESTADO DO MOTOR:    ");
      lcd.setCursor(0, 3);
      lcd.print("DESLIGADO-SEM FASE R");

    } else if(tipoMSG == "SFS"){
      lcd.setCursor(0, 0);
      lcd.print("::::PROJETO PAMT::::");
      lcd.setCursor(0, 1);
      lcd.print(">> ALARME DE FASE <<");
      lcd.setCursor(0, 2);
      lcd.print("ESTADO DO MOTOR:    ");
      lcd.setCursor(0, 3);
      lcd.print("DESLIGADO-SEM FASE S");

    } else if(tipoMSG == "SFT"){
      lcd.setCursor(0, 0);
      lcd.print("::::PROJETO PAMT::::");
      lcd.setCursor(0, 1);
      lcd.print(">> ALARME DE FASE <<");
      lcd.setCursor(0, 2);
      lcd.print("ESTADO DO MOTOR:    ");
      lcd.setCursor(0, 3);
      lcd.print("DESLIGADO-SEM FASE T");

    } else if(tipoMSG == "LMA"){
      lcd.setCursor(0, 0);
      lcd.print("::::PROJETO PAMT::::");
      lcd.setCursor(0, 1);
      lcd.print("--------------------");
      lcd.setCursor(0, 2);
      lcd.print(" LIGANDO O MOTOR... ");
      lcd.setCursor(0, 3);
      lcd.print("     AGUARDE...     ");

    } else if(tipoMSG == "DOM"){
      lcd.setCursor(0, 0);
      lcd.print("::::PROJETO PAMT::::");
      lcd.setCursor(0, 1);
      lcd.print("--------------------");
      lcd.setCursor(0, 2);
      lcd.print("DESLIGANDO O MOTOR..");
      lcd.setCursor(0, 3);
      lcd.print("     AGUARDE...     ");

    } else if(tipoMSG == "MPPSL"){
      lcd.setCursor(0, 0);
      lcd.print("::::PROJETO PAMT::::");
      lcd.setCursor(0, 1);
      lcd.print("--------------------");
      lcd.setCursor(0, 2);
      lcd.print("  MOTOR PRONTO PARA ");
      lcd.setCursor(0, 3);
      lcd.print("     SER LIGADO     ");

    } else if(tipoMSG == "MED"){
      lcd.setCursor(0, 0);
      lcd.print("::::PROJETO PAMT::::");
      lcd.setCursor(0, 1);
      lcd.print("MANTENHA SEMPRE EM  ");
      lcd.setCursor(0, 2);
      lcd.print("DIA A MANUTENCAO DAS");
      lcd.setCursor(0, 3);
      lcd.print("MAQUINAS E MOTORES  ");

    } else if(tipoMSG == "QRST"){
      lcd.setCursor(0, 0);
      lcd.print("::::PROJETO PAMT::::");
      lcd.setCursor(0, 1);
      lcd.print(">> ALARME DE FASES <<");
      lcd.setCursor(0, 2);
      lcd.print("   MOTOR DESLIGADO  ");
      lcd.setCursor(0, 3);
      lcd.print("   SEM FASES R/S/T  ");

    }else{
      lcd.setCursor(0, 0);
      lcd.print("::::PROJETO PAMT::::");
      lcd.setCursor(0, 1);
      lcd.print("--------------------");
      lcd.setCursor(0, 2);
      lcd.print(">>>>>>>>ERRO<<<<<<<<");
      lcd.setCursor(0, 3);
      lcd.print("   CODIGO: LCD#01   ");
    }
    idelay=0;//Reset
  }
}
// tempo para temperatura
void tempoISR()
{
    if (!(--tik_tak_count))        // Conta até 2S
    {
      tik_tak_count = CONTA_TICK; // Recarrega
      tick_2s_isr();              // Chama a rotina de 2S
    }
}
// --------------------------
// tick_2s_isr() Rotina de 2s
// Chamar toda vez que a contagem chegar aos 2S
// --------------------------
void tick_2s_isr()
{
    temperatura =  sensors.getTempCByIndex(0); //Deve ser colocado no início sempre!!
    if (em_isr_longo)                          // Impedir a reentrada deste código
    {
      return;
    }
    em_isr_longo = true;                      // Indica que está no isr longo
      sensors.requestTemperatures();
    em_isr_longo = false;                     // Indica a saída do isr longo
}
