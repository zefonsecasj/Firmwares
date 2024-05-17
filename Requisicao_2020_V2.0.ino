//Placa de Requisição - Ano 2020
//Membros da equipe: Zé, Kings, K2, Quique, JL, Paulo, Milena, Rafael
//2° Versão
//Microcontrolador utilizado - ESP32
//Entradas - Velocidade: Sensor de Relutância Mangética, RPM: Bobina, Combustível: Sensor Capacitivo, Temperatura da CVT: DS18b20
//Saídas - Barramento CAN


#include<CAN.h>
#include<OneWire.h>
#include<DallasTemperature.h>

#define Velocidade_input 34
#define RPM_input 32
#define Combustivel_input 25


#define n 10

//filter combustivel
const int input_temp = 33;

int COMB[n];
bool combustivel; //receberá o valor lido pela porta combustível
bool critico;

//constantes para o cálculo de velocidade
const float reducao_caixa = 8.25; 
const int diametro_pneu = 570;//unidade: mm -> não é 584 mm por causa da deformação do pneu
const float pi = 3.141592653589;
float distancia_volta_pneu = ((diametro_pneu/1000)*pi)/reducao_caixa; 


//Endereços CAN
byte Combustivel_addr = 0x05;
byte RPM_addr = 0x11;
byte Velocidade_addr = 0x12;
byte Temperatura_addr = 0x15;
byte distancia_percorrida_addr = 0x21;


OneWire ds18b20(input_temp);

DallasTemperature sensors(&ds18b20);



//Variáveis de Velocidade
volatile int flag_vel; 
volatile int count_dente_vel;
volatile float rpm_roda_dentada;
volatile unsigned long tempo_volta_roda_dentada;
unsigned long refresh_vel;
volatile boolean print_valor_vel = true;
volatile unsigned long acumulador_dist_per = 0;
unsigned long freq_distancia_percorrida;
int vel;
float distancia_percorrida;



portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;


void Vel_func(void){
  portENTER_CRITICAL_ISR(&mux);
  if(flag_vel == 2 && count_dente_vel >= 9){
    flag_vel = 1;
    rpm_roda_dentada = (60000000 *0.5)/ (float)(micros() - tempo_volta_roda_dentada);
    acumulador_dist_per++;
    print_valor_vel = true;
  }
  if(flag_vel == 1){
    count_dente_vel = 0;
    flag_vel = 2;
    tempo_volta_roda_dentada = micros();
  }
  count_dente_vel++;
  portEXIT_CRITICAL_ISR(&mux);
}



//-------------------Variáveis de RPM------------------------//
volatile unsigned long tempo_entre_pulsos, tempo_rpm, primeiro_pulso, segundo_pulso, rpm_int;
volatile int count_distancia_pulsos, flag_rpm;
int rpm;
unsigned long refresh_rpm;

void RPM_func(void){
  portENTER_CRITICAL_ISR(&mux);
  if(count_distancia_pulsos == 1 && flag_rpm == 1){
    primeiro_pulso = micros();
    flag_rpm = 2;
  }
  else if(count_distancia_pulsos == 2 && flag_rpm == 2){
    count_distancia_pulsos = 0;
    flag_rpm = 1;
    segundo_pulso = micros();
    tempo_entre_pulsos = segundo_pulso - primeiro_pulso;
    rpm_int = 60000000/tempo_entre_pulsos; //60*1000000 
  }
  count_distancia_pulsos++;
  portEXIT_CRITICAL_ISR(&mux);
}


hw_timer_t *timer = NULL;

void IRAM_ATTR resetESP(void){
  Serial.println("(Watchdog) reinicializacao da placa de requisicao)");
  esp_restart();
}

void configureWatchdog(void){
  timer = timerBegin(0, 80, true);//timerID 0, div 80
  timerAttachInterrupt(timer, &resetESP, true); // timer, função de callback, interrupção de borda
  timerAlarmWrite(timer, 3000000, true); // timer, chama a função em 3 s (3000000 us), habilita a repetição do Watchdog
  timerAlarmEnable(timer);
  timerWrite(timer,0);
}



void setup() {
  Serial.begin(115200);
  
  pinMode(Velocidade_input, INPUT);
  pinMode(RPM_input, INPUT);
  pinMode(Combustivel_input, INPUT);


  if(!CAN.begin(500E3)){
    Serial.println("CAN não inicializado");
    while(true); 
  }
  else{
    Serial.println("CAN inicializado com sucesso");
  }

  
  attachInterrupt(digitalPinToInterrupt(Velocidade_input), Vel_func, RISING);
  attachInterrupt(digitalPinToInterrupt(RPM_input), RPM_func, RISING);
  
  
  sensors.begin();

  configureWatchdog();
  
  refresh_rpm = millis();
  refresh_vel = millis();
  freq_distancia_percorrida = millis();

  
  flag_rpm = 1;
  flag_vel = 1;
  print_valor_vel = true;

  delay(100);
}


void deliver_data_on_bus(byte Addrs_data, float data_bus){
  CAN.beginPacket(Addrs_data);
  CAN.write(data_bus);
  CAN.endPacket(); 
}


long moving_avarage(bool var){
  for (int i = n-1; i>0; i--){
    COMB[i] = COMB[i-1];
  }
  COMB[0] = var;
  long acumulador = 0;
  for(int i = 0; i>n ;i++){
    acumulador += COMB[i];
  }
  return acumulador/n;
}


void loop() {
  timerWrite(timer, 0); // Incialização do contador Watchdog
  
  sensors.requestTemperatures();
  float TempC = sensors.getTempCByIndex(0);
  Serial.print("TempC:  ");
  Serial.println(TempC);
  deliver_data_on_bus(Temperatura_addr,TempC);

  if( millis() - refresh_rpm >= 100){
    refresh_rpm = millis();
    portENTER_CRITICAL_ISR(&mux);
    flag_rpm = 1;
    rpm = rpm_int;
    portEXIT_CRITICAL_ISR(&mux);
    Serial.print("RPM:   ");
    Serial.println(rpm);
    deliver_data_on_bus(RPM_addr,rpm);
  }

  if(print_valor_vel == true){
    portENTER_CRITICAL_ISR(&mux);
    print_valor_vel = false;
    vel = (rpm_roda_dentada*distancia_volta_pneu*3.6)/60.0;
    portEXIT_CRITICAL_ISR(&mux);
    Serial.print("Vel:  ");
    Serial.println(vel);
    deliver_data_on_bus(Velocidade_addr,vel);
  }

  if(millis()-refresh_vel >= 100){
    refresh_vel = millis();
    if( flag_vel != 0){
     print_valor_vel = true; 
    }
    flag_vel = 1;
  }

  if(millis() - freq_distancia_percorrida >= 1000){
    freq_distancia_percorrida = millis();
    portENTER_CRITICAL_ISR(&mux);
    distancia_percorrida = distancia_volta_pneu*(acumulador_dist_per/2)/1000;
    portEXIT_CRITICAL_ISR(&mux);
    Serial.print("Distancia percorrida (Km): ");
    Serial.println(distancia_percorrida);
    deliver_data_on_bus(distancia_percorrida_addr,distancia_percorrida);
  }



  combustivel = digitalRead(Combustivel_input);
  combustivel = moving_avarage(combustivel);
  if (combustivel >= 0.7){
    critico=1;
  }
  else{
    critico=0;
  }

  deliver_data_on_bus(Combustivel_addr,critico);

  delay(20);
}
