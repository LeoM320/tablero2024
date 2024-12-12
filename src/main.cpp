/**
 * @file main.cpp
 * @author MEOLANS, Leandro Tomás (meolansleandrotomas@outlook.com)
 * @brief Firmware para tablero de voley
 * @version 0.1
 * @date 05-12-2024
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <Arduino.h>
#include <SoftwareSerial.h>
#define VERSION "05-12-2024-TABLERO-V0.1"

/*
* Definición de Hardware
*/

/**
 * @brief Pines segmentos
 */
typedef struct {
  const uint8_t A = 2;
  const uint8_t B = 3;
  const uint8_t C = 4;
  const uint8_t D = 5;
  const uint8_t E = 6;
  const uint8_t F = 7;
  const uint8_t G = 8;
}s_SEG;

/**
 * @brief Anodos display
 * P1 - Decena local
 * P2 - Unidad local
 * P3 - Decena visitante
 * P4 - Unidad visitante
 * S1 - Set local
 * S2 - Set general
 * S3 - Set visitante
 */
typedef struct {
  const uint8_t P1 = 12;
  const uint8_t P2 = 11;
  const uint8_t P3 = 10;
  const uint8_t P4 = 9;
  const uint8_t S1 = 13;
  const uint8_t S2 = A0;
  const uint8_t S3 = A1;
}s_ANODO;

/**
 * @brief BCD CD4511
 */
typedef struct {
  const uint8_t A = A2;
  const uint8_t B = A3;
  const uint8_t C = A4;
}s_CD4511;

/**
 * @brief Pines relay
 * SIR - Sirena
 * SAQ - Saque
 */
typedef struct {
  const uint8_t SIR = A5;
  const uint8_t SAQ = 1;
}s_RELAY;

/**
 * @brief Estructura de definición de Hardware
 */
struct {
  const s_SEG SEG;
  const s_ANODO ANODO;
  const s_CD4511 CD4511;
  const s_RELAY RELAY;
  const uint8_t RX_RS485 = 0;
}PIN;

/**
 * @brief Máquina de estados
 */
typedef enum {
  ESTADO_1_CONFIG=0,
  ESTADO_2_JUEGO,
  ESTADO_3_PARPADEO,
  ESTADO_4_ESTATICO,
  ESTADO_5_FINALIZADO,
  ESTADO_SIZE
}e_ESTADO;

/**
 * @brief Caracteres empleados en comunicación serial
 */
typedef struct{
  const uint8_t SPL_ = 'a'; //Sumar punto local
  const uint8_t RPL = 'b'; //Restar punto local
  const uint8_t SPV = 'c'; //Sumar punto visitante
  const uint8_t RPV = 'd'; //Restar punto visitante
  const uint8_t SIR_OFF = 'e'; //Sirena OFF
  const uint8_t SIR_ON = 'f'; //Sirena ON
  const uint8_t SSL = 'g'; //Setear bits.saque local
  const uint8_t SSV = 'h'; //Setear bits.saque visitante
  const uint8_t INV = 'i'; //Invertir
  const uint8_t RST = 'j'; //Reset
  const uint8_t CON = 'k'; //Continuar
}s_CH;

/**
 * @brief Constantes de configuración
 * MICRODELAY_MULTIPLEXADO - Tiempo en microsegundos entre etapas de multiplexación
 * Se realiza un barrido simultaneo en los segmentos principales
 * y los secundarios. Estos se encienden, se apagan y así sucesivamente
 * (Por capacidad del micro probablemente no se alcance y se ejecute a tiempo
 * de ejecución de programa, debería comprobarse los tiempos mediante una
 * salida digital y oscilloscopio)
 * DELAY_PARPADEO - Tiempo en ms entre etapas de parpadeo.
 * LIMITE_ITERAR - Cantidad de iteraciones al parpadear. Mínimo 1
 */
struct {
  const s_CH CH;
  const uint16_t MICRODELAY_MULTIPLEXADO = 200;
  const uint16_t DELAY_PARPADEO = 1000;
  const uint8_t LIMITE_ITERAR = 3;
}CST;

e_ESTADO ESTADO = ESTADO_1_CONFIG;

//Variables de tiempo en ms
unsigned long tiempoMultiplexado=0;
unsigned long tiempoParpadeo=0;

/**
 * @brief Estructura de bits para control de funciones
 */
typedef struct {
  uint8_t multiplexado:1;
  uint8_t invertir:1;
  uint8_t reset:1;
  uint8_t continuar:1;
  uint8_t saque:1;
  uint8_t sirena:1;
  uint8_t ganador:1;
  uint8_t b7:1;
}s_bits;

s_bits bits={0,0,0,0,1,0,0,0};

struct {
  int8_t local=0;
  int8_t visitante=0;
}punto;

struct {
  int8_t local=0;
  int8_t visitante=0;
  int8_t general=0;
}set;

uint8_t ch=0; //Caracter recibido
uint8_t etapaPuntos=0; //Contador circular multiplexacion [0-3]
uint8_t etapaSets=0; //Contador circular multiplexacion [0-2]
uint8_t digitos[4]={0}; //Dl, Ul, Dv, Uv - Decenas y unidades
uint8_t sets[3]={0}; //Valor de sets
uint8_t iterar=0; //Contador de iteración

SoftwareSerial mySerial(0, A6); // RX, TX

uint8_t ObtenerDecena(uint8_t numero) {
    return numero / 10;
}

uint8_t ObtenerUnidad(uint8_t numero) {
    return numero % 10;
}

void BCD(uint8_t valor) {
  digitalWrite(PIN.CD4511.A, (valor & 0x01) >> 0);
  digitalWrite(PIN.CD4511.B, (valor & 0x02) >> 1);
  digitalWrite(PIN.CD4511.C, (valor & 0x04) >> 2);
}

const uint8_t segmentos[12] = {
  0b1111110, //0
  0b0110000, //1
  0b1101101, //2
  0b1111001, //3
  0b0110011, //4
  0b1011011, //5
  0b1011111, //6
  0b1110000, //7
  0b1111111, //8
  0b1111011  //9
};

void SetearDigitoPuntos(uint8_t numero,uint8_t anodo) {
  //Entre 0 y 9
  /**
   * @brief Enciende los segmentos segun los bits
   * que corresponden al numero
   */
  uint8_t segmentosActivos = segmentos[numero];
  digitalWrite(PIN.SEG.A, (segmentosActivos & 0x40) >> 6);
  digitalWrite(PIN.SEG.B, (segmentosActivos & 0x20) >> 5);
  digitalWrite(PIN.SEG.C, (segmentosActivos & 0x10) >> 4);
  digitalWrite(PIN.SEG.D, (segmentosActivos & 0x08) >> 3);
  digitalWrite(PIN.SEG.E, (segmentosActivos & 0x04) >> 2);
  digitalWrite(PIN.SEG.F, (segmentosActivos & 0x02) >> 1);
  digitalWrite(PIN.SEG.G, (segmentosActivos & 0x01) >> 0);

  /**
   * @brief Estado apagado en parpadeo
   */
  if(ESTADO==ESTADO_3_PARPADEO){
    digitalWrite(PIN.ANODO.P1, LOW);
    digitalWrite(PIN.ANODO.P2, LOW);
    digitalWrite(PIN.ANODO.P3, LOW);
    digitalWrite(PIN.ANODO.P4, LOW);
  }else{
    //Entre 0 y 3
    /**
     * @brief Enciende el anodo correspondiente
     * y apaga los demas. Se usa un bit desplazado
     * el valor del anodo
     */
    uint8_t mask = 0x08 >> anodo;
    digitalWrite(PIN.ANODO.P1, ((mask & 0x08) >> 3));
    digitalWrite(PIN.ANODO.P2, ((mask & 0x04) >> 2));
    digitalWrite(PIN.ANODO.P3, ((mask & 0x02) >> 1));
    digitalWrite(PIN.ANODO.P4, ((mask & 0x01) >> 0));
  }
}

void SetearDigitoSets(uint8_t numero,uint8_t anodo) { //Chequear
  BCD(numero);
  
  /**
   * @brief Estado apagado en parpadeo
   */
  if(ESTADO==ESTADO_3_PARPADEO){
    digitalWrite(PIN.ANODO.S1, LOW);
    digitalWrite(PIN.ANODO.S2, LOW);
    digitalWrite(PIN.ANODO.S3, LOW);
  }else{
    //Entre 0 y 3
    /**
     * @brief Enciende el anodo correspondiente
     * y apaga los demas. Se usa un bit desplazado
     * el valor del anodo
     */
    uint8_t mask = 0x04 >> anodo;
    digitalWrite(PIN.ANODO.S1, ((mask & 0x04) >> 2));
    digitalWrite(PIN.ANODO.S2, ((mask & 0x02) >> 1));
    digitalWrite(PIN.ANODO.S3, ((mask & 0x01) >> 0));
  }
}

void Multiplexar() {
  if(micros() >= tiempoMultiplexado){
    tiempoMultiplexado = micros() + CST.MICRODELAY_MULTIPLEXADO;

    digitos[0] = ObtenerDecena(punto.local);
    digitos[1] = ObtenerUnidad(punto.local);
    digitos[2] = ObtenerDecena(punto.visitante);
    digitos[3] = ObtenerUnidad(punto.visitante);
    sets[0] = set.local;
    sets[1] = set.general;
    sets[2] = set.visitante;

    /**
     * @brief Se avanza alternadamente entre encendido y apagado
     * Se avanza la etapa de multiplexado y se configuran los
     * pines necesarios para visualizar el numero
     */
    if(!bits.multiplexado){
      SetearDigitoPuntos(digitos[etapaPuntos],etapaPuntos);
      SetearDigitoSets(sets[etapaSets],etapaSets);
      etapaPuntos++;
      etapaSets++;
      if(etapaPuntos>=4){
        etapaPuntos=0;
      }
      if(etapaSets>=3){
        etapaSets=0;
      }
    }else{
      digitalWrite(PIN.ANODO.P1,LOW);
      digitalWrite(PIN.ANODO.P2,LOW);
      digitalWrite(PIN.ANODO.P3,LOW);
      digitalWrite(PIN.ANODO.P4,LOW);
      digitalWrite(PIN.ANODO.S1,LOW);
      digitalWrite(PIN.ANODO.S2,LOW);
      digitalWrite(PIN.ANODO.S3,LOW);
    }
    bits.multiplexado=!bits.multiplexado;
  }
}

void Invertir(){
  int8_t aux = punto.local;
  punto.local = punto.visitante;
  punto.visitante = aux;

  aux = set.local;
  set.local = set.visitante;
  set.visitante = aux;

  bits.saque=!bits.saque;
  digitalWrite(PIN.RELAY.SAQ,bits.saque);
}

void LimitarVariable(int8_t *var,uint8_t max,uint8_t min){
  if(*var>max){
    *var=max;
  }else if(*var<min){
    *var=min;
  }
}

void Recibir(){
  uint8_t ch = mySerial.read();
  /**
   * @brief Se configuran los bits segun el caracter
   * recibido y la etapa actual.
   * Por lo tanto habilita las acciones realizables
   * en cada estado
   */
  switch(ESTADO){
    case ESTADO_1_CONFIG:
      ch == CST.CH.SPL_ ? set.local++ : 0;
      ch == CST.CH.SPV ? set.visitante++ : 0;
      ch == CST.CH.RPL ? set.local-- :
      ch == CST.CH.RPV ? set.visitante-- :
      ch == CST.CH.SIR_OFF ? bits.sirena=0 :
      ch == CST.CH.SIR_ON ? bits.sirena=1 :
      ch == CST.CH.SSL ? bits.saque=1 :
      ch == CST.CH.SSV ? bits.saque=0 :
      ch == CST.CH.INV ? bits.invertir=1 :
      ch == CST.CH.RST ? bits.reset=1 :
      ch == CST.CH.CON ? bits.continuar=1 : 0;
      break;
    case ESTADO_2_JUEGO:
      ch == CST.CH.SPL_ ? punto.local++ : 0;
      ch == CST.CH.SPL_ ? bits.saque=1 : 0;
      ch == CST.CH.SPV ? punto.visitante++ : 0;
      ch == CST.CH.SPV ? bits.saque=0 : 0;
      ch == CST.CH.RPL ? punto.local-- :
      ch == CST.CH.RPV ? punto.visitante-- :
      ch == CST.CH.SIR_OFF ? bits.sirena=0 :
      ch == CST.CH.SIR_ON ? bits.sirena=1 :
      ch == CST.CH.SSL ? bits.saque=1 :
      ch == CST.CH.SSV ? bits.saque=0 :
      ch == CST.CH.INV ? bits.invertir=1 :
      ch == CST.CH.RST ? bits.reset=1 : 0;
      break;
    case ESTADO_3_PARPADEO:
      break;
    case ESTADO_4_ESTATICO:
      break;
    case ESTADO_5_FINALIZADO:
      ch == CST.CH.RST ? bits.reset=1 :
      ch == CST.CH.CON ? bits.continuar=1 : 0;
      break;
    default:
    //ERROR
      break;
  }

  LimitarVariable(&punto.local,99,0);
  LimitarVariable(&punto.visitante,99,0);
  LimitarVariable(&set.local,3,0);
  LimitarVariable(&set.general,5,0);
  LimitarVariable(&set.visitante,3,0);

  if(ch == CST.CH.SIR_OFF || ch == CST.CH.SIR_ON){
    digitalWrite(PIN.RELAY.SIR,bits.sirena);
  }
  if(ch == CST.CH.SSL || ch == CST.CH.SSV || ch == CST.CH.SPL_ || ch== CST.CH.SPV){
    digitalWrite(PIN.RELAY.SAQ,bits.saque);
  }

  if(bits.invertir){
    Invertir();
    bits.invertir=0;
  }else if(bits.reset){
    asm("jmp 0x0000");
    bits.reset=0;
  }
}

void setup() {
  pinMode(PIN.SEG.A,OUTPUT);
  pinMode(PIN.SEG.B,OUTPUT);
  pinMode(PIN.SEG.C,OUTPUT);
  pinMode(PIN.SEG.D,OUTPUT);
  pinMode(PIN.SEG.E,OUTPUT);
  pinMode(PIN.SEG.F,OUTPUT);
  pinMode(PIN.SEG.G,OUTPUT);

  pinMode(PIN.ANODO.P1,OUTPUT);
  pinMode(PIN.ANODO.P2,OUTPUT);
  pinMode(PIN.ANODO.P3,OUTPUT);
  pinMode(PIN.ANODO.P4,OUTPUT);
  pinMode(PIN.ANODO.S1,OUTPUT);
  pinMode(PIN.ANODO.S2,OUTPUT);
  pinMode(PIN.ANODO.S3,OUTPUT);

  pinMode(PIN.CD4511.A,OUTPUT);
  pinMode(PIN.CD4511.B,OUTPUT);
  pinMode(PIN.CD4511.C,OUTPUT);

  pinMode(PIN.RELAY.SAQ,OUTPUT);
  pinMode(PIN.RELAY.SIR,OUTPUT);

  mySerial.begin(9600);
  Serial.println(VERSION);
  digitalWrite(PIN.RELAY.SAQ,HIGH);
}

void Estado_1_Config(){
  /**
   * @brief Al recibirse los caracteres inmediatamente se
   * cambian los valores de sets. Aquí se cambia de estado
   * al pulsarse continuar de cumplirse que ningun equipo
   * tenga 3 sets
   */
  if(bits.continuar){
    if(set.local!=3&&set.visitante!=3){
      ESTADO=ESTADO_2_JUEGO;
      set.general=set.local+set.visitante+1;
    }
    bits.continuar=0;
  }
}

void Estado_2_Juego() {
  /**
   * @brief Todos los sets se juegan a 25 puntos o más
   * con diferencia de 2. Solo el quinto set se juega a
   * 15 con diferencia de 2.
   * Ganará el equipo que logre anotar 3 sets
   */
  uint8_t puntosMinimos = (set.general != 5) ? 25 : 15;
  if (punto.local >= puntosMinimos && (punto.local - punto.visitante) >= 2) {
    //GANADOR LOCAL
    set.local++;
    bits.ganador = 0;
    tiempoParpadeo = millis() + CST.DELAY_PARPADEO;
    ESTADO = ESTADO_3_PARPADEO;
  } else if (punto.visitante >= puntosMinimos && (punto.visitante - punto.local) >= 2) {
    //GANADOR VISITANTE
    set.visitante++;
    bits.ganador = 1;
    tiempoParpadeo = millis() + CST.DELAY_PARPADEO;
    ESTADO = ESTADO_3_PARPADEO;
  }
}

void Estado_3_Parpadeo(){
  /**
   * @brief Tras tiempoParpadeo se cambiará alternadamente
   * entre los estados 3 y 4
   */
  if(millis()>=tiempoParpadeo){
    tiempoParpadeo=millis()+CST.DELAY_PARPADEO;
    iterar++;
    bits.sirena=1;
    digitalWrite(PIN.RELAY.SIR,bits.sirena);
    ESTADO=ESTADO_4_ESTATICO;
  }
}

void Estado_4_Estatico(){
  /**
   * @brief Tras tiempoParpadeo se cambiará alternadamente
   * entre los estados 3 y 4. Hasta que se alcance el limite
   * de iteraciones
   */
  if(millis()>=tiempoParpadeo){
    tiempoParpadeo=millis()+CST.DELAY_PARPADEO;
    iterar++;
    bits.sirena=0;
    digitalWrite(PIN.RELAY.SIR,bits.sirena);
    ESTADO=ESTADO_3_PARPADEO;
  }
  if(iterar>=2*CST.LIMITE_ITERAR){
    bits.sirena=0;
    digitalWrite(PIN.RELAY.SIR,bits.sirena);
    ESTADO=ESTADO_5_FINALIZADO;
  }
}

void Estado_5_Finalizado(){
  /**
   * @brief Se quedará estático y avanzará de etapa
   * Al presionarse continuar
   */
  if(bits.continuar&&set.local<=2&&set.visitante<=2){
    ESTADO=ESTADO_2_JUEGO;
    set.general++;
    punto.local=0;
    punto.visitante=0;
    bits.continuar=0;
    iterar=0;
  }
}

void (*ptr_ESTADO[])() = {Estado_1_Config, Estado_2_Juego, Estado_3_Parpadeo, Estado_4_Estatico, Estado_5_Finalizado};

void loop() {
  Multiplexar();
  if(mySerial.available()>0){
    Recibir();
  }
  /**
   * @brief Se llamará mediante el puntero a función a la
   * función correspondiente al estado actual
   */
  ptr_ESTADO[ESTADO]();
}
