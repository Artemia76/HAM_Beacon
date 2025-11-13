/* 
-------------------------------------------------------------------------------
 Programme pour envoyer un message sous forme de CW ou de son CW pour une balise radio
 F5BU, Jean-Paul Gendner JPG 2020-12

 Point = 1 (+ 1 silence)
 Trait = 3 (+ 1 silence)
 Entre signes = 1 silence (géré après Trait et Point)
 Entre caractères = 3=1+2 silences
 Entre mots (espace) = 7=1+6 silences
-------------------------------------------------------------------------------
*/
#include <Adafruit_BMP280.h>

//Variable pour la communication avec le capteur de pression BMP280
Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();
sensors_event_t pressure_event;

#define Debug 1           // 1 pour affichage de messages sur le Moniteur série

// Ports
#define CW 13             // CW
#define PTT 5             // PTT

// Temporisations (msec)
unsigned long Duree_Point = 100;    // Durée du point en ms (100 -> 11.5 mots/min)
unsigned long TA_MSG = 15000;       // période de répétition du message = nb WDT délais
unsigned long TA_Ini = 2000;        // Temps d'attente initial en ms
unsigned long TA_CW = 5000;         // Temps d'attente entre PTT et CW en ms
unsigned long TA_RX = 10000;         // Temps d'attente entre CW et RX

bool Press_F = true;                // 1 envoi de la pression, 0 pas d'envoi
bool TX_ON = false;                 // 0 TX coupé entre envois messages, 1 TX toujours en émission

bool Temp_F = true;                 // 1 envoi de la température, 0 pas d'envoi
double temp_maxraw = 1023;          // Valeur Max de l'ADC : par exemple 1023 == 5V
double temp_minraw = 0;           // Valeur Mini de l'ADC : par exemple 205 == 1V
double temp_maxEU = 50;            // Valeur de l'échelle maxi : par exemple 100 °C
double temp_minEU = -20;              // Valeur de l'échelle mini : par exemple 0 °C

int iTab;
byte Code;
int iB;
unsigned long delai=0;

// Message de base
// Les options de température et pression se rajouteront d'elle même si activé
// mettre un caractère = pour faire un retour à la ligne (_ . . . _)

String MsgBase = "F6KOH LE HAVRE LOC JN09CM EN TEST = PSE QSL TO BALISE@SHTSF.FR";

// Codage de l'alphabet par Hans Summers G0UPL et Stephen Farthing G0XAR
// de gauche à droite, après le premier 0
// 0 = dot, 1 = dash
const static byte CodeCW[]={
  0b11011111,  // 0
  0b11001111,  // 1
  0b11000111,  // 2
  0b11000011,  // 3
  0b11000001,  // 4
  0b11000000,  // 5
  0b11010000,  // 6
  0b11011000,  // 7
  0b11011100,  // 8
  0b11011110,  // 9
  0b10111000,  // :
  0b10101010,  // ;
  0b10001100,  // < -> ?
  0b11010001,  // =
  0b10001100,  // > -> ?
  0b10001100,  // ?
  0b10011010,  // @ -> ?
  0b11111001,  // A
  0b11101000,  // B
  0b11101010,  // C
  0b11110100,  // D
  0b11111100,  // E
  0b11100010,  // F
  0b11110110,  // G
  0b11100000,  // H
  0b11111000,  // I
  0b11100111,  // J
  0b11110101,  // K
  0b11100100,  // L
  0b11111011,  // M
  0b11111010,  // N
  0b11110111,  // O
  0b11100110,  // P
  0b11101101,  // Q
  0b11110010,  // R
  0b11110000,  // S
  0b11111101,  // T
  0b11110001,  // U
  0b11100001,  // V
  0b11110011,  // W
  0b11101001,  // X
  0b11101011,  // Y
  0b11101100,  // Z
};


//              * * * * *   T R A I T   * * * * *
void Trait() {
  digitalWrite(CW, 1);
  delai=3*Duree_Point;
  delay(delai);
  delai=Duree_Point;
  digitalWrite(CW, 0);
  delay(delai);
}

//              * * * * *   P O I N T   * * * * *
void Point() {
  digitalWrite(CW, 1);
  delai=Duree_Point;
  delay(delai);
  digitalWrite(CW, 0);
  delay(delai);
}

//              * * * * *   Espace entre Caractères   * * * * *
void ECar() {
  delai=2*Duree_Point;
//  digitalWrite(CW, 0);
  delay(delai);
}

//              * * * * *   Espace entre Mots   * * * * *
void EMot() {
  delai=6*Duree_Point;
//  digitalWrite(CW, 0);
  delay(delai);
}

//              * * * * *   S E T U P   * * * * *
void setup()
{
  pinMode(CW, OUTPUT);
  pinMode(PTT, OUTPUT);

  pinMode(4,INPUT_PULLUP);
  pinMode(6,INPUT_PULLUP);
  pinMode(7,INPUT_PULLUP);
  pinMode(8,INPUT_PULLUP);
  pinMode(9,INPUT_PULLUP);
  pinMode(10,INPUT_PULLUP);
  pinMode(11,INPUT_PULLUP);
  pinMode(12,INPUT_PULLUP);

#if Debug==1
  Serial.begin(115200);
#endif
  if (Press_F)
  {
    // initialisation du capteur de pression
    unsigned status;
    //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
    status = bmp.begin(0x76);
    if (!status)
    {
  #if Debug==1
      Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                        "try a different address!"));
      Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
      Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
      Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
      Serial.print("        ID of 0x60 represents a BME 280.\n");
      Serial.print("        ID of 0x61 represents a BME 680.\n");
  #endif
      Press_F = false;
    }
  }

  // Si la mesure de pression barométrique est activée, on initialise le capteur
  if (Press_F)
  {
    /* Default settings from datasheet. */
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

    bmp_temp->printSensorDetails();
  }
  
  // Initialisation sortie PTT
  digitalWrite(PTT, 0);
  digitalWrite(CW, 0);

  delay (TA_Ini);  // Attente avant émission du TX après coupure alim
}

//              * * * * *   M A I N   L O O P   * * * * *
void loop()
{
  String Msg = MsgBase;
  digitalWrite(PTT, 1);   // PTT on
  digitalWrite(CW, 0);
 
  delay(TA_CW);         // délai en ms entre PTT et envoi CW

  // Lecture de la température extérieur si activée
  if (Temp_F)
  {
    int snr = analogRead(A3); // Lecture de la valeur brute ADC (0 - 1023 <=> 0 - 5 V)
    // Mise à l'échelle de la température
    double Temp = ((snr - temp_minraw)/(temp_maxraw - temp_minraw))*(temp_maxEU - temp_minEU) + temp_minEU;
    Msg += " = OAT " + String(Temp, 0) + " DEGC"; // Ajout de la température au message de la balise
  }

  // Lecture de la pression barométrique si activée
  if (Press_F)
  {
    bmp_pressure->getEvent(&pressure_event);
    double Pressure = String(pressure_event.pressure).toDouble();
    Msg += " = QNH " + String(Pressure,0) + " HPA"; // Ajout de la pression barométrique au message de la balise. On en profite pour retirer les décimales
  }

#if Debug==1
  Serial.println(Msg);
#endif

  // On boucle pour chaque caractère du message
  for(int i=0;i<(Msg.length());i++)
  {
#if Debug==1
    Serial.print(Msg[i]);
#endif
    if (Msg[i] == ' ') // Si le char est un espace
    {
      EMot();
    }
    else
    {
      iTab=Msg[i]-'0'; // On ramène la valeur ASCII sur une base 0, par exemple '0' = 0 , 'A' = 17, etc
      if (iTab<0 || iTab>sizeof(CodeCW)) // On test si le char est hors de la plage du tablau de correspondance
      {
        if (Msg[i] == '.')
          Code = 0b10010101;      // .
        else if (Msg[i] == '/')
          Code = 0b11010010;      // /
        else
          Code = 0b10001100;      // ? pour caractères non pris en compte
      }
      else
      {
        Code = CodeCW[iTab]; // Si non , On récupère le code correspondant dans le tableau CW
      }
      iB=8;
      do            // recherche premier bit = 0 dans le code
        iB--;
      while (bitRead(Code,iB)==1);
      do
      {
        iB--;
        if (bitRead(Code,iB)==1)  // envoi d'un trait ou d'un point selon code
          Trait();
        else
          Point();
      }
      while (iB > 0);
      ECar();
    }
  }
#if Debug==1
  Serial.println();
#endif
  digitalWrite(CW, 1);
  delay (TA_RX);         // délai en ms entre fin CW et arrêt émission  
  if (!TX_ON)
  {
    digitalWrite(PTT, 0);   // PTT off
    digitalWrite(CW, 0);
  }
  delay (TA_MSG); // délai en ms avant la répétition du message
}