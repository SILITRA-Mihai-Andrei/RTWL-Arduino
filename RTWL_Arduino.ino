/*
 * 
 * Fisierul "Program_software.ino"
 * 
 * Sistem mobil pentru monitorizarea conditiilor meteo de trafic auto
 * Real Time Weather Location Traffic System (RTWL)
 * 
 * Sistemul de transmisie comunica datele sub forma:
 * “latitude longitude weatherCode temperature himidity airQuality speed direction”
 * 
 * weatherCode (codul vremii) poate fi 100-499       
 * xx    00-33            33-66                 66-99 
 * 1xx   Sunny            Sun                   Heat 
 * 2xx   Soft Rain        Moderate rain         Torrential rain 
 * 3xx   Soft wind        Moderate wind         Torrential wind 
 * 4xx   Soft snow fall   Moderate snow fall    Massive snow fall
*/

#include "RTWL_constants.h"
#include <stdio.h>
#include <stdlib.h>
#include <SoftwareSerial.h> // pentru comunicatia seciala GPS
#include <TinyGPS.h>        // pentru comunicatia GPS
#include <dht11.h>          // pentru modulul de temperatura si umiditate (DHT11)

void(* resetFunc)(void)=0;                                    // reseteaza software microcontrolerul
void checkIntervalToSendData(bool forceSend=false);           // colecteaza toate datele si le trimite
void sendState(int state, int value=NULL);                    // trimite starea senzorilor sau uC
void GPS_GetData();                                           // culege datele GPS
void GetTempAndHumidity();                                    // colecteaza temperatura si umiditatea
int GetWeather();                                             // calculeaza codul vremii
bool CheckValidity(int weather, int temp, int hum, int air);  // verifica valorile senzorilor

SoftwareSerial serialGPS(PIN_GPS_RX, PIN_GPS_TX);  // pentru comunicatia seriala GPS
TinyGPS gps;                                       // pentru prelucrarea datelor GPS
dht11 DHT11;                                       // pentru modulul DHT11 serial

// Variabile ce vor memora functionarea senzorilor
// Fiecare senzor are un numar maxim de esuara
// Depasirea numarului maxim semnalizeaza defectarea senzorului/modulului
// Contorizeaza numarul de citiri nereusite
unsigned int gpsFailures;
bool proxymityFailure;      // este setata doar prin comanda; nu se poate detecta nefunctionarea
unsigned int airFailures;
unsigned int dhtFailures;

// Variabile pentru comunitactia seriala GPS
bool sendGPSCoordinates = false;    // trimiterea la fiecare 1 secunda a datelor GPS fara date meteo
bool newData = false;               // s-a primit un sir de date GPS valide
unsigned long last_GPS_read;        // retine ultima citire a datelor GPS (pentru intarziere)

// Variabile pentru senzorul de proximitate - calcularea frecventei stergatoarelor
unsigned long lastTimeWiper = 0;      // retine ultima detectie a senzorului
bool firstWiperMoveDetected = false;  // setata la prima detectie a senzorului
double period = 0;                    // memoreaza perioara stergatoarelor

// Variabile pentru senzorul de vant
int windSensorValue;  // valoarea data de tensiunea de iesire; dependenta de intensitatea vantului

// Variabile pentru construirea sirului de caractere final
char* string;                       // memoreaza sirul de caractere primit de la modulul Bluetooth
char* msg;                          // mesajul final ce va fi trimis prin Bluetooth                    
char* lastCoordinates;              // ultimele coordonate GPS valide
unsigned long _speed, _direction;   // viteza si directia de deplasare obtinute de la modulul GPS
int weatherCode;                    // codul vremii calculat din valorile senzorilor
int* temp_hum;                      // ultimele valori valide ale temperaturii si umiditatii relative
int air;                            // ultima valoare valid a calitatii aerului sau poluarii
unsigned long timeDelaySensorsRead; // retine ultima citire a senzorilor (pentru intarziere)
bool readyForNewCommand;            // false - ultima comanda nu si-a terminat executia

// Initializeaza variabilele, porneste comunicatiile seriale si pregateste programul pentru incepere
void setup() {    
  // Initializeaza modul de functionare al prinilor I/O
  pinMode(PIN_AIR_QUALITY, INPUT);
  pinMode(PIN_PROXY_SENSOR, INPUT);
  pinMode(PIN_WIND_SENSOR, INPUT);

  // Aloca memorie variabilelor de tip sir de caractere si sterge continutul
  string =          (char*) malloc(SIZE_RECEIVING_STRING);  strcpy(string, "");
  msg =             (char*) malloc(SIZE_MESSAGE_TO_SEND);   strcpy(msg, "");
  lastCoordinates = (char*) malloc(SIZE_COORDINATES);       strcpy(lastCoordinates, "");
  temp_hum =        (int*) malloc(2);

  // Seteaza contoarele de erori pentru senzori si module
  // Daca la inceputul programului acestia nu dau valori, se considera defecti
  // Daca senzorii si modulele functioneaza, vor da valori corecte si contoarele se vor reseta
  gpsFailures = FAILURES_GPS;
  proxymityFailure = false;
  airFailures = FAILURES_AIR_QUALITY;
  dhtFailures = 3;
  
  // Initializeaza variabilele pentru senzori si module cu valori incorecte
  // Daca la inceputul programului acesta nu dau valori corecte, vor ramana valorile incorecte
  temp_hum[0] = LIMIT_MAX_TEMPERATURE+1;
  temp_hum[1] = LIMIT_MAX_HUMIDITY+1;
  windSensorValue = -1;
  air = LIMIT_MAX_AIR+1;
  _speed = _direction = -1;
  weatherCode = -1;
  
  last_GPS_read = lastTimeWiper = timeDelaySensorsRead = millis();
  readyForNewCommand = true;

  // Verifica functionalitatea senzorilor si modulelor
  // Se reseteaza contoarele senzorilor si modulelor ce functioneaza
  // Se sesizeaza senzorii si modulele care nu functioneaza (dupa valoarea contorului)
  int counter = NUMBER_OF_READINGS_STARTUP;
  while(counter--) {
    checkIntervalToSendData();
    delay(MAXIMUM_TIME_READINGS_STARTUP / NUMBER_OF_READINGS_STARTUP);
  }

  // Pornirea comunicatiilor seriale
  Bluetooth.begin(BT_BAUD_RATE); 
  serialGPS.begin(GPS_BAUD_RATE);

  // Se curata memoria tampon (buffer) a comunicatiilor seriale
  while (serialGPS.available()) serialGPS.read();
  while (Bluetooth.available()) Bluetooth.read();

  // Se trimite un mesaj pentru semnalizarea pornirii programului
  Bluetooth.print(STATE_START);
}

void loop() {
  calculatePeriod();          // monitorizeaza stergatoarele de parbriz si calculeaza frecventa
  receiveDataFromBluetooth(); // citeste datele primite de la modulul Bluetooth (comenzi)
  checkIntervalToSendData();  // citeste senzorii si trimite datele
  GPS_GetData();              // curata memoria tampon si citeste datele GPS
  delay(10);
}

/**
 * Citeste datele primite prin Bluetooth. In general, acestea sunt comenzile utilizatorului.
 * Comenzile sunt executate doar atunci cand ultima comanda si-a terminat executia.
 * Daca este primit un sir de caractere prea lung, este golit si se continua citirea.
 * Comenzile primite trebuie sa contina doar litere si cifre, altfel totul se ignora.
 * Literele mici ale comenzii sunt transformate in litere mari pentru toleranta scrierii.
*/
void receiveDataFromBluetooth(){
  // Se verifica daca ultima comanda si-a terminat executia si daca sunt date de primit
  if(!readyForNewCommand || Bluetooth.available()==0) return;
  readyForNewCommand = false; // se incepe algoritmul de citire si executie a comenzii
  int index = 0;  // contorul pentru scrierea caracterelor in sirul de caractere
  strcpy(string, ""); // reseteaza sirul de caractere
  delay(10);
  // Se citesc toate datele primite
  while(Bluetooth.available()){
    // Verifica daca s-a depasit lungimea maxima a comenzii
    if(index >= SIZE_RECEIVING_STRING-1){
      // Este resetat contorul si sirul de caractere
      index = 0;
      strcpy(string, "");
    }
    char ch = Bluetooth.read(); // citeste primul caracter
    if(isNumberOrLetter(ch)){   // verifica daca este litera sau cifra
      string[index++] = ch;     // adauga caracterul in sirul de caractere al comenzii
    }
  }
  string[index] = '\0';       // termina sirul de caractere cu terminatorul de sir
  toUppercase(index);         // transforma toate literele mici in litere mari
  performCommands();          // executa comanda primita
  readyForNewCommand = true;  // semnalizeaza terminarea executiei comenzii
}

/**
 * Trimite starea primita ca parametru modulului Bluetooth.
 * Daca starea este asistata de o valoare (de eroare), este trimisa dupa un caracter special.
 * Daca starea nu este asistata de o valoare, este trimisa starea fara valoare.
 * Starea este un caracter specific fiecarui senzor/modul/erori/avertizari.
 * Valorile starilor pot fi consultate in fisierul de valori.
 * @param state: caracterul starii
 * @param value: valoarea starii (daca exista)
*/
void sendState(char state, int value=NULL){
  Bluetooth.print(state);   // trimite caracterul de stare
  Bluetooth.print(":");     // trimite caracterul de separare
  if(value != NULL){        // verifica daca starea are o valoare numerica
    Bluetooth.print(value); // trimite valoarea numerica dupa separator
  }
  // Termina trimiterea starii prin caracterul de sfarsit de mesaj
  Bluetooth.print(SEND_MESSAGE_DELIMITER);
}

/**
 * Colecteaza toate datele de la senzori si module si calculeaza codul vremii.
 * Senzorul de proximitate este ignorat deoarece acesta este monitorizat in alta functie.
 * Coordonatele GPS sunt ignorate deoarece acestea sunt actualizate mai des in alta functie.
*/
void getAllData(){
  GetTempAndHumidity(); // citeste temperatura si umiditate de la senzorul DHT11
  // Citeste si calculeaza valoarea modulului de calitate a aerului/poluare
  // Mapeaza valoarea pentru limita de 0-100%
  air = map(analogRead(PIN_AIR_QUALITY), 0, 1024, 0, 100);
  windSensorValue = analogRead(PIN_WIND_SENSOR); // citeste intensitatea vantului
  weatherCode = GetWeather(); // determina codul de vreme din valorile senzorilor
}

/**
 * Citeste, verifica si trimite toate datele citite de la senzori.
 * Introduce o intarziere intre fiecare executie.
 * @param forceSend: forteaza citirea si trimiterea datelor, ignorand intarzierea
*/
void checkIntervalToSendData(bool forceSend=false){
  // Verifica daca citirea si trimiterea trebuie realizata fortat
  // Verifica daca intarzierea a trecut
  if( forceSend || millis() - timeDelaySensorsRead > READING_DELAY ){
    getAllData();  // colecteaza toate datele de la senzori si module
    if( CheckValidity() ){  // verifica valorile obtinute
      setMessageToSend();   // construieste mesajul ce va fi trimis
      for(int i=0; i<strlen(msg); i++)  // parcurge mesajul de trimis
        Bluetooth.print(msg[i]);        // trimite fiecare caracter din mesaj
      // Termina trimiterea starii prin caracterul de sfarsit de mesaj
      Bluetooth.print(SEND_MESSAGE_DELIMITER);
    }
    timeDelaySensorsRead = millis();  // actualizeaza timpul ultimei executii
  }
}

/**
 * Construieste mesajul ce va fi trimis modulului Bluetooth.
 * Mesajul este construit din valorile tuturor senzorilor si modulelor.
 * Valorile utilizate sunt ultimele valori valide memorate in variabile.
 * Formatul mesajului este:
 * “latitude longitude weatherCode temperature himidity airQuality speed direction”
*/
void setMessageToSend(){
  char tmp[30];     // memoria tampon utilizata pentru conversiile valorilor
  strcpy(tmp, "");  // goleste memoria
  // Converteste valorile in caractere pentru a fi scrise in mesajul final
  // Valorile transformate in sir de caractere sunt scrise cu separatorul ' '
  // Parametrul 3 reprezinta baza de conversie, respectiv baza 10
  strcpy(msg, lastCoordinates);
  itoa(weatherCode, tmp, 10);
  strcat( msg, " " ); strcat( msg, tmp );
  itoa(temp_hum[0], tmp, 10);
  strcat( msg, " " ); strcat( msg, tmp );
  itoa(temp_hum[1], tmp, 10);
  strcat( msg, " " ); strcat( msg, tmp );
  itoa(air, tmp, 10);
  strcat( msg, " " ); strcat( msg, tmp );
  ltoa(_speed, tmp, 10);
  strcat( msg, " " ); strcat( msg, tmp );
  // Verifica valoarea directiei de deplasare in grade
  if (_direction == -1){
    strcat( msg, " U" );  // valoarea este invalida, trimite caracterul 'U' (unknown)
  }
  else {
    ltoa(_direction, tmp, 10);              // converteste valoarea in sir de caractere
    strcat( msg, " " ); strcat( msg, tmp ); // scrie valoarea in mesajul final
  }
}

/**
 * Citeste datele GPS de la modulul GPS si curata memoria tampon.
 * Trimite daca optiunea de trimitere a coordonatelor GPS la fiecare 1 secunde este activata.
 * Incrementeaza contorul de erori daca se primesc date eronate.
 * Datale GPS sunt: latitudinea, longitudinea, viteza si directia de deplasare.
*/
void GPS_GetData(){
  static char* buff;  // memorie pentru trimiterea datelor GPS la fiecare 1 secunda
  static char* result = (char*) malloc(SIZE_COORDINATES); // memorie pentru coordonatele GPS
  static char* tmp = (char*) malloc(5); // memorie pentru conversia valorile in caractere
  
  // Verifica daca a trecut o secunda de la ultima citire
  if( millis() - last_GPS_read > ONE_SECOND )
  {
    while (serialGPS.available()) // parcurge toate datele trimite serial
    {
      // Citeste primul caracter si il scrie in propozitia finala
      // Verifica daca propozitia finala contine caracterul de final
      if (gps.encode(serialGPS.read()))
        newData = true;
    }
    last_GPS_read = millis();
  }
  
  // Verifica daca s-a primit o propozitie completa
  if (newData)
  {
    newData = false;    // actualizeaza variabila pentru a primi o alta propozitie
    float flat, flon;   // latitudinea si longitudinea primite
    unsigned long age;  // variabila necesara pentru functia de citire - neutilizata
    
    gps.f_get_position(&flat, &flon, &age); // citeste latitudinea si longitudinea
    _speed = gps.f_speed_kmph();            // citeste viteza de deplasare in km/h
    _direction = gps.f_course();            // citeste directia de deplasare in grade

    // Verifica daca valorile primite sunt valide
    if( flat == TinyGPS::GPS_INVALID_F_ANGLE || flon == TinyGPS::GPS_INVALID_F_ANGLE ){
      gpsFailures++;                      // incrementeaza contorul de erori
      return;
    }

    // Converteste valorile latitudinii si longitudinii in siruri de caractere
    dtostrf(flat, 5, 2, tmp);
    strcpy(result, tmp);
    strcat(result, " ");
    dtostrf(flon, 5, 2, tmp);
    strcat(result, tmp);
    strcat(result, "\0");     // scrie caracterul de finalizare a mesajului
    lastCoordinates = result; // actualizeaza ultimele coordonate GPS
    // Verifica daca functia de trimitere a datelor GPS la fiecare 1 secunda este activata
    if(sendGPSCoordinates){
      buff = (char*) malloc(10);    // aloca memorie pentru scrierea coordonatelor GPS
      // Converteste valorile latitudinii si longitudinii in siruri de caractere
      dtostrf(flat, 10, 6, buff);
      Bluetooth.print(buff);
      Bluetooth.print(" ");
      dtostrf(flon, 10, 6, buff);
      Bluetooth.print(buff);  // trimite coordonatele GPS
      Bluetooth.print(SEND_MESSAGE_DELIMITER);  // trimite terminatorul de mesaj
    }

    // Reseteaza contorul de erori, deoarece s-au primit date valide
    gpsFailures = 0;
    // Actualizeaza ultima executie a functiei
    last_GPS_read = millis();
  }
  else{
    // Nu s-au primit date noi dupa citire. Incrementeaza contorul de erori
    gpsFailures++;
  }
}

/**
 * Citeste temperatura si umiditatea de la modulul DHT11 serial.
 * Verifica daca valorile sunt in limitele acceptate.
*/
void GetTempAndHumidity(){
  int chk = DHT11.read(PIN_DHT11);  // citeste octetul de validare
  int tmp = DHT11.temperature;      // citeste temperatura
  int hum = DHT11.humidity;         // citeste umiditatea

  // Verifica octetul de validare si limitele valorilor primite
  if(chk == -2 || tmp > 100 || hum > 100) {
    dhtFailures++; // incrementeaza contorul de erori
    return;
  }
  // Verifica limitele valorilor pentru temperatura si umiditate
  else if( tmp >= LIMIT_MIN_TEMPERATURE && tmp <= LIMIT_MAX_TEMPERATURE 
      && hum >= LIMIT_MIN_HUMIDITY && hum <= LIMIT_MAX_HUMIDITY){
    dhtFailures = 0;                  // reseteaza contorul de erori
    temp_hum[0] = DHT11.temperature;  // actualizeaza ultima temperatura valida
    temp_hum[1] = DHT11.humidity;     // actualizeaza ultima umiditate valida
  }
}

/**
 * Calculeaza perioada si frecventa stergatoarelor de parbriz.
 * Citeste valorile primite de la senzorul de proximitate.
 * Calculeaza timpul dintre detectiile stergatoarelor.
*/
void calculatePeriod(){
  // Verifica daca este prima detectie
  if( !firstWiperMoveDetected ){
    // Verifica daca s-a detectat stergatoarele de parbriz
    // Verifica daca s-a detectat dupa cel putin un interval de timp
    if( (!digitalRead(PIN_PROXY_SENSOR)) && millis() - lastTimeWiper > 1200 ){
      // S-a detectat prima miscare a stergatoarelor de parbriz
      // Se asteapta miscarea de revenire a stergatoarelor (dupa scurt timp)
      firstWiperMoveDetected = true;
      // Senzorul de proximitate este functional, din moment ce a trimis semnal
      proxymityFailure = false;
      // Calculeaza perioada dintre miscarea curenta si cea anterioara
      period = millis() - lastTimeWiper;
      lastTimeWiper = millis(); // actualizeaza timpul ultimei detectii
    } 
  }
  else // se asteapta miscarea de revenire a stergatoarelor dupa scurt timp
  {
    // Verifica daca s-a detectat miscarea dupa un interval scurt de timp
    if( (!digitalRead(PIN_PROXY_SENSOR)) && millis() - lastTimeWiper > 300 ){
      // S-a detectat a doua miscare a stergatoarelor de parbriz
      // Se asteapta din nou prima miscare a stergatoarelor
      firstWiperMoveDetected = false;
    }
  }
  // Nu s-a detectat miscarea stergatoarelor timp de 10 secunde
  if( millis() - lastTimeWiper > 10000 ){
    period = 0; // perioada este zero
    lastTimeWiper = millis(); // actualizeaza timpul ultimei detectii 
  }
}

/**
 * Calculeaza codul de vreme din ultimele valori valide ale senzorilor.
*/
int GetWeather(){  
  // Calculeaza codul pentru o vreme cu soare
  int sun_weather = map(temp_hum[0], 15, LIMIT_MAX_TEMPERATURE, 100, 199);

  // Verifica daca frecventa stergatoarelor este zero (fara ploaie sau ninsoare)
  // valoarea senzorului de vant este relativ scazuta
  // valoarea temperaturii este de cel putin 15 grade Celsius
  if( period == 0 && windSensorValue <= 200 && temp_hum[0] >= 15 ) 
    return sun_weather; // vreme cu soare
  // Verifica daca frecventa stergatoarelor este zero (fara ploaie sau ninsoare)
  // valoarea temperaturii este sub 15 grade Celsius
  // valoarea umiditatii relative este peste 50%
  else if( period > 0 && temp_hum[0] <= 15 && temp_hum[1] >= 50 )
  {
    // Calculeaza valoarea codului de vreme in functie de frecventa stergatoarelor
    if( period < 3333.33 )      return map(period, 1666.66, 3333.33, 200, 233);
    else if( period < 1666.66 ) return map(period, 1111.11, 1666.66, 234, 266);
    else if( period < 1111.11 ) return map(period, 0, 1111.11, 267, 299);
    else return sun_weather;  // frecventa nu este in limitele calculate
  }
  // Verifica daca intensitatea vantului este peste limita minima
  // valoarea temperaturii este sub 30 de grade Celsius
  else if( windSensorValue > LIMIT_MIN_WIND && temp_hum[1] < 30 )
  {
    // Vreme vantoasa
    return map(windSensorValue, 0, 1024, 300, 399);
  }
  // Verifica daca frecventa stergatoarelor peste zero (ploaie sau ninsoare)
  // verifica daca temperatura este sub 5 grade Celsius
  // verifica daca umiditatea relativa este peste 50%
  else if( period > 0 && temp_hum[0] < 5 && temp_hum[1] > 50 )
  {
    // Verifica daca perioada este sub limita maxima
    if( period < 0.90f){
      // Vreme cu ninsoare
      return map(period, 0.01f, 0.90f, 400, 499);
    }
    return sun_weather; // vreme invalida
  }
  return sun_weather; // vreme invalida
}

/**
 * Verifica valorile senzorilor si modulelor.
 * Incrementeaza contorul de erori daca nu sunt in limitele acceptate.
 * Trimite mesaje de stare pentru fiecare eroare detectata.
*/
bool CheckValidity(){
  bool validity = true; // este setata false la nerespectarea limitelor

  // Verifica functionarea senzorului de proximitate
  if(proxymityFailure == true){
    sendState(STATE_PROXY_SENSOR_NOT_WORKING);
    validity = false;
  }
  // Verifica limitele codului de vreme
  if(weatherCode < LIMIT_MIN_WEATHER_CODE || weatherCode > LIMIT_MAX_WEATHER_CODE){
    sendState(STATE_WEATHER_CODE_INVALID, weatherCode);
    validity = false;
  }
  // Verifica contorul de erori al modulului DHT11
  if(dhtFailures > 2){  // peste 3 citiri nereusite - senzorul nu functioneaza
    sendState(STATE_TEMP_HUM_SENSOR_NOT_WORKING);
    validity = false;
  }
  // Verifica limitele temperaturii
  if(temp_hum[0] < LIMIT_MIN_TEMPERATURE || temp_hum[0] > LIMIT_MAX_TEMPERATURE){
    sendState(STATE_TEMP_INVALID_DATA, temp_hum[0]);
    validity = false;
  }
  // Verifica limitele umiditatii relative
  if(temp_hum[1] < LIMIT_MIN_HUMIDITY || temp_hum[1] > LIMIT_MAX_HUMIDITY){
    sendState(STATE_HUM_INVALID_DATA, temp_hum[1]);
    validity = false;
  }
  // Verifica limitele modulului de calitate a aerului/poluare
  if(air < LIMIT_MIN_AIR || air > LIMIT_MAX_AIR){
    airFailures++; // incrementeaza contorul de erori al senzorului MQ-135
    // Verifica contorul de erori
    if(airFailures  >= FAILURES_AIR_QUALITY){
      sendState(STATE_AIR_SENSOR_NOT_WORKING);  // modul nefunctional
    }
    else{
      // Valoarea data de modul este invalida
      // Trimite starea cu valoarea data de modul
      sendState(STATE_AIR_SENSOR_INVALID_DATA, air);
    }
    validity = false;
  }

  // Verifica daca contorul de erori depaseste valoarea maxima
  if(gpsFailures >= FAILURES_GPS) {
    _speed = _direction = -1; //viteza si directia de deplasare sunt invalide
    sendState(STATE_GPS_NOT_WORKING);
    // Actualizeaza ultimele coordonate GPS valide cu valori eronate (pentru sesizare)
    strcpy(lastCoordinates, "0.0 0.0");
  }

  // Reseteaza contorul de erori pentru senzorul de calitate a aerului/poluare
  // Acest lucru este facut aici deoarece functionalitatea modulului este verificata
  // doar prin verificarea valorilor date de acesta
  if (validity) airFailures = 0;
  return validity;
}

/**
 * Verifica daca caracterul este litera sau cifra.
*/
bool isNumberOrLetter(char ch){
  int intCh = (int) ch;
  return intCh == 45 
        || (intCh >= 48 && intCh <= 57) 
        || (intCh >= 65 && intCh <= 90) 
        || (intCh >= 97 && intCh <= 122);
}

/**
 * Transforma toate literele mici ale comenzii primite de la modulul Bluetooth
 * in litere mici. Se realizeaza astfel toleranta la scris.
 * Se pot scrie comenzile in orice forma (cmd, CMD, cMd, CmD).
*/
void toUppercase(int len){
  for(int i=0; i<len; i++){
    int ic = (int) string[i];
    if(ic >= 97 && ic <= 122) 
      string[i] = char(ic-32);
  }
}

/**
 * Executa comanda primita in variabila <string>.
 * Verifica sirul de caractere al comenzii si il compara cu toate comenzile
 * disponibile din program.
*/
void performCommands(){
  if(strlen(string) == 0) return; // comanda nu are nici un caracter
  if(strcmp(string, COMMAND_RESET_INTEGER)==0
     || strcmp(string, COMMAND_RESET_SHORT)==0
     || strcmp(string, COMMAND_RESET_LONG)==0)
     resetFunc();
  else if(strcmp(string, COMMAND_GET_RESPONSE_INTEGER)==0
     || strcmp(string, COMMAND_GET_RESPONSE_SHORT)==0
     || strcmp(string, COMMAND_GET_RESPONSE_LONG)==0){
    Bluetooth.print("PONG!");
    Bluetooth.print(SEND_MESSAGE_DELIMITER);
  }
  else if(strcmp(string, COMMAND_GET_DATA_INTEGER)==0
     || strcmp(string, COMMAND_GET_DATA_SHORT)==0
     || strcmp(string, COMMAND_GET_DATA_LONG)==0)
    checkIntervalToSendData(true);
  else if(strcmp(string, COMMAND_PROXIMITY_SENSOR_DEFECTION_INTEGER)==0
     || strcmp(string, COMMAND_PROXIMITY_SENSOR_DEFECTION_SHORT)==0
     || strcmp(string, COMMAND_PROXIMITY_SENSOR_DEFECTION_LONG)==0){
    proxymityFailure = !proxymityFailure;
    sendState(STATE_PROXY_SENSOR_NOT_WORKING);
  }
  else if(strcmp(string, COMMAND_GET_GPS_COORDINATES_INTEGER)==0
     || strcmp(string, COMMAND_GET_GPS_COORDINATES_SHORT)==0
     || strcmp(string, COMMAND_GET_GPS_COORDINATES_LONG)==0){
    sendGPSCoordinates = true;
    sendState(STATE_GET_GPS_COORDINATES_ENABLED);
  }
  else if(strcmp(string, COMMAND_DISABLE_GET_GPS_COORDINATES_INTEGER)==0
     || strcmp(string, COMMAND_DISABLE_GET_GPS_COORDINATES_SHORT)==0
     || strcmp(string, COMMAND_DISABLE_GET_GPS_COORDINATES_LONG)==0){
    sendGPSCoordinates = false;
    sendState(STATE_GET_GPS_COORDINATES_DISABLED);
  }
  else{
    Bluetooth.print(UNKNOWN_COMMAND);
    Bluetooth.print(SEND_MESSAGE_DELIMITER);
  }
}
