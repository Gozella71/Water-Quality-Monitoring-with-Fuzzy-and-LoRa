#include <AntaresLoRaWAN.h>
#include <lmic.h> //device
#include <OneWire.h>
#include <DallasTemperature.h>

/*
#define pin_ph          A0
#define pin_tds         A1
#define pin_turbidity   A2
#define pin_ds          2
#define pin_led         13
*/

//definisi nilai parameter
//ph
#define ph_asam_bawah 6
#define ph_asam_atas 6.5

#define ph_netral_bawah 6
#define ph_netral_tengah_bawah 6.5
#define ph_netral_tengah_atas 7.4
#define ph_netral_atas 8

#define ph_basa_bawah 7.4
#define ph_basa_atas 8

//kekeruhan
#define kekeruhan_jernih_bawah 5
#define kekeruhan_jernih_atas 15

#define kekeruhan_keruh_bawah 5
#define kekeruhan_keruh_tengah 15
#define kekeruhan_keruh_atas 25

#define kekeruhan_sangatkeruh_bawah 15
#define kekeruhan_sangatkeruh_atas 25

//suhu
#define suhu_dingin_bawah 23
#define suhu_dingin_atas 27

#define suhu_normal_bawah 23
#define suhu_normal_tengah 27
#define suhu_normal_atas 31

#define suhu_panas_bawah 27
#define suhu_panas_atas 31

//TDS
#define tds_sedikit_bawah 400
#define tds_sedikit_atas 600

#define tds_sedang_bawah 400
#define tds_sedang_tengah 600
#define tds_sedang_atas 1000

#define tds_banyak_bawah 400
#define tds_banyak_atas 1000

#define Buruk 25
#define Kurang_Baik 50
#define Baik 75
#define Unggul 100

//Antares
#define ACCESSKEY "1033363d351a72dd:cec3d76a69c739ae"
#define DEVICEID "c7fa188c"

#define pin_ph          34
#define pin_tds         36
#define pin_turbidity   32
#define pin_ds          13
#define pin_led         26

float suhu,tds,ph,Ph,turbidity,Kekeruhan,Suhu,Tds,Kualitas;
float phArr[3];
float turbidityArr[3]; //definisiarray outputnya 3
float suhuArr[3];
float tdsArr[3];

//AntaresLoRa
AntaresLoRaWAN antares;
String dataSend = "";

OneWire oneWire(pin_ds);
DallasTemperature sensors(&oneWire);

#define TdsSensorpin 39
#define VREF 5.0      // analog reference voltage(Volt) of the ADC
#define SCOUNT  30           // sum of sample point
int analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0,copyIndex = 0;
float averageVoltage = 0,tdsValue = 0;
// https://wiki.dfrobot.com/Gravity__Analog_TDS_Sensor___Meter_For_Arduino_SKU__SEN0244#More_Documents
void read_tds(){
  while(1){
  static unsigned long analogSampleTimepoint = millis();
   if(millis()-analogSampleTimepoint > 40U)     //every 40 milliseconds,read the analog value from the ADC
   {
     analogSampleTimepoint = millis();
     analogBuffer[analogBufferIndex] = analogRead(pin_tds);    //read the analog value and store into the buffer
     analogBufferIndex++;
     if(analogBufferIndex == SCOUNT) 
         analogBufferIndex = 0;
   }   
   static unsigned long printTimepoint = millis();
   if(millis()-printTimepoint > 800U)
   {
      printTimepoint = millis();
      for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)
        analogBufferTemp[copyIndex]= analogBuffer[copyIndex];
      averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 4096.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
      float compensationCoefficient=1.0+0.02*(suhu-25.0);    //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
      float compensationVolatge=averageVoltage/compensationCoefficient;  //temperature compensation
      tdsValue=(133.42*compensationVolatge*compensationVolatge*compensationVolatge - 255.86*compensationVolatge*compensationVolatge + 857.39*compensationVolatge)*0.5; //convert voltage value to tds value
      //Serial.print("voltage:");
      //Serial.print(averageVoltage,2);
      //Serial.print("V   ");
      //Serial.print("TDS Value:");
      //Serial.print(tdsValue,0);
      //Serial.println("ppm");
      tds=tdsValue;
      break;
   }
  }
}

#define Offset -2          //deviation compensate
#define samplingInterval 20
#define printInterval 800
#define ArrayLenth  40    //times of collection
int pHArray[ArrayLenth];   //Store the average value of the sensor feedback
int pHArrayIndex=0;
// https://wiki.dfrobot.com/PH_meter_SKU__SEN0161_
void read_ph(){
  while(1){
  static unsigned long samplingTime = millis();
  static unsigned long printTime = millis();
  static float pHValue,voltage;
  if(millis()-samplingTime > samplingInterval)
  {
      pHArray[pHArrayIndex++]=analogRead(pin_ph);
      if(pHArrayIndex==ArrayLenth)pHArrayIndex=0;
      voltage = avergearray(pHArray, ArrayLenth)*5/4096;
      pHValue = 3.5*voltage+Offset;
      samplingTime=millis();
  }
  if(millis() - printTime > printInterval)   //Every 800 milliseconds, print a numerical, convert the state of the LED indicator
  {
        //Serial.print("Voltage:");
        //Serial.print(voltage,2);
        //Serial.print("    pH value: ");
        //Serial.println(pHValue,2);
        ph=pHValue;
        break;
  }
  }
}

double avergearray(int* arr, int number){
  int i;
  int max,min;
  double avg;
  long amount=0;
  if(number<=0){
    Serial.println("Error number for the array to avraging!/n");
    return 0;
  }
  if(number<5){   //less than 5, calculated directly statistics
    for(i=0;i<number;i++){
      amount+=arr[i];
    }
    avg = amount/number;
    return avg;
  }else{
    if(arr[0]<arr[1]){
      min = arr[0];max=arr[1];
    }
    else{
      min=arr[1];max=arr[0];
    }
    for(i=2;i<number;i++){
      if(arr[i]<min){
        amount+=min;        //arr<min
        min=arr[i];
      }else {
        if(arr[i]>max){
          amount+=max;    //arr>max
          max=arr[i];
        }else{
          amount+=arr[i]; //min<=arr<=max
        }
      }//if
    }//for
    avg = (double)amount/(number-2);
  }//if
  return avg;
}

int getMedianNum(int bArray[], int iFilterLen) 
{
      int bTab[iFilterLen];
      for (byte i = 0; i<iFilterLen; i++)
      bTab[i] = bArray[i];
      int i, j, bTemp;
      for (j = 0; j < iFilterLen - 1; j++) 
      {
      for (i = 0; i < iFilterLen - j - 1; i++) 
          {
        if (bTab[i] > bTab[i + 1]) 
            {
        bTemp = bTab[i];
            bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
         }
      }
      }
      if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
      else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
      return bTemp;
}

void setup() {
  Serial.begin(115200);
  pinMode(pin_led,OUTPUT);
  for(int i=0;i<10;i++){digitalWrite( pin_led, digitalRead( pin_led ) ^ 1 );delay(100);}digitalWrite( pin_led,0);
  sensors.begin();

  antares.setPins(2, 14, 12);   // Set pins for NSS, DIO0, and DIO1
  antares.setTxInterval(1);    // Set the amount of interval time (in seconds) to transmit
  antares.setSleep(true);  // antares.setSleep(true, 10);
  antares.init(ACCESSKEY, DEVICEID);
  antares.setDataRateTxPow(DR_SF10, 17);
 
  pinMode(pin_tds,INPUT);
}

void loop() {
  
  sensors.requestTemperatures();
  suhu  = sensors.getTempCByIndex(0);
  read_tds();
  read_ph();
  
//float phArr[3];
//float turbidityArr[3]; //definisiarray outputnya 3
//float suhuArr[3];
//float tdsArr[3];

  phFuzzyfication(ph, phArr); //kananrange //kiriPh 
  turbidityFuzzyfication(turbidity, turbidityArr); //definisarraydanrule //ph nilai input sensor //pharr nilai output
  suhuFuzzyfication(suhu, suhuArr);
  tdsFuzzyfication(tds, tdsArr);

  
  int sensorValue = analogRead(32);
  float voltage = sensorValue * (5.0 / 4096.0);
  float turbidity = (-1120.4 * voltage * voltage) + (5742.3 * voltage) - 4353.8; //in NTU

  if(voltage>4.2){turbidity=0;
  
  //Serial.print ("Tegangan turbidity: ");
  //Serial.println (voltage);
  //Serial.print ("NTU:");
  //Serial.println (turbidity);
  delay(1500);
}
else{
  //Serial.print ("Tegangan turbidity: ");
  //Serial.println (voltage);
  //Serial.print ("NTU:");
  //Serial.println (turbidity);
  delay(1500);
}
  
  Serial.print("Suhu=");
  Serial.println(suhu);
  Serial.print(" TDS=");
  Serial.println(tds);
  Serial.print(" Ph=");
  Serial.println(ph); 
  Serial.print(" Kekeruhan=");
  Serial.println(turbidity);
  Serial.println();
  //Serial.println(phArr[0],ph);

  
float ruleArr[3][3][3][3];   //81
evaluationRule(phArr, turbidityArr, suhuArr, tdsArr, ruleArr); //urutannya
  
  Serial.print("evaluationrule");
  
for (int i = 0; i < 3; i++) {
  for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 3; k++){
        for (int l = 0; l < 3; l++){
         Serial.print(ruleArr[i][j][k][l]);
         Serial.print("|");
        }
     Serial.println();}
   Serial.println();}
 Serial.println();}
    
 
float Kualitas = defuzzyfication(ruleArr);
Serial.print("Nilai Fuzzy    : "); 
Serial.println(Kualitas);
delay(1000);

{if (Kualitas >= 0 && Kualitas <= 25  ) {                               
  Serial.print("      'Buruk'");                                 
   Serial.println("\n");                                  
  delay(1000); }
else if (Kualitas >= 26 && Kualitas <= 50 ) {
   Serial.print("     'Kurang Baik'");
   Serial.println("\n");
   delay(1000);}
else if (Kualitas >= 51 && Kualitas < 75 )  {
   Serial.print("    'Baik'");
   Serial.print("\n");
   delay(1000) ;}
else { 
  Serial.print("      'Unggul'");            
  Serial.println("\n");
  delay(1000);
  }
  }
dataSend ="su: " + String(suhu)+","+"tds: " +String(tds)+","+"ph: " +String(ph)+","+"k: "+String(turbidity)+","+"f: "+String(Kualitas);
sendPacket(dataSend); 
   }
//Fungsi send data, disarankan untuk tidak merubah fungsi ini
  void sendPacket(String &input) {
  String dataA = String(input);
  antares.send(dataA);

  char dataBuf[50];
  int dataLen = dataA.length();

  dataA.toCharArray(dataBuf, dataLen + 1);
  Serial.println("Data :" + (String)dataLen );
  if (dataLen > 1)
  {
    Serial.println("\n[ANTARES] Data: " + dataA + "\n");

    if (LMIC.opmode & OP_TXRXPEND) {
      Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
      LMIC_setTxData2(1, (uint8_t*)dataBuf, dataLen, 0);
      Serial.println(F("Packet queued"));
      esp_sleep_enable_timer_wakeup(10 * 1000000);
      //esp_sleep_enable_ext0_wakeup(GPIO_NUM_34,0);
      esp_deep_sleep_start();

    }
  }
  else
  {
    Serial.println("\n[ANTARES] Data: Kosong\n");
  }
  delay(10000);
}

void phFuzzyfication(const float ph, float * output) {
if (ph <= ph_asam_bawah) {
output[0] = 1;}
else if (ph > ph_asam_bawah && ph < ph_asam_atas) {
output[0] = (ph_asam_atas - ph) / (ph_asam_atas - ph_asam_bawah);}
else {
output[0] = 0;}

if (ph >= ph_basa_atas) {
output[1] = 1;}
else if (ph > ph_basa_bawah && ph < ph_basa_atas) {
output[1] = (ph- ph_basa_bawah) / (ph_basa_atas - ph_basa_bawah);}
else {
output[1] = 0;
}

if (ph >= ph_netral_tengah_bawah && ph <= ph_netral_tengah_atas)
{output[2] = 1 ;} 
else if (ph > ph_netral_bawah && ph < ph_netral_tengah_bawah) {
output[2] = (ph - ph_netral_bawah) / (ph_netral_tengah_bawah - ph_netral_bawah);}
else if (ph > ph_netral_tengah_atas && ph < ph_netral_atas)
output[2] = (ph_netral_atas - ph) / (ph_netral_atas - ph_netral_tengah_bawah); 
else {output[2] = 0;}}

void suhuFuzzyfication(const float suhu, float * output) {
if (suhu <= suhu_dingin_bawah) {
output[0] = 1;}
else if (suhu > suhu_dingin_bawah && suhu < suhu_dingin_atas) {
output[0] = (suhu_dingin_atas - suhu) / (suhu_dingin_atas - suhu_dingin_bawah);}
else {
output[0] = 0;}

if (suhu == suhu_normal_tengah)
{output[2]=1;}
else if (suhu > suhu_normal_bawah && suhu < suhu_normal_tengah) {
output[2] = (suhu - suhu_normal_bawah) / (suhu_normal_tengah - suhu_normal_bawah);}
else if (suhu > suhu_normal_tengah && suhu < suhu_normal_atas) {
output[2] = (suhu_normal_atas - suhu) / (suhu_normal_atas - suhu_normal_tengah);}
//else if (suhu > suhu_normal_bawah && suhu < suhu_normal_atas){
//output[2]=0;} 
else {output[2] = 0;}
 
if (suhu >= suhu_panas_atas) {
output[1] = 1;}
else if (suhu > suhu_panas_bawah && suhu < suhu_panas_atas) {
output[1] = (suhu - suhu_panas_bawah) / (suhu_panas_atas - suhu_panas_bawah);}
else {
output[1] = 0;
}
}

void turbidityFuzzyfication(const float turbidity, float * output) {
if (turbidity >= kekeruhan_sangatkeruh_atas) {
output[0] = 1;}
else if (turbidity > kekeruhan_sangatkeruh_bawah && turbidity < kekeruhan_sangatkeruh_atas) {
output[0] = (turbidity - kekeruhan_sangatkeruh_bawah) / (kekeruhan_sangatkeruh_atas - kekeruhan_sangatkeruh_bawah);}
else {
output[0] = 0;}

if (turbidity == kekeruhan_keruh_tengah)
{output[1]=1;} 
else if (turbidity > kekeruhan_keruh_bawah && turbidity < kekeruhan_keruh_tengah) {
output[1] = (turbidity - kekeruhan_keruh_bawah) / (kekeruhan_keruh_tengah -kekeruhan_keruh_bawah);}
else if (turbidity > kekeruhan_keruh_tengah && turbidity < kekeruhan_keruh_atas) {
output[1] = (kekeruhan_keruh_atas - turbidity) / (kekeruhan_keruh_atas - kekeruhan_keruh_tengah);}
//else if (turbidity > kekeruhan_keruh_bawah && turbidity < kekeruhan_keruh_atas){
//output[1]=1;} 
else {output[1] = 0;}
 
if (turbidity <= kekeruhan_jernih_bawah) {
output[2] = 1;}
else if (turbidity > kekeruhan_jernih_bawah && turbidity < kekeruhan_jernih_atas) {
output[2] = (kekeruhan_jernih_atas - turbidity) / (kekeruhan_jernih_atas - kekeruhan_jernih_bawah);}
else {
output[2] = 0;
}}

void tdsFuzzyfication(const float tds, float * output) {
if (tds >= tds_banyak_atas) {
output[0] = 1;}
else if (tds > tds_banyak_bawah && tds < tds_banyak_atas) {
output[0] = (tds - tds_banyak_atas) / (tds_banyak_atas - tds_banyak_bawah);}
else {
output[0] = 0;}

if (tds == tds_sedang_tengah)
{output[1] = 1;} 
else if (tds > tds_sedang_bawah && tds < tds_sedang_tengah) {
output[1] = (tds - tds_sedang_bawah) / (tds_sedang_tengah - tds_sedang_bawah);}
else if (tds > tds_sedang_tengah && tds < tds_sedang_atas) {
output[1] = (tds_sedang_atas - tds) / (tds_sedang_atas - tds_sedang_tengah);}
//else if (tds > tds_sedang_bawah && tds < tds_sedang_atas){
//output[1]=1;} 
else {output[1] = 0;}

if (tds <= tds_sedikit_bawah) {
output[2] = 1;}
else if (tds > tds_sedikit_bawah && tds < tds_sedikit_atas) {
output[2] = (tds_sedikit_bawah - tds) / (tds_sedikit_atas - tds_sedikit_bawah);}
else {
output[2] = 0;
}}

void evaluationRule(const float * ph, const float * suhu, const float * turbidity, const float * tds, float rule[3][3][3][3]) {
for (int i = 0; i < 3; i++) {
  for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 3; k++){
        for (int l = 0; l < 3; l++){
                      float nmin = min(ph[i], suhu[j]) ;
                        float nimin = min( turbidity[k], tds[l]);
                           rule[i][j][k][l] = min(nmin, nimin);
                                   }
                                 }
                               }
                             }
          }
float defuzzyfication(const float rule[3][3][3][3]) {
//Deffuzyfication Untuk pH Asam
float result = (rule[0][0][0][0] * Buruk) + (rule[0][0][0][1] * Buruk) + (rule[0][0][0][2] * Buruk) 
             + (rule[0][0][1][0] * Buruk) +(rule[0][0][1][1] * Buruk) +(rule[0][0][1][2] * Kurang_Baik) 
             + (rule[0][0][2][0] * Buruk) +(rule[0][0][2][1] * Buruk) +(rule[0][0][2][2] * Kurang_Baik) 
             +(rule[0][1][0][0] * Buruk) +(rule[0][1][0][1] * Buruk) +(rule[0][1][0][2] * Buruk) 
             +(rule[0][1][1][0] * Buruk) +(rule[0][1][1][1] * Buruk) +(rule[0][1][1][2] * Kurang_Baik) 
             +(rule[0][1][2][0] * Kurang_Baik) +(rule[0][1][2][1] * Kurang_Baik) +(rule[0][1][2][2] * Kurang_Baik) 
             +(rule[0][2][0][0] * Buruk) +(rule[0][2][0][1] * Buruk) +(rule[0][2][0][2] * Kurang_Baik) 
             +(rule[0][2][1][0] * Buruk) +(rule[0][2][1][1] * Buruk) +(rule[0][2][1][2] * Kurang_Baik) 
             +(rule[0][2][2][0] * Kurang_Baik) +(rule[0][2][2][1] * Kurang_Baik) +(rule[0][2][2][2] * Kurang_Baik);

//Deffuzyfication Untuk ph Basa
result += (rule[1][0][0][0] * Buruk) + (rule[1][0][0][1] * Buruk) + (rule[1][0][0][2] * Kurang_Baik) 
        + (rule[1][0][1][0] * Buruk) +(rule[1][0][1][1] * Buruk) +(rule[1][0][1][2] * Buruk) 
        +(rule[1][0][2][0] * Kurang_Baik) +(rule[1][0][2][1] * Kurang_Baik) +(rule[1][0][2][2] * Kurang_Baik) 
        +(rule[1][1][0][0] * Buruk) +(rule[1][1][0][1] * Buruk) +(rule[1][1][0][2] * Kurang_Baik) 
        +(rule[1][1][1][0] * Buruk) +(rule[1][1][1][1] * Buruk) +(rule[1][1][1][2] * Buruk) 
        +(rule[1][1][2][0] * Kurang_Baik) +(rule[1][1][2][1] * Kurang_Baik) +(rule[1][1][2][2] * Kurang_Baik) 
        +(rule[1][2][0][0] * Kurang_Baik) +(rule[1][2][0][1] * Kurang_Baik) +(rule[1][2][0][2] * Kurang_Baik) 
        +(rule[1][2][1][0] * Kurang_Baik) +(rule[1][2][1][1] * Kurang_Baik) +(rule[1][2][1][2] * Kurang_Baik) 
        +(rule[1][2][2][0] * Kurang_Baik) +(rule[1][2][2][1] * Kurang_Baik) +(rule[1][2][2][2] * Kurang_Baik);

//Deffuzyfication Untuk pH Netral
result += (rule[2][0][0][0] * Kurang_Baik) + (rule[2][0][0][1] * Kurang_Baik) + (rule[2][0][0][2] * Kurang_Baik)
        + (rule[2][0][1][0] * Kurang_Baik) +(rule[2][0][1][1] * Kurang_Baik) +(rule[2][0][1][2] * Baik) 
        +(rule[2][0][2][0] * Kurang_Baik) +(rule[2][0][2][1] * Baik) +(rule[2][0][2][2] * Unggul) 
        +(rule[2][1][0][0] * Buruk) +(rule[2][1][0][1] * Kurang_Baik) +(rule[2][1][0][2] * Kurang_Baik) 
        +(rule[2][1][1][0] * Kurang_Baik) +(rule[2][1][1][1] * Kurang_Baik) +(rule[2][1][1][2] * Baik) 
        +(rule[2][1][2][0] * Kurang_Baik) +(rule[2][1][2][1] * Baik) +(rule[2][1][2][2] * Baik)  
        +(rule[2][2][0][0] * Buruk) +(rule[2][2][0][1] * Kurang_Baik) +(rule[2][2][0][2] * Kurang_Baik)
        +(rule[2][2][1][0] * Baik) +(rule[2][2][1][1] * Baik) +(rule[2][2][1][2] * Baik) 
        +(rule[2][2][2][0] * Baik) +(rule[2][2][2][1] * Unggul) +(rule[2][2][2][2] * Unggul);
float defuzification = 0;
for (int i = 0; i < 3; i++) {
  for (int j = 0; j < 3; j++) {
    for (int k = 0; k < 3; k++) {
      for (int l = 0; l < 3; l++) {
                     
                       defuzification += rule[i][j][k][l];
                       
                }
            }
}
}
return result = result / defuzification;
}
