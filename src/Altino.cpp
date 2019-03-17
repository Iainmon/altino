#include <Arduino.h>
#include <Altino.h>

unsigned char tx_d[28] = { 0, };
unsigned char Sendbuf[28] = {0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
unsigned char rx_data[1024]= { 0, };
unsigned char rx_d[31] = { 0, };
unsigned char rx_d_sensor1[31] = { 0, };
unsigned char rx_d_sensor2[31] = { 0, };
int len = 0;
int rxcnt=0;



unsigned char check_sum_tx_calcuration(int u16_cnt)
{
  int u16_tx_check_sum = 0;
  int u16_tx_cnt;
  u16_tx_check_sum = u16_tx_check_sum + tx_d[1];
  for (u16_tx_cnt = 3; u16_tx_cnt <= u16_cnt; u16_tx_cnt++) {
    u16_tx_check_sum = u16_tx_check_sum + tx_d[u16_tx_cnt];
  }
  u16_tx_check_sum = u16_tx_check_sum % 256;
  return (byte)(u16_tx_check_sum);
}

void SendData(unsigned char *Sendbuf)
{

  int k;
  tx_d[0] = 0x2;
  tx_d[1] = 28;

  tx_d[3] = 1;
  tx_d[4] = Sendbuf[4];
  tx_d[5] = Sendbuf[5];
  tx_d[6] = Sendbuf[6];
  tx_d[7] = Sendbuf[7];
  tx_d[8] = Sendbuf[8];
  tx_d[9] = Sendbuf[9];
  tx_d[10] = Sendbuf[10];
  tx_d[11] = Sendbuf[11];
  tx_d[12] = Sendbuf[12];
  tx_d[13] = Sendbuf[13];
  tx_d[14] = Sendbuf[14];
  tx_d[15] = Sendbuf[15];
  tx_d[16] = Sendbuf[16];
  tx_d[17] = Sendbuf[17];
  tx_d[18] = Sendbuf[18];
  tx_d[19] = Sendbuf[19];
  tx_d[20] = Sendbuf[20];
  tx_d[21] = Sendbuf[21];

  if (Sendbuf[4] == 1) {
	    Sendbuf[21] = Sendbuf[21] | 0x01;
  }
  else  {
	    Sendbuf[21] = Sendbuf[21];
  }

  tx_d[22] = Sendbuf[22];
  tx_d[23] = Sendbuf[23];
  tx_d[24] = Sendbuf[24];
  tx_d[25] = Sendbuf[25];
  tx_d[26] = 0;
  tx_d[27] = 0x3;

  tx_d[2] = check_sum_tx_calcuration(27);
  Serial.write(tx_d, 28);
  delay(10);
}

void Go(int left, int right)
{
  unsigned int temp=32768;
  unsigned int left_int, right_int;


  if (left < 0) {
    left_int = temp - left;
  }
  else{
	  left_int = left;
  }

  if (right<0){
    right_int = temp - right;
  }
  else {
	  right_int=right;
  }

  if (right==0) 
    Sendbuf[6] = 255;
  else
    Sendbuf[6] = 0;

  Sendbuf[7] = (byte)(right_int / 256);
  Sendbuf[8] = (byte)(right_int % 256);

  if (left == 0)
    Sendbuf[9] = 255;
  else
    Sendbuf[9] = 0;

  Sendbuf[10] = (byte)(left_int / 256);
  Sendbuf[11] = (byte)(left_int % 256);

  SendData(Sendbuf);
}


void Steering(int steeringvalue)
{
	Sendbuf[24] =0;
	Sendbuf[5] = (byte)steeringvalue;
	SendData(Sendbuf);
}

void Steering2(int value1, int value2)
{
	if(value1>127)
		value1=127;
	if(value2>127)
		value2=127;
	if(value1<-127)
		value1=-127;
	if(value2<-127)
		value2=-127;
	if(value1<0)
		value1=128-value1;
	if(value2<0)
		value2=128-value2;

	Sendbuf[5] = (byte)value1;
	Sendbuf[24] = 1;
	Sendbuf[25] = (byte)value2;
	SendData(Sendbuf);
}




void Sound(unsigned char buzzer)
{
	Sendbuf[22] = (byte)buzzer;
	SendData(Sendbuf);
}

void Display(unsigned char ASCII)
{
	Sendbuf[12] = ASCII;
	Sendbuf[13] = 0;
	Sendbuf[14] = 0;
	Sendbuf[15] = 0;
	Sendbuf[16] = 0;
	Sendbuf[17] = 0;
	Sendbuf[18] = 0;
	Sendbuf[19] = 0;
	Sendbuf[20] = 0;
	SendData(Sendbuf);
}

unsigned char dotsw(unsigned char dot)
{
    unsigned char redot=0;
    if(dot&0x01)
        redot=redot+128;
    if(dot&0x02)
        redot=redot+64;
    if(dot&0x04)
        redot=redot+32;
    if(dot&0x08)
        redot=redot+16;
    if(dot&0x10)
        redot=redot+8;
    if(dot&0x20)
        redot=redot+4;
    if(dot&0x40)
        redot=redot+2;
    if(dot&0x80)
        redot=redot+1;

    return redot;
}


void DisplayLine(unsigned char dot0, unsigned char dot1, unsigned char dot2, unsigned char dot3, unsigned char dot4, unsigned char dot5, unsigned char dot6, unsigned char dot7)
{
	Sendbuf[12] = 0;
	Sendbuf[13] = dotsw(dot0);
	Sendbuf[14] = dotsw(dot1);
	Sendbuf[15] = dotsw(dot2);
	Sendbuf[16] = dotsw(dot3);
	Sendbuf[17] = dotsw(dot4);
	Sendbuf[18] = dotsw(dot5);
	Sendbuf[19] = dotsw(dot6);
	Sendbuf[20] = dotsw(dot7);

	SendData(Sendbuf);
}

void Led(unsigned int led)
{
	Sendbuf[23] = (byte)(led%256);
	Sendbuf[21] = (byte)(led/256);
	SendData(Sendbuf);
}

void chekdata()
{
	int rx_check_sum;

	if ((rx_d[0] == 2) && (rx_d[30] == 3) && (rx_d[1] == 31)) {
		rx_check_sum = rx_d[0];
		rx_check_sum = rx_check_sum + rx_d[1];
		rx_check_sum = rx_check_sum + rx_d[3];
		rx_check_sum = rx_check_sum + rx_d[4];
		rx_check_sum = rx_check_sum + rx_d[5];
		rx_check_sum = rx_check_sum + rx_d[6];
		rx_check_sum = rx_check_sum + rx_d[7];
		rx_check_sum = rx_check_sum + rx_d[8];
		rx_check_sum = rx_check_sum + rx_d[9];
		rx_check_sum = rx_check_sum + rx_d[10];
		rx_check_sum = rx_check_sum + rx_d[11];
		rx_check_sum = rx_check_sum + rx_d[12];
		rx_check_sum = rx_check_sum + rx_d[13];
		rx_check_sum = rx_check_sum + rx_d[14];
		rx_check_sum = rx_check_sum + rx_d[15];
		rx_check_sum = rx_check_sum + rx_d[16];
		rx_check_sum = rx_check_sum + rx_d[17];
		rx_check_sum = rx_check_sum + rx_d[18];
		rx_check_sum = rx_check_sum + rx_d[19];
		rx_check_sum = rx_check_sum + rx_d[20];
		rx_check_sum = rx_check_sum + rx_d[21];
		rx_check_sum = rx_check_sum + rx_d[22];
		rx_check_sum = rx_check_sum + rx_d[23];
		rx_check_sum = rx_check_sum + rx_d[24];
		rx_check_sum = rx_check_sum + rx_d[25];
		rx_check_sum = rx_check_sum + rx_d[26];
		rx_check_sum = rx_check_sum + rx_d[27];
		rx_check_sum = rx_check_sum + rx_d[28];
		rx_check_sum = rx_check_sum + rx_d[29];
		rx_check_sum = rx_check_sum + rx_d[30];
		rx_check_sum = rx_check_sum % 256;

		if (rx_check_sum == rx_d[2])
		{
			if (rx_d[4] == 1)
			{
				rx_d_sensor1[7] = rx_d[7];
				rx_d_sensor1[8] = rx_d[8];
				rx_d_sensor1[9] = rx_d[9];
				rx_d_sensor1[10] = rx_d[10];
				rx_d_sensor1[11] = rx_d[11];
				rx_d_sensor1[12] = rx_d[12];
				rx_d_sensor1[13] = rx_d[13];
				rx_d_sensor1[14] = rx_d[14];
				rx_d_sensor1[15] = rx_d[15];
				rx_d_sensor1[16] = rx_d[16];
				rx_d_sensor1[17] = rx_d[17];
				rx_d_sensor1[18] = rx_d[18];
				rx_d_sensor1[19] = rx_d[19];
				rx_d_sensor1[20] = rx_d[20];
				rx_d_sensor1[21] = rx_d[21];
				rx_d_sensor1[22] = rx_d[22];
				rx_d_sensor1[23] = rx_d[23];
				rx_d_sensor1[24] = rx_d[24];
				rx_d_sensor1[25] = rx_d[25];
				rx_d_sensor1[26] = rx_d[26];
			}
			else
			{
				rx_d_sensor2[7] = rx_d[7];
				rx_d_sensor2[8] = rx_d[8];
				rx_d_sensor2[9] = rx_d[9];
				rx_d_sensor2[10] = rx_d[10];
				rx_d_sensor2[11] = rx_d[11];
				rx_d_sensor2[12] = rx_d[12];
				rx_d_sensor2[13] = rx_d[13];
				rx_d_sensor2[14] = rx_d[14];
				rx_d_sensor2[15] = rx_d[15];
				rx_d_sensor2[16] = rx_d[16];
				rx_d_sensor2[17] = rx_d[17];
				rx_d_sensor2[18] = rx_d[18];
				rx_d_sensor2[19] = rx_d[19];
				rx_d_sensor2[20] = rx_d[20];
				rx_d_sensor2[21] = rx_d[21];
				rx_d_sensor2[22] = rx_d[22];
				rx_d_sensor2[23] = rx_d[23];
				rx_d_sensor2[24] = rx_d[24];
				rx_d_sensor2[25] = rx_d[25];
			}
		}
	}
}

SensorData Sensor(int command)
{

	String temp;
	String sCommand;
	SensorData sensordata1;
	unsigned char buf=0;
	unsigned char test;
	int i,j;

	Sendbuf[4] = (byte)command;

	SendData(Sendbuf);	

	while(1) {
		if(Serial.available()>0){
			rx_data[rxcnt]=Serial.read();
			rxcnt++;
		}
		else {
			break;
		}
	}

	for(i=0; i<rxcnt; i++) {		
		for(j=0; j<30; j++) {
			rx_d[j]=rx_d[j+1];
		}
		rx_d[30]=rx_data[i];
		if(rx_d[0]==0x02 && rx_d[30]==0x03 && rx_d[1]==31)
		{
			chekdata();
			
			for(j=0; j<31; j++){
				rx_d[j]=0;
			}
		}
	}
	rxcnt=0;
    Serial.flush();

	sensordata1.IRSensor[0] = rx_d_sensor1[7] * 256 + rx_d_sensor1[8];
	sensordata1.IRSensor[1] = rx_d_sensor1[9] * 256 + rx_d_sensor1[10];
	sensordata1.IRSensor[2] = rx_d_sensor1[11] * 256 + rx_d_sensor1[12];
	sensordata1.IRSensor[3] = rx_d_sensor1[13] * 256 + rx_d_sensor1[14];
	sensordata1.IRSensor[4] = rx_d_sensor1[15] * 256 + rx_d_sensor1[16];
	sensordata1.IRSensor[5] = rx_d_sensor1[17] * 256 + rx_d_sensor1[18];

	sensordata1.TorqueSensor[0] = rx_d_sensor1[19] * 256 + rx_d_sensor1[20];
	sensordata1.TorqueSensor[1] = rx_d_sensor1[21] * 256 + rx_d_sensor1[22];
	sensordata1.TemperatureSensor = rx_d_sensor1[23] * 256 + rx_d_sensor1[24];
	sensordata1.CDSSensor = rx_d_sensor1[25] * 256 + rx_d_sensor1[26];

	sensordata1.GSensor[0] = rx_d_sensor2[7] * 256 + rx_d_sensor2[8];
	sensordata1.GSensor[1] = rx_d_sensor2[9] * 256 + rx_d_sensor2[10];
	sensordata1.GSensor[2] = rx_d_sensor2[11] * 256 + rx_d_sensor2[12];

	sensordata1.MSensor[0] = rx_d_sensor2[13] * 256 + rx_d_sensor2[14];
	sensordata1.MSensor[1] = rx_d_sensor2[15] * 256 + rx_d_sensor2[16];
	sensordata1.MSensor[2] = rx_d_sensor2[17] * 256 + rx_d_sensor2[18];

	sensordata1.SteeringVar = rx_d_sensor2[19] * 256 + rx_d_sensor2[20];
	sensordata1.SteeringTorque = rx_d_sensor2[21] * 256 + rx_d_sensor2[22];
	sensordata1.Battery = rx_d_sensor2[23] * 256 + rx_d_sensor2[24];

	return sensordata1;
}
