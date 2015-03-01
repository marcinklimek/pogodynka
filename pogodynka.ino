#define TIME_KEY   500
#define TIME_SYNC (9000+TIME_KEY)
#define TIME_VALUE_ONE  (4000+TIME_KEY)
#define TIME_VALUE_ZERO (2000+TIME_KEY)
#define TIME_DELTA 400

#define in_range(period, range) ( period > (range-TIME_DELTA) && period < (range+TIME_DELTA) )
#define BUFFER_SIZE 36

enum
{
  state_value = 1,
  state_packet
} states;

unsigned long captured_time;
unsigned long previous_captured_time;

byte nibble_index = 0;
byte was_sync = 0;
byte state = state_value;

unsigned int qbuffer[BUFFER_SIZE];
byte qstart = 0;
byte qend = 0;
byte qactive = 0;
byte nibbles[9] = {0,0,0,0,0,0,0,0,0};

void led_on()
{
  PORTA = 1;
}

void led_off()
{
  PORTA = 0;
}

void q_push(unsigned int p)
{
  qbuffer[qend] = p;
  qend = (qend + 1) % BUFFER_SIZE;

  if (qactive < BUFFER_SIZE)
    qactive++;
  else 
    qstart = (qstart + 1) % BUFFER_SIZE;
}

unsigned int q_pop()
{
  if (!qactive) { return 0; }

  unsigned int p = qbuffer[qstart];
  qstart = (qstart + 1) % BUFFER_SIZE;

  qactive--;
  return p;
}

void search_reset()
{
  was_sync = 0;
  
  nibble_index = 0;
  state = state_value;
  
  for(short i=0; i<9; i++)
    nibbles[i] = 0;
  
  led_off();
}

void interruptHandler()
{
  captured_time = micros();
  int captured_period = (captured_time - previous_captured_time);
  
  if (captured_period >= TIME_DELTA )
  {
    previous_captured_time = captured_time;      
    q_push(captured_period);
  }
}

int state_machine()
{
  if ( qactive == 0 )
    return 0;

  unsigned int captured_period = q_pop();

  if ( state == state_value )
  {
    if ( was_sync )
    {
      if ( in_range(captured_period, TIME_VALUE_ONE) )
      {
        short n_idx = (nibble_index/4);
        short b_idx = nibble_index%4;
        nibbles[n_idx] |= 1 << b_idx;
      }
      else if ( in_range(captured_period, TIME_VALUE_ZERO) )
      {
      }        
      else
      {
        search_reset();
        return state;
      }
      
      nibble_index++;
      if ( nibble_index == 36 )
        state = state_packet;
    }
    else if ( in_range( captured_period, TIME_SYNC) )
    {
        search_reset();
                    
        was_sync = 1;
        state = state_value;
        
        led_on();
    }
    else
    {
      
      search_reset();
    }
  }
  return state;
}


void setup() 
{
  Serial.begin(57600);
  Serial.println("Ready...");
  
  DDRA  = 1;
  led_off();
  search_reset();  
  
  previous_captured_time = millis();
  attachInterrupt(5, interruptHandler, RISING);
    
  interrupts();
}

void send_wrong_crc(char* text)
{
    Serial.print(text);
    Serial.print(";0;");
    
    for(byte i=0; i<9; i++)
    {
      Serial.print(nibbles[i]); 
      Serial.print(" ");
    }   
}

void parse_temp_humi()
{
  byte crc = 0xf;
  for(byte i=0; i<8; i++)
    crc -= nibbles[i];
  crc &= 0xf;

  if ( crc == nibbles[8] )
  {
    Serial.print("Temperature, humidity;1;");

    int temperature = nibbles[3] + (nibbles[4]<<4) + (nibbles[5]<<8) ;
    temperature = (temperature&0x0800) ? temperature | 0xf000 : temperature;
    Serial.print(temperature*0.1f);
    Serial.print(" ");

    int humidity = nibbles[6] + nibbles[7]*10;
    Serial.print(humidity);
  }
  else
  {
    send_wrong_crc("Temperature, humidity - Checksum WRONG");
  }
}

void parse_wind_speed()
{
  byte crc = 0xf;
  for(byte i=0; i<8; i++)
    crc -= nibbles[i];
  crc &= 0xf;

  if ( crc == nibbles[8] )
  {
    Serial.print("Average wind speed;2;");
    byte speed = nibbles[6] + (nibbles[7]<<4);
    Serial.print(speed);
  }
  else
  {
    send_wrong_crc("Average wind speed - Checksum WRONG");
  }
}

void parse_wind_dir()
{
  byte crc = 0xf;
  for(byte i=0; i<8; i++)
    crc -= nibbles[i];
  crc &= 0xf;

  if ( crc == nibbles[8] )
  {
    Serial.print("Wind dir;3;");
    
    unsigned int dir = ((nibbles[3] & 0b1000)>>4) | (nibbles[4]<<5) | (nibbles[5]<<9);
    Serial.print(dir);
    Serial.print(" ");
    byte gust = nibbles[6] + (nibbles[7]<<4);
    Serial.print(gust);
  }
  else
  {
    send_wrong_crc("Wind dir - Checksum WRONG");
  }
}

void parse_rain()
{
  byte crc = 0x7;
  for(byte i=0; i<8; i++)
    crc += nibbles[i];
  crc &= 0xf;

  if ( crc == nibbles[8] )
  {
    Serial.print("Rain;4;");
    unsigned int rain = nibbles[4] | (nibbles[5]<<4) | (nibbles[6]<<8) | (nibbles[7]<<12);
    Serial.print(rain*0.25f);
  }
  else
  {
    send_wrong_crc("Rain - Checksum WRONG");
  }
}

void loop() 
{
  //   ID;name;type_id;data;battery state id; battery state name
  if ( state_machine() == state_packet )
  {
    Serial.print( nibbles[0] );
    Serial.print( "." );
    Serial.print( nibbles[1] );
    Serial.print( ";" );

    byte sensor_type = (nibbles[2] & 0b0110);
    if ( sensor_type == 0b0110 ) 
    {
       byte sensor_sub_type = (nibbles[3] & 0b0111);
       if      ( sensor_sub_type == 0b001)
       {
          parse_wind_speed();
       }
       else if ( sensor_sub_type == 0b111)
       {
          parse_wind_dir();
       }
       else if ( sensor_sub_type == 0b011)
       {
          parse_rain();
       }
    }

    if ( sensor_type == 0b0000 || sensor_type == 0b0010 || sensor_type == 0b0100 ) 
    {
      parse_temp_humi();
    }

    if ( (nibbles[2] & 1)  ) 
      Serial.print(";1;lo bat");
    else
      Serial.print(";0;bat ok");
       
    Serial.println();
    search_reset();
  }
} 
