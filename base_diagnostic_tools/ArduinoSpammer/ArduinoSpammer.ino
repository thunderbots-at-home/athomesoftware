// left and right values
float left;
float right;
unsigned long old_time;

void setup() 
{ 
  Serial.begin(9600);
  left = 0;
  right = 0;
  old_time = millis();
} 

void loop() 
{ 
  char buffer[50];

  int left1 = (left - (int)left) * 100;
  int right1 = (right - (int)right) * 100;

  sprintf(buffer, "distance %0d.%d %0d.%d time %lu", (int)left, left1, (int)right, right1, millis() - old_time);
  Serial.println(buffer);    

  old_time = millis();
  left += 0.01;
  right += 0.01; 

  delay(100);
}

