const int pRes = A0;
const int led = 9;
int CLK = 2;
int DT = 3;
int SW = 21;
float value = 0;
float brightness;
float raw;
float Time = 0;
float timeStep = 0;
float error;
float value_old;
float errint = 0;
float derr;
float u = 0;
int counter = 0;
int currentStateCLK;
int lastStateCLK;
String currentDir = "";
unsigned long ButtonPress = 0;
float change;
float value_filtered = 0;
double outMin = 0.0;
double outMax = 250.0;
double des, Kp, Ki, Kd;

double vars[4] = {960.0, 0.46, 0.000, 3.03}; //{Desired Brightness, Kp, Ki, Kd}




template <int order> // order is 1 or 2
class LowPass
{
  private:
    float a[order];
    float b[order + 1];
    float omega0;
    float dt;
    bool adapt;
    float tn1 = 0;
    float x[order + 1]; // Raw values
    float y[order + 1]; // Filtered values

  public:
    LowPass(float f0, float fs, bool adaptive) {
      // f0: cutoff frequency (Hz)
      // fs: sample frequency (Hz)
      // adaptive: boolean flag, if set to 1, the code will automatically set
      // the sample frequency based on the time history.

      omega0 = 6.28318530718 * f0;
      dt = 1.0 / fs;
      adapt = adaptive;
      tn1 = -dt;
      for (int k = 0; k < order + 1; k++) {
        x[k] = 0;
        y[k] = 0;
      }
      setCoef();
    }

    void setCoef() {
      if (adapt) {
        float t = micros() / 1.0e6;
        dt = t - tn1;
        tn1 = t;
      }

      float alpha = omega0 * dt;
      if (order == 1) {
        a[0] = -(alpha - 2.0) / (alpha + 2.0);
        b[0] = alpha / (alpha + 2.0);
        b[1] = alpha / (alpha + 2.0);
      }
      if (order == 2) {
        float alphaSq = alpha * alpha;
        float beta[] = {1, sqrt(2), 1};
        float D = alphaSq * beta[0] + 2 * alpha * beta[1] + 4 * beta[2];
        b[0] = alphaSq / D;
        b[1] = 2 * b[0];
        b[2] = b[0];
        a[0] = -(2 * alphaSq * beta[0] - 8 * beta[2]) / D;
        a[1] = -(beta[0] * alphaSq - 2 * beta[1] * alpha + 4 * beta[2]) / D;
      }
    }

    float filt(float xn) {
      // Provide me with the current raw value: x
      // I will give you the current filtered value: y
      if (adapt) {
        setCoef(); // Update coefficients if necessary
      }
      y[0] = 0;
      x[0] = xn;
      // Compute the filtered values
      for (int k = 0; k < order; k++) {
        y[0] += a[k] * y[k + 1] + b[k] * x[k];
      }
      y[0] += b[order] * x[order];

      // Save the historical values
      for (int k = order; k > 0; k--) {
        y[k] = y[k - 1];
        x[k] = x[k - 1];
      }

      // Return the filtered value
      return y[0];
    }
};

// Filter instance
LowPass<1> lp(9, 1e3, true);


void setup() {
  // put your setup code here, to run once:
  pinMode(led, OUTPUT);
  pinMode(pRes, INPUT);
  Serial.begin(115200);
  delay (1000);

  // Set encoder pins as inputs
  pinMode(CLK, INPUT);
  pinMode(DT, INPUT);
  pinMode(SW, INPUT_PULLUP);

  // Read the initial state of CLK
  lastStateCLK = digitalRead(CLK);

  // Call updateEncoder() when any high/low changed seen
  // on interrupt 0 (pin 2), or interrupt 1 (pin 3)
  attachInterrupt(0, updateEncoder, CHANGE);
  attachInterrupt(1, updateEncoder, CHANGE);
  attachInterrupt(2, Switch, FALLING);
}

void loop() {
  double des = vars[0];
  double Kp = vars[1];
  double Ki = vars[2];
  double Kd = vars[3];
  timeStep = (millis() - Time) / 1000;
  Time = timeStep + Time;
  if (ButtonPress == 4) {
    raw = analogRead(pRes);
    value = ((value * 15) + raw) / 16;
    value_filtered = lp.filt(value);
    //PID vars
    error = (des - value_filtered);
    derr = (value_old - value_filtered) / timeStep;
    errint = error * timeStep + errint;
    //output limit
    if (errint > outMax) errint = outMax;
    //else if (errint < outMin) errint = outMin;
    u = Kp * error + Ki * errint + Kd * derr;
    brightness = u + brightness;
    if (brightness > outMax) brightness = outMax;
    else if (brightness < outMin) brightness = outMin;
    value_old = value_filtered;
    //output signal
    analogWrite(led, brightness);
    Serial.println(value_filtered);
    delay(55);
  }
}

void updateEncoder() {

  // Read the current state of CLK
  currentStateCLK = digitalRead(CLK);
  // If last and current state of CLK are different, then pulse occurred
  // React to only 1 state change to avoid double count
  if (currentStateCLK != lastStateCLK  && currentStateCLK == 1) {
    String val = "";
    String sendit = "";
    if (ButtonPress == 0) {
      change = 0.1;
    } else if (ButtonPress == 2) {
      change = 0.001;
    }
    else {
      change = 0.01;
    }
    // If the DT state is different than the CLK state then
    // the encoder is rotating CCW so decrement
    if (digitalRead(DT) != currentStateCLK && ButtonPress != 4) {
      counter --;
      currentDir = "CCW";
      vars[ButtonPress] = vars[ButtonPress] - change;
    } else {
      // Encoder is rotating CW so increment
      counter ++;
      currentDir = "CW";
      vars[ButtonPress] = vars[ButtonPress] + change;
    }
    if (ButtonPress == 0) {
      val.concat(vars[0]);
      sendit = val + ": ";
      Serial.print(sendit);
      Serial.println("Desired_Brightness: ");
    }
    if (ButtonPress == 1) {
      val.concat(vars[1]);
      sendit = val + ": ";
      Serial.print(sendit);
      Serial.println("Kp: ");
    }
    if (ButtonPress == 2) {
      val.concat(vars[2]);
      sendit = val + ": ";
      Serial.print(sendit);
      Serial.println("Ki: ");
    }
    if (ButtonPress == 3) {
      val.concat(vars[3]);
      sendit = val + ": ";
      Serial.print(sendit);
      Serial.println("Kd: ");
    }
  }
  // Remember last CLK state
  lastStateCLK = currentStateCLK;
}

void Switch() {
  ButtonPress ++;
  if (ButtonPress == 5) {
    ButtonPress = 0;
  }
  brightness = 0;
  analogWrite(led, brightness);
  errint = 0;
  u = 0;
}

void SetOutput()
{
  raw = analogRead(pRes);
  value = ((value * 15) + raw) / 16;
  value_filtered = lp.filt(value);
  //PID vars
  error = (des - value_filtered);
  derr = (value_old - value_filtered) / timeStep;
  errint = error * timeStep + errint;
  //output limit
  if (errint > outMax) errint = outMax;
  //else if (errint < outMin) errint = outMin;
  u = Kp * error + Ki * errint + Kd * derr;
  brightness = u + brightness;
  if (brightness > outMax) brightness = outMax;
  else if (brightness < outMin) brightness = outMin;
  value_old = value_filtered;
  //output signal
  analogWrite(led, brightness);
  Serial.println(brightness);
  Serial.println(value_filtered);
}
