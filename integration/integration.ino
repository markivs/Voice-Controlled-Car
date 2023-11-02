/*
 * integration.ino
 * Final sketch for SIXT33N Speech version
 *
 * EE16B Fall 2016
 * Emily Naviasky & Nathaniel Mailoa
 *
 * EE 16B Fall 2017
 * Andrew Blatner
 *
 */

/********************************************************/
/********************************************************/
/***                                                  ***/
/*** Constants and global variables from turning.ino. ***/
/***                                                  ***/
/********************************************************/
/********************************************************/

// Change pins here if you did not use the default pins!
#define LEFT_MOTOR                  P1_4
#define LEFT_ENCODER                P6_2
#define RIGHT_MOTOR                 P2_5
#define RIGHT_ENCODER               P6_3
#define PUSH_START                  PUSH1

#define SAMPLING_INTERVAL           100
int sample_lens[4] = {0};

// Operation modes
#define MODE_LISTEN                 0
#define MODE_DRIVE                  1
int timer_mode = MODE_LISTEN;

#define DRIVE_FAR                   0
#define DRIVE_LEFT                  1
#define DRIVE_CLOSE                 2
#define DRIVE_RIGHT                 3

#define JOLT_STEPS                  2

boolean loop_mode = MODE_DRIVE;
int drive_mode = 0;

int step_num = 0;
volatile boolean do_loop = 0; // timer signal to increment timestep

typedef struct encoder {
  int pin;
  int pos;
  bool level;
  int avg;
} encoder_t;

encoder_t left_encoder = {LEFT_ENCODER, 0, LOW, 0};
encoder_t right_encoder = {RIGHT_ENCODER, 0, LOW, 0};

/*---------------------------*/
/*      CODE BLOCK CON1      */
/*      From turning.ino     */
/*---------------------------*/

float theta_left = 0.1311;
float theta_right = 0.1529;
float beta_left = -58.99;
float beta_right = -56.24;
float v_star = 77.3;

// PWM inputs to jolt the car straight
int left_jolt = 200;
int right_jolt = 185;

// Control gains
float f_left = 0.5;
float f_right = 0.5;

/*---------------------------*/
/*      CODE BLOCK CON2      */
/*      From turning.ino     */
/*---------------------------*/

float driveStraight_left(float delta) {
  float u_left = ((v_star + beta_left) - (f_left*delta))/theta_left;
  return u_left;
}

float driveStraight_right(float delta) {
  float u_right = ((v_star + beta_right) + (f_right*delta))/theta_right;
  return u_right;
}

/*---------------------------*/
/*      CODE BLOCK CON3      */
/*      From turning.ino     */
/*---------------------------*/

float delta_ss = 0;

/*---------------------------*/
/*      CODE BLOCK CON4      */
/*      From turning.ino     */
/*---------------------------*/

#define CAR_WIDTH                   15.0 // in cm
#define TURN_RADIUS                 91 // in cm - 6 feet diameter = 3 tiles in 125 Cory
// #define TURN_RADIUS                 60 // in cm - 4 feet diameter = 2 tiles in 125 Cory

int run_times[4] = {7000, 2500, 2500, 2500}; // {DRIVE_FAR, DRIVE_LEFT, DRIVE_CLOSE, DRIVE_RIGHT}

float delta_reference(int n) {
  // Remember to divide the v* value you use in this function by 5 because of sampling interval differences!
  if (drive_mode == DRIVE_RIGHT) { // Return a NEGATIVE expression
    return (TURN_RADIUS-CAR_WIDTH/2) * n * v_star/(TURN_RADIUS*5) - (TURN_RADIUS+CAR_WIDTH/2) * n * v_star/(TURN_RADIUS*5);

  }
  else if (drive_mode == DRIVE_LEFT) { // Return a POSITIVE expression
    return (TURN_RADIUS+CAR_WIDTH/2) * n * v_star/(TURN_RADIUS*5) - (TURN_RADIUS-CAR_WIDTH/2) * n * v_star/(TURN_RADIUS*5);
  }
  else { // DRIVE_STRAIGHT
    return 0;
  }
}

/*---------------------------*/
/*      CODE BLOCK CON5      */
/*      From turning.ino     */
/*---------------------------*/
#define INFINITY                    (3.4e+38)
#define STRAIGHT_RADIUS             INFINITY

float straight_correction(int n) {
  // YOUR CODE HERE
  return 0;
}

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/

/*********************************************************/
/*********************************************************/
/***                                                   ***/
/*** Constants and glboal variables from classify.ino. ***/
/***                                                   ***/
/*********************************************************/
/*********************************************************/

// Change pin here if you did not use the default pin!
#define MIC_INPUT                   P6_0

#define SIZE                        3200
#define SIZE_AFTER_FILTER           200
#define ADC_TIMER_MS                0.35

/*---------------------------*/
/*      CODE BLOCK PCA1      */
/*     From classify.ino     */
/*---------------------------*/

// Enveloping and threshold constants
#define SNIPPET_SIZE                  80
#define PRELENGTH                     5
#define THRESHOLD                     0.7

#define EUCLIDEAN_THRESHOLD         0.025
#define LOUDNESS_THRESHOLD          0.5

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/


/*---------------------------*/
/*      CODE BLOCK PCA2      */
/*     From classify.ino     */
/*---------------------------*/

float pca_vec1[80] = {-0.02414337499362068, -0.03341134811196994, 0.02618844819108862, 0.07565538948197947, 0.1109946004307667, 0.23675558088153895, 0.1916046294015512, 0.19373509314486637, 0.26515803005481076, 0.15763495882166614, 0.21391926888770058, 0.17885929430135716, 0.15593952995172783, 0.21574736175679127, 0.15691552426830083, 0.23654869714045326, 0.1711228516919306, 0.26609455715117836, 0.11872414300999189, 0.2472888145170934, 0.112123953596659, 0.14019484516120642, 0.062268008137709815, 0.027926032556053512, 0.00947156677553519, -0.004172340746255303, -0.026716364865090162, -0.03340441581568929, -0.0354847595506391, -0.048021832956621366, -0.06702124557474834, -0.060006479723869285, -0.08705771702696725, -0.0722401145788623, -0.080963315700087, -0.07110407217909936, -0.055910178749393316, -0.060082028603986065, -0.04274677121303592, -0.04434888404184165, -0.03134273427189571, -0.00997971030244285, -0.006390033306951707, -0.004516162678204693, -0.03344812691728335, -0.026057298146409124, -0.04803413727061034, -0.06135818687999913, -0.08646866045838836, -0.09297259516056693, -0.12679959277507846, -0.11918841202184674, -0.13207789294164957, -0.12195331105647401, -0.17262761981978225, -0.13623743806192362, -0.14492566134662557, -0.15438882672320556, -0.09863291175120763, -0.1095289727927016, -0.09971266201786398, -0.07493300258831147, -0.10684589238869593, -0.06244736997157758, -0.0866338766430149, -0.05187343541210118, -0.05002918921502242, -0.04688204984421531, -0.030683560426307207, -0.04588137108947359, -0.028568990502037418, -0.04162654143472646, -0.036749860349275264, -0.03802516704663409, -0.023626596855871748, -0.0343958129673005, -0.028145996806505737, -0.04141024648626423, -0.03771944056301134, -0.04091658758872267};
float pca_vec2[80] = {-0.06956360795854953, -0.05087950425297705, -0.06292210109955271, -0.14746031508548413, -0.0697865287470733, -0.21655563693536184, -0.11409132456647614, -0.20262657653821614, -0.18665911788218104, -0.1344602626777034, -0.20676606011059814, -0.12320814338302544, -0.14722679361839977, -0.048376171543103585, -0.04245903123150001, 0.05396310455401801, 0.10538599111308071, 0.09883874461723743, 0.20286999883138981, 0.152130160805959, 0.24466982231073864, 0.2160568286530326, 0.23006787752733748, 0.23976013975430527, 0.20234334276765312, 0.18819551552899969, 0.18121349345200805, 0.15383585590718535, 0.14079726179065621, 0.13144773667273915, 0.07204662374795497, 0.07179252938098983, 0.021403813129132497, 0.06362062247197015, 0.06116528845223772, 0.07599414411067529, 0.05934301537708086, 0.07831352687288116, 0.07649654593282025, 0.06646777180994821, 0.06747505316377612, 0.07121303031972083, 0.06238396119937813, 0.06360659664066654, 0.03806036623828721, 0.042472860895576475, 0.04871512974888288, 0.04190103220825809, 0.020475785641894492, 0.007172028046982283, -0.005720449634708684, -0.03600608017747282, -0.04128979008658331, -0.07837698818654394, -0.0950420707050888, -0.12167983179656164, -0.14269296788178712, -0.1325686857939011, -0.12933857939743462, -0.18027403779257622, -0.12369128525051389, -0.12652598835743595, -0.09439296822135665, -0.07435591120466845, -0.050315738492109456, -0.0337846772667987, -0.012396888545217796, -0.02832446625524864, -0.02620815022863055, -0.02466830629630055, -0.028707849611400185, -0.028035206689919923, -0.028100772519411846, -0.014758463245675845, -0.029175294829225967, -0.04146849066421208, -0.028295613664339164, -0.027971526223472362, -0.018712742484086043, -0.02577460254257003};
float projected_mean_vec[3] = {0.0501274254596583, -0.021676165265076826, -0.010571887683466077};
float centroid1[3] = {-0.00044456662981059124, 0.03501455469320625, 0.008827484167260317};
float centroid2[3] = {-0.02431824076383755, -0.01770571673277064, 0.01816660215412807};
float centroid3[3] = {-0.02140120583154765, -0.007866765768278654, -0.025620963476712676};
float centroid4[3] = {0.035845562651383764, -0.00794694163187342, -0.0021805766366070354};
float* centroids[4] = {
  (float *) &centroid1, (float *) &centroid2,
  (float *) &centroid3, (float *) &centroid4
};


/*---------------------------*/
/*---------------------------*/
/*---------------------------*/

float result[SNIPPET_SIZE] = {0};
float proj1 = 0;
float proj2 = 0;

// Data array and index pointer
int re[SIZE] = {0};
volatile int re_pointer = 0;

/*---------------------------------------------------*/
/*---------------------------------------------------*/
/*---------------------------------------------------*/

/*---------------------------*/
/*       Norm functions      */
/*---------------------------*/

// Compute the L2 norm of (dim1, dim2) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        centroid: size-2 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm(float dim1, float dim2, float* centroid) {
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2));
}

// Compute the L2 norm of (dim1, dim2, dim3) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        dim3: 3rd dimension coordinate
//        centroid: size-3 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm3(float dim1, float dim2, float dim3, float* centroid) {
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2) + pow(dim3-centroid[2],2));
}

void setup(void) {
  Serial.begin(38400);

  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(LEFT_ENCODER, INPUT);
  pinMode(RIGHT_MOTOR, OUTPUT);
  pinMode(RIGHT_ENCODER, INPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(MIC_INPUT, INPUT);

  for (int i = 0; i < 4; i++) {
    sample_lens[i] = run_times[i] / SAMPLING_INTERVAL;
  }

  write_pwm(0, 0);
  delay(2000); // Wait 2 seconds to put down car
  reset_blinker();
  start_listen_mode();
}

void loop(void) {
  check_encoders();
  if (timer_mode == MODE_LISTEN && re_pointer == SIZE){
    // Stop motor
    write_pwm(0, 0);
    digitalWrite(RED_LED, LOW);

    // if enveloped data is above some preset value
    if (envelope(re, result)) {

      // Reset projection result variables declared above
      proj1 = 0;
      proj2 = 0;

      /*---------------------------*/
      /*      CODE BLOCK PCA3      */
      /*     From classify.ino     */
      /*     with more changes     */
      /*---------------------------*/

      for (int i = 0; i < SNIPPET_SIZE; i++) {
          proj1 += result[i]*pca_vec1[i];
          proj2 += result[i]*pca_vec2[i];
      }

      // Demean the projection
      proj1 -= projected_mean_vec[0];
      proj2 -= projected_mean_vec[1];

      // Classification
      // Use the function 'l2_norm' defined above
      // ith centroid: 'centroids[i]'
      float best_dist = 999999;
      int best_index = -1;
      float arr[4] = {0};
      for (int i = 0; i < 4; i++) {
          arr[i] = l2_norm(proj1, proj2, centroids[i]);
          if (arr[i] < best_dist) {
            best_dist = arr[i];
            best_index = i;
          }
      }

      Serial.println(proj1);
      Serial.println(proj2);
      Serial.println(best_dist);

      String arg[4] = {"forward", "multiplicity", "contradicting", "port"};
      if (best_dist < EUCLIDEAN_THRESHOLD) {
        drive_mode = best_index; // from 0-3, inclusive
        start_drive_mode();
      }

      /*---------------------------*/
      /*---------------------------*/
      /*---------------------------*/
    }

    delay(2000);
    re_pointer = 0; // start recording from beginning if we don't start driving
  }

  else if (loop_mode == MODE_DRIVE && do_loop) {
    if (step_num < JOLT_STEPS) {
      write_pwm(left_jolt, right_jolt);
    }
    else {

      // Save positions because _left_position and _right_position
      // can change in the middle of one loop.
      int left_position = left_encoder.pos;
      int right_position = right_encoder.pos;

      /*---------------------------*/
      /*      CODE BLOCK CON0      */
      /*---------------------------*/

      float delta = left_position - right_position + delta_ss;
      delta = delta + delta_reference(step_num) + straight_correction(step_num);

      // Drive straight using feedback
      // Compute the needed pwm values for each wheel using delta and v_star
      int left_cur_pwm = driveStraight_left(delta);
      int right_cur_pwm = driveStraight_right(delta);
      write_pwm(left_cur_pwm, right_cur_pwm);

      /*---------------------------*/
      /*---------------------------*/
      /*---------------------------*/
    }

    // Counter for how many times loop is executed since entering DRIVE MODE
    step_num++;

    if (step_num == sample_lens[drive_mode]) {
      // Completely stop and go back to listen MODE after 3 seconds
      start_listen_mode();
    }

    do_loop = 0;
  }
}

// Enveloping function with thresholding and normalizing,
// returns snippet of interest (containing speech)
bool envelope(int* data, float* data_out) {
  int32_t avg = 0;
  float maximum = 0;
  int32_t total = 0;
  int block;

  // Apply enveloping filter while finding maximum value
  for (block = 0; block < SIZE_AFTER_FILTER; block++) {
    avg = 0;
    for (int i = 0; i < 16; i++) {
      avg += data[i+block*16];
    }
    avg = avg >> 4;
    data[block] = abs(data[block*16] - avg);
    for (int i = 1; i < 16; i++) {
      data[block] += abs(data[i+block*16] - avg);
    }
    if (data[block] > maximum) {
      maximum = data[block];
    }
  }

  // If not loud enough, return false
  if (maximum < LOUDNESS_THRESHOLD) {
    return false;
  }

  // Determine threshold
  float thres = THRESHOLD * maximum;

  // Figure out when interesting snippet starts and write to data_out
  block = PRELENGTH;
  while (data[block++] < thres);
  if (block > SIZE_AFTER_FILTER - SNIPPET_SIZE) {
    block = SIZE_AFTER_FILTER - SNIPPET_SIZE;
  }
  for (int i = 0; i < SNIPPET_SIZE; i++) {
    data_out[i] = data[block-PRELENGTH+i];
    total += data_out[i];
  }

  // Normalize data_out
  for (int i = 0; i < SNIPPET_SIZE; i++) {
    data_out[i] = data_out[i] / total;
  }

  return true;
}

/*---------------------------*/
/*     Helper functions      */
/*---------------------------*/

void write_pwm(int pwm_left, int pwm_right) {
  analogWrite(LEFT_MOTOR, (int) min(max(0, pwm_left), 255));
  analogWrite(RIGHT_MOTOR, (int) min(max(0, pwm_right), 255));
}

void reset_blinker(void) {
  digitalWrite(RED_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, LOW);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(GREEN_LED, LOW);
}

void start_listen_mode(void) {
  re_pointer = 0;
  write_pwm(0, 0);
  delay(3000); // 3 seconds buffer for mic cap settling
  timer_mode = MODE_LISTEN;
  setTimer(MODE_LISTEN);
}

void start_drive_mode(void) {
  timer_mode = MODE_DRIVE;
  step_num = 0;
  left_encoder.pos = 0;
  right_encoder.pos = 0;
  setTimer(MODE_DRIVE);
}

/*---------------------------*/
/*    Interrupt functions    */
/*---------------------------*/

#define AVG_DECAY_RATE              0.3
#define LOW_THRESH                  ((int) (0.1*4096))
#define HIGH_THRESH                 ((int) (0.4*4096))

void check_encoder(encoder_t* enc) {
  int new_val = analogRead(enc->pin);
  enc->avg = (int) (AVG_DECAY_RATE*enc->avg + (1 - AVG_DECAY_RATE)*new_val);
  if ((enc->level == LOW && HIGH_THRESH < enc->avg) ||
      (enc->level == HIGH && enc->avg < LOW_THRESH)) {
    enc->pos++;
    enc->level = !enc->level;
  }
}

void check_encoders(void) {
  check_encoder(&left_encoder);
  check_encoder(&right_encoder);
}

// Set timer for timestep; use B0 to free up all other PWM ports
void setTimer(boolean mode) {
  if (mode == MODE_LISTEN) {
    // Set the timer based on 25MHz clock
    TB0CCR0 = (unsigned int) (25000*ADC_TIMER_MS);
    TB0CCTL0 = CCIE;
    __bis_SR_register(GIE);
    TB0CTL = TASSEL_2 + MC_1 + TACLR + ID_0;
  }
  else if (mode == MODE_DRIVE) {
    TB0CCR0 = (unsigned int) (32.768*SAMPLING_INTERVAL); // set the timer based on 32kHz clock
    TB0CCTL0 = CCIE; // enable interrupts for Timer B
    __bis_SR_register(GIE);
    TB0CTL = TASSEL_1 + MC_1 + TACLR + ID_0;
  }
  timer_mode = mode;
}

// ISR for timestep
#pragma vector=TIMER0_B0_VECTOR    // Timer B ISR
__interrupt void Timer0_B0_ISR(void) {
  if (timer_mode == MODE_LISTEN) {
    if (re_pointer < SIZE) {
      digitalWrite(RED_LED, HIGH);
      re[re_pointer] = (analogRead(MIC_INPUT) >> 4) - 128;
      re_pointer += 1;
    }
  }
  else if (timer_mode == MODE_DRIVE) {
    do_loop = 1;
  }
}
