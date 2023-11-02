/*
 * classify.ino
 *
 * EE16B Spring 2016
 * Emily Naviasky & Nathaniel Mailoa
 *
 * EE 16B Fall 2017
 * Andrew Blatner
 *
 */

#define MIC_INPUT                   P6_0

#define SIZE                        2752
#define SIZE_AFTER_FILTER           172
#define ADC_TIMER_MS                0.35

/*---------------------------*/
/*      CODE BLOCK PCA1      */
/*---------------------------*/

// Enveloping and threshold constants
#define SNIPPET_SIZE                  80
#define PRELENGTH                     5
#define THRESHOLD                     0.7

#define EUCLIDEAN_THRESHOLD         0.025
#define LOUDNESS_THRESHOLD          0.5

/*---------------------------*/
/*      CODE BLOCK PCA2      */
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

  pinMode(MIC_INPUT, INPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  re_pointer = 0;
  reset_blinker();
  setTimer();
}

void loop(void) {
  if (re_pointer == SIZE) {
    digitalWrite(RED_LED, LOW);

    // Apply enveloping function and get snippet with speech.
    // Do classification only if loud enough.
    if (envelope(re, result)) {

      // Reset projection result variables declared above
      proj1 = 0;
      proj2 = 0;

      /*---------------------------*/
      /*      CODE BLOCK PCA3      */
      /*---------------------------*/

      // Project 'result' onto the principal components
      // Hint: 'result' is an array
      // Hint: do this entire operation in 1 loop by replacing the '...'
      // YOUR CODE HERE
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
        Serial.println(arg[best_index]); 
      } else {
        Serial.println("The recording is noise");
      }
      /*---------------------------*/
      /*---------------------------*/
      /*---------------------------*/
    }
    else {
      Serial.println("Below LOUDNESS_THRESHOLD.");
    }

    delay(2000);
    re_pointer = 0;
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

void reset_blinker(void) {
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
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

/*---------------------------*/
/*    Interrupt functions    */
/*---------------------------*/

// ISR for timestep
#pragma vector=TIMER2_A0_VECTOR    // Timer A ISR
__interrupt void Timer2_A0_ISR(void) {
  if (re_pointer < SIZE) {
    digitalWrite(RED_LED, HIGH);
    re[re_pointer] = (analogRead(MIC_INPUT) >> 4) - 128;
    re_pointer += 1;
  }
}

// Set timer for timestep; use A2 since A0 & A1 are used by PWM
void setTimer(void) {
  // Set the timer based on 25MHz clock
  TA2CCR0 = (unsigned int) (25000*ADC_TIMER_MS);
  TA2CCTL0 = CCIE;
  __bis_SR_register(GIE);
  TA2CTL = TASSEL_2 + MC_1 + TACLR + ID_0;
}
