/* Coordinate Descent Implementation
 *
 * Author: Tyler Mau
 * Date: 4/18/2016
 */

#include <stdlib.h>
#include <stdio.h>

#define INITIAL_STEP 1
#define STEP_THRESHOLD 0.0001f
#define STEP_SCALE_DOWN 0.9
#define STEP_SCALE_UP 1.1

#define NUM_PARAMS 3
#define P_PARAM 0
#define I_PARAM 1
#define D_PARAM 2

/* Run System and Return Error */
double run_system() {
   double error = 0;

   return error;
}

/* Return Summation of Gain Step Array */
double sum_gain_steps(double *steps) {
   double sum = 0;

   for (int i = 0; i < NUM_PARAMS; i++) {
      sum += steps[i];
   }

   return sum;
}

/* Print PID Gains */
void print_params(double *gains) {
   printf("KP: %lf   KI: %lf   KD: %lf\n", gains[P_PARAM], gains[I_PARAM], gains[D_PARAM]);
}

/* Print Gain Steps*/
void print_steps(double *steps) {
   printf("P_Step: %lf   I_Step: %lf   D_Step: %lf\n", steps[P_PARAM], steps[I_PARAM], steps[D_PARAM]);
}

/* Tune the PID Gains  */
void tune_PID(double **params, double **steps) {
   double min_error, cur_error;
   double step_sum = sum_gain_steps(*steps);

   min_error = run_system();

   /* Run Until Tuned (Based on Threshold) */
   while (step_sum > STEP_THRESHOLD) {
      for (int i = 0; i < NUM_PARAMS; i++) {
         (*params)[i] += (*steps)[i];
         cur_error = run_system();

         /* Decreasing Error */
         if (cur_error < min_error) {
            min_error = cur_error;
            (*steps)[i] *= STEP_SCALE_UP;
         }
         else {
            (*params)[i] -= 2.0 * (*steps)[i];
            cur_error = run_system();

            /* Decreasing Error */
            if (cur_error < min_error) {
               min_error = cur_error;
               (*steps)[i] *= STEP_SCALE_UP;
            }
            /* Decrease Step Size */
            else {
               (*params)[i] += (*steps)[i];
               (*steps)[i] *= STEP_SCALE_DOWN;
            }
         }
      }
      print_steps(*steps);
      step_sum = sum_gain_steps(*steps);
   }
}

/* Initialize PID Tuning */
void init_tuning(double **gain_steps) {
   for (int i = 0; i < NUM_PARAMS; i++) {
      (*gain_steps)[i] = INITIAL_STEP;
   }
}

/* Main: Tune PID Controller */
int main() {
   double *pid_gains = calloc(NUM_PARAMS, sizeof(double));
   double *gain_steps = calloc(NUM_PARAMS, sizeof(double));

   init_tuning(&gain_steps);
   tune_PID(&pid_gains, &gain_steps);
   print_params(pid_gains);

	return 0;
}