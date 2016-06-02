/** Neuron Class
  *
  * Author: Tyler Mau
  * Date: May 16, 2016
  */

#include "Neuron.h"

/* Default Neuron Constructor for Input Layer */
Neuron::Neuron() {
}

/* Neuron Constructor with Linear Activation Function */
Neuron::Neuron(vector<Neuron> *inputs) {
   initNeuron(inputs, LINEAR_ACTIVATION);
}

/* Neuron Constructor for Non-Input Layer */
Neuron::Neuron(vector<Neuron> *inputs, int activation_function) {
   initNeuron(inputs, activation_function);
}

/* Neuron Constructor to Specify Weights/Biases */
/*Neuron::Neuron(vector<Neuron> inputs, vector<float> weights, float bias, int activation_function) {
   activation = activation_function;
}*/

/* Initialize Neuron */
void Neuron::initNeuron(vector<Neuron> *inputs, int activation_function) {
  input_neurons = inputs;

   for (int i = 0; i < (*input_neurons).size(); i++) {
      weights.push_back(rand() / (float)RAND_MAX);
      d_weights.push_back(0);
   }

   bias = rand() / (float)RAND_MAX;
   d_bias = 0;

   activation = activation_function;

   output = 0;
}


/* Return Weights of Neuron */
vector<float> Neuron::getWeights() {
   return weights;
}

/* Adjust Weights */
void Neuron::updateWeight(int index, float value) {
   float dw;

   if (MOMENTUM) {
      dw = value + LEARNING_SCALE * d_weights[index];
      weights[index] -= dw;
      d_weights[index] = dw;
   }
   else {
      weights[index] -= value;
   }
}

/* Adjust Bias */
void Neuron::updateBias(float value) {
   float db;

   if (MOMENTUM) {
      db = value + LEARNING_SCALE * d_bias;
      bias -= db;
      d_bias = db;
   }
   else {
      bias -= value;
   }
}

/* Return Bias of Neuron */
float Neuron::getBias() {
   return bias;
}

/* Return Last Calculated Output of Neuron */
float Neuron::getOutput() {
   return output;
}

/* Return Calculated Derivative of Activation Function */
float Neuron::getPhiDeriv() {
   return phi_deriv;
}

/* Return Input at Index */
float Neuron::getInput(int index) {
   return (*input_neurons)[index].getOutput();
}

/* Set Output Value of Neuron (Ex: Input Neurons)*/
void Neuron::setOutput(float value) {
   output = value;
}

/* Calculate Activation Derivative */
void Neuron::calculatePhiDeriv() {
   if (activation == HT_ACTIVATION) {
      phi_deriv = hyperbolicTangentDerivative(output);
   }
   else {
      phi_deriv = LINEAR_SCALE;
   }
}

/* Calculate Neuron Output */
void Neuron::calculate() {
   float sum = 0;

   for (int i = 0; i < (*input_neurons).size(); i++) {
      sum += (*input_neurons)[i].getOutput() * weights[i];
   }

   sum += bias;

   if (activation == HT_ACTIVATION) {
      output = hyperbolic_tangent(sum, HYPERBOLIC_TANGENT_MAX, HYPERBOLIC_TANGENT_SLOPE);
   }
   else {
      output = sum * LINEAR_SCALE;
   }

   //Check if output is too large
}

/* Hyperbolic Tangent Activation Function */
float Neuron::hyperbolic_tangent(float input, float abs_max, float slope) {
   float exp_pos = exp(slope * input);
   float exp_neg = exp(-slope * input);

   if (exp_pos == HUGE_VALF || exp_neg == HUGE_VALF) {
      if (input > 0) {
         output = abs_max;
      }
      else {
         output = -abs_max;
      }
   }
   else {
      output = abs_max * ((exp_pos - exp_neg) / (exp_pos + exp_neg));
   }

   return output;
   //return tanh(input);
}

/* Derivative of Hyperbolic Tangent Activation Function */
float Neuron::hyperbolicTangentDerivative(float input) {
   return (HYPERBOLIC_TANGENT_SLOPE / HYPERBOLIC_TANGENT_MAX) *
          (HYPERBOLIC_TANGENT_MAX - input) *
          (HYPERBOLIC_TANGENT_MAX + input);
}