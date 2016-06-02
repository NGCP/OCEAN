/** Neuron Class
  *
  * Author: Tyler Mau
  * Date: May 16, 2016
  */

#ifndef NEURON_H

#include <iostream>
#include <vector>
#include <math.h>

#define HT_ACTIVATION 0
#define LINEAR_ACTIVATION 1

#define HYPERBOLIC_TANGENT_MAX 1.0f
#define HYPERBOLIC_TANGENT_SLOPE 1.0f

#define LINEAR_SCALE 1.0f
#define LEARNING_SCALE 0.2f

#define MOMENTUM true

using namespace std;

class Neuron {
   public:
      /* Default Neuron Constructor for Input Layer */
      Neuron();

      /* Neuron Constructor with Linear Activation Function */
      Neuron(vector<Neuron> *inputs);

      /* Neuron Constructor for Non-Input Layer */
      Neuron(vector<Neuron> *inputs, int activation_function);

      /* Neuron Constructor to Specify Weights/Biases */
      //Neuron(vector<Neuron> inputs, vector<float> weights, float bias, int activation_function);

      /* Return Weights of Neuron */
      vector<float> getWeights();

      /* Add to Weights for Updating */
      void updateWeight(int index, float value);

      /* Adjust Bias */
      void updateBias(float value);

      /* Return Bias of Neuron */
      float getBias();

      /* Return Last Calculated Output of Neuron */
      float getOutput();

      /* Return Calculated Derivative of Activation Function */
      float getPhiDeriv();

      /* Return Input at Index */
      float getInput(int index);

      /* Set Output Value of Neuron (Ex: Input Neurons)*/
      void setOutput(float value);

      /* Calculate Activation Derivative */
      void calculatePhiDeriv();

      /* Calculate Neuron Output */
      void calculate();

   private:
      /* Neuron Weights */
      vector<float> weights;

      /* Previous Weight Change */
      vector<float> d_weights;

      /* Neuron Bias */
      float bias;

      /* Previous Bias Change */
      float d_bias;

      /* Input Layer */
      vector<Neuron> *input_neurons;

      /* Calculated Output */
      float output;

      /* Activation Function */
      int activation;

      /* Derivative of Activation Function */
      float phi_deriv;

      /* Initialize Neuron */
      void initNeuron(vector<Neuron> *inputs, int activation_function);

      /* Hyperbolic Tangent Activation Function */
      float hyperbolic_tangent(float input, float abs_max, float slope);

      /* Derivative of Hyperbolic Tangent Activation Function */
      float hyperbolicTangentDerivative(float input);
};

#endif