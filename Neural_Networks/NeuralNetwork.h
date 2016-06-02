/** Artificial Neural Network Library
  *
  * Author: Tyler Mau
  * Date: May 16, 2016
  */

#ifndef NEURAL_NETWORK_H

#include <iostream>
#include <fstream>
#include <vector>

#include "Neuron.h"

using namespace std;

/* Neural Network Class */
class NeuralNetwork {
   public:
      /* Neural Network Constructor */
      NeuralNetwork(int inputs, vector<int> hidden_layers, int outputs);

      /* Print Number of Neurons in Each Layer */
      void printNetworkSetup();

      /* Print Weights of Entire Network */
      void printWB();

      /* Calculate Network Output (Feedforward) */
      vector<float> calculate(vector<float> inputs);

      /* Train Neural Network via Supervised Training */
      void train(float step, int epoch, vector< vector<float> > training_input, vector< vector<float> > training_output);

   private:
      /* Number of Network Inputs */
      int num_inputs;

      /* Number of Network Outputs */
      int num_outputs;

      /* Number of Hidden Layers */
      int num_hidden_layers;

      /* Training Step */
      float training_step;

      /* Vector of Hidden Layers (Includes Output Layer) */
      vector< vector<Neuron>* > hidden_layers;

      /* Contains Input Neurons */
      vector<Neuron> *input_layer;

      /* Contains Output Neurons */
      vector<Neuron> *output_layer;

      /* Holds Calculation of Layer Cost */
      vector< vector<float>* > layer_costs;

      /* Feed Inputs Into Network */
      void setInputs(vector<float> inputs);

      /* Calculate Outputs of Hidden Neurons */
      void calculateHidden();

      /* Calculate Outputs of Network */
      vector<float> calculateOutputs();

      /* Pre-Calculate Reusable Computations */
      void prepareUpdate(vector<float> training_output);

      /* Update Weights and Biases */
      void updateWB();

      /* Calculate MSE */
      void calculateMSE(vector< vector<float> > training_input, vector< vector<float> > training_output, ofstream &file);
};

#endif