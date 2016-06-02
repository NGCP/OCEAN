/** Artificial Neural Network Library
  *
  * Author: Tyler Mau
  * Date: May 16, 2016
  */

#include "NeuralNetwork.h"

/* Neural Network Constructor */
NeuralNetwork::NeuralNetwork(int inputs, vector<int> hidden_config, int outputs) {
   /* Temporary Vector for Creating Layers */
   vector<Neuron> *layer;
   vector<float> *costs;

   //THROW ERRORS IF WRONG INPUTS

   /* Save Network Configuration */
   num_hidden_layers = hidden_config.size();
   num_inputs = inputs;
   num_outputs = outputs;

   /* Create Vector of Input Nodes */
   layer = new vector<Neuron>();
   costs = new vector<float>();
   for (int i = 0; i < num_inputs; i++) {
      (*layer).push_back(Neuron());
      (*costs).push_back(0);
   }
   input_layer = layer;
   layer_costs.push_back(costs);

   /* Create Vector of Hidden Layers */
   for (int i = 0; i < num_hidden_layers; i++) {
      layer = new vector<Neuron>();
      costs = new vector<float>();
      
      /* Create Vector of Nodes for Each Layer */
      for (int j = 0; j < hidden_config[i]; j++) {
         /* Previous Layer is a Hidden Layer */
         if (i) {
            (*layer).push_back(Neuron(hidden_layers[i - 1], HT_ACTIVATION));
         }
         /* Previous Layer is Input Layer */
         else {
            (*layer).push_back(Neuron(input_layer, HT_ACTIVATION));
         }
         (*costs).push_back(0);
      }
      hidden_layers.push_back(layer);
      layer_costs.push_back(costs);
   }

   /* Create Output Layer (Linear Activation Function) */
   layer = new vector<Neuron>();
   costs = new vector<float>();
   for (int i = 0; i < outputs; i++) {
      (*layer).push_back(Neuron(hidden_layers[num_hidden_layers - 1], LINEAR_ACTIVATION));
      (*costs).push_back(0);
   }
   hidden_layers.push_back(layer);
   layer_costs.push_back(costs);
   output_layer = layer;
}

/* Print Number of Neurons in Each Layer */
void NeuralNetwork::printNetworkSetup() {
   /* Print Size of Input Layer */
   cout << "Layer Configuration: " << num_inputs;

   /* Print Size of Each Hidden Layer */
   for (int i = 0; i < num_hidden_layers; i++) {
      cout << " " << (*hidden_layers[i]).size();
   }

   /* Print Size of Output Layer */
   cout << " " << num_outputs << endl << endl;
}

/* Print Weights and Biases of Entire Network */
void NeuralNetwork::printWB() {
   vector<float> weights;
   float bias;

   /* Iterate Through Hidden Layers */
   for (int i = 0; i < num_hidden_layers; i++) {
      cout << "Layer " << i << ":" << endl;

      /* Iterate Through Each Neuron */
      for (int j = 0; j < (*hidden_layers[i]).size(); j++) {
         cout << "   Neuron " << j << ":";

         /* Get Weights from Neuron */
         weights = (*hidden_layers[i])[j].getWeights();

         /* Print Weights */
         for (int k = 0; k < weights.size(); k++) {
            printf(" %5.4f", weights[k]);
         }

         /* Get Bias from Neuron */
         bias = (*hidden_layers[i])[j].getBias();

         /* Print Bias */
         printf("   Bias: %5.4f\n", bias);
      }
      cout << endl;
   }

   cout << "Output Layer:" << endl;

   /* Iterate Through Output Layer */
   for (int i = 0; i < num_outputs; i++) {
      cout << "   Neuron " << i << ":";

      /* Get Weights from Neuron */
      weights = (*output_layer)[i].getWeights();

      /* Print Weights */
      for (int j = 0; j < weights.size(); j++) {
         printf(" %5.4f", weights[j]);
      }

      /* Get Bias from Neuron */
      bias = (*output_layer)[i].getBias();

      /* Print Bias */
      printf("   Bias: %5.4f\n", bias);
   }
}

/* Calculate Network Output (Feedforward) */
vector<float> NeuralNetwork::calculate(vector<float> inputs) {
   /* Set Input Nodes */
   setInputs(inputs);

   /* Calculate Hidden Neuron Outputs */
   //cout << "Hidden: " << endl;
   calculateHidden();

   //cout << "Output: " << endl;
   /* Return Final Neural Network Outputs */
   return calculateOutputs();
}

/* Feed Inputs Into Network */
void NeuralNetwork::setInputs(vector<float> inputs) {
   for (int i = 0; i < (*input_layer).size(); i++) {
      //cout << inputs[i] << endl;
      (*input_layer)[i].setOutput(inputs[i]);
   }
}

/* Calculate Outputs of Hidden Neurons */
void NeuralNetwork::calculateHidden() {
   /* Iterate Through Hidden Layers */
   for (int i = 0; i < num_hidden_layers; i++) {

      /* Iterate Through Neurons */
      for (int j = 0; j < (*hidden_layers[i]).size(); j++) {
         /* Calculate Output of Neuron */
         (*hidden_layers[i])[j].calculate();
      }
   }
}

/* Calculate Outputs of Network */
vector<float> NeuralNetwork::calculateOutputs() {
   vector<float> output;

   /* Iterate Through Output Neurons */
   for (int i = 0; i < (*output_layer).size(); i++) {
      /* Calculate Output of Neuron */
      (*output_layer)[i].calculate();
      output.push_back((*output_layer)[i].getOutput());
   }

   return output;
}

/* Pre-Calculate Reusable Computations */
void NeuralNetwork::prepareUpdate(vector<float> training_output) {
   float error, phi_deriv;
   vector<float> weights;

   /* Calculate Activation Function Derivatives */
   for (int i = 0; i <= num_hidden_layers; i++) {
      for (int j = 0; j < (*hidden_layers[i]).size(); j++) {
         (*hidden_layers[i])[j].calculatePhiDeriv();
      }
   }

   /* Calculate Error Costs */
   for (int i = 0; i < num_outputs; i++) {
      error = training_output[i] - (*hidden_layers[num_hidden_layers])[i].getOutput();
      (*layer_costs[num_hidden_layers + 1])[i] = -error;
   }

   /* Calculate Layer Costs */
   for (int i = num_hidden_layers; i >= 0; i--) {
      for (int j = 0; j < (*layer_costs[i]).size(); j++) {

         (*layer_costs[i])[j] = 0;

         for (int k = 0; k < (*hidden_layers[i]).size(); k++) {
            weights = (*hidden_layers[i])[k].getWeights();
            phi_deriv = (*hidden_layers[i])[k].getPhiDeriv();
            (*layer_costs[i])[j] += (*layer_costs[i + 1])[k] * phi_deriv * weights[j];
         }
      }
   }   
}

/* Update Weights and Biases */
void NeuralNetwork::updateWB() {
   vector<float> weights;
   float gradient, phi_deriv, bias_gradient;

   for (int i = 0; i < hidden_layers.size(); i++) {
      for (int j = 0; j < (*hidden_layers[i]).size(); j++) {
         weights = (*hidden_layers[i])[j].getWeights();

         /* Iterate Through Weights */
         for (int k = 0; k < weights.size(); k++) {
            phi_deriv = (*hidden_layers[i])[j].getPhiDeriv();
            gradient = (*layer_costs[i + 1])[j] * phi_deriv * (*hidden_layers[i])[j].getInput(k);

            /* Update Weight */
            (*hidden_layers[i])[j].updateWeight(k, training_step * gradient);
         }

         /* Update Bias */
         bias_gradient = (*layer_costs[i + 1])[j] * phi_deriv;
         (*hidden_layers[i])[j].updateBias(training_step * bias_gradient);
      }
   }
}

/* Calculate MSE */
void NeuralNetwork::calculateMSE(vector< vector<float> > training_input, vector< vector<float> > training_output, ofstream &file) {
   int input_size = training_input.size();
   vector<float> outputs;
   float err_sum = 0, mse;

   for (int i = 0; i < input_size; i++) {
      outputs = calculate(training_input[i]);

      for (int j = 0; j < num_outputs; j++) {
         err_sum += training_output[i][j] - outputs[j];
      }
   }

   mse = (err_sum * err_sum) / (float)input_size;
   file << mse << endl;
}

/* Train Neural Network via Supervised Training */
void NeuralNetwork::train(float step, int epoch, vector< vector<float> > training_input, vector< vector<float> > training_output) {
   int size = training_input.size();
   int random;
   ofstream myfile;

   myfile.open("errors.txt");

   /* Set Training Step */
   training_step = step;

   /* Train According to Epoch */
   for (int i = 0; i < epoch; i++) {

      /* Randomize Training */
      random = rand() % size;

      /* Calculate Neural Network Output */
      calculate(training_input[random]);

      /* Prepare Neural Network for Updating */
      prepareUpdate(training_output[random]);

      /* Update Weights and Biases */
      updateWB();

      /* Print MSE to File */
      //calculateMSE(training_input, training_output, myfile);

      /* Print Error if Network Calculation Fails */
      if (errno == ERANGE) {
         cout << "Output Exeeds Range..." << endl;
         break;
      }
   }

   myfile.close();
}