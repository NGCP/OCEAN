/** Artificial Neural Network Library Test
  *
  * Author: Tyler Mau
  * Date: May 16, 2016
  */

#include <iostream>
#include <fstream>
#include "NeuralNetwork.h"
#define HIDDEN_LAYERS 1

/* Tests Manufacturer Data for Thrusters */
void testThruster() {
   vector<int> hidden_layers(HIDDEN_LAYERS);

   vector< vector<float> > training_input;
   vector< vector<float> > training_output;

   vector<float> *input;
   vector<float> *output;

   int temp;

   ofstream myfile;
   myfile.open("output.txt");

   float p[] = {1500,  1510,  1520,  1530, 1540, 1550, 1500,  1510,  1520,  1530, 1540, 1550, 1560, 1570, 1580, 1590, 1600, 1610, 1620, 1630, 1640, 1650, 1660, 1670, 1680, 1690, 1700, 1710, 1720, 1730, 1740, 1750, 1760, 1770, 1780, 1790, 1800, 1810, 1820, 1830, 1840, 1850, 1860, 1870, 1880, 1890, 1900, 1530, 1520, 1510, 1500, 1490, 1480, 1470, 1460,  1450,  1440,  1430,  1420,  1410,  1400,  1390,  1380,  1370,  1360, 1350,  1340,  1330,  1320,  1310,  1300,  1290,  1280,  1270,  1260,  1250,  1240,  1230,  1220,  1210,  1200,  1190,  1180,  1170,  1160,  1150,  1140,  1130,  1120,  1110,  1100};
   float t[] = {-0.04, -0.04, -0.04, 0.02, 0.08, 0.14, -0.04, -0.04, -0.04, 0.02, 0.08, 0.14, 0.23, 0.32, 0.43, 0.56, 0.68, 0.81, 0.95, 1.06, 1.2,  1.35, 1.51, 1.62, 1.77, 1.94, 2.12, 2.33, 2.49, 2.61, 2.76, 2.87, 3.05, 3.28, 3.47, 3.64, 3.8,  4.01, 4.31, 4.5,  4.62, 4.8,  4.98, 5.11, 5.32, 5.47, 5.54, 0.15, 0.08, 0.06, 0.06, 0.05, 0.05, 0.02, -0.03, -0.07, -0.11, -0.16, -0.23, -0.31, -0.37, -0.44, -0.51, -0.6, -0.72, -0.81, -0.88, -0.94, -1.04, -1.17, -1.28, -1.37, -1.47, -1.55, -1.66, -1.74, -1.88, -2.04, -2.18, -2.29, -2.43, -2.56, -2.66, -2.77, -2.91, -3.08, -3.17, -3.36, -3.47, -3.61, -3.89};
  
   int p_size = sizeof(p) / sizeof(*p);

   /* Create Sample Training Input and Output */
   for (int i = 0; i < p_size; i++) {

      input = new vector<float>();
      output = new vector<float>();

      (*input).push_back(p[i] / 100.0);
      (*output).push_back(t[i]);

      training_input.push_back(*input);
      training_output.push_back(*output);
   }

   hidden_layers[0] = 15;

   NeuralNetwork net(1, hidden_layers, 1);

   net.printNetworkSetup();

   cout << "After Training:" << endl;
   for (float i = 1100; i < 1910; i += 10) {
      vector<float> test_input, test_output;
      test_input.clear();
      test_input.push_back(i / 100.0);

      test_output = net.calculate(test_input);

      cout << "Input: " << test_input[0] * 100 << "   Output: " << test_output[0] << endl;
   }
   net.train(0.001, 1000000, training_input, training_output);

   cout << "\nAfter Training:" << endl;
   for (float i = 1100; i < 1910; i += 10) {
      vector<float> test_input, test_output;
      test_input.clear();
      test_input.push_back(i / 100.0);

      test_output = net.calculate(test_input);

      myfile << test_output[0] << endl;

      cout << "Input: " << test_input[0] * 100 << "   Output: " << test_output[0] << endl;
   }
}

/* Tests Neural Network Using XOR Problem */
void testXOR() {
   vector<int> hidden_layers(HIDDEN_LAYERS);

   vector<float> *input;
   vector<float> *output;

   vector< vector<float> > training_input;
   vector< vector<float> > training_output;

   float xor_p[4][2] = {{0, 0}, {0, 1}, {1, 0}, {1, 1}};
   float xor_t[] = {0, 1, 1, 0};

   /* Create Sample Training Input and Output */
   for (int i = 0; i < 4; i++) {

      input = new vector<float>();
      output = new vector<float>();

      for (int j = 0; j < 2; j++) {
         (*input).push_back(xor_p[i][j]);
      }

      (*output).push_back(xor_t[i]);

      training_input.push_back(*input);
      training_output.push_back(*output);
   }

   hidden_layers[0] = 5;

   NeuralNetwork net(2, hidden_layers, 1);

   net.printNetworkSetup();

   cout << "Before Training:" << endl;
   net.printWB();

   cout << "\nNetwork Test: " << endl;
   for (int i = 0; i < 4; i++) {
      vector<float> test_input, test_output;
      test_input.clear();

      for (int j = 0; j < 2; j++) {
         test_input.push_back(xor_p[i][j]);
      }

      test_output = net.calculate(test_input);

      printf("   Input: %d %d   Output: %.5f\n", (int)test_input[0], (int)test_input[1], test_output[0]);
   }

   cout << "\n-------------------------------------------------------------------" << endl;

   net.train(0.01, 10000, training_input, training_output);

   cout << "\nAfter Training:" << endl;
   net.printWB();

   cout << "\nNetwork Test: " << endl;
   for (int i = 0; i < 4; i++) {
      vector<float> test_input, test_output;
      test_input.clear();

      for (int j = 0; j < 2; j++) {
         test_input.push_back(xor_p[i][j]);
      }

      test_output = net.calculate(test_input);

      printf("   Input: %d %d   Output: %.5f\n", (int)test_input[0], (int)test_input[1], test_output[0]);
   }
   cout << endl;
}

/* Main */
int main() {
   testXOR();
   //testThruster();

   return 0;
}