#include <iostream>
#include <vector>

using std::cout;
using std::endl;

double motionProbability(bool motion, int previous_position, int current_position)
{
    // Given the motion input and distance, return the probability

  // Calculate the distance from previous_position to current position
  int dist = current_position - previous_position;
  if (dist < 0)
  {
    dist = dist + 20;
  }
	if (!motion)
	{
    if (dist == 0)
    {
      return 1.;
    }
    else
    {
      return 0.;
    }
		
  }
  else
  {
    if (dist == 1)
    {
      return 0.7;
    }

    if (dist == 2)
    {
      return 0.3;
    }
  }
}

double observationProbability(bool observation, int position)
{
  // Given the observation input and position, return the probability
  if (position == 2 || position == 4 || position == 7)
  {
    if (observation)
    {
      return 0.8;
    }
    else
    {
      return 0.2;
    }
  }
  else
  {
    if (observation)
    {
      return 0.1;
    }
    else
    {
      return 0.9;
    }
  }
}

void normaliseState(std::vector<double>& state)
{
  //To normalise, take sum then divide individual probs by sum
  double sum = 0;
  for (double prob : state)
  {
    sum = sum + prob;
  }
  for (double& normprob : state)
  {
    normprob = normprob / sum;
  }
}

void initialiseState(std::vector<double>& state)
{
  // Fill the state variable with initial probabilities
  
  //Calc initial probability, given by 1/number of positions  
  double numpos = 20.;
  // double initprob = 1./numpos;
  double initprob = 1.;

  //Assign initial probability
  for (double& pos : state)
  {
    pos = initprob;
  }

  normaliseState(state);
}

std::vector<double> updateState(const std::vector<double>& previous_state, bool motion, bool observation)
{
  // Declare a new state vector with the same size as previous_state
  // All values in state are initialised with 0.
  std::vector<double> state(previous_state.size());

  //Calc probability of landing in position
  //For all positions in vector
  for (int pos = 0; pos < previous_state.size(); pos++)
  {
  	//Origin if step length is one
    int onestep = pos - 1;

    //Wraparound vector
    if (onestep < 0)
    {
      onestep = onestep + 20;
    }

    //Origin if step length is two
    int twostep = onestep - 1;
    if (twostep < 0)
    {
      //Wraparound vector
      twostep = twostep + 20;
    }

    //Calculate probability of staying in location
    double probstay = previous_state[pos] * motionProbability(motion, pos, pos);

    //Calculate probability of arriving from one step
    double oneprob = previous_state[onestep] * motionProbability(motion, onestep, pos);

    //Calculate probability of arriving from two steps
    double twoprob = previous_state[twostep] * motionProbability(motion, twostep, pos);

    //Sum probabilities
    double moveprob = probstay + oneprob + twoprob;

    //Calculate probability of being in given location once observation is taken into account
    state[pos] = moveprob * observationProbability(observation, pos);
  }

  // Normalise
  normaliseState(state);
  
  return state;
}
void printState(const std::vector<double>& state)
{
  cout << "Position:    ";

  for (int i = 0; i < (state.size() -1); ++i)
  {
    cout << i << "     ";
    if (i < 10)
    {
      cout << " ";
    }
  }
  cout << (state.size() - 1) << endl;

  cout << "Probability: ";

  // Iterate through the state vector until the second last value
  for (auto it = state.begin(); it < (state.end() - 1); ++it)
  {
    cout << *it << " ";
  }

  // Print the last value and a new line
  cout << state.back() << endl;


}

int main()
{
  // Use fixed precision with 1 digit after the decimal
  cout.precision(1);
  cout << std::fixed;

  cout << endl;

  // Test the motionProbability function
  cout << "Testing motionProbability function:" << endl;

  cout << "Probability given no motion input, previous position 13 and current position 13:"
       << "\t" << motionProbability(false, 13, 13) << endl;

  cout << "Probability given no motion input, previous position 4 and current position 5:"
       << "\t\t" << motionProbability(false, 4, 5) << endl;

  cout << "Probability given no motion input, previous position 6 and current position 3:"
       << "\t\t" << motionProbability(false, 6, 3) << endl;

  cout << "Probability given a motion input, previous position 1 and current position 1:"
       << "\t\t" << motionProbability(true, 1, 1) << endl;

  cout << "Probability given a motion input, previous position 8 and current position 9:"
       << "\t\t" << motionProbability(true, 8, 9) << endl;

  cout << "Probability given a motion input, previous position 11 and current position 13:"
       << "\t\t" << motionProbability(true, 11, 13) << endl;

  cout << "Probability given a motion input, previous position 17 and current position 16:"
       << "\t\t" << motionProbability(true, 17, 16) << endl;

  cout << "Probability given a motion input, previous position 19 and current position 0:"
       << "\t\t" << motionProbability(true, 19, 0) << endl;

  cout << "Probability given a motion input, previous position 19 and current position 1:"
       << "\t\t" << motionProbability(true, 19, 1) << endl;

  cout << "Probability given a motion input, previous position 18 and current position 0:"
       << "\t\t" << motionProbability(true, 18, 0) << endl;

  cout << "Probability given a motion input, previous position 18 and current position 1:"
       << "\t\t" << motionProbability(true, 18, 1) << endl;

  cout << endl;

  // Test the observationProbability function
  cout << "Testing observationProbability function:" << endl;

  cout << "Probability given no observation input and position 0:" << "\t" << observationProbability(false, 0) << endl;
  cout << "Probability given no observation input and position 7:" << "\t" << observationProbability(false, 7) << endl;
  cout << "Probability given no observation input and position 13:" << "\t" << observationProbability(false, 13) << endl;

  cout << "Probability given an observation input and position 0:" << "\t" << observationProbability(true, 0) << endl;
  cout << "Probability given an observation input and position 7:" << "\t" << observationProbability(true, 7) << endl;
  cout << "Probability given an observation input and position 13:" << "\t" << observationProbability(true, 13) << endl;

  cout << endl;

  // Use fixed precision with 4 digits after the decimal
  cout.precision(4);
  cout << std::fixed;

  // Test normalisation
  cout << "Testing normaliseState function:" << endl;

  std::vector<double> test = {0.5, 1., 0.5, 2., 1.};

  cout << "Vector before normalisation:" << endl;
  printState(test);

  normaliseState(test);

  cout << "Vector after normalisation:" << endl;
  printState(test);

  // Declare the state vector
  std::vector<double> state;
  state.resize(20);

  cout << endl;

  // Print the state before initialisation
  cout << "Before initialistion:" << endl;
  printState(state);
  cout << endl;

  // Initialise the state
  initialiseState(state);

  // Print the state after initialisation
  cout << "After initialistion:" << endl;
  printState(state);
  cout << endl;

  // First update
  cout << "Step 1 (no motion, door observed)" << endl;
  state = updateState(state, false, true);
  printState(state);
  cout << endl;

  // Second update
  cout << "Step 2 (motion, no door observed)" << endl;
  state = updateState(state, true, false);
  printState(state);
  cout << endl;

  // Third update
  cout << "Step 3 (motion, door observed)" << endl;
  state = updateState(state, true, true);
  printState(state);
  cout << endl;

  // Fourth update
  cout << "Step 4 (motion, no door observed)" << endl;
  state = updateState(state, true, false);
  printState(state);
  cout << endl;

  // Fifth update
  cout << "Step 5 (motion, no door observed)" << endl;
  state = updateState(state, true, false);
  printState(state);
  cout << endl;
  return 0;

}
