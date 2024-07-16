/*POPRACER MODULE*/

/*Neural Network and Genetic Algorithm Module*/
/*By Peter Zeidman                           */
/*
Fixed some variable declarations
	Aaron Sloman 13 Jun 2006
*/



uses objectclass;

vars training =[[0 0 0]
                [0 1 1]
                [1 1 0]
                [1 0 1]];
		
/*vars training =[[1 0]
		[0 1]];*/

vars network = [];
vars totalNeurons = 0;

define sigmoid(input) -> output;
          lvars e = 2.71828183;
		
		  input / 100 -> input;
          ;;;1 / (1 + exp(input * -1)) -> output;
	      1/(1 + e**(input * -1 )) -> output;
enddefine;

untrace sigmoid;


/*************************************************************/
/* Class: Neuron                                             */
/*************************************************************/

define :class Neuron;
	slot numInputs;
	slot weights;
	slot output;
enddefine;

/* Initialize the neuron with a weights array */
define :method init(n:Neuron);
	lvars randomnum,x;

	newarray([1 ^(numInputs(n) + 1)]) -> weights(n); ;;; TODO

	;;; Create a weight for each input
	for x from 1 to (numInputs(n) + 1) do
		(random0(2.0) - 1.0) / 2.0 -> randomnum;
		randomnum -> weights(n)(x);
	endfor;

enddefine;

/* Set the neuron's output */
define :method setOutput(n:Neuron,newOutput);
	newOutput -> output(n);
enddefine;

/*************************************************************/
/* Class: Layer                                              */
/*************************************************************/

define :class Layer;
    slot num_neurons = 0;    /* Number of neurons in layer */
    slot neuronInputs = 0;   /* Number of inputs per neuron */
    slot prevLayer = [];     /* The previous layer of neurons*/
    slot neuronLayer = [];   /* The neurons in this layer */
enddefine;

/* Initialaze the layer by creating neurons */
define :method init(l:Layer);

	lvars x,newNeuron;

	;;; Create neurons for layer
	for x from 1 to num_neurons(l) do

		consNeuron(neuronInputs(l),[],0) -> newNeuron;		
		init(newNeuron);
		newNeuron :: neuronLayer(l) -> neuronLayer(l);



	endfor;
	
	;;;neuronLayer(l) :: network -> network;

enddefine;

/* Set the layer's outputs with predetermined values */
/* Used for input layer */
define :method loadOutputs(l:Layer,layerOut);

	lvars theNeuron,x;

	if (length(layerOut) - 1) /= num_neurons(l) then
		mishap('Couldnt load outputs: wrong number of arguments',[]);
	endif;
	
	for x from 1 to (length(layerOut) - 1) do
		setOutput(neuronLayer(l)(x),layerOut(x));
	endfor;
	
enddefine;

/* Integrate input for the layer's neurons, i.e. inputs * weights + bias */
define :method hiddenIntegrate(l:Layer);
	
	lvars theNeuron,x = 1,prevNeuron,layerNeurons,prevNeurons,theOutput = 0;

	neuronLayer(prevLayer(l))->prevNeurons;

	;;; Loop for each neuron in the layer
	for theNeuron in neuronLayer(l) do

		;;; And for each in the previous
		for prevNeuron in prevNeurons do

			if (output(prevNeuron) /= null) then
	 			(weights(theNeuron)(x) * output(prevNeuron)) + theOutput -> theOutput;
			endif;
	
			x + 1 -> x;

		endfor;	
			(weights(theNeuron)(length(weights(theNeuron))) * -1) + theOutput -> theOutput;
	
		setOutput(theNeuron,sigmoid(theOutput));
		;;;setOutput(theNeuron,theOutput);
		0 -> theOutput;

		1 -> x;
	endfor;

enddefine;

/*************************************************************/
/* Class: NeuralNet                                          */
/*************************************************************/

define :class NeuralNet
     slot numInputs = 0;
     slot numHiddenLayers = 0;
     slot numHiddenPerLayer = 0;
     slot numOutputs = 0;
     slot netInput = [];
     slot layers = []; ;;; Starting with the output layer
enddefine;

/* Initialize the neural net by creating layers and performing */
/* initial integrations */
define :method init(net:NeuralNet);
     lvars x,newLayer;

	numInputs(net) + (numHiddenLayers(net) * numHiddenPerLayer(net)) + numOutputs(net) -> totalNeurons;

     ;;; Create input layer

	 consLayer(numInputs(net),1,[],[]) -> newLayer;
         init(newLayer);
         newLayer :: layers(net) -> layers(net);

     	 loadOutputs(newLayer,netInput(net));

     ;;; Create hidden layer(s)

	;;; First hidden layer takes inputs from input layer
	consLayer(numHiddenPerLayer(net),numInputs(net),newLayer,[]) -> newLayer;
	init(newLayer);
     	newLayer :: layers(net) -> layers(net);

	hiddenIntegrate(newLayer);
	
	
	;;; Remaining hidden layers take input from hidden layers
	if numHiddenLayers(net) > 1 then
	    for x from 2 to numHiddenLayers(net) do;
		consLayer(numHiddenPerLayer(net),numHiddenPerLayer(net),newLayer,[]) -> newLayer;
		init(newLayer);
		newLayer :: layers(net)->layers(net);
		hiddenIntegrate(newLayer);
	    endfor;
	endif;

     ;;; Create output layer

        consLayer(numOutputs(net),numHiddenPerLayer(net),newLayer,[]) -> newLayer;
        init(newLayer);
        newLayer :: layers(net)->layers(net);
	hiddenIntegrate(newLayer);

enddefine;

/* Feed weights forward for all hidden layers */
define :method feedForward(net: NeuralNet);
	lvars layer,neuron,x = 1;
	for x from (length(layers(net)) - 1) by -1 to 1 do;
		hiddenIntegrate(layers(net)(x));
	endfor;
enddefine;


/* Display a graphical view of the network */
define :method viewNet(net: NeuralNet);
	lvars layer,neurons,neuron,x;
	
	for layer in layers(net) do
		neuronLayer(layer) -> neurons;

		for neuron in neurons do
	        neuron=>
			for x from 1 to length(weights(neuron)) do
				if isnumber(weights(neuron)(x)) then
					pr(weights(neuron)(x));
					pr("  -  ");
					pr(output(neuron));
					''=>
				endif;
			endfor;
			''=>
		endfor;
		'================='=>
	endfor;
	pr('^^^ First Layer ^^^');
	nl(1);
enddefine;


/* Return a list of weights representing the net */
/* Starting with 1st hidden layer */
define :method getWeights(net: NeuralNet) -> theNet;

	lvars layer,neuron,neurons,x,y = 1;
	[] -> theNet;

	for layer in layers(net) do
		if y < length(layers(net)) then
			
			neuronLayer(layer) -> neurons;

			for neuron in neurons do

				for x from 1 to length(weights(neuron)) do
					weights(neuron)(x) :: theNet -> theNet;
				endfor;
			endfor;
		endif;
		y + 1 -> y;
	endfor;

enddefine;

/* Load the neural network with new weights*/
define :method setWeights(net: NeuralNet,newWeights);

	lvars layer,neuron,x,y = 1,neurons,passedInput = false;
;;;	[] -> theNet;
	rev(newWeights) -> newWeights;
	for layer in layers(net) do

		if y <= length(newWeights) then
			neuronLayer(layer) -> neurons;

			for neuron in neurons do
				for x from 1 to length(weights(neuron)) do
					newWeights(y) -> weights(neuron)(x);
					y + 1 -> y;
				endfor;
			endfor;
		endif;
	endfor;
	feedForward(net);
enddefine;

/* Extract the network's outputs (must have been calculated elsewhere)*/
define :method getOutputs(net: NeuralNet) -> outputs;

	lvars outLayer,neuron,outputs=[];

	layers(net)(1) -> outLayer;

	for neuron in neuronLayer(outLayer) do
		output(neuron) :: outputs -> outputs;
	endfor;

enddefine;

/* Query the network by setting input values and feeding forward*/
/* Return the network's output */
define :method queryNet(net: NeuralNet, theInputs) -> result;
	
	loadOutputs(layers(net)(length(layers(net))),theInputs);
	
	;;;;viewNet(net);
	feedForward(net);
	
	getOutputs(net) -> result;
	
enddefine;

/*************************************************************/
/* Class: GA                                                 */
/*************************************************************/
define :class GA;
	slot nnStructure;
	slot popSize;
	slot mutationRate;
	slot crossoverRate;
	slot netInputs = [];
	slot thePop = []; ;;; Stores flattened representation of the nets
	slot notes = [];
enddefine;

define :method init(gen: GA);

	;;; Added A.Sloman  13 Jun 2006
	lvars numLayers, numHiddenUnits;
	;;; Create neural networks and work out fitness
		
	lvars i,score,newNet,flatNet;
	nnStructure(gen)(1) -> numLayers;
	nnStructure(gen)(2) -> numHiddenUnits;
	;;; Init population with neural nets	
	for i from 1 to popSize(gen) do
		
		consNeuralNet(7,numLayers,numHiddenUnits,3,netInputs(gen)(i),[]) -> newNet;
		init(newNet);

		[^(getWeights(newNet)) ^(getOutputs(newNet))]->flatNet;

		;;; fitness(gen,flatNet) -> score;
		0 -> score;
		[^flatNet ^score] :: thePop(gen) -> thePop(gen);

	endfor;

enddefine;

define :method setFitness(gen :GA,netID,newScore);
	lvars net;
/*
	for net in thePop(gen) do

		if net(1)(1) matches thePop(gen)(netID)(1)(1) then
			newScore -> thePop(gen)(netID)(2);
		endif;
	endfor;
*/

	newScore -> thePop(gen)(netID)(2);
enddefine;

define :method lowest(gen: GA)->output;
	lvars net,lowest = 99;
	for net in thePop(gen) do
		if net(2) < lowest then net(2)->output endif;
	endfor;
enddefine;

define :method highest(gen: GA)->output;
	lvars net,highest = -99;
	for net in thePop(gen) do
		if net(2) > highest then net(2)->output endif;
	endfor;
enddefine;


define :method average(gen: GA)->output;
	lvars net,numNets,subtotal;

	length(thePop(gen))->numNets;
	
	for net in thePop(gen) do
		net(2) + subtotal -> subtotal;
	endfor;
	
	(subtotal / numNets) -> output;
enddefine;

;;; Moved to prevent variable declaration message
define :method compareNets(net1,net2) -> result;
	net1(2) < net2(2)  -> result;
enddefine;


;;; Forward declarations to prevent variable declaration message
define :generic crossover(net1,net2) -> babyNetwork;
enddefine;

define :generic mutate(gen);
enddefine;

define :method evolve(gen: GA);

	;;; Added A.Sloman  13 Jun 2006
	lvars numLayers, numHiddenUnits;

	lvars parent1,parent2,nextGen,score,flatNet,newNet,i;
	[] -> nextGen;


		;;; Sort population
	syssort(thePop(gen),compareNets) -> thePop(gen);

	;;; Choose parents
	thePop(gen)(1) -> parent1;
	thePop(gen)(2) -> parent2;

	[%
	for i from 1 to popSize(gen)-1 do	
	
		;;; Crossover bits from the two
		crossover(parent1,parent2);

	endfor;
	parent1(1)(1);
	%]->thePop(gen);
	
	
	;;; Mutate networks
	mutate(gen);
    ;;;[^^parent1] -> thePop(gen)(1);

	;;; Run networks to generate outputs
	nnStructure(gen)(1) -> numLayers;
	nnStructure(gen)(2) -> numHiddenUnits;
	1 -> i;
	[%
	for flatNet in thePop(gen) do;

		consNeuralNet(7,numLayers,numHiddenUnits,3,netInputs(gen)(i),[]) -> newNet;
		init(newNet);
		setWeights(newNet,flatNet);

		[^flatNet ^(getOutputs(newNet))]->flatNet;

		;;; Score the network
		;;;fitness(gen,flatNet) -> score;
		0 -> score;
		[^flatNet ^score];
		
		i + 1 -> i;
	endfor;
	%]->thePop(gen);
enddefine;

define :method getNets(gen: GA) -> nets;

	;;; Added A.Sloman  13 Jun 2006
	lvars numLayers, numHiddenUnits, newNet;

	lvars flatNet,i;
	1 -> i;
	nnStructure(gen)(1) -> numLayers;
	nnStructure(gen)(2) -> numHiddenUnits;
	[%
	for flatNet in thePop(gen) do;
		consNeuralNet(7,numLayers,numHiddenUnits,3,netInputs(gen)(i),[]) -> newNet;
		init(newNet);
		setWeights(newNet,flatNet(1)(1));
		newNet;
		i + 1 -> i;
	endfor;
	%]->nets;
	
enddefine;

define :method fitness(gen: GA,flattenedNet) -> output;

	;;;abs(flattenedNet(2)(1)-flattenedNet(2)(2)) -> output;
	flattenedNet(1)(2)(1)+flattenedNet(1)(2)(2) -> output; ;;; total
	;;;random(5) -> output; ;;; random

enddefine;

define :method chooseParent(gen: GA,excludeParent) -> parent1;
	
	;;; I'm quirky - look into me
	
	lvars nn,totalFitness = 0,subtotalFitness = 0,randNum,score;
	
	;;; Roulette wheel selection
/*
	;;; Calc the total fitness of the population
	for nn in thePop(gen) do;
		totalFitness + nn(2) -> totalFitness;
	endfor;

	;;; Generate random number
	random0(totalFitness) -> randNum;		
	
	;;; Loop through parents
	for nn in thePop(gen) do;
		subtotalFitness + nn(2) -> subtotalFitness;
		if subtotalFitness >= randNum then
			nn -> parent1;
			quitloop();
		endif;
	endfor;
*/

	;;; Dim selection

	lvars net,highest = 999999,highestNet;
	for net in thePop(gen) do
		/*
		net(2)=>
		highest=>
		' '=>
		if net(2) < highest and net(2) /= excludeParent then
			net(2)->highest;
            net->highestNet;
		endif;
		*/
		net(2)=>
	endfor;

	highestNet->parent1;	
enddefine;

define :method crossover(net1,net2) -> babyNetwork;

	;;; Create multiple offspring?

	lvars weights1 = net1(1)(1),weights2 = net2(1)(1);
	lvars crossoverPoint,babyNetworkb,i;
	
	[] -> babyNetwork;

	random(length(weights1)) -> crossoverPoint;

	[%
	for i from 1 to crossoverPoint do;
		weights1(i);
	endfor;
	%]->babyNetwork;
	
	[%
	for i from (crossoverPoint + 1) to length(weights1) do
		weights2(i);
	endfor;
	%]->babyNetworkb;
	[^^babyNetwork ^^babyNetworkb] -> babyNetwork;

enddefine;

/*
define :method crossover(net1,net2) -> babyNetwork;

	lvars weights1 = net1(1)(1),weights2 = net2(1)(1);
	lvars crossoverPoint,babyNetworkb,i;

	[] -> babyNetwork;

	[%
		for i from 1 to round(length(weights1)/2) do;
			weights1(i);
		endfor;
	%]->babyNetwork;

	[^^babyNetwork ^^babyNetwork]->babyNetwork;
enddefine;
*/
	


define :method mutate(gen:GA);
	lvars net,weight,mutWeights,i,j;

	for j from 1 to length(thePop(gen))-1 do
		thePop(gen)(j) -> net;
		net -> mutWeights;

		for i from 1 to length(mutWeights) do
			if (random(10) / 10.0) <= 0.1 then
				if random(2) = 1 then
					mutWeights(i) + (mutWeights(i) * random(mutationRate(gen))) -> mutWeights(i);
				else
					mutWeights(i) - (mutWeights(i) * random(mutationRate(gen))) -> mutWeights(i);
                endif;
				
				;;; LIMITS ON WEIGHTS
				if mutWeights(i) > 10 then 10 -> mutWeights(i) endif;
				if mutWeights(i) < -10 then -10 -> mutWeights(i) endif;				
			endif;
			
		endfor;
	endfor;
enddefine;

/*************************************************************/
/* Runtime                                                   */
/*************************************************************/

lvars net,net2,rubbish,ga,x,y,z,inputs;
/*
consNeuralNet(2,1,3,2,[]) -> net;
init(net);
setWeights(net,[1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21]);
feedForward(net);
viewNet(net);
*/
/*
consNeuralNet(2,1,3,2,[]) -> net;
init(net);
setWeights(net,[-0.1 0.2 0.3 0.4 0.5 0.6 -0.7 0.8 0.9
-0.15 0.25 -0.35 -0.45 -0.55 0.65 -0.75 -0.85]);
viewNet(net)=>
getWeights(net)=>

*/
/*
;;; Create GA
;;; Params: Population size, Mutation rate, Crossover rate, Network inputs, []
consGA(10,0.1,0.7,[[1 1 1 0 0 ] [1 1 2 0 0] [1 1 3 0 0] [1 1 1 0 0] [1 1 4 0 0] [1 4 2 0 0] [1 8 4 0 0] [1 5 4 0 0] [1 4 4 0 0] [1 9 1 0 0]],[])->ga;
init(ga);

;;;0 -> z;
for z from 1 to 300 do

	;;; Get list of neural net objects
	getNets(ga) -> x;

	;;; Get a network's output
	;;; getOutputs(x(1))=>
	
	queryNet(x(1),[1 4 2 0 0])=>;

	;;; ** Run the cars **

	;;; Set networks' fitness scores
	for y from 1 to 10 do
		setFitness(ga,y,fitness(ga,thePop(ga)(y)));
	endfor;
		
	;;; Set networks' new inputs
	[[1 1 1 0 0 ] [1 1 2 0 0] [1 1 3 0 0] [1 1 1 0 0] [1 1 4 0 0] [1 4 2 0 0] [1 8 4 0 0] [1 5 4 0 0] [1 4 4 0 0] [1 9 1 0 0]] -> netInputs(ga);
	

	;;; Get the average fitness:
	average(ga)=>;


	;;; Evolve
	evolve(ga);
	
	;;;z + 1->z;

endfor;
*/
npr('* Neural Network and Genetic Algorithm Engine OK');
