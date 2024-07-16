/*
FILE:            simEngine.p
AUTHOR:          Michael David Brook
CREATION DATE:  8 Feb 2005
COURSE:          Team Pop11
PURPOSE:         Popracer :: Simulation Engine (build 1.08)
LAST MODIFIED: 23 March 2005

;;; A.S. 13 Jun 2006
;;; Fixed global variable declarations

/* Version History */

v1.00

	* Main Simulation and Integration with Neural Network Module.
    	* Cleaned up most of code, still some to clean up.

v1.01

	* Added facility to Load and Save G.A Populations (Cool Feature)

v1.02

	* Enabled Plotting of Average Fitness per generation
    	* Added code to enable cars to be evolved on a track.
    	* Added toggles to draw path, intercept point etc.
    	* Added visual car id to car object
    	* Added ability to highlight best car.

V1.03

	* Added GUI for Settings & Loading/Saving of Population
	* Converted some hard coded values into variables

v1.04

	* MODIFIED CARS SO THAT THEY ARE BOUNDED TO A MAXIMUM VELOCITY
    	* NEURAL NETWORK MODIFIED SO THAT THE BEST MEMBER OF THE POPULATION IS KEPT
    	* AUTOMATED ON-THE-TRACK TRAINING - 99% Successful.

v1.05

	* Intergrated Physics Handler, made calcNewPosition redundant as now handled within physics handler.
    	* Altered input to network to throttleLeft and throttleRight of car.
    	* Cars can now skid and powerslide :)
	* Automated ON-THE-TRACK training still very successful.

    Bugs:

	* Still problem with propsheet causing corrupt stack.
		
v1.06

	* runSimulation procedure tidied up a bit to follow our model of the architecture.* Replaced load/save etc gui components with a command line interface.
	* Placed physicsHandler into a physicsEngine.p file, for development purposes.
	
	* Comments!

v1.07

	* Track Editor Fully Intergrated
	* Load/Save of Track
	* On-the-fly Track Drawing
	* Notes added to G.A's
	* Ability to set Neural Network Structure
	* Waypoint Intelligence (3 outputs from NN's), cars decide when they have intercepted a point.
	* Change friction values, so that cars effectively 'crash' when they go too far away from the track.
	* Cars are not drawn if they crash (speeds up sim)
	* Improve File I/O, checks for existence of file when loading.
	* Help File Updated.

v1.08

	* Racing Line and Previous Racing Line of Best Cars Added, which shows the evolution of the path over time.
	* Tidied up some code.
	* Adding file format checking to try and prevent user from loading incorrect files.
	* NEW FILE FORMAT for G.A Population and Tracks (for file format checking)
	* Enabled ability to save populations from initial commandline (to allow old files to be converted)
	* Help Updated. (missing the run command, added save command)
	* Commented 99% of code that requires comments

*/
uses rclib;
uses rc_control_panel;
uses rc_text_input;
uses rc_graphplot;
uses propsheet;
propsheet_init();
/* Global Variables */

lvars width=1000,height=1000; /* Window Parameters */
vars ga=[]; /* The genetic algorithm object */

/* Windows */

vars World;
vars ControlPanel;
vars GraphPlot;
vars GraphPlot2;
vars GraphPlot3;

/* Graph Data */

lvars xGraphData=[],yGraphData=[],yGraphData2=[],xCurrentGraphData=[],yCurrentGraphData=[];


/* File Local Variables (i.e Global within File only) */

lvars scalingFactor=15; /* Scales the size of the cars in relation to the world */
lvars third_of_scalingFactor=round(scalingFactor/3); /*Speeds up drawing of cars */
lvars cars=[]; /* Stores each car object in the world */

lvars pointClickX=0,pointClickY=0,pointClickX2=0,pointClickY2=0; /* Point location to be trained on */

lvars maxSpeed=scalingFactor * 5,maxAcceleration=scalingFactor*2,updateStepTime=0.5;
lvars pauseSim=false;	/*Toggle pausing of the sim */
lvars noreachedEnd=0;	/*Number of cars that have completed track */
lvars generations;	/*number of generations*/
lvars cycle;			/*cycle number*/
lvars trackLoaded=false;	/*has a track been loaded?*/

lvars waypointIntelligence=1;	/* Waypoint Intelligence Setting */
lvars racingLine=[];	/* The racing line of the best car */
lvars prevRacingLine=[];	/* The racing line of best car of the previous generation */
lvars nnSensitivity=500;   /* Neural Net Sensitivity */
lvars mutRate=50; /* Mutation Rate */
lvars crossRate=70; /* Crossover Rate */
lvars cycleLimit=2500;   /* Max Number of Cycles in a single generation */


/* Options */
vars showPaths=false;  ;;; show paths of agent
vars animateFlag = false;	/* Toggle whether to draw the cars or not */
vars showGraph=false;	/* Toggle for realtime velocity graph of best car */
lvars quickTrainLimit=500;	/* When at least 2 cars go around a track in this number of cycles, the cars are drawn */
lvars neuralStructure=[1 10];	/* Neural Network Strucure (1) = Number of Layer, (2) = Number of Hidden Units in each Layer */
lvars numOfCars=50; /* Number of Cars In Simulation */

/* Tracks */
vars bezierPoints=[];	/* Holds Raw Bezier Track Points in the form [xxxxx][yyyyyy] */
vars frictionPoints=[];	/* Holds interpolated points for drawing of track and calculation of friction */
vars offsetX=0;	/*Offset of track (X) */
vars offsetY=0;	/*Offset of track (Y) */

lvars trackWaypoints = [[250 250] [250 -250] [-250 -250] [0 0] [-250 250]]; /*Stores the track waypoints */


/*********************/
/* CLASS DEFINITIONS */
/*********************/

/***** Car object class *****/

define :class car_object; is rc_rotatable;
	/* Car Visual Representation */
	
	slot rc_pic_lines =  [[WIDTH ^(third_of_scalingFactor) COLOUR 'yellow' [{(^(-scalingFactor) ^scalingFactor} {^(scalingFactor*2) ^third_of_scalingFactor} ]]
								[WIDTH ^(third_of_scalingFactor) COLOUR 'yellow' [{(^(-scalingFactor) ^(-third_of_scalingFactor)} {^(scalingFactor*2) ^third_of_scalingFactor} ]]
								[WIDTH ^(third_of_scalingFactor) COLOUR 'cyan' [{^(-scalingFactor) ^scalingFactor} {^scalingFactor ^scalingFactor}  ]]
								[WIDTH ^(third_of_scalingFactor) COLOUR 'cyan' [{^(-scalingFactor) ^(-third_of_scalingFactor)} {^scalingFactor ^(-third_of_scalingFactor)}  ]]
								];
							
	slot waypointQueue = [];	/* A Queue of Waypoints/Goals in the form [x y]*/
	slot reachedEnd = false;	/* Boolean specifying if the car has finished the track or not */

	/* Internal Agent Values */
	slot car_id = gensym("c"); /* Unique Car Id */
	slot rc_picx = 0; /* X location of car */
	slot rc_picy = 0; /* Y location of car */
	slot rc_pic_strings = undef; /*Holds text string, with car id in */
	slot iterations=0; /*Number of iterations car took to reach point */
	slot closestSoFar=10000;    /*Best ever distance to point */
	slot bonus = 0; /* Bonus Score for reaching point */
	slot rc_axis = 90; /* Orientation of car */
	slot crashed=false; /* Boolean specifying whether the car has crashed or not */
	slot pathPoints=[]; /* A list of x,y coordinates describing what path a car has taken */
	/* Throttles */
	slot throttleLeft=0;
	slot throttleRight=0;

	/* Physics */
	slot alpha=0;
	slot mass = 10;
	slot velocityX = 0;
	slot velocityY = 0;
	slot velocityL = 0;
	slot velocityR = 0;
enddefine;


/**************************************************************/
/* AUXILIARY CAR FUNCTIONS                                    */
/* Extra procedures that get around some of pop11's quirks.   */
/**************************************************************/


/*
PROCEDURE: correctForBearing (angle, dy, dx) -> correctedBearing
INPUTS   : angle, dy, dx
  Where  :
    angle is a number representing a 'raw' angle reading in a correct bearing to point.
    dy is a number representing the distance between the car and point (Y)
    dx is a number representing the distance between the car and point (X)
OUTPUTS  : correctedBearing is a 'corrected' bearing.
USED IN  : bearing
CREATED  : 21 Feb 2005
PURPOSE  : Corrects the raw angle value so that it can be used in the car as a bearing to the point.

TESTS:

*/

define correctForBearing(angle,dy,dx) -> correctedBearing;
    /* Added to Correct for Bearings from Absolute */
    /* Correct for 90deg being due East, ie. add 90 to output. */
	 /* Also correct for anticlockwise counting, ie. invert result */

	if dy < 0 and dx >= 0 then
		(90 - angle) + 270 -> correctedBearing;
	endif;

	if dy >= 0 and dx < 0 then
		(90 - angle) + 90-> correctedBearing;
	endif;

	if dy < 0 and dx < 0 then
		180 + angle-> correctedBearing;
	endif;
	
	if dy >= 0 and dx >= 0 then
		angle -> correctedBearing;
	endif;
enddefine;


/*
PROCEDURE: correctAngle (angle) -> correctedAngle
INPUTS   : angle is a number representing a raw angle obtained from pop11 coordinate system
OUTPUTS  : correctedAngle is a angle that standardises the pop11 system, so that it can be used more inituitively.
USED IN  : bearing
CREATED  : 21 Feb 2005
PURPOSE  : Corrects an angle from pop11 coordinate system to a more inituitive standard.

TESTS:

*/

define correctAngle(angle)->correctedAngle;
	/* correcting an angle to fit with the normal way of doing things
		ie. rotate everything anticlockwise by 90 deg then invert from the axis */
	if angle >= 0 and angle <= 90 then
		90 - angle -> correctedAngle;
	elseif angle > 90 and angle <=270 then
		(270 - angle) + 180 -> correctedAngle;
	elseif angle > 270 and angle < 360 then
		(90 - angle) + 360 -> correctedAngle;
	endif;
enddefine;


/****************************************************/
/* MAIN CAR FUNCTIONS                               */
/* Key procedures that simulate the cars 'sensors'  */
/****************************************************/


/*
PROCEDURE: distance (x, y, x2, y2) -> distance
INPUTS   : x, y, x2, y2
  Where  :
    x is a x coordinate representing a point
    y is a y coordinate representing a point
    x2 is a x coordinate representing another point
    y2 is a y coordinate representing another point
OUTPUTS  : distance is a number representing the distance between two points in the world.
USED IN  :
CREATED  : 21 Feb 2005
PURPOSE  : Calculates the distance between two points.

TESTS:

*/

define distance(x,y,x2,y2)->distanceVal;
	lvars xDiff,yDiff,eqA;
	/* Calculate Difference */
	abs(x-x2) -> xDiff;
	abs(y-y2) -> yDiff;
	if xDiff = 0 and yDiff = 0 then
		0 -> distanceVal;
	else
		(xDiff*xDiff) + (yDiff * yDiff) -> eqA;
		sqrt(eqA) -> distanceVal;
	endif;
enddefine;


/*
PROCEDURE: bearing (x1, y1, orientation, x2, y2) -> bearing
INPUTS   : x1, y1, orientation, x2, y2
  Where  :
    x1 is a x coordinate representing the cars location
    y1 is a y coordinate representing the cars location
    orientation is a number representing the current orientation of the car.
    x2 is a x coordinate representing a points location
    y2 is a y coordinate representing a points location
OUTPUTS  : bearing is a number representing the 'corrected' bearing from the car to a point.
USED IN  :
CREATED  : 21 Feb 2005
PURPOSE  : Calculates the 'corrected' bearing from the car to a point in the world.

TESTS:

*/

define bearing(x1,y1,orientation,x2,y2)->bearing;
	lvars dy, dx, angle, bearing;
	/*Calculate Difference*/
	x2-x1 -> dx;
	y2-y1 -> dy;
	
	/* Fix for crash */
	if dy = 0 then
		0.0000001 -> dy;
	endif;	

	arctan(abs(dx)/abs(dy)) -> angle; /* Calculate angle between points */
	correctForBearing(angle,dx,dy) -> bearing;  /* Correct Angle to Bearing */
	(bearing-correctAngle(orientation)) mod 360 -> bearing;     /* Correct for car orientation */
enddefine;

/*************************************************/
/* SIMULATION ENGINE                             */
/* Procedures that control the whole simulation  */
/*************************************************/

/*******************/
/*   WORLD SETUP   */
/*******************/


/*
PROCEDURE: chooseTrack (trackName)
INPUTS   : trackName is the name of the track that the user wants to run the simulation with.
OUTPUTS  : NONE
USED IN  :
CREATED  : 18 Mar 2005
PURPOSE  : Sets the track points up and the offsets for the track specified by the user.

TESTS:

*/

define chooseTrack(trackName);
	if trackName = "straight" then
		/* Straight */
		[[0 400][-400 400]]->bezierPoints;
		300 -> offsetX;
		500 -> offsetY;	
	elseif trackName = "silverstone" then
		/* Silverstone */
		600 ->offsetX;  /* Offset X Coordinate for Track 1 */
		800 -> offsetY;  /* Offset Y Coordinate for Track 1*/
		[[0 1 5 9 14 20 25 30 35 38 42 44 45 46 46 45 43 40 36 32 26 20 13 4 -5 -14 -25 -37 -49 -62 -76 -91 -106 -123 -140 -159 -178 -199 -221 -244 -267 -291 -316 -340 -363 -385 -406 -424
				-438 -450 -457 -460 -458 -452 -441 -426 -407 -385 -360 -331 -298 -260 -215 -160 -90 0] [0 -1 -2 -5 -10 -17 -27 -41 -60 -84 -115 -150 -190 -234 -279 -325 -370 -413 -452 -487 -518
				-543 -562 -576 -585 -589 -589 -585 -577 -568 -556 -543 -530 -517 -503 -490 -478 -467 -458 -448 -440 -432 -424 -416 -408 -398 -387 -374 -359 -341 -321 -298 -272 -243 -212 -179 -145
				-111 -78 -48 -22 -2 11 16 12 0]]->bezierPoints;
	elseif trackName = "hamburg" then
		/* Hamburg */
		[[0 2 5 9 12 15 17 18 18 15 11 5 -3 -13 -26 -40 -55 -72 -90 -108 -126 -144 -160 -174 -187 -199 -208 -216 -222 -227 -232 -237 -243 -249 -258 -268 -280 -294 -310 -328 -347 -366 -385 -401 -414 -422 -423 -414 -392 -356 -301 -226 -126 0] [0 -29 -95 -174 -254 -328 -392 -446 -489 -522 -548 -565 -577 -582 -583 -579 -572 -561 -547 -531 -513 -493 -473 -452 -432 -411 -391 -372 -353 -336 -320 -305 -291 -279 -268 -258 -250 -242 -236 -229 -222 -215 -205 -194 -179 -161 -139 -114 -86 -58 -30 -9 3 0]]-> bezierPoints;
		700 -> offsetX;
		787 -> offsetY;
		
	elseif trackName = "rally" then
		/* Rally */
		[[0 11 35 66 97 126 152 175 194 212 227 242 255 268 280 292 304 315 326 337 347 358 368 377 388 398 409 420 432 444 456 469 482 494 507 519 530 541 553 564 575 587 600 615 631 648
				667 686 705 722 736 745 750 751] [0 8 27 50 73 94 112 125 133 137 135 130 121 109 95 79 63 47 32 19 8 -1 -7 -10 -9 -6 1 10 21 35 50 66 82 98 112 126 138 147 153 157 158 156 151 143
				133 121 107 92 77 64 53 45 42 41]]-> bezierPoints;
		150 -> offsetX;
		500 -> offsetY;
	else
		/* Figure of Eight Track */
		300 -> offsetX;  /* Offset X Coordinate for Track 1 */
		800 -> offsetY;  /* Offset Y Coordinate for Track 1*/
		[[0 23 75 138 201 260 311 354 389 416 437 451 460 464 462 455 444 427 406 382 353 322 288 253 218 184 152 122 96 74 56 44 36 35 38 47 61 78 100 125 152 180 210 239 267 292 315 335
				350 361 366 366 359 347 329 306 278 245 209 172 133 95 59 27 -1 -23 -39 -48 -52 -51 -45 -36 -26 -14 0] [0 0 0 -1 -2 -3 -5 -7 -10 -13 -18 -24 -31 -41 -53 -67 -85 -106 -129 -156 -185
				-216 -249 -283 -318 -352 -385 -418 -448 -477 -503 -527 -549 -568 -585 -599 -611 -621 -629 -635 -639 -642 -643 -642 -640 -635 -628 -619 -608 -594 -577 -556 -533 -506 -476 -442 -405
				-366 -325 -283 -240 -199 -159 -122 -90 -62 -39 -22 -9 0 4 6 6 4 0]]->bezierPoints;
	endif;

	pr('** Using Track '); npr(trackName);
	false -> trackLoaded;
enddefine;



/*
PROCEDURE: createCars (num)
INPUTS   : num is a number representing the number of cars that should be created.
OUTPUTS  : NONE
USED IN  :
CREATED  : 21 Feb 2005
PURPOSE  : Creates 'num' number of car objects and adds them to the 'cars' list

TESTS:

*/

define createCars(num);
	lvars car,dist,b,inputs,aCar,i;
	/* Generate num number of cars */
	for i from 1 to num do
		newcar_object() -> aCar; /* Create car_object */
		/* Setup Starting Parameters */
		/* Set initial location in world */
		-350 -> rc_picx(aCar);
		0 -> rc_picy(aCar);
		/* Set Id of Car to be shown */
		[FONT '8x13bold' COLOUR 'black' {-20 -20 ^(concat_strings([^(car_id(aCar))]))}]->rc_pic_strings(aCar);
		/* Set the cars waypoints */
		trackWaypoints -> waypointQueue(aCar);
		[^aCar ^^cars] -> cars; /* Add car to list */
	endfor;
enddefine;


/*
PROCEDURE: createBrains (num)
INPUTS   : num is a number representing the number of 'brains'/neural networks that are required.
OUTPUTS  : NONE
USED IN  : ???
CREATED  : 21 Feb 2005
PURPOSE  : Creates 'num' number of neural networks and sets up a Genetic Algorithm Population.

TESTS:

*/

define createBrains(num);
	lvars cMutRate,cCrossRate,dist,b,inputs,car;
	/* Build Car Inputs to create brains */
	[%
	 	for car in cars do
			distance(rc_picx(car),rc_picy(car),0,0) -> dist;
			bearing(rc_picx(car),rc_picy(car),rc_axis(car),0,0) -> b;
			[0 0 ^(rc_axis(car)) ^b ^dist ^b ^dist 0];
		endfor;
	%]-> inputs;

	/* Create & Init. GA */
	(mutRate / 10) -> cMutRate;
	(crossRate / 10) -> cCrossRate;

	/* Create G.A Object */
	consGA(neuralStructure,numOfCars,cMutRate,cCrossRate,inputs,[],[])->ga;
	init(ga);
enddefine;



/*
PROCEDURE: convertCoords (coordList) -> newCoordList
INPUTS   : coordList is a list of coordinates for a track generated by the bezierDraw/trackEditor [[xxxxxxxxx][yyyyyyy]]
OUTPUTS  : newCoordList is a list of new coordinates in the format [x y][x y]...
USED IN  :
CREATED  : 18 Mar 2005
PURPOSE  : Converts inputted coordinates from the bezierDraw/trackEditor from the form [xxxxx][yyyy] to [x y] [x y], adding offsets.

TESTS:

*/

define convertCoords(coordList)->newCoordList;
    lvars point,x,y;
	/* Generate a new List */
    [%
	 	for point from 1 to length(coordList(1)) do
	 		/* get x and y from input list */
			coordList(1)(point) -> x;
		  	coordList(2)(point) -> y;
		  	/* add offsets */
		  	x + offsetX -> x;
		  	y + offsetY -> y;
		  	/* Add coordinates into new list in [x y] format] */
		  	[^(x-(width/2)) ^(y-(width/2))];
		endfor;
    %]->newCoordList;
enddefine;

/*
PROCEDURE: createFrictionPoints (inputCoords) -> outputCoords
INPUTS   : inputCoords is a list in the form [x y][x y].. of track waypoints
OUTPUTS  : outputCoords is a list in the form [x y][x y].. of friction points
USED IN  :
CREATED  : 18 Mar 2005
PURPOSE  : Generates a list of 'friction points', by interpolating (creating new points) where track waypoints are
		   a certain distance away from each other.
		   These points are then used to determine if a car is on the track or not.

TESTS:

*/

define createFrictionPoints(inputCoords)->outputCoords;;
	lvars trackDist=80;
	lvars outputCoords,indexPoint,subLoop;
	[%
		for indexPoint from 1 to (length(inputCoords))-1 do
			;;; Put original point on
			[^(inputCoords(indexPoint)(1)) ^(inputCoords(indexPoint)(2))];
			
			;;; x value
			lvars x_val1 = inputCoords(indexPoint)(1);
			;;; x2 value
			lvars x_val2 = inputCoords(indexPoint+1)(1);
			
			lvars y_val1 = inputCoords(indexPoint)(2);
			lvars y_val2 = inputCoords(indexPoint+1)(2);

			;;;Calculate length of line
			lvars line_length = sqrt(((x_val2-x_val1)*(x_val2-x_val1))+((y_val2-y_val1)*(y_val2-y_val1)));
			if line_length>(trackDist/3) then
				lvars newXoffset;
				lvars newYoffset;
				lvars angle = arctan((y_val2-y_val1+0.001)/(x_val2-x_val1+0.001));
				for subLoop from 1 to (line_length/(trackDist/3)) do
					;;;Calculate new points
					cos(angle)*((trackDist/3)*subLoop)-> newXoffset;					
					sin(angle)*((trackDist/3)*subLoop)-> newYoffset;
					[^(inputCoords(indexPoint)(1)+newXoffset) ^(inputCoords(indexPoint)(2)+newYoffset)];
				endfor;
			endif;
		endfor;
	%]->outputCoords;
enddefine;



/****************************/
/*    DRAWING PROCEDURES    */
/****************************/


/*
PROCEDURE: drawTrack ()
INPUTS   : NONE
OUTPUTS  : NONE
USED IN  : ???
CREATED  : 18 Mar 2005
PURPOSE  : Draws the track/friction circles.

TESTS:

*/

define drawTrack(fPoints);
	lvars fPoint;
	/*Draw Green Layer */
	for fPoint in fPoints do
		rc_draw_blob(fPoint(1), fPoint(2), 70,'green');
	endfor;
	/* Draw Brown Layer */
	for fPoint in fPoints do
		rc_draw_blob(fPoint(1), fPoint(2), 55,'brown');
	endfor;
	/* Draw Actual Road Track */
	for fPoint in fPoints do
		rc_draw_blob(fPoint(1), fPoint(2), 45,'black');
	endfor;
enddefine;


/*
PROCEDURE: drawWaypoints (wpList)
INPUTS   : wpList is a list in the form [x y][x y]..
OUTPUTS  : NONE
USED IN  :
CREATED  : 18 Mar 2005
PURPOSE  : Draws the centre line of the track.

TESTS:

*/

define drawWaypoints(wpList);
	lvars prevLineWidth=rc_linewidth,x,y,point;
	XpwSetColor(rc_window, 'white');
	LineOnOffDash -> rc_linestyle;
	1-> rc_linewidth; /* set line width */
	World -> rc_current_window_object;
	lvars prevX=hd(wpList)(1),prevY=hd(wpList)(2);
	for point in wpList do
   		point(1) -> x;
   		point(2) -> y;
		rc_drawline(prevX,prevY,x,y);
		x -> prevX;
		y -> prevY;
    	endfor;
	prevLineWidth -> rc_linewidth;
enddefine;

define drawRacingLine(wpList,prevLine);
	lvars prevLineWidth=rc_linewidth,x,y,point;
	lvars prevX=hd(wpList)(1),prevY=hd(wpList)(2);
	/* Set Colour and Style of Racing Line */
	/* If drawing previous racing line then ... */
	if prevLine = true then
	 	LineOnOffDash -> rc_linestyle;
		XpwSetColor(rc_window, 'blue');
	else
		LineSolid -> rc_linestyle;
		XpwSetColor(rc_window, 'red');
	endif;
	
	3 -> rc_linewidth; /* set line width */
	World -> rc_current_window_object;
	/* Draw the Line */
	for point in wpList do
		point(1) -> x;
		point(2) -> y;
		rc_drawline(prevX,prevY,x,y);
		x -> prevX;
		y -> prevY;
	endfor;
	
	prevLineWidth -> rc_linewidth;
	
enddefine;



/*
PROCEDURE: clearWorld ()
INPUTS   : NONE
OUTPUTS  : NONE
USED IN  : ???
CREATED  : 18 Mar 2005
PURPOSE  : Clears the world (i.e the visual representation currently on screen )

TESTS:

*/

define clearWorld();
	 World -> rc_current_window_object;
	 /*Clear World */
	 rc_draw_blob(0,0,1000,'white');
enddefine;	 				



/***************************************************/
/*      SIMULATION ENGINE SUPPORT PROCEDURES       */
/***************************************************/

/*
PROCEDURE: findClosestFrictionPoint (car) -> closest
INPUTS   : car is a car_object
OUTPUTS  : closest is a number representing the distance between the 'car' and its closest 'friction point'
USED IN  : ???
CREATED  : 18 Mar 2005
PURPOSE  : Calculates the distance of the car to its closest 'friction point' on the track.

TESTS:

*/

define findClosestFrictionPoint(car)->closest;
	lvars closest=999999,fDistance,fPoint,fX,fY,x,y;
	/* Extract location of Car */
	rc_picx(car) -> x;
	rc_picy(car) -> y;
	/* Loop through all the frictionPoints */
	for fPoint in frictionPoints do
		/* Extract friction point location */
		fPoint(1) -> fX;
		fPoint(2) -> fY;
		distance(x,y,fX,fY) -> fDistance; ;;; calc distance between car and friction point;
		/*If closer than closest then update closest */
		if fDistance < closest then
			fDistance -> closest;
		endif;
	endfor;
enddefine;
	

/*
PROCEDURE: averageFitness ()-> avFitness
INPUTS   : NONE
OUTPUTS  : avFitness is a number representing the average fitness values for the cars
USED IN  : ???
CREATED  : 18 Mar 2005
PURPOSE  : Calculates the average fitness values for the cars

TESTS:

*/

define averageFitness()->avFitness;
    lvars total = 0,car;
	 /* Loop through each car */
    for car in cars do
           (iterations(car)-bonus(car)) + total -> total;
    endfor;
    total/(length(cars) * 1.00)  -> avFitness;
enddefine;


/*
PROCEDURE: findBestCar ()-> carIndex
INPUTS   : NONE
OUTPUTS  : carIndex is a number representing the index of the best car.
USED IN  : ???
CREATED  : 18 Mar 2005
PURPOSE  : Returns the index of the best car in the simulation (the one with the lowest fitness value)

TESTS:

*/

define findBestCar()->carIndex;
    lvars bestFitness=9999999,bestCar,i;
	 /* Loop through the index of the cars */
    for i from 1 to length(cars) do
	 	if (iterations(cars(i))-bonus(cars(i))) < bestFitness then
			i -> bestCar;
			(iterations(cars(i))-bonus(cars(i))) -> bestFitness;
		endif;
	endfor;
	bestCar -> carIndex;
enddefine;

/*
PROCEDURE: setCarsAtStart ()
INPUTS   : NONE
OUTPUTS  : NONE
USED IN  : ???
CREATED  : 18 Mar 2005
PURPOSE  : Sets all the cars to the starting point on the track, pointing in the correct direction.

TESTS:

*/

define setCarsAtStart();
	lvars x,x2,y,y2,startPoint,secondPoint,car,xDist,yDist,angleAtStart;
	/* Get first and second points on the track */
	hd(trackWaypoints) -> startPoint;
	trackWaypoints(2) -> secondPoint;
	/* Extract coordinates */
	startPoint(1) -> x;
	startPoint(2) -> y;
	secondPoint(1) -> x2;
	secondPoint(2) -> y2;
	/* Calculate Distance */
	x2- x -> xDist;
	y2- y -> yDist;
	/* Calculate angle that cars should point at the start */
	90 - arctan(yDist/xDist) -> angleAtStart;
	/* For case when the x distance is negative */
	if xDist < 0 then
   		270 - arctan(yDist/xDist) -> angleAtStart;
	endif;
	
	/* Update all the cars locations */
	for car in cars do;
		x -> rc_picx(car);
		y -> rc_picy(car);
		angleAtStart -> rc_axis(car);
		angleAtStart -> alpha(car);
	endfor;
enddefine;



/*
PROCEDURE: graphStats ()
INPUTS   : NONE
OUTPUTS  : NONE
USED IN  : ???
CREATED  : 18 Mar 2005
PURPOSE  : Generates the graphs for the average and best fitness values.

TESTS:

*/

define graphStats();
	lvars score,bCar;
	cars(findBestCar()) -> bCar;
	
	/* Stats and Graphing */
	npr('Graphing Results...');
	GraphPlot -> rc_current_window_object;
	[^^xGraphData ^generations] -> xGraphData;	/* Add Data */
	iterations(bCar) - bonus(bCar) -> score;	/* Calculate fitness score */
	/* Print results to console */
	pr('Generation '); pr(generations); pr('- BEST '); pr(score); pr(' - AVERAGE '); npr(averageFitness());
	[^^yGraphData ^(log(-1 * score))] -> yGraphData; /* Add fitness data */
	[^^yGraphData2 ^(log(-1 * averageFitness()))]->yGraphData2; /* Add fitness data */
	
	/* Graph results */
	rc_graphplot(xGraphData, [Generation], yGraphData, [Best Fitness]);
	GraphPlot2 -> rc_current_window_object;
	rc_graphplot(xGraphData,[Generation],yGraphData2,[Av Fitness]);	

	GraphPlot3 -> rc_current_window_object;
	rc_graphplot(xCurrentGraphData,[Cycle],yCurrentGraphData,[Velocity]);
	[] -> xCurrentGraphData;
	[] -> yCurrentGraphData;
enddefine;


/***********************************/
/*   MAIN SIMULATION PROCEDURES    */
/***********************************/

/*
PROCEDURE: initSimulation ()
INPUTS   : NONE
OUTPUTS  : NONE
USED IN  : ???
CREATED  : 18 Mar 2005
PURPOSE  : Sets up the simulation, clears the world, generates the trackWaypoints & friction points
		   and draws the world.

TESTS:

*/

define initSimulation();
	lvars car;
	npr('** Initialising Simulation for Run...');
	/* Clear Screen */
	clearWorld();
	/* Convert raw coordinates to [x y][x y] format */
	convertCoords(bezierPoints) -> trackWaypoints;
	XpwSetColor(rc_window, 'black');
	/* Generate interpolations */
	createFrictionPoints(trackWaypoints)->frictionPoints;
	World -> rc_current_window_object;
	/* Draw the Track */
	drawTrack(frictionPoints);
	/* Draw the Track Line */
	drawWaypoints(trackWaypoints);
	/*Create new cars */
	createCars(numOfCars);
	/*Set at correct start location and orientation */
	setCarsAtStart();
	
	/* Draw Cars */
	for car in cars do
		rc_draw_linepic(car);
	endfor;
		
	/* Racing Line Drawing */
	if racingLine = [] then
	else
		drawRacingLine(racingLine,false);
	endif;
		
	if prevRacingLine = [] then
	else
		drawRacingLine(prevRacingLine,true);
	endif;
		
	npr('** Done.');
enddefine;


/*
PROCEDURE: initWindows ()
INPUTS   : NONE
OUTPUTS  : NONE
USED IN  : ???
CREATED  : 18 Mar 2005
PURPOSE  : ???

TESTS:

*/

define initWindows();
	lvars car,j;
	nl(1);
	npr('** Building Windows...');
	cleargensymproperty();
	rc_new_window_object(400,40,width,height,false,'PopRacer v1.07 :: Simulation Engine - Click Start') -> World; /* Setup Window */
	rc_new_window_object(400,60,150,300,false,'Control')->ControlPanel; /* Setup Window */
	rc_new_window_object(-1000,50,300,300,false,'Performance Graph (Best)')->GraphPlot;
	rc_new_window_object(-1000,360,300,300,false,'Performance Graph (Average)')->GraphPlot2;
	rc_new_window_object(-1000,670,300,300,false,'Velocity Profile (Best)')->GraphPlot3;
	ControlPanel -> rc_current_window_object;
	vars start_button = create_rc_button(-60,135,90,29,['Start' runSimulation],"action",false); /* Setup a button */
	vars stop_button = create_rc_button(-60,100,90,29,['Stop' stopSim],"action",false); /* Setup a button */
	vars quit_button = create_rc_button(-60,60,90,29,['Quit' sysexit],"action",false); /* Setup a button */
	vars toggleCars = create_rc_button(-60,20, 90, 29, {toggle 'Animate' animateFlag}, false, false);
   vars togglePaths= create_rc_button(-60, -10, 90, 29, {toggle 'Paths' showPaths}, false, false);
	vars toggleGraph= create_rc_button(-60, -40, 90, 29, {toggle 'Graphs' showGraph}, false, false);
	npr('** Done.');
enddefine;




/*
PROCEDURE: runSimulation ()
INPUTS   : NONE
OUTPUTS  : NONE
USED IN  : ???
CREATED  : 18 Mar 2005
PURPOSE  : Main Simulation Loop, that powers the simulation.
		   It integrates all modules of PopRacer.

TESTS:

*/

define runSimulation();

	lvars nets,carIndex,newThrottles,dist,accLeft,accRight,carVLeft,carVRight,carThrottleLeft,carThrottleRight;
	lvars x,y,waypoint,waypoint2,carVel,score,gscore,b1,averageBearing,carBearing,y,redrawFlag=false,car,avFit=1,carWaypoints,bestCar,bestCarId,bestFit,b2,dist2;
	lvars topScore,middleScore,carDetDist,cDist;
	cars(1) -> bestCar;

	if ga = [] then	/* If a g.a population has NOT been loaded then.. */
		npr('*  Creating New Brains...');
		createBrains(numOfCars);	/* Create some new brains (NN's) */
	endif;
	nl(1);
	npr('*** Running Simulation... ***');
	for generations from 1 to 100000 do
		pr('*  Generation '); pr(generations); npr(' Start...');
		cleargensymproperty();  /* Reset Car Ids */
		0 -> noreachedEnd;	/* Reset Number of Cars that have Reached Point */
		getNets(ga)->nets; 		/* Extract flat nets */
		fast_for cycle from 1 to cycleLimit do
			/* Process Each Car */
			fast_for carIndex from 1 to length(cars) do	
				/* Handle Stopping the Sim */
				if pauseSim = true then
					false -> pauseSim;
					return();
				endif;

				cars(carIndex) -> car;	/* Get Car Object */
				
				/* Get Throttle Values for the Car */
				throttleLeft(car) -> carThrottleLeft;
				throttleRight(car) -> carThrottleRight;
			
				/* Get current Location of Car */
				rc_picx(car) -> x;
				rc_picy(car) -> y;
			
				/* Get Waypoints for Car */
				waypointQueue(car) -> carWaypoints;
			
				/* Waypoint Management */
				if length(carWaypoints) > 0 then
					hd(carWaypoints) -> waypoint;
					waypoint(1) -> pointClickX;
					waypoint(2) -> pointClickY;
					/* Calculate Distance and Bearing to Point 1 */
					distance(x,y,pointClickX,pointClickY) -> dist;
					bearing(x,y,rc_axis(car),pointClickX,pointClickY) -> b1;
				
					if length(carWaypoints) > 1 then
						carWaypoints(2)->waypoint2;
						waypoint2(1) -> pointClickX2;
						waypoint2(2) -> pointClickY2;
						/* Calculate Distance and Bearing to Point 2 */
						distance(x,y,pointClickX2,pointClickY2) -> dist2;
						bearing(x,y,rc_axis(car),pointClickX2,pointClickY2) -> b2;
					else
						dist -> dist2;
						b1 -> b2;
					endif;		
				endif;

				/* Neural Network Car Input & Output */
				if length(carWaypoints) > 0 then
					queryNet(nets(carIndex),[^(throttleLeft(car)) ^(throttleRight(car)) ^(rc_axis(car)) ^b1 ^dist ^b2 ^dist2 0]) -> newThrottles;
					/* Scale Outputs for Throttles */
					((0.50 - newThrottles(1)) * nnSensitivity)*2 -> throttleLeft(car);
					((0.50 - newThrottles(2)) * nnSensitivity)*2 -> throttleRight(car);
					/* Determine if using waypoint intelligence mode */
						
					if waypointIntelligence = 1 then
						/* Calculate Distance */
						0.50 - newThrottles(3) -> cDist;
						cDist * 100 -> cDist;
						
						if cDist < 0 then
							0 -> cDist;
						elseif cDist > 1 then
							1 -> cDist;
						endif;
							
						200 * cDist -> carDetDist;
					else
						100 -> carDetDist;
					endif;
						
					/* Throttle Bounding */
					
					if throttleLeft(car) > nnSensitivity/10.00 then
						nnSensitivity/10.00 -> throttleLeft(car);
					endif;
					
					if throttleLeft(car) <  -1 * (nnSensitivity/10.00) then
						-1 * (nnSensitivity/10.00) -> throttleLeft(car);
					endif;
					
					if throttleRight(car) > nnSensitivity/10.00 then
						nnSensitivity/10.00 -> throttleRight(car);
					endif;
					
					if throttleRight(car) <  -1 * (nnSensitivity/10.00) then
						-1 * (nnSensitivity/10.00) -> throttleRight(car);
					endif;

					/* Physics System */
					physicsHandler(car,throttleLeft(car),throttleRight(car))->x->y->carBearing;
				
				elseif reachedEnd(car) = false then /* if car has completed track */
					true -> reachedEnd(car);	/* set car reachedEnd flag */
					noreachedEnd + 1 -> noreachedEnd; /* increment counter */
					pr('*** '); pr(noreachedEnd); pr('. '); pr(car_id(car)); pr(' has completed the course in '); pr(cycle); npr(' cycle(s)');
					/* Get car coordinates */
					rc_picx(car) -> x;
					rc_picy(car) -> y;
				endif;

				World -> rc_current_window_object;
				(carBearing+90) mod 360 -> rc_axis(car);	;;; offset bearing
				LineSolid -> rc_linestyle;
				
				/*check if car has crashed */
				if crashed(car) = true then
					rc_undraw_linepic(car); /*clear car */
				else
					rc_move_by(car,x-rc_picx(car),y-rc_picy(car),animateFlag); /* update cars position and orientation */
				endif;
				
				/* Set Car Text */
				[FONT '8x13bold' COLOUR 'black' {-0 -0 ^(concat_strings([^(car_id(car))]))    }]-> rc_pic_strings(car);
				/* Waypoint Interception and Detection */
				if dist < carDetDist and length(carWaypoints) > 1 then
					if length(carWaypoints) = 0 then
						bonus(car) + 2000 -> bonus(car);
						/* Check if car has reached end */
						if reachedEnd(car) = false then
							true -> reachedEnd(car);
							noreachedEnd + 1 -> noreachedEnd;
						endif;
					else
						/* Remove head of waypoint queue */
						tl(carWaypoints) -> waypointQueue(car);
						0 -> iterations(car);
						bonus(car) + 1000 -> bonus(car);
					endif;
					
				elseif dist < 50 and length(carWaypoints) = 1 then /* Make sure that if on last point, that it is intercepted within stricter limits */
					tl(carWaypoints) -> waypointQueue(car);
					0 -> iterations(car);
					bonus(car) + 1000 -> bonus(car);
				endif;

				/* Draw Cars Path if required */
				if showPaths = true then
					World -> rc_current_window_object;
					XpwSetColor(rc_window, 'red');
					rc_drawline(x,y,x,y);
				endif;

				/* Store Cars Path in itself (for drawing of best cars racing line) */
				if cycle mod 2 = 0 then
					[^^(pathPoints(car)) [^(rc_picx(car)) ^(rc_picy(car))]] -> pathPoints(car);
				endif;

			endfast_for;

			/* End Simulation if at least 2 cars have finished */
			if noreachedEnd >= 2 then
				0 -> noreachedEnd;
				quitloop;
			endif;
			
			/* Get Best Car Data */
			cars(findBestCar()) -> bestCar;
			car_id(bestCar) -> bestCarId;
			iterations(bestCar) - bonus(bestCar) -> bestFit;
			
			
			/* Calculate Cars Velocity (for graphing) */
			sqrt(velocityX(bestCar)**2 + velocityY(bestCar)**2) -> carVel;

			/* Update Graphing Data */
			[^^xCurrentGraphData ^cycle] -> xCurrentGraphData;
			[^^yCurrentGraphData ^carVel] -> yCurrentGraphData;
			
			/* Determine if user wants a realtime graph of velocity */
			if showGraph = true then
				GraphPlot3 -> rc_current_window_object;
				rc_graphplot(xCurrentGraphData,[Cycle],yCurrentGraphData,[Velocity])
			endif;;
			
			World -> rc_current_window_object;
			averageFitness() -> avFit;  /*Calc Average Fitness Score for the Population */
			
			/* Make sure that log function doesn't crash */
			if avFit = 0 then
				1 -> avFit;
			endif;
			
			if bestFit = 0 then
				1 -> bestFit;
			endif;
		
			concat_strings(['PopRacer v1.08 :: Simulation Engine [Cycle Limit:' ^cycleLimit ' Mutation Rate:' ^mutRate '] ' ^generations ':' ^cycle ' Best Car: ' ^bestCarId] ) -> rc_window_title(World);
		endfast_for;
		
		/* Wipe world and draw graphs and calculate stats */
		clearWorld();
		graphStats();
	
		/* Quick Train Auto Switch */
	 	
		if cycle < quickTrainLimit then
			true -> showPaths;
			true -> animateFlag;
			pr('Quick Training Complete! After '); pr(generations); npr(' generations.');
		endif;
		
		World -> rc_current_window_object;

		/* Update Fitness Scores */
		for y from 1 to numOfCars do
			setFitness(ga,y,iterations(cars(y)) - bonus(cars(y)));
		endfor;
		
		/* Set the racing line to be drawn */
		cars(findBestCar()) -> bestCar;
		copydata(racingLine) -> prevRacingLine;
		pathPoints(bestCar) -> racingLine;
		
	
		/* Evolve Population */
		npr('*  Evolving Population...');
		evolve(ga);
		
		/* Clear the cars */
		[]->cars;
		cleargensymproperty(); ;;; reset car ids
		initSimulation();
    endfor;
enddefine;


/***********************************/
/* LOAD/SAVE Procedures            */
/***********************************/


/*
PROCEDURE: savePop (fileName)
INPUTS   : fileName is a ???
OUTPUTS  : NONE
USED IN  : ???
CREATED  : 18 Mar 2005
PURPOSE  : ???

TESTS:

*/

define savePop(fileName);
	lvars packagedGa;
	npr('Type in any notes, and end with RETURN:');
	readline() -> notes(ga);
	[^ga [popracer population]]->packagedGa;
	pr('--> Saving Population to '); npr(fileName);
	packagedGa -> datafile(fileName);
	npr('* Saved!');
enddefine;


/*
PROCEDURE: loadPop (fileName)
INPUTS   : fileName is a string
OUTPUTS  : NONE
USED IN  : ???
CREATED  : 18 Mar 2005
PURPOSE  : Loads car population from 'fileName'.

TESTS:

*/

define loadPop(fileName);
	lvars packagedGa;
	/* Check that file exists */
	if sys_file_exists(fileName >< '.p') = true then
		pr('<--- Loading Population from '); npr(fileName);
		datafile(fileName)->packagedGa; /* get packaged ga object */
		/* Check length */
		if length(packagedGa) = 2 then
			/* Check format tag */
			if packagedGa(2) =[popracer population] then
				packagedGa(1) -> ga;
				nnStructure(ga) -> neuralStructure;
				npr('* Loaded PopRacer Population!');
			else
				npr('*  This file is not a valid Popracer G.A Population');
			endif;
		
		elseif length(packagedGa) = 7 then /* check length (for old version file ) */
			packagedGa -> ga;
			nnStructure(ga) -> neuralStructure;
			npr('* Loaded PopRacer Population!');
			npr('** NOTICE: This is an old format population file, please resave by typing save. **');
		else
			npr('*  This file is not a valid Popracer G.A Population');
		endif;
	else
		npr('* File does not exist!');
	endif;
enddefine;



/*
PROCEDURE: saveTrack (fileName, trackData)
INPUTS   : fileName, trackData
  Where  :
    fileName is a string
    trackData is list containing the track and offsets.
OUTPUTS  : NONE
USED IN  :
CREATED  : 20 Mar 2005
PURPOSE  : Saves a track drawn in the track editor to a file.

TESTS:

*/

define saveTrack(fileName,trackData);
	pr('---> Saving Track to '); npr(fileName);
   trackData -> datafile(fileName);
   npr('* Saved!');
enddefine;


/*
PROCEDURE: loadTrack (fileName)
INPUTS   : fileName is a ???
OUTPUTS  : NONE
USED IN  : ???
CREATED  : 20 Mar 2005
PURPOSE  : ???

TESTS:

*/

define loadTrack(fileName);
	lvars rawTrackData;
	/* Check that file exists */
	if sys_file_exists(fileName >< '.p') then
		pr('<--- Loading Track from '); npr(fileName);
		datafile(fileName)->rawTrackData; /*get raw track data */
		/* Data Format Checking */
		if length(rawTrackData) = 4 then
			if rawTrackData(4) = [popracer trackdata] then
				rawTrackData(1) -> bezierPoints;
				rawTrackData(2) -> offsetX;
				rawTrackData(3) -> offsetY;
   			npr('* Loaded PopRacer Track');
			else
				npr('*  This file is not a valid Popracer Track');
			endif;
		elseif length(rawTrackData) = 3 then /* Dara Format Check (old file format) */
			rawTrackData(1) -> bezierPoints;
			rawTrackData(2) -> offsetX;
			rawTrackData(3) -> offsetY;
			npr('* Loaded PopRacer Track (Pre v1.08 Format)');
			npr('** NOTICE: This is an old format track file, please resave using savetrack. **');
		else			
			npr('*  This file is not a valid Popracer Track.');
		endif;
	else
		npr('* File does not exist!');
	endif;
enddefine;


/***********************************/
/* USER INPUT (GUI) Event Handling */
/***********************************/


/*
PROCEDURE: stopSim ()
INPUTS   : NONE
OUTPUTS  : NONE
USED IN  : ???
CREATED  : 18 Mar 2005
PURPOSE  : Handles stopping the simulation, and taking save/quit commands while paused.

TESTS:

*/

define stopSim();
	lvars command;
	;;; Altered A.Sloman, for use with ! prefix
	lvars fName;
	npr('Simulation Stopped, Type loadcars <file name> or savecars <file name>');
	/* Get input until receives a valid command */
	until command matches [savecars =] or command matches [quit] or command matches [loadcars =] do
		readline()->command;
	enduntil;
	command ==>
	if command = [quit] then
		npr('Exiting PopRacer...');
		sysexit(); /* exit */
	elseif command matches ![savecars ?fName] then
		'Saving to ' >< fName >< '.p' =>
		savePop(fName); /* save population to fName */
	elseif command matches ![loadcars ?fName] then
		'Loading' =>
		loadPop(fName); /* load population from fName */
	else
		'Command not recognized' =>
	endif;
enddefine;


/*********************************/
/*    Command Line Interface     */
/*********************************/


/*
PROCEDURE: commandHelp ()
INPUTS   : NONE
OUTPUTS  : NONE
USED IN  : ???
CREATED  : 18 Mar 2005
PURPOSE  : Prints help documentation to the terminal

TESTS:

*/

define commandHelp();
	npr('PopRacer Help');
	npr('=================');
   nl(1);
	npr('? run');
	npr('    Runs the simulation');
	npr('? setcars <num of cars>');
	npr('    Choose the number of cars to be simulated');
	nl(1);
	npr('? loadcars <filename>');
	npr('    Load a previously-saved population');
	npr('? settrack <trackName>');
	npr('    Choose a previously-saved track');
	nl(1);
	npr('? setcycles <numOfCycles>');
	npr('    Number of cycles of simulation given to the cars as a target to beat');
	npr('? setquicktrain <numOfCycles>');
	npr('    Cars are automatically animated once they can complete a circuit in this many cycles');
	npr('? setmutation <mutationRate>');
	npr('    Mutation rate of the population in the genetic algorithm');
	npr('? sethidden <numOfHiddenUnits>');
	npr('    Sets the number of Hidden Units in each layer in the Neural Networks');
	npr('? setlayers <numOfLayers>');
	npr('    Sets the number of layers in the Neural Networks');
	npr('? waypointai <0/1>');
	npr('    Switches on (1) or off (0), the ability of the cars to determine when they have intercepted a waypoint');
	nl(1);
	npr('? parameters');
	npr('    Display details for the current population');
	npr('? savecars <file name>');
	npr('		Saves a loaded population into the filename specified.');
	npr('? createtrack');
	npr('    Launch the track editor to create a new track');
	npr('? savetrack <file name>');
	npr('    Saves the track in the track editor to the file specified');
	npr('? loadtrack <file name>');
	npr('    Loads a track from the file specified');
	npr('? help');
	npr('    Display help documentation');
	npr('? quit');
	npr('    Cleanly exit the simulation');
enddefine;
	

/*
PROCEDURE: run ()
INPUTS   : NONE
OUTPUTS  : NONE
USED IN  :
CREATED  : 18 Mar 2005
PURPOSE  : Starts the simulation and provides a command line interface to control the setup.

TESTS:

*/

define run();
	lvars command,trackName2='eight',trackData;
	;;; Altered A.Sloman, for use with ! prefix
	lvars
		fName, cNum, trackName, mRate, cLimit, qLimit, layerNum,
		hUnits, bool, fNum,
		;

	nl(2);
	npr('Welcome to PopRacer by Team Popcorn 2005');
	npr('*** Type help to list commands ***');
	npr('Pop Racer Command Line:');
	
	/* Wait until user enters 'run' */
	until command matches [run] do
		/* Wait until user enters a valid command */
		until command matches [loadcars =] or command matches [quit] or command matches [setcars =] or command matches [run] or command matches [settrack =] or command matches [help] or
			command matches [setmutation =] or command matches [setcycles =] or command matches [createtrack] or command matches [parameters] or command matches [setquicktrain =] or command matches
			[savetrack =] or command matches [loadtrack =] or command matches [setlayers =] or command matches [sethidden =] or command matches [waypointai =] or command matches
			[friction =] or command matches [savecars =] do
   		pr('Command');
			readline()->command;
   	enduntil;
		/* Commands */
   	if command = [quit] then
			npr('Exiting PopRacer...');
			sysexit();
   		elseif command matches ![loadcars ?fName] then
			loadPop(fName); /* load population from fName */
			[] -> command;
   		elseif command matches ![setcars ?cNum] then
			cNum -> numOfCars;
			pr('---> Set Number of Cars to '); npr(cNum);
			[] -> command;
		elseif command matches ![settrack ?trackName] then
			pr('---> Set Track to '); npr(trackName);
			trackName -> trackName2;
			[] -> command;
		elseif command matches [help] then
			commandHelp(); /* Print out help documentation */
			[] -> command;
		elseif command matches [createtrack] then
			npr('Loading Track Editor...');
			npr('Instructions:');
			npr('Click the Left Mouse Button to set control points for the track.');
	 		npr('Click the Right Mouse Button to Clear the Track.');
			runTrackEditor(); /* load track editor */
			[] -> command;
		elseif command matches ! [setmutation ?mRate] then
			mRate -> mutRate;
			pr('---> Set Mutation Rate to '); npr(mRate);
			[] -> command;
		elseif command matches ! [setcycles ?cLimit] then
			cLimit -> cycleLimit;
			pr('---> Set Cycle Limit to '); npr(cLimit);
			[] -> command;
		elseif command matches ! [setquicktrain ?qLimit] then
			qLimit -> quickTrainLimit;
			pr('---> Set Quick Train Limit to '); npr(qLimit);
			[] -> command;
		elseif command matches [parameters] then
			/* Print out current simulation parameters */
			npr('PopRacer Simulation Parameters');
			nl(1);
			if trackLoaded = true then
				npr('* Track Name: Loaded Custom Track from File');
			else	
				pr('* Track Name:'); npr(trackName2);
			endif;
			pr('* No Of Cars:'); npr(numOfCars);
			pr('* Mutation Rate:'); npr(mutRate);
			pr('* Cycle Limit:'); npr(cycleLimit);
			pr('* Quick Train Limit:'); npr(quickTrainLimit);
			pr('* Number of Layers in Neural Networks:'); npr(neuralStructure(1));
			pr('* Number of Hidden Units in each layer:'); npr(neuralStructure(2));
			pr('* Waypoint Intelligence:'); npr(waypointIntelligence);
			pr('* Friction Coefficient:'); npr(frictionCoefficient);
			/* Check if a population has been loaded */
			if ga = [] then
		    	npr('* No Population Loaded. A new population will be generated.');
			else
				npr('* Population Loaded from File');
				npr('-------------------------------');
				pr('* Population Notes:'); ppr(notes(ga)); nl(1);
				pr('* Population Size:'); npr(length(thePop(ga)));
				pr('* Number of Layers:'); npr(nnStructure(ga)(1));
				pr('* Hidden Units in Each Layer:'); npr(nnStructure(ga)(2));
			endif;
			
			[] -> command;
		elseif command matches [savetrack ?fName] then
			[^bezierPoints ^offsetX ^offsetY [popracer trackdata]] -> trackData;
			saveTrack(fName,trackData);
			[] -> command;
		elseif command matches [loadtrack ?fName] then
			loadTrack(fName);
			true -> trackLoaded;
			[]->command;
		elseif command matches ! [setlayers ?layerNum] then
			layerNum -> neuralStructure(1);
			pr('---> Set Number of Layers in the Neural Networks to '); npr(layerNum);
			[] -> command;
		elseif command matches ! [sethidden ?hUnits] then
			hUnits -> neuralStructure(2);
			pr('--> Set Number of Hidden Units in each Layer in the Neural Networks to '); npr(hUnits);
			[] -> command;
		elseif command matches ! [waypointai ?bool] then
			if bool = 1 or bool = 0 then
				bool -> waypointIntelligence;
				pr('--> Set Waypoint Intelligence Mode to '); npr(bool);
				[] -> command;
			endif;
		elseif command matches ! [friction ?fNum] then
			pr('---> Set Friction Coefficient to '); npr(fNum);
			fNum -> frictionCoefficient;
			[] -> command;
		elseif command matches ! [savecars ?fName] then
			savePop(fName);
			[] -> command;	
		endif;
	enduntil;
	
	/* Check if using a custom track */
	if bezierPoints = [] then
		chooseTrack(trackName2); /* load track as specified by user */
	elseif trackLoaded = false then
		npr('** Using Custom track');
		rc_kill_window_object(trackEditor); /* close editor */
	endif;
	
	/* Start Simulation */
	npr('***** Starting SimEngine *****');
	initWindows();	/* initialise the windows and build the control panel */
	initSimulation(); /* setup sim for run */
	npr('***** Started SimEngine  *****');
	npr('** Waiting for User.');
enddefine;

run();


/* Keep Simulation Running when Executed from a Terminal */
while true do
endwhile;
