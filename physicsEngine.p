/*POPRACER MODULE*/

/*Physics Engine*/
/*By Damien Clark & Anushka Gunawardana*/

;;; A.Sloman 13 Jun 2006
;;; dealt with declaring variable messages

vars gravitational_constant = 9.81;
vars frictionCoefficient=0.002;

;;; defined in simEngine.p
global vars
	findClosestFrictionPoint, crashed, alpha, mass, velocityX, velocityY;

define frictionCalc(car)->result;
	;;; added A.Sloman
	lvars frictDist;

	findClosestFrictionPoint(car)->frictDist;
	if frictDist > 80 then
		frictionCoefficient*0.5 -> result;
		true -> crashed(car);	;;; car crashed
	elseif frictDist > 70 then
		frictionCoefficient*2.5 -> result;
	elseif frictDist > 45 then
		frictionCoefficient*2 -> result;
	else
		frictionCoefficient -> result;
	endif;
enddefine;

/**** DIFFERENTIAL STEERING / PHYSICS SIMULATION ****/


define physicsHandler(object,forceLeft,forceRight)->x->y->bearing;
    ;;; New bearing calculation - not strictly physically accurate!

	
    lvars forceDiff = forceRight - forceLeft;
    (forceDiff + object.alpha) -> object.alpha;

    ;;; Calculate acceleration with : F/m =  a
    ;;; Note: resultant force is not forceL+forceR in reality.
    lvars acceleration = (forceLeft+forceRight)/(2 * object.mass);
	lvars accelerationLeft = forceLeft / object.mass;
	lvars accelerationRight = forceRight / object.mass;

    ;;; Calculate the new velocity: v = u + (1/2)*a*t
    lvars ux = object.velocityX;
    lvars uy = object.velocityY;
    ux + sin(object.alpha)*acceleration -> object.velocityX;
    uy + cos(object.alpha)*acceleration -> object.velocityY;


    ;;; Calculate losses to friction. HACKY. LOOK AT ME!

    if not(object.velocityX == 0) then
        lvars frictionForce = object.velocityX*object.mass*gravitational_constant*frictionCalc(object);
        object.velocityX - frictionForce -> object.velocityX;
	endif;

    if not(object.velocityY == 0) then
        lvars frictionForce = object.velocityY*object.mass*gravitational_constant*frictionCalc(object);
        object.velocityY - frictionForce -> object.velocityY;
    endif;

    ;;; Calculate the new position.
    object.rc_picx + object.velocityX -> x;
    object.rc_picy + object.velocityY -> y;
    360 - object.alpha -> bearing;
enddefine;

npr('* Physics Engine OK.');
pr('  - Using Gravitional Constant:');npr(gravitational_constant);
pr('  - Using Friction Coefficient:');npr(frictionCoefficient);
