/****************************************/
/* PopRacer AI Programming Team Project */
/*      School of Computer Science      */
/*       University of Birmingham       */
/*                (2005)                */
/****************************************/
/*        CREDITS: Team PopCorn         */
/****************************************/
/* Simulation Engine Development        */
/* -----------------------------        */
/* - Michael Brook                      */
/* - Mark Rowan                         */
/*                                      */
/* Neural Network and Genetic Algorithm */
/* ------------------------------------ */
/* - Peter Zeidman                      */
/*                                      */
/* Physics Engine                       */
/* ------------------------------------ */
/* - Damien Clark                       */
/* - Anushka Gunawardana                */
/*                                      */
/* Track Editor                         */
/* ------------------------------------ */
/* - Mark Rowan                         */
/* - Michael Brook                      */
/*                                      */
/* Graphical/Commandline User Interface */
/* ------------------------------------ */
/* - Michael Brook                      */
/*                                      */
/* Report/Documentation Management      */
/* ------------------------------------ */
/* - H. Francis Tedom Noumbi            */
/****************************************/


/*************** POPRACER LOADER ***************/

/**** Work Around for Declaring Vars ****/
/*
vars warningdetail = 1;
define prwarning(word);
enddefine;
*/

nl(2);
pr('PopRacer v1.08 (c) Team Popcorn 2005 - Loading Modules...');
pr('Final Release by Michael Brook');
nl(1);

uses rclib, rc_window_object, rc_buttons;

compilehere
            nnga.p
	    physicsEngine.p
	    bezierDraw.p
            simEngine.p;
				
nl(1);
