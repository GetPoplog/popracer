/*POPRACER MODULE*/

/*
FILE:            bezierDraw.p
AUTHOR:          Michael David Brook
CREATION DATE:  24 Mar 2005
COURSE:          Team Pop11: Team Popcorn
PURPOSE:         The Track Editor
LAST MODIFIED:  24 Mar 2005

*/

compilehere
        bezier.p

vars editorWidth=1000,editorHeight=1000; ;;; Window Parameters

vars trackEditor;	/* Holds the track editor window */
lvars Points = [];  /* Control Points List */
vars plottedPoints=[];	/* Actual Generated Points */
vars editorOffsetX=0,editorOffsetY=0;  /* Init Offsets */

/* Draw Bezier Function by Michael Brook */
/* Powered by bezier.p by Mark Rowan */
/* v1.00 */

/* v1.01

	* Integrated into Popracer Simulation Engine

*/



/*
METHOD   : rc_button_1_down (pic, x, y, modifiers)
INPUTS   : pic, x, y, modifiers
  Where  :
    pic is a window object
    x is a coordinate of the click
    y is a coordinate of the click
OUTPUTS  : NONE
USED IN  :
CREATED  : 24 Mar 2005
PURPOSE  : An event handler for the left mouse button click event
		   in the track editor. (Draws Track and Updates Simulation Track)

TESTS:

*/

;;; A.S. defined in simEngine.p
global vars
	convertCoords, createFrictionPoints, drawTrack, bezierPoints, offsetX, offsetY;

define :method rc_button_1_down(pic:rc_window_object, x, y, modifiers);
	lvars conPoints,conPoints2;
	/* Check that clicked in correct window */
	if pic = trackEditor then
		/* Draw  Cross at point */
		1 -> rc_linewidth;
		rc_drawline(x,y,x,y);
		rc_drawline(x-5,y,x+5,y);
		rc_drawline(x,y-5,x,y+5);
		/* Check if points is empty */
		if Points=[] then
			/* Get offsets */
			x+(editorWidth/2)->editorOffsetX;
			y+(editorWidth/2)->editorOffsetY; /* get offsets for the first point */
		endif;
        /* Add to points list */
		[ ^^Points [^(x+(editorWidth/2)-editorOffsetX) ^(y+(editorWidth/2)-editorOffsetY)]  ]->Points;
		
		/* Check that length greater than 1, for on the fly drawing of track */
		if length(Points) > 1 then
			/* Clear Screen */
			rc_draw_blob(0,0,1000,'white');
			/* Generate Bezier Points */
			findBezierPoints(Points,listlength(Points)*3) -> plottedPoints;
			/* Convert Coordinates */
			convertCoords(plottedPoints) -> conPoints;
			/* Generate Friction Points/Circles */
			createFrictionPoints(conPoints) -> conPoints2;
			;;; fixed A.S.
			;;; trackEditor -> rc_current_window;
			trackEditor -> rc_current_window_object;
			/* Draw the track */
			drawTrack(conPoints2);
		endif;
		/* Update simulation points list and offsets */
		plottedPoints -> bezierPoints;
		editorOffsetX -> offsetX;
		editorOffsetY -> offsetY;
	 endif;
enddefine;

/*
METHOD   : rc_button_2_down (pic, x, y, modifiers)
INPUTS   : pic, x, y, modifiers
  Where  :
    pic is a window object
    x is a coordinate
    y is a coordinate
OUTPUTS  : NONE
USED IN  :
CREATED  : 24 Mar 2005
PURPOSE  : Handles right mouse button click in the editor. (Clears the points)

TESTS:

*/

define :method rc_button_2_down(pic:rc_window_object, x, y, modifiers);
    [] -> Points;   /* clear Points */
    rc_start(); /* reset screen */
enddefine;

/*
PROCEDURE: runTrackEditor
INPUTS   : NONE
OUTPUTS  : NONE
USED IN  : ???
CREATED  : 24 Mar 2005
PURPOSE  : Loads up the track Editor.

TESTS:

*/

define runTrackEditor;
	npr('* Starting Track Editor...');
	rc_new_window_object(400,40,editorWidth,editorHeight,false,'Track Editor') -> trackEditor; /* Setup Window */
	rc_show_window(trackEditor);	/* Show Window */
	rc_mousepic(trackEditor); /* Setup for Mouse Handler */
enddefine;

npr('  - Track Editor OK.');
