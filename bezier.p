/*POPRACER MODULE*/


/* Algorithm to give points along a Bezier curve
   Takes in a list of control points and number of interpolations
   Returns a list of actual co-ordinates to be plotted */

/* Curve is plotted with straight lines between points.
   Accuracy is improved by increasing number of interpolations
   Adding extra control points does not affect accuracy! */

/* To create a closed loop (eg. a circle) ensure the first and last
   control points in the list are equal. */

/* Mark Rowan - based on lecture notes by Volker Sorge/Iain Styles
   In turn based on an original document by Ela Claridge */

define factorial(value)->result;
  if value<=1 then
    1->result;
  else;
    factorial(round(value)-1)*round(value)->result; /* rounding just in case
    someone was stupid enough to try finding factorial of a non-integer value */
  endif;
enddefine;

define blending(k,n,u)->result;
  /* calculates the blending function result for given k and n at step u */
  lvars temp; /* using temp vars to break up calculation for simplicity */
  factorial(n)/(factorial(k)*factorial(n-k)) -> temp;
  temp*u**k->temp;
  temp*((1-u)**(n-k))->result;
enddefine;


define findBezierPoints(controls,interpolations)->coords;
  lvars controlpoints,k,u,delta_u;
  lvars x,y; /* x and y are lists of x and y co-ordinates */
  lvars xval, yval;
  lvars sumx,sumy;

  if interpolations<2 then interpolations=2; endif; /* min. 2 interps */
  if length(controls)<2 then return; endif; /* disallow fewer than 2 controls */
  length(controls) -> controlpoints; /* number of control points */

  /*
  controls is a list of control points in the form [ [x1 y1] [x2 y2] ]
  stages is an integer describing how many interpolations of the curve to perform
  coords is the output list of co-ordinates to be plotted by a line-drawing
     procedure. It should be of length interpolations.
  k counts from 0 to the number of control points
  u counts from 0 to 1 in steps of delta_u
  */

  1/(interpolations-1)->delta_u;

  [%
  for u from 0 by delta_u to 1 do;
    0->sumx; /* reinitialise sumx */
    for k from 0 to controlpoints do;
      if k>0 then /* can't access list(0) */
        (controls(k))(1)->xval;
        (xval*blending(k,controlpoints,u))+sumx->sumx;
      endif;
    endfor;
    round(sumx);  /* put result on stack */
  endfor;
  %]->x;

  [%
  for u from 0 by delta_u to 1 do;
    0->sumy; /* reinitialise sumy */
    for k from 0 to controlpoints do;
      if k>0 then /* can't access list(0) */
        (controls(k))(2)->yval;
        (yval*blending(k,controlpoints,u))+sumy->sumy;
      endif;
    endfor;
    round(sumy); /* put result on stack */
  endfor;
  %]->y;
  [^x ^y]->coords;
enddefine;


define get_x_for_given_y(y)->x;
  /* see below */
enddefine;


define get_y_for_given_x(x,coords)->y;
/*
for n from 1 to length (x_list)
  if required value of x is at x_list(n) then
    say 'yippee' and just return corresponding y value
    do a return; here to avoid confusing the poor dears at the for-loop factory
      -- no, we really DON'T want any more iterations, and the ones you sent
         last time were all broken anyway!
  else
    say 'oh bugger' and get out the pen and paper
    if required x is between x_list(n) and x_list(n+1)
      find corresponding y pair
      divide difference of x pair by difference of y pair -> gradient
      divide (distance of the required value from lowest of x_pair) by difference between x pairs -> this is fraction of how far between x pair values the required value is
      multiply this fraction by difference of y pair values -> THE RESULT!
      add this result to a LIST cos there may be many more values that match...
        -- this is only good for checking collisions! when plotting, just pass
	   coords in from the list and draw straight lines between them
    endif
  endif
endfor;

wibble;
*/
enddefine;



/* some nice tests */
;;;findBezierPoints([ [100 200] [100 200] [300 300] [400 0] ],10)=> /* hill */
;;;findBezierPoints([[0 0] [0 1000] [1000 1000] [1000 0] [0 0]],20)=> /* circle! */
/* end niceness */

;;;lvars coords=findBezierPoints([ [0 0] [100 200] [300 300] [400 0] ],10);
;;;get_y_for_given_x(x,coords)=>

npr('* Bezier Engine OK.');
