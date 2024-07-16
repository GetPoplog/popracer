****************************************
* PopRacer AI Programming Team Project *
*      School of Computer Science      *
*       University of Birmingham       *
*                (2005)                *
****************************************
*               READ ME                *
****************************************

About
-----

* PopRacer was a project developed by a team of students (Team PopCorn)
  in 2005, for the AI Programming course at the School of Computer
  Science, University of Birmingham.

* PopRacer simulates artificially intelligent virtual 'cars' which learn
  to drive around a track.

* The cars are powered by small neural networks which control their
  behaviour.

* The cars 'learn' by evolving their neural networks using a genetic
  algorithm.

* PopRacer can be extended and modified easily, due to the modular
  nature of its design.

* PopRacer was developed in Pop-11, and utilises the graphical
  libraries the language provides.


Team Members
------------

* Michael Brook
* Mark Rowan
* Peter Zeidman
* Damien Clark
* Anushka Gunawardana
* H. Francis Tedom Noumbi


Running
-------

Ensure Poplog is installed.
Execute pop11 main.p

See 'userguide.pdf' for an explanation on how to use PopRacer.


Architecture
------------

PopRacer is made up of 6 files:

main.p - Runs PopRacer
simEngine.p - The Main Simulation Engine, powers PopRacer and provides the GUI.
nnga.p - The Neural Network and Genetic Algorithm Module.
physicsEngine.p - The Physics Engine
bezierDraw.p - The Track Editor
bezier.p - The bezier engine, generates the bezier polynomial.


Known Issues/Bugs
-----------------

* There are a number of graphical artifacts which appear during the
  execution of PopRacer, possibly due to the graphics libraries having
  to switch 'context' between windows.

* The bezier drawing module needs improvement and refining, particularly
  in regard to the way in which the polynomial (which defines the
  curves) is generated.

* The genetic algorithm module could be improved to use roulette wheel
  fitness proportionate selection, currently we select the top two when
  breeding.

* The physics engine could be improved so that friction is calculated for
  each wheel, to prevent the cars from snaking around the track.


13/06/2006


Addition by A.Sloman 13 Jun 2006
--------------------------------
Two files with saved populations are provided

    demo1.p
    demo2.p

A saved file can be loaded into the program before the 'run'
command is given.

    To the shell type (in the popracer directory):

        pop11 main

    When the program starts, the 'help' command gives more
    information. To load demo1 type this to the popracer prompt:

        loadcars demo1

    (without '.p')

    then to start the program type to the prompt:

        'run'

    rearrange the three windows to make all visible. Click on
    the animate button to turn on animation, the click on 'start'.

    If you do it without the 'loadcars' command you'll see the
    difference made by the evolution before the demo1 file was
    saved.

13 Jun 2006
