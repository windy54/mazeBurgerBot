# mazeBurgerBot
Burger Bot modifed to have 3 range sensors (left front right)so that it can follow a maze.
It uses left and right to avoid going into a wall, when front range is less than
a fixed amount it should turn right.

It uses a Pico Lipo, pico motor shim and two motors with encoders

The code "used" to work and then I started to tidy it by puttimg the logic into functions
it stopped reading the encoders .
at the moment I am confused as to what is happening.
could be it is running out of memory, stack?
