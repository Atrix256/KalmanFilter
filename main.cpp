#include <stdio.h>
#include <direct.h>
#include "Kalman.h"
#include "Examples.h"

int main(int argc, char** argv)
{ 
    _mkdir("out");

    TestExamples();

    return 0;
}

/*
TODO:
- output the details of each step to be able to graph it etc. to csv but probably also something to the screen?
- how was m_processNoiseMatrix derived in example 9?
*/