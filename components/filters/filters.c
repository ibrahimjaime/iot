#include "filters.h"
#include <stdio.h>

void MovingAvaregeInit(struct MovAvrS *MovAvrData,int len){
    MovAvrData->pos = 0; 
    MovAvrData->N = len; 
    MovAvrData->sum = 0; 
    MovAvrData->MovAvrResult = 0; 
    MovAvrData->filled = false; 
}

void MovingAvarageFilter(struct MovAvrS *MovAvrData,float *windowArray,float newData){
    //The program does the following section just once at the beggining.
    if(MovAvrData->filled == false){
        windowArray[MovAvrData->pos] = newData; //Add the new income data to the arrary
        MovAvrData->sum = MovAvrData->sum + newData; //Makes the sum
        MovAvrData->MovAvrResult = MovAvrData->sum / (MovAvrData->pos + 1); //Gets the avarage only with the values it has
        if(MovAvrData->pos == (MovAvrData->N - 1)){ //once the data array is full, it reset the position and makes the filled flag true
            MovAvrData->filled = true;
            MovAvrData->pos = 0;
        }
        else{ //if the data array is not filled, increses one position to add the next data
            MovAvrData->pos++;
        }
    }
    //Once the buffer is full, it works normally
    else if(MovAvrData->filled == true){
        MovAvrData->sum = MovAvrData->sum -  windowArray[MovAvrData->pos] + newData; //Substract the oldest value from the sum
        windowArray[MovAvrData->pos] = newData; //remplace the oldest value for the new one in the array
        MovAvrData->MovAvrResult =  MovAvrData->sum /  MovAvrData->N; //Calculate the new Avarage
        MovAvrData->pos++; //Moves to the next position in the array
        if(MovAvrData->pos >= MovAvrData->N){
            MovAvrData->pos = 0; //Once it reads all the array, warps to the beggining again
        }
    }
}