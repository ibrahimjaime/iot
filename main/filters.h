//Include Guards - Define to prevent recursive inclusion
#ifndef FILTERS_H
#define FILTERS_H

struct MovAvrS{
    int pos; //Actual position in the window array
    int N;  //Length of the window array. Amount of data to work with in the filter - Filter order
    float sum;  //Actual sum of all the elements of the window array
    float MovAvrResult; //Result of the filter
    bool filled; //Flag to indicate that the window array is full and ready to be handled normally
}MovAvrS; //Struct to work with the moving avarage filter type

void MovingAvaregeInit(MovAvrS *MovAvrData,int len); //Function to initialize and also reset the filter. It gives the filter the length of the window array

void MovingAvarageFilter(MovAvrS *MovAvrData,float *windowArray,float newData); //It modifies the MovAvrResult, result of the filter

#endif // FILTERS_H