#ifndef Sort_h
#define Sort_h

#if ARDUINO >= 100
#include "Arduino.h"
#endif


// Länge der Arrays maximal 256 Elemente... sonst gibts Überlauf...
// Arrays werden "in place" verändert!!!
namespace SortMethods {

    template<typename AnyType> 
    inline void Swap(AnyType array[], uint8_t idx1, uint8_t idx2) {
        AnyType tmp = array[idx1];
        array[idx1] = array[idx2];
        array[idx2] = tmp;
    }

    //insertion sort, schnell und schmutzig, sortiert AUFSTEIGEND
	template<typename AnyType> void Sort(AnyType array[], uint8_t sizeOfArray) {
        for (uint8_t i = 1; i < sizeOfArray; i++) 
			for (uint8_t j = i; j > 0 && (array[j-1] > array[j]); j--)
                Swap(array, j-1, j);
	}

    // wenn der Typ Integer ist, wird bei grader Arraylänge nicht korrekt der Mittelwert ausgegeben, sondern das Ergebnis wird abgerundet...
    template<typename AnyType> AnyType Median(AnyType array[], uint8_t sizeOfArray) {
        Sort(array, sizeOfArray);
        if (sizeOfArray % 2 == 0)
            return (array[sizeOfArray / 2 - 1] + array[sizeOfArray / 2]) / 2;
        else
            return array[sizeOfArray / 2];
    }

    template<typename AnyType> inline float Mean(AnyType array[], uint8_t sizeOfArray) {
        float Cum = 0;
        for (uint8_t i = 0; i < sizeOfArray; i++)  
            Cum += array[i];
        return Cum / sizeOfArray;
    }

    template<typename AnyType> inline float Var(AnyType array[], uint8_t sizeOfArray) {
        float M = Mean(array, sizeOfArray);
        float Cum = 0;
        for (uint8_t i = 0; i < sizeOfArray; i++)  
            Cum += (M - array[i]) * (M - array[i]);
        return Cum / sizeOfArray;
    }

    template<typename AnyType> inline float StdDev(AnyType array[], uint8_t sizeOfArray) {
        return sqrt(Var(array, sizeOfArray));
    }


}

#endif