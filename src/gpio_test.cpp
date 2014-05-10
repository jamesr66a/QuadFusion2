#include "include/GPIO.h"
#include <iostream>

int main()
{
    initGPIO(186, true);
    
    while (true)
    {
        std::cout << readGPIO(186) << std::endl;
    }
}
	
