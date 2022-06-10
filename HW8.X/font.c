#include "font.h"
#include "ssd1306.h"


//DRAWCHAR
//In this function, we will draw a character at a specified position
//Posx = start position in the X
//Posy = start position in the Y
//Charac = character that will be drawm.
//Note that this function will not update the SSD display only alter the
//array. 
void drawChar(unsigned int posx, unsigned int posy, char chars){
    static int currentx;    //current position in the x-direction
    static int currenty; //current postion in the y-direction
    int asciipos = chars - 0x20;    //moves the character to an int pos. of 0 for the first valid char.  
    for (int i = 0; i<5; i++){  //Loops in the x-direction.
       unsigned char hexval =  ASCII[asciipos][i]; 
       //redefine the x-condition.
       currentx = posx + i;
       
       for (int j =0; j<8; j++ ){   //Loops in the y-direction.
           //Update the currenty position.
           currenty = posy + j; 
           
           
           //Only will draw the 1's and ignore the 0's
           uint8_t bitty = (uint8_t)(hexval >> j); //right shift so that we always compare the last bit. 
                                                   //cast it to uint8_t due to right shifting automatically turning it to 16bit.
           if ((uint8_t)(bitty<<7) == 0b10000000){  
             ssd1306_drawPixel(currentx,currenty,1); //if the bit is a 1, we light it up. 
           }
           
       }
    }
}

//DRAWSTRING
//This function takes an initial position and a character array. 
//This will print string on the OLED screen. 
void drawString(unsigned int posx, unsigned int posy, char *stringy ){
    int i =0; //Character counter. Remove static so that function is reset every time it is called. 
    int j =0; //Position counter. 
    
    //Avoiding unreadable edge cases. 
    if (posx > 123){
        posx = 123;
    }
    

    
    
    while (stringy[i] != 0){    //Will run as long as it isn't NULL
        int newposx = posx + 5*i;    //5 bits wide
        
        //Don't worry about wrapping. 
        
        drawChar(newposx, posy, stringy[i]); //Draws the char. 
        
        i++; //This will iterate over the elements of the array.
    }
    
}


