
// Fadecandy server
OPC opc;

import org.openkinect.freenect.*;
import org.openkinect.processing.*;

Kinect kinect;

// Options for the Kinect feed
boolean ir = false;
boolean colorDepth = true;
boolean mirror = true;


boolean resetter = false; // Option to turn off the background resetting function
int resetSpeed = 240;  // How often (in frames) to run the resetting function

// Stars are the background animation
Star[] stars;
int numberOfStars = 300;
int numberOfStarstoDisplay = 300; // When there's lot of action happening, this number goes down
int action; // Counts up how much depth stuff is going on

// For dithering the colours back and forth
int maxHueDiff = 80;
int minHueDiff = -80;
float hueChangeSpeed = 5;
float[][] hueDifference;

PImage depth;

float[] depths;  // Depth feed in numbers
float[] originalDepths;  // Background readings to compare to

int[][] isoCounts;
int[][] equiCounts;

float theCX;
float theCY;
float theX;
float theY;
float isoSize = 65;
float sweepX;

void setup() {
  size(1440, 1080);
  colorMode(HSB, 360);
  frameRate(30);
  
  
  // background animation
  stars = new Star[numberOfStars];
  
  // initialise depth and background depth arrays
  depths = new float[307200];
  originalDepths = new float[307200];
   
  for (int i=0; i<depths.length; i++){
    depths[i] = 0;
  }
  for (int i=0; i<originalDepths.length; i++){
    originalDepths[i] = 0;
  }
  
  // initialise an array to dither the hue of each pixel
  hueDifference = new float[307200][2]; 
  
  for (int i=0; i<originalDepths.length; i++){   
    hueDifference[i][0] = random(minHueDiff,maxHueDiff); // start it off somewhere random
    hueDifference[i][1] = random(0,1);     // decides whether we're fading up or down
  }
  
  // background animation, its just a bunch of fadey circles, probably will improve this soon 
  for (int i=0; i<stars.length; i++){
    stars[i] = new Star();
  }


  isoCounts = new int[2][7];
  equiCounts = new int[2][12];  

  //fc count
  isoCounts[0][0] = 0;
  isoCounts[0][1] = 22;
  isoCounts[0][2] = 41; 

  // in strip
  isoCounts[1][0] = 22;
  isoCounts[1][1] = 19;
  isoCounts[1][2] = 19;

  // fc count
  equiCounts[0][0] = 0;
  equiCounts[0][1] = 20;
  equiCounts[0][2] = 40;

  // in strip
  equiCounts[1][0] = 20;
  equiCounts[1][1] = 20;
  equiCounts[1][2] = 20;


  // Connect to the local instance of fcserver. You can change this line to connect to another computer's fcserver
  opc = new OPC(this, "127.0.0.1", 7890);
  opc.showLocations(true);


  // index: Number for the first LED in the strip, starting with zero
  // count: How many LEDs are in the strip?
  // x, y: Center location, in pixels
  // spacing: Spacing between LEDs, in pixels
  // angle: Angle, in radians. Positive is clockwise, 0 is to the right.
  // reversed: true = Right to left, false = Left to right
  // type: 0=Short Short Long, 1=Short Long Short, 2=Long Short Short

  // opc.ledStrip(index, count, x, y, spacing, angle, reversed)
  // opc.ledStrip(0, 10, width/2, 50, 20, 0, false);
  // opc.ledStrip(10, 10, width/2, 100, 20, 0, true);   
  // opc.ledStrip(20, 10, width/2, 150, 20, 0, false);
  // opc.ledStrip(30, 10, width/2, 200, 20, 0, true);

  //void ledIsosceles(int fadecandy, int triangle, float x, float y, float angle)
  // index: start of the triangle on fadecandy
  // x: centre of pentagon
  // y: centre of pentagon
  // angle: angle of triangle from top to base, in degrees to avoid destroying my brain

  // bottom left pentagon
  opc.ledIsosceles(1, 3, 385, 620, 90, 0);      // A3
  opc.ledIsosceles(1, 2, 385, 620, 162, 0);     // A2
  opc.ledIsosceles(1, 6, 385, 620, 234, 3);     // A6
  opc.ledIsosceles(1, 5, 385, 620, 306, 3);     // A5
  opc.ledIsosceles(1, 4, 385, 620, 18, 0);      // A4

  opc.ledIsosceles(1, 8, 185, 685, 342, 1);  // A8

  // top left pentagon
  opc.ledIsosceles(2, 4, 480, 305, 90, 0);      // B4
  opc.ledIsosceles(2, 3, 480, 305, 162, 0);     // B3 
  opc.ledIsosceles(2, 2, 480, 305, 234, 0);     // B2
  opc.ledIsosceles(2, 6, 480, 305, 306, 3);     // B6
  opc.ledIsosceles(2, 5, 480, 305, 18, 4);      // B5 

  opc.ledIsosceles(2, 1, 360, 135, 54, 1);     // B1


  // top right pentagon
  opc.ledIsosceles(3, 5, 802, 305, 90, 3);      // C5
  opc.ledIsosceles(3, 4, 802, 305, 162, 0);     // C4
  opc.ledIsosceles(3, 3, 802, 305, 234, 0);     // C3
  opc.ledIsosceles(3, 2, 802, 305, 306, 0);     // C2
  opc.ledIsosceles(3, 6, 802, 305, 18, 3);      // C6  

  opc.ledIsosceles(3, 1, 925, 135, 126, 8);     // C1

  // bottom right pentagon
  opc.ledIsosceles(4, 6, 902, 620, 90, 3);      // D6
  opc.ledIsosceles(4, 5, 902, 620, 162, 3);     // D5
  opc.ledIsosceles(4, 4, 902, 620, 234, 0);     // D4
  opc.ledIsosceles(4, 3, 902, 620, 306, 0);     // D3
  opc.ledIsosceles(4, 2, 902, 620, 18, 6);      // D2 

  opc.ledIsosceles(4, 1, 1105, 685, 198, 1);      // D1

  // centre lower pentagon
  opc.ledIsosceles(5, 2, 641, 808, 90, 9);   // E2
  opc.ledIsosceles(5, 6, 641, 808, 163, 7);  // E6
  opc.ledIsosceles(5, 5, 641, 808, 234, 8);  // E5
  opc.ledIsosceles(5, 4, 641, 808, 306, 1);  // E4
  opc.ledIsosceles(5, 3, 641, 808, 18, 3);   // E3  



  //void ledEquilateral(int index, float x, float y, float angle){
  // index: start of the triangle on fadecandy
  // x: centre of pentagon
  // y: centre of pentagon
  // angle: angle of triangle from centre to base, in degrees to avoid destroying my brain
  // side 1 is base, clockwise from there.

  // top trio
  opc.ledEquilateral(2, 7, 641, 183, 270, 8); // B7
  opc.ledEquilateral(6, 2, 641, 347, 90, 5);  // F2  
  opc.ledEquilateral(2, 8, 641, 87, 90, 3);  // B8

  // top right trio
  opc.ledEquilateral(3, 7, 956, 420, 342, 4); // C7 ***
  opc.ledEquilateral(6, 3, 813, 470, 162, 5);  // F3  
  opc.ledEquilateral(3, 8, 1050, 385, 162, 5);  // C8 

  // top left trio
  opc.ledEquilateral(1, 7, 330, 420, 198, 4); // A7
  opc.ledEquilateral(6, 1, 470, 470, 18, 4);  // F1
  opc.ledEquilateral(1, 1, 233, 385, 18, 5);  // A1  


  // bottom left trio 
  opc.ledEquilateral(5, 1, 543, 681, 185, 6); // E1
  opc.ledEquilateral(5, 7, 460, 800, 5, 0);   // E7
  opc.ledEquilateral(5, 8, 400, 880, 185, 4); // E8  

  // bottom right trio
  opc.ledEquilateral(6, 4, 740, 681, 355, 4);  // F4
  opc.ledEquilateral(4, 7, 823, 800, 175, 5);  // D7
  opc.ledEquilateral(4, 8, 883, 880, 355, 7);  // D8
}

void draw() {
  fill(0,0,0,30);
  rect(0,0,width,height);  // low opacity background for nice fadey-ness
  
  //image(kinect.getVideoImage(), 620, 0); // useful for testing

   numberOfStarstoDisplay = int( map(action, 0,1500, numberOfStars, 0)); // map the amount of stars to the action, inversely
   action = 0; // reset the action amount
   
  for (int i=0; i< numberOfStarstoDisplay; i++){
    stars[i].update();
    stars[i].display();
  }
  
  fill(100,50,100);
  noStroke();

  depth = kinect.getDepthImage();

  // Take a new background reading
  if (frameCount % resetSpeed == 0 && resetter == true){

    for (int x=0; x < depth.width; x++){
      for (int y=0; y < depth.height; y++){
        int loc = x + y * depth.width;
        color currentColor = depth.pixels[loc];     
        if (hue(currentColor) !=0){
          originalDepths[loc] = hue(currentColor);  
        }
      }
    }      
  }
  

  adjustHue();
  drawDepthDifference();
  
 
  fill(0,0,0);
  rect(0,0,50,30);
  fill(360,0,360);
  text(frameRate,10,10);

  
}



void drawDepthDifference(){
   
  // Go through each pixel in the depth feed (well, actually, skip through them in fours, for speed) 
  for (int x=0; x < depth.width; x+=4){
    for (int y=0; y < depth.height; y+=4){
      int loc = x + y * depth.width;
      color currentColor = depth.pixels[loc];
      
      // if it's sufficiently different from the background depth
      if (hue(currentColor)+10<originalDepths[loc]){
        
        // update the value in the array, but gently, using lerp, this makes everything smoother
        depths[loc] = lerp(depths[loc], hue(currentColor), 0.3);
        
        // checking for outliers
        if (depths[loc] > 20 && depths[loc] < 350){
          
          // dither the hue by the difference amount
          float hue = depths[loc] + hueDifference[loc][0];
          if (hue > 360){
            hue = 360-hue;
          }
          
          if (hue < 0){
            hue = 360+hue;
          }
          // this pixel shows some movement, so add to the action count
          action++;
          
          // Draw it. 
          // Translated around so it fits on the LEDs in the right place
          fill(hue, 200,360,100);
          pushMatrix();
          translate(width/2, height/2);
          rotate(3.14);
          translate(900, -260);
          scale(-1,1);
          rect(x*2,y*2,12,12);
          popMatrix();
        }
      }
    }
  }
}

// Dithering the hue back and forth
void adjustHue(){
  for (int i=0; i<originalDepths.length; i++){   
    if (hueDifference[i][1] > 0.5){
      hueDifference[i][0] += hueChangeSpeed;
    }
    else{
       hueDifference[i][0] -= hueChangeSpeed;
    }
    
    if (hueDifference[i][0] > maxHueDiff){
      hueDifference[i][1] = 0.0; 
    }
    if (hueDifference[i][0] < minHueDiff){
      hueDifference[i][1] = 1.0; 
    }    
  }  
}



void mousePressed(){
  // Do the background reading
  for (int x=0; x < depth.width; x++){
    for (int y=0; y < depth.height; y++){
      int loc = x + y * depth.width;
      color currentColor = depth.pixels[loc];     
      if (hue(currentColor) !=0){
        originalDepths[loc] = hue(currentColor);  
      }  
    }
  }   
}
  


// Other older functions below, not using these any more but could be useful for testing

void drawDepthFull(){
  for (int x=0; x < depth.width; x++){
    for (int y=0; y < depth.height; y++){
      int loc = x + y * depth.width;
      color currentColor = depth.pixels[loc];
      //depths[loc] = hue(currentColor);
      depths[loc] = lerp(depths[loc], hue(currentColor), 0.3);
      if (depths[loc] > 10 && depths[loc] < 350){
        fill(depths[loc], 300,360);
        pushMatrix();
        translate(width/2, height/2);
        rotate(3.14);
        translate(900, -260);
        scale(-1,1);
        rect(x*2,y*2,2,2);
        popMatrix();
      }
    }
  }
}

void drawDepth(){
  for (int x=0; x < depth.width; x+=5){
    for (int y=0; y < depth.height; y+=5){
      int loc = x + y * depth.width;
      color currentColor = depth.pixels[loc];
      //depths[loc] = hue(currentColor);
      depths[loc] = lerp(depths[loc], hue(currentColor), 0.3);
      if (depths[loc] > 10 && depths[loc] < 350){
        fill(depths[loc], 300,360);
        pushMatrix();
        translate(width/2, height/2);
        rotate(3.14);
        translate(-800, -700);
        rect(x*3,y*3,30,30);
        popMatrix();
      }
    }
  }
}


void drawRawDepth(){
  
  pushMatrix();
  scale(2.5);
  translate(width/2, height/2);
  rotate(3.14);
  translate(400, 200);
  image(depth,0, 0);
  popMatrix();
  
}

void drawRawDepthNoAdjust(){
  
 
  image(depth,0, 0);
  
}
