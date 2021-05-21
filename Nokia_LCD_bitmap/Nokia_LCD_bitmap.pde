// Written by Matthew Walker  -  mwwalk@gmail.com

/* This program creates a screen 84 x 48 pixels wide that the user can color (b&w)
 by holding the left mouse button and dragging it over pixels.  You can delete the 
 color by holding the right mouse button.
 
 Click the save button to have the text file saved to the sketch folder.
 
 Click the load image button to load a jpg named "image.jpg" from the sketch folder.
 It will convert the photo to 84x48 pixels and show images from the process.
 
 The other buttons move the pixels around in ways that might be helpful.
 Note that moving pixels at the edges is destructive, so if you move your pixel
 off the screen, it will stop existing.
 
 This program is released under the don't be a douchbag liscense.  You are 
 free to do whatever you want with this, but please give me credit if you can. 
 Remember, Karma's a bitch.
 */


PrintWriter output;    // file output
PFont myFont;          // Helvetica font

// Image files - Only the first two are necessary, 
//  the rest could be deleted if you don't want to see process 
PImage img;            
PImage img2;
PImage img3;
PImage img4;
PImage img5;
PImage img6;


int pixelSize = 10;            // Size of the squares on screen
int x = 84;                    // Pixels in X direction
int y = 48;                    // Pixels in Y direction
int imageNum = 0;              // This is used for showing image process
float averagePixelValue = 0;   // Average pixel brightness


// These are the squares that make up the actual pixels
// Using custom Square class shown below
Square boxes[][];
Square temp[][];

// Create buttons using custom Button class shown below (x,y,width,height, label)
Button saveButton = new Button(107, 550, 100, 20, "Save BMP");
Button imageButton = new Button(107, 490, 100, 20, "Load Image");
Button inverseButton = new Button(633, 550, 100, 20, "Inverse");
Button flipHor = new Button(633, 490, 100, 20, "Flip Horizontally");
Button flipVert = new Button(633, 520, 100, 20, "Flip Vertically");
Button moveLeft = new Button(315, 520, 100, 20, "Move Left");
Button moveDown = new Button(370, 550, 100, 20, "Move Down");
Button moveUp = new Button(370, 490, 100, 20, "Move Up");
Button moveRight = new Button(425, 520, 100, 20, "Move Right");


void setup()
{
  size(840, 580);
  myFont = createFont("Helvetica", 12);

  boxes = new Square[x][y];
  temp = new Square[x][y];

  // Go through and gives Squares location, size, and white color
  for (int i=0; i<x; i++)
  {
    for (int j=0; j<y; j++)
    {
      boxes[i][j] = new Square(i*pixelSize, j*pixelSize, pixelSize, pixelSize);
      temp[i][j] = new Square(i*pixelSize, j*pixelSize, pixelSize, pixelSize);
      boxes[i][j].myColor = color(255);
    }
  }
}


void draw()
{
  background(191);    // Set background to a nice gray

  // Determine if each pixel is being clicked and color correctly.
  // Uses the isOn() function of each square the returns true if mouse is over it
  // This isn't the most effecient way to do it, but allows for easier changes in 
  //  the future to change pixel size and number
  // If left mouse is down, pixel is colored
  // If right mouse is down, pixel is colored (control-click on mac)
  for (int i=0; i<x; i++)
  {
    for (int j=0; j<y; j++)
    {
      if (mouseButton == LEFT)
        if (boxes[i][j].isOn())
          boxes[i][j].myColor = color(0);
      if (mouseButton == RIGHT)
        if (boxes[i][j].isOn())
          boxes[i][j].myColor = color(255);

      // Draws each square using the draw function of that object
      boxes[i][j].draw();
    }
  }

  // Draws the middle lines
  stroke(0);
  strokeWeight(2);
  line((x/2)*pixelSize, 0, (x/2)*pixelSize, y*pixelSize);
  line(0, (y/2)*pixelSize, x*pixelSize, (y/2)*pixelSize);

  // Draws Buttons
  saveButton.draw();
  flipVert.draw();
  flipHor.draw();
  moveLeft.draw();
  moveRight.draw();
  moveUp.draw();
  moveDown.draw();
  imageButton.draw();
  inverseButton.draw();

  // This is used to show the images from processing
  // Could be deleted if not wanted
  if (imageNum > 0)
    delay(1000);
  if (imageNum == 1)
  {
    image(img, 0, 0);
    imageNum = 2;
  }
  else if (imageNum == 2)
  {
    image(img2, 0, 0);
    imageNum = 3;
  }
  else if (imageNum == 3)
  {
    image(img3, 0, 0);
    imageNum = 4;
  }
  else if (imageNum == 4)
  {
    image(img4, 0, 0);
    imageNum = 5;
  }
  else if (imageNum == 5)
  {
    image(img5, 0, 0);
    imageNum = 0;
  }
  
}

void mousePressed()
{
  // Save the picture
  if (saveButton.isClicked())
    savePicture();

  // Process image from sketch folder
  if (imageButton.isClicked())
    processImage();
  
  // Invert the pixels  
  if(inverseButton.isClicked())
  {
    for (int i=0; i<x; i++)
    {
      for (int j=0; j<y; j++)
      {
        if(boxes[i][j].myColor == color(0))
          boxes[i][j].myColor = color(255);
        else
          boxes[i][j].myColor = color(0);
      }
    }
  }

  // Flip pixels horizontally (Around vertical line of symetry)
  // Uses temp sqaures for this, which is easier but not optimal
  if (flipHor.isClicked())
  {
    // Copy colors to temp Squares
    for (int i=0; i<x; i++)
      for (int j=0; j<y; j++)
        temp[i][j].myColor = boxes[x-1-i][j].myColor;

    // Move temp colors back to real Squares
    for (int i=0; i<x; i++)
      for (int j=0; j<y; j++)
        boxes[i][j].myColor = temp[i][j].myColor;
  }

  // Flip pixels vertically (Around horizontal line of symetry)
  // Uses temp sqaures for this, which is easier but not optimal
  if (flipVert.isClicked())
  {
    // Copy colors to temp Squares
    for (int i=0; i<x; i++)
      for (int j=0; j<y; j++)
        temp[i][j].myColor = boxes[i][y-1-j].myColor;

    // Move temp colors back to real Squares
    for (int i=0; i<x; i++)
      for (int j=0; j<y; j++)
        boxes[i][j].myColor = temp[i][j].myColor;
  }

  // The next four functions move the pixels around the screen destructively
  if (moveLeft.isClicked())
  {
    for (int j=0; j<y; j++)
    {
      for (int i=0; i<x-1; i++)
      {
        boxes[i][j].myColor = boxes[i+1][j].myColor;
      }
      boxes[x-1][j].myColor = color(255);
    }
  }

  if (moveRight.isClicked())
  {
    for (int j=0; j<y; j++)
    {
      for (int i=x-1; i>0; i--)
        boxes[i][j].myColor = boxes[i-1][j].myColor;

      boxes[0][j].myColor = color(255);
    }
  }

  if (moveUp.isClicked())
  {
    for (int i=0; i<x; i++)
    {
      for (int j=0; j<y-1; j++)
        boxes[i][j].myColor = boxes[i][j+1].myColor;

      boxes[i][y-1].myColor = color(255);
    }
  }

  if (moveDown.isClicked())
  {
    for (int i=0; i<x; i++)
    {
      for (int j=y-1; j>0; j--)
        boxes[i][j].myColor = boxes[i][j-1].myColor;

      boxes[i][0].myColor = color(255);
    }
  }
}

// Processes image to get pixels from it
// Also generates images to be shown that will 
//  show the process of converting image
// Those images aren't necesary but I like them
void processImage()
{
  // Loads image from file
  img = loadImage("bman.png");
  
  // If it couldn't load image, exit function so that
  //  there won't be a null pointer exception later,
  //  but the program won't exit and delete your work.
  if (img == null)
    return;

  // Resizes to fit in window depending on if the 
  //  image is constrained in the x or y direction
  img2 = img.get();
  if (img2.width / 84. > img2.height / 48.)
    img2.resize(840, 0);
  else
    img2.resize(0, 480);

  // Create another image and fill with white
  img3 = createImage(840, 480, RGB);
  img3.loadPixels();
  for (int i=0; i<840; i++)
  {
    for (int j=0; j < 480; j++)
    {
      img3.pixels[i+840*j] = color(255);
    }
  }

  // Copies original image to new image with white as background
  img2.loadPixels();
  for (int i=0; i<img2.width; i++)
  {
    for (int j=0; j < img2.height; j++)
    {
      img3.pixels[i+840*j] = img2.pixels[i+img2.width*j];
    }
  }
  img3.updatePixels();

  // Makes it gray
  img4 = img3.get();
  img4.filter(GRAY);

  // Resizes to 84x48 and gets average brightness
  img5 = img4.get();
  img5.resize(84, 48);
  img5.loadPixels();
  averagePixelValue = 0;
  for (int i=0; i<84; i++)
  {
    for (int j=0; j<48; j++)
    {
      averagePixelValue += brightness(img5.pixels[i+84*j]);
    }
  }
  averagePixelValue /= (84*48);

  // Go through array and convert to b&w
  // If pixel brightness is more than average, make it white
  // Otherwise, make it black
  for (int i=0; i<img5.pixels.length; i++)
  {
    if (brightness(img5.pixels[i]) >= averagePixelValue)
      img5.pixels[i] = color(255);
    else
      img5.pixels[i] = color(0);
  }
  img5.updatePixels();


  // Color boxes based on the above so that they can be further manipulated
  for (int i=0; i<84; i++)
  {
    for (int j=0; j<48; j++)
    {
      boxes[i][j].myColor = color(brightness(img5.pixels[i+84*j]));
    }
  }

  img5.resize(840, 480);  // resize for better display

  // Change imageNum so that it will display the process
  imageNum = 1;
}



void savePicture()
{
  // Output filename
  output = createWriter("bmp.txt");

  // Helpful information at beginning of file
  output.println("If you want to replace the buffer in the Adafruit_PCD8544.cpp file, use this beginning: ");
  output.println("uint8_t pcd8544_buffer[504]");
  output.println("");
  output.println("If you want to put the use the image as a graphic at more than just the splash screen, use this beginning: ");
  output.println("static unsigned char PROGMEM yourNameHere[]");

  output.println();
  output.println("So it would like like this: ");
  output.println("uint8_t pcd8544_buffer[504] = { Lots of Hex numbers seperated by commas };");
  output.println("");

  output.println("Your numbers are below: ");
  output.println();
  output.print("= {");

  // Go through every pixel by starting at top left corner and going down eight,
  //   then going back up to the next column and going down eight,
  //   continuing until you reach the end of the row and then jumping to the first
  //   column nine pixels down, etc.
  for (int j=0; j<y; j+=8)
  {
    for (int i=0; i<x; i++)
    {
      // For each 8 pixel subset, create string and append 1 or 0 depending on pixel
      // The top pixel is the least significant bit (the one on the right)
      String s = "";
      for (int k=0; k<8; k++)
      {
        if (boxes[i][j+k].myColor == color(255))
          s = "0" + s;
        else
          s = "1" + s;
      }

      //convert binary string to int
      int num = unbinary(s);

      // Output number as hexidecimal
      output.print("0x");
      output.print(hex(num, 2));
      output.print(", ");
    }
  }

  // Print final stuff, flush the file, and close
  output.print("};");
  output.flush();
  output.close();
}


// Button class used for buttons
class Button
{
  public int x;
  public int y;
  public int wide;
  public int high;
  public int visible;
  public int radius;
  public int fontSize;
  public color myColor;
  public color textColor;
  public String label;

  public Button(int x, int y, int w, int h)
  {
    this(x, y, w, h, "");
  }

  public Button(int x, int y, int w, int h, String s)
  {
    this.x = x;
    this.y = y;
    this.wide = w;
    this.high = h;
    this.myColor = 255;
    this.textColor = 0;
    this.label = s;
    this.visible = 1;
    this.fontSize = 12;

    if (wide >= high)
      this.radius = high/4;
    else 
      this.radius = wide/4;
  }

  public void draw()
  {
    fill(myColor);
    noStroke();
    rect(x, y+radius, wide, high-2*radius);
    rect(x+radius, y, wide-2*radius, high);
    ellipse(x+radius, y+radius, 2*radius, 2*radius);
    ellipse(x+radius, y+high-radius, 2*radius, 2*radius);
    ellipse(x+wide-radius, y+radius, 2*radius, 2*radius);
    ellipse(x+wide-radius, y+high-radius, 2*radius, 2*radius);
    fill(textColor);
    textAlign(CENTER, CENTER);
    textSize(fontSize);
    text(this.label, x+wide/2, y+high/2);
  }

  public boolean isClicked()
  {
    if (mouseX > x && mouseX < x+wide && mouseY > y && mouseY < y+high)
      return true;
    else
      return false;
  }
}

// Square class used for the pixels
class Square
{
  public int x;
  public int y;
  public int wide;
  public int high;
  public int myColor;
  public int strokeSize;

  public Square(int x, int y, int wide, int high)
  {
    this.x = x;
    this.y = y;
    this.wide = wide;
    this.high = high;
    this.myColor = color(0);
    this.strokeSize = 1;
  }

  public void draw()
  {
    stroke(0);
    strokeWeight(strokeSize);
    fill(myColor);
    rect(x, y, wide, high);
  }

  public boolean isClicked()
  {
    if (mouseX > x && mouseX < x+wide && mouseY > y && mouseY < y+high)
      return true;
    else
      return false;
  }

  public boolean isOn()
  {
    if (mouseX > x && mouseX < x+wide && mouseY > y && mouseY < y+high)
      return true;
    else
      return false;
  }
}
