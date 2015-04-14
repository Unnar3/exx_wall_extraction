class ColorGradient
{
private:
  struct ColorPoint  // Internal class used to store colors at different points in the gradient.
  {
    float r,g,b;      // Red, green and blue values of our color.
    float val;        // Position of our color along the gradient (between 0 and 1).
    ColorPoint(float red, float green, float blue, float value)
      : r(red), g(green), b(blue), val(value) {}
  };
  std::vector<ColorPoint> color;      // An array of color points in ascending value.
 
public:
  //-- Default constructor:
  ColorGradient(int i = 3)  { createHeatMapGradient(i); }
 
  //-- Inserts a new color point into its correct position:
  void addColorPoint(float red, float green, float blue, float value)
  {
    for(int i=0; i<color.size(); i++)  {
      if(value < color[i].val) {
        color.insert(color.begin()+i, ColorPoint(red,green,blue, value));
        return;  
      }
    }
    color.push_back(ColorPoint(red,green,blue, value));
  }
 
  //-- Inserts a new color point into its correct position:
  void clearGradient() { color.clear(); }
 
  //-- Places a 5 color heapmap gradient into the "color" vector:
  void createHeatMapGradient(int i)
  {
    color.clear();

    if ( i == 2 ){
      color.push_back(ColorPoint(0, 0, 1,   0.0f));      // Blue.
      color.push_back(ColorPoint(1, 0, 0,   1.0f));      // Red.
    }
    else if ( i == 3 ){
      color.push_back(ColorPoint(0, 0, 1,   0.0f));      // Blue
      color.push_back(ColorPoint(0, 1, 0,   0.5f));      // Green.
      color.push_back(ColorPoint(1, 0, 0,   1.0f));      // Red.
    }
    else if ( i == 5 ){
      color.push_back(ColorPoint(0, 0, 1,   0.0f));      // Blue.
      color.push_back(ColorPoint(0, 1, 1,   0.25f));     // Cyan.
      color.push_back(ColorPoint(0, 1, 0,   0.5f));      // Green.
      color.push_back(ColorPoint(1, 1, 0,   0.75f));     // Yellow.
      color.push_back(ColorPoint(1, 0, 0,   1.0f));      // Red.
    }
    else{
      std::cout << "Number of colors not defined, Options are 2, 3 or 5, using 3." << std::endl;
      color.push_back(ColorPoint(0, 0, 1,   0.0f));      // Blue
      color.push_back(ColorPoint(0, 1, 0,   0.5f));      // Green.
      color.push_back(ColorPoint(1, 0, 0,   1.0f));      // Red.
    }
  }
 
  //-- Inputs a (value) between 0 and 1 and outputs the (red), (green) and (blue)
  //-- values representing that position in the gradient.
  void getColorAtValue(const float value, int &red, int &green, int &blue)
  {
    if(color.size()==0)
      return;
 
    for(int i=0; i<color.size(); i++)
    {
      ColorPoint &currC = color[i];
      if(value < currC.val)
      {
        ColorPoint &prevC  = color[ std::max(0,i-1) ];
        float valueDiff    = (prevC.val - currC.val);
        float fractBetween = (valueDiff==0) ? 0 : (value - currC.val) / valueDiff;
        red   = ((prevC.r - currC.r)*fractBetween + currC.r) * 255;
        green = ((prevC.g - currC.g)*fractBetween + currC.g) * 255;
        blue  = ((prevC.b - currC.b)*fractBetween + currC.b) * 255;
        return;
      }
    }
    red   = (color.back().r) * 255;
    green = (color.back().g) * 255;
    blue  = (color.back().b) * 255;
    return;
  }
};