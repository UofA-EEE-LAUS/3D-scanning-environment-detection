int PackageLength = 5;  // How many values do we expect the package to send?
float Package[5];       // This should be the same length as above, I just dont know how to make variable length arrays
// Generic Setup
void setup() 
{
    Serial.begin(9600);
}

//Generic main loop
void loop() 
{
  //If the Serial port is available
  if (Serial.available()>0) 
  {
    String s = "";      //Setup an empty string
    while (Serial.available()>0)    //while there is data being sent to the serial port
    {
      char k = Serial.read();       //Read the character, and put it in our string variable
      s += k;
    }
    //char inp = Serial.read();


    //Now that we have the package in string format, we want to convert it to float
    for (int i=0; i<PackageLength-1; i++)
    {
      //Serial.println(s);
      //We need the package to be broken up with slashes "/"
      //So we loop over how many parts there are in the package -1
      int brk = s.indexOf("/");               //Find the next instance of a break
      String tString = s.substring(0,brk-1);  //add everything before the break to a tempString
      Package[i] = tString.toFloat();         //set the package value at the respective index to the tempString, and change it to a float
      s.remove(0,brk+1);                      //Then remove everthing before and including the break character
    }
    //Serial.print(s);
    //Doing the above loop misses the last part of the string, so we just add it here
    Package[PackageLength-1] = s.toFloat();

    //This is just some debugging code to display on the Raspberry Pi's end
    Serial.print("Arduino Reads: ");
    for (int i=0; i<PackageLength; i++)
    {
      Serial.print(Package[i]);
      Serial.print(" ");
    }
    //Serial.print(s.toFloat());
  }
} 
