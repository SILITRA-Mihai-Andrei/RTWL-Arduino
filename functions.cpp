bool isNumberOrLetter(char ch)
{
  int intCh = (int)ch;
  return intCh == 45 || (intCh >= 48 && intCh <= 57) || (intCh >= 65 && intCh <= 90) || (intCh >= 97 && intCh <= 122);
}

void toUppercase(int len, char *msg_received)
{
  for (int i = 0; i < len; i++)
  {
    int ic = (int)msg_received[i];
    if (ic >= 97 && ic <= 122)
      msg_received[i] = char(ic - 32);
  }
}

char *degrees2direction(int degrees)
{
  static char direction[17][4] = {"N", "NNE", "NE", "ENE", 
                                  "E", "ESE", "SE", "SSE", 
                                  "S", "SSW", "SW", "WSW", 
                                  "W", "WNW", "NW", "NNW", "N"}; // 68 bytes
  
  if(degrees < 0 || degrees >= 17) return "U";
  degrees = degrees % 360;
  int idx = (degrees * 10 + 112) / 225;
  return direction[idx];
}