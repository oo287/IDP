
void read_line_sensors() {                                       // Update the booleans line_N with whether they are on the line or not (cutoff = 500)

  if (analogRead(LS1_PIN) < 500) {                               // Checks if we meet the cutoff or not (less than 500 = White)
    line_1 = true;                                               // line_1, line_2 and line_3 are global variables
  }
  else {
    line_1 = false;
  }
  if (analogRead(LS2_PIN) < 500) {
    line_2 = true;
  }
  else {
    line_2 = false;
  }
  if (analogRead(LS3_PIN) < 500) {
    line_3 = true;
  }
  else {
    line_3 = false;
  }
}
