// mission.ino is a class-like file that controls getting/setting the current mission.
// The mission is a list of target buoys that will be tracked.
int i = 0;
int mission[] = {1, 2, 4};
int getTarget() {
  //return 1;
  return mission[i];
}

// returns true if our mission is over
bool toNextTarget() {
  if ((i + 1) >= sizeof(mission)/sizeof(int)) {
    // mission is over
    i = 0;
    return true;
  }
  else {
    i += 1;
    return false;
  }
}
