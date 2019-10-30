class averageFilter {
    int maxValues;
    float values[32];
    uint8_t iterator;
    float average;
    float threshold = 100.0;

  public:
    averageFilter() {
      maxValues = sizeof(values) / sizeof(float);
      //iterator = 0;
      for (int i = 0; i < maxValues; i++) {
        values[i] = 0;
      }
    }

    float filter(float v) {
     // if (abs(average - v) < threshold && millis() > 1000)
        // return average;

      // shift all values one to the left
      for (int i = 0; i < maxValues - 1; i++) {
        values[i] = values[i + 1];
      }
      // add current value to the latest index
      values[maxValues - 1] = v;

      // calculate the average
      average = 0;
      for (int i = 0; i < maxValues; i++) {
        average += values[i];
      }
      average /= maxValues;
      
      return average;
    }

};
