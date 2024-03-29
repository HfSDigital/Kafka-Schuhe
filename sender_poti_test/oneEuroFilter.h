typedef unsigned long TimeStamp ; // in seconds

static const TimeStamp UndefinedTime = -1.0 ;

class LowPassFilter {

    double y, a, s ;
    bool initialized ;

    void setAlpha(double alpha) {
      if (alpha <= 0.0 || alpha > 1.0)
        Serial.println("alpha should be in (0.0., 1.0]") ;
      a = alpha ;
    }

  public:

    LowPassFilter(double alpha, double initval = 0.0) {
      y = s = initval ;
      setAlpha(alpha) ;
      initialized = false ;
    }

    double filter(double value) {
      double result ;
      if (initialized)
        result = a * value + (1.0 - a) * s ;
      else {
        result = value ;
        initialized = true ;
      }
      y = value ;
      s = result ;
      return result ;
    }

    double filterWithAlpha(double value, double alpha) {
      setAlpha(alpha) ;
      return filter(value) ;
    }

    bool hasLastRawValue(void) {
      return initialized ;
    }

    double lastRawValue(void) {
      return y ;
    }

} ;

// -----------------------------------------------------------------

class OneEuroFilter {

    double freq ;
    double mincutoff ;
    double beta_ ;
    double dcutoff ;
    LowPassFilter *x ;
    LowPassFilter *dx ;
    TimeStamp lasttime ;

    double alpha(double cutoff) {
      double te = 1.0 / freq ;
      double tau = 1.0 / (2 * M_PI * cutoff) ;
      return 1.0 / (1.0 + tau / te) ;
    }

    void setFrequency(double f) {
      if (f <= 0) Serial.println("freq should be >0") ;
      freq = f ;
    }

    void setMinCutoff(double mc) {
      if (mc <= 0) Serial.println("mincutoff should be >0") ;
      mincutoff = mc ;
    }

    void setBeta(double b) {
      beta_ = b ;
    }

    void setDerivateCutoff(double dc) {
      if (dc <= 0) Serial.println("dcutoff should be >0") ;
      dcutoff = dc ;
    }

  public:

    OneEuroFilter(double freq,
                  double mincutoff = 1.0, double beta_ = 0.0, double dcutoff = 1.0) {
      setFrequency(freq) ;
      setMinCutoff(mincutoff) ;
      setBeta(beta_) ;
      setDerivateCutoff(dcutoff) ;
      x = new LowPassFilter(alpha(mincutoff)) ;
      dx = new LowPassFilter(alpha(dcutoff)) ;
      lasttime = UndefinedTime ;
    }

    double filter(double value, TimeStamp timestamp = UndefinedTime) {
      // update the sampling frequency based on timestamps
      if (lasttime != UndefinedTime && timestamp != UndefinedTime)
        freq = 1.0 / (timestamp - lasttime) ;
      lasttime = timestamp ;
      // estimate the current variation per second
      double dvalue = x->hasLastRawValue() ? (value - x->lastRawValue()) * freq : 0.0 ; // FIXME: 0.0 or value?
      double edvalue = dx->filterWithAlpha(dvalue, alpha(dcutoff)) ;
      // use it to update the cutoff frequency
      double cutoff = mincutoff + beta_ * fabs(edvalue) ;
      // filter the given value
      return x->filterWithAlpha(value, alpha(cutoff)) ;
    }

    ~OneEuroFilter(void) {
      delete x ;
      delete dx ;
    }

} ;
