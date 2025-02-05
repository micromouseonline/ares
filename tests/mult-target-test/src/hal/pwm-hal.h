
class PWMHAL {
 public:
  virtual void initialize() = 0;  // Initialize the PWM hardware
  virtual void setFrequency(int frequency) = 0;
  virtual void setDuty(float duty) = 0;

  virtual ~PWMHAL() = default;
};
