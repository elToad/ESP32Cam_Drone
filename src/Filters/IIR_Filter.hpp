class IIR_Filter
{
private:
    const float alpha_ = 0.1f;
    float y = 0.0f;

public:
    IIR_Filter(const float alpha);
    IIR_Filter();
    ~IIR_Filter();
    float filter(float x);
};

IIR_Filter::IIR_Filter(/* args */)
{
}

IIR_Filter::IIR_Filter(const float alpha) : alpha_(alpha)
{
}

IIR_Filter::~IIR_Filter()
{
}

// Process a single input sample x, and return output sample y.
float IIR_Filter::filter(float x){
  y = (1-alpha_)*x + alpha_*y; // Calculate y
  return y;
}