class sensor_interface
{

public:
   virtual int GetSensorData() = 0;
   virtual void PrintData() = 0;
   virtual void SampleData() = 0;
   
private:
/* data */

};

