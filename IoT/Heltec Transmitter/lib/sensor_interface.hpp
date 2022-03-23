class SensorInterface
{
public:
   struct SensorData
   {
   };

   virtual SensorData GetSensorData() = 0;
   virtual void PrintData() = 0;
};

class TransmitterInterface : public SensorInterface
{
public:
   virtual void TransmitData() = 0;
};

class ReceiverInterface : public SensorInterface
{
public:
   virtual void ParseData() = 0;
};
