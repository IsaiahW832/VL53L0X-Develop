# VL53L0X-Time Of Flight Distance Sensor FRC API

API for I2C communication with VL53L0X distance sensor over Roborio using WPI I2C libraries.

To Use:
  1. Create and initilize FRC_VL53L0X object
  
    Ex:
    
       FRC_VL53L0X* lox = new FRC_VL53L0X();
    
  2. Initilize sensor with "begin" method.
  
      Optional Arg1: Sensor Port(Will default to roborio port-kOnBoard)
      
      Optional Arg2: Address of Device(Will default to default device address 0x29)
      
      Optional Arg3: Debug(Will default to false)
  
    Ex:
    
       if(!lox->begin()){
		     std::cout << "Failed to Boot VL53L0X\n" << ":(\n";
		     while(1);
	     }
  
  3. Preform Single Ranging Measurement
	  
    Ex:
    
       VL53L0X_RangingMeasurementData_t measure;
	     lox->getSingleRangingMeasurement(&measure);

	     if (measure.RangeStatus != 4) {  // phase failures have incorrect data
		     std::cout << "Distance (mm): " << measure.RangeMilliMeter << std::endl;
	     } else {
		     std::cout << " out of range " << std::endl;
	     }
