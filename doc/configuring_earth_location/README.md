# Configuring location on Earth in a simulation
`NOTE` - **More technical information on parameters set in this tutorial can be found in [Inclination, declination and reference heading](../inclination_declination_reference_heading/README.md) document**

### To configure the simulation for the new location on the Earth, the following parameters must be set:
- reference_latitude
- reference_longitude
- reference_altitude
- reference_heading
- declination

### Also, it is recommended to set the following parameters, but simulation should work fine even with the default values left unchanged (as it influences magnetic field vector, but shouldn't affect yaw angle resulting from the sensor fusion)
- inclination
- magnitude

---

### To configure the simulation for the new location on the Earth, the following steps should be followed:

1. In [basic_sensors.xacro](https://bitbucket.org/agribotteam/agribot_description/src/master/urdf/basic_sensors.xacro) specify reference_latitude, reference_longitude and reference_altitude for the selected location (altitude can be checked e.g. in Google Earth)
2. In basic_sensors.xacro specify reference_heading (according to the convention explained in [Inclination, declination and reference heading](../inclination_declination_reference_heading/README.md) document) to choose robot's starting orientation.
3. Go to e.g. [British Geological Survey](https://geomag.bgs.ac.uk/data_service/models_compass/wmm_calc.html), input latitude, longitude and altitude information. The resulting table contains information about declination (**D**), inclination (**I**) and magnitude (**F**). **MF** (Main Field) part of the table is interesting for us in this case. 
>> * Declination and inclination from the table are correction values, while inclination and declination settings in the plugin mean value to be added to simulate this error, **therefore inclination and declination values specified in basic_sensors.xacro should be opposite of the values read from the table.**
>> * Magnitude in the table is specified in nanoteslas and should be coverted to Teslas before inputting in basic_sensors.xacro

>> For example, given the following table the settings in basic_sensors.xacro should be:
>> * declination = -5.283
>> * inclination = -67.25
>> * magnitude = 4.976e-5

>> ![inlination_declination_magnitude_table](diagrams/inlination_declination_magnitude_table.png)
