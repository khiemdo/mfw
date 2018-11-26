# Protocol Documentation
<a name="top"/>

## Table of Contents
* [adc.proto](#adc.proto)
 * [Adc](#BetCOM.Adc)
 * [AdcConf](#BetCOM.AdcConf)
 * [Current](#BetCOM.Current)
 * [Voltage](#BetCOM.Voltage)
 * [Current.Channel](#BetCOM.Current.Channel)
 * [Voltage.Channel](#BetCOM.Voltage.Channel)
* [airspeed.proto](#airspeed.proto)
 * [Airspeed](#BetCOM.Airspeed)
 * [AirspeedConf](#BetCOM.AirspeedConf)
* [angle_of_attack.proto](#angle_of_attack.proto)
 * [AngleOfAttack](#BetCOM.AngleOfAttack)
 * [AngleOfAttackConf](#BetCOM.AngleOfAttackConf)
* [baro.proto](#baro.proto)
 * [Barometer](#BetCOM.Barometer)
 * [BarometerConf](#BetCOM.BarometerConf)
* [betcall.proto](#betcall.proto)
 * [Sensors](#BetCALL.Sensors)
* [betconf.proto](#betconf.proto)
 * [Configuration](#BetCONF.Configuration)
* [betpush.proto](#betpush.proto)
 * [Actuators](#BetPUSH.Actuators)
* [gps_piksi.proto](#gps_piksi.proto)
 * [Gps](#BetCOM.Gps)
 * [GpsBaseline](#BetCOM.GpsBaseline)
 * [GpsConf](#BetCOM.GpsConf)
 * [GpsDilutionOfPrecision](#BetCOM.GpsDilutionOfPrecision)
 * [GpsPosition](#BetCOM.GpsPosition)
 * [GpsTime](#BetCOM.GpsTime)
 * [GpsVelocity](#BetCOM.GpsVelocity)
 * [GpsAxialSystem](#BetCOM.GpsAxialSystem)
 * [GpsStatus](#BetCOM.GpsStatus)
* [gps.proto](#gps.proto)
* [humidity.proto](#humidity.proto)
 * [Humidity](#BetCOM.Humidity)
 * [HumidityConf](#BetCOM.HumidityConf)
* [imu.proto](#imu.proto)
 * [Imu](#BetCOM.Imu)
 * [ImuConf](#BetCOM.ImuConf)
 * [ImuMeasurement](#BetCOM.ImuMeasurement)
 * [ImuConf.AccelRange](#BetCOM.ImuConf.AccelRange)
 * [ImuConf.GyroRange](#BetCOM.ImuConf.GyroRange)
 * [ImuType](#BetCOM.ImuType)
* [led.proto](#led.proto)
 * [Led](#BetCOM.Led)
 * [LedBrightness](#BetCOM.LedBrightness)
 * [LedMode](#BetCOM.LedMode)
 * [LedRGB](#BetCOM.LedRGB)
 * [Leds](#BetCOM.Leds)
 * [Led.Location](#BetCOM.Led.Location)
* [line_angle.proto](#line_angle.proto)
 * [LineAngle](#BetCOM.LineAngle)
 * [LineAngleConf](#BetCOM.LineAngleConf)
* [strain.proto](#strain.proto)
 * [Dms](#BetCOM.Dms)
 * [DmsConf](#BetCOM.DmsConf)
* [temperature.proto](#temperature.proto)
 * [Temperature](#BetCOM.Temperature)
 * [TemperatureConf](#BetCOM.TemperatureConf)
* [tube_angle.proto](#tube_angle.proto)
 * [TubeAngle](#BetCOM.TubeAngle)
 * [TubeAngleConf](#BetCOM.TubeAngleConf)
* [types.proto](#types.proto)
 * [BasicConf](#BetCOM.BasicConf)
 * [Ticks](#BetCOM.Ticks)
 * [Time](#BetCOM.Time)
 * [Timestamp](#BetCOM.Timestamp)
 * [XYZW_f](#BetCOM.XYZW_f)
 * [XYZW_i](#BetCOM.XYZW_i)
 * [XYZ_f](#BetCOM.XYZ_f)
 * [XYZ_i](#BetCOM.XYZ_i)
 * [Globals](#BetCOM.Globals)
* [Scalar Value Types](#scalar-value-types)

<a name="adc.proto"/>
<p align="right"><a href="#top">Top</a></p>

## adc.proto

<a name="BetCOM.Adc"/>
### Adc
Message container for ADC message

| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| time | [Time](#BetCOM.Time) | required | timestamp of message |
| current | [Current](#BetCOM.Current) | repeated | current measurements for one or more channels |
| voltage | [Voltage](#BetCOM.Voltage) | repeated | voltage measurements for one ore more channels |

<a name="BetCOM.AdcConf"/>
### AdcConf
ADC configurattion message

| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| basic_conf | [BasicConf](#BetCOM.BasicConf) | required | basic configuration of adc |
| fullscale_voltage_12V | [uint32](#uint32) | optional | basic configuration of adc |
| fullscale_voltage_7V | [uint32](#uint32) | optional |  |
| fullscale_voltage_5V | [uint32](#uint32) | optional |  |
| fullscale_voltage_3_3V | [uint32](#uint32) | optional |  |
| fullscale_voltage_ref_1V | [uint32](#uint32) | optional |  |
| fullscale_voltage_ref_2V | [uint32](#uint32) | optional |  |
| fullscale_voltage_ref_3V | [uint32](#uint32) | optional |  |
| fullscale_current_12V | [uint32](#uint32) | optional |  |
| fullscale_current_7V | [uint32](#uint32) | optional |  |
| fullscale_current_5V | [uint32](#uint32) | optional |  |
| fullscale_current_3_3V | [uint32](#uint32) | optional |  |
| fullscale_current_3_3V_uC | [uint32](#uint32) | optional |  |
| amplification_voltage_12V | [uint32](#uint32) | optional |  |
| amplification_voltage_7V | [uint32](#uint32) | optional |  |
| amplification_voltage_5V | [uint32](#uint32) | optional |  |
| amplification_voltage_3_3V | [uint32](#uint32) | optional |  |
| amplification_voltage_ref_1V | [uint32](#uint32) | optional |  |
| amplification_voltage_ref_2V | [uint32](#uint32) | optional |  |
| amplification_voltage_ref_3V | [uint32](#uint32) | optional |  |
| amplification_current_12V | [uint32](#uint32) | optional |  |
| amplification_current_7V | [uint32](#uint32) | optional |  |
| amplification_current_5V | [uint32](#uint32) | optional |  |
| amplification_current_3_3V | [uint32](#uint32) | optional |  |
| amplification_current_3_3V_uC | [uint32](#uint32) | optional |  |

<a name="BetCOM.Current"/>
### Current
Submessage for current measurements

| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| time | [Time](#BetCOM.Time) | required | timestamp of measurement |
| channel | [Current.Channel](#BetCOM.Current.Channel) | required | Current channel which is measured |
| current_raw | [sint32](#sint32) | optional | measured raw value |
| current | [float](#float) | optional | measrement in A |

<a name="BetCOM.Voltage"/>
### Voltage
Voltage sub-message

| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| time | [Time](#BetCOM.Time) | required | timestamp of measurement |
| channel | [Voltage.Channel](#BetCOM.Voltage.Channel) | required | Current channel which is measured |
| voltage_raw | [sint32](#sint32) | optional | measured raw value |
| voltage | [float](#float) | optional | measurement in V |


<a name="BetCOM.Current.Channel"/>
### Current.Channel
Current channel declarations depending on voltage

| Name | Number | Description |
| ---- | ------ | ----------- |
| CURRENT_12V | 0 | 12V circuit |
| CURRENT_7V | 1 | 7V circuit |
| CURRENT_5V | 2 | 5V circuit |
| CURRENT_3_3V | 3 | 3.3V circuit |

<a name="BetCOM.Voltage.Channel"/>
### Voltage.Channel
Voltage channel declarations

| Name | Number | Description |
| ---- | ------ | ----------- |
| VOLTAGE_12V | 0 |  |
| VOLTAGE_7V | 1 |  |
| VOLTAGE_5V | 2 |  |
| VOLTAGE_3_3V | 3 |  |
| VOLTAGE_3_3V_uC | 4 |  |
| VOLTAGE_REF_1V | 5 |  |
| VOLTAGE_REF_2V | 6 |  |
| VOLTAGE_REF_3V | 7 |  |

<a name="airspeed.proto"/>
<p align="right"><a href="#top">Top</a></p>

## airspeed.proto

<a name="BetCOM.Airspeed"/>
### Airspeed
Airspeed data struct measured with Pitot tube sensor from Eagletree

| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| time | [Time](#BetCOM.Time) | required | timestamp information |
| speed | [float](#float) | optional | airspeed in m/s |
| speed_raw | [uint32](#uint32) | optional | airspeed as raw value |

<a name="BetCOM.AirspeedConf"/>
### AirspeedConf


| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| basic_conf | [BasicConf](#BetCOM.BasicConf) | required | basic configuration of airspeed sensor |
| speed_offset | [float](#float) | optional | offset of airspeed measurements |


<a name="angle_of_attack.proto"/>
<p align="right"><a href="#top">Top</a></p>

## angle_of_attack.proto

<a name="BetCOM.AngleOfAttack"/>
### AngleOfAttack
Angle of Attack struct measured via rotational encoders

| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| time | [Time](#BetCOM.Time) | required |  |
| vertical | [float](#float) | optional | measured vertical angle in rad |
| horizontal | [float](#float) | optional | measured horizontal angle in rad |
| angle_of_attack | [float](#float) | optional | calculated angle of attack |
| vertical_raw | [uint32](#uint32) | optional | measured vertical angle raw value |
| horizontal_raw | [uint32](#uint32) | optional | measured horizontal angle raw value |

<a name="BetCOM.AngleOfAttackConf"/>
### AngleOfAttackConf
Angel of Attack configuration message

| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| basic_conf | [BasicConf](#BetCOM.BasicConf) | required | basic configuration of angle of attack |
| vertical_offset | [float](#float) | optional | offset of vertical angle measurements |
| horizontal_offset | [float](#float) | optional | offset of horizontal angle measurements |


<a name="baro.proto"/>
<p align="right"><a href="#top">Top</a></p>

## baro.proto

<a name="BetCOM.Barometer"/>
### Barometer
Barometer data message

| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| time | [Time](#BetCOM.Time) | required | timestamp information of message |
| pressure | [float](#float) | optional | atmospheric pressure in Pascal |
| altitude | [float](#float) | optional | altitude in meter |
| temperature | [float](#float) | optional | temperature in degree Celsius |

<a name="BetCOM.BarometerConf"/>
### BarometerConf
barometer configuration message

| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| basic_conf | [BasicConf](#BetCOM.BasicConf) | required | basic configuration of barometer |
| pressure_offset | [float](#float) | optional | offset of atmospheric pressure in Pascal |
| altitude_offset | [float](#float) | optional | offset of altitude in meter |
| temperature_offset | [float](#float) | optional | offset of temperature in degree Celsius |


<a name="betcall.proto"/>
<p align="right"><a href="#top">Top</a></p>

## betcall.proto

<a name="BetCALL.Sensors"/>
### Sensors
Wrapper message for sensor communication between Flicght Controller (FC)
and Groundstation (GS). Each sensor is defined in a sperate SENSORNAME.proto file
which is then included for this wrapper message.

| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| time | [Time](#BetCOM.Time) | required | Timestamp information |
| imu | [Imu](#BetCOM.Imu) | repeated | list of received IMU messages |
| airspeed | [Airspeed](#BetCOM.Airspeed) | optional | received Airspeed message |
| gps | [Gps](#BetCOM.Gps) | optional | received gps message |
| angle_of_attack | [AngleOfAttack](#BetCOM.AngleOfAttack) | optional | received angle of attack message |
| tube_angle | [TubeAngle](#BetCOM.TubeAngle) | optional | received tube angle sensor message |
| dms | [Dms](#BetCOM.Dms) | optional | received strain gage message |
| adc | [Adc](#BetCOM.Adc) | optional | received current and voltage measurements from ADC |
| barometer | [Barometer](#BetCOM.Barometer) | optional | received barometer message |
| barometer_arm | [Barometer](#BetCOM.Barometer) | optional | received barometer message |
| temperature | [Temperature](#BetCOM.Temperature) | optional | received temperature message |
| temperature_arm | [Temperature](#BetCOM.Temperature) | optional | received temperature message |
| humidity | [Humidity](#BetCOM.Humidity) | optional | received humidity message |


<a name="betconf.proto"/>
<p align="right"><a href="#top">Top</a></p>

## betconf.proto

<a name="BetCONF.Configuration"/>
### Configuration
Wrapper message for sensor configuration for Flicght Controller (FC)
from Groundstation (GS). Each configuration message is defined in a sperate SENSORNAME.proto file
which is then included for this wrapper message.

| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| gps | [GpsConf](#BetCOM.GpsConf) | optional | gps configuration message |
| airspeed | [AirspeedConf](#BetCOM.AirspeedConf) | optional | airspeed configuration message |
| angle_of_attack | [AngleOfAttackConf](#BetCOM.AngleOfAttackConf) | optional | angle of attack configuration message |
| line_angle | [LineAngleConf](#BetCOM.LineAngleConf) | optional | line angle sensor configuration message |
| tube_angle | [TubeAngleConf](#BetCOM.TubeAngleConf) | optional | tube angle configuration message |
| dms | [DmsConf](#BetCOM.DmsConf) | optional | dms configuration message |
| barometer | [BarometerConf](#BetCOM.BarometerConf) | optional | barometer configuration message |
| imu | [ImuConf](#BetCOM.ImuConf) | repeated | imu configuration messages |
| temperature | [TemperatureConf](#BetCOM.TemperatureConf) | optional | temperature configuration messages |


<a name="betpush.proto"/>
<p align="right"><a href="#top">Top</a></p>

## betpush.proto

<a name="BetPUSH.Actuators"/>
### Actuators


| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| flaps_right | [int32](#int32) | optional |  |
| flaps_left | [int32](#int32) | optional |  |
| ailerons_right | [int32](#int32) | optional |  |
| ailerons_left | [int32](#int32) | optional |  |
| rudder | [int32](#int32) | optional |  |
| elevator | [int32](#int32) | optional |  |
| ticks | [Ticks](#BetCOM.Ticks) | optional |  |


<a name="gps_piksi.proto"/>
<p align="right"><a href="#top">Top</a></p>

## gps_piksi.proto

<a name="BetCOM.Gps"/>
### Gps
GPS data struct for velocity or position measures from PIKSI
Definition close to SBP Protocola defined here: https://github.com/swift-nav/libsbp/raw/master/docs/sbp.pdf

| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| time | [Time](#BetCOM.Time) | required |  |
| gps_time | [GpsTime](#BetCOM.GpsTime) | optional |  |
| baseline | [GpsBaseline](#BetCOM.GpsBaseline) | optional |  |
| position | [GpsPosition](#BetCOM.GpsPosition) | optional |  |
| velocity | [GpsVelocity](#BetCOM.GpsVelocity) | optional |  |
| dilution_of_precision | [GpsDilutionOfPrecision](#BetCOM.GpsDilutionOfPrecision) | optional |  |

<a name="BetCOM.GpsBaseline"/>
### GpsBaseline
GPS Baseline message for differntial GPS estimates

| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| axial_system | [GpsAxialSystem](#BetCOM.GpsAxialSystem) | optional | Baseline estimates are in NED or ECEF axial system |
| time_of_week_ms | [uint32](#uint32) | optional | time of week in milliseconds |
| position | [XYZ_f](#BetCOM.XYZ_f) | required | XYZ baseline position |
| n_satellites | [uint32](#uint32) | optional | number of satellites |
| status | [GpsStatus](#BetCOM.GpsStatus) | optional | gps status information |
| h_accuracy | [float](#float) | optional | horizontal accuracy |
| v_accuracy | [float](#float) | optional | vertical accuracy |

<a name="BetCOM.GpsConf"/>
### GpsConf
GPS configuration message

| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| basic_conf | [BasicConf](#BetCOM.BasicConf) | required | basic configuration of gps piksi sensor |
| velocity_axial_system | [GpsAxialSystem](#BetCOM.GpsAxialSystem) | optional | axial system for velocity measurements |
| position_axial_system | [GpsAxialSystem](#BetCOM.GpsAxialSystem) | optional | axial system for position measurements |
| baseline_axial_system | [GpsAxialSystem](#BetCOM.GpsAxialSystem) | optional | axial system for baseline measurements |
| send_time | [bool](#bool) | optional | flag for sending gps time message |
| send_accuracy | [bool](#bool) | optional | flag for sending gps accuracy measurements |
| send_dilution_of_precision | [bool](#bool) | optional | flag for sending gps dilution of precision message |

<a name="BetCOM.GpsDilutionOfPrecision"/>
### GpsDilutionOfPrecision
GPS dilution of precision message

| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| time_of_week_ms | [uint32](#uint32) | required | time of week in milliseconds |
| geometric_dilution | [uint32](#uint32) | required | Geometric Dilution of Precision |
| position_dilution | [uint32](#uint32) | required | Position Dilution of Precision |
| time_dilution | [uint32](#uint32) | required | Time Dilution of Precision |
| horizontal_dilution | [uint32](#uint32) | required | Horizontal Dilution of Precision |
| vertical_dilution | [uint32](#uint32) | required | Vertical Dilution of Precision |

<a name="BetCOM.GpsPosition"/>
### GpsPosition
GPS position estimate

| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| axial_system | [GpsAxialSystem](#BetCOM.GpsAxialSystem) | required | Position can be in LLH or ECEF format |
| time_of_week_ms | [uint32](#uint32) | optional | time of week in milliseconds |
| position | [XYZ_f](#BetCOM.XYZ_f) | required | XYZ position in float |
| n_satellites | [uint32](#uint32) | optional | number of satellites |
| status | [GpsStatus](#BetCOM.GpsStatus) | optional | gps status information |
| h_accuracy | [float](#float) | optional | horizontal accuracy |
| v_accuracy | [float](#float) | optional | vertical accuracy |

<a name="BetCOM.GpsTime"/>
### GpsTime
High precision time message from GPS module

| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| weeks | [uint32](#uint32) | required | number of weeks |
| time_of_week | [uint32](#uint32) | required | time of week in milliseconds |
| ns | [sint32](#sint32) | required | Nanosecond residual of millisecond-rounded time_of_week (ranges from -500000 to 500000) |

<a name="BetCOM.GpsVelocity"/>
### GpsVelocity
GPS velocity message in corresponding axial system

| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| axial_system | [GpsAxialSystem](#BetCOM.GpsAxialSystem) | optional | velocity estimates are in NED or ECEF axial system |
| time_of_week | [uint32](#uint32) | optional | time of week in milliseconds |
| velocity | [XYZ_f](#BetCOM.XYZ_f) | required | XYZ velocities in m/s |
| n_satellites | [uint32](#uint32) | optional | number of satellites |
| status | [GpsStatus](#BetCOM.GpsStatus) | optional | gps status information |
| h_accuracy | [float](#float) | optional | horizontal accuracy |
| v_accuracy | [float](#float) | optional | vertical accuracy |


<a name="BetCOM.GpsAxialSystem"/>
### GpsAxialSystem
Messages from GPS support different axial systems

| Name | Number | Description |
| ---- | ------ | ----------- |
| NED | 1 | North East Down |
| ECEF | 2 | Earth Centered Rotational |
| LLH | 3 | Latitude Longitude Height |

<a name="BetCOM.GpsStatus"/>
### GpsStatus
Differential GPS RTK solution modi

| Name | Number | Description |
| ---- | ------ | ----------- |
| SPP | 0 | Single Point Positioning (SPP) |
| FLOAT_RTK | 1 | Floating more unprecise mode for RTK solutions |
| FIXED_RTK | 2 | Fixed precise mode for RTK solutions |

<a name="gps.proto"/>
<p align="right"><a href="#top">Top</a></p>

## gps.proto


<a name="humidity.proto"/>
<p align="right"><a href="#top">Top</a></p>

## humidity.proto

<a name="BetCOM.Humidity"/>
### Humidity
Message for humidity sensor

| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| time | [Time](#BetCOM.Time) | required | timestamp information |
| humidity | [float](#float) | optional | humidity in % |
| humidity_raw | [uint32](#uint32) | optional | humidity as raw value |
| temperature | [float](#float) | optional | temperature in degree Celsius |
| temperature_raw | [uint32](#uint32) | optional | temperature as raw value |

<a name="BetCOM.HumidityConf"/>
### HumidityConf
Configuration message for humidity sensor

| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| basic_conf | [BasicConf](#BetCOM.BasicConf) | required | general configuration of humidity sensor |


<a name="imu.proto"/>
<p align="right"><a href="#top">Top</a></p>

## imu.proto

<a name="BetCOM.Imu"/>
### Imu


| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| type | [ImuType](#BetCOM.ImuType) | required | type of IMU |
| time | [Time](#BetCOM.Time) | required | timestamp information |
| measurements | [ImuMeasurement](#BetCOM.ImuMeasurement) | repeated | array of measured data since measurement rat may be higher than sending rate |

<a name="BetCOM.ImuConf"/>
### ImuConf
IMU configuration message

| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| basic_conf | [BasicConf](#BetCOM.BasicConf) | required | basic configuration message |
| type | [ImuType](#BetCOM.ImuType) | required | to associate message with IMU |
| accel_offset | [XYZ_i](#BetCOM.XYZ_i) | optional | XYZ offset values for acceleration |
| gyro_offset | [XYZ_i](#BetCOM.XYZ_i) | optional | XYZ offset values for gyro |
| mag_offset | [XYZ_i](#BetCOM.XYZ_i) | optional | XYZ offset values for magnetometer |
| accel_range | [ImuConf.AccelRange](#BetCOM.ImuConf.AccelRange) | optional | Acceleration range |
| gyro_range | [ImuConf.GyroRange](#BetCOM.ImuConf.GyroRange) | optional | Gyroscope range |

<a name="BetCOM.ImuMeasurement"/>
### ImuMeasurement
Container for several data measured in one IMU

| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| time | [Time](#BetCOM.Time) | required |  |
| accel | [XYZ_f](#BetCOM.XYZ_f) | optional | data of accelerometer in SI units |
| gyro | [XYZ_f](#BetCOM.XYZ_f) | optional | data of gyroscope in SI units |
| mag | [XYZ_f](#BetCOM.XYZ_f) | optional | data of magnetometer in SI units |
| accel_raw | [XYZ_i](#BetCOM.XYZ_i) | optional | raw data of accelerometer |
| gyro_raw | [XYZ_i](#BetCOM.XYZ_i) | optional | raw data of gyroscope |
| mag_raw | [XYZ_i](#BetCOM.XYZ_i) | optional | raw data of magnetometer |
| quaternion | [XYZW_f](#BetCOM.XYZW_f) | optional | filtered quaternion output in SI |
| quaternion_raw | [XYZW_i](#BetCOM.XYZW_i) | optional | filtered quaternion output raw |
| linear_accel | [XYZ_f](#BetCOM.XYZ_f) | optional | filtered linear acceleration in SI |
| linear_accel_raw | [XYZ_i](#BetCOM.XYZ_i) | optional | filtered linear acceleration raw |
| gravity_vector | [XYZ_f](#BetCOM.XYZ_f) | optional | filtered gravity vector |
| gravity_vector_raw | [XYZ_i](#BetCOM.XYZ_i) | optional | filtered gravity vector raw |
| temperature | [float](#float) | optional | measured temperature in degree celsius |


<a name="BetCOM.ImuConf.AccelRange"/>
### ImuConf.AccelRange
Supported accelerometer ranges which influences precision

| Name | Number | Description |
| ---- | ------ | ----------- |
| ACCEL_2G | 1 |  |
| ACCEL_4G | 2 |  |
| ACCEL_8G | 3 |  |
| ACCEL_16G | 4 |  |

<a name="BetCOM.ImuConf.GyroRange"/>
### ImuConf.GyroRange
Supported gyroscope ranges in degree per second which influences precision

| Name | Number | Description |
| ---- | ------ | ----------- |
| GYRO_125DPS | 1 |  |
| GYRO_250DPS | 2 |  |
| GYRO_500DPS | 3 |  |
| GYRO_1000DPS | 4 |  |
| GYRO_2000DPS | 5 |  |

<a name="BetCOM.ImuType"/>
### ImuType
Since we have several IMUs we have different types

| Name | Number | Description |
| ---- | ------ | ----------- |
| FC_BOSCH_BNO055 | 0 | see https://www.bosch-sensortec.com/en/homepage/products_3/sensor_hubs/iot_solutions/bno055_1/bno055_4 |
| FC_XSENS_MTI | 1 | see https://www.xsens.com/products/mti-1-series/ |
| FC_INVENSENS_MPU9250 | 2 | see http://www.invensense.com/products/motion-tracking/9-axis/mpu-9250/ |
| FC_MAX_21100 | 3 | see https://www.maximintegrated.com/en/products/analog/sensors-and-sensor-interface/MAX21100.html |
| ARM_INVENSENS_MPU9250 | 4 | same as FC_INVENSENS_MPU9250 but located on arm of Carousel |

<a name="led.proto"/>
<p align="right"><a href="#top">Top</a></p>

## led.proto

<a name="BetCOM.Led"/>
### Led
LED message for simple on/off LEDs

| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| location | [Led.Location](#BetCOM.Led.Location) | required | location of LED |
| on | [bool](#bool) | optional | value of LED |
| mode | [LedMode](#BetCOM.LedMode) | optional | mode of LED |

<a name="BetCOM.LedBrightness"/>
### LedBrightness
LED on FC which allows to adjust brightness

| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| brightness | [uint32](#uint32) | optional | brightness value between 0..255 |
| mode | [LedMode](#BetCOM.LedMode) | optional | mode of LED |

<a name="BetCOM.LedMode"/>
### LedMode
Allows to set frequency and duty cycle of LED

| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| frequency | [float](#float) | optional | frequency of blinking default: static |
| duty_cycle | [float](#float) | optional | duty cycle of on-phase default: 0.5 |

<a name="BetCOM.LedRGB"/>
### LedRGB
RGB-LED on FC

| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| mode | [LedMode](#BetCOM.LedMode) | optional | mode of LED |
| r_value | [uint32](#uint32) | optional | r value between 0..255 |
| g_value | [uint32](#uint32) | optional | g value between 0..255 |
| b_value | [uint32](#uint32) | optional | b value between 0..255 |

<a name="BetCOM.Leds"/>
### Leds
wrapper message for all LEDs on FC

| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| time | [Time](#BetCOM.Time) | optional | optional timing information |
| leds | [Led](#BetCOM.Led) | repeated | settings array for simple LED values |
| ledBrightness | [LedBrightness](#BetCOM.LedBrightness) | optional | settings for LED with birghtness adjustment |
| ledRGB | [LedRGB](#BetCOM.LedRGB) | optional | ssettings for RGB LED |


<a name="BetCOM.Led.Location"/>
### Led.Location
Psooible  locations of LED on FC

| Name | Number | Description |
| ---- | ------ | ----------- |
| uC_1 | 1 |  |
| uC_2 | 2 |  |
| uC_3 | 3 |  |

<a name="line_angle.proto"/>
<p align="right"><a href="#top">Top</a></p>

## line_angle.proto

<a name="BetCOM.LineAngle"/>
### LineAngle
Line angle sensor data struct

| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| time | [Time](#BetCOM.Time) | required |  |
| azimuth | [float](#float) | required | azimuth angle in rad |
| elevation | [float](#float) | required | elevation angle in rad |
| azimuth_raw | [uint32](#uint32) | optional | azimuth angle as raw value |
| elevation_raw | [uint32](#uint32) | optional | elevation angle as raw value |
| azimuth_valid | [bool](#bool) | required | error flag of azimuth sensor |
| elevation_valid | [bool](#bool) | required | error flag of elevation sensor |

<a name="BetCOM.LineAngleConf"/>
### LineAngleConf
Configuration message for Line Anle sensor

| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| basic_conf | [BasicConf](#BetCOM.BasicConf) | required |  |
| azimuth_offset | [float](#float) | required | azimuth offset in rad |
| elevation_offset | [float](#float) | required | rotation offset in rad |


<a name="strain.proto"/>
<p align="right"><a href="#top">Top</a></p>

## strain.proto

<a name="BetCOM.Dms"/>
### Dms
Strain gauge data struct

| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| time | [Time](#BetCOM.Time) | required |  |
| strain | [float](#float) | optional | current strain in Newton |
| strain_raw | [uint32](#uint32) | optional | current strain as raw value |

<a name="BetCOM.DmsConf"/>
### DmsConf
Configuration message for strain gauge

| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| basic_conf | [BasicConf](#BetCOM.BasicConf) | required | basic configuration message for starin gauge |
| strain_offset | [float](#float) | required | offset of measured strain in N |


<a name="temperature.proto"/>
<p align="right"><a href="#top">Top</a></p>

## temperature.proto

<a name="BetCOM.Temperature"/>
### Temperature
Temperature data struct

| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| time | [Time](#BetCOM.Time) | required | timestamp information |
| temperature | [float](#float) | optional | temperature in degree Celsius |
| temperature_raw | [uint32](#uint32) | optional | temperature as raw value |

<a name="BetCOM.TemperatureConf"/>
### TemperatureConf


| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| basic_conf | [BasicConf](#BetCOM.BasicConf) | required | basic configuration of temperature sensor |


<a name="tube_angle.proto"/>
<p align="right"><a href="#top">Top</a></p>

## tube_angle.proto

<a name="BetCOM.TubeAngle"/>
### TubeAngle
Tube angle sensor data struct

| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| time | [Time](#BetCOM.Time) | required |  |
| rotation | [float](#float) | optional | rotation angle in rad |
| elevation | [float](#float) | optional | elevation angle in rad |
| rotation_raw | [uint32](#uint32) | optional | rotation angle as raw value |
| elevation_raw | [uint32](#uint32) | optional | elevation angle as raw value |
| rotation_valid | [bool](#bool) | required | error flag of rotation sensor |
| elevation_valid | [bool](#bool) | required | error flag of elevation sensor |

<a name="BetCOM.TubeAngleConf"/>
### TubeAngleConf
Tube angle sensor configuration message

| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| basic_conf | [BasicConf](#BetCOM.BasicConf) | required | basic configuration of tube angle sensor |
| rotation_offset | [float](#float) | optional | rotation offset in rad |
| elevation_offset | [float](#float) | optional | rotation offset in rad |


<a name="types.proto"/>
<p align="right"><a href="#top">Top</a></p>

## types.proto

<a name="BetCOM.BasicConf"/>
### BasicConf
Basic Configuration message for sensor

| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| activated | [bool](#bool) | required | Sensor is active ? |
| raw | [bool](#bool) | required | Sent raw values ? |
| rate | [uint32](#uint32) | required | Measurement rate. Maximum depends on sensor. |

<a name="BetCOM.Ticks"/>
### Ticks
Ticks for precise relative time measurement in system

| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| onepps_counter | [uint32](#uint32) | required | Number of 1PPS pulses received |
| ticks | [uint32](#uint32) | required | Number of 125 ns ticks since last 1PPS pulse |

<a name="BetCOM.Time"/>
### Time
Timestamp information wrapper message

| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| ticks_measured | [Ticks](#BetCOM.Ticks) | optional | Ticks when measurement was achieved |
| ticks_sent | [Ticks](#BetCOM.Ticks) | optional | Ticks when measurement was sent to next station |
| ticks_received | [Ticks](#BetCOM.Ticks) | optional | Ticks when message was received on next station |
| timestamp_measured | [Timestamp](#BetCOM.Timestamp) | optional | Placeholder for converted ticks_measured |
| timestamp_sent | [Timestamp](#BetCOM.Timestamp) | optional | Placeholder for converted ticks_sent |
| timestamp_received | [Timestamp](#BetCOM.Timestamp) | optional | Placeholder for converted ticks_received |

<a name="BetCOM.Timestamp"/>
### Timestamp
Timestamp format on BeagleBone or other Linux machines

| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| tsec | [uint64](#uint64) | required | seconds since 1970 (UTC) |
| tnsec | [uint64](#uint64) | required | nanoseconds from clock |

<a name="BetCOM.XYZW_f"/>
### XYZW_f
Quaternion datatype float

| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| x | [float](#float) | required | x value unit depends on message |
| y | [float](#float) | required | y value unit depends on message |
| z | [float](#float) | required | z value unit depends on message |
| w | [float](#float) | required | w value unit depends on message |

<a name="BetCOM.XYZW_i"/>
### XYZW_i
Quaternion datatype integer

| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| x | [sint32](#sint32) | required | x value unit depends on message |
| y | [sint32](#sint32) | required | y value unit depends on message |
| z | [sint32](#sint32) | required | z value unit depends on message |
| w | [sint32](#sint32) | required | w value unit depends on message |

<a name="BetCOM.XYZ_f"/>
### XYZ_f
XYZ data of datatype float

| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| x | [float](#float) | required | x value unit depends on message |
| y | [float](#float) | required | y value unit depends on message |
| z | [float](#float) | required | z value unit depends on message |

<a name="BetCOM.XYZ_i"/>
### XYZ_i
XYZ data of datatype signed integer

| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| x | [sint32](#sint32) | required | x value unit depends on message |
| y | [sint32](#sint32) | required | y value unit depends on message |
| z | [sint32](#sint32) | required | z value unit depends on message |


<a name="BetCOM.Globals"/>
### Globals
Constants which are accessible via generated Headers

| Name | Number | Description |
| ---- | ------ | ----------- |
| MAX_MESSAGE_SIZE | 1024 |  |


<a name="scalar-value-types"/>
## Scalar Value Types

| .proto Type | Notes | C++ Type | Java Type | Python Type |
| ----------- | ----- | -------- | --------- | ----------- |
| <a name="double"/> double |  | double | double | float |
| <a name="float"/> float |  | float | float | float |
| <a name="int32"/> int32 | Uses variable-length encoding. Inefficient for encoding negative numbers – if your field is likely to have negative values, use sint32 instead. | int32 | int | int |
| <a name="int64"/> int64 | Uses variable-length encoding. Inefficient for encoding negative numbers – if your field is likely to have negative values, use sint64 instead. | int64 | long | int/long |
| <a name="uint32"/> uint32 | Uses variable-length encoding. | uint32 | int | int/long |
| <a name="uint64"/> uint64 | Uses variable-length encoding. | uint64 | long | int/long |
| <a name="sint32"/> sint32 | Uses variable-length encoding. Signed int value. These more efficiently encode negative numbers than regular int32s. | int32 | int | int |
| <a name="sint64"/> sint64 | Uses variable-length encoding. Signed int value. These more efficiently encode negative numbers than regular int64s. | int64 | long | int/long |
| <a name="fixed32"/> fixed32 | Always four bytes. More efficient than uint32 if values are often greater than 2^28. | uint32 | int | int |
| <a name="fixed64"/> fixed64 | Always eight bytes. More efficient than uint64 if values are often greater than 2^56. | uint64 | long | int/long |
| <a name="sfixed32"/> sfixed32 | Always four bytes. | int32 | int | int |
| <a name="sfixed64"/> sfixed64 | Always eight bytes. | int64 | long | int/long |
| <a name="bool"/> bool |  | bool | boolean | boolean |
| <a name="string"/> string | A string must always contain UTF-8 encoded or 7-bit ASCII text. | string | String | str/unicode |
| <a name="bytes"/> bytes | May contain any arbitrary sequence of bytes. | string | ByteString | str |
