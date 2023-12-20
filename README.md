# MT6701-Arduino-Library
<p align="center"><img src="/images/mt6701_module.jpg"></p>

## Similar sensors
* [AS5600](https://github.com/S-LABc/AMS-AS5600-Arduino-Library)
* [AS5601](https://github.com/S-LABc/AMS-AS5601-Arduino-Library)

## Warning
* The library has not been fully tested. Errors may occur. By using the library you take all risks upon yourself.
* Not all sensor capabilities are implemented. You can use these methods to interact with sensor registers:
```C++
// Reading the contents of a register
uint8_t MT_RequestSingleRegister(uint8_t _reg_addr);
// Writing new contents to the register
void MT_WriteOneByte(uint8_t _reg_addr, uint8_t _payload);
```

* Use the methods described above through inheritance of the MT6701I2C class
* All available methods, constants, data types can be viewed [here](https://github.com/S-LABc/MT6701-Arduino-Library/blob/main/src/MT6701_I2C.h)

## Links
* [MT6701 datasheet](http://www.magntek.com.cn/upload/MT6701_Rev.1.8.pdf)
* [MT6701 Page](http://www.magntek.com.cn/en/list/177/559.htm)
* [Case for tests](https://github.com/S-LABc/AMS-AS5600-Arduino-Library/tree/main/addons/AS5600-Case-STL)
