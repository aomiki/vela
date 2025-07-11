# Vela

Program for flight computer of the Vela model rocket.

The program runs on STM32 "black pill" at the CPU frequency of 72 MHz.

Implemented peripheral (resides mainly in [vela-modules](https://github.com/aomiki/vela-modules)):

* Radio (sending telemetry to the [ground station](https://github.com/aomiki/vela-telemetry-decoder))
* SD-card (logging)
* Barometer (data)
* Accelerometer (data)
* Servo motor (to open rescue system)
* Buzzer (to make sound when rocket has landed, so it's easier to find)

---

Vela - constellation in the southern sky.
Its name is Latin for the sails of a ship.

``` sh
               /|___
             ///|   ))
           /////|   )))
         ///////|    )))
       /////////|     )))
     ///////////|     ))))
   /////////////|     )))
  //////////////|    )))
////////////////|___)))
  ______________|________
  \                    /
~~~~~~~~~~~~~~~~~~~~~~~~~~~
```
