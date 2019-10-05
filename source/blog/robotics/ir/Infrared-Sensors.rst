Infrared Sensors
=====================

:date: 2017-01-02
:summary: Infrared range sensors

Infrared (IR)
---------------


.. figure:: ir_sensors.jpg
    :align: center
    :width: 400px


A common range sensor is the Sharp IR sensors which almost every electronics source sells.
These sensors output a beam of modulated light and look for its return. Based on the time
of flight, an analog voltage is returned (see below).

.. figure:: ir_range_curve.png
    :align: center

Notice that, although the sensor takes 5V in, it outputs no more than 2.8V. There for this
sensor is safe to work with common 3.3V systems.

# References

---

- `Sharp sensor datasheet <datasheet.pdf>`_
- `wikipedia: Infrared <https://en.wikipedia.org/wiki/Infrared>`_
