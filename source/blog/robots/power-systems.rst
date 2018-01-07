Power Systems
==============

:date: 2016-04-17
:summary: Power system design notes

Preventing Reverse Polarity
----------------------------

The easiest way to prevent a battery from getting hooked up backwards in your
robot is to key your power connectors to they can only be installed one way.
However, if people are making their own, sometimes this isn't good enough. A better
way is to design your circuit so it prevents this.

Diodes
-------

.. image:: {filename}/blog/robots/pics/reverse_polarity_protection.jpg


Using a diode is one easy way to prevent current in your system from flowing in
the wrong direction. However, they have a forward voltage drop which both reduces
the amount of available battery voltage and wastes power.

============== ============================================================== ========
Part           Voltage Drop at 2A                                             Power at 2A
============== ============================================================== ========
Diode (1N5400)  0.85V                                                         1.7W
Schottky Diode  0.55V                                                         1.1W
PNP MOSFET      R_DS(ON)=26 mOhm to 2A * .026 ohm = 52 mV                     1.04mW
============== ============================================================== ========

Using a PNP MOSFET is better because it lowers your battery voltage less and
wastes less power.

This `video <https://www.youtube.com/watch?v=IrB-FPcv1Dc>`_ does a great job of explaining
this a little more. It also explains why you *might* have to include a resistor and zener
diode to clamp the voltage, depending on the MOSFET you select.


.. image:: {filename}/blog/robots/pics/reverse_polarity_protection_2.png