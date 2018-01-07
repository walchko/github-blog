I2S - Digital Audio
=====================

:date: 2016-07-26
:modified: 2017-06-04
:summary: Setting up an RPi3/Raspbian-Jessie with digital audio and avoiding the noise from the Pi's analog output

.. figure:: https://cdn-learn.adafruit.com/assets/assets/000/032/618/medium800/adafruit_products_3006_kit_ORIG.jpg?1464029419
  :align: center

Setup
---------

Setting up I2S audio isn't too hard. I used a `Adafruit i2s 3W amp (MAX98357A) <https://www.adafruit.com/products/3006>`_, to
hook it up to my rpi3. Also the ``i2s-mmap`` seems to help with any popping noise.

Edit ``/boot/config.txt``::

  # dtparam=audio=off
  dtparam=i2s=on
  dtoverlay=hifhiberry-dac
  dtoverlay=i2s-mmap

Create/edit ``/etc/asound.conf`` to setup default audio::

  pcm.hifiberry {
    type hw card 0
  }

  pcm.!default {
    type plug
    slave.pcm "dmixer"
  }

  pcm.dmixer {
    type dmix
    ipc_key 1024
    slave {
      pcm "hifiberry"
      channels 2
    }
  }

  ctl.dmixer {
    type hw
    card 0
  }

Now you have to reboot so the system gets setup correctly (remember, these are boot parameter settings).

Now connect the amp to the rpi where Vin is 5V:

===== ====== ======
      BCM    Pi
===== ====== ======
DIN   21     40
BCLK  18     12
LRCLK 19     35
===== ====== ======

.. figure:: https://cdn-learn.adafruit.com/assets/assets/000/032/643/medium800/adafruit_products_3006_top_demo_ORIG.jpg?1464037283
  :align: center


Now, lots of instructions on the internet say to disable i2c, but you don't have too. The bottom part of my
``/boot/config.txt`` looks like this::

  # Enable audio (loads snd_bcm2835)
  #dtparam=audio=off
  start_x=1
  gpu_mem=16
  dtparam=i2c_arm=on
  dtparam=i2s=on
  enable_uart=0
  dtoverlay=hifiberry-dac
  dtoverlay=i2s-mmap

As you can see, I leave i2c on. Note the ``enable_uart=0`` is so I can use bluetooth. Apparently, the BT module is tied to
the hardware serial port and the don't bother to tell people. You have to spend a lot of time to dig it up.

Jack Server
~~~~~~~~~~~~~~

Not sure you need this::

  sudo apt-get --no-install-recommends install jackd2
  jackd -d alsa

I don't seem to use it.

Issues
~~~~~~~~~

I still had lots of issues. installing ``sudo apt-get install speech-dispatcher`` seemed to help.
I now had pulse audio and `alsamixer` working.

If you get errors about unknown cards::

  ALSA lib pcm.c:2239:(snd_pcm_open_noupdate) Unknown PCM cards.pcm.rear
  ALSA lib pcm.c:2239:(snd_pcm_open_noupdate) Unknown PCM cards.pcm.center_lfe
  ALSA lib pcm.c:2239:(snd_pcm_open_noupdate) Unknown PCM cards.pcm.side

Edit ``/usr/share/alsa/alsa.conf`` and change the cards from ``pcm.front cards.pcm.front`` to
``pcm.front cards.pcm.default``.

Test
-----

`Sox <http://sox.sourceforge.net/>`_ has some useful tools::

  sudo apt-get install sox libsox2

Other useful ones might be: ``libsox-fmt-all`` and ``libsox-dev``.

You can list the devices ALSA see with::

  pi@robot ~ $ aplay -l
  **** List of PLAYBACK Hardware Devices ****
  card 0: sndrpihifiberry [snd_rpi_hifiberry_dac], device 0: HifiBerry DAC HiFi pcm5102a-hifi-0 []
    Subdevices: 1/1
    Subdevice #0: subdevice #0

So we see our I2S amp (snd_rpi_hifiberry_dac) listed there, so we are ready to test it:

1. Random static: ``speaker-test -c2``
2. Wave file: ``speaker-test -c2 --test=wav -w /usr/share/sounds/alsa/Front_Center.wav``
3. Tone: ``play -n synth sin 1000 gain 1``
4. Audio test::

      wget https://cdn.shopify.com/s/files/1/0062/6682/files/sample.wav
      aplay sample.wav

5. Text to speach::

      sudo apt-get install espeak
      espeak "hello world"
      espeak “Hello World!” > /dev/null

Check File Types
~~~~~~~~~~~~~~~~~~~~

Also, you can see what the file is::

  pi@r2d2 tmp $ file sample.wav
  sample.wav: RIFF (little-endian) data, WAVE audio, Microsoft PCM, 8 bit, mono 11025 Hz

Or use `sox`::

  pi@r2d2 tmp $ soxi sample.wav

  Input File     : 'sample.wav'
  Channels       : 1
  Sample Rate    : 11025
  Precision      : 8-bit
  Duration       : 00:00:04.06 = 44800 samples ~ 304.762 CDDA sectors
  File Size      : 44.8k
  Bit Rate       : 88.3k
  Sample Encoding: 8-bit Unsigned Integer PCM

Alsa Mixer
-------------

.. figure:: {filename}/blog/raspbian/pics/alsamixer.png
	:align: center

To see what you have access to::

  pi@r2d2 ~ $ amixer
  Simple mixer control 'Master',0
    Capabilities: pvolume pswitch pswitch-joined
    Playback channels: Front Left - Front Right
    Limits: Playback 0 - 65536
    Mono:
    Front Left: Playback 39344 [60%] [on]
    Front Right: Playback 39344 [60%] [on]
  Simple mixer control 'Capture',0
    Capabilities: cvolume cswitch cswitch-joined
    Capture channels: Front Left - Front Right
    Limits: Capture 0 - 65536
    Front Left: Capture 65536 [100%] [on]
    Front Right: Capture 65536 [100%] [on]

  pi@r2d2 ~ $ amixer controls
  numid=4,iface=MIXER,name='Master Playback Switch'
  numid=3,iface=MIXER,name='Master Playback Volume'
  numid=2,iface=MIXER,name='Capture Switch'
  numid=1,iface=MIXER,name='Capture Volume'

Now to make some changes:

* Get the current value: ``amixer cget numid=3``
* Set the current value: ``amixer cset numid=3 50%``
* Mute all sound (switch Master): ``amixer cset numid=4 off``
* Save changes to ``/var/lib/alsa/asound.state``: ``sudo alsactl store``
* Reset system if you f@$k up: ``sudo /etc/init.d/alsa-utils reset``

References
-----------

* `Adafruit tutorial 1 <https://learn.adafruit.com/adafruit-max98357-i2s-class-d-mono-amp?view=all>`_
* `Adafruit tutorial 2 <https://learn.adafruit.com/raspberry-pi-zero-npr-one-radio?view=all>`_
* `Raspberry Pi Forum discussion <https://www.raspberrypi.org/forums/viewtopic.php?t=97314>`_
* `pimoroni <http://learn.pimoroni.com/tutorial/phat/raspberry-pi-phat-dac-install>`_
* `Raspberry Pi pinout <http://pinout.xyz/>`_
* `Alsa mixer command line <http://blog.scphillips.com/posts/2013/01/sound-configuration-on-raspberry-pi-with-alsa/>`_
* `Fixing alsa issues with espeak <https://www.raspberrypi.org/forums/viewtopic.php?f=28&t=136974#>`_
