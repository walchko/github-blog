Text To Speach (TTS)
=====================

:date: 2016-03-25
:summary: text to speech using the RPi

Google
-------

I used to use Google TTS, but they changed their interface so I couldn't use
it anymore ... so here are some other options.

Linux
------

There are several linux packages that convert text to speech:

* say converts text to audible speech using the GNUstep speech engine::

    sudo apt-get install gnustep-gui-common
    say "hello"

* festival General multi-lingual speech synthesis system::

    sudo apt-get install festival
    echo "hello" | festival --tts

* spd-say sends text-to-speech output request to speech-dispatcher::

     sudo apt-get install speech-dispatcher
     spd-say "hello"

* espeak is a multi-lingual software speech synthesizer::

     sudo apt-get install espeak
     espeak "hello"

``say`` - macOS
-----------------

Mac has a builtin program called ``say`` which works very nicely::

  NAME
         say - Convert text to audible speech

  SYNOPSIS
             say [-v voice] [-r rate] [-o outfile [audio format options] | -n name:port | -a device] [-f file | string ...]

  DESCRIPTION
         This tool uses the Speech Synthesis manager to convert input text to
         audible speech and either play it through the sound output device
         chosen in System Preferences or save it to an AIFF file.

``spd-say``
---------------

Unlike ``say`` above, you can play with the pitch.

::

	pi@zoidberg ~ $ spd-say -h
	Usage: spd-say [options] "some text"
	Speech Dispatcher Say -- a simple client for speech synthesis (GNU GPL)

	-r, --rate			Set the rate of the speech
					(between -100 and +100, default: 0)
	-p, --pitch			Set the pitch of the speech
					(between -100 and +100, default: 0)
	-i, --volume			Set the volume (intensity) of the speech
					(between -100 and +100, default: 0)
	-o, --output-module		Set the output module
	-O, --list-output-modules	Get the list of output modules
	-I, --sound-icon		Play the sound icon
	-l, --language			Set the language (ISO code)
	-t, --voice-type		Set the preferred voice type
					(male1, male2, male3, female1, female2
					female3, child_male, child_female)
	-L, --list-synthesis-voices	Get the list of synthesis voices
	-y, --synthesis-voice		Set the synthesis voice
	-m, --punctuation-mode		Set the punctuation mode (none, some, all)
	-s, --spelling			Spell the message
	-x, --ssml			Set SSML mode on (default: off)

	-e, --pipe-mode			Pipe from stdin to stdout plus Speech Dispatcher
	-P, --priority			Set priority of the message (important, message,
					text, notification, progress;default: text)
	-N, --application-name		Set the application name used to establish
					the connection to specified string value
					(default: spd-say)
	-n, --connection-name		Set the connection name used to establish
					the connection to specified string value
					(default: main)

	-w, --wait			Wait till the message is spoken or discarded
	-S, --stop			Stop speaking the message being spoken
	-C, --cancel			Cancel all messages

	-v, --version			Print version and copyright info
	-h, --help			Print this info

	Copyright (C) 2002-2012 Brailcom, o.p.s.
	This is free software; you can redistribute it and/or modify it
	under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2, or (at your option)
	any later version. Please see COPYING for more details.

	Please report bugs to speechd@lists.freebsoft.org
