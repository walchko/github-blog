---
title: Ascii Art
date: 2017-01-07
---

You can create ascii art from jpegs or text with the programs. The can be useful
if you want some ascii art on your login prompt or whatever.

```
jp2a --background=light -i --output='art.txt' <some_file.jpg>
figlet 'hello world'
```

Install
--------

Use ``brew`` to install:
```
brew install jp2a figlet
```

## `figlet`

```
kevin@Logan ~ $ figlet hello world
 _          _ _                            _     _ 
| |__   ___| | | ___   __      _____  _ __| | __| |
| '_ \ / _ \ | |/ _ \  \ \ /\ / / _ \| '__| |/ _` |
| | | |  __/ | | (_) |  \ V  V / (_) | |  | | (_| |
|_| |_|\___|_|_|\___/    \_/\_/ \___/|_|  |_|\__,_|
```                                                   


## `jp2a`

Using:
	
```
kevin@Logan github $ jp2a --help
jp2a 1.0.6
Copyright (C) 2006 Christian Stigen Larsen
Distributed under the GNU General Public License (GPL) v2.

Usage: jp2a [ options ] [ file(s) | URL(s) ]

Convert files or URLs from JPEG format to ASCII.

OPTIONS
  -                 Read images from standard input.
      --blue=N.N    Set RGB to grayscale conversion weight, default is 0.1145
  -b, --border      Print a border around the output image.
      --chars=...   Select character palette used to paint the image.
		    Leftmost character corresponds to black pixel, right-
		    most to white.  Minimum two characters must be specified.
      --clear       Clears screen before drawing each output image.
      --colors      Use ANSI colors in output.
  -d, --debug       Print additional debug information.
      --fill        When used with --color and/or --html, color each character's
		    background color.
  -x, --flipx       Flip image in X direction.
  -y, --flipy       Flip image in Y direction.
  -f, --term-fit    Use the largest image dimension that fits in your terminal
		    display with correct aspect ratio.
      --term-height Use terminal display height.
      --term-width  Use terminal display width.
  -z, --term-zoom   Use terminal display dimension for output.
      --grayscale   Convert image to grayscale when using --html or --colors
      --green=N.N   Set RGB to grayscale conversion weight, default is 0.5866
      --height=N    Set output height, calculate width from aspect ratio.
  -h, --help        Print program help.
      --html        Produce strict XHTML 1.0 output.
      --html-fill   Same as --fill (will be phased out)
      --html-fontsize=N   Set fontsize to N pt, default is 4.
      --html-no-bold      Do not use bold characters with HTML output
      --html-raw    Output raw HTML codes, i.e. without the <head> section etc.
      --html-title=...  Set HTML output title
  -i, --invert      Invert output image.  Use if your display has a dark
		    background.
      --background=dark   These are just mnemonics whether to use --invert
      --background=light  or not.  If your console has light characters on
		    a dark background, use --background=dark.
      --output=...  Write output to file.
      --red=N.N     Set RGB to grayscale conversion weight, default 0.2989f.
      --size=WxH    Set output width and height.
  -v, --verbose     Verbose output.
  -V, --version     Print program version.
      --width=N     Set output width, calculate height from ratio.

  The default mode is `jp2a --term-fit --background=dark'.
  See the man-page for jp2a for more detailed help text.

Project homepage on http://jp2a.sf.net
Report bugs to <csl@sublevel3.org>
```

There is a lot you can do with it. Example printed to terminal:

```
kevin@Logan pics $ jp2a --background=light  linux.jpg
				    .cx0XWMNX0d;
				 'o0WMMMMMMMMMWWXx:.
			       .kWMMMMMMMMMMMMXxOXWNx'
			      'NMMMMMMMMMMMMMMXKXNWMMW;
			      kMMMMMMMMMMMMMMMMMMMMMMMN,
			     .XMMMMWWMMMMMMMMMWNNWMMMMMN.
			     ,MMW0OOKXMMMMNOdloxOKWMMMMMo
			     ,MWc.. 'OMMMX,  .. .kWMMMMMO
			     ,M0'Kkl ,MNNc .0Wk; .XMMMMMO
			     .NX:NNK;,Okkc cMMW0  0MMMMMk
			     .KMo;Ol;,..',',lxx, lMMMMMM0
			      0WO:'.............'dWMMMMMW:
			      kK:............',,'cWMMWWMMK.
			      oWx:,'....'',,,'',;dWMMKkOXMd
			      dMd:lc;,'','',;,....xWMWKkOWW:
			     ,NMl.':lcc:::;'.      dMMMMMMMNc
			    oWWk.  .';;,..          lWMMMMMMWx.
			  ;XMN:       .              oMMMMMMMMN:
			 oWMN;                       .XMMMMMMMMWx.
		       ,KMMWd.                   .... ;WMMMMMMMMMX,
		      lWMMMOc,.                ..',;;;.;NMWNWMMMMMWo
		     lMWWM0'                        ..,'cWMWNNWMMMMMx
		    ;NWNMX.                            ..:NMWNWWMMMMMO.
		   ,NNNW0.                                lWNNWNXMMMMMk
		   0WNM0.                                 .NMMMMKNMMMMMx
		  oMWWM:                                   kMMMMNNMMMMMM:
		 dWNNM0            .                       oMMMMWWMMMMMMO
	       .kMMXXMc            .                       lMMMMXWMMMMMMW.
	       dMMMNKN,            .                       lMMMNXMMMMMMMM'
	       cN0OKXX;           ..                       lWNNXWWNNNWMMK
	       .'...:xO:.          .                     ..dNWMMMMMMX0NX;
	     .........;0Nd.                            ....cWMMMMMMMMK;..
	   .',.........'dNNk,                         .;..'l0WMMMMWXx,...
     ....',,'............cNMMKl.                     .;c'.';cokOOko:'....
     '....................:KMMMWk.                   .;l,...',,;,,'........
     ''....................'kWMMM:                   .lo:'...................
     .'.....................'o0dl.                  .xKo:'......................
     .'.......................,,.                .:kNM0l;.......................
     ..........................'l;.        ..,cxKWMMMWxc,.....................
     ..........................':OWX0OOO0KXWMMMMMMMMMNo:,...............'...
     .';;;,,,,'''.............',:xWMMMMMMMMMMMMMMMMMMXo:,'.........',;,'..
      ..',,;::cccc:;,,''''''';:clkKOxxxddddoooooddxkOOol:;,,,,,,;:c:;'.....
       .......'',,;:cccccccccloc;'..............''',,,coollccclll:'.......
	 ..........'''',;::::;'........   ...........'',;:::::;,........
	      ...................               ...................
```
