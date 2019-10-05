Curl and Wget
=================

:date: 2015-06-01
:summary: Downloading things from the internet via the command line

wget
------

::

	wget "https://somewhere/file"

curl
---------

This cheatsheet came from `mixu <http://blog.mixu.net/2013/01/29/curl-cheatsheet/>`__.

Simple GET request ::

	curl -k "https://localhost/foo?bar=baz&amp;abc=def"

JSON POST or PUT request ::

	curl -k -H "Content-Type: application/json" -X POST -d '{"accountName":"test","value":"hello"}' https://localhost/foo

or ::

	curl -X "PUT"


POST a file ::

	curl ... --data-binary @filename

Fake a /etc/hosts entry and a Host: header with curl ::

	curl -vvv --resolve 'book.mixu.net:80:123.145.167.189' http://book.mixu.net/

Make a request with basic auth enabled ::

	curl -vvv -u name@foo.com:password http://www.example.com

or ::

	curl --user name:password http://www.example.com

Set the Referer header ::

	curl -e http://curl.haxx.se daniel.haxx.se

Set the User Agent header ::

	curl -A "Mozilla/4.73"

or ::

	curl --user-agent "Mozilla".

Set Cookies ::

	curl -b "name=Daniel"

or ::

	curl --cookie "name=Daniel"

Time a request (connect time + time to first byte + total tile) ::

	curl -o /dev/null -w "Connect: %{time_connect} TTFB: %{time_starttransfer} Total time: %{time_total} \n" http://google.com

Downloading files from Github ::

	curl -O  https://raw.github.com/username/reponame/master/filename
