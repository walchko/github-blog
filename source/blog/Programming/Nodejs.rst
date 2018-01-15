Programming Node.js
======================

:date: 2015-12-13
:modified: 2017-05-21
:summary: Node.js/javascript snipets and useful libraries

Libraries
------------

- `Moments.js <http://momentjs.com/>`_ : time/date manipulation (16.6k)
- `Chart.js <http://www.chartjs.org/>`_ : scatter, pie, polar, bar, etc charts
- `Purecss <https://purecss.io/>`_ : A set of small, responsive CSS modules that you can use in every web project. Maintained by yahoo.
- `FortAwesome <https://github.com/FortAwesome/Font-Awesome>`_ :
- `Nunjucks [not that great] <https://mozilla.github.io/nunjucks/>`_ : a templating engine very similar to python `jinja2 <http://jinja.pocoo.org/>`_ (8k gzipped)

Using the http module
-----------------------

HTTP is one of the pillars behind the world wide web. HTTP describes the
transfer of state between client an server. With HTTP, an embedded device can
answer requests from other places in a network, or it can itself send updates or
fetch instructions from a server.

In Node.js, the simplest way to turn a device into a network application is
with http. This module is part of the Node.js core library, and you find a nice
overview here .

Let’s start by creating a web server locally on your laptop. We then transfer
the project later to an embedded device. We must first install the http module::

  $ npm install --save http

Then, we setup a file server.js:

.. code-block:: javascript

  var http = require('http');
  var server = http.createServer(function (req, res) {
      res.writeHead(200, {'Content-Type': 'text/html'});
      res.end('hello\n');
  });
  // start server at a port
  var port = 3000;
  server.listen(port);
  console.log('Starting server at port: ' + port);

The first thing we do is creating a command createServer. Often it is a good
idea to add some HTTP headers to inform the client about the type of payload.
When the server receives an HTTP request, it says “hello” in the response The
response object res is a data stream which we need to close with res.end() to
transport a response.

To see this in action, you can navigate your browser to localhost:3000, or
send a request with curl::

  $ curl localhost:3000
  hello

Live Reloading
---------------

When developing a server, it can be nice to automatically reload the server
process, when the file server.js has changed.

A good tool to achieve this goal is with nodemon.

Let’s install this next::

  $ npm install nodemon

When you now run the server with::

  $ nodemon server.js

You should see updates whenever you change the file server.js with an editor::

  18 Aug 20:04:59 - [nodemon] v1.4.1
  18 Aug 20:04:59 - [nodemon] to restart at any time, enter `rs`
  18 Aug 20:04:59 - [nodemon] watching: *.*
  18 Aug 20:04:59 - [nodemon] starting `node server.js`
  18 Aug 20:07:17 - [nodemon] restarting due to changes...
  18 Aug 20:07:17 - [nodemon] starting `node server.js`

Once we have a basic server, that we can try on an embedded device, we have to copy the project to the device.
Automatic Deployment

Now that this works, let’s deploy this server to an embedded device such as an Edison or Raspberry Pi. The simplest way to copy an app to server is through scp, a remote copy based on ssh.
Assuming a setup on a machine eddie, you can do this basically with::

  $ scp * root@eddie:~
  root@eddie's password:
  server.js                            100%    0     0.0KB/s   00:00

Instead of scp, it is often nice to setup app deployment based on git. With
git, you get version control out of the box. And git automatically compresses
files for faster file transfer.

First, let’s initialize git in our project with::

  $ git init
  $ cat > .gitignore
  node_modules/**/*
  $ git add .
  $ git commit -m "init"

The first lines make a local git repository. For Node.js projects, it is often
good to skip the local packages in node_modules. This is why we create a .
gitignore file. Last, we commit the everything into the project.

To deploy via git, you first have to setup a so-called “bare” repository on
the device. This bare repository can then be cloned and act as proxy for the
active server.

To do this, you login with::

  $ ssh root@eddie
  $ mkdir -p git/http_server.git
  $ cd git/http_server.git
  $ git init --bare

With the first commands you create empty directories, the second command asks
git to provide an empty shell for a repository. Next, let’s push your server
from previously to this directory.

For this, you do on your local machine::

  $ git remote add eddie ssh://root@eddie:/home/root/git/http_server.git
  $ git push eddie master

Now, the repo on the device is ready to use.
Let’s go to the remote device with::

  $ ssh root@eddie

Now, we first clone the repo with::

  $  git clone git/http_server.git

This new repo tracks the main branch. To see it in action, you can do::

  $ node server.js

And request the URL from the server::

  $ curl eddie:3000
  hello

The last step is to connect a “post-receive” hook to the repo. With this, you
can trigger some script on the device, as soon as there are updates received.
In a file git/http_server.git/hooks/post-receive you insert::

  #!/bin/sh
  git --work-tree=/home/root/projects/simple_http \
    --git-dir=/home/root/projects/git/simple_http.git checkout -f

Then, you make the script executable::

  $ chmod u+x git/http_server.git/hooks/post-receive

If you now push to the repo on the Edison, you’ll automatically get an update
in second directory, where you can run your server process.

Handling Routes
----------------

A request to a web server can take different paths, or routes. Commonly, we
have many states that we want to offer, or to read back. We can implement
routes with a simple if-then tree that parses the incoming request. Since this
quickly gets more difficult, we can also use a module router from npm.

Adding a router
----------------

Every http request is checked for tis path. This makes it necessary to define
“routes” for HTTP requests.

.. code-block:: javascript

  var server = http.createServer(function (req, res) {
    if (req.url == '/') {
      res.writeHead(200, {'Content-Type': 'text/plain'});
      res.end('switch state\n');
    } else if (req.url == '/ON') {
      res.writeHead(200, {'Content-Type': 'text/plain'});
      res.end('on');
    } else if (req.url == '/OFF') {
      res.writeHead(200, {'Content-Type': 'text/plain'});
      res.end('off');
    }
  });
  var port = 3000;
  console.log('Starting server at port: ' + port);
  server.listen(port);

If the data path is the default route, an index HTML is served. If the path
contains ON, we could switch a device ON. Otherwise, the server could switch a
device off.

The router module
-------------------

To manage routes on a server, it is easier to pull in a router module into your project.
A simple approach is the following. We can include a router module with::

  $ npm install --save router

This router handles incoming requests and a finalhandler module delivers a
default response. We need to install a module for this too::

  $ npm install --save finalhandler

Also, a logger can be helpful::

  $ npm install --save morgan

Now, we can rewrite the simple web server from above as follows.
First, we require the new modules and integrate the router:

.. code-block:: javascript

  var fs = require('fs');
  var http = require('http');
  var finalhandler = require('finalhandler');
  var Router = require('router');
  var router = Router();
  router.get('/', function(req, res) {
      res.writeHead(200, {'Content-Type': 'text/html'});
      res.end('Turn a device ON or OFF');
  });
  router.get('/state', function(req,res) {
      res.writeHead(200, {'Content-Type': 'text/plain'});
      res.end(state);
  });
  // add API
  var api = Router();
  api.post('/toggle/:state', function(req, res) {
    console.log('Set embedded state: ' + req.params.state);
    res.writeHead(200, {'Content-Type': 'text/html'});
    // --> integrate hardware connection to come
    res.end('ok');
  });
  router.use('/api', api);
  http.createServer(function (req, res) {
    router(req, res, finalhandler(req, res));
  }).listen(port);

As you can see, there is an additional route for API requests. We are going to
examine how to set and change the hardware with an API in the next chapter.

Driving state with HTTP
------------------------

With curl, it is easily possible to drive state on the server from the command
line. For example, to toggle the state of a LED with curl::

  $ curl -X POST localhost:3474/api/toggle/ON

The same request can be done from the browser application. To call the API from
a browser, if you go to the eddie:3000/state in your browser, you can see that
the path has changed.

This is a good preparation for building the user interface in the next chapter.
Before doing that, let’s first explore an alternative to transfer of state
with HTTP.

The Websocket module
----------------------

Websockets are intensively used for building realtime web applications. They
have two advantages over using HTTP:

1. Websockets add less communication overhead to a network since it does not
use headers for every communication request
2. With Websockets, you can listen for certain messages and push state directly
to a client

The examples with HTTP did not “automatically” update the state of an
device. So, a user must fetch state “manually”. For many situations, we want to
broadcast data from an embedded device. This is when pushing state with
websockets becomes interesting.

NOTE

A number of Node modules for websockets exists. socket.io is popular too and
offers a number of fallbacks when websockets are not available. Websockets is
one possible transport for socket.io (others are flashsocket, htmlfile,
xhr-polling and jsonp-polling)

For now, we are going to use the ws module. Install the module with::

  $ npm install --save ws

First, let’s take an Arduino with a serial link to a Node.js host. To push
data from that device with websockets would look as follows:

.. code-block:: javascript

	var WebSocketServer = require('ws').Server;
	var board = new firmata.Board(modem, function(err){
		console.log('connected \n');
		board.pinMode(13, board.MODES.OUTPUT);
		var wss = new WebSocketServer({server: server});
		wss.on('connection', function connection(ws) {
			ws.on('message', function incoming(message) {
				console.log('received: %s', message);
			});
			board.digitalRead(1, function(val, err) {
				ws.send('{"state": ' + val + '}');
				console.log(val);
			});
		});
	});

Running Code at Specific Times
---------------------------------

Run code at set intervals:

.. code-block:: javascript

  var ONE_MINUTE = 60 * 1000;

  function showTime() {
    console.log(new Date());
  }

  setInterval(showTime, ONE_MINUTE);


npm
-------

- `mjpeg server <https://www.npmjs.com/package/raspberry-pi-mjpeg-server>`_ : raspberry pi camera streamer
- `Raspberry pi version <https://www.npmjs.com/package/raspi-ver>`_ : returns the version and other info for your RPi
- `Great info on how to use npm <https://www.keithcirkel.co.uk/how-to-use-npm-as-a-build-tool/>`_
