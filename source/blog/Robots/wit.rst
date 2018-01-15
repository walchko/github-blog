Speech to Text (STT)
=====================

:date: 2016-03-20
:summary: How to do speech to text

**Note:** Since Facebook bought wit, its original business is changing. The
current python library is a skeleton of what it use to be and the programmers
at wit don't seem to know python very well.

Version 2.0
-------------

Wit.ai changed everything ...

`Wit <http://wit.ai>`_ turns speech into text. Some return examples are::

  {u'msg_body': u'hello world', u'msg_id': u'6a410cda-32e0-4602-bcfb-c20f5e1aed66', u'outcome': {u'entities': {}, u'confidence': 0.525, u'intent': u'order'}}
  {u'msg_body': u'joke', u'outcome': {u'entities': {}, u'confidence': 0.767, u'intent': u'joke'}, u'msg_id': u'4046cbaf-00cc-4305-9393-06a48e7ed4d3'}
  {u'msg_body': u'hello', u'outcome': {u'entities': {}, u'confidence': 0.787, u'intent': u'greeting'}, u'msg_id': u'6dc2722b-5acc-4066-8727-7c506728aa03'}
  {u'msg_body': u'what time is it', u'outcome': {u'entities': {}, u'confidence': 0.691, u'intent': u'time'}, u'msg_id': u'156c0751-644a-4e76-9b19-fff078574f2f'}
  {u'msg_body': u'what is the weather tomorrow', u'outcome': {u'entities': {u'datetime': {u'body': u'tomorrow', u'start': 20, u'end': 28, u'value': {u'to': u'2014-09-14T00:00:00.000-07:00', u'from': u'2014-09-13T00:00:00.000-07:00'}}}, u'confidence': 0.856, u'intent': u'weather'}, u'msg_id': u'c325bf08-20a0-47ad-ab14-33c2200a8be7'}
  {u'msg_body': u'the weather tomorrow', u'outcome': {u'entities': {u'datetime': {u'body': u'tomorrow', u'start': 12, u'end': 20, u'value': {u'to': u'2014-09-14T00:00:00.000-07:00', u'from': u'2014-09-13T00:00:00.000-07:00'}}}, u'confidence': 0.847, u'intent': u'weather'}, u'msg_id': u'ced4d342-2557-4aed-89e6-a948614a87e8'}

They have recently changed their python library, so now all it does is send a
text string to their servers ... kind of stupid. You have to write your own code
to send audio. Their staff doesn't seem very knowledgable on python, hence all
of the need help banners on their issues github page.

Oh well, it is free and works pretty good, so I can't complain too much.

Grab Speech
-------------

Now you have to write your own code to grabe speech (which is fine) and you can
use `sox <https://wit.ai/docs/http/20141022#streaming-audio>`_ per the example
on their website. Then ship it off to there servers to figure out what it means.
