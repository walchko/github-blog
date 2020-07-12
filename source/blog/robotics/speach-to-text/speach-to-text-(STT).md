---
title: Speech to Text (STT)
date: 2016-03-20
---

**Note:** Since Facebook bought wit, its original business is changing.
The current python library is a skeleton of what it use to be and the
programmers at wit don't seem to know python very well.

## Version 2.0

Wit.ai changed everything ...

[Wit](http://wit.ai) turns speech into text. Some return examples are:

```json
{"msg_body": "hello world", "msg_id": "6a410cda-32e0-4602-bcfb-c20f5e1aed66", "outcome": {"entities": {}, "confidence": 0.525, "intent": "order"}}
{"msg_body": "joke", "outcome": {"entities": {}, "confidence": 0.767, "intent": "joke"}, "msg_id": "4046cbaf-00cc-4305-9393-06a48e7ed4d3"}
{"msg_body": "hello", "outcome": {"entities": {}, "confidence": 0.787, "intent": "greeting"}, "msg_id": "6dc2722b-5acc-4066-8727-7c506728aa03"}
{"msg_body": "what time is it", "outcome": {"entities": {}, "confidence": 0.691, "intent": "time"}, "msg_id": "156c0751-644a-4e76-9b19-fff078574f2f"}
{"msg_body": "what is the weather tomorrow", "outcome": {"entities": {"datetime": {"body": "tomorrow", "start": 20, "end": 28, "value": {"to": "2014-09-14T00:00:00.000-07:00", "from": "2014-09-13T00:00:00.000-07:00"}}}, "confidence": 0.856, "intent": "weather"}, "msg_id": "c325bf08-20a0-47ad-ab14-33c2200a8be7"}
{"msg_body": "the weather tomorrow", "outcome": {"entities": {"datetime": {"body": "tomorrow", "start": 12, "end": 20, "value": {"to": "2014-09-14T00:00:00.000-07:00", "from": "2014-09-13T00:00:00.000-07:00"}}}, "confidence": 0.847, "intent": "weather"}, "msg_id": "ced4d342-2557-4aed-89e6-a948614a87e8"}
```

They have recently changed their python library, so now all it does is
send a text string to their servers ... kind of stupid. You have to
write your own code to send audio. Their staff doesn't seem very
knowledgable on python, hence all of the need help banners on their
issues github page.

Oh well, it is free and works pretty good, so I can't complain too much.

## Grab Speech

Now you have to write your own code to grab speech (which is fine) and
you can use [sox](https://wit.ai/docs/http/20141022#streaming-audio) per
the example on their website. Then ship it off to there servers to
figure out what it means.
