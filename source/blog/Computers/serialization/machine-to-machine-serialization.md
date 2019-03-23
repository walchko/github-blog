# Machine to Machine Serialization

date: 20 Feb 2019

- [MessagePack](https://msgpack.org)
- [Google Flatbuffers](https://google.github.io/flatbuffers/)
- [Protocol buffers](https://developers.google.com/protocol-buffers/)
- [Cap’n Proto](https://capnproto.org/)
- [Apache Avro](http://avro.apache.org/docs/current/)

# Performance

- Cap'nPack and Flatbuffers are always stored in serialized form so there is **no** time to pack/unpack the messages. However, they do not produce the smallest message size, since they need to be able to efficiently access message data in memory
- Depending on data size, messagepack and protofuf generally do very well

- [Benchmarks](https://github.com/thekvs/cpp-serializers)
- [more](https://codeburst.io/json-vs-protocol-buffers-vs-flatbuffers-a4247f8bda6f)

## Protocol Buffers (Protobuf)

Protocol buffers are Google's language-neutral, platform-neutral, extensible mechanism for serializing structured data – think XML, but smaller, faster, and simpler. You define how you want your data to be structured once, then you can use special generated source code to easily write and read your structured data to and from a variety of data streams and using a variety of languages.

- Dislikes
  - Separate tool to build message
  - Large use of templates which increases compile time
  - Feels like an 800lb gorilla
  - Difficult to populate message with data or put another way, awkward syntax
  - Simple messages generate large header/implementation files
    - Use every trick in the compsci book which makes thing overly complex for the simplest things
  - No way to extend message class (math, printing, etc)
  - Google centric, if google doesn't care, it won't get done/fixed
    - Issues with proper timestamps and leap seconds

Languages: C++, Python

Install: `brew install capnp` or `pip install pycapnp`

## Cap'n Proto

- The Cap’n Proto encoding is appropriate both as a data interchange format and an in-memory representation, so once your structure is built, you can simply write the bytes straight out to disk!
- Inter-process communication: Multiple processes running on the same machine can share a Cap’n Proto message via shared memory. No need to pipe data through the kernel. Calling another process can be just as fast and easy as calling another thread.
- Created by the Google project manager for protobuf version 2. He tried to fix things he didn't like and get rid of the pack/unpack time
- Random access: You can read just one field of a message without parsing the whole thing.
- Really designed for its own RPC system more scope than just message serialization
- Awkward interfacing with zmq. In memory data structure unable to put into zmq without resorting to in memory copy which seems to negate the capability of storing data in memory the same as on wire
- Cmake support for building messages in c++

- Dislikes
  - Focus seems to be c++, python support spotty
  
Languages: C++, Python

Install: `brew install capnp` or `pip install pycapnp`

## Google Flatbuffers

Similar to Cap'n Proto (already serialized) but also get Protobuf benefits. Used by [Facebook](https://code.fb.com/android/improving-facebook-s-performance-on-android-with-flatbuffers/).
- Zmq example: https://stackoverflow.com/questions/40053351/simple-flatbuffer-over-zeromq-c-example-copying-struct-to-flatbuffer-over-zm
- Seems to have cmake support in c++

Languages: C++, Python

Install: `brew info flatbuffers` or `pip install flatbuffers`

## MessagePack

MessagePack is an efficient binary serialization format. It lets you exchange data among multiple languages like JSON. But it's faster and smaller. Small integers are encoded into a single byte, and typical short strings require only one extra byte in addition to the strings themselves.
- Adding compression for small messages seems unnecessary (small size savings for cpu cycles) or in some cases has resulted in a larger compressed message

- Dislikes
  - Json has no understanding of python tuples, only arrays, so you are limited between one or the other, not both
  - no schema, need to redefine message for every language
    - Also a benefit, you can add more capability to your message (math, printing, etc)

Languages: C++, Python

Install: `brew install msgpack` or `pip install msgpack`

## Apache Avro

Apache Avro is a data serialization system. Avro provides:

- Schema based, with ability to evolve
- A compact, fast, binary data format.
- A container file, to store persistent data.
- Remote procedure call (RPC).
- Simple integration with dynamic languages. Code generation is not required to read or write data files nor to use or implement RPC protocols. Code generation as an optional optimization, only worth implementing for statically typed languages.

## Wikipedia Example

Here is the example:

<script src="https://gist.github.com/walchko/8d3f9edd5ac3d8170cfbbf80bf65a485.js"></script>

When you look the file you can see the avro header, schema, and data:

```
$ od -v -t x1z users.avro 
0000000 4f 62 6a 01 04 14 61 76 72 6f 2e 63 6f 64 65 63  >Obj...avro.codec<
0000020 08 6e 75 6c 6c 16 61 76 72 6f 2e 73 63 68 65 6d  >.null.avro.schem<
0000040 61 ba 03 7b 22 74 79 70 65 22 3a 20 22 72 65 63  >a..{"type": "rec<
0000060 6f 72 64 22 2c 20 22 6e 61 6d 65 22 3a 20 22 55  >ord", "name": "U<
0000100 73 65 72 22 2c 20 22 6e 61 6d 65 73 70 61 63 65  >ser", "namespace<
0000120 22 3a 20 22 65 78 61 6d 70 6c 65 2e 61 76 72 6f  >": "example.avro<
0000140 22 2c 20 22 66 69 65 6c 64 73 22 3a 20 5b 7b 22  >", "fields": [{"<
0000160 74 79 70 65 22 3a 20 22 73 74 72 69 6e 67 22 2c  >type": "string",<
0000200 20 22 6e 61 6d 65 22 3a 20 22 6e 61 6d 65 22 7d  > "name": "name"}<
0000220 2c 20 7b 22 74 79 70 65 22 3a 20 5b 22 69 6e 74  >, {"type": ["int<
0000240 22 2c 20 22 6e 75 6c 6c 22 5d 2c 20 22 6e 61 6d  >", "null"], "nam<
0000260 65 22 3a 20 22 66 61 76 6f 72 69 74 65 5f 6e 75  >e": "favorite_nu<
0000300 6d 62 65 72 22 7d 2c 20 7b 22 74 79 70 65 22 3a  >mber"}, {"type":<
0000320 20 5b 22 73 74 72 69 6e 67 22 2c 20 22 6e 75 6c  > ["string", "nul<
0000340 6c 22 5d 2c 20 22 6e 61 6d 65 22 3a 20 22 66 61  >l"], "name": "fa<
0000360 76 6f 72 69 74 65 5f 63 6f 6c 6f 72 22 7d 5d 7d  >vorite_color"}]}<
0000400 00 05 f9 a3 80 98 47 54 62 bf 68 95 a2 ab 42 ef  >......GTb.h...B.<
0000420 24 04 2c 0c 41 6c 79 73 73 61 00 80 04 02 06 42  >$.,.Alyssa.....B<
0000440 65 6e 00 0e 00 06 72 65 64 05 f9 a3 80 98 47 54  >en....red.....GT<
0000460 62 bf 68 95 a2 ab 42 ef 24                       >b.h...B.$<
0000471
```

[Apache Avro Wikipedia Reference](https://en.wikipedia.org/wiki/Apache_Avro)
  
Languages: C++, Python

Install: `brew install avro-cpp avro-tools` or `pip install avro`
