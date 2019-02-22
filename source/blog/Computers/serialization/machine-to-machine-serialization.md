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

- Rich data structures (XML)
- A compact, fast, binary data format.
- A container file, to store persistent data.
- Remote procedure call (RPC).
- Simple integration with dynamic languages. Code generation is not required to read or write data files nor to use or implement RPC protocols. Code generation as an optional optimization, only worth implementing for statically typed languages.

- Dislikes
  - XML
  
Languages: C++, Python

Install: `brew install avro-cpp avro-tools` or `pip install avro`
