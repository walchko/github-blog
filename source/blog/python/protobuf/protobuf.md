---
title: Python and Google Protobuffers
date: 22 Mar 2020
---

Install python module: `pip install protobuf`

However, you won't actually import that library directly, it will be
done behind the scenes.

You will need access to `protoc` to create the proper libraries from
schemas. On Ubuntu you can do: `sudo apt install libprotobuf-dev` to get
it or build it from google's github repo to get the latest and greatest.

There is another way to get `protoc`, you can install it via the gRPC
tools: `pip install grpcio-tools`. I have not done this. However, once
you install them, you can make a alias to the actual tool: 
`alias protoc='python -m grpc_tools.protoc'`

## Bulding

Create a simple schema:

```protobuf
syntax = "proto3";

message Vector {
  double x = 1;
  double y = 2;
  double z = 3;
}
```

Now compile with: `protoc --proto_path=./ --python_out=./ vector.proto`

A new library should appear that you can import and use: `vector_pb2.py`

## Using

```python
from vector_pb2 import Vector

v = Vector()
v.x = 1
v.y = -0.00002
v.z = 3.14
```

# References

- [Simple python tutorial](https://planspace.org/20170329-protocol_buffers_in_python/)
